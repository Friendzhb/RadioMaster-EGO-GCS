package com.radiomaster.egogcs.db

import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.RenderSnapshot
import java.io.BufferedWriter
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStreamWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicInteger
import kotlin.coroutines.CoroutineContext
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

/**
 * 飞行任务 SQLite 仓库与非阻塞异步批量落库引擎 (`FlightMissionRepository`)：
 * - 遵循 Repository 模式封装 `FlightMissionDao`，通过 `missionsFlow` 实时推送历史任务架次；
 * - 50Hz/20Hz 实时引擎线程仅通过无锁 `ConcurrentLinkedQueue` 投递采样快照，后台单协程串行聚合批量写入 SQLite，
 *   确保高频控制/渲染循环零磁盘 I/O 抖动。
 */
class FlightMissionRepository(
    private val dao: FlightMissionDao,
    ioDispatcher: CoroutineContext = Dispatchers.IO
) {

    companion object {
        const val BATCH_FLUSH_SIZE = 25
        const val MAX_STORED_MISSIONS = 60
        private const val MAX_PENDING_SAMPLES = 4_096
        private const val RAD_TO_DEG = 57.29578f
    }

    private sealed interface WriterCommand {
        data class StartSession(
            val title: String,
            val startedWallMs: Long,
            val linkMode: String,
            val airframeRig: String,
            val swarmCount: Int,
            val startVolt: Float,
            val waypointsCsv: String,
            val onStarted: ((Long) -> Unit)?
        ) : WriterCommand

        data object DrainSamples : WriterCommand

        data class StopSession(
            val endedWallMs: Long,
            val waypointsCsv: String,
            val onStopped: ((FlightMissionEntity?) -> Unit)?
        ) : WriterCommand
    }

    private data class StagedSample(
        val timestampEpochMs: Long,
        val elapsedMs: Long,
        val x: Float,
        val y: Float,
        val z: Float,
        val vx: Float,
        val vy: Float,
        val vz: Float,
        val rollRad: Float,
        val pitchRad: Float,
        val yawRad: Float,
        val quatW: Float,
        val quatX: Float,
        val quatY: Float,
        val quatZ: Float,
        val goalX: Float,
        val goalY: Float,
        val goalZ: Float,
        val activeIsTiltRig: Boolean,
        val tiltS1Rad: Float,
        val tiltS2Rad: Float,
        val tiltCmd1Rad: Float,
        val tiltCmd2Rad: Float,
        val m0: Float,
        val m1: Float,
        val m2: Float,
        val m3: Float,
        val minClearanceM: Float,
        val minInterDroneDistM: Float,
        val solveMs: Float,
        val batVolt: Float,
        val batAmps: Float,
        val batPct: Int,
        val fsmState: String,
        val px4Mode: String,
        val armed: Boolean
    )

    private class ActiveSessionAccumulator(var entity: FlightMissionEntity) {
        var count: Int = 0
        var prevX: Float = Float.NaN
        var prevY: Float = Float.NaN
        var prevZ: Float = Float.NaN
        var distMeters: Float = 0f
        var maxSpd: Float = 0f
        var sumSpd: Float = 0f
        var maxAlt: Float = 0f
        var minClr: Float = 9.5f
        var startV: Float = entity.startVolt
        var minV: Float = if (entity.startVolt > 0f) entity.startVolt else 99f
        var peakWatts: Float = 0f
        var latestWallMs: Long = entity.startedAtEpochMs
        var latestElapsedMs: Long = 0L

        fun fold(s: StagedSample) {
            if (count == 0) {
                startV = s.batVolt
                minV = s.batVolt
                maxAlt = s.z
                minClr = s.minClearanceM
            } else if (prevX.isFinite()) {
                val d = hypot(hypot(s.x - prevX, s.y - prevY), s.z - prevZ)
                if (d.isFinite() && d < 25f) distMeters += d
            }
            prevX = s.x
            prevY = s.y
            prevZ = s.z
            count++

            val spd = hypot(s.vx, s.vy)
            maxSpd = max(maxSpd, spd)
            sumSpd += spd
            maxAlt = max(maxAlt, s.z)
            minClr = min(minClr, s.minClearanceM)
            if (s.batVolt > 0.5f) minV = min(minV, s.batVolt)
            peakWatts = max(peakWatts, s.batVolt * s.batAmps)
            latestWallMs = max(latestWallMs, s.timestampEpochMs)
            latestElapsedMs = max(latestElapsedMs, s.elapsedMs)
        }

        fun buildSummary(finalWaypointsCsv: String? = null): FlightMissionEntity {
            val avgSpd = if (count > 0) sumSpd / count.toFloat() else 0f
            val resolvedMinV = if (minV > 90f) startV else minV
            val resolvedWaypoints = if (!finalWaypointsCsv.isNullOrBlank()) {
                finalWaypointsCsv
            } else {
                entity.waypointsCsv
            }
            entity = entity.copy(
                endedAtEpochMs = latestWallMs,
                durationMs = latestElapsedMs,
                sampleCount = count,
                distanceMeters = distMeters,
                maxSpeedMps = maxSpd,
                avgSpeedMps = avgSpd,
                maxAltitudeM = maxAlt,
                minClearanceM = if (count > 0) minClr else 9.5f,
                startVolt = startV,
                minVolt = resolvedMinV,
                peakPowerWatts = peakWatts,
                waypointsCsv = resolvedWaypoints
            )
            return entity
        }
    }

    val missionsFlow: Flow<List<FlightMissionEntity>> = dao.observeAllMissions()

    private val ioContext = ioDispatcher
    private val writerScope = CoroutineScope(SupervisorJob() + ioContext)
    private val commandChannel = Channel<WriterCommand>(Channel.UNLIMITED)
    private val stagingQueue = ConcurrentLinkedQueue<StagedSample>()
    private val pendingCount = AtomicInteger(0)

    @Volatile
    var activeMissionId: Long = 0L
        private set

    @Volatile
    var isSessionActive: Boolean = false
        private set

    init {
        writerScope.launch {
            processWriterLoop()
        }
    }

    fun startNewMissionSession(
        hub: GcsSharedHub,
        currentSnap: RenderSnapshot?,
        waypointsCsv: String,
        customTag: String? = null,
        nowWallMs: Long = System.currentTimeMillis(),
        onStarted: ((Long) -> Unit)? = null
    ) {
        val stamp = SimpleDateFormat("MM-dd HH:mm:ss", Locale.US).format(Date(nowWallMs))
        val linkMode = if (hub.isRosMode) "ROS1" else "SIM"
        val rig = if (currentSnap?.activeIsTiltRig ?: hub.activeFocusIsTiltRig()) "TILT_4M2S" else "STD_X4"
        val swCnt = currentSnap?.swarmCount ?: (if (hub.swarmModeEnabled) hub.swarmDroneCount else 1)
        val prefix = customTag ?: if (hub.isRosMode) "ROS1实机架次" else "仿真演练架次"
        val title = "$prefix $stamp"
        val startVolt = currentSnap?.batVolt ?: 16.2f

        stagingQueue.clear()
        pendingCount.set(0)
        isSessionActive = true
        commandChannel.trySend(
            WriterCommand.StartSession(
                title = title,
                startedWallMs = nowWallMs,
                linkMode = linkMode,
                airframeRig = rig,
                swarmCount = swCnt,
                startVolt = startVolt,
                waypointsCsv = waypointsCsv,
                onStarted = onStarted
            )
        )
    }

    /**
     * 20Hz 采样热路径无锁入队：不执行任何磁盘或挂起操作，满队列时保护性丢弃最旧未刷盘溢出帧。
     */
    fun enqueueTelemetrySample(
        timestampEpochMs: Long,
        elapsedMs: Long,
        s: RenderSnapshot
    ) {
        if (!isSessionActive) return
        if (pendingCount.get() >= MAX_PENDING_SAMPLES) {
            stagingQueue.poll()?.let { pendingCount.decrementAndGet() }
        }
        stagingQueue.offer(
            StagedSample(
                timestampEpochMs = timestampEpochMs,
                elapsedMs = elapsedMs,
                x = s.uavX,
                y = s.uavY,
                z = s.uavZ,
                vx = s.vx,
                vy = s.vy,
                vz = s.vz,
                rollRad = s.smoothRoll,
                pitchRad = s.smoothPitch,
                yawRad = s.yaw,
                quatW = s.quatW,
                quatX = s.quatX,
                quatY = s.quatY,
                quatZ = s.quatZ,
                goalX = s.goalX,
                goalY = s.goalY,
                goalZ = s.goalZ,
                activeIsTiltRig = s.activeIsTiltRig,
                tiltS1Rad = s.tiltServo1Rad,
                tiltS2Rad = s.tiltServo2Rad,
                tiltCmd1Rad = s.tiltCmd1Rad,
                tiltCmd2Rad = s.tiltCmd2Rad,
                m0 = s.motorNorm4[0],
                m1 = s.motorNorm4[1],
                m2 = s.motorNorm4[2],
                m3 = s.motorNorm4[3],
                minClearanceM = s.minClearance,
                minInterDroneDistM = s.minInterDroneDist,
                solveMs = s.solveMs,
                batVolt = s.batVolt,
                batAmps = s.batCurrentAmps,
                batPct = s.batPct,
                fsmState = s.fsmState,
                px4Mode = s.px4Mode,
                armed = s.armed
            )
        )
        val curBuffered = pendingCount.incrementAndGet()
        if (curBuffered >= BATCH_FLUSH_SIZE) {
            commandChannel.trySend(WriterCommand.DrainSamples)
        }
    }

    fun stopActiveMissionSession(
        waypointsCsv: String = "",
        endedWallMs: Long = System.currentTimeMillis(),
        onStopped: ((FlightMissionEntity?) -> Unit)? = null
    ) {
        if (!isSessionActive && activeMissionId == 0L) {
            onStopped?.invoke(null)
            return
        }
        isSessionActive = false
        commandChannel.trySend(WriterCommand.StopSession(endedWallMs, waypointsCsv, onStopped))
    }

    private suspend fun processWriterLoop() {
        var currentAcc: ActiveSessionAccumulator? = null

        suspend fun drainPendingIntoSession(acc: ActiveSessionAccumulator, forceUpdateSummary: Boolean) {
            val batch = ArrayList<TelemetryPointEntity>(BATCH_FLUSH_SIZE * 2)
            while (true) {
                val raw = stagingQueue.poll() ?: break
                pendingCount.decrementAndGet()
                acc.fold(raw)
                batch.add(raw.toEntity(acc.entity.id))
                if (batch.size >= BATCH_FLUSH_SIZE * 4) {
                    dao.insertTelemetryBatch(batch)
                    batch.clear()
                }
            }
            if (batch.isNotEmpty()) {
                dao.insertTelemetryBatch(batch)
            }
            if (forceUpdateSummary || batch.isNotEmpty()) {
                dao.updateMission(acc.buildSummary())
            }
        }

        for (cmd in commandChannel) {
            try {
                when (cmd) {
                    is WriterCommand.StartSession -> {
                        currentAcc?.let { prev ->
                            drainPendingIntoSession(prev, forceUpdateSummary = true)
                            if (prev.count == 0) {
                                dao.deleteMissionById(prev.entity.id)
                            }
                        }
                        val initial = FlightMissionEntity(
                            title = cmd.title,
                            startedAtEpochMs = cmd.startedWallMs,
                            endedAtEpochMs = cmd.startedWallMs,
                            linkMode = cmd.linkMode,
                            airframeRig = cmd.airframeRig,
                            swarmCount = cmd.swarmCount,
                            startVolt = cmd.startVolt,
                            minVolt = cmd.startVolt,
                            waypointsCsv = cmd.waypointsCsv
                        )
                        val newId = dao.insertMission(initial)
                        val persisted = initial.copy(id = newId)
                        currentAcc = ActiveSessionAccumulator(persisted)
                        activeMissionId = newId
                        dao.pruneOldMissionsKeepLatest(MAX_STORED_MISSIONS)
                        cmd.onStarted?.invoke(newId)
                        if (stagingQueue.isNotEmpty()) {
                            drainPendingIntoSession(currentAcc!!, forceUpdateSummary = true)
                        }
                    }

                    is WriterCommand.DrainSamples -> {
                        val acc = currentAcc
                        if (acc != null) {
                            drainPendingIntoSession(acc, forceUpdateSummary = false)
                        }
                    }

                    is WriterCommand.StopSession -> {
                        val acc = currentAcc
                        currentAcc = null
                        activeMissionId = 0L
                        if (acc != null) {
                            acc.latestWallMs = max(acc.latestWallMs, cmd.endedWallMs)
                            drainPendingIntoSession(acc, forceUpdateSummary = false)
                            if (acc.count <= 0) {
                                dao.deleteMissionById(acc.entity.id)
                                cmd.onStopped?.invoke(null)
                            } else {
                                val finalEntity = acc.buildSummary(cmd.waypointsCsv)
                                dao.updateMission(finalEntity)
                                cmd.onStopped?.invoke(finalEntity)
                            }
                        } else {
                            cmd.onStopped?.invoke(null)
                        }
                    }
                }
            } catch (_: Throwable) {
            }
        }
    }

    private fun StagedSample.toEntity(missionId: Long): TelemetryPointEntity = TelemetryPointEntity(
        missionId = missionId,
        timestampEpochMs = timestampEpochMs,
        elapsedMs = elapsedMs,
        x = x,
        y = y,
        z = z,
        vx = vx,
        vy = vy,
        vz = vz,
        rollRad = rollRad,
        pitchRad = pitchRad,
        yawRad = yawRad,
        quatW = quatW,
        quatX = quatX,
        quatY = quatY,
        quatZ = quatZ,
        goalX = goalX,
        goalY = goalY,
        goalZ = goalZ,
        activeIsTiltRig = activeIsTiltRig,
        tiltS1Rad = tiltS1Rad,
        tiltS2Rad = tiltS2Rad,
        tiltCmd1Rad = tiltCmd1Rad,
        tiltCmd2Rad = tiltCmd2Rad,
        m0 = m0,
        m1 = m1,
        m2 = m2,
        m3 = m3,
        minClearanceM = minClearanceM,
        minInterDroneDistM = minInterDroneDistM,
        solveMs = solveMs,
        batVolt = batVolt,
        batAmps = batAmps,
        batPct = batPct,
        fsmState = fsmState,
        px4Mode = px4Mode,
        armed = armed
    )

    suspend fun loadMissionWithTelemetry(
        missionId: Long
    ): Pair<FlightMissionEntity, List<TelemetryPointEntity>>? = withContext(ioContext) {
        val mission = dao.getMissionById(missionId) ?: return@withContext null
        val pts = dao.getTelemetryPointsForMission(missionId)
        mission to pts
    }

    suspend fun analyzeMission(
        missionId: Long,
        warnClearanceM: Float = 0.65f
    ): MissionAnalysisReport? = withContext(ioContext) {
        val (mission, pts) = loadMissionWithTelemetry(missionId) ?: return@withContext null
        val report = PostFlightAnalyzer.analyze(mission, pts, warnClearanceM)
        if (pts.isNotEmpty() && report.mission != mission) {
            dao.updateMission(report.mission)
        }
        report
    }

    suspend fun deleteMission(missionId: Long) = withContext(ioContext) {
        dao.deleteMissionById(missionId)
    }

    suspend fun exportMissionToCsv(missionId: Long, outFile: File): Int = withContext(ioContext) {
        val (mission, pts) = loadMissionWithTelemetry(missionId) ?: return@withContext 0
        if (pts.isEmpty()) return@withContext 0
        outFile.parentFile?.mkdirs()
        BufferedWriter(OutputStreamWriter(FileOutputStream(outFile), Charsets.UTF_8), 65536).use { w ->
            w.write(
                "timestamp_epoch_ms,t_sec,link_mode,swarm_count,airframe_rig,fsm_state,px4_mode,armed," +
                    "x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,roll_deg,pitch_deg,yaw_deg," +
                    "qw,qx,qy,qz,goal_x_m,goal_y_m,goal_z_m," +
                    "servo1_deg,servo2_deg,cmd1_deg,cmd2_deg,m1_norm,m2_norm,m3_norm,m4_norm," +
                    "min_clearance_m,min_inter_uav_m,plan_solve_ms,bat_volt_v,bat_curr_a,bat_pct\n"
            )
            val sb = StringBuilder(320)
            for (p in pts) {
                val rigStr = if (p.activeIsTiltRig) "TILT_4M2S" else "STD_X4"
                sb.setLength(0)
                sb.append(
                    String.format(
                        Locale.US,
                        "%d,%.3f,%s,%d,%s,%s,%s,%d,",
                        p.timestampEpochMs,
                        p.elapsedMs * 0.001f,
                        mission.linkMode,
                        mission.swarmCount,
                        rigStr,
                        p.fsmState,
                        p.px4Mode,
                        if (p.armed) 1 else 0
                    )
                )
                sb.append(
                    String.format(
                        Locale.US,
                        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,",
                        p.x, p.y, p.z,
                        p.vx, p.vy, p.vz,
                        p.rollRad * RAD_TO_DEG, p.pitchRad * RAD_TO_DEG, p.yawRad * RAD_TO_DEG
                    )
                )
                sb.append(
                    String.format(
                        Locale.US,
                        "%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,",
                        p.quatW, p.quatX, p.quatY, p.quatZ,
                        p.goalX, p.goalY, p.goalZ
                    )
                )
                sb.append(
                    String.format(
                        Locale.US,
                        "%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,",
                        p.tiltS1Rad * RAD_TO_DEG, p.tiltS2Rad * RAD_TO_DEG,
                        p.tiltCmd1Rad * RAD_TO_DEG, p.tiltCmd2Rad * RAD_TO_DEG,
                        p.m0, p.m1, p.m2, p.m3
                    )
                )
                sb.append(
                    String.format(
                        Locale.US,
                        "%.4f,%.4f,%.4f,%.3f,%.3f,%d\n",
                        p.minClearanceM, p.minInterDroneDistM, p.solveMs,
                        p.batVolt, p.batAmps, p.batPct
                    )
                )
                w.write(sb.toString())
            }
            w.flush()
        }
        pts.size
    }
}
