package com.radiomaster.egogcs.engine

import android.os.SystemClock
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.RenderSnapshot
import com.radiomaster.egogcs.model.SwarmPalette
import com.radiomaster.egogcs.model.TiltConfigSpec
import java.util.ArrayDeque
import java.util.Locale
import java.util.Random
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * 零 GC 四缓冲后台物理、FAST-LIO 体素融合与 EGO-Planner 三维膨胀占据体素引擎：
 * - 1:1 还原真实 EGO-Planner (plan_env/grid_map.cpp) 的三维离散膨胀占据栅格 (/grid_map/occupancy_inflate)，
 *   仿真与实机统一输出三维体素块中心点集而非理想化光滑圆柱，调大膨胀半径可直观看到三维体素块向外扩张；
 * - 支持实机高达 4096 个局部三维膨胀体素无锁同步至 OpenGL 硬件 Z-Buffer 与视线隧道透视管线。
 */
class EgoSimEngine(private val hub: GcsSharedHub) {

    private data class Obstacle(
        val cx: Float,
        val cy: Float,
        val rx: Float,
        val ry: Float,
        val h: Float,
        val isCircle: Boolean
    )

    data class PinInspectResult(
        val x: Float,
        val y: Float,
        val z: Float,
        val distFromUav: Float,
        val clearance: Float,
        val isSafe: Boolean
    )

    data class WaypointNodeInfo(
        val index: Int,
        val x: Float,
        val y: Float,
        val z: Float,
        val clearance: Float,
        val isSafe: Boolean
    )

    data class DroneGoalCmd(
        val droneId: Int,
        val gx: Float,
        val gy: Float,
        val gz: Float,
        val curX: Float,
        val curY: Float
    )

    private class SwarmPeerUnit(val id: Int) {
        var homeX: Float = -5.5f
        var homeY: Float = -4.2f
        var rosHomeCaptured: Boolean = false
        var uavX: Float = -5.5f
        var uavY: Float = -4.2f
        var uavZ: Float = 1.2f
        var vx: Float = 0f
        var vy: Float = 0f
        var vz: Float = 0f
        var ax: Float = 0f
        var ay: Float = 0f
        var yaw: Float = 0.65f
        var prevYaw: Float = 0.65f
        var smoothRoll: Float = 0f
        var smoothPitch: Float = 0f
        var tiltS1Rad: Float = 0f
        var tiltS2Rad: Float = 0f
        var rosHasLiveTilt: Boolean = false
        var rosTiltS1: Float = 0f
        var rosTiltS2: Float = 0f
        var lastTiltRxMs: Long = 0L
        var goalX: Float = 5.2f
        var goalY: Float = 3.8f
        var goalZ: Float = 1.2f
        var fsmState: String = "EXEC_TRAJ"
        var batVolt: Float = 16.20f
        var batPct: Int = 88
        var online: Boolean = true
        var lastRosRxMs: Long = 0L

        val ctrlBuf = FloatArray(16 * 3)
        var ctrlCount = 0
        val bsplineBuf = FloatArray(240 * 3)
        var bsplineCount = 0
        val rosPathBuf = FloatArray(240 * 3)
        var rosPathCount = 0
        val trailRing = FloatArray(160 * 3)
        var trailCount = 0
        var replanAcc = 0f
    }

    companion object {
        private const val GRID_RES = 0.25f
        private const val MAP_HALF = 8.0f
        private const val GRID_N = 64
        private const val GRID_CELLS = GRID_N * GRID_N
        private const val MAX_WAYPOINT_QUEUE = 16
        private const val MAX_INFLATE_VOXELS = 4090
        private val RADAR_COS_60 = FloatArray(60) { b -> cos(-PI.toFloat() + (b.toFloat() / 60f) * (PI.toFloat() * 2f)) }
        private val RADAR_SIN_60 = FloatArray(60) { b -> sin(-PI.toFloat() + (b.toFloat() / 60f) * (PI.toFloat() * 2f)) }
    }

    val voxelMap = FastLioVoxelMap()
    val telemetryRecorder = FlightTelemetryRecorder()

    @Volatile var onRosWaypointAdvanced: ((gx: Float, gy: Float, gz: Float, curX: Float, curY: Float) -> Unit)? = null

    @Volatile private var asyncReplanPending: Boolean = false

    private var executor: ScheduledExecutorService? = null
    private val rng = Random()
    private val lock = Any()

    private val snapshotRing = Array(4) { RenderSnapshot() }
    private var snapshotWriteIdx = 0

    private var obstacles = mutableListOf<Obstacle>()
    private val distField = FloatArray(GRID_CELLS)
    private val cellInflatedTopH = FloatArray(GRID_CELLS)
    private val occGrid = IntArray(GRID_CELLS)

    // 预构建三维膨胀表面壳体素缓存: x, y, z, halfSize (最多 4096 个三维体素块)
    private val cachedGridVoxels = FloatArray(4096 * 4)
    private var cachedGridVoxelCount = 0
    private var lastBuiltInflation = -1f

    private var astarSearchEpoch = 1
    private val astarVisitStamp = IntArray(GRID_CELLS)
    private val astarClosedStamp = IntArray(GRID_CELLS)
    private val astarGScore = FloatArray(GRID_CELLS)
    private val astarParent = IntArray(GRID_CELLS)
    private val heapNodes = IntArray(GRID_CELLS + 16)
    private val heapF = FloatArray(GRID_CELLS + 16)
    private val revXScratch = FloatArray(512)
    private val revYScratch = FloatArray(512)
    private val cumScratch = FloatArray(600)
    private val extCtrlScratch = FloatArray(24 * 3)

    private var homeX = -5.5f
    private var homeY = -4.2f
    private var rosHomeCaptured = false

    private var uavX = -5.5f
    private var uavY = -4.2f
    private var uavZ = 1.2f
    private var vx = 0f
    private var vy = 0f
    private var vz = 0f
    private var ax = 0f
    private var ay = 0f
    private var yaw = 0.65f
    private var prevYaw = 0.65f
    private var smoothRoll = 0f
    private var smoothPitch = 0f
    private var rosHasRealQuat = false
    private var rosRawRoll = 0f
    private var rosRawPitch = 0f
    private var curQuatW = 1f
    private var curQuatX = 0f
    private var curQuatY = 0f
    private var curQuatZ = 0f

    // Tilt-UAV-INDI (4电机一阶 tau_m=28ms + 双Yb轴舵机二阶欠阻尼 tau_s=75ms, zeta=0.68, 限速 300 deg/s) 状态
    private var prevAFwd = 0f
    private var simServo1Rad = 0f
    private var simServo2Rad = 0f
    private var simServo1RateRadS = 0f
    private var simServo2RateRadS = 0f
    private var cmdServo1Rad = 0f
    private var cmdServo2Rad = 0f
    private val simMotorNorm = floatArrayOf(0.52f, 0.52f, 0.52f, 0.52f)
    private var simManipulability: Float = 1.0f
    private var simThrustMarginPct: Int = 48
    private var smoothBiasPitchRad: Float = 0f
    private var simServoSaturated = false
    private var rosHasLiveTilt = false
    private var rosLastTiltRxMs = 0L
    private var rosTiltS1Rad = 0f
    private var rosTiltS2Rad = 0f
    private val rawBins60 = FloatArray(60) { 9.5f }
    private val smoothBins60 = FloatArray(60) { 9.5f }

    private var goalX = 5.2f
    private var goalY = 3.8f
    private var goalZ = 1.2f
    private var fsmState = "EXEC_TRAJ"
    private var solveMs = 0.4f
    private var trajLen = 6.0f

    private var hasPreviewPin = false
    private var previewX = 0f
    private var previewY = 0f
    private var previewZ = 1.2f
    private var previewClearance = 9.5f
    private var previewSafe = true

    private val stagedWaypoints = ArrayDeque<FloatArray>()
    private val waypointQueue = ArrayDeque<FloatArray>()
    private val activeMissionBackup = ArrayList<FloatArray>()
    private var missionForwardDir = true

    private val astarBuf = FloatArray(600 * 3)
    private var astarCount = 0
    private val ctrlBuf = FloatArray(16 * 3)
    private var ctrlCount = 0
    private val bsplineBuf = FloatArray(240 * 3)
    private var bsplineCount = 0

    private val trailRing = FloatArray(240 * 3)
    private var trailCount = 0

    private val demoScanBuf = FloatArray(720 * 4)
    private var demoLidarAcc = 0f
    private var demoRosettePhase = 0f

    // 实机 /grid_map/occupancy_inflate 三维膨胀点云缓存 (扩容至 4096 体素 + 自动估计体素尺寸)
    private val rosVoxelsBuf = FloatArray(4096 * 3)
    private var rosVoxelCount = 0
    private var rosEstimatedVoxelHalf = 0.095f

    private val rosNearScanBuf = FloatArray(920 * 3)
    private var rosNearScanCount = 0
    private val rosPathBuf = FloatArray(420 * 3)
    private var rosPathCount = 0
    private val rosLioPathBuf = FloatArray(820 * 3)
    private var rosLioPathCount = 0

    private var rosPx4Mode = "OFFBOARD"
    private var rosArmed = true
    private var hasLiveBatteryTelemetry = false
    private var rosBatVolt = 16.2f
    private var rosBatCurrentAmps = 12.6f
    private var rosBatPct = 86

    private var rosOdomCounter = 0
    private var rosOdomHz = 0
    private var rosCloudCounter = 0
    private var rosCloudHz = 0
    private var lastCloudScanPts = 0
    private var lastHzTickMs = SystemClock.elapsedRealtime()

    private var replanTimer = 0f
    private var lastTickNs = System.nanoTime()

    // 多机集群僚机实例槽位 (1..3 号机独立物理/B样条状态，0 号机复用主状态以保持 100% 单机零开销兼容)
    private val peerUnits = Array(SwarmPalette.MAX_SWARM_DRONES) { idx -> SwarmPeerUnit(idx) }
    private val tmpFormSlotXY = FloatArray(2)
    private var lidarSwarmTurn = 0

    init {
        initSwarmDefaultHomesLocked()
        setScenePreset("pillars")
    }

    fun start() {
        if (executor != null) return
        lastTickNs = System.nanoTime()
        executor = Executors.newSingleThreadScheduledExecutor { r ->
            Thread(r, "EgoFastLioWorker").apply {
                isDaemon = true
                priority = Thread.NORM_PRIORITY + 1
            }
        }.also { exec ->
            exec.scheduleAtFixedRate({ safeTick() }, 0, 20, TimeUnit.MILLISECONDS)
        }
    }

    fun stop() {
        executor?.shutdownNow()
        executor = null
    }

    fun requestReplanAsync() {
        asyncReplanPending = true
    }

    fun resetRosSessionState() {
        synchronized(lock) {
            rosHomeCaptured = false
            rosHasRealQuat = false
            rosHasLiveTilt = false
            hasLiveBatteryTelemetry = false
            hasPreviewPin = false
            rosVoxelCount = 0
            stagedWaypoints.clear()
            waypointQueue.clear()
            activeMissionBackup.clear()
            missionForwardDir = true
        }
    }

    fun clearVoxelMap() {
        voxelMap.clear(hub.mapVoxelSize)
        synchronized(lock) {
            rosLioPathCount = 0
        }
    }

    fun clearWaypointQueue() {
        synchronized(lock) {
            hasPreviewPin = false
            stagedWaypoints.clear()
            waypointQueue.clear()
            activeMissionBackup.clear()
            missionForwardDir = true
        }
    }

    fun clearPreviewPin() {
        synchronized(lock) {
            hasPreviewPin = false
        }
    }

    fun hasActivePreviewPin(): Boolean {
        synchronized(lock) {
            return hasPreviewPin
        }
    }

    fun routeStatusSummary(): Triple<Int, Boolean, Float> {
        synchronized(lock) {
            return if (stagedWaypoints.isNotEmpty()) {
                Triple(stagedWaypoints.size, true, computeStagedRouteLengthLocked())
            } else {
                Triple(waypointQueue.size, false, 0f)
            }
        }
    }

    private fun evaluatePointClearanceLocked(tx: Float, ty: Float, tz: Float): Pair<Float, Boolean> {
        if (!hub.isRosMode) {
            var minClear = 9.5f
            for (i in 0 until obstacles.size) {
                val ob = obstacles[i]
                val d2d = signedDistToObs(tx, ty, ob)
                val d3d = if (tz <= ob.h + 0.18f) {
                    d2d
                } else {
                    hypot(max(0f, d2d), tz - ob.h)
                }
                if (d3d < minClear) minClear = d3d
            }
            val safe = minClear > max(0.12f, hub.obstaclesInflation * 0.65f)
            return max(0f, minClear) to safe
        } else {
            var minDSq = 90.25f
            for (i in 0 until rosVoxelCount) {
                val b = i * 3
                if (abs(rosVoxelsBuf[b + 2] - tz) <= 0.75f) {
                    val dx = rosVoxelsBuf[b] - tx
                    val dy = rosVoxelsBuf[b + 1] - ty
                    val d2 = dx * dx + dy * dy
                    if (d2 < minDSq) minDSq = d2
                }
            }
            for (i in 0 until rosNearScanCount) {
                val b = i * 3
                if (abs(rosNearScanBuf[b + 2] - tz) <= 0.75f) {
                    val dx = rosNearScanBuf[b] - tx
                    val dy = rosNearScanBuf[b + 1] - ty
                    val d2 = dx * dx + dy * dy
                    if (d2 < minDSq) minDSq = d2
                }
            }
            val minD = sqrt(minDSq)
            return minD to (minD > 0.28f)
        }
    }

    private fun computeStagedRouteLengthLocked(): Float {
        if (stagedWaypoints.isEmpty()) return 0f
        var sum = 0f
        var px = uavX
        var py = uavY
        var pz = uavZ
        for (wp in stagedWaypoints) {
            sum += hypot(hypot(wp[0] - px, wp[1] - py), wp[2] - pz)
            px = wp[0]
            py = wp[1]
            pz = wp[2]
        }
        return sum
    }

    fun placePreviewPin(wx: Float, wy: Float): PinInspectResult {
        synchronized(lock) {
            val tx = if (!hub.isRosMode) wx.coerceIn(-7.6f, 7.6f) else wx
            val ty = if (!hub.isRosMode) wy.coerceIn(-7.6f, 7.6f) else wy
            val tz = hub.goalZ
            val (clr, safe) = evaluatePointClearanceLocked(tx, ty, tz)

            hasPreviewPin = true
            previewX = tx
            previewY = ty
            previewZ = tz
            previewClearance = clr
            previewSafe = safe

            val dist = hypot(hypot(tx - uavX, ty - uavY), tz - uavZ)
            return PinInspectResult(tx, ty, tz, dist, clr, safe)
        }
    }

    fun nudgeGoalZ(delta: Float): Pair<Float, PinInspectResult?> {
        synchronized(lock) {
            val nextZ = (hub.goalZ + delta).coerceIn(0.4f, 4.0f)
            hub.goalZ = nextZ
            if (hasPreviewPin) {
                previewZ = nextZ
                val (clr, safe) = evaluatePointClearanceLocked(previewX, previewY, previewZ)
                previewClearance = clr
                previewSafe = safe
                val dist = hypot(hypot(previewX - uavX, previewY - uavY), previewZ - uavZ)
                return nextZ to PinInspectResult(previewX, previewY, previewZ, dist, clr, safe)
            } else {
                goalZ = nextZ
                stagedWaypoints.peekLast()?.let { it[2] = nextZ }
                if (!hub.isRosMode) asyncReplanPending = true
                return nextZ to null
            }
        }
    }

    fun confirmFlyImmediate(directWx: Float? = null, directWy: Float? = null): FloatArray? {
        synchronized(lock) {
            val tx: Float
            val ty: Float
            val tz: Float
            if (directWx != null && directWy != null) {
                tx = if (!hub.isRosMode) directWx.coerceIn(-7.6f, 7.6f) else directWx
                ty = if (!hub.isRosMode) directWy.coerceIn(-7.6f, 7.6f) else directWy
                tz = hub.goalZ
                val (_, safe) = evaluatePointClearanceLocked(tx, ty, tz)
                if (!hub.isRosMode && !safe) return null
            } else {
                if (!hasPreviewPin) return null
                if (!hub.isRosMode && !previewSafe) return null
                tx = previewX
                ty = previewY
                tz = previewZ
            }

            hasPreviewPin = false
            hub.autoPatrol = false
            stagedWaypoints.clear()
            waypointQueue.clear()
            activeMissionBackup.clear()
            missionForwardDir = true
            goalX = tx
            goalY = ty
            goalZ = tz
            fsmState = "GEN_NEW_TRAJ"
            if (!hub.isRosMode) asyncReplanPending = true
            return floatArrayOf(goalX, goalY, goalZ, uavX, uavY)
        }
    }

    fun appendPreviewPinToRoute(): Triple<Boolean, Int, Float> {
        synchronized(lock) {
            if (!hasPreviewPin) return Triple(false, stagedWaypoints.size, computeStagedRouteLengthLocked())
            if (!hub.isRosMode && !previewSafe) return Triple(false, stagedWaypoints.size, computeStagedRouteLengthLocked())

            if (stagedWaypoints.size >= MAX_WAYPOINT_QUEUE) {
                stagedWaypoints.pollFirst()
            }
            stagedWaypoints.addLast(floatArrayOf(previewX, previewY, previewZ))
            hasPreviewPin = false
            return Triple(true, stagedWaypoints.size, computeStagedRouteLengthLocked())
        }
    }

    fun undoLastWaypoint(): Triple<Int, Boolean, Float> {
        synchronized(lock) {
            if (stagedWaypoints.isNotEmpty()) {
                stagedWaypoints.pollLast()
                return Triple(stagedWaypoints.size, true, computeStagedRouteLengthLocked())
            }
            if (waypointQueue.isNotEmpty()) {
                waypointQueue.pollLast()
                return Triple(waypointQueue.size, false, 0f)
            }
            return Triple(0, false, 0f)
        }
    }

    fun commitStagedRoute(): Pair<FloatArray, Int>? {
        synchronized(lock) {
            if (stagedWaypoints.isEmpty()) return null
            hasPreviewPin = false
            hub.autoPatrol = false
            waypointQueue.clear()
            activeMissionBackup.clear()
            missionForwardDir = true
            for (wp in stagedWaypoints) {
                activeMissionBackup.add(floatArrayOf(wp[0], wp[1], wp[2]))
            }
            val first = stagedWaypoints.pollFirst() ?: return null
            val total = 1 + stagedWaypoints.size
            while (stagedWaypoints.isNotEmpty()) {
                waypointQueue.addLast(stagedWaypoints.pollFirst()!!)
            }
            goalX = first[0]
            goalY = first[1]
            goalZ = first[2]
            fsmState = "GEN_NEW_TRAJ"
            if (!hub.isRosMode) asyncReplanPending = true
            return floatArrayOf(goalX, goalY, goalZ, uavX, uavY) to total
        }
    }

    private fun refillOrFinishMissionQueueLocked(): FloatArray? {
        if (waypointQueue.isNotEmpty()) {
            return waypointQueue.pollFirst()
        }
        val mode = hub.routeLoopMode
        if (mode == 1 && activeMissionBackup.size >= 2) {
            for (wp in activeMissionBackup) {
                waypointQueue.addLast(floatArrayOf(wp[0], wp[1], wp[2]))
            }
            return waypointQueue.pollFirst()
        }
        if (mode == 2 && activeMissionBackup.size >= 2) {
            missionForwardDir = !missionForwardDir
            if (missionForwardDir) {
                for (i in 1 until activeMissionBackup.size) {
                    val wp = activeMissionBackup[i]
                    waypointQueue.addLast(floatArrayOf(wp[0], wp[1], wp[2]))
                }
            } else {
                for (i in activeMissionBackup.size - 2 downTo 0) {
                    val wp = activeMissionBackup[i]
                    waypointQueue.addLast(floatArrayOf(wp[0], wp[1], wp[2]))
                }
            }
            return waypointQueue.pollFirst()
        }
        if (activeMissionBackup.isNotEmpty()) {
            activeMissionBackup.clear()
            if (hub.routeFinishAction == 1) {
                return floatArrayOf(homeX, homeY, hub.goalZ)
            }
        }
        return null
    }

    fun snapshotOrchestratedWaypoints(): Pair<Boolean, List<WaypointNodeInfo>> {
        synchronized(lock) {
            val isStaged = stagedWaypoints.isNotEmpty() || waypointQueue.isEmpty()
            val src = if (stagedWaypoints.isNotEmpty()) stagedWaypoints else waypointQueue
            val out = ArrayList<WaypointNodeInfo>(src.size)
            var idx = 0
            for (wp in src) {
                val (clr, safe) = evaluatePointClearanceLocked(wp[0], wp[1], wp[2])
                out.add(WaypointNodeInfo(idx, wp[0], wp[1], wp[2], clr, safe))
                idx++
            }
            return isStaged to out
        }
    }

    fun appendCurrentUavAsWaypoint(): Triple<Boolean, Int, Float> {
        synchronized(lock) {
            if (stagedWaypoints.size >= MAX_WAYPOINT_QUEUE) {
                stagedWaypoints.pollFirst()
            }
            val z = uavZ.coerceIn(0.4f, 4.0f)
            stagedWaypoints.addLast(floatArrayOf(uavX, uavY, z))
            return Triple(true, stagedWaypoints.size, computeStagedRouteLengthLocked())
        }
    }

    fun moveOrchestratedWaypoint(index: Int, delta: Int): Boolean {
        synchronized(lock) {
            val targetDeque = if (stagedWaypoints.isNotEmpty()) stagedWaypoints else waypointQueue
            val list = targetDeque.toMutableList()
            val newIdx = index + delta
            if (index !in list.indices || newIdx !in list.indices) return false
            val tmp = list[index]
            list[index] = list[newIdx]
            list[newIdx] = tmp
            targetDeque.clear()
            for (item in list) targetDeque.addLast(item)
            return true
        }
    }

    fun nudgeOrchestratedWaypointZ(index: Int, deltaZ: Float): Float? {
        synchronized(lock) {
            val targetDeque = if (stagedWaypoints.isNotEmpty()) stagedWaypoints else waypointQueue
            val list = targetDeque.toMutableList()
            if (index !in list.indices) return null
            val nextZ = (list[index][2] + deltaZ).coerceIn(0.4f, 4.0f)
            list[index][2] = nextZ
            targetDeque.clear()
            for (item in list) targetDeque.addLast(item)
            return nextZ
        }
    }

    fun removeOrchestratedWaypointAt(index: Int): Int {
        synchronized(lock) {
            val targetDeque = if (stagedWaypoints.isNotEmpty()) stagedWaypoints else waypointQueue
            val list = targetDeque.toMutableList()
            if (index !in list.indices) return list.size
            list.removeAt(index)
            targetDeque.clear()
            for (item in list) targetDeque.addLast(item)
            return targetDeque.size
        }
    }

    /**
     * v3.8 一键航线三维净空自检与越障修正：
     * 逐站检测航点及相邻航段中点是否侵入三维膨胀障碍物安全裕度；
     * 若净空不足，优先在水平邻域搜索安全绕行点，仍受阻则自动抬升该航点高度越障。
     * @return Pair(修正航点数, 总航点数)
     */
    fun sanitizeOrchestratedRouteClearance(): Pair<Int, Int> {
        synchronized(lock) {
            val targetDeque = if (stagedWaypoints.isNotEmpty()) stagedWaypoints else waypointQueue
            if (targetDeque.isEmpty()) return 0 to 0
            val list = targetDeque.toMutableList()
            var fixedCount = 0
            for (i in list.indices) {
                val wp = list[i]
                val (_, safe) = evaluatePointClearanceLocked(wp[0], wp[1], wp[2])
                var chordBlocked = false
                if (i > 0) {
                    val prev = list[i - 1]
                    val mx = (prev[0] + wp[0]) * 0.5f
                    val my = (prev[1] + wp[1]) * 0.5f
                    val mz = (prev[2] + wp[2]) * 0.5f
                    chordBlocked = !evaluatePointClearanceLocked(mx, my, mz).second
                }
                if (!safe || chordBlocked) {
                    val candidate = findNearestSafePointLocked(wp[0], wp[1], wp[2])
                    val (_, candSafe) = evaluatePointClearanceLocked(candidate[0], candidate[1], candidate[2])
                    if (candSafe) {
                        list[i] = candidate
                    } else {
                        list[i] = floatArrayOf(wp[0], wp[1], (wp[2] + 0.85f).coerceAtMost(3.6f))
                    }
                    fixedCount++
                }
            }
            targetDeque.clear()
            for (item in list) targetDeque.addLast(item)
            return fixedCount to list.size
        }
    }

    fun serializeStagedWaypointsCsv(): String {
        synchronized(lock) {
            val src = if (stagedWaypoints.isNotEmpty()) stagedWaypoints else activeMissionBackup
            if (src.isEmpty()) return ""
            val sb = StringBuilder()
            var idx = 0
            for (wp in src) {
                if (idx > 0) sb.append(';')
                sb.append(String.format(Locale.US, "%.3f,%.3f,%.3f", wp[0], wp[1], wp[2]))
                idx++
            }
            return sb.toString()
        }
    }

    fun restoreStagedWaypointsCsv(raw: String) {
        if (raw.isBlank()) return
        synchronized(lock) {
            if (stagedWaypoints.isNotEmpty() || waypointQueue.isNotEmpty()) return
            for (token in raw.split(';')) {
                if (stagedWaypoints.size >= MAX_WAYPOINT_QUEUE) break
                val parts = token.split(',')
                if (parts.size == 3) {
                    val x = parts[0].toFloatOrNull() ?: continue
                    val y = parts[1].toFloatOrNull() ?: continue
                    val z = parts[2].toFloatOrNull() ?: continue
                    if (x.isFinite() && y.isFinite() && z.isFinite()) {
                        stagedWaypoints.addLast(floatArrayOf(x, y, z.coerceIn(0.4f, 4.0f)))
                    }
                }
            }
        }
    }

    fun reverseOrchestratedWaypoints(): Boolean {
        synchronized(lock) {
            val targetDeque = if (stagedWaypoints.isNotEmpty()) stagedWaypoints else waypointQueue
            if (targetDeque.size < 2) return false
            val rev = targetDeque.toList().asReversed()
            targetDeque.clear()
            for (item in rev) targetDeque.addLast(item)
            return true
        }
    }

    fun setHomeToCurrentUav(): FloatArray {
        synchronized(lock) {
            homeX = uavX
            homeY = uavY
            rosHomeCaptured = true
            return floatArrayOf(homeX, homeY, uavZ)
        }
    }

    private fun findNearestSafePointLocked(rawX: Float, rawY: Float, tz: Float): FloatArray {
        val clampLim = if (!hub.isRosMode) 6.9f else 999f
        val bx = rawX.coerceIn(-clampLim, clampLim)
        val by = rawY.coerceIn(-clampLim, clampLim)
        val (_, ok0) = evaluatePointClearanceLocked(bx, by, tz)
        if (ok0) return floatArrayOf(bx, by, tz)

        val radii = floatArrayOf(0.45f, 0.85f, 1.25f, 1.75f, 2.30f)
        for (r in radii) {
            for (k in 0 until 12) {
                val ang = (k / 12.0) * PI * 2.0
                val cx = (bx + cos(ang).toFloat() * r).coerceIn(-clampLim, clampLim)
                val cy = (by + sin(ang).toFloat() * r).coerceIn(-clampLim, clampLim)
                val (_, safe) = evaluatePointClearanceLocked(cx, cy, tz)
                if (safe) return floatArrayOf(cx, cy, tz)
            }
        }
        return floatArrayOf(bx, by, (tz + 1.2f).coerceAtMost(3.6f))
    }

    fun generateMissionTemplate(templateKey: String): Pair<Int, Float> {
        synchronized(lock) {
            hasPreviewPin = false
            waypointQueue.clear()
            activeMissionBackup.clear()
            stagedWaypoints.clear()

            val baseZ = hub.goalZ.coerceIn(0.6f, 3.5f)
            val cx = if (hub.isRosMode) uavX else 0.0f
            val cy = if (hub.isRosMode) uavY else 0.0f

            val rawPts: List<Pair<Float, Float>> = when (templateKey) {
                "rect" -> listOf(
                    (cx - 4.0f) to (cy - 3.0f),
                    (cx + 4.0f) to (cy - 3.0f),
                    (cx + 4.0f) to (cy + 3.0f),
                    (cx - 4.0f) to (cy + 3.0f)
                )
                "zigzag" -> listOf(
                    (cx - 4.4f) to (cy - 3.2f),
                    (cx + 4.4f) to (cy - 3.2f),
                    (cx + 4.4f) to cy,
                    (cx - 4.4f) to cy,
                    (cx - 4.4f) to (cy + 3.2f),
                    (cx + 4.4f) to (cy + 3.2f)
                )
                else -> {
                    val rad = 3.6f
                    (0 until 6).map { i ->
                        val a = (i / 6.0) * PI * 2.0
                        (cx + cos(a).toFloat() * rad) to (cy + sin(a).toFloat() * rad)
                    }
                }
            }

            for ((px, py) in rawPts) {
                stagedWaypoints.addLast(findNearestSafePointLocked(px, py, baseZ))
            }
            return stagedWaypoints.size to computeStagedRouteLengthLocked()
        }
    }

    fun setScenePreset(preset: String) {
        synchronized(lock) {
            val list = mutableListOf<Obstacle>()
            when (preset) {
                "pillars" -> {
                    // 混合不规则矩形立柱、L形转角短墙与高低错落障碍体，离散化为真实 EGO 膨胀体素块
                    list.add(Obstacle(-3.2f, -1.8f, 0.36f, 0.42f, 2.5f, false))
                    list.add(Obstacle(-1.2f, -3.4f, 0.45f, 0.35f, 2.1f, false))
                    list.add(Obstacle(-2.4f, 1.8f, 0.48f, 0.48f, 2.75f, true))
                    list.add(Obstacle(-0.2f, 0.1f, 0.52f, 0.38f, 2.9f, false))
                    list.add(Obstacle(0.35f, 0.45f, 0.28f, 0.62f, 1.85f, false)) // L型高低阶梯转角
                    list.add(Obstacle(1.9f, -2.0f, 0.44f, 0.44f, 2.4f, true))
                    list.add(Obstacle(2.3f, 1.6f, 0.42f, 0.56f, 2.8f, false))
                    list.add(Obstacle(-4.1f, 3.3f, 0.46f, 0.46f, 2.0f, true))
                    list.add(Obstacle(4.0f, -0.4f, 0.38f, 0.65f, 2.6f, false))
                    list.add(Obstacle(0.3f, 3.5f, 0.46f, 0.36f, 2.35f, false))
                    list.add(Obstacle(3.7f, -3.7f, 0.48f, 0.48f, 1.95f, true))
                }
                "corridor" -> {
                    list.add(Obstacle(-2.2f, -1.8f, 0.34f, 3.2f, 2.65f, false))
                    list.add(Obstacle(1.8f, 1.8f, 0.34f, 3.2f, 2.65f, false))
                    list.add(Obstacle(-0.2f, 4.5f, 3.2f, 0.32f, 2.45f, false))
                    list.add(Obstacle(0.2f, -4.5f, 3.2f, 0.32f, 2.45f, false))
                }
                "utrap" -> {
                    list.add(Obstacle(1.4f, 0.0f, 0.36f, 2.3f, 2.7f, false))
                    list.add(Obstacle(-0.4f, 2.15f, 1.75f, 0.36f, 2.7f, false))
                    list.add(Obstacle(-0.4f, -2.15f, 1.75f, 0.36f, 2.7f, false))
                    list.add(Obstacle(-3.5f, -2.7f, 0.48f, 0.42f, 2.2f, false))
                    list.add(Obstacle(3.7f, 2.7f, 0.46f, 0.52f, 2.4f, false))
                }
                "betel_gap" -> {
                    // 高密海南槟榔林行株距 + 中央窄缝斜穿走廊 (检验 Tilt-UAV-INDI 零俯仰平飞与主动俯仰穿缝)
                    list.add(Obstacle(-0.2f, -2.15f, 0.32f, 1.35f, 2.85f, false))
                    list.add(Obstacle(-0.2f, 2.15f, 0.32f, 1.35f, 2.85f, false))
                    val treeCoords = arrayOf(
                        -3.6f to -3.0f, -3.6f to 0.0f, -3.6f to 3.0f,
                        -1.9f to -3.8f, -1.9f to 3.8f,
                        1.7f to -3.6f, 1.7f to 3.6f,
                        3.4f to -2.6f, 3.4f to 0.3f, 3.4f to 3.1f
                    )
                    for ((tx, ty) in treeCoords) {
                        list.add(Obstacle(tx, ty, 0.30f, 0.30f, 2.95f, true))
                    }
                }
                else -> {
                    for (i in 0 until 10) {
                        val rx = (rng.nextFloat() - 0.5f) * 10.2f
                        val ry = (rng.nextFloat() - 0.5f) * 9.4f
                        if (hypot(rx - uavX, ry - uavY) < 1.8f) continue
                        if (hypot(rx - goalX, ry - goalY) < 1.6f) continue
                        val sx = 0.32f + rng.nextFloat() * 0.42f
                        val sy = 0.32f + rng.nextFloat() * 0.42f
                        list.add(Obstacle(rx, ry, sx, sy, 1.6f + rng.nextFloat() * 1.3f, rng.nextFloat() > 0.55f))
                    }
                }
            }
            obstacles = list
            hasPreviewPin = false
            stagedWaypoints.clear()
            waypointQueue.clear()
            activeMissionBackup.clear()
            missionForwardDir = true
            voxelMap.clear(hub.mapVoxelSize)
            rebuildOccupancyLocked(hub.obstaclesInflation)
            ensureSwarmSafeHomesAndPosesLocked()
            replanLocked(true)
            for (id in 1 until SwarmPalette.MAX_SWARM_DRONES) {
                replanPeerUnitLocked(peerUnits[id], isNew = true)
            }
        }
    }

    fun triggerReturnToHome(): FloatArray {
        synchronized(lock) {
            hub.autoPatrol = false
            hasPreviewPin = false
            stagedWaypoints.clear()
            waypointQueue.clear()
            activeMissionBackup.clear()
            missionForwardDir = true
            goalX = homeX
            goalY = homeY
            goalZ = hub.goalZ
            if (!hub.isRosMode) asyncReplanPending = true else fsmState = "GEN_NEW_TRAJ"
            return floatArrayOf(goalX, goalY, goalZ, uavX, uavY)
        }
    }

    fun triggerEmergencyHover() {
        synchronized(lock) {
            hub.autoPatrol = false
            hasPreviewPin = false
            stagedWaypoints.clear()
            waypointQueue.clear()
            activeMissionBackup.clear()
            missionForwardDir = true
            goalX = uavX
            goalY = uavY
            goalZ = uavZ
            vx = 0f
            vy = 0f
            vz = 0f
            ax = 0f
            ay = 0f
            bsplineCount = 0
            ctrlCount = 0
            fsmState = "EMERGENCY_STOP"
        }
    }

    fun clearTrail() {
        synchronized(lock) {
            trailCount = 0
            rosLioPathCount = 0
            for (id in 1 until SwarmPalette.MAX_SWARM_DRONES) {
                peerUnits[id].trailCount = 0
            }
        }
    }

    // ==================== 多机集群操控、选机与 ROS1 多机遥测注入接口 ====================

    private fun initSwarmDefaultHomesLocked() {
        val baseHomes = arrayOf(
            floatArrayOf(-5.5f, -4.2f, 5.2f, 3.8f),
            floatArrayOf(-5.9f, -2.0f, 4.8f, 5.2f),
            floatArrayOf(-3.5f, -4.8f, 5.6f, 2.0f),
            floatArrayOf(-3.9f, -2.4f, 3.8f, 4.6f)
        )
        for (id in 1 until SwarmPalette.MAX_SWARM_DRONES) {
            val u = peerUnits[id]
            val h = baseHomes[id]
            u.homeX = h[0]
            u.homeY = h[1]
            u.uavX = h[0]
            u.uavY = h[1]
            u.uavZ = 1.2f + (id % 2) * 0.12f
            u.goalX = h[2]
            u.goalY = h[3]
            u.goalZ = 1.2f
            u.batVolt = 16.32f - id * 0.11f
            u.batPct = (89 - id * 3).coerceIn(50, 98)
            u.replanAcc = id * 0.04f
        }
    }

    private fun ensureSwarmSafeHomesAndPosesLocked() {
        val safe0 = findNearestSafePointLocked(homeX, homeY, 1.2f)
        homeX = safe0[0]
        homeY = safe0[1]
        val cur0Safe = evaluatePointClearanceLocked(uavX, uavY, uavZ).second
        if (!cur0Safe) {
            uavX = homeX
            uavY = homeY
        }
        val rawOffsets = arrayOf(
            0f to 0f,
            -0.4f to 2.1f,
            1.9f to -0.5f,
            1.6f to 1.8f
        )
        for (id in 1 until SwarmPalette.MAX_SWARM_DRONES) {
            val u = peerUnits[id]
            val (ox, oy) = rawOffsets[id]
            val sh = findNearestSafePointLocked(homeX + ox, homeY + oy, 1.2f)
            u.homeX = sh[0]
            u.homeY = sh[1]
            if (!evaluatePointClearanceLocked(u.uavX, u.uavY, u.uavZ).second) {
                u.uavX = sh[0]
                u.uavY = sh[1]
            }
            val sg = findNearestSafePointLocked(u.goalX, u.goalY, u.goalZ)
            u.goalX = sg[0]
            u.goalY = sg[1]
        }
    }

    private fun getDroneWorldPosLocked(droneId: Int, outXyz: FloatArray) {
        if (droneId <= 0) {
            outXyz[0] = uavX
            outXyz[1] = uavY
            outXyz[2] = uavZ
        } else {
            val u = peerUnits[droneId.coerceIn(1, SwarmPalette.MAX_SWARM_DRONES - 1)]
            outXyz[0] = u.uavX
            outXyz[1] = u.uavY
            outXyz[2] = u.uavZ
        }
    }

    fun selectSwarmDrone(droneId: Int): FloatArray {
        synchronized(lock) {
            val maxId = if (hub.swarmModeEnabled) (hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES) - 1) else 0
            val clamped = droneId.coerceIn(0, maxId)
            hub.selectedDroneId = clamped
            val pos = FloatArray(3)
            getDroneWorldPosLocked(clamped, pos)
            return pos
        }
    }

    /**
     * 当处于多机模式时，检测地图单击是否落在某架无人机机体半径内，支持直接在 3D 场景点选切换主控飞机。
     */
    fun tryPickSwarmDroneAt(wx: Float, wy: Float, hitRadius: Float = 0.72f): Int {
        synchronized(lock) {
            if (!hub.swarmModeEnabled) return -1
            val cnt = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
            var bestId = -1
            var bestD = hitRadius
            val pos = FloatArray(3)
            for (id in 0 until cnt) {
                getDroneWorldPosLocked(id, pos)
                val d = hypot(wx - pos[0], wy - pos[1])
                if (d < bestD) {
                    bestD = d
                    bestId = id
                }
            }
            return bestId
        }
    }

    /**
     * 统一指点飞行下发（兼容单机模式、多机单控选定机模式、全队几何编队集结模式）。
     */
    fun dispatchFlyCommand(directWx: Float? = null, directWy: Float? = null): List<DroneGoalCmd> {
        synchronized(lock) {
            val tx: Float
            val ty: Float
            val tz: Float
            if (directWx != null && directWy != null) {
                tx = if (!hub.isRosMode) directWx.coerceIn(-7.6f, 7.6f) else directWx
                ty = if (!hub.isRosMode) directWy.coerceIn(-7.6f, 7.6f) else directWy
                tz = hub.goalZ
                val (_, safe) = evaluatePointClearanceLocked(tx, ty, tz)
                if (!hub.isRosMode && !safe && (!hub.swarmModeEnabled || hub.swarmCommandScope == 0)) {
                    return emptyList()
                }
            } else {
                if (!hasPreviewPin) return emptyList()
                if (!hub.isRosMode && !previewSafe && (!hub.swarmModeEnabled || hub.swarmCommandScope == 0)) {
                    return emptyList()
                }
                tx = previewX
                ty = previewY
                tz = previewZ
            }

            hasPreviewPin = false
            hub.autoPatrol = false
            stagedWaypoints.clear()
            waypointQueue.clear()
            activeMissionBackup.clear()
            missionForwardDir = true

            if (!hub.swarmModeEnabled) {
                goalX = tx
                goalY = ty
                goalZ = tz
                fsmState = "GEN_NEW_TRAJ"
                if (!hub.isRosMode) asyncReplanPending = true
                return listOf(DroneGoalCmd(0, goalX, goalY, goalZ, uavX, uavY))
            }

            val cnt = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
            val selId = hub.selectedDroneId.coerceIn(0, cnt - 1)
            if (hub.swarmCommandScope == 0) {
                // 多机单控模式：仅向当前选定飞机下发目标
                if (selId == 0) {
                    goalX = tx
                    goalY = ty
                    goalZ = tz
                    fsmState = "GEN_NEW_TRAJ"
                    if (!hub.isRosMode) asyncReplanPending = true
                    return listOf(DroneGoalCmd(0, goalX, goalY, goalZ, uavX, uavY))
                } else {
                    val u = peerUnits[selId]
                    u.goalX = tx
                    u.goalY = ty
                    u.goalZ = tz
                    u.fsmState = "GEN_NEW_TRAJ"
                    if (!hub.isRosMode) replanPeerUnitLocked(u, isNew = true)
                    return listOf(DroneGoalCmd(selId, u.goalX, u.goalY, u.goalZ, u.uavX, u.uavY))
                }
            }

            // 全队协同编队模式：按机群质心至锚点航向角解算各机防撞编队槽位并同步下发
            val heading = computeSwarmHeadingToTargetLocked(tx, ty, cnt)
            val cmds = ArrayList<DroneGoalCmd>(cnt)
            for (id in 0 until cnt) {
                SwarmPalette.computeWorldSlot(
                    tx,
                    ty,
                    heading,
                    id,
                    cnt,
                    hub.swarmFormationType,
                    hub.swarmSpacingMeters,
                    tmpFormSlotXY
                )
                val safeSlot = findNearestSafePointLocked(tmpFormSlotXY[0], tmpFormSlotXY[1], tz)
                if (id == 0) {
                    goalX = safeSlot[0]
                    goalY = safeSlot[1]
                    goalZ = safeSlot[2]
                    fsmState = "GEN_NEW_TRAJ"
                    if (!hub.isRosMode) asyncReplanPending = true
                    cmds.add(DroneGoalCmd(0, goalX, goalY, goalZ, uavX, uavY))
                } else {
                    val u = peerUnits[id]
                    u.goalX = safeSlot[0]
                    u.goalY = safeSlot[1]
                    u.goalZ = safeSlot[2]
                    u.fsmState = "GEN_NEW_TRAJ"
                    if (!hub.isRosMode) replanPeerUnitLocked(u, isNew = true)
                    cmds.add(DroneGoalCmd(id, u.goalX, u.goalY, u.goalZ, u.uavX, u.uavY))
                }
            }
            return cmds
        }
    }

    /**
     * 以当前领航机/机群中心为锚点，立即按新选定的队形与机间距重排整队。
     */
    fun rearrangeSwarmFormationNow(): List<DroneGoalCmd> {
        synchronized(lock) {
            if (!hub.swarmModeEnabled) return emptyList()
            hub.autoPatrol = false
            val cnt = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
            val anchorX = if (hypot(goalX - uavX, goalY - uavY) > 0.6f) goalX else uavX
            val anchorY = if (hypot(goalX - uavX, goalY - uavY) > 0.6f) goalY else uavY
            val heading = yaw
            val tz = hub.goalZ
            val cmds = ArrayList<DroneGoalCmd>(cnt)
            for (id in 0 until cnt) {
                SwarmPalette.computeWorldSlot(
                    anchorX,
                    anchorY,
                    heading,
                    id,
                    cnt,
                    hub.swarmFormationType,
                    hub.swarmSpacingMeters,
                    tmpFormSlotXY
                )
                val safeSlot = findNearestSafePointLocked(tmpFormSlotXY[0], tmpFormSlotXY[1], tz)
                if (id == 0) {
                    goalX = safeSlot[0]
                    goalY = safeSlot[1]
                    goalZ = safeSlot[2]
                    fsmState = "GEN_NEW_TRAJ"
                    if (!hub.isRosMode) asyncReplanPending = true
                    cmds.add(DroneGoalCmd(0, goalX, goalY, goalZ, uavX, uavY))
                } else {
                    val u = peerUnits[id]
                    u.goalX = safeSlot[0]
                    u.goalY = safeSlot[1]
                    u.goalZ = safeSlot[2]
                    u.fsmState = "GEN_NEW_TRAJ"
                    if (!hub.isRosMode) replanPeerUnitLocked(u, isNew = true)
                    cmds.add(DroneGoalCmd(id, u.goalX, u.goalY, u.goalZ, u.uavX, u.uavY))
                }
            }
            return cmds
        }
    }

    fun dispatchSwarmReturnToHome(): List<DroneGoalCmd> {
        synchronized(lock) {
            hub.autoPatrol = false
            hasPreviewPin = false
            stagedWaypoints.clear()
            waypointQueue.clear()
            activeMissionBackup.clear()
            missionForwardDir = true

            if (!hub.swarmModeEnabled) {
                val r = triggerReturnToHome()
                return listOf(DroneGoalCmd(0, r[0], r[1], r[2], r[3], r[4]))
            }
            val cnt = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
            val selId = hub.selectedDroneId.coerceIn(0, cnt - 1)
            if (hub.swarmCommandScope == 0) {
                return if (selId == 0) {
                    goalX = homeX
                    goalY = homeY
                    goalZ = hub.goalZ
                    if (!hub.isRosMode) asyncReplanPending = true else fsmState = "GEN_NEW_TRAJ"
                    listOf(DroneGoalCmd(0, goalX, goalY, goalZ, uavX, uavY))
                } else {
                    val u = peerUnits[selId]
                    u.goalX = u.homeX
                    u.goalY = u.homeY
                    u.goalZ = hub.goalZ
                    if (!hub.isRosMode) replanPeerUnitLocked(u, isNew = true) else u.fsmState = "GEN_NEW_TRAJ"
                    listOf(DroneGoalCmd(selId, u.goalX, u.goalY, u.goalZ, u.uavX, u.uavY))
                }
            }

            val cmds = ArrayList<DroneGoalCmd>(cnt)
            goalX = homeX
            goalY = homeY
            goalZ = hub.goalZ
            if (!hub.isRosMode) asyncReplanPending = true else fsmState = "GEN_NEW_TRAJ"
            cmds.add(DroneGoalCmd(0, goalX, goalY, goalZ, uavX, uavY))
            for (id in 1 until cnt) {
                val u = peerUnits[id]
                u.goalX = u.homeX
                u.goalY = u.homeY
                u.goalZ = hub.goalZ
                if (!hub.isRosMode) replanPeerUnitLocked(u, isNew = true) else u.fsmState = "GEN_NEW_TRAJ"
                cmds.add(DroneGoalCmd(id, u.goalX, u.goalY, u.goalZ, u.uavX, u.uavY))
            }
            return cmds
        }
    }

    fun dispatchSwarmEmergencyHover(): List<DroneGoalCmd> {
        synchronized(lock) {
            triggerEmergencyHover()
            val cmds = ArrayList<DroneGoalCmd>(SwarmPalette.MAX_SWARM_DRONES)
            cmds.add(DroneGoalCmd(0, uavX, uavY, uavZ, uavX, uavY))
            if (hub.swarmModeEnabled) {
                val cnt = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
                for (id in 1 until cnt) {
                    val u = peerUnits[id]
                    u.goalX = u.uavX
                    u.goalY = u.uavY
                    u.goalZ = u.uavZ
                    u.vx = 0f
                    u.vy = 0f
                    u.vz = 0f
                    u.ax = 0f
                    u.ay = 0f
                    u.bsplineCount = 0
                    u.ctrlCount = 0
                    u.fsmState = "EMERGENCY_STOP"
                    cmds.add(DroneGoalCmd(id, u.uavX, u.uavY, u.uavZ, u.uavX, u.uavY))
                }
            }
            return cmds
        }
    }

    fun onRosSwarmOdometry(
        droneId: Int,
        x: Float,
        y: Float,
        z: Float,
        lvX: Float,
        lvY: Float,
        lvZ: Float,
        yawRad: Float,
        rollRad: Float = 0f,
        pitchRad: Float = 0f
    ) {
        if (droneId <= 0 || droneId >= SwarmPalette.MAX_SWARM_DRONES) return
        synchronized(lock) {
            val u = peerUnits[droneId]
            if (!u.rosHomeCaptured && x.isFinite() && y.isFinite()) {
                u.homeX = x
                u.homeY = y
                u.rosHomeCaptured = true
            }
            u.uavX = x
            u.uavY = y
            u.uavZ = z
            u.vx = lvX
            u.vy = lvY
            u.vz = lvZ
            u.yaw = yawRad
            u.smoothRoll += (rollRad - u.smoothRoll) * 0.35f
            u.smoothPitch += (pitchRad - u.smoothPitch) * 0.35f
            u.online = true
            u.lastRosRxMs = SystemClock.elapsedRealtime()
            appendPeerTrailLocked(u, x, y, z)
        }
    }

    fun onRosSwarmOptimalPath(droneId: Int, flatXyz: FloatArray, count: Int) {
        if (droneId <= 0 || droneId >= SwarmPalette.MAX_SWARM_DRONES) return
        val safeCnt = min(count, 235)
        synchronized(lock) {
            val u = peerUnits[droneId]
            System.arraycopy(flatXyz, 0, u.rosPathBuf, 0, safeCnt * 3)
            u.rosPathCount = safeCnt
            if (safeCnt > 0) {
                val endBase = (safeCnt - 1) * 3
                u.goalX = u.rosPathBuf[endBase]
                u.goalY = u.rosPathBuf[endBase + 1]
                u.goalZ = u.rosPathBuf[endBase + 2]
                u.fsmState = "EXEC_TRAJ"
            }
        }
    }

    fun onRosSwarmBattery(droneId: Int, volt: Float, pct: Int) {
        if (droneId <= 0 || droneId >= SwarmPalette.MAX_SWARM_DRONES) return
        synchronized(lock) {
            val u = peerUnits[droneId]
            u.batVolt = volt
            u.batPct = pct.coerceIn(0, 100)
        }
    }

    // v3.5: 三维旋翼下洗气流椭球等效距离 (垂直 Z 轴缩放 0.52 -> 等效 1.92 倍垂直防下洗安全柱高)
    private fun downwashEllipsoidDist(dx: Float, dy: Float, dz: Float): Float {
        val dHoriz = hypot(dx, dy)
        val dzWeighted = dz * 0.52f
        return hypot(dHoriz, dzWeighted)
    }

    private fun computeSwarmHeadingToTargetLocked(tx: Float, ty: Float, count: Int): Float {
        var sumX = uavX
        var sumY = uavY
        var validN = 1
        val nowMs = SystemClock.elapsedRealtime()
        for (id in 1 until count) {
            val p = peerUnits[id]
            if (hub.isRosMode && (!p.online || nowMs - p.lastRosRxMs > 4500L)) continue
            sumX += p.uavX
            sumY += p.uavY
            validN++
        }
        val cx = sumX / validN
        val cy = sumY / validN
        val dx = tx - cx
        val dy = ty - cy
        return if (hypot(dx, dy) > 0.25f) atan2(dy, dx) else yaw
    }

    private fun appendPeerTrailLocked(u: SwarmPeerUnit, x: Float, y: Float, z: Float) {
        if (u.trailCount > 0) {
            val last = (u.trailCount - 1) * 3
            if (hypot(x - u.trailRing[last], y - u.trailRing[last + 1]) < 0.16f) return
        }
        if (u.trailCount == 160) {
            System.arraycopy(u.trailRing, 3, u.trailRing, 0, 159 * 3)
            u.trailCount = 159
        }
        val base = u.trailCount * 3
        u.trailRing[base] = x
        u.trailRing[base + 1] = y
        u.trailRing[base + 2] = z
        u.trailCount++
    }

    private fun stepSwarmWingmenDemoLocked(dt: Float) {
        if (!hub.swarmModeEnabled) return
        val cnt = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
        for (id in 1 until cnt) {
            val u = peerUnits[id]
            u.replanAcc += dt
            if (u.replanAcc >= 0.16f) {
                u.replanAcc = 0f
                replanPeerUnitLocked(u, isNew = false)
            }

            val dxG = u.goalX - u.uavX
            val dyG = u.goalY - u.uavY
            val dzG = u.goalZ - u.uavZ
            val distGoal = hypot(hypot(dxG, dyG), dzG)
            if (distGoal < 0.30f) {
                u.vx *= 0.80f
                u.vy *= 0.80f
                u.vz *= 0.80f
                u.ax = 0f
                u.ay = 0f
                if (hub.autoPatrol && !hasPreviewPin && stagedWaypoints.isEmpty()) {
                    // 僚机协同巡航：自动围绕领航机目标按当前编队队形生成无碰撞伴飞航点
                    val hdg = atan2(goalY - uavY, goalX - uavX)
                    SwarmPalette.computeWorldSlot(
                        goalX,
                        goalY,
                        hdg,
                        id,
                        cnt,
                        hub.swarmFormationType,
                        hub.swarmSpacingMeters,
                        tmpFormSlotXY
                    )
                    val safeG = findNearestSafePointLocked(tmpFormSlotXY[0], tmpFormSlotXY[1], hub.goalZ)
                    if (hypot(safeG[0] - u.uavX, safeG[1] - u.uavY) > 0.65f) {
                        u.goalX = safeG[0]
                        u.goalY = safeG[1]
                        u.goalZ = safeG[2]
                        replanPeerUnitLocked(u, isNew = true)
                    } else {
                        u.fsmState = "WAIT_TARGET"
                    }
                } else {
                    u.fsmState = "WAIT_TARGET"
                }
                continue
            }

            var lookX = u.goalX
            var lookY = u.goalY
            var lookZ = u.goalZ
            if (u.bsplineCount > 2) {
                val idx = min(u.bsplineCount - 1, 7) * 3
                lookX = u.bsplineBuf[idx]
                lookY = u.bsplineBuf[idx + 1]
                lookZ = u.bsplineBuf[idx + 2]
            }

            val ex = lookX - u.uavX
            val ey = lookY - u.uavY
            val ez = lookZ - u.uavZ
            val dLook = max(1e-4f, hypot(hypot(ex, ey), ez))
            val desSpeed = min(hub.maxVel, distGoal * 1.65f)
            var desVx = (ex / dLook) * desSpeed
            var desVy = (ey / dLook) * desSpeed
            var desVz = (ez * 2.2f).coerceIn(-1.5f, 1.5f)

            // 机间互避速度势场 (ORCA 右手切向绕行 + 径向排斥，防止多机窄走廊对头死锁)
            val minSep = max(0.95f, hub.obstaclesInflation * 2.6f)
            for (otherId in 0 until cnt) {
                if (otherId == id) continue
                val ox = if (otherId == 0) uavX else peerUnits[otherId].uavX
                val oy = if (otherId == 0) uavY else peerUnits[otherId].uavY
                val oz = if (otherId == 0) uavZ else peerUnits[otherId].uavZ
                val rx = u.uavX - ox
                val ry = u.uavY - oy
                val rz = u.uavZ - oz
                val d2 = hypot(rx, ry)
                val dEllip = downwashEllipsoidDist(rx, ry, rz)
                if (dEllip < minSep) {
                    val push = ((minSep - dEllip) / minSep) * hub.maxVel * 1.22f
                    val safeD2 = max(0.08f, d2)
                    val nx = if (d2 > 1e-3f) rx / safeD2 else cos(id * 1.7f)
                    val ny = if (d2 > 1e-3f) ry / safeD2 else sin(id * 1.7f)
                    // v3.5: 椭球径向推离 + 右手规则切向绕飞，彻底防止上下叠飞落入前机旋翼下洗尾流区
                    desVx += (nx * 0.80f - ny * 0.54f) * push
                    desVy += (ny * 0.80f + nx * 0.54f) * push
                    val zSign = if (abs(rz) > 0.08f) (if (rz > 0f) 1f else -1f) else (if (id % 2 == 1) 1f else -1f)
                    desVz += zSign * 0.28f * push
                }
            }

            var dvx = desVx - u.vx
            var dvy = desVy - u.vy
            var dvz = desVz - u.vz
            val maxDv = hub.maxAcc * dt * 2.2f
            val dvNorm = hypot(hypot(dvx, dvy), dvz)
            if (dvNorm > maxDv && dvNorm > 1e-5f) {
                val sc = maxDv / dvNorm
                dvx *= sc
                dvy *= sc
                dvz *= sc
            }

            u.ax = dvx / max(dt, 0.008f)
            u.ay = dvy / max(dt, 0.008f)
            u.vx += dvx
            u.vy += dvy
            u.vz += dvz
            u.uavX += u.vx * dt
            u.uavY += u.vy * dt
            u.uavZ = (u.uavZ + u.vz * dt).coerceIn(0.35f, 3.8f)

            if (hypot(u.vx, u.vy) > 0.12f) {
                val targetYaw = atan2(u.vy, u.vx)
                var diff = targetYaw - u.yaw
                while (diff > PI.toFloat()) diff -= (PI * 2.0).toFloat()
                while (diff < -PI.toFloat()) diff += (PI * 2.0).toFloat()
                u.yaw += diff * min(1.0f, dt * 6.5f)
            }

            val cosY = cos(u.yaw)
            val sinY = sin(u.yaw)
            val vFwd = u.vx * cosY + u.vy * sinY
            val vLeft = -u.vx * sinY + u.vy * cosY
            val aFwd = u.ax * cosY + u.ay * sinY
            val aLeft = -u.ax * sinY + u.ay * cosY
            val unitIsTilt = hub.isDroneTiltRig(id)
            val unitTiltActive = unitIsTilt && hub.tiltCoordMode != TiltConfigSpec.MODE_FIXED_QUAD
            val maxTRad = (hub.tiltMaxAngleDeg.coerceIn(15f, 60f)) * 0.017453292f
            val biasRad = (hub.tiltPitchBiasDeg.coerceIn(-30f, 30f)) * 0.017453292f
            val tPitch = when {
                !unitTiltActive -> (-(aFwd * 0.072f + vFwd * 0.065f)).coerceIn(-0.44f, 0.44f)
                hub.tiltCoordMode == TiltConfigSpec.MODE_ZERO_PITCH -> 0f
                else -> (biasRad - aFwd * 0.016f).coerceIn(-0.50f, 0.50f)
            }
            val desPeerTilt = when {
                !unitTiltActive -> 0f
                hub.tiltCoordMode == TiltConfigSpec.MODE_ZERO_PITCH ->
                    atan2(aFwd + vFwd * 0.40f, 9.81f).coerceIn(-maxTRad, maxTRad)
                else ->
                    (atan2(aFwd + vFwd * 0.40f, 9.81f) + biasRad).coerceIn(-maxTRad, maxTRad)
            }
            val kTiltPeer = 1f - exp(-dt * 13.0f)
            if (!unitIsTilt) {
                u.tiltS1Rad = 0f
                u.tiltS2Rad = 0f
            } else {
                u.tiltS1Rad += (desPeerTilt - u.tiltS1Rad) * kTiltPeer
                u.tiltS2Rad = u.tiltS1Rad
            }
            val tRoll = (-(aLeft * 0.070f + vLeft * 0.045f)).coerceIn(-0.48f, 0.48f)
            val kAtt = 1f - exp(-dt * 10.5f)
            u.smoothPitch += (tPitch - u.smoothPitch) * kAtt
            u.smoothRoll += (tRoll - u.smoothRoll) * kAtt

            appendPeerTrailLocked(u, u.uavX, u.uavY, u.uavZ)
        }
    }

    private fun replanPeerUnitLocked(u: SwarmPeerUnit, isNew: Boolean) {
        val distGoal = hypot(hypot(u.goalX - u.uavX, u.goalY - u.uavY), u.goalZ - u.uavZ)
        if (distGoal < 0.22f) {
            u.fsmState = "WAIT_TARGET"
            u.ctrlCount = 0
            u.bsplineCount = 0
            return
        }
        u.fsmState = if (isNew) {
            "GEN_NEW_TRAJ"
        } else if (distGoal > 1.2f) {
            "REPLAN_TRAJ"
        } else {
            "EXEC_TRAJ"
        }

        val numCtrl = 11
        u.ctrlCount = numCtrl
        val horizon = min(distGoal, 7.2f)
        val dirX = (u.goalX - u.uavX) / max(1e-4f, distGoal)
        val dirY = (u.goalY - u.uavY) / max(1e-4f, distGoal)
        for (k in 0 until numCtrl) {
            val r = k.toFloat() / (numCtrl - 1)
            val s = horizon * r
            u.ctrlBuf[k * 3] = u.uavX + dirX * s
            u.ctrlBuf[k * 3 + 1] = u.uavY + dirY * s
            u.ctrlBuf[k * 3 + 2] = u.uavZ + (u.goalZ - u.uavZ) * r
        }

        val safeMargin = lastBuiltInflation + 0.34f
        val peerMargin = max(0.92f, lastBuiltInflation * 2.5f)
        val activeCnt = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)

        for (it in 0 until 16) {
            for (c in 1 until numCtrl - 1) {
                val b = c * 3
                val sxForce = 0.5f * (u.ctrlBuf[b - 3] + u.ctrlBuf[b + 3]) - u.ctrlBuf[b]
                val syForce = 0.5f * (u.ctrlBuf[b - 2] + u.ctrlBuf[b + 4]) - u.ctrlBuf[b + 1]
                val szForce = 0.5f * (u.ctrlBuf[b - 1] + u.ctrlBuf[b + 5]) - u.ctrlBuf[b + 2]

                val wx = u.ctrlBuf[b]
                val wy = u.ctrlBuf[b + 1]
                var bestD = 99f
                var gradX = 0f
                var gradY = 0f
                for (o in 0 until obstacles.size) {
                    val ob = obstacles[o]
                    if (u.ctrlBuf[b + 2] > ob.h + 0.25f) continue
                    val d = signedDistToObs(wx, wy, ob)
                    if (d < bestD) {
                        bestD = d
                        val eps = 0.04f
                        val dx = signedDistToObs(wx + eps, wy, ob) - d
                        val dy = signedDistToObs(wx, wy + eps, ob) - d
                        val norm = max(1e-5f, hypot(dx, dy))
                        gradX = dx / norm
                        gradY = dy / norm
                    }
                }
                var rx = 0f
                var ry = 0f
                if (bestD < safeMargin) {
                    val push = (safeMargin - bestD) * 0.44f
                    rx += gradX * push
                    ry += gradY * push
                }

                // EGO-Swarm 机间轨迹斥力梯度：避让其他无人机的实时位置与轨迹走廊
                for (otherId in 0 until activeCnt) {
                    if (otherId == u.id) continue
                    val ox = if (otherId == 0) uavX else peerUnits[otherId].uavX
                    val oy = if (otherId == 0) uavY else peerUnits[otherId].uavY
                    val dxP = wx - ox
                    val dyP = wy - oy
                    val dP = hypot(dxP, dyP)
                    if (dP < peerMargin && dP > 1e-4f) {
                        val pPush = (peerMargin - dP) * 0.36f
                        rx += (dxP / dP) * pPush
                        ry += (dyP / dP) * pPush
                    }
                }

                u.ctrlBuf[b] += 0.34f * sxForce + rx
                u.ctrlBuf[b + 1] += 0.34f * syForce + ry
                u.ctrlBuf[b + 2] += 0.35f * szForce
            }
        }

        val extCount = numCtrl + 4
        for (i in 0 until extCount) {
            val srcIdx = (i - 2).coerceIn(0, numCtrl - 1) * 3
            extCtrlScratch[i * 3] = u.ctrlBuf[srcIdx]
            extCtrlScratch[i * 3 + 1] = u.ctrlBuf[srcIdx + 1]
            extCtrlScratch[i * 3 + 2] = u.ctrlBuf[srcIdx + 2]
        }

        var outPts = 0
        val segSteps = 5
        for (i in 0 until extCount - 3) {
            val p0 = i * 3
            val p1 = (i + 1) * 3
            val p2 = (i + 2) * 3
            val p3 = (i + 3) * 3
            for (s in 0 until segSteps) {
                val t = s.toFloat() / segSteps
                val t2 = t * t
                val t3 = t2 * t
                val b0 = (1f - 3f * t + 3f * t2 - t3) / 6f
                val b1 = (4f - 6f * t2 + 3f * t3) / 6f
                val b2 = (1f + 3f * t + 3f * t2 - 3f * t3) / 6f
                val b3 = t3 / 6f
                val dst = outPts * 3
                u.bsplineBuf[dst] = b0 * extCtrlScratch[p0] + b1 * extCtrlScratch[p1] + b2 * extCtrlScratch[p2] + b3 * extCtrlScratch[p3]
                u.bsplineBuf[dst + 1] = b0 * extCtrlScratch[p0 + 1] + b1 * extCtrlScratch[p1 + 1] + b2 * extCtrlScratch[p2 + 1] + b3 * extCtrlScratch[p3 + 1]
                u.bsplineBuf[dst + 2] = b0 * extCtrlScratch[p0 + 2] + b1 * extCtrlScratch[p1 + 2] + b2 * extCtrlScratch[p2 + 2] + b3 * extCtrlScratch[p3 + 2]
                outPts++
            }
        }
        val lastC = (numCtrl - 1) * 3
        u.bsplineBuf[outPts * 3] = u.ctrlBuf[lastC]
        u.bsplineBuf[outPts * 3 + 1] = u.ctrlBuf[lastC + 1]
        u.bsplineBuf[outPts * 3 + 2] = u.ctrlBuf[lastC + 2]
        outPts++
        u.bsplineCount = outPts
    }

    fun onRosOdometry(
        x: Float,
        y: Float,
        z: Float,
        lvX: Float,
        lvY: Float,
        lvZ: Float,
        yawRad: Float,
        rollRad: Float = 0f,
        pitchRad: Float = 0f,
        qw: Float = 1f,
        qx: Float = 0f,
        qy: Float = 0f,
        qz: Float = 0f,
        hasQuat: Boolean = false
    ) {
        var nextDispatched: FloatArray? = null
        synchronized(lock) {
            if (!rosHomeCaptured && x.isFinite() && y.isFinite()) {
                homeX = x
                homeY = y
                rosHomeCaptured = true
            }
            val dtEst = 0.02f
            ax = ((lvX - vx) / dtEst).coerceIn(-12f, 12f)
            ay = ((lvY - vy) / dtEst).coerceIn(-12f, 12f)
            hub.lastRosOdomWallMs = SystemClock.elapsedRealtime()
            uavX = x
            uavY = y
            uavZ = z
            vx = lvX
            vy = lvY
            vz = lvZ
            yaw = yawRad
            if (hasQuat) {
                rosHasRealQuat = true
                rosRawRoll = rollRad
                rosRawPitch = pitchRad
                curQuatW = qw
                curQuatX = qx
                curQuatY = qy
                curQuatZ = qz
            }
            rosOdomCounter++
            appendTrailLocked(x, y, z)

            if ((waypointQueue.isNotEmpty() || activeMissionBackup.isNotEmpty()) && hypot(goalX - x, goalY - y) < 0.45f) {
                val nxt = refillOrFinishMissionQueueLocked()
                if (nxt != null) {
                    goalX = nxt[0]
                    goalY = nxt[1]
                    goalZ = nxt[2]
                    nextDispatched = floatArrayOf(goalX, goalY, goalZ, x, y)
                }
            }
        }
        nextDispatched?.let { d ->
            onRosWaypointAdvanced?.invoke(d[0], d[1], d[2], d[3], d[4])
        }
    }

    fun onRosFastLioCloud(xyzi: FloatArray, count: Int) {
        voxelMap.ensureVoxelResolution(hub.mapVoxelSize)
        voxelMap.insertScan(xyzi, count)

        synchronized(lock) {
            rosCloudCounter++
            lastCloudScanPts = count
            val step = max(1, count / 900)
            var nearCnt = 0
            var idx = 0
            val curZ = uavZ
            while (idx < count && nearCnt < 915) {
                val b = idx * 4
                val pz = xyzi[b + 2]
                if (abs(pz - curZ) <= 0.90f) {
                    val dst = nearCnt * 3
                    rosNearScanBuf[dst] = xyzi[b]
                    rosNearScanBuf[dst + 1] = xyzi[b + 1]
                    rosNearScanBuf[dst + 2] = pz
                    nearCnt++
                }
                idx += step
            }
            rosNearScanCount = nearCnt
            if (hasPreviewPin) {
                val (clr, safe) = evaluatePointClearanceLocked(previewX, previewY, previewZ)
                previewClearance = clr
                previewSafe = safe
            }
        }
    }

    fun onRosFastLioPath(flatXyz: FloatArray, count: Int) {
        val safeCnt = min(count, 815)
        synchronized(lock) {
            System.arraycopy(flatXyz, 0, rosLioPathBuf, 0, safeCnt * 3)
            rosLioPathCount = safeCnt
        }
    }

    fun onRosOptimalPath(flatXyz: FloatArray, count: Int) {
        val safeCnt = min(count, 415)
        synchronized(lock) {
            System.arraycopy(flatXyz, 0, rosPathBuf, 0, safeCnt * 3)
            rosPathCount = safeCnt
            if (safeCnt > 0) {
                trajLen = 0f
                for (i in 1 until safeCnt) {
                    val a = (i - 1) * 3
                    val b = i * 3
                    trajLen += hypot(
                        hypot(rosPathBuf[b] - rosPathBuf[a], rosPathBuf[b + 1] - rosPathBuf[a + 1]),
                        rosPathBuf[b + 2] - rosPathBuf[a + 2]
                    )
                }
                if (waypointQueue.isEmpty()) {
                    val endBase = (safeCnt - 1) * 3
                    goalX = rosPathBuf[endBase]
                    goalY = rosPathBuf[endBase + 1]
                    goalZ = rosPathBuf[endBase + 2]
                }
                fsmState = "EXEC_TRAJ"
            }
        }
    }

    fun onRosInflatedCloud(flatXyz: FloatArray, count: Int) {
        val safeCnt = min(count, MAX_INFLATE_VOXELS)
        // 快速采样相邻非零间距，自适应匹配机载 grid_map/resolution (常见 0.10m / 0.15m / 0.20m)
        var minStep = 99f
        val probeN = min(safeCnt, 48)
        for (i in 1 until probeN) {
            val dx = abs(flatXyz[i * 3] - flatXyz[(i - 1) * 3])
            val dy = abs(flatXyz[i * 3 + 1] - flatXyz[(i - 1) * 3 + 1])
            val dz = abs(flatXyz[i * 3 + 2] - flatXyz[(i - 1) * 3 + 2])
            val step = max(dx, max(dy, dz))
            if (step in 0.05f..0.35f && step < minStep) {
                minStep = step
            }
        }
        val estHalf = if (minStep < 1.0f) (minStep * 0.49f).coerceIn(0.045f, 0.16f) else 0.095f

        synchronized(lock) {
            System.arraycopy(flatXyz, 0, rosVoxelsBuf, 0, safeCnt * 3)
            rosVoxelCount = safeCnt
            rosEstimatedVoxelHalf = estHalf
        }
    }

    fun onRosMavrosState(mode: String, armed: Boolean) {
        synchronized(lock) {
            rosPx4Mode = mode
            rosArmed = armed
        }
    }

    /**
     * v3.6 双链路热备：当 5GHz ROS1 Wi-Fi 在密林遮挡下逾 1.5s 未收到里程计时，
     * 自动无缝采纳 ELRS 2.4GHz/900MHz Backpack 转发的 MAVLink 姿态与解锁心跳。
     */
    fun onMavlinkHeartbeatFallback(mode: String, armed: Boolean) {
        val now = SystemClock.elapsedRealtime()
        if (hub.isRosMode && (now - hub.lastRosOdomWallMs) < 1500L) return
        synchronized(lock) {
            rosPx4Mode = mode
            rosArmed = armed
        }
    }

    fun onMavlinkAttitudeFallback(rollFlu: Float, pitchFlu: Float, yawEnu: Float) {
        val now = SystemClock.elapsedRealtime()
        if (!hub.isRosMode || (now - hub.lastRosOdomWallMs) < 1500L) return
        synchronized(lock) {
            rosRawRoll = rollFlu.coerceIn(-1.2f, 1.2f)
            rosRawPitch = pitchFlu.coerceIn(-1.2f, 1.2f)
            yaw = yawEnu
            rosHasRealQuat = true
        }
    }

    /**
     * 接收来自实机 ROS1 (`Friendzhb/tilt-uav-indi` / `mpc_ros1_for_px4` / `TerrainScan-NMPC` / MAVROS)
     * 的双舵机倾转角 (rad) 与可选四电机归一化推力遥测。
     */
    /**
     * 当用户在「单机/多机」或「倾转矢量 4M2S / 普通固定 X4」或「异构混编」之间实时热切换时调用：
     * 立即清零切回普通 X4 构型机体的残留舵机角度、角速度与饱和标志，防止不同模式状态串扰。
     */
    fun notifyOperatingModeChanged() {
        synchronized(lock) {
            if (!hub.swarmModeEnabled) {
                hub.selectedDroneId = 0
            } else {
                val cnt = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
                if (hub.selectedDroneId >= cnt) hub.selectedDroneId = 0
            }
            if (!hub.isDroneTiltRig(0)) {
                simServo1Rad = 0f
                simServo2Rad = 0f
                simServo1RateRadS = 0f
                simServo2RateRadS = 0f
                cmdServo1Rad = 0f
                cmdServo2Rad = 0f
                simServoSaturated = false
                simManipulability = 1.0f
                rosHasLiveTilt = false
            }
            for (id in 1 until SwarmPalette.MAX_SWARM_DRONES) {
                if (!hub.isDroneTiltRig(id)) {
                    val u = peerUnits[id]
                    u.tiltS1Rad = 0f
                    u.tiltS2Rad = 0f
                    u.rosHasLiveTilt = false
                }
            }
        }
    }

    fun onRosSwarmTiltTelemetry(
        droneId: Int,
        s1Rad: Float,
        s2Rad: Float,
        m0: Float = -1f,
        m1: Float = -1f,
        m2: Float = -1f,
        m3: Float = -1f,
        isActuatorFallback: Boolean = false
    ) {
        if (droneId <= 0) {
            onRosTiltTelemetry(s1Rad, s2Rad, m0, m1, m2, m3, isActuatorFallback)
            return
        }
        if (droneId >= SwarmPalette.MAX_SWARM_DRONES || !s1Rad.isFinite() || !s2Rad.isFinite()) return
        if (!hub.isDroneTiltRig(droneId)) return
        synchronized(lock) {
            val u = peerUnits[droneId]
            u.rosTiltS1 = s1Rad.coerceIn(-1.15f, 1.15f)
            u.rosTiltS2 = s2Rad.coerceIn(-1.15f, 1.15f)
            u.rosHasLiveTilt = true
            u.lastTiltRxMs = SystemClock.elapsedRealtime()
            u.online = true
            u.lastRosRxMs = u.lastTiltRxMs
        }
    }

    fun onRosTiltTelemetry(
        s1Rad: Float,
        s2Rad: Float,
        m0: Float = -1f,
        m1: Float = -1f,
        m2: Float = -1f,
        m3: Float = -1f,
        isActuatorFallback: Boolean = false
    ) {
        if (!s1Rad.isFinite() || !s2Rad.isFinite()) return
        // 若 0 号机当前设为普通固定 X4，忽略 /mavros/actuator_control 混控辅助通道，仅更新四电机转速
        if (!hub.isDroneTiltRig(0)) {
            if (isActuatorFallback && m0 >= 0f) {
                synchronized(lock) {
                    simMotorNorm[0] = m0.coerceIn(0f, 1f)
                    if (m1 >= 0f) simMotorNorm[1] = m1.coerceIn(0f, 1f)
                    if (m2 >= 0f) simMotorNorm[2] = m2.coerceIn(0f, 1f)
                    if (m3 >= 0f) simMotorNorm[3] = m3.coerceIn(0f, 1f)
                }
            }
            return
        }
        synchronized(lock) {
            val prev1 = rosTiltS1Rad
            val prev2 = rosTiltS2Rad
            val now = SystemClock.elapsedRealtime()
            val dtSec = ((now - rosLastTiltRxMs).coerceIn(5L, 250L)) * 0.001f
            rosTiltS1Rad = s1Rad.coerceIn(-1.15f, 1.15f)
            rosTiltS2Rad = s2Rad.coerceIn(-1.15f, 1.15f)
            if (rosHasLiveTilt) {
                simServo1RateRadS = ((rosTiltS1Rad - prev1) / dtSec).coerceIn(-6f, 6f)
                simServo2RateRadS = ((rosTiltS2Rad - prev2) / dtSec).coerceIn(-6f, 6f)
            }
            rosHasLiveTilt = true
            rosLastTiltRxMs = now
            if (m0 >= 0f) simMotorNorm[0] = m0.coerceIn(0f, 1f)
            if (m1 >= 0f) simMotorNorm[1] = m1.coerceIn(0f, 1f)
            if (m2 >= 0f) simMotorNorm[2] = m2.coerceIn(0f, 1f)
            if (m3 >= 0f) simMotorNorm[3] = m3.coerceIn(0f, 1f)
        }
    }

    fun onRosBattery(volt: Float, pct: Int, currentAmps: Float = 0f) {
        synchronized(lock) {
            hasLiveBatteryTelemetry = true
            rosBatVolt = volt
            rosBatPct = pct
            if (currentAmps >= 0.01f && currentAmps < 250f) {
                rosBatCurrentAmps = currentAmps
            }
        }
    }

    private fun safeTick() {
        try {
            val nowNs = System.nanoTime()
            val dt = ((nowNs - lastTickNs) * 1e-9f).coerceIn(0.002f, 0.05f)
            lastTickNs = nowNs

            voxelMap.ensureVoxelResolution(hub.mapVoxelSize)
            voxelMap.updateFocusAnchor(uavX, uavY)

            var simScanCount = 0
            synchronized(lock) {
                val infChanged = abs(hub.obstaclesInflation - lastBuiltInflation) > 0.005f
                if (infChanged) {
                    rebuildOccupancyLocked(hub.obstaclesInflation)
                }
                if (asyncReplanPending || infChanged) {
                    asyncReplanPending = false
                    if (!hasPreviewPin) goalZ = hub.goalZ
                    if (!hub.isRosMode) replanLocked(true)
                }

                if (!hub.isRosMode) {
                    stepDemoPhysicsLocked(dt)
                    stepSwarmWingmenDemoLocked(dt)
                    demoLidarAcc += dt
                    if (demoLidarAcc >= 0.066f) {
                        demoLidarAcc = 0f
                        simScanCount = generateDemo3DLidarScanLocked()
                        lastCloudScanPts = simScanCount
                    }
                } else {
                    val nowMs = SystemClock.elapsedRealtime()
                    if (nowMs - lastHzTickMs >= 1000L) {
                        rosOdomHz = rosOdomCounter
                        rosCloudHz = rosCloudCounter
                        rosOdomCounter = 0
                        rosCloudCounter = 0
                        lastHzTickMs = nowMs
                    }
                }
                stepHsiFilterAndRadarLocked(dt)
                publishSnapshotLocked()
            }

            if (simScanCount > 0) {
                voxelMap.insertScan(demoScanBuf, simScanCount)
            }
        } catch (_: Throwable) {
        }
    }

    private fun generateDemo3DLidarScanLocked(): Int {
        var outCnt = 0
        val maxRange = 7.4f
        val azSteps = when (hub.perfMode) {
            1 -> 56
            2 -> 32
            else -> 48
        }
        val elLevels = floatArrayOf(-0.22f, -0.10f, -0.02f, 0.06f, 0.16f, 0.28f, 0.42f)
        demoRosettePhase += 0.37f

        val activeCnt = if (hub.swarmModeEnabled) hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES) else 1
        lidarSwarmTurn = (lidarSwarmTurn + 1) % activeCnt
        val scanOriginX = if (lidarSwarmTurn == 0) uavX else peerUnits[lidarSwarmTurn].uavX
        val scanOriginY = if (lidarSwarmTurn == 0) uavY else peerUnits[lidarSwarmTurn].uavY
        val scanOriginZ = if (lidarSwarmTurn == 0) uavZ else peerUnits[lidarSwarmTurn].uavZ
        val scanOriginYaw = if (lidarSwarmTurn == 0) yaw else peerUnits[lidarSwarmTurn].yaw

        for (a in 0 until azSteps) {
            val az = scanOriginYaw + demoRosettePhase + (a.toFloat() / azSteps) * (PI.toFloat() * 2f)
            val cosA = cos(az)
            val sinA = sin(az)

            for (eIdx in elLevels.indices) {
                if (outCnt >= 650) break
                val el = elLevels[eIdx] + sin(demoRosettePhase * 2.3f + a * 0.4f) * 0.04f
                val cosE = cos(el)
                val sinE = sin(el)
                val dirX = cosE * cosA
                val dirY = cosE * sinA
                val dirZ = sinE

                var t = 0.28f
                var hit = false
                var hitX = 0f
                var hitY = 0f
                var hitZ = 0f
                var hitIntensity = 0.5f

                for (step in 0 until 16) {
                    val wx = scanOriginX + dirX * t
                    val wy = scanOriginY + dirY * t
                    val wz = scanOriginZ + dirZ * t

                    if (wz <= 0.02f && dirZ < -0.01f) {
                        val tGround = (0.01f - scanOriginZ) / dirZ
                        if (tGround in 0.25f..maxRange) {
                            hitX = scanOriginX + dirX * tGround
                            hitY = scanOriginY + dirY * tGround
                            hitZ = 0.01f
                            hitIntensity = 0.22f
                            hit = true
                        }
                        break
                    }

                    if ((abs(wx) >= 7.85f || abs(wy) >= 7.85f) && wz in 0f..2.4f) {
                        hitX = wx.coerceIn(-7.88f, 7.88f)
                        hitY = wy.coerceIn(-7.88f, 7.88f)
                        hitZ = wz
                        hitIntensity = 0.48f
                        hit = true
                        break
                    }

                    var minSd = 99f
                    var hitObH = 0f
                    for (o in 0 until obstacles.size) {
                        val ob = obstacles[o]
                        if (wz > ob.h) {
                            val d2d = max(0f, signedDistToObs(wx, wy, ob))
                            val dz = wz - ob.h
                            val d3d = hypot(d2d, dz)
                            if (d3d < minSd) minSd = d3d
                        } else {
                            val d = signedDistToObs(wx, wy, ob)
                            if (d < minSd) {
                                minSd = d
                                hitObH = ob.h
                            }
                        }
                    }

                    if (minSd <= 0.045f && wz <= hitObH + 0.05f) {
                        hitX = wx + (rng.nextFloat() - 0.5f) * 0.022f
                        hitY = wy + (rng.nextFloat() - 0.5f) * 0.022f
                        hitZ = wz.coerceAtLeast(0.02f)
                        hitIntensity = (0.55f + (hitZ / 3.0f) * 0.42f).coerceIn(0.1f, 1.0f)
                        hit = true
                        break
                    }

                    t += max(0.15f, minSd * 0.84f)
                    if (t > maxRange || wz > 3.5f) break
                }

                if (hit) {
                    if (hitZ <= 0.04f && (a + eIdx) % 3 != 0) continue
                    val b = outCnt * 4
                    demoScanBuf[b] = hitX
                    demoScanBuf[b + 1] = hitY
                    demoScanBuf[b + 2] = hitZ
                    demoScanBuf[b + 3] = hitIntensity
                    outCnt++
                }
            }
        }
        return outCnt
    }

    private fun stepDemoPhysicsLocked(dt: Float) {
        replanTimer += dt
        if (replanTimer >= 0.14f) {
            replanTimer = 0f
            replanLocked(false)
        }

        val dxG = goalX - uavX
        val dyG = goalY - uavY
        val dzG = goalZ - uavZ
        val distGoal = hypot(hypot(dxG, dyG), dzG)

        if (distGoal < 0.30f) {
            if (waypointQueue.isNotEmpty() || activeMissionBackup.isNotEmpty()) {
                val nxt = refillOrFinishMissionQueueLocked()
                if (nxt != null) {
                    goalX = nxt[0]
                    goalY = nxt[1]
                    goalZ = nxt[2]
                    replanLocked(true)
                    return
                }
            }
            vx *= 0.82f
            vy *= 0.82f
            vz *= 0.82f
            ax = 0f
            ay = 0f
            if (hub.autoPatrol && !hasPreviewPin && stagedWaypoints.isEmpty()) {
                pickRandomGoalLocked()
            } else {
                fsmState = "WAIT_TARGET"
            }
            return
        }

        var lookX = goalX
        var lookY = goalY
        var lookZ = goalZ
        if (bsplineCount > 2) {
            val idx = min(bsplineCount - 1, 7) * 3
            lookX = bsplineBuf[idx]
            lookY = bsplineBuf[idx + 1]
            lookZ = bsplineBuf[idx + 2]
        }

        val ex = lookX - uavX
        val ey = lookY - uavY
        val ez = lookZ - uavZ
        val dLook = max(1e-4f, hypot(hypot(ex, ey), ez))

        val desSpeed = min(hub.maxVel, distGoal * 1.65f)
        var desVx = (ex / dLook) * desSpeed
        var desVy = (ey / dLook) * desSpeed
        var desVz = (ez * 2.2f).coerceIn(-1.5f, 1.5f)

        if (hub.swarmModeEnabled) {
            val cnt = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
            val minSep = max(0.95f, hub.obstaclesInflation * 2.6f)
            for (otherId in 1 until cnt) {
                val peer = peerUnits[otherId]
                val rx = uavX - peer.uavX
                val ry = uavY - peer.uavY
                val rz = uavZ - peer.uavZ
                val d2 = hypot(rx, ry)
                val d3 = hypot(d2, rz)
                if (d3 < minSep && d2 > 1e-4f) {
                    val push = ((minSep - d3) / minSep) * hub.maxVel * 1.15f
                    val nx = rx / d2
                    val ny = ry / d2
                    desVx += (nx * 0.78f - ny * 0.52f) * push
                    desVy += (ny * 0.78f + nx * 0.52f) * push
                    desVz += 0.18f * push
                }
            }
        }

        var dvx = desVx - vx
        var dvy = desVy - vy
        var dvz = desVz - vz
        val maxDv = hub.maxAcc * dt * 2.2f
        val dvNorm = hypot(hypot(dvx, dvy), dvz)
        if (dvNorm > maxDv && dvNorm > 1e-5f) {
            val scale = maxDv / dvNorm
            dvx *= scale
            dvy *= scale
            dvz *= scale
        }

        ax = dvx / max(dt, 0.008f)
        ay = dvy / max(dt, 0.008f)
        vx += dvx
        vy += dvy
        vz += dvz
        uavX += vx * dt
        uavY += vy * dt
        uavZ += vz * dt

        if (hypot(vx, vy) > 0.12f) {
            val targetYaw = atan2(vy, vx)
            var diff = targetYaw - yaw
            while (diff > PI.toFloat()) diff -= (PI * 2.0).toFloat()
            while (diff < -PI.toFloat()) diff += (PI * 2.0).toFloat()
            yaw += diff * min(1.0f, dt * 6.5f)
        }

        appendTrailLocked(uavX, uavY, uavZ)
    }

    private fun pickRandomGoalLocked() {
        for (attempt in 0 until 40) {
            val gx = (rng.nextFloat() - 0.5f) * 12.2f
            val gy = (rng.nextFloat() - 0.5f) * 10.6f
            if (hypot(gx - uavX, gy - uavY) < 4.5f) continue
            val ix = clampGrid(gx)
            val iy = clampGrid(gy)
            if (distField[iy * GRID_N + ix] > hub.obstaclesInflation + 0.45f) {
                goalX = gx
                goalY = gy
                goalZ = (hub.goalZ + (rng.nextFloat() - 0.5f) * 0.55f).coerceIn(0.6f, 2.5f)
                replanLocked(true)
                return
            }
        }
    }

    private fun appendTrailLocked(x: Float, y: Float, z: Float) {
        if (trailCount > 0) {
            val last = (trailCount - 1) * 3
            if (hypot(x - trailRing[last], y - trailRing[last + 1]) < 0.14f) return
        }
        if (trailCount == 240) {
            System.arraycopy(trailRing, 3, trailRing, 0, 239 * 3)
            trailCount = 239
        }
        val base = trailCount * 3
        trailRing[base] = x
        trailRing[base + 1] = y
        trailRing[base + 2] = z
        trailCount++
    }

    private fun signedDistToObs(wx: Float, wy: Float, ob: Obstacle): Float {
        if (ob.isCircle) {
            val dx = wx - ob.cx
            val dy = wy - ob.cy
            return sqrt(dx * dx + dy * dy) - ob.rx
        }
        val dx = abs(wx - ob.cx) - ob.rx
        val dy = abs(wy - ob.cy) - ob.ry
        val ox = max(dx, 0f)
        val oy = max(dy, 0f)
        return sqrt(ox * ox + oy * oy) + min(max(dx, dy), 0f)
    }

    /**
     * 生成与真实 EGO-Planner `/grid_map/occupancy_inflate` 一致的三维离散膨胀占据体素阵列：
     * 仅提取三维膨胀体表面壳层与高低阶梯暴露面 (剔除完全被埋在实心内部不可见的冗余体素)，
     * 既有 100% 完整的 RViz Boxes 立体堆叠体积感，又将顶点数压缩在 60FPS 最优甜点！
     */
    private fun rebuildOccupancyLocked(infR: Float) {
        lastBuiltInflation = infR
        for (gy in 0 until GRID_N) {
            val wy = -MAP_HALF + (gy + 0.5f) * GRID_RES
            for (gx in 0 until GRID_N) {
                val wx = -MAP_HALF + (gx + 0.5f) * GRID_RES
                var minD = 99f
                var maxInflatedH = 0f
                for (i in 0 until obstacles.size) {
                    val ob = obstacles[i]
                    val d = signedDistToObs(wx, wy, ob)
                    if (d < minD) minD = d
                    if (d <= infR) {
                        val hInf = ob.h + min(0.25f, infR * 0.5f)
                        if (hInf > maxInflatedH) maxInflatedH = hInf
                    }
                }
                val idx = gy * GRID_N + gx
                distField[idx] = minD
                cellInflatedTopH[idx] = maxInflatedH
                occGrid[idx] = when {
                    minD <= 0.02f -> 2
                    minD <= infR -> 1
                    else -> 0
                }
            }
        }

        var vCount = 0
        val vzStep = GRID_RES
        val halfBox = GRID_RES * 0.485f
        val neighborDirs = intArrayOf(1, 0, -1, 0, 0, 1, 0, -1)

        for (gy in 0 until GRID_N) {
            val wy = -MAP_HALF + (gy + 0.5f) * GRID_RES
            for (gx in 0 until GRID_N) {
                if (vCount >= MAX_INFLATE_VOXELS) break
                val idx = gy * GRID_N + gx
                val topH = cellInflatedTopH[idx]
                if (topH <= 0.05f) continue

                val wx = -MAP_HALF + (gx + 0.5f) * GRID_RES
                var minNeighborH = topH
                for (d in 0 until 4) {
                    val nx = gx + neighborDirs[d * 2]
                    val ny = gy + neighborDirs[d * 2 + 1]
                    val nh = if (nx in 0 until GRID_N && ny in 0 until GRID_N) {
                        cellInflatedTopH[ny * GRID_N + nx]
                    } else {
                        0f
                    }
                    if (nh < minNeighborH) minNeighborH = nh
                }

                val layers = max(1, floor(topH / vzStep).toInt())
                for (lz in 0 until layers) {
                    if (vCount >= MAX_INFLATE_VOXELS) break
                    val wz = (lz + 0.5f) * vzStep
                    val isTopCap = (lz == layers - 1)
                    val isExposedSide = (wz > minNeighborH - vzStep * 0.45f)
                    if (isTopCap || isExposedSide) {
                        val b = vCount * 4
                        cachedGridVoxels[b] = wx
                        cachedGridVoxels[b + 1] = wy
                        cachedGridVoxels[b + 2] = wz
                        cachedGridVoxels[b + 3] = if (isTopCap) (halfBox + 10.0f) else halfBox
                        vCount++
                    }
                }
            }
        }
        cachedGridVoxelCount = vCount
    }

    private fun clampGrid(w: Float): Int {
        return floor((w + MAP_HALF) / GRID_RES).toInt().coerceIn(0, GRID_N - 1)
    }

    private fun replanLocked(isNew: Boolean) {
        val t0 = SystemClock.elapsedRealtimeNanos()
        val distGoal = hypot(hypot(goalX - uavX, goalY - uavY), goalZ - uavZ)
        if (distGoal < 0.22f) {
            fsmState = "WAIT_TARGET"
            astarCount = 0
            ctrlCount = 0
            bsplineCount = 0
            trajLen = 0f
            return
        }
        fsmState = if (isNew) {
            "GEN_NEW_TRAJ"
        } else if (distGoal > 1.2f) {
            "REPLAN_TRAJ"
        } else {
            "EXEC_TRAJ"
        }

        val sx = clampGrid(uavX)
        val sy = clampGrid(uavY)
        val gx = clampGrid(goalX)
        val gy = clampGrid(goalY)
        val startIdx = sy * GRID_N + sx
        val goalIdx = gy * GRID_N + gx

        astarSearchEpoch++
        if (astarSearchEpoch >= Int.MAX_VALUE - 4) {
            astarVisitStamp.fill(0)
            astarClosedStamp.fill(0)
            astarSearchEpoch = 1
        }
        val ep = astarSearchEpoch

        astarVisitStamp[startIdx] = ep
        astarGScore[startIdx] = 0f
        astarParent[startIdx] = -1

        var heapSize = 0
        fun heapPush(node: Int, fVal: Float) {
            var cur = ++heapSize
            while (cur > 1) {
                val p = cur ushr 1
                if (heapF[p] <= fVal) break
                heapNodes[cur] = heapNodes[p]
                heapF[cur] = heapF[p]
                cur = p
            }
            heapNodes[cur] = node
            heapF[cur] = fVal
        }

        fun heapPop(): Int {
            val top = heapNodes[1]
            val lastN = heapNodes[heapSize]
            val lastF = heapF[heapSize]
            heapSize--
            var cur = 1
            while ((cur shl 1) <= heapSize) {
                var child = cur shl 1
                if (child + 1 <= heapSize && heapF[child + 1] < heapF[child]) child++
                if (lastF <= heapF[child]) break
                heapNodes[cur] = heapNodes[child]
                heapF[cur] = heapF[child]
                cur = child
            }
            if (heapSize > 0) {
                heapNodes[cur] = lastN
                heapF[cur] = lastF
            }
            return top
        }

        heapPush(startIdx, hypot((gx - sx).toFloat(), (gy - sy).toFloat()))

        val dxArr = intArrayOf(1, -1, 0, 0, 1, 1, -1, -1)
        val dyArr = intArrayOf(0, 0, 1, -1, 1, -1, 1, -1)
        val costArr = floatArrayOf(1f, 1f, 1f, 1f, 1.414f, 1.414f, 1.414f, 1.414f)

        var bestIdx = startIdx
        var bestH = 1e9f
        var iter = 0
        while (heapSize > 0 && iter < 4096) {
            iter++
            val cIdx = heapPop()
            if (astarClosedStamp[cIdx] == ep) continue
            astarClosedStamp[cIdx] = ep
            if (cIdx == goalIdx) {
                bestIdx = goalIdx
                break
            }

            val cx = cIdx % GRID_N
            val cy = cIdx / GRID_N
            val hVal = hypot((gx - cx).toFloat(), (gy - cy).toFloat())
            if (hVal < bestH) {
                bestH = hVal
                bestIdx = cIdx
            }

            val baseG = astarGScore[cIdx]
            for (d in 0 until 8) {
                val nx = cx + dxArr[d]
                val ny = cy + dyArr[d]
                if (nx !in 0 until GRID_N || ny !in 0 until GRID_N) continue
                val nIdx = ny * GRID_N + nx
                if (astarClosedStamp[nIdx] == ep) continue
                val occ = occGrid[nIdx]
                if (occ == 2 && nIdx != startIdx) continue

                var pen = if (occ == 1) 8.0f else 0.0f
                val clr = distField[nIdx]
                if (clr > 0f && clr < lastBuiltInflation + 0.45f) {
                    pen += (lastBuiltInflation + 0.45f - clr) * 2.5f
                }
                val tg = baseG + costArr[d] + pen
                if (astarVisitStamp[nIdx] != ep || tg < astarGScore[nIdx]) {
                    astarVisitStamp[nIdx] = ep
                    astarGScore[nIdx] = tg
                    astarParent[nIdx] = cIdx
                    if (heapSize < GRID_CELLS + 8) {
                        heapPush(nIdx, tg + hypot((gx - nx).toFloat(), (gy - ny).toFloat()))
                    }
                }
            }
        }

        val reachedGoal = (astarVisitStamp[goalIdx] == ep && astarParent[goalIdx] != -1)
        var rLen = 0
        var tr = if (reachedGoal) goalIdx else bestIdx
        while (tr != -1 && rLen < 495) {
            val px = tr % GRID_N
            val py = tr / GRID_N
            revXScratch[rLen] = -MAP_HALF + (px + 0.5f) * GRID_RES
            revYScratch[rLen] = -MAP_HALF + (py + 0.5f) * GRID_RES
            rLen++
            if (tr == startIdx) break
            tr = if (astarVisitStamp[tr] == ep) astarParent[tr] else -1
        }

        if (rLen < 2) {
            astarBuf[0] = uavX
            astarBuf[1] = uavY
            astarBuf[2] = uavZ
            astarBuf[3] = goalX
            astarBuf[4] = goalY
            astarBuf[5] = goalZ
            astarCount = 2
        } else {
            astarCount = rLen
            for (i in 0 until rLen) {
                val src = rLen - 1 - i
                astarBuf[i * 3] = revXScratch[src]
                astarBuf[i * 3 + 1] = revYScratch[src]
                astarBuf[i * 3 + 2] = goalZ
            }
            astarBuf[0] = uavX
            astarBuf[1] = uavY
            astarBuf[2] = uavZ
            if (reachedGoal) {
                astarBuf[(rLen - 1) * 3] = goalX
                astarBuf[(rLen - 1) * 3 + 1] = goalY
            }
        }

        cumScratch[0] = 0f
        for (i in 1 until astarCount) {
            val dx = astarBuf[i * 3] - astarBuf[(i - 1) * 3]
            val dy = astarBuf[i * 3 + 1] - astarBuf[(i - 1) * 3 + 1]
            cumScratch[i] = cumScratch[i - 1] + hypot(dx, dy)
        }
        val horizon = min(cumScratch[astarCount - 1], 7.5f)
        val numCtrl = 13
        ctrlCount = numCtrl
        for (k in 0 until numCtrl) {
            val targetS = (horizon * k) / (numCtrl - 1)
            var seg = 0
            while (seg < astarCount - 2 && cumScratch[seg + 1] < targetS) seg++
            val segLen = max(1e-5f, cumScratch[seg + 1] - cumScratch[seg])
            val ratio = ((targetS - cumScratch[seg]) / segLen).coerceIn(0f, 1f)
            ctrlBuf[k * 3] = astarBuf[seg * 3] + (astarBuf[(seg + 1) * 3] - astarBuf[seg * 3]) * ratio
            ctrlBuf[k * 3 + 1] = astarBuf[seg * 3 + 1] + (astarBuf[(seg + 1) * 3 + 1] - astarBuf[seg * 3 + 1]) * ratio
            ctrlBuf[k * 3 + 2] = uavZ + (goalZ - uavZ) * (k.toFloat() / (numCtrl - 1))
        }

        val safeMargin = lastBuiltInflation + 0.32f
        for (it in 0 until 20) {
            for (c in 1 until numCtrl - 1) {
                val b = c * 3
                val sxForce = 0.5f * (ctrlBuf[b - 3] + ctrlBuf[b + 3]) - ctrlBuf[b]
                val syForce = 0.5f * (ctrlBuf[b - 2] + ctrlBuf[b + 4]) - ctrlBuf[b + 1]
                val szForce = 0.5f * (ctrlBuf[b - 1] + ctrlBuf[b + 5]) - ctrlBuf[b + 2]

                var bestD = 99f
                var gradX = 0f
                var gradY = 0f
                val wx = ctrlBuf[b]
                val wy = ctrlBuf[b + 1]
                for (o in 0 until obstacles.size) {
                    val ob = obstacles[o]
                    if (ctrlBuf[b + 2] > ob.h + 0.25f) continue
                    val d = signedDistToObs(wx, wy, ob)
                    if (d < bestD) {
                        bestD = d
                        val eps = 0.04f
                        val dx = signedDistToObs(wx + eps, wy, ob) - d
                        val dy = signedDistToObs(wx, wy + eps, ob) - d
                        val norm = max(1e-5f, hypot(dx, dy))
                        gradX = dx / norm
                        gradY = dy / norm
                    }
                }
                var rx = 0f
                var ry = 0f
                if (bestD < safeMargin) {
                    val push = (safeMargin - bestD) * 0.42f
                    rx = gradX * push
                    ry = gradY * push
                }
                ctrlBuf[b] += 0.34f * sxForce + rx
                ctrlBuf[b + 1] += 0.34f * syForce + ry
                ctrlBuf[b + 2] += 0.35f * szForce
            }
        }

        val extCount = numCtrl + 4
        for (i in 0 until extCount) {
            val srcIdx = (i - 2).coerceIn(0, numCtrl - 1) * 3
            extCtrlScratch[i * 3] = ctrlBuf[srcIdx]
            extCtrlScratch[i * 3 + 1] = ctrlBuf[srcIdx + 1]
            extCtrlScratch[i * 3 + 2] = ctrlBuf[srcIdx + 2]
        }

        var outPts = 0
        val segSteps = 6
        for (i in 0 until extCount - 3) {
            val p0 = i * 3
            val p1 = (i + 1) * 3
            val p2 = (i + 2) * 3
            val p3 = (i + 3) * 3
            for (s in 0 until segSteps) {
                val t = s.toFloat() / segSteps
                val t2 = t * t
                val t3 = t2 * t
                val b0 = (1f - 3f * t + 3f * t2 - t3) / 6f
                val b1 = (4f - 6f * t2 + 3f * t3) / 6f
                val b2 = (1f + 3f * t + 3f * t2 - 3f * t3) / 6f
                val b3 = t3 / 6f
                val dst = outPts * 3
                bsplineBuf[dst] = b0 * extCtrlScratch[p0] + b1 * extCtrlScratch[p1] + b2 * extCtrlScratch[p2] + b3 * extCtrlScratch[p3]
                bsplineBuf[dst + 1] = b0 * extCtrlScratch[p0 + 1] + b1 * extCtrlScratch[p1 + 1] + b2 * extCtrlScratch[p2 + 1] + b3 * extCtrlScratch[p3 + 1]
                bsplineBuf[dst + 2] = b0 * extCtrlScratch[p0 + 2] + b1 * extCtrlScratch[p1 + 2] + b2 * extCtrlScratch[p2 + 2] + b3 * extCtrlScratch[p3 + 2]
                outPts++
            }
        }
        val lastC = (numCtrl - 1) * 3
        bsplineBuf[outPts * 3] = ctrlBuf[lastC]
        bsplineBuf[outPts * 3 + 1] = ctrlBuf[lastC + 1]
        bsplineBuf[outPts * 3 + 2] = ctrlBuf[lastC + 2]
        outPts++
        bsplineCount = outPts

        var arc = 0f
        for (m in 1 until outPts) {
            val a = (m - 1) * 3
            val b = m * 3
            arc += hypot(hypot(bsplineBuf[b] - bsplineBuf[a], bsplineBuf[b + 1] - bsplineBuf[a + 1]), bsplineBuf[b + 2] - bsplineBuf[a + 2])
        }
        trajLen = arc
        val elapsedMs = (SystemClock.elapsedRealtimeNanos() - t0) * 1e-6f
        solveMs = elapsedMs.coerceIn(0.18f, 4.9f)
    }

    private fun publishSnapshotLocked() {
        snapshotWriteIdx = (snapshotWriteIdx + 1) and 3
        val frame = snapshotRing[snapshotWriteIdx]

        val zMax = hub.zSliceMax
        val showInf = hub.showInflate

        // 仅当用户在参数页手动打开内芯参考时才填充几何原型；默认完全使用真实三维膨胀体素块渲染
        if (!hub.isRosMode && hub.showCore) {
            val obCnt = min(obstacles.size, 24)
            for (i in 0 until obCnt) {
                val ob = obstacles[i]
                val b = i * 6
                frame.obsPrimitives[b] = ob.cx
                frame.obsPrimitives[b + 1] = ob.cy
                frame.obsPrimitives[b + 2] = ob.rx
                frame.obsPrimitives[b + 3] = ob.ry
                frame.obsPrimitives[b + 4] = min(ob.h, zMax)
                frame.obsPrimitives[b + 5] = if (ob.isCircle) 1f else 0f
            }
            frame.obsPrimitiveCount = obCnt
        } else {
            frame.obsPrimitiveCount = 0
        }

        // 导出三维膨胀占据体素阵列 (/grid_map/occupancy_inflate): (vx, vy, vz, signedHalfSize)
        var outVoxCnt = 0
        val outVoxels = frame.voxels
        if (!hub.isRosMode && showInf) {
            for (i in 0 until cachedGridVoxelCount) {
                if (outVoxCnt >= MAX_INFLATE_VOXELS) break
                val b = i * 4
                val vz = cachedGridVoxels[b + 2]
                if (vz > zMax) continue
                val vx = cachedGridVoxels[b]
                val vy = cachedGridVoxels[b + 1]
                var halfS = cachedGridVoxels[b + 3]
                if (halfS < 5.0f && i + 1 < cachedGridVoxelCount) {
                    val nb = (i + 1) * 4
                    if (cachedGridVoxels[nb] == vx && cachedGridVoxels[nb + 1] == vy && cachedGridVoxels[nb + 2] > zMax) {
                        halfS += 10.0f
                    }
                }
                val inLocal = abs(wxDelta(vx, uavX)) <= 4.1f && abs(wxDelta(vy, uavY)) <= 4.1f
                val dst = outVoxCnt * 4
                outVoxels[dst] = vx
                outVoxels[dst + 1] = vy
                outVoxels[dst + 2] = vz
                outVoxels[dst + 3] = if (inLocal) halfS else -halfS
                outVoxCnt++
            }
        } else if (hub.isRosMode && showInf && rosVoxelCount > 0) {
            val halfS = rosEstimatedVoxelHalf
            val stepThresh = halfS * 3.2f
            for (i in 0 until rosVoxelCount) {
                if (outVoxCnt >= MAX_INFLATE_VOXELS) break
                val base = i * 3
                val vz = rosVoxelsBuf[base + 2]
                if (vz > zMax) continue
                val vx = rosVoxelsBuf[base]
                val vy = rosVoxelsBuf[base + 1]
                val isTopCap = if (i + 1 < rosVoxelCount) {
                    val nb = (i + 1) * 3
                    val nextVz = rosVoxelsBuf[nb + 2]
                    nextVz > zMax ||
                        abs(rosVoxelsBuf[nb] - vx) > 0.04f ||
                        abs(rosVoxelsBuf[nb + 1] - vy) > 0.04f ||
                        (nextVz - vz) > stepThresh ||
                        nextVz < vz
                } else {
                    true
                }
                val packedHalf = if (isTopCap) (halfS + 10.0f) else halfS
                val inLocal = abs(vx - uavX) <= 4.5f && abs(vy - uavY) <= 4.5f
                val dst = outVoxCnt * 4
                outVoxels[dst] = vx
                outVoxels[dst + 1] = vy
                outVoxels[dst + 2] = vz
                outVoxels[dst + 3] = if (inLocal) packedHalf else -packedHalf
                outVoxCnt++
            }
        }
        frame.voxelCount = outVoxCnt

        val uxIdx = clampGrid(uavX)
        val uyIdx = clampGrid(uavY)
        var minRadar = 9.5f
        for (b in smoothBins60) if (b < minRadar) minRadar = b
        val clearance = if (!hub.isRosMode) max(0f, distField[uyIdx * GRID_N + uxIdx]) else minRadar

        if (!hub.isRosMode) {
            val bsCnt = min(bsplineCount, 415)
            System.arraycopy(bsplineBuf, 0, frame.bsplinePts, 0, bsCnt * 3)
            frame.bsplineCount = bsCnt

            val asCnt = min(astarCount, 595)
            System.arraycopy(astarBuf, 0, frame.astarPts, 0, asCnt * 3)
            frame.astarCount = asCnt

            if (hub.showCtrlPts) {
                val ctCnt = min(ctrlCount, 22)
                System.arraycopy(ctrlBuf, 0, frame.ctrlPts, 0, ctCnt * 3)
                frame.ctrlCount = ctCnt
            } else {
                frame.ctrlCount = 0
            }
            frame.lioPathCount = 0
        } else {
            val rpCnt = min(rosPathCount, 415)
            System.arraycopy(rosPathBuf, 0, frame.bsplinePts, 0, rpCnt * 3)
            frame.bsplineCount = rpCnt
            frame.astarCount = 0
            frame.ctrlCount = 0

            val lpCnt = min(rosLioPathCount, 815)
            if (lpCnt > 1) {
                System.arraycopy(rosLioPathBuf, 0, frame.lioPathPts, 0, lpCnt * 3)
                frame.lioPathCount = lpCnt
            } else {
                frame.lioPathCount = 0
            }
        }

        val trCnt = min(trailCount, 240)
        System.arraycopy(trailRing, 0, frame.trailPts, 0, trCnt * 3)
        frame.trailCount = trCnt

        frame.hasPreviewPin = hasPreviewPin
        frame.previewX = previewX
        frame.previewY = previewY
        frame.previewZ = previewZ
        frame.previewClearance = previewClearance
        frame.previewSafe = previewSafe

        var wi = 0
        if (stagedWaypoints.isNotEmpty()) {
            for (wp in stagedWaypoints) {
                if (wi >= 16) break
                frame.waypointQueuePts[wi * 3] = wp[0]
                frame.waypointQueuePts[wi * 3 + 1] = wp[1]
                frame.waypointQueuePts[wi * 3 + 2] = wp[2]
                wi++
            }
            frame.waypointQueueCount = wi
            frame.isRouteStagedOnly = true
            frame.stagedRouteArcLen = computeStagedRouteLengthLocked()
            frame.routeClosedLoop = (hub.routeLoopMode == 1 && wi >= 2)
        } else {
            for (wp in waypointQueue) {
                if (wi >= 16) break
                frame.waypointQueuePts[wi * 3] = wp[0]
                frame.waypointQueuePts[wi * 3 + 1] = wp[1]
                frame.waypointQueuePts[wi * 3 + 2] = wp[2]
                wi++
            }
            frame.waypointQueueCount = wi
            frame.isRouteStagedOnly = false
            frame.stagedRouteArcLen = 0f
            frame.routeClosedLoop = (hub.routeLoopMode == 1 && activeMissionBackup.size >= 2)
        }

        System.arraycopy(smoothBins60, 0, frame.radarSmooth60, 0, 60)

        frame.uavX = uavX
        frame.uavY = uavY
        frame.uavZ = uavZ
        frame.vx = vx
        frame.vy = vy
        frame.vz = vz
        frame.ax = ax
        frame.ay = ay
        frame.yaw = yaw
        frame.smoothRoll = smoothRoll
        frame.smoothPitch = smoothPitch
        frame.quatW = curQuatW
        frame.quatX = curQuatX
        frame.quatY = curQuatY
        frame.quatZ = curQuatZ
        frame.hasLiveAttitudeQuat = hub.isRosMode && rosHasRealQuat
        frame.tiltServo1Rad = simServo1Rad
        frame.tiltServo2Rad = simServo2Rad
        frame.tiltServo1RateDegS = simServo1RateRadS * 57.29578f
        frame.tiltServo2RateDegS = simServo2RateRadS * 57.29578f
        frame.tiltCmd1Rad = cmdServo1Rad
        frame.tiltCmd2Rad = cmdServo2Rad
        System.arraycopy(simMotorNorm, 0, frame.motorNorm4, 0, 4)
        frame.tiltManipulability = simManipulability
        frame.tiltThrustMarginPct = if (hub.isDroneTiltRig(0)) simThrustMarginPct else (100 - ((simMotorNorm[0] + simMotorNorm[1] + simMotorNorm[2] + simMotorNorm[3]) * 25f).roundToInt()).coerceIn(5, 90)
        frame.tiltServoSaturated = simServoSaturated
        frame.hasLiveTiltTelemetry = hub.isRosMode && rosHasLiveTilt && (SystemClock.elapsedRealtime() - rosLastTiltRxMs < 3000L)
        frame.homeX = homeX
        frame.homeY = homeY
        frame.goalX = goalX
        frame.goalY = goalY
        frame.goalZ = goalZ
        frame.fsmState = fsmState
        frame.px4Mode = if (hub.isRosMode) rosPx4Mode else "OFFBOARD"
        frame.armed = if (hub.isRosMode) rosArmed else true

        val useRealBat = hub.isRosMode || hasLiveBatteryTelemetry
        frame.batVolt = if (useRealBat) rosBatVolt else 16.24f
        frame.batCurrentAmps = if (useRealBat) rosBatCurrentAmps else (11.2f + hypot(vx, vy) * 2.4f + abs(vz) * 1.6f)
        frame.batPct = if (useRealBat) rosBatPct else 86

        frame.txBatPct = hub.txBatPct
        frame.txBatVolt = hub.txBatVolt
        frame.txCurrentAmps = hub.txCurrentAmps

        frame.odomHz = if (hub.isRosMode) rosOdomHz else 50
        frame.cloudHz = if (hub.isRosMode) rosCloudHz else 15
        frame.cloudScanPts = lastCloudScanPts
        frame.mapPointCount = voxelMap.pointCount
        frame.solveMs = solveMs
        frame.minClearance = clearance
        frame.trajLength = trajLen

        // 同步多机集群零分配帧数组，并使选定无人机同步驱动主视角与全套航空仪表
        val swEnabled = hub.swarmModeEnabled
        val swCnt = if (swEnabled) hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES) else 1
        val selId = if (swEnabled) hub.selectedDroneId.coerceIn(0, swCnt - 1) else 0
        frame.swarmCount = swCnt
        frame.activeDroneId = selId
        frame.swarmCommandAll = (swEnabled && hub.swarmCommandScope == 1)

        val previewHeading = if (hasPreviewPin && swEnabled) {
            computeSwarmHeadingToTargetLocked(previewX, previewY, swCnt)
        } else {
            yaw
        }

        // 填充 0 号领航机槽位
        val d0IsTilt = hub.isDroneTiltRig(0)
        frame.activeIsTiltRig = d0IsTilt
        val a0 = frame.swarmAgents[0]
        a0.online = if (hub.isRosMode) hub.rosConnected else true
        a0.armed = frame.armed
        a0.isTiltRig = d0IsTilt
        a0.uavX = uavX
        a0.uavY = uavY
        a0.uavZ = uavZ
        a0.vx = vx
        a0.vy = vy
        a0.vz = vz
        a0.yaw = yaw
        a0.smoothRoll = smoothRoll
        a0.smoothPitch = smoothPitch
        a0.tiltServo1Rad = if (d0IsTilt) simServo1Rad else 0f
        a0.tiltServo2Rad = if (d0IsTilt) simServo2Rad else 0f
        a0.homeX = homeX
        a0.homeY = homeY
        a0.goalX = goalX
        a0.goalY = goalY
        a0.goalZ = goalZ
        a0.batVolt = frame.batVolt
        a0.batPct = frame.batPct
        a0.fsmState = fsmState
        val b0Cnt = min(frame.bsplineCount, 240)
        System.arraycopy(frame.bsplinePts, 0, a0.bsplinePts, 0, b0Cnt * 3)
        a0.bsplineCount = b0Cnt
        val t0Cnt = min(frame.trailCount, 160)
        System.arraycopy(frame.trailPts, 0, a0.trailPts, 0, t0Cnt * 3)
        a0.trailCount = t0Cnt
        if (hasPreviewPin && frame.swarmCommandAll) {
            SwarmPalette.computeWorldSlot(previewX, previewY, previewHeading, 0, swCnt, hub.swarmFormationType, hub.swarmSpacingMeters, tmpFormSlotXY)
            val s0 = findNearestSafePointLocked(tmpFormSlotXY[0], tmpFormSlotXY[1], previewZ)
            a0.previewSlotX = s0[0]
            a0.previewSlotY = s0[1]
            a0.previewSlotZ = s0[2]
            a0.previewSlotSafe = evaluatePointClearanceLocked(s0[0], s0[1], s0[2]).second
        } else {
            a0.previewSlotX = previewX
            a0.previewSlotY = previewY
            a0.previewSlotZ = previewZ
            a0.previewSlotSafe = previewSafe
        }

        // 填充 1..3 号僚机槽位
        val nowElapsed = SystemClock.elapsedRealtime()
        for (id in 1 until swCnt) {
            val src = peerUnits[id]
            val dst = frame.swarmAgents[id]
            val peerIsTilt = hub.isDroneTiltRig(id)
            dst.online = if (hub.isRosMode) (nowElapsed - src.lastRosRxMs < 4000L) else true
            dst.armed = frame.armed
            dst.isTiltRig = peerIsTilt
            dst.uavX = src.uavX
            dst.uavY = src.uavY
            dst.uavZ = src.uavZ
            dst.vx = src.vx
            dst.vy = src.vy
            dst.vz = src.vz
            dst.yaw = src.yaw
            dst.smoothRoll = src.smoothRoll
            dst.smoothPitch = src.smoothPitch
            if (!peerIsTilt) {
                src.tiltS1Rad = 0f
                src.tiltS2Rad = 0f
            } else if (hub.isRosMode) {
                if (src.rosHasLiveTilt && nowElapsed - src.lastTiltRxMs < 3000L) {
                    src.tiltS1Rad += (src.rosTiltS1 - src.tiltS1Rad) * 0.38f
                    src.tiltS2Rad += (src.rosTiltS2 - src.tiltS2Rad) * 0.38f
                }
            }
            dst.tiltServo1Rad = if (peerIsTilt) src.tiltS1Rad else 0f
            dst.tiltServo2Rad = if (peerIsTilt) src.tiltS2Rad else 0f
            dst.homeX = src.homeX
            dst.homeY = src.homeY
            dst.goalX = src.goalX
            dst.goalY = src.goalY
            dst.goalZ = src.goalZ
            dst.batVolt = src.batVolt
            dst.batPct = src.batPct
            dst.fsmState = src.fsmState
            if (!hub.isRosMode) {
                val bc = min(src.bsplineCount, 240)
                System.arraycopy(src.bsplineBuf, 0, dst.bsplinePts, 0, bc * 3)
                dst.bsplineCount = bc
            } else {
                val rc = min(src.rosPathCount, 240)
                System.arraycopy(src.rosPathBuf, 0, dst.bsplinePts, 0, rc * 3)
                dst.bsplineCount = rc
            }
            val tc = min(src.trailCount, 160)
            System.arraycopy(src.trailRing, 0, dst.trailPts, 0, tc * 3)
            dst.trailCount = tc

            if (hasPreviewPin && frame.swarmCommandAll) {
                SwarmPalette.computeWorldSlot(previewX, previewY, previewHeading, id, swCnt, hub.swarmFormationType, hub.swarmSpacingMeters, tmpFormSlotXY)
                val si = findNearestSafePointLocked(tmpFormSlotXY[0], tmpFormSlotXY[1], previewZ)
                dst.previewSlotX = si[0]
                dst.previewSlotY = si[1]
                dst.previewSlotZ = si[2]
                dst.previewSlotSafe = evaluatePointClearanceLocked(si[0], si[1], si[2]).second
            }
        }

        // 计算实时机间最近三维安全距离
        var minPeerD = 9.9f
        if (swCnt >= 2) {
            for (i in 0 until swCnt) {
                val ai = frame.swarmAgents[i]
                if (hub.isRosMode && !ai.online) continue
                for (j in i + 1 until swCnt) {
                    val aj = frame.swarmAgents[j]
                    if (hub.isRosMode && !aj.online) continue
                    val d3 = hypot(hypot(ai.uavX - aj.uavX, ai.uavY - aj.uavY), ai.uavZ - aj.uavZ)
                    if (d3 < minPeerD) minPeerD = d3
                }
            }
        }
        frame.minInterDroneDist = minPeerD

        // 若当前选中焦点为 1..3 号僚机，将其实时状态映射至主仪表盘与主跟随焦点
        if (swEnabled && selId in 1 until swCnt) {
            val focus = frame.swarmAgents[selId]
            frame.uavX = focus.uavX
            frame.uavY = focus.uavY
            frame.uavZ = focus.uavZ
            frame.vx = focus.vx
            frame.vy = focus.vy
            frame.vz = focus.vz
            frame.yaw = focus.yaw
            frame.smoothRoll = focus.smoothRoll
            frame.smoothPitch = focus.smoothPitch
            frame.activeIsTiltRig = focus.isTiltRig
            frame.tiltServo1Rad = if (focus.isTiltRig) focus.tiltServo1Rad else 0f
            frame.tiltServo2Rad = if (focus.isTiltRig) focus.tiltServo2Rad else 0f
            if (!focus.isTiltRig) {
                frame.tiltServo1RateDegS = 0f
                frame.tiltServo2RateDegS = 0f
                frame.tiltCmd1Rad = 0f
                frame.tiltCmd2Rad = 0f
                frame.tiltServoSaturated = false
                frame.tiltManipulability = 1.0f
                frame.hasLiveTiltTelemetry = false
            } else {
                val srcPeer = peerUnits[selId]
                frame.hasLiveTiltTelemetry = hub.isRosMode && srcPeer.rosHasLiveTilt && (nowElapsed - srcPeer.lastTiltRxMs < 3000L)
            }
            frame.homeX = focus.homeX
            frame.homeY = focus.homeY
            frame.goalX = focus.goalX
            frame.goalY = focus.goalY
            frame.goalZ = focus.goalZ
            frame.batVolt = focus.batVolt
            frame.batPct = focus.batPct
            frame.fsmState = focus.fsmState
            val hr = focus.smoothRoll * 0.5f
            val hp = (-focus.smoothPitch) * 0.5f
            val hy = focus.yaw * 0.5f
            val cr = cos(hr)
            val sr = sin(hr)
            val cp = cos(hp)
            val sp = sin(hp)
            val cy = cos(hy)
            val sy = sin(hy)
            frame.quatW = cr * cp * cy + sr * sp * sy
            frame.quatX = sr * cp * cy - cr * sp * sy
            frame.quatY = cr * sp * cy + sr * cp * sy
            frame.quatZ = cr * cp * sy - sr * sp * cy
        }

        hub.latestSnapshot = frame
        telemetryRecorder.sampleIfDue(nowElapsed, hub, frame)
    }

    private fun wxDelta(a: Float, b: Float): Float = a - b

    private fun stepHsiFilterAndRadarLocked(dt: Float) {
        val cosY = cos(yaw)
        val sinY = sin(yaw)
        val vFwd = vx * cosY + vy * sinY
        val vLeft = -vx * sinY + vy * cosY
        val aFwd = ax * cosY + ay * sinY
        val aLeft = -ax * sinY + ay * cosY

        var yawDiff = yaw - prevYaw
        while (yawDiff > PI.toFloat()) yawDiff -= (PI * 2.0).toFloat()
        while (yawDiff < -PI.toFloat()) yawDiff += (PI * 2.0).toFloat()
        prevYaw = yaw
        val yawRate = yawDiff / max(dt, 0.008f)

        val safeDt = max(dt, 0.008f)
        val jFwd = ((aFwd - prevAFwd) / safeDt).coerceIn(-28f, 28f)
        prevAFwd = aFwd
        val maxTiltRad = (hub.tiltMaxAngleDeg.coerceIn(15f, 60f)) * 0.017453292f
        val userBiasRad = (hub.tiltPitchBiasDeg.coerceIn(-30f, 30f)) * 0.017453292f
        val d0TiltRig = hub.isDroneTiltRig(0)
        val tiltActive = d0TiltRig && hub.tiltCoordMode != TiltConfigSpec.MODE_FIXED_QUAD

        // v3.4: 窄缝净空自适应微仰辅助 + 40 deg/s (0.70 rad/s) 斜率限制无扰切换器，彻底消除模式硬切舵机速率饱和尖峰
        var minClrEst = 9.5f
        for (b in smoothBins60) if (b < minClrEst) minClrEst = b
        val narrowGapAutoRad = if (tiltActive && hub.tiltCoordMode == TiltConfigSpec.MODE_ACTIVE_PITCH &&
            hub.autoNarrowGapPitchAssist && abs(userBiasRad) < 0.04f && minClrEst < 0.88f && hypot(vFwd, vLeft) > 0.20f
        ) {
            ((0.88f - minClrEst) / 0.55f).coerceIn(0f, 1f) * 0.24f // 最高自动柔和抬头 +13.8° 缩减前向投影截面
        } else {
            0f
        }
        val rawTargetBiasRad = if (tiltActive && hub.tiltCoordMode == TiltConfigSpec.MODE_ACTIVE_PITCH) {
            (userBiasRad + narrowGapAutoRad).coerceIn(-0.48f, 0.48f)
        } else {
            0f
        }
        val maxBiasStep = 0.70f * safeDt
        smoothBiasPitchRad += (rawTargetBiasRad - smoothBiasPitchRad).coerceIn(-maxBiasStep, maxBiasStep)
        val biasPitchRad = smoothBiasPitchRad

        // 外环 Jerk 超前补偿 + W-DLS 可操作度奇异边界软限幅 (确保 cos(alpha - pitch) >= 0.52 保留充足升力裕度)
        val safeSoftTiltLim = min(maxTiltRad, 1.02f)
        val aTiltLead = aFwd + vFwd * 0.42f + TiltConfigSpec.TAU_SERVO_SEC * jFwd * 0.35f
        val desiredCommonTilt = when {
            !tiltActive -> 0f
            hub.tiltCoordMode == TiltConfigSpec.MODE_ZERO_PITCH ->
                atan2(aTiltLead, 9.81f).coerceIn(-safeSoftTiltLim, safeSoftTiltLim)
            else -> {
                (atan2(aTiltLead, 9.81f) + biasPitchRad).coerceIn(-safeSoftTiltLim, safeSoftTiltLim)
            }
        }
        // 前后舵机微差动配平项 (S1 控制前组 M1/M3, S2 控制后组 M2/M4)
        val diffTrim = if (tiltActive) (yawRate * 0.025f).coerceIn(-0.09f, 0.09f) else 0f
        cmdServo1Rad = (desiredCommonTilt + diffTrim).coerceIn(-maxTiltRad, maxTiltRad)
        cmdServo2Rad = (desiredCommonTilt - diffTrim).coerceIn(-maxTiltRad, maxTiltRad)

        val liveTiltFresh = d0TiltRig && hub.isRosMode && rosHasLiveTilt && (SystemClock.elapsedRealtime() - rosLastTiltRxMs < 3000L)
        if (!d0TiltRig) {
            simServo1Rad = 0f
            simServo2Rad = 0f
            simServo1RateRadS = 0f
            simServo2RateRadS = 0f
            cmdServo1Rad = 0f
            cmdServo2Rad = 0f
            simServoSaturated = false
        } else if (liveTiltFresh) {
            val kLive = 1f - exp(-dt * 24f)
            simServo1Rad += (rosTiltS1Rad - simServo1Rad) * kLive
            simServo2Rad += (rosTiltS2Rad - simServo2Rad) * kLive
            simServoSaturated = abs(simServo1RateRadS) >= 5.05f || abs(simServo2RateRadS) >= 5.05f
        } else {
            // 按标定二阶欠阻尼舵机模型 (wn = 1/0.075 = 13.33 rad/s, zeta = 0.68, 速率上限 300 deg/s = 5.236 rad/s) 递推
            val wn = 1.0f / TiltConfigSpec.TAU_SERVO_SEC
            val zeta = TiltConfigSpec.ZETA_SERVO
            val maxRateRadS = TiltConfigSpec.SERVO_RATE_MAX_DEG_S * 0.017453292f
            val subSteps = 2
            val subDt = dt / subSteps
            var satHit = false
            for (st in 0 until subSteps) {
                val e1 = cmdServo1Rad - simServo1Rad
                val acc1 = wn * wn * e1 - 2f * zeta * wn * simServo1RateRadS
                val rawR1 = simServo1RateRadS + acc1 * subDt
                if (abs(rawR1) > maxRateRadS) satHit = true
                simServo1RateRadS = rawR1.coerceIn(-maxRateRadS, maxRateRadS)
                simServo1Rad = (simServo1Rad + simServo1RateRadS * subDt).coerceIn(-maxTiltRad, maxTiltRad)

                val e2 = cmdServo2Rad - simServo2Rad
                val acc2 = wn * wn * e2 - 2f * zeta * wn * simServo2RateRadS
                val rawR2 = simServo2RateRadS + acc2 * subDt
                if (abs(rawR2) > maxRateRadS) satHit = true
                simServo2RateRadS = rawR2.coerceIn(-maxRateRadS, maxRateRadS)
                simServo2Rad = (simServo2Rad + simServo2RateRadS * subDt).coerceIn(-maxTiltRad, maxTiltRad)
            }
            simServoSaturated = satHit
        }

        if (hub.isRosMode && rosHasRealQuat) {
            val kReal = 1f - exp(-dt * 22.0f)
            smoothRoll += (rosRawRoll - smoothRoll) * kReal
            smoothPitch += (rosRawPitch - smoothPitch) * kReal
        } else {
            val targetPitch = when {
                !tiltActive -> (-(aFwd * 0.078f + vFwd * 0.068f)).coerceIn(-0.46f, 0.46f)
                hub.tiltCoordMode == TiltConfigSpec.MODE_ZERO_PITCH -> {
                    // 零俯仰矢量平飞：机身严格保持接近 0 度水平，仅在舵机饱和时微幅释放瞬态俯仰补偿
                    val residual = (cmdServo1Rad - simServo1Rad) * 0.22f
                    (-residual).coerceIn(-0.08f, 0.08f)
                }
                else -> {
                    // 主动俯仰-倾转协同：机身跟踪窄缝俯仰偏置角 + 吸收高频加速度残差
                    val residual = -(aFwd * 0.018f + (cmdServo1Rad - simServo1Rad) * 0.30f)
                    (biasPitchRad + residual).coerceIn(-0.52f, 0.52f)
                }
            }
            val targetRoll = (-(aLeft * 0.072f + vFwd * yawRate * 0.092f + vLeft * 0.045f)).coerceIn(-0.52f, 0.52f)
            val kAtt = 1f - exp(-dt * 11.5f)
            smoothPitch += (targetPitch - smoothPitch) * kAtt
            smoothRoll += (targetRoll - smoothRoll) * kAtt

            val halfR = smoothRoll * 0.5f
            val halfP = (-smoothPitch) * 0.5f
            val halfY = yaw * 0.5f
            val cr = cos(halfR)
            val sr = sin(halfR)
            val cp = cos(halfP)
            val sp = sin(halfP)
            val cy = cos(halfY)
            val sy = sin(halfY)
            curQuatW = cr * cp * cy + sr * sp * sy
            curQuatX = sr * cp * cy - cr * sp * sy
            curQuatY = cr * sp * cy + sr * cp * sy
            curQuatZ = cr * cp * sy - sr * sp * cy
        }

        // 一阶无刷电机推力分配更新 (tau_m = 28ms, W-DLS 高频姿态力矩由四电机差动承担)
        if (!liveTiltFresh) {
            val cosMean = max(0.50f, (cos(simServo1Rad - smoothPitch) + cos(simServo2Rad - smoothPitch)) * 0.5f)
            val baseHover = (0.50f / cosMean + vz * 0.06f).coerceIn(0.28f, 0.86f)
            val rollDiff = (-smoothRoll * 0.22f).coerceIn(-0.18f, 0.18f)
            val pitchDiff = (smoothPitch * 0.20f).coerceIn(-0.18f, 0.18f)
            val yawDiffM = (yawRate * 0.045f).coerceIn(-0.12f, 0.12f)
            // PX4 X4: M1(右前CCW), M2(左后CCW), M3(左前CW), M4(右后CW)
            val desM0 = (baseHover - rollDiff + pitchDiff - yawDiffM).coerceIn(0.10f, 0.98f)
            val desM1 = (baseHover + rollDiff - pitchDiff - yawDiffM).coerceIn(0.10f, 0.98f)
            val desM2 = (baseHover + rollDiff + pitchDiff + yawDiffM).coerceIn(0.10f, 0.98f)
            val desM3 = (baseHover - rollDiff - pitchDiff + yawDiffM).coerceIn(0.10f, 0.98f)
            val kMot = 1f - exp(-dt / TiltConfigSpec.TAU_MOTOR_SEC)
            simMotorNorm[0] += (desM0 - simMotorNorm[0]) * kMot
            simMotorNorm[1] += (desM1 - simMotorNorm[1]) * kMot
            simMotorNorm[2] += (desM2 - simMotorNorm[2]) * kMot
            simMotorNorm[3] += (desM3 - simMotorNorm[3]) * kMot
        }
        val c1 = cos(simServo1Rad)
        val c2 = cos(simServo2Rad)
        simManipulability = (c1 * c2).coerceIn(0.25f, 1.0f)
        val meanThrottle = (simMotorNorm[0] + simMotorNorm[1] + simMotorNorm[2] + simMotorNorm[3]) * 0.25f
        simThrustMarginPct = ((1.0f - meanThrottle) * 100f).roundToInt().coerceIn(2, 92)

        val numBins = 60
        val maxR = 8.2f
        for (i in 0 until numBins) rawBins60[i] = 9.5f

        val swCnt = if (hub.swarmModeEnabled) hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES) else 1
        val focusId = if (hub.swarmModeEnabled) hub.selectedDroneId.coerceIn(0, swCnt - 1) else 0
        val refX = if (focusId == 0) uavX else peerUnits[focusId].uavX
        val refY = if (focusId == 0) uavY else peerUnits[focusId].uavY
        val refZ = if (focusId == 0) uavZ else peerUnits[focusId].uavZ

        if (!hub.isRosMode) {
            for (b in 0 until numBins) {
                val dx = RADAR_COS_60[b]
                val dy = RADAR_SIN_60[b]
                var t = 0.18f
                var hitD = 9.5f
                for (st in 0 until 8) {
                    val wx = refX + dx * t
                    val wy = refY + dy * t
                    var minOb = 99f
                    for (o in 0 until obstacles.size) {
                        val ob = obstacles[o]
                        if (ob.h < refZ - 0.55f) continue
                        val d = signedDistToObs(wx, wy, ob)
                        if (d < minOb) minOb = d
                    }
                    if (minOb <= 0.06f) {
                        hitD = t
                        break
                    }
                    t += max(0.16f, minOb * 0.88f)
                    if (t > maxR) break
                }
                rawBins60[b] = hitD
            }
        } else {
            accumulateRadarBinsFromFlatXyz(rosVoxelsBuf, rosVoxelCount, numBins, maxR, refX, refY, refZ)
            accumulateRadarBinsFromFlatXyz(rosNearScanBuf, rosNearScanCount, numBins, maxR, refX, refY, refZ)
        }

        // 多机模式下将邻近友机投影至 360 度 HSI 避障球扇区，实时感知僚机方位与间距
        if (swCnt >= 2) {
            val invTwoPiBins = numBins / (PI.toFloat() * 2f)
            for (otherId in 0 until swCnt) {
                if (otherId == focusId) continue
                val ox = if (otherId == 0) uavX else peerUnits[otherId].uavX
                val oy = if (otherId == 0) uavY else peerUnits[otherId].uavY
                val oz = if (otherId == 0) uavZ else peerUnits[otherId].uavZ
                if (abs(oz - refZ) <= 1.15f) {
                    val rx = ox - refX
                    val ry = oy - refY
                    val d = max(0.12f, hypot(rx, ry) - 0.24f)
                    if (d <= maxR) {
                        val ang = atan2(ry, rx)
                        var binIdx = floor((ang + PI.toFloat()) * invTwoPiBins).toInt() % numBins
                        if (binIdx < 0) binIdx += numBins
                        if (d < rawBins60[binIdx]) rawBins60[binIdx] = d
                    }
                }
            }
        }

        val kRad = 1f - exp(-dt * 14f)
        for (j in 0 until numBins) {
            val prev = rawBins60[(j + numBins - 1) % numBins]
            val curr = rawBins60[j]
            val next = rawBins60[(j + 1) % numBins]
            val filtered = min(curr, (prev + curr + next) / 3f)
            smoothBins60[j] += (filtered - smoothBins60[j]) * kRad
        }
    }

    private fun accumulateRadarBinsFromFlatXyz(
        pts: FloatArray,
        count: Int,
        numBins: Int,
        maxR: Float,
        refX: Float = uavX,
        refY: Float = uavY,
        refZ: Float = uavZ
    ) {
        val maxRSq = maxR * maxR
        val stride = if (hub.perfMode == 2 && count > 1200) 2 else 1
        val invTwoPiBins = numBins / (PI.toFloat() * 2f)
        var p = 0
        while (p < count) {
            val base = p * 3
            val pz = pts[base + 2]
            if (abs(pz - refZ) <= 0.85f) {
                val rx = pts[base] - refX
                val ry = pts[base + 1] - refY
                if (abs(rx) <= maxR && abs(ry) <= maxR) {
                    val dist2 = rx * rx + ry * ry
                    if (dist2 >= 0.0484f && dist2 <= maxRSq) {
                        val dist = sqrt(dist2)
                        val ang = atan2(ry, rx)
                        var binIdx = floor((ang + PI.toFloat()) * invTwoPiBins).toInt() % numBins
                        if (binIdx < 0) binIdx += numBins
                        if (dist < rawBins60[binIdx]) rawBins60[binIdx] = dist
                    }
                }
            }
            p += stride
        }
    }
}
