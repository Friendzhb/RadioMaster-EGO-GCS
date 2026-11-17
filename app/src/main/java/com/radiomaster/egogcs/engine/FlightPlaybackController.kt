package com.radiomaster.egogcs.engine

import com.radiomaster.egogcs.db.FlightMissionEntity
import com.radiomaster.egogcs.db.TelemetryPointEntity
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.RenderSnapshot
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sqrt

/**
 * 本地 SQLite 战后三维轨迹回放控制器 (`FlightPlaybackController`)：
 * - 加载指定架次的有序 `TelemetryPointEntity` 时序点集，采用 O(log N) 二分查找定位当前回放游标所在区间；
 * - 对三维位置、速度、最短弧包裹欧拉角、归一化四元数 (NLERP)、双倾转舵机角与四电机推力执行平滑帧间插值；
 * - 支持 `0.5x / 1.0x / 2.0x / 4.0x` 倍速切换、播放/暂停、时间轴任意拖拽擦除重播，并向 3D 地图同步输出全航程预览线与实时渐隐尾迹。
 */
class FlightPlaybackController {

    companion object {
        val SPEED_PRESETS = floatArrayOf(0.5f, 1.0f, 2.0f, 4.0f)
        private const val MAX_OVERVIEW_PTS = 815
        private const val MAX_TRAIL_PTS = 240
        private const val RAD_TO_DEG = 57.29578f
    }

    private val lock = Any()
    private var activeMission: FlightMissionEntity? = null
    private var points: List<TelemetryPointEntity> = emptyList()
    private var baseElapsedMs: Long = 0L
    private var totalSpanMs: Long = 0L

    private val overviewPathPts = FloatArray(MAX_OVERVIEW_PTS * 3)
    private var overviewPathCount: Int = 0

    private val missionWaypointPts = FloatArray(RenderSnapshot.MAX_MISSION_WAYPOINTS * 3)
    private var missionWaypointCount: Int = 0
    private var missionRouteArcLen: Float = 0f

    private var cursorMs: Double = 0.0
    private var speedIndex: Int = 1

    @Volatile
    var isActive: Boolean = false
        private set

    @Volatile
    var isPlaying: Boolean = false
        private set

    @Volatile
    var speedMultiplier: Float = SPEED_PRESETS[1]
        private set

    fun loadMission(
        mission: FlightMissionEntity,
        orderedPoints: List<TelemetryPointEntity>,
        hub: GcsSharedHub
    ): Boolean {
        if (orderedPoints.isEmpty()) return false
        val sorted = if (isMonotonic(orderedPoints)) {
            orderedPoints
        } else {
            orderedPoints.sortedBy { it.elapsedMs }
        }
        synchronized(lock) {
            activeMission = mission
            points = sorted
            baseElapsedMs = sorted.first().elapsedMs
            val rawSpan = (sorted.last().elapsedMs - baseElapsedMs).coerceAtLeast(0L)
            totalSpanMs = max(rawSpan, mission.durationMs.coerceAtLeast(0L))
            cursorMs = 0.0
            speedIndex = 1
            speedMultiplier = SPEED_PRESETS[speedIndex]
            isPlaying = true
            isActive = true

            rebuildOverviewPathLocked(sorted)
            rebuildMissionWaypointsLocked(mission.waypointsCsv)
            syncHubFieldsLocked(hub, sorted.first().timestampEpochMs)
            hub.trailClearEpoch++
        }
        return true
    }

    fun play(hub: GcsSharedHub) {
        synchronized(lock) {
            if (!isActive || points.isEmpty()) return
            if (totalSpanMs > 0L && cursorMs >= totalSpanMs.toDouble() - 5.0) {
                cursorMs = 0.0
                hub.trailClearEpoch++
            }
            isPlaying = true
            hub.replayPlaying = true
        }
    }

    fun pause(hub: GcsSharedHub) {
        synchronized(lock) {
            if (!isActive) return
            isPlaying = false
            hub.replayPlaying = false
        }
    }

    fun togglePlayPause(hub: GcsSharedHub): Boolean {
        synchronized(lock) {
            if (!isActive || points.isEmpty()) return false
            if (isPlaying) {
                isPlaying = false
            } else {
                if (totalSpanMs > 0L && cursorMs >= totalSpanMs.toDouble() - 5.0) {
                    cursorMs = 0.0
                    hub.trailClearEpoch++
                }
                isPlaying = true
            }
            hub.replayPlaying = isPlaying
            return isPlaying
        }
    }

    fun seekToProgress(progress01: Float, hub: GcsSharedHub) {
        val clamped = progress01.coerceIn(0f, 1f)
        synchronized(lock) {
            if (!isActive || points.isEmpty()) return
            cursorMs = clamped.toDouble() * totalSpanMs.toDouble()
            val sampleWall = interpolateWallTimestampLocked(cursorMs.toLong())
            syncHubFieldsLocked(hub, sampleWall)
            hub.trailClearEpoch++
        }
    }

    fun seekToElapsedMs(elapsedMs: Long, hub: GcsSharedHub) {
        synchronized(lock) {
            if (!isActive || points.isEmpty()) return
            cursorMs = elapsedMs.coerceIn(0L, totalSpanMs).toDouble()
            val sampleWall = interpolateWallTimestampLocked(cursorMs.toLong())
            syncHubFieldsLocked(hub, sampleWall)
            hub.trailClearEpoch++
        }
    }

    fun cycleSpeed(hub: GcsSharedHub): Float {
        synchronized(lock) {
            speedIndex = (speedIndex + 1) % SPEED_PRESETS.size
            speedMultiplier = SPEED_PRESETS[speedIndex]
            hub.replaySpeed = speedMultiplier
            return speedMultiplier
        }
    }

    fun stopReplay(hub: GcsSharedHub) {
        synchronized(lock) {
            isActive = false
            isPlaying = false
            activeMission = null
            points = emptyList()
            overviewPathCount = 0
            missionWaypointCount = 0
            cursorMs = 0.0
            hub.isReplayMode = false
            hub.replayPlaying = false
            hub.replayElapsedMs = 0L
            hub.replayTotalMs = 0L
            hub.replayWallTimestampMs = 0L
            hub.replayMissionId = 0L
            hub.replayMissionTitle = ""
            hub.trailClearEpoch++
        }
    }

    /**
     * 在 `EgoSimEngine` 每帧发布前覆写 `RenderSnapshot`，返回是否处于有效回放态。
     */
    fun applyReplayToSnapshot(
        dtSec: Float,
        hub: GcsSharedHub,
        frame: RenderSnapshot,
        evaluateClearance: (Float, Float, Float) -> Pair<Float, Boolean>,
        queryColumnTopZ: (Float, Float) -> Float
    ): Boolean {
        synchronized(lock) {
            val mission = activeMission ?: return false
            val pts = points
            if (!isActive || pts.isEmpty()) return false

            if (isPlaying && dtSec > 0f) {
                cursorMs += dtSec.toDouble() * 1000.0 * speedMultiplier.toDouble()
                if (cursorMs >= totalSpanMs.toDouble()) {
                    cursorMs = totalSpanMs.toDouble()
                    isPlaying = false
                }
            }

            val targetRelMs = baseElapsedMs + cursorMs.toLong()
            val bracket = findBracketIndexLocked(pts, targetRelMs)
            val p0 = pts[bracket]
            val p1 = pts[min(bracket + 1, pts.lastIndex)]
            val segSpan = (p1.elapsedMs - p0.elapsedMs).coerceAtLeast(1L)
            val alpha = if (bracket >= pts.lastIndex) {
                0f
            } else {
                ((targetRelMs - p0.elapsedMs).toFloat() / segSpan.toFloat()).coerceIn(0f, 1f)
            }

            val ux = lerp(p0.x, p1.x, alpha)
            val uy = lerp(p0.y, p1.y, alpha)
            val uz = lerp(p0.z, p1.z, alpha)
            val vx = lerp(p0.vx, p1.vx, alpha)
            val vy = lerp(p0.vy, p1.vy, alpha)
            val vz = lerp(p0.vz, p1.vz, alpha)
            val roll = lerpAngleRad(p0.rollRad, p1.rollRad, alpha)
            val pitch = lerpAngleRad(p0.pitchRad, p1.pitchRad, alpha)
            val yaw = lerpAngleRad(p0.yawRad, p1.yawRad, alpha)

            var q1w = p1.quatW
            var q1x = p1.quatX
            var q1y = p1.quatY
            var q1z = p1.quatZ
            val dot = p0.quatW * q1w + p0.quatX * q1x + p0.quatY * q1y + p0.quatZ * q1z
            if (dot < 0f) {
                q1w = -q1w
                q1x = -q1x
                q1y = -q1y
                q1z = -q1z
            }
            var qw = lerp(p0.quatW, q1w, alpha)
            var qx = lerp(p0.quatX, q1x, alpha)
            var qy = lerp(p0.quatY, q1y, alpha)
            var qz = lerp(p0.quatZ, q1z, alpha)
            val qNorm = sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
            if (qNorm > 1e-4f) {
                val inv = 1f / qNorm
                qw *= inv
                qx *= inv
                qy *= inv
                qz *= inv
            } else {
                qw = 1f
                qx = 0f
                qy = 0f
                qz = 0f
            }

            val dtSegSec = segSpan * 0.001f
            val s1 = lerp(p0.tiltS1Rad, p1.tiltS1Rad, alpha)
            val s2 = lerp(p0.tiltS2Rad, p1.tiltS2Rad, alpha)
            val s1RateDeg = if (dtSegSec > 1e-3f) ((p1.tiltS1Rad - p0.tiltS1Rad) / dtSegSec) * RAD_TO_DEG else 0f
            val s2RateDeg = if (dtSegSec > 1e-3f) ((p1.tiltS2Rad - p0.tiltS2Rad) / dtSegSec) * RAD_TO_DEG else 0f

            frame.uavX = ux
            frame.uavY = uy
            frame.uavZ = uz
            frame.vx = vx
            frame.vy = vy
            frame.vz = vz
            frame.ax = if (dtSegSec > 1e-3f) (p1.vx - p0.vx) / dtSegSec else 0f
            frame.ay = if (dtSegSec > 1e-3f) (p1.vy - p0.vy) / dtSegSec else 0f
            frame.yaw = yaw
            frame.smoothRoll = roll
            frame.smoothPitch = pitch
            frame.quatW = qw
            frame.quatX = qx
            frame.quatY = qy
            frame.quatZ = qz
            frame.hasLiveAttitudeQuat = true

            frame.activeIsTiltRig = p0.activeIsTiltRig
            frame.tiltServo1Rad = s1
            frame.tiltServo2Rad = s2
            frame.tiltServo1RateDegS = s1RateDeg
            frame.tiltServo2RateDegS = s2RateDeg
            frame.tiltCmd1Rad = lerp(p0.tiltCmd1Rad, p1.tiltCmd1Rad, alpha)
            frame.tiltCmd2Rad = lerp(p0.tiltCmd2Rad, p1.tiltCmd2Rad, alpha)
            frame.motorNorm4[0] = lerp(p0.m0, p1.m0, alpha)
            frame.motorNorm4[1] = lerp(p0.m1, p1.m1, alpha)
            frame.motorNorm4[2] = lerp(p0.m2, p1.m2, alpha)
            frame.motorNorm4[3] = lerp(p0.m3, p1.m3, alpha)
            val meanMot = (frame.motorNorm4[0] + frame.motorNorm4[1] + frame.motorNorm4[2] + frame.motorNorm4[3]) * 0.25f
            frame.tiltThrustMarginPct = ((1f - meanMot) * 100f).roundToInt().coerceIn(2, 95)
            frame.tiltServoSaturated = abs(s1RateDeg) >= 285f || abs(s2RateDeg) >= 285f
            frame.hasLiveTiltTelemetry = mission.linkMode == "ROS1"

            frame.homeX = pts.first().x
            frame.homeY = pts.first().y
            frame.goalX = lerp(p0.goalX, p1.goalX, alpha)
            frame.goalY = lerp(p0.goalY, p1.goalY, alpha)
            frame.goalZ = lerp(p0.goalZ, p1.goalZ, alpha)
            frame.fsmState = "REPLAY:${p0.fsmState}"
            frame.px4Mode = p0.px4Mode
            frame.armed = p0.armed

            frame.batVolt = lerp(p0.batVolt, p1.batVolt, alpha)
            frame.batCurrentAmps = lerp(p0.batAmps, p1.batAmps, alpha)
            frame.batPct = lerp(p0.batPct.toFloat(), p1.batPct.toFloat(), alpha).roundToInt().coerceIn(0, 100)
            frame.minClearance = lerp(p0.minClearanceM, p1.minClearanceM, alpha)
            frame.minInterDroneDist = lerp(p0.minInterDroneDistM, p1.minInterDroneDistM, alpha)
            frame.solveMs = lerp(p0.solveMs, p1.solveMs, alpha)
            frame.trajLength = mission.distanceMeters
            frame.hasPreviewPin = false
            frame.astarCount = 0
            frame.bsplineCount = 0
            frame.ctrlCount = 0

            // 全航程完整三维轨迹线写入 lioPathPts 供 3D 地图渲染半透明全局参考走廊
            if (overviewPathCount > 1) {
                System.arraycopy(overviewPathPts, 0, frame.lioPathPts, 0, overviewPathCount * 3)
                frame.lioPathCount = overviewPathCount
            } else {
                frame.lioPathCount = 0
            }

            // 当前回放游标之前的尾迹写入 trailPts，驱动 FlightPathTracer 实时同步生长
            val histCount = bracket + 1
            val outTrailCnt: Int
            if (histCount <= MAX_TRAIL_PTS - 1) {
                var w = 0
                for (i in 0 until histCount) {
                    val pt = pts[i]
                    frame.trailPts[w * 3] = pt.x
                    frame.trailPts[w * 3 + 1] = pt.y
                    frame.trailPts[w * 3 + 2] = pt.z
                    w++
                }
                frame.trailPts[w * 3] = ux
                frame.trailPts[w * 3 + 1] = uy
                frame.trailPts[w * 3 + 2] = uz
                outTrailCnt = w + 1
            } else {
                val keep = MAX_TRAIL_PTS - 1
                for (k in 0 until keep) {
                    val srcIdx = ((k.toLong() * (histCount - 1)) / max(1, keep - 1)).toInt()
                    val pt = pts[srcIdx]
                    frame.trailPts[k * 3] = pt.x
                    frame.trailPts[k * 3 + 1] = pt.y
                    frame.trailPts[k * 3 + 2] = pt.z
                }
                frame.trailPts[keep * 3] = ux
                frame.trailPts[keep * 3 + 1] = uy
                frame.trailPts[keep * 3 + 2] = uz
                outTrailCnt = MAX_TRAIL_PTS
            }
            frame.trailCount = outTrailCnt

            // 还原任务编排航点标记至 3D 地图
            frame.waypointQueueCount = missionWaypointCount
            frame.selectedWaypointIdx = -1
            frame.isRouteStagedOnly = true
            frame.stagedRouteArcLen = missionRouteArcLen
            frame.routeClosedLoop = false
            for (wi in 0 until missionWaypointCount) {
                val wx = missionWaypointPts[wi * 3]
                val wy = missionWaypointPts[wi * 3 + 1]
                val wz = missionWaypointPts[wi * 3 + 2]
                frame.waypointQueuePts[wi * 3] = wx
                frame.waypointQueuePts[wi * 3 + 1] = wy
                frame.waypointQueuePts[wi * 3 + 2] = wz
                val (clr, safe) = evaluateClearance(wx, wy, wz)
                frame.waypointClearances[wi] = clr
                frame.waypointSafeFlags[wi] = safe
                frame.waypointColumnTopZs[wi] = queryColumnTopZ(wx, wy)
            }

            // 同步焦点单机槽位以驱动 3D 模型渲染
            frame.swarmCount = 1
            frame.activeDroneId = 0
            frame.swarmCommandAll = false
            val a0 = frame.swarmAgents[0]
            a0.online = true
            a0.armed = frame.armed
            a0.isTiltRig = frame.activeIsTiltRig
            a0.uavX = ux
            a0.uavY = uy
            a0.uavZ = uz
            a0.vx = vx
            a0.vy = vy
            a0.vz = vz
            a0.yaw = yaw
            a0.smoothRoll = roll
            a0.smoothPitch = pitch
            a0.tiltServo1Rad = s1
            a0.tiltServo2Rad = s2
            a0.homeX = frame.homeX
            a0.homeY = frame.homeY
            a0.goalX = frame.goalX
            a0.goalY = frame.goalY
            a0.goalZ = frame.goalZ
            a0.batVolt = frame.batVolt
            a0.batPct = frame.batPct
            a0.fsmState = frame.fsmState
            a0.bsplineCount = 0
            val a0Trail = min(outTrailCnt, 160)
            val trailStartOffset = (outTrailCnt - a0Trail) * 3
            System.arraycopy(frame.trailPts, trailStartOffset, a0.trailPts, 0, a0Trail * 3)
            a0.trailCount = a0Trail

            val interpWallMs = p0.timestampEpochMs + ((p1.timestampEpochMs - p0.timestampEpochMs) * alpha).toLong()
            syncHubFieldsLocked(hub, interpWallMs)
            return true
        }
    }

    private fun interpolateWallTimestampLocked(cursorElapsedMs: Long): Long {
        val pts = points
        if (pts.isEmpty()) return 0L
        val targetRelMs = baseElapsedMs + cursorElapsedMs
        val idx = findBracketIndexLocked(pts, targetRelMs)
        val p0 = pts[idx]
        val p1 = pts[min(idx + 1, pts.lastIndex)]
        val span = (p1.elapsedMs - p0.elapsedMs).coerceAtLeast(1L)
        val alpha = if (idx >= pts.lastIndex) {
            0f
        } else {
            ((targetRelMs - p0.elapsedMs).toFloat() / span.toFloat()).coerceIn(0f, 1f)
        }
        return p0.timestampEpochMs + ((p1.timestampEpochMs - p0.timestampEpochMs) * alpha).toLong()
    }

    private fun syncHubFieldsLocked(hub: GcsSharedHub, wallTsMs: Long) {
        hub.isReplayMode = isActive
        hub.replayPlaying = isPlaying
        hub.replaySpeed = speedMultiplier
        hub.replayElapsedMs = cursorMs.toLong().coerceIn(0L, totalSpanMs)
        hub.replayTotalMs = totalSpanMs
        hub.replayWallTimestampMs = wallTsMs
        hub.replayMissionId = activeMission?.id ?: 0L
        hub.replayMissionTitle = activeMission?.title.orEmpty()
    }

    private fun rebuildOverviewPathLocked(sorted: List<TelemetryPointEntity>) {
        val n = sorted.size
        if (n <= 1) {
            overviewPathCount = 0
            return
        }
        val outCnt = min(MAX_OVERVIEW_PTS, n)
        for (i in 0 until outCnt) {
            val srcIdx = ((i.toLong() * (n - 1)) / max(1, outCnt - 1)).toInt()
            val pt = sorted[srcIdx]
            overviewPathPts[i * 3] = pt.x
            overviewPathPts[i * 3 + 1] = pt.y
            overviewPathPts[i * 3 + 2] = pt.z
        }
        overviewPathCount = outCnt
    }

    private fun rebuildMissionWaypointsLocked(csv: String) {
        missionWaypointCount = 0
        missionRouteArcLen = 0f
        if (csv.isBlank()) return
        var prevX = Float.NaN
        var prevY = Float.NaN
        var prevZ = Float.NaN
        for (tok in csv.split(';')) {
            if (missionWaypointCount >= RenderSnapshot.MAX_MISSION_WAYPOINTS) break
            val parts = tok.split(',')
            if (parts.size == 3) {
                val x = parts[0].toFloatOrNull() ?: continue
                val y = parts[1].toFloatOrNull() ?: continue
                val z = parts[2].toFloatOrNull() ?: continue
                if (x.isFinite() && y.isFinite() && z.isFinite()) {
                    val b = missionWaypointCount * 3
                    missionWaypointPts[b] = x
                    missionWaypointPts[b + 1] = y
                    missionWaypointPts[b + 2] = z
                    if (prevX.isFinite()) {
                        missionRouteArcLen += hypot(hypot(x - prevX, y - prevY), z - prevZ)
                    }
                    prevX = x
                    prevY = y
                    prevZ = z
                    missionWaypointCount++
                }
            }
        }
    }

    private fun isMonotonic(list: List<TelemetryPointEntity>): Boolean {
        for (i in 1 until list.size) {
            if (list[i].elapsedMs < list[i - 1].elapsedMs) return false
        }
        return true
    }

    internal fun findBracketIndexLocked(
        sorted: List<TelemetryPointEntity>,
        targetElapsedMs: Long
    ): Int {
        if (sorted.size <= 1 || targetElapsedMs <= sorted.first().elapsedMs) return 0
        if (targetElapsedMs >= sorted.last().elapsedMs) return sorted.lastIndex - 1
        var lo = 0
        var hi = sorted.lastIndex
        while (lo + 1 < hi) {
            val mid = (lo + hi) ushr 1
            if (sorted[mid].elapsedMs <= targetElapsedMs) {
                lo = mid
            } else {
                hi = mid
            }
        }
        return lo
    }

    private fun lerp(a: Float, b: Float, t: Float): Float = a + (b - a) * t

    internal fun lerpAngleRad(a: Float, b: Float, t: Float): Float {
        var diff = b - a
        val twoPi = (PI * 2.0).toFloat()
        while (diff > PI.toFloat()) diff -= twoPi
        while (diff < -PI.toFloat()) diff += twoPi
        return a + diff * t
    }
}
