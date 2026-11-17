package com.radiomaster.egogcs.db

import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

/**
 * 飞行任务战后多维动力学与能耗分析报告 (`MissionAnalysisReport`)：
 * 从 SQLite 时序遥测点精确统计三维航迹总里程、速度分布、障碍物净空裕度、姿态/倾转极值、电池压降与库仑积分能耗，
 * 并抽稀生成等距时间桶曲线供战后分析面板绘制高度-地速-净空联合剖面图。
 */
data class MissionAnalysisReport(
    val mission: FlightMissionEntity,
    val sampleCount: Int,
    val durationMs: Long,
    val effectiveHz: Float,
    val distance3dMeters: Float,
    val maxGroundSpeedMps: Float,
    val avgGroundSpeedMps: Float,
    val maxClimbRateMps: Float,
    val minAltitudeM: Float,
    val maxAltitudeM: Float,
    val minClearanceM: Float,
    val nearObstacleWarnFrames: Int,
    val maxAbsRollDeg: Float,
    val maxAbsPitchDeg: Float,
    val maxAbsTiltDeg: Float,
    val avgPlanSolveMs: Float,
    val startVolt: Float,
    val minVolt: Float,
    val endVolt: Float,
    val voltageSagV: Float,
    val peakCurrentAmps: Float,
    val peakPowerWatts: Float,
    val consumedMah: Float,
    val consumedWh: Float,
    val profileBucketCount: Int,
    val altitudeProfileM: FloatArray,
    val speedProfileMps: FloatArray,
    val clearanceProfileM: FloatArray
)

object PostFlightAnalyzer {

    private const val PROFILE_BUCKETS = 36
    private const val RAD_TO_DEG = 57.29578f

    fun analyze(
        mission: FlightMissionEntity,
        points: List<TelemetryPointEntity>,
        warnClearanceThresholdM: Float = 0.65f
    ): MissionAnalysisReport {
        if (points.isEmpty()) {
            return MissionAnalysisReport(
                mission = mission,
                sampleCount = 0,
                durationMs = mission.durationMs,
                effectiveHz = 0f,
                distance3dMeters = mission.distanceMeters,
                maxGroundSpeedMps = mission.maxSpeedMps,
                avgGroundSpeedMps = mission.avgSpeedMps,
                maxClimbRateMps = 0f,
                minAltitudeM = 0f,
                maxAltitudeM = mission.maxAltitudeM,
                minClearanceM = mission.minClearanceM,
                nearObstacleWarnFrames = 0,
                maxAbsRollDeg = 0f,
                maxAbsPitchDeg = 0f,
                maxAbsTiltDeg = 0f,
                avgPlanSolveMs = 0f,
                startVolt = mission.startVolt,
                minVolt = mission.minVolt,
                endVolt = mission.minVolt,
                voltageSagV = (mission.startVolt - mission.minVolt).coerceAtLeast(0f),
                peakCurrentAmps = 0f,
                peakPowerWatts = mission.peakPowerWatts,
                consumedMah = 0f,
                consumedWh = 0f,
                profileBucketCount = 0,
                altitudeProfileM = FloatArray(0),
                speedProfileMps = FloatArray(0),
                clearanceProfileM = FloatArray(0)
            )
        }

        val first = points.first()
        val last = points.last()
        val spanMs = (last.elapsedMs - first.elapsedMs).coerceAtLeast(
            (last.timestampEpochMs - first.timestampEpochMs).coerceAtLeast(0L)
        )
        val effectiveHz = if (spanMs > 0L && points.size > 1) {
            ((points.size - 1) * 1000f) / spanMs.toFloat()
        } else {
            20f
        }

        var dist3d = 0f
        var maxGndSpd = 0f
        var sumGndSpd = 0f
        var maxClimb = 0f
        var minAlt = first.z
        var maxAlt = first.z
        var minClr = first.minClearanceM
        var warnFrames = 0
        var maxRollDeg = 0f
        var maxPitchDeg = 0f
        var maxTiltDeg = 0f
        var sumSolveMs = 0f
        var minVolt = first.batVolt
        var peakAmps = 0f
        var peakWatts = 0f
        var ampSec = 0.0
        var wattSec = 0.0

        for (i in points.indices) {
            val p = points[i]
            if (i > 0) {
                val prev = points[i - 1]
                val seg = hypot(hypot(p.x - prev.x, p.y - prev.y), p.z - prev.z)
                if (seg.isFinite() && seg < 25f) {
                    dist3d += seg
                }
                val dtSec = ((p.elapsedMs - prev.elapsedMs).coerceIn(1L, 2_000L)) * 0.001
                val avgI = ((p.batAmps + prev.batAmps) * 0.5f).coerceAtLeast(0f)
                val avgP = ((p.batVolt * p.batAmps + prev.batVolt * prev.batAmps) * 0.5f).coerceAtLeast(0f)
                ampSec += avgI * dtSec
                wattSec += avgP * dtSec
            }

            val gndSpd = hypot(p.vx, p.vy)
            if (gndSpd > maxGndSpd) maxGndSpd = gndSpd
            sumGndSpd += gndSpd
            val climb = abs(p.vz)
            if (climb > maxClimb) maxClimb = climb

            if (p.z < minAlt) minAlt = p.z
            if (p.z > maxAlt) maxAlt = p.z
            if (p.minClearanceM < minClr) minClr = p.minClearanceM
            if (p.minClearanceM < warnClearanceThresholdM) warnFrames++

            maxRollDeg = max(maxRollDeg, abs(p.rollRad * RAD_TO_DEG))
            maxPitchDeg = max(maxPitchDeg, abs(p.pitchRad * RAD_TO_DEG))
            maxTiltDeg = max(
                maxTiltDeg,
                max(abs(p.tiltS1Rad * RAD_TO_DEG), abs(p.tiltS2Rad * RAD_TO_DEG))
            )
            sumSolveMs += p.solveMs

            if (p.batVolt > 0.5f && p.batVolt < minVolt) minVolt = p.batVolt
            if (p.batAmps > peakAmps) peakAmps = p.batAmps
            val watts = p.batVolt * p.batAmps
            if (watts > peakWatts) peakWatts = watts
        }

        val n = points.size
        val avgSpd = sumGndSpd / n.toFloat()
        val avgSolve = sumSolveMs / n.toFloat()
        val buckets = min(PROFILE_BUCKETS, n)
        val altProf = FloatArray(buckets)
        val spdProf = FloatArray(buckets)
        val clrProf = FloatArray(buckets)
        for (b in 0 until buckets) {
            val idx = if (buckets <= 1) 0 else ((b.toLong() * (n - 1)) / (buckets - 1)).toInt()
            val pt = points[idx]
            altProf[b] = pt.z
            spdProf[b] = hypot(pt.vx, pt.vy)
            clrProf[b] = pt.minClearanceM
        }

        val updatedMission = mission.copy(
            endedAtEpochMs = max(mission.endedAtEpochMs, last.timestampEpochMs),
            durationMs = max(mission.durationMs, spanMs),
            sampleCount = n,
            distanceMeters = dist3d,
            maxSpeedMps = maxGndSpd,
            avgSpeedMps = avgSpd,
            maxAltitudeM = maxAlt,
            minClearanceM = minClr,
            startVolt = first.batVolt,
            minVolt = minVolt,
            peakPowerWatts = peakWatts
        )

        return MissionAnalysisReport(
            mission = updatedMission,
            sampleCount = n,
            durationMs = updatedMission.durationMs,
            effectiveHz = effectiveHz,
            distance3dMeters = dist3d,
            maxGroundSpeedMps = maxGndSpd,
            avgGroundSpeedMps = avgSpd,
            maxClimbRateMps = maxClimb,
            minAltitudeM = minAlt,
            maxAltitudeM = maxAlt,
            minClearanceM = minClr,
            nearObstacleWarnFrames = warnFrames,
            maxAbsRollDeg = maxRollDeg,
            maxAbsPitchDeg = maxPitchDeg,
            maxAbsTiltDeg = maxTiltDeg,
            avgPlanSolveMs = avgSolve,
            startVolt = first.batVolt,
            minVolt = minVolt,
            endVolt = last.batVolt,
            voltageSagV = (first.batVolt - minVolt).coerceAtLeast(0f),
            peakCurrentAmps = peakAmps,
            peakPowerWatts = peakWatts,
            consumedMah = (ampSec * 1000.0 / 3600.0).toFloat(),
            consumedWh = (wattSec / 3600.0).toFloat(),
            profileBucketCount = buckets,
            altitudeProfileM = altProf,
            speedProfileMps = spdProf,
            clearanceProfileM = clrProf
        )
    }
}
