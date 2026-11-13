package com.radiomaster.egogcs.engine

import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.RenderSnapshot
import java.io.BufferedWriter
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStreamWriter
import java.util.Locale
import kotlin.math.roundToInt

/**
 * v3.7 科研级 20Hz 飞行黑匣子遥测记录仪 (`FlightTelemetryRecorder`)：
 * - 固定预分配环形数组缓存最多 24,000 帧 (约 20 分钟连续 20Hz 同步采样，主循环零对象分配)；
 * - 完整记录三维位姿、速度、姿态角、Tilt-UAV-INDI 双舵机实测/指令倾角、四电机混控推力、
 *   最近障碍净空、多机机间距离、规划耗时与动力电池电压电流，一键导出标准 UTF-8 CSV 供论文绘图分析。
 */
class FlightTelemetryRecorder {

    companion object {
        const val MAX_FRAMES = 24_000
        const val COLS_PER_FRAME = 23
    }

    private val lock = Any()
    private val ring = FloatArray(MAX_FRAMES * COLS_PER_FRAME)
    private val metaFlags = IntArray(MAX_FRAMES)
    private var head = 0

    @Volatile var sampleCount: Int = 0
        private set

    @Volatile var isRecording: Boolean = false
        private set
    private var startElapsedMs: Long = 0L
    private var lastSampleMs: Long = 0L

    fun startRecording(nowElapsedMs: Long) {
        synchronized(lock) {
            head = 0
            sampleCount = 0
            startElapsedMs = nowElapsedMs
            lastSampleMs = 0L
            isRecording = true
        }
    }

    fun stopRecording() {
        isRecording = false
    }

    fun clear() {
        synchronized(lock) {
            isRecording = false
            head = 0
            sampleCount = 0
        }
    }

    fun sampleIfDue(nowElapsedMs: Long, hub: GcsSharedHub, s: RenderSnapshot) {
        if (!isRecording) return
        if (nowElapsedMs - lastSampleMs < 48L) return
        lastSampleMs = nowElapsedMs

        synchronized(lock) {
            val slot = if (sampleCount < MAX_FRAMES) {
                val idx = (head + sampleCount) % MAX_FRAMES
                sampleCount++
                idx
            } else {
                val idx = head
                head = (head + 1) % MAX_FRAMES
                idx
            }

            val packedMeta = (if (hub.isRosMode) 1 else 0) or
                ((s.swarmCount and 0x07) shl 1) or
                ((s.activeDroneId and 0x07) shl 4) or
                ((if (s.activeIsTiltRig) 1 else 0) shl 7) or
                ((hub.tiltCoordMode and 0x03) shl 8)
            metaFlags[slot] = packedMeta

            val b = slot * COLS_PER_FRAME
            val rad2deg = 57.29578f
            ring[b] = ((nowElapsedMs - startElapsedMs).coerceAtLeast(0L)) * 0.001f
            ring[b + 1] = s.uavX
            ring[b + 2] = s.uavY
            ring[b + 3] = s.uavZ
            ring[b + 4] = s.vx
            ring[b + 5] = s.vy
            ring[b + 6] = s.vz
            ring[b + 7] = s.smoothRoll * rad2deg
            ring[b + 8] = s.smoothPitch * rad2deg
            ring[b + 9] = s.yaw * rad2deg
            ring[b + 10] = s.tiltServo1Rad * rad2deg
            ring[b + 11] = s.tiltServo2Rad * rad2deg
            ring[b + 12] = s.tiltCmd1Rad * rad2deg
            ring[b + 13] = s.tiltCmd2Rad * rad2deg
            ring[b + 14] = s.motorNorm4[0]
            ring[b + 15] = s.motorNorm4[1]
            ring[b + 16] = s.motorNorm4[2]
            ring[b + 17] = s.motorNorm4[3]
            ring[b + 18] = s.minClearance
            ring[b + 19] = s.minInterDroneDist
            ring[b + 20] = s.solveMs
            ring[b + 21] = s.batVolt
            ring[b + 22] = s.batCurrentAmps
        }
    }

    fun exportToCsv(outFile: File): Int {
        val countSnap: Int
        val headSnap: Int
        val dataSnap: FloatArray
        val metaSnap: IntArray
        synchronized(lock) {
            countSnap = sampleCount
            if (countSnap <= 0) return 0
            headSnap = head
            dataSnap = ring.copyOf()
            metaSnap = metaFlags.copyOf()
        }

        outFile.parentFile?.mkdirs()
        BufferedWriter(OutputStreamWriter(FileOutputStream(outFile), Charsets.UTF_8), 65536).use { w ->
            w.write("t_sec,link_mode,swarm_count,focus_drone_id,airframe_rig,tilt_coord_mode,")
            w.write("x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,roll_deg,pitch_deg,yaw_deg,")
            w.write("servo1_deg,servo2_deg,cmd1_deg,cmd2_deg,m1_norm,m2_norm,m3_norm,m4_norm,")
            w.write("min_clearance_m,min_inter_uav_m,plan_solve_ms,bat_volt_v,bat_curr_a\n")

            val sb = StringBuilder(256)
            for (i in 0 until countSnap) {
                val slot = (headSnap + i) % MAX_FRAMES
                val meta = metaSnap[slot]
                val linkStr = if ((meta and 1) != 0) "ROS1" else "SIM"
                val swCnt = (meta ushr 1) and 0x07
                val dId = (meta ushr 4) and 0x07
                val rigStr = if (((meta ushr 7) and 1) != 0) "TILT_4M2S" else "STD_X4"
                val cMode = (meta ushr 8) and 0x03

                val b = slot * COLS_PER_FRAME
                sb.setLength(0)
                sb.append(String.format(Locale.US, "%.3f,%s,%d,%d,%s,%d,", dataSnap[b], linkStr, swCnt, dId, rigStr, cMode))
                for (c in 1 until COLS_PER_FRAME) {
                    sb.append(String.format(Locale.US, "%.4f", dataSnap[b + c]))
                    sb.append(if (c == COLS_PER_FRAME - 1) '\n' else ',')
                }
                w.write(sb.toString())
            }
            w.flush()
        }
        return countSnap
    }

    fun durationSec(): Int {
        val n = sampleCount
        return (n / 20.0f).roundToInt()
    }
}
