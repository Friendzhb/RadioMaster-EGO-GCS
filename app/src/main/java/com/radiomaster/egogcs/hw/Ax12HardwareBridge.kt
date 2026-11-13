package com.radiomaster.egogcs.hw

import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.BatteryManager
import com.radiomaster.egogcs.engine.EgoSimEngine
import com.radiomaster.egogcs.model.GcsSharedHub
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * RadioMaster AX12 原生零冲突电源与电流遥测桥接器 (Ax12HardwareBridge)：
 *
 * 在绝不触碰 /dev/ttyS0 (UMBUS 独占串口)、不影响官方 Flyshark 控发射安全的前提下，提供双路电压/电流采集：
 * 1. AX12 本机电源监控 (Android BatteryManager)：实时采集内置双节 21700 电池电量 %、电压 V 与整机工作电流 A；
 * 2. ELRS Backpack 原生 Wi-Fi MAVLink UDP (:14550) 零分配监听：免 Root 直接解析飞控回传的
 *    SYS_STATUS (#1) 与 BATTERY_STATUS (#147) 动力电池电压 (V)、实时放电电流 (A) 与剩余电量 (%)。
 */
class Ax12HardwareBridge(
    private val context: Context,
    private val hub: GcsSharedHub,
    private val engine: EgoSimEngine
) {

    private var powerTimer: ScheduledExecutorService? = null

    @Volatile private var udpRunning = false
    private var udpSocket: DatagramSocket? = null
    private var udpThread: Thread? = null

    fun start() {
        startLocalPowerMonitor()
        startMavlinkBackpackUdpListener()
    }

    fun stop() {
        udpRunning = false
        try {
            udpSocket?.close()
        } catch (_: Throwable) {}
        udpSocket = null
        udpThread?.interrupt()
        udpThread = null
        powerTimer?.shutdownNow()
        powerTimer = null
    }

    /**
     * 通道 1：通过 Android 原生 BatteryManager 读取 AX12 遥控器本机双 21700 锂电电压、电量与实时工作电流
     */
    private fun startLocalPowerMonitor() {
        if (powerTimer != null) return
        powerTimer = Executors.newSingleThreadScheduledExecutor { r ->
            Thread(r, "Ax12PowerMonitor").apply { isDaemon = true }
        }.also { exec ->
            exec.scheduleAtFixedRate({ pollLocalBattery() }, 0L, 2500L, TimeUnit.MILLISECONDS)
        }
    }

    private fun pollLocalBattery() {
        try {
            val bm = context.getSystemService(Context.BATTERY_SERVICE) as? BatteryManager
            val sticky = context.registerReceiver(null, IntentFilter(Intent.ACTION_BATTERY_CHANGED))

            val cap = bm?.getIntProperty(BatteryManager.BATTERY_PROPERTY_CAPACITY) ?: -1
            if (cap in 1..100) {
                hub.txBatPct = cap
            } else if (sticky != null) {
                val lv = sticky.getIntExtra(BatteryManager.EXTRA_LEVEL, -1)
                val sc = sticky.getIntExtra(BatteryManager.EXTRA_SCALE, 100).coerceAtLeast(1)
                if (lv >= 0) hub.txBatPct = ((lv * 100f) / sc).roundToInt().coerceIn(0, 100)
            }

            if (sticky != null) {
                val mv = sticky.getIntExtra(BatteryManager.EXTRA_VOLTAGE, -1)
                if (mv > 2500) {
                    val volts = mv / 1000f
                    hub.txBatVolt = if (volts in 3.0f..4.45f) volts * 2f else volts
                }
            }

            val rawCurr = bm?.getIntProperty(BatteryManager.BATTERY_PROPERTY_CURRENT_NOW) ?: Int.MIN_VALUE
            if (rawCurr != Int.MIN_VALUE && rawCurr != 0) {
                val absVal = abs(rawCurr.toFloat())
                val amps = when {
                    absVal > 20_000f -> absVal / 1_000_000f
                    absVal > 20f -> absVal / 1_000f
                    else -> absVal
                }
                if (amps in 0.05f..8.0f) {
                    hub.txCurrentAmps = amps
                }
            }
        } catch (_: Throwable) {
        }
    }

    /**
     * 通道 2：监听 AX12 内置 ELRS Backpack 转发的标准 MAVLink v1/v2 UDP 14550 端口 (零堆分配解析动力电压与电流)
     */
    private fun startMavlinkBackpackUdpListener() {
        if (udpRunning) return
        udpRunning = true
        udpThread = Thread({
            val buf = ByteArray(512)
            val pkt = DatagramPacket(buf, buf.size)
            val bb = ByteBuffer.wrap(buf).order(ByteOrder.LITTLE_ENDIAN)

            while (udpRunning) {
                var sock: DatagramSocket? = null
                try {
                    sock = DatagramSocket(null).apply {
                        reuseAddress = true
                        soTimeout = 2000
                        bind(InetSocketAddress(14550))
                    }
                    udpSocket = sock

                    while (udpRunning && !sock.isClosed) {
                        try {
                            pkt.length = buf.size
                            sock.receive(pkt)
                            if (hub.mavlinkBackpackEnabled && pkt.length >= 8) {
                                parseMavlinkDatagram(buf, pkt.length, bb)
                            }
                        } catch (_: java.net.SocketTimeoutException) {
                        }
                    }
                } catch (_: Throwable) {
                    if (udpRunning) {
                        try {
                            Thread.sleep(3000L)
                        } catch (_: InterruptedException) {
                            break
                        }
                    }
                } finally {
                    try {
                        sock?.close()
                    } catch (_: Throwable) {}
                }
            }
        }, "Ax12MavlinkUdp14550").apply {
            isDaemon = true
            start()
        }
    }

    private fun parseMavlinkDatagram(buf: ByteArray, totalLen: Int, bb: ByteBuffer) {
        var pos = 0
        while (pos + 8 <= totalLen) {
            val magic = buf[pos].toInt() and 0xFF
            if (magic == 0xFD && pos + 12 <= totalLen) {
                val payloadLen = buf[pos + 1].toInt() and 0xFF
                val incompat = buf[pos + 2].toInt() and 0xFF
                val sigLen = if ((incompat and 0x01) != 0) 13 else 0
                val frameTotal = 10 + payloadLen + 2 + sigLen
                if (pos + frameTotal > totalLen) break

                val msgId = (buf[pos + 7].toInt() and 0xFF) or
                    ((buf[pos + 8].toInt() and 0xFF) shl 8) or
                    ((buf[pos + 9].toInt() and 0xFF) shl 16)
                handleMavlinkBatteryPayload(msgId, buf, pos + 10, payloadLen, bb)
                pos += frameTotal
            } else if (magic == 0xFE && pos + 8 <= totalLen) {
                val payloadLen = buf[pos + 1].toInt() and 0xFF
                val frameTotal = 6 + payloadLen + 2
                if (pos + frameTotal > totalLen) break

                val msgId = buf[pos + 5].toInt() and 0xFF
                handleMavlinkBatteryPayload(msgId, buf, pos + 6, payloadLen, bb)
                pos += frameTotal
            } else {
                pos++
            }
        }
    }

    private fun handleMavlinkBatteryPayload(
        msgId: Int,
        buf: ByteArray,
        payOff: Int,
        payLen: Int,
        bb: ByteBuffer
    ) {
        when (msgId) {
            0 -> {
                // v3.6 HEARTBEAT (#0): custom_mode uint32 @0, type uint8 @4, base_mode uint8 @6
                if (payLen >= 9) {
                    val mavType = buf[payOff + 4].toInt() and 0xFF
                    // 仅采纳多旋翼/VTOL飞控本体心跳 (排除 GCS=6 / Gimbal=26 等外设心跳)
                    val isAirframe = when (mavType) {
                        1, 2, 3, 4, 13, 14, 19, 20, 21, 22 -> true else -> false
                    }
                    if (isAirframe) {
                        val baseMode = buf[payOff + 6].toInt() and 0xFF
                        val armed = (baseMode and 0x80) != 0
                        val customMode = bb.getInt(payOff)
                        val mainMode = (customMode ushr 16) and 0xFF
                        val px4Str = when (mainMode) {
                            1 -> "MANUAL"
                            2 -> "ALTCTL"
                            3 -> "POSCTL"
                            6 -> "OFFBOARD"
                            else -> "AUTO"
                        }
                        engine.onMavlinkHeartbeatFallback(px4Str, armed)
                    }
                }
            }
            30 -> {
                // v3.6 ATTITUDE (#30): time_boot_ms uint32 @0, roll float @4, pitch float @8, yaw float @12 (NED -> FLU)
                if (payLen >= 16) {
                    val rollNed = bb.getFloat(payOff + 4)
                    val pitchNed = bb.getFloat(payOff + 8)
                    val yawNed = bb.getFloat(payOff + 12)
                    if (rollNed.isFinite() && pitchNed.isFinite() && yawNed.isFinite()) {
                        // MAVLink NED 转 ROS REP-103 FLU: roll_flu = roll_ned, pitch_flu = pitch_ned(抬头正适配本渲染系), yaw_enu = pi/2 - yaw_ned
                        var yawEnu = (Math.PI.toFloat() * 0.5f) - yawNed
                        while (yawEnu > Math.PI.toFloat()) yawEnu -= (Math.PI.toFloat() * 2f)
                        while (yawEnu < -Math.PI.toFloat()) yawEnu += (Math.PI.toFloat() * 2f)
                        engine.onMavlinkAttitudeFallback(rollNed, pitchNed, yawEnu)
                    }
                }
            }
            1 -> {
                // SYS_STATUS: voltage_battery uint16 @ +14 (mV), current_battery int16 @ +16 (10mA)
                if (payLen >= 18) {
                    val mv = bb.getShort(payOff + 14).toInt() and 0xFFFF
                    val cA = bb.getShort(payOff + 16).toInt()
                    val rem = if (payLen >= 31) buf[payOff + 30].toInt() else -1
                    if (mv in 3000..60000) {
                        val volts = mv / 1000f
                        val amps = if (cA >= 0) cA / 100f else 0f
                        val pct = if (rem in 0..100) rem else estimateLipoPct(volts)
                        engine.onRosBattery(volts, pct, amps)
                    }
                }
            }
            147 -> {
                // BATTERY_STATUS: voltages[10] uint16 @10..29, current_battery int16 @30
                if (payLen >= 32) {
                    var sumMv = 0
                    for (c in 0 until 10) {
                        val cellMv = bb.getShort(payOff + 10 + c * 2).toInt() and 0xFFFF
                        if (cellMv in 500..5000) sumMv += cellMv
                    }
                    val cA = bb.getShort(payOff + 30).toInt()
                    val rem = if (payLen >= 36) buf[payOff + 35].toInt() else -1
                    if (sumMv > 3000) {
                        val volts = sumMv / 1000f
                        val amps = if (cA >= 0) cA / 100f else 0f
                        val pct = if (rem in 0..100) rem else estimateLipoPct(volts)
                        engine.onRosBattery(volts, pct, amps)
                    }
                }
            }
        }
    }

    private fun estimateLipoPct(volt: Float): Int {
        val cells = when {
            volt > 18.0f -> 6f
            volt > 13.0f -> 4f
            else -> 3f
        }
        val vCell = volt / cells
        return (((vCell - 3.50f) / (4.18f - 3.50f)) * 100f).roundToInt().coerceIn(0, 100)
    }
}
