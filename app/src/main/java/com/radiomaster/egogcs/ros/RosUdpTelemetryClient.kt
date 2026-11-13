package com.radiomaster.egogcs.ros

import android.os.SystemClock
import com.radiomaster.egogcs.engine.EgoSimEngine
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.TopicConfig
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.SocketTimeoutException
import java.net.URI
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.sqrt

class RosUdpTelemetryClient(private val hub: GcsSharedHub, private val engine: EgoSimEngine) {
    private val generation = AtomicInteger()
    private val sessionLock = Any()

    @Volatile private var socket: DatagramSocket? = null
    private var worker: Thread? = null

    fun connect(cfg: TopicConfig) {
        disconnect()
        if (!cfg.udpTelemetryEnabled) return
        require(cfg.udpPort in 1024..65535 && cfg.udpPort != 14550)
        val token = generation.get()
        hub.udpTelemetryStatus = "UDP 等待机载桥接；位姿使用可靠链路"
        worker = Thread({ receive(cfg, token) }, "RosUdpTelemetry").apply {
            isDaemon = true
            start()
        }
    }

    fun disconnect() {
        val closing = synchronized(sessionLock) {
            generation.incrementAndGet()
            val previous = socket
            socket = null
            worker?.interrupt()
            worker = null
            previous
        }
        closing?.close()
        hub.udpLastOdomMs = 0L
        hub.udpLastCloudMs = 0L
        hub.udpTelemetryStatus = "UDP 未启用"
    }

    private fun receive(cfg: TopicConfig, token: Int) {
        var localSocket: DatagramSocket? = null
        try {
            val host = URI(if (cfg.protocol == "tcpros") cfg.masterUri else cfg.wsUrl).host
                ?: throw IllegalArgumentException("机载地址无效")
            val source = InetAddress.getByName(host)
            if (generation.get() != token) return
            val s = DatagramSocket(null)
            localSocket = s
            s.soTimeout = 100
            s.receiveBufferSize = 256 * 1024
            synchronized(sessionLock) {
                if (generation.get() != token) return
                socket = s
            }
            s.bind(InetSocketAddress(cfg.udpPort))
            if (generation.get() != token) return
            val bytes = ByteArray(UdpTelemetryAssembler.MAX_DATAGRAM + 1)
            val packet = DatagramPacket(bytes, bytes.size)
            val assembler = UdpTelemetryAssembler()
            val cloud = FloatArray(UdpTelemetryAssembler.MAX_CLOUD_POINTS * 4)
            val odom = FloatArray(10)
            var statusMs = 0L
            while (generation.get() == token && !Thread.currentThread().isInterrupted) {
                try {
                    packet.length = bytes.size
                    s.receive(packet)
                    val now = SystemClock.elapsedRealtime()
                    if (packet.address == source && generation.get() == token) {
                        assembler.accept(bytes, packet.length, now)?.let { frame ->
                            val data = ByteBuffer.wrap(frame.data, 0, frame.length).order(ByteOrder.LITTLE_ENDIAN)
                            if (frame.kind == UdpTelemetryAssembler.CLOUD) {
                                val count = frame.length / 16
                                for (i in 0 until count * 4) cloud[i] = data.float
                                if ((0 until count * 4).all { cloud[it].isFinite() }) {
                                    if (generation.get() == token) engine.onRosFastLioCloud(cloud, count)
                                    if (generation.get() == token) hub.udpLastCloudMs = now
                                }
                            } else {
                                for (i in odom.indices) odom[i] = data.float
                                if (odom.all { it.isFinite() }) dispatchOdometry(odom, now, token)
                            }
                        }
                    }
                } catch (_: SocketTimeoutException) {
                    // Continue to update freshness even when the bridge or Wi-Fi stops.
                }
                val now = SystemClock.elapsedRealtime()
                if (now - statusMs >= 250L && generation.get() == token) {
                    statusMs = now
                    val cloudState = if (hub.udpLastCloudMs > 0 && now - hub.udpLastCloudMs < 750L) "点云实时" else "点云等待/过期"
                    val odomState = if (hub.udpLastOdomMs > 0 && now - hub.udpLastOdomMs < 500L) "UDP位姿" else "位姿回退可靠链路"
                    hub.udpTelemetryStatus = "UDP $cloudState · $odomState · 丢弃${assembler.discarded}"
                }
            }
        } catch (e: Exception) {
            if (generation.get() == token) hub.udpTelemetryStatus = "UDP 未就绪：${e.localizedMessage ?: "连接异常"}；位姿使用可靠链路"
        } finally {
            localSocket?.close()
            synchronized(sessionLock) { if (socket === localSocket) socket = null }
        }
    }

    private fun dispatchOdometry(v: FloatArray, now: Long, token: Int) {
        if (generation.get() != token) return
        val norm = sqrt(v[6] * v[6] + v[7] * v[7] + v[8] * v[8] + v[9] * v[9])
        if (norm < 1e-6f || !norm.isFinite()) return
        val w = v[6] / norm
        val x = v[7] / norm
        val y = v[8] / norm
        val z = v[9] / norm
        val roll = atan2(2f * (w * x + y * z), 1f - 2f * (x * x + y * y))
        val pitch = asin((2f * (w * y - z * x)).coerceIn(-1f, 1f))
        val yaw = atan2(2f * (w * z + x * y), 1f - 2f * (y * y + z * z))
        engine.onRosOdometry(v[0], v[1], v[2], v[3], v[4], v[5], yaw, roll, pitch, w, x, y, z, true)
        if (generation.get() == token) hub.udpLastOdomMs = now
    }
}
