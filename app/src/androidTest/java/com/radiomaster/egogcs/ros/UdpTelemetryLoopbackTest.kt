package com.radiomaster.egogcs.ros

import android.os.SystemClock
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.radiomaster.egogcs.engine.EgoSimEngine
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.RenderSnapshot
import com.radiomaster.egogcs.model.TopicConfig
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.TimeUnit
import java.util.zip.CRC32
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class UdpTelemetryLoopbackTest {
    @Test fun telemetryReachesProductionEngineAndTimeoutRestoresReliableOdometryPreference() {
        val port = DatagramSocket(0).use { it.localPort }
        val hub = GcsSharedHub().apply { isRosMode = true }
        val engine = EgoSimEngine(hub)
        val receiver = RosUdpTelemetryClient(hub, engine)
        val destination = InetAddress.getByName("127.0.0.1")
        engine.start()
        try {
            val cfg = TopicConfig(masterUri = "http://127.0.0.1:11311", udpTelemetryEnabled = true, udpPort = port)
            repeat(5) {
                receiver.connect(cfg)
                receiver.disconnect()
            }
            receiver.connect(cfg)
            val payload = ByteBuffer.allocate(40).order(ByteOrder.LITTLE_ENDIAN).apply {
                for (v in floatArrayOf(1f, 2f, 3f, 0f, 0f, 0f, 1f, 0f, 0f, 0f)) putFloat(v)
            }.array()
            val crc = CRC32().apply { update(payload) }.value.toInt()
            DatagramSocket().use { sender ->
                repeat(15) { sequence ->
                    val data = ByteBuffer.allocate(76).order(ByteOrder.LITTLE_ENDIAN).apply {
                        putInt(0x44554745)
                        put(1)
                        put(1)
                        putShort(0)
                        putInt(123)
                        putInt(sequence)
                        putShort(0)
                        putShort(1)
                        putInt(40)
                        putInt(crc)
                        putLong(123456)
                        put(payload)
                    }.array()
                    sender.send(DatagramPacket(data, data.size, destination, port))
                    Thread.sleep(40)
                }
            }
            assertTrue(hub.hasFreshUdpOdometry(SystemClock.elapsedRealtime()))
            val frame = hub.snapshots.readInto(RenderSnapshot())
            assertEquals(1f, frame.uavX, 0.001f)
            assertEquals(2f, frame.uavY, 0.001f)
            assertEquals(3f, frame.uavZ, 0.001f)
            Thread.sleep(800)
            assertFalse(hub.hasFreshUdpOdometry(SystemClock.elapsedRealtime()))
            assertTrue(hub.udpTelemetryStatus.contains("可靠链路"))
            receiver.disconnect().get(1, TimeUnit.SECONDS)
            assertEquals(0L, hub.udpLastOdomMs)
            DatagramSocket(port).close() // Shutdown released the UDP port.
        } finally {
            receiver.shutdown()
            engine.stop()
        }
    }
}
