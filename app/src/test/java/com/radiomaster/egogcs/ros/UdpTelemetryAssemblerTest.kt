package com.radiomaster.egogcs.ros

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.zip.CRC32
import org.junit.Assert.*
import org.junit.Test

class UdpTelemetryAssemblerTest {
    private fun packets(payload: ByteArray, sequence: Long = 1L, session: Int = 123, kind: Int = 2): List<ByteArray> {
        val crc = CRC32().apply { update(payload) }.value.toInt()
        val size = UdpTelemetryAssembler.FRAGMENT_BYTES
        val total = (payload.size + size - 1) / size
        return (0 until total).map { index ->
            val count = minOf(size, payload.size - index * size)
            ByteBuffer.allocate(36 + count).order(ByteOrder.LITTLE_ENDIAN).apply {
                putInt(0x44554745)
                put(1)
                put(kind.toByte())
                putShort(0)
                putInt(session)
                putInt(sequence.toInt())
                putShort(index.toShort())
                putShort(total.toShort())
                putInt(payload.size)
                putInt(crc)
                putLong(123456L)
                put(payload, index * size, count)
            }.array()
        }
    }

    @Test fun fullCloudReassemblesOutOfOrderAndDuplicatesDoNotRepublish() {
        val a = UdpTelemetryAssembler()
        val payload = ByteArray(128000) { (it % 127).toByte() }
        val chunks = packets(payload)
        var complete: UdpTelemetryAssembler.Frame? = null
        for (p in chunks.reversed()) {
            a.accept(p, p.size, 10)?.let { complete = it }
            assertNull(a.accept(p, p.size, 10))
        }
        assertNotNull(complete)
        assertArrayEquals(payload, complete!!.data.copyOf(complete!!.length))
    }

    @Test fun missingCorruptAndOversizedPacketsNeverPublish() {
        val a = UdpTelemetryAssembler()
        val cloud = packets(ByteArray(4000))
        assertNull(a.accept(cloud[0], cloud[0].size, 10))
        for (p in cloud.drop(1)) assertNull(a.accept(p, p.size, 200))
        val bad = packets(ByteArray(40), sequence = 2, kind = 1).single()
        bad[bad.lastIndex] = 99
        assertNull(a.accept(bad, bad.size, 300))
        val malicious = packets(ByteArray(40), sequence = 3, kind = 1).single()
        ByteBuffer.wrap(malicious).order(ByteOrder.LITTLE_ENDIAN).putInt(20, Int.MAX_VALUE)
        assertNull(a.accept(malicious, malicious.size, 301))
        assertNull(a.accept(ByteArray(1201), 1201, 301))
        assertNull(a.accept(ByteArray(12), 12, 301))
    }

    @Test fun newerFramesReplaceIncompleteOnesAndSequenceWrapWorks() {
        val a = UdpTelemetryAssembler()
        val first = packets(ByteArray(4000), 0xfffffffeL)
        assertNull(a.accept(first[0], first[0].size, 1))
        val next = packets(ByteArray(16) { 1 }, 0xffffffffL).single()
        assertNotNull(a.accept(next, next.size, 2))
        for (old in first) assertNull(a.accept(old, old.size, 3))
        val wrapped = packets(ByteArray(16) { 2 }, 0L).single()
        assertNotNull(a.accept(wrapped, wrapped.size, 4))
        val restart = packets(ByteArray(16) { 3 }, 1, session = 124).single()
        assertNotNull(a.accept(restart, restart.size, 5))
        assertNull(a.accept(wrapped, wrapped.size, 6))
    }

    @Test fun pythonBridgeGoldenPacketIsCompatible() {
        val hex = javaClass.getResource("/udp-odometry.hex")!!.readText().trim()
        val bytes = hex.chunked(2).map { it.toInt(16).toByte() }.toByteArray()
        val f = UdpTelemetryAssembler().accept(bytes, bytes.size, 1)!!
        assertEquals(UdpTelemetryAssembler.ODOMETRY, f.kind)
        assertEquals(40, f.length)
        val floats = ByteBuffer.wrap(f.data).order(ByteOrder.LITTLE_ENDIAN).asFloatBuffer()
        val values = FloatArray(10).also { floats.get(it) }
        assertArrayEquals(floatArrayOf(1f, 2f, 3f, 0.1f, 0.2f, 0.3f, 1f, 0f, 0f, 0f), values, 0.00001f)
    }
}
