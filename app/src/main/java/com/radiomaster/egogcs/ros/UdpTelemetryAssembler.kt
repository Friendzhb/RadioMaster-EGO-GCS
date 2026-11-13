package com.radiomaster.egogcs.ros

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.zip.CRC32

/** Display telemetry only. No UDP packet is interpreted as a command. */
class UdpTelemetryAssembler {
    companion object {
        const val HEADER_BYTES = 36
        const val MAX_DATAGRAM = 1200
        const val FRAGMENT_BYTES = MAX_DATAGRAM - HEADER_BYTES
        const val ODOMETRY = 1
        const val CLOUD = 2
        const val MAX_CLOUD_POINTS = 8000
        const val ASSEMBLY_TIMEOUT_MS = 150L
    }

    class Frame internal constructor(val kind: Int) {
        var data = ByteArray(0)
            internal set
        var length = 0
            internal set
        internal var sequence = -1L
        internal var crc = 0L
        internal var stamp = 0L
        internal var firstMs = 0L
        internal var fragments = 0
        internal var received = 0
        internal var active = false
        internal val seen = BooleanArray(110)
    }

    private val frames = arrayOf(Frame(ODOMETRY), Frame(CLOUD))
    private var session = -1L
    private val retiredSessions = LongArray(8) { -1L }
    private var retireCursor = 0
    private val crc32 = CRC32()
    var discarded = 0L
        private set

    fun accept(packet: ByteArray, length: Int, nowMs: Long): Frame? {
        if (length < HEADER_BYTES || length > MAX_DATAGRAM || length > packet.size) return reject()
        val b = ByteBuffer.wrap(packet, 0, length).order(ByteOrder.LITTLE_ENDIAN)
        if (b.int != 0x44554745 || b.get().toInt() != 1) return reject() // EGUD, v1
        val kind = b.get().toInt()
        if (kind !in ODOMETRY..CLOUD || b.short.toInt() != 0) return reject()
        val incomingSession = b.int.toLong() and 0xffffffffL
        val sequence = b.int.toLong() and 0xffffffffL
        val fragment = b.short.toInt() and 0xffff
        val fragments = b.short.toInt() and 0xffff
        val total = b.int
        val crc = b.int.toLong() and 0xffffffffL
        val stamp = b.long
        val maxBytes = if (kind == ODOMETRY) 40 else MAX_CLOUD_POINTS * 16
        if (total <= 0 || total > maxBytes || (kind == ODOMETRY && total != 40) || (kind == CLOUD && total % 16 != 0)) return reject()
        val expectedFragments = (total + FRAGMENT_BYTES - 1) / FRAGMENT_BYTES
        if (fragments != expectedFragments || fragment >= fragments) return reject()
        val offset = fragment * FRAGMENT_BYTES
        val payloadLength = minOf(FRAGMENT_BYTES, total - offset)
        if (length != HEADER_BYTES + payloadLength) return reject()
        if (incomingSession != session) {
            if (retiredSessions.contains(incomingSession)) return reject()
            if (session >= 0) {
                retiredSessions[retireCursor] = session
                retireCursor = (retireCursor + 1) % retiredSessions.size
            }
            session = incomingSession
            frames.forEach {
                it.sequence = -1L
                it.active = false
            }
        }
        val f = frames[kind - 1]
        if (sequence != f.sequence) {
            if (f.sequence >= 0 && (sequence - f.sequence).toInt() <= 0) return reject()
            if (f.active) discarded++
            f.sequence = sequence
            f.firstMs = nowMs
            f.length = total
            f.crc = crc
            f.stamp = stamp
            f.fragments = fragments
            f.received = 0
            f.seen.fill(false)
            if (f.data.size < total) f.data = ByteArray(total)
            f.active = true
        }
        if (!f.active || nowMs - f.firstMs > ASSEMBLY_TIMEOUT_MS || f.length != total || f.crc != crc || f.stamp != stamp) {
            f.active = false
            return reject()
        }
        if (f.seen[fragment]) return null
        packet.copyInto(f.data, offset, HEADER_BYTES, length)
        f.seen[fragment] = true
        f.received++
        if (f.received != f.fragments) return null
        f.active = false
        crc32.reset()
        crc32.update(f.data, 0, f.length)
        if (crc32.value != f.crc) return reject()
        return f
    }

    private fun reject(): Frame? {
        discarded++
        return null
    }
}
