package com.radiomaster.egogcs.ui

import java.util.concurrent.atomic.AtomicBoolean

/** Phase-preserving pacing: late callbacks never create a catch-up queue. */
class FramePacer {
    private var rate = 0
    private var nextNs = 0L

    fun reset() {
        rate = 0
        nextNs = 0L
    }

    fun due(nowNs: Long, framesPerSecond: Int): Boolean {
        require(framesPerSecond > 0)
        val period = 1_000_000_000L / framesPerSecond
        if (rate != framesPerSecond) {
            rate = framesPerSecond
            nextNs = nowNs + period
            return true
        }
        if (nowNs < nextNs) return false
        nextNs += ((nowNs - nextNs) / period + 1L) * period
        return true
    }
}

class PendingRender {
    private val pending = AtomicBoolean(false)
    fun request(): Boolean = pending.compareAndSet(false, true)
    fun complete() = pending.set(false)
}

/** Exact key, without hashes or quantization that could silently keep stale geometry. */
class GeometryKey(capacity: Int) {
    private val previous = FloatArray(capacity)
    private val next = FloatArray(capacity)
    private var count = 0
    private var previousCount = -1

    fun begin() {
        count = 0
    }

    fun add(value: Float) {
        next[count++] = value
    }

    fun add(values: FloatArray, length: Int = values.size) {
        values.copyInto(next, count, 0, length)
        count += length
    }

    fun unchanged(): Boolean {
        if (previousCount != count) return false
        for (i in 0 until count) if (previous[i].toRawBits() != next[i].toRawBits()) return false
        return true
    }

    fun commit() {
        next.copyInto(previous, 0, 0, count)
        previousCount = count
    }

    fun invalidate() {
        previousCount = -1
    }
}

class RenderMetrics {
    private val lock = Any()
    private val samples = LongArray(240)
    private var cursor = 0
    private var count = 0

    @Volatile var frames = 0L
        private set

    @Volatile var lastCpuMs = 0f
        private set

    @Volatile var pointUploads = 0L
        private set

    @Volatile var voxelCacheHits = 0L
        private set

    fun record(cpuNs: Long, points: Int, cacheHit: Boolean) {
        synchronized(lock) {
            samples[cursor] = cpuNs
            cursor = (cursor + 1) % samples.size
            count = minOf(count + 1, samples.size)
            lastCpuMs = cpuNs * 1e-6f
            pointUploads += points
            if (cacheHit) voxelCacheHits++
            frames++
        }
    }

    fun p95CpuMs(): Float {
        val copy = synchronized(lock) { samples.copyOf(count) }
        if (copy.isEmpty()) return 0f
        copy.sort()
        return copy[((copy.size - 1) * 95 + 99) / 100] * 1e-6f
    }
}
