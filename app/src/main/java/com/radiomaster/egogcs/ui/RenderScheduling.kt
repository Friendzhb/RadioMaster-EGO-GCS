package com.radiomaster.egogcs.ui

import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.PI
import kotlin.math.exp
import kotlin.math.max

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
        val jitterToleranceNs = minOf(1_500_000L, period / 8L)
        if (nowNs + jitterToleranceNs < nextNs) return false
        if (nowNs < nextNs) {
            nextNs += period
        } else {
            nextNs += ((nowNs - nextNs) / period + 1L) * period
        }
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

/**
 * Four-motor physical angular velocity & rotational inertia integrator (`M1..M4`):
 * - Spins ONLY when `armed == true` OR effective motor throttle (`> THRUST_ACTIVE_EPS`) is present.
 * - Smoothly spools up on arm/throttle (`TAU_SPIN_UP_SEC`) and coasts down via aerodynamic drag +
 *   brushless cogging torque (`TAU_SPIN_DOWN_SEC`) to an exact dead stop (`0 rad/s`) when disarmed.
 * - Chooses angular velocities that avoid 30 FPS / 60 FPS 3-blade (`120 deg`) stroboscopic harmonics.
 */
class RotorSpinState {
    val phasesRad = floatArrayOf(0.18f, 1.02f, 2.15f, 0.64f)
    val omegaNorm = FloatArray(4)
    val blurStrength = FloatArray(4)

    var isSpinning: Boolean = false
        private set

    fun reset() {
        phasesRad[0] = 0.18f
        phasesRad[1] = 1.02f
        phasesRad[2] = 2.15f
        phasesRad[3] = 0.64f
        for (i in 0..3) {
            omegaNorm[i] = 0f
            blurStrength[i] = 0f
        }
        isSpinning = false
    }

    /**
     * Advances all 4 rotors by [dtSec]. When `dtSec <= 0f`, preserves exact current state (zero drift).
     * @return `true` if any rotor currently has non-zero angular velocity (including coast-down).
     */
    fun step(
        dtSec: Float,
        armed: Boolean,
        m0: Float,
        m1: Float,
        m2: Float,
        m3: Float
    ): Boolean {
        val safeM0 = if (m0.isFinite()) m0.coerceIn(0f, 1f) else 0f
        val safeM1 = if (m1.isFinite()) m1.coerceIn(0f, 1f) else 0f
        val safeM2 = if (m2.isFinite()) m2.coerceIn(0f, 1f) else 0f
        val safeM3 = if (m3.isFinite()) m3.coerceIn(0f, 1f) else 0f
        val maxThrottle = max(max(safeM0, safeM1), max(safeM2, safeM3))
        val powered = armed || maxThrottle > THRUST_ACTIVE_EPS

        if (dtSec <= 0f || !dtSec.isFinite()) {
            var anyActive = false
            for (i in 0..3) {
                if (omegaNorm[i] > STOP_SNAP_EPS) anyActive = true
            }
            isSpinning = anyActive
            return anyActive
        }

        val dt = dtSec.coerceAtMost(0.08f)
        val kUp = 1f - exp(-dt / TAU_SPIN_UP_SEC)
        val kDown = 1f - exp(-dt / TAU_SPIN_DOWN_SEC)
        var anyMoving = false

        for (i in 0..3) {
            val rawThr = when (i) {
                0 -> safeM0
                1 -> safeM1
                2 -> safeM2
                else -> safeM3
            }
            val target = if (powered) {
                val effThr = if (armed) max(IDLE_THROTTLE_FLOOR, rawThr) else rawThr
                (0.42f + 0.58f * effThr).coerceIn(0f, 1f)
            } else {
                0f
            }

            var cur = omegaNorm[i]
            if (target > cur) {
                cur += (target - cur) * kUp
            } else {
                cur += (target - cur) * kDown
                if (!powered) {
                    // Brushless magnet cogging torque brings slow coasting blades to a crisp physical stop
                    cur = (cur - COGGING_DECEL_PER_SEC * dt).coerceAtLeast(0f)
                }
            }

            if (!powered && cur <= STOP_SNAP_EPS) {
                cur = 0f
            }
            omegaNorm[i] = cur

            if (cur > 0f) {
                anyMoving = true
                // Peak ~53 rad/s (~50.6 deg/frame @60FPS, ~101.2 deg/frame @30FPS), idle ~26.5 rad/s (~25.3 deg/frame)
                // Cleanly avoids 0 deg, 60 deg, and 120 deg 3-blade stroboscopic aliasing!
                val omegaRadSec = cur * MAX_VISUAL_OMEGA_RAD_S * MOTOR_KV_DETUNE[i]
                phasesRad[i] = (phasesRad[i] + omegaRadSec * dt) % PHASE_WRAP_RAD
            }
            blurStrength[i] = ((cur - BLUR_ONSET_NORM) / (1f - BLUR_ONSET_NORM)).coerceIn(0f, 1f)
        }

        isSpinning = anyMoving
        return anyMoving
    }

    companion object {
        const val THRUST_ACTIVE_EPS = 0.045f
        private const val IDLE_THROTTLE_FLOOR = 0.14f
        private const val TAU_SPIN_UP_SEC = 0.135f
        private const val TAU_SPIN_DOWN_SEC = 0.215f
        private const val COGGING_DECEL_PER_SEC = 0.20f
        private const val STOP_SNAP_EPS = 0.008f
        private const val BLUR_ONSET_NORM = 0.24f
        private const val MAX_VISUAL_OMEGA_RAD_S = 53.0f
        private const val PHASE_WRAP_RAD = (PI * 240.0).toFloat()
        private val MOTOR_KV_DETUNE = floatArrayOf(1.000f, 1.027f, 0.979f, 1.016f)
    }
}
