package com.radiomaster.egogcs.ui

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Zero-allocation logarithmic camera zoom & pan interpolator for the 3D/2D GCS tactical map.
 *
 * Key properties:
 * 1. **Log-Scale Geometric Zoom (`ln` / `exp` domain)**: Interpolates `worldHalfSpan` linearly in
 *    logarithmic scale `u = ln(span)`, guaranteeing constant perceived optical magnification velocity
 *    across the entire `[2.2m .. 65.0m]` (~30x) zoom range and exact frame-rate semigroup invariance.
 * 2. **Logarithmic Distance Pan + Hyperbolic Zoom-Pan Coupling**:
 *    - Pure pan transitions decay in compressed log-distance space `ln(1 + beta * D / span)` for
 *      rapid long-range glide and critically damped sub-pixel settling without overshoot.
 *    - Simultaneous zoom + pan transitions couple pan progress to instantaneous scale via the exact
 *      analytic integral `w = (s_next - s_0) / (s_target - s_0)`, keeping pinched ground anchors,
 *      the 3D UAV model, and its `FlightPathTracer` ribbon free of curved screen-space drift.
 * 3. **Moving-Drone Frame Lock**: Tracks recenter offsets relative to the live UAV pose so that
 *    zooming while following a moving drone keeps the 3D UAV and trail head locked with zero lag.
 */
internal object LogarithmicMapInterpolator {

    const val MIN_HALF_SPAN = 2.2f
    const val MAX_HALF_SPAN = 65.0f
    const val DEFAULT_HALF_SPAN = 6.4f

    const val MIN_PITCH_RAD = 0.18f
    const val MAX_PITCH_RAD = 1.44f
    const val DEFAULT_YAW_RAD = 0.62f
    const val DEFAULT_PITCH_RAD = 0.55f

    private const val ZOOM_LOG_RATE = 13.5f
    private const val PAN_LOG_RATE = 13.0f
    private const val ROT_RATE = 10.8f
    private const val CHASE_YAW_RATE = 6.5f
    private const val LOG_PAN_BETA = 1.25f

    private const val SNAP_LOG_SPAN_EPS = 1e-5f
    private const val SNAP_POS_EPS = 1e-5f
    private const val SNAP_ANG_EPS = 1e-5f

    fun clampSpan(span: Float): Float {
        if (!span.isFinite()) return DEFAULT_HALF_SPAN
        return span.coerceIn(MIN_HALF_SPAN, MAX_HALF_SPAN)
    }

    /**
     * Evaluates exact logarithmic (geometric) interpolation between [startSpan] and [targetSpan]
     * at normalized progress [fraction] in `[0, 1]`.
     */
    fun lerpLogarithmic(startSpan: Float, targetSpan: Float, fraction: Float): Float {
        val s0 = clampSpan(startSpan)
        val s1 = clampSpan(targetSpan)
        if (!fraction.isFinite() || fraction <= 0f) return s0
        if (fraction >= 1f) return s1
        val logS0 = ln(s0.toDouble())
        val logS1 = ln(s1.toDouble())
        val deltaLog = logS1 - logS0
        if (abs(deltaLog) <= SNAP_LOG_SPAN_EPS) return s1
        return exp(logS0 + fraction.toDouble() * deltaLog).toFloat().coerceIn(MIN_HALF_SPAN, MAX_HALF_SPAN)
    }

    /**
     * Advances [currentSpan] toward [targetSpan] over [dt] seconds in logarithmic scale space.
     * Preserves exact state (`currentSpan`) when `dt <= 0f`.
     */
    fun stepLogarithmicScale(
        currentSpan: Float,
        targetSpan: Float,
        dt: Float,
        dampingScale: Float = 1.0f,
        rate: Float = ZOOM_LOG_RATE
    ): Float {
        val s0 = clampSpan(currentSpan)
        val s1 = clampSpan(targetSpan)
        if (!dt.isFinite() || dt <= 0f) return s0
        val logRatio = ln((s1 / s0).toDouble())
        if (abs(logRatio) <= SNAP_LOG_SPAN_EPS) return s1

        val damp = if (dampingScale.isFinite()) dampingScale.coerceIn(0.45f, 2.5f) else 1.0f
        val alpha = 1.0 - exp((-rate * damp * dt).toDouble())
        if (alpha >= 0.99999) return s1
        val next = (s0.toDouble() * exp(alpha * logRatio)).toFloat().coerceIn(MIN_HALF_SPAN, MAX_HALF_SPAN)
        return if (abs(ln((s1 / next).toDouble())) <= SNAP_LOG_SPAN_EPS) s1 else next
    }

    /**
     * Computes the per-frame logarithmic pan interpolation weight in `[0, 1]`:
     * - Under constant zoom, decays normalized distance `dNorm = dist / span` in logarithmic space
     *   `ln(1 + beta * dNorm)` (exact semigroup across arbitrary frame rates).
     * - Under active zoom, couples pan displacement to instantaneous logarithmic scale via
     *   `(nextSpan - prevSpan) / (targetSpan - prevSpan)` so screen-space trajectories remain linear
     *   and pinch focal anchors do not drift during transition frames.
     */
    fun computeLogPanStepWeight(
        distMeters: Float,
        prevSpan: Float,
        nextSpan: Float,
        targetSpan: Float,
        dt: Float,
        dampingScale: Float = 1.0f,
        rate: Float = PAN_LOG_RATE
    ): Float {
        if (!dt.isFinite() || dt <= 0f) return 0f
        if (!distMeters.isFinite() || distMeters <= SNAP_POS_EPS) return 1f

        val s0 = clampSpan(prevSpan)
        val sNext = clampSpan(nextSpan)
        val sTarget = clampSpan(targetSpan)
        val damp = if (dampingScale.isFinite()) dampingScale.coerceIn(0.45f, 2.5f) else 1.0f

        val decay = exp((-rate * damp * dt).toDouble())
        val alpha = (1.0 - decay).coerceIn(0.0, 1.0)

        val normDist = (distMeters / max(1.0f, s0)).toDouble()
        val betaD = LOG_PAN_BETA * normDist
        val logDistWeight = if (betaD > 1e-4) {
            val nextBetaD = (1.0 + betaD).pow(decay) - 1.0
            (1.0 - nextBetaD / betaD).coerceIn(alpha, 1.0)
        } else {
            alpha
        }

        val spanDelta = (sTarget - s0).toDouble()
        val absLogZoom = abs(ln((sTarget / s0).toDouble()))
        if (absLogZoom <= 1e-4 || abs(spanDelta) <= 1e-4) {
            return logDistWeight.toFloat()
        }

        val zoomCoupledWeight = ((sNext - s0).toDouble() / spanDelta).coerceIn(0.0, 1.0)
        val couplingBlend = (absLogZoom / 0.05).coerceIn(0.0, 1.0)
        return (logDistWeight + (zoomCoupledWeight - logDistWeight) * couplingBlend).toFloat()
    }

    /**
     * Computes world-space `(dPanX, dPanY)` adjustment when pinch-zooming around `(focusPxX, focusPxY)`
     * so the world coordinate under the user's fingers stays anchored across logarithmic zoom frames.
     */
    fun computePinchFocalPanDelta(
        focusPxX: Float,
        focusPxY: Float,
        viewportW: Int,
        viewportH: Int,
        oldSpanTarget: Float,
        newSpanTarget: Float,
        effYawRad: Float,
        effPitchRad: Float,
        outDeltaXY: FloatArray
    ) {
        outDeltaXY[0] = 0f
        outDeltaXY[1] = 0f
        if (viewportW <= 0 || viewportH <= 0) return
        val sOld = clampSpan(oldSpanTarget)
        val sNew = clampSpan(newSpanTarget)
        if (abs(sNew - sOld) <= 1e-5f) return

        val vh = max(1f, viewportH.toFloat())
        val offPxX = focusPxX - viewportW * 0.5f
        val offPxY = focusPxY - vh * 0.5f
        val scaleDeltaFrac = 1.0f - (sNew / sOld)
        val metersPerPxOld = (sOld * 2.0f) / vh
        val pitchForeshorten = sin(effPitchRad).coerceIn(0.38f, 1.0f)

        val dxScreen = offPxX * metersPerPxOld * scaleDeltaFrac
        val dyScreen = (-offPxY / pitchForeshorten) * metersPerPxOld * scaleDeltaFrac
        val cosY = cos(effYawRad)
        val sinY = sin(effYawRad)
        outDeltaXY[0] = dxScreen * cosY - dyScreen * sinY
        outDeltaXY[1] = dxScreen * sinY + dyScreen * cosY
    }

    /**
     * Smooth C1 logarithmic adaptive display scale for the 3D UAV model and `FlightPathTracer` width
     * so wide-area zoom transitions scale both in lockstep without any linear slope kink.
     */
    fun computeLogAdaptiveUavScale(worldHalfSpan: Float): Float {
        val s = clampSpan(worldHalfSpan)
        if (s <= 6.2f) return 1.0f
        val logOctaves = ln(s / 6.2f)
        // Smooth saturation in [1.0, 1.88] across [6.2m .. 65m]
        val normalized = logOctaves / (1.0f + logOctaves * 0.52f)
        return (1.0f + normalized * 0.80f).coerceIn(1.0f, 1.88f)
    }

    private fun canonicalZero(v: Float): Float = if (v == 0f) 0f else v

    /**
     * Thread-coordinated, zero-allocation camera transition state shared cleanly between UI gesture
     * callbacks and the GL render thread.
     */
    internal class CameraTransitionState {
        private val lock = Any()
        private val pinchDeltaScratch = FloatArray(2)

        // Desired targets (mutated under lock by UI gestures & read under lock by GL frame)
        @Volatile var followMode: Int = 1
            private set
        private var yawDesired: Float = DEFAULT_YAW_RAD
        private var pitchDesired: Float = DEFAULT_PITCH_RAD
        private var spanDesired: Float = DEFAULT_HALF_SPAN
        private var panTargetX: Float = 0f
        private var panTargetY: Float = 0f
        private var pendingRecenterCapture: Boolean = true

        // Interpolated GL-frame camera state (read by shaders, 3D UAV model, and FlightPathTracer)
        var camYaw: Float = DEFAULT_YAW_RAD
            private set
        var camPitch: Float = DEFAULT_PITCH_RAD
            private set
        var worldHalfSpan: Float = DEFAULT_HALF_SPAN
            private set
        var camFocusX: Float = -5.5f
            private set
        var camFocusY: Float = -4.2f
            private set
        var camFocusZ: Float = 0.45f
            private set

        // Residual offset in moving-UAV frame for smooth logarithmic recentering onto a flying drone
        private var followOffsetX: Float = 0f
        private var followOffsetY: Float = 0f
        private var followOffsetZ: Float = 0f

        fun currentSpanDesired(): Float = synchronized(lock) { spanDesired }

        fun zoomByFactor(factor: Float) {
            if (!factor.isFinite() || factor <= 0f) return
            synchronized(lock) {
                spanDesired = clampSpan(spanDesired / factor)
            }
        }

        fun onPinchScale(
            scaleFactor: Float,
            focusPxX: Float,
            focusPxY: Float,
            viewportW: Int,
            viewportH: Int,
            projBlend: Float,
            panBoundMeters: Float
        ) {
            if (!scaleFactor.isFinite() || scaleFactor <= 0f) return
            val clampedFactor = scaleFactor.coerceIn(0.80f, 1.25f)
            synchronized(lock) {
                val oldTarget = spanDesired
                val newTarget = clampSpan(oldTarget / clampedFactor)
                spanDesired = newTarget
                if (followMode == 0) {
                    val b = projBlend.coerceIn(0f, 1f)
                    val effYaw = camYaw * b
                    val effPitch = 1.568f * (1f - b) + camPitch * b
                    computePinchFocalPanDelta(
                        focusPxX = focusPxX,
                        focusPxY = focusPxY,
                        viewportW = viewportW,
                        viewportH = viewportH,
                        oldSpanTarget = oldTarget,
                        newSpanTarget = newTarget,
                        effYawRad = effYaw,
                        effPitchRad = effPitch,
                        outDeltaXY = pinchDeltaScratch
                    )
                    panTargetX = (panTargetX + pinchDeltaScratch[0]).coerceIn(-panBoundMeters, panBoundMeters)
                    panTargetY = (panTargetY + pinchDeltaScratch[1]).coerceIn(-panBoundMeters, panBoundMeters)
                }
            }
        }

        fun enterFreePanIfNeeded(): Boolean = synchronized(lock) {
            if (followMode != 0) {
                panTargetX = camFocusX
                panTargetY = camFocusY
                followMode = 0
                pendingRecenterCapture = false
                true
            } else {
                false
            }
        }

        fun panByScreenDelta(
            distanceX: Float,
            distanceY: Float,
            viewportH: Int,
            projBlend: Float,
            panBoundMeters: Float
        ) {
            synchronized(lock) {
                if (followMode != 0) {
                    panTargetX = camFocusX
                    panTargetY = camFocusY
                    followMode = 0
                    pendingRecenterCapture = false
                }
                // Couple finger delta to geometric mean of rendered & target span for responsive mid-zoom panning
                val activeSpan = sqrt(worldHalfSpan * spanDesired)
                val metersPerPx = (activeSpan * 2.0f) / max(1f, viewportH.toFloat())
                val effYaw = camYaw * projBlend.coerceIn(0f, 1f)
                val cosY = cos(effYaw)
                val sinY = sin(effYaw)
                val dxScreen = distanceX * metersPerPx
                val dyScreen = -distanceY * metersPerPx
                panTargetX = (panTargetX + dxScreen * cosY - dyScreen * sinY).coerceIn(-panBoundMeters, panBoundMeters)
                panTargetY = (panTargetY + dxScreen * sinY + dyScreen * cosY).coerceIn(-panBoundMeters, panBoundMeters)
            }
        }

        fun applyFlingMomentum(
            velocityPxX: Float,
            velocityPxY: Float,
            viewportH: Int,
            projBlend: Float,
            panBoundMeters: Float
        ) {
            if (!velocityPxX.isFinite() || !velocityPxY.isFinite()) return
            synchronized(lock) {
                if (followMode != 0) return
                val speedPx = hypot(velocityPxX, velocityPxY)
                if (speedPx < 180f) return
                val activeSpan = sqrt(worldHalfSpan * spanDesired)
                val metersPerPx = (activeSpan * 2.0f) / max(1f, viewportH.toFloat())
                // Map finger release velocity into a bounded world-space glide target
                val lookaheadSec = 0.13f
                val maxThrowMeters = activeSpan * 1.25f
                val rawDxScreen = (-velocityPxX * lookaheadSec) * metersPerPx
                val rawDyScreen = (velocityPxY * lookaheadSec) * metersPerPx
                val throwMag = hypot(rawDxScreen, rawDyScreen)
                val clampScale = if (throwMag > maxThrowMeters && throwMag > 1e-4f) maxThrowMeters / throwMag else 1f
                val dxScreen = rawDxScreen * clampScale
                val dyScreen = rawDyScreen * clampScale
                val effYaw = camYaw * projBlend.coerceIn(0f, 1f)
                val cosY = cos(effYaw)
                val sinY = sin(effYaw)
                panTargetX = (panTargetX + dxScreen * cosY - dyScreen * sinY).coerceIn(-panBoundMeters, panBoundMeters)
                panTargetY = (panTargetY + dxScreen * sinY + dyScreen * cosY).coerceIn(-panBoundMeters, panBoundMeters)
            }
        }

        fun orbit3D(
            distanceX: Float,
            distanceY: Float,
            density: Float
        ): Int {
            val safeDensity = max(0.5f, density)
            synchronized(lock) {
                var switchedToMode = -1
                if (followMode == 2 && abs(distanceX) > abs(distanceY) * 1.4f && abs(distanceX) > 4f * safeDensity) {
                    followMode = 1
                    switchedToMode = 1
                }
                if (followMode == 1) {
                    yawDesired += distanceX * 0.0072f / safeDensity * 1.6f
                }
                pitchDesired = (pitchDesired + distanceY * 0.0062f / safeDensity * 1.6f)
                    .coerceIn(MIN_PITCH_RAD, MAX_PITCH_RAD)
                return switchedToMode
            }
        }

        fun focusWorldPointSmooth(wx: Float, wy: Float, panBoundMeters: Float) {
            if (!wx.isFinite() || !wy.isFinite()) return
            synchronized(lock) {
                followMode = 0
                pendingRecenterCapture = false
                panTargetX = wx.coerceIn(-panBoundMeters, panBoundMeters)
                panTargetY = wy.coerceIn(-panBoundMeters, panBoundMeters)
            }
        }

        fun recenterLockDrone() {
            synchronized(lock) {
                followMode = 1
                pendingRecenterCapture = true
            }
        }

        fun cycleFollowMode(): Int = synchronized(lock) {
            val next = when (followMode) {
                1 -> {
                    pitchDesired = 0.38f
                    pendingRecenterCapture = true
                    2
                }
                2 -> {
                    panTargetX = camFocusX
                    panTargetY = camFocusY
                    pendingRecenterCapture = false
                    0
                }
                else -> {
                    pitchDesired = DEFAULT_PITCH_RAD
                    pendingRecenterCapture = true
                    1
                }
            }
            followMode = next
            next
        }

        fun resetCameraSmooth() {
            synchronized(lock) {
                followMode = 1
                yawDesired = DEFAULT_YAW_RAD
                pitchDesired = DEFAULT_PITCH_RAD
                spanDesired = DEFAULT_HALF_SPAN
                pendingRecenterCapture = true
            }
        }

        /**
         * Advances zoom, pan, and orbit angles by [dt] seconds on the GL thread using logarithmic
         * interpolation, ensuring atomic synchronization with the current frame's UAV pose `(uavX, uavY, uavZ)`.
         */
        fun stepGlFrame(
            dt: Float,
            uavX: Float,
            uavY: Float,
            uavZ: Float,
            uavYaw: Float,
            projBlend: Float,
            dampingScale: Float
        ) {
            val mode: Int
            val targetYaw: Float
            val targetPitch: Float
            val targetSpan: Float
            val targetPanX: Float
            val targetPanY: Float
            val captureRecenter: Boolean
            synchronized(lock) {
                mode = followMode
                targetYaw = yawDesired
                targetPitch = pitchDesired
                targetSpan = spanDesired
                targetPanX = panTargetX
                targetPanY = panTargetY
                captureRecenter = pendingRecenterCapture
                pendingRecenterCapture = false
            }

            val b = projBlend.coerceIn(0f, 1f)
            val anchorZ = (uavZ * 0.45f) * b

            if (mode != 0 && captureRecenter) {
                followOffsetX = canonicalZero(camFocusX - uavX)
                followOffsetY = canonicalZero(camFocusY - uavY)
                followOffsetZ = canonicalZero(camFocusZ - anchorZ)
            }

            val safeDt = if (dt.isFinite() && dt > 0f) dt else 0f
            val damp = if (dampingScale.isFinite()) dampingScale.coerceIn(0.45f, 2.5f) else 1.0f

            // 1. Orbit yaw & pitch interpolation
            if (safeDt > 0f) {
                val kRot = 1f - exp(-safeDt * ROT_RATE * damp)
                if (mode == 2) {
                    val chaseYaw = uavYaw - PI.toFloat() * 0.5f
                    var diff = chaseYaw - camYaw
                    while (diff > PI.toFloat()) diff -= (PI * 2.0).toFloat()
                    while (diff < -PI.toFloat()) diff += (PI * 2.0).toFloat()
                    camYaw = if (abs(diff) <= SNAP_ANG_EPS) {
                        chaseYaw
                    } else {
                        camYaw + diff * (1f - exp(-safeDt * CHASE_YAW_RATE))
                    }
                    synchronized(lock) {
                        if (followMode == 2) {
                            yawDesired = camYaw
                        }
                    }
                } else {
                    val dyaw = targetYaw - camYaw
                    camYaw = if (abs(dyaw) <= SNAP_ANG_EPS) targetYaw else camYaw + dyaw * kRot
                }
                val dpitch = targetPitch - camPitch
                camPitch = if (abs(dpitch) <= SNAP_ANG_EPS) targetPitch else camPitch + dpitch * kRot
            }

            // 2. Logarithmic zoom interpolation in ln(worldHalfSpan) domain
            val prevSpan = worldHalfSpan
            val nextSpan = stepLogarithmicScale(prevSpan, targetSpan, safeDt, damp)
            worldHalfSpan = nextSpan

            // 3. Coupled logarithmic pan interpolation (free-world frame or moving-UAV frame)
            if (mode != 0) {
                val offDist = sqrt(
                    followOffsetX * followOffsetX +
                        followOffsetY * followOffsetY +
                        followOffsetZ * followOffsetZ
                )
                if (offDist <= SNAP_POS_EPS) {
                    followOffsetX = 0f
                    followOffsetY = 0f
                    followOffsetZ = 0f
                } else if (safeDt > 0f) {
                    val w = computeLogPanStepWeight(offDist, prevSpan, nextSpan, targetSpan, safeDt, damp)
                    val rem = (1f - w).coerceIn(0f, 1f)
                    followOffsetX = if (abs(followOffsetX * rem) <= SNAP_POS_EPS) 0f else followOffsetX * rem
                    followOffsetY = if (abs(followOffsetY * rem) <= SNAP_POS_EPS) 0f else followOffsetY * rem
                    followOffsetZ = if (abs(followOffsetZ * rem) <= SNAP_POS_EPS) 0f else followOffsetZ * rem
                }
                camFocusX = canonicalZero(uavX + followOffsetX)
                camFocusY = canonicalZero(uavY + followOffsetY)
                camFocusZ = canonicalZero(anchorZ + followOffsetZ)
            } else {
                val dx = targetPanX - camFocusX
                val dy = targetPanY - camFocusY
                val dz = 0f - camFocusZ
                val dist = sqrt(dx * dx + dy * dy + dz * dz)
                if (dist <= SNAP_POS_EPS) {
                    camFocusX = canonicalZero(targetPanX)
                    camFocusY = canonicalZero(targetPanY)
                    camFocusZ = 0f
                } else if (safeDt > 0f) {
                    val w = computeLogPanStepWeight(dist, prevSpan, nextSpan, targetSpan, safeDt, damp)
                    val nx = camFocusX + dx * w
                    val ny = camFocusY + dy * w
                    val nz = camFocusZ + dz * w
                    camFocusX = canonicalZero(if (abs(targetPanX - nx) <= SNAP_POS_EPS) targetPanX else nx)
                    camFocusY = canonicalZero(if (abs(targetPanY - ny) <= SNAP_POS_EPS) targetPanY else ny)
                    camFocusZ = canonicalZero(if (abs(nz) <= SNAP_POS_EPS) 0f else nz)
                }
            }
        }
    }
}
