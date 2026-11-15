package com.radiomaster.egogcs.ui

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class LogarithmicMapInterpolatorTest {

    @Test
    fun logarithmicScaleInterpolationProducesGeometricMidpointAndSemigroupInvariance() {
        // Exact geometric midpoint: sqrt(4 * 16) = 8.0 (whereas linear lerp would give 10.0)
        val mid = LogarithmicMapInterpolator.lerpLogarithmic(4.0f, 16.0f, 0.5f)
        assertEquals(8.0f, mid, 1e-5f)

        // Quarter step across 4 octaves (2.5 -> 40.0, ratio 16x): 2.5 * 16^0.25 = 5.0
        val quarter = LogarithmicMapInterpolator.lerpLogarithmic(2.5f, 40.0f, 0.25f)
        assertEquals(5.0f, quarter, 1e-4f)

        // Zero dt must never mutate current span
        val unchanged = LogarithmicMapInterpolator.stepLogarithmicScale(6.4f, 32.0f, 0.0f)
        assertEquals(6.4f, unchanged, 0.0f)

        // Exact frame-rate semigroup invariance across 30 Hz, 60 Hz, and 120 Hz over 0.15s
        var span30 = 4.0f
        repeat(5) {
            span30 = LogarithmicMapInterpolator.stepLogarithmicScale(span30, 48.0f, 0.15f / 5f)
        }
        var span60 = 4.0f
        repeat(9) {
            span60 = LogarithmicMapInterpolator.stepLogarithmicScale(span60, 48.0f, 0.15f / 9f)
        }
        var span120 = 4.0f
        repeat(18) {
            span120 = LogarithmicMapInterpolator.stepLogarithmicScale(span120, 48.0f, 0.15f / 18f)
        }

        assertEquals(span30, span60, 1e-4f)
        assertEquals(span60, span120, 1e-4f)
        assertTrue(span120 > 4.0f && span120 < 48.0f)
    }

    @Test
    fun pinchZoomCoupledPanPreservesWorldFocalAnchorAcrossTransitionFrames() {
        val state = LogarithmicMapInterpolator.CameraTransitionState()
        val viewportW = 1920
        val viewportH = 1080
        val projBlend = 0.0f // Top-down orthographic-equivalent projection (effYaw = 0, sin(effPitch) ~ 1)

        // Settle initial free-pan camera at (2.0, -1.5) with default half-span (6.4m)
        state.focusWorldPointSmooth(2.0f, -1.5f, panBoundMeters = 120f)
        repeat(60) {
            state.stepGlFrame(
                dt = 1f / 60f,
                uavX = 0f,
                uavY = 0f,
                uavZ = 1.2f,
                uavYaw = 0f,
                projBlend = projBlend,
                dampingScale = 1.0f
            )
        }
        assertEquals(2.0f, state.camFocusX, 1e-4f)
        assertEquals(-1.5f, state.camFocusY, 1e-4f)
        val initialSpan = state.worldHalfSpan

        // User pinches in (scaleFactor = 1.25) around off-center pixel (1440, 270)
        val focusPxX = 1440f
        val focusPxY = 270f
        val metersPerPx0 = (initialSpan * 2.0f) / viewportH.toFloat()
        val effPitch = 1.568f
        val pitchForeshorten = sin(effPitch).coerceIn(0.38f, 1.0f)

        // World ground coordinate initially under (focusPxX, focusPxY)
        val anchorWorldX = state.camFocusX + (focusPxX - viewportW * 0.5f) * metersPerPx0
        val anchorWorldY = state.camFocusY + (-(focusPxY - viewportH * 0.5f) / pitchForeshorten) * metersPerPx0

        state.onPinchScale(
            scaleFactor = 1.25f,
            focusPxX = focusPxX,
            focusPxY = focusPxY,
            viewportW = viewportW,
            viewportH = viewportH,
            projBlend = projBlend,
            panBoundMeters = 120f
        )

        // Across every intermediate transition frame, anchorWorld must project back to (focusPxX, focusPxY)
        repeat(45) { frame ->
            state.stepGlFrame(
                dt = 1f / 60f,
                uavX = 0f,
                uavY = 0f,
                uavZ = 1.2f,
                uavYaw = 0f,
                projBlend = projBlend,
                dampingScale = 1.0f
            )
            val metersPerPx = (state.worldHalfSpan * 2.0f) / viewportH.toFloat()
            val projectedPxX = viewportW * 0.5f + (anchorWorldX - state.camFocusX) / metersPerPx
            val projectedPxY = viewportH * 0.5f - ((anchorWorldY - state.camFocusY) * pitchForeshorten) / metersPerPx

            assertEquals("Frame $frame X focal drift", focusPxX, projectedPxX, 0.35f)
            assertEquals("Frame $frame Y focal drift", focusPxY, projectedPxY, 0.35f)
        }
    }

    @Test
    fun movingDrone3DModelAndFlightPathTracerStaySynchronizedDuringZoomAndPanTransitions() {
        val state = LogarithmicMapInterpolator.CameraTransitionState()
        val tracer = FlightPathTracer(capacity = 64, minSampleStepMeters = 0.08f, teleportResetMeters = 15f)
        val triArr = FloatArray(1024 * 8)
        val lineArr = FloatArray(256 * 8)

        // Settle initial follow lock onto starting drone pose
        state.recenterLockDrone()
        repeat(60) {
            state.stepGlFrame(
                dt = 1f / 60f,
                uavX = 0f,
                uavY = 0f,
                uavZ = 1.2f,
                uavYaw = 0f,
                projBlend = 1.0f,
                dampingScale = 1.0f
            )
        }

        // Phase A: Drone flies along a 3D curve while user zooms out logarithmically in follow mode
        state.zoomByFactor(0.35f)
        var prevAdaptiveScale = LogarithmicMapInterpolator.computeLogAdaptiveUavScale(state.worldHalfSpan)
        for (step in 1..40) {
            val t = step * (1f / 60f)
            val uavX = 3.2f * sin(t * 1.4f)
            val uavY = 2.6f * (1f - cos(t * 1.4f))
            val uavZ = 1.2f + 0.45f * sin(t * 2.0f)
            val projBlend = 0.65f // Mid-transition 2D <-> 3D blend to verify unified zScale latch
            val zScale = 0.08f + 0.92f * projBlend

            state.stepGlFrame(
                dt = 1f / 60f,
                uavX = uavX,
                uavY = uavY,
                uavZ = uavZ,
                uavYaw = t * 1.4f,
                projBlend = projBlend,
                dampingScale = 1.0f
            )

            // Steady follow mode must keep zero lag behind the moving 3D drone even while zooming
            assertEquals(uavX, state.camFocusX, 1e-5f)
            assertEquals(uavY, state.camFocusY, 1e-5f)
            assertEquals((uavZ * 0.45f) * projBlend, state.camFocusZ, 1e-5f)

            val uavScale = LogarithmicMapInterpolator.computeLogAdaptiveUavScale(state.worldHalfSpan)
            assertTrue("Adaptive UAV scale must evolve monotonically during zoom-out", uavScale >= prevAdaptiveScale - 1e-5f)
            prevAdaptiveScale = uavScale

            val widthZoom = 0.72f + 0.28f * uavScale
            tracer.recordPose(uavX, uavY, uavZ)
            tracer.emitFadingPolyline(
                liveX = uavX,
                liveY = uavY,
                liveZ = uavZ,
                zScale = zScale,
                eyeX = state.camFocusX - 6f,
                eyeY = state.camFocusY - 6f,
                eyeZ = state.camFocusZ + 5f,
                activePointLimit = 48,
                tailR = 0.14f,
                tailG = 0.54f,
                tailB = 1.0f,
                headR = 0.14f,
                headG = 0.96f,
                headB = 0.88f,
                maxAlpha = 0.90f,
                headHalfWidth = 0.044f * widthZoom,
                tailHalfWidth = 0.007f * widthZoom,
                triArray = triArr,
                triVertStart = 0,
                maxTriVerts = 1024,
                lineArray = lineArr,
                lineVertStart = 0,
                maxLineVerts = 256
            )

            if (tracer.lastLineVertCount > 0) {
                val headBase = (tracer.lastLineVertCount - 1) * 8
                // Trail head vertex must match the 3D UAV model's exact rendered world anchor
                assertEquals(uavX, lineArr[headBase], 1e-5f)
                assertEquals(uavY, lineArr[headBase + 1], 1e-5f)
                assertEquals(uavZ * zScale, lineArr[headBase + 2], 1e-5f)
            }
        }

        // Phase B: Free-pan away + zoom in + smooth recenter glide back onto the moving drone
        state.focusWorldPointSmooth(12.0f, -9.0f, panBoundMeters = 120f)
        repeat(15) {
            state.stepGlFrame(1f / 60f, 2.0f, 1.5f, 1.4f, 0.5f, 1.0f, 1.0f)
        }
        state.recenterLockDrone()
        state.zoomByFactor(2.4f)

        var prevOffsetDist = Float.POSITIVE_INFINITY
        for (step in 1..60) {
            val t = (40 + step) * (1f / 60f)
            val uavX = 3.2f * sin(t * 1.4f)
            val uavY = 2.6f * (1f - cos(t * 1.4f))
            val uavZ = 1.2f + 0.45f * sin(t * 2.0f)

            state.stepGlFrame(1f / 60f, uavX, uavY, uavZ, t * 1.4f, 1.0f, 1.0f)

            val dx = state.camFocusX - uavX
            val dy = state.camFocusY - uavY
            val dist = kotlin.math.hypot(dx, dy)
            assertTrue("Recenter offset must monotonically contract toward moving drone", dist <= prevOffsetDist + 1e-5f)
            prevOffsetDist = dist

            tracer.recordPose(uavX, uavY, uavZ)
            tracer.emitFadingPolyline(
                liveX = uavX,
                liveY = uavY,
                liveZ = uavZ,
                zScale = 1.0f,
                eyeX = state.camFocusX - 5f,
                eyeY = state.camFocusY - 5f,
                eyeZ = state.camFocusZ + 4f,
                activePointLimit = 48,
                tailR = 0.14f,
                tailG = 0.54f,
                tailB = 1.0f,
                headR = 0.14f,
                headG = 0.96f,
                headB = 0.88f,
                maxAlpha = 0.90f,
                headHalfWidth = 0.044f,
                tailHalfWidth = 0.007f,
                triArray = triArr,
                triVertStart = 0,
                maxTriVerts = 1024,
                lineArray = lineArr,
                lineVertStart = 0,
                maxLineVerts = 256
            )
            val headBase = (tracer.lastLineVertCount - 1) * 8
            assertEquals(uavX, lineArr[headBase], 1e-5f)
            assertEquals(uavY, lineArr[headBase + 1], 1e-5f)
            assertEquals(uavZ, lineArr[headBase + 2], 1e-5f)
        }
        assertEquals(0.0f, prevOffsetDist, 1e-4f)
    }

    @Test
    fun canonicalZeroPreventsNegativeZeroBitMismatchAfterSettling() {
        val state = LogarithmicMapInterpolator.CameraTransitionState()
        state.focusWorldPointSmooth(-0.0f, -0.0f, panBoundMeters = 120f)
        repeat(90) {
            state.stepGlFrame(
                dt = 1f / 60f,
                uavX = -0.0f,
                uavY = -0.0f,
                uavZ = 0f,
                uavYaw = 0f,
                projBlend = 0f,
                dampingScale = 1.0f
            )
        }
        assertEquals(0, java.lang.Float.floatToRawIntBits(state.camFocusX))
        assertEquals(0, java.lang.Float.floatToRawIntBits(state.camFocusY))
        assertEquals(0, java.lang.Float.floatToRawIntBits(state.camFocusZ))
        assertTrue(abs(state.worldHalfSpan - LogarithmicMapInterpolator.DEFAULT_HALF_SPAN) <= 1e-5f)
    }
}
