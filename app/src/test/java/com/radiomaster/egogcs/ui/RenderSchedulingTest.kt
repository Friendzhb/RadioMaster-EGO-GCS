package com.radiomaster.egogcs.ui

import org.junit.Assert.*
import org.junit.Test

class RenderSchedulingTest {
    @Test fun pacingSupportsSixtyNinetyAndOneTwentyHertzScreens() {
        for (screen in listOf(60, 90, 120)) {
            for (target in listOf(15, 20, 30, 45, 60)) {
                val pacer = FramePacer()
                val delivered = (0 until screen * 10).count { pacer.due(it * 1_000_000_000L / screen, target) }
                assertTrue("$screen Hz / $target FPS: $delivered", kotlin.math.abs(delivered - target * 10) <= 2)
            }
        }
    }

    @Test fun stallsAndRateChangesNeverProduceCatchUpBursts() {
        val pacer = FramePacer()
        assertTrue(pacer.due(0, 60))
        assertTrue(pacer.due(10_000_000_000L, 60))
        assertFalse(pacer.due(10_000_000_000L, 60))
        assertTrue(pacer.due(10_000_000_000L, 30))
        assertFalse(pacer.due(10_001_000_000L, 30))
        pacer.reset()
        assertTrue(pacer.due(20_000_000_000L, 60))
    }

    @Test fun onlyOneRenderCanBePending() {
        val gate = PendingRender()
        assertTrue(gate.request())
        repeat(1000) { assertFalse(gate.request()) }
        gate.complete()
        assertTrue(gate.request())
    }

    @Test fun exactKeysInvalidateOnGeometryAndCameraChanges() {
        val key = GeometryKey(10)
        val points = floatArrayOf(1f, 2f, 3f)
        key.begin()
        key.add(points)
        key.add(0f)
        assertFalse(key.unchanged())
        key.commit()
        key.begin()
        key.add(points)
        key.add(0f)
        assertTrue(key.unchanged())
        key.begin()
        key.add(points)
        key.add(-0f)
        assertFalse(key.unchanged())
        points[2] = 4f
        key.begin()
        key.add(points)
        key.add(0f)
        assertFalse(key.unchanged())
        key.invalidate()
        assertFalse(key.unchanged())
    }

    @Test fun diagnosticsDoNotHideFramesLongerThanFiftyMilliseconds() {
        val metrics = RenderMetrics()
        metrics.record(200_000_000L, 16_384, false)
        assertEquals(200f, metrics.lastCpuMs, 0.001f)
        assertEquals(200f, metrics.p95CpuMs(), 0.001f)
    }

    @Test fun rotorSpinStaysCompletelyStillWhenDisarmedAndZeroThrottle() {
        val rotor = RotorSpinState()
        val initialPhases = rotor.phasesRad.clone()
        repeat(60) {
            assertFalse(rotor.step(1f / 60f, armed = false, m0 = 0f, m1 = 0f, m2 = 0f, m3 = 0f))
        }
        assertFalse(rotor.isSpinning)
        assertArrayEquals(initialPhases, rotor.phasesRad, 0.0f)
        for (i in 0..3) {
            assertEquals(0f, rotor.omegaNorm[i], 0.0f)
            assertEquals(0f, rotor.blurStrength[i], 0.0f)
        }
    }

    @Test fun rotorSpinSpoolsUpWithInertiaAndCoastsToZeroOnDisarm() {
        val rotor = RotorSpinState()
        val p0Before = rotor.phasesRad[0]
        assertTrue(rotor.step(1f / 60f, armed = true, m0 = 0.55f, m1 = 0.55f, m2 = 0.55f, m3 = 0.55f))
        val firstOmega = rotor.omegaNorm[0]
        assertTrue("First step should spool up gradually with inertia", firstOmega > 0.02f && firstOmega < 0.45f)
        assertNotEquals(p0Before, rotor.phasesRad[0], 0.0001f)

        repeat(45) {
            assertTrue(rotor.step(1f / 60f, armed = true, m0 = 0.55f, m1 = 0.55f, m2 = 0.55f, m3 = 0.55f))
        }
        assertTrue(rotor.omegaNorm[0] > 0.65f)
        assertTrue(rotor.blurStrength[0] > 0.60f)

        // Frozen clock (dt == 0f) must never drift phases or blur (preserves GL cache benchmark parity)
        val frozenPhases = rotor.phasesRad.clone()
        val frozenBlurs = rotor.blurStrength.clone()
        assertTrue(rotor.step(0f, armed = true, m0 = 0.55f, m1 = 0.55f, m2 = 0.55f, m3 = 0.55f))
        assertArrayEquals(frozenPhases, rotor.phasesRad, 0.0f)
        assertArrayEquals(frozenBlurs, rotor.blurStrength, 0.0f)

        // Disarming must coast down over multiple frames before snapping cleanly to exact rest
        assertTrue(
            "Immediately after disarming, rotors should still coast with inertia",
            rotor.step(1f / 60f, armed = false, m0 = 0f, m1 = 0f, m2 = 0f, m3 = 0f)
        )
        assertTrue(rotor.omegaNorm[0] > 0.1f)
        repeat(90) {
            rotor.step(1f / 60f, armed = false, m0 = 0f, m1 = 0f, m2 = 0f, m3 = 0f)
        }
        assertFalse("Rotors must come to a full stop after coast-down", rotor.isSpinning)
        for (i in 0..3) {
            assertEquals(0f, rotor.omegaNorm[i], 0.0f)
            assertEquals(0f, rotor.blurStrength[i], 0.0f)
        }

        // Direct motor throttle > threshold without explicit arm flag should also spin rotors
        assertTrue(rotor.step(1f / 60f, armed = false, m0 = 0.35f, m1 = 0.35f, m2 = 0.35f, m3 = 0.35f))
        assertTrue(rotor.isSpinning)
    }
}
