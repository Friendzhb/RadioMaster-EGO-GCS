package com.radiomaster.egogcs.ui

import org.junit.Assert.*
import org.junit.Test

class RenderSchedulingTest {
    @Test fun pacingSupportsSixtyNinetyAndOneTwentyHertzScreens() {
        for (screen in listOf(60, 90, 120)) {
            for (target in listOf(30, 60)) {
                val pacer = FramePacer()
                val delivered = (0 until screen * 10).count { pacer.due(it * 1_000_000_000L / screen, target) }
                assertTrue("$screen Hz / $target FPS: $delivered", kotlin.math.abs(delivered - target * 10) <= 1)
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
}
