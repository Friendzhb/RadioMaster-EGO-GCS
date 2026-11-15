package com.radiomaster.egogcs.ui

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class FlightPathTracerTest {

    @Test
    fun fifoBufferEvictsOldestPointsAndPreservesChronologicalOrder() {
        val tracer = FlightPathTracer(capacity = 6, minSampleStepMeters = 0.10f, teleportResetMeters = 10f)
        val pt = FloatArray(3)

        for (step in 0 until 10) {
            assertTrue(tracer.recordPose(step * 0.25f, step * 0.10f, 1.2f))
        }

        assertEquals(6, tracer.size)
        for (i in 0 until tracer.size) {
            tracer.getPoint(i, pt)
            val expectedStep = i + 4
            assertEquals(expectedStep * 0.25f, pt[0], 1e-5f)
            assertEquals(expectedStep * 0.10f, pt[1], 1e-5f)
            assertEquals(1.2f, pt[2], 1e-5f)
        }
    }

    @Test
    fun stationaryDroneEmitsZeroVerticesWhileSubStepMotionBridgesToLivePose() {
        val tracer = FlightPathTracer(capacity = 16, minSampleStepMeters = 0.10f, teleportResetMeters = 10f)
        val triArr = FloatArray(512 * 8)
        val lineArr = FloatArray(128 * 8)

        assertTrue(tracer.recordPose(1.0f, 2.0f, 1.2f))
        assertFalse(tracer.recordPose(1.0f, 2.0f, 1.2f))
        assertEquals(1, tracer.size)

        tracer.emitFadingPolyline(
            liveX = 1.0f,
            liveY = 2.0f,
            liveZ = 1.2f,
            zScale = 1.0f,
            eyeX = -4.0f,
            eyeY = -4.0f,
            eyeZ = 5.0f,
            activePointLimit = 16,
            tailR = 0.1f,
            tailG = 0.5f,
            tailB = 1.0f,
            headR = 0.1f,
            headG = 0.95f,
            headB = 0.88f,
            maxAlpha = 0.9f,
            headHalfWidth = 0.04f,
            tailHalfWidth = 0.008f,
            triArray = triArr,
            triVertStart = 0,
            maxTriVerts = 512,
            lineArray = lineArr,
            lineVertStart = 0,
            maxLineVerts = 128
        )
        assertEquals(0, tracer.lastTriVertCount)
        assertEquals(0, tracer.lastLineVertCount)

        // Sub-step motion (5 cm < 10 cm gate) does not push into FIFO yet, but still bridges visually
        assertFalse(tracer.recordPose(1.05f, 2.0f, 1.2f))
        assertEquals(1, tracer.size)
        tracer.emitFadingPolyline(
            liveX = 1.05f,
            liveY = 2.0f,
            liveZ = 1.2f,
            zScale = 1.0f,
            eyeX = -4.0f,
            eyeY = -4.0f,
            eyeZ = 5.0f,
            activePointLimit = 16,
            tailR = 0.1f,
            tailG = 0.5f,
            tailB = 1.0f,
            headR = 0.1f,
            headG = 0.95f,
            headB = 0.88f,
            maxAlpha = 0.9f,
            headHalfWidth = 0.04f,
            tailHalfWidth = 0.008f,
            triArray = triArr,
            triVertStart = 0,
            maxTriVerts = 512,
            lineArray = lineArr,
            lineVertStart = 0,
            maxLineVerts = 128
        )
        assertEquals(12, tracer.lastTriVertCount)
        assertEquals(2, tracer.lastLineVertCount)
    }

    @Test
    fun fadingPolylineMonotonicallyIncreasesAlphaFromZeroTailToDroneHeadAndHonorsActiveLimit() {
        val tracer = FlightPathTracer(capacity = 12, minSampleStepMeters = 0.10f, teleportResetMeters = 10f)
        for (i in 0 until 10) {
            tracer.recordPose(i * 0.20f, 0f, 1.2f)
        }
        assertEquals(10, tracer.size)

        val triArr = FloatArray(512 * 8)
        val lineArr = FloatArray(128 * 8)
        tracer.emitFadingPolyline(
            liveX = 1.80f,
            liveY = 0f,
            liveZ = 1.2f,
            zScale = 1.0f,
            eyeX = 0f,
            eyeY = -6f,
            eyeZ = 4f,
            activePointLimit = 6,
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
            maxTriVerts = 512,
            lineArray = lineArr,
            lineVertStart = 0,
            maxLineVerts = 128
        )

        // 6 active points => 5 segments => 10 line vertices and 60 feathered ribbon vertices
        assertEquals(10, tracer.lastLineVertCount)
        assertEquals(60, tracer.lastTriVertCount)

        // Tail alpha starts at 0.0 and monotonically rises to maxAlpha (0.90) at the drone head
        val firstStartAlpha = lineArr[6]
        assertEquals(0.0f, firstStartAlpha, 1e-5f)

        var prevAlpha = firstStartAlpha
        for (seg in 0 until 5) {
            val endVert = seg * 2 + 1
            val endAlpha = lineArr[endVert * 8 + 6]
            assertTrue("Segment $seg alpha should increase monotonically", endAlpha > prevAlpha)
            prevAlpha = endAlpha
        }
        assertEquals(0.90f, prevAlpha, 1e-4f)

        // Teleport jump resets FIFO history cleanly so no giant line crosses the map
        assertTrue(tracer.recordPose(25.0f, -18.0f, 1.5f))
        assertEquals(1, tracer.size)
    }
}
