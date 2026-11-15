package com.radiomaster.egogcs.ui

import com.radiomaster.egogcs.engine.EgoSimEngine
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.RenderSnapshot
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.tan
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

class MissionRouteVisualizerTest {

    private class RecordingSink : MissionRouteVisualizer.GeometrySink {
        var actorTris = 0
        var actorLines = 0
        var actorQuads = 0
        var actorDiscs = 0
        var actorRings = 0
        var groundLines = 0
        var groundDashedSegs = 0
        var fxRings = 0
        var fxCurtainQuads = 0
        val routeQuadArcRanges = mutableListOf<Pair<Float, Float>>()
        val actorTriAlphas = mutableListOf<Float>()
        val dangerSegmentsRecorded = mutableListOf<Boolean>()

        override fun pushActorTri(
            x0: Float,
            y0: Float,
            z0: Float,
            arc0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            arc1: Float,
            x2: Float,
            y2: Float,
            z2: Float,
            arc2: Float,
            r: Float,
            g: Float,
            b: Float,
            a: Float
        ) {
            actorTris++
            actorTriAlphas.add(a)
        }

        override fun pushActorLine(
            x0: Float,
            y0: Float,
            z0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            r: Float,
            g: Float,
            b: Float,
            a: Float,
            arc0: Float,
            arc1: Float
        ) {
            actorLines++
        }

        override fun pushActorQuadSeg3D(
            x0: Float,
            y0: Float,
            z0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            halfW: Float,
            r: Float,
            g: Float,
            b: Float,
            a: Float,
            arc0: Float,
            arc1: Float
        ) {
            actorQuads++
            val horizSpan = hypot(x1 - x0, y1 - y0)
            if (arc0 >= 0f && arc1 > arc0 && horizSpan > 0.1f) {
                routeQuadArcRanges.add(arc0 to arc1)
                dangerSegmentsRecorded.add(r > 0.95f && g < 0.45f && b < 0.35f)
            }
        }

        override fun pushActorDiscXY(
            cx: Float,
            cy: Float,
            z: Float,
            rad: Float,
            segs: Int,
            r: Float,
            g: Float,
            b: Float,
            a: Float,
            arc: Float
        ) {
            actorDiscs++
        }

        override fun pushActorRingXY(
            cx: Float,
            cy: Float,
            z: Float,
            rad: Float,
            thick: Float,
            segs: Int,
            r: Float,
            g: Float,
            b: Float,
            a: Float,
            arc: Float
        ) {
            actorRings++
        }

        override fun pushGroundLine(
            x0: Float,
            y0: Float,
            z0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            r: Float,
            g: Float,
            b: Float,
            a: Float
        ) {
            groundLines++
        }

        override fun pushGroundDashedSeg(
            x0: Float,
            y0: Float,
            z0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            r: Float,
            g: Float,
            b: Float,
            a: Float
        ) {
            groundDashedSegs++
        }

        override fun pushFxRingXY(
            cx: Float,
            cy: Float,
            z: Float,
            rad: Float,
            thick: Float,
            segs: Int,
            r: Float,
            g: Float,
            b: Float,
            a: Float
        ) {
            fxRings++
        }

        override fun pushFxGradientQuad(
            x0: Float,
            y0: Float,
            z0: Float,
            r0: Float,
            g0: Float,
            b0: Float,
            a0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            r1: Float,
            g1: Float,
            b1: Float,
            a1: Float,
            x2: Float,
            y2: Float,
            z2: Float,
            r2: Float,
            g2: Float,
            b2: Float,
            a2: Float,
            x3: Float,
            y3: Float,
            z3: Float,
            r3: Float,
            g3: Float,
            b3: Float,
            a3: Float
        ) {
            fxCurtainQuads++
        }
    }

    @Test
    fun dynamicWaypointPlacementSelectionDragAndVerticalAltitudeSliderSyncToSnapshot() {
        val hub = GcsSharedHub().apply {
            isRosMode = false // Demo obstacle arena so obstacle column heights can also be queried
            goalZ = 1.4f
            routeLoopMode = 1 // Closed-loop mission
            waypointPlacementMode = true
        }
        val engine = EgoSimEngine(hub)

        val wp0 = engine.placeDynamicWaypoint(-2.0f, 1.0f, 1.2f)
        val wp1 = engine.placeDynamicWaypoint(2.0f, 1.0f, 1.8f)
        val wp2 = engine.placeDynamicWaypoint(2.0f, 4.0f, 2.2f)

        assertEquals(0, wp0.index)
        assertEquals(1, wp1.index)
        assertEquals(2, wp2.index)
        assertEquals(3, wp2.totalCount)
        assertTrue("Route arc length should be positive", wp2.routeArcMeters > 7.0f)

        // Select waypoint #1 (middle waypoint), drag XY on ground plane, and drag its 3D vertical altitude slider
        val selected = engine.selectOrchestratedWaypoint(1)
        assertNotNull(selected)
        assertEquals(1, selected!!.index)
        assertEquals(1.8f, hub.goalZ, 1e-4f)

        val dragged = engine.updateOrchestratedWaypointXY(1, 3.25f, 1.75f)
        assertNotNull(dragged)
        assertEquals(3.25f, dragged!!.x, 1e-4f)
        assertEquals(1.75f, dragged.y, 1e-4f)

        // Continuous vertical slider update clamps to [0.4f, 4.0f] and syncs goalZ for the active waypoint
        val clampedHigh = engine.updateOrchestratedWaypointZ(2, 9.5f)
        assertNotNull(clampedHigh)
        assertEquals(4.0f, clampedHigh!!.z, 1e-4f)

        val altDragged = engine.updateOrchestratedWaypointZ(1, 2.75f)
        assertNotNull(altDragged)
        assertEquals(2.75f, altDragged!!.z, 1e-4f)
        assertEquals(2.75f, hub.goalZ, 1e-4f)

        val (nudgedZ, pinInfo) = engine.nudgeGoalZ(-0.45f)
        assertEquals(2.30f, nudgedZ, 1e-4f)
        assertEquals(null, pinInfo)

        // Step one simulation tick so RenderSnapshot receives full 3D waypoint, clearance & obstacle column top telemetry
        engine.stepSingleTickForTest()
        val snap = hub.snapshots.readInto(RenderSnapshot())
        assertEquals(3, snap.waypointQueueCount)
        assertTrue(snap.isRouteStagedOnly)
        assertTrue(snap.routeClosedLoop)
        assertEquals(1, snap.selectedWaypointIdx)
        assertEquals(-2.0f, snap.routeLoopAnchorX, 1e-4f)
        assertEquals(1.0f, snap.routeLoopAnchorY, 1e-4f)
        assertEquals(1.2f, snap.routeLoopAnchorZ, 1e-4f)
        assertEquals(3.25f, snap.waypointQueuePts[3], 1e-4f)
        assertEquals(1.75f, snap.waypointQueuePts[4], 1e-4f)
        assertEquals(2.30f, snap.waypointQueuePts[5], 1e-4f)
        assertEquals(4.0f, snap.waypointQueuePts[8], 1e-4f)
        assertTrue("Exported clearance must be positive in open space", snap.waypointClearances[1] > 0.2f)
        assertTrue("Column top Z is non-negative", snap.waypointColumnTopZs[1] >= 0f)
    }

    @Test
    fun emitPlannedMissionRouteGeneratesConnected3DLegsVerticalSliderGizmoAndClosedLoop() {
        val emptySnap = RenderSnapshot().apply {
            uavX = -3.0f
            uavY = -2.0f
            uavZ = 1.1f
            waypointQueueCount = 0
        }
        val emptySink = RecordingSink()
        val tip = FloatArray(3)
        MissionRouteVisualizer.emitPlannedMissionRoute(
            snap = emptySnap,
            zScale = 1.0f,
            blend3D = 1.0f,
            uavZoomScale = 1.0f,
            showCurtain = true,
            shaderClockSec = 1.25f,
            spawnAnimIdx = -1,
            spawnAnimScale = 1.0f,
            sink = emptySink,
            outLastTipXYZ = tip
        )
        assertEquals(0, emptySink.actorTris)
        assertEquals(0, emptySink.actorQuads)
        assertEquals(-3.0f, tip[0], 1e-4f)

        val routeSnap = RenderSnapshot().apply {
            uavX = 0.0f
            uavY = 0.0f
            uavZ = 1.0f
            waypointQueueCount = 3
            isRouteStagedOnly = true
            routeClosedLoop = true
            selectedWaypointIdx = 1
            routeLoopAnchorX = 2.0f
            routeLoopAnchorY = 0.0f
            routeLoopAnchorZ = 1.5f
            // WP 0
            waypointQueuePts[0] = 2.0f
            waypointQueuePts[1] = 0.0f
            waypointQueuePts[2] = 1.5f
            waypointSafeFlags[0] = true
            // WP 1 (selected + unsafe + obstacle column underneath -> triggers 3D vertical slider & ceiling ring)
            waypointQueuePts[3] = 2.0f
            waypointQueuePts[4] = 3.0f
            waypointQueuePts[5] = 2.0f
            waypointSafeFlags[1] = false
            waypointColumnTopZs[1] = 1.65f
            // WP 2
            waypointQueuePts[6] = -1.0f
            waypointQueuePts[7] = 3.0f
            waypointQueuePts[8] = 1.2f
            waypointSafeFlags[2] = true
        }

        val sink = RecordingSink()
        MissionRouteVisualizer.emitPlannedMissionRoute(
            snap = routeSnap,
            zScale = 1.0f,
            blend3D = 1.0f,
            uavZoomScale = 1.0f,
            showCurtain = true,
            shaderClockSec = 0.75f,
            spawnAnimIdx = 2,
            spawnAnimScale = 0.92f,
            sink = sink,
            outLastTipXYZ = tip,
            activeAltDragIdx = 1
        )

        // 3 waypoints * 8 octahedral faces (24) + selected WP #1 dual-chevron vertical slider cones (8) = 32 triangles
        assertEquals(32, sink.actorTris)
        // 3 forward connecting legs (UAV->WP0, WP0->WP1, WP1->WP2) + 1 closed-loop return leg (WP2->WP0) = 4 dashed ground legs & 4 curtains
        assertEquals(4, sink.groundDashedSegs)
        assertEquals(4, sink.fxCurtainQuads)
        // Selected WP #1 emits: obstacle-ceiling ring + elevated selection ring + slider knob collar ring = 3 actor rings
        assertEquals(3, sink.actorRings)
        // Route arc distances along the 4 main connecting route legs: verify monotonic cumulative arc
        val legArcs = sink.routeQuadArcRanges
        assertEquals(4, legArcs.size)
        assertEquals(0.0f, legArcs[0].first, 1e-4f)
        assertEquals(legArcs[0].second, legArcs[1].first, 1e-4f)
        assertEquals(legArcs[1].second, legArcs[2].first, 1e-4f)
        assertEquals(legArcs[2].second, legArcs[3].first, 1e-4f)
        assertTrue("Unsafe waypoint leg should be flagged with danger color", sink.dangerSegmentsRecorded.any { it })
        // Last staged tip points to WP 2
        assertEquals(-1.0f, tip[0], 1e-4f)
        assertEquals(3.0f, tip[1], 1e-4f)
        assertEquals(1.2f * 0.92f, tip[2], 1e-4f)
    }

    @Test
    fun pickWaypointInteractionDistinguishes3DVerticalSliderFromHorizontalBodyAndStem() {
        val vw = 1280
        val vh = 720
        val vp = buildPerspectiveLookAtVp(
            eyeX = 0f,
            eyeY = -9.0f,
            eyeZ = 5.5f,
            centerX = 0f,
            centerY = 0f,
            centerZ = 1.0f,
            fovYRad = Math.toRadians(48.0).toFloat(),
            aspect = vw.toFloat() / vh.toFloat(),
            near = 0.2f,
            far = 80f
        )

        val pts = FloatArray(RenderSnapshot.MAX_MISSION_WAYPOINTS * 3)
        // WP 0 at (-2.5, 0.5, 1.8m)
        pts[0] = -2.5f
        pts[1] = 0.5f
        pts[2] = 1.8f
        // WP 1 at (2.2, 1.0, 2.4m) - selected
        pts[3] = 2.2f
        pts[4] = 1.0f
        pts[5] = 2.4f

        val xyw = FloatArray(3)
        assertTrue(MissionRouteVisualizer.projectWorldToScreen(vp, vw, vh, 2.2f, 1.0f, 2.4f, xyw))
        val head1PxX = xyw[0]
        val head1PxY = xyw[1]

        // Touching the upper 3D vertical altitude slider handle / digital callout zone above selected WP 1 -> DRAG_ALTITUDE_Z
        val sliderKnobWorldZ = 2.4f + 0.30f
        assertTrue(MissionRouteVisualizer.projectWorldToScreen(vp, vw, vh, 2.2f, 1.0f, sliderKnobWorldZ, xyw))
        val hitSlider = MissionRouteVisualizer.pickWaypointInteraction(
            touchPxX = xyw[0],
            touchPxY = xyw[1] - 8f,
            viewportW = vw,
            viewportH = vh,
            vpMatrix = vp,
            zScale = 1.0f,
            blend3D = 1.0f,
            waypointPts = pts,
            waypointCount = 2,
            selectedWaypointIdx = 1,
            maxPickRadiusPx = 44f,
            density = 1.0f
        )
        assertEquals(1, hitSlider.index)
        assertEquals(MissionRouteVisualizer.WaypointHitMode.DRAG_ALTITUDE_Z, hitSlider.mode)
        assertEquals(head1PxX, hitSlider.headScreenX, 1e-2f)
        assertEquals(head1PxY, hitSlider.headScreenY, 1e-2f)

        // Touching the lower vertical stem of selected WP 1 -> SELECT_OR_DRAG_XY (horizontal drag)
        assertTrue(MissionRouteVisualizer.projectWorldToScreen(vp, vw, vh, 2.2f, 1.0f, 1.0f, xyw))
        val hitLowerStem1 = MissionRouteVisualizer.pickWaypointInteraction(
            touchPxX = xyw[0],
            touchPxY = xyw[1],
            viewportW = vw,
            viewportH = vh,
            vpMatrix = vp,
            zScale = 1.0f,
            blend3D = 1.0f,
            waypointPts = pts,
            waypointCount = 2,
            selectedWaypointIdx = 1,
            maxPickRadiusPx = 44f,
            density = 1.0f
        )
        assertEquals(1, hitLowerStem1.index)
        assertEquals(MissionRouteVisualizer.WaypointHitMode.SELECT_OR_DRAG_XY, hitLowerStem1.mode)

        // Mid-stem hit on unselected WP 0 -> SELECT_OR_DRAG_XY
        assertTrue(MissionRouteVisualizer.projectWorldToScreen(vp, vw, vh, -2.5f, 0.5f, 0.9f, xyw))
        val hitStem0 = MissionRouteVisualizer.pickWaypointScreenSpace(
            touchPxX = xyw[0],
            touchPxY = xyw[1],
            viewportW = vw,
            viewportH = vh,
            vpMatrix = vp,
            zScale = 1.0f,
            waypointPts = pts,
            waypointCount = 2,
            maxPickRadiusPx = 36f
        )
        assertEquals(0, hitStem0)

        // Far corner tap should miss all waypoints
        val miss = MissionRouteVisualizer.pickWaypointScreenSpace(
            touchPxX = 24f,
            touchPxY = 24f,
            viewportW = vw,
            viewportH = vh,
            vpMatrix = vp,
            zScale = 1.0f,
            waypointPts = pts,
            waypointCount = 2,
            maxPickRadiusPx = 44f
        )
        assertEquals(-1, miss)
    }

    @Test
    fun solveWaypointAltitudeFromScreenYTracksPerspectiveVerticalAxisAndEtaScalesCleanly() {
        val vw = 1280
        val vh = 720
        val vp = buildPerspectiveLookAtVp(
            eyeX = 0f,
            eyeY = -9.5f,
            eyeZ = 5.2f,
            centerX = 0f,
            centerY = 0f,
            centerZ = 1.2f,
            fovYRad = Math.toRadians(48.0).toFloat(),
            aspect = vw.toFloat() / vh.toFloat(),
            near = 0.2f,
            far = 80f
        )

        val wx = 1.8f
        val wy = 0.7f
        val startAlt = 1.50f
        val zScale = 1.15f
        val xyw = FloatArray(3)

        assertTrue(MissionRouteVisualizer.projectWorldToScreen(vp, vw, vh, wx, wy, startAlt * zScale, xyw))
        val headStartY = xyw[1]
        // Simulate user grabbing the vertical slider handle 34px above the diamond head center
        val downTouchY = headStartY - 34f
        val grabOffsetY = downTouchY - headStartY

        // At initial touch down, solved Z must equal startAlt with zero grab jump
        val initialSolvedZ = MissionRouteVisualizer.solveWaypointAltitudeFromScreenY(
            touchScreenY = downTouchY,
            grabOffsetScreenY = grabOffsetY,
            viewportH = vh,
            vpMatrix = vp,
            zScale = zScale,
            blend3D = 1.0f,
            wx = wx,
            wy = wy,
            startAltitudeMeters = startAlt,
            downScreenY = downTouchY,
            density = 2.0f
        )
        assertEquals(startAlt, initialSolvedZ, 1e-3f)

        // Dragging finger to screen Y corresponding to targetAlt = 2.85m (+ grabOffsetY) recovers 2.85m
        val targetAlt = 2.85f
        assertTrue(MissionRouteVisualizer.projectWorldToScreen(vp, vw, vh, wx, wy, targetAlt * zScale, xyw))
        val movedTouchY = xyw[1] + grabOffsetY
        val movedSolvedZ = MissionRouteVisualizer.solveWaypointAltitudeFromScreenY(
            touchScreenY = movedTouchY,
            grabOffsetScreenY = grabOffsetY,
            viewportH = vh,
            vpMatrix = vp,
            zScale = zScale,
            blend3D = 1.0f,
            wx = wx,
            wy = wy,
            startAltitudeMeters = startAlt,
            downScreenY = downTouchY,
            density = 2.0f
        )
        assertEquals(targetAlt, movedSolvedZ, 1e-3f)

        // 2D top-down fallback (`blend3D = 0f`) maps 85dp upward drag to +1.0m
        val topDownZ = MissionRouteVisualizer.solveWaypointAltitudeFromScreenY(
            touchScreenY = 300f - 170f, // 85dp * density(2.0) = 170px upward
            grabOffsetScreenY = -20f,
            viewportH = vh,
            vpMatrix = vp,
            zScale = 0.08f,
            blend3D = 0.0f,
            wx = wx,
            wy = wy,
            startAltitudeMeters = 1.40f,
            downScreenY = 300f,
            density = 2.0f
        )
        assertEquals(2.40f, topDownZ, 1e-3f)

        // Mission ETA helper sanity checks
        assertEquals(0, MissionRouteVisualizer.computeMissionEtaSeconds(0f, 0, 1.8f))
        val etaShort = MissionRouteVisualizer.computeMissionEtaSeconds(10.0f, 2, 2.0f)
        val etaLong = MissionRouteVisualizer.computeMissionEtaSeconds(25.0f, 5, 2.0f)
        assertTrue("Short route ETA should be positive", etaShort in 6..12)
        assertTrue("Longer multi-turn route should have strictly larger ETA", etaLong > etaShort)
    }

    private fun buildPerspectiveLookAtVp(
        eyeX: Float,
        eyeY: Float,
        eyeZ: Float,
        centerX: Float,
        centerY: Float,
        centerZ: Float,
        fovYRad: Float,
        aspect: Float,
        near: Float,
        far: Float
    ): FloatArray {
        var fx = centerX - eyeX
        var fy = centerY - eyeY
        var fz = centerZ - eyeZ
        val fLen = hypot(hypot(fx, fy), fz)
        fx /= fLen
        fy /= fLen
        fz /= fLen

        // World up = (0, 0, 1); s = normalize(f x up)
        var sx = fy * 1f - fz * 0f
        var sy = fz * 0f - fx * 1f
        var sz = 0f
        val sLen = hypot(hypot(sx, sy), sz)
        sx /= sLen
        sy /= sLen
        sz /= sLen

        // u = s x f
        val ux = sy * fz - sz * fy
        val uy = sz * fx - sx * fz
        val uz = sx * fy - sy * fx

        val view = FloatArray(16)
        view[0] = sx
        view[4] = sy
        view[8] = sz
        view[12] = -(sx * eyeX + sy * eyeY + sz * eyeZ)

        view[1] = ux
        view[5] = uy
        view[9] = uz
        view[13] = -(ux * eyeX + uy * eyeY + uz * eyeZ)

        view[2] = -fx
        view[6] = -fy
        view[10] = -fz
        view[14] = (fx * eyeX + fy * eyeY + fz * eyeZ)

        view[3] = 0f
        view[7] = 0f
        view[11] = 0f
        view[15] = 1f

        val fCot = 1.0f / tan(fovYRad * 0.5f)
        val proj = FloatArray(16)
        proj[0] = fCot / aspect
        proj[5] = fCot
        proj[10] = (far + near) / (near - far)
        proj[11] = -1f
        proj[14] = (2f * far * near) / (near - far)

        // Trigger unused-import prevention for sin/cos if needed via orthogonality sanity check
        assertTrue(cos(0.0) + sin(0.0) > 0.99)

        val vp = FloatArray(16)
        for (col in 0..3) {
            for (row in 0..3) {
                var sum = 0f
                for (k in 0..3) {
                    sum += proj[k * 4 + row] * view[col * 4 + k]
                }
                vp[col * 4 + row] = sum
            }
        }
        return vp
    }
}
