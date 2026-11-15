package com.radiomaster.egogcs.ui

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class TouchChainGuardTest {

    @Test
    fun `unscaleAxis and inverseScaleAxis preserve exact unscaled location when icon is spring-scaled`() {
        // 28dp @ 2x = 56px width, pivot = 28px, pressed scale = 0.89f
        val pivotX = 28f
        val scaleX = 0.89f
        val unscaledEdgeTouchX = 55f // 1px inside right edge of 56px icon

        // Android ViewGroup.dispatchTransformedTouchEvent transforms into >56px:
        val transformedX = TouchChainGuard.inverseScaleAxis(unscaledEdgeTouchX, pivotX, scaleX)
        assertTrue("Inverse-scaled point exceeds 56px width ($transformedX)", transformedX > 56f)

        // unscaleAxis recovers exact 55f so border touches do not prematurely cancel press state
        val recoveredX = TouchChainGuard.unscaleAxis(transformedX, pivotX, scaleX)
        assertEquals(unscaledEdgeTouchX, recoveredX, 1e-4f)
        assertTrue(
            TouchChainGuard.isPointInUnscaledBounds(
                unscaledLocalX = recoveredX,
                unscaledLocalY = 26f,
                width = 56,
                height = 52,
                slopPx = 0f
            )
        )
    }

    @Test
    fun `computeEdgeExpandedSlopRect asymmetrically expands slop to physical screen edges`() {
        // 1280x720 @ 2x density: top-right Settings gear [1208..1264, 12..64] (8dp right padding, 6dp top padding)
        val screenW = 1280f
        val screenH = 720f
        val gearRect = TouchChainGuard.TargetRect(
            id = 1,
            left = 1208f,
            top = 12f,
            right = 1264f,
            bottom = 64f
        )
        val gearSlop = TouchChainGuard.computeEdgeExpandedSlopRect(
            rect = gearRect,
            containerWinX = 0f,
            containerWinY = 0f,
            screenWidth = screenW,
            screenHeight = screenH,
            compactThresholdPx = 76f, // 38dp * 2
            compactSlopPx = 17f, // 8.5dp * 2
            standardSlopPx = 11f, // 5.5dp * 2
            edgeSlopPx = 32f, // 16dp * 2
            edgeZonePx = 56f // 28dp * 2
        )

        // Interior left/bottom use compactSlopPx (17f), while top/right reach all the way past (1280, 0)
        assertEquals(1208f - 17f, gearSlop.slopLeft, 1e-4f)
        assertEquals(64f + 17f, gearSlop.slopBottom, 1e-4f)
        assertTrue("Top slop reaches y=0 (${gearSlop.slopTop})", gearSlop.slopTop <= 0f)
        assertTrue("Right slop reaches x=1280 (${gearSlop.slopRight})", gearSlop.slopRight >= screenW)
        assertTrue(gearSlop.containsInSlop(1279f, 1f))

        // Left safety rail hosted at containerWinX = 20px (10dp leftMargin), btnBrake local [0..164, 0..54]
        val brakeLocalRect = TouchChainGuard.TargetRect(id = 0, left = 0f, top = 0f, right = 164f, bottom = 54f)
        val brakeSlop = TouchChainGuard.computeEdgeExpandedSlopRect(
            rect = brakeLocalRect,
            containerWinX = 20f,
            containerWinY = 88f,
            screenWidth = screenW,
            screenHeight = screenH,
            compactThresholdPx = 76f,
            compactSlopPx = 17f,
            standardSlopPx = 11f,
            edgeSlopPx = 32f,
            edgeZonePx = 56f
        )
        // Left slop in local coordinates reaches -22f (window x = -2px <= 0px), covering the full 10dp left screen margin
        assertEquals(-22f, brakeSlop.slopLeft, 1e-4f)
        assertEquals(164f + 17f, brakeSlop.slopRight, 1e-4f)
    }

    @Test
    fun `resolveSlopHitIndex routes extreme screen-corner touch to settings icon and splits adjacent gap`() {
        val screenW = 1280f
        val screenH = 720f
        // Menu [1092..1200, 12..64], 8px gap, Settings gear [1208..1264, 12..64]
        val menuRect = TouchChainGuard.TargetRect(id = 0, left = 1092f, top = 12f, right = 1200f, bottom = 64f)
        val gearRect = TouchChainGuard.TargetRect(id = 1, left = 1208f, top = 12f, right = 1264f, bottom = 64f)

        val candidates = listOf(menuRect, gearRect).map { r ->
            TouchChainGuard.computeEdgeExpandedSlopRect(
                rect = r,
                containerWinX = 0f,
                containerWinY = 0f,
                screenWidth = screenW,
                screenHeight = screenH,
                compactThresholdPx = 76f,
                compactSlopPx = 17f,
                standardSlopPx = 11f,
                edgeSlopPx = 32f,
                edgeZonePx = 56f
            )
        }

        // Top-rightmost pixel of screen (1279, 1) hits Settings gear (index 1)
        assertEquals(1, TouchChainGuard.resolveSlopHitIndex(1279f, 1f, candidates))

        // Gap Voronoi split: 1202 is closer to Menu (dx=2 vs 6); 1206 is closer to Settings gear (dx=2 vs 6)
        assertEquals(0, TouchChainGuard.resolveSlopHitIndex(1202f, 36f, candidates))
        assertEquals(1, TouchChainGuard.resolveSlopHitIndex(1206f, 36f, candidates))

        // Far center spacer (640, 30) hits neither (-1)
        assertEquals(-1, TouchChainGuard.resolveSlopHitIndex(640f, 30f, candidates))
    }

    @Test
    fun `resolveNearestTargetIndex partitions gap between menu and settings icons by Voronoi midpoint`() {
        // Menu button [400..508], 8px gap, Settings gear button [516..572]
        val menuRect = TouchChainGuard.TargetRect(id = 0, left = 400f, top = 7f, right = 508f, bottom = 59f)
        val gearRect = TouchChainGuard.TargetRect(id = 1, left = 516f, top = 7f, right = 572f, bottom = 59f)
        val targets = listOf(menuRect, gearRect)
        val haloSlopPx = 12f

        // Closer to Menu (x = 510 -> dxMenu = 2, dxGear = 6)
        assertEquals(0, TouchChainGuard.resolveNearestTargetIndex(510f, 30f, targets, haloSlopPx))

        // Closer to Settings gear (x = 514 -> dxMenu = 6, dxGear = 2)
        assertEquals(1, TouchChainGuard.resolveNearestTargetIndex(514f, 30f, targets, haloSlopPx))

        // In right padding next to Settings gear (x = 578 -> dxGear = 6 <= 12)
        assertEquals(1, TouchChainGuard.resolveNearestTargetIndex(578f, 30f, targets, haloSlopPx))

        // Far in empty middle spacer of top bar (x = 260 -> outside haloSlopPx, returns -1 so SOLID_BAR absorbs it)
        assertEquals(-1, TouchChainGuard.resolveNearestTargetIndex(260f, 30f, targets, haloSlopPx))
    }

    @Test
    fun `mapParentPointIntoSlopTarget clamps screen-edge bezel touch and releases when dragged outside`() {
        val gearRect = TouchChainGuard.TargetRect(id = 1, left = 1208f, top = 12f, right = 1264f, bottom = 64f)
        val slopRect = TouchChainGuard.EdgeSlopRect(
            rect = gearRect,
            slopLeft = 1191f,
            slopTop = -2f,
            slopRight = 1282f,
            slopBottom = 81f
        )

        // Bezel touch at (1279, 1) is 15px right and 11px above icon, clamped cleanly inside [0.5..55.5, 0.5..51.5]
        val clamped = TouchChainGuard.mapParentPointIntoSlopTarget(1279f, 1f, slopRect)
        assertEquals(55.5f, clamped[0], 1e-4f)
        assertEquals(0.5f, clamped[1], 1e-4f)

        // Dragged down onto map (1236, 120) -> outside slopBottom (81f), returns unclamped local y = 108f to cancel press
        val draggedOut = TouchChainGuard.mapParentPointIntoSlopTarget(1236f, 120f, slopRect)
        assertEquals(28f, draggedOut[0], 1e-4f)
        assertEquals(108f, draggedOut[1], 1e-4f)
        assertFalse(
            TouchChainGuard.isPointInUnscaledBounds(
                unscaledLocalX = draggedOut[0],
                unscaledLocalY = draggedOut[1],
                width = 56,
                height = 52,
                slopPx = 16f
            )
        )
    }

    @Test
    fun `isWithinAnyRectMargin and isPointCoveredByAnyRect enforce gap shield and Z-occlusion`() {
        val btn1 = TouchChainGuard.TargetRect(id = 0, left = 0f, top = 0f, right = 164f, bottom = 58f)
        val btn2 = TouchChainGuard.TargetRect(id = 1, left = 0f, top = 67f, right = 164f, bottom = 125f)
        val visibleRects = listOf(btn1, btn2)

        // 9px vertical gap at y = 62f is shielded with 12px guardMarginPx
        assertTrue(TouchChainGuard.isWithinAnyRectMargin(80f, 62f, visibleRects, marginPx = 12f))

        // Open map area far away from visible cluster children is NOT blocked
        assertFalse(TouchChainGuard.isWithinAnyRectMargin(240f, 62f, visibleRects, marginPx = 12f))

        // Slide-over drawer rect occludes rightCamStack icon centers underneath it
        val drawerOccluder = TouchChainGuard.TargetRect(id = 9, left = 840f, top = 86f, right = 1264f, bottom = 704f)
        assertTrue(TouchChainGuard.isPointCoveredByAnyRect(1200f, 120f, listOf(drawerOccluder)))
        assertFalse(TouchChainGuard.isPointCoveredByAnyRect(1236f, 38f, listOf(drawerOccluder)))
    }
}
