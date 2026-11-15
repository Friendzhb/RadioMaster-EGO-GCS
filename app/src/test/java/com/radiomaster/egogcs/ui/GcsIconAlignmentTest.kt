package com.radiomaster.egogcs.ui

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class GcsIconAlignmentTest {

    @Test
    fun `computeViewportTransform centers glyph sub-pixel inside non-square settings button`() {
        // 28x26dp @ 2x density -> 56x52px button bounds with 13.5dp -> 27px gear glyph
        val tf = GcsIcons.computeViewportTransform(
            boundsLeft = 0,
            boundsTop = 0,
            boundsRight = 56,
            boundsBottom = 52,
            glyphSizePx = 27
        )
        val renderedSide = 24f * tf[2]
        assertEquals(27f, renderedSide, 1e-4f)

        val leftMargin = tf[0] - 0f
        val rightMargin = 56f - (tf[0] + renderedSide)
        val topMargin = tf[1] - 0f
        val bottomMargin = 52f - (tf[1] + renderedSide)

        assertEquals(14.5f, leftMargin, 1e-4f)
        assertEquals(leftMargin, rightMargin, 1e-4f)
        assertEquals(12.5f, topMargin, 1e-4f)
        assertEquals(topMargin, bottomMargin, 1e-4f)
    }

    @Test
    fun `computeViewportTransform clamps gracefully when host bounds smaller than glyph`() {
        val tf = GcsIcons.computeViewportTransform(
            boundsLeft = 4,
            boundsTop = 6,
            boundsRight = 24,
            boundsBottom = 22,
            glyphSizePx = 28
        )
        // Host is 20x16px, so renderedSide clamps to min(20, 16, 28) = 16px
        val renderedSide = 24f * tf[2]
        assertEquals(16f, renderedSide, 1e-4f)
        assertEquals(6f, tf[0], 1e-4f) // 4 + (20 - 16) * 0.5
        assertEquals(6f, tf[1], 1e-4f) // 6 + (16 - 16) * 0.5
    }

    @Test
    fun `computeViewportTransform falls back to glyphSizePx when bounds are unmeasured`() {
        val tf = GcsIcons.computeViewportTransform(
            boundsLeft = 0,
            boundsTop = 0,
            boundsRight = 0,
            boundsBottom = 0,
            glyphSizePx = 24
        )
        assertEquals(0f, tf[0], 1e-4f)
        assertEquals(0f, tf[1], 1e-4f)
        assertEquals(1f, tf[2], 1e-4f)
    }

    @Test
    fun `computeInlineDrawTop centers icon on optical font midline without top-clamping bias`() {
        // Compact 9.6dp label @ 2x density: ascent = -18.2f, descent = 4.8f, iconSizePx = 25px (larger than 23px font box)
        val baselineY = 30
        val ascent = -18.2f
        val descent = 4.8f
        val iconSizePx = 25

        val drawTop = GcsIcons.computeInlineDrawTop(baselineY, ascent, descent, iconSizePx)
        val iconCenterY = drawTop + iconSizePx * 0.5f
        val expectedTextMidlineY = baselineY + (ascent + descent) * 0.5f

        assertEquals(expectedTextMidlineY, iconCenterY, 1e-4f)
        // Verify it is NOT clamped down to baselineY + ascent (11.8f), which previously shifted oversized icons downward
        assertTrue(drawTop < baselineY + ascent)
    }
}
