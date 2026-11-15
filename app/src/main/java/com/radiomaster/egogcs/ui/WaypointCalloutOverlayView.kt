package com.radiomaster.egogcs.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.graphics.Typeface
import android.view.View
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.RenderSnapshot
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * Zero-allocation 60 FPS screen-projected digital callout overlay for 3D mission waypoints.
 *
 * Projects every active 3D waypoint (`WP #1 .. WP #32`) through the live OpenGL `VP` matrix to
 * render crisp, anti-aliased HUD callout badges:
 * - Compact badges on unselected waypoints: `#1 · 1.5m` (with obstacle warning tint when unsafe).
 * - Expanded tactical telemetry card on the selected / actively-dragged waypoint:
 *   - Top `↕ 垂直拉高` handle cue (or live `↕ 调高中` / `↔ 移位中` status)
 *   - Primary readout: `#2 · 高 2.30m`
 *   - Local obstacle / clearance readout: `障顶 1.6m · 越障 +0.7m` or `净空 1.8m · 安全`
 * - Mission route summary strip on the terminal waypoint: `总 18.4m · 约 12s` (ETA).
 */
class WaypointCalloutOverlayView(
    context: Context,
    private val hub: GcsSharedHub,
    private val mapView: GlPointCloudMapView
) : View(context) {

    private val density = resources.displayMetrics.density
    private fun dp(v: Float): Float = v * density

    private val snapBuf = RenderSnapshot()
    private val vpBuf = FloatArray(16)
    private val projOut = FloatArray(3)
    private val baseProjOut = FloatArray(3)

    // Pre-allocated per-waypoint screen projection slots (zero per-frame heap allocation)
    private val slotValid = BooleanArray(RenderSnapshot.MAX_MISSION_WAYPOINTS)
    private val slotScreenX = FloatArray(RenderSnapshot.MAX_MISSION_WAYPOINTS)
    private val slotScreenY = FloatArray(RenderSnapshot.MAX_MISSION_WAYPOINTS)
    private val slotClipW = FloatArray(RenderSnapshot.MAX_MISSION_WAYPOINTS)
    private val drawnBadgeRects = Array(RenderSnapshot.MAX_MISSION_WAYPOINTS) { RectF() }
    private var drawnBadgeCount = 0

    // Zero-GC character scratch buffers
    private val line1Chars = CharArray(48)
    private val line2Chars = CharArray(48)
    private val etaChars = CharArray(40)
    private val handleChars = CharArray(24)

    private val cardRect = RectF()
    private val pillRect = RectF()
    private val etaRect = RectF()
    private val accentStripRect = RectF()
    private val pointerPath = Path()

    private val fillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }
    private val strokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = dp(1.15f)
    }
    private val leaderPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = dp(1.25f)
    }
    private val primaryTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        typeface = Typeface.create(Typeface.MONOSPACE, Typeface.BOLD)
        textSize = dp(10.0f)
    }
    private val subTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        typeface = Typeface.create(Typeface.MONOSPACE, Typeface.BOLD)
        textSize = dp(8.6f)
    }
    private val handleTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        typeface = Typeface.DEFAULT_BOLD
        textSize = dp(8.3f)
    }

    init {
        isClickable = false
        isFocusable = false
        importantForAccessibility = IMPORTANT_FOR_ACCESSIBILITY_NO
    }

    fun onMapFrame() {
        if (visibility == VISIBLE && !mapView.isGlSurfacePaused) {
            postInvalidateOnAnimation()
        }
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (mapView.isGlSurfacePaused) return
        val vw = width
        val vh = height
        if (vw <= 16 || vh <= 16) return

        val snap = hub.snapshots.readInto(snapBuf)
        val count = min(snap.waypointQueueCount, RenderSnapshot.MAX_MISSION_WAYPOINTS)
        if (count <= 0) return

        val projBlend = mapView.copyProjectionState(vpBuf)
        if (projBlend < 0f) return

        val zScale = 0.08f + 0.92f * projBlend
        val isStaged = snap.isRouteStagedOnly
        val selectedIdx = snap.selectedWaypointIdx
        val altDragIdx = mapView.activeAltitudeDragWpIndex
        val xyDragIdx = mapView.activeGroundDragWpIndex
        val pal = GcsTheme.of(hub.themeId)

        // Project all active 3D waypoint callout anchors
        for (i in 0 until count) {
            val b = i * 3
            val wx = snap.waypointQueuePts[b]
            val wy = snap.waypointQueuePts[b + 1]
            val rawZ = snap.waypointQueuePts[b + 2]
            val isSel = (i == selectedIdx || i == altDragIdx || i == xyDragIdx)
            val worldAnchorZ = rawZ * zScale + (if (isSel) 0.44f else 0.22f) * (0.28f + 0.72f * projBlend)
            val ok = MissionRouteVisualizer.projectWorldToScreen(vpBuf, vw, vh, wx, wy, worldAnchorZ, projOut)
            if (!ok ||
                projOut[0] < -dp(60f) || projOut[0] > vw + dp(60f) ||
                projOut[1] < -dp(40f) || projOut[1] > vh + dp(40f)
            ) {
                slotValid[i] = false
            } else {
                slotValid[i] = true
                slotScreenX[i] = projOut[0]
                slotScreenY[i] = projOut[1]
                slotClipW[i] = projOut[2]
            }
        }

        drawnBadgeCount = 0

        // 1. Draw unselected compact digital badges (front-to-back priority for clean decluttering)
        for (i in 0 until count) {
            if (!slotValid[i]) continue
            if (i == selectedIdx || i == altDragIdx || i == xyDragIdx) continue
            drawCompactWaypointBadge(
                canvas = canvas,
                index = i,
                totalCount = count,
                anchorX = slotScreenX[i],
                anchorY = slotScreenY[i],
                rawZ = snap.waypointQueuePts[i * 3 + 2],
                isSafe = snap.waypointSafeFlags[i],
                isStaged = isStaged,
                isLightTheme = pal.isLight,
                vw = vw,
                vh = vh
            )
        }

        // 2. Draw Mission ETA summary pill anchored below the terminal waypoint (#count) when staged/active
        val lastIdx = count - 1
        if (slotValid[lastIdx] && snap.stagedRouteArcLen > 0.08f) {
            val b = lastIdx * 3
            val baseOk = MissionRouteVisualizer.projectWorldToScreen(
                vpBuf,
                vw,
                vh,
                snap.waypointQueuePts[b],
                snap.waypointQueuePts[b + 1],
                0.02f,
                baseProjOut
            )
            if (baseOk) {
                val etaSec = MissionRouteVisualizer.computeMissionEtaSeconds(
                    arcLengthMeters = snap.stagedRouteArcLen,
                    waypointCount = count,
                    maxVelMetersPerSec = hub.maxVel
                )
                drawRouteEtaPill(
                    canvas = canvas,
                    anchorX = baseProjOut[0],
                    anchorY = baseProjOut[1] + dp(10f),
                    arcLenMeters = snap.stagedRouteArcLen,
                    etaSeconds = etaSec,
                    isStaged = isStaged,
                    isClosedLoop = snap.routeClosedLoop,
                    isLightTheme = pal.isLight,
                    vw = vw,
                    vh = vh
                )
            }
        }

        // 3. Draw selected / actively dragged waypoint expanded tactical digital callout on top
        val focusIdx = when {
            altDragIdx in 0 until count -> altDragIdx
            xyDragIdx in 0 until count -> xyDragIdx
            selectedIdx in 0 until count -> selectedIdx
            else -> -1
        }
        if (focusIdx >= 0 && slotValid[focusIdx]) {
            drawSelectedWaypointCallout(
                canvas = canvas,
                index = focusIdx,
                anchorX = slotScreenX[focusIdx],
                anchorY = slotScreenY[focusIdx],
                rawZ = snap.waypointQueuePts[focusIdx * 3 + 2],
                clearance = snap.waypointClearances[focusIdx],
                columnTopZ = snap.waypointColumnTopZs[focusIdx],
                isSafe = snap.waypointSafeFlags[focusIdx],
                isDraggingAlt = (focusIdx == altDragIdx),
                isDraggingXY = (focusIdx == xyDragIdx),
                projBlend = projBlend,
                isLightTheme = pal.isLight,
                vw = vw,
                vh = vh
            )
        }
    }

    private fun drawCompactWaypointBadge(
        canvas: Canvas,
        index: Int,
        totalCount: Int,
        anchorX: Float,
        anchorY: Float,
        rawZ: Float,
        isSafe: Boolean,
        isStaged: Boolean,
        isLightTheme: Boolean,
        vw: Int,
        vh: Int
    ) {
        var len = 0
        line1Chars[len++] = '#'
        len = appendPositiveInt(line1Chars, len, index + 1)
        line1Chars[len++] = ' '
        line1Chars[len++] = '·'
        line1Chars[len++] = ' '
        len = appendFixed1(line1Chars, len, rawZ)
        line1Chars[len++] = 'm'
        if (!isSafe) {
            line1Chars[len++] = ' '
            line1Chars[len++] = '!'
        }

        primaryTextPaint.textSize = dp(9.4f)
        val textW = primaryTextPaint.measureText(line1Chars, 0, len)
        val padH = dp(6.2f)
        val stripW = dp(3.2f)
        val boxW = stripW + padH * 2f + textW
        val boxH = dp(18.5f)

        val cx = anchorX.coerceIn(boxW * 0.5f + dp(6f), vw - boxW * 0.5f - dp(6f))
        val bottomY = (anchorY - dp(4f)).coerceIn(dp(42f) + boxH, vh - dp(10f))
        val topY = bottomY - boxH
        cardRect.set(cx - boxW * 0.5f, topY, cx + boxW * 0.5f, bottomY)

        // Suppress secondary badge if heavily overlapping an already-drawn closer badge (keep #1 & #N always visible)
        if (index != 0 && index != totalCount - 1) {
            for (k in 0 until drawnBadgeCount) {
                if (RectF.intersects(cardRect, drawnBadgeRects[k])) {
                    return
                }
            }
        }
        if (drawnBadgeCount < drawnBadgeRects.size) {
            drawnBadgeRects[drawnBadgeCount++].set(cardRect)
        }

        val accentColor = when {
            !isSafe -> Color.rgb(255, 82, 82)
            index == 0 && isStaged -> Color.rgb(16, 232, 148)
            isStaged -> Color.rgb(255, 196, 36)
            else -> Color.rgb(76, 202, 255)
        }

        fillPaint.color = if (!isSafe) {
            Color.argb(if (isLightTheme) 232 else 218, 68, 16, 20)
        } else if (isLightTheme) {
            Color.argb(226, 246, 250, 255)
        } else {
            Color.argb(208, 13, 21, 33)
        }
        val r = dp(5.2f)
        canvas.drawRoundRect(cardRect, r, r, fillPaint)

        // Left color-coded status accent bar
        accentStripRect.set(cardRect.left + dp(1.5f), cardRect.top + dp(2.5f), cardRect.left + dp(1.5f) + stripW, cardRect.bottom - dp(2.5f))
        fillPaint.color = accentColor
        canvas.drawRoundRect(accentStripRect, dp(1.6f), dp(1.6f), fillPaint)

        strokePaint.strokeWidth = dp(1.0f)
        strokePaint.color = if (!isSafe) {
            Color.rgb(255, 92, 92)
        } else {
            Color.argb(if (isLightTheme) 150 else 135, Color.red(accentColor), Color.green(accentColor), Color.blue(accentColor))
        }
        canvas.drawRoundRect(cardRect, r, r, strokePaint)

        // Leader tick to 3D beacon
        leaderPaint.color = strokePaint.color
        canvas.drawLine(cx, cardRect.bottom, anchorX, min(anchorY + dp(2f), cardRect.bottom + dp(8f)), leaderPaint)

        primaryTextPaint.color = when {
            !isSafe -> Color.rgb(255, 214, 214)
            isLightTheme -> Color.rgb(22, 32, 48)
            else -> Color.rgb(238, 246, 255)
        }
        val fm = primaryTextPaint.fontMetrics
        val baseline = cardRect.centerY() - (fm.ascent + fm.descent) * 0.5f
        canvas.drawText(line1Chars, 0, len, cardRect.left + stripW + padH, baseline, primaryTextPaint)
    }

    private fun drawSelectedWaypointCallout(
        canvas: Canvas,
        index: Int,
        anchorX: Float,
        anchorY: Float,
        rawZ: Float,
        clearance: Float,
        columnTopZ: Float,
        isSafe: Boolean,
        isDraggingAlt: Boolean,
        isDraggingXY: Boolean,
        projBlend: Float,
        isLightTheme: Boolean,
        vw: Int,
        vh: Int
    ) {
        // Line 1: "#2 · 高 2.30m"
        var l1 = 0
        line1Chars[l1++] = '#'
        l1 = appendPositiveInt(line1Chars, l1, index + 1)
        line1Chars[l1++] = ' '
        line1Chars[l1++] = '·'
        line1Chars[l1++] = ' '
        line1Chars[l1++] = '高'
        line1Chars[l1++] = ' '
        l1 = appendFixed2(line1Chars, l1, rawZ)
        line1Chars[l1++] = 'm'

        // Line 2: Obstacle top delta OR 3D clearance
        var l2 = 0
        val hasLocalColumn = columnTopZ > 0.14f
        val deltaAboveObs = rawZ - columnTopZ
        if (hasLocalColumn) {
            l2 = appendLiteral(line2Chars, l2, "障顶")
            l2 = appendFixed1(line2Chars, l2, columnTopZ)
            l2 = appendLiteral(line2Chars, l2, "m · ")
            if (deltaAboveObs >= 0f) {
                l2 = appendLiteral(line2Chars, l2, "越障+")
                l2 = appendFixed1(line2Chars, l2, deltaAboveObs)
                line2Chars[l2++] = 'm'
            } else {
                l2 = appendLiteral(line2Chars, l2, "低于障顶")
                l2 = appendFixed1(line2Chars, l2, -deltaAboveObs)
                line2Chars[l2++] = 'm'
            }
        } else {
            l2 = appendLiteral(line2Chars, l2, "净空 ")
            l2 = appendFixed1(line2Chars, l2, min(clearance, 9.9f))
            l2 = appendLiteral(line2Chars, l2, if (isSafe) "m · 安全" else "m · 贴近障碍!")
        }

        // Top handle cue pill text
        val hLen = when {
            isDraggingAlt -> appendLiteral(handleChars, 0, "↕ 正在无级调高")
            isDraggingXY -> appendLiteral(handleChars, 0, "↔ 正在平移航点")
            projBlend > 0.20f -> appendLiteral(handleChars, 0, "↕ 上下拉动调高")
            else -> appendLiteral(handleChars, 0, "↕ 拖拽此标牌调高")
        }

        primaryTextPaint.textSize = dp(10.4f)
        subTextPaint.textSize = dp(8.7f)
        val w1 = primaryTextPaint.measureText(line1Chars, 0, l1)
        val w2 = subTextPaint.measureText(line2Chars, 0, l2)
        val wH = handleTextPaint.measureText(handleChars, 0, hLen) + dp(12f)

        val padH = dp(8.5f)
        val cardW = max(max(w1, w2) + padH * 2f, wH + dp(10f))
        val cardH = dp(35.5f)
        val handlePillH = dp(14.5f)

        val cx = anchorX.coerceIn(cardW * 0.5f + dp(8f), vw - cardW * 0.5f - dp(8f))
        val cardBottom = (anchorY - dp(6f)).coerceIn(dp(44f) + cardH + handlePillH, vh - dp(12f))
        val cardTop = cardBottom - cardH
        cardRect.set(cx - cardW * 0.5f, cardTop, cx + cardW * 0.5f, cardBottom)

        val borderColor = when {
            !isSafe || (hasLocalColumn && deltaAboveObs < 0.18f) -> Color.rgb(255, 82, 82)
            isDraggingAlt -> Color.rgb(88, 246, 255)
            else -> Color.rgb(36, 224, 255)
        }

        // Crisp leader stem + pointer triangle down to the 3D vertical slider knob
        pointerPath.reset()
        pointerPath.moveTo(cx - dp(4.5f), cardBottom)
        pointerPath.lineTo(cx + dp(4.5f), cardBottom)
        pointerPath.lineTo(cx, cardBottom + dp(5.0f))
        pointerPath.close()

        fillPaint.color = if (isLightTheme) {
            Color.argb(242, 248, 252, 255)
        } else {
            Color.argb(234, 10, 19, 31)
        }
        val r = dp(6.5f)
        canvas.drawRoundRect(cardRect, r, r, fillPaint)
        canvas.drawPath(pointerPath, fillPaint)

        strokePaint.strokeWidth = dp(if (isDraggingAlt) 1.65f else 1.30f)
        strokePaint.color = borderColor
        canvas.drawRoundRect(cardRect, r, r, strokePaint)

        // Top vertical-slider grip tab ("↕ 上下拉动调高")
        val pillW = wH
        pillRect.set(
            cx - pillW * 0.5f,
            cardTop - handlePillH * 0.68f,
            cx + pillW * 0.5f,
            cardTop + handlePillH * 0.32f
        )
        fillPaint.color = if (isDraggingAlt) {
            Color.rgb(0, 188, 212)
        } else if (isLightTheme) {
            Color.rgb(222, 242, 252)
        } else {
            Color.rgb(18, 44, 66)
        }
        canvas.drawRoundRect(pillRect, dp(7f), dp(7f), fillPaint)
        strokePaint.strokeWidth = dp(1.0f)
        canvas.drawRoundRect(pillRect, dp(7f), dp(7f), strokePaint)

        handleTextPaint.color = if (isDraggingAlt) {
            Color.WHITE
        } else if (isLightTheme) {
            Color.rgb(0, 118, 156)
        } else {
            Color.rgb(128, 240, 255)
        }
        val hFm = handleTextPaint.fontMetrics
        val hBase = pillRect.centerY() - (hFm.ascent + hFm.descent) * 0.5f
        canvas.drawText(handleChars, 0, hLen, pillRect.left + dp(6f), hBase, handleTextPaint)

        // Line 1 text
        primaryTextPaint.color = if (isLightTheme) Color.rgb(14, 24, 38) else Color.WHITE
        val fm1 = primaryTextPaint.fontMetrics
        val line1Y = cardTop + dp(14.5f) - (fm1.ascent + fm1.descent) * 0.32f
        canvas.drawText(line1Chars, 0, l1, cardRect.left + padH, line1Y, primaryTextPaint)

        // Line 2 clearance / obstacle delta text
        subTextPaint.color = when {
            !isSafe || (hasLocalColumn && deltaAboveObs < 0.18f) -> Color.rgb(255, 94, 94)
            isLightTheme -> Color.rgb(0, 140, 94)
            else -> Color.rgb(72, 240, 172)
        }
        val fm2 = subTextPaint.fontMetrics
        val line2Y = cardTop + dp(27.5f) - (fm2.ascent + fm2.descent) * 0.32f
        canvas.drawText(line2Chars, 0, l2, cardRect.left + padH, line2Y, subTextPaint)
    }

    private fun drawRouteEtaPill(
        canvas: Canvas,
        anchorX: Float,
        anchorY: Float,
        arcLenMeters: Float,
        etaSeconds: Int,
        isStaged: Boolean,
        isClosedLoop: Boolean,
        isLightTheme: Boolean,
        vw: Int,
        vh: Int
    ) {
        var n = 0
        n = appendLiteral(etaChars, n, if (isStaged) "总 " else "余 ")
        n = appendFixed1(etaChars, n, arcLenMeters)
        n = appendLiteral(etaChars, n, "m · 约 ")
        if (etaSeconds >= 60) {
            n = appendPositiveInt(etaChars, n, etaSeconds / 60)
            etaChars[n++] = 'm'
            n = appendPositiveInt(etaChars, n, etaSeconds % 60)
            etaChars[n++] = 's'
        } else {
            n = appendPositiveInt(etaChars, n, etaSeconds)
            etaChars[n++] = 's'
        }
        if (isClosedLoop) {
            n = appendLiteral(etaChars, n, " ↻")
        }

        subTextPaint.textSize = dp(8.5f)
        val tw = subTextPaint.measureText(etaChars, 0, n)
        val padH = dp(6.5f)
        val bw = tw + padH * 2f
        val bh = dp(16.5f)

        val cx = anchorX.coerceIn(bw * 0.5f + dp(6f), vw - bw * 0.5f - dp(6f))
        val top = anchorY.coerceIn(dp(44f), vh - bh - dp(8f))
        etaRect.set(cx - bw * 0.5f, top, cx + bw * 0.5f, top + bh)

        fillPaint.color = if (isLightTheme) {
            Color.argb(214, 245, 249, 255)
        } else {
            Color.argb(198, 14, 23, 36)
        }
        val r = dp(8.2f)
        canvas.drawRoundRect(etaRect, r, r, fillPaint)

        strokePaint.strokeWidth = dp(0.95f)
        strokePaint.color = if (isStaged) {
            Color.argb(155, 255, 196, 36)
        } else {
            Color.argb(155, 76, 202, 255)
        }
        canvas.drawRoundRect(etaRect, r, r, strokePaint)

        subTextPaint.color = if (isStaged) {
            if (isLightTheme) Color.rgb(176, 102, 0) else Color.rgb(255, 218, 106)
        } else {
            if (isLightTheme) Color.rgb(0, 122, 184) else Color.rgb(122, 224, 255)
        }
        val fm = subTextPaint.fontMetrics
        val base = etaRect.centerY() - (fm.ascent + fm.descent) * 0.5f
        canvas.drawText(etaChars, 0, n, etaRect.left + padH, base, subTextPaint)
    }

    private fun appendLiteral(buf: CharArray, start: Int, literal: String): Int {
        var idx = start
        for (i in 0 until literal.length) {
            if (idx >= buf.size) break
            buf[idx++] = literal[i]
        }
        return idx
    }

    private fun appendPositiveInt(buf: CharArray, start: Int, value: Int): Int {
        var idx = start
        val v = max(0, value)
        if (v >= 100) {
            val h = (v / 100) % 10
            buf[idx++] = ('0'.code + h).toChar()
        }
        if (v >= 10) {
            val t = (v / 10) % 10
            buf[idx++] = ('0'.code + t).toChar()
        }
        buf[idx++] = ('0'.code + (v % 10)).toChar()
        return idx
    }

    private fun appendFixed1(buf: CharArray, start: Int, value: Float): Int {
        var idx = start
        val scaled = (abs(value) * 10f).roundToInt()
        idx = appendPositiveInt(buf, idx, scaled / 10)
        buf[idx++] = '.'
        buf[idx++] = ('0'.code + (scaled % 10)).toChar()
        return idx
    }

    private fun appendFixed2(buf: CharArray, start: Int, value: Float): Int {
        var idx = start
        val scaled = (abs(value) * 100f).roundToInt()
        idx = appendPositiveInt(buf, idx, scaled / 100)
        buf[idx++] = '.'
        val rem = scaled % 100
        buf[idx++] = ('0'.code + (rem / 10)).toChar()
        buf[idx++] = ('0'.code + (rem % 10)).toChar()
        return idx
    }
}
