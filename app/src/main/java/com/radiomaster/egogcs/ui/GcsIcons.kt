package com.radiomaster.egogcs.ui

import android.graphics.Canvas
import android.graphics.ColorFilter
import android.graphics.Paint
import android.graphics.Path
import android.graphics.PixelFormat
import android.graphics.Rect
import android.graphics.RectF
import android.graphics.drawable.Drawable
import android.text.SpannableStringBuilder
import android.text.Spanned
import android.text.style.ReplacementSpan
import android.widget.TextView
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sin

/**
 * 商业航空级纯矢量单色线性图标引擎 (24x24 标准几何视口 · 零表情符号 · 自动随文字状态变色)：
 * 基于 ReplacementSpan 将抗锯齿矢量图标与文字作为整体排版居中，无论按钮宽度如何均保持严丝合缝的工业对齐感。
 */
object GcsIcons {

    enum class Type {
        LINK,
        RADAR_SIGNAL,
        SLIDERS,
        TELEMETRY_PULSE,
        MENU_BARS,
        SETTINGS_GEAR,
        PALETTE_THEME,
        SUN_DAYLIGHT,
        MOON_NIGHT,
        SPARK_MOTION,
        SHIELD_SAFE,
        LAYOUT_GRID,
        SYSTEM_CHIP,
        CLOSE_X,
        BRAKE_HOLD,
        HOME_RTH,
        PLAY,
        PAUSE,
        STOP_SQUARE,
        CUBE_3D,
        DRONE_QUAD,
        SWARM_FLEET,
        TILT_VECTOR,
        COMPASS_AIM,
        XRAY_EYE,
        XRAY_OFF,
        LAYERS,
        GRID_2X2,
        SEARCH,
        REFRESH,
        CAPTURE_FRAME,
        EXPAND_WIDTH,
        COLLAPSE_WIDTH,
        COPY,
        DOWNLOAD,
        TRASH,
        ERASER,
        UNDO,
        SEND_FLY,
        PLUS_NODE,
        TARGET_PIN,
        ROUTE_NODES,
        LOOP_CYCLE,
        SWAP_ORDER,
        ARROW_UP,
        ARROW_DOWN,
        CHECK,
        WARNING,
        GAUGE_LOW,
        GAUGE_MID,
        GAUGE_HIGH,
        CHEVRON_LEFT,
        CHEVRON_RIGHT,
        CHEVRON_DOWN,
        CHEVRON_UP,
        STATUS_DOT,
        STATUS_RING
    }

    private data class IconBindingSpec(
        val leading: Type?,
        val label: String,
        val trailing: Type?,
        val iconSizePx: Int,
        val trailSizePx: Int,
        val gapPx: Int,
        val overrideIconColor: Int?
    )

    private const val TAG_ICON_SPEC = -0x24681357

    /**
     * 为 TextView 绑定商业矢量前置/后置图标与文本；若内容未变则零分配直接跳过，杜绝高频刷新抖动。
     * 当 overrideIconColor == null 时，矢量图标在绘制时自动继承 TextView 当前 textColor。
     */
    fun bind(
        tv: TextView,
        leading: Type?,
        label: String,
        density: Float,
        iconDp: Float = 12.5f,
        gapDp: Float = 4.5f,
        trailing: Type? = null,
        trailDp: Float = 10.5f,
        overrideIconColor: Int? = null
    ) {
        val iconPx = (iconDp * density).roundToInt().coerceAtLeast(10)
        val trailPx = (trailDp * density).roundToInt().coerceAtLeast(8)
        val gapPx = if (label.isEmpty()) 0 else (gapDp * density).roundToInt()

        val spec = IconBindingSpec(leading, label, trailing, iconPx, trailPx, gapPx, overrideIconColor)
        if (tv.getTag(TAG_ICON_SPEC) == spec) return
        tv.setTag(TAG_ICON_SPEC, spec)

        // 保留系统 CJK 字体上下安全内边距并关闭单行横向滚动裁剪，杜绝中文字头字脚与宽屏居中文字被切边
        tv.includeFontPadding = true
        tv.setHorizontallyScrolling(false)
        if (leading == null && trailing == null) {
            tv.text = label
            return
        }

        val ssb = SpannableStringBuilder()
        if (leading != null) {
            ssb.append(" ")
            ssb.setSpan(
                InlineVectorSpan(leading, iconPx, preGapPx = 0, postGapPx = gapPx, explicitColor = overrideIconColor),
                0,
                1,
                Spanned.SPAN_EXCLUSIVE_EXCLUSIVE
            )
        }
        if (label.isNotEmpty()) {
            ssb.append(label)
        }
        if (trailing != null) {
            val start = ssb.length
            ssb.append(" ")
            ssb.setSpan(
                InlineVectorSpan(trailing, trailPx, preGapPx = (3.5f * density).roundToInt(), postGapPx = 0, explicitColor = overrideIconColor),
                start,
                start + 1,
                Spanned.SPAN_EXCLUSIVE_EXCLUSIVE
            )
        }
        tv.text = ssb
    }

    fun drawable(type: Type, sizePx: Int, color: Int): Drawable =
        VectorGlyphDrawable(type, sizePx, color)

    private class InlineVectorSpan(
        type: Type,
        private val sizePx: Int,
        private val preGapPx: Int,
        private val postGapPx: Int,
        private val explicitColor: Int?
    ) : ReplacementSpan() {

        private val glyph = VectorGlyphDrawable(type, sizePx, explicitColor ?: -1).apply {
            setBounds(0, 0, sizePx, sizePx)
        }

        override fun getSize(
            paint: Paint,
            text: CharSequence?,
            start: Int,
            end: Int,
            fm: Paint.FontMetricsInt?
        ): Int {
            if (fm != null) {
                val pfm = paint.fontMetricsInt
                val fullH = pfm.bottom - pfm.top
                if (sizePx + 2 > fullH) {
                    val extra = (sizePx + 2 - fullH + 1) / 2
                    fm.top = pfm.top - extra
                    fm.ascent = minOf(pfm.ascent, fm.top + 1)
                    fm.bottom = pfm.bottom + extra
                    fm.descent = maxOf(pfm.descent, fm.bottom - 1)
                } else {
                    fm.ascent = pfm.ascent
                    fm.top = pfm.top
                    fm.descent = pfm.descent
                    fm.bottom = pfm.bottom
                }
            }
            return preGapPx + sizePx + postGapPx
        }

        override fun draw(
            canvas: Canvas,
            text: CharSequence?,
            start: Int,
            end: Int,
            x: Float,
            top: Int,
            y: Int,
            bottom: Int,
            paint: Paint
        ) {
            glyph.updateBaseColor(explicitColor ?: paint.color)
            val fm = paint.fontMetrics
            val textCenterY = y + (fm.ascent + fm.descent) * 0.5f
            val maxTop = (bottom - sizePx).coerceAtLeast(top).toFloat()
            val drawTop = (textCenterY - sizePx * 0.5f).coerceIn(top.toFloat(), maxTop)

            canvas.save()
            canvas.translate(x + preGapPx, drawTop)
            glyph.draw(canvas)
            canvas.restore()
        }
    }

    private class VectorGlyphDrawable(
        private val type: Type,
        private val sizePx: Int,
        initColor: Int
    ) : Drawable() {

        private val strokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeCap = Paint.Cap.ROUND
            strokeJoin = Paint.Join.ROUND
            color = initColor
        }

        private val fillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.FILL
            color = initColor
        }

        private val path = Path()
        private val arcRect = RectF()

        fun updateBaseColor(c: Int) {
            if (strokePaint.color != c) {
                strokePaint.color = c
                fillPaint.color = c
            }
        }

        override fun getIntrinsicWidth(): Int = sizePx
        override fun getIntrinsicHeight(): Int = sizePx

        override fun setAlpha(alpha: Int) {
            strokePaint.alpha = alpha
            fillPaint.alpha = alpha
        }

        override fun setColorFilter(colorFilter: ColorFilter?) {
            strokePaint.colorFilter = colorFilter
            fillPaint.colorFilter = colorFilter
        }

        @Deprecated("Deprecated in Java")
        override fun getOpacity(): Int = PixelFormat.TRANSLUCENT

        override fun draw(canvas: Canvas) {
            val b: Rect = bounds
            val w = (if (b.isEmpty) sizePx else b.width()).toFloat()
            val h = (if (b.isEmpty) sizePx else b.height()).toFloat()
            val s = min(w, h) / 24f

            canvas.save()
            canvas.translate(b.left + (w - 24f * s) * 0.5f, b.top + (h - 24f * s) * 0.5f)
            canvas.scale(s, s)

            strokePaint.strokeWidth = 2.05f
            path.reset()

            when (type) {
                Type.LINK -> {
                    canvas.save()
                    canvas.rotate(-45f, 12f, 12f)
                    arcRect.set(3.5f, 8.2f, 13.5f, 15.8f)
                    canvas.drawRoundRect(arcRect, 3.8f, 3.8f, strokePaint)
                    arcRect.set(10.5f, 8.2f, 20.5f, 15.8f)
                    canvas.drawRoundRect(arcRect, 3.8f, 3.8f, strokePaint)
                    canvas.restore()
                }

                Type.RADAR_SIGNAL -> {
                    canvas.drawCircle(12f, 13.5f, 2.0f, fillPaint)
                    arcRect.set(7.2f, 8.7f, 16.8f, 18.3f)
                    canvas.drawArc(arcRect, 215f, 110f, false, strokePaint)
                    arcRect.set(3.6f, 5.1f, 20.4f, 21.9f)
                    canvas.drawArc(arcRect, 215f, 110f, false, strokePaint)
                    canvas.drawLine(12f, 15.5f, 12f, 20.2f, strokePaint)
                }

                Type.SLIDERS -> {
                    canvas.drawLine(4f, 6.5f, 20f, 6.5f, strokePaint)
                    canvas.drawCircle(9.5f, 6.5f, 2.2f, fillPaint)
                    canvas.drawLine(4f, 12f, 20f, 12f, strokePaint)
                    canvas.drawCircle(15.5f, 12f, 2.2f, fillPaint)
                    canvas.drawLine(4f, 17.5f, 20f, 17.5f, strokePaint)
                    canvas.drawCircle(8.0f, 17.5f, 2.2f, fillPaint)
                }

                Type.TELEMETRY_PULSE -> {
                    path.moveTo(2.8f, 12.5f)
                    path.lineTo(7.0f, 12.5f)
                    path.lineTo(9.6f, 6.0f)
                    path.lineTo(13.8f, 18.5f)
                    path.lineTo(16.6f, 12.5f)
                    path.lineTo(21.2f, 12.5f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.MENU_BARS -> {
                    canvas.drawLine(4f, 6.5f, 20f, 6.5f, strokePaint)
                    canvas.drawLine(4f, 12f, 20f, 12f, strokePaint)
                    canvas.drawLine(4f, 17.5f, 20f, 17.5f, strokePaint)
                }

                Type.SETTINGS_GEAR -> {
                    canvas.drawCircle(12f, 12f, 3.3f, strokePaint)
                    canvas.drawCircle(12f, 12f, 6.8f, strokePaint)
                    for (i in 0 until 8) {
                        val rad = Math.toRadians(i * 45.0)
                        val cx = cos(rad).toFloat()
                        val sy = sin(rad).toFloat()
                        canvas.drawLine(
                            12f + cx * 6.9f,
                            12f + sy * 6.9f,
                            12f + cx * 9.6f,
                            12f + sy * 9.6f,
                            strokePaint
                        )
                    }
                }

                Type.PALETTE_THEME -> {
                    canvas.drawCircle(12f, 9.2f, 5.2f, strokePaint)
                    canvas.drawCircle(9.0f, 14.5f, 5.2f, strokePaint)
                    canvas.drawCircle(15.0f, 14.5f, 5.2f, strokePaint)
                }

                Type.SUN_DAYLIGHT -> {
                    canvas.drawCircle(12f, 12f, 4.2f, strokePaint)
                    for (i in 0 until 8) {
                        val rad = Math.toRadians(i * 45.0)
                        val cx = cos(rad).toFloat()
                        val sy = sin(rad).toFloat()
                        canvas.drawLine(12f + cx * 6.6f, 12f + sy * 6.6f, 12f + cx * 9.5f, 12f + sy * 9.5f, strokePaint)
                    }
                }

                Type.MOON_NIGHT -> {
                    arcRect.set(4.5f, 4.5f, 19.5f, 19.5f)
                    canvas.drawArc(arcRect, 35f, 250f, false, strokePaint)
                    arcRect.set(8.5f, 3.2f, 21.5f, 16.2f)
                    canvas.drawArc(arcRect, 98f, 145f, false, strokePaint)
                }

                Type.SPARK_MOTION -> {
                    path.moveTo(3.5f, 8.5f)
                    path.lineTo(13.5f, 8.5f)
                    path.moveTo(6.5f, 12.0f)
                    path.lineTo(20.5f, 12.0f)
                    path.moveTo(4.5f, 15.5f)
                    path.lineTo(15.5f, 15.5f)
                    path.moveTo(16.8f, 8.6f)
                    path.lineTo(20.5f, 12.0f)
                    path.lineTo(16.8f, 15.4f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.SHIELD_SAFE -> {
                    path.moveTo(12f, 3.2f)
                    path.lineTo(19.2f, 6.2f)
                    path.lineTo(19.2f, 11.8f)
                    path.quadTo(19.2f, 17.6f, 12f, 20.8f)
                    path.quadTo(4.8f, 17.6f, 4.8f, 11.8f)
                    path.lineTo(4.8f, 6.2f)
                    path.close()
                    canvas.drawPath(path, strokePaint)

                    path.reset()
                    path.moveTo(8.8f, 12.0f)
                    path.lineTo(11.1f, 14.3f)
                    path.lineTo(15.5f, 9.6f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.LAYOUT_GRID -> {
                    arcRect.set(3.8f, 3.8f, 10.6f, 10.6f)
                    canvas.drawRoundRect(arcRect, 1.6f, 1.6f, strokePaint)
                    arcRect.set(13.4f, 3.8f, 20.2f, 10.6f)
                    canvas.drawRoundRect(arcRect, 1.6f, 1.6f, strokePaint)
                    arcRect.set(3.8f, 13.4f, 10.6f, 20.2f)
                    canvas.drawRoundRect(arcRect, 1.6f, 1.6f, strokePaint)
                    arcRect.set(13.4f, 13.4f, 20.2f, 20.2f)
                    canvas.drawRoundRect(arcRect, 1.6f, 1.6f, strokePaint)
                }

                Type.SYSTEM_CHIP -> {
                    arcRect.set(6.5f, 6.5f, 17.5f, 17.5f)
                    canvas.drawRoundRect(arcRect, 2.0f, 2.0f, strokePaint)
                    arcRect.set(9.8f, 9.8f, 14.2f, 14.2f)
                    canvas.drawRoundRect(arcRect, 0.8f, 0.8f, fillPaint)
                    canvas.drawLine(9.5f, 3.2f, 9.5f, 6.5f, strokePaint)
                    canvas.drawLine(14.5f, 3.2f, 14.5f, 6.5f, strokePaint)
                    canvas.drawLine(9.5f, 17.5f, 9.5f, 20.8f, strokePaint)
                    canvas.drawLine(14.5f, 17.5f, 14.5f, 20.8f, strokePaint)
                    canvas.drawLine(3.2f, 9.5f, 6.5f, 9.5f, strokePaint)
                    canvas.drawLine(3.2f, 14.5f, 6.5f, 14.5f, strokePaint)
                    canvas.drawLine(17.5f, 9.5f, 20.8f, 9.5f, strokePaint)
                    canvas.drawLine(17.5f, 14.5f, 20.8f, 14.5f, strokePaint)
                }

                Type.CLOSE_X -> {
                    canvas.drawLine(6f, 6f, 18f, 18f, strokePaint)
                    canvas.drawLine(18f, 6f, 6f, 18f, strokePaint)
                }

                Type.BRAKE_HOLD -> {
                    canvas.drawCircle(12f, 12f, 8.8f, strokePaint)
                    canvas.drawLine(9.7f, 8.6f, 9.7f, 15.4f, strokePaint)
                    canvas.drawLine(14.3f, 8.6f, 14.3f, 15.4f, strokePaint)
                }

                Type.HOME_RTH -> {
                    arcRect.set(4.2f, 14.2f, 19.8f, 20.2f)
                    canvas.drawArc(arcRect, 0f, 360f, false, strokePaint)
                    canvas.drawLine(12f, 3.8f, 12f, 13.2f, strokePaint)
                    path.moveTo(8.5f, 9.8f)
                    path.lineTo(12f, 13.5f)
                    path.lineTo(15.5f, 9.8f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.PLAY -> {
                    path.moveTo(7.2f, 5.2f)
                    path.lineTo(18.5f, 12f)
                    path.lineTo(7.2f, 18.8f)
                    path.close()
                    canvas.drawPath(path, fillPaint)
                }

                Type.PAUSE -> {
                    arcRect.set(6.5f, 5.5f, 10.2f, 18.5f)
                    canvas.drawRoundRect(arcRect, 1.2f, 1.2f, fillPaint)
                    arcRect.set(13.8f, 5.5f, 17.5f, 18.5f)
                    canvas.drawRoundRect(arcRect, 1.2f, 1.2f, fillPaint)
                }

                Type.STOP_SQUARE -> {
                    arcRect.set(6.2f, 6.2f, 17.8f, 17.8f)
                    canvas.drawRoundRect(arcRect, 2.0f, 2.0f, fillPaint)
                }

                Type.CUBE_3D -> {
                    path.moveTo(12f, 3.2f)
                    path.lineTo(19.8f, 7.6f)
                    path.lineTo(19.8f, 16.4f)
                    path.lineTo(12f, 20.8f)
                    path.lineTo(4.2f, 16.4f)
                    path.lineTo(4.2f, 7.6f)
                    path.close()
                    canvas.drawPath(path, strokePaint)
                    canvas.drawLine(4.2f, 7.6f, 12f, 12f, strokePaint)
                    canvas.drawLine(19.8f, 7.6f, 12f, 12f, strokePaint)
                    canvas.drawLine(12f, 12f, 12f, 20.8f, strokePaint)
                }

                Type.DRONE_QUAD -> {
                    canvas.drawLine(6.6f, 6.6f, 17.4f, 17.4f, strokePaint)
                    canvas.drawLine(17.4f, 6.6f, 6.6f, 17.4f, strokePaint)
                    canvas.drawCircle(5.6f, 5.6f, 2.75f, strokePaint)
                    canvas.drawCircle(18.4f, 5.6f, 2.75f, strokePaint)
                    canvas.drawCircle(5.6f, 18.4f, 2.75f, strokePaint)
                    canvas.drawCircle(18.4f, 18.4f, 2.75f, strokePaint)
                    arcRect.set(9.7f, 9.5f, 14.3f, 14.5f)
                    canvas.drawRoundRect(arcRect, 1.3f, 1.3f, fillPaint)
                    path.reset()
                    path.moveTo(12f, 5.2f)
                    path.lineTo(13.8f, 8.3f)
                    path.lineTo(10.2f, 8.3f)
                    path.close()
                    canvas.drawPath(path, fillPaint)
                }

                Type.COMPASS_AIM -> {
                    canvas.drawCircle(12f, 12f, 7.5f, strokePaint)
                    canvas.drawCircle(12f, 12f, 2.0f, fillPaint)
                    canvas.drawLine(12f, 2.5f, 12f, 5.2f, strokePaint)
                    canvas.drawLine(12f, 18.8f, 12f, 21.5f, strokePaint)
                    canvas.drawLine(2.5f, 12f, 5.2f, 12f, strokePaint)
                    canvas.drawLine(18.8f, 12f, 21.5f, 12f, strokePaint)
                }

                Type.XRAY_EYE, Type.XRAY_OFF -> {
                    path.moveTo(2.5f, 12f)
                    path.quadTo(12f, 4.2f, 21.5f, 12f)
                    path.quadTo(12f, 19.8f, 2.5f, 12f)
                    canvas.drawPath(path, strokePaint)
                    canvas.drawCircle(12f, 12f, 2.6f, strokePaint)
                    if (type == Type.XRAY_OFF) {
                        canvas.drawLine(4.5f, 19.5f, 19.5f, 4.5f, strokePaint)
                    }
                }

                Type.LAYERS -> {
                    path.moveTo(12f, 4.0f)
                    path.lineTo(20.5f, 8.5f)
                    path.lineTo(12f, 13.0f)
                    path.lineTo(3.5f, 8.5f)
                    path.close()
                    canvas.drawPath(path, strokePaint)

                    path.reset()
                    path.moveTo(3.5f, 13.0f)
                    path.lineTo(12f, 17.4f)
                    path.lineTo(20.5f, 13.0f)
                    canvas.drawPath(path, strokePaint)

                    path.reset()
                    path.moveTo(3.5f, 16.8f)
                    path.lineTo(12f, 21.0f)
                    path.lineTo(20.5f, 16.8f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.GRID_2X2 -> {
                    arcRect.set(4.0f, 4.0f, 20.0f, 20.0f)
                    canvas.drawRoundRect(arcRect, 2.0f, 2.0f, strokePaint)
                    canvas.drawLine(12f, 4.0f, 12f, 20.0f, strokePaint)
                    canvas.drawLine(4.0f, 12f, 20.0f, 12f, strokePaint)
                }

                Type.SEARCH -> {
                    canvas.drawCircle(10.5f, 10.5f, 6.2f, strokePaint)
                    canvas.drawLine(15.2f, 15.2f, 20.2f, 20.2f, strokePaint)
                }

                Type.REFRESH -> {
                    arcRect.set(4.5f, 4.5f, 19.5f, 19.5f)
                    canvas.drawArc(arcRect, -35f, 285f, false, strokePaint)
                    path.moveTo(15.2f, 4.2f)
                    path.lineTo(19.5f, 5.2f)
                    path.lineTo(18.2f, 9.5f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.CAPTURE_FRAME -> {
                    path.moveTo(4f, 8.5f)
                    path.lineTo(4f, 4.5f)
                    path.lineTo(8f, 4.5f)
                    path.moveTo(16f, 4.5f)
                    path.lineTo(20f, 4.5f)
                    path.lineTo(20f, 8.5f)
                    path.moveTo(20f, 15.5f)
                    path.lineTo(20f, 19.5f)
                    path.lineTo(16f, 19.5f)
                    path.moveTo(8f, 19.5f)
                    path.lineTo(4f, 19.5f)
                    path.lineTo(4f, 15.5f)
                    canvas.drawPath(path, strokePaint)
                    canvas.drawCircle(12f, 12f, 2.5f, fillPaint)
                }

                Type.EXPAND_WIDTH -> {
                    canvas.drawLine(4f, 12f, 20f, 12f, strokePaint)
                    path.moveTo(8f, 8f)
                    path.lineTo(4f, 12f)
                    path.lineTo(8f, 16f)
                    path.moveTo(16f, 8f)
                    path.lineTo(20f, 12f)
                    path.lineTo(16f, 16f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.COLLAPSE_WIDTH -> {
                    canvas.drawLine(12f, 5f, 12f, 19f, strokePaint)
                    path.moveTo(5.5f, 8.5f)
                    path.lineTo(9.2f, 12f)
                    path.lineTo(5.5f, 15.5f)
                    path.moveTo(18.5f, 8.5f)
                    path.lineTo(14.8f, 12f)
                    path.lineTo(18.5f, 15.5f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.COPY -> {
                    arcRect.set(8.5f, 8.0f, 19.5f, 20.0f)
                    canvas.drawRoundRect(arcRect, 2.0f, 2.0f, strokePaint)
                    path.moveTo(15.5f, 8.0f)
                    path.lineTo(15.5f, 5.2f)
                    path.lineTo(5.0f, 5.2f)
                    path.lineTo(5.0f, 16.5f)
                    path.lineTo(8.5f, 16.5f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.DOWNLOAD -> {
                    canvas.drawLine(12f, 4.2f, 12f, 15.0f, strokePaint)
                    path.moveTo(7.8f, 11.2f)
                    path.lineTo(12f, 15.5f)
                    path.lineTo(16.2f, 11.2f)
                    path.moveTo(4.5f, 16.5f)
                    path.lineTo(4.5f, 19.8f)
                    path.lineTo(19.5f, 19.8f)
                    path.lineTo(19.5f, 16.5f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.TRASH -> {
                    canvas.drawLine(4.5f, 6.8f, 19.5f, 6.8f, strokePaint)
                    canvas.drawLine(9.5f, 4.2f, 14.5f, 4.2f, strokePaint)
                    arcRect.set(6.8f, 6.8f, 17.2f, 20.0f)
                    canvas.drawRoundRect(arcRect, 1.8f, 1.8f, strokePaint)
                    canvas.drawLine(10.3f, 10.5f, 10.3f, 16.5f, strokePaint)
                    canvas.drawLine(13.7f, 10.5f, 13.7f, 16.5f, strokePaint)
                }

                Type.ERASER -> {
                    canvas.save()
                    canvas.rotate(-35f, 12f, 12f)
                    arcRect.set(5.5f, 8.0f, 18.5f, 16.0f)
                    canvas.drawRoundRect(arcRect, 2.0f, 2.0f, strokePaint)
                    canvas.drawLine(11.0f, 8.0f, 11.0f, 16.0f, strokePaint)
                    canvas.restore()
                    canvas.drawLine(6f, 19.8f, 19f, 19.8f, strokePaint)
                }

                Type.UNDO -> {
                    arcRect.set(5.2f, 6.2f, 19.2f, 19.5f)
                    canvas.drawArc(arcRect, -95f, 235f, false, strokePaint)
                    path.moveTo(9.2f, 3.8f)
                    path.lineTo(5.2f, 6.5f)
                    path.lineTo(8.8f, 9.8f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.SEND_FLY -> {
                    path.moveTo(20.2f, 4.2f)
                    path.lineTo(10.8f, 20.2f)
                    path.lineTo(9.2f, 13.2f)
                    path.lineTo(3.2f, 10.8f)
                    path.close()
                    canvas.drawPath(path, strokePaint)
                }

                Type.PLUS_NODE -> {
                    canvas.drawCircle(12f, 12f, 8.5f, strokePaint)
                    canvas.drawLine(12f, 8.0f, 12f, 16.0f, strokePaint)
                    canvas.drawLine(8.0f, 12f, 16.0f, 12f, strokePaint)
                }

                Type.TARGET_PIN -> {
                    canvas.drawCircle(12f, 9.5f, 5.2f, strokePaint)
                    canvas.drawCircle(12f, 9.5f, 1.8f, fillPaint)
                    canvas.drawLine(12f, 14.7f, 12f, 20.5f, strokePaint)
                    canvas.drawLine(8.2f, 20.5f, 15.8f, 20.5f, strokePaint)
                }

                Type.ROUTE_NODES -> {
                    canvas.drawCircle(6.0f, 17.5f, 2.3f, strokePaint)
                    canvas.drawCircle(12.0f, 7.0f, 2.3f, strokePaint)
                    canvas.drawCircle(18.2f, 15.5f, 2.3f, fillPaint)
                    canvas.drawLine(7.2f, 15.5f, 10.8f, 9.0f, strokePaint)
                    canvas.drawLine(13.4f, 8.9f, 16.8f, 13.6f, strokePaint)
                }

                Type.LOOP_CYCLE -> {
                    path.moveTo(17.5f, 7.5f)
                    path.lineTo(6.5f, 7.5f)
                    path.lineTo(6.5f, 12.2f)
                    path.moveTo(14.5f, 4.8f)
                    path.lineTo(17.8f, 7.5f)
                    path.lineTo(14.5f, 10.2f)

                    path.moveTo(6.5f, 16.5f)
                    path.lineTo(17.5f, 16.5f)
                    path.lineTo(17.5f, 11.8f)
                    path.moveTo(9.5f, 13.8f)
                    path.lineTo(6.2f, 16.5f)
                    path.lineTo(9.5f, 19.2f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.SWAP_ORDER -> {
                    canvas.drawLine(8.0f, 4.8f, 8.0f, 19.2f, strokePaint)
                    path.moveTo(4.8f, 8.2f)
                    path.lineTo(8.0f, 4.8f)
                    path.lineTo(11.2f, 8.2f)

                    canvas.drawLine(16.0f, 4.8f, 16.0f, 19.2f, strokePaint)
                    path.moveTo(12.8f, 15.8f)
                    path.lineTo(16.0f, 19.2f)
                    path.lineTo(19.2f, 15.8f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.ARROW_UP -> {
                    canvas.drawLine(12f, 5.2f, 12f, 19.0f, strokePaint)
                    path.moveTo(7.2f, 10.0f)
                    path.lineTo(12f, 5.0f)
                    path.lineTo(16.8f, 10.0f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.ARROW_DOWN -> {
                    canvas.drawLine(12f, 5.0f, 12f, 18.8f, strokePaint)
                    path.moveTo(7.2f, 14.0f)
                    path.lineTo(12f, 19.0f)
                    path.lineTo(16.8f, 14.0f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.CHECK -> {
                    path.moveTo(4.8f, 12.5f)
                    path.lineTo(9.8f, 17.5f)
                    path.lineTo(19.4f, 6.8f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.WARNING -> {
                    path.moveTo(12f, 3.8f)
                    path.lineTo(20.8f, 19.5f)
                    path.lineTo(3.2f, 19.5f)
                    path.close()
                    canvas.drawPath(path, strokePaint)
                    canvas.drawLine(12f, 9.5f, 12f, 14.2f, strokePaint)
                    canvas.drawCircle(12f, 17.0f, 1.15f, fillPaint)
                }

                Type.GAUGE_LOW, Type.GAUGE_MID, Type.GAUGE_HIGH -> {
                    arcRect.set(3.8f, 5.5f, 20.2f, 21.9f)
                    canvas.drawArc(arcRect, 180f, 180f, false, strokePaint)
                    val deg = when (type) {
                        Type.GAUGE_LOW -> 215.0
                        Type.GAUGE_MID -> 270.0
                        else -> 328.0
                    }
                    val rad = Math.toRadians(deg)
                    val nx = 12f + cos(rad).toFloat() * 5.8f
                    val ny = 13.7f + sin(rad).toFloat() * 5.8f
                    canvas.drawLine(12f, 13.7f, nx, ny, strokePaint)
                    canvas.drawCircle(12f, 13.7f, 1.8f, fillPaint)
                }

                Type.CHEVRON_LEFT -> {
                    path.moveTo(15.2f, 5.5f)
                    path.lineTo(8.5f, 12f)
                    path.lineTo(15.2f, 18.5f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.CHEVRON_RIGHT -> {
                    path.moveTo(8.8f, 5.5f)
                    path.lineTo(15.5f, 12f)
                    path.lineTo(8.8f, 18.5f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.CHEVRON_DOWN -> {
                    path.moveTo(6.0f, 9.2f)
                    path.lineTo(12f, 15.2f)
                    path.lineTo(18.0f, 9.2f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.CHEVRON_UP -> {
                    path.moveTo(6.0f, 14.8f)
                    path.lineTo(12f, 8.8f)
                    path.lineTo(18.0f, 14.8f)
                    canvas.drawPath(path, strokePaint)
                }

                Type.STATUS_DOT -> {
                    val baseAlpha = strokePaint.alpha
                    canvas.drawCircle(12f, 12f, 4.6f, fillPaint)
                    strokePaint.alpha = (baseAlpha * 0.42f).roundToInt()
                    canvas.drawCircle(12f, 12f, 8.2f, strokePaint)
                    strokePaint.alpha = baseAlpha
                }

                Type.STATUS_RING -> {
                    canvas.drawCircle(12f, 12f, 6.2f, strokePaint)
                }

                Type.SWARM_FLEET -> {
                    // 三机三角楔形编队纯几何矢量图标 (领航机 + 左右僚机 + 战术协同连线)
                    val baseAlpha = strokePaint.alpha
                    strokePaint.alpha = (baseAlpha * 0.48f).roundToInt()
                    canvas.drawLine(12f, 6.2f, 6.0f, 16.5f, strokePaint)
                    canvas.drawLine(12f, 6.2f, 18.0f, 16.5f, strokePaint)
                    canvas.drawLine(6.0f, 16.5f, 18.0f, 16.5f, strokePaint)
                    strokePaint.alpha = baseAlpha

                    path.reset()
                    path.moveTo(12f, 3.3f)
                    path.lineTo(14.8f, 8.2f)
                    path.lineTo(12f, 7.0f)
                    path.lineTo(9.2f, 8.2f)
                    path.close()
                    canvas.drawPath(path, fillPaint)

                    canvas.drawCircle(5.6f, 16.8f, 2.35f, strokePaint)
                    canvas.drawCircle(18.4f, 16.8f, 2.35f, strokePaint)
                }

                Type.TILT_VECTOR -> {
                    // 倾转旋翼短舱 + 矢量推力前倾箭头纯几何线性图标
                    val baseAlpha = strokePaint.alpha
                    strokePaint.alpha = (baseAlpha * 0.45f).roundToInt()
                    canvas.drawLine(4.2f, 17.5f, 19.8f, 17.5f, strokePaint)
                    strokePaint.alpha = baseAlpha

                    // 倾转舵机铰链圆心与倾斜桨盘椭圆截面
                    canvas.drawCircle(9.5f, 14.8f, 2.1f, strokePaint)
                    canvas.drawLine(4.8f, 12.2f, 14.2f, 17.4f, strokePaint)

                    // 沿倾转方向的矢量推力主轴与三角箭头
                    canvas.drawLine(9.5f, 14.8f, 17.6f, 5.4f, strokePaint)
                    path.reset()
                    path.moveTo(18.6f, 4.2f)
                    path.lineTo(13.8f, 5.8f)
                    path.lineTo(17.2f, 8.8f)
                    path.close()
                    canvas.drawPath(path, fillPaint)
                }
            }

            canvas.restore()
        }
    }
}
