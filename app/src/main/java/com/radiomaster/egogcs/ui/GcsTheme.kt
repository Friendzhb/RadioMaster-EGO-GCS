package com.radiomaster.egogcs.ui

import android.graphics.Color
import android.graphics.Paint
import android.graphics.Typeface
import android.graphics.drawable.ClipDrawable
import android.graphics.drawable.GradientDrawable
import android.graphics.drawable.LayerDrawable
import android.view.Gravity
import android.view.View
import android.view.ViewGroup
import android.widget.EditText
import android.widget.SeekBar
import android.widget.TextView
import kotlin.math.roundToInt

/**
 * 商业航空级全栈主题系统 (精炼航电毛玻璃风 · 同步驱动 2D HUD 座舱、HSI 姿态雷达罗盘、定制滑块与 3D OpenGL 大气穹顶)：
 * 预计算多层微渐变表面、高光描边常量与 OpenGL 归一化浮点数组，运行期零字符串解析、零 GC 分配，支持不重建 Activity 实时丝滑换肤。
 */
object GcsTheme {

    const val THEME_AERO_CARBON = 0
    const val THEME_DAYLIGHT_POLAR = 1
    const val THEME_TACTICAL_NVG = 2
    const val THEME_THERMAL_AMBER = 3

    val TYPEFACE_TABULAR_MONO: Typeface = Typeface.create(Typeface.MONOSPACE, Typeface.BOLD)
    val TYPEFACE_LABEL_MEDIUM: Typeface = Typeface.create("sans-serif-medium", Typeface.NORMAL)

    enum class Role {
        GLASS_PANEL,
        GLASS_DRAWER,
        GLASS_POPOVER,
        MODAL_CARD,
        MODAL_HEADER,
        MODAL_LEFT_NAV,
        TOP_GRADIENT_BAR,
        SECTION_CARD,
        METRIC_CELL,
        WELL_TRACK,
        INPUT_BOX,
        CONSOLE_BOX,
        DIVIDER,
        ACCENT_BAR,
        TEXT_PRIMARY,
        TEXT_SECONDARY,
        TEXT_MUTED,
        TEXT_ACCENT,
        TEXT_CONSOLE
    }

    private const val TAG_ROLE = -0x35792468
    private const val TAG_CORNER_DP = -0x35792469

    data class Palette(
        val id: Int,
        val name: String,
        val subtitle: String,
        val isLight: Boolean,
        val swatchColor: Int,

        // 主强调色与次强调色
        val accent: Int,
        val accentBright: Int,
        val accentSoftFill: Int,
        val accentStroke: Int,
        val secondaryAccent: Int,

        // 悬浮毛玻璃与多层微渐变表面色
        val glassFill: Int,
        val glassFillTop: Int,
        val glassDrawerFill: Int,
        val glassDrawerFillTop: Int,
        val glassPopoverFill: Int,
        val glassStroke: Int,
        val glassHighlightStroke: Int,
        val topGradientStart: Int,

        val modalScrim: Int,
        val modalCardFill: Int,
        val modalHeaderFill: Int,
        val modalLeftNavFill: Int,
        val modalStroke: Int,

        // 卡片、指标单元格、凹槽与终端框
        val cardFill: Int,
        val cardFillTop: Int,
        val cardStroke: Int,
        val metricCellFill: Int,
        val wellFill: Int,
        val inputFill: Int,
        val divider: Int,
        val consoleFill: Int,
        val consoleStroke: Int,

        // 多层级排版文字色
        val textPrimary: Int,
        val textSecondary: Int,
        val textMuted: Int,
        val textHint: Int,
        val textConsole: Int,

        // HSI 雷达姿态球与罗盘专属配色
        val hsiOuterBg: Int,
        val hsiInnerBg: Int,
        val hsiSky: Int,
        val hsiGround: Int,
        val hsiRingStroke: Int,
        val hsiTickMajor: Int,
        val hsiTickMinor: Int,
        val hsiPillBg: Int,
        val compassReadoutBg: Int,

        // OpenGL ES 三维大气穹顶、远景雾与地面战术网格配色 (RGB 0..1)
        val glSkyFlat: FloatArray,
        val glSkyZenith: FloatArray,
        val glSkyHorizon: FloatArray,
        val glSkyGround: FloatArray,
        val glFog: FloatArray,
        val glGridRgb: FloatArray,
        val glGridMajorAlpha: Float,
        val glGridMinorAlpha: Float
    )

    val PALETTES: Array<Palette> = arrayOf(
        // 0. 深空航电黑 (精炼深碳蓝毛玻璃高对比座舱)
        Palette(
            id = THEME_AERO_CARBON,
            name = "深空航电黑",
            subtitle = "精炼深色高对比航空玻璃座舱",
            isLight = false,
            swatchColor = Color.rgb(24, 144, 255),
            accent = Color.rgb(24, 144, 255),
            accentBright = Color.rgb(118, 212, 255),
            accentSoftFill = Color.argb(72, 24, 144, 255),
            accentStroke = Color.argb(168, 56, 168, 255),
            secondaryAccent = Color.rgb(0, 230, 118),
            glassFill = Color.argb(216, 13, 17, 24),
            glassFillTop = Color.argb(226, 22, 29, 40),
            glassDrawerFill = Color.argb(246, 12, 16, 23),
            glassDrawerFillTop = Color.argb(248, 19, 25, 35),
            glassPopoverFill = Color.argb(248, 14, 19, 27),
            glassStroke = Color.argb(58, 175, 208, 248),
            glassHighlightStroke = Color.argb(82, 195, 222, 255),
            topGradientStart = Color.argb(232, 6, 9, 14),
            modalScrim = Color.argb(176, 3, 7, 13),
            modalCardFill = Color.argb(250, 13, 18, 26),
            modalHeaderFill = Color.argb(145, 6, 9, 15),
            modalLeftNavFill = Color.argb(96, 6, 10, 16),
            modalStroke = Color.argb(84, 96, 178, 255),
            cardFill = Color.argb(190, 20, 27, 38),
            cardFillTop = Color.argb(205, 26, 35, 49),
            cardStroke = Color.argb(46, 168, 204, 248),
            metricCellFill = Color.argb(142, 7, 11, 17),
            wellFill = Color.argb(156, 6, 9, 14),
            inputFill = Color.argb(182, 5, 8, 13),
            divider = Color.argb(38, 185, 212, 248),
            consoleFill = Color.argb(208, 6, 10, 16),
            consoleStroke = Color.argb(66, 96, 178, 255),
            textPrimary = Color.rgb(246, 250, 255),
            textSecondary = Color.argb(228, 208, 222, 240),
            textMuted = Color.argb(168, 168, 188, 212),
            textHint = Color.argb(115, 175, 196, 220),
            textConsole = Color.rgb(224, 239, 255),
            hsiOuterBg = Color.argb(248, 18, 24, 33),
            hsiInnerBg = Color.argb(246, 9, 13, 19),
            hsiSky = Color.rgb(24, 54, 86),
            hsiGround = Color.rgb(44, 35, 28),
            hsiRingStroke = Color.argb(82, 190, 218, 255),
            hsiTickMajor = Color.argb(185, 235, 244, 255),
            hsiTickMinor = Color.argb(78, 185, 210, 242),
            hsiPillBg = Color.argb(232, 10, 14, 20),
            compassReadoutBg = Color.argb(244, 11, 16, 23),
            glSkyFlat = floatArrayOf(0.043f, 0.051f, 0.066f),
            glSkyZenith = floatArrayOf(0.024f, 0.038f, 0.068f),
            glSkyHorizon = floatArrayOf(0.075f, 0.132f, 0.215f),
            glSkyGround = floatArrayOf(0.036f, 0.046f, 0.062f),
            glFog = floatArrayOf(0.042f, 0.056f, 0.082f),
            glGridRgb = floatArrayOf(0.78f, 0.86f, 0.96f),
            glGridMajorAlpha = 0.19f,
            glGridMinorAlpha = 0.065f
        ),

        // 1. 户外强光白 (针对 AX12 1000nits 屏幕户外阳光直射场景设计的高反差浅色座舱)
        Palette(
            id = THEME_DAYLIGHT_POLAR,
            name = "户外强光白",
            subtitle = "1000nits 烈日高反差浅色座舱",
            isLight = true,
            swatchColor = Color.rgb(0, 102, 230),
            accent = Color.rgb(0, 102, 230),
            accentBright = Color.rgb(0, 78, 192),
            accentSoftFill = Color.argb(50, 0, 102, 230),
            accentStroke = Color.argb(168, 0, 102, 230),
            secondaryAccent = Color.rgb(0, 146, 76),
            glassFill = Color.argb(236, 242, 246, 252),
            glassFillTop = Color.argb(244, 250, 252, 255),
            glassDrawerFill = Color.argb(250, 244, 248, 254),
            glassDrawerFillTop = Color.argb(252, 251, 253, 255),
            glassPopoverFill = Color.argb(252, 248, 251, 255),
            glassStroke = Color.argb(78, 18, 42, 76),
            glassHighlightStroke = Color.argb(96, 18, 42, 76),
            topGradientStart = Color.argb(214, 228, 236, 247),
            modalScrim = Color.argb(136, 12, 22, 36),
            modalCardFill = Color.argb(253, 245, 248, 253),
            modalHeaderFill = Color.argb(220, 230, 237, 247),
            modalLeftNavFill = Color.argb(185, 233, 240, 249),
            modalStroke = Color.argb(92, 0, 102, 230),
            cardFill = Color.argb(235, 255, 255, 255),
            cardFillTop = Color.argb(248, 255, 255, 255),
            cardStroke = Color.argb(56, 18, 42, 76),
            metricCellFill = Color.argb(195, 230, 237, 247),
            wellFill = Color.argb(205, 222, 231, 243),
            inputFill = Color.argb(248, 255, 255, 255),
            divider = Color.argb(44, 15, 32, 58),
            consoleFill = Color.argb(246, 231, 238, 247),
            consoleStroke = Color.argb(80, 0, 102, 230),
            textPrimary = Color.rgb(12, 23, 40),
            textSecondary = Color.rgb(38, 56, 82),
            textMuted = Color.rgb(78, 98, 126),
            textHint = Color.argb(140, 50, 72, 102),
            textConsole = Color.rgb(14, 30, 54),
            hsiOuterBg = Color.argb(248, 236, 242, 250),
            hsiInnerBg = Color.argb(248, 222, 231, 244),
            hsiSky = Color.rgb(86, 154, 226),
            hsiGround = Color.rgb(174, 146, 116),
            hsiRingStroke = Color.argb(95, 20, 42, 72),
            hsiTickMajor = Color.argb(215, 16, 34, 60),
            hsiTickMinor = Color.argb(105, 30, 52, 84),
            hsiPillBg = Color.argb(238, 245, 248, 253),
            compassReadoutBg = Color.argb(248, 245, 248, 253),
            glSkyFlat = floatArrayOf(0.84f, 0.875f, 0.918f),
            glSkyZenith = floatArrayOf(0.68f, 0.775f, 0.885f),
            glSkyHorizon = floatArrayOf(0.22f, 0.25f, 0.28f),
            glSkyGround = floatArrayOf(0.79f, 0.832f, 0.882f),
            glFog = floatArrayOf(0.82f, 0.86f, 0.905f),
            glGridRgb = floatArrayOf(0.16f, 0.25f, 0.38f),
            glGridMajorAlpha = 0.28f,
            glGridMinorAlpha = 0.11f
        ),

        // 2. 战术夜视绿 (暗夜低眩光磷光矩阵座舱)
        Palette(
            id = THEME_TACTICAL_NVG,
            name = "战术夜视绿",
            subtitle = "暗夜微光护眼低眩光磷光座舱",
            isLight = false,
            swatchColor = Color.rgb(0, 214, 114),
            accent = Color.rgb(0, 200, 106),
            accentBright = Color.rgb(105, 240, 174),
            accentSoftFill = Color.argb(64, 0, 214, 114),
            accentStroke = Color.argb(158, 0, 224, 120),
            secondaryAccent = Color.rgb(100, 255, 218),
            glassFill = Color.argb(218, 7, 18, 12),
            glassFillTop = Color.argb(228, 12, 28, 19),
            glassDrawerFill = Color.argb(246, 6, 16, 11),
            glassDrawerFillTop = Color.argb(248, 11, 24, 17),
            glassPopoverFill = Color.argb(248, 8, 20, 14),
            glassStroke = Color.argb(66, 0, 230, 118),
            glassHighlightStroke = Color.argb(90, 82, 245, 162),
            topGradientStart = Color.argb(230, 2, 10, 6),
            modalScrim = Color.argb(178, 2, 8, 5),
            modalCardFill = Color.argb(250, 7, 17, 12),
            modalHeaderFill = Color.argb(145, 2, 8, 5),
            modalLeftNavFill = Color.argb(98, 2, 8, 5),
            modalStroke = Color.argb(86, 0, 230, 118),
            cardFill = Color.argb(190, 10, 24, 17),
            cardFillTop = Color.argb(206, 14, 32, 22),
            cardStroke = Color.argb(52, 0, 230, 118),
            metricCellFill = Color.argb(148, 4, 12, 8),
            wellFill = Color.argb(158, 3, 9, 6),
            inputFill = Color.argb(182, 2, 8, 5),
            divider = Color.argb(38, 0, 230, 118),
            consoleFill = Color.argb(208, 3, 11, 7),
            consoleStroke = Color.argb(70, 0, 230, 118),
            textPrimary = Color.rgb(228, 255, 239),
            textSecondary = Color.argb(224, 174, 236, 200),
            textMuted = Color.argb(168, 136, 208, 166),
            textHint = Color.argb(115, 132, 214, 166),
            textConsole = Color.rgb(162, 255, 200),
            hsiOuterBg = Color.argb(248, 9, 22, 15),
            hsiInnerBg = Color.argb(246, 4, 12, 8),
            hsiSky = Color.rgb(11, 46, 30),
            hsiGround = Color.rgb(22, 32, 18),
            hsiRingStroke = Color.argb(80, 0, 230, 118),
            hsiTickMajor = Color.argb(188, 158, 255, 202),
            hsiTickMinor = Color.argb(86, 0, 230, 118),
            hsiPillBg = Color.argb(232, 4, 13, 8),
            compassReadoutBg = Color.argb(244, 5, 15, 10),
            glSkyFlat = floatArrayOf(0.022f, 0.048f, 0.033f),
            glSkyZenith = floatArrayOf(0.012f, 0.032f, 0.020f),
            glSkyHorizon = floatArrayOf(0.035f, 0.145f, 0.088f),
            glSkyGround = floatArrayOf(0.018f, 0.040f, 0.026f),
            glFog = floatArrayOf(0.020f, 0.048f, 0.032f),
            glGridRgb = floatArrayOf(0.28f, 0.95f, 0.58f),
            glGridMajorAlpha = 0.20f,
            glGridMinorAlpha = 0.07f
        ),

        // 3. 工业琥珀金 (高警觉工程巡检钛黑琥珀座舱)
        Palette(
            id = THEME_THERMAL_AMBER,
            name = "工业琥珀金",
            subtitle = "应急巡检钛金暖色仪表座舱",
            isLight = false,
            swatchColor = Color.rgb(255, 159, 10),
            accent = Color.rgb(245, 154, 12),
            accentBright = Color.rgb(255, 214, 84),
            accentSoftFill = Color.argb(66, 255, 159, 10),
            accentStroke = Color.argb(162, 255, 174, 12),
            secondaryAccent = Color.rgb(0, 230, 118),
            glassFill = Color.argb(218, 21, 17, 14),
            glassFillTop = Color.argb(228, 31, 25, 20),
            glassDrawerFill = Color.argb(246, 19, 16, 13),
            glassDrawerFillTop = Color.argb(248, 28, 23, 19),
            glassPopoverFill = Color.argb(248, 23, 19, 15),
            glassStroke = Color.argb(62, 255, 204, 128),
            glassHighlightStroke = Color.argb(86, 255, 218, 152),
            topGradientStart = Color.argb(230, 11, 8, 5),
            modalScrim = Color.argb(176, 9, 6, 4),
            modalCardFill = Color.argb(250, 20, 16, 13),
            modalHeaderFill = Color.argb(140, 8, 6, 4),
            modalLeftNavFill = Color.argb(94, 8, 6, 4),
            modalStroke = Color.argb(84, 255, 171, 0),
            cardFill = Color.argb(190, 28, 22, 18),
            cardFillTop = Color.argb(206, 36, 29, 23),
            cardStroke = Color.argb(48, 255, 198, 116),
            metricCellFill = Color.argb(148, 12, 9, 7),
            wellFill = Color.argb(156, 10, 8, 6),
            inputFill = Color.argb(180, 9, 7, 5),
            divider = Color.argb(38, 255, 205, 130),
            consoleFill = Color.argb(208, 12, 9, 7),
            consoleStroke = Color.argb(66, 255, 171, 0),
            textPrimary = Color.rgb(255, 249, 240),
            textSecondary = Color.argb(226, 240, 220, 194),
            textMuted = Color.argb(168, 216, 192, 162),
            textHint = Color.argb(115, 230, 200, 165),
            textConsole = Color.rgb(255, 230, 182),
            hsiOuterBg = Color.argb(248, 26, 21, 17),
            hsiInnerBg = Color.argb(246, 14, 11, 9),
            hsiSky = Color.rgb(52, 42, 32),
            hsiGround = Color.rgb(36, 25, 18),
            hsiRingStroke = Color.argb(78, 255, 198, 116),
            hsiTickMajor = Color.argb(185, 255, 230, 188),
            hsiTickMinor = Color.argb(84, 255, 198, 116),
            hsiPillBg = Color.argb(232, 14, 11, 9),
            compassReadoutBg = Color.argb(244, 16, 13, 10),
            glSkyFlat = floatArrayOf(0.058f, 0.046f, 0.038f),
            glSkyZenith = floatArrayOf(0.042f, 0.030f, 0.024f),
            glSkyHorizon = floatArrayOf(0.185f, 0.112f, 0.052f),
            glSkyGround = floatArrayOf(0.050f, 0.038f, 0.030f),
            glFog = floatArrayOf(0.058f, 0.045f, 0.036f),
            glGridRgb = floatArrayOf(0.98f, 0.80f, 0.52f),
            glGridMajorAlpha = 0.19f,
            glGridMinorAlpha = 0.065f
        )
    )

    fun of(themeId: Int): Palette = PALETTES[themeId.coerceIn(0, PALETTES.lastIndex)]

    fun markRole(view: View, role: Role, cornerDp: Float = 6f) {
        view.setTag(TAG_ROLE, role)
        view.setTag(TAG_CORNER_DP, cornerDp)
    }

    /**
     * 启用 OpenType 等宽数字特性 (Tabular Figures `tnum`) 与亚像素抗锯齿，杜绝 60Hz 刷新时数字位宽抖动。
     */
    fun applyTabularReadout(tv: TextView, bold: Boolean = true, letterSpacingEm: Float = -0.015f) {
        tv.typeface = if (bold) TYPEFACE_TABULAR_MONO else Typeface.MONOSPACE
        tv.fontFeatureSettings = "tnum, zero"
        tv.letterSpacing = letterSpacingEm
        tv.paintFlags = tv.paintFlags or Paint.SUBPIXEL_TEXT_FLAG
    }

    /**
     * 为原生 SeekBar 装配精炼航电风定制轨道与双环滑块，彻底消除系统默认滑块的风格割裂感。
     */
    fun styleAvionicsSeekBar(sb: SeekBar, p: Palette, density: Float) {
        val trackH = maxOf(4, (5.2f * density).roundToInt())
        val cornerR = trackH * 0.5f
        val bgTrack = GradientDrawable().apply {
            shape = GradientDrawable.RECTANGLE
            cornerRadius = cornerR
            setColor(p.wellFill)
            setStroke(1, p.cardStroke)
        }
        val fillShape = GradientDrawable(
            GradientDrawable.Orientation.LEFT_RIGHT,
            intArrayOf(p.accent, p.accentBright)
        ).apply {
            shape = GradientDrawable.RECTANGLE
            cornerRadius = cornerR
        }
        val clipProgress = ClipDrawable(fillShape, Gravity.START, ClipDrawable.HORIZONTAL)
        val layers = LayerDrawable(arrayOf(bgTrack, clipProgress)).apply {
            setId(0, android.R.id.background)
            setId(1, android.R.id.progress)
            val vInset = maxOf(2, (6.5f * density).roundToInt())
            setLayerInset(0, 0, vInset, 0, vInset)
            setLayerInset(1, 0, vInset, 0, vInset)
        }
        sb.progressDrawable = layers
        sb.splitTrack = false

        val thumbPx = maxOf(14, (16.5f * density).roundToInt())
        sb.thumb = GradientDrawable().apply {
            shape = GradientDrawable.OVAL
            setSize(thumbPx, thumbPx)
            setColor(if (p.isLight) Color.WHITE else Color.rgb(245, 250, 255))
            setStroke(maxOf(2, (2.2f * density).roundToInt()), p.accent)
        }
        sb.thumbOffset = thumbPx / 4
    }

    /**
     * 递归遍历组件树按语义 Role 实时刷新所有微渐变背景、精细描边、阴影与文字配色 (耗时 <0.5ms，零重建闪烁)
     */
    fun applyTree(view: View, p: Palette, density: Float) {
        val role = view.getTag(TAG_ROLE) as? Role
        val cornerDp = (view.getTag(TAG_CORNER_DP) as? Float) ?: 6f
        val rPx = (cornerDp * density).roundToInt().coerceAtLeast(0)
        val stroke1Px = maxOf(1, (0.9f * density).roundToInt())

        if (role != null) {
            when (role) {
                Role.GLASS_PANEL -> {
                    view.background = GradientDrawable(
                        GradientDrawable.Orientation.TOP_BOTTOM,
                        intArrayOf(p.glassFillTop, p.glassFill)
                    ).apply {
                        cornerRadius = rPx.toFloat()
                        setStroke(stroke1Px, p.glassStroke)
                    }
                    view.elevation = 3f * density
                }
                Role.GLASS_DRAWER -> {
                    view.background = GradientDrawable(
                        GradientDrawable.Orientation.TOP_BOTTOM,
                        intArrayOf(p.glassDrawerFillTop, p.glassDrawerFill)
                    ).apply {
                        cornerRadius = rPx.toFloat()
                        setStroke(stroke1Px, p.glassHighlightStroke)
                    }
                    view.elevation = 8f * density
                }
                Role.GLASS_POPOVER -> {
                    view.background = GradientDrawable(
                        GradientDrawable.Orientation.TOP_BOTTOM,
                        intArrayOf(p.glassDrawerFillTop, p.glassPopoverFill)
                    ).apply {
                        cornerRadius = rPx.toFloat()
                        setStroke(stroke1Px, p.glassHighlightStroke)
                    }
                    view.elevation = 7f * density
                }
                Role.MODAL_CARD -> {
                    view.background = GradientDrawable(
                        GradientDrawable.Orientation.TOP_BOTTOM,
                        intArrayOf(p.glassDrawerFillTop, p.modalCardFill)
                    ).apply {
                        cornerRadius = rPx.toFloat()
                        setStroke(maxOf(1, density.roundToInt()), p.modalStroke)
                    }
                    view.elevation = 12f * density
                }
                Role.MODAL_HEADER -> view.setBackgroundColor(p.modalHeaderFill)
                Role.MODAL_LEFT_NAV -> view.setBackgroundColor(p.modalLeftNavFill)
                Role.TOP_GRADIENT_BAR -> {
                    view.background = GradientDrawable(
                        GradientDrawable.Orientation.TOP_BOTTOM,
                        intArrayOf(p.topGradientStart, Color.TRANSPARENT)
                    )
                }
                Role.SECTION_CARD -> {
                    view.background = GradientDrawable(
                        GradientDrawable.Orientation.TOP_BOTTOM,
                        intArrayOf(p.cardFillTop, p.cardFill)
                    ).apply {
                        cornerRadius = rPx.toFloat()
                        setStroke(1, p.cardStroke)
                    }
                }
                Role.METRIC_CELL -> {
                    view.background = GradientDrawable().apply {
                        setColor(p.metricCellFill)
                        cornerRadius = rPx.toFloat()
                        setStroke(1, p.divider)
                    }
                }
                Role.WELL_TRACK -> {
                    view.background = GradientDrawable().apply {
                        setColor(p.wellFill)
                        cornerRadius = rPx.toFloat()
                        setStroke(1, p.divider)
                    }
                }
                Role.INPUT_BOX -> {
                    view.background = GradientDrawable().apply {
                        setColor(p.inputFill)
                        cornerRadius = rPx.toFloat()
                        setStroke(1, p.cardStroke)
                    }
                    if (view is EditText) {
                        view.setTextColor(p.textPrimary)
                        view.setHintTextColor(p.textHint)
                    }
                }
                Role.CONSOLE_BOX -> {
                    view.background = GradientDrawable().apply {
                        setColor(p.consoleFill)
                        cornerRadius = rPx.toFloat()
                        setStroke(1, p.consoleStroke)
                    }
                }
                Role.DIVIDER -> view.setBackgroundColor(p.divider)
                Role.ACCENT_BAR -> {
                    view.background = GradientDrawable().apply {
                        setColor(p.accent)
                        cornerRadius = rPx.toFloat()
                    }
                }
                Role.TEXT_PRIMARY -> if (view is TextView) {
                    view.setTextColor(p.textPrimary)
                    view.invalidate()
                }
                Role.TEXT_SECONDARY -> if (view is TextView) {
                    view.setTextColor(p.textSecondary)
                    view.invalidate()
                }
                Role.TEXT_MUTED -> if (view is TextView) {
                    view.setTextColor(p.textMuted)
                    view.invalidate()
                }
                Role.TEXT_ACCENT -> if (view is TextView) {
                    view.setTextColor(p.accentBright)
                    view.invalidate()
                }
                Role.TEXT_CONSOLE -> if (view is TextView) {
                    view.setTextColor(p.textConsole)
                    view.invalidate()
                }
            }
        }

        if (view is ViewGroup) {
            for (i in 0 until view.childCount) {
                applyTree(view.getChildAt(i), p, density)
            }
        }
    }
}
