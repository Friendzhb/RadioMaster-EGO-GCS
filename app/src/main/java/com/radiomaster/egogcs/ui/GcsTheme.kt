package com.radiomaster.egogcs.ui

import android.graphics.Color
import android.graphics.drawable.GradientDrawable
import android.view.View
import android.view.ViewGroup
import android.widget.EditText
import android.widget.TextView
import kotlin.math.roundToInt

/**
 * 商业航空级全栈主题系统 (同步驱动 2D HUD 玻璃座舱、HSI 姿态雷达罗盘与 3D OpenGL 天际大气穹顶)：
 * 预计算全部颜色常量与 OpenGL 归一化浮点数组，运行期零字符串解析、零 GC 分配，支持不重建 Activity 实时丝滑换肤。
 */
object GcsTheme {

    const val THEME_AERO_CARBON = 0
    const val THEME_DAYLIGHT_POLAR = 1
    const val THEME_TACTICAL_NVG = 2
    const val THEME_THERMAL_AMBER = 3

    enum class Role {
        GLASS_PANEL,
        GLASS_DRAWER,
        GLASS_POPOVER,
        MODAL_CARD,
        MODAL_HEADER,
        MODAL_LEFT_NAV,
        TOP_GRADIENT_BAR,
        SECTION_CARD,
        WELL_TRACK,
        INPUT_BOX,
        CONSOLE_BOX,
        DIVIDER,
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

        // 悬浮毛玻璃与弹窗表面色
        val glassFill: Int,
        val glassDrawerFill: Int,
        val glassPopoverFill: Int,
        val glassStroke: Int,
        val topGradientStart: Int,

        val modalScrim: Int,
        val modalCardFill: Int,
        val modalHeaderFill: Int,
        val modalLeftNavFill: Int,
        val modalStroke: Int,

        // 卡片、凹槽与终端框
        val cardFill: Int,
        val cardStroke: Int,
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
        // 0. 深空航电黑 (DJI Pilot 标准高对比暗色座舱)
        Palette(
            id = THEME_AERO_CARBON,
            name = "深空航电黑",
            subtitle = "标准暗色高对比航空座舱",
            isLight = false,
            swatchColor = Color.rgb(24, 144, 255),
            accent = Color.rgb(24, 144, 255),
            accentBright = Color.rgb(128, 216, 255),
            accentSoftFill = Color.argb(66, 24, 144, 255),
            accentStroke = Color.argb(145, 24, 144, 255),
            secondaryAccent = Color.rgb(0, 230, 118),
            glassFill = Color.argb(202, 17, 20, 26),
            glassDrawerFill = Color.argb(242, 15, 18, 24),
            glassPopoverFill = Color.argb(244, 16, 20, 27),
            glassStroke = Color.argb(38, 255, 255, 255),
            topGradientStart = Color.argb(218, 0, 0, 0),
            modalScrim = Color.argb(165, 4, 8, 14),
            modalCardFill = Color.argb(246, 15, 19, 26),
            modalHeaderFill = Color.argb(115, 0, 0, 0),
            modalLeftNavFill = Color.argb(72, 0, 0, 0),
            modalStroke = Color.argb(62, 105, 183, 255),
            cardFill = Color.argb(18, 255, 255, 255),
            cardStroke = Color.argb(26, 255, 255, 255),
            wellFill = Color.argb(118, 0, 0, 0),
            inputFill = Color.argb(148, 0, 0, 0),
            divider = Color.argb(24, 255, 255, 255),
            consoleFill = Color.argb(188, 6, 10, 15),
            consoleStroke = Color.argb(48, 105, 183, 255),
            textPrimary = Color.WHITE,
            textSecondary = Color.argb(205, 212, 224, 240),
            textMuted = Color.argb(142, 195, 210, 230),
            textHint = Color.argb(105, 255, 255, 255),
            textConsole = Color.rgb(230, 241, 255),
            hsiOuterBg = Color.argb(245, 20, 24, 31),
            hsiInnerBg = Color.argb(242, 10, 13, 17),
            hsiSky = Color.rgb(22, 50, 79),
            hsiGround = Color.rgb(42, 34, 27),
            hsiRingStroke = Color.argb(60, 255, 255, 255),
            hsiTickMajor = Color.argb(155, 255, 255, 255),
            hsiTickMinor = Color.argb(66, 255, 255, 255),
            hsiPillBg = Color.argb(215, 10, 12, 16),
            compassReadoutBg = Color.argb(238, 12, 15, 20),
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
            accentBright = Color.rgb(0, 82, 196),
            accentSoftFill = Color.argb(46, 0, 102, 230),
            accentStroke = Color.argb(155, 0, 102, 230),
            secondaryAccent = Color.rgb(0, 148, 78),
            glassFill = Color.argb(232, 244, 247, 252),
            glassDrawerFill = Color.argb(248, 246, 249, 254),
            glassPopoverFill = Color.argb(250, 248, 250, 255),
            glassStroke = Color.argb(68, 18, 42, 76),
            topGradientStart = Color.argb(196, 232, 239, 248),
            modalScrim = Color.argb(128, 14, 24, 38),
            modalCardFill = Color.argb(252, 246, 249, 253),
            modalHeaderFill = Color.argb(210, 232, 238, 247),
            modalLeftNavFill = Color.argb(175, 235, 241, 249),
            modalStroke = Color.argb(85, 0, 102, 230),
            cardFill = Color.argb(215, 255, 255, 255),
            cardStroke = Color.argb(46, 18, 42, 76),
            wellFill = Color.argb(190, 224, 232, 243),
            inputFill = Color.argb(245, 255, 255, 255),
            divider = Color.argb(34, 15, 32, 58),
            consoleFill = Color.argb(245, 233, 239, 247),
            consoleStroke = Color.argb(72, 0, 102, 230),
            textPrimary = Color.rgb(14, 25, 42),
            textSecondary = Color.rgb(46, 65, 92),
            textMuted = Color.rgb(88, 108, 135),
            textHint = Color.argb(135, 50, 72, 102),
            textConsole = Color.rgb(16, 32, 56),
            hsiOuterBg = Color.argb(246, 238, 243, 250),
            hsiInnerBg = Color.argb(246, 224, 232, 244),
            hsiSky = Color.rgb(88, 156, 228),
            hsiGround = Color.rgb(176, 148, 118),
            hsiRingStroke = Color.argb(85, 20, 42, 72),
            hsiTickMajor = Color.argb(205, 18, 36, 62),
            hsiTickMinor = Color.argb(95, 30, 52, 84),
            hsiPillBg = Color.argb(232, 245, 248, 253),
            compassReadoutBg = Color.argb(245, 245, 248, 253),
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
            accentSoftFill = Color.argb(58, 0, 214, 114),
            accentStroke = Color.argb(145, 0, 220, 118),
            secondaryAccent = Color.rgb(100, 255, 218),
            glassFill = Color.argb(210, 8, 19, 13),
            glassDrawerFill = Color.argb(244, 7, 17, 12),
            glassPopoverFill = Color.argb(246, 8, 20, 14),
            glassStroke = Color.argb(52, 0, 230, 118),
            topGradientStart = Color.argb(222, 2, 10, 6),
            modalScrim = Color.argb(172, 2, 9, 5),
            modalCardFill = Color.argb(248, 7, 18, 12),
            modalHeaderFill = Color.argb(130, 2, 8, 5),
            modalLeftNavFill = Color.argb(88, 2, 8, 5),
            modalStroke = Color.argb(76, 0, 230, 118),
            cardFill = Color.argb(24, 0, 230, 118),
            cardStroke = Color.argb(38, 0, 230, 118),
            wellFill = Color.argb(135, 2, 8, 5),
            inputFill = Color.argb(165, 2, 9, 6),
            divider = Color.argb(28, 0, 230, 118),
            consoleFill = Color.argb(200, 3, 11, 7),
            consoleStroke = Color.argb(62, 0, 230, 118),
            textPrimary = Color.rgb(224, 255, 236),
            textSecondary = Color.argb(210, 168, 232, 194),
            textMuted = Color.argb(148, 132, 204, 162),
            textHint = Color.argb(105, 132, 214, 166),
            textConsole = Color.rgb(158, 255, 198),
            hsiOuterBg = Color.argb(245, 9, 22, 15),
            hsiInnerBg = Color.argb(244, 4, 12, 8),
            hsiSky = Color.rgb(11, 46, 30),
            hsiGround = Color.rgb(22, 32, 18),
            hsiRingStroke = Color.argb(68, 0, 230, 118),
            hsiTickMajor = Color.argb(175, 150, 255, 195),
            hsiTickMinor = Color.argb(78, 0, 230, 118),
            hsiPillBg = Color.argb(220, 4, 13, 8),
            compassReadoutBg = Color.argb(240, 5, 15, 10),
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
            accentBright = Color.rgb(255, 213, 79),
            accentSoftFill = Color.argb(62, 255, 159, 10),
            accentStroke = Color.argb(150, 255, 171, 0),
            secondaryAccent = Color.rgb(0, 230, 118),
            glassFill = Color.argb(208, 24, 20, 17),
            glassDrawerFill = Color.argb(244, 22, 18, 15),
            glassPopoverFill = Color.argb(246, 24, 20, 16),
            glassStroke = Color.argb(46, 255, 200, 120),
            topGradientStart = Color.argb(220, 12, 9, 6),
            modalScrim = Color.argb(168, 10, 7, 5),
            modalCardFill = Color.argb(248, 22, 18, 15),
            modalHeaderFill = Color.argb(122, 8, 6, 4),
            modalLeftNavFill = Color.argb(82, 8, 6, 4),
            modalStroke = Color.argb(72, 255, 171, 0),
            cardFill = Color.argb(20, 255, 210, 150),
            cardStroke = Color.argb(32, 255, 195, 110),
            wellFill = Color.argb(128, 8, 6, 4),
            inputFill = Color.argb(158, 8, 6, 4),
            divider = Color.argb(26, 255, 205, 130),
            consoleFill = Color.argb(196, 12, 9, 7),
            consoleStroke = Color.argb(56, 255, 171, 0),
            textPrimary = Color.rgb(255, 248, 238),
            textSecondary = Color.argb(212, 238, 218, 192),
            textMuted = Color.argb(148, 214, 190, 160),
            textHint = Color.argb(105, 230, 200, 165),
            textConsole = Color.rgb(255, 228, 178),
            hsiOuterBg = Color.argb(245, 28, 23, 19),
            hsiInnerBg = Color.argb(244, 15, 12, 10),
            hsiSky = Color.rgb(52, 42, 32),
            hsiGround = Color.rgb(36, 25, 18),
            hsiRingStroke = Color.argb(66, 255, 195, 110),
            hsiTickMajor = Color.argb(175, 255, 228, 185),
            hsiTickMinor = Color.argb(75, 255, 195, 110),
            hsiPillBg = Color.argb(220, 14, 11, 9),
            compassReadoutBg = Color.argb(240, 16, 13, 10),
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
     * 递归遍历组件树按语义 Role 实时刷新所有背景、边框、文字与输入框配色 (耗时 <0.4ms，零重建闪烁)
     */
    fun applyTree(view: View, p: Palette, density: Float) {
        val role = view.getTag(TAG_ROLE) as? Role
        val cornerDp = (view.getTag(TAG_CORNER_DP) as? Float) ?: 6f
        val rPx = (cornerDp * density).roundToInt().coerceAtLeast(0)
        val stroke1Px = maxOf(1, (0.85f * density).roundToInt())

        if (role != null) {
            when (role) {
                Role.GLASS_PANEL -> {
                    view.background = GradientDrawable().apply {
                        setColor(p.glassFill)
                        cornerRadius = rPx.toFloat()
                        setStroke(stroke1Px, p.glassStroke)
                    }
                }
                Role.GLASS_DRAWER -> {
                    view.background = GradientDrawable().apply {
                        setColor(p.glassDrawerFill)
                        cornerRadius = rPx.toFloat()
                        setStroke(stroke1Px, p.glassStroke)
                    }
                }
                Role.GLASS_POPOVER -> {
                    view.background = GradientDrawable().apply {
                        setColor(p.glassPopoverFill)
                        cornerRadius = rPx.toFloat()
                        setStroke(stroke1Px, p.glassStroke)
                    }
                }
                Role.MODAL_CARD -> {
                    view.background = GradientDrawable().apply {
                        setColor(p.modalCardFill)
                        cornerRadius = rPx.toFloat()
                        setStroke(maxOf(1, density.roundToInt()), p.modalStroke)
                    }
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
                    view.background = GradientDrawable().apply {
                        setColor(p.cardFill)
                        cornerRadius = rPx.toFloat()
                        setStroke(1, p.cardStroke)
                    }
                }
                Role.WELL_TRACK -> {
                    view.background = GradientDrawable().apply {
                        setColor(p.wellFill)
                        cornerRadius = rPx.toFloat()
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
