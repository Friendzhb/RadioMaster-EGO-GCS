package com.radiomaster.egogcs.ui

import android.content.Context
import android.graphics.Color
import android.graphics.Typeface
import android.graphics.drawable.GradientDrawable
import android.os.Handler
import android.os.Looper
import android.text.Editable
import android.text.InputType
import android.text.TextUtils
import android.text.TextWatcher
import android.util.TypedValue
import android.view.Gravity
import android.view.HapticFeedbackConstants
import android.view.View
import android.view.ViewGroup.LayoutParams.MATCH_PARENT
import android.view.ViewGroup.LayoutParams.WRAP_CONTENT
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.SeekBar
import android.widget.TextView
import androidx.dynamicanimation.animation.DynamicAnimation
import androidx.dynamicanimation.animation.SpringAnimation
import androidx.dynamicanimation.animation.SpringForce
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.RenderSnapshot
import com.radiomaster.egogcs.model.TiltConfigSpec
import com.radiomaster.egogcs.model.TopicConfig
import com.radiomaster.egogcs.ota.OtaUpdateManager
import com.radiomaster.egogcs.ui.GcsIcons.Type
import com.radiomaster.egogcs.ui.GcsTheme.Role
import java.util.Locale
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * 对标主流商业飞控软件（DJI Pilot 2 / DJI Fly / QGroundControl）的精简务实设置面板：
 * 收敛为飞手真正高频使用的四大标准飞控分类，并在「显示与通用」内置 GitHub Releases 直链应用内 OTA 更新中心：
 * 0. 飞行安全：三维障碍物膨胀安全半径、近障与低电量告警阈值、返航二次确认、更新返航点 H；
 * 1. 感知与飞控：平稳/标准/敏捷三档飞行速度模式、最大速度/加速度限制、默认目标高度与高度步幅；
 * 2. 通信链路：原生 TCPROS / rosbridge 协议选择、局域网自动寻机、ELRS 电池遥测与话题预设；
 * 3. 显示与通用：GitHub 直链在线更新、四套环境光照主题、60帧/省电30帧档位、遮挡透视、点云建图精度与系统维护。
 */
class GcsSettingsModal(
    private val context: Context,
    private val hub: GcsSharedHub,
    private val otaManager: OtaUpdateManager,
    private val getCurrentTopicConfig: () -> TopicConfig,
    private val onThemeChanged: (animate: Boolean) -> Unit,
    private val onFlightKinematicsChanged: (inf: Float, vel: Float, acc: Float, gz: Float) -> Unit,
    private val onVisualSettingsChanged: () -> Unit,
    private val onApplyTopicPreset: (String) -> Unit,
    private val onProtocolChanged: (String) -> Unit,
    private val onJumpToConnDrawer: () -> Unit,
    private val onSetHomeHere: () -> Unit,
    private val onExportPcd: () -> Unit,
    private val onClearMapAndTrail: () -> Unit,
    private val onFactoryResetApplied: () -> Unit,
    private val onShowToast: (String) -> Unit,
    private val onTiltConfigChanged: () -> Unit = {}
) {

    private val readSnapshot = RenderSnapshot()

    interface ToggleSyncHandle {
        fun syncFromState(newVal: Boolean)
        fun refreshThemeColors()
    }

    data class SliderHandle(
        val seekBar: SeekBar,
        val valueTv: TextView,
        val minI: Int,
        val scale: Float,
        val fmt: String
    ) {
        fun syncValue(realVal: Float) {
            val p = ((realVal / scale).roundToInt() - minI).coerceIn(0, seekBar.max)
            if (seekBar.progress != p) seekBar.progress = p
            valueTv.text = String.format(Locale.US, fmt, (minI + p) * scale)
        }
    }

    private val d = context.resources.displayMetrics.density
    private val uiHandler = Handler(Looper.getMainLooper())

    lateinit var scrimRoot: FrameLayout
        private set
    private lateinit var dialogCard: LinearLayout
    private lateinit var btnHeaderQuickUpdate: TextView
    private lateinit var btnCloseHeader: ImageView

    var isOpen: Boolean = false
        private set
    private var activeCategory: Int = 0

    private val catNavItems = mutableListOf<LinearLayout>()
    private val catNavLabels = mutableListOf<TextView>()
    private val catNavBars = mutableListOf<View>()
    private val catPages = mutableListOf<View>()
    private val allToggleHandles = mutableListOf<ToggleSyncHandle>()
    private val allActionBtns = mutableListOf<TextView>()
    private val allSeekBars = mutableListOf<SeekBar>()
    private val allSliderValBadges = mutableListOf<TextView>()

    // 0. 飞行安全
    private lateinit var slInflation: SliderHandle
    private lateinit var slWarnClearance: SliderHandle
    private lateinit var slLowBatWarn: SliderHandle
    private lateinit var swRthConfirm: ToggleSyncHandle
    private lateinit var swLongPressFly: ToggleSyncHandle

    // 1. 感知与飞控
    private val flightModeBtns = mutableListOf<TextView>()
    private lateinit var slMaxVel: SliderHandle
    private lateinit var slMaxAcc: SliderHandle
    private lateinit var slGoalZ: SliderHandle
    private val altStepBtns = mutableListOf<TextView>()
    private lateinit var swTiltAirframe: ToggleSyncHandle
    private val tiltModeSegBtns = mutableListOf<TextView>()
    private lateinit var slTiltMaxDeg: SliderHandle
    private lateinit var slTiltPitchBias: SliderHandle

    // 2. 通信链路
    private val protoPrefBtns = mutableListOf<TextView>()
    private lateinit var swAutoConn: ToggleSyncHandle
    private lateinit var swMavlinkUdp: ToggleSyncHandle
    private lateinit var tvTopicSummaryBox: TextView

    // 3. 显示与通用 + GitHub 直链 OTA 更新
    private lateinit var generalScrollView: ScrollView
    private val otaChannelBtns = mutableListOf<TextView>()
    private lateinit var customUrlWrap: LinearLayout
    private lateinit var inpCustomOtaUrl: EditText
    private var suppressCustomUrlWatcher = false
    private lateinit var tvOtaConsoleBox: TextView
    private lateinit var otaProgressTrack: LinearLayout
    private lateinit var otaProgressFill: View
    private lateinit var btnOtaSyncMirrors: TextView
    private lateinit var btnOtaCheck: TextView
    private lateinit var btnOtaPrimaryDownload: TextView
    private lateinit var swOtaAutoCheck: ToggleSyncHandle

    private val themeSegBtns = mutableListOf<TextView>()
    private val perfModeSegBtns = mutableListOf<TextView>()
    private val voxelResBtns = mutableListOf<TextView>()
    private lateinit var swXray: ToggleSyncHandle
    private lateinit var swHaptics: ToggleSyncHandle
    private lateinit var tvSysDiagSummary: TextView
    private lateinit var btnResetFactory: TextView
    private var factoryResetConfirmPending = false

    private val cancelFactoryConfirmRunnable = Runnable {
        factoryResetConfirmPending = false
        if (::btnResetFactory.isInitialized) {
            GcsIcons.bind(btnResetFactory, Type.UNDO, "恢复默认", d, iconDp = 11.5f)
            btnResetFactory.background = pillBg(Color.argb(42, 255, 59, 48), Color.parseColor("#FF3B30"), radiusDp = 6f)
        }
    }

    companion object {
        const val PREFS_NAME = "ego_native_prefs"
        const val CAT_SAFETY = 0
        const val CAT_CONTROL = 1
        const val CAT_COMMS = 2
        const val CAT_GENERAL = 3
        const val CAT_VISUAL_3D = CAT_GENERAL
        const val CAT_THEME_ANIM = CAT_GENERAL

        fun loadPersistedSettings(context: Context, hub: GcsSharedHub) {
            val sp = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
            hub.themeId = sp.getInt("themeId", GcsTheme.THEME_AERO_CARBON).coerceIn(0, 3)
            hub.perfMode = sp.getInt("perfMode", 0).coerceIn(0, 2)
            hub.animStyle = 0
            AnimHelper.animStyle = 0
            hub.cameraDampingScale = 1.0f

            hub.obstaclesInflation = sp.getFloat("obstaclesInflation", 0.30f).coerceIn(0.10f, 0.65f)
            hub.maxVel = sp.getFloat("maxVel", 2.0f).coerceIn(0.5f, 4.5f)
            hub.maxAcc = sp.getFloat("maxAcc", 3.0f).coerceIn(0.5f, 6.0f)
            hub.goalZ = sp.getFloat("goalZ", 1.2f).coerceIn(0.4f, 4.0f)

            hub.autoConnectDiscovered = sp.getBoolean("autoConnectDiscovered", true)
            hub.mavlinkBackpackEnabled = sp.getBoolean("mavlinkBackpackEnabled", true)
            hub.xrayOcclusionEnabled = sp.getBoolean("xrayOcclusionEnabled", true)

            hub.showFastLioMap = sp.getBoolean("showFastLioMap", true)
            hub.showInflate = sp.getBoolean("showInflate", true)
            hub.showLocalBox = sp.getBoolean("showLocalBox", true)
            hub.showCtrlPts = sp.getBoolean("showCtrlPts", true)
            hub.showGroundGrid = true
            hub.showTrajCurtain = true
            hub.showFlightTrail = true
            hub.pointColorMode = sp.getInt("pointColorMode", 0).coerceIn(0, 2)
            hub.mapVoxelSize = sp.getFloat("mapVoxelSize", 0.12f).coerceIn(0.06f, 0.30f)
            hub.zSliceMax = sp.getFloat("zSliceMax", 3.0f).coerceIn(0.8f, 6.0f)
            hub.pointSizeScale = 1.0f

            hub.longPressQuickFlyEnabled = sp.getBoolean("longPressQuickFlyEnabled", true)
            hub.rthDoubleConfirm = sp.getBoolean("rthDoubleConfirm", true)
            hub.hapticFeedbackEnabled = sp.getBoolean("hapticFeedbackEnabled", true)
            AnimHelper.hapticsEnabled = hub.hapticFeedbackEnabled
            hub.autoCloseDrawerOnConnect = true
            hub.altStepMeters = sp.getFloat("altStepMeters", 0.2f).coerceIn(0.1f, 0.5f)
            hub.warnClearanceMeters = sp.getFloat("warnClearanceMeters", 0.65f).coerceIn(0.35f, 1.80f)
            hub.lowBatWarnPct = sp.getInt("lowBatWarnPct", 25).coerceIn(15, 45)

            hub.hudShowCompass = true
            hub.hudShowHsiRadar = true
            hub.hudShowTelemetryBox = true
            hub.hudShowAttitudeWidget = true
            hub.hudShowRightCamStack = true
            hub.hudShowDemoTools = true

            hub.routeLoopMode = sp.getInt("routeLoopMode", 0).coerceIn(0, 2)
            hub.routeFinishAction = sp.getInt("routeFinishAction", 0).coerceIn(0, 1)

            hub.swarmModeEnabled = sp.getBoolean("swarmModeEnabled", false)
            hub.swarmDroneCount = sp.getInt("swarmDroneCount", 3).coerceIn(2, 4)
            hub.selectedDroneId = sp.getInt("selectedDroneId", 0).coerceIn(0, 3)
            hub.swarmCommandScope = sp.getInt("swarmCommandScope", 1).coerceIn(0, 1)
            hub.swarmFormationType = sp.getInt("swarmFormationType", 0).coerceIn(0, 3)
            hub.swarmSpacingMeters = sp.getFloat("swarmSpacingMeters", 1.8f).coerceIn(1.0f, 3.6f)
            hub.showSwarmLinks = sp.getBoolean("showSwarmLinks", true)
            hub.tiltAirframeEnabled = sp.getBoolean("tiltAirframeEnabled", true)
            hub.swarmRigPolicy = sp.getInt("swarmRigPolicy", TiltConfigSpec.FLEET_RIG_ALL_VECTORED).coerceIn(0, 2)
            hub.swarmDroneTiltMask = sp.getInt("swarmDroneTiltMask", 0b0001) and 0x0F
            hub.tiltCoordMode = sp.getInt("tiltCoordMode", TiltConfigSpec.MODE_ACTIVE_PITCH).coerceIn(0, 2)
            hub.tiltPitchBiasDeg = sp.getFloat("tiltPitchBiasDeg", 0.0f).coerceIn(-25f, 25f)
            hub.tiltMaxAngleDeg = sp.getFloat("tiltMaxAngleDeg", 45.0f).coerceIn(15f, 60f)
            hub.autoNarrowGapPitchAssist = sp.getBoolean("autoNarrowGapPitchAssist", true)

            val legacyMode = sp.getInt("otaChannelMode", 0)
            val defaultModeV2 = if (legacyMode == 2) 2 else 0
            hub.otaChannelMode = sp.getInt("otaChannelModeV2", defaultModeV2).coerceIn(0, 2)
            hub.otaCustomUrl = sp.getString("otaCustomUrl", OtaUpdateManager.OFFICIAL_GITHUB_APK_URL)
                ?.takeIf { it.isNotBlank() } ?: OtaUpdateManager.OFFICIAL_GITHUB_APK_URL
            hub.otaAutoCheckOnStart = sp.getBoolean("otaAutoCheckOnStart", true)
        }

        fun persistAllSettings(context: Context, hub: GcsSharedHub) {
            context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE).edit()
                .putInt("themeId", hub.themeId)
                .putInt("perfMode", hub.perfMode)
                .putFloat("obstaclesInflation", hub.obstaclesInflation)
                .putFloat("maxVel", hub.maxVel)
                .putFloat("maxAcc", hub.maxAcc)
                .putFloat("goalZ", hub.goalZ)
                .putBoolean("autoConnectDiscovered", hub.autoConnectDiscovered)
                .putBoolean("mavlinkBackpackEnabled", hub.mavlinkBackpackEnabled)
                .putBoolean("xrayOcclusionEnabled", hub.xrayOcclusionEnabled)
                .putBoolean("showFastLioMap", hub.showFastLioMap)
                .putBoolean("showInflate", hub.showInflate)
                .putBoolean("showLocalBox", hub.showLocalBox)
                .putBoolean("showCtrlPts", hub.showCtrlPts)
                .putInt("pointColorMode", hub.pointColorMode)
                .putFloat("mapVoxelSize", hub.mapVoxelSize)
                .putFloat("zSliceMax", hub.zSliceMax)
                .putBoolean("longPressQuickFlyEnabled", hub.longPressQuickFlyEnabled)
                .putBoolean("rthDoubleConfirm", hub.rthDoubleConfirm)
                .putBoolean("hapticFeedbackEnabled", hub.hapticFeedbackEnabled)
                .putFloat("altStepMeters", hub.altStepMeters)
                .putFloat("warnClearanceMeters", hub.warnClearanceMeters)
                .putInt("lowBatWarnPct", hub.lowBatWarnPct)
                .putInt("routeLoopMode", hub.routeLoopMode)
                .putInt("routeFinishAction", hub.routeFinishAction)
                .putBoolean("swarmModeEnabled", hub.swarmModeEnabled)
                .putInt("swarmDroneCount", hub.swarmDroneCount)
                .putInt("selectedDroneId", hub.selectedDroneId)
                .putInt("swarmCommandScope", hub.swarmCommandScope)
                .putInt("swarmFormationType", hub.swarmFormationType)
                .putFloat("swarmSpacingMeters", hub.swarmSpacingMeters)
                .putBoolean("showSwarmLinks", hub.showSwarmLinks)
                .putBoolean("tiltAirframeEnabled", hub.tiltAirframeEnabled)
                .putInt("swarmRigPolicy", hub.swarmRigPolicy)
                .putInt("swarmDroneTiltMask", hub.swarmDroneTiltMask and 0x0F)
                .putInt("tiltCoordMode", hub.tiltCoordMode)
                .putFloat("tiltPitchBiasDeg", hub.tiltPitchBiasDeg)
                .putFloat("tiltMaxAngleDeg", hub.tiltMaxAngleDeg)
                .putBoolean("autoNarrowGapPitchAssist", hub.autoNarrowGapPitchAssist)
                .putInt("otaChannelMode", hub.otaChannelMode)
                .putInt("otaChannelModeV2", hub.otaChannelMode)
                .putString("otaCustomUrl", hub.otaCustomUrl)
                .putBoolean("otaAutoCheckOnStart", hub.otaAutoCheckOnStart)
                .apply()
        }
    }

    fun buildModalView(): View {
        val dm = context.resources.displayMetrics
        val modalW = min((dm.widthPixels * 0.94f).roundToInt(), dp(724f))
        val modalH = min((dm.heightPixels * 0.92f).roundToInt(), dp(464f))

        allToggleHandles.clear()
        allActionBtns.clear()
        allSeekBars.clear()
        allSliderValBadges.clear()

        scrimRoot = FrameLayout(context).apply {
            setBackgroundColor(GcsTheme.of(hub.themeId).modalScrim)
            visibility = View.GONE
            isClickable = true
            isFocusable = true
            setOnClickListener { setVisible(false) }
        }

        dialogCard = LinearLayout(context).apply {
            orientation = LinearLayout.VERTICAL
            GcsTheme.markRole(this, Role.MODAL_CARD, cornerDp = 11f)
            isClickable = true
            setOnClickListener { }
        }

        val header = LinearLayout(context).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setPadding(dp(12f), dp(6f), dp(8f), dp(6f))
            GcsTheme.markRole(this, Role.MODAL_HEADER)
        }
        val titleTv = TextView(context).apply {
            maxLines = 1
            setHorizontallyScrolling(false)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 11.8f)
            typeface = Typeface.DEFAULT_BOLD
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
            GcsIcons.bind(this, Type.SETTINGS_GEAR, "智航 · 飞控与系统设置", d, iconDp = 13.5f, gapDp = 5f)
        }

        // 顶栏右侧快捷「在线更新」胶囊按钮：在任意分类下均可一键跳至 OTA 面板并触发云端检查
        btnHeaderQuickUpdate = TextView(context).apply {
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            minimumHeight = dp(24f)
            setPadding(dp(8f), dp(3.5f), dp(8f), dp(3.5f))
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
            typeface = Typeface.DEFAULT_BOLD
            GcsIcons.bind(this, Type.DOWNLOAD, "在线更新", d, iconDp = 11f, gapDp = 3.5f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                if (activeCategory != CAT_GENERAL) {
                    switchCategory(CAT_GENERAL, animate = true)
                }
                if (::generalScrollView.isInitialized) {
                    generalScrollView.smoothScrollTo(0, 0)
                }
                otaManager.checkForUpdate(silentOnUpToDate = false)
            }
        }
        AnimHelper.attachSpringTouch(btnHeaderQuickUpdate)

        btnCloseHeader = ImageView(context).apply {
            scaleType = ImageView.ScaleType.CENTER
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                setVisible(false)
            }
        }
        AnimHelper.attachSpringTouch(btnCloseHeader)

        header.addView(titleTv)
        header.addView(View(context), LinearLayout.LayoutParams(0, 1, 1f))
        header.addView(
            btnHeaderQuickUpdate,
            LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply {
                rightMargin = dp(6f)
            }
        )
        header.addView(btnCloseHeader, LinearLayout.LayoutParams(dp(28f), dp(26f)))
        header.minimumHeight = dp(40f)
        dialogCard.addView(header, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT))

        dialogCard.addView(
            View(context).apply {
                GcsTheme.markRole(this, Role.DIVIDER)
            },
            LinearLayout.LayoutParams(MATCH_PARENT, 1)
        )

        val bodyRow = LinearLayout(context).apply {
            orientation = LinearLayout.HORIZONTAL
        }

        val leftNav = LinearLayout(context).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(7f), dp(8f), dp(7f), dp(8f))
            GcsTheme.markRole(this, Role.MODAL_LEFT_NAV)
        }

        val categories = listOf(
            Type.SHIELD_SAFE to "飞行安全",
            Type.SLIDERS to "感知与飞控",
            Type.LINK to "通信链路",
            Type.SETTINGS_GEAR to "显示与通用"
        )

        catNavItems.clear()
        catNavLabels.clear()
        catNavBars.clear()

        for (i in categories.indices) {
            val (ic, name) = categories[i]
            val itemRow = LinearLayout(context).apply {
                orientation = LinearLayout.HORIZONTAL
                gravity = Gravity.CENTER_VERTICAL
                minimumHeight = dp(32f)
                setPadding(dp(7f), dp(6.5f), dp(8f), dp(6.5f))
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    switchCategory(i)
                }
            }
            val bar = View(context)
            val lbl = TextView(context).apply {
                maxLines = 1
                setHorizontallyScrolling(false)
                ellipsize = TextUtils.TruncateAt.END
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.6f)
                typeface = Typeface.DEFAULT_BOLD
                GcsIcons.bind(this, ic, name, d, iconDp = 12.5f, gapDp = 5f)
            }
            itemRow.addView(bar, LinearLayout.LayoutParams(dp(3f), dp(14f)).apply { rightMargin = dp(6f) })
            itemRow.addView(lbl, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))

            catNavItems.add(itemRow)
            catNavBars.add(bar)
            catNavLabels.add(lbl)
            leftNav.addView(
                itemRow,
                LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply {
                    if (i > 0) topMargin = dp(4f)
                }
            )
        }

        bodyRow.addView(leftNav, LinearLayout.LayoutParams(dp(136f), MATCH_PARENT))
        bodyRow.addView(
            View(context).apply {
                GcsTheme.markRole(this, Role.DIVIDER)
            },
            LinearLayout.LayoutParams(1, MATCH_PARENT)
        )

        val rightHost = FrameLayout(context)
        catPages.clear()
        catPages.addAll(
            listOf(
                buildCategorySafety(),
                buildCategoryControl(),
                buildCategoryComms(),
                buildCategoryGeneral()
            )
        )
        for (idx in catPages.indices) {
            catPages[idx].visibility = if (idx == 0) View.VISIBLE else View.GONE
            rightHost.addView(catPages[idx], FrameLayout.LayoutParams(MATCH_PARENT, MATCH_PARENT))
        }

        bodyRow.addView(rightHost, LinearLayout.LayoutParams(0, MATCH_PARENT, 1f))
        dialogCard.addView(bodyRow, LinearLayout.LayoutParams(MATCH_PARENT, 0, 1f))

        scrimRoot.addView(dialogCard, FrameLayout.LayoutParams(modalW, modalH, Gravity.CENTER))
        applyModalThemeColors()
        switchCategory(0, animate = false)
        renderOtaSnapshot(otaManager.currentSnapshot)
        return scrimRoot
    }

    fun applyModalThemeColors() {
        if (!::scrimRoot.isInitialized) return
        val pal = GcsTheme.of(hub.themeId)
        scrimRoot.setBackgroundColor(pal.modalScrim)
        GcsTheme.applyTree(dialogCard, pal, d)

        btnCloseHeader.setImageDrawable(GcsIcons.drawable(Type.CLOSE_X, dp(14f), pal.textPrimary))
        btnCloseHeader.background = roundedColor(pal.cardFill, dp(6f))

        for (ab in allActionBtns) {
            ab.setTextColor(pal.textPrimary)
            ab.background = pillBg(pal.cardFill, pal.cardStroke, radiusDp = 6f)
            ab.invalidate()
        }
        for (sb in allSeekBars) {
            GcsTheme.styleAvionicsSeekBar(sb, pal, d)
        }
        for (vb in allSliderValBadges) {
            vb.setTextColor(pal.accentBright)
            vb.invalidate()
        }
        for (th in allToggleHandles) {
            th.refreshThemeColors()
        }
        syncTiltModeSegButtons()

        syncCategoryNavVisuals()
        syncFlightModeButtons()
        syncAltStepButtons()
        syncProtoPrefButtons()
        syncOtaChannelButtons()
        syncThemeSegButtons()
        syncPerfModeButtons()
        syncVoxelResButtons()
        renderOtaSnapshot(otaManager.currentSnapshot)
    }

    fun setVisible(show: Boolean, jumpCategory: Int? = null) {
        if (jumpCategory != null && jumpCategory in catPages.indices) {
            switchCategory(jumpCategory, animate = false)
        }
        if (show) {
            syncAllControlsFromHub()
        } else {
            uiHandler.removeCallbacks(cancelFactoryConfirmRunnable)
            cancelFactoryConfirmRunnable.run()
        }
        if (isOpen == show) return
        isOpen = show
        AnimHelper.animateSettingsModal(scrimRoot, dialogCard, show, d)
        if (show && activeCategory in catPages.indices) {
            AnimHelper.animateStaggeredChildren(catPages[activeCategory], d)
        }
    }

    fun syncAllControlsFromHub() {
        if (!::scrimRoot.isInitialized) return
        slInflation.syncValue(hub.obstaclesInflation)
        slWarnClearance.syncValue(hub.warnClearanceMeters)
        slLowBatWarn.syncValue(hub.lowBatWarnPct.toFloat())
        swRthConfirm.syncFromState(hub.rthDoubleConfirm)
        swLongPressFly.syncFromState(hub.longPressQuickFlyEnabled)

        slMaxVel.syncValue(hub.maxVel)
        slMaxAcc.syncValue(hub.maxAcc)
        slGoalZ.syncValue(hub.goalZ)
        syncFlightModeButtons()
        syncAltStepButtons()
        if (::swTiltAirframe.isInitialized) {
            swTiltAirframe.syncFromState(hub.activeFocusIsTiltRig())
            slTiltMaxDeg.syncValue(hub.tiltMaxAngleDeg)
            slTiltPitchBias.syncValue(hub.tiltPitchBiasDeg)
            syncTiltModeSegButtons()
        }

        syncProtoPrefButtons()
        swAutoConn.syncFromState(hub.autoConnectDiscovered)
        swMavlinkUdp.syncFromState(hub.mavlinkBackpackEnabled)
        refreshTopicSummaryBox()

        syncOtaChannelButtons()
        swOtaAutoCheck.syncFromState(hub.otaAutoCheckOnStart)
        if (::inpCustomOtaUrl.isInitialized && !TextUtils.equals(inpCustomOtaUrl.text, hub.otaCustomUrl)) {
            suppressCustomUrlWatcher = true
            inpCustomOtaUrl.setText(hub.otaCustomUrl)
            suppressCustomUrlWatcher = false
        }
        renderOtaSnapshot(otaManager.currentSnapshot)

        syncThemeSegButtons()
        syncPerfModeButtons()
        syncVoxelResButtons()
        swXray.syncFromState(hub.xrayOcclusionEnabled)
        swHaptics.syncFromState(hub.hapticFeedbackEnabled)
        refreshSystemDiagnostics()
    }

    /**
     * 实时刷新设置面板内的 GitHub 直链 OTA 状态控制台、进度条与主操作按钮。
     */
    fun renderOtaSnapshot(snap: OtaUpdateManager.Snapshot) {
        if (!::scrimRoot.isInitialized || !::tvOtaConsoleBox.isInitialized) return
        val pal = GcsTheme.of(hub.themeId)

        // 顶栏「在线更新」快捷徽章高亮提示
        if (snap.phase == OtaUpdateManager.Phase.UPDATE_AVAILABLE) {
            btnHeaderQuickUpdate.setTextColor(Color.WHITE)
            btnHeaderQuickUpdate.background = pillBg(pal.accent, pal.accentStroke, radiusDp = 6f)
            GcsIcons.bind(btnHeaderQuickUpdate, Type.DOWNLOAD, "发现新版本", d, iconDp = 11f, gapDp = 3.5f)
        } else {
            btnHeaderQuickUpdate.setTextColor(pal.accent)
            btnHeaderQuickUpdate.background = pillBg(pal.accentSoftFill, pal.accentStroke, radiusDp = 6f)
            GcsIcons.bind(btnHeaderQuickUpdate, Type.DOWNLOAD, "在线更新", d, iconDp = 11f, gapDp = 3.5f)
        }
        btnHeaderQuickUpdate.invalidate()

        tvOtaConsoleBox.setTextColor(pal.textConsole)
        val rawNoScheme = snap.activeDirectUrl
            .removePrefix("https://")
            .removePrefix("http://")
        val shortUrl = if (rawNoScheme.contains("/https://github.com/")) {
            val proxyHost = rawNoScheme.substringBefore("/https://github.com/")
            "$proxyHost -> github.com/.../latest-apk/EGO-Pilot-AX12.apk"
        } else {
            rawNoScheme
        }
        val summaryText = snap.localVersionSummary + "\n" +
            snap.remoteSummary + "\n" +
            snap.mirrorPoolSummary + "\n" +
            "当前通道 : $shortUrl\n" +
            "更新状态 : ${snap.statusMessage}"
        if (!TextUtils.equals(tvOtaConsoleBox.text, summaryText)) {
            tvOtaConsoleBox.text = summaryText
        }

        // 进度条显隐与宽度更新
        val showBar = snap.phase == OtaUpdateManager.Phase.DOWNLOADING ||
            snap.phase == OtaUpdateManager.Phase.READY_TO_INSTALL ||
            snap.phase == OtaUpdateManager.Phase.CHECKING ||
            snap.phase == OtaUpdateManager.Phase.SYNCING_MIRRORS
        otaProgressTrack.visibility = if (showBar) View.VISIBLE else View.GONE
        if (showBar) {
            otaProgressTrack.background = roundedColor(pal.wellFill, dp(4f))
            val fillCol = when (snap.phase) {
                OtaUpdateManager.Phase.READY_TO_INSTALL -> Color.parseColor("#00E676")
                OtaUpdateManager.Phase.CHECKING, OtaUpdateManager.Phase.SYNCING_MIRRORS -> pal.accentStroke
                else -> pal.accent
            }
            otaProgressFill.background = roundedColor(fillCol, dp(4f))
            val pct = when (snap.phase) {
                OtaUpdateManager.Phase.SYNCING_MIRRORS -> 22
                OtaUpdateManager.Phase.CHECKING -> 26
                OtaUpdateManager.Phase.READY_TO_INSTALL -> 100
                else -> snap.progressPct.coerceIn(3, 100)
            }
            val targetWeight = pct.coerceIn(3, 100).toFloat()
            val lp = otaProgressFill.layoutParams as LinearLayout.LayoutParams
            if (lp.weight != targetWeight) {
                lp.weight = targetWeight
                otaProgressFill.layoutParams = lp
            }
        }

        val busyMeta = (
            snap.phase == OtaUpdateManager.Phase.DOWNLOADING ||
                snap.phase == OtaUpdateManager.Phase.CHECKING ||
                snap.phase == OtaUpdateManager.Phase.SYNCING_MIRRORS
            )

        // 实时加速源同步测速按钮状态
        btnOtaSyncMirrors.isEnabled = !busyMeta
        btnOtaSyncMirrors.alpha = if (btnOtaSyncMirrors.isEnabled) 1.0f else 0.52f
        val syncLabel = if (snap.phase == OtaUpdateManager.Phase.SYNCING_MIRRORS) "测速中..." else "刷新加速源"
        GcsIcons.bind(btnOtaSyncMirrors, Type.RADAR_SIGNAL, syncLabel, d, iconDp = 11.5f, gapDp = 3.5f)

        // 检查按钮状态
        btnOtaCheck.isEnabled = !busyMeta
        btnOtaCheck.alpha = if (btnOtaCheck.isEnabled) 1.0f else 0.52f
        val checkLabel = if (snap.phase == OtaUpdateManager.Phase.CHECKING) "查询中..." else "检查更新"
        GcsIcons.bind(btnOtaCheck, Type.REFRESH, checkLabel, d, iconDp = 11.5f, gapDp = 3.5f)

        // 主下载/取消/覆盖安装按钮状态
        btnOtaPrimaryDownload.isEnabled = (snap.phase != OtaUpdateManager.Phase.SYNCING_MIRRORS)
        btnOtaPrimaryDownload.alpha = if (btnOtaPrimaryDownload.isEnabled) 1.0f else 0.62f
        when (snap.phase) {
            OtaUpdateManager.Phase.DOWNLOADING -> {
                btnOtaPrimaryDownload.setTextColor(Color.WHITE)
                btnOtaPrimaryDownload.background = pillBg(Color.parseColor("#E53935"), Color.parseColor("#FF8A80"), radiusDp = 6f)
                GcsIcons.bind(
                    btnOtaPrimaryDownload,
                    Type.STOP_SQUARE,
                    "取消下载 ${snap.progressPct}%",
                    d,
                    iconDp = 11.5f,
                    gapDp = 3.5f
                )
            }
            OtaUpdateManager.Phase.READY_TO_INSTALL -> {
                btnOtaPrimaryDownload.setTextColor(Color.parseColor("#08140E"))
                btnOtaPrimaryDownload.background = pillBg(Color.parseColor("#00E676"), Color.parseColor("#69F0AE"), radiusDp = 6f)
                GcsIcons.bind(
                    btnOtaPrimaryDownload,
                    Type.CHECK,
                    "立即覆盖安装",
                    d,
                    iconDp = 11.5f,
                    gapDp = 3.5f
                )
            }
            OtaUpdateManager.Phase.UPDATE_AVAILABLE -> {
                btnOtaPrimaryDownload.setTextColor(Color.WHITE)
                btnOtaPrimaryDownload.background = pillBg(pal.accent, pal.accentStroke, radiusDp = 6f)
                GcsIcons.bind(
                    btnOtaPrimaryDownload,
                    Type.DOWNLOAD,
                    "直链下载新版",
                    d,
                    iconDp = 11.5f,
                    gapDp = 3.5f
                )
            }
            else -> {
                btnOtaPrimaryDownload.setTextColor(Color.WHITE)
                btnOtaPrimaryDownload.background = pillBg(pal.accent, pal.accentStroke, radiusDp = 6f)
                GcsIcons.bind(
                    btnOtaPrimaryDownload,
                    Type.DOWNLOAD,
                    "直链下载更新",
                    d,
                    iconDp = 11.5f,
                    gapDp = 3.5f
                )
            }
        }
        btnOtaPrimaryDownload.invalidate()
    }

    private fun syncCategoryNavVisuals() {
        val pal = GcsTheme.of(hub.themeId)
        for (i in catNavItems.indices) {
            val sel = (i == activeCategory)
            catNavBars[i].visibility = if (sel) View.VISIBLE else View.INVISIBLE
            catNavBars[i].background = roundedColor(pal.accent, dp(2f))
            catNavLabels[i].setTextColor(if (sel) pal.textPrimary else pal.textMuted)
            catNavLabels[i].invalidate()
            catNavItems[i].background = if (sel) {
                pillBg(pal.accentSoftFill, pal.accentStroke, radiusDp = 7f)
            } else {
                roundedColor(Color.TRANSPARENT, dp(7f))
            }
        }
    }

    private fun switchCategory(idx: Int, animate: Boolean = true) {
        activeCategory = idx.coerceIn(0, catPages.lastIndex)
        syncCategoryNavVisuals()
        for (i in catPages.indices) {
            val page = catPages[i]
            if (i == activeCategory) {
                page.visibility = View.VISIBLE
                if (animate) {
                    page.alpha = 0f
                    page.translationY = dp(5f).toFloat()
                    page.animate()
                        .alpha(1f)
                        .translationY(0f)
                        .setDuration(170L)
                        .setInterpolator(AnimHelper.EASE_OUT_EXPO)
                        .start()
                    AnimHelper.animateStaggeredChildren(page, d)
                } else {
                    page.alpha = 1f
                    page.translationY = 0f
                }
            } else {
                page.visibility = View.GONE
            }
        }
        if (activeCategory == CAT_COMMS) refreshTopicSummaryBox()
        if (activeCategory == CAT_GENERAL) {
            refreshSystemDiagnostics()
            renderOtaSnapshot(otaManager.currentSnapshot)
        }
    }

    // ==================== 0. 飞行安全 ====================

    private fun buildCategorySafety(): View {
        val sv = ScrollView(context)
        val col = pageColumn()

        val obsCard = groupCard("避障安全边界与告警阈值")
        slInflation = addSliderRow(
            obsCard,
            "三维障碍物膨胀半径 (EGO 安全裕度)",
            12,
            60,
            (hub.obstaclesInflation * 100f).roundToInt(),
            0.01f,
            "%.2f m"
        ) { v ->
            hub.obstaclesInflation = v
            persistAllSettings(context, hub)
            onFlightKinematicsChanged(hub.obstaclesInflation, hub.maxVel, hub.maxAcc, hub.goalZ)
            syncFlightModeButtons()
        }
        slWarnClearance = addSliderRow(
            obsCard,
            "近障黄/红变色预警距离",
            40,
            160,
            (hub.warnClearanceMeters * 100f).roundToInt(),
            0.01f,
            "%.2f m"
        ) { v ->
            hub.warnClearanceMeters = v
            persistAllSettings(context, hub)
        }
        slLowBatWarn = addSliderRow(
            obsCard,
            "飞机低电量告警百分比",
            15,
            45,
            hub.lowBatWarnPct,
            1.0f,
            "%.0f %%"
        ) { v ->
            hub.lowBatWarnPct = v.roundToInt()
            persistAllSettings(context, hub)
        }
        col.addView(obsCard)

        val rthCard = groupCard("返航点与防误触保护")
        swRthConfirm = addSwitchRow(
            rthCard,
            Type.HOME_RTH,
            "一键返航二次确认",
            "需在 2.5 秒内连续点击两次才执行返航，防止误触中断任务",
            hub.rthDoubleConfirm
        ) { v ->
            hub.rthDoubleConfirm = v
            persistAllSettings(context, hub)
        }
        swLongPressFly = addSwitchRow(
            rthCard,
            Type.SEND_FLY,
            "长按地图直接下发指点飞行",
            "关闭后长按地图仅放置预览光柱，需在底部点击确认才起飞",
            hub.longPressQuickFlyEnabled,
            showDivider = false
        ) { v ->
            hub.longPressQuickFlyEnabled = v
            persistAllSettings(context, hub)
        }

        val btnSetHome = actionBtn(Type.HOME_RTH, "将飞机当前位置更新为返航点 H") {
            onSetHomeHere()
        }
        rthCard.addView(btnSetHome, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(6f) })
        col.addView(rthCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        sv.addView(col)
        return sv
    }

    // ==================== 1. 感知与飞控 ====================

    private fun buildCategoryControl(): View {
        val sv = ScrollView(context)
        val col = pageColumn()

        val speedCard = groupCard("飞行档位与运动学限制 (实时同步机载 EGO-Planner)")
        val modeSeg = segmentedTrack()
        val modes = listOf(
            Triple(Type.GAUGE_LOW, "平稳 1.0m/s", floatArrayOf(0.35f, 1.0f, 1.8f)),
            Triple(Type.GAUGE_MID, "标准 2.0m/s", floatArrayOf(0.30f, 2.0f, 3.0f)),
            Triple(Type.GAUGE_HIGH, "敏捷 3.2m/s", floatArrayOf(0.22f, 3.2f, 4.6f))
        )
        flightModeBtns.clear()
        for (i in modes.indices) {
            val (ic, title, preset) = modes[i]
            val b = segmentBtn(ic, title) {
                hub.obstaclesInflation = preset[0]
                hub.maxVel = preset[1]
                hub.maxAcc = preset[2]
                persistAllSettings(context, hub)
                slInflation.syncValue(hub.obstaclesInflation)
                slMaxVel.syncValue(hub.maxVel)
                slMaxAcc.syncValue(hub.maxAcc)
                syncFlightModeButtons()
                onFlightKinematicsChanged(hub.obstaclesInflation, hub.maxVel, hub.maxAcc, hub.goalZ)
                onShowToast("已切换飞行档位：$title")
            }
            flightModeBtns.add(b)
            modeSeg.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        speedCard.addView(modeSeg, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { bottomMargin = dp(6f) })

        slMaxVel = addSliderRow(
            speedCard,
            "最大规划水平速度 (max_vel)",
            5,
            45,
            (hub.maxVel * 10f).roundToInt(),
            0.1f,
            "%.1f m/s"
        ) { v ->
            hub.maxVel = v
            persistAllSettings(context, hub)
            syncFlightModeButtons()
            onFlightKinematicsChanged(hub.obstaclesInflation, hub.maxVel, hub.maxAcc, hub.goalZ)
        }

        slMaxAcc = addSliderRow(
            speedCard,
            "最大规划加速度 (max_acc)",
            5,
            60,
            (hub.maxAcc * 10f).roundToInt(),
            0.1f,
            "%.1f m/s²"
        ) { v ->
            hub.maxAcc = v
            persistAllSettings(context, hub)
            syncFlightModeButtons()
            onFlightKinematicsChanged(hub.obstaclesInflation, hub.maxVel, hub.maxAcc, hub.goalZ)
        }
        col.addView(speedCard)

        val altCard = groupCard("目标飞行高度与微调步幅")
        slGoalZ = addSliderRow(
            altCard,
            "默认指点飞行高度 (goal_z)",
            4,
            36,
            (hub.goalZ * 10f).roundToInt(),
            0.1f,
            "%.1f m"
        ) { v ->
            hub.goalZ = v
            persistAllSettings(context, hub)
            onFlightKinematicsChanged(hub.obstaclesInflation, hub.maxVel, hub.maxAcc, hub.goalZ)
        }

        altCard.addView(subFieldLabel("指点高度 +/- 微调步进"))
        val stepSeg = segmentedTrack()
        val steps = floatArrayOf(0.1f, 0.2f, 0.5f)
        altStepBtns.clear()
        for (st in steps) {
            val b = segmentBtn(null, String.format(Locale.US, "%.1f m", st)) {
                hub.altStepMeters = st
                persistAllSettings(context, hub)
                syncAltStepButtons()
            }
            altStepBtns.add(b)
            stepSeg.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        altCard.addView(stepSeg)
        col.addView(altCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        val tiltCard = groupCard("倾转矢量飞控分配 (Tilt-UAV-INDI · 4电机+双舵机)")
        swTiltAirframe = addSwitchRow(
            tiltCard,
            Type.TILT_VECTOR,
            "启用双舵机倾转四旋翼构型 (4M+2S)",
            "S1 控制前组 M1/M3，S2 控制后组 M2/M4；关闭则切换为常规固定四旋翼",
            hub.tiltAirframeEnabled
        ) { v ->
            hub.setActiveFocusTiltRig(v)
            DroneVectorModel.activeRig = DroneVectorModel.rigFor(hub.activeFocusIsTiltRig())
            persistAllSettings(context, hub)
            onTiltConfigChanged()
        }

        tiltCard.addView(subFieldLabel("外环俯仰-倾转协同模式"))
        val tSeg = segmentedTrack()
        tiltModeSegBtns.clear()
        for (i in TiltConfigSpec.SHORT_MODE_NAMES.indices) {
            val b = segmentBtn(null, TiltConfigSpec.SHORT_MODE_NAMES[i]) {
                hub.tiltCoordMode = i
                persistAllSettings(context, hub)
                syncTiltModeSegButtons()
                onTiltConfigChanged()
                onShowToast("已切换倾转模式：${TiltConfigSpec.FULL_MODE_NAMES[i]}")
            }
            tiltModeSegBtns.add(b)
            tSeg.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        tiltCard.addView(tSeg, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { bottomMargin = dp(6f) })

        slTiltMaxDeg = addSliderRow(
            tiltCard,
            "舵机最大倾转角包线 (SERVO_TILT_MAX)",
            15,
            60,
            hub.tiltMaxAngleDeg.roundToInt(),
            1.0f,
            "%.0f°"
        ) { v ->
            hub.tiltMaxAngleDeg = v
            persistAllSettings(context, hub)
            onTiltConfigChanged()
        }

        slTiltPitchBias = addSliderRow(
            tiltCard,
            "窄缝穿越主动俯仰偏置角 (theta_ref)",
            -25,
            25,
            hub.tiltPitchBiasDeg.roundToInt(),
            1.0f,
            "%+.0f°"
        ) { v ->
            hub.tiltPitchBiasDeg = v
            persistAllSettings(context, hub)
            onTiltConfigChanged()
        }
        col.addView(tiltCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        sv.addView(col)
        return sv
    }

    private fun syncTiltModeSegButtons() {
        val pal = GcsTheme.of(hub.themeId)
        for (i in tiltModeSegBtns.indices) {
            styleSegmentSelection(tiltModeSegBtns[i], hub.tiltCoordMode == i, pal.accent, pal)
        }
    }

    private fun syncFlightModeButtons() {
        val pal = GcsTheme.of(hub.themeId)
        val activeIdx = when {
            abs(hub.maxVel - 1.0f) < 0.08f && abs(hub.maxAcc - 1.8f) < 0.15f -> 0
            abs(hub.maxVel - 2.0f) < 0.08f && abs(hub.maxAcc - 3.0f) < 0.15f -> 1
            abs(hub.maxVel - 3.2f) < 0.08f && abs(hub.maxAcc - 4.6f) < 0.15f -> 2
            else -> -1
        }
        for (i in flightModeBtns.indices) {
            styleSegmentSelection(flightModeBtns[i], i == activeIdx, pal.accent, pal)
        }
    }

    private fun syncAltStepButtons() {
        val pal = GcsTheme.of(hub.themeId)
        val steps = floatArrayOf(0.1f, 0.2f, 0.5f)
        for (i in altStepBtns.indices) {
            styleSegmentSelection(altStepBtns[i], abs(hub.altStepMeters - steps[i]) < 0.03f, pal.accent, pal)
        }
    }

    // ==================== 2. 通信链路 ====================

    private fun buildCategoryComms(): View {
        val sv = ScrollView(context)
        val col = pageColumn()

        val linkCard = groupCard("ROS1 传输协议与双路遥测")
        linkCard.addView(subFieldLabel("首选机载通信协议"))
        val protoSeg = segmentedTrack()
        val pSpecs = listOf("tcpros" to "原生 TCPROS (:11311)", "rosbridge" to "rosbridge (:9090)")
        protoPrefBtns.clear()
        for ((key, label) in pSpecs) {
            val b = segmentBtn(null, label) {
                onProtocolChanged(key)
                syncProtoPrefButtons()
                refreshTopicSummaryBox()
            }
            protoPrefBtns.add(b)
            protoSeg.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        linkCard.addView(protoSeg, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { bottomMargin = dp(4f) })

        swAutoConn = addSwitchRow(
            linkCard,
            Type.SEARCH,
            "启动自动扫描并连接无人机",
            "自动探测同一局域网 11311 / 9090 机载节点并完成话题配对",
            hub.autoConnectDiscovered
        ) { v ->
            hub.autoConnectDiscovered = v
            persistAllSettings(context, hub)
        }
        swMavlinkUdp = addSwitchRow(
            linkCard,
            Type.TELEMETRY_PULSE,
            "ELRS Backpack 电池遥测监听 (:14550)",
            "Wi-Fi 断开时仍可通过遥控内置高频头接收飞机电压与电流",
            hub.mavlinkBackpackEnabled,
            showDivider = false
        ) { v ->
            hub.mavlinkBackpackEnabled = v
            persistAllSettings(context, hub)
        }
        col.addView(linkCard)

        val tplCard = groupCard("机载算法栈话题模板")
        val tplRow = LinearLayout(context).apply { orientation = LinearLayout.HORIZONTAL }
        val tpls = listOf("tilt_indi" to "Tilt-INDI", "fastlio" to "FAST-LIO", "vins" to "VINS", "swarm0" to "Swarm_0")
        for (i in tpls.indices) {
            val (k, name) = tpls[i]
            tplRow.addView(
                actionBtn(Type.LAYERS, name) {
                    onApplyTopicPreset(k)
                    refreshTopicSummaryBox()
                },
                LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply {
                    if (i > 0) leftMargin = dp(5f)
                }
            )
        }
        tplCard.addView(tplRow)

        tvTopicSummaryBox = TextView(context).apply {
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.5f)
            typeface = Typeface.MONOSPACE
            setLineSpacing(dp(1.8f).toFloat(), 1.0f)
            setPadding(dp(8f), dp(6f), dp(8f), dp(6f))
            GcsTheme.markRole(this, Role.CONSOLE_BOX, cornerDp = 6f)
        }
        tplCard.addView(tvTopicSummaryBox, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(6f) })

        val jumpBtn = actionBtn(Type.LINK, "打开连接面板修改 IP 与详细话题", trailing = Type.CHEVRON_RIGHT) {
            setVisible(false)
            onJumpToConnDrawer()
        }
        tplCard.addView(jumpBtn, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(6f) })
        col.addView(tplCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        sv.addView(col)
        return sv
    }

    private fun syncProtoPrefButtons() {
        val pal = GcsTheme.of(hub.themeId)
        val cur = getCurrentTopicConfig().protocol
        val keys = arrayOf("tcpros", "rosbridge")
        for (i in protoPrefBtns.indices) {
            styleSegmentSelection(protoPrefBtns[i], cur == keys[i], pal.accent, pal)
        }
    }

    private fun refreshTopicSummaryBox() {
        if (!::tvTopicSummaryBox.isInitialized) return
        val pal = GcsTheme.of(hub.themeId)
        tvTopicSummaryBox.setTextColor(pal.textConsole)
        val c = getCurrentTopicConfig()
        val endpoint = if (c.protocol == "tcpros") "TCPROS  ${c.masterUri}" else "WS      ${c.wsUrl}"
        val txt = "Endpoint : $endpoint\n" +
            "Odom/Map : ${c.odomTopic}  |  ${c.inflateTopic}\n" +
            "Traj/Goal: ${c.optimalTopic}  ->  ${c.goalTopic}\n" +
            "Tilt/Cmd : ${c.tiltTelemetryTopic}  |  ${c.tiltCmdTopic}"
        if (!TextUtils.equals(tvTopicSummaryBox.text, txt)) {
            tvTopicSummaryBox.text = txt
        }
    }

    // ==================== 3. 显示与通用 (含 GitHub 直链 OTA 在线更新) ====================

    private fun buildCategoryGeneral(): View {
        generalScrollView = ScrollView(context)
        val col = pageColumn()

        // ---- 卡片 A：软件版本与 GitHub Releases 直链在线更新 ----
        val otaCard = groupCard("软件版本与 GitHub 直链在线更新")

        tvOtaConsoleBox = TextView(context).apply {
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.3f)
            typeface = Typeface.MONOSPACE
            setLineSpacing(dp(1.8f).toFloat(), 1.0f)
            setPadding(dp(8f), dp(6f), dp(8f), dp(6f))
            GcsTheme.markRole(this, Role.CONSOLE_BOX, cornerDp = 6f)
        }
        otaCard.addView(tvOtaConsoleBox)

        // 平滑圆角下载进度条轨道 (采用 weightSum=100 线性权重布局，杜绝首次显示时宽度为 0 的测量竞态)
        otaProgressTrack = LinearLayout(context).apply {
            orientation = LinearLayout.HORIZONTAL
            weightSum = 100f
            visibility = View.GONE
        }
        otaProgressFill = View(context)
        otaProgressTrack.addView(otaProgressFill, LinearLayout.LayoutParams(0, MATCH_PARENT, 5f))
        otaCard.addView(
            otaProgressTrack,
            LinearLayout.LayoutParams(MATCH_PARENT, dp(6.5f)).apply {
                topMargin = dp(6f)
            }
        )

        otaCard.addView(subFieldLabel("下载通道模式 (默认自动同步云端活跃加速源并测速优选)"))
        val chanSeg = segmentedTrack()
        val chanSpecs = listOf(
            "智能实时加速",
            "GitHub 直链",
            "自定义源"
        )
        otaChannelBtns.clear()
        for (i in chanSpecs.indices) {
            val b = segmentBtn(null, chanSpecs[i]) {
                hub.otaChannelMode = i
                persistAllSettings(context, hub)
                syncOtaChannelButtons()
                otaManager.onChannelOrUrlChanged()
            }
            otaChannelBtns.add(b)
            chanSeg.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        otaCard.addView(chanSeg)

        // 自定义源输入框 (支持填入完整 APK 直链，或直接填入任意加速源域名前缀)
        customUrlWrap = LinearLayout(context).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            visibility = if (hub.otaChannelMode == 2) View.VISIBLE else View.GONE
        }
        inpCustomOtaUrl = EditText(context).apply {
            setText(hub.otaCustomUrl)
            hint = "输入 APK 直链或任意加速源域名 (如 https://gh-proxy.com/)"
            setSingleLine(true)
            minimumHeight = dp(35f)
            inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.5f)
            typeface = Typeface.MONOSPACE
            setPadding(dp(8f), dp(5f), dp(8f), dp(5f))
            GcsTheme.markRole(this, Role.INPUT_BOX, cornerDp = 6f)
            addTextChangedListener(object : TextWatcher {
                override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}
                override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {}
                override fun afterTextChanged(s: Editable?) {
                    if (suppressCustomUrlWatcher) return
                    val v = s?.toString()?.trim().orEmpty()
                    if (v.isNotEmpty()) {
                        hub.otaCustomUrl = v
                        persistAllSettings(context, hub)
                        otaManager.onChannelOrUrlChanged()
                    }
                }
            })
        }
        val btnResetUrl = actionBtn(Type.UNDO, "默认直链") {
            hub.otaCustomUrl = OtaUpdateManager.OFFICIAL_GITHUB_APK_URL
            suppressCustomUrlWatcher = true
            inpCustomOtaUrl.setText(hub.otaCustomUrl)
            suppressCustomUrlWatcher = false
            persistAllSettings(context, hub)
            otaManager.onChannelOrUrlChanged()
            onShowToast("已恢复默认 GitHub latest-apk 直链")
        }
        customUrlWrap.addView(inpCustomOtaUrl, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        customUrlWrap.addView(
            btnResetUrl,
            LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply {
                leftMargin = dp(5f)
            }
        )
        otaCard.addView(
            customUrlWrap,
            LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply {
                topMargin = dp(5f)
            }
        )

        // 实时刷新加速源测速 + 检查更新 + 直链下载覆盖安装三按钮操作行
        val otaActionRow = LinearLayout(context).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }
        btnOtaSyncMirrors = actionBtn(Type.RADAR_SIGNAL, "刷新加速源") {
            otaManager.refreshLiveMirrors(manualTrigger = true)
        }
        btnOtaCheck = actionBtn(Type.REFRESH, "检查更新") {
            otaManager.checkForUpdate(silentOnUpToDate = false)
        }
        btnOtaPrimaryDownload = TextView(context).apply {
            val pal = GcsTheme.of(hub.themeId)
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            ellipsize = TextUtils.TruncateAt.END
            minimumHeight = dp(32f)
            setTextColor(Color.WHITE)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(8f), dp(5.5f), dp(8f), dp(5.5f))
            background = pillBg(pal.accent, pal.accentStroke, radiusDp = 6f)
            GcsIcons.bind(this, Type.DOWNLOAD, "直链下载更新", d, iconDp = 11.5f, gapDp = 3.5f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                otaManager.startDownloadOrCancelOrInstall()
            }
        }
        AnimHelper.attachSpringTouch(btnOtaPrimaryDownload)

        otaActionRow.addView(btnOtaSyncMirrors, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1.0f))
        otaActionRow.addView(
            btnOtaCheck,
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 0.95f).apply {
                leftMargin = dp(5f)
            }
        )
        otaActionRow.addView(
            btnOtaPrimaryDownload,
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1.32f).apply {
                leftMargin = dp(5f)
            }
        )
        otaCard.addView(
            otaActionRow,
            LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply {
                topMargin = dp(7f)
            }
        )

        swOtaAutoCheck = addSwitchRow(
            otaCard,
            Type.RADAR_SIGNAL,
            "启动地面站时后台检测新构建",
            "联网状态下自动比对 GitHub latest-apk 发布时间并提示可用更新",
            hub.otaAutoCheckOnStart,
            showDivider = false
        ) { v ->
            hub.otaAutoCheckOnStart = v
            persistAllSettings(context, hub)
        }
        col.addView(otaCard)

        // ---- 卡片 B：显示主题与渲染功耗策略 ----
        val dispCard = groupCard("显示主题与渲染功耗策略")
        dispCard.addView(subFieldLabel("环境光照主题 (户外强光推荐「强光白」)"))
        val themeSeg = segmentedTrack()
        val shortThemes = listOf("深空黑", "强光白", "夜视绿", "琥珀金")
        themeSegBtns.clear()
        for (i in shortThemes.indices) {
            val b = segmentBtn(null, shortThemes[i]) {
                if (hub.themeId != i) {
                    hub.themeId = i
                    persistAllSettings(context, hub)
                    onThemeChanged(true)
                }
            }
            themeSegBtns.add(b)
            themeSeg.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        dispCard.addView(themeSeg, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { bottomMargin = dp(7f) })

        dispCard.addView(subFieldLabel("帧率与功耗档位"))
        val perfSeg = segmentedTrack()
        val perfSpecs = listOf(
            Triple(Type.GAUGE_MID, "均衡 60帧", "推荐：满帧 60FPS + 视锥裁剪"),
            Triple(Type.GAUGE_HIGH, "极致画质", "全量 4096 膨胀体素渲染"),
            Triple(Type.GAUGE_LOW, "省电 30帧", "户外长续航：限帧 30FPS 降温省电")
        )
        perfModeSegBtns.clear()
        for (i in perfSpecs.indices) {
            val (ic, title, tip) = perfSpecs[i]
            val b = segmentBtn(ic, title) {
                hub.perfMode = i
                persistAllSettings(context, hub)
                syncPerfModeButtons()
                refreshSystemDiagnostics()
                onShowToast("已切换帧率档位：$tip")
            }
            perfModeSegBtns.add(b)
            perfSeg.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        dispCard.addView(perfSeg)
        col.addView(dispCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        // ---- 卡片 C：三维地图辅助显示 ----
        val mapCard = groupCard("三维地图辅助显示")
        swXray = addSwitchRow(
            mapCard,
            Type.XRAY_EYE,
            "障碍物遮挡视线自动透视",
            "挡住飞机的膨胀体素执行单外皮椭球透镜消隐，被挡航线变流动虚线",
            hub.xrayOcclusionEnabled
        ) { v ->
            hub.xrayOcclusionEnabled = v
            persistAllSettings(context, hub)
            onVisualSettingsChanged()
        }
        swHaptics = addSwitchRow(
            mapCard,
            Type.SPARK_MOTION,
            "触控震动反馈",
            "选点、返航与紧急悬停时触发遥控器短促触觉反馈",
            hub.hapticFeedbackEnabled,
            showDivider = false
        ) { v ->
            hub.hapticFeedbackEnabled = v
            AnimHelper.hapticsEnabled = v
            persistAllSettings(context, hub)
        }

        mapCard.addView(subFieldLabel("FAST-LIO 点云建图体素分辨率"))
        val voxSeg = segmentedTrack()
        val voxChoices = listOf(0.08f to "精细 0.08m", 0.12f to "标准 0.12m", 0.20f to "轻量 0.20m")
        voxelResBtns.clear()
        for ((res, label) in voxChoices) {
            val b = segmentBtn(null, label) {
                hub.mapVoxelSize = res
                persistAllSettings(context, hub)
                syncVoxelResButtons()
                onVisualSettingsChanged()
            }
            voxelResBtns.add(b)
            voxSeg.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        mapCard.addView(voxSeg)
        col.addView(mapCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        // ---- 卡片 D：设备运行状态与数据维护 ----
        val sysCard = groupCard("设备运行状态与数据维护")
        tvSysDiagSummary = TextView(context).apply {
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.5f)
            typeface = Typeface.MONOSPACE
            setLineSpacing(dp(1.8f).toFloat(), 1.0f)
            setPadding(dp(8f), dp(6f), dp(8f), dp(6f))
            GcsTheme.markRole(this, Role.CONSOLE_BOX, cornerDp = 6f)
        }
        sysCard.addView(tvSysDiagSummary)

        val maintRow = LinearLayout(context).apply { orientation = LinearLayout.HORIZONTAL }
        maintRow.addView(
            actionBtn(Type.DOWNLOAD, "导出点云") {
                onExportPcd()
            },
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f)
        )
        maintRow.addView(
            actionBtn(Type.ERASER, "清空建图") {
                onClearMapAndTrail()
                refreshSystemDiagnostics()
            },
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dp(5f) }
        )

        btnResetFactory = TextView(context).apply {
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            ellipsize = TextUtils.TruncateAt.END
            minimumHeight = dp(30f)
            setTextColor(Color.parseColor("#FF5252"))
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(8f), dp(5f), dp(8f), dp(5f))
            background = pillBg(Color.argb(42, 255, 59, 48), Color.parseColor("#FF3B30"), radiusDp = 6f)
            GcsIcons.bind(this, Type.UNDO, "恢复默认", d, iconDp = 11.5f)
            setOnClickListener { handleFactoryResetTap() }
        }
        AnimHelper.attachSpringTouch(btnResetFactory)
        maintRow.addView(btnResetFactory, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1.05f).apply { leftMargin = dp(5f) })
        sysCard.addView(maintRow, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(7f) })
        col.addView(sysCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        generalScrollView.addView(col)
        return generalScrollView
    }

    private fun syncOtaChannelButtons() {
        val pal = GcsTheme.of(hub.themeId)
        for (i in otaChannelBtns.indices) {
            styleSegmentSelection(otaChannelBtns[i], hub.otaChannelMode == i, pal.accent, pal)
        }
        if (::customUrlWrap.isInitialized) {
            customUrlWrap.visibility = if (hub.otaChannelMode == 2) View.VISIBLE else View.GONE
        }
    }

    private fun syncThemeSegButtons() {
        val pal = GcsTheme.of(hub.themeId)
        for (i in themeSegBtns.indices) {
            val swatch = GcsTheme.PALETTES[i].swatchColor
            styleSegmentSelection(themeSegBtns[i], hub.themeId == i, swatch, pal)
        }
    }

    private fun syncPerfModeButtons() {
        val pal = GcsTheme.of(hub.themeId)
        for (i in perfModeSegBtns.indices) {
            styleSegmentSelection(perfModeSegBtns[i], hub.perfMode == i, pal.accent, pal)
        }
    }

    private fun syncVoxelResButtons() {
        val pal = GcsTheme.of(hub.themeId)
        val vals = floatArrayOf(0.08f, 0.12f, 0.20f)
        for (i in voxelResBtns.indices) {
            styleSegmentSelection(voxelResBtns[i], abs(hub.mapVoxelSize - vals[i]) < 0.02f, pal.accent, pal)
        }
    }

    private fun refreshSystemDiagnostics() {
        if (!::tvSysDiagSummary.isInitialized) return
        val pal = GcsTheme.of(hub.themeId)
        tvSysDiagSummary.setTextColor(pal.textConsole)
        val dm = context.resources.displayMetrics
        val s = hub.snapshots.readInto(readSnapshot)
        val rt = Runtime.getRuntime()
        val usedMb = ((rt.totalMemory() - rt.freeMemory()) / (1024L * 1024L)).toInt()
        val maxMb = (rt.maxMemory() / (1024L * 1024L)).toInt()
        val modeTxt = if (hub.isRosMode) "ROS1 (${hub.connectionLabel})" else "本地仿真"
        val linkRateInfo = if (hub.isRosMode) {
            String.format(Locale.US, "  |  吞吐 %.1f KB/s (自愈%d)", hub.rosRxKbps, hub.rosReconnectCount)
        } else {
            ""
        }
        val txt = String.format(
            Locale.US,
            "应用版本 : 智航 v3.12.0  |  %d FPS (%.2f ms)  |  内存 %d/%d MB\n" +
                "模式与栈 : %s%s  |  推力裕度 %d%%\n" +
                "体素点云 : 膨胀 %d blks  |  点云 %d pts  |  分辨率 %dx%d\n" +
                "双路电源 : 飞机 %.2fV (%.1fA, %d%%)  |  遥控本机 %.2fV (%d%%)",
            hub.renderFps, hub.frameRenderMs, usedMb, maxMb,
            modeTxt, linkRateInfo, s.tiltThrustMarginPct,
            s.voxelCount, s.mapPointCount, dm.widthPixels, dm.heightPixels,
            s.batVolt, s.batCurrentAmps, s.batPct, s.txBatVolt, s.txBatPct
        )
        if (!TextUtils.equals(tvSysDiagSummary.text, txt)) {
            tvSysDiagSummary.text = txt
        }
    }

    private fun handleFactoryResetTap() {
        if (!factoryResetConfirmPending) {
            factoryResetConfirmPending = true
            AnimHelper.triggerHaptic(btnResetFactory)
            GcsIcons.bind(btnResetFactory, Type.WARNING, "再点重置", d, iconDp = 11.5f)
            btnResetFactory.background = pillBg(Color.argb(145, 255, 59, 48), Color.parseColor("#FFD54F"), radiusDp = 6f)
            uiHandler.removeCallbacks(cancelFactoryConfirmRunnable)
            uiHandler.postDelayed(cancelFactoryConfirmRunnable, 2500L)
            return
        }
        uiHandler.removeCallbacks(cancelFactoryConfirmRunnable)
        cancelFactoryConfirmRunnable.run()
        AnimHelper.triggerHaptic(btnResetFactory, HapticFeedbackConstants.LONG_PRESS)

        hub.themeId = GcsTheme.THEME_AERO_CARBON
        hub.perfMode = 0
        hub.obstaclesInflation = 0.30f
        hub.maxVel = 2.0f
        hub.maxAcc = 3.0f
        hub.goalZ = 1.2f
        hub.autoConnectDiscovered = true
        hub.mavlinkBackpackEnabled = true
        hub.xrayOcclusionEnabled = true
        hub.showFastLioMap = true
        hub.showInflate = true
        hub.showLocalBox = true
        hub.showCtrlPts = true
        hub.pointColorMode = 0
        hub.mapVoxelSize = 0.12f
        hub.zSliceMax = 3.0f
        hub.longPressQuickFlyEnabled = true
        hub.rthDoubleConfirm = true
        hub.hapticFeedbackEnabled = true
        AnimHelper.hapticsEnabled = true
        hub.altStepMeters = 0.2f
        hub.warnClearanceMeters = 0.65f
        hub.lowBatWarnPct = 25
        hub.routeLoopMode = 0
        hub.routeFinishAction = 0
        hub.swarmModeEnabled = false
        hub.swarmDroneCount = 3
        hub.selectedDroneId = 0
        hub.swarmCommandScope = 1
        hub.swarmFormationType = 0
        hub.swarmSpacingMeters = 1.8f
        hub.showSwarmLinks = true
        hub.otaChannelMode = 0
        hub.otaCustomUrl = OtaUpdateManager.OFFICIAL_GITHUB_APK_URL
        hub.otaAutoCheckOnStart = true

        persistAllSettings(context, hub)
        otaManager.onChannelOrUrlChanged()
        onThemeChanged(true)
        syncAllControlsFromHub()
        onFactoryResetApplied()
        onShowToast("已恢复全部飞控与显示默认设置")
    }

    // ==================== 原子组件构建工具 ====================

    private fun dp(v: Float): Int = (v * d).roundToInt()

    private fun pageColumn(): LinearLayout = LinearLayout(context).apply {
        orientation = LinearLayout.VERTICAL
        setPadding(dp(11f), dp(9f), dp(11f), dp(11f))
    }

    private fun groupCard(title: String): LinearLayout = LinearLayout(context).apply {
        orientation = LinearLayout.VERTICAL
        setPadding(dp(10f), dp(8.5f), dp(10f), dp(9f))
        GcsTheme.markRole(this, Role.SECTION_CARD, cornerDp = 8.5f)
        val hdr = LinearLayout(context).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setPadding(0, 0, 0, dp(6f))
        }
        val bar = View(context).apply {
            GcsTheme.markRole(this, Role.ACCENT_BAR, cornerDp = 2f)
        }
        val tv = TextView(context).apply {
            text = title
            maxLines = 2
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.6f)
            typeface = Typeface.DEFAULT_BOLD
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
        }
        hdr.addView(bar, LinearLayout.LayoutParams(dp(3f), dp(11.5f)).apply { rightMargin = dp(6f) })
        hdr.addView(tv, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        addView(hdr)
    }

    private fun subFieldLabel(txt: String): TextView = TextView(context).apply {
        text = txt
        setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
        setPadding(0, dp(4.5f), 0, dp(3.5f))
        GcsTheme.markRole(this, Role.TEXT_MUTED)
    }

    private fun segmentedTrack(): LinearLayout = LinearLayout(context).apply {
        orientation = LinearLayout.HORIZONTAL
        gravity = Gravity.CENTER_VERTICAL
        setPadding(dp(2.5f), dp(2.5f), dp(2.5f), dp(2.5f))
        GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6.5f)
    }

    private fun segmentBtn(icon: Type?, title: String, onClick: () -> Unit): TextView = TextView(context).apply {
        gravity = Gravity.CENTER
        maxLines = 1
        setHorizontallyScrolling(false)
        minimumHeight = dp(30f)
        setPadding(dp(5f), dp(5f), dp(5f), dp(5f))
        setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.8f)
        typeface = Typeface.DEFAULT_BOLD
        GcsIcons.bind(this, icon, title, d, iconDp = 11f, gapDp = 3f)
        setOnClickListener {
            AnimHelper.triggerHaptic(it)
            onClick()
        }
    }

    private fun styleSegmentSelection(btn: TextView, selected: Boolean, activeColor: Int, pal: GcsTheme.Palette) {
        btn.setTextColor(if (selected) Color.WHITE else pal.textMuted)
        btn.background = if (selected) {
            GradientDrawable().apply {
                setColor(activeColor)
                cornerRadius = dp(5f).toFloat()
                setStroke(1, Color.argb(105, 255, 255, 255))
            }
        } else {
            roundedColor(Color.TRANSPARENT, dp(5f))
        }
        btn.invalidate()
    }

    private fun addSwitchRow(
        parent: LinearLayout,
        icon: Type,
        title: String,
        desc: String,
        initial: Boolean,
        showDivider: Boolean = true,
        onToggle: (Boolean) -> Unit
    ): ToggleSyncHandle {
        var cur = initial
        val row = LinearLayout(context).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setPadding(0, dp(5f), 0, dp(5f))
        }
        val textCol = LinearLayout(context).apply { orientation = LinearLayout.VERTICAL }
        val tvTitle = TextView(context).apply {
            maxLines = 2
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.5f)
            typeface = Typeface.DEFAULT_BOLD
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
            GcsIcons.bind(this, icon, title, d, iconDp = 12f, gapDp = 4.5f)
        }
        val tvDesc = TextView(context).apply {
            text = desc
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.0f)
            setPadding(0, dp(2f), dp(8f), 0)
            GcsTheme.markRole(this, Role.TEXT_MUTED)
        }
        textCol.addView(tvTitle)
        textCol.addView(tvDesc)

        val pillSwitch = FrameLayout(context)
        val thumb = View(context).apply {
            background = roundedColor(Color.WHITE, dp(8.5f))
        }
        pillSwitch.addView(
            thumb,
            FrameLayout.LayoutParams(dp(15f), dp(15f), Gravity.CENTER_VERTICAL).apply {
                leftMargin = dp(2.5f)
            }
        )

        fun renderState(animate: Boolean) {
            val pal = GcsTheme.of(hub.themeId)
            pillSwitch.background = pillBg(
                if (cur) pal.accent else pal.wellFill,
                if (cur) pal.accentStroke else pal.cardStroke,
                radiusDp = 11f
            )
            val targetTx = if (cur) dp(16f).toFloat() else 0f
            if (animate) {
                SpringAnimation(thumb, DynamicAnimation.TRANSLATION_X, targetTx).apply {
                    spring = SpringForce(targetTx).apply {
                        stiffness = 520f
                        dampingRatio = 0.78f
                    }
                    start()
                }
            } else {
                thumb.translationX = targetTx
            }
        }
        renderState(false)

        val toggleAction = {
            cur = !cur
            AnimHelper.triggerHaptic(row)
            renderState(true)
            onToggle(cur)
        }
        row.setOnClickListener { toggleAction() }
        pillSwitch.setOnClickListener { toggleAction() }

        row.addView(textCol, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        row.addView(pillSwitch, LinearLayout.LayoutParams(dp(36f), dp(20f)))
        parent.addView(row)

        if (showDivider) {
            parent.addView(
                View(context).apply {
                    GcsTheme.markRole(this, Role.DIVIDER)
                },
                LinearLayout.LayoutParams(MATCH_PARENT, 1)
            )
        }

        val handle = object : ToggleSyncHandle {
            override fun syncFromState(newVal: Boolean) {
                if (cur != newVal) {
                    cur = newVal
                    renderState(false)
                }
            }

            override fun refreshThemeColors() {
                renderState(false)
            }
        }
        allToggleHandles.add(handle)
        return handle
    }

    private fun addSliderRow(
        parent: LinearLayout,
        label: String,
        minI: Int,
        maxI: Int,
        initI: Int,
        scale: Float,
        fmt: String,
        onChanged: (Float) -> Unit
    ): SliderHandle {
        val wrap = LinearLayout(context).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(0, dp(4f), 0, dp(3.5f))
        }
        val top = LinearLayout(context).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }
        val tvLbl = TextView(context).apply {
            text = label
            maxLines = 2
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.1f)
            GcsTheme.markRole(this, Role.TEXT_SECONDARY)
        }
        val safeInit = initI.coerceIn(minI, maxI)
        val tvVal = TextView(context).apply {
            text = String.format(Locale.US, fmt, safeInit * scale)
            maxLines = 1
            gravity = Gravity.CENTER
            minWidth = dp(52f)
            setHorizontallyScrolling(false)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
            GcsTheme.applyTabularReadout(this)
            setPadding(dp(6f), dp(1.5f), dp(6f), dp(1.5f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 4.5f)
            setTextColor(GcsTheme.of(hub.themeId).accentBright)
        }
        allSliderValBadges.add(tvVal)
        top.addView(tvLbl, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { rightMargin = dp(6f) })
        top.addView(tvVal)
        wrap.addView(top)

        val sb = SeekBar(context).apply {
            max = maxI - minI
            progress = safeInit - minI
            minimumHeight = dp(24f)
            setPadding(dp(7f), dp(3f), dp(7f), dp(3f))
            GcsTheme.styleAvionicsSeekBar(this, GcsTheme.of(hub.themeId), d)
            setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
                override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                    if (!fromUser) return
                    val v = (minI + progress) * scale
                    tvVal.text = String.format(Locale.US, fmt, v)
                    onChanged(v)
                }
                override fun onStartTrackingTouch(seekBar: SeekBar?) {}
                override fun onStopTrackingTouch(seekBar: SeekBar?) {}
            })
        }
        allSeekBars.add(sb)
        wrap.addView(sb, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(1.5f) })
        parent.addView(wrap)
        return SliderHandle(sb, tvVal, minI, scale, fmt)
    }

    private fun actionBtn(
        icon: Type,
        label: String,
        trailing: Type? = null,
        onClick: () -> Unit
    ): TextView = TextView(context).apply {
        val pal = GcsTheme.of(hub.themeId)
        gravity = Gravity.CENTER
        maxLines = 1
        setHorizontallyScrolling(false)
        minimumHeight = dp(32f)
        setTextColor(pal.textPrimary)
        setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
        typeface = Typeface.DEFAULT_BOLD
        setPadding(dp(8f), dp(5.5f), dp(8f), dp(5.5f))
        background = pillBg(pal.cardFill, pal.cardStroke, radiusDp = 6f)
        GcsIcons.bind(this, icon, label, d, iconDp = 11.5f, gapDp = 4f, trailing = trailing, trailDp = 9.5f)
        setOnClickListener {
            AnimHelper.triggerHaptic(it)
            onClick()
        }
        AnimHelper.attachSpringTouch(this)
        allActionBtns.add(this)
    }

    private fun roundedColor(col: Int, radiusPx: Int): GradientDrawable = GradientDrawable().apply {
        setColor(col)
        cornerRadius = radiusPx.toFloat()
    }

    private fun pillBg(fillCol: Int, strokeCol: Int, radiusDp: Float = 16f): GradientDrawable = GradientDrawable().apply {
        setColor(fillCol)
        cornerRadius = dp(radiusDp).toFloat()
        setStroke(maxOf(1, dp(0.9f)), strokeCol)
    }
}
