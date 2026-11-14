package com.radiomaster.egogcs

import android.app.Activity
import android.content.ClipData
import android.content.ClipboardManager
import android.content.Context
import android.content.res.Configuration
import android.graphics.Color
import android.graphics.Typeface
import android.graphics.drawable.GradientDrawable
import android.graphics.drawable.InsetDrawable
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.text.Editable
import android.text.InputType
import android.text.TextUtils
import android.text.TextWatcher
import android.util.TypedValue
import android.view.Gravity
import android.view.HapticFeedbackConstants
import android.view.KeyEvent
import android.view.View
import android.view.ViewGroup.LayoutParams.MATCH_PARENT
import android.view.ViewGroup.LayoutParams.WRAP_CONTENT
import android.view.WindowManager
import android.view.inputmethod.InputMethodManager
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import com.radiomaster.egogcs.engine.EgoSimEngine
import com.radiomaster.egogcs.hw.Ax12HardwareBridge
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.RenderSnapshot
import com.radiomaster.egogcs.model.SwarmPalette
import com.radiomaster.egogcs.model.TiltConfigSpec
import com.radiomaster.egogcs.model.TopicConfig
import com.radiomaster.egogcs.ota.OtaUpdateManager
import com.radiomaster.egogcs.ros.DiscoveredDrone
import com.radiomaster.egogcs.ros.LanDroneScanner
import com.radiomaster.egogcs.ros.RosBridgeClient
import com.radiomaster.egogcs.ros.RosConnectionManager
import com.radiomaster.egogcs.ros.RosTopicEntry
import com.radiomaster.egogcs.ros.RosTopicInspector
import com.radiomaster.egogcs.ros.RosUdpTelemetryClient
import com.radiomaster.egogcs.ros.TcpRosNativeClient
import com.radiomaster.egogcs.ros.TopicEchoSnapshot
import com.radiomaster.egogcs.ros.TopicEchoState
import com.radiomaster.egogcs.ui.AnimHelper
import com.radiomaster.egogcs.ui.DjiCompassTapeView
import com.radiomaster.egogcs.ui.DjiHsiRadarView
import com.radiomaster.egogcs.ui.DroneVectorModel
import com.radiomaster.egogcs.ui.FastHudFormatter
import com.radiomaster.egogcs.ui.GcsIcons
import com.radiomaster.egogcs.ui.GcsIcons.Type
import com.radiomaster.egogcs.ui.GcsSettingsModal
import com.radiomaster.egogcs.ui.GcsTheme
import com.radiomaster.egogcs.ui.GcsTheme.Role
import com.radiomaster.egogcs.ui.GlPointCloudMapView
import com.radiomaster.egogcs.ui.UavAttitudeHudView
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.Executors
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sqrt

/**
 * 全界面响应式防重叠对齐 + 精简务实飞控设置版地面站主控座舱 (`MainActivity`)：
 * 1. 五区无碰撞航空布局：紧凑 36dp 顶栏、顶部居中统一堆栈 (航向罗盘 + 回中/遮挡提示 + 消息横幅永不互叠)、
 *    左侧安全操控栏、右侧紧凑相机条与对称等高 94dp 双仪表底栏 (HSI 避障球 + 三维实时姿态视窗)；
 * 2. 空间互斥协调器 (`syncBottomHudAvoidance`)：窄横屏下弹出底部选点指令坞或展开宽屏抽屉时，自动让位次级数字面板，
 *    从几何上彻底根除 `640×360dp` 至 `853×480dp` 全分辨率范围内的任何控件重叠或错位；
 * 3. 对标 DJI Pilot 2 / DJI Fly / QGroundControl 的四大核心飞控设置（飞行安全 · 感知与飞控 · 通信链路 · 显示与通用）。
 */
class MainActivity : Activity() {

    private val readSnapshot = RenderSnapshot()

    private val hub = GcsSharedHub()
    private val uiHandler = Handler(Looper.getMainLooper())
    private lateinit var simEngine: EgoSimEngine
    private lateinit var hwBridge: Ax12HardwareBridge
    private lateinit var rosManager: RosConnectionManager
    private lateinit var topicInspector: RosTopicInspector
    private lateinit var otaManager: OtaUpdateManager
    private lateinit var settingsModal: GcsSettingsModal
    private val lanScanner = LanDroneScanner()
    private val fileIoExecutor = Executors.newSingleThreadExecutor { r ->
        Thread(r, "PcdExportIO").apply { isDaemon = true }
    }

    private lateinit var cockpitRoot: FrameLayout
    private lateinit var topBarView: LinearLayout
    private lateinit var topCenterStack: LinearLayout
    private lateinit var leftSafetyRail: LinearLayout
    private lateinit var bottomLeftCluster: LinearLayout

    private lateinit var mapView: GlPointCloudMapView
    private lateinit var hsiView: DjiHsiRadarView
    private lateinit var attitudeHudView: UavAttitudeHudView
    private lateinit var hdTelemetryBox: LinearLayout
    private lateinit var rightCamStack: LinearLayout
    private lateinit var compassView: DjiCompassTapeView
    private lateinit var recenterBanner: TextView
    private lateinit var occlusionBadge: TextView
    private lateinit var toastBanner: TextView

    private lateinit var topBatFill: View
    private lateinit var statusAccent: View
    private lateinit var accentGreen: GradientDrawable
    private lateinit var accentAmber: GradientDrawable
    private lateinit var accentRed: GradientDrawable
    private var lastAppliedConnLevel = -1
    private var lastIsRosModeUi = false
    private var lastHudThemeId = -1
    private var lastPx4ModeStr = ""
    private var lastArmedFlag = true
    private var lastConnLabelStr = ""
    private var lastFsmStateStr = ""
    private var lastBatBarPx = -1
    private var lastOccludedHudState = false
    private var lastBatDangerLevel = -1
    private var lastClearanceWarnLevel = -1

    // 顶栏精简控件
    private lateinit var tvModeBadge: TextView
    private lateinit var tvConnStatus: TextView
    private lateinit var quickSlotsContainer: LinearLayout
    private var quickPillSwarm: TextView? = null
    private var quickPillRig: TextView? = null
    private var quickPillMission: TextView? = null
    private var quickPillTopics: TextView? = null

    // 顶部中央多机集群战术切换条 (仅在开启多机模式时优雅展开，位于罗盘正下方零重叠)
    private lateinit var swarmHudBar: LinearLayout
    private lateinit var btnSwarmScopeHud: TextView
    private val swarmDroneHudChips = mutableListOf<TextView>()
    private lateinit var btnSwarmFormHud: TextView

    // 抽屉编队/连接/状态页多机控件
    private val swarmScaleSegBtns = mutableListOf<TextView>()
    private val swarmFormSegBtns = mutableListOf<TextView>()
    private val swarmSpaceSegBtns = mutableListOf<TextView>()
    private lateinit var btnToggleSwarmMesh: TextView
    private var segSwarmTopoNsBtn: TextView? = null
    private var segSwarmTopoIpBtn: TextView? = null
    private var wrapSwarmNsFields: LinearLayout? = null
    private var wrapSwarmIpFields: LinearLayout? = null
    private lateinit var inpSwarmPeerIps: EditText
    private lateinit var inpSwarmOdomPat: EditText
    private lateinit var inpSwarmOptPat: EditText
    private lateinit var inpSwarmGoalPat: EditText
    private lateinit var inpSwarmTiltSubPat: EditText
    private lateinit var inpSwarmTiltCmdPat: EditText
    private lateinit var tvSwarmFleetConsole: TextView
    private lateinit var tvTiltIndiConsole: TextView
    private val modeMatrixQuickBtns = mutableListOf<TextView>()
    private val swarmRigPolicyBtns = mutableListOf<TextView>()
    private val swarmPerDroneRigBtns = mutableListOf<TextView>()
    private var wrapSwarmMixedMaskRow: LinearLayout? = null
    private var wrapSwarmFleetRigSection: LinearLayout? = null
    private val tiltModeDrawerBtns = mutableListOf<TextView>()
    private val tiltBiasDrawerBtns = mutableListOf<TextView>()
    private var tvTiltCardHint: TextView? = null
    private lateinit var btnToggleTiltRig: TextView
    private lateinit var inpTiltTelemetry: EditText
    private lateinit var inpTiltCmd: EditText
    private lateinit var tvClearanceTop: TextView
    private lateinit var topClusterDivHz: View
    private lateinit var tvHzTop: TextView
    private lateinit var topClusterDivBat: View
    private lateinit var tvBatTop: TextView
    private lateinit var btnMenuToggle: TextView
    private lateinit var btnSettingsToggle: ImageView
    private var lastArmedBadgeStyled = -1
    private var safeCutoutLeftPx = 0
    private var safeCutoutRightPx = 0

    // 左侧飞行安全操控栏
    private lateinit var btnBrake: TextView
    private lateinit var btnRth: TextView
    private lateinit var demoControlGroup: LinearLayout
    private lateinit var btnPatrol: TextView
    private lateinit var btnDemoScene: TextView

    // 底部中央智能选点/航线指令坞
    private lateinit var commandDock: LinearLayout
    private lateinit var tvDockHeader: TextView
    private lateinit var dockPinRow: LinearLayout
    private lateinit var tvDockAltVal: TextView
    private lateinit var btnDockFlyNow: TextView
    private lateinit var dockRouteRow: LinearLayout
    private lateinit var btnDockStartRoute: TextView
    private lateinit var btnDockUndo: TextView
    private lateinit var btnDockOpenOrchestrate: TextView
    private lateinit var btnDockClearRoute: TextView
    private var dockLogicalActive = false

    // 右侧精简相机栏 + 图层快捷浮窗
    private lateinit var btn3DRef: TextView
    private lateinit var btnFollowRef: TextView
    private lateinit var btnXrayRef: TextView
    private lateinit var btnLayerPopupToggle: TextView
    private lateinit var layerQuickPopover: LinearLayout
    private var layerPopoverOpen = false
    private val colorSegBtns = mutableListOf<TextView>()
    private lateinit var popSwitchInflate: TextView
    private lateinit var popSwitchCloud: TextView
    private lateinit var popSwitchBox: TextView
    private lateinit var popSwitchCtrl: TextView
    private lateinit var popSwitchTouchLock: TextView
    private lateinit var popSwitchNarrowAssist: TextView
    private lateinit var btnClearMapInPopover: TextView

    private var rthConfirmPending = false
    private val resetRthConfirmRunnable = Runnable {
        rthConfirmPending = false
        if (::btnRth.isInitialized) {
            styleRthDefault()
        }
    }

    private var clearMapConfirmPending = false
    private val resetClearMapConfirmRunnable = Runnable {
        clearMapConfirmPending = false
        if (::btnClearMapInPopover.isInitialized) {
            val pal = GcsTheme.of(hub.themeId)
            btnClearMapInPopover.setTextColor(pal.textPrimary)
            GcsIcons.bind(btnClearMapInPopover, Type.TRASH, "清建图", d, iconDp = 11.5f)
            btnClearMapInPopover.background = glassBg(dp(6f))
        }
    }

    private lateinit var tvOsdH: TextView
    private lateinit var tvOsdHs: TextView
    private lateinit var tvOsdD: TextView
    private lateinit var tvOsdMapPts: TextView
    private lateinit var tvOsdSubXyz: TextView
    private lateinit var tvOsdTilt: TextView

    // 四标签主作业抽屉 (连接 | 航线 | 话题 | 状态)
    private lateinit var drawerRoot: LinearLayout
    private lateinit var tabRowRef: LinearLayout
    private lateinit var tabGlider: FrameLayout
    private lateinit var tabGliderBar: View
    private val tabBtns = mutableListOf<TextView>()
    private val tabPanels = mutableListOf<View>()
    private var drawerOpen = false
    private var drawerWideMode = false
    private var activeTabIndex = 0

    // Tab 0 (连接)
    private lateinit var segModeDemoBtn: TextView
    private lateinit var segModeRosBtn: TextView
    private lateinit var demoSectionInConn: LinearLayout
    private lateinit var rosSectionInConn: LinearLayout
    private val demoSceneSegBtns = mutableListOf<TextView>()
    private lateinit var tvScanStatus: TextView
    private lateinit var btnDrawerScan: TextView
    private lateinit var discoveredListContainer: LinearLayout
    private lateinit var segProtoTcpBtn: TextView
    private lateinit var segProtoWsBtn: TextView
    private lateinit var wrapMasterUri: LinearLayout
    private lateinit var wrapWsUrl: LinearLayout
    private lateinit var inpMasterUri: EditText
    private lateinit var inpWsUrl: EditText
    private var udpTelemetryEnabledUi = false
    private lateinit var udpSwitchPill: FrameLayout
    private lateinit var udpSwitchThumb: View
    private lateinit var inpUdpPort: EditText
    private lateinit var tvUdpStatus: TextView
    private lateinit var btnExpandTopicMap: TextView
    private lateinit var topicMapAccordionBody: LinearLayout
    private var topicMapExpanded = false
    private lateinit var inpOdom: EditText
    private lateinit var inpFastLioCloud: EditText
    private lateinit var inpFastLioPath: EditText
    private lateinit var inpInflate: EditText
    private lateinit var inpOptimal: EditText
    private lateinit var inpGoal: EditText

    // Tab 1 (航线编排)
    private val loopModeSegBtns = mutableListOf<TextView>()
    private val finishActSegBtns = mutableListOf<TextView>()
    private lateinit var tvWaypointQueueSummary: TextView
    private lateinit var waypointListContainer: LinearLayout
    private lateinit var btnOrchPrimaryExecute: TextView

    // Tab 2 (话题)
    private lateinit var topicListPage: LinearLayout
    private lateinit var topicDetailPage: LinearLayout
    private lateinit var inpTopicFilter: EditText
    private lateinit var tvTopicSourceHint: TextView
    private lateinit var topicItemsContainer: LinearLayout
    private var cachedTopicEntries: List<RosTopicEntry> = emptyList()
    private var selectedTopicEntry: RosTopicEntry? = null
    private lateinit var tvDetailTopicName: TextView
    private lateinit var tvDetailTopicType: TextView
    private lateinit var btnWideTerminal: TextView
    private lateinit var btnBindTopicChannel: TextView
    private lateinit var tvDetailStatusStrip: TextView
    private lateinit var tvDetailMetricsStrip: TextView
    private lateinit var tvDetailPayloadConsole: TextView
    private lateinit var btnDetailStreamToggle: TextView
    private var isDetailCurrentlyStreaming = false

    // Tab 3 (状态)
    private lateinit var tvUavPowerCard: TextView
    private lateinit var tvTxPowerCard: TextView
    private lateinit var tvMapPtsCard: TextView
    private lateinit var tvCloudRateCard: TextView
    private lateinit var tvMapBoundsCard: TextView
    private lateinit var tvRecorderStatusCard: TextView
    private lateinit var btnToggleFlightRec: TextView
    private lateinit var tvPosX: TextView
    private lateinit var tvPosY: TextView
    private lateinit var tvPosZ: TextView
    private lateinit var tvVelNorm: TextView
    private lateinit var tvRollPitch: TextView
    private lateinit var tvQuatSummary: TextView
    private lateinit var tvYaw: TextView
    private lateinit var tvGoalDist: TextView
    private lateinit var tvClearanceCard: TextView
    private lateinit var tvSolveCard: TextView
    private lateinit var tvVoxelCount: TextView
    private lateinit var tvTrajLen: TextView

    private val discoveredDrones = mutableListOf<DiscoveredDrone>()
    private val sceneCycle = arrayOf("pillars" to "体素柱阵", "betel_gap" to "槟榔窄缝", "corridor" to "错位走廊", "utrap" to "U型陷阱")
    private var sceneIdx = 0

    private var lastBackMs = 0L
    private var d = 1f
    private var screenWidthDp = 640f
    private var screenHeightDp = 360f

    companion object {
        private const val TAB_CONN = 0
        private const val TAB_ORCHESTRATE = 1
        private const val TAB_TOPICS = 2
        private const val TAB_STATUS = 3
        private const val TAB_COUNT = 4
    }

    override fun attachBaseContext(newBase: Context) {
        val orig = newBase.resources.configuration
        if (orig.fontScale != 1.0f) {
            val fixedCfg = Configuration(orig).apply { fontScale = 1.0f }
            super.attachBaseContext(newBase.createConfigurationContext(fixedCfg))
        } else {
            super.attachBaseContext(newBase)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val dm = resources.displayMetrics
        d = dm.density
        screenWidthDp = dm.widthPixels / d
        screenHeightDp = dm.heightPixels / d

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) {
            window.attributes = window.attributes.apply {
                layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES
            }
        }
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        applyImmersiveFullscreen()

        accentGreen = roundedColor(Color.parseColor("#00E676"), dp(2f))
        accentAmber = roundedColor(Color.parseColor("#FFAB00"), dp(2f))
        accentRed = roundedColor(Color.parseColor("#FF3B30"), dp(2f))

        simEngine = EgoSimEngine(hub).apply {
            onRosWaypointAdvanced = { gx, gy, gz, curX, curY ->
                rosManager.publishGoal(gx, gy, gz, curX, curY)
                runOnUiThread {
                    refreshCommandDockUi()
                    refreshWaypointOrchestratorList()
                    AnimHelper.showDjiToast(
                        toastBanner,
                        String.format(Locale.US, "航线推进 -> 下一站 (%.1f, %.1f)", gx, gy),
                        d
                    )
                }
            }
        }

        hwBridge = Ax12HardwareBridge(this, hub, simEngine)

        val wsClient = RosBridgeClient(hub, simEngine) { msg ->
            runOnUiThread { AnimHelper.showDjiToast(toastBanner, msg, d) }
        }
        val tcpClient = TcpRosNativeClient(hub, simEngine) { msg ->
            runOnUiThread { AnimHelper.showDjiToast(toastBanner, msg, d) }
        }
        rosManager = RosConnectionManager(hub, tcpClient, wsClient, RosUdpTelemetryClient(hub, simEngine))
        topicInspector = RosTopicInspector(hub, rosManager, tcpClient, wsClient)

        GcsSettingsModal.loadPersistedSettings(this, hub)
        DroneVectorModel.activeRig = DroneVectorModel.rigFor(hub.activeFocusIsTiltRig())
        simEngine.notifyOperatingModeChanged()
        loadSavedTopics()
        val savedRouteCsv = getSharedPreferences(GcsSettingsModal.PREFS_NAME, Context.MODE_PRIVATE)
            .getString("persistedStagedRouteCsv", "").orEmpty()
        simEngine.restoreStagedWaypointsCsv(savedRouteCsv)

        otaManager = OtaUpdateManager(
            context = this,
            hub = hub,
            onStateChanged = { snap ->
                if (::settingsModal.isInitialized) {
                    settingsModal.renderOtaSnapshot(snap)
                }
            },
            onNotifyToast = { msg ->
                if (::toastBanner.isInitialized) {
                    AnimHelper.showDjiToast(toastBanner, msg, d)
                }
            }
        )

        settingsModal = GcsSettingsModal(
            context = this,
            hub = hub,
            otaManager = otaManager,
            getCurrentTopicConfig = { rosManager.topics },
            onThemeChanged = { animate ->
                applyActiveThemeLive(animate)
                AnimHelper.showDjiToast(toastBanner, "已切换主题：${GcsTheme.of(hub.themeId).name}", d)
            },
            onFlightKinematicsChanged = { inf, vel, acc, gz ->
                applyFlightParametersToEngine(inf, vel, acc, gz)
            },
            onVisualSettingsChanged = {
                syncCameraControlLabels()
                syncLayerPopoverControls()
                if (!hub.xrayOcclusionEnabled && lastOccludedHudState) {
                    updateOcclusionHudVisibility(false)
                }
            },
            onApplyTopicPreset = { key -> applyNamedTopicPreset(key, quiet = false) },
            onProtocolChanged = { pKey ->
                rosManager.topics.protocol = pKey
                syncProtoInputsVisibility()
                saveCurrentTopics(quiet = true)
            },
            onJumpToConnDrawer = {
                syncConnModeSectionsUi(showRosEditor = true)
                topicMapExpanded = true
                topicMapAccordionBody.visibility = View.VISIBLE
                toggleOrJumpDrawerTab(TAB_CONN)
            },
            onSetHomeHere = {
                val h = simEngine.setHomeToCurrentUav()
                mapView.spawnRipple(h[0], h[1])
                AnimHelper.showDjiToast(toastBanner, String.format(Locale.US, "已更新返航点 H -> (%.2f, %.2f)", h[0], h[1]), d)
            },
            onExportPcd = { exportCurrentPointCloudPcd() },
            onClearMapAndTrail = {
                simEngine.clearVoxelMap()
                simEngine.clearTrail()
                AnimHelper.showDjiToast(toastBanner, "已清空全局点云建图与历史飞行航迹", d)
            },
            onFactoryResetApplied = {
                simEngine.notifyOperatingModeChanged()
                DroneVectorModel.activeRig = DroneVectorModel.rigFor(hub.activeFocusIsTiltRig())
                syncSwarmAllControlsUi()
                syncTiltAllControlsUi()
                applyFlightParametersToEngine(0.30f, 2.0f, 3.0f, 1.2f)
                applyActiveThemeLive(animate = true)
                syncCameraControlLabels()
                syncLayerPopoverControls()
                syncMissionStrategyButtons()
                refreshWaypointOrchestratorList()
            },
            onShowToast = { msg -> AnimHelper.showDjiToast(toastBanner, msg, d) },
            onTiltConfigChanged = {
                simEngine.notifyOperatingModeChanged()
                DroneVectorModel.activeRig = DroneVectorModel.rigFor(hub.activeFocusIsTiltRig())
                rosManager.updateTopicsAndResubscribe(rosManager.topics)
                syncSwarmAllControlsUi()
                syncTiltAllControlsUi()
                publishTiltSettingsToRos()
                attitudeHudView.invalidate()
            }
        )

        setContentView(buildNativeDjiCockpit())
        applyActiveThemeLive(animate = false)
        syncCameraControlLabels()
        syncLayerPopoverControls()
        syncMissionStrategyButtons()
        refreshWaypointOrchestratorList()
        syncConnModeSectionsUi(showRosEditor = hub.isRosMode)
        syncModeSpecificControls(force = true)
        syncBottomHudAvoidance()
        simEngine.start()
        hwBridge.start()

        if (hub.autoConnectDiscovered) {
            triggerLanScan(silentIfEmpty = true, autoOpenConnTabOnEmpty = false)
        }
        if (hub.otaAutoCheckOnStart) {
            uiHandler.postDelayed({
                if (!isFinishing && ::otaManager.isInitialized) {
                    otaManager.checkForUpdate(silentOnUpToDate = true)
                }
            }, 2600L)
        }
    }

    override fun onResume() {
        super.onResume()
        applyImmersiveFullscreen()
        mapView.startLoop()
        if (::otaManager.isInitialized) {
            otaManager.onHostResume()
        }
    }

    override fun onPause() {
        topicInspector.stopActiveSession()
        isDetailCurrentlyStreaming = false
        syncDetailStreamButtonUi()
        mapView.stopLoop()
        super.onPause()
    }

    override fun onDestroy() {
        uiHandler.removeCallbacksAndMessages(null)
        if (::otaManager.isInitialized) {
            otaManager.shutdown()
        }
        topicInspector.shutdown()
        lanScanner.cancelScan()
        hwBridge.stop()
        mapView.stopLoop()
        rosManager.shutdown()
        simEngine.stop()
        fileIoExecutor.shutdownNow()
        super.onDestroy()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) applyImmersiveFullscreen()
    }

    @Suppress("DEPRECATION")
    private fun applyImmersiveFullscreen() {
        window.decorView.systemUiVisibility = (
            View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                or View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                or View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                or View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                or View.SYSTEM_UI_FLAG_FULLSCREEN
            )
    }

    private fun hideSoftKeyboard() {
        currentFocus?.let { v ->
            val imm = getSystemService(Context.INPUT_METHOD_SERVICE) as? InputMethodManager
            imm?.hideSoftInputFromWindow(v.windowToken, 0)
            v.clearFocus()
        }
    }

    /**
     * v3.6 RadioMaster AX12 物理按键与滚轮手套盲操热键映射：
     * 当焦点不在文本输入框 (EditText) 时：
     * - 音量+/滚轮上、音量-/滚轮下：平滑缩放 3D 点云地图；
     * - 方向键上/下：按步幅微调目标飞行高度 goalZ；
     * - 空格键 / 媒体停止键：任意时刻一键触发紧急悬停！
     */
    override fun dispatchKeyEvent(event: KeyEvent): Boolean {
        if (event.action == KeyEvent.ACTION_DOWN && currentFocus !is EditText) {
            when (event.keyCode) {
                KeyEvent.KEYCODE_SPACE, KeyEvent.KEYCODE_MEDIA_STOP -> {
                    executeEmergencyHover(btnBrake)
                    return true
                }
                KeyEvent.KEYCODE_VOLUME_UP -> {
                    mapView.zoomBy(1.20f)
                    return true
                }
                KeyEvent.KEYCODE_VOLUME_DOWN -> {
                    mapView.zoomBy(0.83f)
                    return true
                }
                KeyEvent.KEYCODE_DPAD_UP -> {
                    if (!drawerOpen && (!::settingsModal.isInitialized || !settingsModal.isOpen)) {
                        adjustTargetAltitudeByStep(+1f)
                        return true
                    }
                }
                KeyEvent.KEYCODE_DPAD_DOWN -> {
                    if (!drawerOpen && (!::settingsModal.isInitialized || !settingsModal.isOpen)) {
                        adjustTargetAltitudeByStep(-1f)
                        return true
                    }
                }
            }
        }
        return super.dispatchKeyEvent(event)
    }

    @Suppress("DEPRECATION")
    override fun onBackPressed() {
        if (::settingsModal.isInitialized && settingsModal.isOpen) {
            setSettingsModalOpen(false)
            return
        }
        if (layerPopoverOpen) {
            setLayerPopoverVisible(false)
            return
        }
        if (drawerOpen) {
            if (activeTabIndex == TAB_TOPICS && topicDetailPage.visibility == View.VISIBLE) {
                exitTopicDetailToList()
                return
            }
            setDrawerOpenState(false)
            return
        }
        if (simEngine.hasActivePreviewPin()) {
            simEngine.clearPreviewPin()
            refreshCommandDockUi()
            return
        }
        val now = System.currentTimeMillis()
        if (now - lastBackMs < 1800L) {
            super.onBackPressed()
        } else {
            lastBackMs = now
            AnimHelper.showDjiToast(toastBanner, "再按一次返回键退出地面站", d)
        }
    }

    // ==================== 全栈实时换肤与空间防碰撞协调器 ====================

    private fun applyActiveThemeLive(animate: Boolean) {
        val pal = GcsTheme.of(hub.themeId)
        GcsTheme.applyTree(cockpitRoot, pal, d)
        settingsModal.applyModalThemeColors()

        tabGliderBar.background = roundedColor(pal.accent, dp(2f))
        toastBanner.setTextColor(pal.textPrimary)
        toastBanner.background = GradientDrawable(
            GradientDrawable.Orientation.TOP_BOTTOM,
            intArrayOf(pal.glassDrawerFillTop, pal.modalCardFill)
        ).apply {
            cornerRadius = dp(6.5f).toFloat()
            setStroke(maxOf(1, dp(1f)), pal.secondaryAccent)
        }
        toastBanner.elevation = 8f * d

        commandDock.background = GradientDrawable(
            GradientDrawable.Orientation.TOP_BOTTOM,
            intArrayOf(pal.glassDrawerFillTop, pal.glassPopoverFill)
        ).apply {
            cornerRadius = dp(9.5f).toFloat()
            setStroke(maxOf(1, dp(1f)), pal.accentStroke)
        }
        commandDock.elevation = 8f * d

        btnBrake.background = GradientDrawable(
            GradientDrawable.Orientation.TOP_BOTTOM,
            if (pal.isLight) {
                intArrayOf(Color.argb(56, 255, 59, 48), Color.argb(38, 229, 57, 53))
            } else {
                intArrayOf(Color.argb(148, 229, 57, 53), Color.argb(118, 168, 24, 24))
            }
        ).apply {
            cornerRadius = dp(18f).toFloat()
            setStroke(maxOf(1, dp(1.2f)), Color.parseColor("#FF5252"))
        }
        btnBrake.setTextColor(if (pal.isLight) Color.parseColor("#C62828") else Color.WHITE)
        btnBrake.invalidate()

        if (!rthConfirmPending) styleRthDefault()
        styleDemoScenePill()
        syncPatrolPillUi()
        syncTopQuickSlotsVisual()
        syncMenuButtonVisual()
        syncSettingsGearVisual()
        syncCameraControlLabels()
        syncLayerPopoverControls()
        syncTabHeaderColors()
        syncConnModeSectionsUi(showRosEditor = hub.isRosMode || rosSectionInConn.visibility == View.VISIBLE)
        syncProtoInputsVisibility()
        syncUdpSwitchVisual(animate = false)
        syncDemoSceneSegButtons()
        syncMissionStrategyButtons()
        syncSwarmAllControlsUi()
        syncTiltAllControlsUi()
        refreshWaypointOrchestratorList()
        if (cachedTopicEntries.isNotEmpty()) renderFilteredTopicRows()

        lastBatDangerLevel = -1
        lastClearanceWarnLevel = -1
        lastArmedBadgeStyled = -1
        lastHudThemeId = -1
        hsiView.invalidate()
        compassView.invalidate()
        attitudeHudView.invalidate()

        if (animate) {
            AnimHelper.animateThemeCrossfade(
                listOf(topBarView, topCenterStack, leftSafetyRail, bottomLeftCluster, rightCamStack, attitudeHudView, drawerRoot, layerQuickPopover)
            )
        }
    }

    private fun styleRthDefault() {
        val pal = GcsTheme.of(hub.themeId)
        btnRth.setTextColor(if (pal.isLight) Color.rgb(185, 102, 0) else Color.WHITE)
        GcsIcons.bind(btnRth, Type.HOME_RTH, "返航起点", d, iconDp = 12f)
        btnRth.background = pillBg(Color.argb(if (pal.isLight) 38 else 48, 255, 171, 0), Color.parseColor("#FFAB00"))
        btnRth.invalidate()
    }

    private fun styleDemoScenePill() {
        if (!::btnDemoScene.isInitialized) return
        val pal = GcsTheme.of(hub.themeId)
        btnDemoScene.setTextColor(pal.textPrimary)
        btnDemoScene.background = pillBg(pal.glassFill, pal.accentStroke)
        btnDemoScene.invalidate()
    }

    private fun syncTabHeaderColors() {
        val pal = GcsTheme.of(hub.themeId)
        for (i in tabBtns.indices) {
            val active = (i == activeTabIndex)
            tabBtns[i].setTextColor(if (active) pal.textPrimary else pal.textMuted)
            tabBtns[i].background = if (active) {
                InsetDrawable(roundedColor(pal.accentSoftFill, dp(6f)), dp(3f), dp(5f), dp(3f), dp(6f))
            } else {
                null
            }
            tabBtns[i].invalidate()
        }
    }

    private fun normalDrawerWidthPx(): Int {
        val w = resources.displayMetrics.widthPixels
        return min(dp(376f), (w * 0.57f).roundToInt())
    }

    private fun wideDrawerWidthPx(): Int {
        val w = resources.displayMetrics.widthPixels
        return min(dp(484f), (w * 0.74f).roundToInt())
    }

    private fun currentDrawerWidthPx(): Int =
        if (drawerWideMode && activeTabIndex == TAB_TOPICS && topicDetailPage.visibility == View.VISIBLE) {
            wideDrawerWidthPx()
        } else {
            normalDrawerWidthPx()
        }

    /**
     * 计算抽屉展开时左侧剩余地图可视区的精确中心偏移，保证顶部航向堆栈既不碰左安全栏也不碰右抽屉
     */
    private fun computeOpenMapCenterOffsetPx(drawerWidthPx: Int): Float {
        val totalW = cockpitRoot.width.takeIf { it > 0 } ?: resources.displayMetrics.widthPixels
        val leftSafeEdge = (leftSafetyRail.right.takeIf { it > 0 } ?: (safeCutoutLeftPx + dp(98f))) + dp(8f)
        val rightDrawerEdge = totalW - safeCutoutRightPx - drawerWidthPx - dp(14f)
        val targetCenter = ((leftSafeEdge + rightDrawerEdge) * 0.5f)
            .coerceIn(leftSafeEdge + dp(94f).toFloat(), (totalW * 0.5f))
        return targetCenter - totalW * 0.5f
    }

    /**
     * 底部与侧翼组件防重叠协调器：
     * - 展开抽屉时自动收起底部中央指令坞 (避免与抽屉或左仪表碰撞)，并在宽屏终端或超窄屏下自动避让左下数字框；
     * - 抽屉关闭且底部指令坞弹出时，在宽度 < 795dp 的屏幕上平滑避让左下数字框，保留左侧 HSI 球 + 中央指令坞 + 右侧姿态视窗三段式无碰撞对称排布。
     */
    private fun syncBottomHudAvoidance() {
        if (!::hdTelemetryBox.isInitialized || !::attitudeHudView.isInitialized || !::commandDock.isInitialized) return
        val totalW = cockpitRoot.width.takeIf { it > 0 } ?: resources.displayMetrics.widthPixels
        val drawerLeftEdge = totalW - safeCutoutRightPx - currentDrawerWidthPx() - dp(12f)
        val bottomClusterRightApprox = safeCutoutLeftPx + dp(278f)

        val hideTelemetryForDock = (!drawerOpen && dockLogicalActive && screenWidthDp < 795f)
        val hideTelemetryForDrawer = (drawerOpen && drawerLeftEdge < bottomClusterRightApprox + dp(10f))
        val shouldHideTelemetry = hideTelemetryForDock || hideTelemetryForDrawer

        if (shouldHideTelemetry) {
            if (hdTelemetryBox.visibility == View.VISIBLE) {
                hdTelemetryBox.animate().cancel()
                hdTelemetryBox.animate()
                    .alpha(0f)
                    .setDuration(130L)
                    .withEndAction {
                        if (hdTelemetryBox.alpha < 0.05f) hdTelemetryBox.visibility = View.INVISIBLE
                    }
                    .start()
            }
        } else {
            if (hdTelemetryBox.visibility != View.VISIBLE || hdTelemetryBox.alpha < 0.95f) {
                hdTelemetryBox.animate().cancel()
                hdTelemetryBox.visibility = View.VISIBLE
                hdTelemetryBox.animate()
                    .alpha(1f)
                    .setDuration(160L)
                    .start()
            }
        }

        val hideAttitudeForPopover = (layerPopoverOpen && screenHeightDp < 395f)
        if (!drawerOpen) {
            if (hideAttitudeForPopover) {
                attitudeHudView.animate().cancel()
                attitudeHudView.animate()
                    .alpha(0f)
                    .setDuration(130L)
                    .withEndAction {
                        if (layerPopoverOpen) attitudeHudView.visibility = View.INVISIBLE
                    }
                    .start()
            } else if (attitudeHudView.visibility != View.VISIBLE || attitudeHudView.alpha < 0.95f) {
                attitudeHudView.animate().cancel()
                attitudeHudView.visibility = View.VISIBLE
                attitudeHudView.translationX = 0f
                attitudeHudView.animate()
                    .alpha(1f)
                    .setDuration(170L)
                    .start()
            }
        }
    }

    // ==================== 四级互斥菜单状态机 ====================

    private fun setSettingsModalOpen(open: Boolean, jumpCategory: Int? = null) {
        hideSoftKeyboard()
        if (open) {
            if (layerPopoverOpen) setLayerPopoverVisible(false)
            if (drawerOpen) setDrawerOpenState(false)
        }
        settingsModal.setVisible(open, jumpCategory)
        syncSettingsGearVisual()
        AnimHelper.animateGearRotation(btnSettingsToggle, open)
    }

    private fun syncSettingsGearVisual() {
        if (!::btnSettingsToggle.isInitialized) return
        val pal = GcsTheme.of(hub.themeId)
        val active = ::settingsModal.isInitialized && settingsModal.isOpen
        btnSettingsToggle.setImageDrawable(
            GcsIcons.drawable(
                Type.SETTINGS_GEAR,
                dp(13.5f),
                if (active) pal.accentBright else pal.textPrimary
            )
        )
        btnSettingsToggle.background = if (active) {
            pillBg(pal.accentSoftFill, pal.accent)
        } else {
            glassBg(dp(6f))
        }
    }

    private fun toggleOrJumpDrawerTab(targetTab: Int) {
        hideSoftKeyboard()
        if (::settingsModal.isInitialized && settingsModal.isOpen) {
            setSettingsModalOpen(false)
        }
        if (layerPopoverOpen) {
            setLayerPopoverVisible(false)
        }
        if (!drawerOpen) {
            switchTab(targetTab, animatePanel = false)
            setDrawerOpenState(true)
        } else if (activeTabIndex == targetTab) {
            setDrawerOpenState(false)
        } else {
            switchTab(targetTab, animatePanel = true)
        }
    }

    private fun setDrawerOpenState(open: Boolean) {
        if (drawerOpen == open) return
        drawerOpen = open
        hideSoftKeyboard()
        if (open) {
            if (layerPopoverOpen) setLayerPopoverVisible(false)
            if (::settingsModal.isInitialized && settingsModal.isOpen) setSettingsModalOpen(false)
        }
        val prevWidthPx = drawerRoot.layoutParams.width
        if (!open) {
            topicInspector.stopActiveSession()
            isDetailCurrentlyStreaming = false
            syncDetailStreamButtonUi()
            if (drawerWideMode) {
                setWideTerminalMode(false, applyImmediately = true)
            }
        } else if (activeTabIndex == TAB_ORCHESTRATE) {
            refreshWaypointOrchestratorList()
        }
        syncMenuButtonVisual()

        val activeW = currentDrawerWidthPx()
        val animW = maxOf(activeW, prevWidthPx).toFloat()
        val centerShiftPx = computeOpenMapCenterOffsetPx(activeW)
        val activePanel = tabPanels.getOrNull(activeTabIndex)

        AnimHelper.animateDrawer(
            drawer = drawerRoot,
            rightEdgeViews = listOf(rightCamStack, attitudeHudView),
            centerShiftViews = listOf(topCenterStack),
            open = drawerOpen,
            drawerWidthPx = animW,
            centerOffsetPx = centerShiftPx,
            activePanelForCascade = activePanel,
            density = d
        )
        refreshCommandDockUi()
        syncBottomHudAvoidance()
    }

    private fun syncMenuButtonVisual() {
        if (!::btnMenuToggle.isInitialized) return
        val pal = GcsTheme.of(hub.themeId)
        if (drawerOpen) {
            btnMenuToggle.setTextColor(pal.accentBright)
            GcsIcons.bind(btnMenuToggle, Type.CLOSE_X, "收起", d, iconDp = 11.5f, gapDp = 3.5f)
            btnMenuToggle.background = pillBg(pal.accentSoftFill, pal.accent)
        } else {
            btnMenuToggle.setTextColor(pal.textPrimary)
            GcsIcons.bind(btnMenuToggle, Type.MENU_BARS, "菜单", d, iconDp = 11.5f, gapDp = 3.5f)
            btnMenuToggle.background = glassBg(dp(6f))
        }
        btnMenuToggle.invalidate()
        syncTopQuickSlotsVisual()
    }

    private fun syncTopQuickSlotsVisual() {
        val pal = GcsTheme.of(hub.themeId)
        quickPillSwarm?.let { pill ->
            val sw = hub.swarmModeEnabled
            val lbl = if (sw) "多机 ${hub.swarmDroneCount}架" else "单机"
            pill.setTextColor(if (sw) Color.WHITE else pal.textPrimary)
            pill.background = pillBg(
                if (sw) Color.parseColor("#00A86B") else pal.glassFill,
                if (sw) Color.parseColor("#00E676") else pal.glassStroke
            )
            GcsIcons.bind(pill, if (sw) Type.SWARM_FLEET else Type.DRONE_QUAD, lbl, d, iconDp = 11.5f, gapDp = 3f)
            pill.invalidate()
        }
        quickPillRig?.let { pill ->
            val sw = hub.swarmModeEnabled
            val isVec = hub.activeFocusIsTiltRig()
            val isMixed = sw && hub.swarmRigPolicy == TiltConfigSpec.FLEET_RIG_MIXED
            val narrow = screenWidthDp < 615f
            val lbl = when {
                !sw && isVec -> if (narrow) "矢量" else "矢量 4M2S"
                !sw && !isVec -> if (narrow) "普通" else "普通 X4"
                hub.swarmRigPolicy == TiltConfigSpec.FLEET_RIG_ALL_VECTORED -> "全矢量"
                hub.swarmRigPolicy == TiltConfigSpec.FLEET_RIG_ALL_STANDARD -> "全普通"
                else -> if (narrow) "混#${hub.selectedDroneId}${if (isVec) "矢" else "普"}" else "混编#${hub.selectedDroneId}${if (isVec) "矢" else "普"}"
            }
            val fillCol = when {
                isMixed -> Color.argb(if (pal.isLight) 52 else 74, 0, 188, 212)
                isVec -> Color.argb(if (pal.isLight) 48 else 72, 255, 160, 0)
                else -> pal.glassFill
            }
            val strokeCol = when {
                isMixed -> Color.parseColor("#00BCD4")
                isVec -> Color.parseColor("#FFB300")
                else -> pal.glassStroke
            }
            val txtCol = when {
                isMixed -> if (pal.isLight) Color.rgb(0, 125, 148) else Color.parseColor("#80DEEA")
                isVec -> if (pal.isLight) Color.rgb(190, 105, 0) else Color.parseColor("#FFD54F")
                else -> pal.textPrimary
            }
            pill.setTextColor(txtCol)
            pill.background = pillBg(fillCol, strokeCol)
            GcsIcons.bind(pill, if (isVec || isMixed) Type.TILT_VECTOR else Type.DRONE_QUAD, lbl, d, iconDp = 11.5f, gapDp = 3f)
            pill.invalidate()
        }
        quickPillMission?.let { pill ->
            val onOrch = drawerOpen && activeTabIndex == TAB_ORCHESTRATE
            pill.setTextColor(if (pal.isLight) Color.rgb(180, 98, 0) else Color.parseColor("#FFD54F"))
            pill.background = pillBg(
                if (onOrch) Color.argb(85, 255, 171, 0) else Color.argb(if (pal.isLight) 26 else 36, 255, 171, 0),
                if (onOrch) Color.parseColor("#FFAB00") else Color.argb(130, 255, 171, 0)
            )
            pill.invalidate()
        }
        quickPillTopics?.let { pill ->
            val onTopics = drawerOpen && activeTabIndex == TAB_TOPICS
            pill.setTextColor(if (pal.isLight) pal.accent else Color.parseColor("#64FFDA"))
            pill.background = pillBg(
                if (onTopics) pal.accentSoftFill else Color.argb(if (pal.isLight) 22 else 38, 0, 200, 235),
                if (onTopics) pal.accent else Color.argb(125, 0, 190, 230)
            )
            pill.invalidate()
        }
    }

    private fun setWideTerminalMode(wide: Boolean, applyImmediately: Boolean = true) {
        drawerWideMode = wide
        if (::btnWideTerminal.isInitialized) {
            val pal = GcsTheme.of(hub.themeId)
            btnWideTerminal.setTextColor(if (wide) pal.accentBright else pal.textPrimary)
            GcsIcons.bind(
                btnWideTerminal,
                if (wide) Type.COLLAPSE_WIDTH else Type.EXPAND_WIDTH,
                if (wide) "标准" else "宽屏",
                d,
                iconDp = 11f
            )
            btnWideTerminal.background = if (wide) {
                pillBg(pal.accentSoftFill, pal.accent)
            } else {
                glassBg(dp(6f))
            }
        }
        if (applyImmediately) {
            val newW = currentDrawerWidthPx()
            val lp = drawerRoot.layoutParams as FrameLayout.LayoutParams
            lp.width = newW
            drawerRoot.layoutParams = lp
            drawerRoot.post { syncMeasuredTabGlider(animate = false) }
            if (drawerOpen) {
                topCenterStack.translationX = computeOpenMapCenterOffsetPx(newW)
            }
            syncBottomHudAvoidance()
        }
    }

    /**
     * 基于实际测量的标签按钮坐标精确对齐底部高亮滑块，彻底消除硬编码宽度带来的偏移错位
     */
    private fun syncMeasuredTabGlider(animate: Boolean) {
        if (!::tabGlider.isInitialized || activeTabIndex !in tabBtns.indices) return
        val targetBtn = tabBtns[activeTabIndex]
        val btnW = targetBtn.width
        if (btnW <= 0) return

        val gLp = tabGlider.layoutParams as FrameLayout.LayoutParams
        if (gLp.width != btnW) {
            gLp.width = btnW
            tabGlider.layoutParams = gLp
        }
        val barW = (btnW * 0.58f).roundToInt().coerceAtLeast(dp(18f))
        val bLp = tabGliderBar.layoutParams as FrameLayout.LayoutParams
        if (bLp.width != barW) {
            bLp.width = barW
            tabGliderBar.layoutParams = bLp
        }

        val targetX = targetBtn.left.toFloat()
        if (animate) {
            AnimHelper.animateTabSwitch(tabGlider, targetX, tabPanels, activeTabIndex, d)
        } else {
            tabGlider.translationX = targetX
        }
    }

    private fun switchTab(idx: Int, animatePanel: Boolean = true) {
        hideSoftKeyboard()
        if (activeTabIndex == TAB_TOPICS && idx != TAB_TOPICS) {
            topicInspector.stopActiveSession()
            isDetailCurrentlyStreaming = false
            syncDetailStreamButtonUi()
            if (drawerWideMode) setWideTerminalMode(false, applyImmediately = true)
        }
        activeTabIndex = idx
        val curW = currentDrawerWidthPx()
        if (drawerRoot.layoutParams.width != curW) {
            val lp = drawerRoot.layoutParams as FrameLayout.LayoutParams
            lp.width = curW
            drawerRoot.layoutParams = lp
        }
        syncTabHeaderColors()

        val targetBtn = tabBtns.getOrNull(idx)
        if (animatePanel && targetBtn != null && targetBtn.width > 0) {
            syncMeasuredTabGlider(animate = true)
        } else {
            for (i in tabPanels.indices) {
                tabPanels[i].visibility = if (i == idx) View.VISIBLE else View.GONE
                tabPanels[i].alpha = 1f
                tabPanels[i].translationY = 0f
            }
            tabRowRef.post { syncMeasuredTabGlider(animate = false) }
        }
        syncMenuButtonVisual()

        when (idx) {
            TAB_TOPICS -> refreshRosTopicList()
            TAB_ORCHESTRATE -> refreshWaypointOrchestratorList()
            TAB_CONN -> syncConnModeSectionsUi(showRosEditor = hub.isRosMode || rosSectionInConn.visibility == View.VISIBLE)
        }
    }

    private fun setLayerPopoverVisible(show: Boolean) {
        if (layerPopoverOpen == show) return
        layerPopoverOpen = show
        if (show) {
            if (drawerOpen) setDrawerOpenState(false)
            if (::settingsModal.isInitialized && settingsModal.isOpen) setSettingsModalOpen(false)
        }
        val pal = GcsTheme.of(hub.themeId)
        btnLayerPopupToggle.setTextColor(pal.textPrimary)
        btnLayerPopupToggle.background = camPillBg(show)
        GcsIcons.bind(
            btnLayerPopupToggle,
            Type.LAYERS,
            "图层",
            d,
            iconDp = 11.5f,
            gapDp = 3f,
            trailing = if (show) Type.CHEVRON_UP else Type.CHEVRON_DOWN,
            trailDp = 9f
        )
        if (show) syncLayerPopoverControls()
        AnimHelper.animatePopover(layerQuickPopover, show, d)
        syncBottomHudAvoidance()
    }

    // ==================== 膨胀透视与相机标签同步 ====================

    private fun setXrayOcclusionState(enabled: Boolean, showToast: Boolean = true) {
        hub.xrayOcclusionEnabled = enabled
        GcsSettingsModal.persistAllSettings(this, hub)
        syncCameraControlLabels()
        if (!enabled && lastOccludedHudState) {
            updateOcclusionHudVisibility(false)
        }
        if (showToast) {
            AnimHelper.showDjiToast(
                toastBanner,
                if (enabled) "视线遮挡透视已开启：挡视野体素半透明化" else "已切换为实心膨胀体素模式",
                d
            )
        }
    }

    private fun syncCameraControlLabels() {
        val pal = GcsTheme.of(hub.themeId)
        val is3D = mapView.is3DMode
        btn3DRef.setTextColor(pal.textPrimary)
        GcsIcons.bind(btn3DRef, Type.CUBE_3D, if (is3D) "3D视图" else "2D俯视", d, iconDp = 11.5f, gapDp = 3f)
        btn3DRef.background = camPillBg(is3D)

        val fMode = mapView.followMode
        val followTxt = when (fMode) {
            2 -> "追尾"
            0 -> "漫游"
            else -> "环视"
        }
        btnFollowRef.setTextColor(pal.textPrimary)
        GcsIcons.bind(btnFollowRef, Type.COMPASS_AIM, followTxt, d, iconDp = 11.5f, gapDp = 3f)
        btnFollowRef.background = camPillBg(fMode != 0)
        recenterBanner.visibility = if (fMode == 0) View.VISIBLE else View.GONE
        recenterBanner.setTextColor(pal.accentBright)
        recenterBanner.background = pillBg(pal.glassPopoverFill, pal.accent)

        val xrayOn = hub.xrayOcclusionEnabled
        btnXrayRef.setTextColor(pal.textPrimary)
        GcsIcons.bind(
            btnXrayRef,
            if (xrayOn) Type.XRAY_EYE else Type.XRAY_OFF,
            if (xrayOn) "透视" else "实景",
            d,
            iconDp = 11.5f,
            gapDp = 3f
        )
        btnXrayRef.background = camPillBg(xrayOn)
    }

    private fun syncLayerPopoverControls() {
        val pal = GcsTheme.of(hub.themeId)
        if (colorSegBtns.size == 3) {
            for (i in 0..2) {
                val sel = (hub.pointColorMode == i)
                colorSegBtns[i].setTextColor(if (sel) Color.WHITE else pal.textMuted)
                colorSegBtns[i].background = roundedColor(
                    if (sel) pal.accent else Color.TRANSPARENT,
                    dp(5f)
                )
            }
        }
        if (::popSwitchInflate.isInitialized) {
            styleMiniChip(popSwitchInflate, "膨胀体素", hub.showInflate)
            styleMiniChip(popSwitchCloud, "激光点云", hub.showFastLioMap)
            styleMiniChip(popSwitchBox, "局部8m框", hub.showLocalBox)
            styleMiniChip(popSwitchCtrl, "控制点", hub.showCtrlPts)
            styleMiniChip(popSwitchTouchLock, "防误触锁", hub.hudTouchLocked)
            styleMiniChip(popSwitchNarrowAssist, "窄缝微仰", hub.autoNarrowGapPitchAssist)
        }
    }

    private fun styleMiniChip(tv: TextView, label: String, active: Boolean) {
        val pal = GcsTheme.of(hub.themeId)
        tv.setTextColor(if (active) pal.textPrimary else pal.textMuted)
        GcsIcons.bind(tv, if (active) Type.CHECK else null, label, d, iconDp = 10.5f, gapDp = 3f)
        tv.background = pillBg(
            if (active) pal.accentSoftFill else pal.wellFill,
            if (active) pal.accent else pal.cardStroke
        )
        tv.invalidate()
    }

    private fun updateOcclusionHudVisibility(occluded: Boolean) {
        if (occluded == lastOccludedHudState) return
        lastOccludedHudState = occluded
        val pal = GcsTheme.of(hub.themeId)
        occlusionBadge.setTextColor(if (pal.isLight) pal.accent else Color.parseColor("#64FFDA"))
        occlusionBadge.background = pillBg(pal.glassPopoverFill, pal.accent)

        occlusionBadge.animate().cancel()
        if (occluded) {
            occlusionBadge.visibility = View.VISIBLE
            occlusionBadge.alpha = 0f
            occlusionBadge.translationY = -dp(4f).toFloat()
            occlusionBadge.animate()
                .alpha(1f)
                .translationY(0f)
                .setDuration(180L)
                .setInterpolator(AnimHelper.EASE_OUT_EXPO)
                .start()
        } else {
            occlusionBadge.animate()
                .alpha(0f)
                .translationY(-dp(3f).toFloat())
                .setDuration(165L)
                .withEndAction {
                    if (!lastOccludedHudState) {
                        occlusionBadge.visibility = View.GONE
                    }
                }
                .start()
        }
    }

    private fun syncPatrolPillUi() {
        val pal = GcsTheme.of(hub.themeId)
        btnPatrol.setTextColor(if (hub.autoPatrol) pal.secondaryAccent else pal.textMuted)
        GcsIcons.bind(
            btnPatrol,
            if (hub.autoPatrol) Type.PLAY else Type.PAUSE,
            if (hub.autoPatrol) "巡航: 开" else "巡航: 关",
            d,
            iconDp = 11f
        )
        btnPatrol.background = pillBg(
            if (hub.autoPatrol) Color.argb(if (pal.isLight) 36 else 48, Color.red(pal.secondaryAccent), Color.green(pal.secondaryAccent), Color.blue(pal.secondaryAccent)) else pal.glassFill,
            if (hub.autoPatrol) pal.secondaryAccent else pal.glassStroke
        )
        btnPatrol.invalidate()
    }

    private fun syncModeSpecificControls(force: Boolean = false) {
        val rosNow = hub.isRosMode
        if (!force && rosNow == lastIsRosModeUi) return
        lastIsRosModeUi = rosNow
        demoControlGroup.visibility = if (!rosNow) View.VISIBLE else View.GONE
    }

    private fun refreshCommandDockUi(latestPin: EgoSimEngine.PinInspectResult? = null, pulseDock: Boolean = false) {
        val pal = GcsTheme.of(hub.themeId)
        val snap = hub.snapshots.readInto(readSnapshot)
        val hasPin = latestPin != null || simEngine.hasActivePreviewPin()
        val (routeCnt, isStaged, arcLen) = simEngine.routeStatusSummary()

        // 侧边抽屉展开期间不在地图底部重复叠加悬浮指令坞，避免与抽屉或左仪表发生挤压
        if (drawerOpen) {
            dockLogicalActive = false
            AnimHelper.animateCommandDock(commandDock, show = false, density = d)
            syncBottomHudAvoidance()
            return
        }

        if (hasPin) {
            dockLogicalActive = true
            val px = latestPin?.x ?: snap.previewX
            val py = latestPin?.y ?: snap.previewY
            val pz = latestPin?.z ?: hub.goalZ
            val dist = latestPin?.distFromUav ?: hypot(hypot(px - snap.uavX, py - snap.uavY), pz - snap.uavZ)
            val clr = latestPin?.clearance ?: snap.previewClearance
            val safe = latestPin?.isSafe ?: snap.previewSafe

            dockPinRow.visibility = View.VISIBLE
            dockRouteRow.visibility = View.GONE
            AnimHelper.animateCommandDock(commandDock, show = true, density = d, pulseOnUpdate = pulseDock)
            syncBottomHudAvoidance()

            setTextIfChanged(tvDockAltVal, String.format(Locale.US, "高 %.1fm", pz))
            val flyLabel = when {
                !hub.swarmModeEnabled -> "立即飞往"
                hub.swarmCommandScope == 1 -> "全队飞往"
                else -> "#${hub.selectedDroneId} 飞往"
            }
            GcsIcons.bind(btnDockFlyNow, if (hub.swarmModeEnabled && hub.swarmCommandScope == 1) Type.SWARM_FLEET else Type.SEND_FLY, flyLabel, d, iconDp = 11.5f, gapDp = 3.5f)
            if (safe) {
                tvDockHeader.setTextColor(pal.accentBright)
                val prefixTag = when {
                    !hub.swarmModeEnabled -> ""
                    hub.swarmCommandScope == 1 -> "${SwarmPalette.FORMATION_SHORT_NAMES[hub.swarmFormationType]}${hub.swarmDroneCount}机 · "
                    else -> "UAV-#${hub.selectedDroneId} · "
                }
                val routeHint = if (isStaged && routeCnt > 0) " · 航线$routeCnt" else ""
                GcsIcons.bind(
                    tvDockHeader,
                    if (hub.swarmModeEnabled && hub.swarmCommandScope == 1) Type.SWARM_FLEET else Type.TARGET_PIN,
                    String.format(Locale.US, "%s(%.1f, %.1f) 距%.1fm 净空%.1fm%s", prefixTag, px, py, dist, clr, routeHint),
                    d,
                    iconDp = 11.5f
                )
                btnDockFlyNow.alpha = 1.0f
            } else {
                tvDockHeader.setTextColor(Color.parseColor("#FF5252"))
                GcsIcons.bind(
                    tvDockHeader,
                    Type.WARNING,
                    String.format(Locale.US, "贴近障碍 (净空 %.2fm) · 请重选或调高", clr),
                    d,
                    iconDp = 11.5f
                )
                btnDockFlyNow.alpha = if (hub.isRosMode) 0.9f else 0.45f
            }
            return
        }

        if (routeCnt > 0) {
            dockLogicalActive = true
            dockPinRow.visibility = View.GONE
            dockRouteRow.visibility = View.VISIBLE
            AnimHelper.animateCommandDock(commandDock, show = true, density = d, pulseOnUpdate = pulseDock)
            syncBottomHudAvoidance()

            val loopTag = when (hub.routeLoopMode) {
                1 -> " · 循环"
                2 -> " · 往返"
                else -> ""
            }
            if (isStaged) {
                tvDockHeader.setTextColor(if (pal.isLight) Color.rgb(185, 102, 0) else Color.parseColor("#FFD54F"))
                GcsIcons.bind(
                    tvDockHeader,
                    Type.ROUTE_NODES,
                    String.format(Locale.US, "待飞航线：%d 站 · %.1fm%s", routeCnt, arcLen, loopTag),
                    d,
                    iconDp = 11.5f
                )
                btnDockStartRoute.visibility = View.VISIBLE
                GcsIcons.bind(btnDockStartRoute, Type.PLAY, "启动($routeCnt)", d, iconDp = 11f)
                GcsIcons.bind(btnDockClearRoute, Type.TRASH, "清空", d, iconDp = 11f)
            } else {
                tvDockHeader.setTextColor(pal.secondaryAccent)
                GcsIcons.bind(
                    tvDockHeader,
                    Type.SEND_FLY,
                    "航线执行中 · 剩余 $routeCnt 站$loopTag",
                    d,
                    iconDp = 11.5f
                )
                btnDockStartRoute.visibility = View.GONE
                GcsIcons.bind(btnDockClearRoute, Type.STOP_SQUARE, "终止", d, iconDp = 11f)
            }
            return
        }

        dockLogicalActive = false
        AnimHelper.animateCommandDock(commandDock, show = false, density = d)
        syncBottomHudAvoidance()
    }

    private fun adjustTargetAltitudeByStep(sign: Float) {
        val delta = sign * hub.altStepMeters
        val (_, pinInfo) = simEngine.nudgeGoalZ(delta)
        GcsSettingsModal.persistAllSettings(this, hub)
        mapView.triggerPinSpringAnim()
        refreshCommandDockUi(pinInfo, pulseDock = true)
    }

    private fun handleRthButtonTap() {
        if (hub.rthDoubleConfirm && !rthConfirmPending) {
            rthConfirmPending = true
            AnimHelper.triggerHaptic(btnRth)
            btnRth.setTextColor(Color.WHITE)
            GcsIcons.bind(btnRth, Type.WARNING, "再点返航", d, iconDp = 11.5f)
            btnRth.background = pillBg(Color.argb(175, 255, 145, 0), Color.parseColor("#FFD54F"))
            AnimHelper.showDjiToast(toastBanner, "防误触保护：2.5 秒内再次点击确认避障返航 H", d)
            uiHandler.removeCallbacks(resetRthConfirmRunnable)
            uiHandler.postDelayed(resetRthConfirmRunnable, 2500L)
            return
        }

        uiHandler.removeCallbacks(resetRthConfirmRunnable)
        resetRthConfirmRunnable.run()
        AnimHelper.triggerHaptic(btnRth, HapticFeedbackConstants.LONG_PRESS)

        val cmds = simEngine.dispatchSwarmReturnToHome()
        syncPatrolPillUi()
        refreshCommandDockUi()
        refreshWaypointOrchestratorList()
        for (c in cmds) mapView.spawnRipple(c.gx, c.gy)
        rosManager.publishSwarmGoals(cmds)
        val toastTxt = if (hub.swarmModeEnabled && cmds.size > 1) {
            "全队一键返航：${cmds.size} 架无人机正各自避障返回起飞坪 H0..H${cmds.size - 1}"
        } else if (cmds.isNotEmpty()) {
            String.format(Locale.US, "一键返航：正在飞回起飞点 H (%.1f, %.1f)", cmds[0].gx, cmds[0].gy)
        } else {
            "已触发返航"
        }
        AnimHelper.showDjiToast(toastBanner, toastTxt, d)
    }

    private fun handleClearMapTap() {
        if (!clearMapConfirmPending) {
            clearMapConfirmPending = true
            AnimHelper.triggerHaptic(btnClearMapInPopover)
            btnClearMapInPopover.setTextColor(Color.WHITE)
            GcsIcons.bind(btnClearMapInPopover, Type.WARNING, "再点清图", d, iconDp = 11.5f)
            btnClearMapInPopover.background = pillBg(Color.argb(165, 255, 59, 48), Color.parseColor("#FF6961"))
            AnimHelper.showDjiToast(toastBanner, "2.5 秒内再次点击确认清空点云建图", d)
            uiHandler.removeCallbacks(resetClearMapConfirmRunnable)
            uiHandler.postDelayed(resetClearMapConfirmRunnable, 2500L)
            return
        }
        uiHandler.removeCallbacks(resetClearMapConfirmRunnable)
        resetClearMapConfirmRunnable.run()
        AnimHelper.triggerHaptic(btnClearMapInPopover, HapticFeedbackConstants.LONG_PRESS)
        simEngine.clearVoxelMap()
        setLayerPopoverVisible(false)
        AnimHelper.showDjiToast(toastBanner, "已清空全局 FAST-LIO 点云地图", d)
    }

    private fun executeEmergencyHover(viewForHaptic: View) {
        uiHandler.removeCallbacks(resetRthConfirmRunnable)
        resetRthConfirmRunnable.run()
        AnimHelper.triggerHaptic(viewForHaptic, HapticFeedbackConstants.LONG_PRESS)

        val cmds = simEngine.dispatchSwarmEmergencyHover()
        syncPatrolPillUi()
        refreshCommandDockUi()
        refreshWaypointOrchestratorList()
        for (c in cmds) mapView.spawnRipple(c.gx, c.gy)
        rosManager.publishSwarmGoals(cmds)
        val hoverMsg = if (hub.swarmModeEnabled && cmds.size > 1) {
            "全队紧急制动：${cmds.size} 架无人机已同步清空航线并原地悬停"
        } else {
            "紧急制动：已清空待飞路线并下发原地悬停"
        }
        AnimHelper.showDjiToast(toastBanner, hoverMsg, d)
    }

    // ==================== 五区无重叠主座舱装配 ====================

    private fun buildNativeDjiCockpit(): View {
        cockpitRoot = FrameLayout(this).apply {
            setBackgroundColor(Color.TRANSPARENT)
        }

        mapView = GlPointCloudMapView(
            context = this,
            hub = hub,
            voxelMap = simEngine.voxelMap,
            onHudTick = { snap -> updateHudReadouts(snap) },
            onInterceptDrawerClose = {
                when {
                    ::settingsModal.isInitialized && settingsModal.isOpen -> {
                        setSettingsModalOpen(false)
                        true
                    }
                    layerPopoverOpen -> {
                        setLayerPopoverVisible(false)
                        true
                    }
                    drawerOpen -> {
                        setDrawerOpenState(false)
                        true
                    }
                    else -> false
                }
            },
            onSingleTapPreviewPin = { wx, wy ->
                if (hub.hudTouchLocked) {
                    AnimHelper.showDjiToast(toastBanner, "地图防误触已锁定 · 可在右侧「图层」浮窗解锁", d)
                } else {
                    val pickedDrone = simEngine.tryPickSwarmDroneAt(wx, wy)
                    if (pickedDrone >= 0 && pickedDrone != hub.selectedDroneId) {
                        simEngine.selectSwarmDrone(pickedDrone)
                        DroneVectorModel.activeRig = DroneVectorModel.rigFor(hub.activeFocusIsTiltRig())
                        GcsSettingsModal.persistAllSettings(this, hub)
                        mapView.recenterLockDrone()
                        syncCameraControlLabels()
                        syncSwarmAllControlsUi()
                        syncTiltAllControlsUi()
                        attitudeHudView.invalidate()
                        refreshCommandDockUi()
                        val rigTag = if (hub.isDroneTiltRig(pickedDrone)) "矢量 4M2S" else "普通 X4"
                        AnimHelper.showDjiToast(toastBanner, "已切换主控焦点 -> UAV-#$pickedDrone ($rigTag)", d)
                    } else {
                        val wasAlreadyVisible = commandDock.visibility == View.VISIBLE
                        val info = simEngine.placePreviewPin(wx, wy)
                        refreshCommandDockUi(info, pulseDock = wasAlreadyVisible)
                    }
                }
            },
            onLongPressQuickFly = { wx, wy ->
                if (hub.hudTouchLocked) {
                    AnimHelper.showDjiToast(toastBanner, "地图防误触已锁定 · 长按直飞已安全拦截", d)
                } else {
                    val cmds = simEngine.dispatchFlyCommand(wx, wy)
                    if (cmds.isEmpty()) {
                        AnimHelper.showDjiToast(toastBanner, "长按目标位于障碍物内部，已安全拦截", d)
                    } else {
                        syncPatrolPillUi()
                        refreshCommandDockUi()
                        refreshWaypointOrchestratorList()
                        for (c in cmds) mapView.spawnRipple(c.gx, c.gy)
                        rosManager.publishSwarmGoals(cmds)
                        val msg = if (cmds.size > 1) {
                            val fn = SwarmPalette.FORMATION_FULL_NAMES[hub.swarmFormationType]
                            String.format(Locale.US, "全队%s直飞 (%d架) -> 锚点(%.1f, %.1f)", fn, cmds.size, wx, wy)
                        } else {
                            val c0 = cmds[0]
                            val tag = if (hub.swarmModeEnabled) "UAV-#${c0.droneId} " else ""
                            String.format(Locale.US, "%s长按直飞 -> (%.1f, %.1f, %.1fm)", tag, c0.gx, c0.gy, c0.gz)
                        }
                        AnimHelper.showDjiToast(toastBanner, msg, d)
                    }
                }
            },
            onFollowStateChanged = { _, toastMsg ->
                runOnUiThread {
                    syncCameraControlLabels()
                    if (toastMsg != null) {
                        AnimHelper.showDjiToast(toastBanner, toastMsg, d)
                    }
                }
            }
        )
        cockpitRoot.addView(mapView, FrameLayout.LayoutParams(MATCH_PARENT, MATCH_PARENT))

        // 顶部细长电量进度条 (0..2.5dp)
        val batTrack = FrameLayout(this).apply { setBackgroundColor(Color.argb(46, 128, 145, 168)) }
        topBatFill = View(this).apply { setBackgroundColor(Color.parseColor("#00E676")) }
        batTrack.addView(topBatFill, FrameLayout.LayoutParams((resources.displayMetrics.widthPixels * 0.86f).roundToInt(), MATCH_PARENT))
        cockpitRoot.addView(batTrack, FrameLayout.LayoutParams(MATCH_PARENT, dp(2.5f), Gravity.TOP))

        // 1. 紧凑顶栏 (y = 2.5..38.5dp, 保证 640dp 横屏零溢出挤压)
        topBarView = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            minimumHeight = dp(38f)
            setPadding(dp(8f), dp(3.5f), dp(8f), dp(3f))
            GcsTheme.markRole(this, Role.TOP_GRADIENT_BAR)
        }

        tvModeBadge = TextView(this).apply {
            text = "OFFBOARD · ARMED"
            maxLines = 1
            setHorizontallyScrolling(false)
            gravity = Gravity.CENTER_VERTICAL
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
            GcsTheme.applyTabularReadout(this, bold = true, letterSpacingEm = 0.01f)
            setPadding(dp(7f), dp(4f), dp(7f), dp(4f))
            GcsTheme.markRole(this, Role.GLASS_PANEL, cornerDp = 5.5f)
        }
        topBarView.addView(tvModeBadge)

        val statusCapsule = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setPadding(dp(6.5f), dp(4f), dp(7f), dp(4f))
            GcsTheme.markRole(this, Role.GLASS_PANEL, cornerDp = 5.5f)
            setOnClickListener { toggleOrJumpDrawerTab(TAB_CONN) }
        }
        AnimHelper.attachSpringTouch(statusCapsule)
        statusAccent = View(this).apply { background = accentGreen }
        lastAppliedConnLevel = 0
        statusCapsule.addView(statusAccent, LinearLayout.LayoutParams(dp(3f), dp(11f)).apply { rightMargin = dp(5f) })
        tvConnStatus = TextView(this).apply {
            text = "仿真演练 | EXEC_TRAJ"
            maxLines = 1
            setHorizontallyScrolling(false)
            ellipsize = TextUtils.TruncateAt.END
            maxWidth = dp(
                if (screenWidthDp >= 720f) {
                    204f
                } else if (screenWidthDp >= 620f) {
                    142f
                } else {
                    108f
                }
            )
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10f)
            typeface = Typeface.DEFAULT_BOLD
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
        }
        statusCapsule.addView(tvConnStatus)
        topBarView.addView(statusCapsule, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(4.5f) })

        val swPill = TextView(this).apply {
            maxLines = 1
            setHorizontallyScrolling(false)
            gravity = Gravity.CENTER
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(6.5f), dp(4f), dp(6.5f), dp(4f))
            GcsIcons.bind(this, Type.DRONE_QUAD, "单机", d, iconDp = 11.5f, gapDp = 3f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                setSwarmModeEnabledLive(!hub.swarmModeEnabled)
            }
            setOnLongClickListener {
                AnimHelper.triggerHaptic(it, HapticFeedbackConstants.LONG_PRESS)
                toggleOrJumpDrawerTab(TAB_ORCHESTRATE)
                true
            }
        }
        AnimHelper.attachSpringTouch(swPill)
        quickPillSwarm = swPill
        topBarView.addView(swPill, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(4f) })

        val rigPill = TextView(this).apply {
            maxLines = 1
            setHorizontallyScrolling(false)
            gravity = Gravity.CENTER
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(6.5f), dp(4f), dp(6.5f), dp(4f))
            GcsIcons.bind(this, Type.TILT_VECTOR, "矢量 4M2S", d, iconDp = 11.5f, gapDp = 3f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                cycleTopRigModeQuick()
            }
            setOnLongClickListener {
                AnimHelper.triggerHaptic(it, HapticFeedbackConstants.LONG_PRESS)
                toggleOrJumpDrawerTab(TAB_ORCHESTRATE)
                true
            }
        }
        AnimHelper.attachSpringTouch(rigPill)
        quickPillRig = rigPill
        topBarView.addView(rigPill, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(4f) })

        quickSlotsContainer = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            visibility = if (screenWidthDp >= 690f) View.VISIBLE else View.GONE
        }
        val mPill = TextView(this).apply {
            maxLines = 1
            setHorizontallyScrolling(false)
            gravity = Gravity.CENTER
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(7f), dp(4f), dp(7f), dp(4f))
            GcsIcons.bind(this, Type.ROUTE_NODES, "航线", d, iconDp = 11.5f, gapDp = 3f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                toggleOrJumpDrawerTab(TAB_ORCHESTRATE)
            }
        }
        AnimHelper.attachSpringTouch(mPill)
        quickPillMission = mPill
        quickSlotsContainer.addView(mPill, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(4.5f) })

        val tPill = TextView(this).apply {
            maxLines = 1
            setHorizontallyScrolling(false)
            gravity = Gravity.CENTER
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(7f), dp(4f), dp(7f), dp(4f))
            GcsIcons.bind(this, Type.RADAR_SIGNAL, "话题", d, iconDp = 11.5f, gapDp = 3f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                toggleOrJumpDrawerTab(TAB_TOPICS)
            }
        }
        AnimHelper.attachSpringTouch(tPill)
        quickPillTopics = tPill
        quickSlotsContainer.addView(tPill, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(4f) })
        topBarView.addView(quickSlotsContainer)

        topBarView.addView(View(this), LinearLayout.LayoutParams(0, 1, 1f))

        val rightCluster = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setPadding(dp(7.5f), dp(4f), dp(7.5f), dp(4f))
            GcsTheme.markRole(this, Role.GLASS_PANEL, cornerDp = 5.5f)
            setOnClickListener { toggleOrJumpDrawerTab(TAB_STATUS) }
        }
        AnimHelper.attachSpringTouch(rightCluster)
        val showHzInTopBar = screenWidthDp >= 650f
        tvClearanceTop = monoLabel("避障1.4m", "#00E676", 9.6f)
        topClusterDivHz = View(this).apply {
            GcsTheme.markRole(this, Role.DIVIDER)
            visibility = if (showHzInTopBar) View.VISIBLE else View.GONE
        }
        tvHzTop = monoLabel("60fps", "#FFFFFF", 9.6f, Role.TEXT_SECONDARY).apply {
            visibility = if (showHzInTopBar) View.VISIBLE else View.GONE
        }
        topClusterDivBat = View(this).apply {
            GcsTheme.markRole(this, Role.DIVIDER)
        }
        tvBatTop = monoLabel(if (showHzInTopBar) "16.2V·12.6A · 控92%" else "16.2V·控92%", "#00E676", 9.6f)
        rightCluster.addView(tvClearanceTop)
        rightCluster.addView(
            topClusterDivHz,
            LinearLayout.LayoutParams(1, dp(11f)).apply {
                leftMargin = dp(5.5f)
                rightMargin = dp(5.5f)
            }
        )
        rightCluster.addView(tvHzTop)
        rightCluster.addView(
            topClusterDivBat,
            LinearLayout.LayoutParams(1, dp(11f)).apply {
                leftMargin = dp(5.5f)
                rightMargin = dp(5.5f)
            }
        )
        rightCluster.addView(tvBatTop)
        topBarView.addView(rightCluster)

        btnMenuToggle = TextView(this).apply {
            maxLines = 1
            setHorizontallyScrolling(false)
            gravity = Gravity.CENTER
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(7.5f), dp(4f), dp(7.5f), dp(4f))
            GcsIcons.bind(this, Type.MENU_BARS, "菜单", d, iconDp = 11.5f, gapDp = 3.5f)
            setOnClickListener { setDrawerOpenState(!drawerOpen) }
        }
        AnimHelper.attachSpringTouch(btnMenuToggle)
        topBarView.addView(btnMenuToggle, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(4.5f) })

        btnSettingsToggle = ImageView(this).apply {
            scaleType = ImageView.ScaleType.CENTER
            contentDescription = "系统设置"
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                setSettingsModalOpen(!settingsModal.isOpen)
            }
        }
        AnimHelper.attachSpringTouch(btnSettingsToggle)
        topBarView.addView(btnSettingsToggle, LinearLayout.LayoutParams(dp(28f), dp(26f)).apply { leftMargin = dp(4f) })

        cockpitRoot.addView(topBarView, FrameLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT, Gravity.TOP))

        // 2. 顶部居中统一流式堆栈 (航向罗盘 + 回中提示 + 遮挡透视提示 + Toast，垂直流式排布绝不互叠)
        topCenterStack = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER_HORIZONTAL
        }

        compassView = DjiCompassTapeView(this, hub).apply {
            GcsTheme.markRole(this, Role.GLASS_PANEL, cornerDp = 6.5f)
        }
        topCenterStack.addView(compassView, LinearLayout.LayoutParams(dp(196f), dp(27f)))

        swarmHudBar = buildSwarmTacticalHudBar()
        topCenterStack.addView(
            swarmHudBar,
            LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply {
                topMargin = dp(3.5f)
            }
        )

        recenterBanner = TextView(this).apply {
            visibility = View.GONE
            maxLines = 1
            setHorizontallyScrolling(false)
            ellipsize = TextUtils.TruncateAt.END
            maxWidth = dp(284f)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.8f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(9.5f), dp(3.5f), dp(9.5f), dp(3.5f))
            GcsIcons.bind(this, Type.COMPASS_AIM, "视角已漫游 · 点击回中", d, iconDp = 11.5f, gapDp = 3.5f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                mapView.recenterLockDrone()
                syncCameraControlLabels()
            }
        }
        AnimHelper.attachSpringTouch(recenterBanner)
        topCenterStack.addView(
            recenterBanner,
            LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply {
                topMargin = dp(3f)
            }
        )

        occlusionBadge = TextView(this).apply {
            visibility = View.GONE
            maxLines = 1
            setHorizontallyScrolling(false)
            ellipsize = TextUtils.TruncateAt.END
            maxWidth = dp(296f)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.5f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(9f), dp(3f), dp(9f), dp(3f))
            GcsIcons.bind(this, Type.XRAY_EYE, "视线受阻 · 已开启体素透视", d, iconDp = 11f, gapDp = 3.5f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                setXrayOcclusionState(!hub.xrayOcclusionEnabled)
            }
        }
        AnimHelper.attachSpringTouch(occlusionBadge)
        topCenterStack.addView(
            occlusionBadge,
            LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply {
                topMargin = dp(3f)
            }
        )

        toastBanner = TextView(this).apply {
            visibility = View.GONE
            maxLines = 2
            maxWidth = dp(368f)
            gravity = Gravity.CENTER
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(10f), dp(4.5f), dp(10f), dp(4.5f))
        }
        topCenterStack.addView(
            toastBanner,
            LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply {
                topMargin = dp(4f)
            }
        )

        cockpitRoot.addView(
            topCenterStack,
            FrameLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT, Gravity.TOP or Gravity.CENTER_HORIZONTAL).apply {
                topMargin = dp(43f)
            }
        )

        // 3. 左侧飞行安全操控区 (紧凑 84dp 宽度，底部留足 90dp+ 净空)
        leftSafetyRail = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }

        btnBrake = createActionPill(Type.BRAKE_HOLD, "紧急悬停", "#FF3B30", true).apply {
            setOnClickListener { executeEmergencyHover(it) }
        }
        leftSafetyRail.addView(btnBrake)

        btnRth = createActionPill(Type.HOME_RTH, "返航起点", "#FFAB00", true).apply {
            setOnClickListener { handleRthButtonTap() }
        }
        leftSafetyRail.addView(btnRth, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { topMargin = dp(5f) })

        demoControlGroup = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        btnPatrol = createActionPill(Type.PLAY, "巡航: 开", "#00E676", true).apply {
            setOnClickListener {
                hub.autoPatrol = !hub.autoPatrol
                if (hub.autoPatrol) simEngine.clearPreviewPin()
                syncPatrolPillUi()
                refreshCommandDockUi()
                if (hub.autoPatrol) AnimHelper.showDjiToast(toastBanner, "已恢复仿真连续避障巡航", d)
            }
        }
        btnDemoScene = createActionPill(Type.CUBE_3D, "场景: 体素柱阵", "#69B7FF", false).apply {
            setOnClickListener {
                applyDemoSceneByIndex((sceneIdx + 1) % sceneCycle.size)
            }
        }
        demoControlGroup.addView(btnPatrol, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { topMargin = dp(5f) })
        demoControlGroup.addView(btnDemoScene, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { topMargin = dp(5f) })
        leftSafetyRail.addView(demoControlGroup)

        cockpitRoot.addView(
            leftSafetyRail,
            FrameLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT, Gravity.TOP or Gravity.START).apply {
                leftMargin = dp(10f)
                topMargin = dp(44f)
            }
        )

        // 4. 左下角 HSI 避障球 (94x94dp) 与紧凑飞行仪表框
        bottomLeftCluster = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.BOTTOM
        }
        hsiView = DjiHsiRadarView(this, hub).apply {
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                val msg = cycleMode()
                AnimHelper.showDjiToast(toastBanner, msg, d)
            }
        }
        AnimHelper.attachSpringTouch(hsiView)
        bottomLeftCluster.addView(hsiView, LinearLayout.LayoutParams(dp(96f), dp(96f)))

        hdTelemetryBox = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(7f), dp(5f), dp(7f), dp(5f))
            GcsTheme.markRole(this, Role.GLASS_PANEL, cornerDp = 8.5f)
            setOnClickListener {
                mapView.recenterLockDrone()
                syncCameraControlLabels()
                AnimHelper.showDjiToast(toastBanner, "已回中锁定无人机视角", d)
            }
        }
        AnimHelper.attachSpringTouch(hdTelemetryBox)

        fun buildHudMetricCell(microLabel: String, valueView: TextView): LinearLayout = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            minimumWidth = dp(74f)
            setPadding(dp(6f), dp(2.5f), dp(6f), dp(2.5f))
            GcsTheme.markRole(this, Role.METRIC_CELL, cornerDp = 4.5f)
            addView(
                TextView(this@MainActivity).apply {
                    text = microLabel
                    maxLines = 1
                    setHorizontallyScrolling(false)
                    setTextSize(TypedValue.COMPLEX_UNIT_DIP, 7.4f)
                    typeface = GcsTheme.TYPEFACE_LABEL_MEDIUM
                    letterSpacing = 0.03f
                    GcsTheme.markRole(this, Role.TEXT_MUTED)
                }
            )
            addView(valueView)
        }

        val row1 = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        tvOsdH = monoLabel("1.2 m", "#FFFFFF", 11.8f, Role.TEXT_PRIMARY)
        tvOsdHs = monoLabel("0.0 m/s", "#FFFFFF", 11.8f, Role.TEXT_PRIMARY)
        row1.addView(buildHudMetricCell("高度 ALT", tvOsdH), LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        row1.addView(
            buildHudMetricCell("地速 SPD", tvOsdHs),
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dp(4f) }
        )

        val row2 = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        tvOsdD = monoLabel("3.8 m", "#FFFFFF", 11.8f, Role.TEXT_PRIMARY)
        tvOsdMapPts = monoLabel("0", "#69B7FF", 11.5f, Role.TEXT_ACCENT)
        row2.addView(buildHudMetricCell("距目标 DST", tvOsdD), LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        row2.addView(
            buildHudMetricCell("点云 MAP", tvOsdMapPts),
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dp(4f) }
        )

        tvOsdSubXyz = monoLabel("X -5.50  Y -4.20  YAW 37°", "#B8C0CC", 8.4f, Role.TEXT_MUTED).apply {
            GcsTheme.applyTabularReadout(this, bold = false, letterSpacingEm = -0.01f)
            setPadding(dp(1.5f), dp(3f), dp(1.5f), 0)
        }
        tvOsdTilt = monoLabel("TILT·协同 S1 +0.0° S2 +0.0°", "#FFB300", 8.4f).apply {
            setPadding(dp(5.5f), dp(2f), dp(5.5f), dp(2f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 4.5f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                val curVec = hub.activeFocusIsTiltRig()
                val toastTxt: String
                if (!curVec) {
                    hub.setActiveFocusTiltRig(true)
                    hub.tiltCoordMode = TiltConfigSpec.MODE_ZERO_PITCH
                    toastTxt = "已切为「双舵倾转矢量 4M2S · ${TiltConfigSpec.FULL_MODE_NAMES[0]}」"
                } else if (hub.tiltCoordMode < TiltConfigSpec.MODE_FIXED_QUAD) {
                    hub.tiltCoordMode += 1
                    toastTxt = "倾转模式：${TiltConfigSpec.FULL_MODE_NAMES[hub.tiltCoordMode]}"
                } else {
                    hub.setActiveFocusTiltRig(false)
                    toastTxt = "已切为「常规固定 X4 普通四旋翼构型」"
                }
                applyOperatingModeChangeLive(toastTxt)
            }
        }
        hdTelemetryBox.addView(row1, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT))
        hdTelemetryBox.addView(row2, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(3f) })
        hdTelemetryBox.addView(tvOsdSubXyz)
        hdTelemetryBox.addView(
            tvOsdTilt,
            LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(2.5f) }
        )
        bottomLeftCluster.addView(hdTelemetryBox, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(6f) })

        cockpitRoot.addView(
            bottomLeftCluster,
            FrameLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT, Gravity.BOTTOM or Gravity.START).apply {
                leftMargin = dp(10f)
                bottomMargin = dp(8f)
            }
        )

        // 5. 底部中央上下文选点/航线指令坞
        commandDock = buildContextualCommandDock()
        commandDock.visibility = View.GONE
        cockpitRoot.addView(
            commandDock,
            FrameLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT, Gravity.BOTTOM or Gravity.CENTER_HORIZONTAL).apply {
                bottomMargin = dp(8f)
            }
        )

        // 6. 右侧紧凑相机控制条 (y = 42..183dp, 绝不压盖下方 258..352dp 的三维姿态视窗)
        rightCamStack = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.END
        }
        btn3DRef = createCamPillBtn(Type.CUBE_3D, "3D视图", true).apply {
            setOnClickListener {
                val now3D = mapView.toggle3D2DWithAnimation()
                syncCameraControlLabels()
                AnimHelper.showDjiToast(toastBanner, if (now3D) "已切换至 3D 透视地图" else "已切换至 2D 正射俯视地图", d)
            }
        }
        btnFollowRef = createCamPillBtn(Type.COMPASS_AIM, "环视", true).apply {
            setOnClickListener {
                val (_, tip) = mapView.cycleFollowMode()
                syncCameraControlLabels()
                AnimHelper.showDjiToast(toastBanner, tip, d)
            }
        }
        btnXrayRef = createCamPillBtn(Type.XRAY_EYE, "透视", hub.xrayOcclusionEnabled).apply {
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                setXrayOcclusionState(!hub.xrayOcclusionEnabled)
            }
        }
        btnLayerPopupToggle = createCamPillBtn(Type.LAYERS, "图层", false, trailing = Type.CHEVRON_DOWN).apply {
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                setLayerPopoverVisible(!layerPopoverOpen)
            }
        }

        val zoomPair = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            GcsTheme.markRole(this, Role.GLASS_PANEL, cornerDp = 13f)
        }
        val btnZOut = TextView(this).apply {
            text = "-"
            gravity = Gravity.CENTER
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 14f)
            typeface = Typeface.DEFAULT_BOLD
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                mapView.zoomBy(0.80f)
            }
        }
        val btnZIn = TextView(this).apply {
            text = "+"
            gravity = Gravity.CENTER
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 13f)
            typeface = Typeface.DEFAULT_BOLD
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                mapView.zoomBy(1.25f)
            }
        }
        zoomPair.addView(btnZOut, LinearLayout.LayoutParams(0, MATCH_PARENT, 1f))
        zoomPair.addView(View(this).apply { GcsTheme.markRole(this, Role.DIVIDER) }, LinearLayout.LayoutParams(1, MATCH_PARENT))
        zoomPair.addView(btnZIn, LinearLayout.LayoutParams(0, MATCH_PARENT, 1f))

        val pillW = dp(82f)
        rightCamStack.addView(btn3DRef, LinearLayout.LayoutParams(pillW, WRAP_CONTENT))
        rightCamStack.addView(btnFollowRef, LinearLayout.LayoutParams(pillW, WRAP_CONTENT).apply { topMargin = dp(4.5f) })
        rightCamStack.addView(btnXrayRef, LinearLayout.LayoutParams(pillW, WRAP_CONTENT).apply { topMargin = dp(4.5f) })
        rightCamStack.addView(btnLayerPopupToggle, LinearLayout.LayoutParams(pillW, WRAP_CONTENT).apply { topMargin = dp(4.5f) })
        rightCamStack.addView(zoomPair, LinearLayout.LayoutParams(pillW, dp(29f)).apply { topMargin = dp(4.5f) })

        cockpitRoot.addView(
            rightCamStack,
            FrameLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT, Gravity.TOP or Gravity.END).apply {
                rightMargin = dp(10f)
                topMargin = dp(44f)
            }
        )

        // 7. 右下角无人机三维实时姿态视窗 (142x96dp, 与左下 HSI 96x96dp 底边严丝合缝对称对齐)
        attitudeHudView = UavAttitudeHudView(this, hub).apply {
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                val tip = cycleInspectCamera()
                AnimHelper.showDjiToast(toastBanner, tip, d)
            }
        }
        AnimHelper.attachSpringTouch(attitudeHudView)
        cockpitRoot.addView(
            attitudeHudView,
            FrameLayout.LayoutParams(dp(144f), dp(96f), Gravity.BOTTOM or Gravity.END).apply {
                rightMargin = dp(10f)
                bottomMargin = dp(8f)
            }
        )

        // 8. 紧凑图层浮窗
        layerQuickPopover = buildLayerQuickPopover()
        layerQuickPopover.visibility = View.GONE
        cockpitRoot.addView(
            layerQuickPopover,
            FrameLayout.LayoutParams(dp(264f), WRAP_CONTENT, Gravity.TOP or Gravity.END).apply {
                rightMargin = dp(98f)
                topMargin = dp(44f)
            }
        )

        // 9. 右侧自适应滑出作业抽屉
        val drawerW = normalDrawerWidthPx()
        drawerRoot = buildSlideOverDrawer(drawerW)
        drawerRoot.visibility = View.INVISIBLE
        drawerRoot.translationX = (drawerW + dp(24f)).toFloat()
        cockpitRoot.addView(
            drawerRoot,
            FrameLayout.LayoutParams(drawerW, MATCH_PARENT, Gravity.END).apply {
                topMargin = dp(43f)
                rightMargin = dp(8f)
                bottomMargin = dp(8f)
            }
        )

        // 10. 居中模态设置中心 (置于最顶层，遮罩覆盖全屏)
        val modalRoot = settingsModal.buildModalView()
        cockpitRoot.addView(modalRoot, FrameLayout.LayoutParams(MATCH_PARENT, MATCH_PARENT))

        // 11. 异形屏/挖孔屏安全区自适应（RadioMaster AX12 无挖孔屏保持紧凑 8~10dp 边距零浪费）
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) {
            cockpitRoot.setOnApplyWindowInsetsListener { _, insets ->
                val cutout = insets.displayCutout
                val sl = cutout?.safeInsetLeft ?: 0
                val sr = cutout?.safeInsetRight ?: 0
                if (sl != safeCutoutLeftPx || sr != safeCutoutRightPx) {
                    safeCutoutLeftPx = sl
                    safeCutoutRightPx = sr
                    topBarView.setPadding(dp(8f) + sl, dp(3.5f), dp(8f) + sr, dp(3f))
                    (leftSafetyRail.layoutParams as? FrameLayout.LayoutParams)?.let {
                        it.leftMargin = dp(10f) + sl
                        leftSafetyRail.layoutParams = it
                    }
                    (bottomLeftCluster.layoutParams as? FrameLayout.LayoutParams)?.let {
                        it.leftMargin = dp(10f) + sl
                        bottomLeftCluster.layoutParams = it
                    }
                    (rightCamStack.layoutParams as? FrameLayout.LayoutParams)?.let {
                        it.rightMargin = dp(10f) + sr
                        rightCamStack.layoutParams = it
                    }
                    (attitudeHudView.layoutParams as? FrameLayout.LayoutParams)?.let {
                        it.rightMargin = dp(10f) + sr
                        attitudeHudView.layoutParams = it
                    }
                    (layerQuickPopover.layoutParams as? FrameLayout.LayoutParams)?.let {
                        it.rightMargin = dp(98f) + sr
                        layerQuickPopover.layoutParams = it
                    }
                    (drawerRoot.layoutParams as? FrameLayout.LayoutParams)?.let {
                        it.rightMargin = dp(8f) + sr
                        drawerRoot.layoutParams = it
                    }
                    syncBottomHudAvoidance()
                }
                insets
            }
        }

        return cockpitRoot
    }

    private fun buildLayerQuickPopover(): LinearLayout {
        val card = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(9f), dp(8f), dp(9f), dp(8f))
            GcsTheme.markRole(this, Role.GLASS_POPOVER, cornerDp = 9f)
            isClickable = true
        }

        card.addView(
            TextView(this).apply {
                text = "点云配色方案"
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.5f)
                typeface = Typeface.DEFAULT_BOLD
                setPadding(0, 0, 0, dp(3.5f))
                GcsTheme.markRole(this, Role.TEXT_MUTED)
            }
        )

        val colorRow = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dp(2f), dp(2f), dp(2f), dp(2f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        val cNames = listOf("高程彩虹", "反射强度", "战术翠绿")
        colorSegBtns.clear()
        for (i in cNames.indices) {
            val b = TextView(this).apply {
                text = cNames[i]
                maxLines = 1
                setHorizontallyScrolling(false)
                gravity = Gravity.CENTER
                minimumHeight = dp(24f)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.5f)
                typeface = Typeface.DEFAULT_BOLD
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    hub.pointColorMode = i
                    GcsSettingsModal.persistAllSettings(this@MainActivity, hub)
                    syncLayerPopoverControls()
                }
            }
            colorSegBtns.add(b)
            colorRow.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        card.addView(colorRow)

        card.addView(
            TextView(this).apply {
                text = "地图图层显示"
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.5f)
                typeface = Typeface.DEFAULT_BOLD
                setPadding(0, dp(6f), 0, dp(3.5f))
                GcsTheme.markRole(this, Role.TEXT_MUTED)
            }
        )

        val g1 = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        popSwitchInflate = createPopoverToggleChip {
            hub.showInflate = !hub.showInflate
            GcsSettingsModal.persistAllSettings(this, hub)
            syncLayerPopoverControls()
        }
        popSwitchCloud = createPopoverToggleChip {
            hub.showFastLioMap = !hub.showFastLioMap
            GcsSettingsModal.persistAllSettings(this, hub)
            syncLayerPopoverControls()
        }
        g1.addView(popSwitchInflate, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        g1.addView(popSwitchCloud, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dp(4.5f) })
        card.addView(g1)

        val g2 = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        popSwitchBox = createPopoverToggleChip {
            hub.showLocalBox = !hub.showLocalBox
            GcsSettingsModal.persistAllSettings(this, hub)
            syncLayerPopoverControls()
        }
        popSwitchCtrl = createPopoverToggleChip {
            hub.showCtrlPts = !hub.showCtrlPts
            GcsSettingsModal.persistAllSettings(this, hub)
            syncLayerPopoverControls()
        }
        g2.addView(popSwitchBox, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        g2.addView(popSwitchCtrl, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dp(4.5f) })
        card.addView(g2, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(4.5f) })

        val g3 = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        popSwitchTouchLock = createPopoverToggleChip {
            hub.hudTouchLocked = !hub.hudTouchLocked
            syncLayerPopoverControls()
            AnimHelper.showDjiToast(
                toastBanner,
                if (hub.hudTouchLocked) "已开启雨林携行防误触锁 (紧急悬停仍随时有效)" else "已解除地图防误触锁",
                d
            )
        }
        popSwitchNarrowAssist = createPopoverToggleChip {
            hub.autoNarrowGapPitchAssist = !hub.autoNarrowGapPitchAssist
            GcsSettingsModal.persistAllSettings(this, hub)
            syncLayerPopoverControls()
            AnimHelper.showDjiToast(
                toastBanner,
                if (hub.autoNarrowGapPitchAssist) "已开启窄缝自动微仰辅助 (净空<0.88m自动减截面)" else "已关闭窄缝自动微仰辅助",
                d
            )
        }
        g3.addView(popSwitchTouchLock, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        g3.addView(popSwitchNarrowAssist, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dp(4.5f) })
        card.addView(g3, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(4.5f) })

        val actRow = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        val btnResetCam = createSmallIconBtn(Type.UNDO, "复位") {
            mapView.resetCameraSmooth()
            syncCameraControlLabels()
            setLayerPopoverVisible(false)
            AnimHelper.showDjiToast(toastBanner, "已复位默认三维跟随视角", d)
        }
        val btnClearTrail = createSmallIconBtn(Type.ERASER, "清轨迹") {
            simEngine.clearTrail()
            setLayerPopoverVisible(false)
            AnimHelper.showDjiToast(toastBanner, "已清空历史飞行航迹", d)
        }
        btnClearMapInPopover = createSmallIconBtn(Type.TRASH, "清建图") {
            handleClearMapTap()
        }
        actRow.addView(btnResetCam, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        actRow.addView(btnClearTrail, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1.1f).apply { leftMargin = dp(4f) })
        actRow.addView(btnClearMapInPopover, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1.1f).apply { leftMargin = dp(4f) })
        card.addView(actRow, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(6f) })

        return card
    }

    private fun createPopoverToggleChip(onClick: () -> Unit): TextView = TextView(this).apply {
        gravity = Gravity.CENTER
        maxLines = 1
        setHorizontallyScrolling(false)
        minimumHeight = dp(25f)
        setPadding(dp(5f), dp(4f), dp(5f), dp(4f))
        setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
        typeface = Typeface.DEFAULT_BOLD
        setOnClickListener {
            AnimHelper.triggerHaptic(it)
            onClick()
        }
    }

    private fun buildContextualCommandDock(): LinearLayout {
        val dock = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER_HORIZONTAL
            setPadding(dp(9f), dp(6f), dp(9f), dp(6f))
            isClickable = true
        }

        tvDockHeader = TextView(this).apply {
            maxLines = 2
            gravity = Gravity.CENTER
            maxWidth = dp(348f)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.8f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(0, 0, 0, dp(4f))
            GcsIcons.bind(this, Type.TARGET_PIN, "选点预览", d, iconDp = 11.5f)
        }
        dock.addView(tvDockHeader)

        dockPinRow = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }

        btnDockFlyNow = TextView(this).apply {
            maxLines = 1
            setHorizontallyScrolling(false)
            gravity = Gravity.CENTER
            minimumHeight = dp(27f)
            setTextColor(Color.WHITE)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(9f), dp(4.5f), dp(9f), dp(4.5f))
            background = roundedColor(Color.parseColor("#00B865"), dp(5.5f))
            GcsIcons.bind(this, Type.SEND_FLY, "立即飞往", d, iconDp = 11.5f, gapDp = 3.5f)
            setOnClickListener {
                val cmds = simEngine.dispatchFlyCommand()
                if (cmds.isEmpty()) {
                    AnimHelper.triggerHaptic(it, HapticFeedbackConstants.LONG_PRESS)
                    AnimHelper.showDjiToast(toastBanner, "选点位于障碍物内部，请重选或调高高度", d)
                } else {
                    AnimHelper.triggerHaptic(it, HapticFeedbackConstants.LONG_PRESS)
                    syncPatrolPillUi()
                    refreshCommandDockUi()
                    refreshWaypointOrchestratorList()
                    for (c in cmds) mapView.spawnRipple(c.gx, c.gy)
                    rosManager.publishSwarmGoals(cmds)
                    val msg = if (cmds.size > 1) {
                        val fn = SwarmPalette.FORMATION_FULL_NAMES[hub.swarmFormationType]
                        String.format(Locale.US, "全队%s协同出发 (%d架) · 高度 %.1fm", fn, cmds.size, cmds[0].gz)
                    } else {
                        val c0 = cmds[0]
                        val prefix = if (hub.swarmModeEnabled) "UAV-#${c0.droneId} " else ""
                        String.format(Locale.US, "%s已下发指点飞行 -> (%.1f, %.1f, %.1fm)", prefix, c0.gx, c0.gy, c0.gz)
                    }
                    AnimHelper.showDjiToast(toastBanner, msg, d)
                }
            }
        }
        AnimHelper.attachSpringTouch(btnDockFlyNow)

        val btnAddRoute = TextView(this).apply {
            maxLines = 1
            setHorizontallyScrolling(false)
            gravity = Gravity.CENTER
            minimumHeight = dp(27f)
            setTextColor(Color.rgb(215, 130, 0))
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(8f), dp(4.5f), dp(8f), dp(4.5f))
            background = pillBg(Color.argb(46, 255, 171, 0), Color.parseColor("#FFAB00"))
            GcsIcons.bind(this, Type.PLUS_NODE, "加航点", d, iconDp = 11.5f, gapDp = 3f)
            setOnClickListener {
                val (ok, total, _) = simEngine.appendPreviewPinToRoute()
                if (!ok) {
                    AnimHelper.showDjiToast(toastBanner, "选点位于障碍内部，无法加入航线", d)
                } else {
                    AnimHelper.triggerHaptic(it)
                    refreshCommandDockUi(pulseDock = true)
                    refreshWaypointOrchestratorList()
                    AnimHelper.showDjiToast(toastBanner, "已加入航点 #$total", d)
                }
            }
        }
        AnimHelper.attachSpringTouch(btnAddRoute)

        val altInline = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            minimumHeight = dp(28f)
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 5.5f)
        }
        val btnMinus = TextView(this).apply {
            text = "−"
            gravity = Gravity.CENTER
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 13f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(8f), dp(3.5f), dp(8f), dp(3.5f))
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                adjustTargetAltitudeByStep(-1f)
            }
        }
        tvDockAltVal = monoLabel(String.format(Locale.US, "高 %.1fm", hub.goalZ), "#69B7FF", 9.8f, Role.TEXT_ACCENT).apply {
            setPadding(dp(6f), 0, dp(6f), 0)
        }
        val btnPlus = TextView(this).apply {
            text = "+"
            gravity = Gravity.CENTER
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 12.5f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(8f), dp(3.5f), dp(8f), dp(3.5f))
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                adjustTargetAltitudeByStep(+1f)
            }
        }
        altInline.addView(btnMinus)
        altInline.addView(View(this).apply { GcsTheme.markRole(this, Role.DIVIDER) }, LinearLayout.LayoutParams(1, dp(14f)))
        altInline.addView(tvDockAltVal)
        altInline.addView(View(this).apply { GcsTheme.markRole(this, Role.DIVIDER) }, LinearLayout.LayoutParams(1, dp(14f)))
        altInline.addView(btnPlus)

        val btnCancelPin = ImageView(this).apply {
            scaleType = ImageView.ScaleType.CENTER
            setImageDrawable(GcsIcons.drawable(Type.CLOSE_X, dp(12.5f), Color.rgb(130, 145, 165)))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 5.5f)
            setOnClickListener {
                simEngine.clearPreviewPin()
                refreshCommandDockUi()
            }
        }

        dockPinRow.addView(btnDockFlyNow)
        dockPinRow.addView(btnAddRoute, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(5f) })
        dockPinRow.addView(altInline, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(5f) })
        dockPinRow.addView(btnCancelPin, LinearLayout.LayoutParams(dp(27f), dp(27f)).apply { leftMargin = dp(5f) })
        dock.addView(dockPinRow)

        dockRouteRow = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            visibility = View.GONE
        }

        btnDockStartRoute = TextView(this).apply {
            maxLines = 1
            setHorizontallyScrolling(false)
            gravity = Gravity.CENTER
            minimumHeight = dp(27f)
            setTextColor(Color.WHITE)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(9.5f), dp(4.5f), dp(9.5f), dp(4.5f))
            background = roundedColor(Color.parseColor("#00B865"), dp(5.5f))
            GcsIcons.bind(this, Type.PLAY, "启动航线", d, iconDp = 11f)
            setOnClickListener {
                startCommittedRouteExecution(it)
            }
        }
        AnimHelper.attachSpringTouch(btnDockStartRoute)

        btnDockOpenOrchestrate = createSmallIconBtn(Type.ROUTE_NODES, "航点") {
            if (!drawerOpen || activeTabIndex != TAB_ORCHESTRATE) {
                toggleOrJumpDrawerTab(TAB_ORCHESTRATE)
            }
        }

        btnDockUndo = createSmallIconBtn(Type.UNDO, "撤点") {
            val (rem, _, _) = simEngine.undoLastWaypoint()
            refreshCommandDockUi(pulseDock = true)
            refreshWaypointOrchestratorList()
            AnimHelper.showDjiToast(toastBanner, "已撤销末尾航点，剩余 $rem 站", d)
        }

        btnDockClearRoute = createSmallIconBtn(Type.TRASH, "清空") {
            val (_, isStaged, _) = simEngine.routeStatusSummary()
            if (isStaged) {
                simEngine.clearWaypointQueue()
                refreshCommandDockUi()
                refreshWaypointOrchestratorList()
                AnimHelper.showDjiToast(toastBanner, "已清空待执行航线", d)
            } else {
                executeEmergencyHover(btnDockClearRoute)
            }
        }

        dockRouteRow.addView(btnDockStartRoute, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { rightMargin = dp(5f) })
        dockRouteRow.addView(btnDockOpenOrchestrate, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { rightMargin = dp(5f) })
        dockRouteRow.addView(btnDockUndo)
        dockRouteRow.addView(btnDockClearRoute, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(5f) })
        dock.addView(dockRouteRow)

        return dock
    }

    private fun startCommittedRouteExecution(sourceView: View) {
        val committed = simEngine.commitStagedRoute()
        if (committed != null) {
            val (firstGoal, totalPts) = committed
            AnimHelper.triggerHaptic(sourceView, HapticFeedbackConstants.LONG_PRESS)
            mapView.recenterLockDrone()
            syncCameraControlLabels()
            syncPatrolPillUi()
            refreshCommandDockUi(pulseDock = true)
            refreshWaypointOrchestratorList()
            mapView.spawnRipple(firstGoal[0], firstGoal[1])
            rosManager.publishGoal(firstGoal[0], firstGoal[1], firstGoal[2], firstGoal[3], firstGoal[4])
            val modeLabel = when (hub.routeLoopMode) {
                1 -> "闭合循环"
                2 -> "往返巡航"
                else -> "单次推进"
            }
            AnimHelper.showDjiToast(toastBanner, "航线已启动：共 $totalPts 站 ($modeLabel)", d)
        }
    }

    // ==================== 四标签主作业抽屉 (连接 | 航线 | 话题 | 状态) ====================

    private fun buildSlideOverDrawer(drawerWidthPx: Int): LinearLayout {
        val container = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            GcsTheme.markRole(this, Role.GLASS_DRAWER, cornerDp = 11f)
            isClickable = true
            isFocusable = true
        }

        val tabBarWrap = FrameLayout(this).apply {
            GcsTheme.markRole(this, Role.MODAL_HEADER)
        }
        tabRowRef = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }
        val tabDefs = listOf(
            Type.LINK to "连接",
            Type.ROUTE_NODES to "航线",
            Type.RADAR_SIGNAL to "话题",
            Type.TELEMETRY_PULSE to "状态"
        )

        tabBtns.clear()
        for (i in tabDefs.indices) {
            val (ic, title) = tabDefs[i]
            val tb = TextView(this).apply {
                gravity = Gravity.CENTER
                maxLines = 1
                setHorizontallyScrolling(false)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.6f)
                typeface = Typeface.DEFAULT_BOLD
                GcsIcons.bind(this, ic, title, d, iconDp = 12f, gapDp = 3.5f)
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    switchTab(i)
                }
            }
            tabBtns.add(tb)
            tb.minimumHeight = dp(38f)
            tb.setPadding(dp(4f), dp(5f), dp(4f), dp(5f))
            tabRowRef.addView(tb, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }

        val btnCloseDrawer = ImageView(this).apply {
            scaleType = ImageView.ScaleType.CENTER
            setImageDrawable(GcsIcons.drawable(Type.CLOSE_X, dp(13.5f), Color.rgb(140, 155, 175)))
            setPadding(dp(8f), 0, dp(9f), 0)
            setOnClickListener { setDrawerOpenState(false) }
        }
        tabRowRef.addView(btnCloseDrawer, LinearLayout.LayoutParams(dp(34f), dp(38f)))
        tabBarWrap.addView(tabRowRef, FrameLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT))

        val initSlotW = ((drawerWidthPx - dp(32f)) / TAB_COUNT.toFloat()).roundToInt().coerceAtLeast(dp(40f))
        tabGlider = FrameLayout(this).apply {
            tabGliderBar = View(this@MainActivity).apply {
                background = roundedColor(Color.parseColor("#1890FF"), dp(2f))
            }
            addView(tabGliderBar, FrameLayout.LayoutParams((initSlotW * 0.58f).roundToInt(), dp(2.5f), Gravity.CENTER))
        }
        tabBarWrap.addView(tabGlider, FrameLayout.LayoutParams(initSlotW, dp(3f), Gravity.BOTTOM or Gravity.START))

        tabRowRef.addOnLayoutChangeListener { _, _, _, _, _, _, _, _, _ ->
            syncMeasuredTabGlider(animate = false)
        }

        container.addView(tabBarWrap, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT))
        container.addView(View(this).apply { GcsTheme.markRole(this, Role.DIVIDER) }, LinearLayout.LayoutParams(MATCH_PARENT, 1))

        val bodyHost = FrameLayout(this)
        val pConn = buildConnPanel()
        val pOrchestrate = buildOrchestrationPanel()
        val pTopics = buildTopicInspectorPanel()
        val pStatus = buildTelemetryPanel()
        pOrchestrate.visibility = View.GONE
        pTopics.visibility = View.GONE
        pStatus.visibility = View.GONE

        tabPanels.clear()
        tabPanels.addAll(listOf(pConn, pOrchestrate, pTopics, pStatus))
        bodyHost.addView(pConn)
        bodyHost.addView(pOrchestrate)
        bodyHost.addView(pTopics)
        bodyHost.addView(pStatus)
        container.addView(bodyHost, LinearLayout.LayoutParams(MATCH_PARENT, 0, 1f))

        return container
    }

    // ==================== Tab 0: 连接与工作模式面板 ====================

    private fun buildConnPanel(): View {
        val sv = ScrollView(this)
        val col = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(10f), dp(9f), dp(10f), dp(10f))
        }

        val modeSegBar = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dp(2.5f), dp(2.5f), dp(2.5f), dp(2.5f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 7f)
        }
        segModeDemoBtn = TextView(this).apply {
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            minimumHeight = dp(29f)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.5f)
            typeface = Typeface.DEFAULT_BOLD
            GcsIcons.bind(this, Type.CUBE_3D, "内置仿真演练", d, iconDp = 12f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                if (hub.isRosMode) {
                    topicInspector.stopActiveSession()
                    rosManager.switchToDemoMode()
                    simEngine.clearVoxelMap()
                    simEngine.clearWaypointQueue()
                    simEngine.requestReplanAsync()
                    cachedTopicEntries = emptyList()
                    syncModeSpecificControls(force = true)
                    refreshCommandDockUi()
                    refreshWaypointOrchestratorList()
                    AnimHelper.showDjiToast(toastBanner, "已切换回三维膨胀体素仿真演练", d)
                }
                syncConnModeSectionsUi(showRosEditor = false)
            }
        }
        segModeRosBtn = TextView(this).apply {
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            minimumHeight = dp(29f)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.5f)
            typeface = Typeface.DEFAULT_BOLD
            GcsIcons.bind(this, Type.DRONE_QUAD, "实机 ROS1 联飞", d, iconDp = 12.5f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                syncConnModeSectionsUi(showRosEditor = true)
            }
        }
        modeSegBar.addView(segModeDemoBtn, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        modeSegBar.addView(segModeRosBtn, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        col.addView(modeSegBar)

        demoSectionInConn = sectionCard("内置三维膨胀体素仿真环境").apply {
            val desc = TextView(this@MainActivity).apply {
                text = "当前运行本地三维避障与激光点云建图引擎，可切换测试障碍场景或接入真机："
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
                setPadding(0, 0, 0, dp(6.5f))
                GcsTheme.markRole(this, Role.TEXT_MUTED)
            }
            addView(desc)

            val sceneGrid = LinearLayout(this@MainActivity).apply { orientation = LinearLayout.VERTICAL }
            val sceneRowTop = LinearLayout(this@MainActivity).apply { orientation = LinearLayout.HORIZONTAL }
            val sceneRowBot = LinearLayout(this@MainActivity).apply { orientation = LinearLayout.HORIZONTAL }
            demoSceneSegBtns.clear()
            for (i in sceneCycle.indices) {
                val sb = TextView(this@MainActivity).apply {
                    text = sceneCycle[i].second
                    maxLines = 1
                    setHorizontallyScrolling(false)
                    gravity = Gravity.CENTER
                    minimumHeight = dp(30f)
                    setPadding(dp(8f), dp(5f), dp(8f), dp(5f))
                    setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.0f)
                    typeface = Typeface.DEFAULT_BOLD
                    setOnClickListener {
                        AnimHelper.triggerHaptic(it)
                        applyDemoSceneByIndex(i)
                    }
                }
                demoSceneSegBtns.add(sb)
                val targetRow = if (i < 2) sceneRowTop else sceneRowBot
                targetRow.addView(
                    sb,
                    LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply {
                        if (i % 2 == 1) leftMargin = dp(5f)
                    }
                )
            }
            sceneGrid.addView(sceneRowTop)
            sceneGrid.addView(sceneRowBot, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(5f) })
            addView(sceneGrid)

            val jumpScanBtn = TextView(this@MainActivity).apply {
                gravity = Gravity.CENTER
                maxLines = 1
                setHorizontallyScrolling(false)
                minimumHeight = dp(30f)
                setPadding(dp(8f), dp(5f), dp(8f), dp(5f))
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
                typeface = Typeface.DEFAULT_BOLD
                GcsTheme.markRole(this, Role.TEXT_ACCENT)
                background = pillBg(Color.argb(42, 24, 144, 255), Color.argb(115, 24, 144, 255))
                GcsIcons.bind(this, Type.SEARCH, "接入真机 · 扫描局域网无人机", d, iconDp = 11.5f, trailing = Type.CHEVRON_RIGHT)
                setOnClickListener {
                    syncConnModeSectionsUi(showRosEditor = true)
                    triggerLanScan(silentIfEmpty = false, autoOpenConnTabOnEmpty = false)
                }
            }
            AnimHelper.attachSpringTouch(jumpScanBtn)
            addView(jumpScanBtn, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })
        }
        col.addView(demoSectionInConn, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        rosSectionInConn = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            visibility = View.GONE
        }

        val scanCard = sectionCard("1. 局域网自动寻机 (探测 11311 / 9090)")
        val scanOptRow = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        btnDrawerScan = TextView(this).apply {
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            minimumHeight = dp(30f)
            setTextColor(Color.WHITE)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.5f)
            typeface = Typeface.DEFAULT_BOLD
            background = roundedColor(Color.parseColor("#00B865"), dp(6f))
            GcsIcons.bind(this, Type.SEARCH, "扫描局域网飞机", d, iconDp = 12f)
            setOnClickListener { triggerLanScan(silentIfEmpty = false, autoOpenConnTabOnEmpty = false) }
        }
        AnimHelper.attachSpringTouch(btnDrawerScan)

        val btnOpenLinkPrefs = createSmallIconBtn(Type.SETTINGS_GEAR, "链路设置") {
            setSettingsModalOpen(true, jumpCategory = GcsSettingsModal.CAT_COMMS)
        }
        scanOptRow.addView(btnDrawerScan, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1.35f))
        scanOptRow.addView(btnOpenLinkPrefs, LinearLayout.LayoutParams(0, WRAP_CONTENT, 0.85f).apply { leftMargin = dp(5f) })
        scanCard.addView(scanOptRow)

        tvScanStatus = TextView(this).apply {
            text = "支持同一 Wi-Fi / 机载 Ubuntu 热点 (10.42.0.1) 自动探测"
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
            setPadding(0, dp(5f), 0, dp(2f))
            GcsTheme.markRole(this, Role.TEXT_MUTED)
        }
        scanCard.addView(tvScanStatus)

        discoveredListContainer = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        scanCard.addView(discoveredListContainer)
        rosSectionInConn.addView(scanCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        val cfg = rosManager.topics
        val directCard = sectionCard("2. 手动地址直连")
        val protoSeg = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dp(2f), dp(2f), dp(2f), dp(2f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        segProtoTcpBtn = TextView(this).apply {
            text = "原生 TCPROS (:11311)"
            maxLines = 1
            setHorizontallyScrolling(false)
            ellipsize = TextUtils.TruncateAt.END
            gravity = Gravity.CENTER
            minimumHeight = dp(26f)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.8f)
            typeface = Typeface.DEFAULT_BOLD
            setOnClickListener {
                rosManager.topics.protocol = "tcpros"
                syncProtoInputsVisibility()
            }
        }
        segProtoWsBtn = TextView(this).apply {
            text = "rosbridge (:9090)"
            maxLines = 1
            setHorizontallyScrolling(false)
            ellipsize = TextUtils.TruncateAt.END
            gravity = Gravity.CENTER
            minimumHeight = dp(26f)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.8f)
            typeface = Typeface.DEFAULT_BOLD
            setOnClickListener {
                rosManager.topics.protocol = "rosbridge"
                syncProtoInputsVisibility()
            }
        }
        protoSeg.addView(segProtoTcpBtn, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        protoSeg.addView(segProtoWsBtn, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        directCard.addView(protoSeg)

        wrapMasterUri = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        inpMasterUri = labeledInput(wrapMasterUri, "机载 ROS_MASTER_URI", cfg.masterUri)
        directCard.addView(wrapMasterUri)

        wrapWsUrl = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        inpWsUrl = labeledInput(wrapWsUrl, "机载 rosbridge WebSocket 地址", cfg.wsUrl)
        directCard.addView(wrapWsUrl)
        udpTelemetryEnabledUi = cfg.udpTelemetryEnabled
        val udpToggleRow = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setPadding(dp(7.5f), dp(6f), dp(7.5f), dp(6f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        val udpTxtCol = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        udpTxtCol.addView(
            TextView(this).apply {
                text = "UDP 点云/位姿高速直通"
                maxLines = 1
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10f)
                typeface = Typeface.DEFAULT_BOLD
                GcsTheme.markRole(this, Role.TEXT_PRIMARY)
            }
        )
        udpTxtCol.addView(
            TextView(this).apply {
                text = "高频遥测走低延迟 UDP，控制指令保持可靠链路"
                maxLines = 2
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 8.8f)
                setPadding(0, dp(1.5f), dp(6f), 0)
                GcsTheme.markRole(this, Role.TEXT_MUTED)
            }
        )
        udpSwitchPill = FrameLayout(this)
        udpSwitchThumb = View(this).apply {
            background = roundedColor(Color.WHITE, dp(8.5f))
        }
        udpSwitchPill.addView(
            udpSwitchThumb,
            FrameLayout.LayoutParams(dp(15f), dp(15f), Gravity.CENTER_VERTICAL).apply {
                leftMargin = dp(2.5f)
            }
        )
        val toggleUdp = {
            udpTelemetryEnabledUi = !udpTelemetryEnabledUi
            AnimHelper.triggerHaptic(udpToggleRow)
            syncUdpSwitchVisual(animate = true)
        }
        udpToggleRow.setOnClickListener { toggleUdp() }
        udpSwitchPill.setOnClickListener { toggleUdp() }
        udpToggleRow.addView(udpTxtCol, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        udpToggleRow.addView(udpSwitchPill, LinearLayout.LayoutParams(dp(36f), dp(20f)))
        directCard.addView(udpToggleRow, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(7f) })
        syncUdpSwitchVisual(animate = false)

        inpUdpPort = labeledInput(directCard, "本机 UDP 接收端口（需机载 ROS→UDP 桥）", cfg.udpPort.toString()).apply {
            inputType = android.text.InputType.TYPE_CLASS_NUMBER
        }
        tvUdpStatus = monoLabel(hub.udpTelemetryStatus, "#AAB6C7", 9.2f, Role.TEXT_MUTED).apply {
            setPadding(0, dp(3f), 0, 0)
        }
        directCard.addView(tvUdpStatus)

        val actRow = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        val btnConn = TextView(this).apply {
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            minimumHeight = dp(31f)
            setTextColor(Color.WHITE)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.8f)
            typeface = Typeface.DEFAULT_BOLD
            background = roundedColor(Color.parseColor("#1890FF"), dp(6f))
            GcsIcons.bind(this, Type.LINK, "立即连接机载 ROS1", d, iconDp = 12f)
            setOnClickListener {
                hideSoftKeyboard()
                if (saveCurrentTopics(quiet = true, refreshLink = false)) connectWithConfig(rosManager.topics)
            }
        }
        AnimHelper.attachSpringTouch(btnConn)
        val btnDisc = createSmallIconBtn(null, "断开") {
            topicInspector.stopActiveSession()
            rosManager.disconnectAll(silent = false)
            AnimHelper.showDjiToast(toastBanner, "已断开机载 ROS1 连接", d)
        }
        actRow.addView(btnConn, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        btnDisc.minWidth = dp(66f)
        actRow.addView(btnDisc, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(6f) })
        directCard.addView(actRow, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(7f) })
        rosSectionInConn.addView(directCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        val topicCard = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(9f), dp(7.5f), dp(9f), dp(7.5f))
            GcsTheme.markRole(this, Role.SECTION_CARD, cornerDp = 8f)
        }
        btnExpandTopicMap = TextView(this).apply {
            maxLines = 1
            setHorizontallyScrolling(false)
            ellipsize = TextUtils.TruncateAt.END
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(0, dp(2f), 0, dp(2f))
            GcsTheme.markRole(this, Role.TEXT_ACCENT)
            GcsIcons.bind(this, Type.SLIDERS, "高级话题映射配置", d, iconDp = 11.5f, trailing = Type.CHEVRON_DOWN)
            setOnClickListener {
                topicMapExpanded = !topicMapExpanded
                topicMapAccordionBody.visibility = if (topicMapExpanded) View.VISIBLE else View.GONE
                if (topicMapExpanded) AnimHelper.animateStaggeredChildren(topicMapAccordionBody, d)
                GcsIcons.bind(
                    btnExpandTopicMap,
                    Type.SLIDERS,
                    if (topicMapExpanded) "高级话题映射配置 (点击收起)" else "高级话题映射配置",
                    d,
                    iconDp = 11.5f,
                    trailing = if (topicMapExpanded) Type.CHEVRON_UP else Type.CHEVRON_DOWN
                )
            }
        }
        topicCard.addView(btnExpandTopicMap)

        topicMapAccordionBody = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            visibility = View.GONE
            setPadding(0, dp(6f), 0, 0)
        }
        val presetRow = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        presetRow.addView(
            createSmallIconBtn(null, "Tilt-INDI") {
                applyNamedTopicPreset("tilt_indi", quiet = false)
            },
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1.08f)
        )
        presetRow.addView(
            createSmallIconBtn(null, "FAST-LIO") {
                applyNamedTopicPreset("fastlio", quiet = false)
            },
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dp(4f) }
        )
        presetRow.addView(
            createSmallIconBtn(null, "VINS") {
                applyNamedTopicPreset("vins", quiet = false)
            },
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 0.86f).apply { leftMargin = dp(4f) }
        )
        presetRow.addView(
            createSmallIconBtn(null, "Swarm") {
                applyNamedTopicPreset("swarm0", quiet = false)
            },
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 0.92f).apply { leftMargin = dp(4f) }
        )
        topicMapAccordionBody.addView(presetRow)

        inpOdom = labeledInput(topicMapAccordionBody, "里程计位姿 (/Odometry)", cfg.odomTopic)
        inpInflate = labeledInput(topicMapAccordionBody, "三维膨胀地图 (/grid_map/occupancy_inflate)", cfg.inflateTopic)
        inpFastLioCloud = labeledInput(topicMapAccordionBody, "激光点云 (/cloud_registered)", cfg.fastLioCloudTopic)
        inpOptimal = labeledInput(topicMapAccordionBody, "局部轨迹 (/ego_planner_node/optimal_list)", cfg.optimalTopic)
        inpFastLioPath = labeledInput(topicMapAccordionBody, "全局轨迹 (/path)", cfg.fastLioPathTopic)
        inpGoal = labeledInput(topicMapAccordionBody, "目标发布 (/move_base_simple/goal)", cfg.goalTopic)
        inpTiltTelemetry = labeledInput(topicMapAccordionBody, "倾转舵机反馈 (/tilt_uav/servo_angles)", cfg.tiltTelemetryTopic)
        inpTiltCmd = labeledInput(topicMapAccordionBody, "主动倾转指令 (/active_pitch_planner/pitch_tilt_cmd)", cfg.tiltCmdTopic)

        val saveAndCopyRow = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        saveAndCopyRow.addView(
            createSmallIconBtn(Type.DOWNLOAD, "保存映射") {
                hideSoftKeyboard()
                saveCurrentTopics(quiet = false)
            },
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f)
        )
        saveAndCopyRow.addView(
            createSmallIconBtn(Type.COPY, "复制稀疏YAML") {
                val cm = getSystemService(Context.CLIPBOARD_SERVICE) as ClipboardManager
                cm.setPrimaryClip(ClipData.newPlainText("fastlio_yaml", "publish:\n  path_en: true\n  scan_publish_en: true\n  dense_publish_en: false"))
                AnimHelper.showDjiToast(toastBanner, "已复制 FAST-LIO 稀疏点云配置", d)
            },
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1.15f).apply { leftMargin = dp(5f) }
        )
        topicMapAccordionBody.addView(saveAndCopyRow, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(7f) })

        topicCard.addView(topicMapAccordionBody)
        rosSectionInConn.addView(topicCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })
        rosSectionInConn.addView(buildRosSwarmTopologyCard(cfg), LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        col.addView(rosSectionInConn)
        syncProtoInputsVisibility()
        syncDemoSceneSegButtons()

        sv.addView(col)
        return sv
    }

    private fun applyNamedTopicPreset(presetKey: String, quiet: Boolean) {
        when (presetKey) {
            "tilt_indi" -> {
                hub.setActiveFocusTiltRig(true)
                inpOdom.setText("/Odometry")
                inpFastLioCloud.setText("/cloud_registered")
                inpFastLioPath.setText("/path")
                if (!hub.swarmModeEnabled) {
                    inpInflate.setText("/grid_map/occupancy_inflate")
                    inpOptimal.setText("/ego_planner_node/optimal_list")
                }
                inpGoal.setText("/move_base_simple/goal")
                inpTiltTelemetry.setText("/tilt_uav/servo_angles")
                inpTiltCmd.setText("/active_pitch_planner/pitch_tilt_cmd")
                if (::inpSwarmTiltSubPat.isInitialized) {
                    inpSwarmTiltSubPat.setText("/drone_{id}_tilt_uav/servo_angles")
                    inpSwarmTiltCmdPat.setText("/drone_{id}_active_pitch_planner/pitch_tilt_cmd")
                }
                simEngine.notifyOperatingModeChanged()
                DroneVectorModel.activeRig = DroneVectorModel.rigFor(hub.activeFocusIsTiltRig())
                GcsSettingsModal.persistAllSettings(this, hub)
                syncSwarmAllControlsUi()
                syncTiltAllControlsUi()
                attitudeHudView.invalidate()
            }
            "fastlio" -> {
                hub.setActiveFocusTiltRig(false)
                inpOdom.setText("/Odometry")
                inpFastLioCloud.setText("/cloud_registered")
                inpFastLioPath.setText("/path")
                if (!hub.swarmModeEnabled) {
                    inpInflate.setText("/grid_map/occupancy_inflate")
                    inpOptimal.setText("/ego_planner_node/optimal_list")
                }
                inpGoal.setText("/move_base_simple/goal")
                simEngine.notifyOperatingModeChanged()
                DroneVectorModel.activeRig = DroneVectorModel.rigFor(hub.activeFocusIsTiltRig())
                GcsSettingsModal.persistAllSettings(this, hub)
                syncSwarmAllControlsUi()
                syncTiltAllControlsUi()
                attitudeHudView.invalidate()
            }
            "vins" -> {
                hub.setActiveFocusTiltRig(false)
                inpOdom.setText("/vins_estimator/imu_propagate")
                inpFastLioCloud.setText("")
                inpFastLioPath.setText("")
                inpInflate.setText("/grid_map/occupancy_inflate")
                inpOptimal.setText("/ego_planner_node/optimal_list")
                inpGoal.setText("/move_base_simple/goal")
                simEngine.notifyOperatingModeChanged()
                DroneVectorModel.activeRig = DroneVectorModel.rigFor(hub.activeFocusIsTiltRig())
                GcsSettingsModal.persistAllSettings(this, hub)
                syncSwarmAllControlsUi()
                syncTiltAllControlsUi()
                attitudeHudView.invalidate()
            }
            "swarm0" -> {
                hub.swarmModeEnabled = true
                hub.swarmDroneCount = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
                inpOdom.setText("/Odometry")
                inpFastLioCloud.setText("/cloud_registered")
                inpFastLioPath.setText("/path")
                inpInflate.setText("/drone_0_ego_planner_node/grid_map/occupancy_inflate")
                inpOptimal.setText("/drone_0_ego_planner_node/optimal_list")
                inpGoal.setText("/move_base_simple/goal")
                simEngine.notifyOperatingModeChanged()
                DroneVectorModel.activeRig = DroneVectorModel.rigFor(hub.activeFocusIsTiltRig())
                GcsSettingsModal.persistAllSettings(this, hub)
                syncSwarmAllControlsUi()
                syncTiltAllControlsUi()
                attitudeHudView.invalidate()
            }
        }
        saveCurrentTopics(quiet = quiet)
    }

    private fun applyDemoSceneByIndex(newIdx: Int) {
        sceneIdx = newIdx
        val pair = sceneCycle[sceneIdx]
        simEngine.setScenePreset(pair.first)
        GcsIcons.bind(btnDemoScene, Type.CUBE_3D, "场景: ${pair.second}", d, iconDp = 11f)
        syncDemoSceneSegButtons()
        refreshCommandDockUi()
        refreshWaypointOrchestratorList()
        AnimHelper.showDjiToast(toastBanner, "已切换仿真场景: ${pair.second}", d)
    }

    private fun syncDemoSceneSegButtons() {
        val pal = GcsTheme.of(hub.themeId)
        for (i in demoSceneSegBtns.indices) {
            val active = (i == sceneIdx)
            demoSceneSegBtns[i].setTextColor(if (active) pal.textPrimary else pal.textMuted)
            demoSceneSegBtns[i].background = pillBg(
                if (active) pal.accentSoftFill else pal.wellFill,
                if (active) pal.accent else pal.cardStroke
            )
        }
    }

    private fun syncConnModeSectionsUi(showRosEditor: Boolean) {
        if (!::segModeDemoBtn.isInitialized) return
        val pal = GcsTheme.of(hub.themeId)
        val changed = (rosSectionInConn.visibility == View.VISIBLE) != showRosEditor
        demoSectionInConn.visibility = if (showRosEditor) View.GONE else View.VISIBLE
        rosSectionInConn.visibility = if (showRosEditor) View.VISIBLE else View.GONE
        if (changed && drawerOpen) {
            AnimHelper.animateStaggeredChildren(if (showRosEditor) rosSectionInConn else demoSectionInConn, d)
        }

        segModeDemoBtn.setTextColor(if (!showRosEditor) Color.WHITE else pal.textMuted)
        segModeDemoBtn.background = roundedColor(
            if (!showRosEditor) Color.parseColor("#00B865") else Color.TRANSPARENT,
            dp(5.5f)
        )
        segModeDemoBtn.invalidate()

        segModeRosBtn.setTextColor(if (showRosEditor) Color.WHITE else pal.textMuted)
        segModeRosBtn.background = roundedColor(
            if (showRosEditor) pal.accent else Color.TRANSPARENT,
            dp(5.5f)
        )
        segModeRosBtn.invalidate()
    }

    private fun syncProtoInputsVisibility() {
        if (!::segProtoTcpBtn.isInitialized) return
        val pal = GcsTheme.of(hub.themeId)
        val isTcp = rosManager.topics.protocol == "tcpros"
        wrapMasterUri.visibility = if (isTcp) View.VISIBLE else View.GONE
        wrapWsUrl.visibility = if (isTcp) View.GONE else View.VISIBLE

        segProtoTcpBtn.setTextColor(if (isTcp) Color.WHITE else pal.textMuted)
        segProtoTcpBtn.background = roundedColor(if (isTcp) pal.accent else Color.TRANSPARENT, dp(5f))
        segProtoWsBtn.setTextColor(if (!isTcp) Color.WHITE else pal.textMuted)
        segProtoWsBtn.background = roundedColor(if (!isTcp) pal.accent else Color.TRANSPARENT, dp(5f))
    }

    private fun syncUdpSwitchVisual(animate: Boolean) {
        if (!::udpSwitchPill.isInitialized) return
        val pal = GcsTheme.of(hub.themeId)
        udpSwitchPill.background = pillBg(
            if (udpTelemetryEnabledUi) pal.accent else pal.wellFill,
            if (udpTelemetryEnabledUi) pal.accentStroke else pal.cardStroke,
            radiusDp = 11f
        )
        val targetTx = if (udpTelemetryEnabledUi) dp(16f).toFloat() else 0f
        if (animate) {
            udpSwitchThumb.animate()
                .translationX(targetTx)
                .setDuration(165L)
                .setInterpolator(AnimHelper.EASE_OUT_EXPO)
                .start()
        } else {
            udpSwitchThumb.translationX = targetTx
        }
    }

    // ==================== Tab 1: 纯粹专注的航线任务编排面板 ====================

    private fun buildOrchestrationPanel(): View {
        val sv = ScrollView(this)
        val col = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(10f), dp(9f), dp(10f), dp(10f))
        }

        col.addView(buildSwarmControlCardInDrawer())
        col.addView(buildTiltControlCardInDrawer(), LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        val tplCard = sectionCard("智能几何航线模板 (自动避让三维膨胀障碍)")
        val tplRow1 = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        tplRow1.addView(
            createSmallIconBtn(Type.GRID_2X2, "矩形巡检") {
                val (cnt, len) = simEngine.generateMissionTemplate("rect")
                refreshCommandDockUi(pulseDock = true)
                refreshWaypointOrchestratorList()
                AnimHelper.showDjiToast(toastBanner, String.format(Locale.US, "已生成矩形巡检航线：%d 站 (%.1fm)", cnt, len), d)
            },
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f)
        )
        tplRow1.addView(
            createSmallIconBtn(Type.ROUTE_NODES, "S型扫掠") {
                val (cnt, len) = simEngine.generateMissionTemplate("zigzag")
                refreshCommandDockUi(pulseDock = true)
                refreshWaypointOrchestratorList()
                AnimHelper.showDjiToast(toastBanner, String.format(Locale.US, "已生成 S 型扫掠航线：%d 站 (%.1fm)", cnt, len), d)
            },
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dp(4.5f) }
        )
        tplRow1.addView(
            createSmallIconBtn(Type.LOOP_CYCLE, "环绕巡检") {
                val (cnt, len) = simEngine.generateMissionTemplate("orbit")
                refreshCommandDockUi(pulseDock = true)
                refreshWaypointOrchestratorList()
                AnimHelper.showDjiToast(toastBanner, String.format(Locale.US, "已生成环绕巡检航线：%d 站 (%.1fm)", cnt, len), d)
            },
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dp(4.5f) }
        )
        tplCard.addView(tplRow1)

        val tplRow2 = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        tplRow2.addView(
            createSmallIconBtn(Type.PLUS_NODE, "采集当前机位为航点") {
                val (_, total, _) = simEngine.appendCurrentUavAsWaypoint()
                refreshCommandDockUi(pulseDock = true)
                refreshWaypointOrchestratorList()
                AnimHelper.showDjiToast(toastBanner, "已采集当前机位为航点 #$total", d)
            },
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1.35f)
        )
        tplRow2.addView(
            createSmallIconBtn(Type.SLIDERS, "速度与高度设置") {
                setSettingsModalOpen(true, jumpCategory = GcsSettingsModal.CAT_CONTROL)
            },
            LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dp(5f) }
        )
        tplCard.addView(tplRow2, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(5.5f) })
        col.addView(tplCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        val strategyCard = sectionCard("航线循环与完航动作")
        val loopSeg = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dp(2f), dp(2f), dp(2f), dp(2f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        val loopNames = listOf("单次直达", "闭合循环", "往返巡航")
        loopModeSegBtns.clear()
        for (i in loopNames.indices) {
            val b = TextView(this).apply {
                text = loopNames[i]
                maxLines = 1
                setHorizontallyScrolling(false)
                gravity = Gravity.CENTER
                minimumHeight = dp(25f)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
                typeface = Typeface.DEFAULT_BOLD
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    hub.routeLoopMode = i
                    GcsSettingsModal.persistAllSettings(this@MainActivity, hub)
                    syncMissionStrategyButtons()
                    refreshCommandDockUi()
                }
            }
            loopModeSegBtns.add(b)
            loopSeg.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        strategyCard.addView(loopSeg)

        val finishSeg = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dp(2f), dp(2f), dp(2f), dp(2f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        val finNames = listOf("完航末点悬停", "完航自动返航 H")
        finishActSegBtns.clear()
        for (i in finNames.indices) {
            val b = TextView(this).apply {
                text = finNames[i]
                maxLines = 1
                setHorizontallyScrolling(false)
                gravity = Gravity.CENTER
                minimumHeight = dp(25f)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
                typeface = Typeface.DEFAULT_BOLD
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    hub.routeFinishAction = i
                    GcsSettingsModal.persistAllSettings(this@MainActivity, hub)
                    syncMissionStrategyButtons()
                }
            }
            finishActSegBtns.add(b)
            finishSeg.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        strategyCard.addView(finishSeg, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(4.5f) })
        col.addView(strategyCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        val queueCard = sectionCard("航点序列队列 (点击航点平滑运镜预览)")
        val qTopBar = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }
        tvWaypointQueueSummary = TextView(this).apply {
            text = "当前暂无航点"
            maxLines = 1
            setHorizontallyScrolling(false)
            ellipsize = TextUtils.TruncateAt.END
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.5f)
            GcsTheme.markRole(this, Role.TEXT_MUTED)
        }
        val btnSanitizeWp = createSmallIconBtn(Type.SHIELD_SAFE, "自检") {
            val (fixed, total) = simEngine.sanitizeOrchestratedRouteClearance()
            refreshCommandDockUi(pulseDock = true)
            refreshWaypointOrchestratorList()
            val msg = if (total == 0) {
                "当前暂无航点可自检"
            } else if (fixed == 0) {
                "航线净空自检通过：全部 $total 站均安全无遮挡"
            } else {
                "已自动越障修正 $fixed / $total 个近障航点"
            }
            AnimHelper.showDjiToast(toastBanner, msg, d)
        }
        val btnReverse = createSmallIconBtn(Type.SWAP_ORDER, "倒序") {
            if (simEngine.reverseOrchestratedWaypoints()) {
                refreshCommandDockUi(pulseDock = true)
                refreshWaypointOrchestratorList()
                AnimHelper.showDjiToast(toastBanner, "已倒置航点执行顺序", d)
            }
        }
        val btnClearAllWp = createSmallIconBtn(Type.TRASH, "清空") {
            simEngine.clearWaypointQueue()
            refreshCommandDockUi()
            refreshWaypointOrchestratorList()
            AnimHelper.showDjiToast(toastBanner, "已清空全部航点", d)
        }
        qTopBar.addView(tvWaypointQueueSummary, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        qTopBar.addView(btnSanitizeWp)
        qTopBar.addView(btnReverse, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(4f) })
        qTopBar.addView(btnClearAllWp, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(4f) })
        queueCard.addView(qTopBar)

        waypointListContainer = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(0, dp(5.5f), 0, dp(4f))
        }
        queueCard.addView(waypointListContainer)

        btnOrchPrimaryExecute = TextView(this).apply {
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            minimumHeight = dp(32f)
            setTextColor(Color.WHITE)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.8f)
            typeface = Typeface.DEFAULT_BOLD
            background = roundedColor(Color.parseColor("#00B865"), dp(6f))
            GcsIcons.bind(this, Type.PLAY, "启动编排航线", d, iconDp = 11.5f)
            setOnClickListener {
                val (cnt, isStaged, _) = simEngine.routeStatusSummary()
                if (cnt <= 0) {
                    AnimHelper.showDjiToast(toastBanner, "请先在地图选点加入航线或点击上方模板", d)
                } else if (isStaged) {
                    startCommittedRouteExecution(it)
                    setDrawerOpenState(false)
                } else {
                    executeEmergencyHover(it)
                }
            }
        }
        AnimHelper.attachSpringTouch(btnOrchPrimaryExecute)
        queueCard.addView(btnOrchPrimaryExecute, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(4f) })
        col.addView(queueCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        sv.addView(col)
        return sv
    }

    private fun syncMissionStrategyButtons() {
        val pal = GcsTheme.of(hub.themeId)
        for (i in loopModeSegBtns.indices) {
            val sel = (hub.routeLoopMode == i)
            loopModeSegBtns[i].setTextColor(if (sel) Color.WHITE else pal.textMuted)
            loopModeSegBtns[i].background = roundedColor(
                if (sel) pal.accent else Color.TRANSPARENT,
                dp(5f)
            )
        }
        for (i in finishActSegBtns.indices) {
            val sel = (hub.routeFinishAction == i)
            finishActSegBtns[i].setTextColor(if (sel) Color.WHITE else pal.textMuted)
            finishActSegBtns[i].background = roundedColor(
                if (sel) Color.parseColor("#00B865") else Color.TRANSPARENT,
                dp(5f)
            )
        }
    }

    private fun refreshWaypointOrchestratorList() {
        getSharedPreferences(GcsSettingsModal.PREFS_NAME, Context.MODE_PRIVATE).edit()
            .putString("persistedStagedRouteCsv", simEngine.serializeStagedWaypointsCsv())
            .apply()
        if (!::waypointListContainer.isInitialized) return
        val pal = GcsTheme.of(hub.themeId)
        waypointListContainer.removeAllViews()
        val (isStaged, nodes) = simEngine.snapshotOrchestratedWaypoints()
        val (_, _, arcLen) = simEngine.routeStatusSummary()

        if (nodes.isEmpty()) {
            setTextIfChanged(tvWaypointQueueSummary, "暂无航点 · 点地图加航点或选上方模板")
            btnOrchPrimaryExecute.alpha = 0.55f
            GcsIcons.bind(btnOrchPrimaryExecute, Type.PLAY, "暂无待执行航点", d, iconDp = 11.5f)
            btnOrchPrimaryExecute.background = roundedColor(Color.rgb(66, 78, 96), dp(6f))
            return
        }

        btnOrchPrimaryExecute.alpha = 1.0f
        if (isStaged) {
            setTextIfChanged(
                tvWaypointQueueSummary,
                String.format(Locale.US, "待飞 %d 站 · 总长 %.1fm", nodes.size, arcLen)
            )
            GcsIcons.bind(btnOrchPrimaryExecute, Type.PLAY, "启动编排航线 (${nodes.size}站)", d, iconDp = 11.5f)
            btnOrchPrimaryExecute.background = roundedColor(Color.parseColor("#00B865"), dp(6f))
        } else {
            setTextIfChanged(tvWaypointQueueSummary, "执行中 · 剩余 ${nodes.size} 站")
            GcsIcons.bind(btnOrchPrimaryExecute, Type.STOP_SQUARE, "终止航线并悬停", d, iconDp = 11.5f)
            btnOrchPrimaryExecute.background = roundedColor(Color.parseColor("#FF3B30"), dp(6f))
        }

        for (i in nodes.indices) {
            val wp = nodes[i]
            val row = LinearLayout(this).apply {
                orientation = LinearLayout.HORIZONTAL
                gravity = Gravity.CENTER_VERTICAL
                setPadding(dp(7f), dp(5f), dp(7f), dp(5f))
                background = roundedColor(pal.wellFill, dp(6f))
            }

            val infoTv = TextView(this).apply {
                maxLines = 2
                setTextColor(if (wp.isSafe) pal.textPrimary else Color.parseColor("#FF5252"))
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
                typeface = Typeface.MONOSPACE
                GcsIcons.bind(
                    this,
                    if (wp.isSafe) Type.STATUS_DOT else Type.WARNING,
                    String.format(Locale.US, "#%d (%.1f,%.1f) %.1fm", i + 1, wp.x, wp.y, wp.z),
                    d,
                    iconDp = 9.5f,
                    gapDp = 3.5f
                )
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    mapView.focusWorldPointSmooth(wp.x, wp.y)
                    syncCameraControlLabels()
                    AnimHelper.pulseViewSubtle(row)
                }
            }

            fun tinySquareIcon(ic: Type, enabled: Boolean, onTap: () -> Unit): ImageView = ImageView(this).apply {
                scaleType = ImageView.ScaleType.CENTER
                setImageDrawable(
                    GcsIcons.drawable(
                        ic,
                        dp(11f),
                        if (enabled) pal.textPrimary else pal.textHint
                    )
                )
                background = roundedColor(if (enabled) pal.cardFill else Color.TRANSPARENT, dp(4.5f))
                isEnabled = enabled
                setOnClickListener {
                    if (enabled) {
                        AnimHelper.triggerHaptic(it)
                        onTap()
                    }
                }
            }

            fun tinyTextStepBtn(lbl: String, onTap: () -> Unit): TextView = TextView(this).apply {
                text = lbl
                gravity = Gravity.CENTER
                setTextColor(pal.accentBright)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 11f)
                typeface = Typeface.DEFAULT_BOLD
                background = roundedColor(pal.accentSoftFill, dp(4.5f))
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    onTap()
                }
            }

            val btnUp = tinySquareIcon(Type.ARROW_UP, i > 0) {
                if (simEngine.moveOrchestratedWaypoint(i, -1)) {
                    refreshCommandDockUi()
                    refreshWaypointOrchestratorList()
                }
            }
            val btnDown = tinySquareIcon(Type.ARROW_DOWN, i < nodes.lastIndex) {
                if (simEngine.moveOrchestratedWaypoint(i, +1)) {
                    refreshCommandDockUi()
                    refreshWaypointOrchestratorList()
                }
            }
            val btnZMinus = tinyTextStepBtn("-") {
                simEngine.nudgeOrchestratedWaypointZ(i, -hub.altStepMeters)
                refreshCommandDockUi()
                refreshWaypointOrchestratorList()
            }
            val btnZPlus = tinyTextStepBtn("+") {
                simEngine.nudgeOrchestratedWaypointZ(i, +hub.altStepMeters)
                refreshCommandDockUi()
                refreshWaypointOrchestratorList()
            }
            val btnDel = tinySquareIcon(Type.CLOSE_X, true) {
                simEngine.removeOrchestratedWaypointAt(i)
                refreshCommandDockUi()
                refreshWaypointOrchestratorList()
            }

            row.addView(infoTv, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
            row.addView(btnZMinus, LinearLayout.LayoutParams(dp(23f), dp(23f)).apply { leftMargin = dp(3f) })
            row.addView(btnZPlus, LinearLayout.LayoutParams(dp(23f), dp(23f)).apply { leftMargin = dp(2.5f) })
            row.addView(btnUp, LinearLayout.LayoutParams(dp(23f), dp(23f)).apply { leftMargin = dp(3f) })
            row.addView(btnDown, LinearLayout.LayoutParams(dp(23f), dp(23f)).apply { leftMargin = dp(2.5f) })
            row.addView(btnDel, LinearLayout.LayoutParams(dp(23f), dp(23f)).apply { leftMargin = dp(3f) })

            waypointListContainer.addView(
                row,
                LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply {
                    if (i > 0) topMargin = dp(4f)
                }
            )
        }
    }

    private fun applyFlightParametersToEngine(inf: Float, vel: Float, acc: Float, gz: Float) {
        hub.obstaclesInflation = inf
        hub.maxVel = vel
        hub.maxAcc = acc
        val deltaZ = gz - hub.goalZ
        val (_, pinInfo) = simEngine.nudgeGoalZ(deltaZ)

        simEngine.requestReplanAsync()
        rosManager.setParamDebounced("/ego_planner_node/grid_map/obstacles_inflation", inf)
        rosManager.setParamDebounced("/ego_planner_node/manager/max_vel", vel)
        rosManager.setParamDebounced("/ego_planner_node/manager/max_acc", acc)
        refreshCommandDockUi(pinInfo)
    }

    // ==================== Tab 2: ROS1 话题监控 + 单次截取/持续读取 ====================

    private fun buildTopicInspectorPanel(): View {
        val host = FrameLayout(this)

        topicListPage = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(10f), dp(9f), dp(10f), dp(9f))
        }

        val topFilterRow = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }
        inpTopicFilter = EditText(this).apply {
            hint = "过滤话题 (如 grid_map / mavros)..."
            setSingleLine(true)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
            typeface = Typeface.MONOSPACE
            setPadding(dp(8f), dp(5f), dp(8f), dp(5f))
            GcsTheme.markRole(this, Role.INPUT_BOX, cornerDp = 6f)
            addTextChangedListener(object : TextWatcher {
                override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}
                override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {
                    renderFilteredTopicRows()
                }
                override fun afterTextChanged(s: Editable?) {}
            })
        }
        val btnRefreshTopics = TextView(this).apply {
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            minimumHeight = dp(30f)
            setTextColor(Color.WHITE)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(9f), dp(5f), dp(9f), dp(5f))
            background = roundedColor(Color.parseColor("#1890FF"), dp(6f))
            GcsIcons.bind(this, Type.REFRESH, "刷新", d, iconDp = 11.5f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                hideSoftKeyboard()
                refreshRosTopicList()
            }
        }
        AnimHelper.attachSpringTouch(btnRefreshTopics)

        inpTopicFilter.minimumHeight = dp(34f)
        topFilterRow.addView(inpTopicFilter, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        topFilterRow.addView(btnRefreshTopics, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(5f) })
        topicListPage.addView(topFilterRow)

        tvTopicSourceHint = TextView(this).apply {
            text = "点击话题可执行单次截取、持续读取或一键绑定通道"
            maxLines = 2
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
            setPadding(dp(2f), dp(4.5f), 0, dp(4.5f))
            GcsTheme.markRole(this, Role.TEXT_MUTED)
        }
        topicListPage.addView(tvTopicSourceHint)

        val listScroll = ScrollView(this)
        topicItemsContainer = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        listScroll.addView(topicItemsContainer)
        topicListPage.addView(listScroll, LinearLayout.LayoutParams(MATCH_PARENT, 0, 1f))

        topicDetailPage = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            visibility = View.GONE
            setPadding(dp(10f), dp(8f), dp(10f), dp(9f))
        }

        val detailNavRow = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }
        val btnBackList = createSmallIconBtn(Type.CHEVRON_LEFT, "列表") {
            exitTopicDetailToList()
        }
        val titleCol = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(6f), 0, dp(5f), 0)
        }
        tvDetailTopicName = TextView(this).apply {
            text = "/grid_map/occupancy_inflate"
            maxLines = 2
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.5f)
            typeface = Typeface.MONOSPACE
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
        }
        tvDetailTopicType = TextView(this).apply {
            text = "sensor_msgs/PointCloud2"
            maxLines = 1
            setHorizontallyScrolling(false)
            ellipsize = TextUtils.TruncateAt.END
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 8.8f)
            typeface = Typeface.MONOSPACE
            GcsTheme.markRole(this, Role.TEXT_ACCENT)
        }
        titleCol.addView(tvDetailTopicName)
        titleCol.addView(tvDetailTopicType)

        btnWideTerminal = createSmallIconBtn(Type.EXPAND_WIDTH, "宽屏") {
            setWideTerminalMode(!drawerWideMode, applyImmediately = true)
        }
        val btnCopyEcho = ImageView(this).apply {
            scaleType = ImageView.ScaleType.CENTER
            setImageDrawable(GcsIcons.drawable(Type.COPY, dp(13f), Color.rgb(120, 160, 210)))
            GcsTheme.markRole(this, Role.GLASS_PANEL, cornerDp = 6f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                val entry = selectedTopicEntry
                val clipText = buildString {
                    append("Topic: ${entry?.name ?: tvDetailTopicName.text}\n")
                    append("Type: ${entry?.type ?: tvDetailTopicType.text}\n")
                    append("${tvDetailMetricsStrip.text}\n---\n")
                    append(tvDetailPayloadConsole.text)
                }
                val cm = getSystemService(Context.CLIPBOARD_SERVICE) as ClipboardManager
                cm.setPrimaryClip(ClipData.newPlainText("ros_topic_echo", clipText))
                AnimHelper.showDjiToast(toastBanner, "已复制当前话题数据", d)
            }
        }
        AnimHelper.attachSpringTouch(btnCopyEcho)

        detailNavRow.addView(btnBackList)
        detailNavRow.addView(titleCol, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        detailNavRow.addView(btnWideTerminal)
        detailNavRow.addView(btnCopyEcho, LinearLayout.LayoutParams(dp(28f), dp(27f)).apply { leftMargin = dp(4f) })
        topicDetailPage.addView(detailNavRow)

        val actionBar = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }

        val btnOnce = TextView(this).apply {
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            minimumHeight = dp(30f)
            setTextColor(Color.WHITE)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.5f)
            typeface = Typeface.DEFAULT_BOLD
            background = roundedColor(Color.parseColor("#1890FF"), dp(6f))
            GcsIcons.bind(this, Type.CAPTURE_FRAME, "截取一次", d, iconDp = 12f)
            setOnClickListener {
                val entry = selectedTopicEntry ?: return@setOnClickListener
                AnimHelper.triggerHaptic(it)
                isDetailCurrentlyStreaming = false
                syncDetailStreamButtonUi()
                topicInspector.captureOnce(entry.name, entry.type) { snap ->
                    runOnUiThread { applyTopicEchoSnapshot(snap) }
                }
            }
        }
        AnimHelper.attachSpringTouch(btnOnce)

        btnDetailStreamToggle = TextView(this).apply {
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            minimumHeight = dp(30f)
            setTextColor(Color.WHITE)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.5f)
            typeface = Typeface.DEFAULT_BOLD
            background = roundedColor(Color.parseColor("#00B865"), dp(6f))
            GcsIcons.bind(this, Type.PLAY, "持续读取", d, iconDp = 11.5f)
            setOnClickListener {
                val entry = selectedTopicEntry ?: return@setOnClickListener
                AnimHelper.triggerHaptic(it, HapticFeedbackConstants.LONG_PRESS)
                if (isDetailCurrentlyStreaming) {
                    topicInspector.stopActiveSession()
                    isDetailCurrentlyStreaming = false
                    syncDetailStreamButtonUi()
                    tvDetailStatusStrip.setTextColor(Color.rgb(215, 130, 0))
                    GcsIcons.bind(tvDetailStatusStrip, Type.PAUSE, "已暂停持续读取", d, iconDp = 11f)
                } else {
                    isDetailCurrentlyStreaming = true
                    syncDetailStreamButtonUi()
                    topicInspector.startContinuous(entry.name, entry.type) { snap ->
                        runOnUiThread { applyTopicEchoSnapshot(snap) }
                    }
                }
            }
        }
        AnimHelper.attachSpringTouch(btnDetailStreamToggle)

        actionBar.addView(btnOnce, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        actionBar.addView(btnDetailStreamToggle, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1.1f).apply { leftMargin = dp(5f) })
        topicDetailPage.addView(actionBar, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(6f) })

        btnBindTopicChannel = TextView(this).apply {
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            ellipsize = TextUtils.TruncateAt.END
            minimumHeight = dp(27f)
            setPadding(dp(8f), dp(4f), dp(8f), dp(4f))
            visibility = View.GONE
            setTextColor(Color.rgb(215, 130, 0))
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.8f)
            typeface = Typeface.DEFAULT_BOLD
            background = pillBg(Color.argb(42, 255, 171, 0), Color.argb(135, 255, 171, 0))
            GcsIcons.bind(this, Type.LINK, "绑定为地面站通道", d, iconDp = 11f)
            setOnClickListener {
                val entry = selectedTopicEntry ?: return@setOnClickListener
                AnimHelper.triggerHaptic(it)
                bindInspectedTopicToGcs(entry)
            }
        }
        AnimHelper.attachSpringTouch(btnBindTopicChannel)
        topicDetailPage.addView(btnBindTopicChannel, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(5f) })

        val telemetryStripCard = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(7.5f), dp(5f), dp(7.5f), dp(5f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        tvDetailStatusStrip = TextView(this).apply {
            maxLines = 2
            setTextColor(Color.parseColor("#00B865"))
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.5f)
            typeface = Typeface.DEFAULT_BOLD
            GcsIcons.bind(this, Type.STATUS_DOT, "准备就绪", d, iconDp = 10.5f)
        }
        tvDetailMetricsStrip = monoLabel("频率: -- Hz  |  带宽: -- KB/s  |  已收: 0 帧", "#B8C6D8", 9.0f, Role.TEXT_MUTED).apply {
            setPadding(0, dp(2f), 0, 0)
        }
        telemetryStripCard.addView(tvDetailStatusStrip)
        telemetryStripCard.addView(tvDetailMetricsStrip)
        topicDetailPage.addView(telemetryStripCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(5f) })

        val consoleScroll = ScrollView(this).apply {
            setPadding(dp(8f), dp(7f), dp(8f), dp(7f))
            GcsTheme.markRole(this, Role.CONSOLE_BOX, cornerDp = 6.5f)
        }
        tvDetailPayloadConsole = TextView(this).apply {
            text = "# 尚未读取数据"
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.8f)
            typeface = Typeface.MONOSPACE
            setLineSpacing(dp(1.5f).toFloat(), 1.0f)
            setTextIsSelectable(true)
            GcsTheme.markRole(this, Role.TEXT_CONSOLE)
        }
        consoleScroll.addView(tvDetailPayloadConsole)
        topicDetailPage.addView(consoleScroll, LinearLayout.LayoutParams(MATCH_PARENT, 0, 1f).apply { topMargin = dp(5f) })

        host.addView(topicListPage, FrameLayout.LayoutParams(MATCH_PARENT, MATCH_PARENT))
        host.addView(topicDetailPage, FrameLayout.LayoutParams(MATCH_PARENT, MATCH_PARENT))
        return host
    }

    private fun updateSmartBindButtonForTopic(entry: RosTopicEntry) {
        val t = entry.type
        val n = entry.name.lowercase(Locale.US)
        val label: String? = when {
            t.contains("Odometry") -> "一键设为「里程计位姿」话题"
            t.contains("PointCloud2") && (n.contains("inflate") || n.contains("occupancy") || n.contains("grid")) ->
                "一键设为「EGO 三维膨胀地图」话题"
            t.contains("PointCloud2") ->
                "一键设为「FAST-LIO 激光点云」话题"
            t.contains("Marker") -> "一键设为「EGO 局部规划轨迹」话题"
            t.contains("Path") && n.contains("optimal") -> "一键设为「EGO 局部规划轨迹」话题"
            t.contains("Path") -> "一键设为「FAST-LIO 全局轨迹」话题"
            t.contains("PoseStamped") -> "一键设为「指点飞行目标发布」话题"
            t.contains("ActuatorControl") || t.contains("Float32MultiArray") || t.contains("Float64MultiArray") || t.contains("JointState") ->
                "一键设为「倾转舵机遥测」话题"
            t.contains("Vector3Stamped") && (n.contains("cmd") || n.contains("pitch")) ->
                "一键设为「主动俯仰-倾转指令」话题"
            t.contains("Vector3Stamped") ->
                "一键设为「倾转舵机遥测」话题"
            else -> null
        }
        if (label != null) {
            GcsIcons.bind(btnBindTopicChannel, Type.LINK, label, d, iconDp = 11f)
            btnBindTopicChannel.visibility = View.VISIBLE
        } else {
            btnBindTopicChannel.visibility = View.GONE
        }
    }

    private fun bindInspectedTopicToGcs(entry: RosTopicEntry) {
        val t = entry.type
        val name = entry.name
        val roleDesc: String
        when {
            t.contains("Odometry") -> {
                inpOdom.setText(name)
                roleDesc = "里程计位姿"
            }
            t.contains("PointCloud2") -> {
                val curInf = inpInflate.text.toString().trim()
                val preferInflate = (name.contains("inflate", true) || name.contains("occupancy", true) || name.contains("grid", true))
                if (preferInflate && curInf != name) {
                    inpInflate.setText(name)
                    roleDesc = "EGO 三维膨胀体素地图"
                } else if (!preferInflate && inpFastLioCloud.text.toString().trim() != name) {
                    inpFastLioCloud.setText(name)
                    roleDesc = "FAST-LIO 世界系点云"
                } else if (curInf != name) {
                    inpInflate.setText(name)
                    roleDesc = "EGO 三维膨胀体素地图"
                } else {
                    inpFastLioCloud.setText(name)
                    roleDesc = "FAST-LIO 世界系点云"
                }
            }
            t.contains("Marker") || (t.contains("Path") && name.contains("optimal", true)) -> {
                inpOptimal.setText(name)
                roleDesc = "EGO 局部规划轨迹"
            }
            t.contains("Path") -> {
                inpFastLioPath.setText(name)
                roleDesc = "FAST-LIO 全局轨迹"
            }
            t.contains("PoseStamped") -> {
                inpGoal.setText(name)
                roleDesc = "指点目标发布"
            }
            t.contains("Vector3Stamped") && (name.contains("cmd", true) || name.contains("pitch", true)) -> {
                inpTiltCmd.setText(name)
                roleDesc = "主动俯仰-倾转指令"
            }
            t.contains("ActuatorControl") || t.contains("MultiArray") || t.contains("JointState") || t.contains("Vector3Stamped") -> {
                inpTiltTelemetry.setText(name)
                roleDesc = "倾转舵机遥测"
            }
            else -> return
        }
        saveCurrentTopics(quiet = true)
        AnimHelper.showDjiToast(toastBanner, "已将 $name 设为「$roleDesc」", d)
    }

    private fun refreshRosTopicList() {
        if (!::tvTopicSourceHint.isInitialized) return
        if (::inpMasterUri.isInitialized && ::inpWsUrl.isInitialized) {
            val typedMaster = inpMasterUri.text?.toString()?.trim().orEmpty()
            val typedWs = inpWsUrl.text?.toString()?.trim().orEmpty()
            if (typedMaster.isNotEmpty() && typedMaster != rosManager.topics.masterUri) {
                rosManager.topics.masterUri = typedMaster
            }
            if (typedWs.isNotEmpty() && typedWs != rosManager.topics.wsUrl) {
                rosManager.topics.wsUrl = typedWs
            }
        }
        GcsIcons.bind(tvTopicSourceHint, Type.REFRESH, "正在查询活跃 ROS1 话题...", d, iconDp = 10.5f)
        topicInspector.fetchTopicsAsync { list, hint ->
            runOnUiThread {
                cachedTopicEntries = list
                GcsIcons.bind(tvTopicSourceHint, Type.RADAR_SIGNAL, "$hint · 点击进入监控", d, iconDp = 10.5f)
                renderFilteredTopicRows()
                AnimHelper.animateStaggeredChildren(topicItemsContainer, d)
            }
        }
    }

    private fun renderFilteredTopicRows() {
        if (!::topicItemsContainer.isInitialized) return
        val pal = GcsTheme.of(hub.themeId)
        topicItemsContainer.removeAllViews()
        val kw = inpTopicFilter.text?.toString()?.trim()?.lowercase(Locale.US) ?: ""
        val filtered = if (kw.isEmpty()) {
            cachedTopicEntries
        } else {
            cachedTopicEntries.filter {
                it.name.lowercase(Locale.US).contains(kw) || it.type.lowercase(Locale.US).contains(kw)
            }
        }

        if (filtered.isEmpty()) {
            topicItemsContainer.addView(
                TextView(this).apply {
                    text = if (cachedTopicEntries.isEmpty()) "暂无可用话题，请连接 roscore 或点击「刷新」" else "没有匹配 \"$kw\" 的话题"
                    setTextColor(pal.textMuted)
                    setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10f)
                    setPadding(dp(4f), dp(14f), dp(4f), dp(14f))
                }
            )
            return
        }

        for (entry in filtered) {
            val activePub = entry.publisherCount > 0
            val row = LinearLayout(this).apply {
                orientation = LinearLayout.HORIZONTAL
                gravity = Gravity.CENTER_VERTICAL
                setPadding(dp(8.5f), dp(6.5f), dp(8.5f), dp(6.5f))
                background = GradientDrawable().apply {
                    setColor(pal.wellFill)
                    cornerRadius = dp(6.5f).toFloat()
                    setStroke(1, if (activePub) pal.accentStroke else pal.cardStroke)
                }
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    hideSoftKeyboard()
                    openTopicDetail(entry)
                }
            }
            AnimHelper.attachSpringTouch(row)

            val col = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
            val nameTv = TextView(this).apply {
                text = entry.name
                maxLines = 2
                setTextColor(pal.textPrimary)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.5f)
                typeface = Typeface.MONOSPACE
            }
            val metaTv = TextView(this).apply {
                maxLines = 1
                setHorizontallyScrolling(false)
                ellipsize = TextUtils.TruncateAt.END
                setTextColor(if (activePub) pal.secondaryAccent else pal.textMuted)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.0f)
                typeface = Typeface.MONOSPACE
                GcsIcons.bind(
                    this,
                    if (activePub) Type.STATUS_DOT else Type.STATUS_RING,
                    "${entry.type}  [${entry.publisherCount} pub]",
                    d,
                    iconDp = 9f,
                    gapDp = 3.5f
                )
            }
            col.addView(nameTv)
            col.addView(metaTv, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(1.5f) })

            val enterHint = TextView(this).apply {
                maxLines = 1
                setHorizontallyScrolling(false)
                setTextColor(pal.accentBright)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
                typeface = Typeface.DEFAULT_BOLD
                GcsIcons.bind(this, null, "查看", d, trailing = Type.CHEVRON_RIGHT, trailDp = 10f)
            }

            row.addView(col, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
            row.addView(enterHint, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(6f) })
            topicItemsContainer.addView(row, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { bottomMargin = dp(4.5f) })
        }
    }

    private fun openTopicDetail(entry: RosTopicEntry) {
        selectedTopicEntry = entry
        topicListPage.visibility = View.GONE
        topicDetailPage.visibility = View.VISIBLE
        AnimHelper.animateStaggeredChildren(topicDetailPage, d)
        setTextIfChanged(tvDetailTopicName, entry.name)
        setTextIfChanged(tvDetailTopicType, entry.type)
        updateSmartBindButtonForTopic(entry)

        isDetailCurrentlyStreaming = false
        syncDetailStreamButtonUi()
        topicInspector.captureOnce(entry.name, entry.type) { snap ->
            runOnUiThread { applyTopicEchoSnapshot(snap) }
        }
    }

    private fun exitTopicDetailToList() {
        topicInspector.stopActiveSession()
        isDetailCurrentlyStreaming = false
        syncDetailStreamButtonUi()
        if (drawerWideMode) {
            setWideTerminalMode(false, applyImmediately = true)
        }
        topicDetailPage.visibility = View.GONE
        topicListPage.visibility = View.VISIBLE
        AnimHelper.animateStaggeredChildren(topicListPage, d)
    }

    private fun syncDetailStreamButtonUi() {
        if (!::btnDetailStreamToggle.isInitialized) return
        if (isDetailCurrentlyStreaming) {
            GcsIcons.bind(btnDetailStreamToggle, Type.STOP_SQUARE, "停止读取", d, iconDp = 11f)
            btnDetailStreamToggle.background = roundedColor(Color.parseColor("#FF3B30"), dp(6f))
        } else {
            GcsIcons.bind(btnDetailStreamToggle, Type.PLAY, "持续读取", d, iconDp = 11.5f)
            btnDetailStreamToggle.background = roundedColor(Color.parseColor("#00B865"), dp(6f))
        }
    }

    private fun applyTopicEchoSnapshot(snap: TopicEchoSnapshot) {
        if (selectedTopicEntry?.name != snap.topic) return
        val pal = GcsTheme.of(hub.themeId)
        isDetailCurrentlyStreaming = snap.isStreaming
        syncDetailStreamButtonUi()
        if (selectedTopicEntry != null && snap.msgType.isNotBlank() && snap.msgType != "unknown") {
            val updated = selectedTopicEntry!!.copy(type = snap.msgType)
            selectedTopicEntry = updated
            updateSmartBindButtonForTopic(updated)
        }
        setTextIfChanged(tvDetailTopicType, "${snap.msgType} · ${snap.publisherNode}")

        val (stColor, stIcon) = when (snap.state) {
            TopicEchoState.WARNING -> Color.parseColor("#FF5252") to Type.WARNING
            TopicEchoState.STREAMING -> pal.secondaryAccent to Type.STATUS_DOT
            TopicEchoState.CAPTURED -> pal.accentBright to Type.CAPTURE_FRAME
            TopicEchoState.PAUSED -> Color.rgb(215, 130, 0) to Type.PAUSE
            TopicEchoState.CONNECTING -> Color.rgb(215, 130, 0) to Type.REFRESH
        }
        tvDetailStatusStrip.setTextColor(stColor)
        GcsIcons.bind(tvDetailStatusStrip, stIcon, "${snap.statusText} [${snap.timestampText}]", d, iconDp = 10.5f)

        setTextIfChanged(
            tvDetailMetricsStrip,
            String.format(
                Locale.US,
                "%.1f Hz | %.1f KB/s | 已收 %d 帧 (%d B)",
                snap.hz,
                snap.kbps,
                snap.receivedCount,
                snap.lastFrameBytes
            )
        )
        setTextIfChanged(tvDetailPayloadConsole, snap.bodyText)
    }

    // ==================== Tab 3: 状态与双电源遥测面板 ====================

    private fun buildTelemetryPanel(): View {
        val sv = ScrollView(this)
        val col = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(10f), dp(9f), dp(10f), dp(10f))
        }

        tvUavPowerCard = monoLabel("16.24V · 12.6A (86%)", "#00B865", 11.2f)
        tvTxPowerCard = monoLabel("8.08V · 1.05A (92%)", "#69B7FF", 11.2f, Role.TEXT_ACCENT)
        val swarmStatCard = sectionCard("多机集群实时态势与机间防撞净空")
        tvSwarmFleetConsole = TextView(this).apply {
            text = "当前为单机模式 · 可点击顶栏「单机模式」切换为 2~4 架协同编队"
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
            typeface = Typeface.MONOSPACE
            setLineSpacing(dp(1.8f).toFloat(), 1.0f)
            setPadding(dp(8f), dp(6f), dp(8f), dp(6f))
            GcsTheme.markRole(this, Role.CONSOLE_BOX, cornerDp = 6f)
        }
        swarmStatCard.addView(tvSwarmFleetConsole)
        col.addView(swarmStatCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { bottomMargin = dp(8f) })

        val tiltStatCard = sectionCard("Tilt-UAV-INDI 执行器与控制分配遥测 (4M+2S)")
        tvTiltIndiConsole = TextView(this).apply {
            text = "S1(M1/M3): +0.0°  |  S2(M2/M4): +0.0°"
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
            typeface = Typeface.MONOSPACE
            setLineSpacing(dp(1.8f).toFloat(), 1.0f)
            setPadding(dp(8f), dp(6f), dp(8f), dp(6f))
            GcsTheme.markRole(this, Role.CONSOLE_BOX, cornerDp = 6f)
        }
        tiltStatCard.addView(tvTiltIndiConsole)
        col.addView(tiltStatCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { bottomMargin = dp(8f) })

        val pwrCard = sectionCard("双路电源遥测 (当前焦点飞机 + 遥控器本机)")
        pwrCard.addView(fullMetricRow("无人机动力电池", tvUavPowerCard))
        pwrCard.addView(fullMetricRow("遥控双 21700 电池", tvTxPowerCard))
        val btnSafetyThresholds = createSmallIconBtn(Type.SHIELD_SAFE, "调整低电量与避障告警阈值") {
            setSettingsModalOpen(true, jumpCategory = GcsSettingsModal.CAT_SAFETY)
        }
        pwrCard.addView(btnSafetyThresholds, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(5.5f) })
        col.addView(pwrCard)

        tvPosX = monoLabel("0.00 m", "#FFFFFF", 11.5f, Role.TEXT_PRIMARY)
        tvPosY = monoLabel("0.00 m", "#FFFFFF", 11.5f, Role.TEXT_PRIMARY)
        tvPosZ = monoLabel("1.20 m", "#FFFFFF", 11.5f, Role.TEXT_PRIMARY)
        tvVelNorm = monoLabel("0.00 m/s", "#FFFFFF", 11.5f, Role.TEXT_PRIMARY)
        tvRollPitch = monoLabel("R +0.0° · P +0.0°", "#69B7FF", 11.2f, Role.TEXT_ACCENT)
        tvQuatSummary = monoLabel("1.00, 0.00, 0.00, 0.00", "#FFFFFF", 10.6f, Role.TEXT_PRIMARY)
        tvYaw = monoLabel("0.0°", "#FFFFFF", 11.5f, Role.TEXT_PRIMARY)
        tvGoalDist = monoLabel("0.00 m", "#FFFFFF", 11.5f, Role.TEXT_PRIMARY)

        val poseCard = sectionCard("无人机实时里程计与姿态 (/Odometry)")
        poseCard.addView(twoColRow("位置 X", tvPosX, "位置 Y", tvPosY))
        poseCard.addView(twoColRow("高度 Z", tvPosZ, "合速度 |V|", tvVelNorm))
        poseCard.addView(twoColRow("偏航 Yaw", tvYaw, "目标距离", tvGoalDist))
        poseCard.addView(fullMetricRow("横滚 / 俯仰角", tvRollPitch))
        poseCard.addView(fullMetricRow("姿态四元数 (w,x,y,z)", tvQuatSummary))
        col.addView(poseCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        tvClearanceCard = monoLabel("1.45 m", "#00B865", 12f)
        tvSolveCard = monoLabel("0.4 ms", "#69B7FF", 12f, Role.TEXT_ACCENT)
        tvVoxelCount = monoLabel("1620 blks", "#FFFFFF", 12f, Role.TEXT_PRIMARY)
        tvTrajLen = monoLabel("6.2 m", "#FFFFFF", 12f, Role.TEXT_PRIMARY)

        val egoCard = sectionCard("EGO 膨胀体素与局部规划")
        egoCard.addView(twoColRow("最近障碍净空", tvClearanceCard, "规划耗时", tvSolveCard))
        egoCard.addView(twoColRow("膨胀体素数", tvVoxelCount, "轨迹弧长", tvTrajLen))
        col.addView(egoCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        tvMapPtsCard = monoLabel("0 / 300k", "#00B865", 11.5f)
        tvCloudRateCard = monoLabel("15 Hz · 0 pts", "#69B7FF", 11.5f, Role.TEXT_ACCENT)
        tvMapBoundsCard = monoLabel("0.0 x 0.0 x 0.0 m", "#FFFFFF", 10.8f, Role.TEXT_PRIMARY)
        val lioCard = sectionCard("FAST-LIO 三维点云建图统计")
        lioCard.addView(fullMetricRow("累积三维体素点", tvMapPtsCard))
        lioCard.addView(fullMetricRow("实时点云流帧率", tvCloudRateCard))
        lioCard.addView(fullMetricRow("建图三维包络跨度", tvMapBoundsCard))
        val btnExportInDrawer = createSmallIconBtn(Type.DOWNLOAD, "导出标准 .pcd 三维点云文件") {
            exportCurrentPointCloudPcd()
        }
        lioCard.addView(btnExportInDrawer, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(5.5f) })
        col.addView(lioCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        val recCard = sectionCard("科研黑匣子 20Hz 同步遥测记录 (导出论文 CSV)")
        tvRecorderStatusCard = monoLabel("待机 · 已缓存 0 帧", "#B8C0CC", 10.4f, Role.TEXT_SECONDARY)
        recCard.addView(fullMetricRow("记录器状态", tvRecorderStatusCard))
        val recBtnRow = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        btnToggleFlightRec = createSmallIconBtn(Type.PLAY, "开始记录") {
            val rec = simEngine.telemetryRecorder
            if (rec.isRecording) {
                rec.stopRecording()
                syncFlightRecorderUi()
                AnimHelper.showDjiToast(toastBanner, "已暂停遥测记录 (共 ${rec.sampleCount} 帧 · 可点击右侧导出 CSV)", d)
            } else {
                rec.startRecording(android.os.SystemClock.elapsedRealtime())
                syncFlightRecorderUi()
                AnimHelper.showDjiToast(toastBanner, "已开启 20Hz 科研遥测黑匣子同步记录", d)
            }
        }
        val btnExportCsv = createSmallIconBtn(Type.DOWNLOAD, "导出 CSV") {
            exportFlightTelemetryCsv()
        }
        recBtnRow.addView(btnToggleFlightRec, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        recBtnRow.addView(btnExportCsv, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dp(5f) })
        recCard.addView(recBtnRow, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(5.5f) })
        col.addView(recCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        val otaShortcutCard = sectionCard("软件版本与 GitHub 直链在线更新")
        val btnJumpOta = createSmallIconBtn(Type.REFRESH, "打开 GitHub 直链在线更新中心") {
            setDrawerOpenState(false)
            setSettingsModalOpen(true, jumpCategory = GcsSettingsModal.CAT_GENERAL)
        }
        otaShortcutCard.addView(btnJumpOta)
        col.addView(otaShortcutCard, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(8f) })

        sv.addView(col)
        return sv
    }

    // ==================== 局域网寻机与连接管理 ====================

    private fun triggerLanScan(silentIfEmpty: Boolean, autoOpenConnTabOnEmpty: Boolean) {
        if (lanScanner.isScanning) {
            lanScanner.cancelScan()
            GcsIcons.bind(btnDrawerScan, Type.SEARCH, "扫描局域网飞机", d, iconDp = 12f)
            setTextIfChanged(tvScanStatus, "扫描已停止")
            return
        }

        discoveredDrones.clear()
        discoveredListContainer.removeAllViews()
        GcsIcons.bind(btnDrawerScan, Type.STOP_SQUARE, "扫描中 (点此停止)", d, iconDp = 11f)
        setTextIfChanged(tvScanStatus, "正在并发探测局域网 ROS1 机载电脑...")
        if (!silentIfEmpty) {
            AnimHelper.showDjiToast(toastBanner, "正在搜索局域网 ROS1 无人机...", d)
        }

        var autoConnectedThisRun = false

        lanScanner.startScan(
            baseConfig = rosManager.topics.copy(),
            onProgress = { _, hint ->
                runOnUiThread { setTextIfChanged(tvScanStatus, hint) }
            },
            onDroneFound = { drone ->
                runOnUiThread {
                    discoveredDrones.add(drone)
                    addDiscoveredDroneRow(drone)

                    if (hub.autoConnectDiscovered && !hub.rosConnected && !autoConnectedThisRun) {
                        autoConnectedThisRun = true
                        applyDiscoveredDroneAndConnect(drone, fromAuto = true)
                    }
                }
            },
            onComplete = { allFound, subnet ->
                runOnUiThread {
                    GcsIcons.bind(btnDrawerScan, Type.SEARCH, "重新扫描飞机", d, iconDp = 12f)
                    if (allFound.isEmpty()) {
                        setTextIfChanged(tvScanStatus, "在 $subnet 未发现开放 11311 或 9090 的设备")
                        if (!silentIfEmpty) {
                            AnimHelper.showDjiToast(toastBanner, "未搜到无人机，请确认同网段或手动输入 IP", d)
                            if (autoOpenConnTabOnEmpty) toggleOrJumpDrawerTab(TAB_CONN)
                        }
                    } else {
                        setTextIfChanged(tvScanStatus, "扫描完成 ($subnet)：发现 ${allFound.size} 台设备")
                        if (!autoConnectedThisRun && !silentIfEmpty) {
                            AnimHelper.showDjiToast(toastBanner, "发现 ${allFound.size} 台无人机，请点选连接", d)
                        }
                    }
                }
            }
        )
    }

    private fun addDiscoveredDroneRow(drone: DiscoveredDrone) {
        val pal = GcsTheme.of(hub.themeId)
        val row = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setPadding(dp(8.5f), dp(6.5f), dp(8.5f), dp(6.5f))
            background = GradientDrawable().apply {
                setColor(pal.wellFill)
                cornerRadius = dp(6f).toFloat()
                setStroke(1, pal.secondaryAccent)
            }
        }

        val infoCol = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        val protoTag = if (drone.protocol == "tcpros") "TCPROS:${drone.port}" else "WS:${drone.port}"
        val titleTv = TextView(this).apply {
            maxLines = 1
            setHorizontallyScrolling(false)
            setTextColor(pal.textPrimary)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.6f)
            typeface = Typeface.MONOSPACE
            GcsIcons.bind(this, Type.DRONE_QUAD, "${drone.ip} · ${drone.rttMs}ms", d, iconDp = 12f)
        }
        val subTv = TextView(this).apply {
            text = "$protoTag | ${drone.stackSummary}"
            maxLines = 1
            setHorizontallyScrolling(false)
            ellipsize = TextUtils.TruncateAt.END
            setTextColor(pal.secondaryAccent)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.2f)
        }
        infoCol.addView(titleTv)
        infoCol.addView(subTv)

        val btnBind = TextView(this).apply {
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            setTextColor(Color.WHITE)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.5f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(9.5f), dp(5f), dp(9.5f), dp(5f))
            background = roundedColor(pal.accent, dp(5f))
            GcsIcons.bind(this, Type.LINK, "连接", d, iconDp = 11f)
            setOnClickListener { applyDiscoveredDroneAndConnect(drone, fromAuto = false) }
        }
        AnimHelper.attachSpringTouch(btnBind)

        val btnJoinSwarm = TextView(this).apply {
            gravity = Gravity.CENTER
            maxLines = 1
            setHorizontallyScrolling(false)
            setTextColor(Color.parseColor("#00E676"))
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.8f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(7.5f), dp(5f), dp(7.5f), dp(5f))
            background = pillBg(Color.argb(38, 0, 230, 118), Color.parseColor("#00E676"))
            GcsIcons.bind(this, Type.SWARM_FLEET, "编入多机", d, iconDp = 10.5f, gapDp = 2.5f)
            setOnClickListener { enrollDiscoveredDroneIntoSwarm(drone) }
        }
        AnimHelper.attachSpringTouch(btnJoinSwarm)

        row.addView(infoCol, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        row.addView(btnJoinSwarm, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(5f) })
        row.addView(btnBind, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(5f) })
        discoveredListContainer.addView(row, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(5f) })
        AnimHelper.pulseViewSubtle(row)
    }

    private fun applyDiscoveredDroneAndConnect(drone: DiscoveredDrone, fromAuto: Boolean) {
        val sc = drone.suggestedConfig
        rosManager.topics.protocol = sc.protocol
        syncProtoInputsVisibility()

        inpMasterUri.setText(sc.masterUri)
        inpWsUrl.setText(sc.wsUrl)
        inpOdom.setText(sc.odomTopic)
        inpFastLioCloud.setText(sc.fastLioCloudTopic)
        inpFastLioPath.setText(sc.fastLioPathTopic)
        inpInflate.setText(sc.inflateTopic)
        inpOptimal.setText(sc.optimalTopic)
        inpGoal.setText(sc.goalTopic)
        inpTiltTelemetry.setText(sc.tiltTelemetryTopic)
        inpTiltCmd.setText(sc.tiltCmdTopic)

        saveCurrentTopics(quiet = true)
        connectWithConfig(rosManager.topics)
        if (!fromAuto && drawerOpen) {
            setDrawerOpenState(false)
        }

        val prefix = if (fromAuto) "已自动连接" else "已连接"
        AnimHelper.showDjiToast(toastBanner, "$prefix ${drone.ip} (${drone.rttMs}ms)", d)
    }

    private fun connectWithConfig(cfg: TopicConfig) {
        topicInspector.stopActiveSession()
        cachedTopicEntries = emptyList()
        simEngine.resetRosSessionState()
        simEngine.clearVoxelMap()
        rosManager.connect(cfg)
        syncConnModeSectionsUi(showRosEditor = true)
        syncModeSpecificControls(force = true)
        refreshCommandDockUi()
        refreshWaypointOrchestratorList()
    }

    private fun syncFlightRecorderUi() {
        if (!::btnToggleFlightRec.isInitialized) return
        val active = simEngine.telemetryRecorder.isRecording
        val pal = GcsTheme.of(hub.themeId)
        btnToggleFlightRec.setTextColor(if (active) Color.WHITE else pal.textPrimary)
        btnToggleFlightRec.background = if (active) {
            roundedColor(Color.parseColor("#FF3B30"), dp(5.5f))
        } else {
            pillBg(pal.glassFill, pal.cardStroke, radiusDp = 5.5f)
        }
        GcsIcons.bind(
            btnToggleFlightRec,
            if (active) Type.STOP_SQUARE else Type.PLAY,
            if (active) "停止录制" else "开始记录",
            d,
            iconDp = 11f
        )
    }

    private fun exportFlightTelemetryCsv() {
        fileIoExecutor.execute {
            try {
                val baseDir = getExternalFilesDir("telemetry_logs") ?: File(filesDir, "telemetry_logs")
                val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
                val outFile = File(baseDir, "ego_tilt_log_$stamp.csv")
                val rows = simEngine.telemetryRecorder.exportToCsv(outFile)
                runOnUiThread {
                    if (rows <= 0) {
                        AnimHelper.showDjiToast(toastBanner, "当前无缓存遥测帧，请先点击「开始记录」", d)
                    } else {
                        AnimHelper.showDjiToast(toastBanner, "已导出 $rows 行科研遥测至 telemetry_logs/${outFile.name}", d)
                    }
                }
            } catch (e: Exception) {
                runOnUiThread {
                    AnimHelper.showDjiToast(toastBanner, "CSV 导出失败: ${e.localizedMessage ?: "IO异常"}", d)
                }
            }
        }
    }

    private fun exportCurrentPointCloudPcd() {
        fileIoExecutor.execute {
            try {
                val baseDir = getExternalFilesDir("maps") ?: File(filesDir, "maps")
                val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
                val outFile = File(baseDir, "fastlio_map_$stamp.pcd")
                val exported = simEngine.voxelMap.exportBinaryPcd(outFile)
                runOnUiThread {
                    if (exported <= 0) {
                        AnimHelper.showDjiToast(toastBanner, "当前点云地图为空", d)
                    } else {
                        AnimHelper.showDjiToast(toastBanner, "已保存 $exported 点至 maps/${outFile.name}", d)
                    }
                }
            } catch (e: Exception) {
                runOnUiThread {
                    AnimHelper.showDjiToast(toastBanner, "导出失败: ${e.localizedMessage ?: "存储不可写"}", d)
                }
            }
        }
    }

    private fun setTextIfChanged(tv: TextView, newText: String) {
        if (!TextUtils.equals(tv.text, newText)) {
            tv.text = newText
        }
    }

    private fun updateHudReadouts(s: RenderSnapshot) {
        if (::tvUdpStatus.isInitialized && drawerOpen && activeTabIndex == TAB_CONN) {
            setTextIfChanged(tvUdpStatus, hub.udpTelemetryStatus)
        }
        hsiView.invalidateIfChanged()
        compassView.invalidateIfChanged()
        if (!drawerOpen && attitudeHudView.visibility == View.VISIBLE) {
            attitudeHudView.tickAndInvalidate()
        }
        syncModeSpecificControls()
        updateOcclusionHudVisibility(hub.droneCurrentlyOccluded && hub.xrayOcclusionEnabled && mapView.is3DMode)

        val curTheme = hub.themeId
        val pal = GcsTheme.of(curTheme)
        if (curTheme != lastHudThemeId) {
            lastHudThemeId = curTheme
            lastArmedBadgeStyled = -1
            lastClearanceWarnLevel = -1
            lastBatDangerLevel = -1
        }

        val spd = sqrt(s.vx * s.vx + s.vy * s.vy + s.vz * s.vz)
        val dgx = s.goalX - s.uavX
        val dgy = s.goalY - s.uavY
        val dgz = s.goalZ - s.uavZ
        val distGoal = sqrt(dgx * dgx + dgy * dgy + dgz * dgz)
        val yawDeg = s.yaw * 57.29578f

        if (s.px4Mode != lastPx4ModeStr || s.armed != lastArmedFlag) {
            lastPx4ModeStr = s.px4Mode
            lastArmedFlag = s.armed
            tvModeBadge.text = "${s.px4Mode} · ${if (s.armed) "ARMED" else "DISARM"}"
        }
        val armedStyleCode = if (s.armed) 1 else 0
        if (armedStyleCode != lastArmedBadgeStyled) {
            lastArmedBadgeStyled = armedStyleCode
            if (s.armed) {
                tvModeBadge.setTextColor(pal.textPrimary)
                tvModeBadge.background = GradientDrawable(
                    GradientDrawable.Orientation.TOP_BOTTOM,
                    intArrayOf(pal.glassFillTop, pal.glassFill)
                ).apply {
                    cornerRadius = 5.5f * d
                    setStroke(dp(1f).coerceAtLeast(1), pal.glassHighlightStroke)
                }
            } else {
                tvModeBadge.setTextColor(Color.rgb(255, 198, 72))
                tvModeBadge.background = GradientDrawable(
                    GradientDrawable.Orientation.TOP_BOTTOM,
                    intArrayOf(pal.glassFillTop, pal.glassFill)
                ).apply {
                    cornerRadius = 5.5f * d
                    setStroke(dp(1f).coerceAtLeast(1), Color.argb(150, 255, 178, 42))
                }
            }
        }
        val curConnLbl = hub.connectionLabel
        if (curConnLbl != lastConnLabelStr || s.fsmState != lastFsmStateStr) {
            lastConnLabelStr = curConnLbl
            lastFsmStateStr = s.fsmState
            tvConnStatus.text = "$curConnLbl | ${s.fsmState}"
        }

        val lv = hub.connectionLevel
        if (lv != lastAppliedConnLevel) {
            lastAppliedConnLevel = lv
            statusAccent.background = when (lv) {
                1 -> accentAmber
                2 -> accentRed
                else -> accentGreen
            }
        }

        if (!s.hasPreviewPin && (s.waypointQueueCount > 0 || commandDock.visibility == View.VISIBLE)) {
            refreshCommandDockUi()
        }

        val clrLevel = when {
            s.minClearance <= hub.warnClearanceMeters * 0.65f -> 2
            s.minClearance <= hub.warnClearanceMeters -> 1
            else -> 0
        }
        if (clrLevel != lastClearanceWarnLevel) {
            lastClearanceWarnLevel = clrLevel
            tvClearanceTop.setTextColor(
                when (clrLevel) {
                    2 -> Color.parseColor("#FF3B30")
                    1 -> Color.rgb(225, 132, 0)
                    else -> pal.secondaryAccent
                }
            )
        }
        FastHudFormatter.setFixed1(tvClearanceTop, "避障", s.minClearance, "m")
        if (tvHzTop.visibility == View.VISIBLE) {
            FastHudFormatter.setTopHzFps(tvHzTop, hub.isRosMode, s.odomHz, hub.renderFps)
        }

        val batLevel = when {
            s.batPct <= maxOf(10, hub.lowBatWarnPct - 8) -> 2
            s.batPct <= hub.lowBatWarnPct -> 1
            else -> 0
        }
        if (batLevel != lastBatDangerLevel) {
            lastBatDangerLevel = batLevel
            val c = when (batLevel) {
                2 -> Color.parseColor("#FF3B30")
                1 -> Color.rgb(225, 132, 0)
                else -> pal.secondaryAccent
            }
            tvBatTop.setTextColor(c)
            topBatFill.setBackgroundColor(c)
        }
        FastHudFormatter.setTopBattery(tvBatTop, s.batVolt, s.batCurrentAmps, s.txBatPct, compact = screenWidthDp < 650f)

        val batW = (resources.displayMetrics.widthPixels * (s.batPct.coerceIn(5, 100) / 100f)).roundToInt()
        if (batW != lastBatBarPx) {
            lastBatBarPx = batW
            AnimHelper.animateBarWidthSmooth(topBatFill, batW)
        }

        if (hdTelemetryBox.visibility == View.VISIBLE) {
            FastHudFormatter.setFixed1(tvOsdH, "", s.uavZ, " m")
            FastHudFormatter.setFixed1(tvOsdHs, "", spd, " m/s")
            FastHudFormatter.setFixed1(tvOsdD, "", distGoal, " m")
            FastHudFormatter.setCompactPoints(tvOsdMapPts, "", s.mapPointCount, "")
            FastHudFormatter.setSubXyzYaw(tvOsdSubXyz, s.uavX, s.uavY, yawDeg)
            tvOsdTilt.setTextColor(
                when {
                    !s.activeIsTiltRig -> pal.textMuted
                    s.tiltServoSaturated -> Color.parseColor("#FF9100")
                    else -> pal.secondaryAccent
                }
            )
            FastHudFormatter.setTiltHudLine(
                tvOsdTilt,
                s.activeIsTiltRig,
                hub.tiltCoordMode,
                s.tiltServo1Rad * 57.29578f,
                s.tiltServo2Rad * 57.29578f,
                hub.tiltPitchBiasDeg,
                s.smoothRoll * 57.29578f,
                s.smoothPitch * 57.29578f
            )
        }

        if (drawerOpen && activeTabIndex == TAB_STATUS) {
            if (::tvSwarmFleetConsole.isInitialized) {
                val fleetTxt = if (s.swarmCount <= 1) {
                    val rigDesc = if (s.activeIsTiltRig) "双舵倾转矢量 4M2S" else "常规固定 X4 四旋翼"
                    "当前模式 : 单机操控 · $rigDesc (顶栏可一键切换「单机/多机」与「矢量/普通」)"
                } else {
                    val fn = SwarmPalette.FORMATION_FULL_NAMES[hub.swarmFormationType]
                    val rigPolName = TiltConfigSpec.FLEET_RIG_SHORT_NAMES[hub.swarmRigPolicy.coerceIn(0, 2)]
                    val sb = StringBuilder()
                    sb.append(
                        String.format(
                            Locale.US,
                            "编队 : %d架·%s (%s %.1fm) | 机间最近 %.2fm\n",
                            s.swarmCount,
                            rigPolName,
                            fn,
                            hub.swarmSpacingMeters,
                            s.minInterDroneDist
                        )
                    )
                    for (i in 0 until s.swarmCount) {
                        val ag = s.swarmAgents[i]
                        val focusMark = if (i == s.activeDroneId) "*" else " "
                        val rigTag = if (ag.isTiltRig) "4M2S" else "X4  "
                        val vNorm = hypot(ag.vx, ag.vy)
                        sb.append(
                            String.format(
                                Locale.US,
                                "%s#%d[%s]: (%+5.1f,%+5.1f,%.1fm) %.1fm/s %d%% [%s]",
                                focusMark, i, rigTag, ag.uavX, ag.uavY, ag.uavZ, vNorm, ag.batPct, ag.fsmState
                            )
                        )
                        if (i < s.swarmCount - 1) sb.append('\n')
                    }
                    sb.toString()
                }
                setTextIfChanged(tvSwarmFleetConsole, fleetTxt)
            }
            if (::tvTiltIndiConsole.isInitialized) {
                val focusTag = if (s.swarmCount > 1) "UAV-#${s.activeDroneId} " else "单机 "
                val tiltTxt = if (!s.activeIsTiltRig) {
                    String.format(
                        Locale.US,
                        "焦点构型 : %s常规固定 X4 四旋翼 (倾转机构已解耦锁定 0.0°)\n" +
                            "姿态控制 : 欠驱动滚转/俯仰直接耦合前向加减速 (R=%+5.1f° P=%+5.1f°)\n" +
                            "四电机混控: T1=%.2f T2=%.2f T3=%.2f T4=%.2f | 点击顶栏「普通X4」可切矢量",
                        focusTag,
                        s.smoothRoll * 57.29578f,
                        s.smoothPitch * 57.29578f,
                        s.motorNorm4[0], s.motorNorm4[1], s.motorNorm4[2], s.motorNorm4[3]
                    )
                } else {
                    val s1Deg = s.tiltServo1Rad * 57.29578f
                    val s2Deg = s.tiltServo2Rad * 57.29578f
                    val c1Deg = s.tiltCmd1Rad * 57.29578f
                    val c2Deg = s.tiltCmd2Rad * 57.29578f
                    val srcStr = if (s.hasLiveTiltTelemetry) "ROS1实测" else "动力学估计"
                    val satStr = if (s.tiltServoSaturated) "限速重分配(300°/s)" else "线性满秩"
                    val mName = TiltConfigSpec.FULL_MODE_NAMES[hub.tiltCoordMode]
                    String.format(
                        Locale.US,
                        "焦点构型 : %s4M2S矢量 · %s (%s · %s)\n" +
                            "舵机1(M1/M3前): %+5.1f° (指令 %+5.1f° · %+5.0f°/s)\n" +
                            "舵机2(M2/M4后): %+5.1f° (指令 %+5.1f° · %+5.0f°/s)\n" +
                            "四电机推力分配: T1=%.2f T2=%.2f T3=%.2f T4=%.2f | w=%.2f",
                        focusTag, mName, srcStr, satStr,
                        s1Deg, c1Deg, s.tiltServo1RateDegS,
                        s2Deg, c2Deg, s.tiltServo2RateDegS,
                        s.motorNorm4[0], s.motorNorm4[1], s.motorNorm4[2], s.motorNorm4[3],
                        s.tiltManipulability
                    )
                }
                setTextIfChanged(tvTiltIndiConsole, tiltTxt)
            }
            tvUavPowerCard.setTextColor(pal.secondaryAccent)
            tvClearanceCard.setTextColor(pal.secondaryAccent)
            tvMapPtsCard.setTextColor(pal.secondaryAccent)
            FastHudFormatter.setPowerDetail(tvUavPowerCard, s.batVolt, s.batCurrentAmps, 1, s.batPct)
            FastHudFormatter.setPowerDetail(tvTxPowerCard, s.txBatVolt, s.txCurrentAmps, 2, s.txBatPct)
            FastHudFormatter.setCompactPoints(tvMapPtsCard, "", s.mapPointCount, " / 300k")
            FastHudFormatter.setIntMetric(tvCloudRateCard, "", s.cloudHz, " Hz · ", s.cloudScanPts, " pts")
            setTextIfChanged(tvMapBoundsCard, simEngine.voxelMap.formatBoundsSpanSummary())
            val rec = simEngine.telemetryRecorder
            val recTxt = if (rec.isRecording) {
                "录制中 (20Hz) · ${rec.sampleCount} 帧 (${rec.durationSec()}s)"
            } else {
                "待机 · 已缓存 ${rec.sampleCount} 帧"
            }
            setTextIfChanged(tvRecorderStatusCard, recTxt)
            FastHudFormatter.setFixed2(tvPosX, "", s.uavX, " m")
            FastHudFormatter.setFixed2(tvPosY, "", s.uavY, " m")
            FastHudFormatter.setFixed2(tvPosZ, "", s.uavZ, " m")
            FastHudFormatter.setFixed2(tvVelNorm, "", spd, " m/s")
            val rDeg = s.smoothRoll * 57.29578f
            val pDeg = s.smoothPitch * 57.29578f
            FastHudFormatter.setRollPitchPair(tvRollPitch, rDeg, pDeg)
            FastHudFormatter.setQuat4(tvQuatSummary, s.quatW, s.quatX, s.quatY, s.quatZ)
            FastHudFormatter.setFixed1(tvYaw, "", yawDeg, "°")
            FastHudFormatter.setFixed2(tvGoalDist, "", distGoal, " m")
            FastHudFormatter.setFixed2(tvClearanceCard, "", s.minClearance, " m")
            FastHudFormatter.setFixed2(tvSolveCard, "", s.solveMs, " ms")
            FastHudFormatter.setSingleInt(tvVoxelCount, "", s.voxelCount, " blks")
            FastHudFormatter.setFixed1(tvTrajLen, "", s.trajLength, " m")
        }
    }

    private fun loadSavedTopics() {
        val sp = getSharedPreferences(GcsSettingsModal.PREFS_NAME, Context.MODE_PRIVATE)
        val cfg = TopicConfig(
            protocol = sp.getString("protocol", "tcpros") ?: "tcpros",
            udpTelemetryEnabled = sp.getBoolean("udpTelemetryEnabled", false),
            udpPort = sp.getInt("udpPort", 14560).let { if (it in 1024..65535 && it != 14550) it else 14560 },
            masterUri = sp.getString("masterUri", "http://192.168.1.100:11311") ?: "http://192.168.1.100:11311",
            wsUrl = sp.getString("wsUrl", "ws://192.168.1.100:9090") ?: "ws://192.168.1.100:9090",
            odomTopic = sp.getString("odom", "/Odometry") ?: "/Odometry",
            fastLioCloudTopic = sp.getString("fastLioCloud", "/cloud_registered") ?: "/cloud_registered",
            fastLioPathTopic = sp.getString("fastLioPath", "/path") ?: "/path",
            inflateTopic = sp.getString("inflate", "/grid_map/occupancy_inflate") ?: "/grid_map/occupancy_inflate",
            optimalTopic = sp.getString("optimal", "/ego_planner_node/optimal_list") ?: "/ego_planner_node/optimal_list",
            goalTopic = sp.getString("goal", "/move_base_simple/goal") ?: "/move_base_simple/goal",
            swarmTopology = sp.getInt("swarmTopology", 0).coerceIn(0, 1),
            swarmPeerIpsCsv = sp.getString("swarmPeerIpsCsv", "192.168.1.100, 192.168.1.101, 192.168.1.102")
                ?: "192.168.1.100, 192.168.1.101, 192.168.1.102",
            swarmOdomPattern = sp.getString("swarmOdomPattern", "/drone_{id}_visual_slam/odom") ?: "/drone_{id}_visual_slam/odom",
            swarmOptimalPattern = sp.getString("swarmOptimalPattern", "/drone_{id}_ego_planner_node/optimal_list")
                ?: "/drone_{id}_ego_planner_node/optimal_list",
            swarmGoalPattern = sp.getString("swarmGoalPattern", "/drone_{id}_move_base_simple/goal")
                ?: "/drone_{id}_move_base_simple/goal",
            tiltTelemetryTopic = sp.getString("tiltTelemetryTopic", "/tilt_uav/servo_angles")
                ?: "/tilt_uav/servo_angles",
            tiltCmdTopic = sp.getString("tiltCmdTopic", "/active_pitch_planner/pitch_tilt_cmd")
                ?: "/active_pitch_planner/pitch_tilt_cmd",
            swarmTiltSubPattern = sp.getString("swarmTiltSubPattern", "/drone_{id}_tilt_uav/servo_angles")
                ?: "/drone_{id}_tilt_uav/servo_angles",
            swarmTiltCmdPattern = sp.getString("swarmTiltCmdPattern", "/drone_{id}_active_pitch_planner/pitch_tilt_cmd")
                ?: "/drone_{id}_active_pitch_planner/pitch_tilt_cmd"
        )
        rosManager.topics = cfg
        udpTelemetryEnabledUi = cfg.udpTelemetryEnabled
        syncUdpSwitchVisual(animate = false)
    }

    private fun saveCurrentTopics(quiet: Boolean, refreshLink: Boolean = true): Boolean {
        val cur = rosManager.topics
        val udpEnabled = if (::udpSwitchPill.isInitialized) udpTelemetryEnabledUi else cur.udpTelemetryEnabled
        val udpPort = if (::inpUdpPort.isInitialized) inpUdpPort.text.toString().toIntOrNull() else cur.udpPort
        if (udpPort == null || udpPort !in 1024..65535 || udpPort == 14550) {
            AnimHelper.showDjiToast(toastBanner, "UDP 端口需为1024–65535，且不能占用14550", d)
            return false
        }
        val peerCsv = if (::inpSwarmPeerIps.isInitialized) inpSwarmPeerIps.text.toString().trim() else cur.swarmPeerIpsCsv
        val odPat = if (::inpSwarmOdomPat.isInitialized) inpSwarmOdomPat.text.toString().trim() else cur.swarmOdomPattern
        val opPat = if (::inpSwarmOptPat.isInitialized) inpSwarmOptPat.text.toString().trim() else cur.swarmOptimalPattern
        val glPat = if (::inpSwarmGoalPat.isInitialized) inpSwarmGoalPat.text.toString().trim() else cur.swarmGoalPattern
        val tiltSub = if (::inpTiltTelemetry.isInitialized) inpTiltTelemetry.text.toString().trim() else cur.tiltTelemetryTopic
        val tiltPub = if (::inpTiltCmd.isInitialized) inpTiltCmd.text.toString().trim() else cur.tiltCmdTopic
        val swTiltSub = if (::inpSwarmTiltSubPat.isInitialized) inpSwarmTiltSubPat.text.toString().trim() else cur.swarmTiltSubPattern
        val swTiltPub = if (::inpSwarmTiltCmdPat.isInitialized) inpSwarmTiltCmdPat.text.toString().trim() else cur.swarmTiltCmdPattern

        val cfg = TopicConfig(
            protocol = cur.protocol,
            udpTelemetryEnabled = udpEnabled,
            udpPort = udpPort,
            masterUri = inpMasterUri.text.toString().trim(),
            wsUrl = inpWsUrl.text.toString().trim(),
            odomTopic = inpOdom.text.toString().trim(),
            fastLioCloudTopic = inpFastLioCloud.text.toString().trim(),
            fastLioPathTopic = inpFastLioPath.text.toString().trim(),
            inflateTopic = inpInflate.text.toString().trim(),
            optimalTopic = inpOptimal.text.toString().trim(),
            goalTopic = inpGoal.text.toString().trim(),
            swarmTopology = cur.swarmTopology,
            swarmPeerIpsCsv = peerCsv.ifBlank { "192.168.1.100, 192.168.1.101, 192.168.1.102" },
            swarmOdomPattern = odPat.ifBlank { "/drone_{id}_visual_slam/odom" },
            swarmOptimalPattern = opPat.ifBlank { "/drone_{id}_ego_planner_node/optimal_list" },
            swarmGoalPattern = glPat.ifBlank { "/drone_{id}_move_base_simple/goal" },
            tiltTelemetryTopic = tiltSub.ifBlank { "/tilt_uav/servo_angles" },
            tiltCmdTopic = tiltPub.ifBlank { "/active_pitch_planner/pitch_tilt_cmd" },
            swarmTiltSubPattern = swTiltSub.ifBlank { "/drone_{id}_tilt_uav/servo_angles" },
            swarmTiltCmdPattern = swTiltPub.ifBlank { "/drone_{id}_active_pitch_planner/pitch_tilt_cmd" }
        )
        getSharedPreferences(GcsSettingsModal.PREFS_NAME, Context.MODE_PRIVATE).edit()
            .putBoolean("udpTelemetryEnabled", cfg.udpTelemetryEnabled)
            .putInt("udpPort", cfg.udpPort)
            .putString("protocol", cfg.protocol)
            .putString("masterUri", cfg.masterUri)
            .putString("wsUrl", cfg.wsUrl)
            .putString("odom", cfg.odomTopic)
            .putString("fastLioCloud", cfg.fastLioCloudTopic)
            .putString("fastLioPath", cfg.fastLioPathTopic)
            .putString("inflate", cfg.inflateTopic)
            .putString("optimal", cfg.optimalTopic)
            .putString("goal", cfg.goalTopic)
            .putInt("swarmTopology", cfg.swarmTopology)
            .putString("swarmPeerIpsCsv", cfg.swarmPeerIpsCsv)
            .putString("swarmOdomPattern", cfg.swarmOdomPattern)
            .putString("swarmOptimalPattern", cfg.swarmOptimalPattern)
            .putString("swarmGoalPattern", cfg.swarmGoalPattern)
            .putString("tiltTelemetryTopic", cfg.tiltTelemetryTopic)
            .putString("tiltCmdTopic", cfg.tiltCmdTopic)
            .putString("swarmTiltSubPattern", cfg.swarmTiltSubPattern)
            .putString("swarmTiltCmdPattern", cfg.swarmTiltCmdPattern)
            .apply()
        if (refreshLink) rosManager.updateTopicsAndResubscribe(cfg) else rosManager.topics = cfg
        if (!quiet) AnimHelper.showDjiToast(toastBanner, "话题与多机拓扑配置已保存生效", d)
        return true
    }

    // ==================== 多机集群协同编队座舱栏与抽屉控制组件 ====================

    private fun buildSwarmTacticalHudBar(): LinearLayout {
        val bar = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setPadding(dp(4.5f), dp(3f), dp(4.5f), dp(3f))
            GcsTheme.markRole(this, Role.GLASS_PANEL, cornerDp = 14f)
            visibility = if (hub.swarmModeEnabled) View.VISIBLE else View.GONE
        }

        btnSwarmScopeHud = TextView(this).apply {
            maxLines = 1
            setHorizontallyScrolling(false)
            gravity = Gravity.CENTER
            minimumHeight = dp(22f)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(6.5f), dp(2.5f), dp(6.5f), dp(2.5f))
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                hub.swarmCommandScope = if (hub.swarmCommandScope == 1) 0 else 1
                GcsSettingsModal.persistAllSettings(this@MainActivity, hub)
                syncSwarmAllControlsUi()
                refreshCommandDockUi(pulseDock = true)
                val tip = if (hub.swarmCommandScope == 1) {
                    "已切至「全队编队指点」：选点将按队形同步调度全部飞机"
                } else {
                    "已切至「单机独立点控」：选点仅调度当前焦点 UAV-#${hub.selectedDroneId}"
                }
                AnimHelper.showDjiToast(toastBanner, tip, d)
            }
        }
        bar.addView(btnSwarmScopeHud)

        swarmDroneHudChips.clear()
        for (id in 0 until SwarmPalette.MAX_SWARM_DRONES) {
            val chip = TextView(this).apply {
                text = "#$id"
                maxLines = 1
                setHorizontallyScrolling(false)
                gravity = Gravity.CENTER
                minimumHeight = dp(22f)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
                typeface = Typeface.MONOSPACE
                setPadding(dp(6f), dp(2.5f), dp(6f), dp(2.5f))
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    simEngine.selectSwarmDrone(id)
                    DroneVectorModel.activeRig = DroneVectorModel.rigFor(hub.activeFocusIsTiltRig())
                    GcsSettingsModal.persistAllSettings(this@MainActivity, hub)
                    mapView.recenterLockDrone()
                    syncCameraControlLabels()
                    syncSwarmAllControlsUi()
                    syncTiltAllControlsUi()
                    attitudeHudView.invalidate()
                    refreshCommandDockUi()
                    val rTag = if (hub.isDroneTiltRig(id)) "矢量 4M2S" else "普通 X4"
                    AnimHelper.showDjiToast(toastBanner, "主视角与仪表已锁定 -> UAV-#$id ($rTag · 长按编号可切换该机构型)", d)
                }
                setOnLongClickListener {
                    AnimHelper.triggerHaptic(it, HapticFeedbackConstants.LONG_PRESS)
                    val nowVec = hub.toggleSingleDroneRigInSwarm(id)
                    simEngine.selectSwarmDrone(id)
                    applyOperatingModeChangeLive(
                        "异构混编：UAV-#$id 已设为「${if (nowVec) "倾转矢量 4M2S" else "常规普通 X4"}」"
                    )
                    true
                }
            }
            swarmDroneHudChips.add(chip)
            bar.addView(chip, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(3.5f) })
        }

        btnSwarmFormHud = TextView(this).apply {
            maxLines = 1
            setHorizontallyScrolling(false)
            gravity = Gravity.CENTER
            minimumHeight = dp(22f)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
            typeface = Typeface.DEFAULT_BOLD
            setPadding(dp(6.5f), dp(2.5f), dp(6.5f), dp(2.5f))
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                hub.swarmFormationType = (hub.swarmFormationType + 1) % 4
                GcsSettingsModal.persistAllSettings(this@MainActivity, hub)
                syncSwarmAllControlsUi()
                refreshCommandDockUi(pulseDock = true)
                val cmds = simEngine.rearrangeSwarmFormationNow()
                rosManager.publishSwarmGoals(cmds)
                AnimHelper.showDjiToast(toastBanner, "编队切换：${SwarmPalette.FORMATION_FULL_NAMES[hub.swarmFormationType]}", d)
            }
        }
        bar.addView(btnSwarmFormHud, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(4f) })
        return bar
    }

    private fun buildSwarmControlCardInDrawer(): View {
        val card = sectionCard("单/多机 × 矢量/普通 模式与编队协同")

        card.addView(
            TextView(this).apply {
                text = "一键全兼容工作模式切换 (单机普通 / 单机矢量 / 多机普通 / 多机矢量)"
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.3f)
                setPadding(0, 0, 0, dp(3f))
                GcsTheme.markRole(this, Role.TEXT_MUTED)
            }
        )
        val matrixTrack = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dp(2f), dp(2f), dp(2f), dp(2f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        val matrixLabels = arrayOf("单机·普通", "单机·矢量", "多机·普通", "多机·矢量")
        modeMatrixQuickBtns.clear()
        for (mIdx in matrixLabels.indices) {
            val mb = TextView(this).apply {
                text = matrixLabels[mIdx]
                maxLines = 1
                setHorizontallyScrolling(false)
                gravity = Gravity.CENTER
                minimumHeight = dp(27f)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
                typeface = Typeface.DEFAULT_BOLD
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    applyMatrixModeCombo(mIdx)
                }
            }
            modeMatrixQuickBtns.add(mb)
            matrixTrack.addView(mb, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        card.addView(matrixTrack)

        card.addView(
            TextView(this).apply {
                text = "集群规模与单/多机模式"
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
                setPadding(0, dp(6f), 0, dp(3f))
                GcsTheme.markRole(this, Role.TEXT_MUTED)
            }
        )
        val scaleTrack = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dp(2f), dp(2f), dp(2f), dp(2f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        val scaleDefs = listOf(1 to "单机", 2 to "2架双机", 3 to "3架编队", 4 to "4架集群")
        swarmScaleSegBtns.clear()
        for ((cnt, lbl) in scaleDefs) {
            val b = TextView(this).apply {
                text = lbl
                maxLines = 1
                setHorizontallyScrolling(false)
                gravity = Gravity.CENTER
                minimumHeight = dp(26f)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
                typeface = Typeface.DEFAULT_BOLD
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    if (cnt == 1) {
                        setSwarmModeEnabledLive(false)
                    } else {
                        hub.swarmDroneCount = cnt
                        if (hub.selectedDroneId >= cnt) hub.selectedDroneId = 0
                        setSwarmModeEnabledLive(true, customCountToast = true)
                    }
                }
            }
            swarmScaleSegBtns.add(b)
            scaleTrack.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        card.addView(scaleTrack)

        val fleetRigSec = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            visibility = if (hub.swarmModeEnabled) View.VISIBLE else View.GONE
        }
        fleetRigSec.addView(
            TextView(this).apply {
                text = "多机集群构型分配 (支持同构全普通 / 全矢量 / 异构混编)"
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.3f)
                setPadding(0, dp(6f), 0, dp(3f))
                GcsTheme.markRole(this, Role.TEXT_MUTED)
            }
        )
        val policyTrack = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dp(2f), dp(2f), dp(2f), dp(2f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        val polLabels = arrayOf("全普通 X4", "全矢量 4M2S", "异构混编")
        swarmRigPolicyBtns.clear()
        for (pIdx in polLabels.indices) {
            val pb = TextView(this).apply {
                text = polLabels[pIdx]
                maxLines = 1
                setHorizontallyScrolling(false)
                gravity = Gravity.CENTER
                minimumHeight = dp(26f)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
                typeface = Typeface.DEFAULT_BOLD
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    hub.swarmRigPolicy = pIdx
                    if (pIdx != TiltConfigSpec.FLEET_RIG_MIXED) {
                        hub.tiltAirframeEnabled = (pIdx == TiltConfigSpec.FLEET_RIG_ALL_VECTORED)
                    }
                    applyOperatingModeChangeLive("多机构型策略：${TiltConfigSpec.FLEET_RIG_FULL_NAMES[pIdx]}")
                }
            }
            swarmRigPolicyBtns.add(pb)
            policyTrack.addView(pb, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        fleetRigSec.addView(policyTrack)

        val mixedRow = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(0, dp(5f), 0, 0)
            visibility = if (hub.swarmRigPolicy == TiltConfigSpec.FLEET_RIG_MIXED) View.VISIBLE else View.GONE
        }
        swarmPerDroneRigBtns.clear()
        for (dId in 0 until SwarmPalette.MAX_SWARM_DRONES) {
            val db = TextView(this).apply {
                text = "#$dId 矢量"
                maxLines = 1
                setHorizontallyScrolling(false)
                gravity = Gravity.CENTER
                minimumHeight = dp(26f)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.2f)
                typeface = Typeface.DEFAULT_BOLD
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    val nowVec = hub.toggleSingleDroneRigInSwarm(dId)
                    simEngine.selectSwarmDrone(dId)
                    applyOperatingModeChangeLive(
                        "UAV-#$dId 构型已切为「${if (nowVec) "矢量 4M2S" else "普通 X4"}」"
                    )
                }
            }
            swarmPerDroneRigBtns.add(db)
            mixedRow.addView(
                db,
                LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply {
                    if (dId > 0) leftMargin = dp(4f)
                }
            )
        }
        wrapSwarmMixedMaskRow = mixedRow
        fleetRigSec.addView(mixedRow)
        wrapSwarmFleetRigSection = fleetRigSec
        card.addView(fleetRigSec)

        card.addView(
            TextView(this).apply {
                text = "协同编队队形 (果园/林下窄行推荐「纵队」)"
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
                setPadding(0, dp(6f), 0, dp(3f))
                GcsTheme.markRole(this, Role.TEXT_MUTED)
            }
        )
        val formTrack = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dp(2f), dp(2f), dp(2f), dp(2f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        swarmFormSegBtns.clear()
        for (fIdx in SwarmPalette.FORMATION_SHORT_NAMES.indices) {
            val b = TextView(this).apply {
                text = SwarmPalette.FORMATION_SHORT_NAMES[fIdx]
                maxLines = 1
                setHorizontallyScrolling(false)
                gravity = Gravity.CENTER
                minimumHeight = dp(26f)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
                typeface = Typeface.DEFAULT_BOLD
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    hub.swarmFormationType = fIdx
                    if (!hub.swarmModeEnabled) hub.swarmModeEnabled = true
                    GcsSettingsModal.persistAllSettings(this@MainActivity, hub)
                    syncSwarmAllControlsUi()
                    val cmds = simEngine.rearrangeSwarmFormationNow()
                    rosManager.publishSwarmGoals(cmds)
                    refreshCommandDockUi()
                    AnimHelper.showDjiToast(toastBanner, "已按「${SwarmPalette.FORMATION_FULL_NAMES[fIdx]}」整队集结", d)
                }
            }
            swarmFormSegBtns.add(b)
            formTrack.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        card.addView(formTrack)

        card.addView(
            TextView(this).apply {
                text = "编队机间基准间距"
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
                setPadding(0, dp(6f), 0, dp(3f))
                GcsTheme.markRole(this, Role.TEXT_MUTED)
            }
        )
        val spaceTrack = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dp(2f), dp(2f), dp(2f), dp(2f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        val spaceChoices = listOf(1.2f to "1.2m紧凑", 1.8f to "1.8m标准", 2.5f to "2.5m宽距", 3.2f to "3.2m疏散")
        swarmSpaceSegBtns.clear()
        for ((spVal, lbl) in spaceChoices) {
            val b = TextView(this).apply {
                text = lbl
                maxLines = 1
                setHorizontallyScrolling(false)
                gravity = Gravity.CENTER
                minimumHeight = dp(26f)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
                typeface = Typeface.DEFAULT_BOLD
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    hub.swarmSpacingMeters = spVal
                    GcsSettingsModal.persistAllSettings(this@MainActivity, hub)
                    syncSwarmAllControlsUi()
                    if (hub.swarmModeEnabled) {
                        val cmds = simEngine.rearrangeSwarmFormationNow()
                        rosManager.publishSwarmGoals(cmds)
                    }
                    refreshCommandDockUi()
                }
            }
            swarmSpaceSegBtns.add(b)
            spaceTrack.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        card.addView(spaceTrack)

        val actRow = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        btnToggleSwarmMesh = createSmallIconBtn(Type.SWARM_FLEET, "拓扑连线: 开") {
            hub.showSwarmLinks = !hub.showSwarmLinks
            GcsSettingsModal.persistAllSettings(this, hub)
            syncSwarmAllControlsUi()
        }
        val btnAssembleNow = createSmallIconBtn(Type.SEND_FLY, "立即按队形集结") {
            if (!hub.swarmModeEnabled) setSwarmModeEnabledLive(true)
            val cmds = simEngine.rearrangeSwarmFormationNow()
            rosManager.publishSwarmGoals(cmds)
            AnimHelper.showDjiToast(toastBanner, "已下发全队 ${SwarmPalette.FORMATION_FULL_NAMES[hub.swarmFormationType]} 集结指令", d)
        }
        actRow.addView(btnToggleSwarmMesh, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        actRow.addView(btnAssembleNow, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1.15f).apply { leftMargin = dp(5f) })
        card.addView(actRow, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(6.5f) })

        return card
    }

    private fun buildRosSwarmTopologyCard(cfg: TopicConfig): View {
        val card = sectionCard("3. ROS1 多机集群拓扑配置 (兼容 EGO-Swarm)")
        val topoTrack = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dp(2f), dp(2f), dp(2f), dp(2f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        val btnNs = TextView(this).apply {
            text = "同Master命名空间"
            maxLines = 1
            setHorizontallyScrolling(false)
            gravity = Gravity.CENTER
            minimumHeight = dp(26f)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
            typeface = Typeface.DEFAULT_BOLD
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                rosManager.topics.swarmTopology = 0
                syncSwarmAllControlsUi()
                saveCurrentTopics(quiet = true)
            }
        }
        val btnIp = TextView(this).apply {
            text = "多机独立IP直连"
            maxLines = 1
            setHorizontallyScrolling(false)
            gravity = Gravity.CENTER
            minimumHeight = dp(26f)
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
            typeface = Typeface.DEFAULT_BOLD
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                rosManager.topics.swarmTopology = 1
                syncSwarmAllControlsUi()
                saveCurrentTopics(quiet = true)
            }
        }
        segSwarmTopoNsBtn = btnNs
        segSwarmTopoIpBtn = btnIp
        topoTrack.addView(btnNs, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        topoTrack.addView(btnIp, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        card.addView(topoTrack)

        val nsWrap = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        inpSwarmOdomPat = labeledInput(nsWrap, "多机里程计模板 ({id} 自动替换 0..3)", cfg.swarmOdomPattern)
        inpSwarmOptPat = labeledInput(nsWrap, "多机规划轨迹模板 ({id} 自动替换 0..3)", cfg.swarmOptimalPattern)
        inpSwarmGoalPat = labeledInput(nsWrap, "多机目标发布模板 ({id} 自动替换 0..3)", cfg.swarmGoalPattern)
        inpSwarmTiltSubPat = labeledInput(nsWrap, "多机倾转遥测模板 (矢量/混编自动订阅)", cfg.swarmTiltSubPattern)
        inpSwarmTiltCmdPat = labeledInput(nsWrap, "多机倾转指令模板 (矢量/混编自动发布)", cfg.swarmTiltCmdPattern)
        wrapSwarmNsFields = nsWrap
        card.addView(nsWrap)

        val ipWrap = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        inpSwarmPeerIps = labeledInput(ipWrap, "各机载电脑独立 IP 列表 (逗号分隔，按 UAV-0..3 顺序)", cfg.swarmPeerIpsCsv)
        wrapSwarmIpFields = ipWrap
        card.addView(ipWrap)

        val btnSaveSwarmTopo = createSmallIconBtn(Type.CHECK, "应用多机拓扑配置") {
            hideSoftKeyboard()
            saveCurrentTopics(quiet = false)
        }
        card.addView(btnSaveSwarmTopo, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(6.5f) })
        return card
    }

    private fun enrollDiscoveredDroneIntoSwarm(drone: DiscoveredDrone) {
        val curCsv = if (::inpSwarmPeerIps.isInitialized) inpSwarmPeerIps.text.toString() else rosManager.topics.swarmPeerIpsCsv
        val list = curCsv.split(',', ';', ' ')
            .map { it.trim() }
            .filter { it.isNotEmpty() }
            .toMutableList()
        if (!list.contains(drone.ip)) {
            if (list.size >= SwarmPalette.MAX_SWARM_DRONES) list.removeAt(list.lastIndex)
            list.add(drone.ip)
        }
        val joined = list.joinToString(", ")
        rosManager.topics.swarmTopology = 1
        rosManager.topics.swarmPeerIpsCsv = joined
        if (::inpSwarmPeerIps.isInitialized) inpSwarmPeerIps.setText(joined)
        hub.swarmModeEnabled = true
        hub.swarmDroneCount = list.size.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
        GcsSettingsModal.persistAllSettings(this, hub)
        saveCurrentTopics(quiet = true)
        syncSwarmAllControlsUi()
        AnimHelper.showDjiToast(toastBanner, "已将 ${drone.ip} 编入多机集群 (当前共 ${list.size} 台 IP)", d)
    }

    private fun cycleTopRigModeQuick() {
        if (!hub.swarmModeEnabled) {
            hub.tiltAirframeEnabled = !hub.tiltAirframeEnabled
            val desc = if (hub.tiltAirframeEnabled) {
                "单机 · 双舵倾转矢量 4M2S (S1->M1/M3, S2->M2/M4)"
            } else {
                "单机 · 常规固定 X4 四旋翼"
            }
            applyOperatingModeChangeLive("已切换构型：$desc")
        } else {
            hub.swarmRigPolicy = (hub.swarmRigPolicy + 1) % 3
            if (hub.swarmRigPolicy != TiltConfigSpec.FLEET_RIG_MIXED) {
                hub.tiltAirframeEnabled = (hub.swarmRigPolicy == TiltConfigSpec.FLEET_RIG_ALL_VECTORED)
            }
            val polName = TiltConfigSpec.FLEET_RIG_FULL_NAMES[hub.swarmRigPolicy]
            applyOperatingModeChangeLive("多机集群构型：$polName")
        }
    }

    private fun applyMatrixModeCombo(comboIdx: Int) {
        when (comboIdx) {
            0 -> { // 单机·普通
                hub.swarmModeEnabled = false
                hub.selectedDroneId = 0
                hub.tiltAirframeEnabled = false
                mapView.recenterLockDrone()
                syncCameraControlLabels()
                applyOperatingModeChangeLive("已切换至「单机 · 普通 X4 四旋翼」模式")
            }
            1 -> { // 单机·矢量
                hub.swarmModeEnabled = false
                hub.selectedDroneId = 0
                hub.tiltAirframeEnabled = true
                mapView.recenterLockDrone()
                syncCameraControlLabels()
                applyOperatingModeChangeLive("已切换至「单机 · 双舵倾转矢量 4M2S」模式")
            }
            2 -> { // 多机·普通
                hub.swarmModeEnabled = true
                hub.swarmDroneCount = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
                hub.selectedDroneId = hub.selectedDroneId.coerceIn(0, hub.swarmDroneCount - 1)
                hub.swarmRigPolicy = TiltConfigSpec.FLEET_RIG_ALL_STANDARD
                hub.tiltAirframeEnabled = false
                if (!hub.isRosMode) simEngine.rearrangeSwarmFormationNow()
                applyOperatingModeChangeLive("已切换至「多机 (${hub.swarmDroneCount}架) · 全队普通 X4」模式")
            }
            else -> { // 多机·矢量
                hub.swarmModeEnabled = true
                hub.swarmDroneCount = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
                hub.selectedDroneId = hub.selectedDroneId.coerceIn(0, hub.swarmDroneCount - 1)
                hub.swarmRigPolicy = TiltConfigSpec.FLEET_RIG_ALL_VECTORED
                hub.tiltAirframeEnabled = true
                if (!hub.isRosMode) simEngine.rearrangeSwarmFormationNow()
                applyOperatingModeChangeLive("已切换至「多机 (${hub.swarmDroneCount}架) · 全队倾转矢量 4M2S」模式")
            }
        }
    }

    private fun applyOperatingModeChangeLive(toastMsg: String? = null) {
        simEngine.notifyOperatingModeChanged()
        DroneVectorModel.activeRig = DroneVectorModel.rigFor(hub.activeFocusIsTiltRig())
        GcsSettingsModal.persistAllSettings(this, hub)
        rosManager.updateTopicsAndResubscribe(rosManager.topics)
        syncSwarmAllControlsUi()
        syncTiltAllControlsUi()
        publishTiltSettingsToRos()
        attitudeHudView.invalidate()
        refreshCommandDockUi()
        if (!toastMsg.isNullOrBlank()) {
            AnimHelper.showDjiToast(toastBanner, toastMsg, d)
        }
    }

    private fun setSwarmModeEnabledLive(enable: Boolean, customCountToast: Boolean = false) {
        val wasEnabled = hub.swarmModeEnabled
        hub.swarmModeEnabled = enable
        if (!enable) {
            // 从多机同构策略切回单机时，同步保持刚才选用的矢量/普通构型体验一致
            if (wasEnabled && hub.swarmRigPolicy != TiltConfigSpec.FLEET_RIG_MIXED) {
                hub.tiltAirframeEnabled = (hub.swarmRigPolicy == TiltConfigSpec.FLEET_RIG_ALL_VECTORED)
            }
            hub.selectedDroneId = 0
        } else {
            // 从单机切入多机时，若此前非混编，则自动继承当前单机的矢量/普通构型避免突变
            if (!wasEnabled && hub.swarmRigPolicy != TiltConfigSpec.FLEET_RIG_MIXED) {
                hub.swarmRigPolicy = if (hub.tiltAirframeEnabled) {
                    TiltConfigSpec.FLEET_RIG_ALL_VECTORED
                } else {
                    TiltConfigSpec.FLEET_RIG_ALL_STANDARD
                }
            }
            hub.swarmDroneCount = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
            hub.selectedDroneId = hub.selectedDroneId.coerceIn(0, hub.swarmDroneCount - 1)
        }
        simEngine.notifyOperatingModeChanged()
        DroneVectorModel.activeRig = DroneVectorModel.rigFor(hub.activeFocusIsTiltRig())
        GcsSettingsModal.persistAllSettings(this, hub)
        rosManager.updateTopicsAndResubscribe(rosManager.topics)
        syncSwarmAllControlsUi()
        syncTiltAllControlsUi()
        attitudeHudView.invalidate()
        refreshCommandDockUi()
        if (enable) {
            if (!hub.isRosMode) {
                simEngine.rearrangeSwarmFormationNow()
            }
            val fn = SwarmPalette.FORMATION_FULL_NAMES[hub.swarmFormationType]
            val rigStr = TiltConfigSpec.FLEET_RIG_SHORT_NAMES[hub.swarmRigPolicy.coerceIn(0, 2)]
            val msg = if (customCountToast) {
                "多机集群已调整为 ${hub.swarmDroneCount} 架 ($rigStr · $fn)"
            } else {
                "已开启多机模式 (${hub.swarmDroneCount} 架 · $rigStr · $fn)"
            }
            AnimHelper.showDjiToast(toastBanner, msg, d)
        } else {
            mapView.recenterLockDrone()
            syncCameraControlLabels()
            val singleRig = if (hub.tiltAirframeEnabled) "矢量 4M2S" else "普通 X4"
            AnimHelper.showDjiToast(toastBanner, "已切回单机模式 ($singleRig)", d)
        }
    }

    private fun syncSwarmAllControlsUi() {
        val pal = GcsTheme.of(hub.themeId)
        syncTopQuickSlotsVisual()

        if (::swarmHudBar.isInitialized) {
            swarmHudBar.visibility = if (hub.swarmModeEnabled) View.VISIBLE else View.GONE
            val allScope = (hub.swarmCommandScope == 1)
            btnSwarmScopeHud.setTextColor(if (allScope) Color.WHITE else pal.textPrimary)
            btnSwarmScopeHud.background = roundedColor(
                if (allScope) pal.accent else pal.wellFill,
                dp(10f)
            )
            GcsIcons.bind(
                btnSwarmScopeHud,
                if (allScope) Type.SWARM_FLEET else Type.TARGET_PIN,
                if (allScope) "全队" else "单控",
                d,
                iconDp = 10.5f,
                gapDp = 2.5f
            )

            val cnt = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
            val showRigLetter = (hub.swarmRigPolicy == TiltConfigSpec.FLEET_RIG_MIXED)
            for (id in swarmDroneHudChips.indices) {
                val chip = swarmDroneHudChips[id]
                chip.visibility = if (id < cnt) View.VISIBLE else View.GONE
                val dVec = hub.isDroneTiltRig(id)
                chip.text = if (showRigLetter) "#$id${if (dVec) "V" else "X"}" else "#$id"
                val sel = (id == hub.selectedDroneId)
                val cInt = SwarmPalette.COLOR_INT[id]
                chip.setTextColor(if (sel) Color.rgb(8, 16, 20) else cInt)
                chip.background = GradientDrawable().apply {
                    cornerRadius = dp(9f).toFloat()
                    setColor(if (sel) cInt else Color.argb(34, Color.red(cInt), Color.green(cInt), Color.blue(cInt)))
                    setStroke(maxOf(1, dp(0.9f)), cInt)
                }
            }

            btnSwarmFormHud.setTextColor(pal.accentBright)
            btnSwarmFormHud.background = roundedColor(pal.accentSoftFill, dp(10f))
            btnSwarmFormHud.text = SwarmPalette.FORMATION_SHORT_NAMES[hub.swarmFormationType.coerceIn(0, 3)]
        }

        val activeMatrixIdx = when {
            !hub.swarmModeEnabled && !hub.tiltAirframeEnabled -> 0
            !hub.swarmModeEnabled && hub.tiltAirframeEnabled -> 1
            hub.swarmModeEnabled && hub.swarmRigPolicy == TiltConfigSpec.FLEET_RIG_ALL_STANDARD -> 2
            hub.swarmModeEnabled && hub.swarmRigPolicy == TiltConfigSpec.FLEET_RIG_ALL_VECTORED -> 3
            else -> -1
        }
        for (i in modeMatrixQuickBtns.indices) {
            val sel = (i == activeMatrixIdx)
            val hiCol = if (i % 2 == 1) Color.parseColor("#FF8F00") else pal.accent
            modeMatrixQuickBtns[i].setTextColor(if (sel) Color.WHITE else pal.textMuted)
            modeMatrixQuickBtns[i].background = roundedColor(if (sel) hiCol else Color.TRANSPARENT, dp(5f))
        }

        wrapSwarmFleetRigSection?.visibility = if (hub.swarmModeEnabled) View.VISIBLE else View.GONE
        wrapSwarmMixedMaskRow?.visibility =
            if (hub.swarmModeEnabled && hub.swarmRigPolicy == TiltConfigSpec.FLEET_RIG_MIXED) View.VISIBLE else View.GONE
        for (i in swarmRigPolicyBtns.indices) {
            val sel = (hub.swarmRigPolicy == i)
            val hiCol = when (i) {
                TiltConfigSpec.FLEET_RIG_ALL_VECTORED -> Color.parseColor("#FF8F00")
                TiltConfigSpec.FLEET_RIG_MIXED -> Color.parseColor("#00ACC1")
                else -> pal.accent
            }
            swarmRigPolicyBtns[i].setTextColor(if (sel) Color.WHITE else pal.textMuted)
            swarmRigPolicyBtns[i].background = roundedColor(if (sel) hiCol else Color.TRANSPARENT, dp(5f))
        }
        val activeCnt = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
        for (dId in swarmPerDroneRigBtns.indices) {
            val b = swarmPerDroneRigBtns[dId]
            b.visibility = if (dId < activeCnt) View.VISIBLE else View.GONE
            val isVec = hub.isDroneTiltRig(dId)
            b.text = "#$dId ${if (isVec) "矢量" else "普通"}"
            b.setTextColor(if (isVec) Color.WHITE else pal.textPrimary)
            b.background = pillBg(
                if (isVec) Color.argb(190, 255, 143, 0) else pal.wellFill,
                if (isVec) Color.parseColor("#FFB300") else pal.cardStroke,
                radiusDp = 5.5f
            )
        }

        val activeScale = if (hub.swarmModeEnabled) hub.swarmDroneCount.coerceIn(2, 4) else 1
        for (i in swarmScaleSegBtns.indices) {
            val sel = ((i + 1) == activeScale)
            swarmScaleSegBtns[i].setTextColor(if (sel) Color.WHITE else pal.textMuted)
            swarmScaleSegBtns[i].background = roundedColor(if (sel) Color.parseColor("#00A86B") else Color.TRANSPARENT, dp(5f))
        }
        for (i in swarmFormSegBtns.indices) {
            val sel = (hub.swarmFormationType == i)
            swarmFormSegBtns[i].setTextColor(if (sel) Color.WHITE else pal.textMuted)
            swarmFormSegBtns[i].background = roundedColor(if (sel) pal.accent else Color.TRANSPARENT, dp(5f))
        }
        val spaceVals = floatArrayOf(1.2f, 1.8f, 2.5f, 3.2f)
        for (i in swarmSpaceSegBtns.indices) {
            val sel = kotlin.math.abs(hub.swarmSpacingMeters - spaceVals[i]) < 0.15f
            swarmSpaceSegBtns[i].setTextColor(if (sel) Color.WHITE else pal.textMuted)
            swarmSpaceSegBtns[i].background = roundedColor(if (sel) pal.accent else Color.TRANSPARENT, dp(5f))
        }
        if (::btnToggleSwarmMesh.isInitialized) {
            btnToggleSwarmMesh.setTextColor(if (hub.showSwarmLinks) pal.secondaryAccent else pal.textMuted)
            GcsIcons.bind(
                btnToggleSwarmMesh,
                Type.SWARM_FLEET,
                if (hub.showSwarmLinks) "拓扑连线: 开" else "拓扑连线: 关",
                d,
                iconDp = 11f
            )
        }

        val isMultiIp = (rosManager.topics.swarmTopology == 1)
        segSwarmTopoNsBtn?.let { b ->
            b.setTextColor(if (!isMultiIp) Color.WHITE else pal.textMuted)
            b.background = roundedColor(if (!isMultiIp) pal.accent else Color.TRANSPARENT, dp(5f))
        }
        segSwarmTopoIpBtn?.let { b ->
            b.setTextColor(if (isMultiIp) Color.WHITE else pal.textMuted)
            b.background = roundedColor(if (isMultiIp) pal.accent else Color.TRANSPARENT, dp(5f))
        }
        wrapSwarmNsFields?.visibility = if (!isMultiIp) View.VISIBLE else View.GONE
        wrapSwarmIpFields?.visibility = if (isMultiIp) View.VISIBLE else View.GONE
    }

    private fun buildTiltControlCardInDrawer(): View {
        val card = sectionCard("倾转矢量飞控协同 (Tilt-UAV-INDI · 4M+2S)")

        val rigRow = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }
        btnToggleTiltRig = createSmallIconBtn(Type.TILT_VECTOR, "当前机: 矢量 4M2S") {
            val nextVec = !hub.activeFocusIsTiltRig()
            hub.setActiveFocusTiltRig(nextVec)
            val who = if (hub.swarmModeEnabled && hub.swarmRigPolicy == TiltConfigSpec.FLEET_RIG_MIXED) {
                "UAV-#${hub.selectedDroneId}"
            } else if (hub.swarmModeEnabled) {
                "全队 (${hub.swarmDroneCount}架)"
            } else {
                "单机"
            }
            val rigTip = if (nextVec) {
                "$who 已启用双舵倾转矢量构型 (4M+2S)"
            } else {
                "$who 已切换为常规固定 X4 普通四旋翼构型"
            }
            applyOperatingModeChangeLive(rigTip)
        }
        val btnOpenTiltAdv = createSmallIconBtn(Type.SLIDERS, "限幅设置") {
            setDrawerOpenState(false)
            setSettingsModalOpen(true, jumpCategory = GcsSettingsModal.CAT_CONTROL)
        }
        rigRow.addView(btnToggleTiltRig, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1.45f))
        rigRow.addView(btnOpenTiltAdv, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dp(5f) })
        card.addView(rigRow)

        tvTiltCardHint = TextView(this).apply {
            text = "当前焦点为双舵倾转矢量构型 · 支持零俯仰平飞或窄缝主动俯仰协同"
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.1f)
            setPadding(0, dp(5f), 0, dp(1f))
            GcsTheme.markRole(this, Role.TEXT_ACCENT)
        }
        card.addView(tvTiltCardHint)

        card.addView(
            TextView(this).apply {
                text = "俯仰-倾转分配策略 (林下稳激光视场选「平飞解耦」)"
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.3f)
                setPadding(0, dp(6f), 0, dp(3f))
                GcsTheme.markRole(this, Role.TEXT_MUTED)
            }
        )
        val modeTrack = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dp(2f), dp(2f), dp(2f), dp(2f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        tiltModeDrawerBtns.clear()
        for (i in TiltConfigSpec.SHORT_MODE_NAMES.indices) {
            val b = TextView(this).apply {
                text = TiltConfigSpec.SHORT_MODE_NAMES[i]
                maxLines = 1
                setHorizontallyScrolling(false)
                gravity = Gravity.CENTER
                minimumHeight = dp(26f)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.6f)
                typeface = Typeface.DEFAULT_BOLD
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    hub.setActiveFocusTiltRig(true)
                    hub.tiltCoordMode = i
                    applyOperatingModeChangeLive("倾转分配：${TiltConfigSpec.FULL_MODE_NAMES[i]}")
                }
            }
            tiltModeDrawerBtns.add(b)
            modeTrack.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        card.addView(modeTrack)

        card.addView(
            TextView(this).apply {
                text = "窄缝/坡地主动机身俯仰偏置角 (舵机自动反向倾转维持升力)"
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.3f)
                setPadding(0, dp(6f), 0, dp(3f))
                GcsTheme.markRole(this, Role.TEXT_MUTED)
            }
        )
        val biasTrack = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(dp(2f), dp(2f), dp(2f), dp(2f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 6f)
        }
        val biasPresets = listOf(-15f to "-15°前俯", 0f to "0°水平", 15f to "+15°仰穿", 25f to "+25°窄缝")
        tiltBiasDrawerBtns.clear()
        for ((deg, lbl) in biasPresets) {
            val b = TextView(this).apply {
                text = lbl
                maxLines = 1
                setHorizontallyScrolling(false)
                gravity = Gravity.CENTER
                minimumHeight = dp(26f)
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
                typeface = Typeface.DEFAULT_BOLD
                setOnClickListener {
                    AnimHelper.triggerHaptic(it)
                    hub.setActiveFocusTiltRig(true)
                    hub.tiltCoordMode = TiltConfigSpec.MODE_ACTIVE_PITCH
                    hub.tiltPitchBiasDeg = deg
                    applyOperatingModeChangeLive(String.format(Locale.US, "已设定主动机身俯仰偏置 %+.0f°", deg))
                }
            }
            tiltBiasDrawerBtns.add(b)
            biasTrack.addView(b, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        }
        card.addView(biasTrack)
        return card
    }

    private fun publishTiltSettingsToRos() {
        rosManager.publishTiltCommand(hub.tiltPitchBiasDeg, hub.tiltCoordMode, hub.tiltMaxAngleDeg)
        rosManager.setParamDebounced("/active_pitch_planner/tilt_coord_mode", hub.tiltCoordMode.toFloat())
        rosManager.setParamDebounced("/active_pitch_planner/pitch_bias_deg", hub.tiltPitchBiasDeg)
        rosManager.setParamDebounced("/active_pitch_planner/max_tilt_deg", hub.tiltMaxAngleDeg)
    }

    private fun syncTiltAllControlsUi() {
        val pal = GcsTheme.of(hub.themeId)
        syncTopQuickSlotsVisual()
        val tiltOn = hub.activeFocusIsTiltRig()
        val scopePrefix = if (hub.swarmModeEnabled && hub.swarmRigPolicy == TiltConfigSpec.FLEET_RIG_MIXED) {
            "#${hub.selectedDroneId}: "
        } else if (hub.swarmModeEnabled) {
            "全队: "
        } else {
            "单机: "
        }
        if (::btnToggleTiltRig.isInitialized) {
            btnToggleTiltRig.setTextColor(if (tiltOn) pal.secondaryAccent else pal.textPrimary)
            GcsIcons.bind(
                btnToggleTiltRig,
                if (tiltOn) Type.TILT_VECTOR else Type.DRONE_QUAD,
                scopePrefix + if (tiltOn) "矢量 4M2S" else "普通 X4",
                d,
                iconDp = 11.5f
            )
        }
        tvTiltCardHint?.let { hint ->
            if (tiltOn) {
                hint.setTextColor(pal.accentBright)
                hint.text = "${scopePrefix}双舵倾转矢量已激活 (S1->M1/M3, S2->M2/M4)"
            } else {
                hint.setTextColor(pal.textMuted)
                hint.text = "${scopePrefix}当前为常规固定 X4 四旋翼 (点上方按钮或下方模式即可切回倾转矢量)"
            }
        }
        for (i in tiltModeDrawerBtns.indices) {
            val sel = tiltOn && (hub.tiltCoordMode == i)
            tiltModeDrawerBtns[i].setTextColor(if (sel) Color.WHITE else pal.textMuted)
            tiltModeDrawerBtns[i].background = roundedColor(if (sel) pal.accent else Color.TRANSPARENT, dp(5f))
        }
        val biasVals = floatArrayOf(-15f, 0f, 15f, 25f)
        for (i in tiltBiasDrawerBtns.indices) {
            val sel = tiltOn && hub.tiltCoordMode == TiltConfigSpec.MODE_ACTIVE_PITCH &&
                kotlin.math.abs(hub.tiltPitchBiasDeg - biasVals[i]) < 2.5f
            tiltBiasDrawerBtns[i].setTextColor(if (sel) Color.WHITE else pal.textMuted)
            tiltBiasDrawerBtns[i].background = roundedColor(
                if (sel) Color.parseColor("#FF8F00") else Color.TRANSPARENT,
                dp(5f)
            )
        }
    }

    // ==================== 零基线偏移原子组件工具 ====================

    private fun dp(v: Float): Int = (v * d).roundToInt()

    private fun glassBg(radiusPx: Int): GradientDrawable {
        val pal = GcsTheme.of(hub.themeId)
        return GradientDrawable(
            GradientDrawable.Orientation.TOP_BOTTOM,
            intArrayOf(pal.glassFillTop, pal.glassFill)
        ).apply {
            cornerRadius = radiusPx.toFloat()
            setStroke(maxOf(1, dp(0.9f)), pal.glassStroke)
        }
    }

    private fun pillBg(fillCol: Int, strokeCol: Int, radiusDp: Float = 18f): GradientDrawable = GradientDrawable().apply {
        setColor(fillCol)
        cornerRadius = dp(radiusDp).toFloat()
        setStroke(maxOf(1, dp(0.9f)), strokeCol)
    }

    private fun roundedColor(col: Int, radiusPx: Int): GradientDrawable = GradientDrawable().apply {
        setColor(col)
        cornerRadius = radiusPx.toFloat()
    }

    private fun camPillBg(active: Boolean): GradientDrawable {
        val pal = GcsTheme.of(hub.themeId)
        return if (active) {
            GradientDrawable().apply {
                cornerRadius = dp(14f).toFloat()
                setColor(pal.accentSoftFill)
                setStroke(maxOf(1, dp(0.95f)), pal.accent)
            }
        } else {
            GradientDrawable(
                GradientDrawable.Orientation.TOP_BOTTOM,
                intArrayOf(pal.glassFillTop, pal.glassFill)
            ).apply {
                cornerRadius = dp(14f).toFloat()
                setStroke(maxOf(1, dp(0.9f)), pal.glassStroke)
            }
        }
    }

    private fun monoLabel(txt: String, hexCol: String, sizeDp: Float, themeRole: Role? = null): TextView = TextView(this).apply {
        text = txt
        maxLines = 1
        setHorizontallyScrolling(false)
        setTextColor(Color.parseColor(hexCol))
        setTextSize(TypedValue.COMPLEX_UNIT_DIP, sizeDp)
        GcsTheme.applyTabularReadout(this, bold = true)
        if (themeRole != null) {
            GcsTheme.markRole(this, themeRole)
        }
    }

    private fun createActionPill(icon: Type, label: String, accentHex: String, active: Boolean): TextView = TextView(this).apply {
        val pal = GcsTheme.of(hub.themeId)
        maxLines = 1
        setHorizontallyScrolling(false)
        gravity = Gravity.CENTER
        minWidth = dp(88f)
        minimumHeight = dp(30f)
        setTextColor(pal.textPrimary)
        setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
        typeface = Typeface.DEFAULT_BOLD
        setPadding(dp(10.5f), dp(5f), dp(11f), dp(5f))
        val ac = Color.parseColor(accentHex)
        background = pillBg(if (active) Color.argb(56, Color.red(ac), Color.green(ac), Color.blue(ac)) else pal.glassFill, ac)
        GcsIcons.bind(this, icon, label, d, iconDp = 11.5f, gapDp = 4f)
        AnimHelper.attachSpringTouch(this)
    }

    private fun createCamPillBtn(icon: Type, txt: String, active: Boolean, trailing: Type? = null): TextView = TextView(this).apply {
        val pal = GcsTheme.of(hub.themeId)
        maxLines = 1
        setHorizontallyScrolling(false)
        gravity = Gravity.CENTER
        minimumHeight = dp(29f)
        setPadding(dp(8f), dp(4.5f), dp(8f), dp(4.5f))
        setTextColor(pal.textPrimary)
        setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.8f)
        typeface = Typeface.DEFAULT_BOLD
        background = camPillBg(active)
        GcsIcons.bind(this, icon, txt, d, iconDp = 11.5f, gapDp = 3f, trailing = trailing, trailDp = 9f)
        AnimHelper.attachSpringTouch(this)
    }

    private fun createSmallIconBtn(icon: Type?, txt: String, onClick: () -> Unit): TextView = TextView(this).apply {
        maxLines = 1
        setHorizontallyScrolling(false)
        gravity = Gravity.CENTER
        minimumHeight = dp(29f)
        setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.8f)
        typeface = Typeface.DEFAULT_BOLD
        setPadding(dp(7.5f), dp(5f), dp(7.5f), dp(5f))
        GcsTheme.markRole(this, Role.GLASS_PANEL, cornerDp = 5.5f)
        setTextColor(GcsTheme.of(hub.themeId).textPrimary)
        GcsIcons.bind(this, icon, txt, d, iconDp = 11f, gapDp = 3.5f)
        setOnClickListener {
            AnimHelper.triggerHaptic(it)
            onClick()
        }
        AnimHelper.attachSpringTouch(this)
    }

    private fun sectionCard(title: String): LinearLayout = LinearLayout(this).apply {
        orientation = LinearLayout.VERTICAL
        setPadding(dp(9.5f), dp(8f), dp(9.5f), dp(8.5f))
        GcsTheme.markRole(this, Role.SECTION_CARD, cornerDp = 8.5f)
        val hdr = LinearLayout(this@MainActivity).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setPadding(0, 0, 0, dp(5.5f))
        }
        val bar = View(this@MainActivity).apply {
            GcsTheme.markRole(this, Role.ACCENT_BAR, cornerDp = 2f)
        }
        val tv = TextView(this@MainActivity).apply {
            text = title
            maxLines = 2
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.2f)
            typeface = Typeface.DEFAULT_BOLD
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
        }
        hdr.addView(bar, LinearLayout.LayoutParams(dp(3f), dp(11f)).apply { rightMargin = dp(5.5f) })
        hdr.addView(tv, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        addView(hdr)
    }

    private fun fullMetricRow(label: String, valueTv: TextView): View {
        return LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setPadding(dp(8f), dp(5.5f), dp(8f), dp(5.5f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 5f)
            addView(
                TextView(this@MainActivity).apply {
                    text = label
                    maxLines = 1
                    setHorizontallyScrolling(false)
                    setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.4f)
                    GcsTheme.markRole(this, Role.TEXT_MUTED)
                },
                LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f)
            )
            addView(valueTv, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(6f) })
            layoutParams = LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply {
                topMargin = dp(2.5f)
                bottomMargin = dp(2.5f)
            }
        }
    }

    private fun twoColRow(lbl1: String, val1: TextView, lbl2: String, val2: TextView): View {
        val row = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            setPadding(0, dp(2.5f), 0, dp(2.5f))
        }
        fun cell(l: String, v: TextView): LinearLayout = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(7f), dp(5f), dp(7f), dp(5f))
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 5f)
            addView(
                TextView(this@MainActivity).apply {
                    text = l
                    maxLines = 1
                    setHorizontallyScrolling(false)
                    setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.2f)
                    setPadding(0, 0, 0, dp(2f))
                    GcsTheme.markRole(this, Role.TEXT_MUTED)
                }
            )
            addView(v)
        }
        row.addView(cell(lbl1, val1), LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        row.addView(cell(lbl2, val2), LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dp(5f) })
        return row
    }

    private fun labeledInput(parent: LinearLayout, title: String, initVal: String): EditText {
        parent.addView(
            TextView(this).apply {
                text = title
                maxLines = 2
                setTextSize(TypedValue.COMPLEX_UNIT_DIP, 9.5f)
                setPadding(0, dp(5.5f), 0, dp(2.5f))
                GcsTheme.markRole(this, Role.TEXT_MUTED)
            }
        )
        return EditText(this).apply {
            setText(initVal)
            setSingleLine(true)
            minimumHeight = dp(35f)
            inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI
            setTextSize(TypedValue.COMPLEX_UNIT_DIP, 10.6f)
            typeface = Typeface.MONOSPACE
            setPadding(dp(8f), dp(5.5f), dp(8f), dp(5.5f))
            GcsTheme.markRole(this, Role.INPUT_BOX, cornerDp = 5f)
            parent.addView(this, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT))
        }
    }
}
