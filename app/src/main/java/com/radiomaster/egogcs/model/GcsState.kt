package com.radiomaster.egogcs.model

import android.graphics.Color
import kotlin.math.cos
import kotlin.math.sin

/**
 * 多机集群战术配色与几何编队解算工具 (`SwarmPalette`)：
 * 为 UAV-0..UAV-3 提供高对比航空战术色标及四种防撞编队几何偏移（三角楔形、横向一字、穿林纵队、菱形方阵）。
 */
/**
 * 倾转四旋翼 (`Friendzhb/tilt-uav-indi` · 4电机 + 2沿 Yb 轴倾转舵机) 硬件标定参数与控制分配模式常量：
 * - 电机通道 (M1..M4): 一阶响应时间常数 tau_m = 0.028s
 * - 倾转舵机 (S1 控制 M1/M3 前组, S2 控制 M2/M4 后组): 二阶欠阻尼 tau_s = 0.075s, zeta = 0.68, 速率限幅 300 deg/s
 */
object TiltConfigSpec {
    const val MODE_ZERO_PITCH = 0 // 零俯仰平飞解耦 (机身水平稳激光视场，纯倾转矢量产生水平推力)
    const val MODE_ACTIVE_PITCH = 1 // 主动俯仰-倾转协同 (active_pitch_planner_node Jerk前馈 + 窄缝姿态偏置)
    const val MODE_FIXED_QUAD = 2 // 倾转舵机归零锁定 (退化为常规欠驱动四旋翼基准)

    const val TAU_MOTOR_SEC = 0.028f
    const val TAU_SERVO_SEC = 0.075f
    const val ZETA_SERVO = 0.68f
    const val SERVO_RATE_MAX_DEG_S = 300.0f

    // 多机集群构型分配策略: 0=全普通X4, 1=全矢量4M2S, 2=异构混编(按机位掩码独立构型)
    const val FLEET_RIG_ALL_STANDARD = 0
    const val FLEET_RIG_ALL_VECTORED = 1
    const val FLEET_RIG_MIXED = 2

    val SHORT_MODE_NAMES = arrayOf("平飞解耦", "主动协同", "倾转锁定")
    val FULL_MODE_NAMES = arrayOf("零俯仰矢量平飞", "主动俯仰-倾转协同", "倾转归零常规模式")

    val FLEET_RIG_SHORT_NAMES = arrayOf("全普通", "全矢量", "混编")
    val FLEET_RIG_FULL_NAMES = arrayOf("全队普通 X4 四旋翼", "全队双舵倾转矢量 4M2S", "异构混编 (按机独立指定)")
}

object SwarmPalette {
    const val MAX_SWARM_DRONES = 4

    val COLOR_HEX = arrayOf(
        "#00E676", // UAV-0 领航翠绿
        "#29B6F6", // UAV-1 僚机天青
        "#FFB300", // UAV-2 僚机琥珀
        "#FF5286" // UAV-3 僚机珊瑚
    )

    val COLOR_INT = IntArray(MAX_SWARM_DRONES) { i -> Color.parseColor(COLOR_HEX[i]) }

    val RGB_FLOAT = arrayOf(
        floatArrayOf(0.00f, 0.90f, 0.46f),
        floatArrayOf(0.16f, 0.71f, 0.96f),
        floatArrayOf(1.00f, 0.70f, 0.00f),
        floatArrayOf(1.00f, 0.32f, 0.53f)
    )

    val FORMATION_SHORT_NAMES = arrayOf("楔形", "横队", "纵队", "菱形")
    val FORMATION_FULL_NAMES = arrayOf("三角楔形", "横向一字", "穿林纵队", "菱形方阵")

    /**
     * 计算第 [slotIdx] 架无人机在编队局部系下的 (前进轴 fwd, 左侧轴 left) 米制偏移，并旋转至世界坐标写入 [outWorldXY]。
     */
    fun computeWorldSlot(
        anchorX: Float,
        anchorY: Float,
        headingRad: Float,
        slotIdx: Int,
        totalCount: Int,
        formationType: Int,
        spacingMeters: Float,
        outWorldXY: FloatArray
    ) {
        val n = totalCount.coerceIn(1, MAX_SWARM_DRONES)
        if (n <= 1) {
            outWorldXY[0] = anchorX
            outWorldXY[1] = anchorY
            return
        }
        val s = spacingMeters.coerceIn(0.8f, 5.0f)
        var fwd = 0f
        var left = 0f

        when (formationType.coerceIn(0, 3)) {
            0 -> { // 三角楔形 (V-Wedge)
                when (n) {
                    2 -> {
                        fwd = if (slotIdx == 0) 0.35f * s else -0.35f * s
                        left = if (slotIdx == 0) 0.52f * s else -0.52f * s
                    }
                    3 -> when (slotIdx) {
                        0 -> {
                            fwd = 0.52f * s
                            left = 0f
                        }
                        1 -> {
                            fwd = -0.38f * s
                            left = 0.82f * s
                        }
                        else -> {
                            fwd = -0.38f * s
                            left = -0.82f * s
                        }
                    }
                    else -> when (slotIdx) {
                        0 -> {
                            fwd = 0.65f * s
                            left = 0f
                        }
                        1 -> {
                            fwd = -0.12f * s
                            left = 0.84f * s
                        }
                        2 -> {
                            fwd = -0.12f * s
                            left = -0.84f * s
                        }
                        else -> {
                            fwd = -0.88f * s
                            left = 0f
                        }
                    }
                }
            }
            1 -> { // 横向一字 (Line Abreast)
                val center = (n - 1) * 0.5f
                fwd = 0f
                left = (center - slotIdx) * s
            }
            2 -> { // 穿林纵队 (Column - 适配槟榔林/果园窄行距纵向穿越)
                val center = (n - 1) * 0.5f
                fwd = (center - slotIdx) * s
                left = 0f
            }
            else -> { // 菱形方阵 (Diamond)
                when (n) {
                    2 -> {
                        fwd = 0f
                        left = if (slotIdx == 0) 0.55f * s else -0.55f * s
                    }
                    3 -> when (slotIdx) {
                        0 -> {
                            fwd = 0.58f * s
                            left = 0f
                        }
                        1 -> {
                            fwd = -0.32f * s
                            left = 0.68f * s
                        }
                        else -> {
                            fwd = -0.32f * s
                            left = -0.68f * s
                        }
                    }
                    else -> when (slotIdx) {
                        0 -> {
                            fwd = 0.72f * s
                            left = 0f
                        }
                        1 -> {
                            fwd = 0f
                            left = 0.72f * s
                        }
                        2 -> {
                            fwd = 0f
                            left = -0.72f * s
                        }
                        else -> {
                            fwd = -0.72f * s
                            left = 0f
                        }
                    }
                }
            }
        }

        val c = cos(headingRad)
        val sn = sin(headingRad)
        outWorldXY[0] = anchorX + fwd * c - left * sn
        outWorldXY[1] = anchorY + fwd * sn + left * c
    }
}

/**
 * 单架集群无人机零分配渲染帧槽位 (`SwarmAgentFrame`)：
 * 预分配在四缓冲 `RenderSnapshot` 内，60FPS 渲染循环零 GC。
 */
class SwarmAgentFrame {
    var droneId: Int = 0
    var online: Boolean = true
    var armed: Boolean = true

    var uavX: Float = -5.5f
    var uavY: Float = -4.2f
    var uavZ: Float = 1.2f
    var vx: Float = 0f
    var vy: Float = 0f
    var vz: Float = 0f

    var yaw: Float = 0.65f
    var smoothRoll: Float = 0f
    var smoothPitch: Float = 0f
    var isTiltRig: Boolean = true
    var tiltServo1Rad: Float = 0f
    var tiltServo2Rad: Float = 0f

    var homeX: Float = -5.5f
    var homeY: Float = -4.2f
    var goalX: Float = 5.2f
    var goalY: Float = 3.8f
    var goalZ: Float = 1.2f

    var batVolt: Float = 16.24f
    var batPct: Int = 86
    var fsmState: String = "EXEC_TRAJ"

    val bsplinePts: FloatArray = FloatArray(240 * 3)
    var bsplineCount: Int = 0

    val trailPts: FloatArray = FloatArray(160 * 3)
    var trailCount: Int = 0

    // 全队编队选点预览时该架无人机的目标落位槽坐标与安全净空标志
    var previewSlotX: Float = 0f
    var previewSlotY: Float = 0f
    var previewSlotZ: Float = 1.2f
    var previewSlotSafe: Boolean = true
}

/**
 * 预分配四缓冲零 GC 渲染帧快照 (Zero-Allocation Quad-Buffered Frame Snapshot)：
 * 包含完整三维位置、速度、真实四元数与欧拉角姿态 (Roll / Pitch / Yaw)、多机编队态势及 EGO 三维膨胀体素地图。
 */
class RenderSnapshot {
    var uavX: Float = -5.5f
    var uavY: Float = -4.2f
    var uavZ: Float = 1.2f
    var vx: Float = 0f
    var vy: Float = 0f
    var vz: Float = 0f
    var ax: Float = 0f
    var ay: Float = 0f

    // 当前主控焦点无人机实时三维姿态：yaw (世界系偏航 rad), smoothRoll (右倾为正 rad), smoothPitch (抬头为正 rad) + 单位四元数
    var yaw: Float = 0.65f
    var smoothRoll: Float = 0f
    var smoothPitch: Float = 0f
    var quatW: Float = 1f
    var quatX: Float = 0f
    var quatY: Float = 0f
    var quatZ: Float = 0f
    var hasLiveAttitudeQuat: Boolean = false

    // 当前主控焦点机是否为倾转矢量构型 (4M2S=true, 常规固定X4=false)
    var activeIsTiltRig: Boolean = true

    // Tilt-UAV-INDI 双舵机倾转角与四电机控制分配实时遥测 (S1=M1/M3前倾转轴, S2=M2/M4后倾转轴)
    var tiltServo1Rad: Float = 0f
    var tiltServo2Rad: Float = 0f
    var tiltServo1RateDegS: Float = 0f
    var tiltServo2RateDegS: Float = 0f
    var tiltCmd1Rad: Float = 0f
    var tiltCmd2Rad: Float = 0f
    val motorNorm4: FloatArray = floatArrayOf(0.52f, 0.52f, 0.52f, 0.52f)
    var tiltManipulability: Float = 0.96f
    var tiltThrustMarginPct: Int = 48
    var tiltServoSaturated: Boolean = false
    var hasLiveTiltTelemetry: Boolean = false

    var homeX: Float = -5.5f
    var homeY: Float = -4.2f
    var goalX: Float = 5.2f
    var goalY: Float = 3.8f
    var goalZ: Float = 1.2f
    var fsmState: String = "EXEC_TRAJ"
    var px4Mode: String = "OFFBOARD"
    var armed: Boolean = true

    // 当前焦点无人机动力电池遥测 (来自 ROS1 /mavros/battery 或 AX12 ELRS Backpack MAVLink UDP:14550)
    var batVolt: Float = 16.24f
    var batCurrentAmps: Float = 12.6f
    var batPct: Int = 86

    // AX12 遥控器本机电源遥测 (来自 Android 原生 BatteryManager，双节 21700 10000mAh)
    var txBatPct: Int = 92
    var txBatVolt: Float = 8.08f
    var txCurrentAmps: Float = 1.05f

    var odomHz: Int = 50
    var cloudHz: Int = 15
    var cloudScanPts: Int = 0
    var mapPointCount: Int = 0
    var solveMs: Float = 0.4f
    var minClearance: Float = 1.45f
    var trajLength: Float = 6.2f

    // 选点预览光柱状态
    var hasPreviewPin: Boolean = false
    var previewX: Float = 0f
    var previewY: Float = 0f
    var previewZ: Float = 1.2f
    var previewClearance: Float = 9.5f
    var previewSafe: Boolean = true

    // 多机集群协同态势快照 (swarmCount == 1 表示单机模式，2..4 表示多机编队模式)
    var swarmCount: Int = 1
    var activeDroneId: Int = 0
    var swarmCommandAll: Boolean = true
    var minInterDroneDist: Float = 9.9f
    val swarmAgents: Array<SwarmAgentFrame> = Array(SwarmPalette.MAX_SWARM_DRONES) { idx ->
        SwarmAgentFrame().apply { droneId = idx }
    }

    val obsPrimitives: FloatArray = FloatArray(24 * 6)
    var obsPrimitiveCount: Int = 0

    val voxels: FloatArray = FloatArray(4096 * 4)
    var voxelCount: Int = 0

    val astarPts: FloatArray = FloatArray(600 * 3)
    var astarCount: Int = 0

    val bsplinePts: FloatArray = FloatArray(420 * 3)
    var bsplineCount: Int = 0

    val ctrlPts: FloatArray = FloatArray(24 * 3)
    var ctrlCount: Int = 0

    val trailPts: FloatArray = FloatArray(240 * 3)
    var trailCount: Int = 0

    val lioPathPts: FloatArray = FloatArray(820 * 3)
    var lioPathCount: Int = 0

    val waypointQueuePts: FloatArray = FloatArray(16 * 3)
    var waypointQueueCount: Int = 0
    var isRouteStagedOnly: Boolean = false
    var stagedRouteArcLen: Float = 0f
    var routeClosedLoop: Boolean = false

    val radarSmooth60: FloatArray = FloatArray(60) { 9.5f }
}

data class TopicConfig(
    var protocol: String = "tcpros",
    var masterUri: String = "http://192.168.1.100:11311",
    var wsUrl: String = "ws://192.168.1.100:9090",
    var odomTopic: String = "/Odometry",
    var fastLioCloudTopic: String = "/cloud_registered",
    var fastLioPathTopic: String = "/path",
    var inflateTopic: String = "/grid_map/occupancy_inflate",
    var optimalTopic: String = "/ego_planner_node/optimal_list",
    var goalTopic: String = "/move_base_simple/goal",
    // ROS1 多机集群拓扑: 0=同Master多命名空间(兼容 ego-planner-swarm drone_{id}), 1=多机独立IP直连(各机独立 roscore)
    var swarmTopology: Int = 0,
    var swarmPeerIpsCsv: String = "192.168.1.100, 192.168.1.101, 192.168.1.102",
    var swarmOdomPattern: String = "/drone_{id}_visual_slam/odom",
    var swarmOptimalPattern: String = "/drone_{id}_ego_planner_node/optimal_list",
    var swarmGoalPattern: String = "/drone_{id}_move_base_simple/goal",
    // Tilt-UAV-INDI 单机与多机倾转舵机反馈及外环主动俯仰-倾转指令话题模板
    var tiltTelemetryTopic: String = "/tilt_uav/servo_angles",
    var tiltCmdTopic: String = "/active_pitch_planner/pitch_tilt_cmd",
    var swarmTiltSubPattern: String = "/drone_{id}_tilt_uav/servo_angles",
    var swarmTiltCmdPattern: String = "/drone_{id}_active_pitch_planner/pitch_tilt_cmd"
)

class GcsSharedHub {
    @Volatile var isRosMode: Boolean = false

    @Volatile var rosConnected: Boolean = false

    @Volatile var connectionLabel: String = "仿真演练"

    @Volatile var connectionLevel: Int = 0

    @Volatile var rosRxKbps: Float = 0f

    @Volatile var rosReconnectCount: Int = 0

    @Volatile var lastRosOdomWallMs: Long = 0L

    @Volatile var renderFps: Int = 60

    @Volatile var frameRenderMs: Float = 2.0f

    // 硬件渲染性能模式: 0=均衡高帧(60FPS推荐), 1=极致画质(全量体素), 2=户外长续航省电(30FPS温控节流)
    @Volatile var perfMode: Int = 0

    @Volatile var themeId: Int = 0

    @Volatile var animStyle: Int = 0

    @Volatile var cameraDampingScale: Float = 1.0f

    @Volatile var autoConnectDiscovered: Boolean = true

    @Volatile var mavlinkBackpackEnabled: Boolean = true

    @Volatile var xrayOcclusionEnabled: Boolean = true

    @Volatile var droneCurrentlyOccluded: Boolean = false

    @Volatile var txBatPct: Int = 92

    @Volatile var txBatVolt: Float = 8.08f

    @Volatile var txCurrentAmps: Float = 1.05f

    @Volatile var obstaclesInflation: Float = 0.30f

    @Volatile var maxVel: Float = 2.0f

    @Volatile var maxAcc: Float = 3.0f

    @Volatile var goalZ: Float = 1.2f

    @Volatile var zSliceMax: Float = 3.0f

    @Volatile var mapVoxelSize: Float = 0.12f

    @Volatile var showFastLioMap: Boolean = true

    @Volatile var showInflate: Boolean = true

    @Volatile var showCore: Boolean = false

    @Volatile var showCtrlPts: Boolean = true

    @Volatile var showLocalBox: Boolean = true

    @Volatile var showGroundGrid: Boolean = true

    @Volatile var showTrajCurtain: Boolean = true

    @Volatile var showFlightTrail: Boolean = true

    @Volatile var autoPatrol: Boolean = true

    @Volatile var pointColorMode: Int = 0

    @Volatile var pointSizeScale: Float = 1.0f

    @Volatile var longPressQuickFlyEnabled: Boolean = true

    // v3.9: 雨林水滴/携行防误触地图选点锁 (开启后屏蔽地图单点/长按指点，紧急悬停始终可用)
    @Volatile var hudTouchLocked: Boolean = false

    @Volatile var rthDoubleConfirm: Boolean = true

    @Volatile var hapticFeedbackEnabled: Boolean = true

    @Volatile var autoCloseDrawerOnConnect: Boolean = true

    @Volatile var altStepMeters: Float = 0.2f

    @Volatile var warnClearanceMeters: Float = 0.65f

    @Volatile var lowBatWarnPct: Int = 25

    // HUD 座舱组件显隐与顶栏快捷入口编排
    @Volatile var hudShowCompass: Boolean = true

    @Volatile var hudShowHsiRadar: Boolean = true

    @Volatile var hudShowTelemetryBox: Boolean = true

    @Volatile var hudShowAttitudeWidget: Boolean = true

    @Volatile var hudShowRightCamStack: Boolean = true

    @Volatile var hudShowDemoTools: Boolean = true

    @Volatile var quickSlotOrderCsv: String = "mission,topics,pcd,recenter"

    @Volatile var quickShowMission: Boolean = true

    @Volatile var quickShowTopics: Boolean = true

    @Volatile var quickShowPcd: Boolean = false

    @Volatile var quickShowRecenter: Boolean = false

    @Volatile var routeLoopMode: Int = 0

    @Volatile var routeFinishAction: Int = 0

    // EGO-Planner-Swarm 多机协同编队模式参数
    @Volatile var swarmModeEnabled: Boolean = false

    @Volatile var swarmDroneCount: Int = 3

    @Volatile var selectedDroneId: Int = 0

    // 0=单机指点控制(仅控制 selectedDroneId), 1=全队协同编队集结(全部活跃飞机按队形同步避障飞行)
    @Volatile var swarmCommandScope: Int = 1

    // 0=三角楔形(V-Wedge), 1=横向一字(Line), 2=穿林纵队(Column), 3=菱形方阵(Diamond)
    @Volatile var swarmFormationType: Int = 0

    @Volatile var swarmSpacingMeters: Float = 1.8f

    @Volatile var showSwarmLinks: Boolean = true

    // Tilt-UAV-INDI (4电机 + 2倾转舵机) 与常规固定 X4 构型无缝切换及多机异构混编参数
    @Volatile var tiltAirframeEnabled: Boolean = true

    @Volatile var swarmRigPolicy: Int = TiltConfigSpec.FLEET_RIG_ALL_VECTORED

    // 异构混编模式下 UAV-0..3 的矢量构型位掩码 (bit i == 1 表示 UAV-#i 为 4M2S 矢量机，0 表示普通 X4)
    @Volatile var swarmDroneTiltMask: Int = 0b0001

    @Volatile var tiltCoordMode: Int = TiltConfigSpec.MODE_ACTIVE_PITCH

    @Volatile var tiltPitchBiasDeg: Float = 0.0f

    @Volatile var tiltMaxAngleDeg: Float = 45.0f

    // v3.4: 窄缝净空低于阈值时自动微幅抬头减小水平桨盘迎风/碰撞截面
    @Volatile var autoNarrowGapPitchAssist: Boolean = true

    fun isDroneTiltRig(droneId: Int): Boolean {
        if (!swarmModeEnabled) return tiltAirframeEnabled
        return when (swarmRigPolicy) {
            TiltConfigSpec.FLEET_RIG_ALL_STANDARD -> false
            TiltConfigSpec.FLEET_RIG_ALL_VECTORED -> true
            else -> (swarmDroneTiltMask and (1 shl droneId.coerceIn(0, 3))) != 0
        }
    }

    fun activeFocusIsTiltRig(): Boolean =
        isDroneTiltRig(if (swarmModeEnabled) selectedDroneId else 0)

    fun anyActiveDroneUsesTilt(): Boolean {
        if (!swarmModeEnabled) return tiltAirframeEnabled
        val cnt = swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
        for (id in 0 until cnt) {
            if (isDroneTiltRig(id)) return true
        }
        return false
    }

    fun setActiveFocusTiltRig(vectored: Boolean) {
        if (!swarmModeEnabled) {
            tiltAirframeEnabled = vectored
            return
        }
        when (swarmRigPolicy) {
            TiltConfigSpec.FLEET_RIG_MIXED -> {
                val bit = 1 shl selectedDroneId.coerceIn(0, 3)
                swarmDroneTiltMask = if (vectored) (swarmDroneTiltMask or bit) else (swarmDroneTiltMask and bit.inv())
            }
            else -> {
                swarmRigPolicy = if (vectored) TiltConfigSpec.FLEET_RIG_ALL_VECTORED else TiltConfigSpec.FLEET_RIG_ALL_STANDARD
                tiltAirframeEnabled = vectored
            }
        }
    }

    fun toggleSingleDroneRigInSwarm(droneId: Int): Boolean {
        val safeId = droneId.coerceIn(0, 3)
        if (swarmRigPolicy != TiltConfigSpec.FLEET_RIG_MIXED) {
            // 从同构集群平滑派生初始位掩码后再翻转目标飞机
            swarmDroneTiltMask = if (swarmRigPolicy == TiltConfigSpec.FLEET_RIG_ALL_VECTORED) 0b1111 else 0b0000
            swarmRigPolicy = TiltConfigSpec.FLEET_RIG_MIXED
        }
        val bit = 1 shl safeId
        swarmDroneTiltMask = swarmDroneTiltMask xor bit
        return (swarmDroneTiltMask and bit) != 0
    }

    fun formatRosModeSuffix(): String {
        return if (!swarmModeEnabled) {
            if (tiltAirframeEnabled) "单机矢量" else "单机X4"
        } else {
            val cnt = swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
            val tag = TiltConfigSpec.FLEET_RIG_SHORT_NAMES[swarmRigPolicy.coerceIn(0, 2)]
            "多机$cnt·$tag"
        }
    }

    // GitHub Releases 直链 OTA 更新通道: 0=智能实时加速(默认推荐), 1=GitHub官方直链, 2=自定义直链/镜像源
    @Volatile var otaChannelMode: Int = 0

    @Volatile var otaCustomUrl: String =
        "https://github.com/Friendzhb/RadioMaster-EGO-GCS/releases/download/latest-apk/EGO-Pilot-AX12.apk"

    @Volatile var otaAutoCheckOnStart: Boolean = true

    @Volatile var latestSnapshot: RenderSnapshot = RenderSnapshot()
}
