package com.radiomaster.egogcs.ui

import com.radiomaster.egogcs.model.GcsSharedHub
import kotlin.math.min

/**
 * OpenGL 动态热负载帧率节流控制器 ([ThermalFrameGovernor])：
 *
 * 面向 RadioMaster AX12 (60Hz 显示面板 + 嵌入式多核 SoC + 内置 ELRS 射频模组共壳散热) 设计五档无抖动刷新率阶梯：
 * - [Level.NOMINAL]  (0): **60 FPS** (16.67 ms) / HUD 10 Hz —— 芯片清凉且帧耗时充裕；
 * - [Level.WARM]     (1): **45 FPS** (22.22 ms) / HUD 10 Hz —— 轻度温升，降低 25% 顶点构建与缓冲交换功耗；
 * - [Level.MODERATE] (2): **30 FPS** (33.33 ms, 60Hz 1/2 整数分频) / HUD 8 Hz —— 中度热负载或内核开始 DVFS 限频；
 * - [Level.SEVERE]   (3): **20 FPS** (50.00 ms, 60Hz 1/3 整数分频) / HUD 7 Hz —— 高温告警，削减 67% 渲染负载保障遥控链路；
 * - [Level.CRITICAL] (4): **15 FPS** (66.67 ms, 60Hz 1/4 整数分频) / HUD 5 Hz —— 极限过热保护，保留流畅安全避障态势且杜绝死机降频卡顿。
 *
 * 稳定性保障机制：
 * 1. **非对称施密特迟滞死区 (Asymmetric Schmitt Hysteresis)**：每个热源与帧耗时指标均设独立的「触发阈值」与「解除保持阈值」，彻底消除临界温度附近的帧率反复横跳；
 * 2. **分级驻留时间窗 ( Asymmetric Dwell Timers)**：`CRITICAL` 0ms 瞬时介入保护，`SEVERE` 0.5s 快速降档，常规温升 1.5s 滤波降档；降温恢复强制 4.5s 持续冷却且逐级单步回升 (`15 -> 20 -> 30 -> 45 -> 60`)；
 * 3. **安全触控交互瞬态提速 (Safe Interactive Touch Boost)**：仅在 `WARM` / `MODERATE` 轻中度温控下，当飞手手指正在拖拽三维航点或环视地图时临时放宽一档刷新率，松手 400ms 后平滑回落；在 `SEVERE` / `CRITICAL` 高温区严禁提速以确保硬件绝对稳定。
 */
class ThermalFrameGovernor(
    private val stepDownDwellNs: Long = DEFAULT_STEP_DOWN_DWELL_NS,
    private val severeStepDownDwellNs: Long = DEFAULT_SEVERE_STEP_DOWN_DWELL_NS,
    private val stepUpRecoveryDwellNs: Long = DEFAULT_STEP_UP_RECOVERY_DWELL_NS,
    private val touchBoostWindowNs: Long = DEFAULT_TOUCH_BOOST_WINDOW_NS
) {

    enum class Level(
        val code: Int,
        val targetFps: Int,
        val hudHz: Int
    ) {
        NOMINAL(0, 60, 10),
        WARM(1, 45, 10),
        MODERATE(2, 30, 8),
        SEVERE(3, 20, 7),
        CRITICAL(4, 15, 5);

        companion object {
            private val VALUES = entries.toTypedArray()

            fun fromCode(code: Int): Level = VALUES[code.coerceIn(0, VALUES.lastIndex)]
        }
    }

    @Volatile var currentLevel: Level = Level.NOMINAL
        private set

    @Volatile var currentTargetFps: Int = Level.NOMINAL.targetFps
        private set

    @Volatile var currentTargetHudHz: Int = Level.NOMINAL.hudHz
        private set

    @Volatile var dominantSource: String = "IDLE"
        private set

    @Volatile var emaFrameCpuMs: Float = 2.0f
        private set

    @Volatile private var warmupFramesRemaining: Int = WARMUP_FRAMES

    @Volatile private var lastTouchInteractionNs: Long = Long.MIN_VALUE

    private var pendingTargetCode: Int = 0
    private var pendingSinceNs: Long = 0L
    private var initializedClock: Boolean = false

    fun reset() {
        currentLevel = Level.NOMINAL
        currentTargetFps = Level.NOMINAL.targetFps
        currentTargetHudHz = Level.NOMINAL.hudHz
        dominantSource = "IDLE"
        emaFrameCpuMs = 2.0f
        warmupFramesRemaining = WARMUP_FRAMES
        lastTouchInteractionNs = Long.MIN_VALUE
        pendingTargetCode = 0
        pendingSinceNs = 0L
        initializedClock = false
    }

    /**
     * 在 GL Surface 恢复或新建后跳过冷启动着色器编译与首轮 VBO 分配帧，防止瞬态初始化耗时误触发热节流。
     */
    fun notifyRendererResumed() {
        warmupFramesRemaining = WARMUP_FRAMES
        pendingTargetCode = currentLevel.code
        pendingSinceNs = 0L
    }

    fun notifyTouchInteraction(nowNs: Long) {
        lastTouchInteractionNs = nowNs
    }

    /**
     * 由 GL 渲染线程在每帧结束时上报本帧真实 CPU 几何构建与 GL 提交耗时（零对象分配）。
     */
    fun recordFrameWork(workNs: Long) {
        if (workNs <= 0L) return
        val left = warmupFramesRemaining
        if (left > 0) {
            warmupFramesRemaining = left - 1
            return
        }
        val clampedMs = (workNs * 1e-6f).coerceIn(0.1f, 80.0f)
        val prev = emaFrameCpuMs
        emaFrameCpuMs = prev + (clampedMs - prev) * EMA_ALPHA
    }

    /**
     * 在每次 Choreographer VSync 到来时评估融合热负载，返回当前应执行的动态目标帧率 (FPS)，
     * 并同步更新 [currentTargetHudHz] 与 [hub] 诊断字段（全程零堆内存分配）。
     */
    fun evaluate(nowNs: Long, hub: GcsSharedHub): Int {
        val prevCode = currentLevel.code
        val demandCode = computeFusedDemandCode(prevCode, hub)

        if (!initializedClock) {
            initializedClock = true
            pendingTargetCode = demandCode
            pendingSinceNs = nowNs
        }

        if (demandCode > prevCode) {
            // 需要进一步降温限帧（阶跃升高）
            val requiredDwellNs = when {
                demandCode >= Level.CRITICAL.code -> 0L
                demandCode >= Level.SEVERE.code -> severeStepDownDwellNs
                else -> stepDownDwellNs
            }
            if (pendingTargetCode <= prevCode) {
                pendingTargetCode = demandCode
                pendingSinceNs = nowNs
            } else if (demandCode > pendingTargetCode) {
                // 若等待期间出现更严峻的热等级，保留已累计时间并升级目标等级
                pendingTargetCode = demandCode
            }
            if (requiredDwellNs <= 0L || nowNs - pendingSinceNs >= requiredDwellNs) {
                currentLevel = Level.fromCode(pendingTargetCode)
                pendingTargetCode = currentLevel.code
                pendingSinceNs = nowNs
            }
        } else if (demandCode < prevCode) {
            // 热负载回落：必须持续稳定低于解除阈值满 stepUpRecoveryDwellNs 后，才允许单步回升 1 档
            val nextRecoverCode = prevCode - 1
            if (pendingTargetCode != nextRecoverCode) {
                pendingTargetCode = nextRecoverCode
                pendingSinceNs = nowNs
            }
            if (stepUpRecoveryDwellNs <= 0L || nowNs - pendingSinceNs >= stepUpRecoveryDwellNs) {
                currentLevel = Level.fromCode(nextRecoverCode)
                pendingTargetCode = if (demandCode < nextRecoverCode) nextRecoverCode - 1 else nextRecoverCode
                pendingSinceNs = nowNs
            }
        } else {
            pendingTargetCode = prevCode
            pendingSinceNs = nowNs
        }

        val activeLv = currentLevel
        val touchActive = lastTouchInteractionNs != Long.MIN_VALUE &&
            nowNs >= lastTouchInteractionNs &&
            (nowNs - lastTouchInteractionNs) <= touchBoostWindowNs
        val effectiveLv = if (touchActive && activeLv.code in Level.WARM.code..Level.MODERATE.code) {
            Level.fromCode(activeLv.code - 1)
        } else {
            activeLv
        }

        val perfCeilingFps = if (hub.perfMode == 2) 30 else 60
        val resolvedFps = min(perfCeilingFps, effectiveLv.targetFps)
        val resolvedHudHz = if (hub.perfMode == 2) {
            min(7, effectiveLv.hudHz)
        } else {
            effectiveLv.hudHz
        }

        currentTargetFps = resolvedFps
        currentTargetHudHz = resolvedHudHz
        hub.thermalThrottleLevel = activeLv.code
        hub.targetRenderFps = resolvedFps
        hub.thermalSourceTag = dominantSource
        return resolvedFps
    }

    private fun computeFusedDemandCode(prevCode: Int, hub: GcsSharedHub): Int {
        val autoEnabled = hub.thermalAutoThrottleEnabled
        val socLevel = classifySocTemp(hub.cpuTempCelsius, prevCode, hub.perfMode)
        val osStatusLevel = classifyOsThermalStatus(hub.osThermalStatus)

        if (!autoEnabled) {
            // 关闭自动温控节流时，仅保留 85°C / OS_CRITICAL 硬件过热熔断保护底线
            return if (socLevel >= Level.CRITICAL.code || osStatusLevel >= Level.CRITICAL.code) {
                dominantSource = if (socLevel >= osStatusLevel) "SOC" else "OS"
                Level.CRITICAL.code
            } else {
                dominantSource = "OFF"
                Level.NOMINAL.code
            }
        }

        val headroomLevel = classifyThermalHeadroom(hub.thermalHeadroom, prevCode)
        val dvfsLevel = classifyDvfsClamp(hub.cpuFreqCapRatio, prevCode)
        val chassisLevel = classifyChassisTemp(hub.txChassisTempCelsius, prevCode)
        val renderLoadLevel = if (warmupFramesRemaining <= 0) {
            classifyCpuFrameLoad(emaFrameCpuMs, prevCode)
        } else {
            0
        }

        var maxCode = 0
        var src = "IDLE"

        if (socLevel > maxCode) {
            maxCode = socLevel
            src = "SOC"
        }
        if (osStatusLevel > maxCode) {
            maxCode = osStatusLevel
            src = "OS"
        }
        if (headroomLevel > maxCode) {
            maxCode = headroomLevel
            src = "OS"
        }
        if (dvfsLevel > maxCode) {
            maxCode = dvfsLevel
            src = "DVFS"
        }
        if (chassisLevel > maxCode) {
            maxCode = chassisLevel
            src = "BAT"
        }
        if (renderLoadLevel > maxCode) {
            maxCode = renderLoadLevel
            src = "LOAD"
        }

        if (maxCode == 0) {
            src = when {
                hub.cpuTempCelsius.isFinite() -> "SOC"
                hub.osThermalStatus > 0 || hub.thermalHeadroom.isFinite() -> "OS"
                hub.txChassisTempCelsius.isFinite() -> "BAT"
                else -> "LOAD"
            }
        }
        dominantSource = src
        return maxCode
    }

    private fun classifySocTemp(tempC: Float, prevCode: Int, perfMode: Int): Int {
        if (!tempC.isFinite()) return 0
        val warmEnter = if (perfMode == 1) 65.0f else 62.0f
        val warmHold = if (perfMode == 1) 61.0f else 58.5f
        return when {
            tempC >= if (prevCode >= 4) 80.5f else 85.0f -> 4
            tempC >= if (prevCode >= 3) 74.0f else 78.0f -> 3
            tempC >= if (prevCode >= 2) 66.0f else 70.0f -> 2
            tempC >= if (prevCode >= 1) warmHold else warmEnter -> 1
            else -> 0
        }
    }

    private fun classifyOsThermalStatus(osStatus: Int): Int = when {
        osStatus >= 4 -> 4 // THERMAL_STATUS_CRITICAL / EMERGENCY / SHUTDOWN
        osStatus == 3 -> 3 // THERMAL_STATUS_SEVERE
        osStatus == 2 -> 2 // THERMAL_STATUS_MODERATE
        osStatus == 1 -> 1 // THERMAL_STATUS_LIGHT
        else -> 0
    }

    private fun classifyThermalHeadroom(headroom: Float, prevCode: Int): Int {
        if (!headroom.isFinite() || headroom <= 0f) return 0
        return when {
            headroom >= if (prevCode >= 4) 1.08f else 1.18f -> 4
            headroom >= if (prevCode >= 3) 0.96f else 1.04f -> 3
            headroom >= if (prevCode >= 2) 0.84f else 0.92f -> 2
            headroom >= if (prevCode >= 1) 0.70f else 0.78f -> 1
            else -> 0
        }
    }

    private fun classifyDvfsClamp(freqCapRatio: Float, prevCode: Int): Int {
        if (!freqCapRatio.isFinite() || freqCapRatio <= 0f || freqCapRatio >= 0.98f) return 0
        return when {
            freqCapRatio <= if (prevCode >= 3) 0.74f else 0.66f -> 3
            freqCapRatio <= if (prevCode >= 2) 0.86f else 0.80f -> 2
            freqCapRatio <= if (prevCode >= 1) 0.94f else 0.90f -> 1
            else -> 0
        }
    }

    private fun classifyChassisTemp(chassisC: Float, prevCode: Int): Int {
        if (!chassisC.isFinite()) return 0
        return when {
            chassisC >= if (prevCode >= 4) 48.5f else 50.5f -> 4
            chassisC >= if (prevCode >= 3) 45.5f else 47.5f -> 3
            chassisC >= if (prevCode >= 2) 42.5f else 44.5f -> 2
            chassisC >= if (prevCode >= 1) 39.5f else 41.5f -> 1
            else -> 0
        }
    }

    /**
     * 基于单帧 CPU 耗时 EMA 的闭环预算评估：
     * 相邻档位的「解除阈值 (hold)」严格高于低一档的「触发阈值 (enter)」
     * (`9.2 < 12.2 < 12.8 < 16.8 < 19.0 < 25.5 < 28.5 < 38.0 ms`)，
     * 数学上保证在任意恒定场景复杂度下绝不发生跨档位乒乓震荡。
     */
    private fun classifyCpuFrameLoad(emaMs: Float, prevCode: Int): Int {
        if (!emaMs.isFinite()) return 0
        return when {
            emaMs >= if (prevCode >= 4) 28.5f else 38.0f -> 4
            emaMs >= if (prevCode >= 3) 19.0f else 25.5f -> 3
            emaMs >= if (prevCode >= 2) 12.8f else 16.8f -> 2
            emaMs >= if (prevCode >= 1) 9.2f else 12.2f -> 1
            else -> 0
        }
    }

    companion object {
        const val WARMUP_FRAMES = 18
        private const val EMA_ALPHA = 0.18f
        const val DEFAULT_STEP_DOWN_DWELL_NS = 1_500_000_000L
        const val DEFAULT_SEVERE_STEP_DOWN_DWELL_NS = 500_000_000L
        const val DEFAULT_STEP_UP_RECOVERY_DWELL_NS = 4_500_000_000L
        const val DEFAULT_TOUCH_BOOST_WINDOW_NS = 400_000_000L
    }
}
