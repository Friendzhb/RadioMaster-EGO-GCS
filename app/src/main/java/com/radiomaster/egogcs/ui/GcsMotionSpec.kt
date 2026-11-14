package com.radiomaster.egogcs.ui

import android.animation.TimeInterpolator
import android.animation.ValueAnimator
import android.os.Build
import android.util.SparseArray
import android.view.HapticFeedbackConstants
import android.view.View
import android.view.animation.OvershootInterpolator
import android.view.animation.PathInterpolator
import androidx.dynamicanimation.animation.DynamicAnimation
import androidx.dynamicanimation.animation.SpringAnimation
import androidx.dynamicanimation.animation.SpringForce
import java.util.WeakHashMap

/**
 * 智航 (RadioMaster EGO GCS) 全局统一动效设计系统 Token 与物理弹簧注册中心：
 *
 * 1. 所有时长、缓动曲线、物理弹簧刚度/阻尼、按压形变角色、触觉反馈分级统一在此声明，业务代码严禁散落硬编码魔法值；
 * 2. 支持三档全局动效策略（标准物理 / 敏捷利落 / 极简省电），并与渲染功耗模式 (`perfMode == 2`) 及系统无障碍动画开关自动协同；
 * 3. 基于 `(View, ViewProperty)` 弱引用缓存池复用 `SpringAnimation`，保证高频连点时继承瞬时速度 (`animateToFinalPosition`)，零对象竞争、零内存泄漏。
 */
object GcsMotionSpec {

    const val PROFILE_STANDARD = 0 // 标准物理：60FPS 自然阻尼弹簧 + 层级微错峰
    const val PROFILE_SNAPPY = 1 // 敏捷利落：户外高频操控，高刚度快收敛
    const val PROFILE_MINIMAL = 2 // 极简省电：临界阻尼瞬态到位，关闭级联瀑布与微脉冲

    val SHORT_PROFILE_NAMES = arrayOf("标准物理", "敏捷利落", "极简省电")
    val PROFILE_TIPS = arrayOf(
        "60FPS 自然物理弹簧 + 空间连续过渡",
        "高刚度紧凑响应 · 适合户外高频盲操",
        "极简瞬态过渡 · 降低 GPU/CPU 合成功耗"
    )

    /**
     * 标准时长阶梯 Token（基准档位 [PROFILE_STANDARD] 下的毫秒数）
     */
    enum class DurationToken(val baseMs: Long, val minFloorMs: Long) {
        INSTANT(65L, 35L), // 瞬态保护与微状态切变
        MICRO(125L, 45L), // 胶囊芯片显隐、HUD 防碰撞避让淡出
        FAST(160L, 55L), // 浮窗/模态框收起、手风琴折叠、避让恢复
        STANDARD(195L, 70L), // 抽屉透明度、浮窗展开、页签/分类方向切换、Toast 入场
        SPATIAL(255L, 85L), // 三维姿态视窗环绕、指点光柱弹落、HSI 量程缩放
        EMPHASIZED(340L, 110L) // 3D/2D 投影平滑变形、HSI 机头/北向旋转融合
    }

    /**
     * 物理弹簧分级 Token（基准刚度与阻尼比）
     */
    enum class SpringToken(val stiffness: Float, val dampingRatio: Float) {
        PRESS_PILL(660f, 0.72f), // 标准胶囊/功能按钮按压与回弹
        PRESS_CHIP(720f, 0.76f), // 分段选择器、图层小芯片、步进微调键
        PRESS_ICON(740f, 0.68f), // 紧凑图标按钮（齿轮、关闭、排序、缩放）
        PRESS_INSTRUMENT(560f, 0.80f), // 大尺寸仪表盘（HSI、三维姿态窗、遥测框、列表行）
        PRESS_SAFETY(800f, 0.64f), // 安全关键大按钮（紧急悬停、返航、清图、重置）
        DETENT_POP(620f, 0.66f), // 状态翻转/档位锁入微弹确认
        WARNING_ARM(540f, 0.56f), // 二次确认待命态警示弹跳
        SWITCH_THUMB(580f, 0.76f), // 拨动开关滑块水平位移与流体拉伸
        GLIDER_TAB(500f, 0.82f), // 抽屉顶栏与左侧分类导航高亮滑块
        SPATIAL_DOCK(460f, 0.82f), // 底部上下文指令坞弹出与高度微调反馈
        SPATIAL_MODAL(450f, 0.82f), // 居中模态设置面板缩放展开
        SPATIAL_DRAWER(420f, 0.85f), // 侧边抽屉滑入与顶部罗盘堆栈防碰撞平移
        GEAR_ROTATE(360f, 0.76f) // 顶栏设置齿轮旋转啮合
    }

    /**
     * 交互控件按压角色：按控件视觉面积与安全等级匹配最佳缩放幅度、弹簧参数与触觉反馈
     */
    enum class PressRole(
        val pressedScale: Float,
        val minimalScale: Float,
        val springToken: SpringToken,
        val downHaptic: HapticRole
    ) {
        PILL_BUTTON(0.94f, 0.97f, SpringToken.PRESS_PILL, HapticRole.DETENT_SELECT),
        SEGMENT_CHIP(0.95f, 0.975f, SpringToken.PRESS_CHIP, HapticRole.TICK_STEP),
        ICON_CONTROL(0.89f, 0.95f, SpringToken.PRESS_ICON, HapticRole.TAP_LIGHT),
        HUD_INSTRUMENT(0.968f, 0.985f, SpringToken.PRESS_INSTRUMENT, HapticRole.TAP_LIGHT),
        CRITICAL_SAFETY(0.91f, 0.955f, SpringToken.PRESS_SAFETY, HapticRole.WARNING_ARM)
    }

    /**
     * 航空级语义化触觉反馈分级（兼容 Android API 26+ RadioMaster AX12）
     */
    enum class HapticRole {
        TAP_LIGHT, // 常规轻触
        TICK_STEP, // 滑块步进、高度 +/- 微调、分段切换刻度感
        DETENT_SELECT, // 模式切换、胶囊状态锁入
        WARNING_ARM, // 返航/清图/恢复出厂首次点击待命预警
        CRITICAL_COMMIT, // 紧急悬停、确认返航、立即起飞、启动航线
        REJECT_BLOCKED // 防误触锁拦截、禁飞/障碍内选点拒绝
    }

    // 惰性初始化插值器，确保纯 JVM 单元测试加载本类时不触发 Android Framework 构造异常
    val EASE_SPATIAL_OUT: TimeInterpolator by lazy(LazyThreadSafetyMode.PUBLICATION) {
        PathInterpolator(0.22f, 1.0f, 0.36f, 1.0f)
    }
    val EASE_EXIT_IN: TimeInterpolator by lazy(LazyThreadSafetyMode.PUBLICATION) {
        PathInterpolator(0.40f, 0.0f, 1.0f, 1.0f)
    }
    val EASE_STANDARD: TimeInterpolator by lazy(LazyThreadSafetyMode.PUBLICATION) {
        PathInterpolator(0.20f, 0.0f, 0.0f, 1.0f)
    }
    val EASE_PIN_DROP: TimeInterpolator by lazy(LazyThreadSafetyMode.PUBLICATION) {
        OvershootInterpolator(1.42f)
    }

    private const val PROP_SCALE_X = 1
    private const val PROP_SCALE_Y = 2
    private const val PROP_TRANS_X = 3
    private const val PROP_TRANS_Y = 4
    private const val PROP_ROTATION = 5
    private const val PROP_ALPHA = 6

    private val springCache = WeakHashMap<View, SparseArray<SpringAnimation>>()

    @Volatile var userAnimStyle: Int = PROFILE_STANDARD

    @Volatile var linkedPerfMode: Int = 0

    fun isSystemAnimatorEnabled(): Boolean = try {
        ValueAnimator.areAnimatorsEnabled()
    } catch (_: Throwable) {
        true
    }

    /**
     * 计算当前生效的动效档位：
     * - 系统无障碍关闭动画时强制降级为 [PROFILE_MINIMAL]；
     * - 渲染处于「省电 30帧」(`perfMode == 2`) 且用户未手动强制指定敏捷档时，自动联动为 [PROFILE_MINIMAL]。
     */
    fun resolveEffectiveProfile(
        userStyle: Int = userAnimStyle,
        perfMode: Int = linkedPerfMode,
        systemAnimEnabled: Boolean = isSystemAnimatorEnabled()
    ): Int {
        if (!systemAnimEnabled) return PROFILE_MINIMAL
        val clamped = userStyle.coerceIn(PROFILE_STANDARD, PROFILE_MINIMAL)
        if (clamped == PROFILE_MINIMAL) return PROFILE_MINIMAL
        if (perfMode == 2 && clamped == PROFILE_STANDARD) return PROFILE_MINIMAL
        return clamped
    }

    fun effectiveProfile(): Int = resolveEffectiveProfile()

    fun isMinimalMotion(): Boolean = effectiveProfile() == PROFILE_MINIMAL

    fun scaleDurationMs(
        token: DurationToken,
        profile: Int = effectiveProfile()
    ): Long = scaleRawDurationMs(token.baseMs, token.minFloorMs, profile)

    fun scaleRawDurationMs(
        baseMs: Long,
        minFloorMs: Long = 45L,
        profile: Int = effectiveProfile()
    ): Long = when (profile) {
        PROFILE_SNAPPY -> (baseMs * 0.68f).toLong().coerceAtLeast(minFloorMs)
        PROFILE_MINIMAL -> (baseMs * 0.34f).toLong().coerceAtLeast(minFloorMs)
        else -> baseMs.coerceAtLeast(minFloorMs)
    }

    fun scaleSpringStiffness(
        token: SpringToken,
        profile: Int = effectiveProfile()
    ): Float = when (profile) {
        PROFILE_SNAPPY -> token.stiffness * 1.45f
        PROFILE_MINIMAL -> token.stiffness * 2.45f
        else -> token.stiffness
    }

    fun scaleSpringDamping(
        token: SpringToken,
        profile: Int = effectiveProfile()
    ): Float = when (profile) {
        PROFILE_SNAPPY -> (token.dampingRatio + 0.08f).coerceAtMost(0.96f)
        PROFILE_MINIMAL -> 0.98f
        else -> token.dampingRatio
    }

    fun staggerStepMs(profile: Int = effectiveProfile()): Long = when (profile) {
        PROFILE_SNAPPY -> 12L
        PROFILE_MINIMAL -> 0L
        else -> 18L
    }

    fun cameraDampingForProfile(profile: Int = effectiveProfile()): Float = when (profile) {
        PROFILE_SNAPPY -> 1.32f
        PROFILE_MINIMAL -> 1.65f
        else -> 1.0f
    }

    fun resolveHapticConstant(role: HapticRole): Int = when (role) {
        HapticRole.TAP_LIGHT -> HapticFeedbackConstants.KEYBOARD_TAP
        HapticRole.TICK_STEP -> HapticFeedbackConstants.CLOCK_TICK
        HapticRole.DETENT_SELECT -> HapticFeedbackConstants.VIRTUAL_KEY
        HapticRole.WARNING_ARM -> HapticFeedbackConstants.CONTEXT_CLICK
        HapticRole.CRITICAL_COMMIT -> HapticFeedbackConstants.LONG_PRESS
        HapticRole.REJECT_BLOCKED -> if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            HapticFeedbackConstants.REJECT
        } else {
            HapticFeedbackConstants.LONG_PRESS
        }
    }

    private fun propertyKey(property: DynamicAnimation.ViewProperty): Int = when (property) {
        DynamicAnimation.SCALE_X -> PROP_SCALE_X
        DynamicAnimation.SCALE_Y -> PROP_SCALE_Y
        DynamicAnimation.TRANSLATION_X -> PROP_TRANS_X
        DynamicAnimation.TRANSLATION_Y -> PROP_TRANS_Y
        DynamicAnimation.ROTATION -> PROP_ROTATION
        else -> PROP_ALPHA
    }

    /**
     * 获取或创建绑定于指定 `(View, ViewProperty)` 的单例物理弹簧，更新最新刚度/阻尼并平滑重定向至目标位置。
     * 高频连点或中途反向操作时自动继承当前运动速度，彻底消除多实例抢占抖动。
     */
    fun springTo(
        view: View,
        property: DynamicAnimation.ViewProperty,
        finalPosition: Float,
        token: SpringToken,
        startVelocity: Float? = null,
        onEnd: (() -> Unit)? = null
    ): SpringAnimation {
        val key = propertyKey(property)
        val map = springCache.getOrPut(view) { SparseArray(4) }
        val profile = effectiveProfile()
        val anim = map.get(key) ?: SpringAnimation(view, property, finalPosition).also { created ->
            created.spring = SpringForce(finalPosition)
            map.put(key, created)
        }
        val force = anim.spring ?: SpringForce(finalPosition).also { anim.spring = it }
        force.stiffness = scaleSpringStiffness(token, profile)
        force.dampingRatio = scaleSpringDamping(token, profile)
        if (startVelocity != null) {
            anim.setStartVelocity(startVelocity)
        }
        if (onEnd != null) {
            val listener = object : DynamicAnimation.OnAnimationEndListener {
                override fun onAnimationEnd(
                    animation: DynamicAnimation<*>?,
                    canceled: Boolean,
                    value: Float,
                    velocity: Float
                ) {
                    anim.removeEndListener(this)
                    if (!canceled) onEnd()
                }
            }
            anim.addEndListener(listener)
        }
        anim.animateToFinalPosition(finalPosition)
        return anim
    }

    fun cancelSprings(view: View) {
        val map = springCache[view] ?: return
        for (i in 0 until map.size()) {
            map.valueAt(i)?.cancel()
        }
    }
}
