package com.radiomaster.egogcs.ui

import android.animation.ValueAnimator
import android.annotation.SuppressLint
import android.view.HapticFeedbackConstants
import android.view.MotionEvent
import android.view.View
import android.view.ViewGroup
import android.view.animation.CycleInterpolator
import android.widget.FrameLayout
import android.widget.ScrollView
import android.widget.TextView
import androidx.dynamicanimation.animation.DynamicAnimation
import com.radiomaster.egogcs.ui.GcsMotionSpec.DurationToken
import com.radiomaster.egogcs.ui.GcsMotionSpec.HapticRole
import com.radiomaster.egogcs.ui.GcsMotionSpec.PressRole
import com.radiomaster.egogcs.ui.GcsMotionSpec.SpringToken
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sign

/**
 * 商业航空级 60FPS 物理弹簧与空间因果微交互编排引擎 (`AnimHelper`)：
 * - 所有时间曲线、弹簧刚度、触觉分级统一受控于 [GcsMotionSpec]；
 * - 覆盖按压回弹、开关流体滑块、分段锁入、安全二次确认、禁飞/防误触拦截抖动、方向感知翻页、抽屉宽屏平滑伸缩等全场景交互。
 */
object AnimHelper {

    val EASE_OUT_EXPO get() = GcsMotionSpec.EASE_SPATIAL_OUT
    val EASE_EXIT_IN get() = GcsMotionSpec.EASE_EXIT_IN
    val EASE_IN_OUT_CUBIC get() = GcsMotionSpec.EASE_STANDARD

    private var toastHideRunnable: Runnable? = null
    private var dockVisibleLogical: Boolean = false
    private var drawerWidthAnimator: ValueAnimator? = null
    private var lastSliderTickMs: Long = 0L

    @Volatile var hapticsEnabled: Boolean = true

    var animStyle: Int
        get() = GcsMotionSpec.userAnimStyle
        set(value) {
            GcsMotionSpec.userAnimStyle = value.coerceIn(
                GcsMotionSpec.PROFILE_STANDARD,
                GcsMotionSpec.PROFILE_MINIMAL
            )
        }

    fun syncMotionContext(userStyle: Int, perfMode: Int): Float {
        GcsMotionSpec.userAnimStyle = userStyle.coerceIn(
            GcsMotionSpec.PROFILE_STANDARD,
            GcsMotionSpec.PROFILE_MINIMAL
        )
        GcsMotionSpec.linkedPerfMode = perfMode.coerceIn(0, 2)
        return GcsMotionSpec.cameraDampingForProfile()
    }

    fun dur(token: DurationToken): Long = GcsMotionSpec.scaleDurationMs(token)

    fun triggerHaptic(view: View, feedbackConstant: Int = HapticFeedbackConstants.KEYBOARD_TAP) {
        if (hapticsEnabled) {
            view.performHapticFeedback(feedbackConstant)
        }
    }

    fun triggerRoleHaptic(view: View, role: HapticRole) {
        if (hapticsEnabled) {
            view.performHapticFeedback(GcsMotionSpec.resolveHapticConstant(role))
        }
    }

    fun triggerSliderStepTick(view: View) {
        if (!hapticsEnabled) return
        val now = android.os.SystemClock.uptimeMillis()
        if (now - lastSliderTickMs >= 42L) {
            lastSliderTickMs = now
            view.performHapticFeedback(GcsMotionSpec.resolveHapticConstant(HapticRole.TICK_STEP))
        }
    }

    /**
     * 按控件角色绑定零泄漏物理按压弹簧，连点或滑出边界时保持速度连续回弹。
     */
    @SuppressLint("ClickableViewAccessibility")
    fun attachSpringTouch(view: View, role: PressRole = PressRole.PILL_BUTTON) {
        view.setOnTouchListener { v, event ->
            if (!v.isEnabled) return@setOnTouchListener false
            when (event.actionMasked) {
                MotionEvent.ACTION_DOWN -> {
                    triggerRoleHaptic(v, role.downHaptic)
                    val targetScale = if (GcsMotionSpec.isMinimalMotion()) {
                        role.minimalScale
                    } else {
                        role.pressedScale
                    }
                    GcsMotionSpec.springTo(v, DynamicAnimation.SCALE_X, targetScale, role.springToken)
                    GcsMotionSpec.springTo(v, DynamicAnimation.SCALE_Y, targetScale, role.springToken)
                }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    GcsMotionSpec.springTo(v, DynamicAnimation.SCALE_X, 1.0f, role.springToken)
                    GcsMotionSpec.springTo(v, DynamicAnimation.SCALE_Y, 1.0f, role.springToken)
                }
            }
            false
        }
    }

    /**
     * 分段选择器、胶囊芯片或模式开关状态切换时的档位锁入微弹反馈（Detent Pop）。
     */
    fun animateStateFlip(view: View) {
        if (GcsMotionSpec.isMinimalMotion()) {
            view.scaleX = 1f
            view.scaleY = 1f
            return
        }
        view.scaleX = 0.94f
        view.scaleY = 0.94f
        GcsMotionSpec.springTo(view, DynamicAnimation.SCALE_X, 1f, SpringToken.DETENT_POP)
        GcsMotionSpec.springTo(view, DynamicAnimation.SCALE_Y, 1f, SpringToken.DETENT_POP)
    }

    /**
     * 统一拨动开关滑块物理位移 + 运动瞬态流体微拉伸动效。
     */
    fun animateSwitchThumb(thumb: View, checked: Boolean, travelPx: Float, animate: Boolean) {
        val targetTx = if (checked) travelPx else 0f
        if (!animate) {
            GcsMotionSpec.cancelSprings(thumb)
            thumb.translationX = targetTx
            thumb.scaleX = 1f
            thumb.scaleY = 1f
            return
        }
        if (!GcsMotionSpec.isMinimalMotion()) {
            thumb.scaleX = 1.16f
            thumb.scaleY = 0.92f
            GcsMotionSpec.springTo(thumb, DynamicAnimation.SCALE_X, 1f, SpringToken.SWITCH_THUMB)
            GcsMotionSpec.springTo(thumb, DynamicAnimation.SCALE_Y, 1f, SpringToken.SWITCH_THUMB)
        }
        GcsMotionSpec.springTo(thumb, DynamicAnimation.TRANSLATION_X, targetTx, SpringToken.SWITCH_THUMB)
    }

    /**
     * 滑块（SeekBar）拖拽起止时右侧数值胶囊的抬升聚焦与归位反馈。
     */
    fun animateSliderBadgeFocus(badgeView: View, tracking: Boolean) {
        if (GcsMotionSpec.isMinimalMotion()) {
            badgeView.scaleX = 1f
            badgeView.scaleY = 1f
            return
        }
        val target = if (tracking) 1.08f else 1.0f
        GcsMotionSpec.springTo(badgeView, DynamicAnimation.SCALE_X, target, SpringToken.PRESS_CHIP)
        GcsMotionSpec.springTo(badgeView, DynamicAnimation.SCALE_Y, target, SpringToken.PRESS_CHIP)
    }

    /**
     * 安全关键二次确认（返航 / 清空建图 / 恢复出厂）待命态警示脉冲与复位。
     */
    fun animateArmedConfirm(view: View, armed: Boolean) {
        if (armed) {
            triggerRoleHaptic(view, HapticRole.WARNING_ARM)
            if (!GcsMotionSpec.isMinimalMotion()) {
                view.scaleX = 1.06f
                view.scaleY = 1.06f
                GcsMotionSpec.springTo(view, DynamicAnimation.SCALE_X, 1.0f, SpringToken.WARNING_ARM)
                GcsMotionSpec.springTo(view, DynamicAnimation.SCALE_Y, 1.0f, SpringToken.WARNING_ARM)
            }
        } else {
            GcsMotionSpec.springTo(view, DynamicAnimation.SCALE_X, 1.0f, SpringToken.PRESS_PILL)
            GcsMotionSpec.springTo(view, DynamicAnimation.SCALE_Y, 1.0f, SpringToken.PRESS_PILL)
        }
    }

    /**
     * 紧急悬停（急停制动）触发时的强阻尼制动压缩与回弹确认。
     */
    fun animateEmergencyBrake(brakeView: View) {
        triggerRoleHaptic(brakeView, HapticRole.CRITICAL_COMMIT)
        brakeView.scaleX = 0.88f
        brakeView.scaleY = 0.88f
        GcsMotionSpec.springTo(
            brakeView,
            DynamicAnimation.SCALE_X,
            1.0f,
            SpringToken.PRESS_SAFETY,
            startVelocity = 3.2f
        )
        GcsMotionSpec.springTo(
            brakeView,
            DynamicAnimation.SCALE_Y,
            1.0f,
            SpringToken.PRESS_SAFETY,
            startVelocity = 3.2f
        )
    }

    /**
     * 操作被安全拦截（如防误触锁开启、目标处于障碍物内部、空航线启动）时的水平阻尼拒绝抖动。
     */
    fun animateRejectShake(view: View, density: Float) {
        triggerRoleHaptic(view, HapticRole.REJECT_BLOCKED)
        view.animate().cancel()
        if (GcsMotionSpec.isMinimalMotion()) {
            pulseViewSubtle(view)
            return
        }
        val amp = 5.5f * density
        ValueAnimator.ofFloat(0f, 1f).apply {
            duration = GcsMotionSpec.scaleRawDurationMs(260L, 120L)
            interpolator = CycleInterpolator(2.0f)
            addUpdateListener { a ->
                val f = a.animatedFraction
                val decay = 1f - f
                view.translationX = (a.animatedValue as Float) * amp * decay
            }
            addListener(object : android.animation.AnimatorListenerAdapter() {
                override fun onAnimationEnd(animation: android.animation.Animator) {
                    view.translationX = 0f
                }

                override fun onAnimationCancel(animation: android.animation.Animator) {
                    view.translationX = 0f
                }
            })
            start()
        }
    }

    /**
     * 高度步进微调（+/- 或遥控滚轮方向键）时高度读数的方向性微跳反馈。
     */
    fun animateValueStepNudge(valueView: View, directionSign: Float, density: Float) {
        if (GcsMotionSpec.isMinimalMotion()) return
        val dir = if (directionSign >= 0f) -1f else 1f
        valueView.translationY = dir * 3.0f * density
        GcsMotionSpec.springTo(valueView, DynamicAnimation.TRANSLATION_Y, 0f, SpringToken.DETENT_POP)
    }

    /**
     * 顶部悬浮状态胶囊（漫游回中提示、视线遮挡透视徽章、多机战术栏）的平滑入场与加速离场。
     */
    fun animateFloatingBadge(view: View, show: Boolean, density: Float, slideDp: Float = -5f) {
        val targetTag = if (show) "BADGE_SHOW" else "BADGE_HIDE"
        if (view.tag == targetTag && ((show && view.visibility == View.VISIBLE) || (!show && view.visibility == View.GONE))) {
            return
        }
        view.tag = targetTag
        view.animate().cancel()
        val offsetPx = slideDp * density
        if (show) {
            if (view.visibility != View.VISIBLE) {
                view.visibility = View.VISIBLE
                view.alpha = 0f
                view.translationY = offsetPx
                view.scaleX = 0.94f
                view.scaleY = 0.94f
            }
            view.animate()
                .alpha(1f)
                .translationY(0f)
                .scaleX(1f)
                .scaleY(1f)
                .setDuration(dur(DurationToken.FAST))
                .setInterpolator(EASE_OUT_EXPO)
                .start()
        } else if (view.visibility == View.VISIBLE) {
            view.animate()
                .alpha(0f)
                .translationY(offsetPx * 0.75f)
                .scaleX(0.95f)
                .scaleY(0.95f)
                .setDuration(dur(DurationToken.MICRO))
                .setInterpolator(EASE_EXIT_IN)
                .withEndAction {
                    if (view.tag == "BADGE_HIDE") {
                        view.visibility = View.GONE
                        view.translationY = 0f
                        view.scaleX = 1f
                        view.scaleY = 1f
                    }
                }
                .start()
        } else {
            view.visibility = View.GONE
        }
    }

    /**
     * 折叠面板 / 条件输入区（高级话题手风琴、自定义源地址栏、多机构型子面板）平滑展开与收起。
     */
    fun animateCollapsibleSection(view: View, expand: Boolean, density: Float, staggerChildren: Boolean = false) {
        val targetTag = if (expand) "SEC_OPEN" else "SEC_CLOSED"
        if (view.tag == targetTag && ((expand && view.visibility == View.VISIBLE) || (!expand && view.visibility == View.GONE))) {
            return
        }
        view.tag = targetTag
        view.animate().cancel()
        if (expand) {
            view.visibility = View.VISIBLE
            view.alpha = 0f
            view.translationY = -4f * density
            view.animate()
                .alpha(1f)
                .translationY(0f)
                .setDuration(dur(DurationToken.FAST))
                .setInterpolator(EASE_OUT_EXPO)
                .start()
            if (staggerChildren) {
                animateStaggeredChildren(view, density)
            }
        } else if (view.visibility == View.VISIBLE) {
            view.animate()
                .alpha(0f)
                .translationY(-3f * density)
                .setDuration(dur(DurationToken.MICRO))
                .setInterpolator(EASE_EXIT_IN)
                .withEndAction {
                    if (view.tag == "SEC_CLOSED") {
                        view.visibility = View.GONE
                        view.alpha = 1f
                        view.translationY = 0f
                    }
                }
                .start()
        } else {
            view.visibility = View.GONE
        }
    }

    /**
     * 底部与侧翼 HUD 组件空间避让（抽屉/指令坞/浮窗防重叠平滑退让与归位）。
     */
    fun animateHudAvoidance(view: View, show: Boolean, hideOffsetX: Float = 0f, hideOffsetY: Float = 0f) {
        if (!show) {
            if (view.visibility == View.VISIBLE && view.tag != "HUD_AVOIDING") {
                view.tag = "HUD_AVOIDING"
                view.animate().cancel()
                view.animate()
                    .alpha(0f)
                    .translationX(hideOffsetX)
                    .translationY(hideOffsetY)
                    .setDuration(dur(DurationToken.MICRO))
                    .setInterpolator(EASE_EXIT_IN)
                    .withEndAction {
                        if (view.tag == "HUD_AVOIDING") {
                            view.visibility = View.INVISIBLE
                        }
                    }
                    .start()
            }
        } else {
            if (view.visibility != View.VISIBLE || view.alpha < 0.95f || view.tag == "HUD_AVOIDING") {
                view.tag = "HUD_SHOWN"
                view.animate().cancel()
                if (view.visibility != View.VISIBLE) {
                    view.visibility = View.VISIBLE
                }
                view.animate()
                    .alpha(1f)
                    .translationX(0f)
                    .translationY(0f)
                    .setDuration(dur(DurationToken.FAST))
                    .setInterpolator(EASE_OUT_EXPO)
                    .start()
            }
        }
    }

    fun animateDrawer(
        drawer: View,
        rightEdgeViews: List<View>,
        centerShiftViews: List<View>,
        open: Boolean,
        drawerWidthPx: Float,
        centerOffsetPx: Float,
        activePanelForCascade: View? = null,
        density: Float = 2f
    ) {
        drawerWidthAnimator?.cancel()
        if (open) drawer.visibility = View.VISIBLE
        drawer.setLayerType(View.LAYER_TYPE_HARDWARE, null)

        val finalTx = if (open) 0f else drawerWidthPx + 32f
        GcsMotionSpec.springTo(drawer, DynamicAnimation.TRANSLATION_X, finalTx, SpringToken.SPATIAL_DRAWER) {
            drawer.setLayerType(View.LAYER_TYPE_NONE, null)
            if (!open) drawer.visibility = View.INVISIBLE
        }

        drawer.animate().cancel()
        drawer.animate()
            .alpha(if (open) 1f else 0f)
            .setDuration(dur(if (open) DurationToken.STANDARD else DurationToken.FAST))
            .setInterpolator(if (open) EASE_OUT_EXPO else EASE_EXIT_IN)
            .start()

        if (open && activePanelForCascade != null && !GcsMotionSpec.isMinimalMotion()) {
            animateStaggeredChildren(activePanelForCascade, density)
        }

        for (rv in rightEdgeViews) {
            if (rv.tag == "HIDDEN_BY_PREF") {
                rv.visibility = View.GONE
                continue
            }
            rv.animate().cancel()
            if (open) {
                rv.animate()
                    .alpha(0f)
                    .translationX(14f * density * 0.5f)
                    .setDuration(dur(DurationToken.MICRO))
                    .setInterpolator(EASE_EXIT_IN)
                    .withEndAction { rv.visibility = View.INVISIBLE }
                    .start()
            } else {
                rv.visibility = View.VISIBLE
                rv.animate()
                    .alpha(1f)
                    .translationX(0f)
                    .setDuration(dur(DurationToken.STANDARD))
                    .setInterpolator(EASE_OUT_EXPO)
                    .start()
            }
        }

        val targetCenterTx = if (open) centerOffsetPx else 0f
        for (cv in centerShiftViews) {
            GcsMotionSpec.springTo(cv, DynamicAnimation.TRANSLATION_X, targetCenterTx, SpringToken.SPATIAL_DRAWER)
        }
    }

    /**
     * 抽屉内「标准 / 宽屏终端」切换时的平滑宽度插值与顶部堆栈防碰撞同步位移。
     */
    fun animateDrawerWidthMorph(
        drawer: View,
        targetWidthPx: Int,
        centerShiftViews: List<View>,
        targetCenterOffsetPx: Float,
        drawerOpen: Boolean,
        onFinished: () -> Unit
    ) {
        drawerWidthAnimator?.cancel()
        val lp = drawer.layoutParams as? FrameLayout.LayoutParams ?: return
        val startW = lp.width
        if (!drawerOpen || GcsMotionSpec.isMinimalMotion() || abs(startW - targetWidthPx) <= 4) {
            lp.width = targetWidthPx
            drawer.layoutParams = lp
            if (drawerOpen) {
                for (cv in centerShiftViews) {
                    GcsMotionSpec.springTo(cv, DynamicAnimation.TRANSLATION_X, targetCenterOffsetPx, SpringToken.SPATIAL_DRAWER)
                }
            }
            onFinished()
            return
        }

        for (cv in centerShiftViews) {
            GcsMotionSpec.springTo(cv, DynamicAnimation.TRANSLATION_X, targetCenterOffsetPx, SpringToken.SPATIAL_DRAWER)
        }
        drawerWidthAnimator = ValueAnimator.ofInt(startW, targetWidthPx).apply {
            duration = dur(DurationToken.STANDARD)
            interpolator = EASE_OUT_EXPO
            addUpdateListener { a ->
                lp.width = a.animatedValue as Int
                drawer.layoutParams = lp
            }
            addListener(object : android.animation.AnimatorListenerAdapter() {
                override fun onAnimationEnd(animation: android.animation.Animator) {
                    onFinished()
                }
            })
            start()
        }
    }

    fun animatePopover(popover: View, show: Boolean, density: Float) {
        popover.animate().cancel()
        popover.pivotX = popover.width.takeIf { it > 0 }?.toFloat() ?: (200f * density)
        popover.pivotY = 10f * density
        if (show) {
            popover.visibility = View.VISIBLE
            popover.alpha = 0f
            popover.scaleX = 0.90f
            popover.scaleY = 0.90f
            popover.translationX = 6f * density
            popover.animate()
                .alpha(1f)
                .scaleX(1f)
                .scaleY(1f)
                .translationX(0f)
                .setDuration(dur(DurationToken.STANDARD))
                .setInterpolator(EASE_OUT_EXPO)
                .start()
        } else {
            popover.animate()
                .alpha(0f)
                .scaleX(0.93f)
                .scaleY(0.93f)
                .translationX(5f * density)
                .setDuration(dur(DurationToken.FAST))
                .setInterpolator(EASE_EXIT_IN)
                .withEndAction { popover.visibility = View.GONE }
                .start()
        }
    }

    fun animateSettingsModal(scrimHost: View, dialogCard: View, show: Boolean, density: Float) {
        scrimHost.animate().cancel()
        dialogCard.animate().cancel()
        if (show) {
            scrimHost.visibility = View.VISIBLE
            scrimHost.alpha = 0f
            dialogCard.alpha = 0f
            dialogCard.scaleX = 0.93f
            dialogCard.scaleY = 0.93f
            dialogCard.translationY = 12f * density

            scrimHost.animate()
                .alpha(1f)
                .setDuration(dur(DurationToken.STANDARD))
                .setInterpolator(EASE_OUT_EXPO)
                .start()

            dialogCard.animate()
                .alpha(1f)
                .translationY(0f)
                .setDuration(dur(DurationToken.STANDARD))
                .setInterpolator(EASE_OUT_EXPO)
                .start()

            GcsMotionSpec.springTo(dialogCard, DynamicAnimation.SCALE_X, 1f, SpringToken.SPATIAL_MODAL)
            GcsMotionSpec.springTo(dialogCard, DynamicAnimation.SCALE_Y, 1f, SpringToken.SPATIAL_MODAL)
        } else {
            GcsMotionSpec.cancelSprings(dialogCard)
            scrimHost.animate()
                .alpha(0f)
                .setDuration(dur(DurationToken.FAST))
                .setInterpolator(EASE_EXIT_IN)
                .withEndAction { scrimHost.visibility = View.GONE }
                .start()

            dialogCard.animate()
                .alpha(0f)
                .scaleX(0.95f)
                .scaleY(0.95f)
                .translationY(8f * density)
                .setDuration(dur(DurationToken.FAST))
                .setInterpolator(EASE_EXIT_IN)
                .start()
        }
    }

    fun animateGearRotation(gearView: View, open: Boolean) {
        val targetDeg = if (open) 75f else 0f
        GcsMotionSpec.springTo(gearView, DynamicAnimation.ROTATION, targetDeg, SpringToken.GEAR_ROTATE)
    }

    fun animateCommandDock(dock: View, show: Boolean, density: Float, pulseOnUpdate: Boolean = false) {
        if (show) {
            if (!dockVisibleLogical || dock.visibility != View.VISIBLE) {
                dockVisibleLogical = true
                dock.animate().cancel()
                dock.visibility = View.VISIBLE
                dock.alpha = 0f
                dock.translationY = 14f * density
                dock.scaleX = 0.96f
                dock.scaleY = 0.96f
                dock.animate()
                    .alpha(1f)
                    .setDuration(dur(DurationToken.FAST))
                    .setInterpolator(EASE_OUT_EXPO)
                    .start()
                GcsMotionSpec.springTo(dock, DynamicAnimation.TRANSLATION_Y, 0f, SpringToken.SPATIAL_DOCK)
                GcsMotionSpec.springTo(dock, DynamicAnimation.SCALE_X, 1f, SpringToken.SPATIAL_DOCK)
                GcsMotionSpec.springTo(dock, DynamicAnimation.SCALE_Y, 1f, SpringToken.SPATIAL_DOCK)
            } else if (pulseOnUpdate && !GcsMotionSpec.isMinimalMotion()) {
                pulseViewSubtle(dock)
            }
        } else if (dockVisibleLogical || dock.visibility == View.VISIBLE) {
            dockVisibleLogical = false
            GcsMotionSpec.cancelSprings(dock)
            dock.animate().cancel()
            dock.animate()
                .alpha(0f)
                .translationY(12f * density)
                .scaleX(0.96f)
                .scaleY(0.96f)
                .setDuration(dur(DurationToken.MICRO))
                .setInterpolator(EASE_EXIT_IN)
                .withEndAction {
                    if (!dockVisibleLogical) {
                        dock.visibility = View.GONE
                        dock.translationY = 0f
                        dock.scaleX = 1f
                        dock.scaleY = 1f
                    }
                }
                .start()
        }
    }

    fun pulseViewSubtle(view: View) {
        if (GcsMotionSpec.isMinimalMotion()) return
        view.scaleX = 0.975f
        view.scaleY = 0.975f
        GcsMotionSpec.springTo(view, DynamicAnimation.SCALE_X, 1f, SpringToken.DETENT_POP)
        GcsMotionSpec.springTo(view, DynamicAnimation.SCALE_Y, 1f, SpringToken.DETENT_POP)
    }

    /**
     * 抽屉横向页签切换：底部滑块物理弹簧平移 + 面板水平方向因果滑入。
     */
    fun animateTabSwitch(
        glider: View,
        targetX: Float,
        panels: List<View>,
        tabIndex: Int,
        density: Float,
        prevTabIndex: Int = tabIndex
    ) {
        GcsMotionSpec.springTo(glider, DynamicAnimation.TRANSLATION_X, targetX, SpringToken.GLIDER_TAB)

        val dirSign = sign((tabIndex - prevTabIndex).toFloat()).let { if (it == 0f) 1f else it }
        val offsetX = if (prevTabIndex != tabIndex) dirSign * 9f * density else 0f
        val offsetY = if (prevTabIndex == tabIndex) 5f * density else 0f

        for (i in panels.indices) {
            val p = panels[i]
            p.animate().cancel()
            if (i == tabIndex) {
                p.visibility = View.VISIBLE
                p.alpha = 0f
                p.translationX = offsetX
                p.translationY = offsetY
                p.animate()
                    .alpha(1f)
                    .translationX(0f)
                    .translationY(0f)
                    .setDuration(dur(DurationToken.STANDARD))
                    .setInterpolator(EASE_OUT_EXPO)
                    .start()
                if (!GcsMotionSpec.isMinimalMotion()) {
                    animateStaggeredChildren(p, density)
                }
            } else {
                p.translationX = 0f
                p.translationY = 0f
                p.visibility = View.GONE
            }
        }
    }

    /**
     * 设置面板纵向分类切换：按新旧索引上下方向滑入 + 左侧指示条弹性伸展。
     */
    fun animateVerticalCategorySwitch(
        pages: List<View>,
        navBars: List<View>,
        newIndex: Int,
        oldIndex: Int,
        density: Float
    ) {
        val dirSign = sign((newIndex - oldIndex).toFloat()).let { if (it == 0f) 1f else it }
        for (i in navBars.indices) {
            val bar = navBars[i]
            if (i == newIndex && !GcsMotionSpec.isMinimalMotion()) {
                bar.scaleY = 0.35f
                GcsMotionSpec.springTo(bar, DynamicAnimation.SCALE_Y, 1f, SpringToken.GLIDER_TAB)
            } else {
                bar.scaleY = 1f
            }
        }
        for (i in pages.indices) {
            val page = pages[i]
            page.animate().cancel()
            if (i == newIndex) {
                page.visibility = View.VISIBLE
                page.alpha = 0f
                page.translationY = dirSign * 7f * density
                page.animate()
                    .alpha(1f)
                    .translationY(0f)
                    .setDuration(dur(DurationToken.STANDARD))
                    .setInterpolator(EASE_OUT_EXPO)
                    .start()
                animateStaggeredChildren(page, density)
            } else {
                page.translationY = 0f
                page.visibility = View.GONE
            }
        }
    }

    /**
     * 二级层级页面推入与弹出（如 ROS1 话题列表 <-> 话题实时详情终端）。
     */
    fun animateHierarchicalPage(listPage: View, detailPage: View, enterDetail: Boolean, density: Float) {
        listPage.animate().cancel()
        detailPage.animate().cancel()
        if (enterDetail) {
            listPage.visibility = View.GONE
            detailPage.visibility = View.VISIBLE
            detailPage.alpha = 0f
            detailPage.translationX = 16f * density
            detailPage.animate()
                .alpha(1f)
                .translationX(0f)
                .setDuration(dur(DurationToken.STANDARD))
                .setInterpolator(EASE_OUT_EXPO)
                .start()
            animateStaggeredChildren(detailPage, density)
        } else {
            detailPage.visibility = View.GONE
            detailPage.translationX = 0f
            listPage.visibility = View.VISIBLE
            listPage.alpha = 0f
            listPage.translationX = -14f * density
            listPage.animate()
                .alpha(1f)
                .translationX(0f)
                .setDuration(dur(DurationToken.STANDARD))
                .setInterpolator(EASE_OUT_EXPO)
                .start()
        }
    }

    fun animateStaggeredChildren(pageRoot: View, density: Float) {
        val step = GcsMotionSpec.staggerStepMs()
        if (step <= 0L) return
        val container: ViewGroup = when (pageRoot) {
            is ScrollView -> (pageRoot.getChildAt(0) as? ViewGroup) ?: return
            is ViewGroup -> pageRoot
            else -> return
        }
        val count = min(container.childCount, 6)
        var visibleIdx = 0
        for (i in 0 until count) {
            val child = container.getChildAt(i)
            if (child.visibility != View.VISIBLE) continue
            child.animate().cancel()
            child.alpha = 0.0f
            child.translationY = 6.5f * density
            child.animate()
                .alpha(1f)
                .translationY(0f)
                .setStartDelay(visibleIdx * step)
                .setDuration(dur(DurationToken.STANDARD))
                .setInterpolator(EASE_OUT_EXPO)
                .start()
            visibleIdx++
        }
    }

    fun animateThemeCrossfade(views: List<View>) {
        if (GcsMotionSpec.isMinimalMotion()) return
        for (v in views) {
            if (v.visibility != View.VISIBLE) continue
            v.animate().cancel()
            v.alpha = 0.84f
            v.animate()
                .alpha(1f)
                .setDuration(dur(DurationToken.STANDARD))
                .setInterpolator(EASE_IN_OUT_CUBIC)
                .start()
        }
    }

    fun animateBarWidthSmooth(barView: View, targetWidthPx: Int) {
        val curW = barView.layoutParams.width
        if (abs(curW - targetWidthPx) <= 2) return
        if (GcsMotionSpec.isMinimalMotion() || curW <= 0) {
            barView.layoutParams.width = targetWidthPx
            barView.requestLayout()
            return
        }
        ValueAnimator.ofInt(curW, targetWidthPx).apply {
            duration = dur(DurationToken.STANDARD)
            interpolator = EASE_OUT_EXPO
            addUpdateListener { a ->
                barView.layoutParams.width = a.animatedValue as Int
                barView.requestLayout()
            }
            start()
        }
    }

    fun showDjiToast(banner: TextView, msg: String, density: Float) {
        banner.text = msg
        banner.visibility = View.VISIBLE
        toastHideRunnable?.let { banner.removeCallbacks(it) }

        banner.animate().cancel()
        banner.translationY = -8f * density
        banner.scaleX = 0.95f
        banner.scaleY = 0.95f
        banner.alpha = 0f
        banner.animate()
            .translationY(0f)
            .scaleX(1f)
            .scaleY(1f)
            .alpha(1f)
            .setDuration(dur(DurationToken.STANDARD))
            .setInterpolator(EASE_OUT_EXPO)
            .start()

        val hideTask = Runnable {
            banner.animate()
                .translationY(-6f * density)
                .scaleX(0.96f)
                .scaleY(0.96f)
                .alpha(0f)
                .setDuration(dur(DurationToken.FAST))
                .setInterpolator(EASE_EXIT_IN)
                .withEndAction { banner.visibility = View.GONE }
                .start()
        }
        toastHideRunnable = hideTask
        banner.postDelayed(hideTask, 2200L)
    }
}
