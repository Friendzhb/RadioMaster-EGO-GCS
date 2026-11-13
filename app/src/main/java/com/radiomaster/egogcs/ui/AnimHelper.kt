package com.radiomaster.egogcs.ui

import android.animation.ValueAnimator
import android.annotation.SuppressLint
import android.view.HapticFeedbackConstants
import android.view.MotionEvent
import android.view.View
import android.view.ViewGroup
import android.view.animation.PathInterpolator
import android.widget.ScrollView
import android.widget.TextView
import androidx.dynamicanimation.animation.DynamicAnimation
import androidx.dynamicanimation.animation.SpringAnimation
import androidx.dynamicanimation.animation.SpringForce
import kotlin.math.abs
import kotlin.math.min

/**
 * 商业航空级 60FPS 物理弹簧与防碰撞连贯微交互动画引擎：
 * - 抽屉开合时根据屏幕剩余可视地图净宽精确平移顶部航向堆栈，彻底杜绝撞上左侧安全栏或右侧抽屉；
 * - Toast 提示隐藏时自动置为 View.GONE，在垂直堆栈中零高度占位、零遮挡重叠。
 */
object AnimHelper {

    val EASE_OUT_EXPO = PathInterpolator(0.22f, 1f, 0.36f, 1f)
    val EASE_IN_OUT_CUBIC = PathInterpolator(0.65f, 0f, 0.35f, 1f)

    private var toastHideRunnable: Runnable? = null
    private var dockVisibleLogical: Boolean = false

    @Volatile var hapticsEnabled: Boolean = true

    @Volatile var animStyle: Int = 0

    private fun dur(baseMs: Long): Long = when (animStyle) {
        1 -> (baseMs * 0.68f).toLong().coerceAtLeast(80L)
        2 -> (baseMs * 0.35f).toLong().coerceAtLeast(45L)
        else -> baseMs
    }

    private fun springStiffness(base: Float): Float = when (animStyle) {
        1 -> base * 1.45f
        2 -> base * 2.4f
        else -> base
    }

    private fun springDamping(base: Float): Float = when (animStyle) {
        1 -> (base + 0.08f).coerceAtMost(0.96f)
        2 -> 0.98f
        else -> base
    }

    fun triggerHaptic(view: View, feedbackConstant: Int = HapticFeedbackConstants.KEYBOARD_TAP) {
        if (hapticsEnabled) {
            view.performHapticFeedback(feedbackConstant)
        }
    }

    @SuppressLint("ClickableViewAccessibility")
    fun attachSpringTouch(view: View) {
        val scaleXAnim = SpringAnimation(view, DynamicAnimation.SCALE_X, 1f).apply {
            spring = SpringForce(1f).apply {
                stiffness = SpringForce.STIFFNESS_MEDIUM
                dampingRatio = SpringForce.DAMPING_RATIO_MEDIUM_BOUNCY
            }
        }
        val scaleYAnim = SpringAnimation(view, DynamicAnimation.SCALE_Y, 1f).apply {
            spring = SpringForce(1f).apply {
                stiffness = SpringForce.STIFFNESS_MEDIUM
                dampingRatio = SpringForce.DAMPING_RATIO_MEDIUM_BOUNCY
            }
        }

        view.setOnTouchListener { v, event ->
            when (event.actionMasked) {
                MotionEvent.ACTION_DOWN -> {
                    if (hapticsEnabled) {
                        v.performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY)
                    }
                    val pressScale = if (animStyle == 2) 0.96f else 0.93f
                    scaleXAnim.spring.stiffness = springStiffness(SpringForce.STIFFNESS_MEDIUM)
                    scaleYAnim.spring.stiffness = springStiffness(SpringForce.STIFFNESS_MEDIUM)
                    scaleXAnim.animateToFinalPosition(pressScale)
                    scaleYAnim.animateToFinalPosition(pressScale)
                }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    scaleXAnim.animateToFinalPosition(1.0f)
                    scaleYAnim.animateToFinalPosition(1.0f)
                }
            }
            false
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
        if (open) drawer.visibility = View.VISIBLE
        drawer.setLayerType(View.LAYER_TYPE_HARDWARE, null)

        val finalTx = if (open) 0f else drawerWidthPx + 32f
        SpringAnimation(drawer, DynamicAnimation.TRANSLATION_X, finalTx).apply {
            spring = SpringForce(finalTx).apply {
                stiffness = springStiffness(420f)
                dampingRatio = springDamping(0.85f)
            }
            addEndListener { _, _, _, _ ->
                drawer.setLayerType(View.LAYER_TYPE_NONE, null)
                if (!open) drawer.visibility = View.INVISIBLE
            }
            start()
        }

        drawer.animate()
            .alpha(if (open) 1f else 0f)
            .setDuration(dur(190L))
            .setInterpolator(EASE_OUT_EXPO)
            .start()

        if (open && activePanelForCascade != null && animStyle != 2) {
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
                    .translationX(16f)
                    .setDuration(dur(150L))
                    .withEndAction { rv.visibility = View.INVISIBLE }
                    .start()
            } else {
                rv.visibility = View.VISIBLE
                rv.animate()
                    .alpha(1f)
                    .translationX(0f)
                    .setDuration(dur(200L))
                    .setInterpolator(EASE_OUT_EXPO)
                    .start()
            }
        }

        val targetCenterTx = if (open) centerOffsetPx else 0f
        for (cv in centerShiftViews) {
            SpringAnimation(cv, DynamicAnimation.TRANSLATION_X, targetCenterTx).apply {
                spring = SpringForce(targetCenterTx).apply {
                    stiffness = springStiffness(400f)
                    dampingRatio = springDamping(0.86f)
                }
                start()
            }
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
                .setDuration(dur(185L))
                .setInterpolator(EASE_OUT_EXPO)
                .start()
        } else {
            popover.animate()
                .alpha(0f)
                .scaleX(0.93f)
                .scaleY(0.93f)
                .translationX(5f * density)
                .setDuration(dur(145L))
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
                .setDuration(dur(190L))
                .setInterpolator(EASE_OUT_EXPO)
                .start()

            dialogCard.animate()
                .alpha(1f)
                .translationY(0f)
                .setDuration(dur(200L))
                .setInterpolator(EASE_OUT_EXPO)
                .start()

            SpringAnimation(dialogCard, DynamicAnimation.SCALE_X, 1f).apply {
                spring = SpringForce(1f).apply {
                    stiffness = springStiffness(450f)
                    dampingRatio = springDamping(0.82f)
                }
                start()
            }
            SpringAnimation(dialogCard, DynamicAnimation.SCALE_Y, 1f).apply {
                spring = SpringForce(1f).apply {
                    stiffness = springStiffness(450f)
                    dampingRatio = springDamping(0.82f)
                }
                start()
            }
        } else {
            scrimHost.animate()
                .alpha(0f)
                .setDuration(dur(155L))
                .withEndAction { scrimHost.visibility = View.GONE }
                .start()

            dialogCard.animate()
                .alpha(0f)
                .scaleX(0.95f)
                .scaleY(0.95f)
                .translationY(8f * density)
                .setDuration(dur(145L))
                .start()
        }
    }

    fun animateGearRotation(gearView: View, open: Boolean) {
        val targetDeg = if (open) 75f else 0f
        SpringAnimation(gearView, DynamicAnimation.ROTATION, targetDeg).apply {
            spring = SpringForce(targetDeg).apply {
                stiffness = springStiffness(360f)
                dampingRatio = springDamping(0.76f)
            }
            start()
        }
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
                    .setDuration(dur(165L))
                    .setInterpolator(EASE_OUT_EXPO)
                    .start()
                SpringAnimation(dock, DynamicAnimation.TRANSLATION_Y, 0f).apply {
                    spring = SpringForce(0f).apply {
                        stiffness = springStiffness(460f)
                        dampingRatio = springDamping(0.82f)
                    }
                    start()
                }
                SpringAnimation(dock, DynamicAnimation.SCALE_X, 1f).apply {
                    spring = SpringForce(1f).apply {
                        stiffness = springStiffness(460f)
                        dampingRatio = springDamping(0.80f)
                    }
                    start()
                }
                SpringAnimation(dock, DynamicAnimation.SCALE_Y, 1f).apply {
                    spring = SpringForce(1f).apply {
                        stiffness = springStiffness(460f)
                        dampingRatio = springDamping(0.80f)
                    }
                    start()
                }
            } else if (pulseOnUpdate && animStyle != 2) {
                pulseViewSubtle(dock)
            }
        } else if (dockVisibleLogical || dock.visibility == View.VISIBLE) {
            dockVisibleLogical = false
            dock.animate().cancel()
            dock.animate()
                .alpha(0f)
                .translationY(12f * density)
                .scaleX(0.96f)
                .scaleY(0.96f)
                .setDuration(dur(140L))
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
        if (animStyle == 2) return
        view.scaleX = 0.98f
        view.scaleY = 0.98f
        SpringAnimation(view, DynamicAnimation.SCALE_X, 1f).apply {
            spring = SpringForce(1f).apply {
                stiffness = 580f
                dampingRatio = 0.74f
            }
            start()
        }
        SpringAnimation(view, DynamicAnimation.SCALE_Y, 1f).apply {
            spring = SpringForce(1f).apply {
                stiffness = 580f
                dampingRatio = 0.74f
            }
            start()
        }
    }

    fun animateTabSwitch(glider: View, targetX: Float, panels: List<View>, tabIndex: Int, density: Float) {
        SpringAnimation(glider, DynamicAnimation.TRANSLATION_X, targetX).apply {
            spring = SpringForce(targetX).apply {
                stiffness = springStiffness(480f)
                dampingRatio = springDamping(0.82f)
            }
            start()
        }

        for (i in panels.indices) {
            val p = panels[i]
            if (i == tabIndex) {
                p.visibility = View.VISIBLE
                p.alpha = 0f
                p.translationY = 6f * density
                p.animate()
                    .alpha(1f)
                    .translationY(0f)
                    .setDuration(dur(185L))
                    .setInterpolator(EASE_OUT_EXPO)
                    .start()
                if (animStyle != 2) {
                    animateStaggeredChildren(p, density)
                }
            } else {
                p.visibility = View.GONE
            }
        }
    }

    fun animateStaggeredChildren(pageRoot: View, density: Float) {
        if (animStyle == 2) return
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
            child.translationY = 7f * density
            child.animate()
                .alpha(1f)
                .translationY(0f)
                .setStartDelay(visibleIdx * 18L)
                .setDuration(dur(195L))
                .setInterpolator(EASE_OUT_EXPO)
                .start()
            visibleIdx++
        }
    }

    fun animateThemeCrossfade(views: List<View>) {
        if (animStyle == 2) return
        for (v in views) {
            if (v.visibility != View.VISIBLE) continue
            v.animate().cancel()
            v.alpha = 0.84f
            v.animate()
                .alpha(1f)
                .setDuration(dur(220L))
                .setInterpolator(EASE_OUT_EXPO)
                .start()
        }
    }

    fun animateBarWidthSmooth(barView: View, targetWidthPx: Int) {
        val curW = barView.layoutParams.width
        if (abs(curW - targetWidthPx) <= 2) return
        if (animStyle == 2 || curW <= 0) {
            barView.layoutParams.width = targetWidthPx
            barView.requestLayout()
            return
        }
        ValueAnimator.ofInt(curW, targetWidthPx).apply {
            duration = dur(210L)
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
            .setDuration(dur(190L))
            .setInterpolator(EASE_OUT_EXPO)
            .start()

        val hideTask = Runnable {
            banner.animate()
                .translationY(-6f * density)
                .scaleX(0.96f)
                .scaleY(0.96f)
                .alpha(0f)
                .setDuration(dur(170L))
                .withEndAction { banner.visibility = View.GONE }
                .start()
        }
        toastHideRunnable = hideTask
        banner.postDelayed(hideTask, 2200L)
    }
}
