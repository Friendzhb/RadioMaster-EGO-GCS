package com.radiomaster.egogcs.ui

import android.annotation.SuppressLint
import android.content.Context
import android.view.MotionEvent
import android.view.View
import android.view.ViewConfiguration
import android.view.ViewGroup
import android.view.ViewParent
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.HorizontalScrollView
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.SeekBar
import kotlin.math.abs

/**
 * 图标容器与屏幕边缘 `onInterceptTouchEvent` 热区容差引擎 (`TouchChainGuard`)：
 * 1. **显式 `onInterceptTouchEvent` 热区扩展**：通过 [IconSlopFrameLayout] 与 [IconSlopLinearLayout] 重写
 *    `onInterceptTouchEvent`，围绕交互图标建立命中容差缓冲区（Hit-Test Slop Buffer），在子视图未直接命中时主动拦截并代理完整手势。
 * 2. **屏幕边缘非对称热区补偿**：针对紧贴屏幕四边/圆角区的图标（如右上角设置齿轮、左侧紧急悬停、右侧相机栈、底部指令坞），
 *    自动向物理屏幕边缘扩展热区直至像素边界 (`0` / `screenW` / `screenH`)，彻底消除屏幕边缘死区。
 * 3. **反缩放矩阵坐标还原与跨层隔离**：消除弹簧按压缩小 (`0.89x`) 导致的边缘掉按压态，关闭多指事件分裂，阻断手势穿透到底层 3D 地图。
 */
object TouchChainGuard {

    enum class ContainerMode {
        /** 顶层座舱根容器：跨子容器边界拦截贴边图标边缘触点，空白地图区域正常放行至 3D 地图 */
        ROOT_COCKPIT,

        /** 实心/半透明完整栏位（顶栏、多机栏、指令坞、浮窗、抽屉、弹窗卡片）：扩展图标热区并消费全部栏内死区触点 */
        SOLID_BAR,

        /** 悬浮稀疏簇（左安全栏、右相机栈、左下仪表簇、顶部居中栈）：扩展图标热区并保护按钮间隙，空白区放行 */
        FLOATING_CLUSTER,

        /** 全屏模态遮罩：展开态支持卡片边缘关闭键热区扩展，遮罩空白处走原生点击关闭，退场动画期间立即放行 */
        MODAL_SCRIM,

        /** 滚动列表内的紧凑图标行（如航点编排行 23x23dp 图标）：扩展行内小图标热区，纵向拖动超阈值时平滑让渡给 ScrollView */
        SCROLLABLE_ICON_ROW
    }

    private const val TAG_PASSTHROUGH_SURFACE = -0x35792468
    private const val TAG_DECORATIVE_STRIP = -0x35792469

    internal data class TargetRect(
        val id: Int,
        val left: Float,
        val top: Float,
        val right: Float,
        val bottom: Float
    ) {
        val width: Float get() = (right - left).coerceAtLeast(0f)
        val height: Float get() = (bottom - top).coerceAtLeast(0f)
        val centerX: Float get() = (left + right) * 0.5f
        val centerY: Float get() = (top + bottom) * 0.5f
    }

    /**
     * 含屏幕边缘非对称补偿的图标热区包络矩形（坐标系与 [rect] 一致，均为宿主容器局部未缩放坐标）。
     */
    internal data class EdgeSlopRect(
        val rect: TargetRect,
        val slopLeft: Float,
        val slopTop: Float,
        val slopRight: Float,
        val slopBottom: Float
    ) {
        fun containsInSlop(x: Float, y: Float): Boolean =
            x >= slopLeft && x <= slopRight && y >= slopTop && y <= slopBottom
    }

    private data class BoundSlopTarget(
        val view: View,
        val slopRect: EdgeSlopRect
    )

    internal interface SlopManagedContainer {
        val containerMode: ContainerMode
        fun isSlopActive(): Boolean
        fun wouldClaimPoint(localX: Float, localY: Float): Boolean
    }

    fun markPassThroughSurface(view: View) {
        view.setTag(TAG_PASSTHROUGH_SURFACE, true)
    }

    fun markDecorativeStrip(view: View) {
        view.setTag(TAG_DECORATIVE_STRIP, true)
    }

    private fun isPassThroughSurface(view: View): Boolean =
        view.getTag(TAG_PASSTHROUGH_SURFACE) == true

    private fun isDecorativeStrip(view: View): Boolean =
        view.getTag(TAG_DECORATIVE_STRIP) == true

    internal fun unscaleAxis(transformedLocal: Float, pivot: Float, scale: Float): Float {
        if (!scale.isFinite() || abs(scale) < 1e-4f || abs(scale - 1f) < 1e-4f) {
            return transformedLocal
        }
        return pivot + (transformedLocal - pivot) * scale
    }

    internal fun inverseScaleAxis(unscaledLocal: Float, pivot: Float, scale: Float): Float {
        if (!scale.isFinite() || abs(scale) < 1e-4f || abs(scale - 1f) < 1e-4f) {
            return unscaledLocal
        }
        return pivot + (unscaledLocal - pivot) / scale
    }

    internal fun isPointInUnscaledBounds(
        unscaledLocalX: Float,
        unscaledLocalY: Float,
        width: Int,
        height: Int,
        slopPx: Float
    ): Boolean {
        return unscaledLocalX >= -slopPx &&
            unscaledLocalX <= width + slopPx &&
            unscaledLocalY >= -slopPx &&
            unscaledLocalY <= height + slopPx
    }

    internal fun distanceSquaredToRect(x: Float, y: Float, rect: TargetRect): Float {
        val dx = when {
            x < rect.left -> rect.left - x
            x > rect.right -> x - rect.right
            else -> 0f
        }
        val dy = when {
            y < rect.top -> rect.top - y
            y > rect.bottom -> y - rect.bottom
            else -> 0f
        }
        return dx * dx + dy * dy
    }

    internal fun isWithinAnyRectMargin(
        x: Float,
        y: Float,
        rects: List<TargetRect>,
        marginPx: Float
    ): Boolean {
        for (i in rects.indices) {
            val r = rects[i]
            if (x >= r.left - marginPx &&
                x <= r.right + marginPx &&
                y >= r.top - marginPx &&
                y <= r.bottom + marginPx
            ) {
                return true
            }
        }
        return false
    }

    internal fun isPointCoveredByAnyRect(x: Float, y: Float, rects: List<TargetRect>): Boolean {
        for (i in rects.indices) {
            val r = rects[i]
            if (x >= r.left && x <= r.right && y >= r.top && y <= r.bottom) {
                return true
            }
        }
        return false
    }

    /**
     * 计算带屏幕边缘非对称补偿的图标热区范围：
     * - 紧凑图标（任一边 `< compactThresholdPx`）赋予更大的基础容差 `compactSlopPx`，常规胶囊赋予 `standardSlopPx`；
     * - 若图标某侧距物理屏幕边界不超过 `edgeZonePx`，该侧容差自动放大至覆盖到屏幕最外侧像素边界（上限 `edgeSlopPx`），
     *   保证手指点在贴边留白/边距上仍能 100% 稳定命中图标。
     */
    internal fun computeEdgeExpandedSlopRect(
        rect: TargetRect,
        containerWinX: Float,
        containerWinY: Float,
        screenWidth: Float,
        screenHeight: Float,
        compactThresholdPx: Float,
        compactSlopPx: Float,
        standardSlopPx: Float,
        edgeSlopPx: Float,
        edgeZonePx: Float
    ): EdgeSlopRect {
        val isCompact = rect.width < compactThresholdPx || rect.height < compactThresholdPx
        val baseSlop = if (isCompact) compactSlopPx else standardSlopPx

        val winLeft = containerWinX + rect.left
        val winTop = containerWinY + rect.top
        val winRight = containerWinX + rect.right
        val winBottom = containerWinY + rect.bottom

        val distLeft = winLeft.coerceAtLeast(0f)
        val distTop = winTop.coerceAtLeast(0f)
        val distRight = (screenWidth - winRight).coerceAtLeast(0f)
        val distBottom = (screenHeight - winBottom).coerceAtLeast(0f)

        val expandLeft = if (screenWidth > 0f && winLeft <= edgeZonePx) {
            maxOf(baseSlop, minOf(edgeSlopPx, distLeft + 2f))
        } else {
            baseSlop
        }
        val expandTop = if (screenHeight > 0f && winTop <= edgeZonePx) {
            maxOf(baseSlop, minOf(edgeSlopPx, distTop + 2f))
        } else {
            baseSlop
        }
        val expandRight = if (screenWidth > 0f && distRight <= edgeZonePx) {
            maxOf(baseSlop, minOf(edgeSlopPx, distRight + 2f))
        } else {
            baseSlop
        }
        val expandBottom = if (screenHeight > 0f && distBottom <= edgeZonePx) {
            maxOf(baseSlop, minOf(edgeSlopPx, distBottom + 2f))
        } else {
            baseSlop
        }

        return EdgeSlopRect(
            rect = rect,
            slopLeft = rect.left - expandLeft,
            slopTop = rect.top - expandTop,
            slopRight = rect.right + expandRight,
            slopBottom = rect.bottom + expandBottom
        )
    }

    /**
     * 在扩展热区候选中解析最佳命中图标索引：
     * - 优先返回未缩放实体矩形直接包含 `(x, y)` 的上层控件（修复 `scale < 1` 期间边缘连点漏判）；
     * - 其次在包含 `(x, y)` 的扩展热区中按到实体矩形的欧氏距离最近邻唯一选取（相邻图标自动沿中垂线平分间隙）。
     */
    internal fun resolveSlopHitIndex(
        x: Float,
        y: Float,
        candidates: List<EdgeSlopRect>
    ): Int {
        for (i in candidates.indices.reversed()) {
            val r = candidates[i].rect
            if (x >= r.left && x <= r.right && y >= r.top && y <= r.bottom) {
                return i
            }
        }
        var bestIdx = -1
        var bestDistSq = Float.MAX_VALUE
        for (i in candidates.indices.reversed()) {
            val c = candidates[i]
            if (c.containsInSlop(x, y)) {
                val dSq = distanceSquaredToRect(x, y, c.rect)
                if (dSq < bestDistSq) {
                    bestDistSq = dSq
                    bestIdx = i
                }
            }
        }
        return bestIdx
    }

    /**
     * 兼容纯矩形统一容差查询的便捷重载。
     */
    internal fun resolveNearestTargetIndex(
        x: Float,
        y: Float,
        targets: List<TargetRect>,
        haloSlopPx: Float
    ): Int {
        if (haloSlopPx < 0f) return -1
        val wrapped = targets.map { r ->
            EdgeSlopRect(r, r.left - haloSlopPx, r.top - haloSlopPx, r.right + haloSlopPx, r.bottom + haloSlopPx)
        }
        return resolveSlopHitIndex(x, y, wrapped)
    }

    /**
     * 将父容器触点映射为目标图标的未缩放局部坐标：
     * - 当触点处于该图标的 [EdgeSlopRect] 热区内时，钳位至图标内部 `[inset, size - inset]` 确保贴边点击可靠触发；
     * - 当手指拖出热区外时，输出真实越界坐标，使目标图标立即平滑回弹并取消点击，绝不误触发。
     */
    internal fun mapParentPointIntoSlopTarget(
        parentX: Float,
        parentY: Float,
        slopRect: EdgeSlopRect,
        outLocal: FloatArray = FloatArray(2)
    ): FloatArray {
        val r = slopRect.rect
        val rawLocalX = parentX - r.left
        val rawLocalY = parentY - r.top
        val w = r.width
        val h = r.height
        if (slopRect.containsInSlop(parentX, parentY) && w > 0f && h > 0f) {
            val insetX = minOf(0.5f, w * 0.25f)
            val insetY = minOf(0.5f, h * 0.25f)
            outLocal[0] = rawLocalX.coerceIn(insetX, (w - insetX).coerceAtLeast(insetX))
            outLocal[1] = rawLocalY.coerceIn(insetY, (h - insetY).coerceAtLeast(insetY))
        } else {
            outLocal[0] = rawLocalX
            outLocal[1] = rawLocalY
        }
        return outLocal
    }

    internal fun mapParentPointToUnscaledChildLocal(
        parentX: Float,
        parentY: Float,
        rect: TargetRect,
        haloSlopPx: Float,
        outLocal: FloatArray = FloatArray(2)
    ): FloatArray {
        val sr = EdgeSlopRect(
            rect = rect,
            slopLeft = rect.left - haloSlopPx,
            slopTop = rect.top - haloSlopPx,
            slopRight = rect.right + haloSlopPx,
            slopBottom = rect.bottom + haloSlopPx
        )
        return mapParentPointIntoSlopTarget(parentX, parentY, sr, outLocal)
    }

    fun hasScrollableAncestor(view: View): Boolean {
        var p: ViewParent? = view.parent
        while (p != null) {
            if (p is ScrollView || p is HorizontalScrollView) return true
            p = p.parent
        }
        return false
    }

    fun disableMotionSplittingHierarchy(group: ViewGroup) {
        group.isMotionEventSplittingEnabled = false
        for (i in 0 until group.childCount) {
            val c = group.getChildAt(i)
            if (c is ViewGroup) {
                disableMotionSplittingHierarchy(c)
            }
        }
    }

    /**
     * 重写 `onInterceptTouchEvent` 与 `onTouchEvent` 的线性图标容器，原生支持图标边缘与屏幕边界热区扩展。
     */
    open class IconSlopLinearLayout(
        context: Context,
        override val containerMode: ContainerMode,
        density: Float,
        compactSlopDp: Float = 8.5f,
        standardSlopDp: Float = 5.5f,
        edgeSlopDp: Float = 16f,
        guardMarginDp: Float = 6.5f,
        private val activePredicate: () -> Boolean = { true }
    ) : LinearLayout(context), SlopManagedContainer {

        private val controller = SlopInterceptController(
            host = this,
            mode = containerMode,
            density = density,
            compactSlopDp = compactSlopDp,
            standardSlopDp = standardSlopDp,
            edgeSlopDp = edgeSlopDp,
            guardMarginDp = guardMarginDp,
            activePredicate = activePredicate
        )

        init {
            isMotionEventSplittingEnabled = false
        }

        override fun isSlopActive(): Boolean =
            visibility == View.VISIBLE && isEnabled && activePredicate()

        override fun wouldClaimPoint(localX: Float, localY: Float): Boolean =
            controller.wouldClaimPoint(localX, localY)

        override fun dispatchTouchEvent(ev: MotionEvent): Boolean {
            if (!activePredicate()) {
                controller.reset()
                return false
            }
            return super.dispatchTouchEvent(ev)
        }

        override fun onInterceptTouchEvent(ev: MotionEvent): Boolean {
            if (controller.onInterceptTouchEvent(ev)) {
                return true
            }
            return super.onInterceptTouchEvent(ev)
        }

        @SuppressLint("ClickableViewAccessibility")
        override fun onTouchEvent(event: MotionEvent): Boolean {
            return controller.onTouchEvent(event) { super.onTouchEvent(it) }
        }
    }

    /**
     * 重写 `onInterceptTouchEvent` 与 `onTouchEvent` 的帧布局容器（用于 `cockpitRoot` 顶层屏幕边缘热区拦截及模态遮罩）。
     */
    open class IconSlopFrameLayout(
        context: Context,
        override val containerMode: ContainerMode,
        density: Float,
        compactSlopDp: Float = 8.5f,
        standardSlopDp: Float = 5.5f,
        edgeSlopDp: Float = 16f,
        guardMarginDp: Float = 6.5f,
        private val activePredicate: () -> Boolean = { true }
    ) : FrameLayout(context), SlopManagedContainer {

        private val controller = SlopInterceptController(
            host = this,
            mode = containerMode,
            density = density,
            compactSlopDp = compactSlopDp,
            standardSlopDp = standardSlopDp,
            edgeSlopDp = edgeSlopDp,
            guardMarginDp = guardMarginDp,
            activePredicate = activePredicate
        )

        init {
            isMotionEventSplittingEnabled = false
        }

        override fun isSlopActive(): Boolean =
            visibility == View.VISIBLE && isEnabled && activePredicate()

        override fun wouldClaimPoint(localX: Float, localY: Float): Boolean =
            controller.wouldClaimPoint(localX, localY)

        override fun dispatchTouchEvent(ev: MotionEvent): Boolean {
            if (!activePredicate()) {
                controller.reset()
                return false
            }
            return super.dispatchTouchEvent(ev)
        }

        override fun onInterceptTouchEvent(ev: MotionEvent): Boolean {
            if (controller.onInterceptTouchEvent(ev)) {
                return true
            }
            return super.onInterceptTouchEvent(ev)
        }

        @SuppressLint("ClickableViewAccessibility")
        override fun onTouchEvent(event: MotionEvent): Boolean {
            return controller.onTouchEvent(event) { super.onTouchEvent(it) }
        }
    }

    private class SlopInterceptController(
        private val host: ViewGroup,
        private val mode: ContainerMode,
        density: Float,
        compactSlopDp: Float,
        standardSlopDp: Float,
        edgeSlopDp: Float,
        guardMarginDp: Float,
        private val activePredicate: () -> Boolean
    ) {
        private val systemTouchSlop = ViewConfiguration.get(host.context).scaledTouchSlop.toFloat()
        private val compactThresholdPx = 38f * density
        private val compactSlopPx = (compactSlopDp * density).coerceAtLeast(systemTouchSlop * 0.5f)
        private val standardSlopPx = (standardSlopDp * density).coerceAtLeast(4f * density)
        private val edgeSlopPx = (edgeSlopDp * density).coerceAtLeast(compactSlopPx)
        private val edgeZonePx = 28f * density
        private val guardMarginPx = (guardMarginDp * density).coerceAtLeast(standardSlopPx)

        private val winLocScratch = IntArray(2)
        private val localScratch = FloatArray(2)

        private var interceptedTarget: BoundSlopTarget? = null
        private var interceptedPointerId: Int = MotionEvent.INVALID_POINTER_ID
        private var downRawX: Float = 0f
        private var downRawY: Float = 0f
        private var scrollableHost: Boolean = false
        private var absorbingDeadZone: Boolean = false

        fun reset() {
            interceptedTarget = null
            interceptedPointerId = MotionEvent.INVALID_POINTER_ID
            absorbingDeadZone = false
        }

        fun wouldClaimPoint(localX: Float, localY: Float): Boolean {
            if (host.visibility != View.VISIBLE || !host.isEnabled || !activePredicate()) return false
            if (willDirectDescendantHandle(host, localX, localY)) return true
            val interactive = ArrayList<BoundSlopTarget>(8)
            val visibleRects = ArrayList<TargetRect>(8)
            val occluders = ArrayList<TargetRect>(4)
            val (winX, winY, screenW, screenH) = queryScreenContext()
            collectSlopCandidates(host, 0f, 0f, winX, winY, screenW, screenH, interactive, visibleRects, occluders)
            if (resolveSlopHitIndex(localX, localY, interactive.map { it.slopRect }) >= 0) {
                return true
            }
            return when (mode) {
                ContainerMode.SOLID_BAR, ContainerMode.MODAL_SCRIM -> true
                ContainerMode.FLOATING_CLUSTER -> isWithinAnyRectMargin(localX, localY, visibleRects, guardMarginPx)
                ContainerMode.ROOT_COCKPIT, ContainerMode.SCROLLABLE_ICON_ROW -> false
            }
        }

        /**
         * 显式在 `onInterceptTouchEvent` 中检测扩展热区：
         * 若子控件原始变换边界未能直接命中，但触点落入某个可见交互图标的屏幕边缘自适应热区容差内，
         * 立即返回 `true` 拦截该手势并交由本容器 `onTouchEvent` 代理分发至目标图标。
         */
        fun onInterceptTouchEvent(ev: MotionEvent): Boolean {
            if (host.visibility != View.VISIBLE || !host.isEnabled || !activePredicate()) {
                reset()
                return false
            }
            host.isMotionEventSplittingEnabled = false
            if (ev.actionMasked != MotionEvent.ACTION_DOWN) {
                return false
            }

            reset()
            if (willDirectDescendantHandle(host, ev.x, ev.y)) {
                return false
            }

            val interactive = ArrayList<BoundSlopTarget>(12)
            val visibleRects = ArrayList<TargetRect>(12)
            val occluders = ArrayList<TargetRect>(4)
            val (winX, winY, screenW, screenH) = queryScreenContext()
            collectSlopCandidates(host, 0f, 0f, winX, winY, screenW, screenH, interactive, visibleRects, occluders)

            val hitIdx = resolveSlopHitIndex(ev.x, ev.y, interactive.map { it.slopRect })
            if (hitIdx >= 0) {
                interceptedTarget = interactive[hitIdx]
                interceptedPointerId = ev.getPointerId(0)
                downRawX = ev.rawX
                downRawY = ev.rawY
                scrollableHost = hasScrollableAncestor(host)
                return true
            }
            return false
        }

        fun onTouchEvent(event: MotionEvent, callSuper: (MotionEvent) -> Boolean): Boolean {
            if (host.visibility != View.VISIBLE || !host.isEnabled || !activePredicate()) {
                reset()
                return false
            }

            when (event.actionMasked) {
                MotionEvent.ACTION_DOWN -> {
                    val active = interceptedTarget
                    if (active != null) {
                        host.parent?.requestDisallowInterceptTouchEvent(true)
                        dispatchMappedToTarget(active, event, pointerIndex = 0)
                        return true
                    }

                    val interactive = ArrayList<BoundSlopTarget>(12)
                    val visibleRects = ArrayList<TargetRect>(12)
                    val occluders = ArrayList<TargetRect>(4)
                    val (winX, winY, screenW, screenH) = queryScreenContext()
                    collectSlopCandidates(host, 0f, 0f, winX, winY, screenW, screenH, interactive, visibleRects, occluders)

                    val hitIdx = resolveSlopHitIndex(event.x, event.y, interactive.map { it.slopRect })
                    if (hitIdx >= 0) {
                        val chosen = interactive[hitIdx]
                        interceptedTarget = chosen
                        interceptedPointerId = event.getPointerId(0)
                        downRawX = event.rawX
                        downRawY = event.rawY
                        scrollableHost = hasScrollableAncestor(host)
                        host.parent?.requestDisallowInterceptTouchEvent(true)
                        dispatchMappedToTarget(chosen, event, pointerIndex = 0)
                        return true
                    }

                    return when (mode) {
                        ContainerMode.SOLID_BAR -> {
                            absorbingDeadZone = true
                            host.parent?.requestDisallowInterceptTouchEvent(true)
                            true
                        }
                        ContainerMode.FLOATING_CLUSTER -> {
                            if (isWithinAnyRectMargin(event.x, event.y, visibleRects, guardMarginPx)) {
                                absorbingDeadZone = true
                                host.parent?.requestDisallowInterceptTouchEvent(true)
                                true
                            } else {
                                false
                            }
                        }
                        ContainerMode.MODAL_SCRIM,
                        ContainerMode.SCROLLABLE_ICON_ROW,
                        ContainerMode.ROOT_COCKPIT -> callSuper(event)
                    }
                }

                MotionEvent.ACTION_MOVE,
                MotionEvent.ACTION_POINTER_DOWN,
                MotionEvent.ACTION_POINTER_UP -> {
                    val active = interceptedTarget
                    if (active != null) {
                        val pIdx = event.findPointerIndex(interceptedPointerId)
                        if (pIdx < 0 ||
                            (
                                event.actionMasked == MotionEvent.ACTION_POINTER_UP &&
                                    event.getPointerId(event.actionIndex) == interceptedPointerId
                                )
                        ) {
                            dispatchSyntheticCancel(active.view, event)
                            interceptedTarget = null
                            interceptedPointerId = MotionEvent.INVALID_POINTER_ID
                            absorbingDeadZone = mode != ContainerMode.SCROLLABLE_ICON_ROW
                            return true
                        }
                        if (event.actionMasked == MotionEvent.ACTION_MOVE) {
                            if (scrollableHost) {
                                val dyRaw = abs(event.rawY - downRawY)
                                val dxRaw = abs(event.rawX - downRawX)
                                if (dyRaw > systemTouchSlop || dxRaw > systemTouchSlop * 1.5f) {
                                    host.parent?.requestDisallowInterceptTouchEvent(false)
                                }
                            }
                            dispatchMappedToTarget(active, event, pointerIndex = pIdx)
                        }
                        return true
                    }
                    if (absorbingDeadZone || mode == ContainerMode.SOLID_BAR) {
                        return true
                    }
                    return callSuper(event)
                }

                MotionEvent.ACTION_UP,
                MotionEvent.ACTION_CANCEL -> {
                    val active = interceptedTarget
                    val wasAbsorbing = absorbingDeadZone
                    val trackedId = interceptedPointerId
                    reset()

                    if (active != null) {
                        val pIdx = event.findPointerIndex(trackedId)
                        if (pIdx >= 0 && event.actionMasked == MotionEvent.ACTION_UP) {
                            dispatchMappedToTarget(active, event, pointerIndex = pIdx)
                        } else {
                            dispatchSyntheticCancel(active.view, event)
                        }
                        return true
                    }
                    if (wasAbsorbing || mode == ContainerMode.SOLID_BAR) {
                        return true
                    }
                    return callSuper(event)
                }

                else -> {
                    if (interceptedTarget != null || absorbingDeadZone || mode == ContainerMode.SOLID_BAR) {
                        return true
                    }
                    return callSuper(event)
                }
            }
        }

        private fun queryScreenContext(): FloatArray {
            host.getLocationInWindow(winLocScratch)
            val dm = host.resources.displayMetrics
            val rootW = host.rootView?.width?.takeIf { it > 0 }?.toFloat() ?: dm.widthPixels.toFloat()
            val rootH = host.rootView?.height?.takeIf { it > 0 }?.toFloat() ?: dm.heightPixels.toFloat()
            return floatArrayOf(
                winLocScratch[0].toFloat(),
                winLocScratch[1].toFloat(),
                rootW,
                rootH
            )
        }

        private fun willDirectDescendantHandle(group: ViewGroup, x: Float, y: Float): Boolean {
            for (i in group.childCount - 1 downTo 0) {
                val child = group.getChildAt(i)
                if (child.visibility != View.VISIBLE || child.alpha <= 0.05f) continue
                if (isPassThroughSurface(child) || isDecorativeStrip(child)) continue
                if (child is SlopManagedContainer && !child.isSlopActive()) continue

                val unscaledL = child.left + child.translationX
                val unscaledT = child.top + child.translationY
                val unscaledR = child.right + child.translationX
                val unscaledB = child.bottom + child.translationY
                val inUnscaled = x >= unscaledL && x <= unscaledR && y >= unscaledT && y <= unscaledB

                if (child is SlopManagedContainer) {
                    if (inUnscaled) {
                        when (child.containerMode) {
                            ContainerMode.SOLID_BAR,
                            ContainerMode.MODAL_SCRIM -> return true
                            ContainerMode.FLOATING_CLUSTER,
                            ContainerMode.SCROLLABLE_ICON_ROW,
                            ContainerMode.ROOT_COCKPIT -> {
                                if (child.wouldClaimPoint(x - unscaledL, y - unscaledT)) {
                                    return true
                                }
                            }
                        }
                    }
                    continue
                }

                if (child is ScrollView ||
                    child is HorizontalScrollView ||
                    child is EditText ||
                    child is SeekBar
                ) {
                    if (inUnscaled && child.isEnabled) return true
                    continue
                }

                if (child.isEnabled && (child.isClickable || child.isLongClickable)) {
                    if (willChildReceiveTransformedHit(child, x, y)) {
                        return true
                    }
                } else if (child is ViewGroup && inUnscaled) {
                    if (willDirectDescendantHandle(child, x - unscaledL, y - unscaledT)) {
                        return true
                    }
                }
            }
            return false
        }

        private fun willChildReceiveTransformedHit(child: View, parentX: Float, parentY: Float): Boolean {
            val w = child.width
            val h = child.height
            if (w <= 0 || h <= 0) return false
            val unscaledLocalX = parentX - child.left - child.translationX
            val unscaledLocalY = parentY - child.top - child.translationY
            val tx = inverseScaleAxis(unscaledLocalX, child.pivotX, child.scaleX)
            val ty = inverseScaleAxis(unscaledLocalY, child.pivotY, child.scaleY)
            return tx >= 0f && tx <= w.toFloat() && ty >= 0f && ty <= h.toFloat()
        }

        private fun collectSlopCandidates(
            group: ViewGroup,
            offsetX: Float,
            offsetY: Float,
            hostWinX: Float,
            hostWinY: Float,
            screenW: Float,
            screenH: Float,
            outInteractive: MutableList<BoundSlopTarget>,
            outVisibleRects: MutableList<TargetRect>,
            occluders: MutableList<TargetRect>
        ) {
            for (i in group.childCount - 1 downTo 0) {
                val child = group.getChildAt(i)
                if (child.visibility != View.VISIBLE || child.alpha <= 0.05f) continue
                if (isPassThroughSurface(child) || isDecorativeStrip(child)) continue
                if (child is SlopManagedContainer && !child.isSlopActive()) continue

                val l = offsetX + child.left + child.translationX
                val t = offsetY + child.top + child.translationY
                val r = offsetX + child.right + child.translationX
                val b = offsetY + child.bottom + child.translationY
                if (r <= l || b <= t) continue

                val rect = TargetRect(outVisibleRects.size, l, t, r, b)
                val isDiscreteControl = child.isEnabled &&
                    (child.isClickable || child.isLongClickable) &&
                    child !is SlopManagedContainer &&
                    child !is EditText &&
                    child !is SeekBar &&
                    child !is ScrollView &&
                    child !is HorizontalScrollView

                if (isDiscreteControl) {
                    if (!isPointCoveredByAnyRect(rect.centerX, rect.centerY, occluders)) {
                        outVisibleRects.add(rect)
                        val slopRect = computeEdgeExpandedSlopRect(
                            rect = rect,
                            containerWinX = hostWinX,
                            containerWinY = hostWinY,
                            screenWidth = screenW,
                            screenHeight = screenH,
                            compactThresholdPx = compactThresholdPx,
                            compactSlopPx = compactSlopPx,
                            standardSlopPx = standardSlopPx,
                            edgeSlopPx = edgeSlopPx,
                            edgeZonePx = edgeZonePx
                        )
                        outInteractive.add(0, BoundSlopTarget(child, slopRect))
                    }
                } else if (child is ViewGroup &&
                    child !is ScrollView &&
                    child !is HorizontalScrollView
                ) {
                    if (child.background != null && child !is SlopManagedContainer) {
                        outVisibleRects.add(rect)
                    }
                    collectSlopCandidates(
                        group = child,
                        offsetX = l,
                        offsetY = t,
                        hostWinX = hostWinX,
                        hostWinY = hostWinY,
                        screenW = screenW,
                        screenH = screenH,
                        outInteractive = outInteractive,
                        outVisibleRects = outVisibleRects,
                        occluders = occluders
                    )
                    if (child is SlopManagedContainer &&
                        (
                            child.containerMode == ContainerMode.SOLID_BAR ||
                                child.containerMode == ContainerMode.MODAL_SCRIM
                            )
                    ) {
                        occluders.add(rect)
                    }
                } else {
                    outVisibleRects.add(rect)
                }
            }
        }

        private fun dispatchMappedToTarget(
            bound: BoundSlopTarget,
            srcEvent: MotionEvent,
            pointerIndex: Int
        ) {
            val target = bound.view
            val px = srcEvent.getX(pointerIndex)
            val py = srcEvent.getY(pointerIndex)
            mapParentPointIntoSlopTarget(px, py, bound.slopRect, localScratch)
            val tx = inverseScaleAxis(localScratch[0], target.pivotX, target.scaleX)
            val ty = inverseScaleAxis(localScratch[1], target.pivotY, target.scaleY)
            val action = when (srcEvent.actionMasked) {
                MotionEvent.ACTION_DOWN -> MotionEvent.ACTION_DOWN
                MotionEvent.ACTION_UP -> MotionEvent.ACTION_UP
                MotionEvent.ACTION_CANCEL -> MotionEvent.ACTION_CANCEL
                else -> MotionEvent.ACTION_MOVE
            }
            val mapped = MotionEvent.obtain(
                srcEvent.downTime,
                srcEvent.eventTime,
                action,
                tx,
                ty,
                srcEvent.metaState
            )
            try {
                target.dispatchTouchEvent(mapped)
            } finally {
                mapped.recycle()
            }
        }

        private fun dispatchSyntheticCancel(target: View, srcEvent: MotionEvent) {
            val cancelEvent = MotionEvent.obtain(
                srcEvent.downTime,
                srcEvent.eventTime,
                MotionEvent.ACTION_CANCEL,
                0f,
                0f,
                srcEvent.metaState
            )
            try {
                target.dispatchTouchEvent(cancelEvent)
            } finally {
                cancelEvent.recycle()
            }
        }
    }
}
