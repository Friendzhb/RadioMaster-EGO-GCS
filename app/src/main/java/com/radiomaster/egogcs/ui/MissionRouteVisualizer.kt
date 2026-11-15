package com.radiomaster.egogcs.ui

import com.radiomaster.egogcs.model.RenderSnapshot
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

/**
 * Zero-allocation 3D waypoint marker, vertical altitude slider gizmo, planned mission route
 * visualizer, and perspective-exact 3D screen-space hit-tester / vertical axis unprojector.
 */
internal object MissionRouteVisualizer {

    enum class WaypointHitMode {
        NONE,
        SELECT_OR_DRAG_XY,
        DRAG_ALTITUDE_Z
    }

    class WaypointHitResult(
        var index: Int = -1,
        var mode: WaypointHitMode = WaypointHitMode.NONE,
        var headScreenX: Float = 0f,
        var headScreenY: Float = 0f
    ) {
        fun reset() {
            index = -1
            mode = WaypointHitMode.NONE
            headScreenX = 0f
            headScreenY = 0f
        }
    }

    interface GeometrySink {
        fun pushActorTri(
            x0: Float,
            y0: Float,
            z0: Float,
            arc0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            arc1: Float,
            x2: Float,
            y2: Float,
            z2: Float,
            arc2: Float,
            r: Float,
            g: Float,
            b: Float,
            a: Float
        )

        fun pushActorLine(
            x0: Float,
            y0: Float,
            z0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            r: Float,
            g: Float,
            b: Float,
            a: Float,
            arc0: Float,
            arc1: Float
        )

        fun pushActorQuadSeg3D(
            x0: Float,
            y0: Float,
            z0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            halfW: Float,
            r: Float,
            g: Float,
            b: Float,
            a: Float,
            arc0: Float,
            arc1: Float
        )

        fun pushActorDiscXY(
            cx: Float,
            cy: Float,
            z: Float,
            rad: Float,
            segs: Int,
            r: Float,
            g: Float,
            b: Float,
            a: Float,
            arc: Float
        )

        fun pushActorRingXY(
            cx: Float,
            cy: Float,
            z: Float,
            rad: Float,
            thick: Float,
            segs: Int,
            r: Float,
            g: Float,
            b: Float,
            a: Float,
            arc: Float
        )

        fun pushGroundLine(
            x0: Float,
            y0: Float,
            z0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            r: Float,
            g: Float,
            b: Float,
            a: Float
        )

        fun pushGroundDashedSeg(
            x0: Float,
            y0: Float,
            z0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            r: Float,
            g: Float,
            b: Float,
            a: Float
        )

        fun pushFxRingXY(
            cx: Float,
            cy: Float,
            z: Float,
            rad: Float,
            thick: Float,
            segs: Int,
            r: Float,
            g: Float,
            b: Float,
            a: Float
        )

        fun pushFxGradientQuad(
            x0: Float,
            y0: Float,
            z0: Float,
            r0: Float,
            g0: Float,
            b0: Float,
            a0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            r1: Float,
            g1: Float,
            b1: Float,
            a1: Float,
            x2: Float,
            y2: Float,
            z2: Float,
            r2: Float,
            g2: Float,
            b2: Float,
            a2: Float,
            x3: Float,
            y3: Float,
            z3: Float,
            r3: Float,
            g3: Float,
            b3: Float,
            a3: Float
        )
    }

    /**
     * Projects a 3D world point `(wx, wy, wz)` into viewport pixel coordinates using column-major
     * [vpMatrix]. Writes `[screenPxX, screenPxY, clipW]` into [outScreenXYW] and returns `true`
     * when the point is in front of the camera (`clipW > 0.05f`).
     */
    fun projectWorldToScreen(
        vpMatrix: FloatArray,
        viewportW: Int,
        viewportH: Int,
        wx: Float,
        wy: Float,
        wz: Float,
        outScreenXYW: FloatArray
    ): Boolean {
        if (viewportW <= 0 || viewportH <= 0 || vpMatrix.size < 16 || outScreenXYW.size < 3) return false
        val w = vpMatrix[3] * wx + vpMatrix[7] * wy + vpMatrix[11] * wz + vpMatrix[15]
        outScreenXYW[2] = w
        if (w <= 0.05f) return false
        val invW = 1.0f / w
        val ndcX = (vpMatrix[0] * wx + vpMatrix[4] * wy + vpMatrix[8] * wz + vpMatrix[12]) * invW
        val ndcY = (vpMatrix[1] * wx + vpMatrix[5] * wy + vpMatrix[9] * wz + vpMatrix[13]) * invW
        outScreenXYW[0] = (ndcX * 0.5f + 0.5f) * viewportW.toFloat()
        outScreenXYW[1] = (0.5f - ndcY * 0.5f) * viewportH.toFloat()
        return true
    }

    /**
     * Estimates realistic mission completion ETA (in whole seconds) given total 3D route arc
     * length, waypoint count, and configured cruise speed limit.
     */
    fun computeMissionEtaSeconds(
        arcLengthMeters: Float,
        waypointCount: Int,
        maxVelMetersPerSec: Float
    ): Int {
        if (arcLengthMeters <= 0.05f || waypointCount <= 0) return 0
        val effectiveCruise = (maxVelMetersPerSec.coerceIn(0.4f, 12.0f) * 0.74f).coerceAtLeast(0.35f)
        val turnAllowanceSec = max(0, waypointCount - 1) * 0.85f + 0.65f
        return ceil((arcLengthMeters / effectiveCruise) + turnAllowanceSec).toInt().coerceAtLeast(1)
    }

    /**
     * Solves for a waypoint's new 3D altitude `z` (in meters, clamped to `[0.4, 4.0]`) as the user
     * drags vertically at [touchScreenY].
     *
     * In 3D perspective views (`blend3D >= 0.22`), intersects the horizontal screen scanline
     * `touchScreenY - grabOffsetScreenY` directly with the waypoint's vertical world axis `(wx, wy, z)`
     * so the 3D beacon tracks the user's finger 1:1 without perspective lag or initial jump.
     * In top-down 2D views, smoothly maps vertical screen motion to altitude at `85dp / meter`.
     */
    fun solveWaypointAltitudeFromScreenY(
        touchScreenY: Float,
        grabOffsetScreenY: Float,
        viewportH: Int,
        vpMatrix: FloatArray,
        zScale: Float,
        blend3D: Float,
        wx: Float,
        wy: Float,
        startAltitudeMeters: Float,
        downScreenY: Float,
        density: Float
    ): Float {
        val safeDensity = max(1f, density)
        val linearFallback = (startAltitudeMeters + (downScreenY - touchScreenY) / (85f * safeDensity)).coerceIn(0.4f, 4.0f)
        if (viewportH <= 0 || vpMatrix.size < 16 || blend3D < 0.22f) {
            return linearFallback
        }
        val targetScreenY = touchScreenY - grabOffsetScreenY
        val ndcY = 1.0f - 2.0f * (targetScreenY / viewportH.toFloat())
        val denom = ndcY * vpMatrix[11] - vpMatrix[9]
        if (abs(denom) <= 0.08f) {
            return linearFallback
        }
        val num = (vpMatrix[1] * wx + vpMatrix[5] * wy + vpMatrix[13]) -
            ndcY * (vpMatrix[3] * wx + vpMatrix[7] * wy + vpMatrix[15])
        val wzScaled = num / denom
        if (!wzScaled.isFinite()) return linearFallback
        val rawMeters = wzScaled / max(0.08f, zScale)
        return rawMeters.coerceIn(0.4f, 4.0f)
    }

    /**
     * Evaluates touch interaction against active 3D waypoints, distinguishing between:
     * 1. [WaypointHitMode.DRAG_ALTITUDE_Z] when grabbing the 3D vertical altitude slider handle (`▲/▼`)
     *    or projected digital altitude badge above the selected waypoint;
     * 2. [WaypointHitMode.SELECT_OR_DRAG_XY] when tapping/dragging the 3D beacon head, vertical stem,
     *    or ground pad.
     */
    fun pickWaypointInteraction(
        touchPxX: Float,
        touchPxY: Float,
        viewportW: Int,
        viewportH: Int,
        vpMatrix: FloatArray,
        zScale: Float,
        blend3D: Float,
        waypointPts: FloatArray,
        waypointCount: Int,
        selectedWaypointIdx: Int,
        maxPickRadiusPx: Float,
        density: Float,
        outResult: WaypointHitResult = WaypointHitResult()
    ): WaypointHitResult {
        outResult.reset()
        if (waypointCount <= 0 || viewportW <= 0 || viewportH <= 0 || vpMatrix.size < 16) return outResult
        val vw = viewportW.toFloat()
        val vh = viewportH.toFloat()
        val limit = min(waypointCount, waypointPts.size / 3)
        val safeDensity = max(1f, density)

        // 1. Check the selected waypoint's upper 3D vertical altitude slider handle & callout zone first
        if (selectedWaypointIdx in 0 until limit) {
            val sBase = selectedWaypointIdx * 3
            val sxW = waypointPts[sBase]
            val syW = waypointPts[sBase + 1]
            val szTop = waypointPts[sBase + 2] * zScale
            val wHead = vpMatrix[3] * sxW + vpMatrix[7] * syW + vpMatrix[11] * szTop + vpMatrix[15]
            if (wHead > 0.05f) {
                val invWHead = 1.0f / wHead
                val headPxX = ((vpMatrix[0] * sxW + vpMatrix[4] * syW + vpMatrix[8] * szTop + vpMatrix[12]) * invWHead * 0.5f + 0.5f) * vw
                val headPxY = (0.5f - (vpMatrix[1] * sxW + vpMatrix[5] * syW + vpMatrix[9] * szTop + vpMatrix[13]) * invWHead * 0.5f) * vh

                val knobWorldZ = szTop + 0.42f * (0.25f + 0.75f * blend3D)
                val wKnob = vpMatrix[3] * sxW + vpMatrix[7] * syW + vpMatrix[11] * knobWorldZ + vpMatrix[15]
                var handleTopPxX = headPxX
                var handleTopPxY = headPxY - 38f * safeDensity
                if (wKnob > 0.05f) {
                    val invWK = 1.0f / wKnob
                    val projKX = ((vpMatrix[0] * sxW + vpMatrix[4] * syW + vpMatrix[8] * knobWorldZ + vpMatrix[12]) * invWK * 0.5f + 0.5f) * vw
                    val projKY = (0.5f - (vpMatrix[1] * sxW + vpMatrix[5] * syW + vpMatrix[9] * knobWorldZ + vpMatrix[13]) * invWK * 0.5f) * vh
                    handleTopPxX = projKX
                    handleTopPxY = min(projKY - 16f * safeDensity, headPxY - 38f * safeDensity)
                }

                // Altitude handle zone starts slightly above the diamond center so grabbing the upper
                // slider knob or callout pulls Z, while grabbing the lower stem/ground pad drags XY.
                val startX = headPxX + (handleTopPxX - headPxX) * 0.22f
                val startY = headPxY - 9f * safeDensity
                val segX = handleTopPxX - startX
                val segY = handleTopPxY - startY
                val segLenSq = segX * segX + segY * segY
                val handleDistSq = if (segLenSq > 1.0f) {
                    val t = (((touchPxX - startX) * segX + (touchPxY - startY) * segY) / segLenSq).coerceIn(0f, 1f)
                    val cx = startX + segX * t
                    val cy = startY + segY * t
                    val dx = touchPxX - cx
                    val dy = touchPxY - cy
                    dx * dx + dy * dy
                } else {
                    val dx = touchPxX - handleTopPxX
                    val dy = touchPxY - handleTopPxY
                    dx * dx + dy * dy
                }
                val handleRadius = max(maxPickRadiusPx * 0.92f, 28f * safeDensity)
                if (handleDistSq <= handleRadius * handleRadius) {
                    outResult.index = selectedWaypointIdx
                    outResult.mode = WaypointHitMode.DRAG_ALTITUDE_Z
                    outResult.headScreenX = headPxX
                    outResult.headScreenY = headPxY
                    return outResult
                }
            }
        }

        // 2. Standard 3D beacon head + vertical stem + ground pad hit-testing
        val maxDistSq = maxPickRadiusPx * maxPickRadiusPx
        var bestIdx = -1
        var bestScoreSq = maxDistSq
        var bestHeadX = 0f
        var bestHeadY = 0f

        for (i in 0 until limit) {
            val base = i * 3
            val wx = waypointPts[base]
            val wy = waypointPts[base + 1]
            val wzTop = waypointPts[base + 2] * zScale

            val wHead = vpMatrix[3] * wx + vpMatrix[7] * wy + vpMatrix[11] * wzTop + vpMatrix[15]
            if (wHead <= 0.05f) continue
            val invWHead = 1.0f / wHead
            val headPxX = ((vpMatrix[0] * wx + vpMatrix[4] * wy + vpMatrix[8] * wzTop + vpMatrix[12]) * invWHead * 0.5f + 0.5f) * vw
            val headPxY = (0.5f - (vpMatrix[1] * wx + vpMatrix[5] * wy + vpMatrix[9] * wzTop + vpMatrix[13]) * invWHead * 0.5f) * vh

            val dxHead = touchPxX - headPxX
            val dyHead = touchPxY - headPxY
            val headDistSq = dxHead * dxHead + dyHead * dyHead

            var minDistSq = headDistSq
            val wBase = vpMatrix[3] * wx + vpMatrix[7] * wy + vpMatrix[11] * 0.02f + vpMatrix[15]
            if (wBase > 0.05f) {
                val invWBase = 1.0f / wBase
                val basePxX = ((vpMatrix[0] * wx + vpMatrix[4] * wy + vpMatrix[8] * 0.02f + vpMatrix[12]) * invWBase * 0.5f + 0.5f) * vw
                val basePxY = (0.5f - (vpMatrix[1] * wx + vpMatrix[5] * wy + vpMatrix[9] * 0.02f + vpMatrix[13]) * invWBase * 0.5f) * vh

                val segX = basePxX - headPxX
                val segY = basePxY - headPxY
                val segLenSq = segX * segX + segY * segY
                val stemDistSq = if (segLenSq > 1.0f) {
                    val t = (((touchPxX - headPxX) * segX + (touchPxY - headPxY) * segY) / segLenSq).coerceIn(0f, 1f)
                    val cx = headPxX + segX * t
                    val cy = headPxY + segY * t
                    val dx = touchPxX - cx
                    val dy = touchPxY - cy
                    dx * dx + dy * dy
                } else {
                    headDistSq
                }
                minDistSq = min(headDistSq * 0.90f, stemDistSq)
            }

            if (minDistSq <= bestScoreSq) {
                bestScoreSq = minDistSq
                bestIdx = i
                bestHeadX = headPxX
                bestHeadY = headPxY
            }
        }

        if (bestIdx >= 0) {
            outResult.index = bestIdx
            outResult.mode = WaypointHitMode.SELECT_OR_DRAG_XY
            outResult.headScreenX = bestHeadX
            outResult.headScreenY = bestHeadY
        }
        return outResult
    }

    /**
     * Convenience screen-space waypoint index picker (`head` + vertical stem + `ground pad`).
     */
    fun pickWaypointScreenSpace(
        touchPxX: Float,
        touchPxY: Float,
        viewportW: Int,
        viewportH: Int,
        vpMatrix: FloatArray,
        zScale: Float,
        waypointPts: FloatArray,
        waypointCount: Int,
        maxPickRadiusPx: Float
    ): Int {
        return pickWaypointInteraction(
            touchPxX = touchPxX,
            touchPxY = touchPxY,
            viewportW = viewportW,
            viewportH = viewportH,
            vpMatrix = vpMatrix,
            zScale = zScale,
            blend3D = 1.0f,
            waypointPts = waypointPts,
            waypointCount = waypointCount,
            selectedWaypointIdx = -1,
            maxPickRadiusPx = maxPickRadiusPx,
            density = 1.0f
        ).index
    }

    /**
     * Emits all 3D waypoint markers, 3D vertical altitude slider handles, and connecting planned
     * mission route geometry into [sink], writing the final route tip world coordinates into [outLastTipXYZ].
     */
    fun emitPlannedMissionRoute(
        snap: RenderSnapshot,
        zScale: Float,
        blend3D: Float,
        uavZoomScale: Float,
        showCurtain: Boolean,
        shaderClockSec: Float,
        spawnAnimIdx: Int,
        spawnAnimScale: Float,
        sink: GeometrySink,
        outLastTipXYZ: FloatArray,
        activeAltDragIdx: Int = -1
    ) {
        val uavScaledZ = snap.uavZ * zScale
        outLastTipXYZ[0] = snap.uavX
        outLastTipXYZ[1] = snap.uavY
        outLastTipXYZ[2] = uavScaledZ

        val count = min(snap.waypointQueueCount, RenderSnapshot.MAX_MISSION_WAYPOINTS)
        if (count <= 0) return

        val isStaged = snap.isRouteStagedOnly
        val originX = if (isStaged) snap.uavX else snap.goalX
        val originY = if (isStaged) snap.uavY else snap.goalY
        val originZ = if (isStaged) uavScaledZ else snap.goalZ * zScale

        val baseR = if (isStaged) 1.00f else 0.34f
        val baseG = if (isStaged) 0.79f else 0.78f
        val baseB = if (isStaged) 0.16f else 1.00f

        val widthScale = (0.84f + 0.16f * uavZoomScale).coerceIn(0.84f, 1.32f)
        val beaconZoomScale = (0.82f + 0.18f * uavZoomScale).coerceIn(0.82f, 1.36f)
        val wps = snap.waypointQueuePts
        val safeFlags = snap.waypointSafeFlags
        val columnTops = snap.waypointColumnTopZs
        val selectedIdx = snap.selectedWaypointIdx

        var prevX = originX
        var prevY = originY
        var prevZ = originZ
        var prevSafe = true
        var cumRouteArc = 0f

        for (w in 0 until count) {
            val idx3 = w * 3
            val animFactor = if (w == spawnAnimIdx) spawnAnimScale.coerceIn(0.15f, 1.20f) else 1.0f
            val wx = wps[idx3]
            val wy = wps[idx3 + 1]
            val rawZ = wps[idx3 + 2]
            val wz = rawZ * zScale * animFactor.coerceAtMost(1.0f)
            val wpSafe = safeFlags[w]

            // 1. Connecting Planned Route Leg (prev -> current waypoint)
            val isLeadInLeg = (w == 0)
            val legSafe = wpSafe && (isLeadInLeg || prevSafe)
            val legR = if (!legSafe) 1.00f else baseR
            val legG = if (!legSafe) 0.36f else baseG
            val legB = if (!legSafe) 0.28f else baseB
            val legHalfW = (if (isLeadInLeg) 0.024f else 0.036f) * widthScale
            val legAlpha = if (isLeadInLeg) 0.74f else 0.94f

            cumRouteArc = emitConnectingRouteLeg(
                x0 = prevX,
                y0 = prevY,
                z0 = prevZ,
                x1 = wx,
                y1 = wy,
                z1 = wz,
                halfW = legHalfW,
                r = legR,
                g = legG,
                b = legB,
                alpha = legAlpha,
                startArc = cumRouteArc,
                blend3D = blend3D,
                showCurtain = showCurtain,
                curtainAlpha = (if (isLeadInLeg) 0.08f else 0.14f) * blend3D,
                shaderClockSec = shaderClockSec,
                chevronScale = widthScale * (if (isLeadInLeg) 0.82f else 1.0f),
                sink = sink
            )

            // 2. 3D Waypoint Marker Pylon + Optional 3D Vertical Altitude Slider Gizmo
            val nextHeadingRad = if (w + 1 < count) {
                val nx = wps[(w + 1) * 3]
                val ny = wps[(w + 1) * 3 + 1]
                atan2(ny - wy, nx - wx)
            } else {
                atan2(wy - prevY, wx - prevX)
            }
            emitWaypointPylon3D(
                index = w,
                wx = wx,
                wy = wy,
                wz = wz,
                rawAltitudeMeters = rawZ,
                columnTopMeters = columnTops[w],
                zScale = zScale,
                blend3D = blend3D,
                isStaged = isStaged,
                isSafe = wpSafe,
                isSelected = (w == selectedIdx),
                isActivelyDraggingAlt = (w == activeAltDragIdx),
                headingRad = nextHeadingRad,
                beaconScale = beaconZoomScale * animFactor,
                shaderClockSec = shaderClockSec,
                sink = sink
            )

            prevX = wx
            prevY = wy
            prevZ = wz
            prevSafe = wpSafe
        }

        // 3. Closed-Loop Return Leg (WP_last -> WP_anchor)
        if (snap.routeClosedLoop && count >= 2) {
            val fx = snap.routeLoopAnchorX
            val fy = snap.routeLoopAnchorY
            val fz = snap.routeLoopAnchorZ * zScale
            emitConnectingRouteLeg(
                x0 = prevX,
                y0 = prevY,
                z0 = prevZ,
                x1 = fx,
                y1 = fy,
                z1 = fz,
                halfW = 0.025f * widthScale,
                r = baseR,
                g = baseG,
                b = baseB,
                alpha = 0.68f,
                startArc = cumRouteArc,
                blend3D = blend3D,
                showCurtain = showCurtain,
                curtainAlpha = 0.08f * blend3D,
                shaderClockSec = shaderClockSec,
                chevronScale = widthScale * 0.85f,
                sink = sink
            )
        }

        if (isStaged) {
            outLastTipXYZ[0] = prevX
            outLastTipXYZ[1] = prevY
            outLastTipXYZ[2] = prevZ
        }
    }

    private fun emitConnectingRouteLeg(
        x0: Float,
        y0: Float,
        z0: Float,
        x1: Float,
        y1: Float,
        z1: Float,
        halfW: Float,
        r: Float,
        g: Float,
        b: Float,
        alpha: Float,
        startArc: Float,
        blend3D: Float,
        showCurtain: Boolean,
        curtainAlpha: Float,
        shaderClockSec: Float,
        chevronScale: Float,
        sink: GeometrySink
    ): Float {
        val dx = x1 - x0
        val dy = y1 - y0
        val dz = z1 - z0
        val horizLen = hypot(dx, dy)
        val segLen = hypot(horizLen, dz)
        if (segLen <= 1e-4f) return startArc

        val endArc = startArc + segLen

        sink.pushActorQuadSeg3D(x0, y0, z0, x1, y1, z1, halfW, r, g, b, alpha, startArc, endArc)
        sink.pushActorLine(
            x0, y0, z0 + 0.004f,
            x1, y1, z1 + 0.004f,
            min(1f, r * 1.15f + 0.12f),
            min(1f, g * 1.15f + 0.12f),
            min(1f, b * 1.15f + 0.12f),
            min(1f, alpha + 0.06f),
            startArc,
            endArc
        )

        sink.pushGroundDashedSeg(x0, y0, 0.016f, x1, y1, 0.016f, r, g, b, 0.52f * alpha)

        if (blend3D > 0.06f) {
            if (showCurtain && curtainAlpha > 0.005f) {
                sink.pushFxGradientQuad(
                    x0, y0, 0.008f, r, g, b, 0.012f,
                    x1, y1, 0.008f, r, g, b, 0.012f,
                    x1, y1, z1, r, g, b, curtainAlpha,
                    x0, y0, z0, r, g, b, curtainAlpha
                )
            }
            val dropSteps = (horizLen / 1.45f).toInt().coerceIn(0, 6)
            for (s in 1..dropSteps) {
                val f = s.toFloat() / (dropSteps + 1).toFloat()
                val rx = x0 + dx * f
                val ry = y0 + dy * f
                val rz = z0 + dz * f
                sink.pushGroundLine(rx, ry, 0.012f, rx, ry, rz, r, g, b, 0.22f * blend3D)
            }
        }

        if (horizLen > 0.42f) {
            val invH = 1.0f / horizLen
            val ux = dx * invH
            val uy = dy * invH
            val px = -uy
            val py = ux
            val spacing = 0.95f
            val phase = (shaderClockSec * 0.90f) - floor(shaderClockSec * 0.90f / spacing) * spacing
            var distAlong = phase
            if (distAlong < 0.22f) distAlong += spacing * 0.5f
            val back = 0.15f * chevronScale
            val wing = 0.115f * chevronScale
            val chevHalfW = 0.018f * chevronScale
            while (distAlong < horizLen - 0.22f) {
                val frac = distAlong * invH
                val cx = x0 + dx * frac
                val cy = y0 + dy * frac
                val cz = z0 + dz * frac + 0.012f
                val backFrac = ((distAlong - back) * invH).coerceIn(0f, 1f)
                val bz = z0 + dz * backFrac + 0.012f
                sink.pushActorQuadSeg3D(
                    cx - ux * back + px * wing,
                    cy - uy * back + py * wing,
                    bz,
                    cx,
                    cy,
                    cz,
                    chevHalfW,
                    1.0f,
                    0.99f,
                    0.94f,
                    0.94f * alpha,
                    -1f,
                    -1f
                )
                sink.pushActorQuadSeg3D(
                    cx - ux * back - px * wing,
                    cy - uy * back - py * wing,
                    bz,
                    cx,
                    cy,
                    cz,
                    chevHalfW,
                    1.0f,
                    0.99f,
                    0.94f,
                    0.94f * alpha,
                    -1f,
                    -1f
                )
                distAlong += spacing
            }
        }

        return endArc
    }

    private fun emitWaypointPylon3D(
        index: Int,
        wx: Float,
        wy: Float,
        wz: Float,
        rawAltitudeMeters: Float,
        columnTopMeters: Float,
        zScale: Float,
        blend3D: Float,
        isStaged: Boolean,
        isSafe: Boolean,
        isSelected: Boolean,
        isActivelyDraggingAlt: Boolean,
        headingRad: Float,
        beaconScale: Float,
        shaderClockSec: Float,
        sink: GeometrySink
    ) {
        val mr: Float
        val mg: Float
        val mb: Float
        when {
            !isSafe -> {
                mr = 1.00f
                mg = 0.28f
                mb = 0.24f
            }
            index == 0 && isStaged -> {
                mr = 0.08f
                mg = 0.95f
                mb = 0.62f
            }
            isStaged -> {
                mr = 1.00f
                mg = 0.78f
                mb = 0.16f
            }
            else -> {
                mr = 0.34f
                mg = 0.80f
                mb = 1.00f
            }
        }

        val selBoost = if (isSelected || isActivelyDraggingAlt) 1.22f else 1.0f
        val rad = 0.125f * beaconScale * selBoost
        val halfH = 0.165f * beaconScale * selBoost
        // Freeze vertical bobbing while actively dragging Z so the 3D height ruler reads crystal-steady
        val bob = if (isActivelyDraggingAlt) 0f else sin(shaderClockSec * 3.1f + index * 0.85f) * 0.022f * blend3D
        val cz = wz + bob

        // 1. Vertical 3D altitude tether stem + 0.5m elevation ruler ticks
        if (blend3D > 0.05f && wz > 0.04f) {
            val stemTop = max(0.02f, cz - halfH * 0.78f)
            sink.pushActorQuadSeg3D(
                wx, wy, 0.012f,
                wx, wy, stemTop,
                (if (isActivelyDraggingAlt) 0.023f else 0.017f) * beaconScale,
                mr, mg, mb,
                if (isSelected || isActivelyDraggingAlt) 0.96f else 0.78f,
                0f, stemTop
            )
            val tickStepM = if (isSelected || isActivelyDraggingAlt) 0.25f else 0.50f
            val maxTickMeters = if (isActivelyDraggingAlt) 4.0f else rawAltitudeMeters
            val tickCount = (maxTickMeters / tickStepM).toInt().coerceIn(0, 16)
            for (t in 1..tickCount) {
                val meters = t * tickStepM
                val tz = meters * zScale
                if (tz < stemTop - 0.04f || isActivelyDraggingAlt) {
                    val isMajor = abs(meters - floor(meters + 1e-3f)) < 0.05f
                    val isHalf = abs((meters * 2f) - floor(meters * 2f + 1e-3f)) < 0.05f
                    val tickW = when {
                        isMajor -> 0.092f
                        isHalf -> 0.062f
                        else -> 0.038f
                    } * beaconScale
                    val tr = if (tz > cz) 0.30f else mr
                    val tg = if (tz > cz) 0.92f else mg
                    val tb = if (tz > cz) 1.00f else mb
                    val ta = (if (tz > cz) 0.44f else 0.72f) * blend3D
                    sink.pushActorLine(wx - tickW, wy, tz, wx + tickW, wy, tz, tr, tg, tb, ta, -1f, -1f)
                }
            }

            // Local obstacle-ceiling warning ring on the vertical pole when an obstacle lies beneath (wx, wy)
            if ((isSelected || isActivelyDraggingAlt) && columnTopMeters > 0.12f) {
                val ceilZ = columnTopMeters * zScale
                val aboveCeil = rawAltitudeMeters > columnTopMeters + 0.20f
                val cr = if (aboveCeil) 1.00f else 1.00f
                val cg = if (aboveCeil) 0.76f else 0.24f
                val cb = if (aboveCeil) 0.16f else 0.22f
                sink.pushActorRingXY(
                    wx, wy, ceilZ,
                    0.18f * beaconScale, 0.024f * beaconScale, 18,
                    cr, cg, cb, 0.88f * blend3D, -1f
                )
            }
        }

        // 2. Ground anchor pad, crosshair ticks & ordinal waypoint pips
        val groundRingR = 0.23f * beaconScale
        sink.pushActorDiscXY(wx, wy, 0.018f, 0.068f * beaconScale, 12, mr, mg, mb, 0.88f, -1f)
        sink.pushFxRingXY(wx, wy, 0.020f, groundRingR, 0.034f * beaconScale, 20, mr, mg, mb, 0.90f)
        if (index == 0 && isStaged) {
            sink.pushFxRingXY(wx, wy, 0.021f, groundRingR * 1.32f, 0.022f * beaconScale, 22, mr, mg, mb, 0.68f)
        }
        val crossOuter = groundRingR + 0.09f * beaconScale
        val crossInner = groundRingR * 0.62f
        sink.pushGroundLine(wx - crossOuter, wy, 0.020f, wx - crossInner, wy, 0.020f, mr, mg, mb, 0.74f)
        sink.pushGroundLine(wx + crossInner, wy, 0.020f, wx + crossOuter, wy, 0.020f, mr, mg, mb, 0.74f)
        sink.pushGroundLine(wx, wy - crossOuter, 0.020f, wx, wy - crossInner, 0.020f, mr, mg, mb, 0.74f)
        sink.pushGroundLine(wx, wy + crossInner, 0.020f, wx, wy + crossOuter, 0.020f, mr, mg, mb, 0.74f)

        val pipCount = min(index + 1, 8)
        val pipOrbitR = groundRingR + 0.065f * beaconScale
        for (p in 0 until pipCount) {
            val ang = (PI * 0.5) - p * (PI * 0.22)
            val px = wx + cos(ang).toFloat() * pipOrbitR
            val py = wy + sin(ang).toFloat() * pipOrbitR
            sink.pushActorDiscXY(px, py, 0.023f, 0.024f * beaconScale, 6, 1.0f, 0.98f, 0.92f, 0.92f, -1f)
        }

        // Pulsing selection halo reticle + active Z-drag laser reference plane
        if (isSelected || isActivelyDraggingAlt) {
            val pulse = 0.5f + 0.5f * sin(shaderClockSec * 6.2f)
            val selRad = (0.34f + pulse * 0.08f) * beaconScale
            sink.pushFxRingXY(wx, wy, 0.025f, selRad, 0.036f * beaconScale, 24, 0.16f, 0.96f, 1.0f, 0.95f)
            val elevRingMul = if (isActivelyDraggingAlt) 2.25f else (1.55f + pulse * 0.18f)
            sink.pushActorRingXY(
                wx, wy, cz,
                rad * elevRingMul,
                (if (isActivelyDraggingAlt) 0.030f else 0.024f) * beaconScale,
                24,
                0.20f, 0.98f, 1.0f,
                if (isActivelyDraggingAlt) 0.98f else 0.92f,
                -1f
            )
        }

        // 3. Shaded 8-face 3D Octahedral Waypoint Beacon + Equatorial Collar
        val rotYaw = headingRad + (if (isActivelyDraggingAlt) 0f else shaderClockSec * 0.55f * blend3D)
        val c0 = cos(rotYaw)
        val s0 = sin(rotYaw)
        val x0 = wx + c0 * rad
        val y0 = wy + s0 * rad
        val x1 = wx - s0 * rad
        val y1 = wy + c0 * rad
        val x2 = wx - c0 * rad
        val y2 = wy - s0 * rad
        val x3 = wx + s0 * rad
        val y3 = wy - c0 * rad

        val zTop = cz + halfH * (0.25f + 0.75f * blend3D)
        val zBot = max(0.025f, cz - halfH * (0.25f + 0.75f * blend3D))

        emitShadedFace(sink, wx, wy, zTop, x0, y0, cz, x1, y1, cz, mr, mg, mb, 1.06f)
        emitShadedFace(sink, wx, wy, zTop, x1, y1, cz, x2, y2, cz, mr, mg, mb, 0.88f)
        emitShadedFace(sink, wx, wy, zTop, x2, y2, cz, x3, y3, cz, mr, mg, mb, 0.70f)
        emitShadedFace(sink, wx, wy, zTop, x3, y3, cz, x0, y0, cz, mr, mg, mb, 0.94f)

        emitShadedFace(sink, wx, wy, zBot, x1, y1, cz, x0, y0, cz, mr, mg, mb, 0.62f)
        emitShadedFace(sink, wx, wy, zBot, x2, y2, cz, x1, y1, cz, mr, mg, mb, 0.52f)
        emitShadedFace(sink, wx, wy, zBot, x3, y3, cz, x2, y2, cz, mr, mg, mb, 0.44f)
        emitShadedFace(sink, wx, wy, zBot, x0, y0, cz, x3, y3, cz, mr, mg, mb, 0.56f)

        sink.pushActorLine(x0, y0, cz, x1, y1, cz, 1f, 1f, 1f, 0.86f, -1f, -1f)
        sink.pushActorLine(x1, y1, cz, x2, y2, cz, 1f, 1f, 1f, 0.86f, -1f, -1f)
        sink.pushActorLine(x2, y2, cz, x3, y3, cz, 1f, 1f, 1f, 0.86f, -1f, -1f)
        sink.pushActorLine(x3, y3, cz, x0, y0, cz, 1f, 1f, 1f, 0.86f, -1f, -1f)

        // 4. 3D Vertical Altitude Slider Rail & Dual-Chevron Pull Handle (▲ / ▼) on Selected Waypoint
        if ((isSelected || isActivelyDraggingAlt) && blend3D > 0.12f) {
            val railTopZ = if (isActivelyDraggingAlt) {
                max(zTop + 0.48f * blend3D, 4.0f * zScale)
            } else {
                zTop + 0.46f * beaconScale * blend3D
            }
            val hr = if (isActivelyDraggingAlt) 0.45f else 0.20f
            val hg = 0.98f
            val hb = 1.00f
            // Upper guide rod
            sink.pushActorQuadSeg3D(
                wx, wy, zTop,
                wx, wy, railTopZ,
                0.014f * beaconScale,
                hr, hg, hb,
                if (isActivelyDraggingAlt) 0.92f else 0.78f,
                -1f, -1f
            )

            // 3D Altitude Slider Knob perched above zTop (upward + downward 4-sided chevron cones)
            val knobCenterZ = zTop + 0.22f * beaconScale * blend3D
            val knobR = (if (isActivelyDraggingAlt) 0.088f else 0.072f) * beaconScale
            val coneH = (if (isActivelyDraggingAlt) 0.115f else 0.092f) * beaconScale * blend3D
            val gapZ = 0.022f * beaconScale * blend3D
            val kx0 = wx + knobR
            val ky0 = wy
            val kx1 = wx
            val ky1 = wy + knobR
            val kx2 = wx - knobR
            val ky2 = wy
            val kx3 = wx
            val ky3 = wy - knobR

            val upBaseZ = knobCenterZ + gapZ
            val upTipZ = upBaseZ + coneH
            emitShadedFace(sink, wx, wy, upTipZ, kx0, ky0, upBaseZ, kx1, ky1, upBaseZ, hr, hg, hb, 1.05f)
            emitShadedFace(sink, wx, wy, upTipZ, kx1, ky1, upBaseZ, kx2, ky2, upBaseZ, hr, hg, hb, 0.86f)
            emitShadedFace(sink, wx, wy, upTipZ, kx2, ky2, upBaseZ, kx3, ky3, upBaseZ, hr, hg, hb, 0.72f)
            emitShadedFace(sink, wx, wy, upTipZ, kx3, ky3, upBaseZ, kx0, ky0, upBaseZ, hr, hg, hb, 0.95f)

            val dnBaseZ = knobCenterZ - gapZ
            val dnTipZ = max(zTop + 0.01f, dnBaseZ - coneH * 0.78f)
            emitShadedFace(sink, wx, wy, dnTipZ, kx1, ky1, dnBaseZ, kx0, ky0, dnBaseZ, hr, hg, hb, 0.82f)
            emitShadedFace(sink, wx, wy, dnTipZ, kx2, ky2, dnBaseZ, kx1, ky1, dnBaseZ, hr, hg, hb, 0.68f)
            emitShadedFace(sink, wx, wy, dnTipZ, kx3, ky3, dnBaseZ, kx2, ky2, dnBaseZ, hr, hg, hb, 0.58f)
            emitShadedFace(sink, wx, wy, dnTipZ, kx0, ky0, dnBaseZ, kx3, ky3, dnBaseZ, hr, hg, hb, 0.75f)

            sink.pushActorRingXY(
                wx, wy, knobCenterZ,
                knobR * 1.18f, 0.018f * beaconScale, 16,
                1.0f, 1.0f, 1.0f, 0.92f * blend3D, -1f
            )
        }
    }

    private fun emitShadedFace(
        sink: GeometrySink,
        ax: Float,
        ay: Float,
        az: Float,
        bx: Float,
        by: Float,
        bz: Float,
        cx: Float,
        cy: Float,
        cz: Float,
        r: Float,
        g: Float,
        b: Float,
        shade: Float
    ) {
        sink.pushActorTri(
            ax, ay, az, -1f,
            bx, by, bz, -1f,
            cx, cy, cz, -1f,
            min(1f, r * shade),
            min(1f, g * shade),
            min(1f, b * shade),
            0.96f
        )
    }
}
