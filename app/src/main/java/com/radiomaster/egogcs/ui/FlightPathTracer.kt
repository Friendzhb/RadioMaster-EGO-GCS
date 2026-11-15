package com.radiomaster.egogcs.ui

import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Zero-allocation circular FIFO flight path tracer and 3D fading polyline builder.
 *
 * Maintains a bounded ring buffer of recent 3D drone positions (`O(1)` enqueue & oldest-point
 * eviction) and emits a view-aligned, width-tapered, per-vertex alpha-fading polyline ribbon
 * plus a zoom-resilient core filament directly into `GlPointCloudMapView`'s interleaved vertex
 * arrays (`x, y, z, r, g, b, a, arc`).
 */
class FlightPathTracer(
    capacity: Int = DEFAULT_MAX_POINTS,
    val minSampleStepMeters: Float = DEFAULT_MIN_SAMPLE_STEP_M,
    val teleportResetMeters: Float = DEFAULT_TELEPORT_RESET_M
) {
    companion object {
        const val DEFAULT_MAX_POINTS = 160
        const val SWARM_WINGMAN_MAX_POINTS = 96
        const val POWER_SAVE_MAX_POINTS = 80
        const val POWER_SAVE_WINGMAN_POINTS = 48

        const val DEFAULT_MIN_SAMPLE_STEP_M = 0.08f
        const val DEFAULT_TELEPORT_RESET_M = 9.5f

        private const val MIN_CAPACITY = 4
        private const val MAX_CAPACITY = 512
        private const val FLOATS_PER_VERT = 8
    }

    val maxPoints: Int = capacity.coerceIn(MIN_CAPACITY, MAX_CAPACITY)

    private val ringX = FloatArray(maxPoints)
    private val ringY = FloatArray(maxPoints)
    private val ringZ = FloatArray(maxPoints)
    private var head = 0

    var size: Int = 0
        private set

    // Pre-allocated scratch buffers (+1 slot for live sub-step drone tail bridge)
    private val pathX = FloatArray(maxPoints + 1)
    private val pathY = FloatArray(maxPoints + 1)
    private val pathZ = FloatArray(maxPoints + 1)
    private val cumArc = FloatArray(maxPoints + 1)
    private val segNx = FloatArray(maxPoints)
    private val segNy = FloatArray(maxPoints)
    private val segNz = FloatArray(maxPoints)

    private val miterNx = FloatArray(maxPoints + 1)
    private val miterNy = FloatArray(maxPoints + 1)
    private val miterNz = FloatArray(maxPoints + 1)
    private val nodeHalfW = FloatArray(maxPoints + 1)
    private val nodeFade = FloatArray(maxPoints + 1)
    private val nodeProg = FloatArray(maxPoints + 1)

    var lastTriVertCount: Int = 0
        private set
    var lastLineVertCount: Int = 0
        private set

    fun clear() {
        head = 0
        size = 0
        lastTriVertCount = 0
        lastLineVertCount = 0
    }

    /**
     * Seeds the FIFO buffer once from chronological `(x, y, z)` triples when empty.
     */
    fun seedIfEmpty(orderedPts: FloatArray, ptCount: Int) {
        if (size > 0 || ptCount <= 0) return
        val available = min(ptCount, orderedPts.size / 3)
        if (available <= 0) return
        val start = max(0, available - maxPoints)
        for (i in start until available) {
            val base = i * 3
            recordPose(orderedPts[base], orderedPts[base + 1], orderedPts[base + 2])
        }
    }

    /**
     * Samples a new 3D drone pose into the bounded FIFO ring buffer.
     *
     * @return `true` if the FIFO buffer accepted a new sample (or reset on teleport), `false` if
     * ignored due to non-finite values or sub-step stationary hovering.
     */
    fun recordPose(x: Float, y: Float, z: Float): Boolean {
        if (!x.isFinite() || !y.isFinite() || !z.isFinite()) return false

        if (size == 0) {
            ringX[0] = x
            ringY[0] = y
            ringZ[0] = z
            head = 0
            size = 1
            return true
        }

        val newestIdx = (head + size - 1) % maxPoints
        val dx = x - ringX[newestIdx]
        val dy = y - ringY[newestIdx]
        val dz = z - ringZ[newestIdx]
        val dist = sqrt(dx * dx + dy * dy + dz * dz)

        if (dist >= teleportResetMeters) {
            ringX[0] = x
            ringY[0] = y
            ringZ[0] = z
            head = 0
            size = 1
            return true
        }

        if (dist < minSampleStepMeters) {
            return false
        }

        if (size < maxPoints) {
            val writeIdx = (head + size) % maxPoints
            ringX[writeIdx] = x
            ringY[writeIdx] = y
            ringZ[writeIdx] = z
            size++
        } else {
            ringX[head] = x
            ringY[head] = y
            ringZ[head] = z
            head = (head + 1) % maxPoints
        }
        return true
    }

    /**
     * Reads the `logicalIndex`-th point (`0` = oldest, `size - 1` = newest) into [outXyz].
     */
    fun getPoint(logicalIndex: Int, outXyz: FloatArray) {
        require(logicalIndex in 0 until size) { "Index $logicalIndex out of bounds for size $size" }
        val idx = (head + logicalIndex) % maxPoints
        outXyz[0] = ringX[idx]
        outXyz[1] = ringY[idx]
        outXyz[2] = ringZ[idx]
    }

    /**
     * Emits a fading, width-tapered 3D polyline behind the drone into [triArray] and [lineArray].
     *
     * Updates [lastTriVertCount] and [lastLineVertCount] with the updated total vertex counts.
     */
    fun emitFadingPolyline(
        liveX: Float,
        liveY: Float,
        liveZ: Float,
        zScale: Float,
        eyeX: Float,
        eyeY: Float,
        eyeZ: Float,
        activePointLimit: Int,
        tailR: Float,
        tailG: Float,
        tailB: Float,
        headR: Float,
        headG: Float,
        headB: Float,
        maxAlpha: Float,
        headHalfWidth: Float,
        tailHalfWidth: Float,
        triArray: FloatArray,
        triVertStart: Int,
        maxTriVerts: Int,
        lineArray: FloatArray,
        lineVertStart: Int,
        maxLineVerts: Int
    ) {
        lastTriVertCount = triVertStart
        lastLineVertCount = lineVertStart
        if (size == 0 || maxAlpha <= 0.005f) return

        val clampedLimit = activePointLimit.coerceIn(2, maxPoints)
        val takeCount = min(size, clampedLimit)
        val startOffset = size - takeCount

        for (i in 0 until takeCount) {
            val ringIdx = (head + startOffset + i) % maxPoints
            pathX[i] = ringX[ringIdx]
            pathY[i] = ringY[ringIdx]
            pathZ[i] = max(0.016f, ringZ[ringIdx] * zScale)
        }

        var totalPts = takeCount
        if (liveX.isFinite() && liveY.isFinite() && liveZ.isFinite()) {
            val liveScaledZ = max(0.016f, liveZ * zScale)
            val lastIdx = takeCount - 1
            val dx = liveX - pathX[lastIdx]
            val dy = liveY - pathY[lastIdx]
            val dz = liveScaledZ - pathZ[lastIdx]
            val liveDist = sqrt(dx * dx + dy * dy + dz * dz)
            if (liveDist in 0.004f..teleportResetMeters) {
                pathX[takeCount] = liveX
                pathY[takeCount] = liveY
                pathZ[takeCount] = liveScaledZ
                totalPts = takeCount + 1
            } else if (liveDist <= 0.004f) {
                pathX[lastIdx] = liveX
                pathY[lastIdx] = liveY
                pathZ[lastIdx] = liveScaledZ
            }
        }

        if (totalPts < 2) return

        cumArc[0] = 0f
        val segCount = totalPts - 1
        for (i in 0 until segCount) {
            val sx = pathX[i + 1] - pathX[i]
            val sy = pathY[i + 1] - pathY[i]
            val sz = pathZ[i + 1] - pathZ[i]
            val len = sqrt(sx * sx + sy * sy + sz * sz)
            cumArc[i + 1] = cumArc[i] + len

            val mx = (pathX[i] + pathX[i + 1]) * 0.5f
            val my = (pathY[i] + pathY[i + 1]) * 0.5f
            val mz = (pathZ[i] + pathZ[i + 1]) * 0.5f
            val vx = eyeX - mx
            val vy = eyeY - my
            val vz = eyeZ - mz

            // 3D view-aligned billboard side vector: normalize(S x V)
            var cx = sy * vz - sz * vy
            var cy = sz * vx - sx * vz
            var cz = sx * vy - sy * vx
            var cLen = sqrt(cx * cx + cy * cy + cz * cz)
            if (cLen < 1e-5f) {
                val hLen = hypot(sx, sy)
                if (hLen > 1e-5f) {
                    cx = -sy / hLen
                    cy = sx / hLen
                    cz = 0f
                } else {
                    cx = 1f
                    cy = 0f
                    cz = 0f
                }
            } else {
                val inv = 1f / cLen
                cx *= inv
                cy *= inv
                cz *= inv
            }
            segNx[i] = cx
            segNy[i] = cy
            segNz[i] = cz
        }

        val totalArc = cumArc[segCount]
        if (totalArc < 0.012f) return

        val invTotalArc = 1f / totalArc
        val invLastIdx = 1f / segCount.toFloat()

        for (i in 0 until totalPts) {
            val arcFrac = cumArc[i] * invTotalArc
            val idxFrac = i.toFloat() * invLastIdx
            val u = (arcFrac * 0.70f + idxFrac * 0.30f).coerceIn(0f, 1f)
            // Smoothstep fade guarantees alpha(0) = 0 and zero slope at FIFO eviction boundary
            val fade = u * u * (3f - 2f * u)
            nodeProg[i] = u
            nodeFade[i] = fade
            nodeHalfW[i] = tailHalfWidth + (headHalfWidth - tailHalfWidth) * u

            when (i) {
                0 -> {
                    miterNx[0] = segNx[0]
                    miterNy[0] = segNy[0]
                    miterNz[0] = segNz[0]
                }
                segCount -> {
                    miterNx[i] = segNx[segCount - 1]
                    miterNy[i] = segNy[segCount - 1]
                    miterNz[i] = segNz[segCount - 1]
                }
                else -> {
                    var ax = segNx[i - 1] + segNx[i]
                    var ay = segNy[i - 1] + segNy[i]
                    var az = segNz[i - 1] + segNz[i]
                    val aLen = sqrt(ax * ax + ay * ay + az * az)
                    if (aLen > 1e-4f) {
                        val inv = 1f / aLen
                        ax *= inv
                        ay *= inv
                        az *= inv
                        val dot = (ax * segNx[i] + ay * segNy[i] + az * segNz[i]).coerceAtLeast(0.62f)
                        val miterScale = (1f / dot).coerceAtMost(1.60f)
                        miterNx[i] = ax * miterScale
                        miterNy[i] = ay * miterScale
                        miterNz[i] = az * miterScale
                    } else {
                        miterNx[i] = segNx[i]
                        miterNy[i] = segNy[i]
                        miterNz[i] = segNz[i]
                    }
                }
            }
        }

        var triCur = triVertStart
        var lineCur = lineVertStart

        for (i in 0 until segCount) {
            val j = i + 1
            val a0 = maxAlpha * nodeFade[i]
            val a1 = maxAlpha * nodeFade[j]
            if (a0 <= 0.002f && a1 <= 0.002f) continue

            val u0 = nodeProg[i]
            val u1 = nodeProg[j]
            val r0 = tailR + (headR - tailR) * u0
            val g0 = tailG + (headG - tailG) * u0
            val b0 = tailB + (headB - tailB) * u0
            val r1 = tailR + (headR - tailR) * u1
            val g1 = tailG + (headG - tailG) * u1
            val b1 = tailB + (headB - tailB) * u1

            // Bright hot-core tint for centerline spine & zoom-resilient filament
            val coreBoost0 = 0.34f * u0
            val cr0 = min(1f, r0 + (1f - r0) * coreBoost0)
            val cg0 = min(1f, g0 + (1f - g0) * coreBoost0)
            val cb0 = min(1f, b0 + (1f - b0) * coreBoost0)

            val coreBoost1 = 0.34f * u1
            val cr1 = min(1f, r1 + (1f - r1) * coreBoost1)
            val cg1 = min(1f, g1 + (1f - g1) * coreBoost1)
            val cb1 = min(1f, b1 + (1f - b1) * coreBoost1)

            val arc0 = cumArc[i]
            val arc1 = cumArc[j]

            val x0 = pathX[i]
            val y0 = pathY[i]
            val z0 = pathZ[i]
            val x1 = pathX[j]
            val y1 = pathY[j]
            val z1 = pathZ[j]

            // Center-spine feathered ribbon: outer edges interpolate to alpha = 0 for hardware-free AA
            if (triCur + 12 <= maxTriVerts) {
                val hw0 = nodeHalfW[i]
                val hw1 = nodeHalfW[j]
                val ox0 = miterNx[i] * hw0
                val oy0 = miterNy[i] * hw0
                val oz0 = miterNz[i] * hw0
                val ox1 = miterNx[j] * hw1
                val oy1 = miterNy[j] * hw1
                val oz1 = miterNz[j] * hw1

                val lx0 = x0 - ox0
                val ly0 = y0 - oy0
                val lz0 = z0 - oz0
                val rx0 = x0 + ox0
                val ry0 = y0 + oy0
                val rz0 = z0 + oz0

                val lx1 = x1 - ox1
                val ly1 = y1 - oy1
                val lz1 = z1 - oz1
                val rx1 = x1 + ox1
                val ry1 = y1 + oy1
                val rz1 = z1 + oz1

                val spineA0 = a0 * 0.84f
                val spineA1 = a1 * 0.84f

                // Left feathered half-ribbon (outer alpha = 0 -> center spine alpha = spineA)
                writeVert(triArray, triCur++, lx0, ly0, lz0, r0, g0, b0, 0f, arc0)
                writeVert(triArray, triCur++, x0, y0, z0, cr0, cg0, cb0, spineA0, arc0)
                writeVert(triArray, triCur++, x1, y1, z1, cr1, cg1, cb1, spineA1, arc1)

                writeVert(triArray, triCur++, lx0, ly0, lz0, r0, g0, b0, 0f, arc0)
                writeVert(triArray, triCur++, x1, y1, z1, cr1, cg1, cb1, spineA1, arc1)
                writeVert(triArray, triCur++, lx1, ly1, lz1, r1, g1, b1, 0f, arc1)

                // Right feathered half-ribbon (center spine alpha = spineA -> outer alpha = 0)
                writeVert(triArray, triCur++, x0, y0, z0, cr0, cg0, cb0, spineA0, arc0)
                writeVert(triArray, triCur++, rx0, ry0, rz0, r0, g0, b0, 0f, arc0)
                writeVert(triArray, triCur++, rx1, ry1, rz1, r1, g1, b1, 0f, arc1)

                writeVert(triArray, triCur++, x0, y0, z0, cr0, cg0, cb0, spineA0, arc0)
                writeVert(triArray, triCur++, rx1, ry1, rz1, r1, g1, b1, 0f, arc1)
                writeVert(triArray, triCur++, x1, y1, z1, cr1, cg1, cb1, spineA1, arc1)
            }

            if (lineCur + 2 <= maxLineVerts) {
                writeVert(lineArray, lineCur++, x0, y0, z0, cr0, cg0, cb0, a0, arc0)
                writeVert(lineArray, lineCur++, x1, y1, z1, cr1, cg1, cb1, a1, arc1)
            }
        }

        lastTriVertCount = triCur
        lastLineVertCount = lineCur
    }

    private fun writeVert(
        arr: FloatArray,
        vertIdx: Int,
        x: Float,
        y: Float,
        z: Float,
        r: Float,
        g: Float,
        b: Float,
        a: Float,
        arc: Float
    ) {
        var o = vertIdx * FLOATS_PER_VERT
        arr[o++] = x
        arr[o++] = y
        arr[o++] = z
        arr[o++] = r
        arr[o++] = g
        arr[o++] = b
        arr[o++] = a
        arr[o] = arc
    }
}
