package com.radiomaster.egogcs.ui

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Prepares GPU-ready interleaved VBO geometry (`px, py, pz, nx, ny, nz`, 24-byte stride) for the
 * `Friendzhb/tilt-uav-indi` 4M2S Yb-axis tilt-rotor UAV:
 * 1. Optimizes raw `vector_uav.glb` (~1.3M vertices) via octant-preserving spatial vertex clustering
 *    while extracting exact propeller shaft centers so M1..M4 spin around their true axes.
 * 2. Synthesizes a high-detail articulated 3D CAD fallback in the exact URDF meter-scale Z-up frame
 *    when the external 31 MB GLB binary is not bundled in the workspace.
 */
internal object VectorUavMeshBuilder {
    const val STRIDE_FLOATS = 6
    const val STRIDE_BYTES = STRIDE_FLOATS * 4
    const val ARM_SPAN_RADIUS = 0.18817742f
    private const val FRONT_PIVOT_X = 0.11667f
    private const val REAR_PIVOT_X = -0.11667f
    private const val PIVOT_Z = 0.056f
    private const val MOTOR_Y = 0.147644f
    private const val DECIMATE_THRESHOLD_VERTS = 4800
    private const val CLUSTER_GRID = 88

    class GpuPart(
        val name: String,
        val group: Int,
        val rotorIndex: Int,
        val transform: FloatArray,
        val firstVertex: Int,
        val vertexCount: Int,
        val color: FloatArray,
        val specular: Float = 0.32f
    )

    class RotorHub(
        val motorId: Int,
        val group: Int,
        val x: Float,
        val y: Float,
        val z: Float,
        val radius: Float,
        val spinSign: Int,
        val isFront: Boolean
    )

    class AuxSpan(val firstVertex: Int, val vertexCount: Int)

    class PreparedModel(
        val parts: List<GpuPart>,
        val frontPivot: FloatArray,
        val rearPivot: FloatArray,
        val rotorHubs: Array<RotorHub>,
        val vertexBuffer: FloatBuffer,
        val totalVertices: Int,
        val rotorDisc: AuxSpan,
        val rotorRing: AuxSpan,
        val rotorSweep: AuxSpan,
        val thrustShaft: AuxSpan,
        val thrustCone: AuxSpan,
        val noseChevron: AuxSpan,
        val isProceduralFallback: Boolean
    )

    private class FloatCollector(initialFloats: Int = 65536) {
        var data = FloatArray(initialFloats)
        var size = 0
        val vertexCount: Int get() = size / STRIDE_FLOATS

        private fun ensure(extra: Int) {
            if (size + extra <= data.size) return
            var next = data.size * 2
            while (next < size + extra) next *= 2
            data = data.copyOf(next)
        }

        fun pushVertex(px: Float, py: Float, pz: Float, nx: Float, ny: Float, nz: Float) {
            ensure(6)
            val d = data
            val o = size
            d[o] = px
            d[o + 1] = py
            d[o + 2] = pz
            d[o + 3] = nx
            d[o + 4] = ny
            d[o + 5] = nz
            size = o + 6
        }

        fun pushTri(
            x0: Float,
            y0: Float,
            z0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            x2: Float,
            y2: Float,
            z2: Float
        ) {
            val ux = x1 - x0
            val uy = y1 - y0
            val uz = z1 - z0
            val vx = x2 - x0
            val vy = y2 - y0
            val vz = z2 - z0
            var nx = uy * vz - uz * vy
            var ny = uz * vx - ux * vz
            var nz = ux * vy - uy * vx
            val len = sqrt(nx * nx + ny * ny + nz * nz)
            if (len < 1e-11f) return
            val inv = 1f / len
            nx *= inv
            ny *= inv
            nz *= inv
            pushVertex(x0, y0, z0, nx, ny, nz)
            pushVertex(x1, y1, z1, nx, ny, nz)
            pushVertex(x2, y2, z2, nx, ny, nz)
        }

        fun pushQuad(
            x0: Float,
            y0: Float,
            z0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            x2: Float,
            y2: Float,
            z2: Float,
            x3: Float,
            y3: Float,
            z3: Float
        ) {
            pushTri(x0, y0, z0, x1, y1, z1, x2, y2, z2)
            pushTri(x0, y0, z0, x2, y2, z2, x3, y3, z3)
        }
    }

    private fun defaultRotorHubs(frontPivot: FloatArray, rearPivot: FloatArray): Array<RotorHub> = arrayOf(
        RotorHub(1, 1, frontPivot[0], -MOTOR_Y, PIVOT_Z + 0.025f, 0.070f, 1, true),
        RotorHub(2, 2, rearPivot[0], MOTOR_Y, PIVOT_Z + 0.025f, 0.070f, 1, false),
        RotorHub(3, 1, frontPivot[0], MOTOR_Y, PIVOT_Z + 0.025f, 0.070f, -1, true),
        RotorHub(4, 2, rearPivot[0], -MOTOR_Y, PIVOT_Z + 0.025f, 0.070f, -1, false)
    )

    private fun detectRotorIndex(name: String): Int {
        val lower = name.lowercase()
        if (!lower.contains("prop")) return 0
        return when {
            lower.contains("m1") -> 1
            lower.contains("m2") -> 2
            lower.contains("m3") -> 3
            lower.contains("m4") -> 4
            else -> 0
        }
    }

    fun prepareFromGlb(parsed: VectorGlbAsset.Model, binarySlice: ByteBuffer): PreparedModel {
        val bin = binarySlice.duplicate().order(ByteOrder.LITTLE_ENDIAN)
        val collector = FloatCollector(262_144)
        val gpuParts = ArrayList<GpuPart>(parsed.parts.size)
        val hubs = defaultRotorHubs(parsed.frontPivot, parsed.rearPivot)

        val propMinX = FloatArray(4) { Float.POSITIVE_INFINITY }
        val propMaxX = FloatArray(4) { Float.NEGATIVE_INFINITY }
        val propMinY = FloatArray(4) { Float.POSITIVE_INFINITY }
        val propMaxY = FloatArray(4) { Float.NEGATIVE_INFINITY }
        val propMaxZ = FloatArray(4) { Float.NEGATIVE_INFINITY }

        for (part in parsed.parts) {
            val rIdx = detectRotorIndex(part.name)
            if (rIdx in 1..4) {
                val m = part.transform
                val slot = rIdx - 1
                var pOff = part.positionOffset
                val step = max(1, part.vertexCount / 2048)
                var i = 0
                while (i < part.vertexCount) {
                    val o = pOff + i * 12
                    val lx = bin.getFloat(o)
                    val ly = bin.getFloat(o + 4)
                    val lz = bin.getFloat(o + 8)
                    val bx = m[0] * lx + m[4] * ly + m[8] * lz + m[12]
                    val by = m[1] * lx + m[5] * ly + m[9] * lz + m[13]
                    val bz = m[2] * lx + m[6] * ly + m[10] * lz + m[14]
                    if (bx < propMinX[slot]) propMinX[slot] = bx
                    if (bx > propMaxX[slot]) propMaxX[slot] = bx
                    if (by < propMinY[slot]) propMinY[slot] = by
                    if (by > propMaxY[slot]) propMaxY[slot] = by
                    if (bz > propMaxZ[slot]) propMaxZ[slot] = bz
                    i += step
                }
            }
        }

        for (slot in 0..3) {
            if (propMinX[slot].isFinite() && propMaxX[slot].isFinite()) {
                val cx = (propMinX[slot] + propMaxX[slot]) * 0.5f
                val cy = (propMinY[slot] + propMaxY[slot]) * 0.5f
                val rx = (propMaxX[slot] - propMinX[slot]) * 0.5f
                val ry = (propMaxY[slot] - propMinY[slot]) * 0.5f
                val radius = max(rx, ry).coerceIn(0.042f, 0.115f)
                val prev = hubs[slot]
                hubs[slot] = RotorHub(
                    motorId = prev.motorId,
                    group = prev.group,
                    x = cx,
                    y = cy,
                    z = if (propMaxZ[slot].isFinite()) propMaxZ[slot] else prev.z,
                    radius = radius,
                    spinSign = prev.spinSign,
                    isFront = prev.isFront
                )
            }
        }

        val clusterMap = IntHashTable(32_768)
        for (part in parsed.parts) {
            val startVert = collector.vertexCount
            val rIdx = detectRotorIndex(part.name)
            if (part.vertexCount > DECIMATE_THRESHOLD_VERTS) {
                decimatePart(bin, part, collector, clusterMap)
                if (collector.vertexCount == startVert) {
                    copyPartRaw(bin, part, collector)
                }
            } else {
                copyPartRaw(bin, part, collector)
            }
            val count = collector.vertexCount - startVert
            if (count > 0) {
                val spec = when {
                    rIdx > 0 -> 0.46f
                    part.name.contains("motor", ignoreCase = true) -> 0.52f
                    else -> 0.30f
                }
                gpuParts.add(
                    GpuPart(
                        name = part.name,
                        group = part.group,
                        rotorIndex = rIdx,
                        transform = part.transform.copyOf(),
                        firstVertex = startVert,
                        vertexCount = count,
                        color = enhanceGlbMaterialColor(part.name, part.group, part.color),
                        specular = spec
                    )
                )
            }
        }

        return finalizeModel(
            parts = gpuParts,
            frontPivot = parsed.frontPivot.copyOf(),
            rearPivot = parsed.rearPivot.copyOf(),
            rotorHubs = hubs,
            collector = collector,
            isProceduralFallback = false
        )
    }

    private fun enhanceGlbMaterialColor(name: String, group: Int, src: FloatArray): FloatArray {
        val out = src.copyOf()
        val luma = 0.2126f * out[0] + 0.7152f * out[1] + 0.0722f * out[2]
        val lower = name.lowercase()
        if (lower.contains("prop")) {
            if (luma < 0.35f) {
                if (group == 1) {
                    out[0] = 0.86f
                    out[1] = 0.36f
                    out[2] = 0.30f
                } else {
                    out[0] = 0.22f
                    out[1] = 0.82f
                    out[2] = 0.54f
                }
            }
        } else if (lower.contains("motor") && luma < 0.22f) {
            out[0] = 0.30f
            out[1] = 0.36f
            out[2] = 0.45f
        } else if (luma < 0.16f) {
            out[0] = max(out[0], 0.20f)
            out[1] = max(out[1], 0.24f)
            out[2] = max(out[2], 0.30f)
        }
        return out
    }

    private fun copyPartRaw(bin: ByteBuffer, part: VectorGlbAsset.Part, out: FloatCollector) {
        val pBase = part.positionOffset
        val nBase = part.normalOffset
        for (i in 0 until part.vertexCount) {
            val po = pBase + i * 12
            val no = nBase + i * 12
            var nx = bin.getFloat(no)
            var ny = bin.getFloat(no + 4)
            var nz = bin.getFloat(no + 8)
            val len = sqrt(nx * nx + ny * ny + nz * nz)
            if (len > 1e-6f) {
                val inv = 1f / len
                nx *= inv
                ny *= inv
                nz *= inv
            } else {
                nz = 1f
            }
            out.pushVertex(bin.getFloat(po), bin.getFloat(po + 4), bin.getFloat(po + 8), nx, ny, nz)
        }
    }

    private fun decimatePart(
        bin: ByteBuffer,
        part: VectorGlbAsset.Part,
        out: FloatCollector,
        map: IntHashTable
    ) {
        val pBase = part.positionOffset
        val nBase = part.normalOffset
        val vCount = part.vertexCount
        var minX = Float.POSITIVE_INFINITY
        var minY = Float.POSITIVE_INFINITY
        var minZ = Float.POSITIVE_INFINITY
        var maxX = Float.NEGATIVE_INFINITY
        var maxY = Float.NEGATIVE_INFINITY
        var maxZ = Float.NEGATIVE_INFINITY

        for (i in 0 until vCount) {
            val po = pBase + i * 12
            val x = bin.getFloat(po)
            val y = bin.getFloat(po + 4)
            val z = bin.getFloat(po + 8)
            if (x < minX) minX = x
            if (x > maxX) maxX = x
            if (y < minY) minY = y
            if (y > maxY) maxY = y
            if (z < minZ) minZ = z
            if (z > maxZ) maxZ = z
        }

        val scaleX = (CLUSTER_GRID - 1) / max(1e-5f, maxX - minX)
        val scaleY = (CLUSTER_GRID - 1) / max(1e-5f, maxY - minY)
        val scaleZ = (CLUSTER_GRID - 1) / max(1e-5f, maxZ - minZ)

        map.clear()
        var accCap = 4096
        var sumFloats = FloatArray(accCap * 6)
        var counts = IntArray(accCap)
        var clusterCount = 0

        fun clusterOf(vIdx: Int): Int {
            val po = pBase + vIdx * 12
            val no = nBase + vIdx * 12
            val px = bin.getFloat(po)
            val py = bin.getFloat(po + 4)
            val pz = bin.getFloat(po + 8)
            val nx = bin.getFloat(no)
            val ny = bin.getFloat(no + 4)
            val nz = bin.getFloat(no + 8)
            val gx = ((px - minX) * scaleX).toInt().coerceIn(0, CLUSTER_GRID - 1)
            val gy = ((py - minY) * scaleY).toInt().coerceIn(0, CLUSTER_GRID - 1)
            val gz = ((pz - minZ) * scaleZ).toInt().coerceIn(0, CLUSTER_GRID - 1)
            val oct = (if (nx >= 0f) 1 else 0) or (if (ny >= 0f) 2 else 0) or (if (nz >= 0f) 4 else 0)
            val key = (((gx * CLUSTER_GRID + gy) * CLUSTER_GRID + gz) shl 3) or oct
            var cid = map.get(key)
            if (cid < 0) {
                cid = clusterCount++
                map.put(key, cid)
                if (cid >= accCap) {
                    accCap *= 2
                    sumFloats = sumFloats.copyOf(accCap * 6)
                    counts = counts.copyOf(accCap)
                }
            }
            val b = cid * 6
            sumFloats[b] += px
            sumFloats[b + 1] += py
            sumFloats[b + 2] += pz
            sumFloats[b + 3] += nx
            sumFloats[b + 4] += ny
            sumFloats[b + 5] += nz
            counts[cid]++
            return cid
        }

        val triClusters = IntArray(vCount)
        for (i in 0 until vCount) {
            triClusters[i] = clusterOf(i)
        }

        for (c in 0 until clusterCount) {
            val b = c * 6
            val invCnt = 1f / max(1, counts[c])
            sumFloats[b] *= invCnt
            sumFloats[b + 1] *= invCnt
            sumFloats[b + 2] *= invCnt
            val nx = sumFloats[b + 3]
            val ny = sumFloats[b + 4]
            val nz = sumFloats[b + 5]
            val nLen = sqrt(nx * nx + ny * ny + nz * nz)
            if (nLen > 1e-6f) {
                val invN = 1f / nLen
                sumFloats[b + 3] = nx * invN
                sumFloats[b + 4] = ny * invN
                sumFloats[b + 5] = nz * invN
            } else {
                sumFloats[b + 3] = 0f
                sumFloats[b + 4] = 0f
                sumFloats[b + 5] = 1f
            }
        }

        var t = 0
        while (t + 2 < vCount) {
            val c0 = triClusters[t]
            val c1 = triClusters[t + 1]
            val c2 = triClusters[t + 2]
            if (c0 != c1 && c1 != c2 && c2 != c0) {
                val b0 = c0 * 6
                val b1 = c1 * 6
                val b2 = c2 * 6
                out.pushVertex(sumFloats[b0], sumFloats[b0 + 1], sumFloats[b0 + 2], sumFloats[b0 + 3], sumFloats[b0 + 4], sumFloats[b0 + 5])
                out.pushVertex(sumFloats[b1], sumFloats[b1 + 1], sumFloats[b1 + 2], sumFloats[b1 + 3], sumFloats[b1 + 4], sumFloats[b1 + 5])
                out.pushVertex(sumFloats[b2], sumFloats[b2 + 1], sumFloats[b2 + 2], sumFloats[b2 + 3], sumFloats[b2 + 4], sumFloats[b2 + 5])
            }
            t += 3
        }
    }

    /**
     * Builds a solid, articulated 3D CAD mesh for the `Friendzhb/tilt-uav-indi` 4M2S vector quadrotor
     * in exact URDF meter-scale Z-up coordinates (`armSpanRadius = 0.18817742m`, pivots at `x = ±0.11667m`).
     */
    fun buildProceduralTiltUav(): PreparedModel {
        val frontPivot = floatArrayOf(FRONT_PIVOT_X, 0f, PIVOT_Z)
        val rearPivot = floatArrayOf(REAR_PIVOT_X, 0f, PIVOT_Z)
        val hubs = defaultRotorHubs(frontPivot, rearPivot)
        val collector = FloatCollector(98_304)
        val parts = ArrayList<GpuPart>(16)
        val idMat = floatArrayOf(
            1f, 0f, 0f, 0f,
            0f, 1f, 0f, 0f,
            0f, 0f, 1f, 0f,
            0f, 0f, 0f, 1f
        )

        fun part(name: String, group: Int, rotorIdx: Int, r: Float, g: Float, b: Float, spec: Float, build: () -> Unit) {
            val start = collector.vertexCount
            build()
            val cnt = collector.vertexCount - start
            if (cnt > 0) {
                parts.add(GpuPart(name, group, rotorIdx, idMat.copyOf(), start, cnt, floatArrayOf(r, g, b, 1f), spec))
            }
        }

        // 1. Matte carbon-fiber fuselage hull, tubular X-arms, diagonal trusses & twin landing skids
        part("fuselage_carbon_visual", 0, 0, 0.22f, 0.26f, 0.32f, 0.28f) {
            addOctagonalHull(collector, -0.088f, 0.098f, 0.048f, 0.033f, 0.068f)
            val roots = arrayOf(
                floatArrayOf(0.050f, -0.036f, FRONT_PIVOT_X, -0.118f),
                floatArrayOf(-0.050f, 0.036f, REAR_PIVOT_X, 0.118f),
                floatArrayOf(0.050f, 0.036f, FRONT_PIVOT_X, 0.118f),
                floatArrayOf(-0.050f, -0.036f, REAR_PIVOT_X, -0.118f)
            )
            for (a in roots) {
                addTube(collector, a[0], a[1], 0.053f, a[2], a[3], PIVOT_Z, 0.0068f, 8)
                addTube(collector, a[0] * 0.86f, a[1] * 0.86f, 0.036f, a[2] * 0.90f, a[3] * 0.88f, PIVOT_Z - 0.005f, 0.0042f, 6)
            }
            for (side in intArrayOf(-1, 1)) {
                val sy = side * 0.066f
                val skidZ = -0.010f
                addTube(collector, -0.082f, sy, skidZ, 0.086f, sy, skidZ, 0.0044f, 8)
                addTube(collector, 0.086f, sy, skidZ, 0.104f, sy, skidZ + 0.014f, 0.0040f, 6)
                addTube(collector, -0.082f, sy, skidZ, -0.094f, sy, skidZ + 0.010f, 0.0038f, 6)
                addTube(collector, 0.044f, side * 0.038f, 0.035f, 0.052f, sy, skidZ, 0.0042f, 6)
                addTube(collector, -0.044f, side * 0.038f, 0.035f, -0.052f, sy, skidZ, 0.0042f, 6)
            }
            addBox(collector, -0.054f, 0.048f, -0.026f, 0.026f, 0.013f, 0.033f)
        }

        // 2. Anodized titanium upper spine, CNC boom collars, S1/S2 servo pods & stereo camera housing
        part("avionics_titanium_visual", 0, 0, 0.46f, 0.53f, 0.63f, 0.52f) {
            addOctagonalHull(collector, -0.064f, 0.074f, 0.032f, 0.068f, 0.076f)
            addBox(collector, FRONT_PIVOT_X - 0.015f, FRONT_PIVOT_X + 0.015f, -0.022f, 0.022f, 0.045f, 0.067f)
            addBox(collector, REAR_PIVOT_X - 0.015f, REAR_PIVOT_X + 0.015f, -0.022f, 0.022f, 0.045f, 0.067f)
            addTube(collector, 0.072f, 0f, 0.056f, FRONT_PIVOT_X - 0.014f, 0f, PIVOT_Z, 0.0056f, 8)
            addTube(collector, -0.070f, 0f, 0.056f, REAR_PIVOT_X + 0.014f, 0f, PIVOT_Z, 0.0056f, 8)
            for (sx in floatArrayOf(FRONT_PIVOT_X, REAR_PIVOT_X)) {
                for (sy in floatArrayOf(-0.118f, 0.118f)) {
                    addTube(collector, sx, sy - 0.009f, PIVOT_Z, sx, sy + 0.009f, PIVOT_Z, 0.0092f, 10)
                }
            }
            addBox(collector, 0.092f, 0.113f, -0.030f, 0.030f, 0.043f, 0.061f)
            addCylinderZ(collector, -0.006f, 0f, 0.075f, 0.084f, 0.028f, 0.027f, 14)
            addCylinderZ(collector, -0.006f, 0f, 0.101f, 0.111f, 0.026f, 0.021f, 14)
        }

        // 3. Cyan-sapphire Livox Mid-360 optical emitter window, stereo camera lenses & forward nose chevron
        part("sensor_optics_visual", 0, 0, 0.10f, 0.82f, 0.98f, 0.72f) {
            addCylinderZ(collector, -0.006f, 0f, 0.084f, 0.101f, 0.0268f, 0.0258f, 14)
            addTube(collector, 0.111f, -0.017f, 0.052f, 0.116f, -0.017f, 0.052f, 0.0058f, 10)
            addTube(collector, 0.111f, 0.017f, 0.052f, 0.116f, 0.017f, 0.052f, 0.0058f, 10)
            collector.pushTri(0.070f, 0f, 0.0775f, 0.036f, 0.020f, 0.0775f, 0.046f, 0f, 0.0775f)
            collector.pushTri(0.070f, 0f, 0.0775f, 0.046f, 0f, 0.0775f, 0.036f, -0.020f, 0.0775f)
        }

        // 4. Anodized gold servo linkage horns & battery retention strap
        part("servo_gold_accent_visual", 0, 0, 0.96f, 0.68f, 0.10f, 0.55f) {
            addBox(collector, -0.010f, 0.006f, -0.028f, 0.028f, 0.012f, 0.034f)
            addBox(collector, FRONT_PIVOT_X - 0.005f, FRONT_PIVOT_X + 0.005f, -0.026f, 0.026f, 0.052f, 0.070f)
            addBox(collector, REAR_PIVOT_X - 0.005f, REAR_PIVOT_X + 0.005f, -0.026f, 0.026f, 0.052f, 0.070f)
        }

        // 5. Articulated front (group 1) & rear (group 2) motor nacelles, ducts, navigation arcs & 3-blade props
        for (grp in 1..2) {
            val px = if (grp == 1) FRONT_PIVOT_X else REAR_PIVOT_X
            val isFront = grp == 1
            val suffix = if (isFront) "f" else "b"
            part("tilt_shaft_${suffix}_visual", grp, 0, 0.56f, 0.63f, 0.72f, 0.56f) {
                addTube(collector, px, -MOTOR_Y, PIVOT_Z, px, MOTOR_Y, PIVOT_Z, 0.0048f, 10)
            }

            val groupHubs = hubs.filter { it.group == grp }
            for (h in groupHubs) {
                val mTag = "m${h.motorId}"
                part("motor_${mTag}_visual", grp, 0, 0.31f, 0.37f, 0.46f, 0.54f) {
                    addBox(collector, h.x - 0.014f, h.x + 0.014f, h.y - 0.014f, h.y + 0.014f, PIVOT_Z - 0.006f, PIVOT_Z + 0.005f)
                    addCylinderZ(collector, h.x, h.y, PIVOT_Z + 0.004f, PIVOT_Z + 0.021f, 0.0165f, 0.0158f, 14)
                    addCylinderZ(collector, h.x, h.y, PIVOT_Z + 0.021f, PIVOT_Z + 0.033f, 0.0062f, 0.0018f, 10)
                    addShroudRing(collector, h.x, h.y, PIVOT_Z + 0.012f, PIVOT_Z + 0.025f, h.radius * 1.07f, 0.0036f, 0f, (PI * 2.0).toFloat(), 20)
                    for (st in 0..2) {
                        val ang = (st * 2.0 * PI / 3.0 + if (h.y > 0f) 0.52 else -0.52).toFloat()
                        addTube(
                            collector,
                            h.x + cos(ang) * 0.014f,
                            h.y + sin(ang) * 0.014f,
                            PIVOT_Z + 0.010f,
                            h.x + cos(ang) * (h.radius * 1.06f),
                            h.y + sin(ang) * (h.radius * 1.06f),
                            PIVOT_Z + 0.016f,
                            0.0024f,
                            6
                        )
                    }
                }

                part("stator_${mTag}_visual", grp, 0, 0.88f, 0.48f, 0.16f, 0.62f) {
                    addCylinderZ(collector, h.x, h.y, PIVOT_Z + 0.007f, PIVOT_Z + 0.017f, 0.0170f, 0.0170f, 12)
                }

                val navR = if (isFront) 1.00f else 0.06f
                val navG = if (isFront) 0.28f else 0.90f
                val navB = if (isFront) 0.22f else 0.50f
                val arcCenter = if (isFront) 0f else PI.toFloat()
                part("nav_arc_${mTag}_visual", grp, 0, navR, navG, navB, 0.48f) {
                    addShroudRing(
                        collector,
                        h.x,
                        h.y,
                        PIVOT_Z + 0.015f,
                        PIVOT_Z + 0.027f,
                        h.radius * 1.095f,
                        0.0040f,
                        arcCenter - 1.25f,
                        arcCenter + 1.25f,
                        12
                    )
                }

                val bladeR = if (isFront) 0.88f else 0.74f
                val bladeG = if (isFront) 0.82f else 0.88f
                val bladeB = if (isFront) 0.84f else 0.84f
                part("propeller_${mTag}_visual", grp, h.motorId, bladeR, bladeG, bladeB, 0.50f) {
                    addCylinderZ(collector, h.x, h.y, h.z - 0.003f, h.z + 0.004f, 0.0085f, 0.0072f, 10)
                    addThreeBladePropeller(collector, h.x, h.y, h.z, h.radius * 0.98f, h.spinSign)
                }
            }
        }

        return finalizeModel(
            parts = parts,
            frontPivot = frontPivot,
            rearPivot = rearPivot,
            rotorHubs = hubs,
            collector = collector,
            isProceduralFallback = true
        )
    }

    private fun finalizeModel(
        parts: List<GpuPart>,
        frontPivot: FloatArray,
        rearPivot: FloatArray,
        rotorHubs: Array<RotorHub>,
        collector: FloatCollector,
        isProceduralFallback: Boolean
    ): PreparedModel {
        val discSpan = appendAuxRotorDisc(collector)
        val ringSpan = appendAuxRotorRing(collector)
        val sweepSpan = appendAuxRotorSweep(collector)
        val shaftSpan = appendAuxUnitCylinderZ(collector)
        val coneSpan = appendAuxUnitConeZ(collector)
        val chevronSpan = appendAuxNoseChevron(collector)

        val totalVerts = collector.vertexCount
        val byteBuf = ByteBuffer.allocateDirect(collector.size * 4).order(ByteOrder.nativeOrder())
        val floatBuf = byteBuf.asFloatBuffer()
        floatBuf.put(collector.data, 0, collector.size)
        floatBuf.position(0)

        return PreparedModel(
            parts = parts,
            frontPivot = frontPivot,
            rearPivot = rearPivot,
            rotorHubs = rotorHubs,
            vertexBuffer = floatBuf,
            totalVertices = totalVerts,
            rotorDisc = discSpan,
            rotorRing = ringSpan,
            rotorSweep = sweepSpan,
            thrustShaft = shaftSpan,
            thrustCone = coneSpan,
            noseChevron = chevronSpan,
            isProceduralFallback = isProceduralFallback
        )
    }

    private fun addBox(
        out: FloatCollector,
        x0: Float,
        x1: Float,
        y0: Float,
        y1: Float,
        z0: Float,
        z1: Float
    ) {
        out.pushQuad(x0, y0, z1, x1, y0, z1, x1, y1, z1, x0, y1, z1)
        out.pushQuad(x0, y1, z0, x1, y1, z0, x1, y0, z0, x0, y0, z0)
        out.pushQuad(x1, y0, z0, x1, y1, z0, x1, y1, z1, x1, y0, z1)
        out.pushQuad(x0, y1, z0, x0, y0, z0, x0, y0, z1, x0, y1, z1)
        out.pushQuad(x0, y1, z0, x1, y1, z0, x1, y1, z1, x0, y1, z1)
        out.pushQuad(x1, y0, z0, x0, y0, z0, x0, y0, z1, x1, y0, z1)
    }

    private fun addOctagonalHull(
        out: FloatCollector,
        backX: Float,
        frontX: Float,
        halfY: Float,
        zBot: Float,
        zTop: Float
    ) {
        val midF = frontX - (frontX - backX) * 0.24f
        val midB = backX + (frontX - backX) * 0.22f
        val tipY = halfY * 0.46f
        val ring = arrayOf(
            floatArrayOf(frontX, -tipY),
            floatArrayOf(frontX, tipY),
            floatArrayOf(midF, halfY),
            floatArrayOf(midB, halfY * 0.94f),
            floatArrayOf(backX, tipY * 0.92f),
            floatArrayOf(backX, -tipY * 0.92f),
            floatArrayOf(midB, -halfY * 0.94f),
            floatArrayOf(midF, -halfY)
        )
        val cx = (frontX + backX) * 0.5f
        for (i in ring.indices) {
            val nxt = (i + 1) % ring.size
            val x0 = ring[i][0]
            val y0 = ring[i][1]
            val x1 = ring[nxt][0]
            val y1 = ring[nxt][1]
            val bx0 = cx + (x0 - cx) * 0.90f
            val by0 = y0 * 0.90f
            val bx1 = cx + (x1 - cx) * 0.90f
            val by1 = y1 * 0.90f
            out.pushTri(cx, 0f, zTop, x0, y0, zTop, x1, y1, zTop)
            out.pushTri(cx, 0f, zBot, bx1, by1, zBot, bx0, by0, zBot)
            out.pushQuad(bx0, by0, zBot, bx1, by1, zBot, x1, y1, zTop, x0, y0, zTop)
        }
    }

    private fun addCylinderZ(
        out: FloatCollector,
        cx: Float,
        cy: Float,
        z0: Float,
        z1: Float,
        r0: Float,
        r1: Float,
        segs: Int
    ) {
        val step = (PI * 2.0 / segs).toFloat()
        for (i in 0 until segs) {
            val a0 = i * step
            val a1 = (i + 1) * step
            val c0 = cos(a0)
            val s0 = sin(a0)
            val c1 = cos(a1)
            val s1 = sin(a1)
            out.pushQuad(
                cx + c0 * r0, cy + s0 * r0, z0,
                cx + c1 * r0, cy + s1 * r0, z0,
                cx + c1 * r1, cy + s1 * r1, z1,
                cx + c0 * r1, cy + s0 * r1, z1
            )
            if (r1 > 1e-4f) {
                out.pushTri(cx, cy, z1, cx + c0 * r1, cy + s0 * r1, z1, cx + c1 * r1, cy + s1 * r1, z1)
            }
            if (r0 > 1e-4f) {
                out.pushTri(cx, cy, z0, cx + c1 * r0, cy + s1 * r0, z0, cx + c0 * r0, cy + s0 * r0, z0)
            }
        }
    }

    private fun addTube(
        out: FloatCollector,
        x0: Float,
        y0: Float,
        z0: Float,
        x1: Float,
        y1: Float,
        z1: Float,
        radius: Float,
        segs: Int
    ) {
        val dx = x1 - x0
        val dy = y1 - y0
        val dz = z1 - z0
        val len = sqrt(dx * dx + dy * dy + dz * dz)
        if (len < 1e-5f) return
        val tx = dx / len
        val ty = dy / len
        val tz = dz / len
        val refZ = if (kotlin.math.abs(tz) < 0.85f) 1f else 0f
        val refX = if (refZ == 0f) 1f else 0f
        var ux = ty * refZ
        var uy = tz * refX - tx * refZ
        var uz = -ty * refX
        val uLen = max(1e-5f, sqrt(ux * ux + uy * uy + uz * uz))
        ux /= uLen
        uy /= uLen
        uz /= uLen
        val vx = ty * uz - tz * uy
        val vy = tz * ux - tx * uz
        val vz = tx * uy - ty * ux

        val step = (PI * 2.0 / segs).toFloat()
        for (i in 0 until segs) {
            val a0 = i * step
            val a1 = (i + 1) * step
            val c0 = cos(a0) * radius
            val s0 = sin(a0) * radius
            val c1 = cos(a1) * radius
            val s1 = sin(a1) * radius
            val ox0 = ux * c0 + vx * s0
            val oy0 = uy * c0 + vy * s0
            val oz0 = uz * c0 + vz * s0
            val ox1 = ux * c1 + vx * s1
            val oy1 = uy * c1 + vy * s1
            val oz1 = uz * c1 + vz * s1
            out.pushQuad(
                x0 + ox0, y0 + oy0, z0 + oz0,
                x1 + ox0, y1 + oy0, z1 + oz0,
                x1 + ox1, y1 + oy1, z1 + oz1,
                x0 + ox1, y0 + oy1, z0 + oz1
            )
            out.pushTri(x1, y1, z1, x1 + ox0, y1 + oy0, z1 + oz0, x1 + ox1, y1 + oy1, z1 + oz1)
            out.pushTri(x0, y0, z0, x0 + ox1, y0 + oy1, z0 + oz1, x0 + ox0, y0 + oy0, z0 + oz0)
        }
    }

    private fun addShroudRing(
        out: FloatCollector,
        cx: Float,
        cy: Float,
        z0: Float,
        z1: Float,
        radius: Float,
        thickness: Float,
        angStart: Float,
        angEnd: Float,
        segs: Int
    ) {
        val rIn = radius - thickness * 0.5f
        val rOut = radius + thickness * 0.5f
        val step = (angEnd - angStart) / segs
        for (i in 0 until segs) {
            val a0 = angStart + i * step
            val a1 = angStart + (i + 1) * step
            val c0 = cos(a0)
            val s0 = sin(a0)
            val c1 = cos(a1)
            val s1 = sin(a1)
            out.pushQuad(
                cx + c0 * rOut, cy + s0 * rOut, z0,
                cx + c1 * rOut, cy + s1 * rOut, z0,
                cx + c1 * rOut, cy + s1 * rOut, z1,
                cx + c0 * rOut, cy + s0 * rOut, z1
            )
            out.pushQuad(
                cx + c1 * rIn, cy + s1 * rIn, z0,
                cx + c0 * rIn, cy + s0 * rIn, z0,
                cx + c0 * rIn, cy + s0 * rIn, z1,
                cx + c1 * rIn, cy + s1 * rIn, z1
            )
            out.pushQuad(
                cx + c0 * rIn, cy + s0 * rIn, z1,
                cx + c0 * rOut, cy + s0 * rOut, z1,
                cx + c1 * rOut, cy + s1 * rOut, z1,
                cx + c1 * rIn, cy + s1 * rIn, z1
            )
            out.pushQuad(
                cx + c1 * rIn, cy + s1 * rIn, z0,
                cx + c1 * rOut, cy + s1 * rOut, z0,
                cx + c0 * rOut, cy + s0 * rOut, z0,
                cx + c0 * rIn, cy + s0 * rIn, z0
            )
        }
    }

    private fun addThreeBladePropeller(
        out: FloatCollector,
        cx: Float,
        cy: Float,
        cz: Float,
        radius: Float,
        spinSign: Int
    ) {
        val sgn = if (spinSign >= 0) 1f else -1f
        for (b in 0..2) {
            val ang = (b * 2.0 * PI / 3.0).toFloat()
            val rx = cos(ang)
            val ry = sin(ang)
            val px = -ry * sgn
            val py = rx * sgn
            val rRoot = radius * 0.13f
            val rMid = radius * 0.60f
            val rTip = radius
            val cRoot = radius * 0.095f
            val cMid = radius * 0.135f
            val cTip = radius * 0.055f

            val xR0 = cx + rx * rRoot + px * cRoot
            val yR0 = cy + ry * rRoot + py * cRoot
            val zR0 = cz + 0.0022f
            val xR1 = cx + rx * rRoot - px * cRoot * 0.7f
            val yR1 = cy + ry * rRoot - py * cRoot * 0.7f
            val zR1 = cz - 0.0018f

            val xM0 = cx + rx * rMid + px * cMid
            val yM0 = cy + ry * rMid + py * cMid
            val zM0 = cz + 0.0028f
            val xM1 = cx + rx * rMid - px * cMid * 0.75f
            val yM1 = cy + ry * rMid - py * cMid * 0.75f
            val zM1 = cz - 0.0020f

            val xT0 = cx + rx * rTip + px * cTip
            val yT0 = cy + ry * rTip + py * cTip
            val zT0 = cz + 0.0014f
            val xT1 = cx + rx * rTip - px * cTip
            val yT1 = cy + ry * rTip - py * cTip
            val zT1 = cz - 0.0010f

            out.pushQuad(xR0, yR0, zR0, xM0, yM0, zM0, xM1, yM1, zM1, xR1, yR1, zR1)
            out.pushQuad(xM0, yM0, zM0, xT0, yT0, zT0, xT1, yT1, zT1, xM1, yM1, zM1)
            out.pushQuad(xR1, yR1, zR1 - 0.0008f, xM1, yM1, zM1 - 0.0008f, xM0, yM0, zM0 - 0.0008f, xR0, yR0, zR0 - 0.0008f)
            out.pushQuad(xM1, yM1, zM1 - 0.0008f, xT1, yT1, zT1 - 0.0008f, xT0, yT0, zT0 - 0.0008f, xM0, yM0, zM0 - 0.0008f)
        }
    }

    private fun appendAuxRotorDisc(out: FloatCollector): AuxSpan {
        val start = out.vertexCount
        val segs = 28
        val step = (PI * 2.0 / segs).toFloat()
        for (i in 0 until segs) {
            val a0 = i * step
            val a1 = (i + 1) * step
            out.pushVertex(0f, 0f, 0f, 0f, 0f, 1f)
            out.pushVertex(cos(a0), sin(a0), 0f, 0f, 0f, 1f)
            out.pushVertex(cos(a1), sin(a1), 0f, 0f, 0f, 1f)
        }
        return AuxSpan(start, out.vertexCount - start)
    }

    private fun appendAuxRotorRing(out: FloatCollector): AuxSpan {
        val start = out.vertexCount
        val segs = 28
        val rIn = 0.88f
        val step = (PI * 2.0 / segs).toFloat()
        for (i in 0 until segs) {
            val a0 = i * step
            val a1 = (i + 1) * step
            val c0 = cos(a0)
            val s0 = sin(a0)
            val c1 = cos(a1)
            val s1 = sin(a1)
            out.pushQuad(c0 * rIn, s0 * rIn, 0f, c0, s0, 0f, c1, s1, 0f, c1 * rIn, s1 * rIn, 0f)
        }
        return AuxSpan(start, out.vertexCount - start)
    }

    private fun appendAuxRotorSweep(out: FloatCollector): AuxSpan {
        val start = out.vertexCount
        val arcSegs = 7
        val sweepSpan = 0.92f
        for (lobe in 0..1) {
            val base = lobe * PI.toFloat()
            val step = sweepSpan / arcSegs
            for (i in 0 until arcSegs) {
                val a0 = base + i * step
                val a1 = base + (i + 1) * step
                val c0 = cos(a0)
                val s0 = sin(a0)
                val c1 = cos(a1)
                val s1 = sin(a1)
                out.pushQuad(c0 * 0.18f, s0 * 0.18f, 0f, c0 * 0.96f, s0 * 0.96f, 0f, c1 * 0.96f, s1 * 0.96f, 0f, c1 * 0.18f, s1 * 0.18f, 0f)
            }
        }
        return AuxSpan(start, out.vertexCount - start)
    }

    private fun appendAuxUnitCylinderZ(out: FloatCollector): AuxSpan {
        val start = out.vertexCount
        addCylinderZ(out, 0f, 0f, 0f, 1f, 1f, 1f, 10)
        return AuxSpan(start, out.vertexCount - start)
    }

    private fun appendAuxUnitConeZ(out: FloatCollector): AuxSpan {
        val start = out.vertexCount
        addCylinderZ(out, 0f, 0f, 0f, 1f, 1f, 0.02f, 10)
        return AuxSpan(start, out.vertexCount - start)
    }

    private fun appendAuxNoseChevron(out: FloatCollector): AuxSpan {
        val start = out.vertexCount
        out.pushTri(1.0f, 0f, 0f, 0.25f, 0.48f, 0f, 0.48f, 0f, 0f)
        out.pushTri(1.0f, 0f, 0f, 0.48f, 0f, 0f, 0.25f, -0.48f, 0f)
        out.pushQuad(0.0f, -0.09f, 0f, 0.56f, -0.09f, 0f, 0.56f, 0.09f, 0f, 0.0f, 0.09f, 0f)
        return AuxSpan(start, out.vertexCount - start)
    }

    private class IntHashTable(initialCapacity: Int) {
        private var cap = Integer.highestOneBit((initialCapacity * 2).coerceAtLeast(1024) - 1) shl 1
        private var keys = IntArray(cap) { -1 }
        private var vals = IntArray(cap)
        private var used = 0

        fun clear() {
            keys.fill(-1)
            used = 0
        }

        fun get(key: Int): Int {
            val mask = cap - 1
            var idx = mix(key) and mask
            while (true) {
                val k = keys[idx]
                if (k == key) return vals[idx]
                if (k == -1) return -1
                idx = (idx + 1) and mask
            }
        }

        fun put(key: Int, value: Int) {
            if (used * 10 >= cap * 6) rehash()
            val mask = cap - 1
            var idx = mix(key) and mask
            while (true) {
                val k = keys[idx]
                if (k == -1 || k == key) {
                    if (k == -1) used++
                    keys[idx] = key
                    vals[idx] = value
                    return
                }
                idx = (idx + 1) and mask
            }
        }

        private fun rehash() {
            val oldK = keys
            val oldV = vals
            cap *= 2
            keys = IntArray(cap) { -1 }
            vals = IntArray(cap)
            used = 0
            for (i in oldK.indices) {
                if (oldK[i] != -1) put(oldK[i], oldV[i])
            }
        }

        private fun mix(x: Int): Int {
            var h = x * -1640531527
            h = h xor (h ushr 16)
            return h
        }
    }
}
