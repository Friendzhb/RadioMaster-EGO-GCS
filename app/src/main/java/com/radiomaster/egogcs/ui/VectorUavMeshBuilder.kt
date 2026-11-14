package com.radiomaster.egogcs.ui

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Prepares GPU-ready interleaved VBO geometry (`px, py, pz, nx, ny, nz`, 24-byte stride) for both
 * research UAV configurations in Z-up meter-scale FLU coordinates (`ARM_SPAN_RADIUS = 0.18817742m`):
 * 1. **4M2S Dual-Servo Tilt-Rotor (`buildProceduralTiltUav` / `prepareFromGlb`)**:
 *    Twin-deck carbon-fiber chassis, CNC standoffs, Orin heatsink fins, Livox Mid-360 LiDAR with
 *    roll-cage, RealSense D435i stereo camera, under-slung LiPo + XT60 plug, S1/S2 gold servo linkages,
 *    slotted brushless outrunner bells with copper stator windings, and 3-blade twisted carbon airfoil
 *    propellers with high-visibility painted blade tips.
 * 2. **X4 Standard Research Quadrotor (`buildProceduralStandardQuad`)**:
 *    Same research avionics core paired with rigid 45-degree diagonal tubular carbon booms, CNC end
 *    mounts, front/rear LED strobes, and 4 upright brushless motors + spinning 3-blade carbon props.
 */
internal object VectorUavMeshBuilder {
    const val STRIDE_FLOATS = 6
    const val STRIDE_BYTES = STRIDE_FLOATS * 4
    const val ARM_SPAN_RADIUS = 0.18817742f

    private const val FRONT_PIVOT_X = 0.11667f
    private const val REAR_PIVOT_X = -0.11667f
    private const val PIVOT_Z = 0.056f
    private const val MOTOR_Y = 0.147644f

    private const val QUAD_MOTOR_XY = 0.13306f
    private const val QUAD_MOTOR_Z = 0.054f

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
        val specular: Float = 0.34f,
        val emissive: Float = 0.0f
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
        val rotorSweepCcw: AuxSpan,
        val rotorSweepCw: AuxSpan,
        val thrustShaft: AuxSpan,
        val thrustCone: AuxSpan,
        val noseChevron: AuxSpan,
        val isProceduralFallback: Boolean
    )

    private class FloatCollector(initialFloats: Int = 131_072) {
        var data = FloatArray(initialFloats)
        var size = 0
        val vertexCount: Int get() = size / STRIDE_FLOATS

        private fun ensure(extra: Int) {
            if (size + extra <= data.size) return
            var next = data.size * 2
            while (next < size + extra) next *= 2
            data = data.copyOf(next)
        }

        fun pushVertexUnit(px: Float, py: Float, pz: Float, nx: Float, ny: Float, nz: Float) {
            ensure(6)
            var ux = nx
            var uy = ny
            var uz = nz
            val len = sqrt(ux * ux + uy * uy + uz * uz)
            if (len > 1e-6f) {
                val inv = 1f / len
                ux *= inv
                uy *= inv
                uz *= inv
            } else {
                ux = 0f
                uy = 0f
                uz = 1f
            }
            val d = data
            val o = size
            d[o] = px
            d[o + 1] = py
            d[o + 2] = pz
            d[o + 3] = ux
            d[o + 4] = uy
            d[o + 5] = uz
            size = o + 6
        }

        /**
         * Encodes a vertex fade weight `fade in [0.02, 1.0]` inside `nz` while keeping `|(nx, ny, nz)| == 1.0`
         * so pre-transform `aNormal.z` supplies smooth hardware-interpolated motion-blur alpha in GLES 2.0.
         */
        fun pushFadeVertexXY(px: Float, py: Float, pz: Float, fade: Float) {
            val f = fade.coerceIn(0.015f, 1.0f)
            val nx = sqrt((1.0f - f * f).coerceAtLeast(0f))
            pushVertexUnit(px, py, pz, nx, 0f, f)
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
            val nx = uy * vz - uz * vy
            val ny = uz * vx - ux * vz
            val nz = ux * vy - uy * vx
            if (nx * nx + ny * ny + nz * nz < 1e-20f) return
            pushVertexUnit(x0, y0, z0, nx, ny, nz)
            pushVertexUnit(x1, y1, z1, nx, ny, nz)
            pushVertexUnit(x2, y2, z2, nx, ny, nz)
        }

        fun pushSmoothQuad(
            x0: Float,
            y0: Float,
            z0: Float,
            nx0: Float,
            ny0: Float,
            nz0: Float,
            x1: Float,
            y1: Float,
            z1: Float,
            nx1: Float,
            ny1: Float,
            nz1: Float,
            x2: Float,
            y2: Float,
            z2: Float,
            nx2: Float,
            ny2: Float,
            nz2: Float,
            x3: Float,
            y3: Float,
            z3: Float,
            nx3: Float,
            ny3: Float,
            nz3: Float
        ) {
            pushVertexUnit(x0, y0, z0, nx0, ny0, nz0)
            pushVertexUnit(x1, y1, z1, nx1, ny1, nz1)
            pushVertexUnit(x2, y2, z2, nx2, ny2, nz2)
            pushVertexUnit(x0, y0, z0, nx0, ny0, nz0)
            pushVertexUnit(x2, y2, z2, nx2, ny2, nz2)
            pushVertexUnit(x3, y3, z3, nx3, ny3, nz3)
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

    private fun defaultTiltRotorHubs(frontPivot: FloatArray, rearPivot: FloatArray): Array<RotorHub> = arrayOf(
        RotorHub(1, 1, frontPivot[0], -MOTOR_Y, PIVOT_Z + 0.026f, 0.072f, 1, true),
        RotorHub(2, 2, rearPivot[0], MOTOR_Y, PIVOT_Z + 0.026f, 0.072f, 1, false),
        RotorHub(3, 1, frontPivot[0], MOTOR_Y, PIVOT_Z + 0.026f, 0.072f, -1, true),
        RotorHub(4, 2, rearPivot[0], -MOTOR_Y, PIVOT_Z + 0.026f, 0.072f, -1, false)
    )

    private fun defaultStandardQuadHubs(): Array<RotorHub> = arrayOf(
        RotorHub(1, 0, QUAD_MOTOR_XY, -QUAD_MOTOR_XY, QUAD_MOTOR_Z + 0.025f, 0.076f, 1, true),
        RotorHub(2, 0, -QUAD_MOTOR_XY, QUAD_MOTOR_XY, QUAD_MOTOR_Z + 0.025f, 0.076f, 1, false),
        RotorHub(3, 0, QUAD_MOTOR_XY, QUAD_MOTOR_XY, QUAD_MOTOR_Z + 0.025f, 0.076f, -1, true),
        RotorHub(4, 0, -QUAD_MOTOR_XY, -QUAD_MOTOR_XY, QUAD_MOTOR_Z + 0.025f, 0.076f, -1, false)
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

    private fun detectMotorIndex(name: String): Int {
        val lower = name.lowercase()
        if (!lower.contains("motor")) return 0
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
        val hubs = defaultTiltRotorHubs(parsed.frontPivot, parsed.rearPivot)

        // First locate concentric motor shaft centers from circularly-symmetric motor bells (or node origins)
        // so 3-blade propeller bounding-box asymmetry never offsets the spin axis!
        val motorMinX = FloatArray(4) { Float.POSITIVE_INFINITY }
        val motorMaxX = FloatArray(4) { Float.NEGATIVE_INFINITY }
        val motorMinY = FloatArray(4) { Float.POSITIVE_INFINITY }
        val motorMaxY = FloatArray(4) { Float.NEGATIVE_INFINITY }
        val propNodeOriginX = FloatArray(4) { Float.NaN }
        val propNodeOriginY = FloatArray(4) { Float.NaN }

        for (part in parsed.parts) {
            val mIdx = detectMotorIndex(part.name)
            if (mIdx in 1..4) {
                val m = part.transform
                val slot = mIdx - 1
                val step = max(1, part.vertexCount / 2048)
                var i = 0
                while (i < part.vertexCount) {
                    val o = part.positionOffset + i * 12
                    val lx = bin.getFloat(o)
                    val ly = bin.getFloat(o + 4)
                    val lz = bin.getFloat(o + 8)
                    val bx = m[0] * lx + m[4] * ly + m[8] * lz + m[12]
                    val by = m[1] * lx + m[5] * ly + m[9] * lz + m[13]
                    if (bx < motorMinX[slot]) motorMinX[slot] = bx
                    if (bx > motorMaxX[slot]) motorMaxX[slot] = bx
                    if (by < motorMinY[slot]) motorMinY[slot] = by
                    if (by > motorMaxY[slot]) motorMaxY[slot] = by
                    i += step
                }
            }
            val rIdx = detectRotorIndex(part.name)
            if (rIdx in 1..4) {
                val slot = rIdx - 1
                val tx = part.transform[12]
                val ty = part.transform[13]
                if (abs(tx) > 0.02f && abs(ty) > 0.02f) {
                    propNodeOriginX[slot] = tx
                    propNodeOriginY[slot] = ty
                }
            }
        }

        val axisX = FloatArray(4) { s ->
            when {
                !propNodeOriginX[s].isNaN() -> propNodeOriginX[s]
                motorMinX[s].isFinite() && motorMaxX[s].isFinite() -> (motorMinX[s] + motorMaxX[s]) * 0.5f
                else -> hubs[s].x
            }
        }
        val axisY = FloatArray(4) { s ->
            when {
                !propNodeOriginY[s].isNaN() -> propNodeOriginY[s]
                motorMinY[s].isFinite() && motorMaxY[s].isFinite() -> (motorMinY[s] + motorMaxY[s]) * 0.5f
                else -> hubs[s].y
            }
        }

        val propMaxR = FloatArray(4) { 0f }
        val propMaxZ = FloatArray(4) { Float.NEGATIVE_INFINITY }

        for (part in parsed.parts) {
            val rIdx = detectRotorIndex(part.name)
            if (rIdx in 1..4) {
                val m = part.transform
                val slot = rIdx - 1
                val cx = axisX[slot]
                val cy = axisY[slot]
                val step = max(1, part.vertexCount / 2048)
                var i = 0
                while (i < part.vertexCount) {
                    val o = part.positionOffset + i * 12
                    val lx = bin.getFloat(o)
                    val ly = bin.getFloat(o + 4)
                    val lz = bin.getFloat(o + 8)
                    val bx = m[0] * lx + m[4] * ly + m[8] * lz + m[12]
                    val by = m[1] * lx + m[5] * ly + m[9] * lz + m[13]
                    val bz = m[2] * lx + m[6] * ly + m[10] * lz + m[14]
                    val rad = hypot(bx - cx, by - cy)
                    if (rad > propMaxR[slot]) propMaxR[slot] = rad
                    if (bz > propMaxZ[slot]) propMaxZ[slot] = bz
                    i += step
                }
            }
        }

        for (slot in 0..3) {
            val prev = hubs[slot]
            val radius = if (propMaxR[slot] > 0.02f) propMaxR[slot].coerceIn(0.045f, 0.115f) else prev.radius
            hubs[slot] = RotorHub(
                motorId = prev.motorId,
                group = prev.group,
                x = axisX[slot],
                y = axisY[slot],
                z = if (propMaxZ[slot].isFinite()) propMaxZ[slot] else prev.z,
                radius = radius,
                spinSign = prev.spinSign,
                isFront = prev.isFront
            )
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
                    rIdx > 0 -> 0.56f
                    part.name.contains("motor", ignoreCase = true) -> 0.68f
                    else -> 0.36f
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
                        specular = spec,
                        emissive = if (rIdx > 0) 0.08f else 0f
                    )
                )
            }
        }

        // Append high-visibility painted blade tips onto GLB rotors as well so rotation is crystal clear
        val idMat = floatArrayOf(
            1f, 0f, 0f, 0f,
            0f, 1f, 0f, 0f,
            0f, 0f, 1f, 0f,
            0f, 0f, 0f, 1f
        )
        for (h in hubs) {
            val tipStart = collector.vertexCount
            addPropellerTipCapsOnly(collector, h.x, h.y, h.z + 0.0006f, h.radius * 0.96f, h.spinSign)
            val tipCnt = collector.vertexCount - tipStart
            if (tipCnt > 0) {
                val tipCol = if (h.isFront) {
                    floatArrayOf(1.00f, 0.56f, 0.08f, 1f)
                } else {
                    floatArrayOf(0.18f, 0.96f, 0.62f, 1f)
                }
                gpuParts.add(
                    GpuPart(
                        name = "prop_tip_m${h.motorId}_visual",
                        group = h.group,
                        rotorIndex = h.motorId,
                        transform = idMat.copyOf(),
                        firstVertex = tipStart,
                        vertexCount = tipCnt,
                        color = tipCol,
                        specular = 0.55f,
                        emissive = 0.26f
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
                    out[0] = 0.88f
                    out[1] = 0.38f
                    out[2] = 0.30f
                } else {
                    out[0] = 0.24f
                    out[1] = 0.84f
                    out[2] = 0.56f
                }
            }
        } else if (lower.contains("motor") && luma < 0.24f) {
            out[0] = 0.32f
            out[1] = 0.38f
            out[2] = 0.48f
        } else if (luma < 0.16f) {
            out[0] = max(out[0], 0.18f)
            out[1] = max(out[1], 0.22f)
            out[2] = max(out[2], 0.28f)
        }
        return out
    }

    private fun copyPartRaw(bin: ByteBuffer, part: VectorGlbAsset.Part, out: FloatCollector) {
        val pBase = part.positionOffset
        val nBase = part.normalOffset
        for (i in 0 until part.vertexCount) {
            val po = pBase + i * 12
            val no = nBase + i * 12
            out.pushVertexUnit(
                bin.getFloat(po),
                bin.getFloat(po + 4),
                bin.getFloat(po + 8),
                bin.getFloat(no),
                bin.getFloat(no + 4),
                bin.getFloat(no + 8)
            )
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
                out.pushVertexUnit(sumFloats[b0], sumFloats[b0 + 1], sumFloats[b0 + 2], sumFloats[b0 + 3], sumFloats[b0 + 4], sumFloats[b0 + 5])
                out.pushVertexUnit(sumFloats[b1], sumFloats[b1 + 1], sumFloats[b1 + 2], sumFloats[b1 + 3], sumFloats[b1 + 4], sumFloats[b1 + 5])
                out.pushVertexUnit(sumFloats[b2], sumFloats[b2 + 1], sumFloats[b2 + 2], sumFloats[b2 + 3], sumFloats[b2 + 4], sumFloats[b2 + 5])
            }
            t += 3
        }
    }

    /**
     * Builds the shared research-lab autonomous UAV central airframe (`group = 0`):
     * - Upper & lower CNC sandwich carbon-fiber center decks + rear tail deck & front camera tongue
     * - 6x brushed titanium hex standoffs
     * - Jetson Orin / NUC gunmetal compute module with 7 machined heatsink cooling fins + Pixhawk FC
     * - Top Livox Mid-360 LiDAR with sapphire emitter band, domed cap & 4-strut protective roll-cage
     * - Nose Intel RealSense D435i stereo depth camera bar with 3 recessed optical lenses
     * - Belly 6S LiPo battery pack + crimson Kevlar strap + yellow XT60 power plug + power leads
     * - Twin swept carbon-fiber tubular landing skids with anti-vibration toe boots + V-dipole antennas
     */
    private fun appendResearchAvionicsCore(
        collector: FloatCollector,
        partFn: (name: String, group: Int, rotorIdx: Int, r: Float, g: Float, b: Float, spec: Float, emissive: Float, build: () -> Unit) -> Unit
    ) {
        // 1. Matte woven carbon-fiber sandwich center plates, landing skids, roll-cage & antenna tubes
        partFn("fuselage_carbon_visual", 0, 0, 0.15f, 0.18f, 0.23f, 0.36f, 0f) {
            // Lower structural carbon plate (2.4mm thick) + Upper avionics carbon deck (2.0mm thick)
            addOctagonalPlate(collector, -0.088f, 0.094f, 0.048f, 0.036f, 0.0388f)
            addOctagonalPlate(collector, -0.078f, 0.086f, 0.043f, 0.066f, 0.0684f)
            // Front camera vibration-isolated carbon tongue & rear XT60/VTX shelf
            addBox(collector, 0.076f, 0.114f, -0.028f, 0.028f, 0.044f, 0.0468f)
            addBox(collector, -0.098f, -0.074f, -0.022f, 0.022f, 0.044f, 0.0468f)
            // Matte dark LiPo cell pack under belly
            addBeveledBatteryPack(collector, -0.056f, 0.050f, -0.025f, 0.025f, 0.011f, 0.0355f)

            // Twin swept carbon-fiber tubular landing skids
            for (side in intArrayOf(-1, 1)) {
                val sy = side * 0.065f
                val skidZ = -0.011f
                addSmoothTube(collector, -0.084f, sy, skidZ, 0.088f, sy, skidZ, 0.0040f, 10)
                addSmoothTube(collector, 0.088f, sy, skidZ, 0.105f, sy, skidZ + 0.013f, 0.0036f, 8)
                addSmoothTube(collector, -0.084f, sy, skidZ, -0.096f, sy, skidZ + 0.010f, 0.0036f, 8)
                // Angled carbon strut legs with cross-brace
                addSmoothTube(collector, 0.045f, side * 0.036f, 0.037f, 0.052f, sy, skidZ, 0.0038f, 8)
                addSmoothTube(collector, -0.045f, side * 0.036f, 0.037f, -0.052f, sy, skidZ, 0.0038f, 8)
            }

            // Protective 4-post carbon roll-cage around top Livox Mid-360 LiDAR
            val cageBaseZ = 0.068f
            val cageTopZ = 0.117f
            val cageCorners = arrayOf(
                floatArrayOf(0.028f, -0.031f, 0.020f, -0.024f),
                floatArrayOf(0.028f, 0.031f, 0.020f, 0.024f),
                floatArrayOf(-0.040f, 0.031f, -0.032f, 0.024f),
                floatArrayOf(-0.040f, -0.031f, -0.032f, -0.024f)
            )
            for (c in cageCorners) {
                addSmoothTube(collector, c[0], c[1], cageBaseZ, c[2], c[3], cageTopZ, 0.0022f, 8)
            }
            addSmoothShroudRing(collector, -0.006f, 0f, cageTopZ - 0.002f, cageTopZ + 0.0018f, 0.030f, 0.0028f, 0f, (PI * 2.0).toFloat(), 18)

            // Rear ELRS / telemetry V-dipole diversity antenna tubes
            addSmoothTube(collector, -0.092f, -0.012f, 0.046f, -0.122f, -0.046f, 0.080f, 0.0019f, 6)
            addSmoothTube(collector, -0.092f, 0.012f, 0.046f, -0.122f, 0.046f, 0.080f, 0.0019f, 6)
        }

        // 2. Gunmetal anodized onboard compute (Jetson Orin / NUC) heatsink fins & LiDAR base
        partFn("compute_gunmetal_visual", 0, 0, 0.28f, 0.34f, 0.42f, 0.64f, 0f) {
            // Central compute brick between sandwich plates
            addBox(collector, -0.056f, 0.048f, -0.031f, 0.031f, 0.040f, 0.062f)
            // 7 CNC longitudinal heatsink cooling fins protruding through upper deck cutout
            for (f in -3..3) {
                val fy = f * 0.0076f
                addBox(collector, -0.046f, 0.034f, fy - 0.0016f, fy + 0.0016f, 0.062f, 0.0725f)
            }
            // Livox Mid-360 lower fluted heatsink base & upper dome cap
            addSmoothCylinderZ(collector, -0.006f, 0f, 0.0684f, 0.0845f, 0.0275f, 0.0265f, 18)
            addSmoothCylinderZ(collector, -0.006f, 0f, 0.1015f, 0.1110f, 0.0256f, 0.0205f, 18)
            addSmoothCylinderZ(collector, -0.006f, 0f, 0.1110f, 0.1145f, 0.0205f, 0.0080f, 16)
            // Dark recessed stereo camera glass visor
            addBox(collector, 0.1142f, 0.1162f, -0.029f, 0.029f, 0.0485f, 0.0595f)
        }

        // 3. Brushed titanium-silver CNC standoffs & Intel RealSense D435i aluminum stereo housing
        partFn("avionics_titanium_visual", 0, 0, 0.62f, 0.68f, 0.76f, 0.82f, 0f) {
            val standoffs = arrayOf(
                floatArrayOf(0.068f, -0.030f),
                floatArrayOf(0.068f, 0.030f),
                floatArrayOf(0.002f, -0.039f),
                floatArrayOf(0.002f, 0.039f),
                floatArrayOf(-0.064f, -0.030f),
                floatArrayOf(-0.064f, 0.030f)
            )
            for (st in standoffs) {
                addSmoothCylinderZ(collector, st[0], st[1], 0.0388f, 0.0660f, 0.0031f, 0.0031f, 8)
            }
            // Intel RealSense D435i machined aluminum stereo bar + rounded end-caps
            addBox(collector, 0.095f, 0.1145f, -0.031f, 0.031f, 0.047f, 0.0615f)
            addSmoothTube(collector, 0.095f, -0.031f, 0.0542f, 0.1145f, -0.031f, 0.0542f, 0.0072f, 10)
            addSmoothTube(collector, 0.095f, 0.031f, 0.0542f, 0.1145f, 0.031f, 0.0542f, 0.0072f, 10)
        }

        // 4. Sapphire-cyan Livox Mid-360 optical emitter window & RealSense D435i triple camera lenses
        partFn("sensor_optics_visual", 0, 0, 0.08f, 0.80f, 0.98f, 0.92f, 0.24f) {
            addSmoothCylinderZ(collector, -0.006f, 0f, 0.0845f, 0.1015f, 0.0266f, 0.0256f, 20)
            // Left IR, Center Projector/RGB, Right IR optical apertures
            for (ly in floatArrayOf(-0.020f, 0.0f, 0.020f)) {
                val rLens = if (ly == 0f) 0.0038f else 0.0048f
                addSmoothTube(collector, 0.1135f, ly, 0.0542f, 0.1174f, ly, 0.0542f, rLens, 12)
            }
            // Forward orientation indicator chevron on top deck
            collector.pushTri(0.082f, 0f, 0.0692f, 0.054f, 0.015f, 0.0692f, 0.062f, 0f, 0.0692f)
            collector.pushTri(0.082f, 0f, 0.0692f, 0.062f, 0f, 0.0692f, 0.054f, -0.015f, 0.0692f)
        }

        // 5. High-visibility XT60 yellow power connector & Pixhawk gold accents
        partFn("xt60_gold_accent_visual", 0, 0, 0.98f, 0.76f, 0.08f, 0.58f, 0.06f) {
            // XT60 plug on left rear power distribution side
            addBox(collector, -0.044f, -0.024f, 0.026f, 0.038f, 0.024f, 0.035f)
            // Front FC status indicator bezel
            addBox(collector, 0.038f, 0.050f, -0.011f, 0.011f, 0.0684f, 0.0722f)
        }

        // 6. Crimson Kevlar LiPo strap, 12AWG red silicone lead, antenna caps & front skid toe boots
        partFn("kevlar_red_detail_visual", 0, 0, 0.92f, 0.20f, 0.20f, 0.42f, 0.10f) {
            addBox(collector, -0.012f, 0.006f, -0.0268f, 0.0268f, 0.0098f, 0.0366f)
            addSmoothTube(collector, -0.030f, 0.036f, 0.029f, -0.012f, 0.024f, 0.041f, 0.0022f, 6)
            // Front skid damper boots
            for (side in intArrayOf(-1, 1)) {
                val sy = side * 0.065f
                addSmoothTube(collector, 0.098f, sy, -0.002f, 0.107f, sy, 0.004f, 0.0046f, 8)
            }
            // Antenna tip caps
            addSmoothTube(collector, -0.118f, -0.041f, 0.075f, -0.124f, -0.048f, 0.082f, 0.0027f, 6)
            addSmoothTube(collector, -0.118f, 0.041f, 0.075f, -0.124f, 0.048f, 0.082f, 0.0027f, 6)
        }
    }

    /**
     * Appends a realistic brushless outrunner motor assembly (slotted dark-titanium bell, exposed
     * copper stator windings, CNC spinner nut, twisted 3-blade carbon propeller + painted blade tips).
     */
    private fun appendBrushlessMotorAndProp(
        collector: FloatCollector,
        partFn: (name: String, group: Int, rotorIdx: Int, r: Float, g: Float, b: Float, spec: Float, emissive: Float, build: () -> Unit) -> Unit,
        h: RotorHub,
        mountBaseZ: Float,
        includePartialGuardArc: Boolean
    ) {
        val grp = h.group
        val mTag = "m${h.motorId}"
        val isFront = h.isFront

        // 1. Slotted CNC brushless outrunner bell + base mount + optional outer carbon bumper arc
        partFn("motor_${mTag}_visual", grp, 0, 0.27f, 0.33f, 0.42f, 0.72f, 0f) {
            // CNC aluminum motor base plate & lower bearing cup
            addSmoothCylinderZ(collector, h.x, h.y, mountBaseZ - 0.004f, mountBaseZ + 0.003f, 0.0145f, 0.0155f, 14)
            // Lower & upper machined bell rings (exposing copper windings in the middle gap!)
            addSmoothCylinderZ(collector, h.x, h.y, mountBaseZ + 0.003f, mountBaseZ + 0.0072f, 0.0172f, 0.0176f, 16)
            addSmoothCylinderZ(collector, h.x, h.y, mountBaseZ + 0.0148f, mountBaseZ + 0.0205f, 0.0176f, 0.0162f, 16)
            // 6 vertical bell cage pillars bridging lower & upper bell rings
            for (p in 0 until 6) {
                val ang = (p * PI / 3.0).toFloat()
                val px = h.x + cos(ang) * 0.0166f
                val py = h.y + sin(ang) * 0.0166f
                addSmoothCylinderZ(collector, px, py, mountBaseZ + 0.0068f, mountBaseZ + 0.0152f, 0.0020f, 0.0020f, 6)
            }
            // Top radial cooling spokes on outrunner bell
            for (sp in 0 until 5) {
                val ang = (sp * 2.0 * PI / 5.0).toFloat()
                addSmoothTube(
                    collector,
                    h.x + cos(ang) * 0.005f,
                    h.y + sin(ang) * 0.005f,
                    mountBaseZ + 0.0208f,
                    h.x + cos(ang) * 0.0155f,
                    h.y + sin(ang) * 0.0155f,
                    mountBaseZ + 0.0198f,
                    0.0018f,
                    6
                )
            }
            if (includePartialGuardArc) {
                val outSign = if (h.y >= 0f) 1f else -1f
                val arcMid = (if (isFront) 0.55f else 2.59f) * outSign
                addSmoothShroudRing(
                    collector,
                    h.x,
                    h.y,
                    mountBaseZ + 0.011f,
                    mountBaseZ + 0.019f,
                    h.radius * 1.07f,
                    0.0028f,
                    arcMid - 1.35f,
                    arcMid + 1.35f,
                    14
                )
                for (st in intArrayOf(-1, 1)) {
                    val sa = arcMid + st * 0.92f
                    addSmoothTube(
                        collector,
                        h.x + cos(sa) * 0.014f,
                        h.y + sin(sa) * 0.014f,
                        mountBaseZ + 0.005f,
                        h.x + cos(sa) * (h.radius * 1.06f),
                        h.y + sin(sa) * (h.radius * 1.06f),
                        mountBaseZ + 0.014f,
                        0.0019f,
                        6
                    )
                }
            }
        }

        // 2. Exposed enamel-copper stator windings glowing inside the slotted bell
        partFn("stator_${mTag}_visual", grp, 0, 0.92f, 0.46f, 0.14f, 0.78f, 0.08f) {
            addSmoothCylinderZ(collector, h.x, h.y, mountBaseZ + 0.0068f, mountBaseZ + 0.0152f, 0.0154f, 0.0154f, 16)
        }

        // 3. High-intensity aviation LED navigation beacon on outer nacelle/arm tip (Front Red / Rear Emerald)
        val navR = if (isFront) 1.00f else 0.06f
        val navG = if (isFront) 0.22f else 0.95f
        val navB = if (isFront) 0.18f else 0.48f
        val outDirX = if (isFront) 1f else -1f
        val outDirY = if (h.y >= 0f) 1f else -1f
        partFn("nav_led_${mTag}_visual", grp, 0, navR, navG, navB, 0.65f, 0.62f) {
            addSmoothCylinderZ(
                collector,
                h.x + outDirX * 0.0135f,
                h.y + outDirY * 0.0135f,
                mountBaseZ - 0.0055f,
                mountBaseZ + 0.0045f,
                0.0044f,
                0.0038f,
                10
            )
            if (includePartialGuardArc) {
                val arcMid = (if (isFront) 0.55f else 2.59f) * outDirY
                addSmoothShroudRing(
                    collector,
                    h.x,
                    h.y,
                    mountBaseZ + 0.0135f,
                    mountBaseZ + 0.0205f,
                    h.radius * 1.088f,
                    0.0030f,
                    arcMid - 0.65f,
                    arcMid + 0.65f,
                    8
                )
            }
        }

        // 4. Spinning 3-blade twisted carbon-fiber airfoil propeller (`rotorIndex = h.motorId`)
        partFn("propeller_${mTag}_visual", grp, h.motorId, 0.17f, 0.20f, 0.25f, 0.62f, 0f) {
            // Molded carbon prop hub ring
            addSmoothCylinderZ(collector, h.x, h.y, h.z - 0.0028f, h.z + 0.0028f, 0.0086f, 0.0080f, 14)
            // 3 cambered, pitch-twisted airfoil blades (0% -> 82% span in dark satin carbon)
            addTwistedAirfoilPropellerBody(collector, h.x, h.y, h.z, h.radius * 0.98f, h.spinSign)
        }

        // 5. Machined silver conical prop spinner nut + high-visibility painted blade tips (`rotorIndex = h.motorId`)
        partFn("prop_spinner_${mTag}_visual", grp, h.motorId, 0.76f, 0.82f, 0.90f, 0.90f, 0.06f) {
            addSmoothCylinderZ(collector, h.x, h.y, h.z + 0.0026f, h.z + 0.0118f, 0.0064f, 0.0012f, 12)
        }

        val tipR = if (isFront) 1.00f else 0.20f
        val tipG = if (isFront) 0.58f else 0.96f
        val tipB = if (isFront) 0.08f else 0.62f
        partFn("prop_tip_${mTag}_visual", grp, h.motorId, tipR, tipG, tipB, 0.58f, 0.28f) {
            addPropellerTipCapsOnly(collector, h.x, h.y, h.z, h.radius * 0.98f, h.spinSign)
        }
    }

    /**
     * Builds the `Friendzhb/tilt-uav-indi` 4M2S dual-servo tilt-rotor research UAV in URDF meter-scale Z-up coordinates.
     */
    fun buildProceduralTiltUav(): PreparedModel {
        val frontPivot = floatArrayOf(FRONT_PIVOT_X, 0f, PIVOT_Z)
        val rearPivot = floatArrayOf(REAR_PIVOT_X, 0f, PIVOT_Z)
        val hubs = defaultTiltRotorHubs(frontPivot, rearPivot)
        val collector = FloatCollector(163_840)
        val parts = ArrayList<GpuPart>(32)
        val idMat = floatArrayOf(
            1f, 0f, 0f, 0f,
            0f, 1f, 0f, 0f,
            0f, 0f, 1f, 0f,
            0f, 0f, 0f, 1f
        )

        fun part(
            name: String,
            group: Int,
            rotorIdx: Int,
            r: Float,
            g: Float,
            b: Float,
            spec: Float,
            emissive: Float,
            build: () -> Unit
        ) {
            val start = collector.vertexCount
            build()
            val cnt = collector.vertexCount - start
            if (cnt > 0) {
                parts.add(
                    GpuPart(
                        name = name,
                        group = group,
                        rotorIndex = rotorIdx,
                        transform = idMat.copyOf(),
                        firstVertex = start,
                        vertexCount = cnt,
                        color = floatArrayOf(r, g, b, 1f),
                        specular = spec,
                        emissive = emissive
                    )
                )
            }
        }

        appendResearchAvionicsCore(collector, ::part)

        // Structural X-booms & central digital servo pods (group = 0)
        part("tilt_booms_carbon_visual", 0, 0, 0.16f, 0.19f, 0.24f, 0.40f, 0f) {
            val roots = arrayOf(
                floatArrayOf(0.052f, -0.035f, FRONT_PIVOT_X, -0.118f),
                floatArrayOf(-0.052f, 0.035f, REAR_PIVOT_X, 0.118f),
                floatArrayOf(0.052f, 0.035f, FRONT_PIVOT_X, 0.118f),
                floatArrayOf(-0.052f, -0.035f, REAR_PIVOT_X, -0.118f)
            )
            for (a in roots) {
                addSmoothTube(collector, a[0], a[1], 0.053f, a[2], a[3], PIVOT_Z, 0.0064f, 10)
                addSmoothTube(collector, a[0] * 0.86f, a[1] * 0.86f, 0.038f, a[2] * 0.90f, a[3] * 0.88f, PIVOT_Z - 0.005f, 0.0038f, 8)
            }
            // Longitudinal carbon boom spine to S1 & S2 digital servo housings
            addSmoothTube(collector, 0.070f, 0f, 0.056f, FRONT_PIVOT_X - 0.014f, 0f, PIVOT_Z, 0.0058f, 10)
            addSmoothTube(collector, -0.068f, 0f, 0.056f, REAR_PIVOT_X + 0.014f, 0f, PIVOT_Z, 0.0058f, 10)
            // S1 & S2 digital metal-case servo bodies
            addBox(collector, FRONT_PIVOT_X - 0.015f, FRONT_PIVOT_X + 0.015f, -0.020f, 0.020f, 0.043f, 0.065f)
            addBox(collector, REAR_PIVOT_X - 0.015f, REAR_PIVOT_X + 0.015f, -0.020f, 0.020f, 0.043f, 0.065f)
        }

        // CNC bearing pillow-blocks & anodized gold servo horns (group = 0)
        part("tilt_bearing_clamps_visual", 0, 0, 0.62f, 0.68f, 0.77f, 0.82f, 0f) {
            for (sx in floatArrayOf(FRONT_PIVOT_X, REAR_PIVOT_X)) {
                for (sy in floatArrayOf(-0.118f, 0.118f)) {
                    addSmoothTube(collector, sx, sy - 0.0085f, PIVOT_Z, sx, sy + 0.0085f, PIVOT_Z, 0.0088f, 12)
                }
            }
        }

        // Articulated front (group 1) & rear (group 2) tilt shafts, gold servo horns & motor nacelles
        for (grp in 1..2) {
            val px = if (grp == 1) FRONT_PIVOT_X else REAR_PIVOT_X
            val suffix = if (grp == 1) "f" else "b"
            part("tilt_shaft_${suffix}_visual", grp, 0, 0.66f, 0.72f, 0.80f, 0.84f, 0f) {
                addSmoothTube(collector, px, -MOTOR_Y, PIVOT_Z, px, MOTOR_Y, PIVOT_Z, 0.0046f, 12)
            }
            part("servo_horn_gold_${suffix}_visual", grp, 0, 0.96f, 0.70f, 0.12f, 0.78f, 0.05f) {
                addBox(collector, px - 0.0055f, px + 0.0055f, -0.025f, 0.025f, PIVOT_Z - 0.005f, PIVOT_Z + 0.015f)
                // Dual ball-link pushrods moving with tilt cradle
                addSmoothTube(collector, px + (if (grp == 1) -0.011f else 0.011f), -0.018f, PIVOT_Z - 0.008f, px, -0.018f, PIVOT_Z + 0.012f, 0.0018f, 6)
                addSmoothTube(collector, px + (if (grp == 1) -0.011f else 0.011f), 0.018f, PIVOT_Z - 0.008f, px, 0.018f, PIVOT_Z + 0.012f, 0.0018f, 6)
            }

            val groupHubs = hubs.filter { it.group == grp }
            for (h in groupHubs) {
                appendBrushlessMotorAndProp(
                    collector = collector,
                    partFn = ::part,
                    h = h,
                    mountBaseZ = PIVOT_Z + 0.002f,
                    includePartialGuardArc = true
                )
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

    /**
     * Builds the X4 fixed-motor autonomous research quadrotor (`EGO-Planner` style) with the same
     * high-detail carbon/LiDAR/RealSense avionics core and 4 diagonal tubular arms + spinning 3-blade props.
     */
    fun buildProceduralStandardQuad(): PreparedModel {
        val zeroPivot = floatArrayOf(0f, 0f, QUAD_MOTOR_Z)
        val hubs = defaultStandardQuadHubs()
        val collector = FloatCollector(163_840)
        val parts = ArrayList<GpuPart>(28)
        val idMat = floatArrayOf(
            1f, 0f, 0f, 0f,
            0f, 1f, 0f, 0f,
            0f, 0f, 1f, 0f,
            0f, 0f, 0f, 1f
        )

        fun part(
            name: String,
            group: Int,
            rotorIdx: Int,
            r: Float,
            g: Float,
            b: Float,
            spec: Float,
            emissive: Float,
            build: () -> Unit
        ) {
            val start = collector.vertexCount
            build()
            val cnt = collector.vertexCount - start
            if (cnt > 0) {
                parts.add(
                    GpuPart(
                        name = name,
                        group = group,
                        rotorIndex = rotorIdx,
                        transform = idMat.copyOf(),
                        firstVertex = start,
                        vertexCount = cnt,
                        color = floatArrayOf(r, g, b, 1f),
                        specular = spec,
                        emissive = emissive
                    )
                )
            }
        }

        appendResearchAvionicsCore(collector, ::part)

        // 4 rigid 45-degree diagonal woven carbon tubular booms + lower diagonal stiffeners
        part("x4_booms_carbon_visual", 0, 0, 0.16f, 0.19f, 0.24f, 0.40f, 0f) {
            for (h in hubs) {
                val sx = if (h.x >= 0f) 1f else -1f
                val sy = if (h.y >= 0f) 1f else -1f
                addSmoothTube(collector, sx * 0.044f, sy * 0.036f, 0.052f, h.x, h.y, QUAD_MOTOR_Z, 0.0068f, 10)
                addSmoothTube(collector, sx * 0.038f, sy * 0.030f, 0.038f, h.x * 0.82f, h.y * 0.82f, QUAD_MOTOR_Z - 0.004f, 0.0038f, 8)
            }
        }

        // Brushed CNC aluminum root C-clamps & motor end-mounts
        part("x4_cnc_mounts_visual", 0, 0, 0.62f, 0.68f, 0.77f, 0.84f, 0f) {
            for (h in hubs) {
                val sx = if (h.x >= 0f) 1f else -1f
                val sy = if (h.y >= 0f) 1f else -1f
                addSmoothTube(collector, sx * 0.054f, sy * 0.046f, 0.052f, sx * 0.068f, sy * 0.060f, 0.0525f, 0.0086f, 10)
                addSmoothCylinderZ(collector, h.x, h.y, QUAD_MOTOR_Z - 0.0065f, QUAD_MOTOR_Z + 0.0020f, 0.0168f, 0.0162f, 14)
            }
        }

        for (h in hubs) {
            appendBrushlessMotorAndProp(
                collector = collector,
                partFn = ::part,
                h = h,
                mountBaseZ = QUAD_MOTOR_Z + 0.001f,
                includePartialGuardArc = true
            )
        }

        return finalizeModel(
            parts = parts,
            frontPivot = zeroPivot.copyOf(),
            rearPivot = zeroPivot.copyOf(),
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
        val discSpan = appendAuxRotorDiscFeathered(collector)
        val ringSpan = appendAuxRotorRingFeathered(collector)
        val sweepCcwSpan = appendAuxRotorSweep3Lobe(collector, spinSign = 1)
        val sweepCwSpan = appendAuxRotorSweep3Lobe(collector, spinSign = -1)
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
            rotorSweepCcw = sweepCcwSpan,
            rotorSweepCw = sweepCwSpan,
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

    private fun addBeveledBatteryPack(
        out: FloatCollector,
        x0: Float,
        x1: Float,
        y0: Float,
        y1: Float,
        z0: Float,
        z1: Float
    ) {
        val bev = 0.0035f
        addBox(out, x0, x1, y0 + bev, y1 - bev, z0 + bev, z1 - bev)
        out.pushQuad(x0, y0 + bev, z0 + bev, x1, y0 + bev, z0 + bev, x1, y0 + bev * 1.8f, z0, x0, y0 + bev * 1.8f, z0)
        out.pushQuad(x1, y1 - bev, z0 + bev, x0, y1 - bev, z0 + bev, x0, y1 - bev * 1.8f, z0, x1, y1 - bev * 1.8f, z0)
        out.pushQuad(x0, y0 + bev * 1.8f, z0, x1, y0 + bev * 1.8f, z0, x1, y1 - bev * 1.8f, z0, x0, y1 - bev * 1.8f, z0)
    }

    private fun addOctagonalPlate(
        out: FloatCollector,
        backX: Float,
        frontX: Float,
        halfY: Float,
        zBot: Float,
        zTop: Float
    ) {
        val midF = frontX - (frontX - backX) * 0.22f
        val midB = backX + (frontX - backX) * 0.22f
        val tipY = halfY * 0.50f
        val ring = arrayOf(
            floatArrayOf(frontX, -tipY),
            floatArrayOf(frontX, tipY),
            floatArrayOf(midF, halfY),
            floatArrayOf(midB, halfY * 0.94f),
            floatArrayOf(backX, tipY * 0.90f),
            floatArrayOf(backX, -tipY * 0.90f),
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
            out.pushTri(cx, 0f, zTop, x0, y0, zTop, x1, y1, zTop)
            out.pushTri(cx, 0f, zBot, x1, y1, zBot, x0, y0, zBot)
            out.pushQuad(x0, y0, zBot, x1, y1, zBot, x1, y1, zTop, x0, y0, zTop)
        }
    }

    private fun addSmoothCylinderZ(
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
        val dz = max(1e-5f, z1 - z0)
        val slopeZ = (r0 - r1) / dz
        for (i in 0 until segs) {
            val a0 = i * step
            val a1 = (i + 1) * step
            val c0 = cos(a0)
            val s0 = sin(a0)
            val c1 = cos(a1)
            val s1 = sin(a1)
            out.pushSmoothQuad(
                cx + c0 * r0, cy + s0 * r0, z0, c0, s0, slopeZ,
                cx + c1 * r0, cy + s1 * r0, z0, c1, s1, slopeZ,
                cx + c1 * r1, cy + s1 * r1, z1, c1, s1, slopeZ,
                cx + c0 * r1, cy + s0 * r1, z1, c0, s0, slopeZ
            )
            if (r1 > 1e-4f) {
                out.pushTri(cx, cy, z1, cx + c0 * r1, cy + s0 * r1, z1, cx + c1 * r1, cy + s1 * r1, z1)
            }
            if (r0 > 1e-4f) {
                out.pushTri(cx, cy, z0, cx + c1 * r0, cy + s1 * r0, z0, cx + c0 * r0, cy + s0 * r0, z0)
            }
        }
    }

    private fun addSmoothTube(
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
        val refZ = if (abs(tz) < 0.85f) 1f else 0f
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
            val c0 = cos(a0)
            val s0 = sin(a0)
            val c1 = cos(a1)
            val s1 = sin(a1)
            val nx0 = ux * c0 + vx * s0
            val ny0 = uy * c0 + vy * s0
            val nz0 = uz * c0 + vz * s0
            val nx1 = ux * c1 + vx * s1
            val ny1 = uy * c1 + vy * s1
            val nz1 = uz * c1 + vz * s1
            val ox0 = nx0 * radius
            val oy0 = ny0 * radius
            val oz0 = nz0 * radius
            val ox1 = nx1 * radius
            val oy1 = ny1 * radius
            val oz1 = nz1 * radius
            out.pushSmoothQuad(
                x0 + ox0, y0 + oy0, z0 + oz0, nx0, ny0, nz0,
                x1 + ox0, y1 + oy0, z1 + oz0, nx0, ny0, nz0,
                x1 + ox1, y1 + oy1, z1 + oz1, nx1, ny1, nz1,
                x0 + ox1, y0 + oy1, z0 + oz1, nx1, ny1, nz1
            )
            out.pushTri(x1, y1, z1, x1 + ox0, y1 + oy0, z1 + oz0, x1 + ox1, y1 + oy1, z1 + oz1)
            out.pushTri(x0, y0, z0, x0 + ox1, y0 + oy1, z0 + oz1, x0 + ox0, y0 + oy0, z0 + oz0)
        }
    }

    private fun addSmoothShroudRing(
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
            // Outer curved wall
            out.pushSmoothQuad(
                cx + c0 * rOut, cy + s0 * rOut, z0, c0, s0, 0f,
                cx + c1 * rOut, cy + s1 * rOut, z0, c1, s1, 0f,
                cx + c1 * rOut, cy + s1 * rOut, z1, c1, s1, 0f,
                cx + c0 * rOut, cy + s0 * rOut, z1, c0, s0, 0f
            )
            // Inner curved wall
            out.pushSmoothQuad(
                cx + c1 * rIn, cy + s1 * rIn, z0, -c1, -s1, 0f,
                cx + c0 * rIn, cy + s0 * rIn, z0, -c0, -s0, 0f,
                cx + c0 * rIn, cy + s0 * rIn, z1, -c0, -s0, 0f,
                cx + c1 * rIn, cy + s1 * rIn, z1, -c1, -s1, 0f
            )
            // Top & bottom rims
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

    /**
     * Sculpted 3-blade twisted carbon-fiber airfoil (root -> 45% -> 80% span; outer 80%..100% is the painted tip cap).
     */
    private fun addTwistedAirfoilPropellerBody(
        out: FloatCollector,
        cx: Float,
        cy: Float,
        cz: Float,
        radius: Float,
        spinSign: Int
    ) {
        val sgn = if (spinSign >= 0) 1f else -1f
        val stationsR = floatArrayOf(0.11f, 0.44f, 0.80f)
        val stationsChord = floatArrayOf(0.115f, 0.168f, 0.118f)
        val stationsPitchRad = floatArrayOf(0.44f, 0.28f, 0.17f)
        val stationsSweep = floatArrayOf(0.00f, 0.025f, 0.010f)

        for (b in 0..2) {
            val ang = (b * 2.0 * PI / 3.0).toFloat()
            val rx = cos(ang)
            val ry = sin(ang)
            // Leading-edge tangent along rotation direction
            val lx = -ry * sgn
            val ly = rx * sgn

            for (s in 0..1) {
                addAirfoilSegment(
                    out, cx, cy, cz, rx, ry, lx, ly, radius,
                    stationsR[s], stationsChord[s], stationsPitchRad[s], stationsSweep[s],
                    stationsR[s + 1], stationsChord[s + 1], stationsPitchRad[s + 1], stationsSweep[s + 1]
                )
            }
        }
    }

    /**
     * Outer 80%..100% high-visibility painted propeller blade tips so rotation & stationary orientation pop clearly.
     */
    private fun addPropellerTipCapsOnly(
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
            val lx = -ry * sgn
            val ly = rx * sgn
            addAirfoilSegment(
                out, cx, cy, cz, rx, ry, lx, ly, radius,
                0.80f, 0.118f, 0.17f, 0.010f,
                1.00f, 0.052f, 0.11f, -0.022f
            )
        }
    }

    private fun addAirfoilSegment(
        out: FloatCollector,
        cx: Float,
        cy: Float,
        cz: Float,
        rx: Float,
        ry: Float,
        lx: Float,
        ly: Float,
        radius: Float,
        rFrac0: Float,
        cFrac0: Float,
        pitch0: Float,
        sweep0: Float,
        rFrac1: Float,
        cFrac1: Float,
        pitch1: Float,
        sweep1: Float
    ) {
        val r0 = radius * rFrac0
        val r1 = radius * rFrac1
        val c0 = radius * cFrac0
        val c1 = radius * cFrac1
        val sw0 = radius * sweep0
        val sw1 = radius * sweep1
        val cp0 = cos(pitch0)
        val sp0 = sin(pitch0)
        val cp1 = cos(pitch1)
        val sp1 = sin(pitch1)

        // Leading edge (+0.58 * chord), cambered crest (+0.12 * chord), trailing edge (-0.55 * chord)
        val xLe0 = cx + rx * r0 + lx * (sw0 + 0.58f * c0 * cp0)
        val yLe0 = cy + ry * r0 + ly * (sw0 + 0.58f * c0 * cp0)
        val zLe0 = cz + 0.58f * c0 * sp0

        val xMid0 = cx + rx * r0 + lx * (sw0 + 0.06f * c0 * cp0)
        val yMid0 = cy + ry * r0 + ly * (sw0 + 0.06f * c0 * cp0)
        val zMidTop0 = cz + 0.06f * c0 * sp0 + 0.0019f
        val zMidBot0 = cz + 0.06f * c0 * sp0 - 0.0012f

        val xTe0 = cx + rx * r0 + lx * (sw0 - 0.54f * c0 * cp0)
        val yTe0 = cy + ry * r0 + ly * (sw0 - 0.54f * c0 * cp0)
        val zTe0 = cz - 0.54f * c0 * sp0

        val xLe1 = cx + rx * r1 + lx * (sw1 + 0.58f * c1 * cp1)
        val yLe1 = cy + ry * r1 + ly * (sw1 + 0.58f * c1 * cp1)
        val zLe1 = cz + 0.58f * c1 * sp1

        val xMid1 = cx + rx * r1 + lx * (sw1 + 0.06f * c1 * cp1)
        val yMid1 = cy + ry * r1 + ly * (sw1 + 0.06f * c1 * cp1)
        val zMidTop1 = cz + 0.06f * c1 * sp1 + 0.0015f
        val zMidBot1 = cz + 0.06f * c1 * sp1 - 0.0010f

        val xTe1 = cx + rx * r1 + lx * (sw1 - 0.54f * c1 * cp1)
        val yTe1 = cy + ry * r1 + ly * (sw1 - 0.54f * c1 * cp1)
        val zTe1 = cz - 0.54f * c1 * sp1

        // Upper cambered airfoil surface (2 chordwise quads for smooth specular roll-off)
        out.pushQuad(xLe0, yLe0, zLe0, xLe1, yLe1, zLe1, xMid1, yMid1, zMidTop1, xMid0, yMid0, zMidTop0)
        out.pushQuad(xMid0, yMid0, zMidTop0, xMid1, yMid1, zMidTop1, xTe1, yTe1, zTe1, xTe0, yTe0, zTe0)
        // Lower airfoil surface
        out.pushQuad(xMid0, yMid0, zMidBot0, xMid1, yMid1, zMidBot1, xLe1, yLe1, zLe1, xLe0, yLe0, zLe0)
        out.pushQuad(xTe0, yTe0, zTe0, xTe1, yTe1, zTe1, xMid1, yMid1, zMidBot1, xMid0, yMid0, zMidBot0)
    }

    private fun appendAuxRotorDiscFeathered(out: FloatCollector): AuxSpan {
        val start = out.vertexCount
        val segs = 32
        val step = (PI * 2.0 / segs).toFloat()
        val rInner = 0.14f
        val rMid = 0.68f
        for (i in 0 until segs) {
            val a0 = i * step
            val a1 = (i + 1) * step
            val c0 = cos(a0)
            val s0 = sin(a0)
            val c1 = cos(a1)
            val s1 = sin(a1)
            // Inner annulus: soft hub -> mid-span peak
            out.pushFadeVertexXY(c0 * rInner, s0 * rInner, 0f, 0.18f)
            out.pushFadeVertexXY(c0 * rMid, s0 * rMid, 0f, 0.86f)
            out.pushFadeVertexXY(c1 * rMid, s1 * rMid, 0f, 0.86f)

            out.pushFadeVertexXY(c0 * rInner, s0 * rInner, 0f, 0.18f)
            out.pushFadeVertexXY(c1 * rMid, s1 * rMid, 0f, 0.86f)
            out.pushFadeVertexXY(c1 * rInner, s1 * rInner, 0f, 0.18f)

            // Outer annulus: mid-span peak -> feathered tip edge
            out.pushFadeVertexXY(c0 * rMid, s0 * rMid, 0f, 0.86f)
            out.pushFadeVertexXY(c0, s0, 0f, 0.24f)
            out.pushFadeVertexXY(c1, s1, 0f, 0.24f)

            out.pushFadeVertexXY(c0 * rMid, s0 * rMid, 0f, 0.86f)
            out.pushFadeVertexXY(c1, s1, 0f, 0.24f)
            out.pushFadeVertexXY(c1 * rMid, s1 * rMid, 0f, 0.86f)
        }
        return AuxSpan(start, out.vertexCount - start)
    }

    private fun appendAuxRotorRingFeathered(out: FloatCollector): AuxSpan {
        val start = out.vertexCount
        val segs = 32
        val rIn = 0.91f
        val rMid = 0.97f
        val rOut = 1.01f
        val step = (PI * 2.0 / segs).toFloat()
        for (i in 0 until segs) {
            val a0 = i * step
            val a1 = (i + 1) * step
            val c0 = cos(a0)
            val s0 = sin(a0)
            val c1 = cos(a1)
            val s1 = sin(a1)
            out.pushFadeVertexXY(c0 * rIn, s0 * rIn, 0f, 0.16f)
            out.pushFadeVertexXY(c0 * rMid, s0 * rMid, 0f, 0.98f)
            out.pushFadeVertexXY(c1 * rMid, s1 * rMid, 0f, 0.98f)

            out.pushFadeVertexXY(c0 * rIn, s0 * rIn, 0f, 0.16f)
            out.pushFadeVertexXY(c1 * rMid, s1 * rMid, 0f, 0.98f)
            out.pushFadeVertexXY(c1 * rIn, s1 * rIn, 0f, 0.16f)

            out.pushFadeVertexXY(c0 * rMid, s0 * rMid, 0f, 0.98f)
            out.pushFadeVertexXY(c0 * rOut, s0 * rOut, 0f, 0.12f)
            out.pushFadeVertexXY(c1 * rOut, s1 * rOut, 0f, 0.12f)

            out.pushFadeVertexXY(c0 * rMid, s0 * rMid, 0f, 0.98f)
            out.pushFadeVertexXY(c1 * rOut, s1 * rOut, 0f, 0.12f)
            out.pushFadeVertexXY(c1 * rMid, s1 * rMid, 0f, 0.98f)
        }
        return AuxSpan(start, out.vertexCount - start)
    }

    /**
     * 3-lobe directional motion-blur wake fan matching the 3-blade propeller:
     * Each lobe starts right at the physical blade leading edge (`fade = 0.98`) and trails smoothly
     * behind the rotation direction (`-spinSign`) over `92 deg` down to `fade = 0.02`.
     */
    private fun appendAuxRotorSweep3Lobe(out: FloatCollector, spinSign: Int): AuxSpan {
        val start = out.vertexCount
        val arcSegs = 8
        val wakeArcRad = 1.60f
        val dir = if (spinSign >= 0) -1f else 1f
        val rHub = 0.16f
        val rTip = 0.98f

        for (blade in 0..2) {
            val leadAng = (blade * 2.0 * PI / 3.0).toFloat()
            for (i in 0 until arcSegs) {
                val f0 = i.toFloat() / arcSegs
                val f1 = (i + 1).toFloat() / arcSegs
                val a0 = leadAng + dir * f0 * wakeArcRad
                val a1 = leadAng + dir * f1 * wakeArcRad
                val fade0 = ((1f - f0) * (1f - f0) * 0.96f + 0.02f).coerceIn(0.02f, 0.98f)
                val fade1 = ((1f - f1) * (1f - f1) * 0.96f + 0.02f).coerceIn(0.02f, 0.98f)
                val c0 = cos(a0)
                val s0 = sin(a0)
                val c1 = cos(a1)
                val s1 = sin(a1)
                val hubFade0 = min(1f, fade0 * 0.35f + 0.02f)
                val hubFade1 = min(1f, fade1 * 0.35f + 0.02f)

                out.pushFadeVertexXY(c0 * rHub, s0 * rHub, 0f, hubFade0)
                out.pushFadeVertexXY(c0 * rTip, s0 * rTip, 0f, fade0)
                out.pushFadeVertexXY(c1 * rTip, s1 * rTip, 0f, fade1)

                out.pushFadeVertexXY(c0 * rHub, s0 * rHub, 0f, hubFade0)
                out.pushFadeVertexXY(c1 * rTip, s1 * rTip, 0f, fade1)
                out.pushFadeVertexXY(c1 * rHub, s1 * rHub, 0f, hubFade1)
            }
        }
        return AuxSpan(start, out.vertexCount - start)
    }

    private fun appendAuxUnitCylinderZ(out: FloatCollector): AuxSpan {
        val start = out.vertexCount
        addSmoothCylinderZ(out, 0f, 0f, 0f, 1f, 1f, 1f, 12)
        return AuxSpan(start, out.vertexCount - start)
    }

    private fun appendAuxUnitConeZ(out: FloatCollector): AuxSpan {
        val start = out.vertexCount
        addSmoothCylinderZ(out, 0f, 0f, 0f, 1f, 1f, 0.02f, 12)
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
