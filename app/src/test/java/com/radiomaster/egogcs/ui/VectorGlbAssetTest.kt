package com.radiomaster.egogcs.ui

import java.io.File
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.security.MessageDigest
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import org.json.JSONArray
import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Assume.assumeTrue
import org.junit.Test

class VectorGlbAssetTest {
    @Test fun originalModelIsIncludedWithIndependentTiltGroups() {
        val file = listOf(
            File("src/main/assets/vector_uav.glb"),
            File("app/src/main/assets/vector_uav.glb")
        ).firstOrNull { it.isFile }
        assumeTrue("External vector_uav.glb asset is not bundled in this workspace", file != null)
        val bytes = requireNotNull(file).readBytes()
        val hash = MessageDigest.getInstance("SHA-256").digest(bytes).joinToString("") { "%02x".format(it) }
        assertEquals("0ff402d8e9a7e9470e0f2c6b010955f035e5667617715c4ef02a303e6af14ab5", hash)
        val model = VectorGlbAsset.parse(ByteBuffer.wrap(bytes))
        assertEquals(12, model.parts.size)
        assertTrue(model.parts.sumOf { it.vertexCount } > 1_000_000)
        assertEquals(0.11667f, model.frontPivot[0], 0.000001f)
        assertEquals(-0.11667f, model.rearPivot[0], 0.000001f)
        val front = model.parts.filter { it.group == 1 }.map { it.name }.toSet()
        val rear = model.parts.filter { it.group == 2 }.map { it.name }.toSet()
        assertTrue(front.containsAll(setOf("motor_m1_visual", "motor_m3_visual", "propeller_m1_visual", "propeller_m3_visual")))
        assertTrue(rear.containsAll(setOf("motor_m2_visual", "motor_m4_visual", "propeller_m2_visual", "propeller_m4_visual")))
        assertFalse(front.any { it in rear })

        val bin = ByteBuffer.wrap(bytes, model.binaryOffset, model.binaryLength)
        val prepared = VectorUavMeshBuilder.prepareFromGlb(model, bin.slice())
        assertTrue(
            "Expected prepared.totalVertices in 6_000..180_000, got ${prepared.totalVertices}",
            prepared.totalVertices in 6_000..180_000
        )
        assertEquals(4, prepared.rotorHubs.size)
        assertAllVertexNormalsUnitLength(prepared)
    }

    @Test fun highDensityTwelvePartGlbOverOneMillionVerticesDecimatesWithinBudget() {
        val glbBytes = buildSyntheticHighDensityTiltUavGlb()
        val model = VectorGlbAsset.parse(ByteBuffer.wrap(glbBytes))
        assertEquals(12, model.parts.size)
        val rawTotal = model.parts.sumOf { it.vertexCount }
        assertTrue("Expected > 1,000,000 raw GLB vertices, got $rawTotal", rawTotal > 1_000_000)
        assertEquals(0.11667f, model.frontPivot[0], 0.000001f)
        assertEquals(-0.11667f, model.rearPivot[0], 0.000001f)

        val front = model.parts.filter { it.group == 1 }.map { it.name }.toSet()
        val rear = model.parts.filter { it.group == 2 }.map { it.name }.toSet()
        assertTrue(front.containsAll(setOf("motor_m1_visual", "motor_m3_visual", "propeller_m1_visual", "propeller_m3_visual")))
        assertTrue(rear.containsAll(setOf("motor_m2_visual", "motor_m4_visual", "propeller_m2_visual", "propeller_m4_visual")))
        assertFalse(front.any { it in rear })

        val bin = ByteBuffer.wrap(glbBytes, model.binaryOffset, model.binaryLength)
        val prepared = VectorUavMeshBuilder.prepareFromGlb(model, bin.slice())
        assertFalse(prepared.isProceduralFallback)
        assertTrue(
            "Expected decimated totalVertices in 6_000..180_000, got ${prepared.totalVertices}",
            prepared.totalVertices in 6_000..180_000
        )
        assertEquals(4, prepared.rotorHubs.size)
        assertEquals(0.11667f, prepared.rotorHubs[0].x, 0.002f)
        assertEquals(-0.147644f, prepared.rotorHubs[0].y, 0.002f)
        assertEquals(-0.11667f, prepared.rotorHubs[1].x, 0.002f)
        assertEquals(0.147644f, prepared.rotorHubs[1].y, 0.002f)
        val rotorIds = prepared.parts.filter { it.rotorIndex > 0 }.map { it.rotorIndex }.toSet()
        assertEquals(setOf(1, 2, 3, 4), rotorIds)
        assertAllVertexNormalsUnitLength(prepared)
    }

    private fun buildSyntheticHighDensityTiltUavGlb(): ByteArray {
        // 3 distinct dense 3D surface meshes reused across 4 fuselage + 4 motor + 4 propeller nodes = 12 parts, 1,080,000 raw vertices
        val bodyVerts = 120_000 // 200 x 100 quads * 6
        val motorVerts = 90_000 // 150 x 100 quads * 6
        val propVerts = 60_000 // 100 x 100 quads * 6
        val totalUniqueVerts = bodyVerts + motorVerts + propVerts
        val binLength = totalUniqueVerts * 24 // 12B position + 12B normal per vertex (~6.48 MB)
        val binBuf = ByteBuffer.allocate(binLength).order(ByteOrder.LITTLE_ENDIAN)

        fun writeTorusOrCylinderShell(uSegs: Int, vSegs: Int, rx: Float, ry: Float, rz: Float): Pair<Int, Int> {
            val posOffset = binBuf.position()
            for (u in 0 until uSegs) {
                val a0 = (u * 2.0 * PI / uSegs).toFloat()
                val a1 = ((u + 1) * 2.0 * PI / uSegs).toFloat()
                val cu0 = cos(a0)
                val su0 = sin(a0)
                val cu1 = cos(a1)
                val su1 = sin(a1)
                for (v in 0 until vSegs) {
                    val b0 = (v * PI / vSegs - PI * 0.5).toFloat()
                    val b1 = ((v + 1) * PI / vSegs - PI * 0.5).toFloat()
                    val cb0 = cos(b0)
                    val sb0 = sin(b0)
                    val cb1 = cos(b1)
                    val sb1 = sin(b1)

                    fun putPos(cu: Float, su: Float, cb: Float, sb: Float) {
                        binBuf.putFloat(cu * cb * rx)
                        binBuf.putFloat(su * cb * ry)
                        binBuf.putFloat(sb * rz)
                    }
                    putPos(cu0, su0, cb0, sb0)
                    putPos(cu1, su1, cb0, sb0)
                    putPos(cu1, su1, cb1, sb1)
                    putPos(cu0, su0, cb0, sb0)
                    putPos(cu1, su1, cb1, sb1)
                    putPos(cu0, su0, cb1, sb1)
                }
            }
            val normOffset = binBuf.position()
            for (u in 0 until uSegs) {
                val a0 = (u * 2.0 * PI / uSegs).toFloat()
                val a1 = ((u + 1) * 2.0 * PI / uSegs).toFloat()
                val cu0 = cos(a0)
                val su0 = sin(a0)
                val cu1 = cos(a1)
                val su1 = sin(a1)
                for (v in 0 until vSegs) {
                    val b0 = (v * PI / vSegs - PI * 0.5).toFloat()
                    val b1 = ((v + 1) * PI / vSegs - PI * 0.5).toFloat()
                    val cb0 = cos(b0)
                    val sb0 = sin(b0)
                    val cb1 = cos(b1)
                    val sb1 = sin(b1)

                    fun putNorm(cu: Float, su: Float, cb: Float, sb: Float) {
                        binBuf.putFloat(cu * cb)
                        binBuf.putFloat(su * cb)
                        binBuf.putFloat(sb)
                    }
                    putNorm(cu0, su0, cb0, sb0)
                    putNorm(cu1, su1, cb0, sb0)
                    putNorm(cu1, su1, cb1, sb1)
                    putNorm(cu0, su0, cb0, sb0)
                    putNorm(cu1, su1, cb1, sb1)
                    putNorm(cu0, su0, cb1, sb1)
                }
            }
            return posOffset to normOffset
        }

        val (bodyPosOff, bodyNormOff) = writeTorusOrCylinderShell(200, 100, 0.092f, 0.048f, 0.028f)
        val (motorPosOff, motorNormOff) = writeTorusOrCylinderShell(150, 100, 0.017f, 0.017f, 0.012f)
        val (propPosOff, propNormOff) = writeTorusOrCylinderShell(100, 100, 0.072f, 0.072f, 0.004f)
        assertEquals(binLength, binBuf.position())

        val bufferViews = JSONArray()
        val accessors = JSONArray()
        val meshes = JSONArray()

        fun registerMesh(posOff: Int, normOff: Int, count: Int): Int {
            val posViewIdx = bufferViews.length()
            bufferViews.put(JSONObject().put("buffer", 0).put("byteOffset", posOff).put("byteLength", count * 12))
            val normViewIdx = bufferViews.length()
            bufferViews.put(JSONObject().put("buffer", 0).put("byteOffset", normOff).put("byteLength", count * 12))

            val posAccIdx = accessors.length()
            accessors.put(JSONObject().put("bufferView", posViewIdx).put("componentType", 5126).put("count", count).put("type", "VEC3"))
            val normAccIdx = accessors.length()
            accessors.put(JSONObject().put("bufferView", normViewIdx).put("componentType", 5126).put("count", count).put("type", "VEC3"))

            val prim = JSONObject()
                .put("mode", 4)
                .put("attributes", JSONObject().put("POSITION", posAccIdx).put("NORMAL", normAccIdx))
                .put("material", 0)
            val meshIdx = meshes.length()
            meshes.put(JSONObject().put("primitives", JSONArray().put(prim)))
            return meshIdx
        }

        val bodyMesh = registerMesh(bodyPosOff, bodyNormOff, bodyVerts)
        val motorMesh = registerMesh(motorPosOff, motorNormOff, motorVerts)
        val propMesh = registerMesh(propPosOff, propNormOff, propVerts)

        fun vec3(x: Double, y: Double, z: Double) = JSONArray().put(x).put(y).put(z)

        val nodes = JSONArray()
            // 0: base_link (group 0, mesh 0)
            .put(JSONObject().put("name", "base_link").put("mesh", bodyMesh).put("children", JSONArray(intArrayOf(1, 2, 3, 4, 5))))
            // 1..3: three more fuselage visual parts (group 0)
            .put(JSONObject().put("name", "avionics_deck_visual").put("mesh", bodyMesh).put("translation", vec3(0.0, 0.0, 0.025)))
            .put(JSONObject().put("name", "lidar_cage_visual").put("mesh", bodyMesh).put("translation", vec3(0.0, 0.0, 0.065)).put("scale", vec3(0.45, 0.45, 0.55)))
            .put(JSONObject().put("name", "battery_bay_visual").put("mesh", bodyMesh).put("translation", vec3(0.0, 0.0, -0.022)).put("scale", vec3(0.65, 0.55, 0.45)))
            // 4: joint_f (group 1 pivot)
            .put(
                JSONObject()
                    .put("name", "joint_f")
                    .put("translation", vec3(0.11667, 0.0, 0.056))
                    .put("extras", JSONObject().put("axis_local", vec3(0.0, 1.0, 0.0)))
                    .put("children", JSONArray(intArrayOf(6, 7, 8, 9)))
            )
            // 5: joint_b (group 2 pivot)
            .put(
                JSONObject()
                    .put("name", "joint_b")
                    .put("translation", vec3(-0.11667, 0.0, 0.056))
                    .put("extras", JSONObject().put("axis_local", vec3(0.0, 1.0, 0.0)))
                    .put("children", JSONArray(intArrayOf(10, 11, 12, 13)))
            )
            // 6..9: front motors & props (group 1)
            .put(JSONObject().put("name", "motor_m1_visual").put("mesh", motorMesh).put("translation", vec3(0.0, -0.147644, 0.0)))
            .put(JSONObject().put("name", "motor_m3_visual").put("mesh", motorMesh).put("translation", vec3(0.0, 0.147644, 0.0)))
            .put(JSONObject().put("name", "propeller_m1_visual").put("mesh", propMesh).put("translation", vec3(0.0, -0.147644, 0.026)))
            .put(JSONObject().put("name", "propeller_m3_visual").put("mesh", propMesh).put("translation", vec3(0.0, 0.147644, 0.026)))
            // 10..13: rear motors & props (group 2)
            .put(JSONObject().put("name", "motor_m2_visual").put("mesh", motorMesh).put("translation", vec3(0.0, 0.147644, 0.0)))
            .put(JSONObject().put("name", "motor_m4_visual").put("mesh", motorMesh).put("translation", vec3(0.0, -0.147644, 0.0)))
            .put(JSONObject().put("name", "propeller_m2_visual").put("mesh", propMesh).put("translation", vec3(0.0, 0.147644, 0.026)))
            .put(JSONObject().put("name", "propeller_m4_visual").put("mesh", propMesh).put("translation", vec3(0.0, -0.147644, 0.026)))

        val doc = JSONObject()
            .put("asset", JSONObject().put("version", "2.0"))
            .put("buffers", JSONArray().put(JSONObject().put("byteLength", binLength)))
            .put("bufferViews", bufferViews)
            .put("accessors", accessors)
            .put(
                "materials",
                JSONArray().put(
                    JSONObject().put(
                        "pbrMetallicRoughness",
                        JSONObject().put("baseColorFactor", JSONArray(doubleArrayOf(0.28, 0.34, 0.42, 1.0)))
                    )
                )
            )
            .put("meshes", meshes)
            .put("nodes", nodes)

        val jsonRaw = doc.toString().toByteArray(Charsets.UTF_8)
        val jsonPad = (4 - (jsonRaw.size % 4)) % 4
        val jsonChunkLen = jsonRaw.size + jsonPad
        val totalLen = 12 + 8 + jsonChunkLen + 8 + binLength

        val glb = ByteBuffer.allocate(totalLen).order(ByteOrder.LITTLE_ENDIAN)
        glb.putInt(0x46546c67) // "glTF"
        glb.putInt(2) // version 2
        glb.putInt(totalLen)
        glb.putInt(jsonChunkLen)
        glb.putInt(0x4e4f534a) // "JSON"
        glb.put(jsonRaw)
        repeat(jsonPad) { glb.put(' '.code.toByte()) }
        glb.putInt(binLength)
        glb.putInt(0x004e4942) // "BIN\0"
        glb.put(binBuf.array())
        return glb.array()
    }

    @Test fun proceduralTiltUavHasIndependentTiltGroupsAndValidNormals() {
        val prepared = VectorUavMeshBuilder.buildProceduralTiltUav()
        assertTrue(prepared.isProceduralFallback)
        assertEquals(0.11667f, prepared.frontPivot[0], 0.000001f)
        assertEquals(-0.11667f, prepared.rearPivot[0], 0.000001f)
        assertEquals(4, prepared.rotorHubs.size)

        val front = prepared.parts.filter { it.group == 1 }.map { it.name }.toSet()
        val rear = prepared.parts.filter { it.group == 2 }.map { it.name }.toSet()
        val body = prepared.parts.filter { it.group == 0 }.map { it.name }.toSet()
        assertTrue(body.isNotEmpty())
        assertTrue(front.containsAll(setOf("motor_m1_visual", "motor_m3_visual", "propeller_m1_visual", "propeller_m3_visual")))
        assertTrue(rear.containsAll(setOf("motor_m2_visual", "motor_m4_visual", "propeller_m2_visual", "propeller_m4_visual")))
        assertFalse(front.any { it in rear })

        val rotorIds = prepared.parts.filter { it.rotorIndex > 0 }.map { it.rotorIndex }.toSet()
        assertEquals(setOf(1, 2, 3, 4), rotorIds)
        assertTrue(prepared.parts.any { it.name.startsWith("prop_tip_") && it.emissive > 0.1f })

        assertAllVertexNormalsUnitLength(prepared)
    }

    @Test fun proceduralStandardQuadHasRigidAvionicsAndFourSpinningRotors() {
        val prepared = VectorUavMeshBuilder.buildProceduralStandardQuad()
        assertTrue(prepared.isProceduralFallback)
        assertEquals(4, prepared.rotorHubs.size)
        assertTrue(prepared.parts.all { it.group == 0 })
        assertTrue(prepared.rotorHubs.all { it.group == 0 })

        val rotorIds = prepared.parts.filter { it.rotorIndex > 0 }.map { it.rotorIndex }.toSet()
        assertEquals(setOf(1, 2, 3, 4), rotorIds)
        assertTrue(prepared.parts.any { it.name == "sensor_optics_visual" })
        assertTrue(prepared.parts.any { it.name == "stator_m1_visual" })
        assertTrue(prepared.parts.any { it.name.startsWith("prop_tip_") && it.emissive > 0.1f })

        assertAllVertexNormalsUnitLength(prepared)
    }

    private fun assertAllVertexNormalsUnitLength(prepared: VectorUavMeshBuilder.PreparedModel) {
        val buf = prepared.vertexBuffer.duplicate()
        assertEquals(prepared.totalVertices * VectorUavMeshBuilder.STRIDE_FLOATS, buf.limit())
        for (v in 0 until prepared.totalVertices) {
            val base = v * VectorUavMeshBuilder.STRIDE_FLOATS
            val px = buf.get(base)
            val py = buf.get(base + 1)
            val pz = buf.get(base + 2)
            val nx = buf.get(base + 3)
            val ny = buf.get(base + 4)
            val nz = buf.get(base + 5)
            assertTrue(px.isFinite() && py.isFinite() && pz.isFinite())
            val nLen = sqrt(nx * nx + ny * ny + nz * nz)
            assertEquals(1f, nLen, 0.01f)
        }
    }

    @Test fun materialDetailTexturePixelsAreCachedAndEncodeWovenCarbonAndBrushedGrain() {
        val first = VectorGlbRenderer.loadMaterialDetailPixels()
        val expectedBytes = VectorGlbRenderer.DETAIL_TEX_SIZE * VectorGlbRenderer.DETAIL_TEX_SIZE * 4
        assertEquals(0, first.position())
        assertEquals(expectedBytes, first.remaining())

        var minR = 255
        var maxR = 0
        var minG = 255
        var maxG = 0
        val copyA = ByteArray(expectedBytes)
        for (i in 0 until expectedBytes step 4) {
            val r = first.get().toInt() and 0xFF
            val g = first.get().toInt() and 0xFF
            val b = first.get().toInt() and 0xFF
            val a = first.get().toInt() and 0xFF
            copyA[i] = r.toByte()
            copyA[i + 1] = g.toByte()
            copyA[i + 2] = b.toByte()
            copyA[i + 3] = a.toByte()
            if (r < minR) minR = r
            if (r > maxR) maxR = r
            if (g < minG) minG = g
            if (g > maxG) maxG = g
            assertEquals(255, a)
        }
        assertTrue("Carbon weave channel must have clear relief contrast", maxR - minR >= 80)
        assertTrue("Brushed CNC grain channel must have micro-relief variation", maxG - minG >= 40)

        // Mutating position on first duplicate must not corrupt subsequent callers
        val second = VectorGlbRenderer.loadMaterialDetailPixels()
        assertEquals(0, second.position())
        assertEquals(expectedBytes, second.remaining())
        val copyB = ByteArray(expectedBytes)
        second.get(copyB)
        assertTrue(copyA.contentEquals(copyB))
    }

    @Test(expected = IllegalArgumentException::class)
    fun rejectsTruncatedFile() {
        VectorGlbAsset.parse(ByteBuffer.wrap(ByteArray(20)))
    }
}
