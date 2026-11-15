package com.radiomaster.egogcs.ui

import java.io.File
import java.nio.ByteBuffer
import java.security.MessageDigest
import kotlin.math.sqrt
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
        assertTrue(prepared.totalVertices in 6_000..180_000)
        assertEquals(4, prepared.rotorHubs.size)
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
