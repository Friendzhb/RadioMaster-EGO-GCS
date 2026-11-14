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

    @Test(expected = IllegalArgumentException::class)
    fun rejectsTruncatedFile() {
        VectorGlbAsset.parse(ByteBuffer.wrap(ByteArray(20)))
    }
}
