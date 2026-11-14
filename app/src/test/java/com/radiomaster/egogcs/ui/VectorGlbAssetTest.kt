package com.radiomaster.egogcs.ui

import java.io.File
import java.nio.ByteBuffer
import java.security.MessageDigest
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class VectorGlbAssetTest {
    @Test fun originalModelIsIncludedWithIndependentTiltGroups() {
        val file = listOf(
            File("src/main/assets/vector_uav.glb"),
            File("app/src/main/assets/vector_uav.glb")
        ).first { it.isFile }
        val bytes = file.readBytes()
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
    }

    @Test(expected = IllegalArgumentException::class)
    fun rejectsTruncatedFile() {
        VectorGlbAsset.parse(ByteBuffer.wrap(ByteArray(20)))
    }
}
