package com.radiomaster.egogcs.ui

import com.radiomaster.egogcs.ui.GcsMotionSpec.DurationToken
import com.radiomaster.egogcs.ui.GcsMotionSpec.SpringToken
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class GcsMotionSpecTest {

    @Test
    fun resolveEffectiveProfile_respectsSystemAccessibilityAndPowerSaveLinkage() {
        // 标准渲染模式 + 标准动效 -> 标准物理档
        assertEquals(
            GcsMotionSpec.PROFILE_STANDARD,
            GcsMotionSpec.resolveEffectiveProfile(
                userStyle = GcsMotionSpec.PROFILE_STANDARD,
                perfMode = 0,
                systemAnimEnabled = true
            )
        )

        // 省电 30帧模式 (perfMode == 2) 下，默认标准动效自动降级为极简省电档
        assertEquals(
            GcsMotionSpec.PROFILE_MINIMAL,
            GcsMotionSpec.resolveEffectiveProfile(
                userStyle = GcsMotionSpec.PROFILE_STANDARD,
                perfMode = 2,
                systemAnimEnabled = true
            )
        )

        // 省电 30帧模式下，用户显式指定「敏捷利落」档位则保留用户选择
        assertEquals(
            GcsMotionSpec.PROFILE_SNAPPY,
            GcsMotionSpec.resolveEffectiveProfile(
                userStyle = GcsMotionSpec.PROFILE_SNAPPY,
                perfMode = 2,
                systemAnimEnabled = true
            )
        )

        // 系统无障碍关闭全局动画时，任何档位强制降级为极简档
        assertEquals(
            GcsMotionSpec.PROFILE_MINIMAL,
            GcsMotionSpec.resolveEffectiveProfile(
                userStyle = GcsMotionSpec.PROFILE_SNAPPY,
                perfMode = 0,
                systemAnimEnabled = false
            )
        )
    }

    @Test
    fun durationTokens_maintainMonotonicHierarchyAndFloorBoundsAcrossProfiles() {
        val orderedTokens = listOf(
            DurationToken.INSTANT,
            DurationToken.MICRO,
            DurationToken.FAST,
            DurationToken.STANDARD,
            DurationToken.SPATIAL,
            DurationToken.EMPHASIZED
        )

        for (profile in listOf(
            GcsMotionSpec.PROFILE_STANDARD,
            GcsMotionSpec.PROFILE_SNAPPY,
            GcsMotionSpec.PROFILE_MINIMAL
        )) {
            var prev = 0L
            for (token in orderedTokens) {
                val scaled = GcsMotionSpec.scaleDurationMs(token, profile)
                assertTrue(
                    "Scaled duration ($scaled) must respect floor (${token.minFloorMs})",
                    scaled >= token.minFloorMs
                )
                assertTrue(
                    "Duration ladder must stay non-decreasing ($prev -> $scaled) in profile $profile",
                    scaled >= prev
                )
                prev = scaled
            }
        }

        // 同一 Token 下：标准档 >= 敏捷档 >= 极简档
        for (token in orderedTokens) {
            val std = GcsMotionSpec.scaleDurationMs(token, GcsMotionSpec.PROFILE_STANDARD)
            val snappy = GcsMotionSpec.scaleDurationMs(token, GcsMotionSpec.PROFILE_SNAPPY)
            val minimal = GcsMotionSpec.scaleDurationMs(token, GcsMotionSpec.PROFILE_MINIMAL)
            assertTrue(std >= snappy)
            assertTrue(snappy >= minimal)
        }
    }

    @Test
    fun springScalingAndStagger_stayPhysicallyBounded() {
        for (token in SpringToken.values()) {
            val stdStiff = GcsMotionSpec.scaleSpringStiffness(token, GcsMotionSpec.PROFILE_STANDARD)
            val snappyStiff = GcsMotionSpec.scaleSpringStiffness(token, GcsMotionSpec.PROFILE_SNAPPY)
            val minStiff = GcsMotionSpec.scaleSpringStiffness(token, GcsMotionSpec.PROFILE_MINIMAL)
            assertTrue(minStiff > snappyStiff && snappyStiff > stdStiff)

            for (profile in 0..2) {
                val damping = GcsMotionSpec.scaleSpringDamping(token, profile)
                assertTrue("Damping must stay in (0.4, 1.0): $damping", damping in 0.45f..0.99f)
            }
        }

        assertEquals(18L, GcsMotionSpec.staggerStepMs(GcsMotionSpec.PROFILE_STANDARD))
        assertEquals(12L, GcsMotionSpec.staggerStepMs(GcsMotionSpec.PROFILE_SNAPPY))
        assertEquals(0L, GcsMotionSpec.staggerStepMs(GcsMotionSpec.PROFILE_MINIMAL))

        assertTrue(
            GcsMotionSpec.cameraDampingForProfile(GcsMotionSpec.PROFILE_MINIMAL) >
                GcsMotionSpec.cameraDampingForProfile(GcsMotionSpec.PROFILE_SNAPPY)
        )
        assertTrue(
            GcsMotionSpec.cameraDampingForProfile(GcsMotionSpec.PROFILE_SNAPPY) >
                GcsMotionSpec.cameraDampingForProfile(GcsMotionSpec.PROFILE_STANDARD)
        )
    }
}
