package com.radiomaster.egogcs.ui

import com.radiomaster.egogcs.hw.Ax12ThermalMonitor
import com.radiomaster.egogcs.model.GcsSharedHub
import java.io.File
import java.nio.file.Files
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ThermalFrameGovernorTest {

    @Test
    fun socThermalEscalationHysteresisAndGradualRecoveryAreDeterministic() {
        val hub = GcsSharedHub()
        val gov = ThermalFrameGovernor(
            stepDownDwellNs = 1_500_000_000L,
            severeStepDownDwellNs = 500_000_000L,
            stepUpRecoveryDwellNs = 4_500_000_000L,
            touchBoostWindowNs = 400_000_000L
        )

        var nowNs = 1_000_000_000L
        hub.cpuTempCelsius = 48.0f
        assertEquals(60, gov.evaluate(nowNs, hub))
        assertEquals(ThermalFrameGovernor.Level.NOMINAL, gov.currentLevel)

        // Rise to 64°C (WARM): holds 60 FPS during 1.5s dwell, then steps down to 45 FPS
        hub.cpuTempCelsius = 64.0f
        assertEquals(60, gov.evaluate(nowNs, hub))
        nowNs += 1_000_000_000L
        assertEquals(60, gov.evaluate(nowNs, hub))
        nowNs += 600_000_000L
        assertEquals(45, gov.evaluate(nowNs, hub))
        assertEquals(ThermalFrameGovernor.Level.WARM, gov.currentLevel)
        assertEquals(45, hub.effectiveHudTargetFps())

        // Hysteresis check: cooling slightly to 60.0°C (above 58.5°C hold) must NOT bounce back to 60 FPS
        hub.cpuTempCelsius = 60.0f
        repeat(10) {
            nowNs += 1_000_000_000L
            assertEquals(45, gov.evaluate(nowNs, hub))
        }

        // Escalate to 79.5°C (SEVERE): fast 0.5s dwell -> 20 FPS
        hub.cpuTempCelsius = 79.5f
        assertEquals(45, gov.evaluate(nowNs, hub))
        nowNs += 550_000_000L
        assertEquals(20, gov.evaluate(nowNs, hub))
        assertEquals(ThermalFrameGovernor.Level.SEVERE, gov.currentLevel)
        assertEquals(7, gov.currentTargetHudHz)

        // Escalate to 86.0°C (CRITICAL): immediate 0ms intervention -> 15 FPS
        hub.cpuTempCelsius = 86.0f
        nowNs += 16_666_667L
        assertEquals(15, gov.evaluate(nowNs, hub))
        assertEquals(ThermalFrameGovernor.Level.CRITICAL, gov.currentLevel)
        assertEquals(5, gov.currentTargetHudHz)

        // Cool all the way down to 45°C: must recover ONE tier per 4.5s dwell (15 -> 20 -> 30 -> 45 -> 60)
        hub.cpuTempCelsius = 45.0f
        assertEquals(15, gov.evaluate(nowNs, hub))

        val expectedLadder = listOf(20, 30, 45, 60)
        for (expectedFps in expectedLadder) {
            nowNs += 4_600_000_000L
            assertEquals(expectedFps, gov.evaluate(nowNs, hub))
        }
        assertEquals(ThermalFrameGovernor.Level.NOMINAL, gov.currentLevel)
    }

    @Test
    fun closedLoopCpuFrameTimeLoadStabilizesWithoutPingPongOscillation() {
        val hub = GcsSharedHub()
        val gov = ThermalFrameGovernor()
        var nowNs = 2_000_000_000L

        // Cold-start shader/VBO spikes during warmup frames must be ignored
        repeat(ThermalFrameGovernor.WARMUP_FRAMES) {
            gov.recordFrameWork(45_000_000L) // 45 ms cold spike
            nowNs += 16_666_667L
            assertEquals(60, gov.evaluate(nowNs, hub))
        }

        // Sustained 18.5 ms CPU work per frame exceeds 60 FPS & 45 FPS budgets -> settles cleanly at 30 FPS (MODERATE)
        repeat(180) {
            gov.recordFrameWork(18_500_000L)
            nowNs += 33_333_333L
            gov.evaluate(nowNs, hub)
        }
        assertEquals(ThermalFrameGovernor.Level.MODERATE, gov.currentLevel)
        assertEquals(30, gov.currentTargetFps)

        // Maintain constant 18.5 ms per-frame work for another 20 seconds: must stay locked at 30 FPS with zero oscillation!
        repeat(600) {
            gov.recordFrameWork(18_500_000L)
            nowNs += 33_333_333L
            assertEquals(30, gov.evaluate(nowNs, hub))
        }
    }

    @Test
    fun interactiveTouchBoostLiftsModerateTierButNeverOverridesSevereOrCriticalSafety() {
        val hub = GcsSharedHub()
        val gov = ThermalFrameGovernor(
            stepDownDwellNs = 100_000_000L,
            severeStepDownDwellNs = 100_000_000L,
            touchBoostWindowNs = 400_000_000L
        )
        var nowNs = 5_000_000_000L

        // Reach MODERATE (30 FPS)
        hub.cpuTempCelsius = 72.0f
        gov.evaluate(nowNs, hub)
        nowNs += 200_000_000L
        assertEquals(30, gov.evaluate(nowNs, hub))

        // Active touch boosts MODERATE (30 FPS) -> 45 FPS for 400ms, then settles back to 30 FPS
        gov.notifyTouchInteraction(nowNs)
        assertEquals(45, gov.evaluate(nowNs + 100_000_000L, hub))
        assertEquals(30, gov.evaluate(nowNs + 500_000_000L, hub))

        // Reach SEVERE (20 FPS): touch interaction MUST NOT override thermal safety
        nowNs += 600_000_000L
        hub.cpuTempCelsius = 80.0f
        gov.evaluate(nowNs, hub)
        nowNs += 200_000_000L
        assertEquals(20, gov.evaluate(nowNs, hub))
        gov.notifyTouchInteraction(nowNs)
        assertEquals(20, gov.evaluate(nowNs + 50_000_000L, hub))

        // Power-save mode (perfMode == 2) caps ceiling at 30 FPS even on touch boost, while still stepping down to 20 FPS
        hub.perfMode = 2
        assertEquals(20, gov.evaluate(nowNs + 60_000_000L, hub))
        hub.cpuTempCelsius = 50.0f
        gov.reset()
        assertEquals(30, gov.evaluate(nowNs + 100_000_000L, hub))
    }

    @Test
    fun sysfsThermalAndDvfsParsersFilterAx12ZonesAndDetectKernelClamp() {
        val tempRoot = Files.createTempDirectory("ax12_thermal_test").toFile()
        try {
            val z0 = File(tempRoot, "thermal_zone0").apply { mkdirs() }
            File(z0, "type").writeText("battery\n")
            File(z0, "temp").writeText("43200\n")

            val z1 = File(tempRoot, "thermal_zone1").apply { mkdirs() }
            File(z1, "type").writeText("soc-thermal\n")
            File(z1, "temp").writeText("71500\n")

            val z2 = File(tempRoot, "thermal_zone2").apply { mkdirs() }
            File(z2, "type").writeText("gpu-thermal\n")
            File(z2, "temp").writeText("68200\n")

            assertFalse(Ax12ThermalMonitor.isCpuOrSocZoneType("battery"))
            assertFalse(Ax12ThermalMonitor.isCpuOrSocZoneType("pmic-therm"))
            assertTrue(Ax12ThermalMonitor.isCpuOrSocZoneType("soc-thermal"))
            assertTrue(Ax12ThermalMonitor.isCpuOrSocZoneType("tsens_tz_sensor0"))

            val zones = Ax12ThermalMonitor.discoverSysfsCpuThermalFiles(tempRoot)
            assertEquals(2, zones.size)
            assertEquals(71.5f, Ax12ThermalMonitor.readHottestTempCelsius(zones), 0.01f)
            assertEquals(42.5f, Ax12ThermalMonitor.normalizeRawThermalTempCelsius("425"), 0.01f)
            assertEquals(63.0f, Ax12ThermalMonitor.normalizeRawThermalTempCelsius("63"), 0.01f)

            val cpuRoot = File(tempRoot, "cpu").apply { mkdirs() }
            val pol0 = File(cpuRoot, "cpufreq/policy0").apply { mkdirs() }
            File(pol0, "scaling_max_freq").writeText("1416000\n")
            File(pol0, "cpuinfo_max_freq").writeText("1416000\n")

            val pol4 = File(cpuRoot, "cpufreq/policy4").apply { mkdirs() }
            File(pol4, "scaling_max_freq").writeText("1200000\n")
            File(pol4, "cpuinfo_max_freq").writeText("1800000\n")

            val pairs = Ax12ThermalMonitor.discoverCpuFreqCapPairs(cpuRoot)
            assertEquals(2, pairs.size)
            val capRatio = Ax12ThermalMonitor.readMinFreqCapRatio(pairs)
            assertEquals(1200000f / 1800000f, capRatio, 0.001f)
        } finally {
            tempRoot.deleteRecursively()
        }
    }
}
