package com.radiomaster.egogcs.hw

import android.content.Context
import android.os.Build
import android.os.PowerManager
import androidx.annotation.RequiresApi
import com.radiomaster.egogcs.model.GcsSharedHub
import java.io.File
import java.util.Locale
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit

/**
 * RadioMaster AX12 多源零权限 CPU 热负载采集器 ([Ax12ThermalMonitor])：
 *
 * 针对 RadioMaster AX12 (Android 9.0 / API 28 嵌入式主控 + 内置高频 ELRS 发射机热耦合机身) 以及高版本 Android 设备，
 * 构建四层无特权硬件热源融合：
 * 1. Linux `sysfs` SoC/CPU 核心热区 (`/sys/class/thermal/thermal_zone*`，适配 Rockchip `soc-thermal`/`cpu-thermal`、
 *    Qualcomm `tsens_tz_sensor*`、Allwinner `ths`、MTK `mtktscpu` 等，自动过滤电池/充电热区并归一化毫摄氏度)；
 * 2. Linux `cpufreq` 内核热限频比 (`scaling_max_freq / cpuinfo_max_freq`，直接捕获内核温控 Governor 对大核上限的压制)；
 * 3. Android OS `PowerManager` 热状态推送与预测热裕度 (`API 29+ getCurrentThermalStatus` / `API 30+ getThermalHeadroom`)；
 * 4. AX12 机壳与双 21700 电池仓耦合温度 (`BatteryManager.EXTRA_TEMPERATURE`，由 [Ax12HardwareBridge] 实时注入)。
 */
class Ax12ThermalMonitor(
    private val context: Context,
    private val hub: GcsSharedHub
) {
    private var monitorExecutor: ScheduledExecutorService? = null

    @Volatile private var cachedZoneTempFiles: List<File>? = null

    @Volatile private var cachedFreqCapPairs: List<Pair<File, File>>? = null

    @Volatile private var lastHeadroomPollMs: Long = 0L

    private var registeredListener: Any? = null

    fun start() {
        if (monitorExecutor != null) return
        val exec = Executors.newSingleThreadScheduledExecutor { r ->
            Thread(r, "Ax12ThermalMonitor").apply { isDaemon = true }
        }
        monitorExecutor = exec
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            registerOsThermalListenerApi29(exec)
        }
        exec.scheduleAtFixedRate({ pollThermalTelemetry() }, 0L, POLL_INTERVAL_MS, TimeUnit.MILLISECONDS)
    }

    fun stop() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            unregisterOsThermalListenerApi29()
        }
        monitorExecutor?.shutdownNow()
        monitorExecutor = null
    }

    @RequiresApi(Build.VERSION_CODES.Q)
    private fun registerOsThermalListenerApi29(exec: ScheduledExecutorService) {
        try {
            val pm = context.getSystemService(Context.POWER_SERVICE) as? PowerManager ?: return
            hub.osThermalStatus = pm.currentThermalStatus.coerceAtLeast(0)
            val listener = PowerManager.OnThermalStatusChangedListener { status ->
                hub.osThermalStatus = status.coerceAtLeast(0)
            }
            pm.addThermalStatusListener(exec, listener)
            registeredListener = listener
        } catch (_: Throwable) {
        }
    }

    @RequiresApi(Build.VERSION_CODES.Q)
    private fun unregisterOsThermalListenerApi29() {
        try {
            val pm = context.getSystemService(Context.POWER_SERVICE) as? PowerManager
            val listener = registeredListener as? PowerManager.OnThermalStatusChangedListener
            if (pm != null && listener != null) {
                pm.removeThermalStatusListener(listener)
            }
        } catch (_: Throwable) {
        } finally {
            registeredListener = null
        }
    }

    @RequiresApi(Build.VERSION_CODES.R)
    private fun pollThermalHeadroomApi30(pm: PowerManager, nowMs: Long) {
        if (nowMs - lastHeadroomPollMs < MIN_HEADROOM_INTERVAL_MS) return
        lastHeadroomPollMs = nowMs
        try {
            val hr = pm.getThermalHeadroom(HEADROOM_FORECAST_SEC)
            if (hr.isFinite() && hr > 0f) {
                hub.thermalHeadroom = hr
            }
        } catch (_: Throwable) {
        }
    }

    private fun pollThermalTelemetry() {
        try {
            val zoneFiles = cachedZoneTempFiles ?: discoverSysfsCpuThermalFiles().also {
                cachedZoneTempFiles = it
            }
            val socTemp = readHottestTempCelsius(zoneFiles)
            if (socTemp.isFinite()) {
                hub.cpuTempCelsius = if (hub.cpuTempCelsius.isFinite()) {
                    hub.cpuTempCelsius * 0.35f + socTemp * 0.65f
                } else {
                    socTemp
                }
            }

            val freqPairs = cachedFreqCapPairs ?: discoverCpuFreqCapPairs().also {
                cachedFreqCapPairs = it
            }
            hub.cpuFreqCapRatio = readMinFreqCapRatio(freqPairs)

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                val pm = context.getSystemService(Context.POWER_SERVICE) as? PowerManager
                if (pm != null) {
                    hub.osThermalStatus = pm.currentThermalStatus.coerceAtLeast(0)
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                        pollThermalHeadroomApi30(pm, System.currentTimeMillis())
                    }
                }
            }
        } catch (_: Throwable) {
        }
    }

    companion object {
        private const val POLL_INTERVAL_MS = 2_000L
        private const val MIN_HEADROOM_INTERVAL_MS = 2_500L
        private const val HEADROOM_FORECAST_SEC = 5

        private val CPU_SOC_KEYWORDS = arrayOf(
            "cpu",
            "soc",
            "tsens",
            "ap_ntc",
            "ap-therm",
            "cluster",
            "big",
            "little",
            "core",
            "pkg",
            "gpu",
            "rk",
            "sunxi",
            "ths",
            "mtktscpu",
            "acpitz"
        )

        private val EXCLUDED_ZONE_KEYWORDS = arrayOf(
            "battery",
            "bat",
            "bms",
            "chg",
            "charger",
            "usb",
            "pmic",
            "skin",
            "xo_therm",
            "pa_therm",
            "wifi",
            "modem",
            "cam",
            "flash"
        )

        internal fun isCpuOrSocZoneType(rawType: String): Boolean {
            val lower = rawType.trim().lowercase(Locale.US)
            if (lower.isEmpty()) return false
            for (ex in EXCLUDED_ZONE_KEYWORDS) {
                if (lower.contains(ex)) return false
            }
            for (kw in CPU_SOC_KEYWORDS) {
                if (lower.contains(kw)) return true
            }
            return false
        }

        internal fun normalizeRawThermalTempCelsius(rawText: String): Float {
            val trimmed = rawText.trim()
            if (trimmed.isEmpty()) return Float.NaN
            val token = trimmed.lineSequence().firstOrNull()?.trim().orEmpty()
            val rawVal = token.toFloatOrNull() ?: return Float.NaN
            if (!rawVal.isFinite()) return Float.NaN
            val celsius = when {
                rawVal >= 10_000f -> rawVal * 0.001f
                rawVal >= 200f -> rawVal * 0.1f
                else -> rawVal
            }
            return if (celsius in 12.0f..125.0f) celsius else Float.NaN
        }

        internal fun discoverSysfsCpuThermalFiles(
            thermalRoot: File = File("/sys/class/thermal")
        ): List<File> {
            if (!thermalRoot.isDirectory) return emptyList()
            val dirs = thermalRoot.listFiles { f ->
                f.isDirectory && f.name.startsWith("thermal_zone")
            } ?: return emptyList()

            val matched = ArrayList<File>(8)
            var untypedZone0Temp: File? = null

            for (dir in dirs.sortedBy { it.name }) {
                val tempFile = File(dir, "temp")
                if (!tempFile.isFile || !tempFile.canRead()) continue
                val typeFile = File(dir, "type")
                val zoneType = runCatching {
                    if (typeFile.isFile && typeFile.canRead()) typeFile.readText() else ""
                }.getOrDefault("")

                if (isCpuOrSocZoneType(zoneType)) {
                    val probe = runCatching { normalizeRawThermalTempCelsius(tempFile.readText()) }.getOrDefault(Float.NaN)
                    if (probe.isFinite()) {
                        matched.add(tempFile)
                    }
                } else if (dir.name == "thermal_zone0" && zoneType.isBlank()) {
                    val probe = runCatching { normalizeRawThermalTempCelsius(tempFile.readText()) }.getOrDefault(Float.NaN)
                    if (probe.isFinite()) {
                        untypedZone0Temp = tempFile
                    }
                }
            }

            if (matched.isEmpty() && untypedZone0Temp != null) {
                matched.add(untypedZone0Temp)
            }
            return matched
        }

        internal fun readHottestTempCelsius(tempFiles: List<File>): Float {
            var hottest = Float.NaN
            for (i in tempFiles.indices) {
                val f = tempFiles[i]
                val c = runCatching { normalizeRawThermalTempCelsius(f.readText()) }.getOrDefault(Float.NaN)
                if (c.isFinite() && (!hottest.isFinite() || c > hottest)) {
                    hottest = c
                }
            }
            return hottest
        }

        internal fun discoverCpuFreqCapPairs(
            cpuRoot: File = File("/sys/devices/system/cpu")
        ): List<Pair<File, File>> {
            if (!cpuRoot.isDirectory) return emptyList()
            val result = ArrayList<Pair<File, File>>(4)
            val cpufreqDir = File(cpuRoot, "cpufreq")
            val policyDirs = if (cpufreqDir.isDirectory) {
                cpufreqDir.listFiles { f -> f.isDirectory && f.name.startsWith("policy") }?.toList().orEmpty()
            } else {
                emptyList()
            }

            val candidates = if (policyDirs.isNotEmpty()) {
                policyDirs
            } else {
                cpuRoot.listFiles { f ->
                    f.isDirectory && f.name.length >= 4 && f.name.startsWith("cpu") && f.name[3].isDigit()
                }?.map { File(it, "cpufreq") }.orEmpty()
            }

            val seenPaths = HashSet<String>()
            for (dir in candidates) {
                if (!dir.isDirectory) continue
                val scalingMax = File(dir, "scaling_max_freq")
                val infoMax = File(dir, "cpuinfo_max_freq")
                if (scalingMax.isFile && scalingMax.canRead() && infoMax.isFile && infoMax.canRead()) {
                    val key = runCatching { scalingMax.canonicalPath }.getOrDefault(scalingMax.absolutePath)
                    if (seenPaths.add(key)) {
                        result.add(scalingMax to infoMax)
                    }
                }
            }
            return result
        }

        internal fun readMinFreqCapRatio(pairs: List<Pair<File, File>>): Float {
            if (pairs.isEmpty()) return 1.0f
            var minRatio = 1.0f
            for (i in pairs.indices) {
                val (scalingFile, infoFile) = pairs[i]
                val curMax = runCatching { scalingFile.readText().trim().toLongOrNull() }.getOrNull() ?: continue
                val hwMax = runCatching { infoFile.readText().trim().toLongOrNull() }.getOrNull() ?: continue
                if (hwMax > 100_000L && curMax in 50_000L..hwMax) {
                    val ratio = (curMax.toFloat() / hwMax.toFloat()).coerceIn(0.25f, 1.0f)
                    if (ratio < minRatio) {
                        minRatio = ratio
                    }
                }
            }
            return minRatio
        }
    }
}
