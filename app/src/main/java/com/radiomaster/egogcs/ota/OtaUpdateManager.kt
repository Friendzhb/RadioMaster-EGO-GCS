package com.radiomaster.egogcs.ota

import android.content.Context
import android.content.Intent
import android.content.pm.PackageInfo
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.provider.Settings
import androidx.core.content.FileProvider
import com.radiomaster.egogcs.R
import com.radiomaster.egogcs.model.GcsSharedHub
import java.io.File
import java.io.FileInputStream
import java.io.FileOutputStream
import java.io.IOException
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone
import java.util.concurrent.Callable
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import okhttp3.Call
import okhttp3.OkHttpClient
import okhttp3.Request
import org.json.JSONArray
import org.json.JSONObject

/**
 * 商业飞控级 GitHub Releases 实时加速源自动同步与直链 OTA 更新管理器 (`OtaUpdateManager`)：
 *
 * 1. 动态云端镜像注册表同步 (`syncCloudMirrorRegistries`)：
 *    实时从多路国内可达 CDN 与镜像聚合目录（MirrorHub 6小时自动巡检 JSON 多路 jsDelivr CDN、
 *    GHProxy 备用域名发布页 `ghproxy.link`、GitHub Proxy 聚合页 `github.akams.cn` 等）拉取最新活跃加速源前缀，
 *    自动过滤非代理噪声域名并与内置高可用种子池合并持久化；
 * 2. 并发真包魔数测速优选 (`raceMirrorCandidates`)：
 *    对待选加速源与 GitHub 官方直链并发发起轻量 4 字节 `Range: bytes=0-3` 探测，
 *    严格校验返回二进制头是否为真实 APK/ZIP 魔数 (`PK\x03\x04`, `0x50 0x4B 0x03 0x04`) 且体积正常，
 *    毫秒级剔除失效域名、Cloudflare 质询网页与校园网认证劫持页，按真实端到端 RTT 升序优选最快节点；
 * 3. 多源自动故障切换下载与飞行安全互锁：
 *    流式下载中途若首选镜像限速或断流，自动无缝切换至测速榜单第 2/3 备选节点及官方直链继续拉取；
 *    下载完成后二次校验 `PackageManager.getPackageArchiveInfo` 包名完整性，并在实机解锁飞行态锁定覆盖安装。
 */
class OtaUpdateManager(
    private val context: Context,
    private val hub: GcsSharedHub,
    private val onStateChanged: (Snapshot) -> Unit,
    private val onNotifyToast: (String) -> Unit
) {

    enum class Phase {
        IDLE,
        SYNCING_MIRRORS,
        CHECKING,
        UPDATE_AVAILABLE,
        UP_TO_DATE,
        DOWNLOADING,
        READY_TO_INSTALL,
        ERROR
    }

    data class VerifiedMirrorNode(
        val prefix: String,
        val hostLabel: String,
        val rttMs: Long,
        val sizeBytes: Long,
        val updatedEpochMs: Long
    )

    data class Snapshot(
        val phase: Phase,
        val progressPct: Int,
        val downloadedBytes: Long,
        val totalBytes: Long,
        val speedBytesPerSec: Long,
        val localVersionSummary: String,
        val remoteSummary: String,
        val mirrorPoolSummary: String,
        val statusMessage: String,
        val activeDirectUrl: String,
        val hasVerifiedApk: Boolean
    )

    companion object {
        const val OFFICIAL_GITHUB_APK_URL =
            "https://github.com/Friendzhb/RadioMaster-EGO-GCS/releases/download/latest-apk/EGO-Pilot-AX12.apk"
        const val GITHUB_RELEASE_API_URL =
            "https://api.github.com/repos/Friendzhb/RadioMaster-EGO-GCS/releases/tags/latest-apk"

        // 实时维护的云端 GitHub 加速源注册表与聚合目录 (多路国内直连 CDN 解决冷启动拉取问题)
        val CLOUD_MIRROR_REGISTRIES = listOf(
            "https://testingcf.jsdelivr.net/gh/mihsian77/MirrorHub@main/active-nodes.json",
            "https://gcore.jsdelivr.net/gh/mihsian77/MirrorHub@main/active-nodes.json",
            "https://fastly.jsdelivr.net/gh/mihsian77/MirrorHub@main/active-nodes.json",
            "https://cdn.jsdelivr.net/gh/mihsian77/MirrorHub@main/active-nodes.json",
            "https://ghproxy.link/",
            "https://github.akams.cn/"
        )

        // 经过验证的 URL 前缀代理种子池 (支持 Releases 大文件二进制流转发，与云端动态池去重合并参与实时测速)
        val VERIFIED_SEED_PREFIXES = listOf(
            "https://gh-proxy.com/",
            "https://hk.gh-proxy.com/",
            "https://ghfast.top/",
            "https://gh-proxy.org/",
            "https://edgeone.gh-proxy.org/",
            "https://cdn.gh-proxy.org/",
            "https://v4.gh-proxy.org/",
            "https://ghproxy.net/",
            "https://gh.monlor.com/",
            "https://ghp.ci/",
            "https://fastgit.cc/",
            "https://hub.gitmirror.com/"
        )

        // 排除非 Release URL 前缀代理的常见站点，防止聚合源噪声混入
        private val EXCLUDED_HOST_SUFFIXES = listOf(
            "github.com",
            "githubusercontent.com",
            "githubassets.com",
            "githubstatus.com",
            "jsdelivr.net",
            "gitclone.com",
            "kkgithub.com",
            "bgithub.xyz",
            "ghproxy.link",
            "akams.cn",
            "w3.org",
            "schema.org",
            "googleapis.com",
            "gstatic.com",
            "google.com",
            "microsoft.com",
            "linkedin.com",
            "twitter.com",
            "x.com",
            "chatgpt.com",
            "openai.com",
            "anthropic.com",
            "claude.com",
            "discord.gg",
            "telegram.org",
            "t.me",
            "youtu.be",
            "tiktok.com",
            "douyin.com",
            "baidu.com",
            "qq.com",
            "tencent.com",
            "tencentcloud.com",
            "tencentcloudapi.com",
            "aliyun.com",
            "jdcloud.com",
            "feishu.cn",
            "dingtalk.com",
            "zhihu.com",
            "b23.tv",
            "apple.com",
            "steampowered.com",
            "steamcommunity.com",
            "paypal.com",
            "wikimedia.org",
            "imgur.com",
            "shields.io",
            "mozilla.org",
            "apache.org",
            "eclipse.dev",
            "gradle.org",
            "ietf.org",
            "gnu.org",
            "freebsd.org",
            "snowflake.com",
            "codecov.io",
            "sonarcloud.io",
            "ngrok.com",
            "cursor.com",
            "opencode.ai"
        )

        private const val OTA_CACHE_PREFS = "ego_ota_mirror_cache"
        private const val KEY_BEST_PREFIX = "best_mirror_prefix"
        private const val KEY_BEST_RTT_MS = "best_mirror_rtt_ms"
        private const val KEY_RANKED_CSV = "ranked_prefixes_csv"
        private const val KEY_CLOUD_DYNAMIC_CSV = "cloud_dynamic_prefixes_csv"
        private const val KEY_LAST_RACE_MS = "last_race_epoch_ms"

        private const val APK_FILE_NAME = "EGO-Pilot-AX12.apk"
        private const val CI_PUBLISH_GRACE_MS = 240_000L
        private const val RACE_FRESH_TTL_MS = 180_000L
        private const val MAX_CONCURRENT_RACE_CANDIDATES = 14
        private val URL_EXTRACT_REGEX = Regex("""https://[a-zA-Z0-9.-]+(?:\.[a-zA-Z]{2,})(?:/gh-proxy)?/?""")
        private val HOST_EXTRACT_REGEX = Regex("""^https?://([^/\s:?#]+)""", RegexOption.IGNORE_CASE)
    }

    private val mainHandler = Handler(Looper.getMainLooper())
    private val ioExecutor = Executors.newSingleThreadExecutor { r ->
        Thread(r, "EgoOtaUpdater").apply { isDaemon = true }
    }
    private val probeExecutor = Executors.newFixedThreadPool(8) { r ->
        Thread(r, "EgoOtaMirrorProbe").apply { isDaemon = true }
    }

    private val httpClient: OkHttpClient = OkHttpClient.Builder()
        .connectTimeout(12, TimeUnit.SECONDS)
        .readTimeout(30, TimeUnit.SECONDS)
        .writeTimeout(20, TimeUnit.SECONDS)
        .followRedirects(true)
        .followSslRedirects(true)
        .retryOnConnectionFailure(true)
        .build()

    private val probeClient: OkHttpClient = httpClient.newBuilder()
        .connectTimeout(2600, TimeUnit.MILLISECONDS)
        .readTimeout(3000, TimeUnit.MILLISECONDS)
        .callTimeout(3800, TimeUnit.MILLISECONDS)
        .retryOnConnectionFailure(false)
        .build()

    private val stateLock = Any()

    @Volatile private var activeCall: Call? = null

    @Volatile private var userCancelled = false

    @Volatile private var pendingInstallAfterPermission = false

    private val localBuildEpochMs: Long = runCatching {
        context.getString(R.string.ego_build_epoch_ms).trim().toLong()
    }.getOrDefault(0L)

    private val localBuildStamp: String = runCatching {
        context.getString(R.string.ego_build_stamp)
    }.getOrDefault("本地构建")

    private var curPhase: Phase = Phase.IDLE
    private var curProgressPct: Int = 0
    private var curDownloaded: Long = 0L
    private var curTotal: Long = 0L
    private var curSpeedBps: Long = 0L
    private var curRemoteSummary: String = "云端分支 : latest-apk / EGO-Pilot-AX12.apk"
    private var curStatusMsg: String = "就绪 · 智能加速已启用，可刷新加速源测速或直接下载更新"

    // 实时加速源缓存状态
    private val dynamicCloudPrefixes = LinkedHashSet<String>()
    private var rankedVerifiedNodes: List<VerifiedMirrorNode> = emptyList()
    private var bestMirrorPrefix: String = VERIFIED_SEED_PREFIXES.first()
    private var bestMirrorRttMs: Long = 0L
    private var lastRaceWallMs: Long = 0L
    private var lastCloudSyncCount: Int = 0

    init {
        loadMirrorCacheFromPrefs()
    }

    val currentSnapshot: Snapshot
        get() = synchronized(stateLock) { buildSnapshotLocked() }

    /**
     * 解析当前选定模式的实际下载 URL：
     * - 模式 0 (智能实时加速)：优先选用最近一次真包魔数测速胜出的最低延迟节点 (含官方直链公平竞速结果)；
     * - 模式 1 (GitHub 官方直链)：固定使用 github.com 官方 Releases 直链；
     * - 模式 2 (自定义源)：既支持粘贴完整 `.apk` 下载直链，也支持粘贴任意私有镜像加速域名（自动补全目标直链）。
     */
    fun resolveDirectDownloadUrl(channelOverride: Int = hub.otaChannelMode): String {
        return when (channelOverride) {
            0 -> {
                val p = synchronized(stateLock) { bestMirrorPrefix }
                buildCandidateApkUrl(p)
            }
            1 -> OFFICIAL_GITHUB_APK_URL
            2 -> resolveCustomInputUrl(hub.otaCustomUrl)
            else -> OFFICIAL_GITHUB_APK_URL
        }
    }

    fun onChannelOrUrlChanged() {
        synchronized(stateLock) {
            if (curPhase != Phase.DOWNLOADING && curPhase != Phase.CHECKING && curPhase != Phase.SYNCING_MIRRORS) {
                curStatusMsg = when (hub.otaChannelMode) {
                    0 -> "已切至「智能实时加速」，下载前将自动优选最低延迟镜像源"
                    1 -> "已切至「GitHub 官方直链」，将直连 github.com 下载"
                    else -> "已切至「自定义源」，支持完整 APK 直链或自定义加速域名前缀"
                }
            }
        }
        dispatchSnapshot()
    }

    /**
     * 手动或后台触发：从云端实时拉取最新活跃 GitHub 加速源列表，并对真实 APK 魔数 (`PK\x03\x04`) 并发测速排序。
     */
    fun refreshLiveMirrors(manualTrigger: Boolean = true) {
        synchronized(stateLock) {
            if (curPhase == Phase.DOWNLOADING || curPhase == Phase.CHECKING || curPhase == Phase.SYNCING_MIRRORS) {
                return
            }
            curPhase = Phase.SYNCING_MIRRORS
            curProgressPct = 18
            curStatusMsg = "正在同步云端实时加速源注册表并并发校验真包延迟..."
        }
        dispatchSnapshot()

        ioExecutor.execute {
            try {
                val winners = syncAndBenchmarkMirrorsInternal(forceSync = true)
                synchronized(stateLock) {
                    if (winners.isNotEmpty()) {
                        val top = winners.first()
                        if (top.sizeBytes > 0L) {
                            val utcTxt = if (top.updatedEpochMs > 0L) formatUtcMs(top.updatedEpochMs) else "时间待查"
                            curRemoteSummary = "云端产物 : EGO-Pilot-AX12.apk (${formatBytes(top.sizeBytes)} · $utcTxt)"
                        }
                        val isNewer = top.updatedEpochMs > 0L &&
                            localBuildEpochMs > 0L &&
                            (top.updatedEpochMs > localBuildEpochMs + CI_PUBLISH_GRACE_MS)
                        curPhase = if (isNewer) Phase.UPDATE_AVAILABLE else Phase.IDLE
                        curStatusMsg = "实时测速完成：已锁定最优源 ${top.hostLabel} (${top.rttMs} ms)，共 ${winners.size} 个真包可用源"
                    } else {
                        curPhase = Phase.IDLE
                        curStatusMsg = "当前网络未测出可用镜像，请检查网络连接或切换热点后重试"
                    }
                }
                dispatchSnapshot()

                if (manualTrigger) {
                    if (winners.isNotEmpty()) {
                        val top = winners.first()
                        postToast("已更新实时加速源：最优 ${top.hostLabel} (${top.rttMs}ms)，共 ${winners.size} 源可用")
                    } else {
                        postToast("未探测到可达加速源，请检查网络连通性")
                    }
                }
            } catch (e: Throwable) {
                synchronized(stateLock) {
                    curPhase = Phase.IDLE
                    curStatusMsg = "加速源同步异常：${(e.message ?: "网络受限").take(48)}"
                }
                dispatchSnapshot()
                if (manualTrigger) {
                    postToast("刷新加速源失败，将使用内置优选备选池")
                }
            }
        }
    }

    /**
     * 当用户从系统「允许安装未知应用」授权页返回地面站时，自动续接弹出系统 APK 安装界面。
     */
    fun onHostResume() {
        if (!pendingInstallAfterPermission) return
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O && !context.packageManager.canRequestPackageInstalls()) {
            return
        }
        pendingInstallAfterPermission = false
        val apk = getVerifiedCachedApk()
        if (apk != null) {
            triggerPackageInstallerUi(apk)
        }
    }

    fun checkForUpdate(silentOnUpToDate: Boolean = false) {
        synchronized(stateLock) {
            if (curPhase == Phase.DOWNLOADING || curPhase == Phase.CHECKING || curPhase == Phase.SYNCING_MIRRORS) return
            curPhase = Phase.CHECKING
            curProgressPct = 20
            curStatusMsg = "正在同步加速通道并查询 GitHub latest-apk 构建状态..."
        }
        dispatchSnapshot()

        ioExecutor.execute {
            try {
                // 智能加速模式下顺带刷新最优实时镜像，使 API 查询与后续下载均走最快通道
                val needFreshRace = (System.currentTimeMillis() - lastRaceWallMs) > RACE_FRESH_TTL_MS || rankedVerifiedNodes.isEmpty()
                if (hub.otaChannelMode == 0 && needFreshRace) {
                    runCatching { syncAndBenchmarkMirrorsInternal(forceSync = true) }
                }

                val meta = fetchReleaseMetadata()
                val remoteUtcStr = if (meta.updatedEpochMs > 0L) formatUtcMs(meta.updatedEpochMs) else "时间未知"
                val sizeStr = if (meta.sizeBytes > 0L) formatBytes(meta.sizeBytes) else "大小未知"
                val isNewer = meta.updatedEpochMs > 0L &&
                    localBuildEpochMs > 0L &&
                    (meta.updatedEpochMs > localBuildEpochMs + CI_PUBLISH_GRACE_MS)

                synchronized(stateLock) {
                    curRemoteSummary = "云端产物 : EGO-Pilot-AX12.apk ($sizeStr · $remoteUtcStr)"
                    if (isNewer) {
                        curPhase = Phase.UPDATE_AVAILABLE
                        curStatusMsg = "检测到云端新构建版本，点击「直链下载新版」将自动走最优源升级"
                    } else {
                        curPhase = Phase.UP_TO_DATE
                        curStatusMsg = "当前已是最新构建（仍可随时点击「直链下载更新」覆盖重装）"
                    }
                }
                dispatchSnapshot()

                if (isNewer) {
                    postToast("发现 GitHub 新版地面站安装包 ($sizeStr)，可在「设置 -> 显示与通用」一键更新")
                } else if (!silentOnUpToDate) {
                    postToast("当前地面站已是最新构建版本 ($sizeStr)")
                }
            } catch (e: Throwable) {
                synchronized(stateLock) {
                    curPhase = Phase.IDLE
                    curStatusMsg = "云端元数据查询受限，可直接点击「直链下载更新」并发优选加速源拉取"
                }
                dispatchSnapshot()
                if (!silentOnUpToDate) {
                    postToast("元数据查询超时，可直接点击「直链下载更新」自动优选镜像下载")
                }
            }
        }
    }

    fun startDownloadOrCancelOrInstall() {
        val phaseSnapshot: Phase
        synchronized(stateLock) {
            phaseSnapshot = curPhase
        }
        if (phaseSnapshot == Phase.DOWNLOADING) {
            cancelActiveDownload()
            return
        }
        if (phaseSnapshot == Phase.SYNCING_MIRRORS) {
            return
        }
        val readyApk = if (phaseSnapshot == Phase.READY_TO_INSTALL) getVerifiedCachedApk() else null
        if (readyApk != null) {
            requestInstallVerifiedApk(readyApk)
            return
        }
        startDirectDownload()
    }

    fun startDirectDownload() {
        if (isRealDroneArmedInFlight()) {
            postToast("实机处于解锁飞行状态，为保障飞行安全请降落上锁后再更新软件")
            return
        }
        synchronized(stateLock) {
            if (curPhase == Phase.DOWNLOADING) return
            userCancelled = false
            curPhase = Phase.DOWNLOADING
            curProgressPct = 2
            curDownloaded = 0L
            curTotal = 0L
            curSpeedBps = 0L
            curStatusMsg = if (hub.otaChannelMode == 0) {
                "正在优选最低延迟实时加速源..."
            } else {
                "正在连接下载通道..."
            }
        }
        dispatchSnapshot()

        val modeSnapshot = hub.otaChannelMode
        ioExecutor.execute {
            try {
                val candidateUrls = buildOrderedDownloadChain(modeSnapshot)
                var downloadedFile: File? = null
                var lastErr: Throwable? = null

                for (idx in candidateUrls.indices) {
                    if (userCancelled) throw IOException("Cancelled")
                    val targetUrl = candidateUrls[idx]
                    val nodeName = extractDisplayHost(targetUrl)
                    if (idx > 0) {
                        updateProgressState(
                            2,
                            0L,
                            0L,
                            0L,
                            "前一节点响应异常，自动切换备选源 ($nodeName, ${idx + 1}/${candidateUrls.size})..."
                        )
                    } else {
                        updateProgressState(
                            3,
                            0L,
                            0L,
                            0L,
                            "已锁定节点 $nodeName，正在建立高速下载流..."
                        )
                    }
                    try {
                        downloadedFile = downloadAndVerifyApk(targetUrl, nodeName)
                        // 记录实际下载成功的优选镜像前缀
                        val matchedPrefix = extractMirrorPrefixFromDownloadUrl(targetUrl)
                        if (matchedPrefix != null && modeSnapshot == 0) {
                            synchronized(stateLock) {
                                bestMirrorPrefix = matchedPrefix
                            }
                            persistMirrorCacheToPrefs()
                        }
                        break
                    } catch (err: Throwable) {
                        lastErr = err
                        if (userCancelled) throw err
                    }
                }

                val apkFile = downloadedFile ?: throw (lastErr ?: IOException("所有候选下载节点均连接失败"))

                synchronized(stateLock) {
                    curPhase = Phase.READY_TO_INSTALL
                    curProgressPct = 100
                    curDownloaded = apkFile.length()
                    curTotal = apkFile.length()
                    curSpeedBps = 0L
                    curStatusMsg = "安装包校验通过 (${formatBytes(apkFile.length())})，正在调起系统覆盖安装..."
                }
                dispatchSnapshot()

                mainHandler.post {
                    requestInstallVerifiedApk(apkFile)
                }
            } catch (e: Throwable) {
                synchronized(stateLock) {
                    activeCall = null
                    if (userCancelled) {
                        curPhase = Phase.IDLE
                        curProgressPct = 0
                        curSpeedBps = 0L
                        curStatusMsg = "已取消下载"
                    } else {
                        curPhase = Phase.ERROR
                        curSpeedBps = 0L
                        val msg = (e.message ?: "网络连接中断").take(58)
                        curStatusMsg = "下载失败：$msg（可点击「刷新加速源」重测速后重试）"
                    }
                }
                dispatchSnapshot()
                if (!userCancelled) {
                    postToast("更新包下载失败，可点击「刷新加速源」重测速或检查网络")
                }
            }
        }
    }

    fun cancelActiveDownload() {
        userCancelled = true
        activeCall?.cancel()
        activeCall = null
        synchronized(stateLock) {
            curPhase = Phase.IDLE
            curProgressPct = 0
            curSpeedBps = 0L
            curStatusMsg = "已取消下载任务"
        }
        dispatchSnapshot()
    }

    fun shutdown() {
        userCancelled = true
        activeCall?.cancel()
        ioExecutor.shutdownNow()
        probeExecutor.shutdownNow()
    }

    // ==================== 实时云端镜像注册表同步 + 并发 APK 魔数测速引擎 ====================

    private fun syncAndBenchmarkMirrorsInternal(forceSync: Boolean): List<VerifiedMirrorNode> {
        if (forceSync || dynamicCloudPrefixes.isEmpty()) {
            val fetched = fetchLivePrefixesFromCloudRegistries()
            if (fetched.isNotEmpty()) {
                synchronized(stateLock) {
                    dynamicCloudPrefixes.clear()
                    dynamicCloudPrefixes.addAll(fetched)
                    lastCloudSyncCount = fetched.size
                }
            }
        }

        val candidates = LinkedHashSet<String>()
        synchronized(stateLock) {
            if (bestMirrorPrefix.isNotEmpty()) candidates.add(bestMirrorPrefix)
            for (n in rankedVerifiedNodes) {
                if (n.prefix.isNotEmpty()) candidates.add(n.prefix)
            }
        }
        // 先纳入经过高纯度筛选的云端动态节点与内置高可靠种子节点
        synchronized(stateLock) {
            for (p in dynamicCloudPrefixes) {
                if (candidates.size >= MAX_CONCURRENT_RACE_CANDIDATES - 1) break
                if (isHighSignalMirrorDomain(p)) {
                    candidates.add(p)
                }
            }
        }
        for (seed in VERIFIED_SEED_PREFIXES) {
            if (candidates.size >= MAX_CONCURRENT_RACE_CANDIDATES - 1) break
            candidates.add(seed)
        }
        synchronized(stateLock) {
            for (p in dynamicCloudPrefixes) {
                if (candidates.size >= MAX_CONCURRENT_RACE_CANDIDATES - 1) break
                candidates.add(p)
            }
        }
        // 始终将 GitHub 官方直链 ("") 纳入同场并发测速：海外/科研网直连快时自动选官方，国内直连受阻时自动选最快国内节点
        candidates.add("")

        val winners = raceMirrorCandidates(candidates.toList())
        if (winners.isNotEmpty()) {
            synchronized(stateLock) {
                rankedVerifiedNodes = winners
                bestMirrorPrefix = winners.first().prefix
                bestMirrorRttMs = winners.first().rttMs
                lastRaceWallMs = System.currentTimeMillis()
            }
            persistMirrorCacheToPrefs()
        }
        return winners
    }

    private fun fetchLivePrefixesFromCloudRegistries(): List<String> {
        val discovered = LinkedHashSet<String>()
        val tasks = CLOUD_MIRROR_REGISTRIES.map { regUrl ->
            Callable<List<String>> {
                runCatching {
                    val req = Request.Builder()
                        .url(regUrl)
                        .header("User-Agent", "RadioMaster-EGO-Pilot-MirrorSync/2.3")
                        .header("Accept", "application/json,text/html,*/*")
                        .get()
                        .build()
                    probeClient.newCall(req).execute().use { resp ->
                        if (!resp.isSuccessful) return@Callable emptyList()
                        val raw = resp.body?.string().orEmpty()
                        parseRegistryPayloadToPrefixes(raw)
                    }
                }.getOrDefault(emptyList())
            }
        }

        val futures = runCatching {
            probeExecutor.invokeAll(tasks, 3600L, TimeUnit.MILLISECONDS)
        }.getOrDefault(emptyList())

        val highPriority = LinkedHashSet<String>()
        val secondary = LinkedHashSet<String>()
        for (fut in futures) {
            if (fut.isDone && !fut.isCancelled) {
                val list = runCatching { fut.get() }.getOrDefault(emptyList())
                for (p in list) {
                    if (isHighSignalMirrorDomain(p)) highPriority.add(p) else secondary.add(p)
                }
            }
        }
        discovered.addAll(highPriority)
        for (p in secondary) {
            if (discovered.size >= 24) break
            discovered.add(p)
        }
        return discovered.take(24)
    }

    /**
     * 兼容多格式在线加速源列表解析（支持 MirrorHub `active-nodes.json` / `nodes.json` / `status.json`
     * 以及 `ghproxy.link` / `github.akams.cn` 网页或内嵌 JSON 节点数组）。
     */
    private fun parseRegistryPayloadToPrefixes(raw: String): List<String> {
        if (raw.isBlank()) return emptyList()
        val out = LinkedHashSet<String>()
        val trimmed = raw.trim()

        // 1. 尝试结构化 JSON 解析 (过滤掉仅支持 raw/clone/网页镜像的节点，仅保留通用 Release 代理)
        if (trimmed.startsWith("{") || trimmed.startsWith("[")) {
            runCatching {
                if (trimmed.startsWith("[")) {
                    extractPrefixesFromJsonArray(JSONArray(trimmed), out)
                } else {
                    val obj = JSONObject(trimmed)
                    val arrayKeys = listOf("nodes", "active", "active_nodes", "mirrors", "data", "list", "proxies")
                    var matchedArray = false
                    for (k in arrayKeys) {
                        val arr = obj.optJSONArray(k)
                        if (arr != null) {
                            extractPrefixesFromJsonArray(arr, out)
                            matchedArray = true
                        }
                    }
                    if (!matchedArray) {
                        val names = obj.names()
                        if (names != null) {
                            for (i in 0 until names.length()) {
                                val key = names.optString(i, "")
                                val child = obj.optJSONObject(key)
                                if (child != null) {
                                    extractPrefixFromNodeObject(child, out)
                                } else {
                                    val strVal = obj.optString(key, "")
                                    normalizeMirrorPrefix(strVal)?.let { out.add(it) }
                                }
                            }
                        }
                    }
                }
            }
        }

        // 2. 对网页聚合源 (如 ghproxy.link / github.akams.cn) 使用正则提取候选 HTTPS 加速前缀
        if (out.isEmpty()) {
            for (m in URL_EXTRACT_REGEX.findAll(raw)) {
                normalizeMirrorPrefix(m.value)?.let { out.add(it) }
                if (out.size >= 28) break
            }
        }
        return out.toList()
    }

    private fun extractPrefixesFromJsonArray(arr: JSONArray, out: MutableSet<String>) {
        val len = arr.length().coerceAtMost(180)
        for (i in 0 until len) {
            val itemObj = arr.optJSONObject(i)
            if (itemObj != null) {
                extractPrefixFromNodeObject(itemObj, out)
            } else {
                val itemStr = arr.optString(i, "")
                normalizeMirrorPrefix(itemStr)?.let { out.add(it) }
            }
            if (out.size >= 32) break
        }
    }

    private fun extractPrefixFromNodeObject(obj: JSONObject, out: MutableSet<String>) {
        val status = obj.optString("status", "online").lowercase(Locale.US)
        if (status.isNotEmpty() && status != "online" && status != "ok" && status != "up" && status != "healthy" && status != "true") {
            return
        }
        val category = obj.optString("category", obj.optString("type", "universal_proxy")).lowercase(Locale.US)
        if (category.contains("cdn_raw") || category.contains("clone") || category.contains("web_mirror")) {
            return
        }
        val rawUrl = obj.optString("url", "")
            .ifBlank { obj.optString("prefix", "") }
            .ifBlank { obj.optString("domain", "") }
            .ifBlank { obj.optString("host", "") }
        normalizeMirrorPrefix(rawUrl)?.let { out.add(it) }
    }

    private fun normalizeMirrorPrefix(rawCandidate: String): String? {
        val s = rawCandidate.trim().trim('"', '\'', ',', ';')
        if (!s.startsWith("https://", ignoreCase = true)) return null
        if (s.contains('?') || s.contains('#') || s.contains(' ')) return null
        val host = HOST_EXTRACT_REGEX.find(s)?.groupValues?.getOrNull(1)?.lowercase(Locale.US) ?: return null
        if (!host.contains('.') || host.length < 4) return null
        for (bad in EXCLUDED_HOST_SUFFIXES) {
            if (host == bad || host.endsWith(".$bad")) return null
        }
        // 仅接受根域名代理或 `/gh-proxy/` 标准子路径，排除普通文章链接或静态图片资源
        val pathPart = s.substringAfter(host, "").trimEnd('/')
        if (pathPart.isNotEmpty() && !pathPart.equals("/gh-proxy", ignoreCase = true)) {
            return null
        }
        return "https://$host$pathPart/"
    }

    private fun isHighSignalMirrorDomain(prefix: String): Boolean {
        val h = prefix.lowercase(Locale.US)
        return h.contains("ghproxy") ||
            h.contains("gh-proxy") ||
            h.contains("ghfast") ||
            h.contains("gitmirror") ||
            h.contains("fastgit") ||
            h.contains("ghp.") ||
            h.contains("//gh.") ||
            h.contains("//github.") ||
            h.contains("//git.") ||
            h.contains("mirror") ||
            h.contains("proxy")
    }

    /**
     * 并发向各候选节点请求真实 APK 的前 4 字节 (`Range: bytes=0-3`)，
     * 只有返回 `PK\x03\x04` 二进制文件头且总长正常的节点才算有效加速源，按实测 RTT 升序返回。
     */
    private fun raceMirrorCandidates(prefixes: List<String>): List<VerifiedMirrorNode> {
        if (prefixes.isEmpty()) return emptyList()
        val tasks = prefixes.map { prefix ->
            Callable<VerifiedMirrorNode?> {
                probeSingleCandidateApkHeader(prefix)
            }
        }
        val futures = runCatching {
            probeExecutor.invokeAll(tasks, 4000L, TimeUnit.MILLISECONDS)
        }.getOrDefault(emptyList())

        val valid = ArrayList<VerifiedMirrorNode>(prefixes.size)
        for (fut in futures) {
            if (fut.isDone && !fut.isCancelled) {
                val node = runCatching { fut.get() }.getOrNull()
                if (node != null) valid.add(node)
            }
        }
        valid.sortBy { it.rttMs }
        return valid
    }

    private fun probeSingleCandidateApkHeader(prefix: String): VerifiedMirrorNode? {
        val url = buildCandidateApkUrl(prefix)
        val label = if (prefix.isEmpty()) "github.com (官方)" else extractDisplayHost(prefix)
        val t0 = SystemClock.elapsedRealtime()
        val req = Request.Builder()
            .url(url)
            .header("User-Agent", "RadioMaster-EGO-Pilot-Android-OTA/2.3")
            .header("Range", "bytes=0-3")
            .header("Accept", "application/octet-stream,*/*")
            .get()
            .build()

        val call = probeClient.newCall(req)
        return try {
            call.execute().use { resp ->
                if (resp.code != 206 && resp.code != 200) return null
                val body = resp.body ?: return null

                val magic = ByteArray(4)
                var got = 0
                val stream = body.byteStream()
                while (got < 4) {
                    val n = stream.read(magic, got, 4 - got)
                    if (n <= 0) break
                    got += n
                }
                // 读取完前 4 字节立即终止底层流，防止忽略 Range 的代理继续下发整包耗费流量
                runCatching { call.cancel() }

                if (got < 4) return null
                val isZipApk = magic[0] == 0x50.toByte() &&
                    magic[1] == 0x4B.toByte() &&
                    magic[2] == 0x03.toByte() &&
                    magic[3] == 0x04.toByte()
                if (!isZipApk) return null

                val contentRange = resp.header("Content-Range").orEmpty()
                val totalFromRange = contentRange.substringAfter('/', "").trim().toLongOrNull()
                val totalLen = totalFromRange ?: resp.header("Content-Length")?.toLongOrNull() ?: 0L
                if (totalLen in 1L..249_999L) return null

                val lastMod = parseHttpDateMs(resp.header("Last-Modified").orEmpty())
                val rtt = (SystemClock.elapsedRealtime() - t0).coerceAtLeast(1L)
                VerifiedMirrorNode(
                    prefix = prefix,
                    hostLabel = label,
                    rttMs = rtt,
                    sizeBytes = totalLen,
                    updatedEpochMs = lastMod
                )
            }
        } catch (_: Throwable) {
            runCatching { call.cancel() }
            null
        }
    }

    private fun buildOrderedDownloadChain(mode: Int): List<String> {
        val urls = LinkedHashSet<String>()
        when (mode) {
            0 -> {
                val stale = (System.currentTimeMillis() - lastRaceWallMs) > RACE_FRESH_TTL_MS || rankedVerifiedNodes.isEmpty()
                if (stale) {
                    updateProgressState(2, 0L, 0L, 0L, "正在实时同步云端加速源并并发优选最快节点...")
                    runCatching { syncAndBenchmarkMirrorsInternal(forceSync = true) }
                }
                val rankedSnap = synchronized(stateLock) { rankedVerifiedNodes.toList() }
                for (node in rankedSnap.take(5)) {
                    urls.add(buildCandidateApkUrl(node.prefix))
                }
                if (urls.isEmpty()) {
                    val cachedBest = synchronized(stateLock) { bestMirrorPrefix }
                    urls.add(buildCandidateApkUrl(cachedBest))
                    for (seed in VERIFIED_SEED_PREFIXES.take(3)) {
                        urls.add(buildCandidateApkUrl(seed))
                    }
                }
                urls.add(OFFICIAL_GITHUB_APK_URL)
            }
            1 -> {
                urls.add(OFFICIAL_GITHUB_APK_URL)
            }
            else -> {
                urls.add(resolveCustomInputUrl(hub.otaCustomUrl))
            }
        }
        return urls.toList()
    }

    private fun buildCandidateApkUrl(prefix: String): String {
        if (prefix.isBlank()) return OFFICIAL_GITHUB_APK_URL
        val clean = if (prefix.endsWith("/")) prefix else "$prefix/"
        return clean + OFFICIAL_GITHUB_APK_URL
    }

    /**
     * 智能解析自定义输入：既支持完整 `.apk` 直链，也支持直接填入任意自建镜像加速域名（如 `https://my-gh.workers.dev`）。
     */
    private fun resolveCustomInputUrl(rawCustom: String): String {
        val c = rawCustom.trim()
        if (!c.startsWith("http://", ignoreCase = true) && !c.startsWith("https://", ignoreCase = true)) {
            return OFFICIAL_GITHUB_APK_URL
        }
        val lower = c.lowercase(Locale.US)
        if (lower.endsWith(".apk") || lower.contains("/releases/download/") || lower.contains("github.com/")) {
            return c
        }
        val prefix = if (c.endsWith("/")) c else "$c/"
        return prefix + OFFICIAL_GITHUB_APK_URL
    }

    private fun extractDisplayHost(urlOrPrefix: String): String {
        val cleaned = urlOrPrefix.trim()
        if (cleaned.startsWith(OFFICIAL_GITHUB_APK_URL, ignoreCase = true) || cleaned.isEmpty()) {
            return "github.com (官方)"
        }
        return HOST_EXTRACT_REGEX.find(cleaned)?.groupValues?.getOrNull(1) ?: "直链节点"
    }

    private fun extractMirrorPrefixFromDownloadUrl(downloadUrl: String): String? {
        if (downloadUrl == OFFICIAL_GITHUB_APK_URL) return ""
        val idx = downloadUrl.indexOf(OFFICIAL_GITHUB_APK_URL)
        if (idx > 0) {
            return downloadUrl.substring(0, idx)
        }
        return null
    }

    private fun loadMirrorCacheFromPrefs() {
        runCatching {
            val sp = context.getSharedPreferences(OTA_CACHE_PREFS, Context.MODE_PRIVATE)
            val savedBest = sp.getString(KEY_BEST_PREFIX, null)
            if (savedBest != null && (savedBest.isEmpty() || savedBest.startsWith("https://"))) {
                bestMirrorPrefix = savedBest
            }
            bestMirrorRttMs = sp.getLong(KEY_BEST_RTT_MS, 0L)
            val cloudCsv = sp.getString(KEY_CLOUD_DYNAMIC_CSV, "").orEmpty()
            if (cloudCsv.isNotBlank()) {
                for (item in cloudCsv.split(',')) {
                    normalizeMirrorPrefix(item)?.let { dynamicCloudPrefixes.add(it) }
                }
                lastCloudSyncCount = dynamicCloudPrefixes.size
            }
            val rankedCsv = sp.getString(KEY_RANKED_CSV, "").orEmpty()
            if (rankedCsv.isNotBlank()) {
                val restored = ArrayList<VerifiedMirrorNode>()
                for (entry in rankedCsv.split(',')) {
                    val parts = entry.split('|')
                    val pref = parts.getOrNull(0)?.trim() ?: continue
                    val rtt = parts.getOrNull(1)?.toLongOrNull() ?: 0L
                    if (pref.isEmpty() || pref.startsWith("https://")) {
                        val lbl = if (pref.isEmpty()) "github.com (官方)" else extractDisplayHost(pref)
                        restored.add(VerifiedMirrorNode(pref, lbl, rtt, 0L, 0L))
                    }
                }
                if (restored.isNotEmpty()) {
                    rankedVerifiedNodes = restored
                }
            }
            lastRaceWallMs = sp.getLong(KEY_LAST_RACE_MS, 0L)
        }
    }

    private fun persistMirrorCacheToPrefs() {
        runCatching {
            val cloudCsv: String
            val rankedCsv: String
            val bestP: String
            val bestRtt: Long
            val raceMs: Long
            synchronized(stateLock) {
                cloudCsv = dynamicCloudPrefixes.take(20).joinToString(",")
                rankedCsv = rankedVerifiedNodes.take(6).joinToString(",") { "${it.prefix}|${it.rttMs}" }
                bestP = bestMirrorPrefix
                bestRtt = bestMirrorRttMs
                raceMs = lastRaceWallMs
            }
            context.getSharedPreferences(OTA_CACHE_PREFS, Context.MODE_PRIVATE).edit()
                .putString(KEY_BEST_PREFIX, bestP)
                .putLong(KEY_BEST_RTT_MS, bestRtt)
                .putString(KEY_CLOUD_DYNAMIC_CSV, cloudCsv)
                .putString(KEY_RANKED_CSV, rankedCsv)
                .putLong(KEY_LAST_RACE_MS, raceMs)
                .apply()
        }
    }

    // ==================== 系统安装器拉起、流式下载与完整性校验 ====================

    private fun requestInstallVerifiedApk(apkFile: File) {
        if (!apkFile.exists() || apkFile.length() < 200_000L) {
            startDirectDownload()
            return
        }
        if (isRealDroneArmedInFlight()) {
            synchronized(stateLock) {
                curPhase = Phase.READY_TO_INSTALL
                curStatusMsg = "安装包已就绪，请先将实机降落上锁后再点击「立即覆盖安装」"
            }
            dispatchSnapshot()
            postToast("安装包已下载就绪，请降落上锁后再执行覆盖安装")
            return
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O && !context.packageManager.canRequestPackageInstalls()) {
            pendingInstallAfterPermission = true
            synchronized(stateLock) {
                curPhase = Phase.READY_TO_INSTALL
                curStatusMsg = "请开启「允许安装未知应用」权限，返回后将自动弹出安装界面"
            }
            dispatchSnapshot()
            postToast("请开启「允许安装未知应用」权限后返回自动安装")
            runCatching {
                val permIntent = Intent(
                    Settings.ACTION_MANAGE_UNKNOWN_APP_SOURCES,
                    Uri.parse("package:${context.packageName}")
                ).apply {
                    addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
                }
                context.startActivity(permIntent)
            }
            return
        }

        triggerPackageInstallerUi(apkFile)
    }

    private fun triggerPackageInstallerUi(apkFile: File) {
        try {
            val contentUri = FileProvider.getUriForFile(
                context,
                "${context.packageName}.fileprovider",
                apkFile
            )
            val installIntent = Intent(Intent.ACTION_VIEW).apply {
                setDataAndType(contentUri, "application/vnd.android.package-archive")
                addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
                addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
            }
            context.startActivity(installIntent)
            synchronized(stateLock) {
                curPhase = Phase.READY_TO_INSTALL
                curStatusMsg = "已调起系统安装器（若首次切换固定签名提示冲突，卸载旧版重装一次后即可永久直连覆盖）"
            }
            dispatchSnapshot()
        } catch (e: Throwable) {
            synchronized(stateLock) {
                curPhase = Phase.ERROR
                curStatusMsg = "调起系统安装器失败：${e.message ?: "未知异常"}"
            }
            dispatchSnapshot()
            postToast("无法调起系统安装器：${e.message ?: "请检查系统安装权限"}")
        }
    }

    private fun downloadAndVerifyApk(url: String, nodeLabel: String): File {
        val otaDir = File(context.cacheDir, "ota")
        if (!otaDir.exists()) otaDir.mkdirs()
        val partFile = File(otaDir, "$APK_FILE_NAME.part")
        val finalFile = File(otaDir, APK_FILE_NAME)
        // v3.10: 支持跨加速源 HTTP 206 Range 断点续传；若已有合法 PK 头分片则直接从断点续接
        var existingBytes = if (partFile.exists() && partFile.length() > 4096L) partFile.length() else 0L
        if (existingBytes > 0L && !hasValidZipHeaderQuick(partFile)) {
            partFile.delete()
            existingBytes = 0L
        }

        val reqBuilder = Request.Builder()
            .url(url)
            .header("User-Agent", "RadioMaster-EGO-Pilot-Android-OTA/3.10")
            .header("Accept", "application/octet-stream,*/*")
        if (existingBytes > 0L) {
            reqBuilder.header("Range", "bytes=$existingBytes-")
        }
        val req = reqBuilder.get().build()

        val call = httpClient.newCall(req)
        activeCall = call

        call.execute().use { resp ->
            if (!resp.isSuccessful) {
                throw IOException("HTTP ${resp.code}")
            }
            val body = resp.body ?: throw IOException("空响应数据")
            val isPartialResume = (resp.code == 206 && existingBytes > 0L)
            val bodyLen = body.contentLength()
            val totalLen = if (isPartialResume && bodyLen > 0L) (existingBytes + bodyLen) else bodyLen
            if (!isPartialResume && existingBytes > 0L) {
                existingBytes = 0L
            }

            body.byteStream().use { input ->
                FileOutputStream(partFile, isPartialResume).use { output ->
                    val buf = ByteArray(32 * 1024)
                    var downloaded = existingBytes
                    var windowBytes = 0L
                    var lastUiTickMs = SystemClock.elapsedRealtime()
                    var currentSpeed = 0L

                    while (true) {
                        if (userCancelled) throw IOException("Cancelled")
                        val n = input.read(buf)
                        if (n <= 0) break
                        // 首包即刻校验 ZIP 魔数，若加速节点返回 HTML 错误页立即快速失败并触发切源
                        if (downloaded == 0L && n >= 4) {
                            val okMagic = buf[0] == 0x50.toByte() &&
                                buf[1] == 0x4B.toByte() &&
                                buf[2] == 0x03.toByte() &&
                                buf[3] == 0x04.toByte()
                            if (!okMagic) {
                                throw IOException("节点返回非 APK 内容 ($nodeLabel)")
                            }
                        }
                        output.write(buf, 0, n)
                        downloaded += n
                        windowBytes += n

                        val nowMs = SystemClock.elapsedRealtime()
                        val elapsedMs = nowMs - lastUiTickMs
                        if (elapsedMs >= 90L) {
                            val instantBps = (windowBytes * 1000L) / elapsedMs.coerceAtLeast(1L)
                            currentSpeed = if (currentSpeed == 0L) instantBps else ((currentSpeed * 3L + instantBps) / 4L)
                            windowBytes = 0L
                            lastUiTickMs = nowMs

                            val pct = if (totalLen > 0L) {
                                ((downloaded * 100L) / totalLen).toInt().coerceIn(0, 99)
                            } else {
                                ((downloaded / (32 * 1024L)).toInt()).coerceIn(1, 95)
                            }
                            val progText = if (totalLen > 0L) {
                                "经 $nodeLabel 下载 $pct% (${formatBytes(downloaded)} / ${formatBytes(totalLen)} · ${formatBytes(currentSpeed)}/s)"
                            } else {
                                "经 $nodeLabel 下载 (${formatBytes(downloaded)} · ${formatBytes(currentSpeed)}/s)"
                            }
                            updateProgressState(pct, downloaded, totalLen, currentSpeed, progText)
                        }
                    }
                    output.flush()
                }
            }
        }
        activeCall = null

        verifyDownloadedApkOrThrow(partFile)
        if (finalFile.exists()) finalFile.delete()
        if (!partFile.renameTo(finalFile)) {
            partFile.copyTo(finalFile, overwrite = true)
            partFile.delete()
        }
        return finalFile
    }

    private fun hasValidZipHeaderQuick(file: File): Boolean = runCatching {
        val m = ByteArray(4)
        FileInputStream(file).use { fis ->
            fis.read(m) == 4 &&
                m[0] == 0x50.toByte() && m[1] == 0x4B.toByte() &&
                m[2] == 0x03.toByte() && m[3] == 0x04.toByte()
        }
    }.getOrDefault(false)

    /**
     * v3.10 扫描文件末尾 66KB 校验标准 ZIP End-of-Central-Directory 签名 (`0x50 0x4B 0x05 0x06`)，
     * 100% 拦截镜像代理中途断流造成的半截残损 APK。
     */
    private fun verifyZipEocdTail(file: File): Boolean {
        return try {
            val len = file.length()
            if (len < 22L) return false
            val inspectLen = minOf(len, 65_557L).toInt()
            val tail = ByteArray(inspectLen)
            java.io.RandomAccessFile(file, "r").use { raf ->
                raf.seek(len - inspectLen)
                raf.readFully(tail)
            }
            var found = false
            for (i in inspectLen - 22 downTo 0) {
                if (tail[i] == 0x50.toByte() &&
                    tail[i + 1] == 0x4B.toByte() &&
                    tail[i + 2] == 0x05.toByte() &&
                    tail[i + 3] == 0x06.toByte()
                ) {
                    found = true
                    break
                }
            }
            found
        } catch (_: Throwable) {
            false
        }
    }

    private fun verifyDownloadedApkOrThrow(file: File) {
        if (!file.exists() || file.length() < 250_000L) {
            file.delete()
            throw IOException("安装包体积异常 (${file.length()} B)")
        }
        val magic = ByteArray(4)
        FileInputStream(file).use { fis ->
            if (fis.read(magic) != 4) {
                throw IOException("无法读取文件头")
            }
        }
        val isZipHeader = magic[0] == 0x50.toByte() &&
            magic[1] == 0x4B.toByte() &&
            magic[2] == 0x03.toByte() &&
            magic[3] == 0x04.toByte()
        if (!isZipHeader) {
            file.delete()
            throw IOException("非有效 APK 归档（请检查热点是否需要网页登录）")
        }
        if (!verifyZipEocdTail(file)) {
            file.delete()
            throw IOException("APK 归档尾部 EOCD 校验未通过（代理流可能被截断）")
        }

        val archiveInfo = getArchivePackageInfoCompat(file.absolutePath)
        if (archiveInfo == null) {
            file.delete()
            throw IOException("APK 包解析校验未通过")
        }
        if (archiveInfo.packageName != context.packageName) {
            file.delete()
            throw IOException("包名不匹配 (${archiveInfo.packageName})")
        }
    }

    private fun getVerifiedCachedApk(): File? {
        val f = File(File(context.cacheDir, "ota"), APK_FILE_NAME)
        if (!f.exists() || f.length() < 250_000L) return null
        val info = getArchivePackageInfoCompat(f.absolutePath) ?: return null
        return if (info.packageName == context.packageName) f else null
    }

    private data class ReleaseMeta(val sizeBytes: Long, val updatedEpochMs: Long)

    private fun fetchReleaseMetadata(): ReleaseMeta {
        // 1. 优先通过已测速最优加速源及官方 GitHub REST API 获取精确发布时间与字节数
        val apiEndpoints = LinkedHashSet<String>()
        val bestPref = synchronized(stateLock) { bestMirrorPrefix }
        if (hub.otaChannelMode == 0 && bestPref.isNotEmpty()) {
            apiEndpoints.add(bestPref + GITHUB_RELEASE_API_URL)
        }
        apiEndpoints.add(GITHUB_RELEASE_API_URL)
        for (seed in VERIFIED_SEED_PREFIXES.take(2)) {
            apiEndpoints.add(seed + GITHUB_RELEASE_API_URL)
        }

        for (apiUrl in apiEndpoints) {
            val parsed = runCatching {
                val apiReq = Request.Builder()
                    .url(apiUrl)
                    .header("Accept", "application/vnd.github+json")
                    .header("User-Agent", "RadioMaster-EGO-Pilot-Android-OTA/2.3")
                    .get()
                    .build()
                probeClient.newCall(apiReq).execute().use { resp ->
                    if (!resp.isSuccessful) return@use null
                    val raw = resp.body?.string().orEmpty()
                    val root = JSONObject(raw)
                    val assets = root.optJSONArray("assets") ?: return@use null
                    for (i in 0 until assets.length()) {
                        val item = assets.optJSONObject(i) ?: continue
                        val name = item.optString("name", "")
                        if (name.equals(APK_FILE_NAME, ignoreCase = true) || name.endsWith(".apk", ignoreCase = true)) {
                            val sz = item.optLong("size", 0L)
                            val updatedIso = item.optString("updated_at", item.optString("created_at", ""))
                            val epoch = parseIso8601UtcMs(updatedIso)
                            if (sz > 0L || epoch > 0L) {
                                return@use ReleaseMeta(sz, epoch)
                            }
                        }
                    }
                    null
                }
            }.getOrNull()
            if (parsed != null) return parsed
        }

        // 2. 回退方案：复用测速缓存头信息或直接发起 4 字节真包 Range GET 探测
        val rankedTop = synchronized(stateLock) { rankedVerifiedNodes.firstOrNull { it.sizeBytes > 0L } }
        if (rankedTop != null && rankedTop.sizeBytes > 0L) {
            return ReleaseMeta(rankedTop.sizeBytes, rankedTop.updatedEpochMs)
        }

        val probed = probeSingleCandidateApkHeader(if (hub.otaChannelMode == 1) "" else bestPref)
            ?: throw IOException("无法获取云端构建元数据")
        return ReleaseMeta(probed.sizeBytes, probed.updatedEpochMs)
    }

    private fun isRealDroneArmedInFlight(): Boolean {
        return hub.isRosMode && hub.rosConnected && hub.snapshots.armed
    }

    private fun updateProgressState(pct: Int, down: Long, total: Long, speed: Long, msg: String) {
        synchronized(stateLock) {
            curPhase = Phase.DOWNLOADING
            curProgressPct = pct
            curDownloaded = down
            curTotal = total
            curSpeedBps = speed
            curStatusMsg = msg
        }
        dispatchSnapshot()
    }

    private fun buildMirrorPoolSummaryLocked(): String {
        return when (hub.otaChannelMode) {
            1 -> "加速源池 : 当前固定使用 GitHub 官方直链 (可切至「智能实时加速」自动优选镜像)"
            2 -> "加速源池 : 当前使用自定义下载源 (${extractDisplayHost(resolveCustomInputUrl(hub.otaCustomUrl))})"
            else -> {
                if (rankedVerifiedNodes.isNotEmpty()) {
                    val first = rankedVerifiedNodes[0]
                    val secondTxt = if (rankedVerifiedNodes.size >= 2) {
                        val sec = rankedVerifiedNodes[1]
                        " | 备选 ${sec.hostLabel} (${sec.rttMs}ms)"
                    } else {
                        ""
                    }
                    val cloudNote = if (lastCloudSyncCount > 0) "云端同步 $lastCloudSyncCount 源 · " else ""
                    "加速源池 : 最优 ${first.hostLabel} (${first.rttMs}ms)$secondTxt · ${cloudNote}${rankedVerifiedNodes.size} 源就绪"
                } else {
                    val host = if (bestMirrorPrefix.isEmpty()) "github.com (官方)" else extractDisplayHost(bestMirrorPrefix)
                    val rttPart = if (bestMirrorRttMs > 0L) " (${bestMirrorRttMs}ms)" else ""
                    "加速源池 : 预设优选 $host$rttPart · 点「刷新加速源」实时拉取云端源测速"
                }
            }
        }
    }

    private fun buildSnapshotLocked(): Snapshot {
        val localInfo = getSelfPackageInfoCompat()
        val vName = localInfo?.versionName ?: "2.2.0-ota"
        val vCode = getVersionCodeCompat(localInfo)
        val localSummary = "当前版本 : v$vName (#$vCode · 编译于 $localBuildStamp)"
        return Snapshot(
            phase = curPhase,
            progressPct = curProgressPct,
            downloadedBytes = curDownloaded,
            totalBytes = curTotal,
            speedBytesPerSec = curSpeedBps,
            localVersionSummary = localSummary,
            remoteSummary = curRemoteSummary,
            mirrorPoolSummary = buildMirrorPoolSummaryLocked(),
            statusMessage = curStatusMsg,
            activeDirectUrl = resolveDirectDownloadUrl(),
            hasVerifiedApk = (curPhase == Phase.READY_TO_INSTALL)
        )
    }

    private fun dispatchSnapshot() {
        val snap = currentSnapshot
        mainHandler.post { onStateChanged(snap) }
    }

    private fun postToast(msg: String) {
        mainHandler.post { onNotifyToast(msg) }
    }

    @Suppress("DEPRECATION")
    private fun getArchivePackageInfoCompat(archivePath: String): PackageInfo? {
        return runCatching {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                context.packageManager.getPackageArchiveInfo(
                    archivePath,
                    PackageManager.PackageInfoFlags.of(0L)
                )
            } else {
                context.packageManager.getPackageArchiveInfo(archivePath, 0)
            }
        }.getOrNull()
    }

    @Suppress("DEPRECATION")
    private fun getSelfPackageInfoCompat(): PackageInfo? {
        return runCatching {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                context.packageManager.getPackageInfo(
                    context.packageName,
                    PackageManager.PackageInfoFlags.of(0L)
                )
            } else {
                context.packageManager.getPackageInfo(context.packageName, 0)
            }
        }.getOrNull()
    }

    @Suppress("DEPRECATION")
    private fun getVersionCodeCompat(info: PackageInfo?): Long {
        if (info == null) return 0L
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) {
            info.longVersionCode
        } else {
            info.versionCode.toLong()
        }
    }

    private fun parseIso8601UtcMs(iso: String): Long {
        if (iso.isBlank()) return 0L
        return runCatching {
            val sdf = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'", Locale.US).apply {
                timeZone = TimeZone.getTimeZone("UTC")
            }
            sdf.parse(iso.trim())?.time ?: 0L
        }.getOrDefault(0L)
    }

    private fun parseHttpDateMs(httpDate: String): Long {
        if (httpDate.isBlank()) return 0L
        return runCatching {
            val sdf = SimpleDateFormat("EEE, dd MMM yyyy HH:mm:ss zzz", Locale.US).apply {
                timeZone = TimeZone.getTimeZone("GMT")
            }
            sdf.parse(httpDate.trim())?.time ?: 0L
        }.getOrDefault(0L)
    }

    private fun formatUtcMs(epochMs: Long): String {
        val sdf = SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.US).apply {
            timeZone = TimeZone.getTimeZone("UTC")
        }
        return sdf.format(Date(epochMs)) + " UTC"
    }

    private fun formatBytes(bytes: Long): String {
        if (bytes < 1024L) return "$bytes B"
        val kb = bytes / 1024.0
        if (kb < 1024.0) return String.format(Locale.US, "%.0f KB", kb)
        val mb = kb / 1024.0
        return String.format(Locale.US, "%.2f MB", mb)
    }
}
