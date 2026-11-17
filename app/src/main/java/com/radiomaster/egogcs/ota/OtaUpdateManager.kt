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
import java.security.MessageDigest
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
 *    实时从多路国内可达 CDN 与镜像聚合目录拉取最新活跃加速源前缀，自动过滤非代理噪声域名并与内置高可用种子池合并持久化；
 * 2. 新鲜度门控 + 并发真包魔数测速优选 (`raceMirrorCandidates` + `filterAndRankFreshMirrors`)：
 *    对待选加速源与 GitHub 官方直链并发发起 `no-cache` 轻量 4 字节 `Range: bytes=0-3` 探测，
 *    同时校验 `PK\x03\x04` 魔数、`Content-Range` 总长与 `Last-Modified` 新鲜度，剔除仍缓存旧版 APK 的过期 CDN 边缘节点；
 * 3. 多维版本精确判定与 SHA-256 强校验 (`isRemoteReleaseNewer` + `verifyDownloadedApkOrThrow`)：
 *    并发拉取 `build-info.json` 与 GitHub Releases API 元数据，按「本机已装 APK SHA-256 -> Android versionCode ->
 *    Git Commit SHA -> CI 全流程宽限时间戳」四级精确判定是否存在新版，下载后校验包体积、SHA-256 与归档版本号，
 *    若命中过期镜像缓存自动无缝切换下一节点直至官方源，彻底杜绝重复提示与反复安装旧版本问题。
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

    internal data class ReleaseMeta(
        val sizeBytes: Long,
        val updatedEpochMs: Long,
        val versionCode: Long = 0L,
        val versionName: String = "",
        val commitSha: String = "",
        val sha256: String = ""
    )

    private data class VerifiedApkSummary(
        val file: File,
        val versionCode: Long,
        val versionName: String
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
        const val OFFICIAL_BUILD_INFO_URL =
            "https://github.com/Friendzhb/RadioMaster-EGO-GCS/releases/download/latest-apk/build-info.json"
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
        private const val KEY_SELF_SHA_VER_CODE = "self_apk_sha_ver_code"
        private const val KEY_SELF_SHA_UPDATE_TIME = "self_apk_sha_update_ms"
        private const val KEY_SELF_SHA_HEX = "self_apk_sha256_hex"

        private const val APK_FILE_NAME = "EGO-Pilot-AX12.apk"

        // 覆盖 GitHub Actions 从编译完成到模拟器测试通过再执行 publish-apk 的完整流水线窗口 (25 分钟)
        internal const val CI_PUBLISH_GRACE_MS = 25 * 60_000L
        internal const val STALE_MIRROR_TOLERANCE_MS = 120_000L
        private const val RACE_FRESH_TTL_MS = 180_000L
        private const val MAX_CONCURRENT_RACE_CANDIDATES = 14
        private val URL_EXTRACT_REGEX = Regex("""https://[a-zA-Z0-9.-]+(?:\.[a-zA-Z]{2,})(?:/gh-proxy)?/?""")
        private val HOST_EXTRACT_REGEX = Regex("""^https?://([^/\s:?#]+)""", RegexOption.IGNORE_CASE)
        private val HEX_SHA_REGEX = Regex("""^[0-9a-f]{7,64}$""")

        internal fun parseIso8601UtcMsStatic(iso: String): Long {
            if (iso.isBlank()) return 0L
            return runCatching {
                val sdf = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'", Locale.US).apply {
                    timeZone = TimeZone.getTimeZone("UTC")
                }
                sdf.parse(iso.trim())?.time ?: 0L
            }.getOrDefault(0L)
        }

        private fun optCleanString(obj: JSONObject, key: String): String {
            if (obj.isNull(key)) return ""
            return obj.optString(key, "").trim()
        }

        internal fun parseBuildInfoJson(raw: String): ReleaseMeta? {
            if (raw.isBlank()) return null
            return runCatching {
                val obj = JSONObject(raw)
                val bytes = if (obj.isNull("bytes")) 0L else obj.optLong("bytes", 0L)
                val verCode = if (obj.isNull("version_code")) 0L else obj.optLong("version_code", 0L)
                val verName = optCleanString(obj, "version_name")
                val commitRaw = optCleanString(obj, "source_commit").lowercase(Locale.US)
                val commitSha = if (HEX_SHA_REGEX.matches(commitRaw)) commitRaw else ""
                val shaRaw = optCleanString(obj, "sha256").lowercase(Locale.US)
                val sha256 = if (shaRaw.length == 64 && HEX_SHA_REGEX.matches(shaRaw)) shaRaw else ""
                val genEpochMs = if (obj.isNull("metadata_generated_epoch_ms")) {
                    0L
                } else {
                    obj.optLong("metadata_generated_epoch_ms", 0L)
                }
                if (bytes <= 0L && verCode <= 0L && sha256.isEmpty() && commitSha.isEmpty()) {
                    null
                } else {
                    ReleaseMeta(
                        sizeBytes = bytes,
                        updatedEpochMs = genEpochMs,
                        versionCode = verCode,
                        versionName = verName,
                        commitSha = commitSha,
                        sha256 = sha256
                    )
                }
            }.getOrNull()
        }

        internal fun parseGithubReleaseApiJson(raw: String): ReleaseMeta? {
            if (raw.isBlank()) return null
            return runCatching {
                val root = JSONObject(raw)
                val commitRaw = optCleanString(root, "target_commitish").lowercase(Locale.US)
                val commitSha = if (HEX_SHA_REGEX.matches(commitRaw)) commitRaw else ""
                val assets = root.optJSONArray("assets") ?: return@runCatching null
                for (i in 0 until assets.length()) {
                    val item = assets.optJSONObject(i) ?: continue
                    val name = optCleanString(item, "name")
                    if (name.equals(APK_FILE_NAME, ignoreCase = true) || name.endsWith(".apk", ignoreCase = true)) {
                        val sz = item.optLong("size", 0L)
                        val updatedIso = optCleanString(item, "updated_at")
                            .ifEmpty { optCleanString(item, "created_at") }
                        val epoch = parseIso8601UtcMsStatic(updatedIso)
                        val digestRaw = optCleanString(item, "digest")
                            .lowercase(Locale.US)
                            .removePrefix("sha256:")
                        val sha256 = if (digestRaw.length == 64 && HEX_SHA_REGEX.matches(digestRaw)) digestRaw else ""
                        if (sz > 0L || epoch > 0L || sha256.isNotEmpty()) {
                            return@runCatching ReleaseMeta(
                                sizeBytes = sz,
                                updatedEpochMs = epoch,
                                commitSha = commitSha,
                                sha256 = sha256
                            )
                        }
                    }
                }
                null
            }.getOrNull()
        }

        internal fun mergeReleaseMetadata(apiMeta: ReleaseMeta?, buildInfoMeta: ReleaseMeta?): ReleaseMeta? {
            if (apiMeta == null) return buildInfoMeta
            if (buildInfoMeta == null) return apiMeta

            val sizeConflict = apiMeta.sizeBytes > 0L &&
                buildInfoMeta.sizeBytes > 0L &&
                apiMeta.sizeBytes != buildInfoMeta.sizeBytes
            val shaConflict = apiMeta.sha256.isNotEmpty() &&
                buildInfoMeta.sha256.isNotEmpty() &&
                !apiMeta.sha256.equals(buildInfoMeta.sha256, ignoreCase = true)
            val commitConflict = apiMeta.commitSha.length >= 7 &&
                buildInfoMeta.commitSha.length >= 7 &&
                !apiMeta.commitSha.startsWith(buildInfoMeta.commitSha) &&
                !buildInfoMeta.commitSha.startsWith(apiMeta.commitSha)

            if (sizeConflict || shaConflict || commitConflict) {
                // 当镜像缓存的 build-info.json 与实时 GitHub Release API 不一致时，以更新时间更新的一方为准，绝不混用过期字段
                return if (buildInfoMeta.updatedEpochMs > apiMeta.updatedEpochMs + 60_000L) {
                    buildInfoMeta
                } else {
                    apiMeta
                }
            }

            return ReleaseMeta(
                sizeBytes = if (apiMeta.sizeBytes > 0L) apiMeta.sizeBytes else buildInfoMeta.sizeBytes,
                updatedEpochMs = if (apiMeta.updatedEpochMs > 0L) apiMeta.updatedEpochMs else buildInfoMeta.updatedEpochMs,
                versionCode = if (buildInfoMeta.versionCode > 0L) buildInfoMeta.versionCode else apiMeta.versionCode,
                versionName = buildInfoMeta.versionName.ifEmpty { apiMeta.versionName },
                commitSha = buildInfoMeta.commitSha.ifEmpty { apiMeta.commitSha },
                sha256 = buildInfoMeta.sha256.ifEmpty { apiMeta.sha256 }
            )
        }

        /**
         * 四级确定性云端新版本判定：
         * 1. 若云端 APK SHA-256 与本机当前运行 APK 完全一致，直接判定已是最新；
         * 2. 若云端元数据含明确 `versionCode`，严格按 `remoteVersionCode > localVersionCode` 判定；
         * 3. 若本机与云端记录了相同 Git Commit SHA，即使 CI 经历模拟器测试延迟发布也判定为同一构建；
         * 4. 兜底才使用覆盖完整 CI 周期 (25 min) 的发布时间戳宽限窗口。
         */
        internal fun isRemoteReleaseNewer(
            localVersionCode: Long,
            localBuildEpochMs: Long,
            localGitSha: String,
            installedApkSha256: String,
            meta: ReleaseMeta
        ): Boolean {
            if (installedApkSha256.length == 64 &&
                meta.sha256.length == 64 &&
                installedApkSha256.equals(meta.sha256, ignoreCase = true)
            ) {
                return false
            }

            if (meta.versionCode > 0L && localVersionCode > 0L) {
                return meta.versionCode > localVersionCode
            }

            val localSha = localGitSha.trim().lowercase(Locale.US)
            val remoteSha = meta.commitSha.trim().lowercase(Locale.US)
            if (localSha.length >= 7 &&
                remoteSha.length >= 7 &&
                HEX_SHA_REGEX.matches(localSha) &&
                HEX_SHA_REGEX.matches(remoteSha)
            ) {
                if (remoteSha.startsWith(localSha) || localSha.startsWith(remoteSha)) {
                    return false
                }
            }

            return meta.updatedEpochMs > 0L &&
                localBuildEpochMs > 0L &&
                (meta.updatedEpochMs > localBuildEpochMs + CI_PUBLISH_GRACE_MS)
        }

        /**
         * 剔除仍缓存旧版 APK 的过期 CDN 镜像节点：
         * - 若已知权威 `expectedMeta`，严格过滤 `sizeBytes` 不匹配或 `Last-Modified` 早于权威时间 >2 分钟的节点；
         * - 若尚未拉取 `expectedMeta`，以同场测速最新 `Last-Modified` 节点为基准剔除落后 >2 分钟或体积不一致的旧缓存节点。
         */
        internal fun filterAndRankFreshMirrors(
            probed: List<VerifiedMirrorNode>,
            expectedMeta: ReleaseMeta? = null
        ): List<VerifiedMirrorNode> {
            if (probed.isEmpty()) return emptyList()
            val sortedByRtt = probed.sortedBy { it.rttMs }

            val targetSize = expectedMeta?.sizeBytes?.takeIf { it > 0L } ?: 0L
            val maxProbedUpdated = sortedByRtt.maxOf { it.updatedEpochMs }
            val referenceUpdatedMs = maxOf(expectedMeta?.updatedEpochMs ?: 0L, maxProbedUpdated)

            val consensusSize = if (targetSize > 0L) {
                targetSize
            } else if (referenceUpdatedMs > 0L) {
                sortedByRtt.firstOrNull {
                    it.updatedEpochMs >= referenceUpdatedMs - STALE_MIRROR_TOLERANCE_MS && it.sizeBytes > 0L
                }?.sizeBytes ?: 0L
            } else {
                0L
            }

            val fresh = sortedByRtt.filter { node ->
                val sizeMatches = consensusSize <= 0L || node.sizeBytes <= 0L || node.sizeBytes == consensusSize
                val timeFresh = referenceUpdatedMs <= 0L ||
                    node.updatedEpochMs <= 0L ||
                    (node.updatedEpochMs + STALE_MIRROR_TOLERANCE_MS >= referenceUpdatedMs)
                sizeMatches && timeFresh
            }

            return if (fresh.isNotEmpty()) fresh else sortedByRtt
        }

        internal fun computeFileSha256Hex(file: File): String {
            val digest = MessageDigest.getInstance("SHA-256")
            FileInputStream(file).use { fis ->
                val buf = ByteArray(64 * 1024)
                while (true) {
                    val n = fis.read(buf)
                    if (n <= 0) break
                    digest.update(buf, 0, n)
                }
            }
            val bytes = digest.digest()
            val sb = StringBuilder(bytes.size * 2)
            for (b in bytes) {
                sb.append(String.format(Locale.US, "%02x", b.toInt() and 0xFF))
            }
            return sb.toString()
        }
    }

    private val mainHandler = Handler(Looper.getMainLooper())
    private val ioExecutor = Executors.newSingleThreadExecutor { r ->
        Thread(r, "EgoOtaUpdater").apply { isDaemon = true }
    }
    private val probeExecutor = Executors.newFixedThreadPool(8) { r ->
        Thread(r, "EgoOtaMirrorProbe").apply { isDaemon = true }
    }

    private val httpClient: OkHttpClient by lazy {
        OkHttpClient.Builder()
            .connectTimeout(12, TimeUnit.SECONDS)
            .readTimeout(30, TimeUnit.SECONDS)
            .writeTimeout(20, TimeUnit.SECONDS)
            .followRedirects(true)
            .followSslRedirects(true)
            .retryOnConnectionFailure(true)
            .build()
    }

    private val probeClient: OkHttpClient by lazy {
        httpClient.newBuilder()
            .connectTimeout(2600, TimeUnit.MILLISECONDS)
            .readTimeout(3000, TimeUnit.MILLISECONDS)
            .callTimeout(3800, TimeUnit.MILLISECONDS)
            .retryOnConnectionFailure(false)
            .build()
    }

    private val stateLock = Any()

    @Volatile private var activeCall: Call? = null

    @Volatile private var userCancelled = false

    @Volatile private var pendingInstallAfterPermission = false

    @Volatile private var latestKnownMeta: ReleaseMeta? = null

    @Volatile private var cachedSelfApkSha256: String = ""

    private val localBuildEpochMs: Long = runCatching {
        context.getString(R.string.ego_build_epoch_ms).trim().toLong()
    }.getOrDefault(0L)

    private val localBuildStamp: String = runCatching {
        context.getString(R.string.ego_build_stamp)
    }.getOrDefault("本地构建")

    private val localGitSha: String = runCatching {
        context.getString(R.string.ego_git_sha).trim().lowercase(Locale.US)
    }.getOrDefault("")

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

    private val deferredCleanupRunnable = Runnable {
        if (!ioExecutor.isShutdown) {
            ioExecutor.execute {
                cleanupInstalledCachedApkIfNeeded()
            }
        }
    }

    init {
        loadMirrorCacheFromPrefs()
        scheduleDeferredCachedApkCleanup(2500L)
    }

    private fun scheduleDeferredCachedApkCleanup(delayMs: Long) {
        mainHandler.removeCallbacks(deferredCleanupRunnable)
        mainHandler.postDelayed(deferredCleanupRunnable, delayMs)
    }

    val currentSnapshot: Snapshot
        get() = synchronized(stateLock) { buildSnapshotLocked() }

    /**
     * 解析当前选定模式的实际下载 URL：
     * - 模式 0 (智能实时加速)：优先选用最近一次新鲜度与真包魔数测速胜出的最低延迟节点；
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
                    0 -> "已切至「智能实时加速」，下载前将自动过滤旧缓存并优选最低延迟镜像源"
                    1 -> "已切至「GitHub 官方直链」，将直连 github.com 下载"
                    else -> "已切至「自定义源」，支持完整 APK 直链或自定义加速域名前缀"
                }
            }
        }
        dispatchSnapshot()
    }

    /**
     * 手动或后台触发：从云端实时拉取最新活跃 GitHub 加速源列表，结合权威元数据过滤旧缓存节点并并发测速排序。
     */
    fun refreshLiveMirrors(manualTrigger: Boolean = true) {
        synchronized(stateLock) {
            if (curPhase == Phase.DOWNLOADING || curPhase == Phase.CHECKING || curPhase == Phase.SYNCING_MIRRORS) {
                return
            }
            curPhase = Phase.SYNCING_MIRRORS
            curProgressPct = 18
            curStatusMsg = "正在同步云端实时加速源注册表并过滤过期缓存节点..."
        }
        dispatchSnapshot()

        ioExecutor.execute {
            try {
                val meta = runCatching { fetchReleaseMetadata() }.getOrNull()
                if (meta != null) {
                    latestKnownMeta = meta
                }
                val winners = syncAndBenchmarkMirrorsInternal(forceSync = true, expectedMeta = meta ?: latestKnownMeta)
                val localInfo = getSelfPackageInfoCompat()
                val localVerCode = getVersionCodeCompat(localInfo)
                val selfSha = resolveInstalledApkSha256(localInfo)

                synchronized(stateLock) {
                    if (winners.isNotEmpty()) {
                        val top = winners.first()
                        val effectiveMeta = meta ?: ReleaseMeta(
                            sizeBytes = top.sizeBytes,
                            updatedEpochMs = top.updatedEpochMs
                        )
                        curRemoteSummary = formatRemoteMetaSummary(effectiveMeta)
                        val isNewer = isRemoteReleaseNewer(
                            localVersionCode = localVerCode,
                            localBuildEpochMs = localBuildEpochMs,
                            localGitSha = localGitSha,
                            installedApkSha256 = selfSha,
                            meta = effectiveMeta
                        )
                        curPhase = if (isNewer) Phase.UPDATE_AVAILABLE else Phase.IDLE
                        curStatusMsg = "实时测速完成：已锁定新鲜最优源 ${top.hostLabel} (${top.rttMs} ms)，共 ${winners.size} 个可用源"
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
     * 当用户从系统「允许安装未知应用」授权页或安装器界面返回地面站时，自动续接安装或清理已安装的旧缓存包。
     */
    fun onHostResume() {
        if (pendingInstallAfterPermission) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O &&
                !context.packageManager.canRequestPackageInstalls()
            ) {
                return
            }
            pendingInstallAfterPermission = false
            val apk = getVerifiedCachedApk(requireNewerThanInstalled = false)
            if (apk != null) {
                triggerPackageInstallerUi(apk)
                return
            }
        }
        scheduleDeferredCachedApkCleanup(1800L)
    }

    fun checkForUpdate(silentOnUpToDate: Boolean = false) {
        synchronized(stateLock) {
            if (curPhase == Phase.DOWNLOADING || curPhase == Phase.CHECKING || curPhase == Phase.SYNCING_MIRRORS) return
            curPhase = Phase.CHECKING
            curProgressPct = 20
            curStatusMsg = "正在校验云端 build-info 元数据与加速通道状态..."
        }
        dispatchSnapshot()

        ioExecutor.execute {
            try {
                cleanupInstalledCachedApkIfNeeded()

                val meta = fetchReleaseMetadata()
                latestKnownMeta = meta

                // 智能加速模式下结合最新权威体积与时间戳刷新镜像池，提前剔除缓存旧版的 CDN 节点
                val needFreshRace = (System.currentTimeMillis() - lastRaceWallMs) > RACE_FRESH_TTL_MS ||
                    rankedVerifiedNodes.isEmpty() ||
                    (meta.sizeBytes > 0L && rankedVerifiedNodes.firstOrNull()?.let { it.sizeBytes > 0L && it.sizeBytes != meta.sizeBytes } == true)
                if (hub.otaChannelMode == 0 && needFreshRace) {
                    runCatching { syncAndBenchmarkMirrorsInternal(forceSync = true, expectedMeta = meta) }
                }

                val localInfo = getSelfPackageInfoCompat()
                val localVerCode = getVersionCodeCompat(localInfo)
                val localVerName = localInfo?.versionName ?: "3.13.0"
                val selfSha = resolveInstalledApkSha256(localInfo)
                val isNewer = isRemoteReleaseNewer(
                    localVersionCode = localVerCode,
                    localBuildEpochMs = localBuildEpochMs,
                    localGitSha = localGitSha,
                    installedApkSha256 = selfSha,
                    meta = meta
                )

                val sizeStr = if (meta.sizeBytes > 0L) formatBytes(meta.sizeBytes) else "大小未知"
                val remoteVerBadge = if (meta.versionName.isNotEmpty()) "v${meta.versionName} · $sizeStr" else sizeStr

                synchronized(stateLock) {
                    curRemoteSummary = formatRemoteMetaSummary(meta)
                    if (isNewer) {
                        curPhase = Phase.UPDATE_AVAILABLE
                        curStatusMsg = if (meta.versionName.isNotEmpty()) {
                            "检测到云端新版 v${meta.versionName}，点击「直链下载新版」自动校验升级"
                        } else {
                            "检测到云端新构建版本，点击「直链下载新版」将自动走最优源升级"
                        }
                    } else {
                        curPhase = Phase.UP_TO_DATE
                        curStatusMsg = "当前 v$localVerName 已是最新构建（仍可随时点击「直链下载更新」覆盖重装）"
                    }
                }
                dispatchSnapshot()

                if (isNewer) {
                    postToast("发现新版地面站 ($remoteVerBadge)，可在「设置 -> 显示与通用」一键更新")
                } else if (!silentOnUpToDate) {
                    postToast("当前地面站 v$localVerName 已是最新版本")
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
        val readyApk = if (phaseSnapshot == Phase.READY_TO_INSTALL) {
            getVerifiedCachedApk(requireNewerThanInstalled = true)
        } else {
            null
        }
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
                "正在同步云端权威校验指纹并优选最新鲜加速源..."
            } else {
                "正在获取云端校验信息并连接下载通道..."
            }
        }
        dispatchSnapshot()

        val modeSnapshot = hub.otaChannelMode
        ioExecutor.execute {
            try {
                // 自定义私有链接不强制绑定官方仓库的 SHA-256/体积预期，仅保留防降级与包名校验
                val expectedMeta = if (modeSnapshot == 2) {
                    null
                } else {
                    runCatching { fetchReleaseMetadata() }.getOrNull()?.also {
                        latestKnownMeta = it
                        synchronized(stateLock) {
                            curRemoteSummary = formatRemoteMetaSummary(it)
                        }
                    } ?: latestKnownMeta
                }

                val candidateUrls = buildOrderedDownloadChain(modeSnapshot, expectedMeta)
                var verifiedApk: VerifiedApkSummary? = null
                var lastErr: Throwable? = null

                for (idx in candidateUrls.indices) {
                    if (userCancelled) throw IOException("Cancelled")
                    val targetUrl = candidateUrls[idx]
                    val nodeName = extractDisplayHost(targetUrl)
                    if (idx > 0) {
                        val reasonHint = lastErr?.message?.take(28)?.let { " ($it)" }.orEmpty()
                        updateProgressState(
                            2,
                            0L,
                            0L,
                            0L,
                            "前一节点不可用或缓存旧包$reasonHint，切换备选源 ($nodeName, ${idx + 1}/${candidateUrls.size})..."
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
                        verifiedApk = downloadAndVerifyApk(targetUrl, nodeName, expectedMeta)
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

                val summary = verifiedApk ?: throw (lastErr ?: IOException("所有候选下载节点均连接或校验失败"))
                val apkFile = summary.file
                val verDesc = if (summary.versionName.isNotEmpty()) {
                    "v${summary.versionName} (#${summary.versionCode} · ${formatBytes(apkFile.length())})"
                } else {
                    "#${summary.versionCode} · ${formatBytes(apkFile.length())}"
                }

                synchronized(stateLock) {
                    curPhase = Phase.READY_TO_INSTALL
                    curProgressPct = 100
                    curDownloaded = apkFile.length()
                    curTotal = apkFile.length()
                    curSpeedBps = 0L
                    curStatusMsg = "安装包校验通过 ($verDesc)，正在调起系统覆盖安装..."
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
                        val msg = (e.message ?: "网络连接中断").take(64)
                        curStatusMsg = "下载失败：$msg（可切至「GitHub 官方直链」或点「刷新加速源」重试）"
                    }
                }
                dispatchSnapshot()
                if (!userCancelled) {
                    postToast("更新包下载或校验失败，可点「刷新加速源」重测速后重试")
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
        mainHandler.removeCallbacks(deferredCleanupRunnable)
        activeCall?.cancel()
        ioExecutor.shutdownNow()
        probeExecutor.shutdownNow()
    }

    // ==================== 实时云端镜像注册表同步 + 并发 APK 魔数与新鲜度测速引擎 ====================

    private fun syncAndBenchmarkMirrorsInternal(
        forceSync: Boolean,
        expectedMeta: ReleaseMeta? = null
    ): List<VerifiedMirrorNode> {
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
        // 始终将 GitHub 官方直链 ("") 纳入同场并发测速与新鲜度基准比对
        candidates.add("")

        val winners = raceMirrorCandidates(candidates.toList(), expectedMeta)
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
                        .header("User-Agent", "RadioMaster-EGO-Pilot-MirrorSync/3.13")
                        .header("Accept", "application/json,text/html,*/*")
                        .header("Cache-Control", "no-cache, max-age=0")
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

    private fun parseRegistryPayloadToPrefixes(raw: String): List<String> {
        if (raw.isBlank()) return emptyList()
        val out = LinkedHashSet<String>()
        val trimmed = raw.trim()

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
        if (status.isNotEmpty() &&
            status != "online" &&
            status != "ok" &&
            status != "up" &&
            status != "healthy" &&
            status != "true"
        ) {
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

    private fun raceMirrorCandidates(
        prefixes: List<String>,
        expectedMeta: ReleaseMeta? = null
    ): List<VerifiedMirrorNode> {
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
        return filterAndRankFreshMirrors(valid, expectedMeta)
    }

    private fun probeSingleCandidateApkHeader(prefix: String): VerifiedMirrorNode? {
        val url = buildCandidateApkUrl(prefix)
        val label = if (prefix.isEmpty()) "github.com (官方)" else extractDisplayHost(prefix)
        val t0 = SystemClock.elapsedRealtime()
        val req = Request.Builder()
            .url(url)
            .header("User-Agent", "RadioMaster-EGO-Pilot-Android-OTA/3.13")
            .header("Range", "bytes=0-3")
            .header("Accept", "application/octet-stream,*/*")
            .header("Cache-Control", "no-cache, no-store, max-age=0")
            .header("Pragma", "no-cache")
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

    private fun buildOrderedDownloadChain(mode: Int, expectedMeta: ReleaseMeta?): List<String> {
        val urls = LinkedHashSet<String>()
        when (mode) {
            0 -> {
                val currentRanked = synchronized(stateLock) { rankedVerifiedNodes.toList() }
                val hasSizeMismatch = expectedMeta != null &&
                    expectedMeta.sizeBytes > 0L &&
                    currentRanked.firstOrNull()?.let { it.sizeBytes > 0L && it.sizeBytes != expectedMeta.sizeBytes } == true
                val stale = (System.currentTimeMillis() - lastRaceWallMs) > RACE_FRESH_TTL_MS ||
                    currentRanked.isEmpty() ||
                    hasSizeMismatch
                if (stale) {
                    updateProgressState(2, 0L, 0L, 0L, "正在实时同步云端加速源并剔除旧版缓存节点...")
                    runCatching { syncAndBenchmarkMirrorsInternal(forceSync = true, expectedMeta = expectedMeta) }
                }
                val rankedSnap = filterAndRankFreshMirrors(
                    synchronized(stateLock) { rankedVerifiedNodes.toList() },
                    expectedMeta
                )
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

    private fun buildCandidateAssetUrl(prefix: String, officialAssetUrl: String): String {
        if (prefix.isBlank()) return officialAssetUrl
        val clean = if (prefix.endsWith("/")) prefix else "$prefix/"
        return clean + officialAssetUrl
    }

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
                    val sz = parts.getOrNull(2)?.toLongOrNull() ?: 0L
                    val upd = parts.getOrNull(3)?.toLongOrNull() ?: 0L
                    if (pref.isEmpty() || pref.startsWith("https://")) {
                        val lbl = if (pref.isEmpty()) "github.com (官方)" else extractDisplayHost(pref)
                        restored.add(VerifiedMirrorNode(pref, lbl, rtt, sz, upd))
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
                rankedCsv = rankedVerifiedNodes.take(6).joinToString(",") {
                    "${it.prefix}|${it.rttMs}|${it.sizeBytes}|${it.updatedEpochMs}"
                }
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

    private fun resolveInstalledApkSha256(selfInfo: PackageInfo?): String {
        val currentCached = cachedSelfApkSha256
        if (currentCached.length == 64) return currentCached

        val verCode = getVersionCodeCompat(selfInfo)
        val lastUpdateMs = selfInfo?.lastUpdateTime ?: 0L
        val sp = runCatching {
            context.getSharedPreferences(OTA_CACHE_PREFS, Context.MODE_PRIVATE)
        }.getOrNull()

        if (sp != null && verCode > 0L) {
            val savedVer = sp.getLong(KEY_SELF_SHA_VER_CODE, -1L)
            val savedUpd = sp.getLong(KEY_SELF_SHA_UPDATE_TIME, -1L)
            val savedHex = sp.getString(KEY_SELF_SHA_HEX, "").orEmpty()
            if (savedVer == verCode && savedUpd == lastUpdateMs && savedHex.length == 64) {
                cachedSelfApkSha256 = savedHex
                return savedHex
            }
        }

        val computed = runCatching {
            val srcPath = context.applicationInfo?.sourceDir.orEmpty()
            if (srcPath.isBlank()) return@runCatching ""
            val apkFile = File(srcPath)
            if (!apkFile.isFile || apkFile.length() <= 0L) return@runCatching ""
            computeFileSha256Hex(apkFile)
        }.getOrDefault("")

        if (computed.length == 64) {
            cachedSelfApkSha256 = computed
            if (sp != null && verCode > 0L) {
                runCatching {
                    sp.edit()
                        .putLong(KEY_SELF_SHA_VER_CODE, verCode)
                        .putLong(KEY_SELF_SHA_UPDATE_TIME, lastUpdateMs)
                        .putString(KEY_SELF_SHA_HEX, computed)
                        .apply()
                }
            }
        }
        return computed
    }

    // ==================== 系统安装器拉起、流式下载与完整性/新鲜度校验 ====================

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
                curStatusMsg = "已调起系统安装器（安装完成后应用将自动重启并切换至新版本）"
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

    private fun downloadAndVerifyApk(
        url: String,
        nodeLabel: String,
        expectedMeta: ReleaseMeta?
    ): VerifiedApkSummary {
        val otaDir = File(context.cacheDir, "ota")
        if (!otaDir.exists()) otaDir.mkdirs()
        val partFile = File(otaDir, "$APK_FILE_NAME.part")
        val partMetaFile = File(otaDir, "$APK_FILE_NAME.part.meta")
        val finalFile = File(otaDir, APK_FILE_NAME)

        val expectedIdentity = if (expectedMeta != null &&
            (expectedMeta.sha256.isNotEmpty() || expectedMeta.sizeBytes > 0L)
        ) {
            "${expectedMeta.sizeBytes}|${expectedMeta.sha256}"
        } else {
            "url|$url"
        }
        val previousIdentity = runCatching {
            if (partMetaFile.isFile) partMetaFile.readText().trim() else ""
        }.getOrDefault("")

        if (partFile.exists() && previousIdentity != expectedIdentity) {
            partFile.delete()
            partMetaFile.delete()
        }

        var existingBytes = if (partFile.exists() && partFile.length() > 4096L) partFile.length() else 0L
        if (existingBytes > 0L &&
            expectedMeta != null &&
            expectedMeta.sizeBytes > 0L &&
            existingBytes >= expectedMeta.sizeBytes
        ) {
            partFile.delete()
            existingBytes = 0L
        }
        if (existingBytes > 0L && !hasValidZipHeaderQuick(partFile)) {
            partFile.delete()
            existingBytes = 0L
        }
        runCatching { partMetaFile.writeText(expectedIdentity) }

        val reqBuilder = Request.Builder()
            .url(url)
            .header("User-Agent", "RadioMaster-EGO-Pilot-Android-OTA/3.13")
            .header("Accept", "application/octet-stream,*/*")
            .header("Cache-Control", "no-cache, no-store, max-age=0")
            .header("Pragma", "no-cache")
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

            // 下载刚开始即可根据响应 Content-Length 拦截仍缓存旧版 APK 的过期 CDN 边缘节点，无需白跑整包流量
            if (expectedMeta != null &&
                expectedMeta.sizeBytes > 250_000L &&
                totalLen > 0L &&
                totalLen != expectedMeta.sizeBytes
            ) {
                partFile.delete()
                partMetaFile.delete()
                throw IOException("节点缓存旧包体积不符 ($nodeLabel)")
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
                            currentSpeed = if (currentSpeed == 0L) {
                                instantBps
                            } else {
                                (currentSpeed * 3L + instantBps) / 4L
                            }
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

        val verified = verifyDownloadedApkOrThrow(partFile, expectedMeta)
        partMetaFile.delete()
        if (finalFile.exists()) finalFile.delete()
        if (!partFile.renameTo(finalFile)) {
            partFile.copyTo(finalFile, overwrite = true)
            partFile.delete()
        }
        return verified.copy(file = finalFile)
    }

    private fun hasValidZipHeaderQuick(file: File): Boolean = runCatching {
        val m = ByteArray(4)
        FileInputStream(file).use { fis ->
            fis.read(m) == 4 &&
                m[0] == 0x50.toByte() && m[1] == 0x4B.toByte() &&
                m[2] == 0x03.toByte() && m[3] == 0x04.toByte()
        }
    }.getOrDefault(false)

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

    private fun verifyDownloadedApkOrThrow(file: File, expectedMeta: ReleaseMeta?): VerifiedApkSummary {
        if (!file.exists() || file.length() < 250_000L) {
            file.delete()
            throw IOException("安装包体积异常 (${file.length()} B)")
        }
        if (expectedMeta != null && expectedMeta.sizeBytes > 250_000L && file.length() != expectedMeta.sizeBytes) {
            val actualLen = file.length()
            file.delete()
            throw IOException("镜像节点返回旧缓存包 (${formatBytes(actualLen)} != ${formatBytes(expectedMeta.sizeBytes)})")
        }

        val magic = ByteArray(4)
        FileInputStream(file).use { fis ->
            if (fis.read(magic) != 4) {
                file.delete()
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
            throw IOException("APK 归档尾部 EOCD 校验未通过（代理流被截断）")
        }

        if (expectedMeta != null && expectedMeta.sha256.length == 64) {
            val actualSha = computeFileSha256Hex(file)
            if (!actualSha.equals(expectedMeta.sha256, ignoreCase = true)) {
                file.delete()
                throw IOException("SHA-256 指纹不匹配，镜像节点尚未同步最新包")
            }
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

        val archiveVerCode = getVersionCodeCompat(archiveInfo)
        val archiveVerName = archiveInfo.versionName.orEmpty()
        val localVerCode = getVersionCodeCompat(getSelfPackageInfoCompat())

        if (localVerCode > 0L && archiveVerCode in 1L until localVerCode) {
            file.delete()
            throw IOException("节点返回旧版本 v$archiveVerName (#$archiveVerCode < 本机 #$localVerCode)")
        }
        if (expectedMeta != null &&
            expectedMeta.versionCode > localVerCode &&
            archiveVerCode in 1L until expectedMeta.versionCode
        ) {
            file.delete()
            throw IOException("节点缓存仍为旧版 v$archiveVerName (#$archiveVerCode < 云端 #${expectedMeta.versionCode})")
        }

        return VerifiedApkSummary(
            file = file,
            versionCode = archiveVerCode,
            versionName = archiveVerName
        )
    }

    private fun getVerifiedCachedApk(requireNewerThanInstalled: Boolean): File? {
        val f = File(File(context.cacheDir, "ota"), APK_FILE_NAME)
        if (!f.exists() || f.length() < 250_000L) return null
        val info = getArchivePackageInfoCompat(f.absolutePath) ?: return null
        if (info.packageName != context.packageName) {
            f.delete()
            return null
        }
        val cachedVerCode = getVersionCodeCompat(info)
        val localInfo = getSelfPackageInfoCompat()
        val localVerCode = getVersionCodeCompat(localInfo)
        if (localVerCode > 0L && cachedVerCode in 1L until localVerCode) {
            f.delete()
            return null
        }
        if (requireNewerThanInstalled && localVerCode > 0L && cachedVerCode <= localVerCode) {
            return null
        }
        return f
    }

    private fun cleanupInstalledCachedApkIfNeeded() {
        runCatching {
            val otaDir = File(context.cacheDir, "ota")
            val cachedApk = File(otaDir, APK_FILE_NAME)
            if (!cachedApk.exists()) return@runCatching

            val archiveInfo = getArchivePackageInfoCompat(cachedApk.absolutePath)
            val selfInfo = getSelfPackageInfoCompat()
            val localVerCode = getVersionCodeCompat(selfInfo)
            val localVerName = selfInfo?.versionName ?: "3.13.0"
            val archiveVerCode = getVersionCodeCompat(archiveInfo)

            var alreadyInstalledOrOlder = archiveInfo == null ||
                archiveInfo.packageName != context.packageName ||
                (localVerCode > 0L && archiveVerCode in 1L..localVerCode)

            if (!alreadyInstalledOrOlder) {
                val selfSha = resolveInstalledApkSha256(selfInfo)
                if (selfSha.length == 64) {
                    val cachedSha = runCatching { computeFileSha256Hex(cachedApk) }.getOrDefault("")
                    if (cachedSha.equals(selfSha, ignoreCase = true)) {
                        alreadyInstalledOrOlder = true
                    }
                }
            }

            if (alreadyInstalledOrOlder) {
                cachedApk.delete()
                File(otaDir, "$APK_FILE_NAME.part").delete()
                File(otaDir, "$APK_FILE_NAME.part.meta").delete()
                var needDispatch = false
                synchronized(stateLock) {
                    if (curPhase == Phase.READY_TO_INSTALL) {
                        curPhase = Phase.UP_TO_DATE
                        curProgressPct = 0
                        curStatusMsg = "已完成更新 · 当前运行版本 v$localVerName (#$localVerCode)"
                        needDispatch = true
                    }
                }
                if (needDispatch) {
                    dispatchSnapshot()
                }
            }
        }
    }

    private fun fetchBuildInfoFromCandidates(prefixes: List<String>): ReleaseMeta? {
        val urls = LinkedHashSet<String>()
        urls.add(OFFICIAL_BUILD_INFO_URL)
        for (p in prefixes) {
            if (p.isNotEmpty()) {
                urls.add(buildCandidateAssetUrl(p, OFFICIAL_BUILD_INFO_URL))
            }
        }
        val tasks = urls.map { targetUrl ->
            Callable<ReleaseMeta?> {
                runCatching {
                    val req = Request.Builder()
                        .url(targetUrl)
                        .header("Accept", "application/json,*/*")
                        .header("User-Agent", "RadioMaster-EGO-Pilot-Android-OTA/3.13")
                        .header("Cache-Control", "no-cache, no-store, max-age=0")
                        .header("Pragma", "no-cache")
                        .get()
                        .build()
                    probeClient.newCall(req).execute().use { resp ->
                        if (!resp.isSuccessful) return@use null
                        parseBuildInfoJson(resp.body?.string().orEmpty())
                    }
                }.getOrNull()
            }
        }
        val futures = runCatching {
            probeExecutor.invokeAll(tasks, 3400L, TimeUnit.MILLISECONDS)
        }.getOrDefault(emptyList())

        val results = ArrayList<ReleaseMeta>(urls.size)
        for (fut in futures) {
            if (fut.isDone && !fut.isCancelled) {
                fut.let { runCatching { it.get() }.getOrNull() }?.let { results.add(it) }
            }
        }
        return results.maxWithOrNull(
            compareBy<ReleaseMeta> { it.versionCode }.thenBy { it.updatedEpochMs }
        )
    }

    private fun fetchGithubReleaseApiFromCandidates(prefixes: List<String>): ReleaseMeta? {
        val apiEndpoints = LinkedHashSet<String>()
        apiEndpoints.add(GITHUB_RELEASE_API_URL)
        for (p in prefixes) {
            if (p.isNotEmpty()) {
                apiEndpoints.add(p + GITHUB_RELEASE_API_URL)
            }
        }
        for (apiUrl in apiEndpoints) {
            val parsed = runCatching {
                val apiReq = Request.Builder()
                    .url(apiUrl)
                    .header("Accept", "application/vnd.github+json")
                    .header("User-Agent", "RadioMaster-EGO-Pilot-Android-OTA/3.13")
                    .header("Cache-Control", "no-cache, no-store, max-age=0")
                    .header("Pragma", "no-cache")
                    .get()
                    .build()
                probeClient.newCall(apiReq).execute().use { resp ->
                    if (!resp.isSuccessful) return@use null
                    parseGithubReleaseApiJson(resp.body?.string().orEmpty())
                }
            }.getOrNull()
            if (parsed != null) return parsed
        }
        return null
    }

    private fun fetchReleaseMetadata(): ReleaseMeta {
        val bestPref = synchronized(stateLock) { bestMirrorPrefix }
        val mirrorCandidates = if (hub.otaChannelMode == 1) {
            emptyList()
        } else {
            buildList {
                if (bestPref.isNotEmpty()) add(bestPref)
                addAll(VERIFIED_SEED_PREFIXES.take(2))
            }.distinct()
        }

        val buildInfoMeta = fetchBuildInfoFromCandidates(mirrorCandidates)
        val apiMeta = fetchGithubReleaseApiFromCandidates(mirrorCandidates)
        val merged = mergeReleaseMetadata(apiMeta, buildInfoMeta)
        if (merged != null) {
            return merged
        }

        val rankedTop = synchronized(stateLock) { rankedVerifiedNodes.firstOrNull { it.sizeBytes > 0L } }
        if (rankedTop != null && rankedTop.sizeBytes > 0L) {
            return ReleaseMeta(rankedTop.sizeBytes, rankedTop.updatedEpochMs)
        }

        val probed = probeSingleCandidateApkHeader(if (hub.otaChannelMode == 1) "" else bestPref)
            ?: throw IOException("无法获取云端构建元数据")
        return ReleaseMeta(probed.sizeBytes, probed.updatedEpochMs)
    }

    private fun formatRemoteMetaSummary(meta: ReleaseMeta): String {
        val sizeStr = if (meta.sizeBytes > 0L) formatBytes(meta.sizeBytes) else "大小待查"
        val utcStr = if (meta.updatedEpochMs > 0L) formatUtcMs(meta.updatedEpochMs) else "时间待查"
        val verPrefix = when {
            meta.versionName.isNotEmpty() && meta.versionCode > 0L ->
                "v${meta.versionName} (#${meta.versionCode}) · "
            meta.versionName.isNotEmpty() ->
                "v${meta.versionName} · "
            meta.commitSha.length >= 7 ->
                "commit ${meta.commitSha.take(7)} · "
            else -> ""
        }
        return "云端产物 : ${verPrefix}$sizeStr · $utcStr"
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
                    val host = if (bestMirrorPrefix.isEmpty()) {
                        "github.com (官方)"
                    } else {
                        extractDisplayHost(bestMirrorPrefix)
                    }
                    val rttPart = if (bestMirrorRttMs > 0L) " (${bestMirrorRttMs}ms)" else ""
                    "加速源池 : 预设优选 $host$rttPart · 点「刷新加速源」实时拉取云端源测速"
                }
            }
        }
    }

    private fun buildSnapshotLocked(): Snapshot {
        val localInfo = getSelfPackageInfoCompat()
        val vName = localInfo?.versionName ?: "3.13.0"
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
