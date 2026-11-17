package com.radiomaster.egogcs.ros

import com.radiomaster.egogcs.model.TopicConfig
import java.io.ByteArrayOutputStream
import java.net.ConnectException
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.Inet4Address
import java.net.InetAddress
import java.net.InetSocketAddress
import java.nio.charset.StandardCharsets
import java.util.Collections
import java.util.Locale
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

/**
 * 发现的局域网机载 ROS1 无人机 / 机载计算节点条目：
 * 包含 IP、开放协议端口、RTT 往返延迟、自动指纹识别出的机载算法栈名称、自动配对好的 TopicConfig，
 * 以及 `rosReady` 标志（`true` 表示 `11311`/`9090` 已就绪可直接一键连接；`false` 表示机载 Linux 在线但尚未启动 `roscore`）。
 */
data class DiscoveredDrone(
    val ip: String,
    val port: Int,
    // "tcpros" (11311) 或 "rosbridge" (9090 / 8765)
    val protocol: String,
    val rttMs: Int,
    val stackSummary: String,
    val topicCount: Int,
    val suggestedConfig: TopicConfig,
    val rosReady: Boolean = true
)

/**
 * 多引擎抗干扰局域网无人机自动发现与 ROS1 话题指纹识别器 (`LanDroneScanner`)：
 *
 * 1. **物理网卡精准路由 (`LanNetworkBinder`)**：综合 `ConnectivityManager` (`TRANSPORT_WIFI / ETHERNET / USB`)、
 *    `WifiManager.dhcpInfo` 与物理网口枚举，彻底过滤 4G/5G (`rmnet*`) 与 VPN/代理 (`tun*`/`wg*`/`198.18.*`)，
 *    并将所有探测 Socket 绑定至物理局域网网卡，解决无外网机载热点下被蜂窝流量劫持导致的“全网段漏扫”；
 * 2. **mDNS (`224.0.0.251:5353`) + ARP 邻居表 + 跨子网 (`/22~/24`) 联合加速**：开局并行广播 mDNS 服务与常见机载主机名查询
 *    (`ubuntu.local`, `jetson.local`, `orin.local`, `nuc.local`, `_ros._tcp.local` 等)，即使在实验室 `/16`、`/22` 大子网也能秒级命中；
 * 3. **自适应双波次并发与冷启动 ARP 重试**：第一波高优先级节点采用 `650ms + 二次重试` 穿透 Wi-Fi 省电休眠与冷 ARP 解析，
 *    第二波 `28` 线程温和扫完所有活动 `/24` 块，避免 40+ 线程并发广播击穿路由器/网卡 ARP 队列；
 * 4. **全兼容 XML-RPC 协议栈与机载 Linux 在线诊断**：同时兼容 Python `roscore` `<string>` 与 C++ `xmlrpcpp` 省略 `<string>` 的响应，
 *    并在机载电脑刚开机尚未启动 `roscore` 时通过 OpenSSH 横幅识别在线 Ubuntu/Jetson 主机，彻底告别“盲猜 IP”。
 */
class LanDroneScanner(
    internal val standardBackoffPolicy: ExponentialBackoffPolicy = ExponentialBackoffPolicy(
        baseDelayMs = 2_000L,
        maxDelayMs = 30_000L,
        multiplier = 2.0,
        jitterRatio = 0.15
    ),
    internal val bootingHostBackoffPolicy: ExponentialBackoffPolicy = ExponentialBackoffPolicy(
        baseDelayMs = 1_500L,
        maxDelayMs = 12_000L,
        multiplier = 1.8,
        jitterRatio = 0.12
    )
) {

    private sealed class PortProbeResult {
        data class Open(val rttMs: Int) : PortProbeResult()
        object Refused : PortProbeResult()
        object TimeoutOrUnreachable : PortProbeResult()
    }

    private val scanGen = AtomicInteger(0)
    private val retryGen = AtomicInteger(0)
    private val retryLock = Any()
    private val shuttingDown = AtomicBoolean(false)

    private val retryScheduler: ScheduledExecutorService = Executors.newSingleThreadScheduledExecutor { r ->
        Thread(r, "LanDiscoveryBackoff").apply { isDaemon = true }
    }

    @Volatile
    private var pendingRetryFuture: ScheduledFuture<*>? = null

    @Volatile
    var lastScheduledDelayMs: Long = 0L
        private set

    @Volatile
    var currentBackoffAttempt: Int = 0
        private set

    private val knownCandidateIps: MutableSet<String> = ConcurrentHashMap.newKeySet()

    @Volatile
    private var activeGen: Int = 0

    @Volatile
    private var scanMasterThread: Thread? = null

    val isScanning: Boolean
        get() = activeGen != 0

    val isBackoffScheduled: Boolean
        get() = synchronized(retryLock) {
            val fut = pendingRetryFuture
            fut != null && !fut.isDone && !fut.isCancelled
        }

    fun rememberCandidateHost(rawHostOrUri: String?) {
        val host = extractEndpointHost(rawHostOrUri) ?: return
        if (LanNetworkBinder.isUsableLanIpv4(host) || host == "127.0.0.1") {
            knownCandidateIps.add(host)
        }
    }

    fun cancelScheduledBackoff() {
        retryGen.incrementAndGet()
        synchronized(retryLock) {
            pendingRetryFuture?.cancel(false)
            pendingRetryFuture = null
        }
    }

    fun resetBackoff() {
        cancelScheduledBackoff()
        standardBackoffPolicy.reset()
        bootingHostBackoffPolicy.reset()
        currentBackoffAttempt = 0
        lastScheduledDelayMs = 0L
    }

    fun cancelScan(cancelAutoRetry: Boolean = true) {
        if (cancelAutoRetry) {
            cancelScheduledBackoff()
        }
        scanGen.incrementAndGet()
        activeGen = 0
        scanMasterThread?.interrupt()
    }

    fun shutdown() {
        shuttingDown.set(true)
        resetBackoff()
        cancelScan(cancelAutoRetry = true)
        retryScheduler.shutdownNow()
    }

    fun startScan(
        baseConfig: TopicConfig,
        manualHintRaw: String? = null,
        backgroundRecoveryPass: Boolean = false,
        restrictToHintsOnly: Boolean = false,
        shouldScheduleAutoRetry: () -> Boolean = { false },
        supplyLatestConfig: () -> TopicConfig = { baseConfig },
        supplyLatestHints: () -> String? = { manualHintRaw },
        onBackoffScheduled: (attempt: Int, delayMs: Long, reason: DiscoveryRecoveryReason) -> Unit = { _, _, _ -> },
        onProgress: (percent: Int, hint: String) -> Unit,
        onDroneFound: (DiscoveredDrone) -> Unit,
        onComplete: (List<DiscoveredDrone>, String) -> Unit
    ) {
        if (shuttingDown.get()) return
        cancelScheduledBackoff()
        if (!backgroundRecoveryPass) {
            standardBackoffPolicy.reset()
            bootingHostBackoffPolicy.reset()
            currentBackoffAttempt = 0
            lastScheduledDelayMs = 0L
        }

        val myGen = scanGen.incrementAndGet()
        activeGen = myGen
        scanMasterThread?.interrupt()

        scanMasterThread = Thread({
            val foundByIp = ConcurrentHashMap<String, DiscoveredDrone>()
            val orderedFound = Collections.synchronizedList(mutableListOf<DiscoveredDrone>())
            var subnetSummary = "局域网"
            try {
                val lanInterfaces = if (restrictToHintsOnly) emptyList() else LanNetworkBinder.discoverActiveLanInterfaces()
                val localIps = HashSet<String>()
                val gatewayIps = LinkedHashSet<String>()
                val prefixes = LinkedHashSet<String>()

                for (iface in lanInterfaces) {
                    localIps.add(iface.localIp)
                    if (!iface.gatewayIp.isNullOrBlank()) {
                        gatewayIps.add(iface.gatewayIp)
                    }
                    for (pfx in LanNetworkBinder.expandSubnet24Prefixes(iface.localIp, iface.prefixLength, iface.gatewayIp)) {
                        prefixes.add(pfx)
                    }
                }

                // 提取历史保存地址、多机编队节点 IP 以及输入框当前文本中的 IP / 网段提示
                val hintHosts = LinkedHashSet<String>()
                extractEndpointHost(baseConfig.masterUri)?.let { hintHosts.add(it) }
                extractEndpointHost(baseConfig.wsUrl)?.let { hintHosts.add(it) }
                if (baseConfig.swarmPeerIpsCsv.isNotBlank()) {
                    for (part in baseConfig.swarmPeerIpsCsv.split(',', ';', ' ', '\n')) {
                        extractEndpointHost(part)?.let { hintHosts.add(it) }
                    }
                }
                if (!manualHintRaw.isNullOrBlank()) {
                    for (token in manualHintRaw.split(',', ';', ' ', '\t', '\n')) {
                        extractEndpointHost(token)?.let { hintHosts.add(it) }
                    }
                }

                for (h in hintHosts) {
                    if (LanNetworkBinder.isUsableLanIpv4(h)) {
                        val dot = h.lastIndexOf('.')
                        if (dot > 0 && prefixes.size < 4) {
                            prefixes.add(h.substring(0, dot + 1))
                        }
                    }
                }

                val arpPeers = if (restrictToHintsOnly) emptySet() else LanNetworkBinder.readKnownArpNeighborIps()
                for (arpIp in arpPeers) {
                    val dot = arpIp.lastIndexOf('.')
                    if (dot > 0 && prefixes.size < 4) {
                        prefixes.add(arpIp.substring(0, dot + 1))
                    }
                }

                val hasActiveLanLink = lanInterfaces.isNotEmpty()
                if (prefixes.isEmpty() && !restrictToHintsOnly) {
                    // 未检测到常规局域网接口时，兜底覆盖 Ubuntu 热点 / 手机热点 / 数字图传网口 / 常用路由默认网段
                    prefixes.add("10.42.0.1".substring(0, 8))
                    prefixes.add("192.168.4.")
                    prefixes.add("192.168.43.")
                    prefixes.add("192.168.144.")
                    prefixes.add("192.168.1.")
                }

                val activePrefixes = when {
                    restrictToHintsOnly -> emptyList()
                    hasActiveLanLink -> prefixes.take(4)
                    else -> prefixes.take(2)
                }

                subnetSummary = when {
                    restrictToHintsOnly -> "指定目标节点"
                    hasActiveLanLink -> {
                        val primary = lanInterfaces.first()
                        val extra = if (activePrefixes.size > 1) " 等${activePrefixes.size}个网段" else ""
                        "${primary.interfaceName} ${primary.localIp}/${primary.prefixLength}$extra"
                    }
                    else -> "默认机载热点网段 (${activePrefixes.joinToString(", ") { "${it}0/24" }})"
                }

                val customMasterPort = extractEndpointPort(baseConfig.masterUri)
                val customWsPort = extractEndpointPort(baseConfig.wsUrl)
                val masterPorts = buildList {
                    if (customMasterPort != null && customMasterPort != 11311 && customMasterPort != 9090) {
                        add(customMasterPort)
                    }
                    add(11311)
                }
                val wsPorts = buildList {
                    if (customWsPort != null && customWsPort != 9090 && customWsPort != 11311) {
                        add(customWsPort)
                    }
                    add(9090)
                }

                // 阶段 0：断链快速重捕获目标 (历史命中无人机 IP + 用户提示 IP + ARP 邻居)
                val rapidTargets = LinkedHashSet<String>()
                val priorityTargets = LinkedHashSet<String>()
                val confirmedPeers = ConcurrentHashMap.newKeySet<String>()
                confirmedPeers.addAll(arpPeers)
                confirmedPeers.addAll(knownCandidateIps)

                for (knownIp in knownCandidateIps) {
                    if (knownIp !in localIps && (restrictToHintsOnly || knownIp != "127.0.0.1")) {
                        rapidTargets.add(knownIp)
                        priorityTargets.add(knownIp)
                    }
                }
                for (h in hintHosts) {
                    val allowLoopback = restrictToHintsOnly
                    if ((allowLoopback || (h != "127.0.0.1" && !h.equals("localhost", ignoreCase = true))) && h !in localIps) {
                        rapidTargets.add(h)
                        priorityTargets.add(h)
                    }
                }
                if (!restrictToHintsOnly) {
                    for (gw in gatewayIps) {
                        if (gw !in localIps) {
                            rapidTargets.add(gw)
                            priorityTargets.add(gw)
                        }
                    }
                    for (arpIp in arpPeers) {
                        if (arpIp !in localIps) {
                            rapidTargets.add(arpIp)
                            priorityTargets.add(arpIp)
                        }
                    }
                    // 常见机载热点 / 图传固定地址
                    for (wellKnown in arrayOf("10.42.0.1", "192.168.4.1", "192.168.144.10", "192.168.144.20", "192.168.42.129")) {
                        if (wellKnown !in localIps) {
                            rapidTargets.add(wellKnown)
                            priorityTargets.add(wellKnown)
                        }
                    }

                    val hotSuffixes = intArrayOf(
                        1, 2, 10, 11, 20, 30, 42, 50, 60, 80, 88, 99,
                        100, 101, 102, 103, 104, 105, 108, 110, 120, 128, 150, 168, 188, 200, 250, 254
                    )
                    for (pfx in activePrefixes) {
                        for (s in hotSuffixes) {
                            val ip = "$pfx$s"
                            if (ip !in localIps) priorityTargets.add(ip)
                        }
                    }
                }

                // 第二波：活动网段全量 1..254 覆盖队列
                val fullSweepTargets = LinkedHashSet<String>()
                if (!restrictToHintsOnly) {
                    for (pfx in activePrefixes) {
                        for (hostId in 1..254) {
                            val ip = "$pfx$hostId"
                            if (ip !in localIps && ip !in priorityTargets) {
                                fullSweepTargets.add(ip)
                            }
                        }
                    }
                }

                val totalPlanned = (priorityTargets.size + fullSweepTargets.size).coerceAtLeast(1)
                val finishedCount = AtomicInteger(0)
                val submittedIps = ConcurrentHashMap.newKeySet<String>()

                if (scanGen.get() == myGen) {
                    val modeTag = if (backgroundRecoveryPass) "后台自愈重探" else "正在探测"
                    onProgress(3, "$modeTag $subnetSummary (优先热区 + mDNS 广播)...")
                }

                fun reportHit(hit: DiscoveredDrone) {
                    if (scanGen.get() != myGen) return
                    rememberCandidateHost(hit.ip)
                    val prev = foundByIp.putIfAbsent(hit.ip, hit)
                    if (prev == null) {
                        orderedFound.add(hit)
                        onDroneFound(hit)
                    } else if (!prev.rosReady && hit.rosReady) {
                        foundByIp[hit.ip] = hit
                        synchronized(orderedFound) {
                            val idx = orderedFound.indexOfFirst { it.ip == hit.ip }
                            if (idx >= 0) orderedFound[idx] = hit else orderedFound.add(hit)
                        }
                        onDroneFound(hit)
                    }
                }

                val workerCount = when {
                    restrictToHintsOnly -> rapidTargets.size.coerceIn(1, 6)
                    backgroundRecoveryPass -> 16
                    else -> 28
                }
                val pool = Executors.newFixedThreadPool(workerCount) { r ->
                    Thread(r, "LanRosProbe").apply { isDaemon = true }
                }

                try {
                    // 后台自愈模式下先对历史已知节点 + ARP 邻居执行极速重捕获：若原 IP 无人机已恢复则毫秒级返回，免扫整网段
                    var shortCircuitAfterRapid = false
                    if (backgroundRecoveryPass && rapidTargets.isNotEmpty()) {
                        val rapidLatch = CountDownLatch(rapidTargets.size)
                        for (ip in rapidTargets) {
                            pool.execute {
                                try {
                                    if (scanGen.get() != myGen) return@execute
                                    if (!submittedIps.add(ip)) return@execute
                                    val resolvedIp = resolveHostnameIfNeeded(ip, allowLoopback = restrictToHintsOnly) ?: return@execute
                                    if (resolvedIp in localIps) return@execute
                                    if (resolvedIp != ip && !submittedIps.add(resolvedIp)) return@execute
                                    val hit = probeSingleHost(
                                        ip = resolvedIp,
                                        baseCfg = baseConfig,
                                        masterPorts = masterPorts,
                                        wsPorts = wsPorts,
                                        isPriority = true,
                                        isConfirmedPeer = resolvedIp in confirmedPeers,
                                        isRouterGateway = resolvedIp in gatewayIps &&
                                            resolvedIp != "10.42.0.1" &&
                                            resolvedIp != "192.168.4.1"
                                    )
                                    if (hit != null) reportHit(hit)
                                } catch (_: Throwable) {
                                } finally {
                                    finishedCount.incrementAndGet()
                                    rapidLatch.countDown()
                                }
                            }
                        }
                        rapidLatch.await(3500L, TimeUnit.MILLISECONDS)
                        shortCircuitAfterRapid = synchronized(orderedFound) { orderedFound.any { it.rosReady } }
                    }

                    if (!shortCircuitAfterRapid) {
                        // 并行启动 mDNS 局域网零配置发现 (穿透 /16 大网段与动态 DHCP 分配)
                        val mdnsLatch = CountDownLatch(if (restrictToHintsOnly) 0 else 1)
                        if (!restrictToHintsOnly) {
                            pool.execute {
                                try {
                                    if (scanGen.get() == myGen) {
                                        val mdnsIps = discoverMdnsPeers(localIps, timeoutMs = 420)
                                        for (mIp in mdnsIps) {
                                            if (scanGen.get() != myGen) break
                                            if (mIp in localIps) continue
                                            confirmedPeers.add(mIp)
                                            // 若 mDNS 发现了跨网段或尚未探测的新 IP，立即插队直探
                                            if (submittedIps.add(mIp)) {
                                                val hit = probeSingleHost(
                                                    ip = mIp,
                                                    baseCfg = baseConfig,
                                                    masterPorts = masterPorts,
                                                    wsPorts = wsPorts,
                                                    isPriority = true,
                                                    isConfirmedPeer = true,
                                                    isRouterGateway = mIp in gatewayIps && mIp != "10.42.0.1" && mIp != "192.168.4.1"
                                                )
                                                if (hit != null) reportHit(hit)
                                            }
                                        }
                                    }
                                } catch (_: Throwable) {
                                } finally {
                                    mdnsLatch.countDown()
                                }
                            }
                        }

                        // 先派发第一波高优先级目标，再无缝衔接全子网扫描
                        val remainingTargets = ArrayList<Pair<String, Boolean>>(priorityTargets.size + fullSweepTargets.size)
                        for (ip in priorityTargets) {
                            if (ip !in submittedIps) remainingTargets.add(ip to true)
                        }
                        for (ip in fullSweepTargets) {
                            if (ip !in submittedIps) remainingTargets.add(ip to false)
                        }

                        val sweepLatch = CountDownLatch(remainingTargets.size)
                        for ((ip, isPrio) in remainingTargets) {
                            pool.execute {
                                try {
                                    if (scanGen.get() != myGen) return@execute
                                    if (!submittedIps.add(ip)) return@execute
                                    val resolvedIp = resolveHostnameIfNeeded(ip, allowLoopback = restrictToHintsOnly) ?: return@execute
                                    if (resolvedIp in localIps) return@execute
                                    if (resolvedIp != ip && !submittedIps.add(resolvedIp)) return@execute

                                    val hit = probeSingleHost(
                                        ip = resolvedIp,
                                        baseCfg = baseConfig,
                                        masterPorts = masterPorts,
                                        wsPorts = wsPorts,
                                        isPriority = isPrio,
                                        isConfirmedPeer = resolvedIp in confirmedPeers,
                                        isRouterGateway = resolvedIp in gatewayIps &&
                                            resolvedIp != "10.42.0.1" &&
                                            resolvedIp != "192.168.4.1"
                                    )
                                    if (hit != null) {
                                        reportHit(hit)
                                    }
                                } catch (_: Throwable) {
                                } finally {
                                    val done = finishedCount.incrementAndGet()
                                    if (scanGen.get() == myGen && (done % 16 == 0 || done >= totalPlanned)) {
                                        val pct = ((done * 100) / totalPlanned).coerceIn(5, 99)
                                        val foundCnt = orderedFound.size
                                        val suffix = if (foundCnt > 0) " · 已发现 $foundCnt 台" else ""
                                        onProgress(pct, "正在扫描 $subnetSummary ($pct%$suffix)")
                                    }
                                    sweepLatch.countDown()
                                }
                            }
                        }

                        mdnsLatch.await(1200, TimeUnit.MILLISECONDS)
                        // 根据扫描规模动态留足等待上限，防止双子网/四子网未扫完被提前截断
                        val maxWaitSec = (8L + (remainingTargets.size / 32L)).coerceIn(6L, 26L)
                        sweepLatch.await(maxWaitSec, TimeUnit.SECONDS)
                    }
                } finally {
                    pool.shutdownNow()
                }
            } catch (_: Throwable) {
            } finally {
                if (scanGen.get() == myGen) {
                    activeGen = 0
                    val snapshot = synchronized(orderedFound) { ArrayList(orderedFound) }
                    val sorted = snapshot.sortedWith(
                        compareBy<DiscoveredDrone>(
                            { if (it.rosReady) 0 else 1 },
                            { if (it.protocol == "tcpros") 0 else 1 },
                            { it.rttMs }
                        )
                    )
                    onComplete(sorted, subnetSummary)

                    val foundReadyDrone = sorted.any { it.rosReady }
                    if (foundReadyDrone) {
                        resetBackoff()
                    } else if (!shuttingDown.get() && shouldScheduleAutoRetry()) {
                        scheduleNextBackoffPass(
                            sawBootingHost = sorted.isNotEmpty(),
                            restrictToHintsOnly = restrictToHintsOnly,
                            shouldScheduleAutoRetry = shouldScheduleAutoRetry,
                            supplyLatestConfig = supplyLatestConfig,
                            supplyLatestHints = supplyLatestHints,
                            onBackoffScheduled = onBackoffScheduled,
                            onProgress = onProgress,
                            onDroneFound = onDroneFound,
                            onComplete = onComplete
                        )
                    }
                }
            }
        }, "LanDroneScanMaster").apply {
            isDaemon = true
            start()
        }
    }

    private fun scheduleNextBackoffPass(
        sawBootingHost: Boolean,
        restrictToHintsOnly: Boolean,
        shouldScheduleAutoRetry: () -> Boolean,
        supplyLatestConfig: () -> TopicConfig,
        supplyLatestHints: () -> String?,
        onBackoffScheduled: (attempt: Int, delayMs: Long, reason: DiscoveryRecoveryReason) -> Unit,
        onProgress: (percent: Int, hint: String) -> Unit,
        onDroneFound: (DiscoveredDrone) -> Unit,
        onComplete: (List<DiscoveredDrone>, String) -> Unit
    ) {
        if (shuttingDown.get()) return
        val policy = if (sawBootingHost) {
            standardBackoffPolicy.reset()
            bootingHostBackoffPolicy
        } else {
            bootingHostBackoffPolicy.reset()
            standardBackoffPolicy
        }
        val delayMs = policy.nextDelayMs()
        if (delayMs < 0L) return
        val attempt = policy.attemptCount
        val reason = if (sawBootingHost) {
            DiscoveryRecoveryReason.HOST_BOOTING_ROSCORE
        } else {
            DiscoveryRecoveryReason.EMPTY_SCAN_BACKOFF
        }
        currentBackoffAttempt = attempt
        lastScheduledDelayMs = delayMs
        val myRetryToken = retryGen.incrementAndGet()

        onBackoffScheduled(attempt, delayMs, reason)

        try {
            val fut = retryScheduler.schedule({
                if (shuttingDown.get() || retryGen.get() != myRetryToken) return@schedule
                synchronized(retryLock) {
                    pendingRetryFuture = null
                }
                if (!shouldScheduleAutoRetry()) return@schedule
                startScan(
                    baseConfig = supplyLatestConfig(),
                    manualHintRaw = supplyLatestHints(),
                    backgroundRecoveryPass = true,
                    restrictToHintsOnly = restrictToHintsOnly,
                    shouldScheduleAutoRetry = shouldScheduleAutoRetry,
                    supplyLatestConfig = supplyLatestConfig,
                    supplyLatestHints = supplyLatestHints,
                    onBackoffScheduled = onBackoffScheduled,
                    onProgress = onProgress,
                    onDroneFound = onDroneFound,
                    onComplete = onComplete
                )
            }, delayMs, TimeUnit.MILLISECONDS)
            synchronized(retryLock) {
                if (retryGen.get() == myRetryToken && !shuttingDown.get()) {
                    pendingRetryFuture = fut
                } else {
                    fut.cancel(false)
                }
            }
        } catch (_: Throwable) {
        }
    }

    /**
     * 对单个目标 IP 执行多协议栈探测（供主扫描引擎与单元测试直接调用）。
     */
    internal fun probeSingleHost(
        ip: String,
        baseCfg: TopicConfig,
        masterPorts: List<Int> = listOf(11311),
        wsPorts: List<Int> = listOf(9090),
        isPriority: Boolean = false,
        isConfirmedPeer: Boolean = false,
        isRouterGateway: Boolean = false
    ): DiscoveredDrone? {
        var hostAliveOnLan = isConfirmedPeer
        val masterTimeout = if (isPriority || isConfirmedPeer) 650 else 450

        // 1. 优先探测原生 ROS1 Master (默认 11311 及用户自定义端口)
        for (port in masterPorts) {
            var res = tcpProbePort(ip, port, masterTimeout)
            if (res is PortProbeResult.TimeoutOrUnreachable && (isPriority || isConfirmedPeer)) {
                // 高优先级目标或已知在线邻居二次重试，穿透冷 ARP 广播与网卡省电休眠
                res = tcpProbePort(ip, port, 750)
            }
            when (res) {
                is PortProbeResult.Open -> {
                    val inspected = inspectRosMaster(ip, port, res.rttMs, baseCfg)
                    if (inspected != null) return inspected
                    // 即使 XML-RPC 因负载或自定义安全网关超时，只要 11311 TCP 握手成功也绝不丢弃真机！
                    val fallbackCfg = baseCfg.copy(
                        protocol = "tcpros",
                        masterUri = "http://$ip:$port",
                        wsUrl = "ws://$ip:${wsPorts.firstOrNull() ?: 9090}"
                    )
                    return DiscoveredDrone(
                        ip = ip,
                        port = port,
                        protocol = "tcpros",
                        rttMs = res.rttMs,
                        stackSummary = "ROS1 Master 端口在线",
                        topicCount = 0,
                        suggestedConfig = fallbackCfg,
                        rosReady = true
                    )
                }
                is PortProbeResult.Refused -> {
                    hostAliveOnLan = true
                }
                is PortProbeResult.TimeoutOrUnreachable -> Unit
            }
        }

        // 2. 探测 rosbridge WebSocket (默认 9090 及用户自定义 WS 端口)
        val wsTimeout = when {
            hostAliveOnLan -> 380 // 已知主机存活且 ARP 已就绪，快速确认 9090
            isPriority -> 520
            else -> 290
        }
        for (port in wsPorts) {
            var res = tcpProbePort(ip, port, wsTimeout)
            if (res is PortProbeResult.TimeoutOrUnreachable && isPriority) {
                res = tcpProbePort(ip, port, 620)
            }
            when (res) {
                is PortProbeResult.Open -> {
                    val cfg = baseCfg.copy(
                        protocol = "rosbridge",
                        masterUri = "http://$ip:${masterPorts.firstOrNull() ?: 11311}",
                        wsUrl = "ws://$ip:$port"
                    )
                    return DiscoveredDrone(
                        ip = ip,
                        port = port,
                        protocol = "rosbridge",
                        rttMs = res.rttMs,
                        stackSummary = "rosbridge WebSocket 在线",
                        topicCount = 0,
                        suggestedConfig = cfg,
                        rosReady = true
                    )
                }
                is PortProbeResult.Refused -> {
                    hostAliveOnLan = true
                }
                is PortProbeResult.TimeoutOrUnreachable -> Unit
            }
        }

        // 3. 对确认在线的局域网主机，额外探测 8765 (Foxglove/WebSocket 桥) 与 22 (机载 Linux OpenSSH 在线诊断)
        if (hostAliveOnLan) {
            if (8765 !in wsPorts) {
                val foxRes = tcpProbePort(ip, 8765, 220)
                if (foxRes is PortProbeResult.Open) {
                    val cfg = baseCfg.copy(
                        protocol = "rosbridge",
                        masterUri = "http://$ip:${masterPorts.firstOrNull() ?: 11311}",
                        wsUrl = "ws://$ip:8765"
                    )
                    return DiscoveredDrone(
                        ip = ip,
                        port = 8765,
                        protocol = "rosbridge",
                        rttMs = foxRes.rttMs,
                        stackSummary = "WebSocket 桥接端口 (:8765) 在线",
                        topicCount = 0,
                        suggestedConfig = cfg,
                        rosReady = true
                    )
                }
            }

            val sshDiagnostic = inspectSshLinuxBanner(ip, sshPort = 22, isRouterGateway = isRouterGateway, baseCfg = baseCfg)
            if (sshDiagnostic != null) {
                return sshDiagnostic
            }
        }

        return null
    }

    private fun tcpProbePort(ip: String, port: Int, timeoutMs: Int): PortProbeResult {
        val t0 = System.nanoTime()
        return try {
            LanNetworkBinder.createBoundSocket(ip).use { s ->
                s.tcpNoDelay = true
                s.connect(InetSocketAddress(ip, port), timeoutMs)
            }
            val rtt = ((System.nanoTime() - t0) / 1_000_000L).toInt().coerceAtLeast(1)
            PortProbeResult.Open(rtt)
        } catch (_: ConnectException) {
            // 收到对端内核发回的 TCP RST，证明该 IP 设备 100% 在线且 ARP 已解析完成
            PortProbeResult.Refused
        } catch (_: Throwable) {
            PortProbeResult.TimeoutOrUnreachable
        }
    }

    internal fun inspectRosMaster(ip: String, port: Int, rttMs: Int, baseCfg: TopicConfig): DiscoveredDrone? {
        val masterUri = "http://$ip:$port"
        val reqXml = "<?xml version=\"1.0\"?><methodCall><methodName>getSystemState</methodName>" +
            "<params><param><value><string>/ego_lan_scanner</string></value></param></params></methodCall>"

        var respXml: String? = null
        for (attempt in 0..1) {
            respXml = try {
                LanNetworkBinder.postXmlRpc(
                    uri = masterUri,
                    xmlBody = reqXml,
                    connectTimeoutMs = if (attempt == 0) 800 else 1200,
                    readTimeoutMs = if (attempt == 0) 1100 else 1600
                )
            } catch (_: Throwable) {
                null
            }
            if (respXml != null && respXml.contains("<methodResponse")) break
        }

        val xml = respXml ?: return null
        if (!xml.contains("<methodResponse")) return null

        // 同时兼容 Python roscore `<value><string>/Odometry</string></value>` 与 C++ xmlrpcpp `<value>/Odometry</value>`
        val rosNames = LinkedHashSet<String>()
        val valRegex = Regex("<value>\\s*(?:<string>)?\\s*(/[^<\\s]+)\\s*(?:</string>)?\\s*</value>")
        for (m in valRegex.findAll(xml)) {
            rosNames.add(m.groupValues[1])
        }
        val strFallbackRegex = Regex("<string>\\s*(/[^<\\s]+)\\s*</string>")
        for (m in strFallbackRegex.findAll(xml)) {
            rosNames.add(m.groupValues[1])
        }

        val cfg = baseCfg.copy(
            protocol = "tcpros",
            masterUri = masterUri,
            wsUrl = "ws://$ip:9090"
        )

        val tags = mutableListOf<String>()

        // 1. 识别 FAST-LIO / FAST-LIO2 / VINS-Fusion / MAVROS 里程计与点云话题
        val hasFastLioCloud = "/cloud_registered" in rosNames ||
            "/cloud_registered_body" in rosNames ||
            "/laser_cloud_surround" in rosNames
        val hasFastLioOdom = "/Odometry" in rosNames || "/aft_mapped_to_init" in rosNames
        if (hasFastLioCloud || hasFastLioOdom) {
            tags.add("FAST-LIO2")
            if ("/Odometry" in rosNames) {
                cfg.odomTopic = "/Odometry"
            } else if ("/aft_mapped_to_init" in rosNames) {
                cfg.odomTopic = "/aft_mapped_to_init"
            }
            when {
                "/cloud_registered" in rosNames -> cfg.fastLioCloudTopic = "/cloud_registered"
                "/cloud_registered_body" in rosNames -> cfg.fastLioCloudTopic = "/cloud_registered_body"
                "/laser_cloud_surround" in rosNames -> cfg.fastLioCloudTopic = "/laser_cloud_surround"
            }
            if ("/path" in rosNames) cfg.fastLioPathTopic = "/path"
        } else {
            val vinsOdom = sequenceOf(
                "/vins_fusion/imu_propagate",
                "/vins_fusion/odometry",
                "/vins_estimator/imu_propagate",
                "/vins_estimator/odometry",
                "/visual_slam/odom"
            ).firstOrNull { it in rosNames }
            if (vinsOdom != null) {
                tags.add("VINS-Fusion")
                cfg.odomTopic = vinsOdom
            } else if ("/mavros/local_position/odom" in rosNames) {
                cfg.odomTopic = "/mavros/local_position/odom"
            }
        }

        // 2. 识别单机 EGO-Planner 或多机 EGO-Swarm (drone_X)
        val swarmOptMatch = rosNames.firstOrNull { it.endsWith("_ego_planner_node/optimal_list") }
        if (swarmOptMatch != null) {
            val prefix = swarmOptMatch.removeSuffix("_ego_planner_node/optimal_list") // e.g. "/drone_0"
            val droneLabel = prefix.removePrefix("/")
            tags.add("Swarm($droneLabel)")
            cfg.optimalTopic = swarmOptMatch
            val swarmInf = "${prefix}_ego_planner_node/grid_map/occupancy_inflate"
            if (swarmInf in rosNames) cfg.inflateTopic = swarmInf
        } else if ("/ego_planner_node/optimal_list" in rosNames || "/grid_map/occupancy_inflate" in rosNames) {
            tags.add("EGO-Planner")
            if ("/ego_planner_node/optimal_list" in rosNames) cfg.optimalTopic = "/ego_planner_node/optimal_list"
            if ("/grid_map/occupancy_inflate" in rosNames) cfg.inflateTopic = "/grid_map/occupancy_inflate"
        }

        // 3. 识别主动倾转 / PX4 飞控链路
        val tiltMatch = rosNames.firstOrNull {
            val l = it.lowercase(Locale.US)
            l.contains("tilt") || l.contains("servo_angles") || l.contains("indi") || l.contains("active_pitch")
        }
        if (tiltMatch != null) {
            tags.add("Tilt-INDI")
            if (!tiltMatch.contains("cmd")) {
                cfg.tiltTelemetryTopic = tiltMatch
            }
            val cmdMatch = rosNames.firstOrNull {
                val l = it.lowercase(Locale.US)
                l.contains("pitch_tilt_cmd") || l.contains("active_pitch")
            }
            if (cmdMatch != null) cfg.tiltCmdTopic = cmdMatch
        } else if ("/mavros/actuator_control" in rosNames) {
            tags.add("PX4-Tilt")
            cfg.tiltTelemetryTopic = "/mavros/actuator_control"
        } else if ("/mavros/state" in rosNames) {
            tags.add("PX4")
        }

        val summary = if (tags.isNotEmpty()) tags.joinToString(" + ") else "ROS1 roscore 在线"
        return DiscoveredDrone(
            ip = ip,
            port = port,
            protocol = "tcpros",
            rttMs = rttMs,
            stackSummary = summary,
            topicCount = rosNames.size,
            suggestedConfig = cfg,
            rosReady = true
        )
    }

    /**
     * 对已确认存活但未开放 `11311/9090` 的局域网主机执行快速 SSH (`:22`) 横幅识别：
     * 精准捕获刚开机尚未运行 `roslaunch`（或防火墙拦截 `11311`）的 Ubuntu / Debian / JetPack 机载电脑，
     * 同时排除普通家用路由器的 `Dropbear` 嵌入式 SSH，避免误报。
     */
    internal fun inspectSshLinuxBanner(
        ip: String,
        sshPort: Int = 22,
        isRouterGateway: Boolean,
        baseCfg: TopicConfig
    ): DiscoveredDrone? {
        val t0 = System.nanoTime()
        return try {
            LanNetworkBinder.createBoundSocket(ip).use { s ->
                s.tcpNoDelay = true
                s.soTimeout = 200
                s.connect(InetSocketAddress(ip, sshPort), 200)
                val rtt = ((System.nanoTime() - t0) / 1_000_000L).toInt().coerceAtLeast(1)
                val buf = ByteArray(96)
                val n = try {
                    s.getInputStream().read(buf)
                } catch (_: Throwable) {
                    0
                }
                if (n <= 0) return null
                val banner = String(buf, 0, n, StandardCharsets.US_ASCII).trim()
                if (!banner.startsWith("SSH-", ignoreCase = true)) return null
                val lower = banner.lowercase(Locale.US)
                // 排除普通家用路由器常见的 dropbear，除非该 IP 本身是 10.42.0.1 / 192.168.4.1 机载热点
                if (lower.contains("dropbear") && isRouterGateway) return null
                if (isRouterGateway && !lower.contains("ubuntu") && !lower.contains("debian")) return null

                val distroTag = when {
                    lower.contains("ubuntu") -> "Ubuntu"
                    lower.contains("debian") || lower.contains("raspbian") -> "Debian"
                    lower.contains("openssh") -> "Linux"
                    else -> return null
                }
                val cfg = baseCfg.copy(
                    protocol = "tcpros",
                    masterUri = "http://$ip:11311",
                    wsUrl = "ws://$ip:9090"
                )
                DiscoveredDrone(
                    ip = ip,
                    port = sshPort,
                    protocol = "tcpros",
                    rttMs = rtt,
                    stackSummary = "机载 $distroTag 在线 · 待启动 roscore",
                    topicCount = 0,
                    suggestedConfig = cfg,
                    rosReady = false
                )
            }
        } catch (_: Throwable) {
            null
        }
    }

    /**
     * 发送轻量级标准 RFC 6762 mDNS 查询报文至 `224.0.0.251:5353`，秒级发现运行 `avahi-daemon` 的 Ubuntu / Jetson 机载电脑，
     * 即使在大掩码网段 (`/16`、`/22`) 下也能直接获取其真实 IPv4。
     */
    private fun discoverMdnsPeers(localIps: Set<String>, timeoutMs: Int): Set<String> {
        val foundIps = LinkedHashSet<String>()
        val mdnsAddr = try {
            InetAddress.getByName("224.0.0.251")
        } catch (_: Throwable) {
            return foundIps
        }

        val queryGroups = listOf(
            listOf(
                "_ros._tcp.local" to 12, // PTR
                "_rosbridge._tcp.local" to 12,
                "_workstation._tcp.local" to 12,
                "_ssh._tcp.local" to 12
            ),
            listOf(
                "ubuntu.local" to 1, // A
                "jetson.local" to 1,
                "orin.local" to 1,
                "nx.local" to 1,
                "nuc.local" to 1,
                "raspberrypi.local" to 1,
                "orangepi.local" to 1,
                "ego.local" to 1,
                "drone0.local" to 1,
                "uav.local" to 1
            )
        )

        var sock: DatagramSocket? = null
        try {
            val ds = DatagramSocket(null)
            sock = ds
            ds.reuseAddress = true
            ds.bind(InetSocketAddress(0))
            LanNetworkBinder.bindDatagramSocketToLan(ds, "224.0.0.251")
            ds.soTimeout = 90

            for (group in queryGroups) {
                val payload = buildDnsQueryPacket(group)
                val pkt = DatagramPacket(payload, payload.size, mdnsAddr, 5353)
                try {
                    ds.send(pkt)
                } catch (_: Throwable) {
                }
            }

            val rxBuf = ByteArray(1500)
            val rxPkt = DatagramPacket(rxBuf, rxBuf.size)
            val deadlineNs = System.nanoTime() + timeoutMs.toLong() * 1_000_000L
            while (System.nanoTime() < deadlineNs) {
                try {
                    rxPkt.length = rxBuf.size
                    ds.receive(rxPkt)
                    val sender = rxPkt.address
                    if (sender is Inet4Address && !sender.isLoopbackAddress) {
                        val sIp = sender.hostAddress
                        if (!sIp.isNullOrBlank() && sIp !in localIps && LanNetworkBinder.isUsableLanIpv4(sIp)) {
                            foundIps.add(sIp)
                        }
                    }
                    for (aIp in extractDnsARecordIpv4s(rxBuf, rxPkt.length)) {
                        if (aIp !in localIps && LanNetworkBinder.isUsableLanIpv4(aIp)) {
                            foundIps.add(aIp)
                        }
                    }
                } catch (_: Throwable) {
                    if (System.nanoTime() >= deadlineNs) break
                }
            }
        } catch (_: Throwable) {
        } finally {
            try {
                sock?.close()
            } catch (_: Throwable) {
            }
        }
        return foundIps
    }

    internal fun buildDnsQueryPacket(questions: List<Pair<String, Int>>): ByteArray {
        val out = ByteArrayOutputStream(256)
        // Header: Transaction ID = 0x0000, Flags = 0x0000 (Standard Query), QDCOUNT = questions.size
        out.write(0)
        out.write(0)
        out.write(0)
        out.write(0)
        out.write((questions.size ushr 8) and 0xFF)
        out.write(questions.size and 0xFF)
        // ANCOUNT, NSCOUNT, ARCOUNT = 0
        for (i in 0 until 6) out.write(0)

        for ((name, qType) in questions) {
            for (label in name.trim('.').split('.')) {
                val bytes = label.toByteArray(StandardCharsets.UTF_8)
                if (bytes.isEmpty() || bytes.size > 63) continue
                out.write(bytes.size)
                out.write(bytes)
            }
            out.write(0) // root label
            out.write((qType ushr 8) and 0xFF)
            out.write(qType and 0xFF)
            // QCLASS: QU (unicast-response preferred = 0x8001) so peer replies directly to our ephemeral UDP port!
            out.write(0x80)
            out.write(0x01)
        }
        return out.toByteArray()
    }

    internal fun extractDnsARecordIpv4s(buf: ByteArray, length: Int): List<String> {
        val results = ArrayList<String>(4)
        if (length < 12) return results
        val qdCount = ((buf[4].toInt() and 0xFF) shl 8) or (buf[5].toInt() and 0xFF)
        val anCount = ((buf[6].toInt() and 0xFF) shl 8) or (buf[7].toInt() and 0xFF)
        val nsCount = ((buf[8].toInt() and 0xFF) shl 8) or (buf[9].toInt() and 0xFF)
        val arCount = ((buf[10].toInt() and 0xFF) shl 8) or (buf[11].toInt() and 0xFF)
        val totalRr = anCount + nsCount + arCount
        if (totalRr <= 0 || totalRr > 64) return results

        var pos = 12
        for (q in 0 until qdCount) {
            pos = skipDnsName(buf, length, pos)
            if (pos < 0 || pos + 4 > length) return results
            pos += 4
        }

        for (rr in 0 until totalRr) {
            pos = skipDnsName(buf, length, pos)
            if (pos < 0 || pos + 10 > length) break
            val rrType = ((buf[pos].toInt() and 0xFF) shl 8) or (buf[pos + 1].toInt() and 0xFF)
            val rdLen = ((buf[pos + 8].toInt() and 0xFF) shl 8) or (buf[pos + 9].toInt() and 0xFF)
            pos += 10
            if (rdLen < 0 || pos + rdLen > length) break
            if (rrType == 1 && rdLen == 4) {
                val b0 = buf[pos].toInt() and 0xFF
                val b1 = buf[pos + 1].toInt() and 0xFF
                val b2 = buf[pos + 2].toInt() and 0xFF
                val b3 = buf[pos + 3].toInt() and 0xFF
                results.add("$b0.$b1.$b2.$b3")
            }
            pos += rdLen
        }
        return results
    }

    private fun skipDnsName(buf: ByteArray, length: Int, startPos: Int): Int {
        var p = startPos
        var safety = 0
        while (p in 0 until length && safety++ < 64) {
            val len = buf[p].toInt() and 0xFF
            when {
                len == 0 -> return p + 1
                (len and 0xC0) == 0xC0 -> return if (p + 1 < length) p + 2 else -1
                len <= 63 -> p += 1 + len
                else -> return -1
            }
        }
        return -1
    }

    private fun resolveHostnameIfNeeded(hostOrIp: String, allowLoopback: Boolean = false): String? {
        if (LanNetworkBinder.ipv4ToUnsignedInt(hostOrIp) != null) return hostOrIp
        return try {
            val addrs = InetAddress.getAllByName(hostOrIp)
            addrs.firstOrNull { it is Inet4Address && (allowLoopback || !it.isLoopbackAddress) }?.hostAddress
        } catch (_: Throwable) {
            null
        }
    }

    internal fun extractEndpointHost(raw: String?): String? {
        var s = raw?.trim().orEmpty()
        if (s.isEmpty()) return null
        if (s.contains("://")) {
            s = s.substringAfter("://")
        }
        s = s.substringBefore('/').trim()
        if (s.endsWith(".0")) {
            // 兼容用户输入形如 192.168.50.0/24 的网段提示，转成 .1 以便触发该网段扩展
            val prefix = s.removeSuffix(".0")
            if (prefix.count { it == '.' } == 2) return "$prefix.1"
        }
        val host = s.substringBefore(':').trim()
        if (host.isEmpty()) return null
        return host
    }

    internal fun extractEndpointPort(raw: String?): Int? {
        var s = raw?.trim().orEmpty()
        if (s.isEmpty()) return null
        if (s.contains("://")) {
            s = s.substringAfter("://")
        }
        s = s.substringBefore('/')
        val colon = s.lastIndexOf(':')
        if (colon <= 0) return null
        val port = s.substring(colon + 1).trim().toIntOrNull() ?: return null
        return port.takeIf { it in 1..65535 }
    }
}
