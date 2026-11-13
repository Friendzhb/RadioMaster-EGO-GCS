package com.radiomaster.egogcs.ros

import com.radiomaster.egogcs.model.TopicConfig
import java.net.HttpURLConnection
import java.net.Inet4Address
import java.net.InetSocketAddress
import java.net.NetworkInterface
import java.net.Socket
import java.net.URI
import java.net.URL
import java.util.Collections
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

/**
 * 发现的局域网机载 ROS1 无人机条目：
 * 包含 IP、开放协议端口、RTT 往返延迟、自动指纹识别出的机载算法栈名称与自动配对好的 TopicConfig。
 */
data class DiscoveredDrone(
    val ip: String,
    val port: Int,
    val protocol: String, // "tcpros" (11311) 或 "rosbridge" (9090)
    val rttMs: Int,
    val stackSummary: String,
    val topicCount: Int,
    val suggestedConfig: TopicConfig
)

/**
 * 局域网机载电脑高速并发自动发现与 ROS1 话题指纹识别器：
 * 1. 自动枚举遥控器当前活动的 Wi-Fi / USB-RNDIS / 以太网网段 (如 192.168.1.*, 192.168.31.*, 10.42.0.* Ubuntu 热点等)；
 * 2. 优先探测历史保存 IP 与机载常见固定地址 (.1, .10, .100, .101, 10.42.0.1)，随后 40 线程并发扫完整个 /24 网段 (约 1.1 秒)；
 * 3. 对开放 11311 端口的主机自动发起轻量级 XML-RPC getSystemState 探测，不仅验证真伪 roscore，
 *    还能自动识别机载运行的是 FAST-LIO2、VINS-Fusion、单机 EGO 还是 EGO-Swarm (自动提取 drone_X 编号) 并生成最优话题映射；
 * 4. 同步探测 9090 端口 (rosbridge_server) 作为备用协议候选。
 */
class LanDroneScanner {

    private val scanning = AtomicBoolean(false)
    private var scanMasterThread: Thread? = null

    val isScanning: Boolean
        get() = scanning.get()

    fun cancelScan() {
        scanning.set(false)
        scanMasterThread?.interrupt()
    }

    fun startScan(
        baseConfig: TopicConfig,
        onProgress: (percent: Int, hint: String) -> Unit,
        onDroneFound: (DiscoveredDrone) -> Unit,
        onComplete: (List<DiscoveredDrone>, String) -> Unit
    ) {
        if (!scanning.compareAndSet(false, true)) return

        scanMasterThread = Thread({
            val foundList = Collections.synchronizedList(mutableListOf<DiscoveredDrone>())
            var subnetSummary = "局域网"
            try {
                val localIps = mutableSetOf<String>()
                val prefixes = LinkedHashSet<String>()

                // 1. 提取当前活动的非回环 IPv4 网段
                try {
                    val nifs = NetworkInterface.getNetworkInterfaces()
                    while (nifs != null && nifs.hasMoreElements()) {
                        val nif = nifs.nextElement()
                        if (!nif.isUp || nif.isLoopback) continue
                        val addrs = nif.inetAddresses
                        while (addrs.hasMoreElements()) {
                            val a = addrs.nextElement()
                            if (a is Inet4Address && !a.isLoopbackAddress) {
                                val ip = a.hostAddress ?: continue
                                localIps.add(ip)
                                val dot = ip.lastIndexOf('.')
                                if (dot > 0) {
                                    prefixes.add(ip.substring(0, dot + 1))
                                }
                            }
                        }
                    }
                } catch (_: Throwable) {}

                // 提取上次保存的主机 IP 作为最高优先级探测目标
                val savedHost = try {
                    URI(baseConfig.masterUri.trim()).host
                } catch (_: Throwable) {
                    null
                }

                if (!savedHost.isNullOrBlank() && savedHost.count { it == '.' } == 3) {
                    val dot = savedHost.lastIndexOf('.')
                    if (dot > 0 && prefixes.isEmpty()) {
                        prefixes.add(savedHost.substring(0, dot + 1))
                    }
                }

                // 若当前无普通局域网 IP，补充机载 Ubuntu Wi-Fi 热点常用默认网段 10.42.0.* 与 192.168.4.*
                if (prefixes.isEmpty()) {
                    prefixes.add("10.42.0.")
                    prefixes.add("192.168.1.")
                }

                val primaryPrefix = prefixes.first()
                subnetSummary = "${primaryPrefix}0/24"

                // 2. 构建去重候选探测队列：优先高频机载静态 IP，再铺开全网段 1..254
                val candidates = LinkedHashSet<String>()
                if (!savedHost.isNullOrBlank() && savedHost != "127.0.0.1" && savedHost !in localIps) {
                    candidates.add(savedHost)
                }
                // Ubuntu NetworkManager 机载热点固定网关
                if ("10.42.0.1" !in localIps) candidates.add("10.42.0.1")

                val hotSuffixes = intArrayOf(1, 2, 10, 20, 50, 88, 100, 101, 102, 105, 110, 120, 150, 200)
                for (pfx in prefixes.take(2)) {
                    for (s in hotSuffixes) {
                        val ip = "$pfx$s"
                        if (ip !in localIps) candidates.add(ip)
                    }
                }
                for (pfx in prefixes.take(2)) {
                    for (hostId in 1..254) {
                        val ip = "$pfx$hostId"
                        if (ip !in localIps) candidates.add(ip)
                    }
                }

                val targetList = candidates.toList()
                val total = targetList.size.coerceAtLeast(1)
                val finishedCount = AtomicInteger(0)

                onProgress(2, "正在扫描 $subnetSummary (${targetList.size} 个节点)...")

                val pool = Executors.newFixedThreadPool(40) { r ->
                    Thread(r, "LanRosProbe").apply { isDaemon = true }
                }
                val latch = CountDownLatch(targetList.size)

                for (ip in targetList) {
                    pool.execute {
                        try {
                            if (!scanning.get()) return@execute
                            val hit = probeSingleHost(ip, baseConfig)
                            if (hit != null && scanning.get()) {
                                foundList.add(hit)
                                onDroneFound(hit)
                            }
                        } catch (_: Throwable) {
                        } finally {
                            val done = finishedCount.incrementAndGet()
                            if (done % 18 == 0 || done == total) {
                                val pct = ((done * 100) / total).coerceIn(5, 99)
                                onProgress(pct, "正在扫描 $subnetSummary ($pct%)")
                            }
                            latch.countDown()
                        }
                    }
                }

                latch.await(6, TimeUnit.SECONDS)
                pool.shutdownNow()
            } catch (_: Throwable) {
            } finally {
                val wasActive = scanning.getAndSet(false)
                if (wasActive) {
                    val sorted = foundList.sortedWith(
                        compareBy<DiscoveredDrone>({ if (it.protocol == "tcpros") 0 else 1 }, { it.rttMs })
                    )
                    onComplete(sorted, subnetSummary)
                }
            }
        }, "LanDroneScanMaster").apply {
            isDaemon = true
            start()
        }
    }

    private fun probeSingleHost(ip: String, baseCfg: TopicConfig): DiscoveredDrone? {
        // 优先探测原生 ROS1 Master (11311)
        val rtt11311 = tcpPingPort(ip, 11311, timeoutMs = 220)
        if (rtt11311 >= 0) {
            val inspected = inspectRosMaster(ip, rtt11311, baseCfg)
            if (inspected != null) return inspected
        }

        // 其次探测 rosbridge WebSocket (9090)
        val rtt9090 = tcpPingPort(ip, 9090, timeoutMs = 190)
        if (rtt9090 >= 0) {
            val cfg = baseCfg.copy(
                protocol = "rosbridge",
                masterUri = "http://$ip:11311",
                wsUrl = "ws://$ip:9090"
            )
            return DiscoveredDrone(
                ip = ip,
                port = 9090,
                protocol = "rosbridge",
                rttMs = rtt9090,
                stackSummary = "rosbridge WebSocket 在线",
                topicCount = 0,
                suggestedConfig = cfg
            )
        }
        return null
    }

    private fun tcpPingPort(ip: String, port: Int, timeoutMs: Int): Int {
        val t0 = System.nanoTime()
        return try {
            Socket().use { s ->
                s.tcpNoDelay = true
                s.connect(InetSocketAddress(ip, port), timeoutMs)
            }
            ((System.nanoTime() - t0) / 1_000_000L).toInt().coerceAtLeast(1)
        } catch (_: Throwable) {
            -1
        }
    }

    private fun inspectRosMaster(ip: String, rttMs: Int, baseCfg: TopicConfig): DiscoveredDrone? {
        val masterUri = "http://$ip:11311"
        val reqXml = "<?xml version=\"1.0\"?><methodCall><methodName>getSystemState</methodName>" +
            "<params><param><value><string>/ego_lan_scanner</string></value></param></params></methodCall>"

        val conn = try {
            (URL(masterUri).openConnection() as HttpURLConnection).apply {
                requestMethod = "POST"
                connectTimeout = 420
                readTimeout = 520
                doOutput = true
                setRequestProperty("Content-Type", "text/xml")
            }
        } catch (_: Throwable) {
            return null
        }

        val respXml = try {
            conn.outputStream.use { it.write(reqXml.toByteArray(Charsets.UTF_8)) }
            conn.inputStream.bufferedReader(Charsets.UTF_8).use { it.readText() }
        } catch (_: Throwable) {
            return null
        } finally {
            conn.disconnect()
        }

        if (!respXml.contains("<methodResponse>")) return null

        val strRegex = Regex("<string>\\s*(/[^<\\s]+)\\s*</string>")
        val rosNames = strRegex.findAll(respXml).map { it.groupValues[1] }.toSet()

        val cfg = baseCfg.copy(
            protocol = "tcpros",
            masterUri = masterUri,
            wsUrl = "ws://$ip:9090"
        )

        val tags = mutableListOf<String>()

        // 1. 识别 FAST-LIO / FAST-LIO2 话题
        val hasFastLioCloud = "/cloud_registered" in rosNames || "/cloud_registered_body" in rosNames
        val hasFastLioOdom = "/Odometry" in rosNames
        if (hasFastLioCloud || hasFastLioOdom) {
            tags.add("FAST-LIO2")
            if (hasFastLioOdom) cfg.odomTopic = "/Odometry"
            if ("/cloud_registered" in rosNames) {
                cfg.fastLioCloudTopic = "/cloud_registered"
            } else if ("/cloud_registered_body" in rosNames) {
                cfg.fastLioCloudTopic = "/cloud_registered_body"
            }
            if ("/path" in rosNames) cfg.fastLioPathTopic = "/path"
        } else if ("/vins_estimator/imu_propagate" in rosNames || "/vins_estimator/odometry" in rosNames) {
            tags.add("VINS-Fusion")
            cfg.odomTopic = if ("/vins_estimator/imu_propagate" in rosNames) {
                "/vins_estimator/imu_propagate"
            } else {
                "/vins_estimator/odometry"
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

        val tiltMatch = rosNames.firstOrNull {
            val l = it.lowercase()
            l.contains("tilt") || l.contains("servo_angles") || l.contains("indi") || l.contains("active_pitch")
        }
        if (tiltMatch != null) {
            tags.add("Tilt-INDI")
            if (!tiltMatch.contains("cmd")) {
                cfg.tiltTelemetryTopic = tiltMatch
            }
            val cmdMatch = rosNames.firstOrNull { it.lowercase().contains("pitch_tilt_cmd") || it.lowercase().contains("active_pitch") }
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
            port = 11311,
            protocol = "tcpros",
            rttMs = rttMs,
            stackSummary = summary,
            topicCount = rosNames.size,
            suggestedConfig = cfg
        )
    }
}
