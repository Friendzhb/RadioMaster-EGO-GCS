package com.radiomaster.egogcs.ros

import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.RenderSnapshot
import java.io.BufferedInputStream
import java.io.BufferedOutputStream
import java.io.DataInputStream
import java.io.EOFException
import java.net.HttpURLConnection
import java.net.Inet4Address
import java.net.InetSocketAddress
import java.net.NetworkInterface
import java.net.Socket
import java.net.SocketTimeoutException
import java.net.URL
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt
import org.json.JSONArray
import org.json.JSONObject

enum class TopicEchoState {
    CONNECTING,
    STREAMING,
    CAPTURED,
    PAUSED,
    WARNING
}

data class RosTopicEntry(
    val name: String,
    val type: String,
    val publisherCount: Int = 1
)

data class TopicEchoSnapshot(
    val topic: String,
    val msgType: String,
    val publisherNode: String,
    val state: TopicEchoState,
    val statusText: String,
    val isStreaming: Boolean,
    val receivedCount: Long,
    val hz: Float,
    val kbps: Float,
    val lastFrameBytes: Int,
    val timestampText: String,
    val bodyText: String
)

/**
 * 原生 ROS1 话题列表探测与单次截取 / 持续实时读取监控器 (`RosTopicInspector`)：
 *
 * 针对实机 C++ `roscpp` (`xmlrpcpp`) 与 Python `rospy` 混合节点的全兼容实现：
 * 1. 兼容 C++ `roscpp` `XmlRpcValue::stringToXml()` 省略 `<string>` 标签的 `<value>TCPROS</value>` 响应格式；
 * 2. 自动将机载未配置 `ROS_IP` 时返回的计算机名（含下划线主机名、`*.local` mDNS 域名、`localhost`）重写为可达的 `roscore` IP；
 * 3. 优先通过只读 `getSystemState` + `lookupNode` 定位所有活跃 Publisher 并逐一尝试连接（自动跳过僵尸节点，零脏订阅残留）；
 * 4. 支持 TCPROS `md5sum=*` 握手与严格节点 MD5 自动提取重连，内置常用飞控/SLAM 二进制消息直接解析及通用 `message_definition` 递归解码。
 */
class RosTopicInspector(
    private val hub: GcsSharedHub,
    private val rosManager: RosConnectionManager,
    private val tcpClient: TcpRosNativeClient,
    private val wsClient: RosBridgeClient
) {

    private val readSnapshot = RenderSnapshot()

    companion object {
        private const val INSPECTOR_CALLER = "/ego_topic_inspector"
        private val IPV4_REGEX = Regex("^\\d{1,3}(?:\\.\\d{1,3}){3}$")
        private val HTTP_HOST_PORT_REGEX = Regex("^http://([^/:\\s]+):(\\d+)(/.*)?$", RegexOption.IGNORE_CASE)
    }

    private data class MsgFieldSpec(
        val baseType: String,
        val name: String,
        val isArray: Boolean,
        val fixedArrayLen: Int
    )

    private data class PublisherCandidate(
        val nodeName: String,
        val xmlRpcUri: String
    )

    private val sessionGen = AtomicInteger(0)

    @Volatile private var activeSocket: Socket? = null

    @Volatile private var demoStreamTask: ScheduledFuture<*>? = null

    @Volatile private var rosApiTimeoutTask: ScheduledFuture<*>? = null

    private val workerPool = Executors.newCachedThreadPool { r ->
        Thread(r, "RosTopicInspectorIO").apply { isDaemon = true }
    }
    private val demoScheduler = Executors.newSingleThreadScheduledExecutor { r ->
        Thread(r, "RosDemoTopicEcho").apply { isDaemon = true }
    }

    fun fetchTopicsAsync(onResult: (topics: List<RosTopicEntry>, sourceHint: String) -> Unit) {
        if (!hub.isRosMode) {
            val demoList = buildDemoTopicList()
            onResult(demoList, "仿真演练环境 · ${demoList.size} 个活跃 ROS1 话题")
            return
        }

        val cfg = rosManager.topics.copy()
        if (cfg.protocol == "tcpros") {
            workerPool.execute {
                try {
                    val masterUri = normalizeHttpUri(cfg.masterUri, defaultPort = 11311)
                    val masterHost = extractHostFromUri(masterUri) ?: masterUri
                    val sorted = queryTopicsFromRosMaster(masterUri)
                    onResult(sorted, "roscore ($masterHost) · 共 ${sorted.size} 个话题")
                } catch (e: Exception) {
                    onResult(emptyList(), "读取 roscore 话题列表失败: ${e.localizedMessage ?: "请检查机载 IP 与 11311 端口"}")
                }
            }
        } else {
            rosApiTimeoutTask?.cancel(false)
            val answered = AtomicBoolean(false)

            wsClient.onRosApiTopicsResult = { list ->
                if (answered.compareAndSet(false, true)) {
                    rosApiTimeoutTask?.cancel(false)
                    if (list.isNotEmpty()) {
                        val sorted = list.sortedWith(
                            compareBy<RosTopicEntry> { it.name.startsWith("/rosout") }.thenBy { it.name }
                        )
                        onResult(sorted, "rosbridge WebSocket · 共 ${sorted.size} 个话题")
                    } else {
                        // rosapi 节点未启动时自动尝试同机 11311 roscore 查询，再回退预设话题
                        workerPool.execute {
                            val wsHost = extractHostFromWsUrl(cfg.wsUrl)
                            val fallbackMaster = if (!wsHost.isNullOrBlank()) "http://$wsHost:11311" else normalizeHttpUri(cfg.masterUri, 11311)
                            val viaMaster = runCatching { queryTopicsFromRosMaster(fallbackMaster) }.getOrNull()
                            if (!viaMaster.isNullOrEmpty()) {
                                onResult(viaMaster, "rosbridge + roscore · 共 ${viaMaster.size} 个话题")
                            } else {
                                val preset = buildConfiguredFallbackTopics(cfg)
                                onResult(preset, "机载未启动 rosapi · 已展示常用飞控话题 (${preset.size})")
                            }
                        }
                    }
                }
            }

            val sent = wsClient.requestTopicsViaRosApi()
            if (!sent) {
                answered.set(true)
                val fallback = buildConfiguredFallbackTopics(cfg)
                onResult(fallback, "rosbridge 未连接 · 当前展示预设话题 (${fallback.size})")
            } else {
                rosApiTimeoutTask = demoScheduler.schedule({
                    if (answered.compareAndSet(false, true)) {
                        wsClient.onRosApiTopicsResult = null
                        workerPool.execute {
                            val wsHost = extractHostFromWsUrl(cfg.wsUrl)
                            val fallbackMaster = if (!wsHost.isNullOrBlank()) "http://$wsHost:11311" else normalizeHttpUri(cfg.masterUri, 11311)
                            val viaMaster = runCatching { queryTopicsFromRosMaster(fallbackMaster) }.getOrNull()
                            if (!viaMaster.isNullOrEmpty()) {
                                onResult(viaMaster, "roscore ($wsHost) · 共 ${viaMaster.size} 个话题")
                            } else {
                                val preset = buildConfiguredFallbackTopics(cfg)
                                onResult(preset, "rosapi 响应超时 · 已展示常用飞控话题 (${preset.size})")
                            }
                        }
                    }
                }, 2200L, TimeUnit.MILLISECONDS)
            }
        }
    }

    private fun queryTopicsFromRosMaster(masterUri: String): List<RosTopicEntry> {
        val typesXml = xmlRpcCall(masterUri, "getTopicTypes", listOf(INSPECTOR_CALLER))
        val stateXml = try {
            xmlRpcCall(masterUri, "getSystemState", listOf(INSPECTOR_CALLER))
        } catch (_: Throwable) {
            ""
        }

        val pubNodesByTopic = parsePublisherNodesFromSystemState(stateXml)
        val pairs = parseTopicTypePairs(typesXml)

        val merged = LinkedHashMap<String, RosTopicEntry>()
        for ((tName, tType) in pairs) {
            val cnt = pubNodesByTopic[tName]?.size ?: 0
            merged[tName] = RosTopicEntry(tName, tType, cnt)
        }
        for ((tName, nodes) in pubNodesByTopic) {
            if (!merged.containsKey(tName)) {
                merged[tName] = RosTopicEntry(tName, "unknown", nodes.size)
            }
        }

        return merged.values.sortedWith(
            compareByDescending<RosTopicEntry> { it.publisherCount > 0 }
                .thenBy { it.name.startsWith("/rosout") }
                .thenBy { it.name }
        )
    }

    private fun buildConfiguredFallbackTopics(cfg: com.radiomaster.egogcs.model.TopicConfig): List<RosTopicEntry> {
        return listOf(
            RosTopicEntry(cfg.odomTopic, "nav_msgs/Odometry", 1),
            RosTopicEntry(cfg.fastLioCloudTopic, "sensor_msgs/PointCloud2", 1),
            RosTopicEntry(cfg.inflateTopic, "sensor_msgs/PointCloud2", 1),
            RosTopicEntry(cfg.optimalTopic, "visualization_msgs/Marker", 1),
            RosTopicEntry(cfg.fastLioPathTopic, "nav_msgs/Path", 1),
            RosTopicEntry("/mavros/state", "mavros_msgs/State", 1),
            RosTopicEntry("/mavros/battery", "sensor_msgs/BatteryState", 1),
            RosTopicEntry(cfg.tiltTelemetryTopic.ifBlank { "/tilt_uav/servo_angles" }, "std_msgs/Float32MultiArray", 1),
            RosTopicEntry(cfg.tiltCmdTopic.ifBlank { "/active_pitch_planner/pitch_tilt_cmd" }, "geometry_msgs/Vector3Stamped", 1),
            RosTopicEntry("/mavros/actuator_control", "mavros_msgs/ActuatorControl", 1),
            RosTopicEntry("/mavros/imu/data", "sensor_msgs/Imu", 1),
            RosTopicEntry(cfg.goalTopic, "geometry_msgs/PoseStamped", 1)
        ).filter { it.name.isNotBlank() }.distinctBy { it.name }
    }

    fun captureOnce(topic: String, knownType: String, onUpdate: (TopicEchoSnapshot) -> Unit) {
        startSessionInternal(topic, knownType, singleShot = true, onUpdate = onUpdate)
    }

    fun startContinuous(topic: String, knownType: String, onUpdate: (TopicEchoSnapshot) -> Unit) {
        startSessionInternal(topic, knownType, singleShot = false, onUpdate = onUpdate)
    }

    fun stopActiveSession() {
        sessionGen.incrementAndGet()
        demoStreamTask?.cancel(true)
        demoStreamTask = null
        tcpClient.onWatchedRawFrame = null
        tcpClient.watchedRawTopic = null
        wsClient.onWatchedTopicJson = null
        wsClient.stopWatchingTopic()
        try {
            activeSocket?.close()
        } catch (_: Throwable) {}
        activeSocket = null
    }

    fun shutdown() {
        stopActiveSession()
        rosApiTimeoutTask?.cancel(true)
        demoScheduler.shutdownNow()
        workerPool.shutdownNow()
    }

    private fun startSessionInternal(
        topic: String,
        knownType: String,
        singleShot: Boolean,
        onUpdate: (TopicEchoSnapshot) -> Unit
    ) {
        stopActiveSession()
        val mySid = sessionGen.incrementAndGet()

        onUpdate(
            TopicEchoSnapshot(
                topic = topic,
                msgType = knownType.ifBlank { "探测中..." },
                publisherNode = "--",
                state = TopicEchoState.CONNECTING,
                statusText = if (singleShot) "正在连接节点并截取首帧..." else "正在建立实时订阅数据流...",
                isStreaming = !singleShot,
                receivedCount = 0L,
                hz = 0f,
                kbps = 0f,
                lastFrameBytes = 0,
                timestampText = nowTimeStr(),
                bodyText = "# 正在握手话题 $topic ...\n# 若该话题为按需触发话题，将等待下一帧消息发布。"
            )
        )

        if (!hub.isRosMode) {
            runDemoInspectSession(mySid, topic, knownType, singleShot, onUpdate)
            return
        }

        val cfg = rosManager.topics.copy()
        if (cfg.protocol == "tcpros") {
            if (tcpClient.isTopicActivelyStreaming(topic)) {
                runSharedTcpStreamTap(mySid, cfg, topic, knownType, singleShot, onUpdate)
            } else {
                workerPool.execute {
                    runTcpRosInspectSession(
                        mySid,
                        normalizeHttpUri(cfg.masterUri, defaultPort = 11311),
                        topic,
                        knownType,
                        singleShot,
                        onUpdate
                    )
                }
            }
        } else {
            runRosBridgeInspectSession(mySid, cfg, topic, knownType, singleShot, onUpdate)
        }
    }

    /**
     * 对已建立核心数据流的话题直接旁路复用现有 TCPROS Socket，零额外带宽、毫秒级瞬时出帧；
     * 若 1.8 秒内旁路未收到帧，自动无感切回独立 TCPROS 握手通道。
     */
    private fun runSharedTcpStreamTap(
        sid: Int,
        cfg: com.radiomaster.egogcs.model.TopicConfig,
        topic: String,
        knownType: String,
        singleShot: Boolean,
        onUpdate: (TopicEchoSnapshot) -> Unit
    ) {
        val gotFirstFrame = AtomicBoolean(false)
        var totalFrames = 0L
        var winFrames = 0
        var winBytes = 0L
        var winStartMs = System.currentTimeMillis()
        var smoothHz = 0f
        var smoothKbps = 0f
        var lastUiPushMs = 0L
        var cachedDef = ""
        var cachedSchema: Map<String, List<MsgFieldSpec>> = emptyMap()

        tcpClient.onWatchedRawFrame = { rxTopic, msgType, pubCaller, msgDef, buf, frameLen ->
            if (sessionGen.get() == sid && rxTopic == topic) {
                gotFirstFrame.set(true)
                totalFrames++
                winFrames++
                winBytes += (frameLen + 4)
                val nowMs = System.currentTimeMillis()
                val dtSec = (nowMs - winStartMs) / 1000f
                if (dtSec >= 0.45f) {
                    val instHz = winFrames / dtSec
                    val instKbps = (winBytes / 1024f) / dtSec
                    smoothHz = if (smoothHz <= 0.01f) instHz else (smoothHz * 0.4f + instHz * 0.6f)
                    smoothKbps = if (smoothKbps <= 0.01f) instKbps else (smoothKbps * 0.4f + instKbps * 0.6f)
                    winFrames = 0
                    winBytes = 0L
                    winStartMs = nowMs
                }

                val effType = msgType.ifBlank { knownType.ifBlank { "unknown" } }
                if (msgDef != cachedDef || cachedSchema.isEmpty()) {
                    cachedDef = msgDef
                    cachedSchema = parseMessageDefinitionBundle(effType, msgDef)
                }

                if (singleShot) {
                    tcpClient.onWatchedRawFrame = null
                    tcpClient.watchedRawTopic = null
                    val frameCopy = buf.copyOf(frameLen)
                    val text = formatBinaryRosPayload(effType, frameCopy, frameLen, cachedSchema)
                    onUpdate(
                        TopicEchoSnapshot(
                            topic = topic,
                            msgType = effType,
                            publisherNode = pubCaller,
                            state = TopicEchoState.CAPTURED,
                            statusText = "单次截取完成 (1 帧 · $frameLen B)",
                            isStreaming = false,
                            receivedCount = 1L,
                            hz = smoothHz,
                            kbps = smoothKbps,
                            lastFrameBytes = frameLen,
                            timestampText = nowTimeStr(),
                            bodyText = text
                        )
                    )
                } else if (totalFrames == 1L || nowMs - lastUiPushMs >= 140L) {
                    lastUiPushMs = nowMs
                    val frameCopy = buf.copyOf(frameLen)
                    val text = formatBinaryRosPayload(effType, frameCopy, frameLen, cachedSchema)
                    onUpdate(
                        TopicEchoSnapshot(
                            topic = topic,
                            msgType = effType,
                            publisherNode = pubCaller,
                            state = TopicEchoState.STREAMING,
                            statusText = "实时读取中 ($pubCaller · 零带宽复用)",
                            isStreaming = true,
                            receivedCount = totalFrames,
                            hz = smoothHz,
                            kbps = smoothKbps,
                            lastFrameBytes = frameLen,
                            timestampText = nowTimeStr(),
                            bodyText = text
                        )
                    )
                }
            }
        }
        tcpClient.watchedRawTopic = topic

        demoScheduler.schedule({
            if (sessionGen.get() == sid && !gotFirstFrame.get()) {
                tcpClient.onWatchedRawFrame = null
                tcpClient.watchedRawTopic = null
                workerPool.execute {
                    runTcpRosInspectSession(
                        sid,
                        normalizeHttpUri(cfg.masterUri, defaultPort = 11311),
                        topic,
                        knownType,
                        singleShot,
                        onUpdate
                    )
                }
            }
        }, 1800L, TimeUnit.MILLISECONDS)
    }

    private fun runTcpRosInspectSession(
        sid: Int,
        masterUri: String,
        topic: String,
        knownType: String,
        singleShot: Boolean,
        onUpdate: (TopicEchoSnapshot) -> Unit
    ) {
        var registeredCallerApi: String? = null
        var sock: Socket? = null
        try {
            val masterHost = extractHostFromUri(masterUri)
                ?: throw IllegalArgumentException("Master URI 地址格式无效: $masterUri")

            // 1. 若消息类型未知，先从 roscore 查询精确类型，避免使用 type=* 触发严格节点拒绝
            var resolvedType = knownType.trim()
            if (resolvedType.isEmpty() || resolvedType == "unknown") {
                runCatching {
                    val typesXml = xmlRpcCall(masterUri, "getTopicTypes", listOf(INSPECTOR_CALLER))
                    val foundType = parseTopicTypePairs(typesXml).firstOrNull { it.first == topic }?.second
                    if (!foundType.isNullOrBlank()) resolvedType = foundType
                }
            }

            // 2. 优先通过只读 getSystemState + lookupNode 获取所有活跃 Publisher 的 XML-RPC 地址
            val candidates = LinkedHashMap<String, PublisherCandidate>()
            runCatching {
                val stateXml = xmlRpcCall(masterUri, "getSystemState", listOf(INSPECTOR_CALLER))
                val pubNodes = parsePublisherNodesFromSystemState(stateXml)[topic].orEmpty()
                for (nodeName in pubNodes) {
                    val lookupXml = runCatching {
                        xmlRpcCall(masterUri, "lookupNode", listOf(INSPECTOR_CALLER, nodeName))
                    }.getOrDefault("")
                    for (u in extractHttpUris(lookupXml)) {
                        candidates[u] = PublisherCandidate(nodeName, u)
                    }
                }
            }

            // 3. 补充回退：若 lookupNode 未拿到地址，则使用真实类型调用 registerSubscriber 获取发布者列表
            if (candidates.isEmpty()) {
                val localIp = detectLocalIp(masterHost)
                val callerApi = "http://$localIp:49151/"
                val regType = if (resolvedType.isNotBlank() && resolvedType != "unknown") resolvedType else "*"
                val regXml = xmlRpcCall(masterUri, "registerSubscriber", listOf(INSPECTOR_CALLER, topic, regType, callerApi))
                registeredCallerApi = callerApi
                for (u in extractHttpUris(regXml)) {
                    candidates[u] = PublisherCandidate("publisher", u)
                }
            }

            if (candidates.isEmpty()) {
                if (sessionGen.get() == sid) {
                    onUpdate(
                        TopicEchoSnapshot(
                            topic = topic,
                            msgType = resolvedType.ifBlank { "unknown" },
                            publisherNode = "无活跃节点",
                            state = TopicEchoState.WARNING,
                            statusText = "当前无活跃节点发布该话题",
                            isStreaming = false,
                            receivedCount = 0L,
                            hz = 0f,
                            kbps = 0f,
                            lastFrameBytes = 0,
                            timestampText = nowTimeStr(),
                            bodyText = "# roscore 中话题 $topic 当前有 0 个活跃 Publisher。\n# 例如目标话题 /move_base_simple/goal 仅在指点发送时短暂触发，或对应机载节点尚未启动。"
                        )
                    )
                }
                return
            }

            // 4. 遍历所有候选 Publisher，跳过已失效僵尸节点，兼容 roscpp 无 <string> 标签与非 IP 主机名重写
            var connectedSock: Socket? = null
            var connectedInp: DataInputStream? = null
            var actualType = resolvedType.ifBlank { "unknown" }
            var pubNode = "--"
            var connectedEndpointDesc = ""
            var schemaMap: Map<String, List<MsgFieldSpec>> = emptyMap()
            var scratch = ByteArray(65536)
            var lastHandshakeErr: String? = null

            for (cand in candidates.values) {
                if (sessionGen.get() != sid) return
                try {
                    val pubRpcTargets = buildCandidateHttpUris(cand.xmlRpcUri, masterHost)
                    var topicResp: String? = null
                    val reqBody = buildString {
                        append("<?xml version=\"1.0\"?><methodCall><methodName>requestTopic</methodName><params>")
                        append("<param><value><string>$INSPECTOR_CALLER</string></value></param>")
                        append("<param><value><string>${escapeXml(topic)}</string></value></param>")
                        append("<param><value><array><data><value><array><data><value><string>TCPROS</string></value></data></array></value></data></array></value></param>")
                        append("</params></methodCall>")
                    }
                    for (rpcUrl in pubRpcTargets) {
                        try {
                            val r = postXml(rpcUrl, reqBody)
                            if (r.contains("TCPROS", ignoreCase = true)) {
                                topicResp = r
                                break
                            } else {
                                lastHandshakeErr = "节点 ${cand.nodeName} 未启用 TCPROS"
                            }
                        } catch (rpcErr: Exception) {
                            lastHandshakeErr = "${cand.nodeName} RPC 不可达 (${rpcErr.localizedMessage ?: "连接失败"})"
                        }
                    }
                    if (topicResp == null) continue

                    val epPair = parseRequestTopicEndpoint(topicResp, masterHost)
                    if (epPair == null) {
                        lastHandshakeErr = "解析 ${cand.nodeName} TCPROS 端口失败"
                        continue
                    }

                    val hostCandidates = buildCandidateTcpHosts(epPair.first, masterHost)
                    val tcpPort = epPair.second

                    for (tcpHost in hostCandidates) {
                        var handshaked = openAndHandshakeTcpRos(
                            host = tcpHost,
                            port = tcpPort,
                            topic = topic,
                            reqType = if (resolvedType.isNotBlank() && resolvedType != "unknown") resolvedType else "*",
                            reqMd5 = "*",
                            singleShot = singleShot,
                            scratchInit = scratch
                        )
                        // 若严格节点拒绝 md5sum=* 但在响应头或错误信息中暴露了真实 32 位 MD5，立即用真实 MD5 自动重连
                        val retryMd5 = handshaked.discoveredMd5
                        if (handshaked.errorText != null && retryMd5 != null) {
                            handshaked = openAndHandshakeTcpRos(
                                host = tcpHost,
                                port = tcpPort,
                                topic = topic,
                                reqType = handshaked.discoveredType ?: resolvedType.ifBlank { "*" },
                                reqMd5 = retryMd5,
                                singleShot = singleShot,
                                scratchInit = handshaked.scratch
                            )
                        }
                        scratch = handshaked.scratch
                        if (handshaked.sock != null && handshaked.inp != null) {
                            connectedSock = handshaked.sock
                            connectedInp = handshaked.inp
                            actualType = handshaked.discoveredType?.takeIf { it.isNotBlank() } ?: actualType
                            pubNode = handshaked.publisherCallerId?.takeIf { it.isNotBlank() } ?: cand.nodeName
                            schemaMap = parseMessageDefinitionBundle(actualType, handshaked.messageDefinition.orEmpty())
                            connectedEndpointDesc = "$tcpHost:$tcpPort"
                            break
                        } else if (handshaked.errorText != null) {
                            lastHandshakeErr = handshaked.errorText
                        }
                    }
                    if (connectedSock != null) break
                } catch (candErr: Exception) {
                    lastHandshakeErr = candErr.localizedMessage ?: candErr.javaClass.simpleName
                }
            }

            sock = connectedSock
            activeSocket = sock
            val inp = connectedInp
            if (sock == null || inp == null) {
                throw IllegalStateException(
                    lastHandshakeErr ?: "无法连接话题发布节点的 TCPROS 数据端口（请检查机载防火墙或 ROS_IP 配置）"
                )
            }

            fun readNextFrame(maxAllowed: Int): Int {
                val b0 = inp.read()
                if (b0 < 0) return -1
                val b1 = inp.read()
                val b2 = inp.read()
                val b3 = inp.read()
                if ((b1 or b2 or b3) < 0) throw EOFException()
                val len = (b0 and 0xFF) or ((b1 and 0xFF) shl 8) or ((b2 and 0xFF) shl 16) or ((b3 and 0xFF) shl 24)
                if (len < 0 || len > maxAllowed) throw IllegalStateException("消息帧长度异常: $len")
                if (len > scratch.size) {
                    var cap = scratch.size
                    while (cap < len) cap = cap shl 1
                    scratch = ByteArray(cap)
                }
                inp.readFully(scratch, 0, len)
                return len
            }

            if (sessionGen.get() == sid && !singleShot) {
                onUpdate(
                    TopicEchoSnapshot(
                        topic = topic,
                        msgType = actualType,
                        publisherNode = pubNode,
                        state = TopicEchoState.CONNECTING,
                        statusText = "已直连 $pubNode · 等待首帧...",
                        isStreaming = true,
                        receivedCount = 0L,
                        hz = 0f,
                        kbps = 0f,
                        lastFrameBytes = 0,
                        timestampText = nowTimeStr(),
                        bodyText = "# 已建立 TCPROS 数据通道: $pubNode ($connectedEndpointDesc)\n# 消息类型: $actualType\n# 正在接收实时二进制消息帧..."
                    )
                )
            }

            var totalFrames = 0L
            var winFrames = 0
            var winBytes = 0L
            var winStartMs = System.currentTimeMillis()
            var smoothHz = 0f
            var smoothKbps = 0f
            var lastUiPushMs = 0L

            while (sessionGen.get() == sid && !sock.isClosed) {
                val frameLen = try {
                    readNextFrame(16 * 1024 * 1024)
                } catch (_: SocketTimeoutException) {
                    if (singleShot) {
                        onUpdate(
                            TopicEchoSnapshot(
                                topic = topic,
                                msgType = actualType,
                                publisherNode = pubNode,
                                state = TopicEchoState.WARNING,
                                statusText = "已连接节点，但 6s 内无新帧发布",
                                isStreaming = false,
                                receivedCount = 0L,
                                hz = 0f,
                                kbps = 0f,
                                lastFrameBytes = 0,
                                timestampText = nowTimeStr(),
                                bodyText = "# 已成功握手发布节点 $pubNode ($connectedEndpointDesc)，但该话题在 6 秒内未发出新消息。\n# 若这是按需触发或低频话题，请点击上方「持续读取」保持监听。"
                            )
                        )
                        break
                    } else {
                        smoothHz = 0f
                        smoothKbps = 0f
                        continue
                    }
                }
                if (frameLen < 0) break

                totalFrames++
                winFrames++
                winBytes += (frameLen + 4)
                val nowMs = System.currentTimeMillis()
                val dtSec = (nowMs - winStartMs) / 1000f
                if (dtSec >= 0.45f) {
                    val instHz = winFrames / dtSec
                    val instKbps = (winBytes / 1024f) / dtSec
                    smoothHz = if (smoothHz <= 0.01f) instHz else (smoothHz * 0.4f + instHz * 0.6f)
                    smoothKbps = if (smoothKbps <= 0.01f) instKbps else (smoothKbps * 0.4f + instKbps * 0.6f)
                    winFrames = 0
                    winBytes = 0L
                    winStartMs = nowMs
                }

                if (singleShot) {
                    val text = formatBinaryRosPayload(actualType, scratch, frameLen, schemaMap)
                    onUpdate(
                        TopicEchoSnapshot(
                            topic = topic,
                            msgType = actualType,
                            publisherNode = pubNode,
                            state = TopicEchoState.CAPTURED,
                            statusText = "单次截取完成 (1 帧 · $frameLen B)",
                            isStreaming = false,
                            receivedCount = 1L,
                            hz = smoothHz,
                            kbps = smoothKbps,
                            lastFrameBytes = frameLen,
                            timestampText = nowTimeStr(),
                            bodyText = text
                        )
                    )
                    break
                } else if (totalFrames == 1L || nowMs - lastUiPushMs >= 140L) {
                    lastUiPushMs = nowMs
                    val text = formatBinaryRosPayload(actualType, scratch, frameLen, schemaMap)
                    onUpdate(
                        TopicEchoSnapshot(
                            topic = topic,
                            msgType = actualType,
                            publisherNode = pubNode,
                            state = TopicEchoState.STREAMING,
                            statusText = "实时读取中 ($pubNode)",
                            isStreaming = true,
                            receivedCount = totalFrames,
                            hz = smoothHz,
                            kbps = smoothKbps,
                            lastFrameBytes = frameLen,
                            timestampText = nowTimeStr(),
                            bodyText = text
                        )
                    )
                }
            }
        } catch (e: Exception) {
            if (sessionGen.get() == sid) {
                onUpdate(
                    TopicEchoSnapshot(
                        topic = topic,
                        msgType = knownType.ifBlank { "unknown" },
                        publisherNode = "--",
                        state = TopicEchoState.WARNING,
                        statusText = "读取失败: ${e.localizedMessage ?: "连接中断"}",
                        isStreaming = false,
                        receivedCount = 0L,
                        hz = 0f,
                        kbps = 0f,
                        lastFrameBytes = 0,
                        timestampText = nowTimeStr(),
                        bodyText = "# 无法读取话题 $topic:\n# ${e.localizedMessage ?: e.javaClass.simpleName}\n# 提示: 若机载电脑设置了不可路由的 ROS_IP，请在机载 ~/.bashrc 中设置 export ROS_IP=<机载局域网IP>。"
                    )
                )
            }
        } finally {
            try {
                sock?.close()
            } catch (_: Throwable) {}
            if (activeSocket === sock) activeSocket = null
            if (registeredCallerApi != null) {
                try {
                    xmlRpcCall(masterUri, "unregisterSubscriber", listOf(INSPECTOR_CALLER, topic, registeredCallerApi))
                } catch (_: Throwable) {}
            }
        }
    }

    private class HandshakeOutcome(
        val sock: Socket?,
        val inp: DataInputStream?,
        val scratch: ByteArray,
        val discoveredType: String?,
        val discoveredMd5: String?,
        val publisherCallerId: String?,
        val messageDefinition: String?,
        val errorText: String?
    )

    private fun openAndHandshakeTcpRos(
        host: String,
        port: Int,
        topic: String,
        reqType: String,
        reqMd5: String,
        singleShot: Boolean,
        scratchInit: ByteArray
    ): HandshakeOutcome {
        var scratch = scratchInit
        val s = Socket()
        try {
            s.tcpNoDelay = true
            s.receiveBufferSize = 1024 * 1024
            s.soTimeout = if (singleShot) 6000 else 3500
            s.connect(InetSocketAddress(host, port), 2600)

            val out = BufferedOutputStream(s.getOutputStream())
            val inp = DataInputStream(BufferedInputStream(s.getInputStream(), 65536))

            val hdrFields = ArrayList<String>(6)
            hdrFields.add("callerid=$INSPECTOR_CALLER")
            hdrFields.add("topic=$topic")
            hdrFields.add("md5sum=$reqMd5")
            if (reqType.isNotBlank()) {
                hdrFields.add("type=$reqType")
            }
            hdrFields.add("tcp_nodelay=1")

            out.write(encodeTcpRosHeader(hdrFields))
            out.flush()

            val b0 = inp.read()
            if (b0 < 0) {
                s.close()
                return HandshakeOutcome(null, null, scratch, null, null, null, null, "TCPROS 握手被远端提前关闭")
            }
            val b1 = inp.read()
            val b2 = inp.read()
            val b3 = inp.read()
            if ((b1 or b2 or b3) < 0) {
                s.close()
                return HandshakeOutcome(null, null, scratch, null, null, null, null, "TCPROS 响应头不完整")
            }
            val hdrLen = (b0 and 0xFF) or ((b1 and 0xFF) shl 8) or ((b2 and 0xFF) shl 16) or ((b3 and 0xFF) shl 24)
            if (hdrLen <= 0 || hdrLen > 524_288) {
                s.close()
                return HandshakeOutcome(null, null, scratch, null, null, null, null, "TCPROS 响应头长度异常: $hdrLen")
            }
            if (hdrLen > scratch.size) {
                var cap = scratch.size
                while (cap < hdrLen) cap = cap shl 1
                scratch = ByteArray(cap)
            }
            inp.readFully(scratch, 0, hdrLen)

            val hdrMap = decodeTcpRosHeader(scratch, hdrLen)
            val err = hdrMap["error"]
            val replyMd5 = hdrMap["md5sum"]?.takeIf { it.length == 32 }
                ?: err?.let { Regex("\\b([0-9a-fA-F]{32})\\b").findAll(it).lastOrNull()?.groupValues?.get(1) }
            val replyType = hdrMap["type"]

            if (err != null) {
                s.close()
                return HandshakeOutcome(null, null, scratch, replyType, replyMd5, hdrMap["callerid"], hdrMap["message_definition"], "握手拒绝: $err")
            }

            return HandshakeOutcome(
                sock = s,
                inp = inp,
                scratch = scratch,
                discoveredType = replyType,
                discoveredMd5 = replyMd5,
                publisherCallerId = hdrMap["callerid"],
                messageDefinition = hdrMap["message_definition"],
                errorText = null
            )
        } catch (e: Exception) {
            try {
                s.close()
            } catch (_: Throwable) {}
            return HandshakeOutcome(
                null,
                null,
                scratch,
                null,
                null,
                null,
                null,
                "$host:$port 连接失败 (${e.localizedMessage ?: e.javaClass.simpleName})"
            )
        }
    }

    private fun runRosBridgeInspectSession(
        sid: Int,
        cfg: com.radiomaster.egogcs.model.TopicConfig,
        topic: String,
        knownType: String,
        singleShot: Boolean,
        onUpdate: (TopicEchoSnapshot) -> Unit
    ) {
        if (!hub.rosConnected) {
            // 若 rosbridge 未连但同机 11311 roscore 可达，自动通过原生 TCPROS 读取话题！
            val wsHost = extractHostFromWsUrl(cfg.wsUrl)
            val fallbackMaster = if (!wsHost.isNullOrBlank()) "http://$wsHost:11311" else normalizeHttpUri(cfg.masterUri, 11311)
            workerPool.execute {
                val masterAlive = runCatching {
                    val xml = xmlRpcCall(fallbackMaster, "getUri", listOf(INSPECTOR_CALLER))
                    xml.contains("<methodResponse>")
                }.getOrDefault(false)
                if (masterAlive) {
                    runTcpRosInspectSession(sid, fallbackMaster, topic, knownType, singleShot, onUpdate)
                } else if (sessionGen.get() == sid) {
                    onUpdate(
                        TopicEchoSnapshot(
                            topic = topic,
                            msgType = knownType,
                            publisherNode = "未连接机载节点",
                            state = TopicEchoState.WARNING,
                            statusText = "请先在「连接」面板连接机载 ROS1 或 rosbridge",
                            isStreaming = false,
                            receivedCount = 0L,
                            hz = 0f,
                            kbps = 0f,
                            lastFrameBytes = 0,
                            timestampText = nowTimeStr(),
                            bodyText = "# 当前尚未连接机载 rosbridge (${cfg.wsUrl}) 或 roscore ($fallbackMaster)。"
                        )
                    )
                }
            }
            return
        }

        var count = 0L
        var winFrames = 0
        var winBytes = 0L
        var winStartMs = System.currentTimeMillis()
        var hz = 0f
        var kbps = 0f
        var lastUiMs = 0L

        wsClient.onWatchedTopicJson = { rxTopic, msgJson, rawLen ->
            if (sessionGen.get() == sid && rxTopic == topic) {
                count++
                winFrames++
                winBytes += rawLen
                val now = System.currentTimeMillis()
                val dt = (now - winStartMs) / 1000f
                if (dt >= 0.45f) {
                    hz = winFrames / dt
                    kbps = (winBytes / 1024f) / dt
                    winFrames = 0
                    winBytes = 0L
                    winStartMs = now
                }

                if (singleShot) {
                    wsClient.onWatchedTopicJson = null
                    wsClient.stopWatchingTopic(topic)
                    val formatted = formatJsonAsYaml(msgJson, 0)
                    onUpdate(
                        TopicEchoSnapshot(
                            topic = topic,
                            msgType = knownType,
                            publisherNode = "rosbridge_websocket",
                            state = TopicEchoState.CAPTURED,
                            statusText = "单次截取完成 ($rawLen B)",
                            isStreaming = false,
                            receivedCount = 1L,
                            hz = hz,
                            kbps = kbps,
                            lastFrameBytes = rawLen,
                            timestampText = nowTimeStr(),
                            bodyText = formatted
                        )
                    )
                } else if (count == 1L || now - lastUiMs >= 140L) {
                    lastUiMs = now
                    val formatted = formatJsonAsYaml(msgJson, 0)
                    onUpdate(
                        TopicEchoSnapshot(
                            topic = topic,
                            msgType = knownType,
                            publisherNode = "rosbridge_websocket",
                            state = TopicEchoState.STREAMING,
                            statusText = "实时读取中 (rosbridge)",
                            isStreaming = true,
                            receivedCount = count,
                            hz = hz,
                            kbps = kbps,
                            lastFrameBytes = rawLen,
                            timestampText = nowTimeStr(),
                            bodyText = formatted
                        )
                    )
                }
            }
        }
        wsClient.startWatchingTopic(topic, knownType, singleShot)
    }

    private fun buildDemoTopicList(): List<RosTopicEntry> = listOf(
        RosTopicEntry("/Odometry", "nav_msgs/Odometry", 1),
        RosTopicEntry("/grid_map/occupancy_inflate", "sensor_msgs/PointCloud2", 1),
        RosTopicEntry("/cloud_registered", "sensor_msgs/PointCloud2", 1),
        RosTopicEntry("/ego_planner_node/optimal_list", "visualization_msgs/Marker", 1),
        RosTopicEntry("/planning/pos_cmd", "quadrotor_msgs/PositionCommand", 1),
        RosTopicEntry("/path", "nav_msgs/Path", 1),
        RosTopicEntry("/mavros/state", "mavros_msgs/State", 1),
        RosTopicEntry("/mavros/battery", "sensor_msgs/BatteryState", 1),
        RosTopicEntry("/mavros/imu/data", "sensor_msgs/Imu", 1),
        RosTopicEntry("/tilt_uav/servo_angles", "std_msgs/Float32MultiArray", 1),
        RosTopicEntry("/active_pitch_planner/pitch_tilt_cmd", "geometry_msgs/Vector3Stamped", 1),
        RosTopicEntry("/mavros/actuator_control", "mavros_msgs/ActuatorControl", 1),
        RosTopicEntry("/move_base_simple/goal", "geometry_msgs/PoseStamped", 1)
    )

    private fun runDemoInspectSession(
        sid: Int,
        topic: String,
        knownType: String,
        singleShot: Boolean,
        onUpdate: (TopicEchoSnapshot) -> Unit
    ) {
        val nominalHz = when (topic) {
            "/Odometry", "/planning/pos_cmd" -> 50f
            "/mavros/imu/data" -> 200f
            "/cloud_registered", "/path" -> 15f
            "/grid_map/occupancy_inflate", "/ego_planner_node/optimal_list" -> 12.5f
            else -> 5f
        }

        if (singleShot) {
            demoScheduler.schedule({
                if (sessionGen.get() != sid) return@schedule
                val (text, estBytes, pub) = synthesizeDemoMessage(topic, 1L)
                onUpdate(
                    TopicEchoSnapshot(
                        topic = topic,
                        msgType = knownType,
                        publisherNode = pub,
                        state = TopicEchoState.CAPTURED,
                        statusText = "单次截取完成 (仿真 · $estBytes B)",
                        isStreaming = false,
                        receivedCount = 1L,
                        hz = nominalHz,
                        kbps = (nominalHz * estBytes) / 1024f,
                        lastFrameBytes = estBytes,
                        timestampText = nowTimeStr(),
                        bodyText = text
                    )
                )
            }, 60L, TimeUnit.MILLISECONDS)
        } else {
            val startMs = System.currentTimeMillis()
            var seq = 0L
            demoStreamTask = demoScheduler.scheduleAtFixedRate({
                if (sessionGen.get() != sid) return@scheduleAtFixedRate
                seq++
                val elapsedSec = max(0.15f, (System.currentTimeMillis() - startMs) / 1000f)
                val simTotalFrames = max(seq, (elapsedSec * nominalHz).toLong())
                val (text, estBytes, pub) = synthesizeDemoMessage(topic, simTotalFrames)
                onUpdate(
                    TopicEchoSnapshot(
                        topic = topic,
                        msgType = knownType,
                        publisherNode = pub,
                        state = TopicEchoState.STREAMING,
                        statusText = "实时读取中 ($pub · 仿真)",
                        isStreaming = true,
                        receivedCount = simTotalFrames,
                        hz = nominalHz,
                        kbps = (nominalHz * estBytes) / 1024f,
                        lastFrameBytes = estBytes,
                        timestampText = nowTimeStr(),
                        bodyText = text
                    )
                )
            }, 50L, 160L, TimeUnit.MILLISECONDS)
        }
    }

    private fun synthesizeDemoMessage(topic: String, seq: Long): Triple<String, Int, String> {
        val s = hub.snapshots.readInto(readSnapshot)
        val nowMs = System.currentTimeMillis()
        val secs = nowMs / 1000L
        val nsecs = (nowMs % 1000L) * 1_000_000L
        val halfYaw = s.yaw * 0.5
        val qz = sin(halfYaw)
        val qw = cos(halfYaw)

        return when (topic) {
            "/Odometry" -> Triple(
                String.format(
                    Locale.US,
                    """header:
  seq: %d
  stamp: %d.%09d
  frame_id: "world"
child_frame_id: "body"
pose:
  position: { x: %.4f, y: %.4f, z: %.4f }
  orientation: { x: %.4f, y: %.4f, z: %.4f, w: %.4f }
twist:
  linear: { x: %.4f, y: %.4f, z: %.4f }
  angular: { x: 0.0012, y: -0.0008, z: %.4f }""",
                    seq, secs, nsecs, s.uavX, s.uavY, s.uavZ,
                    s.smoothRoll * 0.5f, s.smoothPitch * 0.5f, qz, qw,
                    s.vx, s.vy, s.vz, s.ay * 0.12f
                ),
                736,
                "/laserMapping"
            )

            "/grid_map/occupancy_inflate" -> {
                val n = s.voxelCount
                val sb = StringBuilder()
                sb.append(String.format(Locale.US, "header:\n  seq: %d\n  stamp: %d.%09d\n  frame_id: \"world\"\n", seq, secs, nsecs))
                sb.append("height: 1\nwidth: $n\npoint_step: 12\nis_dense: true\n")
                sb.append("fields: [x(FLOAT32), y(FLOAT32), z(FLOAT32)]\n")
                sb.append("# EGO-Planner 膨胀占据体素块中心点摘要 (共 $n 个三维体素):\n")
                val sampleCnt = min(n, 5)
                for (i in 0 until sampleCnt) {
                    sb.append(String.format(Locale.US, "  voxel[%d]: (%.2f, %.2f, %.2f)\n", i, s.voxels[i * 4], s.voxels[i * 4 + 1], s.voxels[i * 4 + 2]))
                }
                if (n > sampleCnt) sb.append("  ... (其余 ${n - sampleCnt} 个膨胀体素已省略)")
                Triple(sb.toString(), 64 + n * 12, "/ego_planner_node")
            }

            "/cloud_registered" -> Triple(
                String.format(
                    Locale.US,
                    """header:
  seq: %d
  stamp: %d.%09d
  frame_id: "camera_init"
height: 1
width: %d
point_step: 16
fields: [x(FLOAT32), y(FLOAT32), z(FLOAT32), intensity(FLOAT32)]
is_dense: true
# FAST-LIO 配准点云统计:
  单帧激光点数: %d pts
  全局累积体素: %d / 300000""",
                    seq,
                    secs,
                    nsecs,
                    s.cloudScanPts,
                    s.cloudScanPts,
                    s.mapPointCount
                ),
                80 + max(1, s.cloudScanPts) * 16,
                "/laserMapping"
            )

            "/ego_planner_node/optimal_list" -> {
                val cnt = s.bsplineCount
                val sb = StringBuilder()
                sb.append(String.format(Locale.US, "header:\n  seq: %d\n  stamp: %d.%09d\n  frame_id: \"world\"\n", seq, secs, nsecs))
                sb.append("ns: \"optimal_list\"\ntype: 4 (LINE_STRIP)\naction: 0 (ADD)\n")
                sb.append(String.format(Locale.US, "scale: { x: 0.10, y: 0.10, z: 0.10 }\npoints_count: %d (弧长 %.2f m)\n", cnt, s.trajLength))
                val showN = min(cnt, 4)
                for (i in 0 until showN) {
                    sb.append(String.format(Locale.US, "  pt[%d]: { x: %.3f, y: %.3f, z: %.3f }\n", i, s.bsplinePts[i * 3], s.bsplinePts[i * 3 + 1], s.bsplinePts[i * 3 + 2]))
                }
                Triple(sb.toString().trimEnd(), 148 + cnt * 24, "/ego_planner_node")
            }

            "/planning/pos_cmd" -> Triple(
                String.format(
                    Locale.US,
                    """header:
  seq: %d
  stamp: %d.%09d
  frame_id: "world"
position:     { x: %.4f, y: %.4f, z: %.4f }
velocity:     { x: %.4f, y: %.4f, z: %.4f }
acceleration: { x: %.4f, y: %.4f, z: 0.0000 }
yaw: %.4f
yaw_dot: 0.0000
trajectory_flag: 1 (TRAJECTORY_STATUS_READY)""",
                    seq, secs, nsecs, s.uavX, s.uavY, s.uavZ, s.vx, s.vy, s.vz, s.ax, s.ay, s.yaw
                ),
                136,
                "/traj_server"
            )

            "/mavros/state" -> Triple(
                String.format(
                    Locale.US,
                    """header:
  seq: %d
  stamp: %d.%09d
connected: true
armed: %s
guided: true
manual_input: false
mode: "%s"
system_status: 4 (MAV_STATE_ACTIVE)""",
                    seq,
                    secs,
                    nsecs,
                    s.armed.toString(),
                    s.px4Mode
                ),
                52,
                "/mavros"
            )

            "/mavros/battery" -> Triple(
                String.format(
                    Locale.US,
                    """header:
  seq: %d
  stamp: %d.%09d
voltage: %.3f
current: -%.2f
percentage: %.2f
power_supply_status: 2 (DISCHARGING)""",
                    seq,
                    secs,
                    nsecs,
                    s.batVolt,
                    s.batCurrentAmps,
                    s.batPct / 100f
                ),
                68,
                "/mavros"
            )

            "/tilt_uav/servo_angles" -> Triple(
                String.format(
                    Locale.US,
                    """layout:
  dim: [{ label: "servo_rad_and_rate", size: 4, stride: 4 }]
  data_offset: 0
# Tilt-UAV-INDI (g1_indi_xyz): S1(M1,M3前组) / S2(M2,M4后组)
data: [%.4f, %.4f, %.2f, %.2f]
deg_summary: { servo_1_deg: %+.2f°, servo_2_deg: %+.2f°, manipulability_w: %.3f }""",
                    s.tiltServo1Rad, s.tiltServo2Rad, s.tiltServo1RateDegS, s.tiltServo2RateDegS,
                    s.tiltServo1Rad * 57.29578f, s.tiltServo2Rad * 57.29578f, s.tiltManipulability
                ),
                48,
                "/px4_ego_live_bridge"
            )

            "/active_pitch_planner/pitch_tilt_cmd" -> Triple(
                String.format(
                    Locale.US,
                    """header:
  seq: %d
  stamp: %d.%09d
  frame_id: "base_link"
# x=pitch_bias_deg, y=coord_mode(0:平飞,1:协同,2:锁倾), z=max_tilt_deg
vector: { x: %+.2f, y: %d.0, z: %.1f }""",
                    seq,
                    secs,
                    nsecs,
                    hub.tiltPitchBiasDeg,
                    hub.tiltCoordMode,
                    hub.tiltMaxAngleDeg
                ),
                52,
                "/active_pitch_planner_node"
            )

            "/mavros/actuator_control" -> Triple(
                String.format(
                    Locale.US,
                    """header:
  seq: %d
  stamp: %d.%09d
  frame_id: "base_link"
group_mix: 0
# u = [T1, T2, T3, T4, alpha_1_norm, alpha_2_norm, 0, 0]
controls: [%.3f, %.3f, %.3f, %.3f, %+.3f, %+.3f, 0.000, 0.000]""",
                    seq, secs, nsecs,
                    s.motorNorm4[0], s.motorNorm4[1], s.motorNorm4[2], s.motorNorm4[3],
                    (s.tiltServo1Rad / 0.7854f).coerceIn(-1f, 1f),
                    (s.tiltServo2Rad / 0.7854f).coerceIn(-1f, 1f)
                ),
                61,
                "/mavros"
            )

            else -> Triple(
                String.format(
                    Locale.US,
                    """header:
  seq: %d
  stamp: %d.%09d
  frame_id: "world"
pose:
  position: { x: %.3f, y: %.3f, z: %.3f }
  orientation: { x: 0.000, y: 0.000, z: %.4f, w: %.4f }""",
                    seq, secs, nsecs, s.goalX, s.goalY, s.goalZ, qz, qw
                ),
                84,
                "/ego_pilot_android"
            )
        }
    }

    private fun parseMessageDefinitionBundle(rootType: String, rawDef: String): Map<String, List<MsgFieldSpec>> {
        val map = HashMap<String, List<MsgFieldSpec>>()
        // 预置标准 std_msgs/Header 定义，防止部分节点未附带子消息段时无法展开 Header
        map["std_msgs/Header"] = listOf(
            MsgFieldSpec("uint32", "seq", false, 0),
            MsgFieldSpec("time", "stamp", false, 0),
            MsgFieldSpec("string", "frame_id", false, 0)
        )
        map["Header"] = map["std_msgs/Header"]!!

        if (rawDef.isBlank()) return map
        val sections = rawDef.split(Regex("={10,}"))
        for ((idx, sec) in sections.withIndex()) {
            val lines = sec.lines()
            var secType = if (idx == 0) rootType else ""
            val fields = ArrayList<MsgFieldSpec>()

            for (rawLine in lines) {
                val clean = rawLine.substringBefore('#').trim()
                if (clean.isEmpty()) continue
                if (clean.startsWith("MSG:")) {
                    secType = clean.removePrefix("MSG:").trim()
                    continue
                }
                if (clean.contains('=')) continue

                val normLine = clean.replace(Regex("\\s*\\[\\s*"), "[").replace(Regex("\\s*\\]"), "]")
                val parts = normLine.split(Regex("\\s+"))
                if (parts.size < 2) continue
                val rawFieldType = parts[0].trim()
                val fieldName = parts[1].trim()

                val bracketOpen = rawFieldType.indexOf('[')
                if (bracketOpen > 0 && rawFieldType.endsWith(']')) {
                    val baseT = rawFieldType.substring(0, bracketOpen)
                    val inside = rawFieldType.substring(bracketOpen + 1, rawFieldType.length - 1).trim()
                    val fixedLen = if (inside.isEmpty()) -1 else (inside.toIntOrNull() ?: -1)
                    fields.add(MsgFieldSpec(baseT, fieldName, isArray = true, fixedArrayLen = fixedLen))
                } else {
                    fields.add(MsgFieldSpec(rawFieldType, fieldName, isArray = false, fixedArrayLen = 0))
                }
            }
            if (secType.isNotEmpty() && fields.isNotEmpty()) {
                map[secType] = fields
                val shortName = secType.substringAfterLast('/')
                if (!map.containsKey(shortName)) {
                    map[shortName] = fields
                }
            }
        }
        return map
    }

    private fun formatBinaryRosPayload(
        rootType: String,
        bytes: ByteArray,
        length: Int,
        schemaMap: Map<String, List<MsgFieldSpec>>
    ): String {
        if (rootType == "sensor_msgs/PointCloud2" || rootType.endsWith("/PointCloud2")) {
            formatPointCloud2Summary(bytes, length)?.let { return it }
        }
        // 对高频飞控核心消息提供零依赖内置结构化解码（即使发布节点省略 message_definition 也能精准解析）
        formatWellKnownBuiltinType(rootType, bytes, length)?.let { return it }

        if (schemaMap.containsKey(rootType)) {
            try {
                val bb = ByteBuffer.wrap(bytes, 0, length).order(ByteOrder.LITTLE_ENDIAN)
                val sb = StringBuilder()
                val parentPkg = rootType.substringBefore('/', "")
                decodeStructRecursive(bb, rootType, parentPkg, schemaMap, sb, indent = 0, depth = 0)
                if (sb.isNotEmpty()) return sb.toString().trimEnd()
            } catch (_: Throwable) {}
        }
        return formatHexFallback(rootType, bytes, length)
    }

    private fun formatWellKnownBuiltinType(rootType: String, bytes: ByteArray, length: Int): String? {
        try {
            val bb = ByteBuffer.wrap(bytes, 0, length).order(ByteOrder.LITTLE_ENDIAN)
            when {
                rootType == "nav_msgs/Odometry" || rootType.endsWith("/Odometry") -> {
                    val hdr = readStdHeaderSummary(bb) ?: return null
                    val childFrame = readSafeRosString(bb) ?: return null
                    if (bb.remaining() < 344) return null
                    val px = bb.double
                    val py = bb.double
                    val pz = bb.double
                    val qx = bb.double
                    val qy = bb.double
                    val qz = bb.double
                    val qw = bb.double
                    bb.position(bb.position() + 288)
                    val vx = if (bb.remaining() >= 24) bb.double else 0.0
                    val vy = if (bb.remaining() >= 16) bb.double else 0.0
                    val vz = if (bb.remaining() >= 8) bb.double else 0.0
                    val wx = if (bb.remaining() >= 24) bb.double else 0.0
                    val wy = if (bb.remaining() >= 16) bb.double else 0.0
                    val wz = if (bb.remaining() >= 8) bb.double else 0.0
                    val nrm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw).coerceAtLeast(1e-9)
                    val nx = qx / nrm
                    val ny = qy / nrm
                    val nz = qz / nrm
                    val nw = qw / nrm
                    val rollDeg = Math.toDegrees(atan2(2.0 * (nw * nx + ny * nz), 1.0 - 2.0 * (nx * nx + ny * ny)))
                    val pitchDeg = Math.toDegrees(asin((2.0 * (nw * ny - nz * nx)).coerceIn(-1.0, 1.0)))
                    val yawDeg = Math.toDegrees(atan2(2.0 * (nw * nz + nx * ny), 1.0 - 2.0 * (ny * ny + nz * nz)))
                    return String.format(
                        Locale.US,
                        "%schild_frame_id: \"%s\"\npose:\n  position:    { x: %.4f, y: %.4f, z: %.4f }\n  orientation: { x: %.4f, y: %.4f, z: %.4f, w: %.4f }\n  euler_deg:   { roll: %+.2f°, pitch: %+.2f°, yaw: %+.2f° }\ntwist:\n  linear:      { x: %.4f, y: %.4f, z: %.4f }\n  angular:     { x: %.4f, y: %.4f, z: %.4f }",
                        hdr, childFrame, px, py, pz, qx, qy, qz, qw, rollDeg, pitchDeg, yawDeg, vx, vy, vz, wx, wy, wz
                    )
                }
                rootType == "sensor_msgs/Imu" || rootType.endsWith("/Imu") -> {
                    val hdr = readStdHeaderSummary(bb) ?: return null
                    if (bb.remaining() < 32 + 72 + 24 + 72 + 24) return null
                    val qx = bb.double
                    val qy = bb.double
                    val qz = bb.double
                    val qw = bb.double
                    bb.position(bb.position() + 72)
                    val gx = bb.double
                    val gy = bb.double
                    val gz = bb.double
                    bb.position(bb.position() + 72)
                    val ax = bb.double
                    val ay = bb.double
                    val az = bb.double
                    return String.format(
                        Locale.US,
                        "%sorientation:         { x: %.4f, y: %.4f, z: %.4f, w: %.4f }\nangular_velocity:    { x: %.4f, y: %.4f, z: %.4f } rad/s\nlinear_acceleration: { x: %.4f, y: %.4f, z: %.4f } m/s²",
                        hdr, qx, qy, qz, qw, gx, gy, gz, ax, ay, az
                    )
                }
                rootType == "mavros_msgs/State" -> {
                    val hdr = readStdHeaderSummary(bb) ?: return null
                    if (bb.remaining() < 5) return null
                    val connected = bb.get().toInt() != 0
                    val armed = bb.get().toInt() != 0
                    val guided = bb.get().toInt() != 0
                    val manual = bb.get().toInt() != 0
                    val mode = readSafeRosString(bb) ?: ""
                    val sysStatus = if (bb.remaining() >= 1) (bb.get().toInt() and 0xFF) else 0
                    return "${hdr}connected: $connected\narmed: $armed\nguided: $guided\nmanual_input: $manual\nmode: \"$mode\"\nsystem_status: $sysStatus"
                }
                rootType == "geometry_msgs/Vector3Stamped" || rootType.endsWith("/Vector3Stamped") -> {
                    val hdr = readStdHeaderSummary(bb) ?: return null
                    if (bb.remaining() < 24) return null
                    val vx = bb.double
                    val vy = bb.double
                    val vz = bb.double
                    return String.format(Locale.US, "%svector: { x: %.4f, y: %.4f, z: %.4f }", hdr, vx, vy, vz)
                }
                rootType == "mavros_msgs/ActuatorControl" || rootType.endsWith("/ActuatorControl") -> {
                    val hdr = readStdHeaderSummary(bb) ?: return null
                    if (bb.remaining() < 33) return null
                    val grp = bb.get().toInt() and 0xFF
                    val c = FloatArray(8) { bb.float }
                    return String.format(
                        Locale.US,
                        "%sgroup_mix: %d\ncontrols_motors_1_4: [%.3f, %.3f, %.3f, %.3f]\ncontrols_tilt_servos: [%+.3f, %+.3f]",
                        hdr, grp, c[0], c[1], c[2], c[3], c[4], c[5]
                    )
                }
                rootType == "geometry_msgs/PoseStamped" -> {
                    val hdr = readStdHeaderSummary(bb) ?: return null
                    if (bb.remaining() < 56) return null
                    val px = bb.double
                    val py = bb.double
                    val pz = bb.double
                    val qx = bb.double
                    val qy = bb.double
                    val qz = bb.double
                    val qw = bb.double
                    return String.format(
                        Locale.US,
                        "%spose:\n  position:    { x: %.4f, y: %.4f, z: %.4f }\n  orientation: { x: %.4f, y: %.4f, z: %.4f, w: %.4f }",
                        hdr, px, py, pz, qx, qy, qz, qw
                    )
                }
            }
        } catch (_: Throwable) {}
        return null
    }

    private fun readStdHeaderSummary(bb: ByteBuffer): String? {
        if (bb.remaining() < 16) return null
        val seq = bb.int.toLong() and 0xFFFFFFFFL
        val sec = bb.int.toLong() and 0xFFFFFFFFL
        val nsec = bb.int.toLong() and 0xFFFFFFFFL
        val frameId = readSafeRosString(bb) ?: return null
        return String.format(Locale.US, "header:\n  seq: %d\n  stamp: %d.%09d\n  frame_id: \"%s\"\n", seq, sec, nsec, frameId)
    }

    private fun readSafeRosString(bb: ByteBuffer): String? {
        if (bb.remaining() < 4) return null
        val sLen = bb.int
        if (sLen < 0 || sLen > bb.remaining()) return null
        val pos = bb.position()
        val s = String(bb.array(), bb.arrayOffset() + pos, sLen, Charsets.UTF_8)
        bb.position(pos + sLen)
        return s
    }

    private fun decodeStructRecursive(
        bb: ByteBuffer,
        typeName: String,
        parentPkg: String,
        schemaMap: Map<String, List<MsgFieldSpec>>,
        sb: StringBuilder,
        indent: Int,
        depth: Int
    ) {
        if (depth > 8) return
        val resolvedKey = resolveSubTypeKey(typeName, parentPkg, schemaMap)
            ?: throw IllegalStateException("Unknown nested type: $typeName")
        val fields = schemaMap[resolvedKey] ?: return
        val curPkg = resolvedKey.substringBefore('/', parentPkg)
        val pad = "  ".repeat(indent)

        for (f in fields) {
            if (!f.isArray) {
                if (isPrimitiveRosType(f.baseType)) {
                    val vStr = readPrimitiveAsString(bb, f.baseType)
                    sb.append(pad).append(f.name).append(": ").append(vStr).append('\n')
                } else {
                    sb.append(pad).append(f.name).append(":\n")
                    decodeStructRecursive(bb, f.baseType, curPkg, schemaMap, sb, indent + 1, depth + 1)
                }
            } else {
                val count = if (f.fixedArrayLen >= 0) f.fixedArrayLen else bb.int
                if (count < 0 || count > 20_000_000) throw IllegalStateException("Invalid array count: $count")

                if ((f.baseType == "uint8" || f.baseType == "byte" || f.baseType == "int8") && count > 16) {
                    if (bb.remaining() < count) throw EOFException()
                    bb.position(bb.position() + count)
                    sb.append(pad).append(f.name).append(": [二进制数据流 ").append(count).append(" bytes]\n")
                } else if (isPrimitiveRosType(f.baseType)) {
                    val previewMax = 8
                    val items = ArrayList<String>(min(count, previewMax))
                    for (i in 0 until count) {
                        val sVal = readPrimitiveAsString(bb, f.baseType)
                        if (i < previewMax) items.add(sVal)
                    }
                    val tail = if (count > previewMax) ", ... 共 $count 项" else ""
                    sb.append(pad).append(f.name).append(": [").append(items.joinToString(", ")).append(tail).append("]\n")
                } else {
                    sb.append(pad).append(f.name).append(": (共 ").append(count).append(" 项)\n")
                    val showStructMax = 4
                    val dummySb = StringBuilder()
                    for (i in 0 until count) {
                        if (i < showStructMax) {
                            sb.append(pad).append("  - [").append(i).append("]:\n")
                            decodeStructRecursive(bb, f.baseType, curPkg, schemaMap, sb, indent + 2, depth + 1)
                        } else {
                            dummySb.setLength(0)
                            decodeStructRecursive(bb, f.baseType, curPkg, schemaMap, dummySb, 0, depth + 1)
                        }
                    }
                    if (count > showStructMax) {
                        sb.append(pad).append("  ... (其余 ").append(count - showStructMax).append(" 项已折叠)\n")
                    }
                }
            }
        }
    }

    private fun resolveSubTypeKey(rawType: String, parentPkg: String, schemaMap: Map<String, List<MsgFieldSpec>>): String? {
        if (rawType == "Header" || rawType == "std_msgs/Header") {
            if (schemaMap.containsKey("std_msgs/Header")) return "std_msgs/Header"
            if (schemaMap.containsKey("Header")) return "Header"
        }
        if (schemaMap.containsKey(rawType)) return rawType
        if (!rawType.contains('/') && parentPkg.isNotEmpty()) {
            val qualified = "$parentPkg/$rawType"
            if (schemaMap.containsKey(qualified)) return qualified
        }
        val shortName = rawType.substringAfterLast('/')
        return if (schemaMap.containsKey(shortName)) shortName else null
    }

    private fun isPrimitiveRosType(t: String): Boolean = when (t) {
        "bool", "int8", "uint8", "byte", "char",
        "int16", "uint16", "int32", "uint32", "int64", "uint64",
        "float32", "float64", "string", "time", "duration" -> true
        else -> false
    }

    private fun readPrimitiveAsString(bb: ByteBuffer, t: String): String = when (t) {
        "bool" -> if (bb.get().toInt() != 0) "true" else "false"
        "int8", "byte" -> bb.get().toInt().toString()
        "uint8", "char" -> (bb.get().toInt() and 0xFF).toString()
        "int16" -> bb.short.toInt().toString()
        "uint16" -> (bb.short.toInt() and 0xFFFF).toString()
        "int32" -> bb.int.toString()
        "uint32" -> (bb.int.toLong() and 0xFFFFFFFFL).toString()
        "int64" -> bb.long.toString()
        "uint64" -> java.lang.Long.toUnsignedString(bb.long)
        "float32" -> String.format(Locale.US, "%.4f", bb.float)
        "float64" -> String.format(Locale.US, "%.4f", bb.double)
        "time", "duration" -> {
            val sec = bb.int.toLong() and 0xFFFFFFFFL
            val nsec = bb.int.toLong() and 0xFFFFFFFFL
            String.format(Locale.US, "%d.%09d", sec, nsec)
        }
        "string" -> {
            val s = readSafeRosString(bb) ?: throw EOFException()
            "\"$s\""
        }
        else -> "?"
    }

    private fun formatPointCloud2Summary(bytes: ByteArray, length: Int): String? {
        try {
            val bb = ByteBuffer.wrap(bytes, 0, length).order(ByteOrder.LITTLE_ENDIAN)
            if (bb.remaining() < 24) return null
            val seq = bb.int.toLong() and 0xFFFFFFFFL
            val secs = bb.int.toLong() and 0xFFFFFFFFL
            val nsecs = bb.int.toLong() and 0xFFFFFFFFL
            val fIdLen = bb.int
            if (fIdLen < 0 || bb.remaining() < fIdLen + 12) return null
            val fIdPos = bb.position()
            val frameId = String(bytes, fIdPos, fIdLen, Charsets.UTF_8)
            bb.position(fIdPos + fIdLen)

            val height = bb.int
            val width = bb.int
            val fieldCount = bb.int
            if (fieldCount < 0 || fieldCount > 64) return null

            val fieldDesc = ArrayList<String>(fieldCount)
            var offX = -1
            var offY = -1
            var offZ = -1
            for (i in 0 until fieldCount) {
                val nLen = bb.int
                if (nLen < 0 || bb.remaining() < nLen + 9) return null
                val np = bb.position()
                val fName = String(bytes, np, nLen, Charsets.UTF_8)
                bb.position(np + nLen)
                val offset = bb.int
                val dtype = bb.get().toInt() and 0xFF
                bb.int
                fieldDesc.add("$fName(off=$offset,t=$dtype)")
                when (fName) {
                    "x" -> offX = offset
                    "y" -> offY = offset
                    "z" -> offZ = offset
                }
            }
            if (bb.remaining() < 14) return null
            val bigEnd = bb.get().toInt() != 0
            val pointStep = bb.int
            val rowStep = bb.int
            val dataLen = bb.int
            if (pointStep <= 0 || dataLen < 0 || bb.remaining() < dataLen) return null
            val dataPos = bb.position()
            bb.position(dataPos + dataLen)
            val isDense = if (bb.remaining() >= 1) (bb.get().toInt() != 0) else true

            val totalPts = if (pointStep > 0) dataLen / pointStep else (width * height)
            val sb = StringBuilder()
            sb.append(String.format(Locale.US, "header:\n  seq: %d\n  stamp: %d.%09d\n  frame_id: \"%s\"\n", seq, secs, nsecs, frameId))
            sb.append("height: $height\nwidth: $width (总点数/体素数: $totalPts)\n")
            sb.append("point_step: $pointStep  row_step: $rowStep  is_dense: $isDense\n")
            sb.append("fields: ").append(fieldDesc.joinToString(", ")).append('\n')

            if (offX >= 0 && offY >= 0 && offZ >= 0 && totalPts > 0) {
                val view = ByteBuffer.wrap(bytes, dataPos, dataLen)
                    .order(if (bigEnd) ByteOrder.BIG_ENDIAN else ByteOrder.LITTLE_ENDIAN)
                var minX = Float.POSITIVE_INFINITY
                var maxX = Float.NEGATIVE_INFINITY
                var minY = Float.POSITIVE_INFINITY
                var maxY = Float.NEGATIVE_INFINITY
                var minZ = Float.POSITIVE_INFINITY
                var maxZ = Float.NEGATIVE_INFINITY
                val step = max(1, totalPts / 500)
                var p = 0
                val req = max(offX, max(offY, offZ)) + 4
                while (p < totalPts) {
                    val base = dataPos + p * pointStep
                    if (base + req > dataPos + dataLen) break
                    val x = view.getFloat(base + offX)
                    val y = view.getFloat(base + offY)
                    val z = view.getFloat(base + offZ)
                    if (x.isFinite() && y.isFinite() && z.isFinite()) {
                        if (x < minX) minX = x
                        if (x > maxX) maxX = x
                        if (y < minY) minY = y
                        if (y > maxY) maxY = y
                        if (z < minZ) minZ = z
                        if (z > maxZ) maxZ = z
                    }
                    p += step
                }
                if (minX.isFinite()) {
                    sb.append(String.format(Locale.US, "xyz_bounds:\n  X: [%.2f, %.2f] m\n  Y: [%.2f, %.2f] m\n  Z: [%.2f, %.2f] m\n", minX, maxX, minY, maxY, minZ, maxZ))
                }
                sb.append("sample_points:\n")
                val showK = min(totalPts, 5)
                for (k in 0 until showK) {
                    val base = dataPos + k * pointStep
                    if (base + req > dataPos + dataLen) break
                    sb.append(String.format(Locale.US, "  [%d]: (%.3f, %.3f, %.3f)\n", k, view.getFloat(base + offX), view.getFloat(base + offY), view.getFloat(base + offZ)))
                }
            }
            return sb.toString().trimEnd()
        } catch (_: Throwable) {
            return null
        }
    }

    private fun formatHexFallback(msgType: String, bytes: ByteArray, length: Int): String {
        val sb = StringBuilder()
        sb.append("# 消息类型: $msgType (总长 $length bytes)\n")
        val showN = min(length, 96)
        for (i in 0 until showN step 16) {
            sb.append(String.format(Locale.US, "%04X: ", i))
            val end = min(i + 16, showN)
            for (j in i until end) {
                sb.append(String.format(Locale.US, "%02X ", bytes[j].toInt() and 0xFF))
            }
            sb.append('\n')
        }
        if (length > showN) sb.append("... (后续 ${length - showN} 字节已省略)")
        return sb.toString()
    }

    private fun formatJsonAsYaml(obj: JSONObject, indent: Int): String {
        val sb = StringBuilder()
        val pad = "  ".repeat(indent)
        val keys = obj.keys()
        while (keys.hasNext()) {
            val k = keys.next()
            val v = obj.opt(k)
            when (v) {
                is JSONObject -> {
                    sb.append(pad).append(k).append(":\n").append(formatJsonAsYaml(v, indent + 1))
                }
                is JSONArray -> {
                    val n = v.length()
                    if (n == 0) {
                        sb.append(pad).append(k).append(": []\n")
                    } else if (v.opt(0) is JSONObject) {
                        sb.append(pad).append(k).append(": (共 ").append(n).append(" 项)\n")
                        val lim = min(n, 4)
                        for (i in 0 until lim) {
                            sb.append(pad).append("  - [").append(i).append("]:\n")
                            sb.append(formatJsonAsYaml(v.optJSONObject(i), indent + 2))
                        }
                        if (n > lim) sb.append(pad).append("  ... (其余 ").append(n - lim).append(" 项已折叠)\n")
                    } else {
                        val lim = min(n, 8)
                        val list = (0 until lim).map { idx -> v.opt(idx).toString() }
                        val tail = if (n > lim) ", ... 共 $n 项" else ""
                        sb.append(pad).append(k).append(": [").append(list.joinToString(", ")).append(tail).append("]\n")
                    }
                }
                is String -> {
                    if (v.length > 120) {
                        sb.append(pad).append(k).append(": [Base64/长文本 ").append(v.length).append(" chars]\n")
                    } else {
                        sb.append(pad).append(k).append(": \"").append(v).append("\"\n")
                    }
                }
                is Number -> {
                    val d = v.toDouble()
                    if (v is Double || v is Float) {
                        sb.append(pad).append(k).append(": ").append(String.format(Locale.US, "%.4f", d)).append('\n')
                    } else {
                        sb.append(pad).append(k).append(": ").append(v).append('\n')
                    }
                }
                else -> sb.append(pad).append(k).append(": ").append(v).append('\n')
            }
        }
        return sb.toString()
    }

    private fun parseTopicTypePairs(xml: String): List<Pair<String, String>> {
        val out = ArrayList<Pair<String, String>>()
        val pairRegex = Regex(
            "<array>\\s*<data>\\s*<value>\\s*(?:<string>)?([^<]+?)(?:</string>)?\\s*</value>\\s*<value>\\s*(?:<string>)?([^<]+?)(?:</string>)?\\s*</value>\\s*</data>\\s*</array>"
        )
        for (m in pairRegex.findAll(xml)) {
            val name = m.groupValues[1].trim()
            val tp = m.groupValues[2].trim()
            if (name.startsWith('/')) {
                out.add(name to tp)
            }
        }
        return out
    }

    private fun parsePublisherNodesFromSystemState(xml: String): Map<String, List<String>> {
        if (xml.isBlank()) return emptyMap()
        val result = HashMap<String, List<String>>()
        val pubSection = xml.substringAfter("<data>", "").substringAfter("<data>", "")
        val itemRegex = Regex(
            "<array>\\s*<data>\\s*<value>\\s*(?:<string>)?(/[^<\\s]+)(?:</string>)?\\s*</value>\\s*<value>\\s*<array>\\s*<data>(.*?)</data>\\s*</array>\\s*</value>\\s*</data>\\s*</array>",
            setOf(RegexOption.DOT_MATCHES_ALL)
        )
        val nodeValRegex = Regex("<value>\\s*(?:<string>)?([^<]+?)(?:</string>)?\\s*</value>")
        for (m in itemRegex.findAll(pubSection)) {
            val topic = m.groupValues[1].trim()
            val inner = m.groupValues[2]
            if (!result.containsKey(topic)) {
                val nodes = nodeValRegex.findAll(inner)
                    .map { it.groupValues[1].trim() }
                    .filter { it.isNotEmpty() }
                    .toList()
                if (nodes.isNotEmpty()) {
                    result[topic] = nodes
                }
            }
        }
        return result
    }

    private fun extractHttpUris(xml: String): List<String> {
        return Regex("http://[^<\\s\"']+").findAll(xml).map { it.value.trim() }.distinct().toList()
    }

    /**
     * 兼容 Python rospy (`<value><string>TCPROS</string></value>`) 与 C++ roscpp xmlrpcpp (`<value>TCPROS</value>`) 的端口提取器！
     */
    private fun parseRequestTopicEndpoint(xml: String, fallbackHost: String): Pair<String, Int>? {
        val protoArrayRegex = Regex(
            "<array>\\s*<data>\\s*<value>\\s*(?:<string>)?\\s*TCPROS\\s*(?:</string>)?\\s*</value>\\s*" +
                "<value>\\s*(?:<string>)?\\s*([^<]*?)\\s*(?:</string>)?\\s*</value>\\s*" +
                "<value>\\s*<(?:int|i4)>\\s*(\\d+)\\s*</(?:int|i4)>\\s*</value>\\s*</data>\\s*</array>",
            setOf(RegexOption.DOT_MATCHES_ALL, RegexOption.IGNORE_CASE)
        )
        protoArrayRegex.find(xml)?.let { m ->
            val rawHost = m.groupValues[1].trim().ifEmpty { fallbackHost }
            val port = m.groupValues[2].trim().toIntOrNull() ?: return null
            if (port in 1..65535) return rawHost to port
        }

        // 兜底解析：提取所有原子 <value> 项（无论是否带 <string> 标签）
        val scalarValues = Regex("<value>\\s*(?:<(string|int|i4)>)?\\s*([^<]+?)\\s*(?:</(?:string|int|i4)>)?\\s*</value>")
            .findAll(xml)
            .map { it.groupValues[2].trim() }
            .toList()
        val tcpIdx = scalarValues.indexOfFirst { it.equals("TCPROS", ignoreCase = true) }
        if (tcpIdx >= 0 && tcpIdx + 2 < scalarValues.size) {
            val rawHost = scalarValues[tcpIdx + 1].ifEmpty { fallbackHost }
            val port = scalarValues[tcpIdx + 2].toIntOrNull()
            if (port != null && port in 1..65535) return rawHost to port
        }
        return null
    }

    /**
     * 构建候选 Publisher XML-RPC URL 列表：
     * - 使用正则代替 `java.net.URI`（避免含下划线的主机名如 `zhb_nuc12` 导致 `URI.host == null, port == -1`）；
     * - 若节点返回的主机名非 IPv4（如 `ubuntu`, `nuc12.local`, `localhost`），优先使用可达的 `masterHost` IP，彻底解决机载未设 `ROS_IP` 导致的连接失败！
     */
    private fun buildCandidateHttpUris(rawPubUri: String, masterHost: String): List<String> {
        val m = HTTP_HOST_PORT_REGEX.find(rawPubUri.trim()) ?: return listOf(rawPubUri.trim())
        val rawHost = m.groupValues[1]
        val port = m.groupValues[2]
        val path = m.groupValues[3].ifEmpty { "/" }
        val rewrittenToMaster = "http://$masterHost:$port$path"

        val isRoutableIpv4 = IPV4_REGEX.matches(rawHost) && !rawHost.startsWith("127.") && rawHost != "0.0.0.0"
        return if (isRoutableIpv4) {
            if (rawHost == masterHost) listOf(rawPubUri.trim()) else listOf(rawPubUri.trim(), rewrittenToMaster)
        } else {
            // 非纯 IPv4 主机名（如 nuc12 / nuc12.local / zhb_uav / localhost）优先重写为 masterHost
            listOf(rewrittenToMaster, rawPubUri.trim()).distinct()
        }
    }

    private fun buildCandidateTcpHosts(rawHost: String, masterHost: String): List<String> {
        val clean = rawHost.trim()
        val isRoutableIpv4 = IPV4_REGEX.matches(clean) && !clean.startsWith("127.") && clean != "0.0.0.0"
        return if (isRoutableIpv4) {
            if (clean == masterHost) listOf(clean) else listOf(clean, masterHost)
        } else {
            if (clean.isEmpty() || clean == "localhost" || clean.startsWith("127.") || clean == "0.0.0.0") {
                listOf(masterHost)
            } else {
                listOf(masterHost, clean).distinct()
            }
        }
    }

    private fun normalizeHttpUri(raw: String, defaultPort: Int): String {
        var s = raw.trim().removeSuffix("/")
        if (!s.startsWith("http://", ignoreCase = true) && !s.startsWith("https://", ignoreCase = true)) {
            s = "http://$s"
        }
        val afterScheme = s.substringAfter("://")
        if (!afterScheme.contains(':')) {
            s = "$s:$defaultPort"
        }
        return s
    }

    private fun extractHostFromUri(httpUri: String): String? {
        val s = httpUri.trim().substringAfter("://", "")
        if (s.isEmpty()) return null
        return s.substringBefore('/').substringBefore(':').trim().takeIf { it.isNotEmpty() }
    }

    private fun extractHostFromWsUrl(wsUrl: String): String? {
        val s = wsUrl.trim().substringAfter("://", "")
        if (s.isEmpty()) return null
        return s.substringBefore('/').substringBefore(':').trim().takeIf { it.isNotEmpty() }
    }

    private fun encodeTcpRosHeader(fields: List<String>): ByteArray {
        val encoded = fields.map { it.toByteArray(Charsets.UTF_8) }
        val payloadLen = encoded.sumOf { 4 + it.size }
        val bb = ByteBuffer.allocate(4 + payloadLen).order(ByteOrder.LITTLE_ENDIAN)
        bb.putInt(payloadLen)
        for (f in encoded) {
            bb.putInt(f.size)
            bb.put(f)
        }
        return bb.array()
    }

    private fun decodeTcpRosHeader(bytes: ByteArray, length: Int): Map<String, String> {
        val map = HashMap<String, String>()
        val bb = ByteBuffer.wrap(bytes, 0, length).order(ByteOrder.LITTLE_ENDIAN)
        while (bb.remaining() >= 4) {
            val fLen = bb.int
            if (fLen <= 0 || bb.remaining() < fLen) break
            val pos = bb.position()
            val str = String(bytes, pos, fLen, Charsets.UTF_8)
            bb.position(pos + fLen)
            val eq = str.indexOf('=')
            if (eq > 0) map[str.substring(0, eq)] = str.substring(eq + 1)
        }
        return map
    }

    private fun xmlRpcCall(uri: String, method: String, stringArgs: List<String>): String {
        val xml = buildString {
            append("<?xml version=\"1.0\"?><methodCall><methodName>$method</methodName><params>")
            for (arg in stringArgs) {
                append("<param><value><string>${escapeXml(arg)}</string></value></param>")
            }
            append("</params></methodCall>")
        }
        return postXml(uri, xml)
    }

    private fun postXml(uri: String, xmlBody: String): String {
        val conn = (URL(uri).openConnection() as HttpURLConnection).apply {
            requestMethod = "POST"
            connectTimeout = 2800
            readTimeout = 3500
            doOutput = true
            setRequestProperty("Content-Type", "text/xml")
            setRequestProperty("Connection", "close")
        }
        return try {
            val bytes = xmlBody.toByteArray(Charsets.UTF_8)
            conn.outputStream.use { it.write(bytes) }
            conn.inputStream.bufferedReader(Charsets.UTF_8).use { it.readText() }
        } finally {
            conn.disconnect()
        }
    }

    private fun detectLocalIp(remoteHost: String): String {
        try {
            Socket().use { probe ->
                probe.connect(InetSocketAddress(remoteHost, 11311), 1200)
                val addr = probe.localAddress?.hostAddress
                if (!addr.isNullOrBlank() && addr != "127.0.0.1") return addr
            }
        } catch (_: Throwable) {}
        try {
            val interfaces = NetworkInterface.getNetworkInterfaces()
            while (interfaces.hasMoreElements()) {
                val nif = interfaces.nextElement()
                if (!nif.isUp || nif.isLoopback) continue
                val addrs = nif.inetAddresses
                while (addrs.hasMoreElements()) {
                    val a = addrs.nextElement()
                    if (a is Inet4Address && !a.isLoopbackAddress) return a.hostAddress ?: continue
                }
            }
        } catch (_: Throwable) {}
        return "127.0.0.1"
    }

    private fun nowTimeStr(): String =
        SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(Date())

    private fun escapeXml(s: String): String =
        s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
}
