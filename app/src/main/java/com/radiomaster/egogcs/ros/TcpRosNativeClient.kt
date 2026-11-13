package com.radiomaster.egogcs.ros

import com.radiomaster.egogcs.engine.EgoSimEngine
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.SwarmPalette
import com.radiomaster.egogcs.model.TopicConfig
import java.io.BufferedInputStream
import java.io.BufferedOutputStream
import java.io.DataInputStream
import java.io.EOFException
import java.net.HttpURLConnection
import java.net.Inet4Address
import java.net.InetSocketAddress
import java.net.NetworkInterface
import java.net.ServerSocket
import java.net.Socket
import java.net.URL
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.CopyOnWriteArrayList
import java.util.concurrent.Executors
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sin

class TcpRosNativeClient(
    private val hub: GcsSharedHub,
    private val engine: EgoSimEngine,
    private val onStateToast: (String) -> Unit
) : RosTransport {

    override val protocolId: String = "tcpros"

    companion object {
        private const val CALLER_ID = "/ego_pilot_android"
        private const val POSE_STAMPED_MD5 = "d3812c3cbc69362b77dc0b19b345f8f5"
        private const val VECTOR3_STAMPED_MD5 = "7b324c7325e683bf02a9b14b01090ec7"
    }

    private data class GoalSubClient(
        val remoteIp: String,
        val topic: String,
        val out: BufferedOutputStream
    )

    private class StreamScratch(initBytes: Int) {
        var frameBytes = ByteArray(initBytes)
        val cloudXyzi = FloatArray(8016 * 4)
        val inflateXyz = FloatArray(4096 * 3)
        val pathXyz = FloatArray(820 * 3)

        fun ensureFrameCapacity(required: Int): ByteArray {
            if (required > frameBytes.size) {
                var nextCap = frameBytes.size
                while (nextCap < required) nextCap = nextCap shl 1
                frameBytes = ByteArray(nextCap)
            }
            return frameBytes
        }
    }

    @Volatile private var active = false

    @Volatile var topics = TopicConfig()

    // 已建立实时数据流的核心话题登记表与零额外带宽旁路帧监听钩子 (供 RosTopicInspector 秒级直读)
    private val liveConnectedTopics = ConcurrentHashMap<String, Boolean>()
    private val startedSubKeys = ConcurrentHashMap<String, Boolean>()
    private val registeredPubKeys = ConcurrentHashMap<String, Boolean>()
    private val droneIndexToHostIp = ConcurrentHashMap<Int, String>()

    @Volatile private var activeLocalIp: String = "127.0.0.1"

    @Volatile private var activeMasterUri: String = ""

    @Volatile private var activeMasterHost: String = ""
    private val rxWindowBytes = java.util.concurrent.atomic.AtomicLong(0L)

    @Volatile private var lastRateCalcMs: Long = System.currentTimeMillis()

    @Volatile var watchedRawTopic: String? = null

    @Volatile var onWatchedRawFrame: ((topic: String, msgType: String, callerId: String, msgDef: String, buf: ByteArray, len: Int) -> Unit)? = null

    fun isTopicActivelyStreaming(topic: String): Boolean = active && liveConnectedTopics.containsKey(topic)

    private val ioPool = Executors.newCachedThreadPool { r ->
        Thread(r, "TcpRosNativeIO").apply { isDaemon = true }
    }
    private val openSockets = CopyOnWriteArrayList<Socket>()
    private var slaveRpcServer: ServerSocket? = null
    private var pubTcpServer: ServerSocket? = null
    private val goalSubscribers = CopyOnWriteArrayList<GoalSubClient>()
    private val goalPacketLock = Any()
    private val goalPacketBuf = ByteBuffer.allocate(128).order(ByteOrder.LITTLE_ENDIAN)
    private var goalSeq = 1

    private fun expandPattern(pattern: String, droneId: Int): String =
        pattern.trim().replace("{id}", droneId.toString())

    override fun connect(cfg: TopicConfig) {
        topics = cfg.copy()
        disconnect(silent = true)
        active = true

        hub.isRosMode = true
        hub.connectionLevel = 1
        hub.connectionLabel = "正在直连 roscore..."

        ioPool.execute {
            try {
                val peerIps = parsePeerIps(cfg.swarmPeerIpsCsv)
                val primaryMasterUri = if (hub.swarmModeEnabled && cfg.swarmTopology == 1 && peerIps.isNotEmpty()) {
                    normalizeMasterUri(peerIps[0])
                } else {
                    normalizeMasterUri(cfg.masterUri)
                }
                val masterHost = extractHostSimple(primaryMasterUri) ?: throw IllegalArgumentException("Master IP 无效")
                val localIp = detectLocalWifiIp(masterHost)
                activeLocalIp = localIp
                activeMasterUri = primaryMasterUri
                activeMasterHost = masterHost

                startGoalPublisherServer(localIp)
                registerPublisherOnce(primaryMasterUri, localIp, cfg.goalTopic)
                if (cfg.tiltCmdTopic.isNotBlank()) {
                    registerVector3PublisherOnce(primaryMasterUri, localIp, cfg.tiltCmdTopic.trim())
                }

                val pingXml = xmlRpcCall(primaryMasterUri, "getUri", listOf(CALLER_ID))
                if (!pingXml.contains("<int>1</int>") && !pingXml.contains("<i4>1</i4>")) {
                    throw IllegalStateException("roscore 未就绪")
                }

                hub.rosConnected = true
                hub.connectionLevel = 0
                hub.connectionLabel = "TCPROS · ${hub.formatRosModeSuffix()}"
                onStateToast(
                    if (hub.swarmModeEnabled && cfg.swarmTopology == 1 && peerIps.size > 1) {
                        "已直连多机独立 roscore 集群 (${peerIps.size} 台)"
                    } else {
                        "已通过原生 TCPROS 直连 $primaryMasterUri"
                    }
                )

                // 订阅 0 号主机核心感知与建图流
                droneIndexToHostIp[0] = masterHost
                launchSubOnce(primaryMasterUri, masterHost, localIp, cfg.odomTopic.trim(), "odom", 0)
                if (!cfg.udpTelemetryEnabled) {
                    launchSubOnce(primaryMasterUri, masterHost, localIp, cfg.fastLioCloudTopic.trim(), "fastlio_cloud", 0)
                }
                if (cfg.fastLioPathTopic.isNotBlank() && cfg.fastLioPathTopic.trim() != cfg.optimalTopic.trim()) {
                    launchSubOnce(primaryMasterUri, masterHost, localIp, cfg.fastLioPathTopic.trim(), "fastlio_path", 0)
                }
                launchSubOnce(primaryMasterUri, masterHost, localIp, cfg.optimalTopic.trim(), "optimal", 0)
                if (cfg.inflateTopic.isNotBlank() && cfg.inflateTopic.trim() != cfg.fastLioCloudTopic.trim()) {
                    launchSubOnce(primaryMasterUri, masterHost, localIp, cfg.inflateTopic.trim(), "inflate", 0)
                }
                launchSubOnce(primaryMasterUri, masterHost, localIp, "/mavros/state", "state", 0)
                launchSubOnce(primaryMasterUri, masterHost, localIp, "/mavros/battery", "bat", 0)
                if (cfg.tiltTelemetryTopic.isNotBlank()) {
                    launchSubOnce(primaryMasterUri, masterHost, localIp, cfg.tiltTelemetryTopic.trim(), "tilt", 0)
                }
                if (cfg.tiltTelemetryTopic.trim() != "/mavros/actuator_control") {
                    launchSubOnce(primaryMasterUri, masterHost, localIp, "/mavros/actuator_control", "tilt_act", 0)
                }

                if (hub.swarmModeEnabled) {
                    startSwarmStreamsInternal(cfg, primaryMasterUri, masterHost, localIp)
                }
                // v3.3: 启动 Master 心跳看门狗与实时吞吐率统计协程，机载 roscore 重启后自动重连恢复
                launchMasterWatchdogAndThroughputLoop(cfg)
            } catch (e: Exception) {
                if (active) {
                    hub.rosConnected = false
                    hub.connectionLevel = 2
                    hub.connectionLabel = "重连等待 roscore..."
                    onStateToast("TCPROS 首次握手未就绪 (${e.localizedMessage ?: "等待机载网络"}) · 后台自动重连中")
                    scheduleExponentialMasterReconnect(cfg, attempt = 1)
                }
            }
        }
    }

    /**
     * 当用户在已连接实机状态下动态开启多机模式或增减飞机数量时，无感补齐多机话题订阅与目标发布器。
     */
    fun ensureSwarmSubscriptionsIfActive() {
        if (!active || !hub.rosConnected) return
        val cfg = topics.copy()
        hub.connectionLabel = "TCPROS · ${hub.formatRosModeSuffix()}"
        val mUri = activeMasterUri.ifBlank { normalizeMasterUri(cfg.masterUri) }
        val mHost = activeMasterHost.ifBlank { extractHostSimple(mUri) ?: return }
        val lIp = activeLocalIp
        ioPool.execute {
            if (hub.isDroneTiltRig(0)) {
                if (cfg.tiltCmdTopic.isNotBlank()) {
                    registerVector3PublisherOnce(mUri, lIp, cfg.tiltCmdTopic.trim())
                }
                if (cfg.tiltTelemetryTopic.isNotBlank()) {
                    launchSubOnce(mUri, mHost, lIp, cfg.tiltTelemetryTopic.trim(), "tilt", 0)
                }
            }
            if (hub.swarmModeEnabled) {
                startSwarmStreamsInternal(cfg, mUri, mHost, lIp)
            }
        }
    }

    private fun startSwarmStreamsInternal(cfg: TopicConfig, primaryMasterUri: String, primaryHost: String, localIp: String) {
        val count = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
        if (cfg.swarmTopology == 1) {
            // 拓扑 1：多机各自运行独立 roscore (:11311)，并行连接每台机载电脑 IP
            val ips = parsePeerIps(cfg.swarmPeerIpsCsv)
            for (id in 0 until minOf(count, ips.size)) {
                val peerUri = normalizeMasterUri(ips[id])
                val peerHost = extractHostSimple(peerUri) ?: continue
                droneIndexToHostIp[id] = peerHost
                val peerLocalIp = detectLocalWifiIp(peerHost)
                registerPublisherOnce(peerUri, peerLocalIp, cfg.goalTopic.trim())
                launchSubOnce(peerUri, peerHost, peerLocalIp, cfg.odomTopic.trim(), "odom", id)
                launchSubOnce(peerUri, peerHost, peerLocalIp, cfg.optimalTopic.trim(), "optimal", id)
                launchSubOnce(peerUri, peerHost, peerLocalIp, "/mavros/battery", "bat", id)
                if (hub.isDroneTiltRig(id)) {
                    if (cfg.tiltCmdTopic.isNotBlank()) {
                        registerVector3PublisherOnce(peerUri, peerLocalIp, cfg.tiltCmdTopic.trim())
                    }
                    if (cfg.tiltTelemetryTopic.isNotBlank()) {
                        launchSubOnce(peerUri, peerHost, peerLocalIp, cfg.tiltTelemetryTopic.trim(), "tilt", id)
                    }
                }
            }
        } else {
            // 拓扑 0：同 Master 多命名空间 (ZJU-FAST-Lab/ego-planner-swarm 标准 drone_0..N 架构，兼容同构/异构倾转子话题)
            for (id in 0 until count) {
                droneIndexToHostIp[id] = primaryHost
                val odMain = expandPattern(cfg.swarmOdomPattern, id)
                val odAlt = "/drone_$id/Odometry"
                val optTopic = expandPattern(cfg.swarmOptimalPattern, id)
                val goalTopic = expandPattern(cfg.swarmGoalPattern, id)
                registerPublisherOnce(primaryMasterUri, localIp, goalTopic)
                launchSubOnce(primaryMasterUri, primaryHost, localIp, odMain, "odom", id)
                if (odAlt != odMain) {
                    launchSubOnce(primaryMasterUri, primaryHost, localIp, odAlt, "odom", id)
                }
                launchSubOnce(primaryMasterUri, primaryHost, localIp, optTopic, "optimal", id)
                if (hub.isDroneTiltRig(id)) {
                    val tiltSub = expandPattern(cfg.swarmTiltSubPattern, id)
                    val tiltPub = expandPattern(cfg.swarmTiltCmdPattern, id)
                    if (tiltPub.isNotBlank()) registerVector3PublisherOnce(primaryMasterUri, localIp, tiltPub)
                    if (tiltSub.isNotBlank()) launchSubOnce(primaryMasterUri, primaryHost, localIp, tiltSub, "tilt", id)
                }
            }
        }
    }

    private fun launchSubOnce(masterUri: String, masterHost: String, localIp: String, topic: String, kind: String, droneId: Int) {
        if (topic.isBlank()) return
        val key = "$masterUri|$topic|$kind|$droneId"
        if (startedSubKeys.putIfAbsent(key, true) == null) {
            ioPool.execute { subscribeTopicLoop(masterUri, masterHost, localIp, topic, kind, droneId) }
        }
    }

    private fun registerPublisherOnce(masterUri: String, localIp: String, goalTopic: String) {
        if (goalTopic.isBlank()) return
        val rpcPort = slaveRpcServer?.localPort ?: return
        val key = "$masterUri|$goalTopic"
        if (registeredPubKeys.putIfAbsent(key, true) == null) {
            runCatching {
                xmlRpcCall(masterUri, "registerPublisher", listOf(CALLER_ID, goalTopic, "geometry_msgs/PoseStamped", "http://$localIp:$rpcPort/"))
            }
        }
    }

    private fun registerVector3PublisherOnce(masterUri: String, localIp: String, vecTopic: String) {
        if (vecTopic.isBlank()) return
        val rpcPort = slaveRpcServer?.localPort ?: return
        val key = "$masterUri|$vecTopic|vec3"
        if (registeredPubKeys.putIfAbsent(key, true) == null) {
            runCatching {
                xmlRpcCall(masterUri, "registerPublisher", listOf(CALLER_ID, vecTopic, "geometry_msgs/Vector3Stamped", "http://$localIp:$rpcPort/"))
            }
        }
    }

    private fun parsePeerIps(csv: String): List<String> =
        csv.split(',', ';', ' ', '\n')
            .map { it.trim() }
            .filter { it.isNotEmpty() }
            .distinct()
            .take(SwarmPalette.MAX_SWARM_DRONES)

    private fun scheduleExponentialMasterReconnect(cfg: TopicConfig, attempt: Int) {
        if (!active) return
        val delayMs = (1500L * (1 shl attempt.coerceIn(0, 3))).coerceAtMost(8000L)
        ioPool.execute {
            try {
                Thread.sleep(delayMs)
            } catch (_: InterruptedException) {
                return@execute
            }
            if (!active || hub.rosConnected || !hub.isRosMode) return@execute
            hub.rosReconnectCount += 1
            connect(cfg)
        }
    }

    private fun launchMasterWatchdogAndThroughputLoop(cfg: TopicConfig) {
        ioPool.execute {
            var consecutiveMasterFails = 0
            lastRateCalcMs = System.currentTimeMillis()
            rxWindowBytes.set(0L)
            while (active) {
                try {
                    Thread.sleep(2000L)
                } catch (_: InterruptedException) {
                    break
                }
                if (!active) break
                val now = System.currentTimeMillis()
                val dtSec = ((now - lastRateCalcMs).coerceAtLeast(200L)) * 0.001f
                val bytes = rxWindowBytes.getAndSet(0L)
                val instKbps = (bytes / 1024.0f) / dtSec
                hub.rosRxKbps = if (hub.rosRxKbps <= 0.1f) instKbps else (hub.rosRxKbps * 0.6f + instKbps * 0.4f)
                lastRateCalcMs = now

                val mUri = activeMasterUri
                if (mUri.isNotBlank()) {
                    val ok = runCatching {
                        val pong = xmlRpcCall(mUri, "getUri", listOf(CALLER_ID))
                        pong.contains("<int>1</int>") || pong.contains("<i4>1</i4>")
                    }.getOrDefault(false)
                    if (!ok) {
                        consecutiveMasterFails++
                        if (consecutiveMasterFails >= 3 && active) {
                            hub.rosConnected = false
                            hub.connectionLevel = 1
                            hub.connectionLabel = "链路重连中 (#${hub.rosReconnectCount + 1})"
                            scheduleExponentialMasterReconnect(cfg, attempt = 1)
                            break
                        }
                    } else {
                        consecutiveMasterFails = 0
                        if (!hub.rosConnected) {
                            hub.rosConnected = true
                            hub.connectionLevel = 0
                            hub.connectionLabel = "TCPROS · ${hub.formatRosModeSuffix()}"
                        }
                    }
                }
            }
        }
    }

    override fun disconnect(silent: Boolean) {
        active = false
        hub.rosRxKbps = 0f
        liveConnectedTopics.clear()
        startedSubKeys.clear()
        registeredPubKeys.clear()
        droneIndexToHostIp.clear()
        watchedRawTopic = null
        hub.rosConnected = false
        for (s in openSockets) {
            try {
                s.close()
            } catch (_: Throwable) {}
        }
        openSockets.clear()
        goalSubscribers.clear()
        try {
            slaveRpcServer?.close()
        } catch (_: Throwable) {}
        try {
            pubTcpServer?.close()
        } catch (_: Throwable) {}
        slaveRpcServer = null
        pubTcpServer = null

        if (!silent) {
            if (hub.isRosMode) {
                hub.connectionLevel = 2
                hub.connectionLabel = "TCPROS 已断开"
            } else {
                hub.connectionLevel = 0
                hub.connectionLabel = "正常飞行 (DEMO)"
            }
        }
    }

    override fun setParamAsync(paramName: String, value: Float) {
        if (!active || !hub.rosConnected) return
        val masterUri = topics.masterUri.trim().removeSuffix("/")
        ioPool.execute {
            try {
                val body = buildString {
                    append("<?xml version=\"1.0\"?><methodCall><methodName>setParam</methodName><params>")
                    append("<param><value><string>$CALLER_ID</string></value></param>")
                    append("<param><value><string>${escapeXml(paramName)}</string></value></param>")
                    append("<param><value><double>$value</double></value></param>")
                    append("</params></methodCall>")
                }
                postXml(masterUri, body)
            } catch (_: Throwable) {}
        }
    }

    override fun publishTiltCommand(pitchBiasDeg: Float, coordMode: Int, maxTiltDeg: Float) {
        if (!active || !hub.anyActiveDroneUsesTilt() || goalSubscribers.isEmpty()) return
        val cfg = topics
        val targetTopics = HashSet<String>()
        val targetIps = HashSet<String>()
        if (!hub.swarmModeEnabled) {
            if (cfg.tiltCmdTopic.isNotBlank()) targetTopics.add(cfg.tiltCmdTopic.trim())
        } else {
            val cnt = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
            val ids = if (hub.swarmCommandScope == 1) (0 until cnt).toList() else listOf(hub.selectedDroneId.coerceIn(0, cnt - 1))
            for (id in ids) {
                if (!hub.isDroneTiltRig(id)) continue
                if (cfg.swarmTopology == 1) {
                    droneIndexToHostIp[id]?.let { targetIps.add(it) }
                    if (cfg.tiltCmdTopic.isNotBlank()) targetTopics.add(cfg.tiltCmdTopic.trim())
                } else {
                    if (cfg.swarmTiltCmdPattern.contains("{id}")) {
                        targetTopics.add(expandPattern(cfg.swarmTiltCmdPattern, id))
                    }
                    if (id == 0 && cfg.tiltCmdTopic.isNotBlank()) {
                        targetTopics.add(cfg.tiltCmdTopic.trim())
                    }
                }
            }
        }
        if (targetTopics.isEmpty()) return

        ioPool.execute {
            synchronized(goalPacketLock) {
                val frameBytes = "base_link".toByteArray(Charsets.US_ASCII)
                val payloadLen = 16 + frameBytes.size + 24
                goalPacketBuf.clear()
                goalPacketBuf.putInt(payloadLen)
                goalPacketBuf.putInt(goalSeq++)
                val nowMs = System.currentTimeMillis()
                goalPacketBuf.putInt((nowMs / 1000L).toInt())
                goalPacketBuf.putInt(((nowMs % 1000L) * 1_000_000L).toInt())
                goalPacketBuf.putInt(frameBytes.size)
                goalPacketBuf.put(frameBytes)
                goalPacketBuf.putDouble(pitchBiasDeg.toDouble())
                goalPacketBuf.putDouble(coordMode.toDouble())
                goalPacketBuf.putDouble(maxTiltDeg.toDouble())
                val totalBytes = 4 + payloadLen
                val packetCopy = goalPacketBuf.array()
                for (sub in goalSubscribers) {
                    val ipOk = targetIps.isEmpty() || sub.remoteIp in targetIps
                    if (ipOk && sub.topic in targetTopics) {
                        try {
                            synchronized(sub.out) {
                                sub.out.write(packetCopy, 0, totalBytes)
                                sub.out.flush()
                            }
                        } catch (_: Throwable) {
                            goalSubscribers.remove(sub)
                        }
                    }
                }
            }
        }
    }

    override fun publishGoal(gx: Float, gy: Float, gz: Float, curX: Float, curY: Float) {
        publishDroneGoal(0, gx, gy, gz, curX, curY)
    }

    override fun publishDroneGoal(droneId: Int, gx: Float, gy: Float, gz: Float, curX: Float, curY: Float) {
        if (!active || goalSubscribers.isEmpty()) return
        val cfg = topics
        val wantIpFilter = if (hub.swarmModeEnabled && cfg.swarmTopology == 1) {
            droneIndexToHostIp[droneId]
        } else {
            null
        }
        val wantTopic = if (hub.swarmModeEnabled && cfg.swarmTopology == 0 && cfg.swarmGoalPattern.contains("{id}")) {
            expandPattern(cfg.swarmGoalPattern, droneId)
        } else {
            cfg.goalTopic.trim()
        }

        ioPool.execute {
            synchronized(goalPacketLock) {
                val frameBytes = "world".toByteArray(Charsets.US_ASCII)
                val payloadLen = 16 + frameBytes.size + 56
                goalPacketBuf.clear()
                goalPacketBuf.putInt(payloadLen)
                goalPacketBuf.putInt(goalSeq++)
                val nowMs = System.currentTimeMillis()
                goalPacketBuf.putInt((nowMs / 1000L).toInt())
                goalPacketBuf.putInt(((nowMs % 1000L) * 1_000_000L).toInt())
                goalPacketBuf.putInt(frameBytes.size)
                goalPacketBuf.put(frameBytes)

                val yaw = atan2((gy - curY).toDouble(), (gx - curX).toDouble())
                goalPacketBuf.putDouble(gx.toDouble())
                goalPacketBuf.putDouble(gy.toDouble())
                goalPacketBuf.putDouble(gz.toDouble())
                goalPacketBuf.putDouble(0.0)
                goalPacketBuf.putDouble(0.0)
                goalPacketBuf.putDouble(sin(yaw * 0.5))
                goalPacketBuf.putDouble(cos(yaw * 0.5))
                val totalBytes = 4 + payloadLen
                val packetCopy = goalPacketBuf.array()

                var sentAny = false
                for (sub in goalSubscribers) {
                    val ipMatch = (wantIpFilter == null || sub.remoteIp == wantIpFilter)
                    val topicMatch = (sub.topic.isEmpty() || sub.topic == wantTopic || (droneId == 0 && sub.topic == cfg.goalTopic.trim()))
                    if (ipMatch && topicMatch) {
                        try {
                            synchronized(sub.out) {
                                sub.out.write(packetCopy, 0, totalBytes)
                                sub.out.flush()
                                sentAny = true
                            }
                        } catch (_: Throwable) {
                            goalSubscribers.remove(sub)
                        }
                    }
                }
                // 若节点均监听在共享 /move_base_simple/goal 上且未匹配独立子话题，兜底回退至默认目标流
                if (!sentAny && wantIpFilter == null && droneId == 0) {
                    for (sub in goalSubscribers) {
                        if (sub.topic == cfg.goalTopic.trim()) {
                            runCatching {
                                synchronized(sub.out) {
                                    sub.out.write(packetCopy, 0, totalBytes)
                                    sub.out.flush()
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    private fun subscribeTopicLoop(masterUri: String, masterHost: String, localIp: String, topic: String, kind: String, droneId: Int) {
        val initSize = if (kind == "fastlio_cloud" || kind == "inflate") 262_144 else 4096
        val scratch = StreamScratch(initSize)

        while (active) {
            try {
                val slavePort = slaveRpcServer?.localPort ?: 0
                val callerApi = "http://$localIp:$slavePort/"
                val regResp = xmlRpcCall(masterUri, "registerSubscriber", listOf(CALLER_ID, topic, "*", callerApi))
                val pubUris = extractHttpUris(regResp)

                if (pubUris.isEmpty()) {
                    Thread.sleep(1500L)
                    continue
                }

                val reqBody = buildString {
                    append("<?xml version=\"1.0\"?><methodCall><methodName>requestTopic</methodName><params>")
                    append("<param><value><string>$CALLER_ID</string></value></param>")
                    append("<param><value><string>${escapeXml(topic)}</string></value></param>")
                    append("<param><value><array><data><value><array><data><value><string>TCPROS</string></value></data></array></value></data></array></value></param>")
                    append("</params></methodCall>")
                }
                var endpoint: Pair<String, Int>? = null
                for (rawPub in pubUris) {
                    val pubUri = rewriteHostIfLocal(rawPub, masterHost)
                    val resp = runCatching { postXml(pubUri, reqBody) }.getOrNull()
                        ?: if (pubUri != rawPub) runCatching { postXml(rawPub, reqBody) }.getOrNull() else null
                    if (resp != null) {
                        endpoint = parseRequestTopicEndpoint(resp, masterHost)
                        if (endpoint != null) break
                    }
                }
                if (endpoint == null) {
                    Thread.sleep(1500L)
                    continue
                }

                streamTcpRosTopic(endpoint.first, endpoint.second, topic, kind, droneId, scratch)
            } catch (_: Throwable) {
                if (active) {
                    try {
                        Thread.sleep(1500L)
                    } catch (_: InterruptedException) {
                        break
                    }
                }
            }
        }
    }

    private fun streamTcpRosTopic(host: String, port: Int, topic: String, kind: String, droneId: Int, scratch: StreamScratch) {
        val sock = Socket()
        openSockets.add(sock)
        try {
            sock.tcpNoDelay = true
            sock.keepAlive = true
            // v3.3: 配置分级读取超时，彻底根除机载节点重启或林下瞬断导致的半开僵尸 Socket 永久阻塞
            sock.soTimeout = if (kind == "state" || kind == "bat" || kind == "fastlio_path") 14_000 else 6_500
            sock.receiveBufferSize = if (kind == "fastlio_cloud" || kind == "inflate") 2 * 1024 * 1024 else 512 * 1024
            sock.connect(InetSocketAddress(host, port), 3500)

            val out = BufferedOutputStream(sock.getOutputStream())
            val bufSz = if (kind == "fastlio_cloud" || kind == "inflate") 131072 else 32768
            val inp = DataInputStream(BufferedInputStream(sock.getInputStream(), bufSz))

            val hdr = encodeTcpRosHeader(
                listOf(
                    "callerid=$CALLER_ID",
                    "topic=$topic",
                    "md5sum=*",
                    "type=*",
                    "tcp_nodelay=1"
                )
            )
            out.write(hdr)
            out.flush()

            val hdrLen = readFrameIntoScratch(inp, scratch, maxFrame = 262_144)
            if (hdrLen <= 0) return
            val hdrFields = decodeTcpRosHeader(scratch.frameBytes, hdrLen)
            if (hdrFields["error"] != null) return
            val msgType = hdrFields["type"] ?: ""
            val msgMd5 = hdrFields["md5sum"] ?: ""
            val pubCallerId = hdrFields["callerid"] ?: host
            val msgDef = hdrFields["message_definition"] ?: ""
            val isNoeticBattery = msgMd5.startsWith("4ddae")
            liveConnectedTopics[topic] = true

            var lastInflateMs = 0L
            while (active && !sock.isClosed) {
                val frameLen = readFrameIntoScratch(inp, scratch, maxFrame = 16 * 1024 * 1024)
                if (frameLen < 0) break
                rxWindowBytes.addAndGet((frameLen + 4).toLong())
                val buf = scratch.frameBytes
                if (topic == watchedRawTopic) {
                    onWatchedRawFrame?.invoke(topic, msgType, pubCallerId, msgDef, buf, frameLen)
                }
                when (kind) {
                    "odom" -> decodeNativeOdometry(buf, frameLen, droneId)
                    "fastlio_cloud" -> decodeNativeFastLioCloud(buf, frameLen, scratch)
                    "fastlio_path" -> decodeNativePath(buf, frameLen, scratch, isGlobalLioPath = true, droneId = droneId)
                    "optimal" -> {
                        if (msgType.contains("Path")) {
                            decodeNativePath(buf, frameLen, scratch, isGlobalLioPath = false, droneId = droneId)
                        } else {
                            decodeNativeMarker(buf, frameLen, scratch, droneId = droneId)
                        }
                    }
                    "inflate" -> {
                        val now = System.currentTimeMillis()
                        if (now - lastInflateMs >= 80L) {
                            lastInflateMs = now
                            decodeNativeInflateCloud(buf, frameLen, scratch)
                        }
                    }
                    "state" -> if (droneId == 0) decodeNativeMavrosState(buf, frameLen)
                    "bat" -> decodeNativeBattery(buf, frameLen, isNoeticBattery, droneId)
                    "tilt", "tilt_act" -> decodeNativeTiltTelemetry(msgType, buf, frameLen, droneId, kind == "tilt_act")
                }
            }
        } finally {
            liveConnectedTopics.remove(topic)
            openSockets.remove(sock)
            try {
                sock.close()
            } catch (_: Throwable) {}
        }
    }

    private fun skipStdHeader(bb: ByteBuffer): Boolean {
        if (bb.remaining() < 16) return false
        bb.position(bb.position() + 12)
        val frameLen = bb.int
        if (frameLen < 0 || bb.remaining() < frameLen) return false
        bb.position(bb.position() + frameLen)
        return true
    }

    private fun skipRosString(bb: ByteBuffer): Boolean {
        if (bb.remaining() < 4) return false
        val len = bb.int
        if (len < 0 || bb.remaining() < len) return false
        bb.position(bb.position() + len)
        return true
    }

    private fun readRosString(bb: ByteBuffer): String? {
        if (bb.remaining() < 4) return null
        val len = bb.int
        if (len < 0 || bb.remaining() < len) return null
        val start = bb.position()
        val str = String(bb.array(), bb.arrayOffset() + start, len, Charsets.UTF_8)
        bb.position(start + len)
        return str
    }

    private fun decodeNativeOdometry(bytes: ByteArray, length: Int, droneId: Int) {
        if (droneId == 0 && topics.udpTelemetryEnabled && hub.hasFreshUdpOdometry(android.os.SystemClock.elapsedRealtime())) return
        val bb = ByteBuffer.wrap(bytes, 0, length).order(ByteOrder.LITTLE_ENDIAN)
        if (!skipStdHeader(bb)) return
        if (!skipRosString(bb)) return
        if (bb.remaining() < 344) return
        val px = bb.double.toFloat()
        val py = bb.double.toFloat()
        val pz = bb.double.toFloat()
        val qx = bb.double
        val qy = bb.double
        val qz = bb.double
        val qw = bb.double
        bb.position(bb.position() + 288)

        var vx = 0f
        var vy = 0f
        var vz = 0f
        if (bb.remaining() >= 24) {
            vx = bb.double.toFloat()
            vy = bb.double.toFloat()
            vz = bb.double.toFloat()
        }
        val nrm = kotlin.math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        val nx = if (nrm > 1e-6) qx / nrm else 0.0
        val ny = if (nrm > 1e-6) qy / nrm else 0.0
        val nz = if (nrm > 1e-6) qz / nrm else 0.0
        val nw = if (nrm > 1e-6) qw / nrm else 1.0
        val roll = atan2(2.0 * (nw * nx + ny * nz), 1.0 - 2.0 * (nx * nx + ny * ny)).toFloat()
        val sinp = (2.0 * (nw * ny - nz * nx)).coerceIn(-1.0, 1.0)
        val pitch = (-kotlin.math.asin(sinp)).toFloat()
        val yaw = atan2(2.0 * (nw * nz + nx * ny), 1.0 - 2.0 * (ny * ny + nz * nz)).toFloat()
        val hasQuat = nrm > 0.2 && (kotlin.math.abs(nx) > 1e-5 || kotlin.math.abs(ny) > 1e-5 || kotlin.math.abs(nz) > 1e-5)
        if (droneId <= 0) {
            engine.onRosOdometry(
                px, py, pz, vx, vy, vz, yaw,
                roll, pitch, nw.toFloat(), nx.toFloat(), ny.toFloat(), nz.toFloat(), hasQuat
            )
        } else {
            engine.onRosSwarmOdometry(droneId, px, py, pz, vx, vy, vz, yaw, roll, pitch)
        }
    }

    private fun decodeNativeMarker(bytes: ByteArray, length: Int, scratch: StreamScratch, droneId: Int) {
        val bb = ByteBuffer.wrap(bytes, 0, length).order(ByteOrder.LITTLE_ENDIAN)
        if (!skipStdHeader(bb)) return
        if (!skipRosString(bb)) return
        if (bb.remaining() < 121) return
        bb.position(bb.position() + 117)
        val ptCount = bb.int
        if (ptCount <= 0 || ptCount > 10000 || bb.remaining() < ptCount * 24) return

        val stride = max(1, ptCount / 400)
        val flat = scratch.pathXyz
        var outCnt = 0
        for (i in 0 until ptCount) {
            val px = bb.double.toFloat()
            val py = bb.double.toFloat()
            val pz = bb.double.toFloat()
            if ((i % stride == 0 || i == ptCount - 1) && outCnt < 410) {
                flat[outCnt * 3] = px
                flat[outCnt * 3 + 1] = py
                flat[outCnt * 3 + 2] = pz
                outCnt++
            }
        }
        if (droneId <= 0) {
            engine.onRosOptimalPath(flat, outCnt)
        } else {
            engine.onRosSwarmOptimalPath(droneId, flat, outCnt)
        }
    }

    private fun decodeNativePath(bytes: ByteArray, length: Int, scratch: StreamScratch, isGlobalLioPath: Boolean, droneId: Int) {
        val bb = ByteBuffer.wrap(bytes, 0, length).order(ByteOrder.LITTLE_ENDIAN)
        if (!skipStdHeader(bb)) return
        if (bb.remaining() < 4) return
        val count = bb.int
        if (count <= 0 || count > 200_000) return

        val maxCap = if (isGlobalLioPath) 800 else 400
        val stride = max(1, count / maxCap)
        val flat = scratch.pathXyz
        var valid = 0
        for (i in 0 until count) {
            if (!skipStdHeader(bb)) break
            if (bb.remaining() < 56) break
            val px = bb.double.toFloat()
            val py = bb.double.toFloat()
            val pz = bb.double.toFloat()
            bb.position(bb.position() + 32)
            if ((i % stride == 0 || i == count - 1) && valid < 815) {
                flat[valid * 3] = px
                flat[valid * 3 + 1] = py
                flat[valid * 3 + 2] = pz
                valid++
            }
        }
        if (valid > 0) {
            if (isGlobalLioPath) {
                engine.onRosFastLioPath(flat, valid)
            } else if (droneId <= 0) {
                engine.onRosOptimalPath(flat, valid)
            } else {
                engine.onRosSwarmOptimalPath(droneId, flat, valid)
            }
        }
    }

    private fun decodeNativeFastLioCloud(bytes: ByteArray, length: Int, scratch: StreamScratch) {
        val bb = ByteBuffer.wrap(bytes, 0, length).order(ByteOrder.LITTLE_ENDIAN)
        if (!skipStdHeader(bb)) return
        if (bb.remaining() < 12) return
        bb.int
        bb.int
        val fieldCount = bb.int
        if (fieldCount < 0 || fieldCount > 64) return

        var offX = 0
        var offY = 4
        var offZ = 8
        var offInten = -1
        var intenType = 7
        for (f in 0 until fieldCount) {
            val name = readRosString(bb) ?: return
            if (bb.remaining() < 9) return
            val offset = bb.int
            val dtype = bb.get().toInt() and 0xFF
            bb.int
            when (name) {
                "x" -> offX = offset
                "y" -> offY = offset
                "z" -> offZ = offset
                "intensity", "reflectivity" -> {
                    offInten = offset
                    intenType = dtype
                }
            }
        }
        if (bb.remaining() < 13) return
        val bigEndian = bb.get().toInt() != 0
        val pointStep = bb.int
        bb.int
        val dataLen = bb.int
        if (pointStep < 12 || dataLen <= 0 || bb.remaining() < dataLen) return

        val dataStart = bb.position()
        val cloudView = ByteBuffer.wrap(bytes, dataStart, dataLen)
            .order(if (bigEndian) ByteOrder.BIG_ENDIAN else ByteOrder.LITTLE_ENDIAN)

        val totalPts = dataLen / pointStep
        val stride = max(1, totalPts / 8000)
        val xyzi = scratch.cloudXyzi
        var outCnt = 0
        var p = 0
        val reqTail = max(max(offX, max(offY, offZ)) + 4, if (offInten >= 0) offInten + 4 else 12)

        while (p < totalPts && outCnt < 8010) {
            val pos = dataStart + p * pointStep
            if (pos + reqTail > dataStart + dataLen) break
            val px = cloudView.getFloat(pos + offX)
            val py = cloudView.getFloat(pos + offY)
            val pz = cloudView.getFloat(pos + offZ)
            if (px.isFinite() && py.isFinite() && pz.isFinite()) {
                val normInten = if (offInten >= 0) {
                    val rawI = when (intenType) {
                        2 -> (cloudView.get(pos + offInten).toInt() and 0xFF) / 255f
                        4 -> (cloudView.getShort(pos + offInten).toInt() and 0xFFFF) / 65535f
                        else -> {
                            val fv = cloudView.getFloat(pos + offInten)
                            if (fv > 1.5f) (fv / 255f) else fv
                        }
                    }
                    rawI.coerceIn(0.05f, 1.0f)
                } else {
                    ((pz + 0.5f) / 3.5f).coerceIn(0.05f, 1.0f)
                }
                val dst = outCnt * 4
                xyzi[dst] = px
                xyzi[dst + 1] = py
                xyzi[dst + 2] = pz
                xyzi[dst + 3] = normInten
                outCnt++
            }
            p += stride
        }
        if (outCnt > 0) {
            engine.onRosFastLioCloud(xyzi, outCnt)
        }
    }

    private fun decodeNativeInflateCloud(bytes: ByteArray, length: Int, scratch: StreamScratch) {
        val bb = ByteBuffer.wrap(bytes, 0, length).order(ByteOrder.LITTLE_ENDIAN)
        if (!skipStdHeader(bb)) return
        if (bb.remaining() < 12) return
        bb.int
        bb.int
        val fieldCount = bb.int
        if (fieldCount < 0 || fieldCount > 64) return

        var offX = 0
        var offY = 4
        var offZ = 8
        for (f in 0 until fieldCount) {
            val name = readRosString(bb) ?: return
            if (bb.remaining() < 9) return
            val offset = bb.int
            bb.get()
            bb.int
            when (name) {
                "x" -> offX = offset
                "y" -> offY = offset
                "z" -> offZ = offset
            }
        }
        if (bb.remaining() < 13) return
        val bigEndian = bb.get().toInt() != 0
        val pointStep = bb.int
        bb.int
        val dataLen = bb.int
        if (pointStep < 12 || dataLen <= 0 || bb.remaining() < dataLen) return

        val dataStart = bb.position()
        val cloudView = ByteBuffer.wrap(bytes, dataStart, dataLen)
            .order(if (bigEndian) ByteOrder.BIG_ENDIAN else ByteOrder.LITTLE_ENDIAN)

        val totalPts = dataLen / pointStep
        val stride = max(1, totalPts / 4080)
        val flat = scratch.inflateXyz
        var outCnt = 0
        var p = 0
        val reqTail = max(offX, max(offY, offZ)) + 4
        while (p < totalPts && outCnt < 4090) {
            val pos = dataStart + p * pointStep
            if (pos + reqTail > dataStart + dataLen) break
            val px = cloudView.getFloat(pos + offX)
            val py = cloudView.getFloat(pos + offY)
            val pz = cloudView.getFloat(pos + offZ)
            if (px.isFinite() && py.isFinite() && pz.isFinite()) {
                flat[outCnt * 3] = px
                flat[outCnt * 3 + 1] = py
                flat[outCnt * 3 + 2] = pz
                outCnt++
            }
            p += stride
        }
        engine.onRosInflatedCloud(flat, outCnt)
    }

    /**
     * 自适应解析多种机载倾转/执行器 ROS1 二进制消息格式：
     * - std_msgs/Float32MultiArray 或 Float64MultiArray ([alpha1, alpha2] 或 [T1,T2,T3,T4,alpha1,alpha2])
     * - geometry_msgs/Vector3Stamped (x=alpha1, y=alpha2)
     * - sensor_msgs/JointState (position[0..1] 或最后两维舵机角)
     * - mavros_msgs/ActuatorControl (controls[0..3]=电机推力, controls[4..5]=倾转舵机 [-1,1])
     */
    private fun decodeNativeTiltTelemetry(
        msgType: String,
        bytes: ByteArray,
        length: Int,
        droneId: Int = 0,
        isActuatorFallback: Boolean = false
    ) {
        try {
            val bb = ByteBuffer.wrap(bytes, 0, length).order(ByteOrder.LITTLE_ENDIAN)
            val maxRad = hub.tiltMaxAngleDeg.coerceIn(15f, 60f) * 0.017453292f
            fun normAngleToRad(v: Float): Float =
                if (abs(v) > 1.35f) (v * 0.017453292f) else v
            fun emitTilt(s1: Float, s2: Float, m0: Float = -1f, m1: Float = -1f, m2: Float = -1f, m3: Float = -1f) {
                if (droneId <= 0) {
                    engine.onRosTiltTelemetry(s1, s2, m0, m1, m2, m3, isActuatorFallback)
                } else {
                    engine.onRosSwarmTiltTelemetry(droneId, s1, s2, m0, m1, m2, m3, isActuatorFallback)
                }
            }

            when {
                msgType.contains("ActuatorControl") -> {
                    if (!skipStdHeader(bb)) return
                    if (bb.remaining() < 1 + 8 * 4) return
                    val groupMix = bb.get().toInt() and 0xFF
                    if (groupMix != 0 && groupMix != 1) return
                    val c = FloatArray(8) { bb.float }
                    val s1 = (c[4].coerceIn(-1f, 1f)) * maxRad
                    val s2 = (c[5].coerceIn(-1f, 1f)) * maxRad
                    emitTilt(s1, s2, c[0], c[1], c[2], c[3])
                }
                msgType.contains("Float32MultiArray") -> {
                    if (bb.remaining() < 8) return
                    val dimCnt = bb.int
                    if (dimCnt < 0 || dimCnt > 16) return
                    for (i in 0 until dimCnt) {
                        if (!skipRosString(bb)) return
                        if (bb.remaining() < 8) return
                        bb.int
                        bb.int
                    }
                    if (bb.remaining() < 8) return
                    bb.int // data_offset
                    val n = bb.int
                    if (n < 2 || n > 64 || bb.remaining() < n * 4) return
                    val arr = FloatArray(n) { bb.float }
                    if (n >= 6) {
                        emitTilt(
                            normAngleToRad(arr[4]),
                            normAngleToRad(arr[5]),
                            arr[0],
                            arr[1],
                            arr[2],
                            arr[3]
                        )
                    } else {
                        emitTilt(normAngleToRad(arr[0]), normAngleToRad(arr[1]))
                    }
                }
                msgType.contains("Float64MultiArray") -> {
                    if (bb.remaining() < 8) return
                    val dimCnt = bb.int
                    if (dimCnt < 0 || dimCnt > 16) return
                    for (i in 0 until dimCnt) {
                        if (!skipRosString(bb)) return
                        if (bb.remaining() < 8) return
                        bb.int
                        bb.int
                    }
                    if (bb.remaining() < 8) return
                    bb.int
                    val n = bb.int
                    if (n < 2 || n > 64 || bb.remaining() < n * 8) return
                    val arr = FloatArray(n) { bb.double.toFloat() }
                    if (n >= 6) {
                        emitTilt(
                            normAngleToRad(arr[4]),
                            normAngleToRad(arr[5]),
                            arr[0],
                            arr[1],
                            arr[2],
                            arr[3]
                        )
                    } else {
                        emitTilt(normAngleToRad(arr[0]), normAngleToRad(arr[1]))
                    }
                }
                msgType.contains("Vector3Stamped") -> {
                    if (!skipStdHeader(bb) || bb.remaining() < 24) return
                    val s1 = normAngleToRad(bb.double.toFloat())
                    val s2 = normAngleToRad(bb.double.toFloat())
                    engine.onRosTiltTelemetry(s1, s2)
                }
                msgType.contains("JointState") -> {
                    if (!skipStdHeader(bb) || bb.remaining() < 4) return
                    val nameCnt = bb.int
                    if (nameCnt < 0 || nameCnt > 32) return
                    for (i in 0 until nameCnt) if (!skipRosString(bb)) return
                    if (bb.remaining() < 4) return
                    val posCnt = bb.int
                    if (posCnt < 2 || posCnt > 32 || bb.remaining() < posCnt * 8) return
                    val pos = FloatArray(posCnt) { bb.double.toFloat() }
                    val s1 = normAngleToRad(pos[if (posCnt >= 6) 4 else 0])
                    val s2 = normAngleToRad(pos[if (posCnt >= 6) 5 else 1])
                    engine.onRosTiltTelemetry(s1, s2)
                }
            }
        } catch (_: Throwable) {}
    }

    private fun decodeNativeMavrosState(bytes: ByteArray, length: Int) {
        val bb = ByteBuffer.wrap(bytes, 0, length).order(ByteOrder.LITTLE_ENDIAN)
        if (!skipStdHeader(bb)) return
        if (bb.remaining() < 5) return
        bb.get()
        val armed = bb.get().toInt() != 0
        bb.get()
        bb.get()
        val mode = readRosString(bb) ?: "OFFBOARD"
        engine.onRosMavrosState(mode, armed)
    }

    private fun decodeNativeBattery(bytes: ByteArray, length: Int, isNoetic: Boolean, droneId: Int) {
        val bb = ByteBuffer.wrap(bytes, 0, length).order(ByteOrder.LITTLE_ENDIAN)
        if (!skipStdHeader(bb)) return
        if (bb.remaining() < 12) return

        val basePos = bb.position()
        val volt = bb.getFloat(basePos)
        if (!volt.isFinite() || volt <= 1.0f) return

        val rawCurrent = if (isNoetic && bb.remaining() >= 12) {
            bb.getFloat(basePos + 8)
        } else {
            bb.getFloat(basePos + 4)
        }
        val amps = if (rawCurrent.isFinite()) abs(rawCurrent) else 0f

        val rawPctOffset = if (isNoetic) 24 else 20
        val pct = if (bb.remaining() >= rawPctOffset + 4) {
            val pVal = bb.getFloat(basePos + rawPctOffset)
            if (pVal.isFinite() && pVal > 0f) {
                (if (pVal <= 1.05f) pVal * 100f else pVal).roundToInt().coerceIn(0, 100)
            } else {
                -1
            }
        } else {
            -1
        }

        val finalPct = if (pct in 0..100) {
            pct
        } else {
            val cells = if (volt > 18.0f) 6f else 4f
            (((volt / cells - 3.50f) / (4.18f - 3.50f)) * 100f).roundToInt().coerceIn(0, 100)
        }
        if (droneId <= 0) {
            engine.onRosBattery(volt, finalPct, amps)
        } else {
            engine.onRosSwarmBattery(droneId, volt, finalPct)
        }
    }

    private fun startGoalPublisherServer(localIp: String) {
        val tcpSrv = ServerSocket(0)
        val rpcSrv = ServerSocket(0)
        pubTcpServer = tcpSrv
        slaveRpcServer = rpcSrv
        val tcpPort = tcpSrv.localPort

        ioPool.execute {
            val handshakeScratch = StreamScratch(4096)
            while (active && !tcpSrv.isClosed) {
                try {
                    val client = tcpSrv.accept()
                    client.tcpNoDelay = true
                    openSockets.add(client)
                    val remoteHostIp = client.inetAddress?.hostAddress ?: ""
                    ioPool.execute {
                        try {
                            val inp = DataInputStream(BufferedInputStream(client.getInputStream()))
                            val out = BufferedOutputStream(client.getOutputStream())
                            val hLen = readFrameIntoScratch(inp, handshakeScratch, 65536)
                            val reqMap = if (hLen > 0) decodeTcpRosHeader(handshakeScratch.frameBytes, hLen) else emptyMap()
                            val reqTopic = reqMap["topic"] ?: topics.goalTopic
                            val isTiltCmd = reqTopic.isNotEmpty() && reqTopic == topics.tiltCmdTopic.trim()
                            val reply = encodeTcpRosHeader(
                                listOf(
                                    "callerid=$CALLER_ID",
                                    "topic=$reqTopic",
                                    if (isTiltCmd) "type=geometry_msgs/Vector3Stamped" else "type=geometry_msgs/PoseStamped",
                                    if (isTiltCmd) "md5sum=$VECTOR3_STAMPED_MD5" else "md5sum=$POSE_STAMPED_MD5",
                                    "latching=1"
                                )
                            )
                            out.write(reply)
                            out.flush()
                            goalSubscribers.add(GoalSubClient(remoteHostIp, reqTopic, out))
                        } catch (_: Throwable) {}
                    }
                } catch (_: Throwable) {
                    break
                }
            }
        }

        ioPool.execute {
            while (active && !rpcSrv.isClosed) {
                try {
                    val sock = rpcSrv.accept()
                    ioPool.execute {
                        try {
                            val reqBuf = ByteArray(4096)
                            val n = sock.getInputStream().read(reqBuf)
                            val reqStr = if (n > 0) String(reqBuf, 0, n, Charsets.UTF_8) else ""
                            val xmlResp = if (reqStr.contains("requestTopic")) {
                                "<?xml version=\"1.0\"?><methodResponse><params><param><value><array><data>" +
                                    "<value><int>1</int></value><value><string>ready</string></value>" +
                                    "<value><array><data><value><string>TCPROS</string></value>" +
                                    "<value><string>$localIp</string></value>" +
                                    "<value><int>$tcpPort</int></value></data></array></value>" +
                                    "</data></array></value></param></params></methodResponse>"
                            } else {
                                "<?xml version=\"1.0\"?><methodResponse><params><param><value><array><data>" +
                                    "<value><int>1</int></value><value><string>ok</string></value><value><int>0</int></value>" +
                                    "</data></array></value></param></params></methodResponse>"
                            }
                            val bodyBytes = xmlResp.toByteArray(Charsets.UTF_8)
                            val httpHeader = "HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\nContent-Length: ${bodyBytes.size}\r\nConnection: close\r\n\r\n"
                            val out = sock.getOutputStream()
                            out.write(httpHeader.toByteArray(Charsets.UTF_8))
                            out.write(bodyBytes)
                            out.flush()
                        } catch (_: Throwable) {
                        } finally {
                            try {
                                sock.close()
                            } catch (_: Throwable) {}
                        }
                    }
                } catch (_: Throwable) {
                    break
                }
            }
        }
    }

    private fun readFrameIntoScratch(inp: DataInputStream, scratch: StreamScratch, maxFrame: Int): Int {
        val b0 = inp.read()
        if (b0 < 0) return -1
        val b1 = inp.read()
        val b2 = inp.read()
        val b3 = inp.read()
        if ((b1 or b2 or b3) < 0) throw EOFException()
        val len = (b0 and 0xFF) or ((b1 and 0xFF) shl 8) or ((b2 and 0xFF) shl 16) or ((b3 and 0xFF) shl 24)
        if (len < 0 || len > maxFrame) throw IllegalStateException("Frame size out of bounds: $len")
        val target = scratch.ensureFrameCapacity(len)
        inp.readFully(target, 0, len)
        return len
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
        val map = mutableMapOf<String, String>()
        val bb = ByteBuffer.wrap(bytes, 0, length).order(ByteOrder.LITTLE_ENDIAN)
        while (bb.remaining() >= 4) {
            val fLen = bb.int
            if (fLen <= 0 || bb.remaining() < fLen) break
            val start = bb.position()
            val str = String(bytes, start, fLen, Charsets.UTF_8)
            bb.position(start + fLen)
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
            connectTimeout = 3500
            readTimeout = 4000
            doOutput = true
            setRequestProperty("Content-Type", "text/xml")
        }
        return try {
            val bytes = xmlBody.toByteArray(Charsets.UTF_8)
            conn.outputStream.use { it.write(bytes) }
            conn.inputStream.bufferedReader(Charsets.UTF_8).use { it.readText() }
        } finally {
            conn.disconnect()
        }
    }

    private fun extractHttpUris(xml: String): List<String> {
        val regex = Regex("http://[^<\\s\"']+")
        return regex.findAll(xml).map { it.value }.toList()
    }

    private fun normalizeMasterUri(raw: String): String {
        var s = raw.trim().removeSuffix("/")
        if (!s.startsWith("http://", ignoreCase = true) && !s.startsWith("https://", ignoreCase = true)) {
            s = "http://$s"
        }
        if (!s.substringAfter("://").contains(':')) s = "$s:11311"
        return s
    }

    private fun extractHostSimple(httpUri: String): String? {
        val s = httpUri.trim().substringAfter("://", "")
        if (s.isEmpty()) return null
        return s.substringBefore('/').substringBefore(':').trim().takeIf { it.isNotEmpty() }
    }

    private fun isRoutableIpv4(h: String): Boolean {
        return Regex("^\\d{1,3}(?:\\.\\d{1,3}){3}$").matches(h) && !h.startsWith("127.") && h != "0.0.0.0"
    }

    private fun rewriteHostIfLocal(pubUri: String, masterHost: String): String {
        val m = Regex("^http://([^/:\\s]+):(\\d+)(/.*)?$", RegexOption.IGNORE_CASE).find(pubUri.trim())
            ?: return pubUri
        val h = m.groupValues[1]
        val port = m.groupValues[2]
        val path = m.groupValues[3].ifEmpty { "/" }
        return if (isRoutableIpv4(h)) pubUri else "http://$masterHost:$port$path"
    }

    private fun parseRequestTopicEndpoint(xml: String, fallbackHost: String): Pair<String, Int>? {
        val protoRegex = Regex(
            "<array>\\s*<data>\\s*<value>\\s*(?:<string>)?\\s*TCPROS\\s*(?:</string>)?\\s*</value>\\s*" +
                "<value>\\s*(?:<string>)?\\s*([^<]*?)\\s*(?:</string>)?\\s*</value>\\s*" +
                "<value>\\s*<(?:int|i4)>\\s*(\\d+)\\s*</(?:int|i4)>\\s*</value>\\s*</data>\\s*</array>",
            setOf(RegexOption.DOT_MATCHES_ALL, RegexOption.IGNORE_CASE)
        )
        protoRegex.find(xml)?.let { m ->
            val rawH = m.groupValues[1].trim()
            val port = m.groupValues[2].trim().toIntOrNull() ?: return null
            val host = if (isRoutableIpv4(rawH)) rawH else fallbackHost
            if (port in 1..65535) return host to port
        }

        val scalars = Regex("<value>\\s*(?:<(?:string|int|i4)>)?\\s*([^<]+?)\\s*(?:</(?:string|int|i4)>)?\\s*</value>")
            .findAll(xml)
            .map { it.groupValues[1].trim() }
            .toList()
        val tcpIdx = scalars.indexOfFirst { it.equals("TCPROS", ignoreCase = true) }
        if (tcpIdx >= 0 && tcpIdx + 2 < scalars.size) {
            val rawH = scalars[tcpIdx + 1]
            val port = scalars[tcpIdx + 2].toIntOrNull() ?: return null
            val host = if (isRoutableIpv4(rawH)) rawH else fallbackHost
            if (port in 1..65535) return host to port
        }
        return null
    }

    private fun detectLocalWifiIp(remoteHost: String): String {
        try {
            Socket().use { probe ->
                probe.connect(InetSocketAddress(remoteHost, 11311), 1500)
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

    private fun escapeXml(s: String): String =
        s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
}
