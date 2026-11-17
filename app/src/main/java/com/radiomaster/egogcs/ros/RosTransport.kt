package com.radiomaster.egogcs.ros

import com.radiomaster.egogcs.engine.EgoSimEngine
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.TopicConfig
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean

interface RosTransport {
    val protocolId: String
    fun connect(cfg: TopicConfig)
    fun disconnect(silent: Boolean = false)
    fun publishGoal(gx: Float, gy: Float, gz: Float, curX: Float, curY: Float)
    fun publishDroneGoal(droneId: Int, gx: Float, gy: Float, gz: Float, curX: Float, curY: Float) {
        publishGoal(gx, gy, gz, curX, curY)
    }
    fun setParamAsync(paramName: String, value: Float)
    fun publishTiltCommand(pitchBiasDeg: Float, coordMode: Int, maxTiltDeg: Float) {}
}

class RosConnectionManager(
    private val hub: GcsSharedHub,
    private val tcpRosTransport: TcpRosNativeClient,
    private val rosBridgeTransport: RosBridgeClient,
    private val udpTelemetry: RosUdpTelemetryClient
) {
    @Volatile var topics: TopicConfig = TopicConfig()

    /**
     * 当底层 TCPROS 或 rosbridge 连续退避重连未命中（例如路由器重连后机载电脑 DHCP IP 变更或桥接协议切换）时，
     * 自动上抛给寻机协调器启动局域网指数退避扫描。
     */
    @Volatile var onRequestDiscoveryRecovery: ((reason: DiscoveryRecoveryReason) -> Unit)? = null

    /**
     * 当底层传输层成功握手或自动自愈重连成功时回调机载主机地址，用于记忆高优先级候选 IP 并复位寻机退避。
     */
    @Volatile var onLinkConnected: ((remoteHost: String) -> Unit)? = null

    private val pendingParams = ConcurrentHashMap<String, Float>()
    private val flushScheduled = AtomicBoolean(false)
    private val debounceScheduler = Executors.newSingleThreadScheduledExecutor { r ->
        Thread(r, "RosParamDebouncer").apply { isDaemon = true }
    }

    init {
        val escalationHandler: (Int) -> Unit = { failedAttempts ->
            if (failedAttempts >= 2 && hub.isRosMode && !hub.rosConnected) {
                onRequestDiscoveryRecovery?.invoke(DiscoveryRecoveryReason.BRIDGE_DISCONNECTED)
            }
        }
        tcpRosTransport.onDisconnectEscalation = escalationHandler
        rosBridgeTransport.onDisconnectEscalation = escalationHandler

        val connectedHandler: (String) -> Unit = { host ->
            onLinkConnected?.invoke(host)
        }
        tcpRosTransport.onLinkConnected = connectedHandler
        rosBridgeTransport.onLinkConnected = connectedHandler
    }

    private fun activeTransport(): RosTransport =
        if (topics.protocol == "tcpros") tcpRosTransport else rosBridgeTransport

    fun connect(cfg: TopicConfig = topics) {
        udpTelemetry.connect(cfg)
        topics = cfg.copy()
        tcpRosTransport.topics = topics.copy()
        rosBridgeTransport.topics = topics.copy()

        if (topics.protocol == "tcpros") {
            rosBridgeTransport.disconnect(silent = true)
            tcpRosTransport.connect(topics)
        } else {
            tcpRosTransport.disconnect(silent = true)
            rosBridgeTransport.connect(topics)
        }
    }

    /**
     * 当物理 Wi-Fi / 有线网口从瞬断中恢复时调用：若当前处于实机模式但尚未连上，立即跳过长退避窗口重连。
     */
    fun onPhysicalLanRestored() {
        if (!hub.isRosMode || hub.rosConnected) return
        if (topics.protocol == "tcpros") {
            tcpRosTransport.triggerImmediateReconnectIfDisconnected()
        } else {
            rosBridgeTransport.triggerImmediateReconnectIfDisconnected()
        }
    }

    fun disconnectAll(silent: Boolean = false) {
        udpTelemetry.disconnect()
        pendingParams.clear()
        rosBridgeTransport.disconnect(silent = silent)
        tcpRosTransport.disconnect(silent = silent)
    }

    fun switchToDemoMode() {
        disconnectAll(silent = true)
        hub.isRosMode = false
        hub.rosConnected = false
        hub.connectionLevel = 0
        hub.connectionLabel = "仿真演练"
    }

    fun updateTopicsAndResubscribe(cfg: TopicConfig) {
        if (hub.isRosMode && hub.rosConnected && (topics.udpTelemetryEnabled != cfg.udpTelemetryEnabled || topics.udpPort != cfg.udpPort)) {
            connect(cfg)
            return
        }
        topics = cfg.copy()
        tcpRosTransport.topics = topics.copy()
        if (hub.isRosMode && hub.rosConnected) {
            if (topics.protocol == "tcpros") {
                tcpRosTransport.ensureSwarmSubscriptionsIfActive()
            } else {
                rosBridgeTransport.resubscribeIfConnected(topics)
            }
        }
    }

    fun publishGoal(gx: Float, gy: Float, gz: Float, curX: Float, curY: Float) {
        if (!hub.isRosMode) return
        val targetId = if (hub.swarmModeEnabled) hub.selectedDroneId else 0
        activeTransport().publishDroneGoal(targetId, gx, gy, gz, curX, curY)
    }

    fun publishTiltCommand(pitchBiasDeg: Float, coordMode: Int, maxTiltDeg: Float) {
        if (!hub.isRosMode || !hub.rosConnected) return
        activeTransport().publishTiltCommand(pitchBiasDeg, coordMode, maxTiltDeg)
    }

    fun publishSwarmGoals(cmds: List<EgoSimEngine.DroneGoalCmd>) {
        if (!hub.isRosMode || cmds.isEmpty()) return
        val transport = activeTransport()
        for (c in cmds) {
            transport.publishDroneGoal(c.droneId, c.gx, c.gy, c.gz, c.curX, c.curY)
        }
    }

    fun setParamDebounced(paramName: String, value: Float) {
        if (!hub.isRosMode || !hub.rosConnected) return
        pendingParams[paramName] = value
        if (flushScheduled.compareAndSet(false, true)) {
            debounceScheduler.schedule({
                try {
                    val transport = activeTransport()
                    val keys = ArrayList(pendingParams.keys)
                    for (k in keys) {
                        val v = pendingParams.remove(k) ?: continue
                        transport.setParamAsync(k, v)
                    }
                } finally {
                    flushScheduled.set(false)
                    if (pendingParams.isNotEmpty() && flushScheduled.compareAndSet(false, true)) {
                        debounceScheduler.execute {
                            val transport = activeTransport()
                            for ((k, v) in pendingParams) {
                                pendingParams.remove(k)
                                transport.setParamAsync(k, v)
                            }
                            flushScheduled.set(false)
                        }
                    }
                }
            }, 180L, TimeUnit.MILLISECONDS)
        }
    }

    fun shutdown() {
        disconnectAll(silent = true)
        rosBridgeTransport.shutdown()
        tcpRosTransport.shutdown()
        udpTelemetry.shutdown()
        debounceScheduler.shutdownNow()
    }
}
