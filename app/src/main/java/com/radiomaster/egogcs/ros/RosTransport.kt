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
    private val rosBridgeTransport: RosBridgeClient
) {
    @Volatile var topics: TopicConfig = TopicConfig()

    private val pendingParams = ConcurrentHashMap<String, Float>()
    private val flushScheduled = AtomicBoolean(false)
    private val debounceScheduler = Executors.newSingleThreadScheduledExecutor { r ->
        Thread(r, "RosParamDebouncer").apply { isDaemon = true }
    }

    private fun activeTransport(): RosTransport =
        if (topics.protocol == "tcpros") tcpRosTransport else rosBridgeTransport

    fun connect(cfg: TopicConfig = topics) {
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

    fun disconnectAll(silent: Boolean = false) {
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
        debounceScheduler.shutdownNow()
    }
}
