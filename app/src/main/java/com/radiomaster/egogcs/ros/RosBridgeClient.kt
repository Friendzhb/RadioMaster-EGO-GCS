package com.radiomaster.egogcs.ros

import android.util.Base64
import com.radiomaster.egogcs.engine.EgoSimEngine
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.SwarmPalette
import com.radiomaster.egogcs.model.TopicConfig
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sin
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import org.json.JSONArray
import org.json.JSONObject

class RosBridgeClient(
    private val hub: GcsSharedHub,
    private val engine: EgoSimEngine,
    private val onStateToast: (String) -> Unit
) : RosTransport {

    override val protocolId: String = "rosbridge"

    private val client = OkHttpClient.Builder()
        .connectTimeout(5, TimeUnit.SECONDS)
        .readTimeout(0, TimeUnit.MILLISECONDS)
        .pingInterval(5, TimeUnit.SECONDS)
        .build()

    @Volatile private var ws: WebSocket? = null

    @Volatile var topics = TopicConfig()

    // 供 RosTopicInspector 在 rosbridge 模式下获取话题列表及监听任意指定话题
    @Volatile var watchedTopicName: String? = null

    @Volatile var onWatchedTopicJson: ((topic: String, msgJson: JSONObject, rawBytes: Int) -> Unit)? = null

    @Volatile var onRosApiTopicsResult: ((List<RosTopicEntry>) -> Unit)? = null

    private val swarmOdomTopicToId = ConcurrentHashMap<String, Int>()
    private val swarmOptTopicToId = ConcurrentHashMap<String, Int>()
    private val swarmTiltTopicToId = ConcurrentHashMap<String, Int>()

    private val reuseXyzi = FloatArray(6016 * 4)
    private val reuseInflate = FloatArray(4096 * 3)
    private val reusePath = FloatArray(820 * 3)

    private fun expandPattern(pattern: String, droneId: Int): String =
        pattern.trim().replace("{id}", droneId.toString())

    private fun coreSubscribedSet(cfg: TopicConfig): Set<String> {
        val base = mutableSetOf(
            cfg.odomTopic,
            cfg.fastLioCloudTopic,
            cfg.fastLioPathTopic,
            cfg.inflateTopic,
            cfg.optimalTopic,
            "/mavros/state",
            "/mavros/battery",
            cfg.tiltTelemetryTopic,
            "/mavros/actuator_control"
        )
        base.addAll(swarmOdomTopicToId.keys)
        base.addAll(swarmOptTopicToId.keys)
        base.addAll(swarmTiltTopicToId.keys)
        return base.filter { it.isNotBlank() }.toSet()
    }

    override fun connect(cfg: TopicConfig) {
        topics = cfg.copy()
        disconnect(silent = true)

        hub.isRosMode = true
        hub.connectionLevel = 1
        hub.connectionLabel = "正在连接机载电脑..."

        val req = try {
            Request.Builder().url(cfg.wsUrl.trim()).build()
        } catch (e: Exception) {
            hub.connectionLevel = 2
            hub.connectionLabel = "WS 地址无效"
            onStateToast("WebSocket 地址格式有误")
            return
        }

        ws = client.newWebSocket(
            req,
            object : WebSocketListener() {
                override fun onOpen(webSocket: WebSocket, response: Response) {
                    hub.rosConnected = true
                    hub.connectionLevel = 0
                    hub.connectionLabel = "WS · ${hub.formatRosModeSuffix()}"
                    subscribeAll(webSocket, topics)
                    onStateToast("已连接机载 rosbridge: ${cfg.wsUrl}")
                }

                override fun onMessage(webSocket: WebSocket, text: String) {
                    parseRosPacket(text)
                }

                override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                    hub.rosConnected = false
                    if (hub.isRosMode) {
                        hub.connectionLevel = 2
                        hub.connectionLabel = "未连接机载电脑"
                    }
                    onStateToast("ROS1 连接失败: ${t.localizedMessage ?: "网络不可达"}")
                }

                override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                    hub.rosConnected = false
                    if (hub.isRosMode) {
                        hub.connectionLevel = 2
                        hub.connectionLabel = "ROS1 已断开"
                    }
                }
            }
        )
    }

    override fun disconnect(silent: Boolean) {
        watchedTopicName = null
        swarmOdomTopicToId.clear()
        swarmOptTopicToId.clear()
        swarmTiltTopicToId.clear()
        try {
            ws?.cancel()
        } catch (_: Throwable) {}
        ws = null
        hub.rosConnected = false
        if (!silent) {
            if (hub.isRosMode) {
                hub.connectionLevel = 2
                hub.connectionLabel = "ROS1 已断开"
            } else {
                hub.connectionLevel = 0
                hub.connectionLabel = "正常飞行 (DEMO)"
            }
        }
    }

    fun requestTopicsViaRosApi(): Boolean {
        val active = ws ?: return false
        if (!hub.rosConnected) return false
        val pkt = JSONObject()
            .put("op", "call_service")
            .put("id", "inspector_topics_req")
            .put("service", "/rosapi/topics")
        return active.send(pkt.toString())
    }

    fun startWatchingTopic(topic: String, knownType: String = "", singleShot: Boolean = false) {
        val prev = watchedTopicName
        if (prev != null && prev != topic) {
            stopWatchingTopic(prev)
        }
        watchedTopicName = topic
        val active = ws ?: return
        if (!hub.rosConnected) return
        if (topic !in coreSubscribedSet(topics)) {
            val sub = JSONObject()
                .put("op", "subscribe")
                .put("id", "inspector_sub_$topic")
                .put("topic", topic)
            if (knownType.isNotBlank() && knownType != "unknown") {
                sub.put("type", knownType)
            }
            if (!singleShot) {
                sub.put("throttle_rate", 100)
            }
            active.send(sub.toString())
        }
    }

    fun stopWatchingTopic(topic: String? = watchedTopicName) {
        val target = topic ?: return
        if (watchedTopicName == target) watchedTopicName = null
        val active = ws ?: return
        if (!hub.rosConnected) return
        if (target !in coreSubscribedSet(topics)) {
            val unsub = JSONObject()
                .put("op", "unsubscribe")
                .put("id", "inspector_sub_$target")
                .put("topic", target)
            active.send(unsub.toString())
        }
    }

    fun resubscribeIfConnected(cfg: TopicConfig) {
        topics = cfg.copy()
        val active = ws
        if (hub.rosConnected && active != null) {
            hub.connectionLabel = "WS · ${hub.formatRosModeSuffix()}"
            subscribeAll(active, topics)
        }
    }

    override fun setParamAsync(paramName: String, value: Float) {
        val active = ws ?: return
        if (!hub.rosConnected) return
        val pkt = JSONObject()
            .put("op", "call_service")
            .put("service", "/rosapi/set_param")
            .put("args", JSONObject().put("name", paramName).put("value", value.toString()))
        active.send(pkt.toString())
    }

    override fun publishTiltCommand(pitchBiasDeg: Float, coordMode: Int, maxTiltDeg: Float) {
        val active = ws ?: return
        if (!hub.rosConnected || !hub.anyActiveDroneUsesTilt()) return
        val targetTopics = LinkedHashSet<String>()
        if (!hub.swarmModeEnabled) {
            if (topics.tiltCmdTopic.isNotBlank()) targetTopics.add(topics.tiltCmdTopic.trim())
        } else {
            val cnt = hub.swarmDroneCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
            val ids = if (hub.swarmCommandScope == 1) (0 until cnt).toList() else listOf(hub.selectedDroneId.coerceIn(0, cnt - 1))
            for (id in ids) {
                if (!hub.isDroneTiltRig(id)) continue
                if (topics.swarmTiltCmdPattern.contains("{id}")) {
                    targetTopics.add(expandPattern(topics.swarmTiltCmdPattern, id))
                }
                if (id == 0 && topics.tiltCmdTopic.isNotBlank()) {
                    targetTopics.add(topics.tiltCmdTopic.trim())
                }
            }
        }
        for (tp in targetTopics) {
            val pkt = JSONObject().apply {
                put("op", "publish")
                put("topic", tp)
                put(
                    "msg",
                    JSONObject().apply {
                        put("header", JSONObject().put("frame_id", "base_link"))
                        put(
                            "vector",
                            JSONObject()
                                .put("x", pitchBiasDeg.toDouble())
                                .put("y", coordMode.toDouble())
                                .put("z", maxTiltDeg.toDouble())
                        )
                    }
                )
            }
            active.send(pkt.toString())
        }
    }

    override fun publishGoal(gx: Float, gy: Float, gz: Float, curX: Float, curY: Float) {
        publishDroneGoal(0, gx, gy, gz, curX, curY)
    }

    override fun publishDroneGoal(droneId: Int, gx: Float, gy: Float, gz: Float, curX: Float, curY: Float) {
        val active = ws ?: return
        if (!hub.rosConnected) return
        val targetTopic = if (hub.swarmModeEnabled && topics.swarmGoalPattern.contains("{id}")) {
            expandPattern(topics.swarmGoalPattern, droneId)
        } else {
            topics.goalTopic
        }
        val yaw = atan2((gy - curY).toDouble(), (gx - curX).toDouble())
        val pkt = JSONObject().apply {
            put("op", "publish")
            put("topic", targetTopic)
            put(
                "msg",
                JSONObject().apply {
                    put("header", JSONObject().put("frame_id", "world"))
                    put(
                        "pose",
                        JSONObject().apply {
                            put(
                                "position",
                                JSONObject()
                                    .put("x", gx.toDouble())
                                    .put("y", gy.toDouble())
                                    .put("z", gz.toDouble())
                            )
                            put(
                                "orientation",
                                JSONObject()
                                    .put("x", 0.0)
                                    .put("y", 0.0)
                                    .put("z", sin(yaw * 0.5))
                                    .put("w", cos(yaw * 0.5))
                            )
                        }
                    )
                }
            )
        }
        active.send(pkt.toString())
    }

    private fun subscribeAll(webSocket: WebSocket, cfg: TopicConfig) {
        swarmOdomTopicToId.clear()
        swarmOptTopicToId.clear()
        swarmTiltTopicToId.clear()

        val subs = LinkedHashMap<String, Int>()
        if (cfg.odomTopic.isNotBlank()) subs[cfg.odomTopic.trim()] = 25
        if (cfg.fastLioCloudTopic.isNotBlank()) subs[cfg.fastLioCloudTopic.trim()] = 80
        if (cfg.optimalTopic.isNotBlank()) subs[cfg.optimalTopic.trim()] = 50
        subs["/mavros/state"] = 500
        subs["/mavros/battery"] = 800
        if (cfg.tiltTelemetryTopic.isNotBlank()) subs[cfg.tiltTelemetryTopic.trim()] = 40
        subs["/mavros/actuator_control"] = 50

        if (cfg.fastLioPathTopic.isNotBlank() && cfg.fastLioPathTopic != cfg.optimalTopic) {
            subs[cfg.fastLioPathTopic.trim()] = 250
        }
        if (cfg.inflateTopic.isNotBlank() && cfg.inflateTopic != cfg.fastLioCloudTopic) {
            subs[cfg.inflateTopic.trim()] = 100
        }

        for (id in 0 until SwarmPalette.MAX_SWARM_DRONES) {
            val od1 = expandPattern(cfg.swarmOdomPattern, id)
            val od2 = "/drone_$id/Odometry"
            val op1 = expandPattern(cfg.swarmOptimalPattern, id)
            if (od1.isNotBlank()) {
                swarmOdomTopicToId[od1] = id
                if (hub.swarmModeEnabled) subs[od1] = 35
            }
            swarmOdomTopicToId[od2] = id
            if (hub.swarmModeEnabled) subs[od2] = 35
            if (op1.isNotBlank()) {
                swarmOptTopicToId[op1] = id
                if (hub.swarmModeEnabled) subs[op1] = 60
            }
            val tl1 = expandPattern(cfg.swarmTiltSubPattern, id)
            if (tl1.isNotBlank()) {
                swarmTiltTopicToId[tl1] = id
                if (hub.swarmModeEnabled && hub.isDroneTiltRig(id)) subs[tl1] = 40
            }
        }

        for ((topic, rate) in subs) {
            if (topic.isNotBlank()) {
                val obj = JSONObject()
                    .put("op", "subscribe")
                    .put("topic", topic)
                    .put("throttle_rate", rate)
                webSocket.send(obj.toString())
            }
        }
    }

    private fun parseRosPacket(raw: String) {
        try {
            val root = JSONObject(raw)
            val op = root.optString("op")
            if (op == "service_response" && root.optString("id") == "inspector_topics_req") {
                val values = root.optJSONObject("values")
                val tArr = values?.optJSONArray("topics")
                val tyArr = values?.optJSONArray("types")
                if (tArr != null) {
                    val out = ArrayList<RosTopicEntry>(tArr.length())
                    for (i in 0 until tArr.length()) {
                        val name = tArr.optString(i, "")
                        val tp = tyArr?.optString(i, "unknown") ?: "unknown"
                        if (name.isNotBlank()) out.add(RosTopicEntry(name, tp, 1))
                    }
                    onRosApiTopicsResult?.invoke(out)
                } else {
                    onRosApiTopicsResult?.invoke(emptyList())
                }
                return
            }
            if (op != "publish") return

            val topic = root.optString("topic")
            val msg = root.optJSONObject("msg") ?: return

            if (topic == watchedTopicName) {
                onWatchedTopicJson?.invoke(topic, msg, raw.length)
            }

            val swarmOdomId = swarmOdomTopicToId[topic]
            if (swarmOdomId != null) {
                decodeJsonOdometryIntoEngine(swarmOdomId, msg)
                if (topic != topics.odomTopic) return
            }
            val swarmOptId = swarmOptTopicToId[topic]
            if (swarmOptId != null && topic != topics.optimalTopic) {
                decodePathOrMarker(msg, isGlobalLio = false, targetDroneId = swarmOptId)
                return
            }
            val swarmTiltId = swarmTiltTopicToId[topic]
            if (swarmTiltId != null && topic != topics.tiltTelemetryTopic.trim()) {
                decodeJsonTiltIntoEngine(msg, droneId = swarmTiltId, isActuatorFallback = false)
                return
            }

            when (topic) {
                topics.odomTopic -> decodeJsonOdometryIntoEngine(0, msg)
                topics.fastLioCloudTopic -> decodeBinaryPointCloud2(msg, isFastLioGlobal = true)
                topics.fastLioPathTopic -> decodePathOrMarker(msg, isGlobalLio = true, targetDroneId = 0)
                topics.optimalTopic -> decodePathOrMarker(msg, isGlobalLio = false, targetDroneId = 0)
                topics.inflateTopic -> decodeBinaryPointCloud2(msg, isFastLioGlobal = false)

                "/mavros/state" -> {
                    val mode = msg.optString("mode", "OFFBOARD")
                    val armed = msg.optBoolean("armed", false)
                    engine.onRosMavrosState(mode, armed)
                }

                "/mavros/battery" -> {
                    val volt = msg.optDouble("voltage", 16.0).toFloat()
                    val rawCurr = msg.optDouble("current", 0.0).toFloat()
                    val amps = if (rawCurr.isFinite()) abs(rawCurr) else 0f
                    val rawPct = msg.optDouble("percentage", 0.85)
                    val pct = if (rawPct <= 1.0) (rawPct * 100.0).roundToInt() else rawPct.roundToInt()
                    engine.onRosBattery(volt, pct.coerceIn(0, 100), amps)
                }

                topics.tiltTelemetryTopic.trim(), "/mavros/actuator_control" -> {
                    decodeJsonTiltIntoEngine(
                        msg,
                        droneId = 0,
                        isActuatorFallback = (topic == "/mavros/actuator_control" && topic != topics.tiltTelemetryTopic.trim())
                    )
                }
            }
        } catch (_: Throwable) {
        }
    }

    private fun decodeJsonTiltIntoEngine(msg: JSONObject, droneId: Int = 0, isActuatorFallback: Boolean = false) {
        val maxRad = hub.tiltMaxAngleDeg.coerceIn(15f, 60f) * 0.017453292f
        fun normRad(v: Float): Float = if (abs(v) > 1.35f) v * 0.017453292f else v
        fun emitTilt(s1: Float, s2: Float, m0: Float = -1f, m1: Float = -1f, m2: Float = -1f, m3: Float = -1f) {
            if (droneId <= 0) {
                engine.onRosTiltTelemetry(s1, s2, m0, m1, m2, m3, isActuatorFallback)
            } else {
                engine.onRosSwarmTiltTelemetry(droneId, s1, s2, m0, m1, m2, m3, isActuatorFallback)
            }
        }

        val controls = msg.optJSONArray("controls")
        if (controls != null && controls.length() >= 6) {
            val s1 = controls.optDouble(4, 0.0).toFloat().coerceIn(-1f, 1f) * maxRad
            val s2 = controls.optDouble(5, 0.0).toFloat().coerceIn(-1f, 1f) * maxRad
            emitTilt(
                s1,
                s2,
                controls.optDouble(0, 0.5).toFloat(),
                controls.optDouble(1, 0.5).toFloat(),
                controls.optDouble(2, 0.5).toFloat(),
                controls.optDouble(3, 0.5).toFloat()
            )
            return
        }
        val dataArr = msg.optJSONArray("data") ?: msg.optJSONArray("position")
        if (dataArr != null && dataArr.length() >= 2) {
            if (dataArr.length() >= 6) {
                emitTilt(
                    normRad(dataArr.optDouble(4, 0.0).toFloat()),
                    normRad(dataArr.optDouble(5, 0.0).toFloat()),
                    dataArr.optDouble(0, 0.5).toFloat(),
                    dataArr.optDouble(1, 0.5).toFloat(),
                    dataArr.optDouble(2, 0.5).toFloat(),
                    dataArr.optDouble(3, 0.5).toFloat()
                )
            } else {
                emitTilt(
                    normRad(dataArr.optDouble(0, 0.0).toFloat()),
                    normRad(dataArr.optDouble(1, 0.0).toFloat())
                )
            }
            return
        }
        val vec = msg.optJSONObject("vector")
        if (vec != null) {
            emitTilt(
                normRad(vec.optDouble("x", 0.0).toFloat()),
                normRad(vec.optDouble("y", 0.0).toFloat())
            )
        }
    }

    private fun decodeJsonOdometryIntoEngine(droneId: Int, msg: JSONObject) {
        val poseObj = msg.optJSONObject("pose")?.optJSONObject("pose") ?: return
        val pos = poseObj.optJSONObject("position") ?: return
        val ori = poseObj.optJSONObject("orientation")
        val x = pos.optDouble("x", 0.0).toFloat()
        val y = pos.optDouble("y", 0.0).toFloat()
        val z = pos.optDouble("z", 0.0).toFloat()

        var yaw = 0f
        var roll = 0f
        var pitch = 0f
        var qwF = 1f
        var qxF = 0f
        var qyF = 0f
        var qzF = 0f
        var hasQuat = false
        if (ori != null) {
            val ox = ori.optDouble("x", 0.0)
            val oy = ori.optDouble("y", 0.0)
            val oz = ori.optDouble("z", 0.0)
            val ow = ori.optDouble("w", 1.0)
            val nrm = kotlin.math.sqrt(ox * ox + oy * oy + oz * oz + ow * ow)
            val nx = if (nrm > 1e-6) ox / nrm else 0.0
            val ny = if (nrm > 1e-6) oy / nrm else 0.0
            val nz = if (nrm > 1e-6) oz / nrm else 0.0
            val nw = if (nrm > 1e-6) ow / nrm else 1.0
            roll = atan2(2.0 * (nw * nx + ny * nz), 1.0 - 2.0 * (nx * nx + ny * ny)).toFloat()
            pitch = (-kotlin.math.asin((2.0 * (nw * ny - nz * nx)).coerceIn(-1.0, 1.0))).toFloat()
            yaw = atan2(2.0 * (nw * nz + nx * ny), 1.0 - 2.0 * (ny * ny + nz * nz)).toFloat()
            qwF = nw.toFloat()
            qxF = nx.toFloat()
            qyF = ny.toFloat()
            qzF = nz.toFloat()
            hasQuat = nrm > 0.2 && (kotlin.math.abs(nx) > 1e-5 || kotlin.math.abs(ny) > 1e-5 || kotlin.math.abs(nz) > 1e-5)
        }
        val lin = msg.optJSONObject("twist")?.optJSONObject("twist")?.optJSONObject("linear")
        val vx = (lin?.optDouble("x", 0.0) ?: 0.0).toFloat()
        val vy = (lin?.optDouble("y", 0.0) ?: 0.0).toFloat()
        val vz = (lin?.optDouble("z", 0.0) ?: 0.0).toFloat()
        if (droneId <= 0) {
            engine.onRosOdometry(x, y, z, vx, vy, vz, yaw, roll, pitch, qwF, qxF, qyF, qzF, hasQuat)
        } else {
            engine.onRosSwarmOdometry(droneId, x, y, z, vx, vy, vz, yaw, roll, pitch)
        }
    }

    private fun decodePathOrMarker(msg: JSONObject, isGlobalLio: Boolean, targetDroneId: Int) {
        val ptsArr: JSONArray? = msg.optJSONArray("points") ?: msg.optJSONArray("poses")
        if (ptsArr != null && ptsArr.length() > 0) {
            val n = ptsArr.length()
            val maxCap = if (isGlobalLio) 800 else 400
            val stride = max(1, n / maxCap)
            var valid = 0
            var i = 0
            while (i < n && valid < 815) {
                val item = ptsArr.optJSONObject(i)
                if (item != null) {
                    val p = item.optJSONObject("pose")?.optJSONObject("position") ?: item
                    reusePath[valid * 3] = p.optDouble("x", 0.0).toFloat()
                    reusePath[valid * 3 + 1] = p.optDouble("y", 0.0).toFloat()
                    reusePath[valid * 3 + 2] = p.optDouble("z", 0.0).toFloat()
                    valid++
                }
                i += stride
            }
            if (valid > 0) {
                if (isGlobalLio) {
                    engine.onRosFastLioPath(reusePath, valid)
                } else if (targetDroneId <= 0) {
                    engine.onRosOptimalPath(reusePath, valid)
                } else {
                    engine.onRosSwarmOptimalPath(targetDroneId, reusePath, valid)
                }
            }
        }
    }

    private fun decodeBinaryPointCloud2(msg: JSONObject, isFastLioGlobal: Boolean) {
        val dataB64 = msg.optString("data", "")
        val pointStep = msg.optInt("point_step", 0)
        val fields = msg.optJSONArray("fields")
        if (dataB64.isEmpty() || pointStep < 12 || fields == null) return

        val rawBytes = try {
            Base64.decode(dataB64, Base64.DEFAULT)
        } catch (_: IllegalArgumentException) {
            return
        }
        var offX = 0
        var offY = 4
        var offZ = 8
        var offInten = -1
        for (i in 0 until fields.length()) {
            val f = fields.optJSONObject(i) ?: continue
            when (f.optString("name")) {
                "x" -> offX = f.optInt("offset", 0)
                "y" -> offY = f.optInt("offset", 4)
                "z" -> offZ = f.optInt("offset", 8)
                "intensity", "reflectivity" -> offInten = f.optInt("offset", -1)
            }
        }
        val order = if (msg.optBoolean("is_bigendian", false)) ByteOrder.BIG_ENDIAN else ByteOrder.LITTLE_ENDIAN
        val bb = ByteBuffer.wrap(rawBytes).order(order)
        val total = rawBytes.size / pointStep

        if (isFastLioGlobal) {
            val stride = max(1, total / 6000)
            var count = 0
            var p = 0
            val req = max(max(offX, max(offY, offZ)) + 4, if (offInten >= 0) offInten + 4 else 12)
            while (p < total && count < 6010) {
                val base = p * pointStep
                if (base + req > rawBytes.size) break
                val px = bb.getFloat(base + offX)
                val py = bb.getFloat(base + offY)
                val pz = bb.getFloat(base + offZ)
                if (px.isFinite() && py.isFinite() && pz.isFinite()) {
                    val inten = if (offInten >= 0) {
                        val fv = bb.getFloat(base + offInten)
                        (if (fv > 1.5f) fv / 255f else fv).coerceIn(0.05f, 1.0f)
                    } else {
                        ((pz + 0.5f) / 3.5f).coerceIn(0.05f, 1.0f)
                    }
                    val dst = count * 4
                    reuseXyzi[dst] = px
                    reuseXyzi[dst + 1] = py
                    reuseXyzi[dst + 2] = pz
                    reuseXyzi[dst + 3] = inten
                    count++
                }
                p += stride
            }
            if (count > 0) engine.onRosFastLioCloud(reuseXyzi, count)
        } else {
            val stride = max(1, total / 4080)
            var count = 0
            var p = 0
            while (p < total && count < 4090) {
                val base = p * pointStep
                if (base + max(offX, max(offY, offZ)) + 4 > rawBytes.size) break
                val px = bb.getFloat(base + offX)
                val py = bb.getFloat(base + offY)
                val pz = bb.getFloat(base + offZ)
                if (px.isFinite() && py.isFinite() && pz.isFinite()) {
                    reuseInflate[count * 3] = px
                    reuseInflate[count * 3 + 1] = py
                    reuseInflate[count * 3 + 2] = pz
                    count++
                }
                p += stride
            }
            engine.onRosInflatedCloud(reuseInflate, count)
        }
    }
}
