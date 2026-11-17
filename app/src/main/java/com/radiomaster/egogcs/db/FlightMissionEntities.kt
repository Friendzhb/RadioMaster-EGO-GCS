package com.radiomaster.egogcs.db

import androidx.room.Entity
import androidx.room.ForeignKey
import androidx.room.Index
import androidx.room.PrimaryKey

/**
 * 单架次飞行任务元数据与战后汇总指标表 (`flight_missions`)：
 * 存储架次起止墙钟时间戳、链路模式 (ROS1/SIM)、机体构型、总三维航程、极值地速、最低障碍净空、电压压降与编排航点快照。
 */
@Entity(
    tableName = "flight_missions",
    indices = [Index(value = ["startedAtEpochMs"])]
)
data class FlightMissionEntity(
    @PrimaryKey(autoGenerate = true) val id: Long = 0L,
    val title: String,
    val startedAtEpochMs: Long,
    val endedAtEpochMs: Long = startedAtEpochMs,
    val durationMs: Long = 0L,
    val sampleCount: Int = 0,
    val linkMode: String = "SIM",
    val airframeRig: String = "TILT_4M2S",
    val swarmCount: Int = 1,
    val distanceMeters: Float = 0f,
    val maxSpeedMps: Float = 0f,
    val avgSpeedMps: Float = 0f,
    val maxAltitudeM: Float = 0f,
    val minClearanceM: Float = 9.5f,
    val startVolt: Float = 0f,
    val minVolt: Float = 0f,
    val peakPowerWatts: Float = 0f,
    val waypointsCsv: String = ""
)

/**
 * 20Hz 时序飞行遥测点表 (`telemetry_points`)：
 * 通过外键级联关联所属 `flight_missions.id`，并在 `(missionId, elapsedMs)` 与 `timestampEpochMs` 上建立联合索引，
 * 支持毫秒级全架次轨迹加载、时间轴二分检索回放与科研 CSV 导出。
 */
@Entity(
    tableName = "telemetry_points",
    foreignKeys = [
        ForeignKey(
            entity = FlightMissionEntity::class,
            parentColumns = ["id"],
            childColumns = ["missionId"],
            onDelete = ForeignKey.CASCADE
        )
    ],
    indices = [
        Index(value = ["missionId", "elapsedMs"]),
        Index(value = ["timestampEpochMs"])
    ]
)
data class TelemetryPointEntity(
    @PrimaryKey(autoGenerate = true) val id: Long = 0L,
    val missionId: Long,
    val timestampEpochMs: Long,
    val elapsedMs: Long,
    val x: Float,
    val y: Float,
    val z: Float,
    val vx: Float,
    val vy: Float,
    val vz: Float,
    val rollRad: Float,
    val pitchRad: Float,
    val yawRad: Float,
    val quatW: Float,
    val quatX: Float,
    val quatY: Float,
    val quatZ: Float,
    val goalX: Float,
    val goalY: Float,
    val goalZ: Float,
    val activeIsTiltRig: Boolean,
    val tiltS1Rad: Float,
    val tiltS2Rad: Float,
    val tiltCmd1Rad: Float,
    val tiltCmd2Rad: Float,
    val m0: Float,
    val m1: Float,
    val m2: Float,
    val m3: Float,
    val minClearanceM: Float,
    val minInterDroneDistM: Float,
    val solveMs: Float,
    val batVolt: Float,
    val batAmps: Float,
    val batPct: Int,
    val fsmState: String,
    val px4Mode: String,
    val armed: Boolean
)
