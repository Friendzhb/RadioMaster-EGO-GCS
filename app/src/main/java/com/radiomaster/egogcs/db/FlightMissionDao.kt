package com.radiomaster.egogcs.db

import androidx.room.Dao
import androidx.room.Insert
import androidx.room.OnConflictStrategy
import androidx.room.Query
import androidx.room.Update
import kotlinx.coroutines.flow.Flow

/**
 * 本地 SQLite 飞行任务与遥测时序数据访问接口 (`FlightMissionDao`)：
 * - 查询统一返回 `Flow` 以驱动响应式历史架次列表刷新；
 * - 写入/更新/删除全部采用 `suspend` 批量事务，杜绝阻塞 UI 或 50Hz 引擎线程。
 */
@Dao
interface FlightMissionDao {

    @Query("SELECT * FROM flight_missions ORDER BY startedAtEpochMs DESC, id DESC")
    fun observeAllMissions(): Flow<List<FlightMissionEntity>>

    @Query("SELECT * FROM flight_missions WHERE id = :missionId LIMIT 1")
    suspend fun getMissionById(missionId: Long): FlightMissionEntity?

    @Insert(onConflict = OnConflictStrategy.REPLACE)
    suspend fun insertMission(mission: FlightMissionEntity): Long

    @Update
    suspend fun updateMission(mission: FlightMissionEntity)

    @Insert(onConflict = OnConflictStrategy.REPLACE)
    suspend fun insertTelemetryBatch(points: List<TelemetryPointEntity>)

    @Query("SELECT * FROM telemetry_points WHERE missionId = :missionId ORDER BY elapsedMs ASC, id ASC")
    suspend fun getTelemetryPointsForMission(missionId: Long): List<TelemetryPointEntity>

    @Query("DELETE FROM flight_missions WHERE id = :missionId")
    suspend fun deleteMissionById(missionId: Long)

    @Query(
        """
        DELETE FROM flight_missions
        WHERE id NOT IN (
            SELECT id FROM flight_missions
            ORDER BY startedAtEpochMs DESC, id DESC
            LIMIT :keepCount
        )
        """
    )
    suspend fun pruneOldMissionsKeepLatest(keepCount: Int)
}
