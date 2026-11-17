package com.radiomaster.egogcs.db

import com.radiomaster.egogcs.engine.FlightPlaybackController
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.RenderSnapshot
import java.io.File
import java.nio.file.Files
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.runBlocking
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

class FlightMissionPlaybackTest {

    /**
     * 内存关系型假 DAO：严格模拟 SQLite 自增主键、外键 `ON DELETE CASCADE` 与时间倒序查询。
     */
    private class FakeFlightMissionDao : FlightMissionDao {
        private val lock = Any()
        private var nextMissionId = 1L
        private var nextPointId = 1L
        private val missions = LinkedHashMap<Long, FlightMissionEntity>()
        private val pointsByMission = HashMap<Long, MutableList<TelemetryPointEntity>>()
        private val flow = MutableStateFlow<List<FlightMissionEntity>>(emptyList())

        private fun emitSortedLocked() {
            flow.value = missions.values.sortedWith(
                compareByDescending<FlightMissionEntity> { it.startedAtEpochMs }
                    .thenByDescending { it.id }
            )
        }

        override fun observeAllMissions(): Flow<List<FlightMissionEntity>> = flow

        override suspend fun getMissionById(missionId: Long): FlightMissionEntity? =
            synchronized(lock) { missions[missionId] }

        override suspend fun insertMission(mission: FlightMissionEntity): Long = synchronized(lock) {
            val id = if (mission.id > 0L) mission.id else nextMissionId++
            val saved = mission.copy(id = id)
            missions[id] = saved
            pointsByMission.putIfAbsent(id, mutableListOf())
            emitSortedLocked()
            id
        }

        override suspend fun updateMission(mission: FlightMissionEntity) = synchronized(lock) {
            if (missions.containsKey(mission.id)) {
                missions[mission.id] = mission
                emitSortedLocked()
            }
        }

        override suspend fun insertTelemetryBatch(points: List<TelemetryPointEntity>) = synchronized(lock) {
            for (pt in points) {
                val bucket = pointsByMission[pt.missionId] ?: continue
                bucket.add(pt.copy(id = nextPointId++))
            }
        }

        override suspend fun getTelemetryPointsForMission(missionId: Long): List<TelemetryPointEntity> =
            synchronized(lock) {
                pointsByMission[missionId]
                    ?.sortedWith(compareBy<TelemetryPointEntity> { it.elapsedMs }.thenBy { it.id })
                    ?.toList()
                    ?: emptyList()
            }

        override suspend fun deleteMissionById(missionId: Long) = synchronized(lock) {
            missions.remove(missionId)
            pointsByMission.remove(missionId)
            emitSortedLocked()
            Unit
        }

        override suspend fun pruneOldMissionsKeepLatest(keepCount: Int) = synchronized(lock) {
            val ordered = missions.values.sortedWith(
                compareByDescending<FlightMissionEntity> { it.startedAtEpochMs }
                    .thenByDescending { it.id }
            )
            if (ordered.size > keepCount) {
                for (m in ordered.drop(keepCount)) {
                    missions.remove(m.id)
                    pointsByMission.remove(m.id)
                }
                emitSortedLocked()
            }
        }
    }

    @Test
    fun repositoryBatches20HzSamplesComputesAnalyticsAndExportsCsv() = runBlocking {
        val dao = FakeFlightMissionDao()
        val repo = FlightMissionRepository(dao, Dispatchers.Default)
        val hub = GcsSharedHub().apply { isRosMode = true }
        val snap = RenderSnapshot()

        val startLatch = CountDownLatch(1)
        var createdId = 0L
        val baseWallMs = 1_763_300_000_000L
        repo.startNewMissionSession(
            hub = hub,
            currentSnap = snap,
            waypointsCsv = "0.0,0.0,1.2;4.0,0.0,1.6",
            customTag = "单元测试航线",
            nowWallMs = baseWallMs,
            onStarted = { id ->
                createdId = id
                startLatch.countDown()
            }
        )
        assertTrue(startLatch.await(3, TimeUnit.SECONDS))
        assertTrue(createdId > 0L)

        // 连续投递 60 帧 (3.0s @ 20Hz)，沿 +X 轴以 2.0 m/s 匀速推进 5.9m
        for (i in 0 until 60) {
            val elapsedMs = i * 50L
            snap.uavX = i * 0.10f
            snap.uavY = 0f
            snap.uavZ = 1.2f + i * 0.01f
            snap.vx = 2.0f
            snap.vy = 0f
            snap.vz = 0.2f
            snap.minClearance = if (i in 20..24) 0.45f else 1.50f
            snap.batVolt = 16.40f - i * 0.01f
            snap.batCurrentAmps = 15.0f
            repo.enqueueTelemetrySample(baseWallMs + elapsedMs, elapsedMs, snap)
        }

        val stopLatch = CountDownLatch(1)
        var finalEntity: FlightMissionEntity? = null
        repo.stopActiveMissionSession(
            waypointsCsv = "0.0,0.0,1.2;4.0,0.0,1.6",
            endedWallMs = baseWallMs + 2950L,
            onStopped = { saved ->
                finalEntity = saved
                stopLatch.countDown()
            }
        )
        assertTrue(stopLatch.await(3, TimeUnit.SECONDS))
        assertNotNull(finalEntity)
        assertEquals(60, finalEntity!!.sampleCount)
        assertEquals("ROS1", finalEntity!!.linkMode)
        assertTrue(finalEntity!!.distanceMeters > 5.8f)

        val storedList = repo.missionsFlow.first()
        assertEquals(1, storedList.size)

        val report = repo.analyzeMission(createdId, warnClearanceM = 0.65f)
        assertNotNull(report)
        assertEquals(60, report!!.sampleCount)
        assertEquals(5, report.nearObstacleWarnFrames)
        assertEquals(2.0f, report.maxGroundSpeedMps, 0.02f)
        assertTrue(report.voltageSagV > 0.55f)
        assertTrue(report.consumedMah > 10f)
        assertEquals(36, report.profileBucketCount)

        val tmpDir = Files.createTempDirectory("ego_sql_csv_test").toFile()
        try {
            val csvFile = File(tmpDir, "mission.csv")
            val exportedRows = repo.exportMissionToCsv(createdId, csvFile)
            assertEquals(60, exportedRows)
            val lines = csvFile.readLines()
            assertEquals(61, lines.size)
            assertTrue(lines.first().startsWith("timestamp_epoch_ms,t_sec,link_mode"))
            assertTrue(lines[1].startsWith("$baseWallMs,0.000,ROS1"))
        } finally {
            tmpDir.deleteRecursively()
        }

        repo.deleteMission(createdId)
        assertTrue(repo.missionsFlow.first().isEmpty())
    }

    @Test
    fun playbackControllerInterpolatesPoseWrapsYawAndSupportsScrubbingAndSpeed() {
        val hub = GcsSharedHub()
        val controller = FlightPlaybackController()
        val mission = FlightMissionEntity(
            id = 7L,
            title = "回放插值验证架次",
            startedAtEpochMs = 1_763_300_100_000L,
            endedAtEpochMs = 1_763_300_102_000L,
            durationMs = 2000L,
            sampleCount = 3,
            waypointsCsv = "1.0,2.0,1.4;3.5,-1.0,1.8"
        )
        val nearPosPi = (175.0 * PI / 180.0).toFloat()
        val nearNegPi = (-175.0 * PI / 180.0).toFloat()
        val pts = listOf(
            samplePoint(7L, 1_763_300_100_000L, 0L, x = 0f, y = 0f, z = 1.0f, yaw = nearPosPi),
            samplePoint(7L, 1_763_300_101_000L, 1000L, x = 4f, y = 2f, z = 2.0f, yaw = nearNegPi),
            samplePoint(7L, 1_763_300_102_000L, 2000L, x = 8f, y = 2f, z = 2.0f, yaw = nearNegPi)
        )

        assertTrue(controller.loadMission(mission, pts, hub))
        assertTrue(hub.isReplayMode)
        assertTrue(hub.replayPlaying)

        val frame = RenderSnapshot()
        // 推进 0.5 秒 (1.0x 倍速 -> 位于 0ms 与 1000ms 正中点 alpha = 0.5)
        assertTrue(
            controller.applyReplayToSnapshot(
                dtSec = 0.5f,
                hub = hub,
                frame = frame,
                evaluateClearance = { _, _, _ -> 1.8f to true },
                queryColumnTopZ = { _, _ -> 0f }
            )
        )
        assertEquals(2.0f, frame.uavX, 0.01f)
        assertEquals(1.0f, frame.uavY, 0.01f)
        assertEquals(1.5f, frame.uavZ, 0.01f)
        // 175° 到 -175° 最短弧中点应穿过 ±180° (|yaw| ≈ π)，绝不应反向经过 0°
        assertTrue(abs(frame.yaw) > 3.05f)
        assertEquals(2, frame.waypointQueueCount)
        assertEquals(3, frame.lioPathCount)

        // 切换倍速 1.0x -> 2.0x，再推进 0.5s 应前进 1000ms 抵达 1500ms 处 (x = 6.0m)
        assertEquals(2.0f, controller.cycleSpeed(hub), 0.001f)
        controller.applyReplayToSnapshot(
            dtSec = 0.5f,
            hub = hub,
            frame = frame,
            evaluateClearance = { _, _, _ -> 1.8f to true },
            queryColumnTopZ = { _, _ -> 0f }
        )
        assertEquals(6.0f, frame.uavX, 0.02f)

        // 时间轴拖动至 25% (500ms) 应立即递增 trailClearEpoch 擦除旧尾迹
        val prevEpoch = hub.trailClearEpoch
        controller.seekToProgress(0.25f, hub)
        assertTrue(hub.trailClearEpoch > prevEpoch)
        assertEquals(500L, hub.replayElapsedMs)

        controller.stopReplay(hub)
        assertFalse(hub.isReplayMode)
        assertFalse(controller.isActive)
    }

    private fun samplePoint(
        missionId: Long,
        wallMs: Long,
        elapsedMs: Long,
        x: Float,
        y: Float,
        z: Float,
        yaw: Float
    ) = TelemetryPointEntity(
        missionId = missionId,
        timestampEpochMs = wallMs,
        elapsedMs = elapsedMs,
        x = x,
        y = y,
        z = z,
        vx = 2.0f,
        vy = 0f,
        vz = 0f,
        rollRad = 0f,
        pitchRad = 0f,
        yawRad = yaw,
        quatW = 1f,
        quatX = 0f,
        quatY = 0f,
        quatZ = 0f,
        goalX = 8f,
        goalY = 2f,
        goalZ = 2f,
        activeIsTiltRig = true,
        tiltS1Rad = 0.15f,
        tiltS2Rad = 0.15f,
        tiltCmd1Rad = 0.16f,
        tiltCmd2Rad = 0.16f,
        m0 = 0.55f,
        m1 = 0.55f,
        m2 = 0.55f,
        m3 = 0.55f,
        minClearanceM = 1.6f,
        minInterDroneDistM = 9.9f,
        solveMs = 0.4f,
        batVolt = 16.1f,
        batAmps = 12.5f,
        batPct = 84,
        fsmState = "EXEC_TRAJ",
        px4Mode = "OFFBOARD",
        armed = true
    )
}
