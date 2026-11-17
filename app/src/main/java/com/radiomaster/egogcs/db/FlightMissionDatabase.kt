package com.radiomaster.egogcs.db

import android.content.Context
import androidx.room.Database
import androidx.room.Room
import androidx.room.RoomDatabase

/**
 * RadioMaster AX12 本地 SQLite 飞行任务黑匣子数据库 (`ego_flight_missions.db`)：
 * 启用 WAL (Write-Ahead Logging) 日志模式，支持后台 20Hz 批量遥测落库与前台历史分析并发无锁读取。
 */
@Database(
    entities = [FlightMissionEntity::class, TelemetryPointEntity::class],
    version = 1,
    exportSchema = false
)
abstract class FlightMissionDatabase : RoomDatabase() {

    abstract fun flightMissionDao(): FlightMissionDao

    companion object {
        private const val DB_NAME = "ego_flight_missions.db"

        @Volatile
        private var instance: FlightMissionDatabase? = null

        fun getInstance(context: Context): FlightMissionDatabase {
            return instance ?: synchronized(this) {
                instance ?: Room.databaseBuilder(
                    context.applicationContext,
                    FlightMissionDatabase::class.java,
                    DB_NAME
                )
                    .setJournalMode(JournalMode.WRITE_AHEAD_LOGGING)
                    .fallbackToDestructiveMigration()
                    .build()
                    .also { instance = it }
            }
        }
    }
}
