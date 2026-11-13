package com.radiomaster.egogcs.engine

import android.opengl.GLES20
import java.io.BufferedOutputStream
import java.io.File
import java.io.FileOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.concurrent.withLock
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

/** Voxel hash map with versioned, bounded uploads. GL never waits for a map producer. */
class FastLioVoxelMap {

    companion object {
        const val MAX_POINTS = 300_000
        const val FLOATS_PER_VERTEX = 4
        const val BYTES_PER_VERTEX = FLOATS_PER_VERTEX * 4
        private const val HASH_BITS = 19
        private const val HASH_CAP = 1 shl HASH_BITS
        private const val HASH_MASK = HASH_CAP - 1
        private const val EMPTY_KEY = Long.MIN_VALUE
        private const val TOMBSTONE_KEY = Long.MIN_VALUE + 1L
    }

    private val pointStore = PagedPointStore(MAX_POINTS)
    private val producerLock = pointStore.lock
    private val uploadCursor = PointUploadCursor(MAX_POINTS)
    private val pageSink = PointPageSink { start, count, values ->
        GLES20.glBufferSubData(GLES20.GL_ARRAY_BUFFER, start * BYTES_PER_VERTEX, count * BYTES_PER_VERTEX, values)
    }
    val uploadedPointsLastFrame: Int get() = uploadCursor.uploadedPoints
    val uploadedPagesLastFrame: Int get() = uploadCursor.uploadedPages

    private val tableKeys = LongArray(HASH_CAP) { EMPTY_KEY }
    private val tableSlots = IntArray(HASH_CAP) { -1 }
    private val slotToKey = LongArray(MAX_POINTS) { EMPTY_KEY }

    private val vertexShadow = pointStore.vertices

    @Volatile var activeVoxelSize: Float = 0.12f
        private set

    @Volatile var pointCount: Int = 0
        private set

    private var writeCursor: Int = 0
    private var tombstoneCount: Int = 0

    // v3.2: 近机 ROI 驻留锚点与点云三维空间包络统计 + Livox/Velodyne 反射率量程自适应峰值
    @Volatile private var focusAnchorX: Float = 0f

    @Volatile private var focusAnchorY: Float = 0f

    @Volatile private var peakObservedIntensity: Float = 1.0f

    @Volatile var minBoundX: Float = 0f
        private set

    @Volatile var maxBoundX: Float = 0f
        private set

    @Volatile var minBoundY: Float = 0f
        private set

    @Volatile var maxBoundY: Float = 0f
        private set

    @Volatile var minBoundZ: Float = 0f
        private set

    @Volatile var maxBoundZ: Float = 0f
        private set

    fun updateFocusAnchor(ux: Float, uy: Float) {
        if (ux.isFinite() && uy.isFinite()) {
            focusAnchorX = ux
            focusAnchorY = uy
        }
    }

    fun formatBoundsSpanSummary(): String {
        val cnt = pointCount
        if (cnt <= 0) return "0.0 x 0.0 x 0.0 m"
        val sx = (maxBoundX - minBoundX).coerceAtLeast(0f)
        val sy = (maxBoundY - minBoundY).coerceAtLeast(0f)
        val sz = (maxBoundZ - minBoundZ).coerceAtLeast(0f)
        return String.format(java.util.Locale.US, "%.1f x %.1f x %.1f m", sx, sy, sz)
    }

    fun clear(newVoxelSize: Float = activeVoxelSize) {
        producerLock.withLock {
            activeVoxelSize = newVoxelSize.coerceIn(0.05f, 0.50f)
            tableKeys.fill(EMPTY_KEY)
            tableSlots.fill(-1)
            slotToKey.fill(EMPTY_KEY)
            pointCount = 0
            writeCursor = 0
            tombstoneCount = 0
            peakObservedIntensity = 1.0f
            minBoundX = 0f
            maxBoundX = 0f
            minBoundY = 0f
            maxBoundY = 0f
            minBoundZ = 0f
            maxBoundZ = 0f
            pointStore.clear()
        }
    }

    fun ensureVoxelResolution(desiredRes: Float) {
        val clamped = desiredRes.coerceIn(0.05f, 0.50f)
        if (abs(clamped - activeVoxelSize) > 0.004f) {
            clear(clamped)
        }
    }

    fun exportBinaryPcd(outFile: File): Int {
        val snapshotCopy: FloatArray
        val count: Int
        producerLock.withLock {
            count = pointCount
            if (count <= 0) return 0
            snapshotCopy = vertexShadow.copyOf(count * FLOATS_PER_VERTEX)
        }

        outFile.parentFile?.mkdirs()
        BufferedOutputStream(FileOutputStream(outFile), 65536).use { out ->
            val header = buildString {
                append("# .PCD v0.7 - Point Cloud Data file format\n")
                append("VERSION 0.7\n")
                append("FIELDS x y z intensity\n")
                append("SIZE 4 4 4 4\n")
                append("TYPE F F F F\n")
                append("COUNT 1 1 1 1\n")
                append("WIDTH $count\n")
                append("HEIGHT 1\n")
                append("VIEWPOINT 0 0 0 1 0 0 0\n")
                append("POINTS $count\n")
                append("DATA binary\n")
            }
            out.write(header.toByteArray(Charsets.US_ASCII))

            val chunkPts = 2048
            val byteBuf = ByteBuffer.allocate(chunkPts * BYTES_PER_VERTEX).order(ByteOrder.LITTLE_ENDIAN)
            var idx = 0
            while (idx < count) {
                val n = min(chunkPts, count - idx)
                byteBuf.clear()
                for (k in 0 until n) {
                    val base = (idx + k) * FLOATS_PER_VERTEX
                    byteBuf.putFloat(snapshotCopy[base])
                    byteBuf.putFloat(snapshotCopy[base + 1])
                    byteBuf.putFloat(snapshotCopy[base + 2])
                    byteBuf.putFloat(snapshotCopy[base + 3])
                }
                out.write(byteBuf.array(), 0, n * BYTES_PER_VERTEX)
                idx += n
            }
            out.flush()
        }
        return count
    }

    fun insertScan(xyzi: FloatArray, count: Int) {
        if (count <= 0) return
        producerLock.withLock {
            val res = activeVoxelSize
            val invRes = 1.0f / res
            val limit = min(count, xyzi.size / 4)

            if (tombstoneCount > 110_000) {
                rehashLocked()
            }

            for (i in 0 until limit) {
                val base = i * 4
                val px = xyzi[base]
                val py = xyzi[base + 1]
                val pz = xyzi[base + 2]
                val rawInten = xyzi[base + 3]
                if (!px.isFinite() || !py.isFinite() || !pz.isFinite()) continue
                if (rawInten.isFinite() && rawInten > peakObservedIntensity) {
                    peakObservedIntensity = min(255f, rawInten)
                }
                val inten = if (peakObservedIntensity > 1.5f) {
                    (rawInten / peakObservedIntensity).coerceIn(0f, 1f)
                } else {
                    rawInten.coerceIn(0f, 1f)
                }

                val ix = floor(px * invRes).toInt()
                val iy = floor(py * invRes).toInt()
                val iz = floor(pz * invRes).toInt()
                if (ix !in -500_000..500_000 || iy !in -500_000..500_000 || iz !in -100_000..100_000) continue

                val key = packKey(ix, iy, iz)
                var bucket = mixHash(key) and HASH_MASK
                var firstTombstone = -1
                var existingSlot = -1

                while (true) {
                    val k = tableKeys[bucket]
                    if (k == EMPTY_KEY) break
                    if (k == key) {
                        existingSlot = tableSlots[bucket]
                        break
                    }
                    if (k == TOMBSTONE_KEY && firstTombstone < 0) {
                        firstTombstone = bucket
                    }
                    bucket = (bucket + 1) and HASH_MASK
                }

                if (existingSlot >= 0) {
                    // 静态体素已存在：跳过重复显存上传，彻底消除分散槽位导致的整块 VBO 重传卡顿
                    continue
                }

                // v3.2: 满 300k 体素环形淘汰时优先保护无人机周围 8.5m 近场避障体素，最多向前探测 6 槽寻找远场旧体素淘汰
                if (pointCount >= MAX_POINTS) {
                    val fx = focusAnchorX
                    val fy = focusAnchorY
                    for (skip in 0 until 6) {
                        val candOff = writeCursor * FLOATS_PER_VERTEX
                        val dxOld = vertexShadow[candOff] - fx
                        val dyOld = vertexShadow[candOff + 1] - fy
                        if (dxOld * dxOld + dyOld * dyOld > 72.25f) break
                        writeCursor++
                        if (writeCursor >= MAX_POINTS) writeCursor = 0
                    }
                }
                val slot = writeCursor
                writeCursor++
                if (writeCursor >= MAX_POINTS) writeCursor = 0

                if (pointCount == 0) {
                    minBoundX = px
                    maxBoundX = px
                    minBoundY = py
                    maxBoundY = py
                    minBoundZ = pz
                    maxBoundZ = pz
                } else {
                    if (px < minBoundX) {
                        minBoundX = px
                    } else if (px > maxBoundX) {
                        maxBoundX = px
                    }
                    if (py < minBoundY) {
                        minBoundY = py
                    } else if (py > maxBoundY) {
                        maxBoundY = py
                    }
                    if (pz < minBoundZ) {
                        minBoundZ = pz
                    } else if (pz > maxBoundZ) {
                        maxBoundZ = pz
                    }
                }

                if (pointCount < MAX_POINTS) {
                    pointCount++
                } else {
                    val oldKey = slotToKey[slot]
                    if (oldKey != EMPTY_KEY) {
                        removeKeyLocked(oldKey)
                    }
                }

                val targetBucket = if (firstTombstone >= 0) {
                    tombstoneCount = max(0, tombstoneCount - 1)
                    firstTombstone
                } else {
                    bucket
                }
                tableKeys[targetBucket] = key
                tableSlots[targetBucket] = slot
                slotToKey[slot] = key

                val vOff = slot * FLOATS_PER_VERTEX
                vertexShadow[vOff] = px
                vertexShadow[vOff + 1] = py
                vertexShadow[vOff + 2] = pz
                vertexShadow[vOff + 3] = inten

                pointStore.written(slot, pointCount)
            }
        }
    }

    private fun removeKeyLocked(key: Long) {
        var bucket = mixHash(key) and HASH_MASK
        var probes = 0
        while (probes < HASH_CAP) {
            val k = tableKeys[bucket]
            if (k == EMPTY_KEY) return
            if (k == key) {
                tableKeys[bucket] = TOMBSTONE_KEY
                tableSlots[bucket] = -1
                tombstoneCount++
                return
            }
            bucket = (bucket + 1) and HASH_MASK
            probes++
        }
    }

    private fun rehashLocked() {
        tableKeys.fill(EMPTY_KEY)
        tableSlots.fill(-1)
        tombstoneCount = 0
        for (slot in 0 until pointCount) {
            val k = slotToKey[slot]
            if (k == EMPTY_KEY) continue
            var b = mixHash(k) and HASH_MASK
            while (tableKeys[b] != EMPTY_KEY) {
                b = (b + 1) and HASH_MASK
            }
            tableKeys[b] = k
            tableSlots[b] = slot
        }
    }

    fun notifyGlContextRecreated() {
        uploadCursor.reset()
    }

    fun syncToGpuVbo(vboId: Int): Int {
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboId)
        return uploadCursor.sync(pointStore, pageSink)
    }

    private fun packKey(ix: Int, iy: Int, iz: Int): Long {
        val ux = (ix.toLong() + 1_048_576L) and 0x1FFFFFL
        val uy = (iy.toLong() + 1_048_576L) and 0x1FFFFFL
        val uz = (iz.toLong() + 1_048_576L) and 0x1FFFFFL
        return (ux shl 42) or (uy shl 21) or uz
    }

    private fun mixHash(k: Long): Int {
        var z = k
        z = (z xor (z ushr 30)) * -4658895280553007687L
        z = (z xor (z ushr 27)) * -7723592293110705685L
        z = z xor (z ushr 31)
        return z.toInt()
    }
}
