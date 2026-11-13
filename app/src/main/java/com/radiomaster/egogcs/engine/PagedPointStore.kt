package com.radiomaster.egogcs.engine

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock
import kotlin.math.min

/** CPU point data and page revisions share the map's producer lock. */
class PagedPointStore(val capacity: Int) {
    companion object {
        const val PAGE_POINTS = 4096
        const val VALUES_PER_POINT = 4
    }

    val lock = ReentrantLock()
    internal val vertices = FloatArray(capacity * VALUES_PER_POINT)
    internal val pageVersions = LongArray((capacity + PAGE_POINTS - 1) / PAGE_POINTS)
    internal var count = 0

    @Volatile var generation = 0L
        private set

    /** Called with [lock] held, after all four values have been written. */
    internal fun written(slot: Int, pointCount: Int) {
        pageVersions[slot / PAGE_POINTS]++
        count = pointCount
    }

    internal fun clear() = lock.withLock {
        count = 0
        pageVersions.fill(0L)
        generation++
    }
}

fun interface PointPageSink {
    /** The buffer is valid only for this call. The sink must not retain it. */
    fun upload(startPoint: Int, pointCount: Int, values: FloatBuffer)
}

/** GL-thread-owned cursor: bounded work, no blocking on producers, and no GL call under the lock. */
class PointUploadCursor(
    private val capacity: Int,
    private val nanoTime: () -> Long = System::nanoTime
) {
    companion object {
        const val MAX_PAGES_PER_FRAME = 4
        const val SOFT_BUDGET_NS = 2_000_000L
    }

    private val pageCount = (capacity + PagedPointStore.PAGE_POINTS - 1) / PagedPointStore.PAGE_POINTS
    private val uploadedVersions = LongArray(pageCount) { -1L }
    private val uploadedCounts = IntArray(pageCount)
    private val stagedPages = IntArray(MAX_PAGES_PER_FRAME)
    private val stagedCounts = IntArray(MAX_PAGES_PER_FRAME)
    private val stagedVersions = LongArray(MAX_PAGES_PER_FRAME)
    private val pageValues = PagedPointStore.PAGE_POINTS * PagedPointStore.VALUES_PER_POINT
    private val staging = ByteBuffer.allocateDirect(MAX_PAGES_PER_FRAME * pageValues * 4)
        .order(ByteOrder.nativeOrder()).asFloatBuffer()
    private var seenGeneration = -1L
    private var nextPage = 0
    private var drawableCount = 0

    var uploadedPoints = 0
        private set
    var uploadedPages = 0
        private set
    var producerBusy = false
        private set

    fun reset() {
        seenGeneration = -1L
        drawableCount = 0
    }

    fun sync(store: PagedPointStore, sink: PointPageSink): Int {
        require(store.capacity == capacity)
        val started = nanoTime()
        uploadedPoints = 0
        uploadedPages = 0
        producerBusy = !store.lock.tryLock()
        if (producerBusy) return if (store.generation == seenGeneration) drawableCount else 0

        val generation: Long
        val total: Int
        var staged = 0
        try {
            generation = store.generation
            total = store.count
            if (seenGeneration != generation) {
                seenGeneration = generation
                uploadedVersions.fill(-1L)
                uploadedCounts.fill(0)
                drawableCount = 0
                nextPage = 0
            }
            var inspected = 0
            while (inspected < pageCount && staged < MAX_PAGES_PER_FRAME) {
                val page = nextPage
                nextPage = (nextPage + 1) % pageCount
                inspected++
                val start = page * PagedPointStore.PAGE_POINTS
                val count = min(PagedPointStore.PAGE_POINTS, total - start)
                if (count <= 0 || (uploadedVersions[page] == store.pageVersions[page] && uploadedCounts[page] == count)) continue
                staging.clear()
                staging.position(staged * pageValues)
                staging.put(store.vertices, start * PagedPointStore.VALUES_PER_POINT, count * PagedPointStore.VALUES_PER_POINT)
                stagedPages[staged] = page
                stagedCounts[staged] = count
                stagedVersions[staged] = store.pageVersions[page]
                staged++
            }
        } finally {
            store.lock.unlock()
        }

        for (i in 0 until staged) {
            if (store.generation != generation) break
            if (i > 0 && nanoTime() - started >= SOFT_BUDGET_NS) break
            val page = stagedPages[i]
            val count = stagedCounts[i]
            staging.clear()
            staging.position(i * pageValues)
            staging.limit(i * pageValues + count * PagedPointStore.VALUES_PER_POINT)
            sink.upload(page * PagedPointStore.PAGE_POINTS, count, staging)
            uploadedVersions[page] = stagedVersions[i]
            uploadedCounts[page] = count
            uploadedPoints += count
            uploadedPages++
        }
        if (store.generation != generation) {
            drawableCount = 0
            return 0
        }
        var visible = 0
        for (page in 0 until pageCount) {
            val required = min(PagedPointStore.PAGE_POINTS, total - page * PagedPointStore.PAGE_POINTS)
            if (required <= 0) break
            visible += min(required, uploadedCounts[page])
            if (uploadedCounts[page] < required) break
        }
        drawableCount = visible
        return drawableCount
    }
}
