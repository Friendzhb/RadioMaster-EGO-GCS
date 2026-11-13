package com.radiomaster.egogcs.engine

import java.util.concurrent.CountDownLatch
import kotlin.concurrent.withLock
import org.junit.Assert.*
import org.junit.Test

class PointUploadTest {
    private fun write(store: PagedPointStore, start: Int, end: Int, value: Float) = store.lock.withLock {
        for (i in start until end) {
            for (c in 0..3) store.vertices[i * 4 + c] = value
            store.written(i, maxOf(store.count, i + 1))
        }
    }

    @Test fun burstLargerThanOldQueueConvergesWithinPerFrameBudget() {
        val store = PagedPointStore(100_000)
        val cursor = PointUploadCursor(store.capacity) { 0L }
        write(store, 0, store.capacity, 3f)
        val gpu = FloatArray(store.capacity * 4)
        val sink = PointPageSink { offset, count, values ->
            assertFalse(store.lock.isHeldByCurrentThread)
            values.get(gpu, offset * 4, count * 4)
        }
        var visible = 0
        repeat(20) {
            visible = cursor.sync(store, sink)
            assertTrue(cursor.uploadedPoints <= 16_384)
            assertTrue(cursor.uploadedPages <= 4)
        }
        assertEquals(store.capacity, visible)
        assertTrue(gpu.all { it == 3f })
        cursor.sync(store, sink)
        assertEquals(0, cursor.uploadedPoints)
        cursor.reset()
        gpu.fill(0f)
        repeat(20) { visible = cursor.sync(store, sink) }
        assertEquals(store.capacity, visible)
        assertTrue(gpu.all { it == 3f })
    }

    @Test fun modificationDuringUploadIsNotAcknowledgedAsNewerData() {
        val store = PagedPointStore(12_000)
        val cursor = PointUploadCursor(store.capacity) { 0L }
        write(store, 0, store.capacity, 1f)
        val gpu = FloatArray(store.capacity * 4)
        var changed = false
        val sink = PointPageSink { offset, count, values ->
            values.get(gpu, offset * 4, count * 4)
            if (!changed) {
                changed = true
                write(store, 0, 5000, 2f)
            }
        }
        repeat(10) { cursor.sync(store, sink) }
        assertTrue(gpu.take(5000 * 4).all { it == 2f })
        assertTrue(gpu.drop(5000 * 4).all { it == 1f })
    }

    @Test fun clearDuringUploadCannotExposeOldGenerationOrUninitializedSlots() {
        val store = PagedPointStore(20_000)
        val cursor = PointUploadCursor(store.capacity) { 0L }
        write(store, 0, store.capacity, 7f)
        var cleared = false
        assertEquals(
            0,
            cursor.sync(
                store,
                PointPageSink { _, _, _ ->
                    if (!cleared) {
                        store.clear()
                        cleared = true
                    }
                }
            )
        )
        write(store, 0, 17, 2f)
        var lastUpload = 0
        val visible = cursor.sync(
            store,
            PointPageSink { _, count, values ->
                assertEquals(17, count)
                while (values.hasRemaining()) assertEquals(2f, values.get(), 0f)
                lastUpload = count
            }
        )
        assertEquals(17, visible)
        assertEquals(17, lastUpload)
    }

    @Test fun slowSinkStopsAfterOnePageButEventuallyUploadsEveryPage() {
        var now = 0L
        val store = PagedPointStore(20_000)
        val cursor = PointUploadCursor(store.capacity) { now }
        write(store, 0, store.capacity, 1f)
        var visible = 0
        repeat(40) {
            visible = cursor.sync(store, PointPageSink { _, _, _ -> now += 3_000_000L })
            assertTrue(cursor.uploadedPages <= 1)
        }
        assertEquals(store.capacity, visible)
    }

    @Test fun busyProducerNeverBlocksTheRenderer() {
        val store = PagedPointStore(100)
        val locked = CountDownLatch(1)
        val release = CountDownLatch(1)
        val producer = Thread {
            store.lock.withLock {
                locked.countDown()
                release.await()
            }
        }
        producer.start()
        locked.await()
        try {
            val cursor = PointUploadCursor(store.capacity)
            assertEquals(0, cursor.sync(store, PointPageSink { _, _, _ -> fail("No upload while producer is busy") }))
            assertTrue(cursor.producerBusy)
        } finally {
            release.countDown()
            producer.join()
        }
    }
}
