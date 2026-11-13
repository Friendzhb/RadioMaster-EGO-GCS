package com.radiomaster.egogcs.model

import java.lang.reflect.Modifier
import java.util.concurrent.CountDownLatch
import java.util.concurrent.atomic.AtomicReference
import org.junit.Assert.*
import org.junit.Test

class SnapshotExchangeTest {
    @Test fun everyFieldAndNestedArrayIsCopiedWithoutAliasing() {
        val source = RenderSnapshot()
        fill(source)
        val exchange = SnapshotExchange()
        exchange.publish(source)
        val read = exchange.readInto(RenderSnapshot())
        compare(source, read)
        source.voxels.fill(-100f)
        source.swarmAgents[0].trailPts.fill(-100f)
        assertNotEquals(source.voxels[0], read.voxels[0])
        assertNotEquals(source.swarmAgents[0].trailPts[0], read.swarmAgents[0].trailPts[0])
    }

    @Test fun stalledReaderSurvivesManyPublicationsAndConcurrentReadersStayCoherent() {
        val exchange = SnapshotExchange()
        val source = RenderSnapshot()
        val frozen = exchange.readInto(RenderSnapshot())
        val start = CountDownLatch(1)
        val error = AtomicReference<Throwable?>()
        val writer = Thread {
            try {
                start.await()
                for (generation in 1..1000) {
                    source.uavX = generation.toFloat()
                    source.uavY = source.uavX
                    source.voxels.fill(source.uavX)
                    source.swarmAgents[0].trailPts.fill(source.uavX)
                    exchange.publish(source)
                }
            } catch (t: Throwable) {
                error.set(t)
            }
        }
        writer.start()
        start.countDown()
        val own = RenderSnapshot()
        while (writer.isAlive) {
            exchange.readInto(own)
            if (own.uavX > 0f) {
                assertEquals(own.uavX, own.uavY, 0f)
                assertTrue(own.voxels.all { it == own.uavX })
                assertTrue(own.swarmAgents[0].trailPts.all { it == own.uavX })
            }
        }
        writer.join()
        error.get()?.let { throw it }
        assertEquals(-5.5f, frozen.uavX, 0f)
        assertEquals(1000f, exchange.readInto(own).uavX, 0f)
    }

    private fun fill(value: Any) {
        value.javaClass.declaredFields.filterNot { Modifier.isStatic(it.modifiers) }.forEachIndexed { i, f ->
            f.isAccessible = true
            when (val old = f.get(value)) {
                is Float -> f.setFloat(value, i + 0.25f)
                is Int -> f.setInt(value, i + 1)
                is Boolean -> f.setBoolean(value, !old)
                is String -> f.set(value, "field-$i")
                is FloatArray -> old.fill(i + 0.5f)
                is Array<*> -> old.forEach { fill(requireNotNull(it)) }
                else -> error("Uncovered field: ${f.name}")
            }
        }
    }

    private fun compare(a: Any, b: Any) {
        a.javaClass.declaredFields.filterNot { Modifier.isStatic(it.modifiers) }.forEach { f ->
            f.isAccessible = true
            when (val left = f.get(a)) {
                is FloatArray -> {
                    assertNotSame(left, f.get(b))
                    assertArrayEquals(f.name, left, f.get(b) as FloatArray, 0f)
                }
                is Array<*> -> left.forEachIndexed { i, x -> compare(requireNotNull(x), requireNotNull((f.get(b) as Array<*>)[i])) }
                else -> assertEquals(f.name, left, f.get(b))
            }
        }
    }
}
