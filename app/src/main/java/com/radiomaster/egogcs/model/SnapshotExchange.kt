package com.radiomaster.egogcs.model

/** One writer, multiple private readers. No shared frame escapes the short copy lock. */
class SnapshotExchange {
    private val lock = Any()
    private val published = RenderSnapshot()

    val armed: Boolean
        get() = synchronized(lock) { published.armed }

    fun publish(source: RenderSnapshot) {
        synchronized(lock) { published.copyFrom(source) }
    }

    fun readInto(destination: RenderSnapshot): RenderSnapshot {
        synchronized(lock) { destination.copyFrom(published) }
        return destination
    }
}
