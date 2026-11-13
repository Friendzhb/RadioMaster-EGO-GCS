package com.radiomaster.egogcs.ui

import android.app.Activity
import android.os.Bundle
import com.radiomaster.egogcs.engine.FastLioVoxelMap
import com.radiomaster.egogcs.model.GcsSharedHub

/** Instrumentation fixture with no radio, ROS, flight command, or discovery connection. */
class RenderHarnessActivity : Activity() {
    val hub = GcsSharedHub()
    lateinit var map: GlPointCloudMapView
        private set
    var hudTicks = 0
        private set

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        map = GlPointCloudMapView(this, hub, FastLioVoxelMap(), { hudTicks++ }, { false }, { _, _ -> }, { _, _ -> }, { _, _ -> })
        setContentView(map)
    }

    override fun onResume() {
        super.onResume()
        map.startLoop()
    }

    override fun onPause() {
        map.stopLoop()
        super.onPause()
    }
}
