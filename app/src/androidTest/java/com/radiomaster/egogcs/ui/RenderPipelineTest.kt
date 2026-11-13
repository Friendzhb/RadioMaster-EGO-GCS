package com.radiomaster.egogcs.ui

import android.opengl.EGL14
import android.opengl.GLES20
import android.view.View
import android.view.ViewGroup
import android.widget.CheckBox
import androidx.lifecycle.Lifecycle
import androidx.test.core.app.ActivityScenario
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import com.radiomaster.egogcs.MainActivity
import com.radiomaster.egogcs.engine.FastLioVoxelMap
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.RenderSnapshot
import java.io.File
import java.nio.ByteBuffer
import org.json.JSONObject
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class RenderPipelineTest {
    @Test fun mainCockpitStartsWithOptionalUdpDisabled() {
        val instrument = InstrumentationRegistry.getInstrumentation()
        instrument.targetContext.getSharedPreferences("ego_native_prefs", 0).edit()
            .putBoolean("autoConnectDiscovered", false)
            .putBoolean("otaAutoCheckOnStart", false)
            .putBoolean("udpTelemetryEnabled", false)
            .commit()
        ActivityScenario.launch(MainActivity::class.java).use { scenario ->
            Thread.sleep(900)
            scenario.onActivity { activity ->
                fun find(view: View): CheckBox? {
                    if (view is CheckBox && view.text.startsWith("UDP 点云")) return view
                    if (view is ViewGroup) {
                        for (i in 0 until view.childCount) find(view.getChildAt(i))?.let { return it }
                    }
                    return null
                }
                val checkbox = find(activity.window.decorView)
                assertNotNull("UDP option should be available in the connection panel", checkbox)
                assertFalse(checkbox!!.isChecked)
                assertFalse(activity.isFinishing)
            }
            instrument.uiAutomation.takeScreenshot()?.let { bitmap ->
                File(instrument.targetContext.filesDir, "cockpit-smoke.png").outputStream().use {
                    bitmap.compress(android.graphics.Bitmap.CompressFormat.PNG, 100, it)
                }
                bitmap.recycle()
            }
        }
    }

    @Test fun cachedAndUncachedGeometryProduceIdenticalPixelsAndLessCpuWork() {
        val instrument = InstrumentationRegistry.getInstrumentation()
        val hub = GcsSharedHub()
        hub.perfMode = 1
        val frame = RenderSnapshot().apply {
            uavX = 0f
            uavY = 0f
            homeX = 0f
            homeY = 0f
            obsPrimitiveCount = 0
            voxelCount = 4096
            for (i in 0 until voxelCount) {
                voxels[i * 4] = (i % 32) * 0.25f - 4f
                voxels[i * 4 + 1] = (i / 32 % 16) * 0.25f - 2f
                voxels[i * 4 + 2] = (i / 512) * 0.25f + 0.125f
                voxels[i * 4 + 3] = 0.1f + if (i >= 3584) 10f else 0f
            }
        }
        hub.snapshots.publish(frame)
        var clock = 1_000_000_000L
        lateinit var view: GlPointCloudMapView
        instrument.runOnMainSync {
            view = GlPointCloudMapView(instrument.targetContext, hub, FastLioVoxelMap(), {}, { false }, { _, _ -> }, { _, _ -> }, { _, _ -> }, { clock })
        }
        val report = JSONObject()
        EglBuffer().use {
            view.onSurfaceCreated(null, null)
            view.onSurfaceChanged(null, 640, 360)
            repeat(90) {
                clock += 16_666_667L
                view.onDrawFrame(null)
            }
            view.onDrawFrame(null)
            val uncached = ArrayList<Float>()
            val cached = ArrayList<Float>()
            for (projection in 0..1) {
                if (projection == 1) {
                    instrument.runOnMainSync { view.toggle3D2DWithAnimation() }
                    Thread.sleep(500)
                }
                for (theme in 0..3) {
                    hub.themeId = theme
                    repeat(45) {
                        clock += 16_666_667L
                        view.onDrawFrame(null)
                    }
                    view.onDrawFrame(null)
                    view.geometryCachingEnabled = false
                    repeat(12) {
                        view.onDrawFrame(null)
                        uncached.add(view.renderMetrics.lastCpuMs)
                    }
                    val expected = pixels()
                    view.geometryCachingEnabled = true
                    repeat(12) {
                        view.onDrawFrame(null)
                        cached.add(view.renderMetrics.lastCpuMs)
                    }
                    assertArrayEquals("Theme $theme", expected, pixels())
                    assertEquals(GLES20.GL_NO_ERROR, GLES20.glGetError())
                }
            }
            instrument.runOnMainSync { view.focusWorldPointSmooth(2f, -1f) }
            repeat(30) {
                clock += 16_666_667L
                view.onDrawFrame(null)
            }
            view.onDrawFrame(null)
            view.geometryCachingEnabled = false
            view.onDrawFrame(null)
            val moved = pixels()
            view.geometryCachingEnabled = true
            view.onDrawFrame(null)
            assertArrayEquals("Camera pan invalidates geometry", moved, pixels())
            // Changing geometry and toggling the optional core must invalidate every affected batch.
            for (core in listOf(true, false)) {
                frame.voxels[2] += 0.25f
                frame.obsPrimitiveCount = if (core) 1 else 0
                frame.obsPrimitives[2] = 0.5f
                frame.obsPrimitives[3] = 0.5f
                frame.obsPrimitives[4] = 2f
                hub.snapshots.publish(frame)
                view.geometryCachingEnabled = false
                view.onDrawFrame(null)
                val expected = pixels()
                view.geometryCachingEnabled = true
                view.onDrawFrame(null)
                assertArrayEquals("Changed voxels / core=$core", expected, pixels())
            }
            uncached.sort()
            cached.sort()
            report.put("comparison", "same renderer, geometry cache off/on, fixed input and animation clock")
            report.put("device", android.os.Build.MODEL)
            report.put("uncached_cpu_p50_ms", uncached[uncached.size / 2])
            report.put("cached_cpu_p50_ms", cached[cached.size / 2])
            report.put("uncached_cpu_p95_ms", uncached[(uncached.size * 0.95).toInt()])
            report.put("cached_cpu_p95_ms", cached[(cached.size * 0.95).toInt()])
            report.put("cache_hits", view.renderMetrics.voxelCacheHits)
            report.put("pixel_comparisons", "4 themes x 2 projections, camera pan, changed voxels, core on/off")
            val runtime = Runtime.getRuntime()
            System.gc()
            Thread.sleep(100)
            val heapBefore = runtime.totalMemory() - runtime.freeMemory()
            repeat(300) { view.onDrawFrame(null) }
            System.gc()
            Thread.sleep(100)
            val heapGrowth = runtime.totalMemory() - runtime.freeMemory() - heapBefore
            report.put("java_heap_growth_after_300_static_frames_bytes", heapGrowth)
            assertTrue("Unexpected retained heap growth: $heapGrowth", heapGrowth < 8 * 1024 * 1024L)
            assertTrue(view.renderMetrics.voxelCacheHits > 40L)
            assertTrue("Cached median regressed: $report", cached[cached.size / 2] <= uncached[uncached.size / 2] * 1.05f)
        }
        instrument.runOnMainSync { view.onPause() }
        File(instrument.targetContext.filesDir, "render-benchmark.json").writeText(report.toString(2))
    }

    @Test fun pointBurstClearAndGlRecreationRespectUploadBudget() {
        val instrument = InstrumentationRegistry.getInstrumentation()
        val map = FastLioVoxelMap()
        val points = FloatArray(300_000 * 4)
        for (i in 0 until 300_000) {
            points[i * 4] = (i % 500) * 0.13f
            points[i * 4 + 1] = (i / 500) * 0.13f
            points[i * 4 + 2] = 1f
            points[i * 4 + 3] = 0.5f
        }
        map.insertScan(points, 300_000)
        lateinit var view: GlPointCloudMapView
        instrument.runOnMainSync {
            view = GlPointCloudMapView(instrument.targetContext, GcsSharedHub(), map, {}, { false }, { _, _ -> }, { _, _ -> }, { _, _ -> })
        }
        repeat(2) {
            EglBuffer().use {
                view.onSurfaceCreated(null, null)
                view.onSurfaceChanged(null, 640, 360)
                repeat(100) {
                    view.onDrawFrame(null)
                    assertTrue(map.uploadedPointsLastFrame <= 16_384)
                    assertTrue(map.uploadedPagesLastFrame <= 4)
                    assertEquals(GLES20.GL_NO_ERROR, GLES20.glGetError())
                }
                assertEquals(300_000, map.pointCount)
                assertEquals(0, map.uploadedPointsLastFrame)
            }
        }
        map.clear()
        assertEquals(0, map.pointCount)
        instrument.runOnMainSync { view.onPause() }
    }

    @Test fun backgroundStopsFramesAndHudThenResumeRestartsThem() {
        ActivityScenario.launch(RenderHarnessActivity::class.java).use { scenario ->
            Thread.sleep(900)
            var frames = 0L
            var hud = 0
            scenario.moveToState(Lifecycle.State.CREATED)
            scenario.onActivity {
                frames = it.map.renderMetrics.frames
                hud = it.hudTicks
            }
            Thread.sleep(350)
            scenario.onActivity {
                assertEquals(frames, it.map.renderMetrics.frames)
                assertEquals(hud, it.hudTicks)
                it.hub.perfMode = 2
            }
            scenario.moveToState(Lifecycle.State.RESUMED)
            Thread.sleep(900)
            scenario.onActivity {
                assertTrue(it.map.renderMetrics.frames > frames)
                assertTrue(it.hudTicks > hud)
            }
        }
    }

    private fun pixels(): ByteArray {
        val b = ByteBuffer.allocateDirect(640 * 360 * 4)
        GLES20.glReadPixels(0, 0, 640, 360, GLES20.GL_RGBA, GLES20.GL_UNSIGNED_BYTE, b)
        return ByteArray(b.capacity()).also {
            b.position(0)
            b.get(it)
        }
    }

    private class EglBuffer : AutoCloseable {
        private val display = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY)
        private val context: android.opengl.EGLContext
        private val surface: android.opengl.EGLSurface
        init {
            check(EGL14.eglInitialize(display, IntArray(2), 0, IntArray(2), 1))
            val configs = arrayOfNulls<android.opengl.EGLConfig>(1)
            check(EGL14.eglChooseConfig(display, intArrayOf(EGL14.EGL_RENDERABLE_TYPE, 4, EGL14.EGL_SURFACE_TYPE, EGL14.EGL_PBUFFER_BIT, EGL14.EGL_RED_SIZE, 8, EGL14.EGL_GREEN_SIZE, 8, EGL14.EGL_BLUE_SIZE, 8, EGL14.EGL_ALPHA_SIZE, 8, EGL14.EGL_DEPTH_SIZE, 16, EGL14.EGL_NONE), 0, configs, 0, 1, IntArray(1), 0))
            context = EGL14.eglCreateContext(display, configs[0], EGL14.EGL_NO_CONTEXT, intArrayOf(EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE), 0)
            surface = EGL14.eglCreatePbufferSurface(display, configs[0], intArrayOf(EGL14.EGL_WIDTH, 640, EGL14.EGL_HEIGHT, 360, EGL14.EGL_NONE), 0)
            check(EGL14.eglMakeCurrent(display, surface, surface, context))
        }
        override fun close() {
            EGL14.eglMakeCurrent(display, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT)
            EGL14.eglDestroySurface(display, surface)
            EGL14.eglDestroyContext(display, context)
            EGL14.eglTerminate(display)
        }
    }
}
