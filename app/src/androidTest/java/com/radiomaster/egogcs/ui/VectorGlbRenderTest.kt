package com.radiomaster.egogcs.ui

import android.graphics.Bitmap
import android.opengl.EGL14
import android.opengl.GLES20
import android.opengl.Matrix
import android.view.TextureView
import android.view.View
import android.view.ViewGroup
import androidx.lifecycle.Lifecycle
import androidx.test.core.app.ActivityScenario
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import com.radiomaster.egogcs.MainActivity
import com.radiomaster.egogcs.engine.FastLioVoxelMap
import com.radiomaster.egogcs.model.GcsSharedHub
import java.io.File
import java.nio.ByteBuffer
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicInteger
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class VectorGlbRenderTest {
    @Test fun mainMapLoadsTheOriginalGlb() {
        val instrument = InstrumentationRegistry.getInstrumentation()
        lateinit var map: GlPointCloudMapView
        instrument.runOnMainSync {
            map = GlPointCloudMapView(instrument.targetContext, GcsSharedHub(), FastLioVoxelMap(), {}, { false }, { _, _ -> }, { _, _ -> }, { _, _ -> })
        }
        EglBuffer().use {
            map.onSurfaceCreated(null, null)
            assertTrue("Main map should load the original GLB", map.vectorGlbReady)
            map.onSurfaceChanged(null, 320, 240)
            map.onDrawFrame(null)
            assertEquals(GLES20.GL_NO_ERROR, GLES20.glGetError())
        }
        instrument.runOnMainSync { map.onPause() }
    }

    @Test fun originalGlbRendersBothTiltAxesIndependently() {
        val instrument = InstrumentationRegistry.getInstrumentation()
        EglBuffer().use {
            val renderer = VectorGlbRenderer(instrument.targetContext)
            renderer.create()
            assertTrue("Full GLB should load into GLES", renderer.ready)
            val view = FloatArray(16)
            val projection = FloatArray(16)
            val vp = FloatArray(16)
            Matrix.setLookAtM(view, 0, 1.4f, -1.7f, 1.2f, 0f, 0f, 0f, 0f, 0f, 1f)
            Matrix.perspectiveM(projection, 0, 45f, 4f / 3f, 0.1f, 20f)
            Matrix.multiplyMM(vp, 0, projection, 0, view, 0)
            GLES20.glViewport(0, 0, 320, 240)
            GLES20.glEnable(GLES20.GL_DEPTH_TEST)
            fun draw(front: Float, rear: Float): ByteArray {
                GLES20.glClearColor(0f, 0f, 0f, 1f)
                GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
                renderer.draw(vp, 0f, 0f, 0f, 0f, 0f, 0f, front, rear, 0.45f, false)
                GLES20.glFinish()
                val pixels = ByteBuffer.allocateDirect(320 * 240 * 4)
                GLES20.glReadPixels(0, 0, 320, 240, GLES20.GL_RGBA, GLES20.GL_UNSIGNED_BYTE, pixels)
                assertEquals(GLES20.GL_NO_ERROR, GLES20.glGetError())
                return ByteArray(pixels.capacity()).also {
                    pixels.position(0)
                    pixels.get(it)
                }
            }
            val neutral = draw(0f, 0f)
            val front = draw(0.5235988f, 0f)
            val rear = draw(0f, -0.5235988f)
            assertTrue("Model should produce visible pixels", neutral.indices.any { it % 4 != 3 && neutral[it] != 0.toByte() })
            assertFalse("Front tilt must change the rendered model", neutral.contentEquals(front))
            assertFalse("Rear tilt must change the rendered model", neutral.contentEquals(rear))
            assertFalse("Front and rear movements should be different", front.contentEquals(rear))
        }
    }

    @Test fun attitudeHudLoadsTheSameModel() {
        val instrument = InstrumentationRegistry.getInstrumentation()
        instrument.targetContext.getSharedPreferences("ego_native_prefs", 0).edit()
            .putBoolean("autoConnectDiscovered", false)
            .putBoolean("otaAutoCheckOnStart", false)
            .putBoolean("udpTelemetryEnabled", false)
            .commit()
        ActivityScenario.launch(MainActivity::class.java).use { scenario ->
            var hud: UavAttitudeHudView? = null
            var map: GlPointCloudMapView? = null
            scenario.onActivity { activity ->
                fun find(view: View): UavAttitudeHudView? {
                    if (view is UavAttitudeHudView) return view
                    if (view is ViewGroup) for (i in 0 until view.childCount) find(view.getChildAt(i))?.let { return it }
                    return null
                }
                hud = find(activity.window.decorView)
                fun findMap(view: View): GlPointCloudMapView? {
                    if (view is GlPointCloudMapView) return view
                    if (view is ViewGroup) for (i in 0 until view.childCount) findMap(view.getChildAt(i))?.let { return it }
                    return null
                }
                map = findMap(activity.window.decorView)
            }
            assertNotNull(hud)
            assertNotNull(map)
            val deadline = System.currentTimeMillis() + 30_000
            while (hud?.vectorGlbReady != true && System.currentTimeMillis() < deadline) Thread.sleep(100)
            assertTrue("Attitude HUD should load the GLB", hud?.vectorGlbReady == true)
            var visiblePixels = 0
            while (visiblePixels == 0 && System.currentTimeMillis() < deadline) {
                scenario.onActivity {
                    val texture = hud?.getChildAt(0) as? TextureView
                    texture?.bitmap?.let { bitmap ->
                        for (y in 0 until bitmap.height step 2) {
                            for (x in 0 until bitmap.width step 2) {
                                val pixel = bitmap.getPixel(x, y)
                                if ((pixel ushr 24) != 0 && (pixel and 0x00ffffff) != 0) visiblePixels++
                            }
                        }
                        bitmap.recycle()
                    }
                }
                if (visiblePixels == 0) Thread.sleep(100)
            }
            assertTrue("Attitude HUD must display the model, not just load it", visiblePixels > 0)
            assertTrue("Attitude HUD renderer must remain active", hud?.vectorGlbReady == true)
            instrument.uiAutomation.takeScreenshot()?.let { screenshot ->
                File(instrument.targetContext.filesDir, "vector-uav-hud.png").outputStream().use {
                    screenshot.compress(Bitmap.CompressFormat.PNG, 100, it)
                }
                screenshot.recycle()
            }
            val beforeResume = requireNotNull(map).renderMetrics.frames
            scenario.moveToState(Lifecycle.State.CREATED)
            scenario.moveToState(Lifecycle.State.RESUMED)
            val resumeDeadline = System.currentTimeMillis() + 30_000
            while ((hud?.vectorGlbReady != true || requireNotNull(map).renderMetrics.frames <= beforeResume) && System.currentTimeMillis() < resumeDeadline) Thread.sleep(100)
            assertTrue("Attitude HUD should recover after background and resume", hud?.vectorGlbReady == true)
            assertTrue("Main map should continue drawing after the HUD context is recreated", requireNotNull(map).renderMetrics.frames > beforeResume)
            val glError = AtomicInteger(-1)
            val checked = CountDownLatch(1)
            scenario.onActivity {
                requireNotNull(map).queueEvent {
                    glError.set(GLES20.glGetError())
                    checked.countDown()
                }
                requireNotNull(map).requestRender()
            }
            assertTrue("Main map GL thread should respond", checked.await(15, TimeUnit.SECONDS))
            assertEquals(GLES20.GL_NO_ERROR, glError.get())
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
            surface = EGL14.eglCreatePbufferSurface(display, configs[0], intArrayOf(EGL14.EGL_WIDTH, 320, EGL14.EGL_HEIGHT, 240, EGL14.EGL_NONE), 0)
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
