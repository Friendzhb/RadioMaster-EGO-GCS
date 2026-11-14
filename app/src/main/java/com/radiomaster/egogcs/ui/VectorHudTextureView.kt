package com.radiomaster.egogcs.ui

import android.content.Context
import android.graphics.SurfaceTexture
import android.opengl.EGL14
import android.opengl.EGLConfig
import android.opengl.GLES20
import android.opengl.Matrix
import android.util.Log
import android.view.TextureView
import kotlin.math.PI
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

/** Transparent GLB layer over the attitude card, using the same telemetry as the map. */
internal class VectorHudTextureView(context: Context, private val onReadyChanged: () -> Unit) : TextureView(context), TextureView.SurfaceTextureListener {
    data class Pose(
        val roll: Float,
        val pitch: Float,
        val yaw: Float,
        val frontTilt: Float,
        val rearTilt: Float,
        val cameraAzimuth: Float,
        val cameraElevation: Float
    )

    @Volatile var modelReady = false
        private set

    @Volatile private var worker: HudThread? = null
    private var latestPose: Pose? = null

    init {
        isOpaque = false
        surfaceTextureListener = this
    }

    fun updatePose(pose: Pose) {
        latestPose = pose
        worker?.request(pose)
    }

    override fun onSurfaceTextureAvailable(surface: SurfaceTexture, width: Int, height: Int) {
        val thread = HudThread(surface, width, height)
        worker = thread
        thread.start()
        latestPose?.let { thread.request(it) }
    }

    override fun onSurfaceTextureSizeChanged(surface: SurfaceTexture, width: Int, height: Int) {
        worker?.resize(width, height)
    }

    override fun onSurfaceTextureDestroyed(surface: SurfaceTexture): Boolean {
        modelReady = false
        val old = worker
        worker = null
        // If EGL is still using the texture, release it on that thread after EGL teardown.
        return old?.shutdownAndTakeSurface() != true
    }

    override fun onSurfaceTextureUpdated(surface: SurfaceTexture) = Unit

    private inner class HudThread(private val texture: SurfaceTexture, width: Int, height: Int) : Thread("vector-uav-hud") {
        private val lock = Object()
        private var running = true
        private var finished = false
        private var releaseSurface = false
        private var dirty = false
        private var pose: Pose? = null
        private var widthPx = width
        private var heightPx = height

        fun request(value: Pose) = synchronized(lock) {
            pose = value
            dirty = true
            lock.notifyAll()
        }

        fun resize(width: Int, height: Int) = synchronized(lock) {
            widthPx = width
            heightPx = height
            dirty = true
            lock.notifyAll()
        }

        fun shutdownAndTakeSurface(): Boolean = synchronized(lock) {
            running = false
            lock.notifyAll()
            if (finished) {
                false
            } else {
                releaseSurface = true
                true
            }
        }

        override fun run() {
            var display = EGL14.EGL_NO_DISPLAY
            var eglContext = EGL14.EGL_NO_CONTEXT
            var surface = EGL14.EGL_NO_SURFACE
            try {
                display = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY)
                check(display != EGL14.EGL_NO_DISPLAY)
                check(EGL14.eglInitialize(display, IntArray(2), 0, IntArray(2), 0))
                val attributes = intArrayOf(
                    EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
                    EGL14.EGL_SURFACE_TYPE, EGL14.EGL_WINDOW_BIT,
                    EGL14.EGL_RED_SIZE, 8, EGL14.EGL_GREEN_SIZE, 8,
                    EGL14.EGL_BLUE_SIZE, 8, EGL14.EGL_ALPHA_SIZE, 8,
                    EGL14.EGL_DEPTH_SIZE, 16, EGL14.EGL_NONE
                )
                val configs = arrayOfNulls<EGLConfig>(1)
                val count = IntArray(1)
                check(EGL14.eglChooseConfig(display, attributes, 0, configs, 0, 1, count, 0) && count[0] > 0)
                eglContext = EGL14.eglCreateContext(display, configs[0], EGL14.EGL_NO_CONTEXT, intArrayOf(EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE), 0)
                check(eglContext != EGL14.EGL_NO_CONTEXT)
                surface = EGL14.eglCreateWindowSurface(display, configs[0], texture, intArrayOf(EGL14.EGL_NONE), 0)
                check(surface != EGL14.EGL_NO_SURFACE && EGL14.eglMakeCurrent(display, surface, surface, eglContext))
                val renderer = VectorGlbRenderer(context)
                renderer.create()
                check(renderer.ready) { "The HUD GLB did not load" }
                synchronized(lock) {
                    if (!running) return
                    if (worker === this) modelReady = true
                }
                post { onReadyChanged() }

                val view = FloatArray(16)
                val projection = FloatArray(16)
                val viewProjection = FloatArray(16)
                while (true) {
                    var current: Pose? = null
                    var w = 1
                    var h = 1
                    val stopped: Boolean
                    synchronized(lock) {
                        while (running && (!dirty || pose == null)) lock.wait()
                        stopped = !running
                        if (!stopped) {
                            current = pose
                            w = widthPx.coerceAtLeast(1)
                            h = heightPx.coerceAtLeast(1)
                            dirty = false
                        }
                    }
                    if (stopped) break
                    val frame = current ?: continue
                    GLES20.glViewport(0, 0, w, h)
                    GLES20.glClearColor(0f, 0f, 0f, 0f)
                    GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
                    GLES20.glEnable(GLES20.GL_DEPTH_TEST)
                    GLES20.glDepthFunc(GLES20.GL_LEQUAL)
                    GLES20.glDepthMask(true)
                    GLES20.glDisable(GLES20.GL_BLEND)
                    val az = frame.cameraAzimuth
                    val el = frame.cameraElevation
                    val distance = 3.35f
                    Matrix.setLookAtM(
                        view, 0, distance * cos(el) * cos(az), distance * cos(el) * sin(az), distance * sin(el),
                        0f, 0f, 0f, 0f, 0f, 1f
                    )
                    val focal = min(w * 0.42f, h * 0.47f) * 2.55f
                    val fov = (2.0 * atan(h / (2.0 * focal.coerceAtLeast(1f))) * 180.0 / PI).toFloat()
                    Matrix.perspectiveM(projection, 0, fov, w.toFloat() / h, 0.1f, 50f)
                    Matrix.multiplyMM(viewProjection, 0, projection, 0, view, 0)
                    renderer.draw(
                        viewProjection, 0f, 0f, 0f, frame.roll, frame.pitch, frame.yaw,
                        frame.frontTilt, frame.rearTilt, 0.88f, false
                    )
                    check(EGL14.eglSwapBuffers(display, surface)) { "HUD EGL swap failed" }
                }
            } catch (e: Exception) {
                Log.e("VectorHudTextureView", "Could not render the original UAV model in the HUD", e)
            } finally {
                if (worker === this) modelReady = false
                if (display != EGL14.EGL_NO_DISPLAY) {
                    EGL14.eglMakeCurrent(display, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT)
                    if (surface != EGL14.EGL_NO_SURFACE) EGL14.eglDestroySurface(display, surface)
                    if (eglContext != EGL14.EGL_NO_CONTEXT) EGL14.eglDestroyContext(display, eglContext)
                    // The map also uses EGL_DEFAULT_DISPLAY. Terminating it here would invalidate its context.
                    EGL14.eglReleaseThread()
                }
                val release = synchronized(lock) {
                    finished = true
                    releaseSurface
                }
                if (release) texture.release()
                if (worker === this) post { onReadyChanged() }
            }
        }
    }
}
