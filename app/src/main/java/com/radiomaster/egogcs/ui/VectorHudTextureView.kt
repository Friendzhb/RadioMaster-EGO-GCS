package com.radiomaster.egogcs.ui

import android.content.Context
import android.graphics.SurfaceTexture
import android.opengl.EGL14
import android.opengl.EGLConfig
import android.opengl.EGLDisplay
import android.opengl.GLES20
import android.opengl.Matrix
import android.util.Log
import android.view.TextureView
import kotlin.math.PI
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

/**
 * Transparent 60 FPS OpenGL ES 2.0 layer over the attitude HUD card:
 * - Requests 4x MSAA with automatic fallback for software/older GPUs.
 * - Uses zero-allocation primitive pose transfer between UI Choreographer and the EGL worker thread.
 */
internal class VectorHudTextureView(
    context: Context,
    private val onReadyChanged: () -> Unit
) : TextureView(context), TextureView.SurfaceTextureListener {

    @Volatile var modelReady = false
        private set

    @Volatile private var worker: HudThread? = null
    private var hasCachedPose = false
    private var cachedRoll = 0f
    private var cachedPitch = 0f
    private var cachedYaw = 0f
    private var cachedFrontTilt = 0f
    private var cachedRearTilt = 0f
    private var cachedAzimuth = 0f
    private var cachedElevation = 0.33f
    private var cachedSpinPhase = 0f
    private var cachedArmed = false
    private var cachedM0 = 0.35f
    private var cachedM1 = 0.35f
    private var cachedM2 = 0.35f
    private var cachedM3 = 0.35f
    private var cachedLightTheme = false

    init {
        isOpaque = false
        surfaceTextureListener = this
    }

    fun updatePose(
        roll: Float,
        pitch: Float,
        yaw: Float,
        frontTilt: Float,
        rearTilt: Float,
        cameraAzimuth: Float,
        cameraElevation: Float,
        spinPhase: Float,
        armed: Boolean,
        m0: Float,
        m1: Float,
        m2: Float,
        m3: Float,
        isLightTheme: Boolean
    ) {
        hasCachedPose = true
        cachedRoll = roll
        cachedPitch = pitch
        cachedYaw = yaw
        cachedFrontTilt = frontTilt
        cachedRearTilt = rearTilt
        cachedAzimuth = cameraAzimuth
        cachedElevation = cameraElevation
        cachedSpinPhase = spinPhase
        cachedArmed = armed
        cachedM0 = m0
        cachedM1 = m1
        cachedM2 = m2
        cachedM3 = m3
        cachedLightTheme = isLightTheme
        worker?.request(
            roll, pitch, yaw, frontTilt, rearTilt, cameraAzimuth, cameraElevation,
            spinPhase, armed, m0, m1, m2, m3, isLightTheme
        )
    }

    override fun onSurfaceTextureAvailable(surface: SurfaceTexture, width: Int, height: Int) {
        val thread = HudThread(surface, width, height)
        worker = thread
        thread.start()
        if (hasCachedPose) {
            thread.request(
                cachedRoll, cachedPitch, cachedYaw, cachedFrontTilt, cachedRearTilt,
                cachedAzimuth, cachedElevation, cachedSpinPhase, cachedArmed,
                cachedM0, cachedM1, cachedM2, cachedM3, cachedLightTheme
            )
        }
    }

    override fun onSurfaceTextureSizeChanged(surface: SurfaceTexture, width: Int, height: Int) {
        worker?.resize(width, height)
    }

    override fun onSurfaceTextureDestroyed(surface: SurfaceTexture): Boolean {
        modelReady = false
        val old = worker
        worker = null
        return old?.shutdownAndTakeSurface() != true
    }

    override fun onSurfaceTextureUpdated(surface: SurfaceTexture) = Unit

    private inner class HudThread(
        private val texture: SurfaceTexture,
        width: Int,
        height: Int
    ) : Thread("vector-uav-hud") {
        private val lock = Object()
        private var running = true
        private var finished = false
        private var releaseSurface = false
        private var dirty = false
        private var hasPose = false
        private var widthPx = width
        private var heightPx = height

        private var pRoll = 0f
        private var pPitch = 0f
        private var pYaw = 0f
        private var pFrontTilt = 0f
        private var pRearTilt = 0f
        private var pAzimuth = 0f
        private var pElevation = 0.33f
        private var pSpinPhase = 0f
        private var pArmed = false
        private var pM0 = 0.35f
        private var pM1 = 0.35f
        private var pM2 = 0.35f
        private var pM3 = 0.35f
        private var pLightTheme = false

        fun request(
            roll: Float,
            pitch: Float,
            yaw: Float,
            frontTilt: Float,
            rearTilt: Float,
            az: Float,
            el: Float,
            spinPhase: Float,
            armed: Boolean,
            m0: Float,
            m1: Float,
            m2: Float,
            m3: Float,
            isLightTheme: Boolean
        ) = synchronized(lock) {
            pRoll = roll
            pPitch = pitch
            pYaw = yaw
            pFrontTilt = frontTilt
            pRearTilt = rearTilt
            pAzimuth = az
            pElevation = el
            pSpinPhase = spinPhase
            pArmed = armed
            pM0 = m0
            pM1 = m1
            pM2 = m2
            pM3 = m3
            pLightTheme = isLightTheme
            hasPose = true
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

        private fun chooseStandardConfig(display: EGLDisplay): EGLConfig {
            val configs = arrayOfNulls<EGLConfig>(1)
            val count = IntArray(1)
            val stdAttrs = intArrayOf(
                EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
                EGL14.EGL_SURFACE_TYPE, EGL14.EGL_WINDOW_BIT,
                EGL14.EGL_RED_SIZE, 8, EGL14.EGL_GREEN_SIZE, 8,
                EGL14.EGL_BLUE_SIZE, 8, EGL14.EGL_ALPHA_SIZE, 8,
                EGL14.EGL_DEPTH_SIZE, 16,
                EGL14.EGL_NONE
            )
            check(EGL14.eglChooseConfig(display, stdAttrs, 0, configs, 0, 1, count, 0) && count[0] > 0 && configs[0] != null)
            return requireNotNull(configs[0])
        }

        private fun chooseMsaaConfigOrNull(display: EGLDisplay): EGLConfig? {
            val configs = arrayOfNulls<EGLConfig>(1)
            val count = IntArray(1)
            val msaaAttrs = intArrayOf(
                EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
                EGL14.EGL_SURFACE_TYPE, EGL14.EGL_WINDOW_BIT,
                EGL14.EGL_RED_SIZE, 8, EGL14.EGL_GREEN_SIZE, 8,
                EGL14.EGL_BLUE_SIZE, 8, EGL14.EGL_ALPHA_SIZE, 8,
                EGL14.EGL_DEPTH_SIZE, 16,
                EGL14.EGL_SAMPLE_BUFFERS, 1,
                EGL14.EGL_SAMPLES, 4,
                EGL14.EGL_NONE
            )
            return if (EGL14.eglChooseConfig(display, msaaAttrs, 0, configs, 0, 1, count, 0) && count[0] > 0) {
                configs[0]
            } else {
                null
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
                val ctxAttrs = intArrayOf(EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE)
                val surfAttrs = intArrayOf(EGL14.EGL_NONE)
                val msaaCfg = chooseMsaaConfigOrNull(display)
                if (msaaCfg != null) {
                    val tryCtx = EGL14.eglCreateContext(display, msaaCfg, EGL14.EGL_NO_CONTEXT, ctxAttrs, 0)
                    val trySurf = if (tryCtx != EGL14.EGL_NO_CONTEXT) {
                        EGL14.eglCreateWindowSurface(display, msaaCfg, texture, surfAttrs, 0)
                    } else {
                        EGL14.EGL_NO_SURFACE
                    }
                    if (tryCtx != EGL14.EGL_NO_CONTEXT && trySurf != EGL14.EGL_NO_SURFACE &&
                        EGL14.eglMakeCurrent(display, trySurf, trySurf, tryCtx)
                    ) {
                        eglContext = tryCtx
                        surface = trySurf
                    } else {
                        if (trySurf != EGL14.EGL_NO_SURFACE) EGL14.eglDestroySurface(display, trySurf)
                        if (tryCtx != EGL14.EGL_NO_CONTEXT) EGL14.eglDestroyContext(display, tryCtx)
                    }
                }
                if (eglContext == EGL14.EGL_NO_CONTEXT || surface == EGL14.EGL_NO_SURFACE) {
                    val stdCfg = chooseStandardConfig(display)
                    eglContext = EGL14.eglCreateContext(display, stdCfg, EGL14.EGL_NO_CONTEXT, ctxAttrs, 0)
                    check(eglContext != EGL14.EGL_NO_CONTEXT)
                    surface = EGL14.eglCreateWindowSurface(display, stdCfg, texture, surfAttrs, 0)
                    check(surface != EGL14.EGL_NO_SURFACE && EGL14.eglMakeCurrent(display, surface, surface, eglContext))
                }
                val renderer = VectorGlbRenderer(context)
                renderer.create()
                check(renderer.ready) { "The HUD 3D UAV model did not load" }
                synchronized(lock) {
                    if (!running) return
                    if (worker === this) modelReady = true
                }
                post { onReadyChanged() }

                val view = FloatArray(16)
                val projection = FloatArray(16)
                val viewProjection = FloatArray(16)
                while (true) {
                    var w = 1
                    var h = 1
                    var fRoll = 0f
                    var fPitch = 0f
                    var fYaw = 0f
                    var fFront = 0f
                    var fRear = 0f
                    var fAz = 0f
                    var fEl = 0.33f
                    var fSpin = 0f
                    var fArmed = false
                    var fM0 = 0.35f
                    var fM1 = 0.35f
                    var fM2 = 0.35f
                    var fM3 = 0.35f
                    var fLight = false
                    val stopped: Boolean
                    synchronized(lock) {
                        while (running && (!dirty || !hasPose)) lock.wait()
                        stopped = !running
                        if (!stopped) {
                            w = widthPx.coerceAtLeast(1)
                            h = heightPx.coerceAtLeast(1)
                            fRoll = pRoll
                            fPitch = pPitch
                            fYaw = pYaw
                            fFront = pFrontTilt
                            fRear = pRearTilt
                            fAz = pAzimuth
                            fEl = pElevation
                            fSpin = pSpinPhase
                            fArmed = pArmed
                            fM0 = pM0
                            fM1 = pM1
                            fM2 = pM2
                            fM3 = pM3
                            fLight = pLightTheme
                            dirty = false
                        }
                    }
                    if (stopped) break

                    GLES20.glViewport(0, 0, w, h)
                    GLES20.glClearColor(0f, 0f, 0f, 0f)
                    GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
                    GLES20.glEnable(GLES20.GL_DEPTH_TEST)
                    GLES20.glDepthFunc(GLES20.GL_LEQUAL)
                    GLES20.glDepthMask(true)
                    GLES20.glDisable(GLES20.GL_BLEND)

                    val distance = 3.06f
                    val eyeX = distance * cos(fEl) * cos(fAz)
                    val eyeY = distance * cos(fEl) * sin(fAz)
                    val eyeZ = distance * sin(fEl)
                    Matrix.setLookAtM(view, 0, eyeX, eyeY, eyeZ, 0f, 0f, 0.02f, 0f, 0f, 1f)
                    val focal = min(w * 0.42f, h * 0.47f) * 2.55f
                    val fov = (2.0 * atan(h / (2.0 * focal.coerceAtLeast(1f))) * 180.0 / PI).toFloat()
                    Matrix.perspectiveM(projection, 0, fov, w.toFloat() / h, 0.1f, 50f)
                    Matrix.multiplyMM(viewProjection, 0, projection, 0, view, 0)

                    renderer.draw(
                        viewProjection = viewProjection,
                        x = 0f,
                        y = 0f,
                        z = 0f,
                        roll = fRoll,
                        pitch = fPitch,
                        yaw = fYaw,
                        frontTilt = fFront,
                        rearTilt = fRear,
                        displayScale = 0.94f,
                        occluded = false,
                        spinPhase = fSpin,
                        armed = fArmed,
                        m0 = fM0,
                        m1 = fM1,
                        m2 = fM2,
                        m3 = fM3,
                        showThrustVectors = true,
                        showHeadingCue = false,
                        eyeX = eyeX,
                        eyeY = eyeY,
                        eyeZ = eyeZ,
                        isLightTheme = fLight
                    )
                    check(EGL14.eglSwapBuffers(display, surface)) { "HUD EGL swap failed" }
                }
            } catch (e: Exception) {
                Log.e("VectorHudTextureView", "Could not render the 3D UAV model in the HUD", e)
            } finally {
                if (worker === this) modelReady = false
                if (display != EGL14.EGL_NO_DISPLAY) {
                    EGL14.eglMakeCurrent(display, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT)
                    if (surface != EGL14.EGL_NO_SURFACE) EGL14.eglDestroySurface(display, surface)
                    if (eglContext != EGL14.EGL_NO_CONTEXT) EGL14.eglDestroyContext(display, eglContext)
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
