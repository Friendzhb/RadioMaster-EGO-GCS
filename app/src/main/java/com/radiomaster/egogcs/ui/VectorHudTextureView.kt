package com.radiomaster.egogcs.ui

import android.content.Context
import android.graphics.SurfaceTexture
import android.opengl.EGL14
import android.opengl.EGLConfig
import android.opengl.EGLDisplay
import android.opengl.GLES20
import android.opengl.Matrix
import android.os.Build
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
 * - Supports [preserveEGLContextOnPause] so switching between the main map, telemetry dashboard,
 *   and modal overlays keeps the `EGLContext` and 3D UAV textures/VBOs/shaders resident in VRAM.
 * - Renders both 4M2S Dual-Servo Tilt-Rotor and X4 Standard Research Quadrotor 3D rigs with
 *   zero-allocation pose transfer between UI Choreographer and the EGL worker thread.
 */
internal class VectorHudTextureView(
    context: Context,
    private val onReadyChanged: () -> Unit
) : TextureView(context), TextureView.SurfaceTextureListener {

    @Volatile var preserveEGLContextOnPause: Boolean = true

    @Volatile var modelReady = false
        private set

    private val renderer = VectorGlbRenderer(context)
    internal val textureUploadCount: Int get() = renderer.textureUploadCount

    private val glLock = Object()
    private var worker: HudGlThread? = null
    private var paused = false
    private var activeSurfaceTexture: SurfaceTexture? = null
    private var hasBoundEglSurface = false
    private var frameDirty = false
    private var widthPx = 1
    private var heightPx = 1

    private var hasCachedPose = false
    private var cachedRoll = 0f
    private var cachedPitch = 0f
    private var cachedYaw = 0f
    private var cachedFrontTilt = 0f
    private var cachedRearTilt = 0f
    private var cachedAzimuth = 0f
    private var cachedElevation = 0.42f
    private var cachedTiltRig = true
    private var cachedPh0 = 0.18f
    private var cachedPh1 = 1.02f
    private var cachedPh2 = 2.15f
    private var cachedPh3 = 0.64f
    private var cachedBl0 = 0f
    private var cachedBl1 = 0f
    private var cachedBl2 = 0f
    private var cachedBl3 = 0f
    private var cachedArmed = false
    private var cachedM0 = 0f
    private var cachedM1 = 0f
    private var cachedM2 = 0f
    private var cachedM3 = 0f
    private var cachedLightTheme = false

    init {
        isOpaque = false
        surfaceTextureListener = this
    }

    fun onPause() {
        synchronized(glLock) {
            if (paused) return
            paused = true
            glLock.notifyAll()
        }
    }

    fun onResume() {
        val currentSt = if (isAvailable) surfaceTexture else null
        val w = width
        val h = height
        synchronized(glLock) {
            paused = false
            if (currentSt != null && activeSurfaceTexture == null) {
                activeSurfaceTexture = currentSt
                if (w > 0) widthPx = w
                if (h > 0) heightPx = h
            }
            frameDirty = true
            if (activeSurfaceTexture != null) {
                ensureGlThreadLocked()
            }
            glLock.notifyAll()
        }
    }

    fun releaseGlResources() {
        val threadToStop: HudGlThread?
        synchronized(glLock) {
            threadToStop = worker
            worker = null
            activeSurfaceTexture = null
            if (threadToStop != null) {
                threadToStop.requestExitLocked()
            }
            glLock.notifyAll()
        }
        threadToStop?.awaitExit(500L)
        if (modelReady) {
            modelReady = false
            post { onReadyChanged() }
        }
    }

    override fun onDetachedFromWindow() {
        releaseGlResources()
        super.onDetachedFromWindow()
    }

    fun updatePose(
        roll: Float,
        pitch: Float,
        yaw: Float,
        frontTilt: Float,
        rearTilt: Float,
        cameraAzimuth: Float,
        cameraElevation: Float,
        isTiltRig: Boolean,
        ph0: Float,
        ph1: Float,
        ph2: Float,
        ph3: Float,
        bl0: Float,
        bl1: Float,
        bl2: Float,
        bl3: Float,
        armed: Boolean,
        m0: Float,
        m1: Float,
        m2: Float,
        m3: Float,
        isLightTheme: Boolean
    ) {
        synchronized(glLock) {
            hasCachedPose = true
            cachedRoll = roll
            cachedPitch = pitch
            cachedYaw = yaw
            cachedFrontTilt = frontTilt
            cachedRearTilt = rearTilt
            cachedAzimuth = cameraAzimuth
            cachedElevation = cameraElevation
            cachedTiltRig = isTiltRig
            cachedPh0 = ph0
            cachedPh1 = ph1
            cachedPh2 = ph2
            cachedPh3 = ph3
            cachedBl0 = bl0
            cachedBl1 = bl1
            cachedBl2 = bl2
            cachedBl3 = bl3
            cachedArmed = armed
            cachedM0 = m0
            cachedM1 = m1
            cachedM2 = m2
            cachedM3 = m3
            cachedLightTheme = isLightTheme
            frameDirty = true
            if (!paused && activeSurfaceTexture != null) {
                ensureGlThreadLocked()
            }
            glLock.notifyAll()
        }
    }

    override fun onSurfaceTextureAvailable(surface: SurfaceTexture, width: Int, height: Int) {
        synchronized(glLock) {
            activeSurfaceTexture = surface
            widthPx = width.coerceAtLeast(1)
            heightPx = height.coerceAtLeast(1)
            frameDirty = true
            ensureGlThreadLocked()
            glLock.notifyAll()
        }
    }

    override fun onSurfaceTextureSizeChanged(surface: SurfaceTexture, width: Int, height: Int) {
        synchronized(glLock) {
            widthPx = width.coerceAtLeast(1)
            heightPx = height.coerceAtLeast(1)
            frameDirty = true
            glLock.notifyAll()
        }
    }

    override fun onSurfaceTextureDestroyed(surface: SurfaceTexture): Boolean {
        synchronized(glLock) {
            if (activeSurfaceTexture === surface) {
                activeSurfaceTexture = null
            }
            glLock.notifyAll()
            val curWorker = worker
            if (curWorker != null && Thread.currentThread() !== curWorker) {
                val deadlineNs = System.nanoTime() + 500_000_000L
                while (hasBoundEglSurface && !curWorker.isExitedLocked()) {
                    val waitMs = (deadlineNs - System.nanoTime()) / 1_000_000L
                    if (waitMs <= 0L) break
                    try {
                        glLock.wait(waitMs)
                    } catch (_: InterruptedException) {
                        Thread.currentThread().interrupt()
                        break
                    }
                }
            }
        }
        return true
    }

    override fun onSurfaceTextureUpdated(surface: SurfaceTexture) = Unit

    private fun ensureGlThreadLocked() {
        val existing = worker
        if (existing != null && !existing.isExitedLocked()) return
        val created = HudGlThread()
        worker = created
        created.start()
    }

    private fun updateModelReadyFlag(ready: Boolean, owner: HudGlThread) {
        val changed = synchronized(glLock) {
            if (worker !== owner && ready) {
                false
            } else if (modelReady != ready) {
                modelReady = ready
                true
            } else {
                false
            }
        }
        if (changed) {
            post { onReadyChanged() }
        }
    }

    private inner class HudGlThread : Thread("vector-uav-hud") {
        private var shouldExit = false
        private var exited = false

        fun requestExitLocked() {
            shouldExit = true
        }

        fun isExitedLocked(): Boolean = exited

        fun awaitExit(timeoutMs: Long) {
            if (currentThread() === this) return
            synchronized(glLock) {
                val deadlineNs = System.nanoTime() + timeoutMs * 1_000_000L
                while (!exited) {
                    val remMs = (deadlineNs - System.nanoTime()) / 1_000_000L
                    if (remMs <= 0L) break
                    try {
                        glLock.wait(remMs)
                    } catch (_: InterruptedException) {
                        currentThread().interrupt()
                        break
                    }
                }
            }
        }

        private fun findExactRgba8Config(display: EGLDisplay, attrs: IntArray): EGLConfig? {
            val count = IntArray(1)
            if (!EGL14.eglChooseConfig(display, attrs, 0, null, 0, 0, count, 0) || count[0] <= 0) {
                return null
            }
            val cap = count[0].coerceAtMost(64)
            val configs = arrayOfNulls<EGLConfig>(cap)
            if (!EGL14.eglChooseConfig(display, attrs, 0, configs, 0, cap, count, 0) || count[0] <= 0) {
                return null
            }
            val actual = min(cap, count[0])
            val valBuf = IntArray(1)
            for (i in 0 until actual) {
                val cfg = configs[i] ?: continue
                fun attr(key: Int): Int = if (EGL14.eglGetConfigAttrib(display, cfg, key, valBuf, 0)) valBuf[0] else 0
                val r = attr(EGL14.EGL_RED_SIZE)
                val g = attr(EGL14.EGL_GREEN_SIZE)
                val b = attr(EGL14.EGL_BLUE_SIZE)
                val a = attr(EGL14.EGL_ALPHA_SIZE)
                val d = attr(EGL14.EGL_DEPTH_SIZE)
                val caveat = attr(EGL14.EGL_CONFIG_CAVEAT)
                // 0x3339 = EGL_COLOR_COMPONENT_TYPE_EXT, 0x333B = EGL_COLOR_COMPONENT_TYPE_FLOAT_EXT
                val compType = attr(0x3339)
                if (r == 8 && g == 8 && b == 8 && a == 8 && d >= 16 &&
                    caveat != EGL14.EGL_NON_CONFORMANT_CONFIG && compType != 0x333B
                ) {
                    return cfg
                }
            }
            return configs.firstOrNull { it != null }
        }

        private fun chooseStandardConfig(display: EGLDisplay): EGLConfig {
            val stdAttrs = intArrayOf(
                EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
                EGL14.EGL_SURFACE_TYPE, EGL14.EGL_WINDOW_BIT,
                EGL14.EGL_RED_SIZE, 8, EGL14.EGL_GREEN_SIZE, 8,
                EGL14.EGL_BLUE_SIZE, 8, EGL14.EGL_ALPHA_SIZE, 8,
                EGL14.EGL_DEPTH_SIZE, 16,
                EGL14.EGL_NONE
            )
            return checkNotNull(findExactRgba8Config(display, stdAttrs)) {
                "No compatible RGBA8888 GLES 2.0 EGLConfig for Attitude HUD"
            }
        }

        private fun chooseMsaaConfigOrNull(display: EGLDisplay): EGLConfig? {
            val hw = Build.HARDWARE.lowercase()
            if (hw.contains("ranchu") || hw.contains("goldfish")) {
                return null
            }
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
            return findExactRgba8Config(display, msaaAttrs)
        }

        override fun run() {
            var display = EGL14.EGL_NO_DISPLAY
            var eglContext = EGL14.EGL_NO_CONTEXT
            var eglSurface = EGL14.EGL_NO_SURFACE
            var chosenConfig: EGLConfig? = null
            var boundTexture: SurfaceTexture? = null

            fun destroySurfaceOnly() {
                if (display != EGL14.EGL_NO_DISPLAY && eglSurface != EGL14.EGL_NO_SURFACE) {
                    EGL14.eglMakeCurrent(display, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT)
                    EGL14.eglDestroySurface(display, eglSurface)
                }
                eglSurface = EGL14.EGL_NO_SURFACE
                boundTexture = null
                synchronized(glLock) {
                    hasBoundEglSurface = false
                    glLock.notifyAll()
                }
                updateModelReadyFlag(false, this)
            }

            fun destroyContextAndSurface() {
                if (display != EGL14.EGL_NO_DISPLAY && eglContext != EGL14.EGL_NO_CONTEXT) {
                    if (eglSurface != EGL14.EGL_NO_SURFACE &&
                        EGL14.eglMakeCurrent(display, eglSurface, eglSurface, eglContext)
                    ) {
                        renderer.release()
                    }
                    EGL14.eglMakeCurrent(display, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT)
                    if (eglSurface != EGL14.EGL_NO_SURFACE) {
                        EGL14.eglDestroySurface(display, eglSurface)
                    }
                    EGL14.eglDestroyContext(display, eglContext)
                }
                renderer.release()
                eglSurface = EGL14.EGL_NO_SURFACE
                eglContext = EGL14.EGL_NO_CONTEXT
                chosenConfig = null
                boundTexture = null
                synchronized(glLock) {
                    hasBoundEglSurface = false
                    glLock.notifyAll()
                }
                updateModelReadyFlag(false, this)
            }

            fun bindOrCreateContextAndSurface(dpy: EGLDisplay, st: SurfaceTexture): Boolean {
                val ctxAttrs = intArrayOf(EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE)
                val surfAttrs = intArrayOf(EGL14.EGL_NONE)

                // Fast-path: attach a new window surface to our preserved EGLContext
                if (eglContext != EGL14.EGL_NO_CONTEXT && chosenConfig != null) {
                    val reusedSurf = try {
                        EGL14.eglCreateWindowSurface(dpy, chosenConfig, st, surfAttrs, 0)
                    } catch (_: Exception) {
                        EGL14.EGL_NO_SURFACE
                    }
                    if (reusedSurf != EGL14.EGL_NO_SURFACE &&
                        EGL14.eglMakeCurrent(dpy, reusedSurf, reusedSurf, eglContext)
                    ) {
                        eglSurface = reusedSurf
                        return true
                    }
                    val err = EGL14.eglGetError()
                    if (reusedSurf != EGL14.EGL_NO_SURFACE) {
                        EGL14.eglDestroySurface(dpy, reusedSurf)
                    }
                    if (err == EGL14.EGL_CONTEXT_LOST) {
                        EGL14.eglDestroyContext(dpy, eglContext)
                        eglContext = EGL14.EGL_NO_CONTEXT
                        chosenConfig = null
                        renderer.release()
                    } else {
                        return false
                    }
                }

                val msaaCfg = chooseMsaaConfigOrNull(dpy)
                if (msaaCfg != null) {
                    val tryCtx = EGL14.eglCreateContext(dpy, msaaCfg, EGL14.EGL_NO_CONTEXT, ctxAttrs, 0)
                    val trySurf = if (tryCtx != EGL14.EGL_NO_CONTEXT) {
                        try {
                            EGL14.eglCreateWindowSurface(dpy, msaaCfg, st, surfAttrs, 0)
                        } catch (_: Exception) {
                            EGL14.EGL_NO_SURFACE
                        }
                    } else {
                        EGL14.EGL_NO_SURFACE
                    }
                    if (tryCtx != EGL14.EGL_NO_CONTEXT && trySurf != EGL14.EGL_NO_SURFACE &&
                        EGL14.eglMakeCurrent(dpy, trySurf, trySurf, tryCtx)
                    ) {
                        eglContext = tryCtx
                        eglSurface = trySurf
                        chosenConfig = msaaCfg
                        return true
                    }
                    if (trySurf != EGL14.EGL_NO_SURFACE) EGL14.eglDestroySurface(dpy, trySurf)
                    if (tryCtx != EGL14.EGL_NO_CONTEXT) EGL14.eglDestroyContext(dpy, tryCtx)
                }

                val stdCfg = chooseStandardConfig(dpy)
                val stdCtx = EGL14.eglCreateContext(dpy, stdCfg, EGL14.EGL_NO_CONTEXT, ctxAttrs, 0)
                if (stdCtx == EGL14.EGL_NO_CONTEXT) return false
                val stdSurf = try {
                    EGL14.eglCreateWindowSurface(dpy, stdCfg, st, surfAttrs, 0)
                } catch (_: Exception) {
                    EGL14.EGL_NO_SURFACE
                }
                if (stdSurf == EGL14.EGL_NO_SURFACE || !EGL14.eglMakeCurrent(dpy, stdSurf, stdSurf, stdCtx)) {
                    if (stdSurf != EGL14.EGL_NO_SURFACE) EGL14.eglDestroySurface(dpy, stdSurf)
                    EGL14.eglDestroyContext(dpy, stdCtx)
                    return false
                }
                eglContext = stdCtx
                eglSurface = stdSurf
                chosenConfig = stdCfg
                return true
            }

            try {
                display = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY)
                check(display != EGL14.EGL_NO_DISPLAY)
                check(EGL14.eglInitialize(display, IntArray(2), 0, IntArray(2), 0))

                val view = FloatArray(16)
                val projection = FloatArray(16)
                val viewProjection = FloatArray(16)
                val rotorPhases = FloatArray(4)
                val rotorBlur = FloatArray(4)

                while (true) {
                    var targetTexture: SurfaceTexture? = null
                    var isPausedNow = false
                    var preserveCtx = true
                    var needSurfaceBind = false
                    var readyToDraw = false

                    var w = 1
                    var h = 1
                    var fRoll = 0f
                    var fPitch = 0f
                    var fYaw = 0f
                    var fFront = 0f
                    var fRear = 0f
                    var fAz = 0f
                    var fEl = 0.42f
                    var fTiltRig = true
                    var fArmed = false
                    var fM0 = 0f
                    var fM1 = 0f
                    var fM2 = 0f
                    var fM3 = 0f
                    var fLight = false

                    synchronized(glLock) {
                        while (true) {
                            if (shouldExit) break
                            targetTexture = activeSurfaceTexture
                            isPausedNow = paused
                            preserveCtx = preserveEGLContextOnPause

                            val needReleaseSurface = eglSurface != EGL14.EGL_NO_SURFACE &&
                                (targetTexture == null || targetTexture !== boundTexture || (isPausedNow && !preserveCtx))
                            val needReleaseContext = eglContext != EGL14.EGL_NO_CONTEXT &&
                                isPausedNow && !preserveCtx
                            if (needReleaseSurface || needReleaseContext) {
                                break
                            }

                            if (!isPausedNow && targetTexture != null) {
                                if (eglSurface == EGL14.EGL_NO_SURFACE || boundTexture !== targetTexture) {
                                    needSurfaceBind = true
                                    break
                                }
                                if (hasCachedPose && frameDirty) {
                                    w = widthPx.coerceAtLeast(1)
                                    h = heightPx.coerceAtLeast(1)
                                    fRoll = cachedRoll
                                    fPitch = cachedPitch
                                    fYaw = cachedYaw
                                    fFront = cachedFrontTilt
                                    fRear = cachedRearTilt
                                    fAz = cachedAzimuth
                                    fEl = cachedElevation
                                    fTiltRig = cachedTiltRig
                                    rotorPhases[0] = cachedPh0
                                    rotorPhases[1] = cachedPh1
                                    rotorPhases[2] = cachedPh2
                                    rotorPhases[3] = cachedPh3
                                    rotorBlur[0] = cachedBl0
                                    rotorBlur[1] = cachedBl1
                                    rotorBlur[2] = cachedBl2
                                    rotorBlur[3] = cachedBl3
                                    fArmed = cachedArmed
                                    fM0 = cachedM0
                                    fM1 = cachedM1
                                    fM2 = cachedM2
                                    fM3 = cachedM3
                                    fLight = cachedLightTheme
                                    frameDirty = false
                                    readyToDraw = true
                                    break
                                }
                            }
                            glLock.wait()
                        }
                    }

                    if (shouldExit) break

                    if (isPausedNow && !preserveCtx && eglContext != EGL14.EGL_NO_CONTEXT) {
                        destroyContextAndSurface()
                        continue
                    }
                    if (eglSurface != EGL14.EGL_NO_SURFACE && (targetTexture == null || targetTexture !== boundTexture)) {
                        destroySurfaceOnly()
                        continue
                    }

                    if (needSurfaceBind) {
                        val st = targetTexture ?: continue
                        if (!bindOrCreateContextAndSurface(display, st)) {
                            synchronized(glLock) {
                                if (activeSurfaceTexture === st) {
                                    activeSurfaceTexture = null
                                }
                                hasBoundEglSurface = false
                                glLock.notifyAll()
                            }
                            updateModelReadyFlag(false, this)
                            continue
                        }
                        renderer.create()
                        if (!renderer.ready) {
                            Log.e("VectorHudTextureView", "The HUD 3D UAV model did not load")
                            destroyContextAndSurface()
                            synchronized(glLock) {
                                if (activeSurfaceTexture === st) {
                                    activeSurfaceTexture = null
                                }
                            }
                            continue
                        }
                        boundTexture = st
                        synchronized(glLock) {
                            hasBoundEglSurface = true
                            frameDirty = true
                            glLock.notifyAll()
                        }
                        updateModelReadyFlag(true, this)
                        continue
                    }

                    if (!readyToDraw) continue

                    GLES20.glViewport(0, 0, w, h)
                    GLES20.glClearColor(0f, 0f, 0f, 0f)
                    GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
                    GLES20.glEnable(GLES20.GL_DEPTH_TEST)
                    GLES20.glDepthFunc(GLES20.GL_LEQUAL)
                    GLES20.glDepthMask(true)
                    GLES20.glDisable(GLES20.GL_BLEND)

                    val distance = 2.28f
                    val eyeX = distance * cos(fEl) * cos(fAz)
                    val eyeY = distance * cos(fEl) * sin(fAz)
                    val eyeZ = distance * sin(fEl)
                    Matrix.setLookAtM(view, 0, eyeX, eyeY, eyeZ, 0f, 0f, 0.032f, 0f, 0f, 1f)
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
                        displayScale = 1.08f,
                        occluded = false,
                        spinPhase = rotorPhases[0],
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
                        isLightTheme = fLight,
                        isTiltRig = fTiltRig,
                        rotorPhases = rotorPhases,
                        rotorBlur = rotorBlur
                    )
                    if (!EGL14.eglSwapBuffers(display, eglSurface)) {
                        val err = EGL14.eglGetError()
                        if (err == EGL14.EGL_CONTEXT_LOST) {
                            Log.w("VectorHudTextureView", "HUD EGL context lost; scheduling recreation")
                            destroyContextAndSurface()
                        } else {
                            Log.w("VectorHudTextureView", "HUD EGL swap failed (err=$err); rebinding surface")
                            destroySurfaceOnly()
                        }
                    }
                }
            } catch (e: Exception) {
                Log.e("VectorHudTextureView", "Could not render the 3D UAV model in the HUD", e)
            } finally {
                destroyContextAndSurface()
                if (display != EGL14.EGL_NO_DISPLAY) {
                    EGL14.eglReleaseThread()
                }
                synchronized(glLock) {
                    exited = true
                    hasBoundEglSurface = false
                    if (worker === this) {
                        worker = null
                    }
                    glLock.notifyAll()
                }
            }
        }
    }
}
