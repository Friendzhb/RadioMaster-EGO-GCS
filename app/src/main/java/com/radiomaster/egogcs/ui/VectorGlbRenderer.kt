package com.radiomaster.egogcs.ui

import android.content.Context
import android.content.res.AssetManager
import android.opengl.EGL14
import android.opengl.EGLContext
import android.opengl.GLES20
import android.opengl.Matrix
import android.util.Log
import java.io.FileInputStream
import java.io.FileNotFoundException
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.channels.FileChannel
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sin

/**
 * Renders both research UAV configurations in GLES 2.0:
 * - **4M2S Dual-Servo Tilt-Rotor** (`isTiltRig = true`): uses `vector_uav.glb` when bundled, or
 *   synthesizes the full solid 3D research rig (`buildProceduralTiltUav`) with woven carbon chassis,
 *   Livox Mid-360 LiDAR + roll-cage, RealSense D435i stereo camera, Orin heatsink, LiPo + XT60,
 *   anodized gold servo linkages, slotted outrunner motors with copper stator windings, and 3-blade
 *   twisted carbon propellers with high-visibility painted tips.
 * - **X4 Standard Research Quadrotor** (`isTiltRig = false`): shares the same research avionics core
 *   paired with rigid 45-degree tubular carbon booms (`buildProceduralStandardQuad`).
 * - Uploads a mipmapped 64x64 RGBA8 GPU surface detail texture (`GL_TEXTURE_2D`) encoding 2x2 twill
 *   carbon-fiber weave and brushed CNC metallic grain, and tracks the owning `EGLContext` so
 *   `preserveEGLContextOnPause` transitions never reload textures, shaders, or VBOs.
 */
internal class VectorGlbRenderer(context: Context) {
    private val assets = context.applicationContext.assets

    @Volatile private var tiltModel: VectorUavMeshBuilder.PreparedModel? = null

    @Volatile private var quadModel: VectorUavMeshBuilder.PreparedModel? = null
    private var tiltUploaded = false
    private var quadUploaded = false
    private var boundEglContext: EGLContext = EGL14.EGL_NO_CONTEXT
    private var program = 0
    private var tiltVertexBuffer = 0
    private var quadVertexBuffer = 0
    private var materialTextureId = 0
    private var aPosition = -1
    private var aNormal = -1
    private var uMvp = -1
    private var uModel = -1
    private var uColor = -1
    private var uEyePos = -1
    private var uTint = -1
    private var uParams = -1
    private var uDetailTex = -1

    @Volatile var textureUploadCount: Int = 0
        private set

    private val body = FloatArray(16)
    private val front = FloatArray(16)
    private val rear = FloatArray(16)
    private val mvpBody = FloatArray(16)
    private val mvpFront = FloatArray(16)
    private val mvpRear = FloatArray(16)
    private val spinTemp = FloatArray(16)
    private val auxLocal = FloatArray(16)
    private val partMatrix = FloatArray(16)
    private val mvp = FloatArray(16)

    val ready: Boolean
        get() = program != 0 && materialTextureId != 0 &&
            (
                (tiltUploaded && tiltModel != null && tiltVertexBuffer != 0) ||
                    (quadUploaded && quadModel != null && quadVertexBuffer != 0)
                )

    /**
     * Checks whether all compiled shaders, VBOs, and 3D UAV surface textures are already resident in [ctx].
     * Validating both the `EGLContext` handle and `glIsTexture`/`glIsProgram`/`glIsBuffer` prevents
     * redundant texture reloads on preserved contexts while safely detecting genuine `EGL_CONTEXT_LOST`
     * or recycled native handle addresses.
     */
    fun isContextResident(ctx: EGLContext = EGL14.eglGetCurrentContext()): Boolean {
        if (ctx == EGL14.EGL_NO_CONTEXT || ctx != boundEglContext) return false
        if (!ready || tiltVertexBuffer == 0 || quadVertexBuffer == 0) return false
        return GLES20.glIsTexture(materialTextureId) &&
            GLES20.glIsProgram(program) &&
            GLES20.glIsBuffer(tiltVertexBuffer) &&
            GLES20.glIsBuffer(quadVertexBuffer)
    }

    fun create(preferTiltRig: Boolean = true) {
        val currentCtx = EGL14.eglGetCurrentContext()
        if (isContextResident(currentCtx)) {
            return
        }
        // Only delete old GL handles if they belong to the currently active context
        if (currentCtx != EGL14.EGL_NO_CONTEXT && currentCtx == boundEglContext) {
            deleteCurrentGlObjects()
        } else {
            program = 0
            tiltVertexBuffer = 0
            quadVertexBuffer = 0
            materialTextureId = 0
        }
        tiltModel = null
        quadModel = null
        tiltUploaded = false
        quadUploaded = false
        boundEglContext = EGL14.EGL_NO_CONTEXT
        try {
            program = createProgram()
            aPosition = GLES20.glGetAttribLocation(program, "aPosition")
            aNormal = GLES20.glGetAttribLocation(program, "aNormal")
            uMvp = GLES20.glGetUniformLocation(program, "uMvp")
            uModel = GLES20.glGetUniformLocation(program, "uModel")
            uColor = GLES20.glGetUniformLocation(program, "uColor")
            uEyePos = GLES20.glGetUniformLocation(program, "uEyePos")
            uTint = GLES20.glGetUniformLocation(program, "uTint")
            uParams = GLES20.glGetUniformLocation(program, "uParams")
            uDetailTex = GLES20.glGetUniformLocation(program, "uDetailTex")
            check(
                aPosition >= 0 && aNormal >= 0 && uMvp >= 0 && uModel >= 0 &&
                    uColor >= 0 && uEyePos >= 0 && uTint >= 0 && uParams >= 0 && uDetailTex >= 0
            )

            val ids = IntArray(2)
            GLES20.glGenBuffers(2, ids, 0)
            tiltVertexBuffer = ids[0]
            quadVertexBuffer = ids[1]
            check(tiltVertexBuffer != 0 && quadVertexBuffer != 0) { "Could not allocate UAV GL buffers" }
            // Materialize both buffer objects immediately so glIsBuffer() is valid even before secondary rig upload
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, tiltVertexBuffer)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, quadVertexBuffer)

            if (preferTiltRig) {
                val preparedTilt = loadTiltModelStatic(assets)
                uploadStaticModelVbo(tiltVertexBuffer, preparedTilt)
                tiltModel = preparedTilt
                tiltUploaded = true
            } else {
                val preparedQuad = loadQuadModelStatic()
                uploadStaticModelVbo(quadVertexBuffer, preparedQuad)
                quadModel = preparedQuad
                quadUploaded = true
            }
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)

            materialTextureId = uploadMaterialDetailTexture()
            check(materialTextureId != 0) { "Could not upload UAV material detail texture" }
            textureUploadCount++

            boundEglContext = currentCtx
        } catch (e: Exception) {
            Log.e("VectorGlbRenderer", "Could not initialize 3D research UAV renderer", e)
            deleteCurrentGlObjects()
            tiltModel = null
            quadModel = null
            tiltUploaded = false
            quadUploaded = false
            boundEglContext = EGL14.EGL_NO_CONTEXT
        }
    }

    fun release() {
        val currentCtx = EGL14.eglGetCurrentContext()
        if (currentCtx != EGL14.EGL_NO_CONTEXT && currentCtx == boundEglContext) {
            deleteCurrentGlObjects()
        } else {
            program = 0
            tiltVertexBuffer = 0
            quadVertexBuffer = 0
            materialTextureId = 0
        }
        tiltModel = null
        quadModel = null
        tiltUploaded = false
        quadUploaded = false
        boundEglContext = EGL14.EGL_NO_CONTEXT
    }

    private fun deleteCurrentGlObjects() {
        if (materialTextureId != 0) {
            GLES20.glDeleteTextures(1, intArrayOf(materialTextureId), 0)
            materialTextureId = 0
        }
        if (tiltVertexBuffer != 0) {
            GLES20.glDeleteBuffers(1, intArrayOf(tiltVertexBuffer), 0)
            tiltVertexBuffer = 0
        }
        if (quadVertexBuffer != 0) {
            GLES20.glDeleteBuffers(1, intArrayOf(quadVertexBuffer), 0)
            quadVertexBuffer = 0
        }
        if (program != 0) {
            GLES20.glDeleteProgram(program)
            program = 0
        }
    }

    private fun uploadMaterialDetailTexture(): Int {
        val texIds = IntArray(1)
        GLES20.glGenTextures(1, texIds, 0)
        val texId = texIds[0]
        if (texId == 0) return 0
        val pixels = loadMaterialDetailPixels().duplicate().apply { position(0) }
        GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, texId)
        GLES20.glPixelStorei(GLES20.GL_UNPACK_ALIGNMENT, 4)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR_MIPMAP_LINEAR)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_REPEAT)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_REPEAT)
        GLES20.glTexImage2D(
            GLES20.GL_TEXTURE_2D,
            0,
            GLES20.GL_RGBA,
            DETAIL_TEX_SIZE,
            DETAIL_TEX_SIZE,
            0,
            GLES20.GL_RGBA,
            GLES20.GL_UNSIGNED_BYTE,
            pixels
        )
        GLES20.glGenerateMipmap(GLES20.GL_TEXTURE_2D)
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, 0)
        val error = GLES20.glGetError()
        if (error != GLES20.GL_NO_ERROR) {
            GLES20.glDeleteTextures(1, intArrayOf(texId), 0)
            error("UAV detail texture upload failed: $error")
        }
        return texId
    }

    private fun uploadStaticModelVbo(vboId: Int, prepared: VectorUavMeshBuilder.PreparedModel) {
        val byteCount = prepared.totalVertices * VectorUavMeshBuilder.STRIDE_BYTES
        val uploadBuffer = prepared.vertexBuffer.duplicate().apply { position(0) }
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboId)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, byteCount, uploadBuffer, GLES20.GL_STATIC_DRAW)
        val error = GLES20.glGetError()
        check(error == GLES20.GL_NO_ERROR) { "UAV GL buffer upload failed: $error" }
    }

    private fun ensureTiltModelUploaded(): VectorUavMeshBuilder.PreparedModel? {
        if (tiltUploaded && tiltModel != null) return tiltModel
        if (tiltVertexBuffer == 0) return null
        return try {
            val prepared = loadTiltModelStatic(assets)
            uploadStaticModelVbo(tiltVertexBuffer, prepared)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
            tiltModel = prepared
            tiltUploaded = true
            prepared
        } catch (e: Exception) {
            Log.e("VectorGlbRenderer", "Could not upload tilt UAV rig VBO", e)
            null
        }
    }

    private fun ensureQuadModelUploaded(): VectorUavMeshBuilder.PreparedModel? {
        if (quadUploaded && quadModel != null) return quadModel
        if (quadVertexBuffer == 0) return null
        return try {
            val prepared = loadQuadModelStatic()
            uploadStaticModelVbo(quadVertexBuffer, prepared)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
            quadModel = prepared
            quadUploaded = true
            prepared
        } catch (e: Exception) {
            Log.e("VectorGlbRenderer", "Could not upload standard quad UAV rig VBO", e)
            null
        }
    }

    fun draw(
        viewProjection: FloatArray,
        x: Float,
        y: Float,
        z: Float,
        roll: Float,
        pitch: Float,
        yaw: Float,
        frontTilt: Float,
        rearTilt: Float,
        displayScale: Float,
        occluded: Boolean
    ) {
        draw(
            viewProjection = viewProjection,
            x = x,
            y = y,
            z = z,
            roll = roll,
            pitch = pitch,
            yaw = yaw,
            frontTilt = frontTilt,
            rearTilt = rearTilt,
            displayScale = displayScale,
            occluded = occluded,
            spinPhase = 0f,
            armed = false,
            m0 = 0f,
            m1 = 0f,
            m2 = 0f,
            m3 = 0f,
            showThrustVectors = false,
            showHeadingCue = false,
            eyeX = x + 2.2f,
            eyeY = y - 2.2f,
            eyeZ = z + 1.8f,
            isLightTheme = false
        )
    }

    fun draw(
        viewProjection: FloatArray,
        x: Float,
        y: Float,
        z: Float,
        roll: Float,
        pitch: Float,
        yaw: Float,
        frontTilt: Float,
        rearTilt: Float,
        displayScale: Float,
        occluded: Boolean,
        spinPhase: Float,
        armed: Boolean,
        m0: Float,
        m1: Float,
        m2: Float,
        m3: Float,
        showThrustVectors: Boolean,
        showHeadingCue: Boolean,
        eyeX: Float,
        eyeY: Float,
        eyeZ: Float,
        isLightTheme: Boolean,
        tintR: Float = 0f,
        tintG: Float = 0.90f,
        tintB: Float = 0.46f,
        tintStrength: Float = 0f,
        isTiltRig: Boolean = true,
        rotorPhases: FloatArray? = null,
        rotorBlur: FloatArray? = null
    ) {
        if (!ready || !x.isFinite() || !y.isFinite() || !z.isFinite() ||
            !roll.isFinite() || !pitch.isFinite() || !yaw.isFinite() ||
            !frontTilt.isFinite() || !rearTilt.isFinite()
        ) {
            return
        }
        val loaded = (if (isTiltRig) ensureTiltModelUploaded() else ensureQuadModelUploaded()) ?: return
        val activeVbo = if (isTiltRig) tiltVertexBuffer else quadVertexBuffer
        if (activeVbo == 0) return

        Matrix.setIdentityM(body, 0)
        Matrix.translateM(body, 0, x, y, z)
        Matrix.rotateM(body, 0, yaw * RAD_TO_DEG, 0f, 0f, 1f)
        Matrix.rotateM(body, 0, -pitch * RAD_TO_DEG, 0f, 1f, 0f)
        Matrix.rotateM(body, 0, roll * RAD_TO_DEG, 1f, 0f, 0f)
        val scale = displayScale / VectorUavMeshBuilder.ARM_SPAN_RADIUS
        Matrix.scaleM(body, 0, scale, scale, scale)
        Matrix.translateM(body, 0, 0f, 0f, -0.05f)
        if (isTiltRig) {
            rotateGroup(front, body, loaded.frontPivot, frontTilt)
            rotateGroup(rear, body, loaded.rearPivot, rearTilt)
        } else {
            System.arraycopy(body, 0, front, 0, 16)
            System.arraycopy(body, 0, rear, 0, 16)
        }

        GLES20.glUseProgram(program)
        GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, materialTextureId)
        GLES20.glUniform1i(uDetailTex, 0)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, activeVbo)
        GLES20.glEnableVertexAttribArray(aPosition)
        GLES20.glEnableVertexAttribArray(aNormal)
        GLES20.glVertexAttribPointer(aPosition, 3, GLES20.GL_FLOAT, false, VectorUavMeshBuilder.STRIDE_BYTES, 0)
        GLES20.glVertexAttribPointer(aNormal, 3, GLES20.GL_FLOAT, false, VectorUavMeshBuilder.STRIDE_BYTES, 12)

        GLES20.glUniform3f(uEyePos, eyeX, eyeY, eyeZ)
        GLES20.glUniform4f(uTint, tintR, tintG, tintB, tintStrength.coerceIn(0f, 0.85f))
        val darkCarbonLift = if (isLightTheme) 0f else 1f

        if (occluded) {
            GLES20.glEnable(GLES20.GL_BLEND)
        } else {
            GLES20.glDisable(GLES20.GL_BLEND)
            GLES20.glDepthMask(true)
        }

        val hasPhaseArr = rotorPhases != null && rotorPhases.size >= 4
        val hasBlurArr = rotorBlur != null && rotorBlur.size >= 4
        var maxRotorBlur = 0f
        if (hasBlurArr) {
            val rb = requireNotNull(rotorBlur)
            maxRotorBlur = max(max(rb[0], rb[1]), max(rb[2], rb[3]))
        } else if (armed) {
            maxRotorBlur = 0.72f
        }

        Matrix.multiplyMM(mvpBody, 0, viewProjection, 0, body, 0)
        if (isTiltRig) {
            Matrix.multiplyMM(mvpFront, 0, viewProjection, 0, front, 0)
            Matrix.multiplyMM(mvpRear, 0, viewProjection, 0, rear, 0)
        } else {
            System.arraycopy(mvpBody, 0, mvpFront, 0, 16)
            System.arraycopy(mvpBody, 0, mvpRear, 0, 16)
        }

        if (occluded) {
            GLES20.glUniform4f(uParams, 0f, 1f, darkCarbonLift, 0f)
            GLES20.glUniform4f(uColor, 0.16f, 0.88f, 1.0f, 0.38f)
        }

        val parts = loaded.parts
        var lastUploadedGroup = -1
        var pIdx = 0
        while (pIdx < parts.size) {
            val part = parts[pIdx]
            pIdx++
            val groupIdx = part.group
            val group = when (groupIdx) {
                1 -> front
                2 -> rear
                else -> body
            }
            val rIdx = part.rotorIndex
            val phase = if (rIdx in 1..4) {
                val mIdx = rIdx - 1
                if (hasPhaseArr) requireNotNull(rotorPhases)[mIdx] else spinPhase
            } else {
                0f
            }
            if (rIdx in 1..4 && phase != 0f) {
                val hub = loaded.rotorHubs[rIdx - 1]
                val deg = (phase * hub.spinSign * RAD_TO_DEG) % 360f
                System.arraycopy(group, 0, spinTemp, 0, 16)
                Matrix.translateM(spinTemp, 0, hub.x, hub.y, 0f)
                Matrix.rotateM(spinTemp, 0, deg, 0f, 0f, 1f)
                Matrix.translateM(spinTemp, 0, -hub.x, -hub.y, 0f)
                if (part.isIdentityTransform) {
                    Matrix.multiplyMM(mvp, 0, viewProjection, 0, spinTemp, 0)
                    GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
                    GLES20.glUniformMatrix4fv(uModel, 1, false, spinTemp, 0)
                } else {
                    Matrix.multiplyMM(partMatrix, 0, spinTemp, 0, part.transform, 0)
                    Matrix.multiplyMM(mvp, 0, viewProjection, 0, partMatrix, 0)
                    GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
                    GLES20.glUniformMatrix4fv(uModel, 1, false, partMatrix, 0)
                }
                lastUploadedGroup = -1
            } else if (part.isIdentityTransform) {
                if (lastUploadedGroup != groupIdx) {
                    val gMvp = when (groupIdx) {
                        1 -> mvpFront
                        2 -> mvpRear
                        else -> mvpBody
                    }
                    GLES20.glUniformMatrix4fv(uMvp, 1, false, gMvp, 0)
                    GLES20.glUniformMatrix4fv(uModel, 1, false, group, 0)
                    lastUploadedGroup = groupIdx
                }
            } else {
                Matrix.multiplyMM(partMatrix, 0, group, 0, part.transform, 0)
                Matrix.multiplyMM(mvp, 0, viewProjection, 0, partMatrix, 0)
                GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
                GLES20.glUniformMatrix4fv(uModel, 1, false, partMatrix, 0)
                lastUploadedGroup = -1
            }
            if (!occluded) {
                val partTintBoost = when {
                    rIdx > 0 -> 0f
                    part.emissive > 0.25f -> tintStrength * 0.15f
                    groupIdx == 0 && tintStrength > 0.01f -> tintStrength
                    else -> tintStrength * 0.35f
                }
                GLES20.glUniform4f(uTint, tintR, tintG, tintB, partTintBoost)
                GLES20.glUniform4f(uParams, part.specular, 0f, darkCarbonLift, part.emissive)
                GLES20.glUniform4fv(uColor, 1, part.color, 0)
            }
            GLES20.glDrawArrays(GLES20.GL_TRIANGLES, part.firstVertex, part.vertexCount)
        }

        val anyRotorActive = maxRotorBlur > 0.008f
        val drawActiveThrust = showThrustVectors && (armed || anyRotorActive)

        if (!occluded && (anyRotorActive || drawActiveThrust || showHeadingCue)) {
            GLES20.glEnable(GLES20.GL_BLEND)
            GLES20.glBlendFuncSeparate(
                GLES20.GL_SRC_ALPHA,
                GLES20.GL_ONE_MINUS_SRC_ALPHA,
                GLES20.GL_ONE,
                GLES20.GL_ONE_MINUS_SRC_ALPHA
            )
            GLES20.glDepthMask(false)
            GLES20.glUniform4f(uTint, 0f, 0f, 0f, 0f)

            if (anyRotorActive || drawActiveThrust) {
                drawRotorOverlaysAndThrustVectors(
                    loaded = loaded,
                    viewProjection = viewProjection,
                    spinPhase = spinPhase,
                    rotorPhases = rotorPhases,
                    rotorBlur = rotorBlur,
                    armed = armed,
                    m0 = m0,
                    m1 = m1,
                    m2 = m2,
                    m3 = m3,
                    drawActiveThrust = drawActiveThrust,
                    isLightTheme = isLightTheme,
                    darkCarbonLift = darkCarbonLift
                )
            }

            if (showHeadingCue) {
                GLES20.glUniform4f(uParams, 0f, 2f, darkCarbonLift, 0f)
                Matrix.setIdentityM(auxLocal, 0)
                Matrix.translateM(auxLocal, 0, 0.118f, 0f, 0.056f)
                Matrix.scaleM(auxLocal, 0, 0.068f, 0.042f, 1f)
                Matrix.multiplyMM(partMatrix, 0, body, 0, auxLocal, 0)
                Matrix.multiplyMM(mvp, 0, viewProjection, 0, partMatrix, 0)
                GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
                GLES20.glUniformMatrix4fv(uModel, 1, false, partMatrix, 0)
                val hr = if (tintStrength > 0.05f) tintR else 0.08f
                val hg = if (tintStrength > 0.05f) tintG else 0.90f
                val hb = if (tintStrength > 0.05f) tintB else 1.00f
                GLES20.glUniform4f(uColor, hr, hg, hb, if (isLightTheme) 0.88f else 0.82f)
                GLES20.glDrawArrays(GLES20.GL_TRIANGLES, loaded.noseChevron.firstVertex, loaded.noseChevron.vertexCount)
            }

            GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA)
            GLES20.glDepthMask(true)
        } else if (occluded && drawActiveThrust) {
            GLES20.glUniform4f(uParams, 0f, 2f, darkCarbonLift, 0f)
            drawOccludedThrustVectors(loaded, viewProjection, m0, m1, m2, m3)
        }

        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, 0)
        GLES20.glDisableVertexAttribArray(aPosition)
        GLES20.glDisableVertexAttribArray(aNormal)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
    }

    private fun drawRotorOverlaysAndThrustVectors(
        loaded: VectorUavMeshBuilder.PreparedModel,
        viewProjection: FloatArray,
        spinPhase: Float,
        rotorPhases: FloatArray?,
        rotorBlur: FloatArray?,
        armed: Boolean,
        m0: Float,
        m1: Float,
        m2: Float,
        m3: Float,
        drawActiveThrust: Boolean,
        isLightTheme: Boolean,
        darkCarbonLift: Float
    ) {
        val hubs = loaded.rotorHubs
        val hasPhases = rotorPhases != null && rotorPhases.size >= 4
        val hasBlurs = rotorBlur != null && rotorBlur.size >= 4

        var i = 0
        while (i < 4) {
            val hub = hubs[i]
            val group = when (hub.group) {
                1 -> front
                2 -> rear
                else -> body
            }
            val throttle = when (i) {
                0 -> m0
                1 -> m1
                2 -> m2
                else -> m3
            }.coerceIn(0f, 1f)
            val phase = if (hasPhases) requireNotNull(rotorPhases)[i] else spinPhase
            val blur = if (hasBlurs) {
                requireNotNull(rotorBlur)[i].coerceIn(0f, 1f)
            } else if (armed) {
                (0.45f + throttle * 0.50f).coerceIn(0f, 1f)
            } else {
                0f
            }
            i++

            if (blur > 0.008f) {
                // Mode 3.0: Per-vertex feathered motion-blur disc, directional 3-blade wake & tip-vortex ring
                GLES20.glUniform4f(uParams, 0f, 3f, darkCarbonLift, 0f)

                val navR = if (hub.isFront) 1.00f else 0.08f
                val navG = if (hub.isFront) 0.36f else 0.94f
                val navB = if (hub.isFront) 0.14f else 0.54f

                Matrix.setIdentityM(auxLocal, 0)
                Matrix.translateM(auxLocal, 0, hub.x, hub.y, hub.z + 0.0018f)
                Matrix.rotateM(auxLocal, 0, (phase * hub.spinSign * RAD_TO_DEG) % 360f, 0f, 0f, 1f)
                Matrix.scaleM(auxLocal, 0, hub.radius, hub.radius, 1f)
                Matrix.multiplyMM(partMatrix, 0, group, 0, auxLocal, 0)
                Matrix.multiplyMM(mvp, 0, viewProjection, 0, partMatrix, 0)
                GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
                GLES20.glUniformMatrix4fv(uModel, 1, false, partMatrix, 0)

                val discAlpha = (if (isLightTheme) 0.20f else 0.16f) * blur
                GLES20.glUniform4f(uColor, navR, navG, navB, discAlpha)
                GLES20.glDrawArrays(GLES20.GL_TRIANGLES, loaded.rotorDisc.firstVertex, loaded.rotorDisc.vertexCount)

                val sweepSpan = if (hub.spinSign >= 0) loaded.rotorSweepCcw else loaded.rotorSweepCw
                val sweepBase = if (isLightTheme) 0.24f else 0.92f
                val sweepAlpha = (if (isLightTheme) 0.34f else 0.32f) * blur
                GLES20.glUniform4f(
                    uColor,
                    sweepBase,
                    minOf(1f, sweepBase + 0.04f),
                    minOf(1f, sweepBase + 0.08f),
                    sweepAlpha
                )
                GLES20.glDrawArrays(GLES20.GL_TRIANGLES, sweepSpan.firstVertex, sweepSpan.vertexCount)

                // Luminous painted blade-tip streak arc (outer 77%..99% span)
                val tipSpan = if (hub.spinSign >= 0) loaded.rotorTipBandCcw else loaded.rotorTipBandCw
                val tipR = if (hub.isFront) 1.00f else 0.14f
                val tipG = if (hub.isFront) 0.56f else 0.98f
                val tipB = if (hub.isFront) 0.10f else 0.60f
                val tipAlpha = (if (isLightTheme) 0.64f else 0.62f) * blur
                GLES20.glUniform4f(uColor, tipR, tipG, tipB, tipAlpha)
                GLES20.glDrawArrays(GLES20.GL_TRIANGLES, tipSpan.firstVertex, tipSpan.vertexCount)

                val ringAlpha = (if (isLightTheme) 0.78f else 0.74f) * blur
                GLES20.glUniform4f(uColor, navR, navG, navB, ringAlpha)
                GLES20.glDrawArrays(GLES20.GL_TRIANGLES, loaded.rotorRing.firstVertex, loaded.rotorRing.vertexCount)
            }

            if (drawActiveThrust && (armed || blur > 0.02f)) {
                GLES20.glUniform4f(uParams, 0f, 2f, darkCarbonLift, 0f)
                val effThrottle = max(throttle, blur * 0.42f)
                val shaftLen = 0.034f + effThrottle * 0.058f
                val vr = when (hub.group) {
                    1 -> 1.00f
                    2 -> 0.14f
                    else -> if (hub.isFront) 1.00f else 0.10f
                }
                val vg = when (hub.group) {
                    1 -> 0.78f
                    2 -> 0.80f
                    else -> if (hub.isFront) 0.56f else 0.92f
                }
                val vb = when (hub.group) {
                    1 -> 0.06f
                    2 -> 1.00f
                    else -> if (hub.isFront) 0.16f else 0.62f
                }
                val va = (if (armed) 0.86f else 0.72f * blur).coerceIn(0f, 0.90f)

                Matrix.setIdentityM(auxLocal, 0)
                Matrix.translateM(auxLocal, 0, hub.x, hub.y, hub.z + 0.004f)
                Matrix.scaleM(auxLocal, 0, 0.0028f, 0.0028f, shaftLen)
                Matrix.multiplyMM(partMatrix, 0, group, 0, auxLocal, 0)
                Matrix.multiplyMM(mvp, 0, viewProjection, 0, partMatrix, 0)
                GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
                GLES20.glUniformMatrix4fv(uModel, 1, false, partMatrix, 0)
                GLES20.glUniform4f(uColor, vr, vg, vb, va)
                GLES20.glDrawArrays(GLES20.GL_TRIANGLES, loaded.thrustShaft.firstVertex, loaded.thrustShaft.vertexCount)

                Matrix.setIdentityM(auxLocal, 0)
                Matrix.translateM(auxLocal, 0, hub.x, hub.y, hub.z + 0.004f + shaftLen)
                Matrix.scaleM(auxLocal, 0, 0.0070f, 0.0070f, 0.018f)
                Matrix.multiplyMM(partMatrix, 0, group, 0, auxLocal, 0)
                Matrix.multiplyMM(mvp, 0, viewProjection, 0, partMatrix, 0)
                GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
                GLES20.glUniformMatrix4fv(uModel, 1, false, partMatrix, 0)
                GLES20.glUniform4f(uColor, vr, vg, vb, minOf(1f, va + 0.08f))
                GLES20.glDrawArrays(GLES20.GL_TRIANGLES, loaded.thrustCone.firstVertex, loaded.thrustCone.vertexCount)
            }
        }
    }

    private fun drawOccludedThrustVectors(
        loaded: VectorUavMeshBuilder.PreparedModel,
        viewProjection: FloatArray,
        m0: Float,
        m1: Float,
        m2: Float,
        m3: Float
    ) {
        val hubs = loaded.rotorHubs
        var i = 0
        while (i < 4) {
            val hub = hubs[i]
            val group = when (hub.group) {
                1 -> front
                2 -> rear
                else -> body
            }
            val throttle = when (i) {
                0 -> m0
                1 -> m1
                2 -> m2
                else -> m3
            }.coerceIn(0f, 1f)
            i++
            val shaftLen = 0.036f + throttle * 0.056f
            val vr = if (hub.isFront) 1.00f else 0.18f
            val vg = if (hub.isFront) 0.78f else 0.86f
            val vb = if (hub.isFront) 0.12f else 1.00f

            Matrix.setIdentityM(auxLocal, 0)
            Matrix.translateM(auxLocal, 0, hub.x, hub.y, hub.z + 0.004f)
            Matrix.scaleM(auxLocal, 0, 0.0028f, 0.0028f, shaftLen)
            Matrix.multiplyMM(partMatrix, 0, group, 0, auxLocal, 0)
            Matrix.multiplyMM(mvp, 0, viewProjection, 0, partMatrix, 0)
            GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
            GLES20.glUniformMatrix4fv(uModel, 1, false, partMatrix, 0)
            GLES20.glUniform4f(uColor, vr, vg, vb, 0.55f)
            GLES20.glDrawArrays(GLES20.GL_TRIANGLES, loaded.thrustShaft.firstVertex, loaded.thrustShaft.vertexCount)
        }
    }

    private fun rotateGroup(out: FloatArray, base: FloatArray, pivot: FloatArray, radians: Float) {
        System.arraycopy(base, 0, out, 0, 16)
        Matrix.translateM(out, 0, pivot[0], pivot[1], pivot[2])
        Matrix.rotateM(out, 0, radians * RAD_TO_DEG, 0f, 1f, 0f)
        Matrix.translateM(out, 0, -pivot[0], -pivot[1], -pivot[2])
    }

    private fun createProgram(): Int {
        fun shader(type: Int, source: String): Int {
            val id = GLES20.glCreateShader(type)
            GLES20.glShaderSource(id, source)
            GLES20.glCompileShader(id)
            val ok = IntArray(1)
            GLES20.glGetShaderiv(id, GLES20.GL_COMPILE_STATUS, ok, 0)
            if (ok[0] == 0) {
                val message = GLES20.glGetShaderInfoLog(id)
                GLES20.glDeleteShader(id)
                error("UAV shader compilation failed: $message")
            }
            return id
        }
        val vs = shader(GLES20.GL_VERTEX_SHADER, VERTEX_SHADER)
        val fs = shader(GLES20.GL_FRAGMENT_SHADER, FRAGMENT_SHADER)
        val result = GLES20.glCreateProgram()
        GLES20.glAttachShader(result, vs)
        GLES20.glAttachShader(result, fs)
        GLES20.glLinkProgram(result)
        val ok = IntArray(1)
        GLES20.glGetProgramiv(result, GLES20.GL_LINK_STATUS, ok, 0)
        GLES20.glDeleteShader(vs)
        GLES20.glDeleteShader(fs)
        if (ok[0] == 0) {
            val message = GLES20.glGetProgramInfoLog(result)
            GLES20.glDeleteProgram(result)
            error("UAV program link failed: $message")
        }
        return result
    }

    companion object {
        internal const val DETAIL_TEX_SIZE = 64
        private const val RAD_TO_DEG = 57.2957795f
        private val TILT_MODEL_LOCK = Any()
        private val QUAD_MODEL_LOCK = Any()
        private val DETAIL_TEX_LOCK = Any()
        private val prewarmStarted = AtomicBoolean(false)

        @Volatile private var cachedTiltModel: VectorUavMeshBuilder.PreparedModel? = null

        @Volatile private var cachedQuadModel: VectorUavMeshBuilder.PreparedModel? = null

        @Volatile private var cachedDetailPixels: ByteBuffer? = null

        /**
         * Pre-synthesizes CPU-side procedural UAV meshes and material detail texture in parallel
         * with Activity UI inflation so `GLSurfaceView.onSurfaceCreated()` hits warm RAM caches.
         */
        internal fun prewarmCpuCachesAsync(context: Context, preferTiltFirst: Boolean = true) {
            if (cachedTiltModel != null && cachedQuadModel != null && cachedDetailPixels != null) return
            if (!prewarmStarted.compareAndSet(false, true)) return
            val appAssets = context.applicationContext.assets
            Thread(
                {
                    try {
                        loadMaterialDetailPixels()
                        if (preferTiltFirst) {
                            loadTiltModelStatic(appAssets)
                            loadQuadModelStatic()
                        } else {
                            loadQuadModelStatic()
                            loadTiltModelStatic(appAssets)
                        }
                    } catch (t: Throwable) {
                        Log.w("VectorGlbRenderer", "Background UAV mesh prewarm skipped", t)
                    } finally {
                        prewarmStarted.set(false)
                    }
                },
                "EgoUavMeshPrewarm"
            ).apply {
                isDaemon = true
                priority = Thread.NORM_PRIORITY
                start()
            }
        }

        private fun loadTiltModelStatic(assets: AssetManager): VectorUavMeshBuilder.PreparedModel {
            cachedTiltModel?.let { return it }
            synchronized(TILT_MODEL_LOCK) {
                cachedTiltModel?.let { return it }
                val built = try {
                    assets.openFd("vector_uav.glb").use { descriptor ->
                        FileInputStream(descriptor.fileDescriptor).channel.use { channel ->
                            val mapped = channel.map(FileChannel.MapMode.READ_ONLY, descriptor.startOffset, descriptor.length)
                            val parsed = VectorGlbAsset.parse(mapped)
                            val binary = mapped.duplicate()
                            binary.position(parsed.binaryOffset)
                            binary.limit(parsed.binaryOffset + parsed.binaryLength)
                            VectorUavMeshBuilder.prepareFromGlb(parsed, binary.slice())
                        }
                    }
                } catch (_: FileNotFoundException) {
                    VectorUavMeshBuilder.buildProceduralTiltUav()
                } catch (e: Exception) {
                    Log.w("VectorGlbRenderer", "Falling back to built-in 3D CAD tilt-rotor UAV rig", e)
                    VectorUavMeshBuilder.buildProceduralTiltUav()
                }
                cachedTiltModel = built
                return built
            }
        }

        private fun loadQuadModelStatic(): VectorUavMeshBuilder.PreparedModel {
            cachedQuadModel?.let { return it }
            synchronized(QUAD_MODEL_LOCK) {
                cachedQuadModel?.let { return it }
                val built = VectorUavMeshBuilder.buildProceduralStandardQuad()
                cachedQuadModel = built
                return built
            }
        }

        /**
         * Synthesizes and caches a seamless 64x64 RGBA8 procedural material detail texture:
         * - R: 2x2 twill woven carbon-fiber relief (warp/weft tow alternation + filament micro-grooves)
         * - G: Brushed CNC aluminum / titanium linear micro-grain
         * - B: Anodized metallic micro-specular sheen
         * - A: Opaque (255)
         */
        internal fun loadMaterialDetailPixels(): ByteBuffer {
            cachedDetailPixels?.let { return it.duplicate().order(ByteOrder.nativeOrder()).apply { position(0) } }
            synchronized(DETAIL_TEX_LOCK) {
                cachedDetailPixels?.let { return it.duplicate().order(ByteOrder.nativeOrder()).apply { position(0) } }
                val buf = ByteBuffer.allocateDirect(DETAIL_TEX_SIZE * DETAIL_TEX_SIZE * 4).order(ByteOrder.nativeOrder())
                val cellSize = 8
                val twoPi = (PI * 2.0).toFloat()
                for (y in 0 until DETAIL_TEX_SIZE) {
                    val ty = y / cellSize
                    val ly = (y % cellSize + 0.5f) / cellSize
                    val vNorm = y.toFloat() / DETAIL_TEX_SIZE
                    for (x in 0 until DETAIL_TEX_SIZE) {
                        val tx = x / cellSize
                        val lx = (x % cellSize + 0.5f) / cellSize
                        val uNorm = x.toFloat() / DETAIL_TEX_SIZE

                        val isWarpOver = ((tx + ty) and 3) < 2
                        val across = if (isWarpOver) lx else ly
                        val along = if (isWarpOver) ly else lx
                        val towCrown = sin(across * PI.toFloat()).coerceIn(0f, 1f)
                        val filament = 0.90f + 0.10f * sin(across * twoPi * 3f)
                        val crimp = 0.88f + 0.12f * cos((along - 0.5f) * PI.toFloat())
                        val orientBias = if (isWarpOver) 1.04f else 0.94f
                        val carbonVal = (0.28f + 0.68f * towCrown * filament * crimp * orientBias).coerceIn(0f, 1f)

                        val grain1 = sin(uNorm * twoPi * 16f + sin(vNorm * twoPi * 2f) * 0.35f)
                        val grain2 = cos(vNorm * twoPi * 24f + uNorm * twoPi * 4f)
                        val brushedVal = (0.54f + 0.24f * grain1 + 0.16f * grain2).coerceIn(0f, 1f)

                        val glintVal = (0.42f + 0.38f * towCrown * towCrown + 0.20f * max(0f, grain1)).coerceIn(0f, 1f)

                        buf.put((carbonVal * 255f).roundToInt().coerceIn(0, 255).toByte())
                        buf.put((brushedVal * 255f).roundToInt().coerceIn(0, 255).toByte())
                        buf.put((glintVal * 255f).roundToInt().coerceIn(0, 255).toByte())
                        buf.put(0xFF.toByte())
                    }
                }
                buf.position(0)
                cachedDetailPixels = buf
                return buf.duplicate().order(ByteOrder.nativeOrder()).apply { position(0) }
            }
        }

        private const val VERTEX_SHADER = """
            attribute vec3 aPosition;
            attribute vec3 aNormal;
            uniform mat4 uMvp;
            uniform mat4 uModel;
            varying vec3 vWorldPos;
            varying vec3 vWorldNormal;
            varying vec3 vLocalPos;
            varying float vFade;
            void main() {
                vec4 wp = uModel * vec4(aPosition, 1.0);
                vWorldPos = wp.xyz;
                vWorldNormal = (uModel * vec4(aNormal, 0.0)).xyz;
                vLocalPos = aPosition;
                vFade = clamp(aNormal.z, 0.0, 1.0);
                gl_Position = uMvp * vec4(aPosition, 1.0);
            }
        """
        private const val FRAGMENT_SHADER = """
            precision mediump float;
            uniform vec4 uColor;
            uniform vec3 uEyePos;
            uniform vec4 uTint;
            uniform vec4 uParams;
            uniform sampler2D uDetailTex;
            varying vec3 vWorldPos;
            varying vec3 vWorldNormal;
            varying vec3 vLocalPos;
            varying float vFade;
            void main() {
                if (uParams.y > 2.5) {
                    gl_FragColor = vec4(uColor.rgb, uColor.a * vFade);
                    return;
                }
                vec3 n = normalize(vWorldNormal);
                vec3 v = normalize(uEyePos - vWorldPos);
                if (uParams.y > 1.5) {
                    float glow = 0.86 + 0.14 * abs(dot(n, v));
                    gl_FragColor = vec4(min(uColor.rgb * glow, vec3(1.0)), uColor.a);
                    return;
                }
                if (uParams.y > 0.5) {
                    float f0 = clamp(1.0 - abs(dot(n, v)), 0.0, 1.0);
                    float fres = f0 * sqrt(max(f0, 0.0));
                    vec3 holo = mix(vec3(0.15, 0.88, 1.0), uTint.rgb, uTint.a * 0.65);
                    gl_FragColor = vec4(min(holo * (0.72 + 0.52 * fres), vec3(1.0)), clamp(0.24 + 0.46 * fres, 0.22, 0.72));
                    return;
                }
                if (dot(n, v) < 0.0) {
                    n = -n;
                }
                vec2 detailUv = (vLocalPos.xy + vec2(vLocalPos.z * 0.65, -vLocalPos.z * 0.65)) * 28.0;
                vec4 detail = texture2D(uDetailTex, detailUv);
                float nonEmissive = 1.0 - clamp(uParams.w * 2.2, 0.0, 1.0);
                float metalness = smoothstep(0.52, 0.82, uParams.x);
                float microRelief = mix(detail.r, detail.g, metalness);
                float texMod = mix(1.0, 0.86 + 0.28 * microRelief, nonEmissive * (1.0 - uTint.a * 0.65));
                float specTex = mix(1.0, 0.78 + 0.44 * detail.b, nonEmissive);
                vec3 base = mix(uColor.rgb, uTint.rgb, uTint.a);
                float liftMask = uParams.z * (1.0 - clamp(uParams.w, 0.0, 1.0));
                base = max(base, vec3(0.16, 0.20, 0.27) * liftMask) * texMod;
                float hemi = n.z * 0.5 + 0.5;
                vec3 amb = mix(vec3(0.28, 0.33, 0.42), vec3(0.64, 0.72, 0.84), hemi);
                vec3 lKey = normalize(vec3(0.42, -0.38, 0.82));
                vec3 lFill = normalize(vec3(-0.56, 0.54, 0.34));
                float diffKey = max(dot(n, lKey), 0.0);
                float diffFill = max(dot(n, lFill), 0.0);
                float nv = max(dot(n, v), 0.0);
                float camFill = nv * 0.16;
                vec3 hVec = normalize(lKey + v);
                float sd = max(dot(n, hVec), 0.0);
                float s2 = sd * sd;
                float s4 = s2 * s2;
                float s8 = s4 * s4;
                float s16 = s8 * s8;
                float s32 = s16 * s16;
                float specBroad = (s8 * sd) * (0.14 + 0.16 * uParams.x) * specTex;
                float specSharp = (s32 * s8 * s2) * uParams.x * 1.15 * specTex;
                float fn = clamp(1.0 - nv, 0.0, 1.0);
                float fres = fn * fn * sqrt(max(fn, 0.0));
                vec3 rimCol = mix(vec3(0.62, 0.86, 1.0), uTint.rgb, uTint.a * 0.65);
                vec3 rim = rimCol * (fres * (0.24 + 0.22 * uParams.z));
                vec3 lit = base * (amb + vec3(0.76, 0.74, 0.70) * diffKey + vec3(0.28, 0.35, 0.44) * diffFill + camFill + uParams.w) +
                    vec3(0.94, 0.97, 1.0) * (specBroad + specSharp) + rim;
                gl_FragColor = vec4(min(lit, vec3(1.0)), uColor.a);
            }
        """
    }
}
