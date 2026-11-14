package com.radiomaster.egogcs.ui

import android.content.Context
import android.opengl.GLES20
import android.opengl.Matrix
import android.util.Log
import java.io.FileInputStream
import java.io.FileNotFoundException
import java.nio.channels.FileChannel

/**
 * Renders the articulated `Friendzhb/tilt-uav-indi` 4M2S vector UAV in GLES 2.0:
 * - Uses decimated `vector_uav.glb` when bundled, or seamlessly synthesizes a solid 3D URDF-scale CAD rig.
 * - Provides 3-point hemisphere + Blinn-Phong metallic shading, grazing Fresnel rim light, X-Ray hologram
 *   mode, real-time M1..M4 propeller rotation, swept rotor discs, and 3D tilted thrust vector arrows.
 */
internal class VectorGlbRenderer(context: Context) {
    private val assets = context.applicationContext.assets

    @Volatile private var model: VectorUavMeshBuilder.PreparedModel? = null
    private var program = 0
    private var vertexBuffer = 0
    private var aPosition = -1
    private var aNormal = -1
    private var uMvp = -1
    private var uModel = -1
    private var uColor = -1
    private var uEyePos = -1
    private var uTint = -1
    private var uParams = -1

    private val body = FloatArray(16)
    private val front = FloatArray(16)
    private val rear = FloatArray(16)
    private val spinTemp = FloatArray(16)
    private val auxLocal = FloatArray(16)
    private val partMatrix = FloatArray(16)
    private val mvp = FloatArray(16)

    val ready: Boolean get() = model != null && program != 0 && vertexBuffer != 0

    fun create() {
        model = null
        program = 0
        vertexBuffer = 0
        try {
            val prepared = loadPreparedModel()
            program = createProgram()
            aPosition = GLES20.glGetAttribLocation(program, "aPosition")
            aNormal = GLES20.glGetAttribLocation(program, "aNormal")
            uMvp = GLES20.glGetUniformLocation(program, "uMvp")
            uModel = GLES20.glGetUniformLocation(program, "uModel")
            uColor = GLES20.glGetUniformLocation(program, "uColor")
            uEyePos = GLES20.glGetUniformLocation(program, "uEyePos")
            uTint = GLES20.glGetUniformLocation(program, "uTint")
            uParams = GLES20.glGetUniformLocation(program, "uParams")
            check(
                aPosition >= 0 && aNormal >= 0 && uMvp >= 0 && uModel >= 0 &&
                    uColor >= 0 && uEyePos >= 0 && uTint >= 0 && uParams >= 0
            )

            val ids = IntArray(1)
            GLES20.glGenBuffers(1, ids, 0)
            vertexBuffer = ids[0]
            check(vertexBuffer != 0) { "Could not allocate UAV GL buffer" }
            val byteCount = prepared.totalVertices * VectorUavMeshBuilder.STRIDE_BYTES
            val uploadBuffer = prepared.vertexBuffer.duplicate().apply { position(0) }
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vertexBuffer)
            GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, byteCount, uploadBuffer, GLES20.GL_STATIC_DRAW)
            val error = GLES20.glGetError()
            check(error == GLES20.GL_NO_ERROR) { "UAV GL buffer upload failed: $error" }
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
            model = prepared
        } catch (e: Exception) {
            Log.e("VectorGlbRenderer", "Could not initialize 3D vector UAV renderer", e)
            if (vertexBuffer != 0) GLES20.glDeleteBuffers(1, intArrayOf(vertexBuffer), 0)
            if (program != 0) GLES20.glDeleteProgram(program)
            vertexBuffer = 0
            program = 0
            model = null
        }
    }

    private fun loadPreparedModel(): VectorUavMeshBuilder.PreparedModel {
        cachedPreparedModel?.let { return it }
        synchronized(MODEL_CACHE_LOCK) {
            cachedPreparedModel?.let { return it }
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
                Log.w("VectorGlbRenderer", "Falling back to built-in 3D CAD vector UAV rig", e)
                VectorUavMeshBuilder.buildProceduralTiltUav()
            }
            cachedPreparedModel = built
            return built
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
            m0 = 0.35f,
            m1 = 0.35f,
            m2 = 0.35f,
            m3 = 0.35f,
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
        tintStrength: Float = 0f
    ) {
        val loaded = model ?: return
        if (!ready || !x.isFinite() || !y.isFinite() || !z.isFinite() ||
            !roll.isFinite() || !pitch.isFinite() || !yaw.isFinite() ||
            !frontTilt.isFinite() || !rearTilt.isFinite()
        ) {
            return
        }

        Matrix.setIdentityM(body, 0)
        Matrix.translateM(body, 0, x, y, z)
        Matrix.rotateM(body, 0, yaw * RAD_TO_DEG, 0f, 0f, 1f)
        Matrix.rotateM(body, 0, -pitch * RAD_TO_DEG, 0f, 1f, 0f)
        Matrix.rotateM(body, 0, roll * RAD_TO_DEG, 1f, 0f, 0f)
        val scale = displayScale / VectorUavMeshBuilder.ARM_SPAN_RADIUS
        Matrix.scaleM(body, 0, scale, scale, scale)
        Matrix.translateM(body, 0, 0f, 0f, -0.05f)
        rotateGroup(front, body, loaded.frontPivot, frontTilt)
        rotateGroup(rear, body, loaded.rearPivot, rearTilt)

        GLES20.glUseProgram(program)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vertexBuffer)
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

        val parts = loaded.parts
        var pIdx = 0
        while (pIdx < parts.size) {
            val part = parts[pIdx]
            pIdx++
            val group = when (part.group) {
                1 -> front
                2 -> rear
                else -> body
            }
            val rIdx = part.rotorIndex
            if (rIdx in 1..4 && (armed || spinPhase != 0f)) {
                val hub = loaded.rotorHubs[rIdx - 1]
                val deg = (spinPhase * hub.spinSign * RAD_TO_DEG) % 360f
                System.arraycopy(group, 0, spinTemp, 0, 16)
                Matrix.translateM(spinTemp, 0, hub.x, hub.y, 0f)
                Matrix.rotateM(spinTemp, 0, deg, 0f, 0f, 1f)
                Matrix.translateM(spinTemp, 0, -hub.x, -hub.y, 0f)
                Matrix.multiplyMM(partMatrix, 0, spinTemp, 0, part.transform, 0)
            } else {
                Matrix.multiplyMM(partMatrix, 0, group, 0, part.transform, 0)
            }
            Matrix.multiplyMM(mvp, 0, viewProjection, 0, partMatrix, 0)
            GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
            GLES20.glUniformMatrix4fv(uModel, 1, false, partMatrix, 0)
            if (occluded) {
                GLES20.glUniform3f(uParams, 0f, 1f, darkCarbonLift)
                GLES20.glUniform4f(uColor, 0.16f, 0.88f, 1.0f, 0.38f)
            } else {
                val partTintBoost = if (part.group == 0 && tintStrength > 0.01f) tintStrength else tintStrength * 0.35f
                GLES20.glUniform4f(uTint, tintR, tintG, tintB, partTintBoost)
                GLES20.glUniform3f(uParams, part.specular, 0f, darkCarbonLift)
                GLES20.glUniform4fv(uColor, 1, part.color, 0)
            }
            GLES20.glDrawArrays(GLES20.GL_TRIANGLES, part.firstVertex, part.vertexCount)
        }

        if (!occluded && (armed || showThrustVectors || showHeadingCue)) {
            GLES20.glEnable(GLES20.GL_BLEND)
            GLES20.glBlendFuncSeparate(
                GLES20.GL_SRC_ALPHA,
                GLES20.GL_ONE_MINUS_SRC_ALPHA,
                GLES20.GL_ONE,
                GLES20.GL_ONE_MINUS_SRC_ALPHA
            )
            GLES20.glDepthMask(false)
            GLES20.glUniform3f(uParams, 0f, 2f, darkCarbonLift)
            GLES20.glUniform4f(uTint, 0f, 0f, 0f, 0f)

            if (armed || showThrustVectors) {
                drawRotorOverlaysAndThrustVectors(
                    loaded = loaded,
                    viewProjection = viewProjection,
                    spinPhase = spinPhase,
                    armed = armed,
                    m0 = m0,
                    m1 = m1,
                    m2 = m2,
                    m3 = m3,
                    showThrustVectors = showThrustVectors,
                    isLightTheme = isLightTheme
                )
            }

            if (showHeadingCue) {
                Matrix.setIdentityM(auxLocal, 0)
                Matrix.translateM(auxLocal, 0, 0.114f, 0f, 0.054f)
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
        } else if (occluded && showThrustVectors) {
            GLES20.glUniform3f(uParams, 0f, 2f, darkCarbonLift)
            drawOccludedThrustVectors(loaded, viewProjection, m0, m1, m2, m3)
        }

        GLES20.glDisableVertexAttribArray(aPosition)
        GLES20.glDisableVertexAttribArray(aNormal)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
    }

    private fun drawRotorOverlaysAndThrustVectors(
        loaded: VectorUavMeshBuilder.PreparedModel,
        viewProjection: FloatArray,
        spinPhase: Float,
        armed: Boolean,
        m0: Float,
        m1: Float,
        m2: Float,
        m3: Float,
        showThrustVectors: Boolean,
        isLightTheme: Boolean
    ) {
        val hubs = loaded.rotorHubs
        var i = 0
        while (i < 4) {
            val hub = hubs[i]
            val group = if (hub.group == 1) front else rear
            val throttle = when (i) {
                0 -> m0
                1 -> m1
                2 -> m2
                else -> m3
            }.coerceIn(0f, 1f)
            i++

            val navR = if (hub.isFront) 1.00f else 0.06f
            val navG = if (hub.isFront) 0.30f else 0.90f
            val navB = if (hub.isFront) 0.24f else 0.50f

            Matrix.setIdentityM(auxLocal, 0)
            Matrix.translateM(auxLocal, 0, hub.x, hub.y, hub.z + 0.0014f)
            Matrix.rotateM(auxLocal, 0, (spinPhase * hub.spinSign * RAD_TO_DEG) % 360f, 0f, 0f, 1f)
            Matrix.scaleM(auxLocal, 0, hub.radius, hub.radius, 1f)
            Matrix.multiplyMM(partMatrix, 0, group, 0, auxLocal, 0)
            Matrix.multiplyMM(mvp, 0, viewProjection, 0, partMatrix, 0)
            GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
            GLES20.glUniformMatrix4fv(uModel, 1, false, partMatrix, 0)

            val discAlpha = if (armed) (if (isLightTheme) 0.19f else 0.14f) else 0.06f
            GLES20.glUniform4f(uColor, navR, navG, navB, discAlpha)
            GLES20.glDrawArrays(GLES20.GL_TRIANGLES, loaded.rotorDisc.firstVertex, loaded.rotorDisc.vertexCount)

            if (armed) {
                val sweepCol = if (isLightTheme) 0.22f else 0.92f
                val sweepAlpha = if (isLightTheme) 0.22f else 0.20f
                GLES20.glUniform4f(uColor, sweepCol, sweepCol + 0.04f, minOf(1f, sweepCol + 0.08f), sweepAlpha)
                GLES20.glDrawArrays(GLES20.GL_TRIANGLES, loaded.rotorSweep.firstVertex, loaded.rotorSweep.vertexCount)
            }

            val ringAlpha = if (armed) 0.62f else 0.36f
            GLES20.glUniform4f(uColor, navR, navG, navB, ringAlpha)
            GLES20.glDrawArrays(GLES20.GL_TRIANGLES, loaded.rotorRing.firstVertex, loaded.rotorRing.vertexCount)

            if (showThrustVectors) {
                val shaftLen = 0.038f + throttle * 0.056f
                val vr = if (hub.group == 1) 1.00f else 0.14f
                val vg = if (hub.group == 1) 0.78f else 0.80f
                val vb = if (hub.group == 1) 0.06f else 1.00f
                val va = if (armed) 0.88f else 0.48f

                Matrix.setIdentityM(auxLocal, 0)
                Matrix.translateM(auxLocal, 0, hub.x, hub.y, hub.z + 0.003f)
                Matrix.scaleM(auxLocal, 0, 0.0029f, 0.0029f, shaftLen)
                Matrix.multiplyMM(partMatrix, 0, group, 0, auxLocal, 0)
                Matrix.multiplyMM(mvp, 0, viewProjection, 0, partMatrix, 0)
                GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
                GLES20.glUniformMatrix4fv(uModel, 1, false, partMatrix, 0)
                GLES20.glUniform4f(uColor, vr, vg, vb, va)
                GLES20.glDrawArrays(GLES20.GL_TRIANGLES, loaded.thrustShaft.firstVertex, loaded.thrustShaft.vertexCount)

                Matrix.setIdentityM(auxLocal, 0)
                Matrix.translateM(auxLocal, 0, hub.x, hub.y, hub.z + 0.003f + shaftLen)
                Matrix.scaleM(auxLocal, 0, 0.0072f, 0.0072f, 0.019f)
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
            val group = if (hub.group == 1) front else rear
            val throttle = when (i) {
                0 -> m0
                1 -> m1
                2 -> m2
                else -> m3
            }.coerceIn(0f, 1f)
            i++
            val shaftLen = 0.038f + throttle * 0.056f
            val vr = if (hub.group == 1) 1.00f else 0.18f
            val vg = if (hub.group == 1) 0.80f else 0.86f
            val vb = if (hub.group == 1) 0.12f else 1.00f

            Matrix.setIdentityM(auxLocal, 0)
            Matrix.translateM(auxLocal, 0, hub.x, hub.y, hub.z + 0.003f)
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
        private const val RAD_TO_DEG = 57.2957795f
        private val MODEL_CACHE_LOCK = Any()

        @Volatile private var cachedPreparedModel: VectorUavMeshBuilder.PreparedModel? = null

        private const val VERTEX_SHADER = """
            attribute vec3 aPosition;
            attribute vec3 aNormal;
            uniform mat4 uMvp;
            uniform mat4 uModel;
            varying vec3 vWorldPos;
            varying vec3 vWorldNormal;
            void main() {
                vec4 wp = uModel * vec4(aPosition, 1.0);
                vWorldPos = wp.xyz;
                vWorldNormal = (uModel * vec4(aNormal, 0.0)).xyz;
                gl_Position = uMvp * vec4(aPosition, 1.0);
            }
        """
        private const val FRAGMENT_SHADER = """
            precision mediump float;
            uniform vec4 uColor;
            uniform vec3 uEyePos;
            uniform vec4 uTint;
            uniform vec3 uParams;
            varying vec3 vWorldPos;
            varying vec3 vWorldNormal;
            void main() {
                vec3 n = normalize(vWorldNormal);
                vec3 v = normalize(uEyePos - vWorldPos);
                if (uParams.y > 1.5) {
                    float glow = 0.86 + 0.14 * abs(dot(n, v));
                    gl_FragColor = vec4(min(uColor.rgb * glow, vec3(1.0)), uColor.a);
                    return;
                }
                if (uParams.y > 0.5) {
                    float fres = pow(clamp(1.0 - abs(dot(n, v)), 0.0001, 1.0), 1.75);
                    vec3 holo = mix(vec3(0.15, 0.88, 1.0), uTint.rgb, uTint.a * 0.65);
                    gl_FragColor = vec4(min(holo * (0.72 + 0.52 * fres), vec3(1.0)), clamp(0.24 + 0.46 * fres, 0.22, 0.72));
                    return;
                }
                if (dot(n, v) < 0.0) {
                    n = -n;
                }
                vec3 base = mix(uColor.rgb, uTint.rgb, uTint.a);
                base = max(base, vec3(0.15, 0.18, 0.23) * uParams.z);
                float hemi = n.z * 0.5 + 0.5;
                vec3 amb = mix(vec3(0.26, 0.30, 0.36), vec3(0.58, 0.64, 0.74), hemi);
                vec3 lKey = normalize(vec3(0.42, -0.36, 0.83));
                vec3 lFill = normalize(vec3(-0.58, 0.52, 0.28));
                float diffKey = max(dot(n, lKey), 0.0);
                float diffFill = max(dot(n, lFill), 0.0);
                vec3 hVec = normalize(lKey + v);
                float specDot = clamp(dot(n, hVec), 0.0001, 1.0);
                float spec = (dot(n, hVec) > 0.0 ? pow(specDot, 26.0) : 0.0) * uParams.x;
                float fres = pow(clamp(1.0 - max(dot(n, v), 0.0), 0.0001, 1.0), 2.8);
                vec3 rim = mix(vec3(0.52, 0.80, 1.0), uTint.rgb, uTint.a * 0.65) * (fres * (0.20 + 0.16 * uParams.z));
                vec3 lit = base * (amb + vec3(0.70, 0.68, 0.65) * diffKey + vec3(0.24, 0.30, 0.38) * diffFill) +
                    vec3(0.88, 0.94, 1.0) * spec + rim;
                gl_FragColor = vec4(min(lit, vec3(1.0)), uColor.a);
            }
        """
    }
}
