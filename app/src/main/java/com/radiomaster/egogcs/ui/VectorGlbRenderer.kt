package com.radiomaster.egogcs.ui

import android.content.Context
import android.opengl.GLES20
import android.opengl.Matrix
import android.util.Log
import java.io.FileInputStream
import java.nio.channels.FileChannel

/** Draws the original GLB in the existing GLES2 context; its demo animation is never applied. */
internal class VectorGlbRenderer(context: Context) {
    private val assets = context.applicationContext.assets

    @Volatile private var model: VectorGlbAsset.Model? = null
    private var program = 0
    private var vertexBuffer = 0
    private var aPosition = -1
    private var aNormal = -1
    private var uMvp = -1
    private var uModel = -1
    private var uColor = -1
    private val body = FloatArray(16)
    private val front = FloatArray(16)
    private val rear = FloatArray(16)
    private val partMatrix = FloatArray(16)
    private val mvp = FloatArray(16)

    val ready: Boolean get() = model != null && program != 0 && vertexBuffer != 0

    fun create() {
        // A recreated EGL context owns new GL names; never reuse the old context's IDs.
        model = null
        program = 0
        vertexBuffer = 0
        try {
            assets.openFd("vector_uav.glb").use { descriptor ->
                FileInputStream(descriptor.fileDescriptor).channel.use { channel ->
                    val mapped = channel.map(FileChannel.MapMode.READ_ONLY, descriptor.startOffset, descriptor.length)
                    val parsed = VectorGlbAsset.parse(mapped)
                    program = createProgram()
                    aPosition = GLES20.glGetAttribLocation(program, "aPosition")
                    aNormal = GLES20.glGetAttribLocation(program, "aNormal")
                    uMvp = GLES20.glGetUniformLocation(program, "uMvp")
                    uModel = GLES20.glGetUniformLocation(program, "uModel")
                    uColor = GLES20.glGetUniformLocation(program, "uColor")
                    check(aPosition >= 0 && aNormal >= 0 && uMvp >= 0 && uModel >= 0 && uColor >= 0)

                    val ids = IntArray(1)
                    GLES20.glGenBuffers(1, ids, 0)
                    vertexBuffer = ids[0]
                    check(vertexBuffer != 0) { "Could not allocate UAV GL buffer" }
                    val binary = mapped.duplicate()
                    binary.position(parsed.binaryOffset)
                    binary.limit(parsed.binaryOffset + parsed.binaryLength)
                    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vertexBuffer)
                    GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, parsed.binaryLength, binary.slice(), GLES20.GL_STATIC_DRAW)
                    val error = GLES20.glGetError()
                    check(error == GLES20.GL_NO_ERROR) { "UAV GL buffer upload failed: $error" }
                    GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
                    model = parsed
                }
            }
        } catch (e: Exception) {
            Log.e("VectorGlbRenderer", "Could not load vector_uav.glb; using the previous rig", e)
            if (vertexBuffer != 0) GLES20.glDeleteBuffers(1, intArrayOf(vertexBuffer), 0)
            if (program != 0) GLES20.glDeleteProgram(program)
            vertexBuffer = 0
            program = 0
            model = null
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
        val loaded = model ?: return
        if (!ready || !x.isFinite() || !y.isFinite() || !z.isFinite() ||
            !roll.isFinite() || !pitch.isFinite() || !yaw.isFinite() ||
            !frontTilt.isFinite() || !rearTilt.isFinite()
        ) {
            return
        }

        // The original model is in meters and Z-up. This display scale matches the existing map icon.
        Matrix.setIdentityM(body, 0)
        Matrix.translateM(body, 0, x, y, z)
        Matrix.rotateM(body, 0, yaw * RAD_TO_DEG, 0f, 0f, 1f)
        Matrix.rotateM(body, 0, -pitch * RAD_TO_DEG, 0f, 1f, 0f)
        Matrix.rotateM(body, 0, roll * RAD_TO_DEG, 1f, 0f, 0f)
        val scale = displayScale / 0.18817742f
        Matrix.scaleM(body, 0, scale, scale, scale)
        Matrix.translateM(body, 0, 0f, 0f, -0.05f)
        rotateGroup(front, body, loaded.frontPivot, frontTilt)
        rotateGroup(rear, body, loaded.rearPivot, rearTilt)

        GLES20.glUseProgram(program)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vertexBuffer)
        GLES20.glEnableVertexAttribArray(aPosition)
        GLES20.glEnableVertexAttribArray(aNormal)
        for (part in loaded.parts) {
            val group = when (part.group) {
                1 -> front
                2 -> rear
                else -> body
            }
            Matrix.multiplyMM(partMatrix, 0, group, 0, part.transform, 0)
            Matrix.multiplyMM(mvp, 0, viewProjection, 0, partMatrix, 0)
            GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
            GLES20.glUniformMatrix4fv(uModel, 1, false, partMatrix, 0)
            if (occluded) {
                GLES20.glUniform4f(uColor, 0.18f, 0.85f, 1f, 0.35f)
            } else {
                GLES20.glUniform4fv(uColor, 1, part.color, 0)
            }
            GLES20.glVertexAttribPointer(aPosition, 3, GLES20.GL_FLOAT, false, 0, part.positionOffset)
            GLES20.glVertexAttribPointer(aNormal, 3, GLES20.GL_FLOAT, false, 0, part.normalOffset)
            GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, part.vertexCount)
        }
        GLES20.glDisableVertexAttribArray(aPosition)
        GLES20.glDisableVertexAttribArray(aNormal)
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
        private const val VERTEX_SHADER = """
            attribute vec3 aPosition;
            attribute vec3 aNormal;
            uniform mat4 uMvp;
            uniform mat4 uModel;
            varying float vLight;
            void main() {
                vec3 normal = normalize((uModel * vec4(aNormal, 0.0)).xyz);
                vLight = 0.55 + 0.45 * abs(dot(normal, normalize(vec3(0.4, -0.3, 0.85))));
                gl_Position = uMvp * vec4(aPosition, 1.0);
            }
        """
        private const val FRAGMENT_SHADER = """
            precision mediump float;
            uniform vec4 uColor;
            varying float vLight;
            void main() {
                gl_FragColor = vec4(min(vec3(1.0), uColor.rgb * vLight + vec3(0.06)), uColor.a);
            }
        """
    }
}
