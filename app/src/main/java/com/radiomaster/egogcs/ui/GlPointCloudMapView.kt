package com.radiomaster.egogcs.ui

import android.animation.ValueAnimator
import android.annotation.SuppressLint
import android.content.Context
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.view.Choreographer
import android.view.GestureDetector
import android.view.HapticFeedbackConstants
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.animation.OvershootInterpolator
import com.radiomaster.egogcs.engine.FastLioVoxelMap
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.RenderSnapshot
import com.radiomaster.egogcs.model.SwarmAgentFrame
import com.radiomaster.egogcs.model.SwarmPalette
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.tan

/**
 * 商业三维引擎级「外表面深度预通单层透镜 (Single-Skin Depth-Prepass Cutaway Lens) + 双色对比流动虚线 X-Ray」渲染管线：
 *
 * 针对传统半透明体素「前后混合次序颠倒、相邻体素内表面重影乱纹、近机端截断失效、缺乏三维接触阴影」四大痛点彻底重构：
 * 1. 严格物理前后合成次序 (True Physical Compositing Order)：
 *    - Pass 0/1：3D 天际大气穹顶 + 透视地面网格 + 障碍物/机体接地环境光遮蔽投影 (Ground Contact Drop Shadows)；
 *    - Pass 2：100% 纯不透明远景体素与点云 (`GL_BLEND` 关闭，彻底杜绝地面网格穿透实体墙的伪影)，带高程垂直 AO 与柱顶微倒角；
 *    - Pass 3：前景三维无人机与 B 样条轨迹实体渲染 (`GL_LEQUAL`)；
 *    - Pass 4：视线遮挡体素「外表面单层光学透镜」(先以 `glColorMask(false)` 写入最近外表面硬件深度，再以 `GL_EQUAL` 仅对最外层单张表皮执行逐像素平滑椭球透镜消隐，100% 硬件剔除体素簇内部所有共享内壁，零排序破洞、零多层混浊叠加)；
 *    - Pass 5：硬件 `GL_GREATER` 逆深度 X-Ray 叠加遍，对处于障碍物后方的轨迹绘制「连续微光导轨 + 高对比抗锯齿流动虚线」，对被挡无人机绘制稳态全息轮廓；
 *    - Pass 6：半透明航线高程垂幕与触控水波纹。
 */
class GlPointCloudMapView(
    context: Context,
    private val hub: GcsSharedHub,
    private val voxelMap: FastLioVoxelMap,
    private val onHudTick: (RenderSnapshot) -> Unit,
    private val onInterceptDrawerClose: () -> Boolean,
    private val onSingleTapPreviewPin: (wx: Float, wy: Float) -> Unit,
    private val onLongPressQuickFly: (wx: Float, wy: Float) -> Unit,
    private val onFollowStateChanged: (followMode: Int, toastMsg: String?) -> Unit,
    private val animationTimeNanos: () -> Long = System::nanoTime
) : GLSurfaceView(context), GLSurfaceView.Renderer {

    companion object {
        private const val MAX_ENV_LINE_VERTS = 36_000
        private const val MAX_ENV_TRI_VERTS = 96_000
        private const val MAX_GLASS_TRI_VERTS = 72_000
        private const val MAX_GLASS_LINE_VERTS = 24_000
        private const val MAX_FX_TRI_VERTS = 32_000
        private const val MAX_ACTOR_LINE_VERTS = 18_000
        private const val MAX_ACTOR_TRI_VERTS = 36_000

        // 8 floats per vertex (32 bytes): x, y, z, r, g, b, a, arcDist
        private const val FLOATS_PER_VERT = 8
        private const val STRIDE_VERT = FLOATS_PER_VERT * 4

        private const val LUT_N = 256
        private val COS_LUT = FloatArray(LUT_N + 1) { i -> cos((i.toFloat() / LUT_N) * (PI.toFloat() * 2f)) }
        private val SIN_LUT = FloatArray(LUT_N + 1) { i -> sin((i.toFloat() / LUT_N) * (PI.toFloat() * 2f)) }

        // 1. 天际大气穹顶着色器
        private const val SKY_VS = """
            attribute vec2 aNdcPos;
            varying vec2 vUv;
            void main() {
                vUv = aNdcPos * 0.5 + 0.5;
                gl_Position = vec4(aNdcPos, 0.999, 1.0);
            }
        """

        private const val SKY_FS = """
            precision mediump float;
            uniform float uHorizonY;
            uniform float uBlend3D;
            uniform vec3 uSkyFlat;
            uniform vec3 uSkyZenith;
            uniform vec3 uSkyHorizon;
            uniform vec3 uSkyGround;
            varying vec2 vUv;
            void main() {
                float above = smoothstep(uHorizonY - 0.09, 1.0, vUv.y);
                float band = exp(-abs(vUv.y - uHorizonY) * 9.5);
                vec3 sky3d = mix(uSkyGround, uSkyZenith, above) + uSkyHorizon * band * 0.56;
                gl_FragColor = vec4(mix(uSkyFlat, sky3d, uBlend3D), 1.0);
            }
        """

        // 2. 点云浮雕着色器 + 4x4 Bayer 有序抖动视线透镜消隐 (16 级无噪平滑过渡)
        private const val POINT_CLOUD_VS = """
            uniform mat4 uMVP;
            uniform float uZMax;
            uniform float uPointScale;
            uniform float uZFlatten;
            uniform int uColorMode;
            uniform vec3 uEyePos;
            uniform vec3 uDronePos;
            uniform vec3 uAheadPos;
            uniform float uInnerR;
            uniform float uOuterR;
            uniform float uFogSpan;
            uniform int uXrayOn;
            attribute vec3 aPos;
            attribute float aMeta;
            varying vec4 vColor;
            varying float vLosFade;
            varying float vFog;

            vec3 heightRamp(float t) {
                float c = clamp(t, 0.0, 1.0);
                if (c < 0.33) {
                    float u = c / 0.33;
                    return mix(vec3(0.08, 0.56, 1.00), vec3(0.00, 0.90, 0.52), u);
                } else if (c < 0.68) {
                    float u = (c - 0.33) / 0.35;
                    return mix(vec3(0.00, 0.90, 0.52), vec3(1.00, 0.78, 0.08), u);
                } else {
                    float u = (c - 0.68) / 0.32;
                    return mix(vec3(1.00, 0.78, 0.08), vec3(1.00, 0.24, 0.20), u);
                }
            }

            float capsuleRayDist(vec3 p, vec3 eye, vec3 target) {
                vec3 seg = target - eye;
                float len2 = dot(seg, seg);
                if (len2 < 0.15) return 99.0;
                float rawT = dot(p - eye, seg) / len2;
                if (rawT > 1.02 || rawT < 0.02) return 99.0;
                float tc = clamp(rawT, 0.03, 1.0);
                return length((eye + seg * tc) - p);
            }

            void main() {
                if (aPos.z > uZMax) {
                    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
                    gl_PointSize = 0.0;
                    vColor = vec4(0.0);
                    vLosFade = 1.0;
                    vFog = 0.0;
                    return;
                }
                vec3 worldPos = vec3(aPos.x, aPos.y, aPos.z * uZFlatten);
                vec4 clip = uMVP * vec4(worldPos, 1.0);
                gl_Position = clip;
                float dist = max(1.8, clip.w);
                gl_PointSize = clamp(uPointScale / dist, 2.4, 9.0);

                vec3 rgb;
                if (uColorMode == 1) {
                    float it = clamp(aMeta, 0.0, 1.0);
                    rgb = mix(vec3(0.12, 0.68, 1.00), vec3(1.00, 0.48, 0.10), it);
                } else if (uColorMode == 2) {
                    float hz = clamp((aPos.z + 0.2) / 3.0, 0.15, 1.0);
                    rgb = vec3(0.05, 0.88, 0.48) * (0.45 + 0.55 * hz);
                } else {
                    float hz = clamp((aPos.z + 0.15) / max(0.8, uZMax), 0.0, 1.0);
                    rgb = heightRamp(hz);
                }

                vLosFade = 1.0;
                if (uXrayOn == 1 && worldPos.z > 0.12) {
                    float d1 = capsuleRayDist(worldPos, uEyePos, uDronePos);
                    float d2 = capsuleRayDist(worldPos, uEyePos, uAheadPos) * 1.25;
                    float dMin = min(d1, d2);
                    vLosFade = smoothstep(uInnerR * 0.65, uOuterR, dMin);
                }

                vFog = clamp((length(worldPos.xy - uDronePos.xy) - uFogSpan * 0.85) / max(6.0, uFogSpan * 1.4), 0.0, 0.65);
                float alpha = (aPos.z < 0.06) ? 0.55 : 0.96;
                vColor = vec4(rgb, alpha);
            }
        """

        private const val POINT_CLOUD_FS = """
            precision mediump float;
            uniform vec3 uFogColor;
            varying vec4 vColor;
            varying float vLosFade;
            varying float vFog;
            void main() {
                vec2 uv = gl_PointCoord - vec2(0.5);
                float r2 = dot(uv, uv);
                if (r2 > 0.25) discard;

                if (vLosFade < 0.94) {
                    vec2 p = floor(mod(gl_FragCoord.xy, 4.0));
                    vec2 p0 = mod(p, 2.0);
                    float bLow = mod(p0.x * 2.0 + p0.y * 3.0, 4.0);
                    vec2 q = floor(p * 0.5);
                    float bHigh = mod(q.x * 2.0 + q.y * 3.0, 4.0);
                    float bayer4 = (bLow * 4.0 + bHigh + 0.5) / 16.0;
                    if (vLosFade < bayer4 * 0.90 + 0.08) discard;
                }

                float nz = sqrt(max(0.0, 1.0 - r2 * 4.0));
                vec3 norm = vec3(uv.x * 2.0, -uv.y * 2.0, nz);
                float diff = max(0.0, dot(norm, vec3(0.38, 0.48, 0.79)));
                float edlRim = 1.0 - smoothstep(0.14, 0.25, r2) * 0.46;
                vec3 litRgb = vColor.rgb * (0.56 + 0.58 * diff) * edlRim;
                vec3 fogRgb = mix(litRgb, uFogColor, vFog);
                gl_FragColor = vec4(fogRgb, vColor.a);
            }
        """

        // 3. 标准三维实体/网格着色器
        private const val PRIM_VS = """
            uniform mat4 uMVP;
            uniform vec2 uFocusXY;
            uniform float uFogSpan;
            attribute vec3 aPos;
            attribute vec4 aColor;
            attribute float aArc;
            varying vec4 vColor;
            varying float vFog;
            void main() {
                gl_Position = uMVP * vec4(aPos, 1.0);
                vColor = aColor;
                float d = length(aPos.xy - uFocusXY);
                vFog = clamp((d - uFogSpan * 0.95) / max(8.0, uFogSpan * 1.5), 0.0, 0.62);
            }
        """

        private const val PRIM_FS = """
            precision mediump float;
            uniform vec3 uFogColor;
            varying vec4 vColor;
            varying float vFog;
            void main() {
                vec3 rgb = mix(vColor.rgb, uFogColor, vFog);
                gl_FragColor = vec4(rgb, vColor.a);
            }
        """

        // 4. 逐像素平滑椭球透镜消隐着色器 (配合 Depth Pre-Pass + GL_EQUAL 单外皮渲染，无内表面乱纹、无方块跳变)
        private const val CUTAWAY_VS = """
            uniform mat4 uMVP;
            uniform vec3 uEyePos;
            uniform vec2 uFocusXY;
            uniform float uFogSpan;
            attribute vec3 aPos;
            attribute vec4 aColor;
            attribute float aArc;
            varying vec3 vRelEye;
            varying vec3 vWorldPos;
            varying vec4 vColor;
            varying float vFog;
            void main() {
                gl_Position = uMVP * vec4(aPos, 1.0);
                vRelEye = aPos - uEyePos;
                vWorldPos = aPos;
                vColor = aColor;
                float d = length(aPos.xy - uFocusXY);
                vFog = clamp((d - uFogSpan * 0.95) / max(8.0, uFogSpan * 1.5), 0.0, 0.62);
            }
        """

        private const val CUTAWAY_FS = """
            #ifdef GL_FRAGMENT_PRECISION_HIGH
            precision highp float;
            #else
            precision mediump float;
            #endif
            uniform vec3 uSegDrone;
            uniform float uInvDroneLen2;
            uniform vec3 uSegAhead;
            uniform float uInvAheadLen2;
            uniform float uInnerR;
            uniform float uOuterR;
            uniform vec3 uFogColor;
            varying vec3 vRelEye;
            varying vec3 vWorldPos;
            varying vec4 vColor;
            varying float vFog;

            float capsuleDist(vec3 relP, vec3 seg, float invLen2) {
                if (invLen2 <= 0.0) return 99.0;
                float rawT = dot(relP, seg) * invLen2;
                if (rawT > 1.03 || rawT < 0.01) return 99.0;
                float tc = clamp(rawT, 0.02, 1.0);
                return length(relP - seg * tc);
            }

            void main() {
                float dDrone = capsuleDist(vRelEye, uSegDrone, uInvDroneLen2);
                float dAhead = capsuleDist(vRelEye, uSegAhead, uInvAheadLen2) * 1.28;
                float dNearEye = length(vRelEye);
                float lensFade = smoothstep(uInnerR, uOuterR, min(dDrone, dAhead));
                float nearLensFade = smoothstep(0.9, 2.3, dNearEye);
                float fade = min(lensFade, nearLensFade);

                // 透镜中心仅保留 16% 清透磨砂薄壳，向边缘丝滑过渡至 100% 原始实体体素色 (彻底消除方块阶梯跳变)
                float shellAlpha = mix(0.16, 1.0, fade);

                // 在透镜边缘勾勒柔和全息光学光晕环 (Aperture Rim)，提示飞手此处存在被透视剖开的真实障碍物厚度
                float dr = (fade - 0.42) * 4.2;
                float rimBand = exp(-dr * dr) * (1.0 - fade * 0.55);
                // 轻量高程等高微线 (仅在外表皮淡淡浮现，强化膨胀体高度感知而不遮挡中心飞机)
                float zContour = smoothstep(0.90, 0.99, abs(fract(vWorldPos.z * 4.0) * 2.0 - 1.0)) * (1.0 - fade) * 0.22;

                vec3 glassTint = mix(vec3(0.16, 0.82, 0.98), vColor.rgb, 0.45);
                vec3 baseRgb = mix(glassTint, vColor.rgb, fade);
                vec3 rimAdded = baseRgb + vec3(0.18, 0.88, 1.00) * (rimBand * 0.30 + zContour);
                vec3 finalRgb = mix(rimAdded, uFogColor, vFog);

                gl_FragColor = vec4(finalRgb, clamp(shellAlpha + rimBand * 0.14 + zContour, 0.14, 1.0));
            }
        """

        // 5. 商业航测级双色对比 X-Ray 遮挡着色器 (连续微光导轨 + 高对比抗锯齿流动虚线 + 稳态全息机身轮廓)
        private const val OCCLUDED_VS = """
            uniform mat4 uMVP;
            attribute vec3 aPos;
            attribute vec4 aColor;
            attribute float aArc;
            varying vec4 vColor;
            varying float vArc;
            void main() {
                gl_Position = uMVP * vec4(aPos, 1.0);
                vColor = aColor;
                vArc = aArc;
            }
        """

        private const val OCCLUDED_FS = """
            precision mediump float;
            uniform float uTime;
            varying vec4 vColor;
            varying float vArc;
            void main() {
                if (vArc >= 0.0) {
                    // 空间等距 0.36m 节距向前流动的抗锯齿胶囊虚线 + 16% 连续底轨保持拐弯拓扑不断裂
                    float phase = fract(vArc * 2.75 - uTime * 1.85);
                    float dashMask = smoothstep(0.02, 0.10, phase) * (1.0 - smoothstep(0.48, 0.56, phase));
                    vec3 brightDash = mix(vColor.rgb, vec3(0.55, 0.98, 1.00), 0.45);
                    vec3 railRgb = mix(vColor.rgb, vec3(0.15, 0.78, 0.95), 0.35);
                    vec3 outRgb = mix(railRgb, brightDash, dashMask);
                    float outAlpha = mix(0.16, 0.92, dashMask) * clamp(vColor.a, 0.35, 1.0);
                    gl_FragColor = vec4(outRgb, outAlpha);
                } else {
                    float pulse = 0.82 + 0.18 * sin(uTime * 4.2);
                    vec3 holoRgb = mix(vColor.rgb, vec3(0.22, 0.95, 1.00), 0.62) * pulse;
                    gl_FragColor = vec4(holoRgb, 0.72);
                }
            }
        """
    }

    private val density = resources.displayMetrics.density

    var is3DMode: Boolean = true
        private set

    @Volatile private var projBlend: Float = 1.0f
    private var blendAnimator: ValueAnimator? = null

    @Volatile private var pinAnimScale: Float = 1.0f
    private var pinAnimator: ValueAnimator? = null

    @Volatile var followMode: Int = 1
        private set

    @Volatile private var camYawDesired = 0.62f

    @Volatile private var camPitchDesired = 0.55f

    @Volatile private var worldHalfSpanDesired = 6.4f

    private var camYaw = 0.62f
    private var camPitch = 0.55f
    private var worldHalfSpan = 6.4f

    @Volatile private var camPanX = 0f

    @Volatile private var camPanY = 0f
    private var camFocusX = -5.5f
    private var camFocusY = -4.2f
    private var camFocusZ = 0.45f

    private var curEyeX = -8f
    private var curEyeY = -8f
    private var curEyeZ = 5f

    // 当前帧视线透镜参数 (供 CPU 粗筛与 GPU 逐像素着色器共享)
    private var curAheadX = -5.5f
    private var curAheadY = -4.2f
    private var curAheadZ = 1.2f
    private var curSegDroneX = 0f
    private var curSegDroneY = 0f
    private var curSegDroneZ = 0f
    private var curInvDroneLen2 = 0f
    private var curSegAheadX = 0f
    private var curSegAheadY = 0f
    private var curSegAheadZ = 0f
    private var curInvAheadLen2 = 0f
    private var curCutInnerR = 0.85f
    private var curCutOuterR = 2.15f

    private val obsSmoothAlpha = FloatArray(24) { 1.0f }
    private val tmpVoxelRgb = FloatArray(3)
    private val uavTmpA = FloatArray(3)
    private val uavTmpB = FloatArray(3)
    private val vectorGlb = VectorGlbRenderer(context)
    internal val vectorGlbReady: Boolean get() = vectorGlb.ready
    private var droneOccludedBlend = 0f
    private var shaderClockSec = 0f

    private var viewportW = 1
    private var viewportH = 1
    private var lastFrameNs = System.nanoTime()
    private var fpsFrameCount = 0
    private var fpsTimeAcc = 0f

    private var multiTouchActive = false

    private val viewMatrix = FloatArray(16)
    private val projMatrix = FloatArray(16)
    private val vpMatrix = FloatArray(16)
    private val glTmpInv16 = FloatArray(16)
    private val invVpLock = Any()
    private val sharedInvVp = FloatArray(16)
    private var invVpValid = false

    private val uiInvCopy16 = FloatArray(16)
    private val uiNearIn4 = FloatArray(4)
    private val uiFarIn4 = FloatArray(4)
    private val uiNearOut4 = FloatArray(4)
    private val uiFarOut4 = FloatArray(4)
    private val uiGroundOut2 = FloatArray(2)

    private val rippleLock = Any()
    private val rippleX = FloatArray(6)
    private val rippleY = FloatArray(6)
    private val rippleAge = FloatArray(6) { -1f }

    private val skyQuadBuf: FloatBuffer = ByteBuffer.allocateDirect(8 * 4)
        .order(ByteOrder.nativeOrder())
        .asFloatBuffer()
        .apply {
            put(floatArrayOf(-1f, -1f, 1f, -1f, -1f, 1f, 1f, 1f))
            position(0)
        }

    private val groundLineArray = FloatArray(MAX_ENV_LINE_VERTS * FLOATS_PER_VERT)
    private var groundLineVerts = 0

    private val solidObsTriArray = FloatArray(MAX_ENV_TRI_VERTS * FLOATS_PER_VERT)
    private var solidObsTriVerts = 0
    private val solidObsEdgeArray = FloatArray(MAX_ENV_LINE_VERTS * FLOATS_PER_VERT)
    private var solidObsEdgeVerts = 0

    private val glassObsTriArray = FloatArray(MAX_GLASS_TRI_VERTS * FLOATS_PER_VERT)
    private var glassObsTriVerts = 0
    private val glassObsEdgeArray = FloatArray(MAX_GLASS_LINE_VERTS * FLOATS_PER_VERT)
    private var glassObsEdgeVerts = 0

    private val fxAlphaTriArray = FloatArray(MAX_FX_TRI_VERTS * FLOATS_PER_VERT)
    private var fxAlphaTriVerts = 0

    private val actorLineArray = FloatArray(MAX_ACTOR_LINE_VERTS * FLOATS_PER_VERT)
    private var actorLineVerts = 0
    private val actorTriArray = FloatArray(MAX_ACTOR_TRI_VERTS * FLOATS_PER_VERT)
    private var actorTriVerts = 0

    private val envLineBuf: FloatBuffer = ByteBuffer.allocateDirect(MAX_ENV_LINE_VERTS * STRIDE_VERT)
        .order(ByteOrder.nativeOrder()).asFloatBuffer()
    private val envTriBuf: FloatBuffer = ByteBuffer.allocateDirect(MAX_ENV_TRI_VERTS * STRIDE_VERT)
        .order(ByteOrder.nativeOrder()).asFloatBuffer()
    private val actorLineBuf: FloatBuffer = ByteBuffer.allocateDirect(MAX_ACTOR_LINE_VERTS * STRIDE_VERT)
        .order(ByteOrder.nativeOrder()).asFloatBuffer()
    private val actorTriBuf: FloatBuffer = ByteBuffer.allocateDirect(MAX_ACTOR_TRI_VERTS * STRIDE_VERT)
        .order(ByteOrder.nativeOrder()).asFloatBuffer()

    // Shader Programs & Uniform/Attrib Handles
    private var progSky = 0
    private var aSkyPos = 0
    private var uSkyHorizonY = 0
    private var uSkyBlend3D = 0
    private var uSkyFlat = 0
    private var uSkyZenith = 0
    private var uSkyHorizon = 0
    private var uSkyGround = 0

    private val curSkyFlat = floatArrayOf(0.043f, 0.051f, 0.066f)
    private val curSkyZenith = floatArrayOf(0.024f, 0.038f, 0.068f)
    private val curSkyHorizon = floatArrayOf(0.075f, 0.132f, 0.215f)
    private val curSkyGround = floatArrayOf(0.036f, 0.046f, 0.062f)
    private val curFogRgb = floatArrayOf(0.042f, 0.056f, 0.082f)
    private val curGridRgb = floatArrayOf(0.78f, 0.86f, 0.96f)
    private var curGridMajorAlpha = 0.19f
    private var curGridMinorAlpha = 0.065f
    private var themeInitialized = false

    private var progCloud = 0
    private var uCloudMvp = 0
    private var uCloudZMax = 0
    private var uCloudPointScale = 0
    private var uCloudZFlatten = 0
    private var uCloudColorMode = 0
    private var uCloudEyePos = 0
    private var uCloudDronePos = 0
    private var uCloudAheadPos = 0
    private var uCloudInnerR = 0
    private var uCloudOuterR = 0
    private var uCloudFogSpan = 0
    private var uCloudXrayOn = 0
    private var uCloudFogColor = 0
    private var aCloudPos = 0
    private var aCloudMeta = 0

    private var progPrim = 0
    private var uPrimMvp = 0
    private var uPrimFocusXY = 0
    private var uPrimFogSpan = 0
    private var uPrimFogColor = 0
    private var aPrimPos = 0
    private var aPrimColor = 0
    private var aPrimArc = 0

    private var progCutaway = 0
    private var uCutMvp = 0
    private var uCutEyePos = 0
    private var uCutFocusXY = 0
    private var uCutFogSpan = 0
    private var uCutSegDrone = 0
    private var uCutInvDroneLen2 = 0
    private var uCutSegAhead = 0
    private var uCutInvAheadLen2 = 0
    private var uCutInnerR = 0
    private var uCutOuterR = 0
    private var uCutFogColor = 0
    private var aCutPos = 0
    private var aCutColor = 0
    private var aCutArc = 0

    private var progOccluded = 0
    private var uOccMvp = 0
    private var uOccTime = 0
    private var aOccPos = 0
    private var aOccColor = 0
    private var aOccArc = 0

    // 9 路专属持久化 VBO
    private var vboCloudId = 0
    private var vboGroundLineId = 0
    private var vboSolidTriId = 0
    private var vboSolidEdgeId = 0
    private var vboGlassTriId = 0
    private var vboGlassEdgeId = 0
    private var vboFxTriId = 0
    private var vboActorLineId = 0
    private var vboActorTriId = 0

    private val gridKey = GeometryKey(16)
    private val voxelKey = GeometryKey(4096 * 4 + 64)
    private var gridCacheHit = false
    private var voxelCacheHit = false
    private var cachedGridVerts = 0
    private val cachedVoxelCounts = IntArray(4)
    private val cachedVoxelShadows = FloatArray(MAX_FX_TRI_VERTS * FLOATS_PER_VERT)
    private var cachedVoxelShadowVerts = 0
    private var cachedDroneBlocked = false
    private var previousHadCore = false
    private var vboStaticGridId = 0

    private val glSnapshot = RenderSnapshot()
    private val uiSnapshot = RenderSnapshot()

    val renderMetrics = RenderMetrics()
    internal var geometryCachingEnabled = true
    private val renderPending = PendingRender()
    private val framePacer = FramePacer()
    private val hudPacer = FramePacer()
    private var loopRunning = false
    private var lastAnimationNs = 0L

    @Volatile private var resetFrameClock = true
    private val frameCallback = object : Choreographer.FrameCallback {
        override fun doFrame(frameTimeNanos: Long) {
            if (!loopRunning) return
            if (framePacer.due(frameTimeNanos, if (hub.perfMode == 2) 30 else 60) && renderPending.request()) {
                requestRender()
            }
            if (hudPacer.due(frameTimeNanos, if (hub.perfMode == 2) 7 else 10)) {
                onHudTick(hub.snapshots.readInto(uiSnapshot))
            }
            if (loopRunning) Choreographer.getInstance().postFrameCallback(this)
        }
    }

    private val scaleDetector = ScaleGestureDetector(
        context,
        object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
            override fun onScale(detector: ScaleGestureDetector): Boolean {
                val factor = 1.0f / detector.scaleFactor.coerceIn(0.8f, 1.25f)
                worldHalfSpanDesired = (worldHalfSpanDesired * factor).coerceIn(2.2f, 65.0f)
                return true
            }
        }
    )

    private val gestureDetector = GestureDetector(
        context,
        object : GestureDetector.SimpleOnGestureListener() {
            override fun onDown(e: MotionEvent): Boolean = true

            override fun onScroll(e1: MotionEvent?, e2: MotionEvent, distanceX: Float, distanceY: Float): Boolean {
                if (e2.pointerCount >= 2) {
                    if (followMode != 0) {
                        val s = hub.snapshots.readInto(uiSnapshot)
                        camPanX = s.uavX
                        camPanY = s.uavY
                        followMode = 0
                        onFollowStateChanged(0, null)
                    }
                    panCameraByScreenDelta(distanceX, distanceY)
                    return true
                }

                if (scaleDetector.isInProgress) return true

                if (is3DMode && followMode != 0) {
                    if (followMode == 2 && abs(distanceX) > abs(distanceY) * 1.4f && abs(distanceX) > 4f * density) {
                        followMode = 1
                        onFollowStateChanged(1, null)
                    }
                    if (followMode == 1) {
                        camYawDesired += distanceX * 0.0072f / density * 1.6f
                    }
                    camPitchDesired = (camPitchDesired + distanceY * 0.0062f / density * 1.6f).coerceIn(0.18f, 1.44f)
                } else {
                    if (!is3DMode && followMode != 0 && hypot(distanceX, distanceY) > 6f * density) {
                        val s = hub.snapshots.readInto(uiSnapshot)
                        camPanX = s.uavX
                        camPanY = s.uavY
                        followMode = 0
                        onFollowStateChanged(0, null)
                    }
                    panCameraByScreenDelta(distanceX, distanceY)
                }
                return true
            }

            override fun onDoubleTap(e: MotionEvent): Boolean {
                if (onInterceptDrawerClose()) return true
                if (hub.hapticFeedbackEnabled) performHapticFeedback(HapticFeedbackConstants.KEYBOARD_TAP)
                recenterLockDrone()
                onFollowStateChanged(1, "已平滑回中并锁定跟随无人机")
                return true
            }

            override fun onSingleTapConfirmed(e: MotionEvent): Boolean {
                if (onInterceptDrawerClose()) return true
                val world = unprojectToGroundUiThread(e.x, e.y) ?: return true
                if (hub.hapticFeedbackEnabled) performHapticFeedback(HapticFeedbackConstants.KEYBOARD_TAP)
                spawnRipple(world[0], world[1])
                triggerPinSpringAnim()
                onSingleTapPreviewPin(world[0], world[1])
                return true
            }

            override fun onLongPress(e: MotionEvent) {
                if (multiTouchActive || scaleDetector.isInProgress) return
                if (onInterceptDrawerClose()) return
                val world = unprojectToGroundUiThread(e.x, e.y) ?: return
                if (!hub.longPressQuickFlyEnabled) {
                    if (hub.hapticFeedbackEnabled) performHapticFeedback(HapticFeedbackConstants.KEYBOARD_TAP)
                    spawnRipple(world[0], world[1])
                    triggerPinSpringAnim()
                    onSingleTapPreviewPin(world[0], world[1])
                    return
                }
                if (hub.hapticFeedbackEnabled) performHapticFeedback(HapticFeedbackConstants.LONG_PRESS)
                spawnRipple(world[0], world[1])
                onLongPressQuickFly(world[0], world[1])
            }
        }
    )

    private fun panCameraByScreenDelta(distanceX: Float, distanceY: Float) {
        val metersPerPx = (worldHalfSpan * 2.0f) / max(1f, viewportH.toFloat())
        val effY = camYaw * projBlend
        val cosY = cos(effY)
        val sinY = sin(effY)
        val dxScreen = distanceX * metersPerPx
        val dyScreen = -distanceY * metersPerPx
        camPanX += dxScreen * cosY - dyScreen * sinY
        camPanY += dxScreen * sinY + dyScreen * cosY
    }

    init {
        setEGLContextClientVersion(2)
        setEGLConfigChooser(8, 8, 8, 8, 16, 0)
        preserveEGLContextOnPause = true
        setRenderer(this)
        renderMode = RENDERMODE_WHEN_DIRTY
    }

    @SuppressLint("ClickableViewAccessibility")
    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.actionMasked) {
            MotionEvent.ACTION_POINTER_DOWN -> multiTouchActive = true
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> multiTouchActive = false
        }
        scaleDetector.onTouchEvent(event)
        gestureDetector.onTouchEvent(event)
        return true
    }

    fun startLoop() {
        if (loopRunning) return
        loopRunning = true
        resetFrameClock = true
        framePacer.reset()
        hudPacer.reset()
        onResume()
        Choreographer.getInstance().postFrameCallback(frameCallback)
    }

    fun stopLoop() {
        if (!loopRunning) return
        loopRunning = false
        Choreographer.getInstance().removeFrameCallback(frameCallback)
        onPause()
        renderPending.complete()
    }

    override fun onDetachedFromWindow() {
        stopLoop()
        super.onDetachedFromWindow()
    }

    fun triggerPinSpringAnim() {
        pinAnimator?.cancel()
        pinAnimScale = 0.15f
        pinAnimator = ValueAnimator.ofFloat(0.15f, 1.0f).apply {
            duration = 250L
            interpolator = OvershootInterpolator(1.55f)
            addUpdateListener { anim -> pinAnimScale = anim.animatedValue as Float }
            start()
        }
    }

    private fun animateProjectionTo(target3D: Boolean) {
        if (is3DMode == target3D) return
        is3DMode = target3D
        blendAnimator?.cancel()
        blendAnimator = ValueAnimator.ofFloat(projBlend, if (target3D) 1f else 0f).apply {
            duration = 400L
            interpolator = AnimHelper.EASE_OUT_EXPO
            addUpdateListener { anim -> projBlend = anim.animatedValue as Float }
            start()
        }
    }

    fun toggle3D2DWithAnimation(): Boolean {
        animateProjectionTo(!is3DMode)
        return is3DMode
    }

    fun recenterLockDrone() {
        camPanX = 0f
        camPanY = 0f
        followMode = 1
    }

    fun cycleFollowMode(): Pair<Int, String> {
        followMode = when (followMode) {
            1 -> {
                camPanX = 0f
                camPanY = 0f
                camPitchDesired = 0.38f
                2
            }
            2 -> {
                val s = hub.snapshots.readInto(uiSnapshot)
                camPanX = s.uavX
                camPanY = s.uavY
                0
            }
            else -> {
                camPanX = 0f
                camPanY = 0f
                camPitchDesired = 0.55f
                1
            }
        }
        val tip = when (followMode) {
            2 -> "相机视角：机尾低仰角追尾透视"
            0 -> "相机视角：自由平移漫游 (双击回中)"
            else -> "相机视角：锁定飞机 · 360° 自由环视"
        }
        return followMode to tip
    }

    fun zoomBy(factor: Float) {
        worldHalfSpanDesired = (worldHalfSpanDesired / factor).coerceIn(2.2f, 65.0f)
    }

    fun resetCameraSmooth() {
        followMode = 1
        camYawDesired = 0.62f
        camPitchDesired = 0.55f
        worldHalfSpanDesired = 6.4f
        camPanX = 0f
        camPanY = 0f
        animateProjectionTo(true)
    }

    fun cycleColorMode(): Pair<Int, String> {
        val next = (hub.pointColorMode + 1) % 3
        hub.pointColorMode = next
        val tip = when (next) {
            1 -> "点云着色：反射强度伪彩"
            2 -> "点云着色：战术翠绿单色"
            else -> "点云着色：高程彩虹伪彩"
        }
        return next to tip
    }

    fun spawnRipple(wx: Float, wy: Float) {
        synchronized(rippleLock) {
            for (i in rippleAge.indices) {
                if (rippleAge[i] < 0f || rippleAge[i] > 1.15f) {
                    rippleX[i] = wx
                    rippleY[i] = wy
                    rippleAge[i] = 0f
                    return
                }
            }
            rippleX[0] = wx
            rippleY[0] = wy
            rippleAge[0] = 0f
        }
    }

    private fun unprojectToGroundUiThread(sx: Float, sy: Float): FloatArray? {
        val vw = max(1, width).toFloat()
        val vh = max(1, height).toFloat()
        val ndcX = (sx / vw) * 2f - 1f
        val ndcY = 1f - (sy / vh) * 2f

        synchronized(invVpLock) {
            if (!invVpValid) return null
            System.arraycopy(sharedInvVp, 0, uiInvCopy16, 0, 16)
        }

        uiNearIn4[0] = ndcX
        uiNearIn4[1] = ndcY
        uiNearIn4[2] = -1f
        uiNearIn4[3] = 1f
        uiFarIn4[0] = ndcX
        uiFarIn4[1] = ndcY
        uiFarIn4[2] = 1f
        uiFarIn4[3] = 1f
        Matrix.multiplyMV(uiNearOut4, 0, uiInvCopy16, 0, uiNearIn4, 0)
        Matrix.multiplyMV(uiFarOut4, 0, uiInvCopy16, 0, uiFarIn4, 0)
        if (abs(uiNearOut4[3]) < 1e-6f || abs(uiFarOut4[3]) < 1e-6f) return null

        val nx = uiNearOut4[0] / uiNearOut4[3]
        val ny = uiNearOut4[1] / uiNearOut4[3]
        val nz = uiNearOut4[2] / uiNearOut4[3]
        val fx = uiFarOut4[0] / uiFarOut4[3]
        val fy = uiFarOut4[1] / uiFarOut4[3]
        val fz = uiFarOut4[2] / uiFarOut4[3]

        val dz = fz - nz
        if (abs(dz) < 1e-5f) return null
        val t = -nz / dz
        if (t < 0f || t > 1.5f) return null

        val wx = nx + t * (fx - nx)
        val wy = ny + t * (fy - ny)
        if (!hub.isRosMode) {
            uiGroundOut2[0] = wx.coerceIn(-7.6f, 7.6f)
            uiGroundOut2[1] = wy.coerceIn(-7.6f, 7.6f)
        } else {
            uiGroundOut2[0] = wx.coerceIn(-500f, 500f)
            uiGroundOut2[1] = wy.coerceIn(-500f, 500f)
        }
        return uiGroundOut2
    }

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.038f, 0.048f, 0.065f, 1.0f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        GLES20.glDepthFunc(GLES20.GL_LEQUAL)
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA)

        progSky = buildProgram(SKY_VS, SKY_FS)
        aSkyPos = GLES20.glGetAttribLocation(progSky, "aNdcPos")
        uSkyHorizonY = GLES20.glGetUniformLocation(progSky, "uHorizonY")
        uSkyBlend3D = GLES20.glGetUniformLocation(progSky, "uBlend3D")
        uSkyFlat = GLES20.glGetUniformLocation(progSky, "uSkyFlat")
        uSkyZenith = GLES20.glGetUniformLocation(progSky, "uSkyZenith")
        uSkyHorizon = GLES20.glGetUniformLocation(progSky, "uSkyHorizon")
        uSkyGround = GLES20.glGetUniformLocation(progSky, "uSkyGround")

        progCloud = buildProgram(POINT_CLOUD_VS, POINT_CLOUD_FS)
        uCloudMvp = GLES20.glGetUniformLocation(progCloud, "uMVP")
        uCloudZMax = GLES20.glGetUniformLocation(progCloud, "uZMax")
        uCloudPointScale = GLES20.glGetUniformLocation(progCloud, "uPointScale")
        uCloudZFlatten = GLES20.glGetUniformLocation(progCloud, "uZFlatten")
        uCloudColorMode = GLES20.glGetUniformLocation(progCloud, "uColorMode")
        uCloudEyePos = GLES20.glGetUniformLocation(progCloud, "uEyePos")
        uCloudDronePos = GLES20.glGetUniformLocation(progCloud, "uDronePos")
        uCloudAheadPos = GLES20.glGetUniformLocation(progCloud, "uAheadPos")
        uCloudInnerR = GLES20.glGetUniformLocation(progCloud, "uInnerR")
        uCloudOuterR = GLES20.glGetUniformLocation(progCloud, "uOuterR")
        uCloudFogSpan = GLES20.glGetUniformLocation(progCloud, "uFogSpan")
        uCloudXrayOn = GLES20.glGetUniformLocation(progCloud, "uXrayOn")
        uCloudFogColor = GLES20.glGetUniformLocation(progCloud, "uFogColor")
        aCloudPos = GLES20.glGetAttribLocation(progCloud, "aPos")
        aCloudMeta = GLES20.glGetAttribLocation(progCloud, "aMeta")

        progPrim = buildProgram(PRIM_VS, PRIM_FS)
        uPrimMvp = GLES20.glGetUniformLocation(progPrim, "uMVP")
        uPrimFocusXY = GLES20.glGetUniformLocation(progPrim, "uFocusXY")
        uPrimFogSpan = GLES20.glGetUniformLocation(progPrim, "uFogSpan")
        uPrimFogColor = GLES20.glGetUniformLocation(progPrim, "uFogColor")
        aPrimPos = GLES20.glGetAttribLocation(progPrim, "aPos")
        aPrimColor = GLES20.glGetAttribLocation(progPrim, "aColor")
        aPrimArc = GLES20.glGetAttribLocation(progPrim, "aArc")

        progCutaway = buildProgram(CUTAWAY_VS, CUTAWAY_FS)
        uCutMvp = GLES20.glGetUniformLocation(progCutaway, "uMVP")
        uCutEyePos = GLES20.glGetUniformLocation(progCutaway, "uEyePos")
        uCutFocusXY = GLES20.glGetUniformLocation(progCutaway, "uFocusXY")
        uCutFogSpan = GLES20.glGetUniformLocation(progCutaway, "uFogSpan")
        uCutSegDrone = GLES20.glGetUniformLocation(progCutaway, "uSegDrone")
        uCutInvDroneLen2 = GLES20.glGetUniformLocation(progCutaway, "uInvDroneLen2")
        uCutSegAhead = GLES20.glGetUniformLocation(progCutaway, "uSegAhead")
        uCutInvAheadLen2 = GLES20.glGetUniformLocation(progCutaway, "uInvAheadLen2")
        uCutInnerR = GLES20.glGetUniformLocation(progCutaway, "uInnerR")
        uCutOuterR = GLES20.glGetUniformLocation(progCutaway, "uOuterR")
        uCutFogColor = GLES20.glGetUniformLocation(progCutaway, "uFogColor")
        aCutPos = GLES20.glGetAttribLocation(progCutaway, "aPos")
        aCutColor = GLES20.glGetAttribLocation(progCutaway, "aColor")
        aCutArc = GLES20.glGetAttribLocation(progCutaway, "aArc")

        progOccluded = buildProgram(OCCLUDED_VS, OCCLUDED_FS)
        uOccMvp = GLES20.glGetUniformLocation(progOccluded, "uMVP")
        uOccTime = GLES20.glGetUniformLocation(progOccluded, "uTime")
        aOccPos = GLES20.glGetAttribLocation(progOccluded, "aPos")
        aOccColor = GLES20.glGetAttribLocation(progOccluded, "aColor")
        aOccArc = GLES20.glGetAttribLocation(progOccluded, "aArc")

        val bufs = IntArray(10)
        GLES20.glGenBuffers(10, bufs, 0)
        vboCloudId = bufs[0]
        vboGroundLineId = bufs[1]
        vboSolidTriId = bufs[2]
        vboSolidEdgeId = bufs[3]
        vboGlassTriId = bufs[4]
        vboGlassEdgeId = bufs[5]
        vboFxTriId = bufs[6]
        vboActorLineId = bufs[7]
        vboActorTriId = bufs[8]
        vboStaticGridId = bufs[9]
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboStaticGridId)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, 128 * STRIDE_VERT, null, GLES20.GL_STATIC_DRAW)

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboCloudId)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, FastLioVoxelMap.MAX_POINTS * FastLioVoxelMap.BYTES_PER_VERTEX, null, GLES20.GL_DYNAMIC_DRAW)

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboGroundLineId)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, MAX_ENV_LINE_VERTS * STRIDE_VERT, null, GLES20.GL_STREAM_DRAW)

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboSolidTriId)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, MAX_ENV_TRI_VERTS * STRIDE_VERT, null, GLES20.GL_STREAM_DRAW)

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboSolidEdgeId)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, MAX_ENV_LINE_VERTS * STRIDE_VERT, null, GLES20.GL_STREAM_DRAW)

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboGlassTriId)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, MAX_GLASS_TRI_VERTS * STRIDE_VERT, null, GLES20.GL_STREAM_DRAW)

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboGlassEdgeId)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, MAX_GLASS_LINE_VERTS * STRIDE_VERT, null, GLES20.GL_STREAM_DRAW)

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboFxTriId)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, MAX_FX_TRI_VERTS * STRIDE_VERT, null, GLES20.GL_STREAM_DRAW)

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboActorLineId)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, MAX_ACTOR_LINE_VERTS * STRIDE_VERT, null, GLES20.GL_STREAM_DRAW)

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboActorTriId)
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, MAX_ACTOR_TRI_VERTS * STRIDE_VERT, null, GLES20.GL_STREAM_DRAW)

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
        vectorGlb.create()
        voxelMap.notifyGlContextRecreated()
        gridKey.invalidate()
        voxelKey.invalidate()
        resetFrameClock = true
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        viewportW = max(1, width)
        viewportH = max(1, height)
        GLES20.glViewport(0, 0, viewportW, viewportH)
    }

    override fun onDrawFrame(gl: GL10?) {
        val frameStartNs = System.nanoTime()
        val animationNs = animationTimeNanos()
        val rawDt = if (resetFrameClock) 1f / 60f else (frameStartNs - lastFrameNs) * 1e-9f
        val dt = if (resetFrameClock) 1f / 60f else ((animationNs - lastAnimationNs) * 1e-9f).coerceIn(0f, 0.05f)
        resetFrameClock = false
        lastFrameNs = frameStartNs
        lastAnimationNs = animationNs
        shaderClockSec = (shaderClockSec + dt) % 3600f
        try {
            fpsFrameCount++
            fpsTimeAcc += rawDt
            if (fpsTimeAcc >= 0.5f) {
                hub.renderFps = (fpsFrameCount / fpsTimeAcc).roundToInt().coerceIn(1, 120)
                fpsFrameCount = 0
                fpsTimeAcc = 0f
            }

            val snap = hub.snapshots.readInto(glSnapshot)
            stepThemeAtmosphereBlend(dt)
            updateCameraMatrices(dt, snap)

            GLES20.glClearColor(curSkyFlat[0], curSkyFlat[1], curSkyFlat[2], 1.0f)
            GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)

            // Pass 0: 3D 天际大气渐变穹顶
            drawSkyDomePass()

            groundLineVerts = 0
            solidObsTriVerts = 0
            solidObsEdgeVerts = 0
            glassObsTriVerts = 0
            glassObsEdgeVerts = 0
            fxAlphaTriVerts = 0
            actorLineVerts = 0
            actorTriVerts = 0

            buildSceneBatches(dt, snap)

            val zFlat = 0.08f + 0.92f * projBlend

            // Pass 1: 透视地面网格与同心测距环 (开启混合，不写深度)
            GLES20.glEnable(GLES20.GL_DEPTH_TEST)
            GLES20.glDepthFunc(GLES20.GL_LEQUAL)
            GLES20.glDepthMask(false)
            GLES20.glEnable(GLES20.GL_BLEND)
            flushEnvLines(vboStaticGridId, groundLineArray, cachedGridVerts, max(1f, 1.12f * density), upload = !gridCacheHit)
            flushEnvLines(vboGroundLineId, groundLineArray, groundLineVerts - cachedGridVerts, max(1f, 1.12f * density), sourceOffset = cachedGridVerts)

            // Pass 2: 100% 纯不透明实体膨胀体素与 4x4 Bayer 消隐点云 (关闭实体三角形混合，彻底杜绝地面网格穿透实体障碍)
            GLES20.glDepthMask(true)
            GLES20.glDisable(GLES20.GL_BLEND)
            flushEnvTriangles(vboSolidTriId, solidObsTriArray, solidObsTriVerts, upload = !voxelCacheHit || snap.obsPrimitiveCount > 0)

            GLES20.glEnable(GLES20.GL_BLEND)
            flushEnvLines(vboSolidEdgeId, solidObsEdgeArray, solidObsEdgeVerts, max(1f, 1.0f * density), upload = !voxelCacheHit || snap.obsPrimitiveCount > 0)

            val cloudVerts = voxelMap.syncToGpuVbo(vboCloudId)
            val pMode = hub.perfMode
            val drawCloudCnt = if (pMode == 2) min(cloudVerts, 90_000) else cloudVerts
            if (hub.showFastLioMap && drawCloudCnt > 0) {
                GLES20.glUseProgram(progCloud)
                GLES20.glUniformMatrix4fv(uCloudMvp, 1, false, vpMatrix, 0)
                GLES20.glUniform1f(uCloudZMax, hub.zSliceMax)
                val fovScale = (viewportH / 720f).coerceAtLeast(0.75f) * 34.0f * hub.pointSizeScale * density
                GLES20.glUniform1f(uCloudPointScale, fovScale * (0.35f + 0.65f * projBlend))
                GLES20.glUniform1f(uCloudZFlatten, zFlat)
                GLES20.glUniform1i(uCloudColorMode, hub.pointColorMode)
                GLES20.glUniform3f(uCloudEyePos, curEyeX, curEyeY, curEyeZ)
                GLES20.glUniform3f(uCloudDronePos, snap.uavX, snap.uavY, snap.uavZ * zFlat)
                GLES20.glUniform3f(uCloudAheadPos, curAheadX, curAheadY, curAheadZ)
                GLES20.glUniform1f(uCloudInnerR, curCutInnerR)
                GLES20.glUniform1f(uCloudOuterR, curCutOuterR)
                GLES20.glUniform1f(uCloudFogSpan, worldHalfSpan * 2.2f)
                GLES20.glUniform1i(uCloudXrayOn, if (hub.xrayOcclusionEnabled && projBlend > 0.12f) 1 else 0)
                GLES20.glUniform3f(uCloudFogColor, curFogRgb[0], curFogRgb[1], curFogRgb[2])

                GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboCloudId)
                GLES20.glEnableVertexAttribArray(aCloudPos)
                GLES20.glVertexAttribPointer(aCloudPos, 3, GLES20.GL_FLOAT, false, FastLioVoxelMap.BYTES_PER_VERTEX, 0)
                GLES20.glEnableVertexAttribArray(aCloudMeta)
                GLES20.glVertexAttribPointer(aCloudMeta, 1, GLES20.GL_FLOAT, false, FastLioVoxelMap.BYTES_PER_VERTEX, 12)

                GLES20.glDrawArrays(GLES20.GL_POINTS, 0, drawCloudCnt)

                GLES20.glDisableVertexAttribArray(aCloudPos)
                GLES20.glDisableVertexAttribArray(aCloudMeta)
            }

            uploadActorVbos()

            // Pass 3: 前景实体无人机与规划航线遍 (在半透明遮挡体素之前绘制！确保随后透镜外壳按真实物理顺序覆盖在其前方)
            GLES20.glDepthFunc(GLES20.GL_LEQUAL)
            GLES20.glDepthMask(true)
            drawActorBatchesVisible(max(1.2f, 1.5f * density))
            drawVectorGlbModels(snap, zFlat, occluded = false)

            // Pass 4: 视线走廊遮挡体素「单层外表皮深度预通透镜」(Single-Skin Depth-Prepass Cutaway Lens)
            // 4a: 静默深度预通 (仅写最近外表面 Z 值，零颜色输出) -> 4b: GL_EQUAL 单层外皮逐像素平滑消隐 (100% 消除内部共享乱面与多层叠加混浊！)
            if (glassObsTriVerts > 0) {
                drawCutawaySingleSkinVolume()
                if (glassObsEdgeVerts > 0) {
                    GLES20.glDepthFunc(GLES20.GL_LEQUAL)
                    GLES20.glDepthMask(false)
                    flushEnvLines(vboGlassEdgeId, glassObsEdgeArray, glassObsEdgeVerts, max(1.0f, 1.15f * density), upload = !voxelCacheHit || snap.obsPrimitiveCount > 0)
                }
            }

            // Pass 5: 硬件深度逆测试 GL_GREATER 遮挡 X-Ray 增强遍 (对任何处于实体墙或透视薄壳后方的轨迹叠加双色抗锯齿流动虚线！)
            if (hub.xrayOcclusionEnabled && projBlend > 0.12f) {
                GLES20.glDepthFunc(GLES20.GL_GREATER)
                GLES20.glDepthMask(false)
                drawActorBatchesOccluded(max(1.25f, 1.55f * density))
                drawVectorGlbModels(snap, zFlat, occluded = true)
            }

            // Pass 6: 半透明高程垂幕、接地阴影与触控波纹
            GLES20.glDepthFunc(GLES20.GL_LEQUAL)
            GLES20.glDepthMask(false)
            flushEnvTriangles(vboFxTriId, fxAlphaTriArray, fxAlphaTriVerts)

            GLES20.glDepthMask(true)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)

            val workNs = System.nanoTime() - frameStartNs
            val workMs = workNs * 1e-6f
            hub.frameRenderMs += (workMs - hub.frameRenderMs) * 0.16f
            renderMetrics.record(workNs, voxelMap.uploadedPointsLastFrame, voxelCacheHit)
        } finally {
            renderPending.complete()
        }
    }

    private fun drawCutawaySingleSkinVolume() {
        if (!voxelCacheHit || glSnapshot.obsPrimitiveCount > 0) {
            envTriBuf.clear()
            envTriBuf.put(glassObsTriArray, 0, glassObsTriVerts * FLOATS_PER_VERT)
            envTriBuf.position(0)
        }

        GLES20.glUseProgram(progCutaway)
        GLES20.glUniformMatrix4fv(uCutMvp, 1, false, vpMatrix, 0)
        GLES20.glUniform3f(uCutEyePos, curEyeX, curEyeY, curEyeZ)
        GLES20.glUniform2f(uCutFocusXY, camFocusX, camFocusY)
        GLES20.glUniform1f(uCutFogSpan, worldHalfSpan * 2.2f)
        GLES20.glUniform3f(uCutSegDrone, curSegDroneX, curSegDroneY, curSegDroneZ)
        GLES20.glUniform1f(uCutInvDroneLen2, curInvDroneLen2)
        GLES20.glUniform3f(uCutSegAhead, curSegAheadX, curSegAheadY, curSegAheadZ)
        GLES20.glUniform1f(uCutInvAheadLen2, curInvAheadLen2)
        GLES20.glUniform1f(uCutInnerR, curCutInnerR)
        GLES20.glUniform1f(uCutOuterR, curCutOuterR)
        GLES20.glUniform3f(uCutFogColor, curFogRgb[0], curFogRgb[1], curFogRgb[2])

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboGlassTriId)
        if (!voxelCacheHit || glSnapshot.obsPrimitiveCount > 0) {
            GLES20.glBufferSubData(GLES20.GL_ARRAY_BUFFER, 0, glassObsTriVerts * STRIDE_VERT, envTriBuf)
        }

        GLES20.glEnableVertexAttribArray(aCutPos)
        GLES20.glVertexAttribPointer(aCutPos, 3, GLES20.GL_FLOAT, false, STRIDE_VERT, 0)
        GLES20.glEnableVertexAttribArray(aCutColor)
        GLES20.glVertexAttribPointer(aCutColor, 4, GLES20.GL_FLOAT, false, STRIDE_VERT, 12)
        if (aCutArc >= 0) {
            GLES20.glEnableVertexAttribArray(aCutArc)
            GLES20.glVertexAttribPointer(aCutArc, 1, GLES20.GL_FLOAT, false, STRIDE_VERT, 28)
        }

        // Step 4a: Depth-Only Pre-Pass — 仅将整个膨胀体素簇朝向相机的最前表面写入硬件 Z-Buffer
        GLES20.glColorMask(false, false, false, false)
        GLES20.glDepthMask(true)
        GLES20.glDepthFunc(GLES20.GL_LEQUAL)
        GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, glassObsTriVerts)

        // Step 4b: Single-Skin Color Pass — 严格匹配 GL_EQUAL 最外层表面，所有内部共享面与后层体素 100% 硬件剔除！
        GLES20.glColorMask(true, true, true, true)
        GLES20.glDepthMask(false)
        GLES20.glDepthFunc(GLES20.GL_EQUAL)
        GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, glassObsTriVerts)

        GLES20.glDisableVertexAttribArray(aCutPos)
        GLES20.glDisableVertexAttribArray(aCutColor)
        if (aCutArc >= 0) GLES20.glDisableVertexAttribArray(aCutArc)
    }

    private fun stepThemeAtmosphereBlend(dt: Float) {
        val pal = GcsTheme.of(hub.themeId)
        val k = if (!themeInitialized) {
            themeInitialized = true
            1.0f
        } else {
            1.0f - exp(-dt * 9.0f)
        }
        for (i in 0..2) {
            curSkyFlat[i] += (pal.glSkyFlat[i] - curSkyFlat[i]) * k
            curSkyZenith[i] += (pal.glSkyZenith[i] - curSkyZenith[i]) * k
            curSkyHorizon[i] += (pal.glSkyHorizon[i] - curSkyHorizon[i]) * k
            curSkyGround[i] += (pal.glSkyGround[i] - curSkyGround[i]) * k
            curFogRgb[i] += (pal.glFog[i] - curFogRgb[i]) * k
            curGridRgb[i] += (pal.glGridRgb[i] - curGridRgb[i]) * k
        }
        curGridMajorAlpha += (pal.glGridMajorAlpha - curGridMajorAlpha) * k
        curGridMinorAlpha += (pal.glGridMinorAlpha - curGridMinorAlpha) * k
    }

    private fun drawSkyDomePass() {
        GLES20.glDisable(GLES20.GL_DEPTH_TEST)
        GLES20.glDepthMask(false)
        GLES20.glDisable(GLES20.GL_BLEND)
        GLES20.glUseProgram(progSky)
        val horizonUvY = (0.93f - camPitch * 0.35f).coerceIn(0.40f, 0.92f)
        GLES20.glUniform1f(uSkyHorizonY, horizonUvY)
        GLES20.glUniform1f(uSkyBlend3D, projBlend)
        GLES20.glUniform3f(uSkyFlat, curSkyFlat[0], curSkyFlat[1], curSkyFlat[2])
        GLES20.glUniform3f(uSkyZenith, curSkyZenith[0], curSkyZenith[1], curSkyZenith[2])
        GLES20.glUniform3f(uSkyHorizon, curSkyHorizon[0], curSkyHorizon[1], curSkyHorizon[2])
        GLES20.glUniform3f(uSkyGround, curSkyGround[0], curSkyGround[1], curSkyGround[2])

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
        skyQuadBuf.position(0)
        GLES20.glEnableVertexAttribArray(aSkyPos)
        GLES20.glVertexAttribPointer(aSkyPos, 2, GLES20.GL_FLOAT, false, 0, skyQuadBuf)
        GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4)
        GLES20.glDisableVertexAttribArray(aSkyPos)
    }

    fun focusWorldPointSmooth(wx: Float, wy: Float) {
        followMode = 0
        camPanX = wx
        camPanY = wy
        spawnRipple(wx, wy)
        onFollowStateChanged(0, null)
    }

    private fun updateCameraMatrices(dt: Float, snap: RenderSnapshot) {
        val damp = hub.cameraDampingScale.coerceIn(0.45f, 2.0f)
        val kRot = 1f - exp(-dt * 10.5f * damp)
        val kZoom = 1f - exp(-dt * 13.0f * damp)

        if (followMode == 2) {
            val chaseYaw = snap.yaw - PI.toFloat() * 0.5f
            var diff = chaseYaw - camYaw
            while (diff > PI.toFloat()) diff -= (PI * 2.0).toFloat()
            while (diff < -PI.toFloat()) diff += (PI * 2.0).toFloat()
            camYaw += diff * (1f - exp(-dt * 6.5f))
            camYawDesired = camYaw
        } else {
            camYaw += (camYawDesired - camYaw) * kRot
        }
        camPitch += (camPitchDesired - camPitch) * kRot
        worldHalfSpan += (worldHalfSpanDesired - worldHalfSpan) * kZoom

        val b = projBlend
        val desFX = if (followMode != 0) snap.uavX + camPanX else camPanX
        val desFY = if (followMode != 0) snap.uavY + camPanY else camPanY
        val desFZ = if (followMode != 0) (snap.uavZ * 0.45f) * b else 0f
        val kPos = 1f - exp(-dt * 11.5f * damp)
        camFocusX += (desFX - camFocusX) * kPos
        camFocusY += (desFY - camFocusY) * kPos
        camFocusZ += (desFZ - camFocusZ) * kPos

        val effYaw = camYaw * b
        val effPitch = 1.568f * (1f - b) + camPitch * b
        // 51° 自然广角透视视场，强化近大远小的三维空间纵深与视差立体感
        val fovYDeg = 4.2f + 46.8f * b
        val halfFovRad = (fovYDeg * 0.5f) * 0.017453292f
        val camDist = worldHalfSpan / max(0.02f, tan(halfFovRad))

        val horizBack = camDist * cos(effPitch)
        curEyeX = camFocusX + sin(effYaw) * horizBack
        curEyeY = camFocusY - cos(effYaw) * horizBack
        curEyeZ = camFocusZ + camDist * sin(effPitch)

        Matrix.setLookAtM(
            viewMatrix, 0,
            curEyeX, curEyeY, curEyeZ,
            camFocusX, camFocusY, camFocusZ,
            0f, 0f, 1f
        )

        val aspect = viewportW.toFloat() / max(1f, viewportH.toFloat())
        val nearClip = max(0.25f, camDist * 0.04f)
        val farClip = camDist * 8.5f + 260f
        Matrix.perspectiveM(projMatrix, 0, fovYDeg, aspect, nearClip, farClip)
        Matrix.multiplyMM(vpMatrix, 0, projMatrix, 0, viewMatrix, 0)

        if (Matrix.invertM(glTmpInv16, 0, vpMatrix, 0)) {
            synchronized(invVpLock) {
                System.arraycopy(glTmpInv16, 0, sharedInvVp, 0, 16)
                invVpValid = true
            }
        }

        synchronized(rippleLock) {
            for (i in rippleAge.indices) {
                if (rippleAge[i] >= 0f) {
                    rippleAge[i] += dt
                    if (rippleAge[i] > 1.15f) rippleAge[i] = -1f
                }
            }
        }
    }

    /**
     * 胶囊视线距离平方计算：覆盖 [相机 -> 目标] 全区间 (rawT <= 1.03)，
     * 解决紧贴飞机机头前方的膨胀障碍物此前因 t >= 0.96 被漏判而不透明的核心缺陷！
     */
    private fun distanceSqToCapsuleSeg(
        px: Float,
        py: Float,
        pz: Float,
        sx: Float,
        sy: Float,
        sz: Float,
        invLen2: Float
    ): Float {
        if (invLen2 <= 0f) return 9999f
        val rx = px - curEyeX
        val ry = py - curEyeY
        val rz = pz - curEyeZ
        val rawT = (rx * sx + ry * sy + rz * sz) * invLen2
        if (rawT <= 0.01f || rawT >= 1.03f) return 9999f
        val tc = rawT.coerceIn(0.02f, 1.0f)
        val cx = tc * sx - rx
        val cy = tc * sy - ry
        val cz = tc * sz - rz
        return cx * cx + cy * cy + cz * cz
    }

    private fun distanceFromSightRayToColumn(
        cx: Float,
        cy: Float,
        rx: Float,
        ry: Float,
        zBottom: Float,
        zTop: Float,
        isCircle: Boolean,
        uavX: Float,
        uavY: Float,
        uavZ: Float
    ): Float {
        val vxRay = uavX - curEyeX
        val vyRay = uavY - curEyeY
        val horizLen2 = vxRay * vxRay + vyRay * vyRay
        if (horizLen2 < 0.05f) return 99f

        val rawS = ((cx - curEyeX) * vxRay + (cy - curEyeY) * vyRay) / horizLen2
        if (rawS <= 0.02f || rawS >= 1.03f) return 99f
        val s = rawS.coerceIn(0.03f, 1.0f)
        val px = curEyeX + s * vxRay
        val py = curEyeY + s * vyRay
        val pz = curEyeZ + s * (uavZ - curEyeZ)

        val d2d = if (isCircle) {
            hypot(px - cx, py - cy) - rx
        } else {
            val dx = abs(px - cx) - rx
            val dy = abs(py - cy) - ry
            hypot(max(dx, 0f), max(dy, 0f)) + min(max(dx, dy), 0f)
        }
        val dz = when {
            pz > zTop -> pz - zTop
            pz < zBottom -> zBottom - pz
            else -> 0f
        }
        return hypot(max(0f, d2d), dz)
    }

    private fun computeInflateVoxelColor(vzRaw: Float, zMax: Float, inLocalWindow: Boolean, outRgb: FloatArray) {
        val t = (vzRaw / max(0.8f, zMax * 0.85f)).coerceIn(0f, 1f)
        val r: Float
        val g: Float
        val b: Float
        if (t < 0.35f) {
            val u = t / 0.35f
            r = 0.10f + (0.06f - 0.10f) * u
            g = 0.56f + (0.86f - 0.56f) * u
            b = 0.96f + (0.58f - 0.96f) * u
        } else if (t < 0.72f) {
            val u = (t - 0.35f) / 0.37f
            r = 0.06f + (0.98f - 0.06f) * u
            g = 0.86f + (0.72f - 0.86f) * u
            b = 0.58f + (0.14f - 0.58f) * u
        } else {
            val u = (t - 0.72f) / 0.28f
            r = 0.98f + (1.00f - 0.98f) * u
            g = 0.72f + (0.30f - 0.72f) * u
            b = 0.14f + (0.22f - 0.14f) * u
        }
        val boost = if (inLocalWindow) 1.0f else 0.76f
        outRgb[0] = r * boost
        outRgb[1] = g * boost
        outRgb[2] = b * boost
    }

    private fun buildSceneBatches(dt: Float, snap: RenderSnapshot) {
        val b = projBlend
        val zScale = 0.08f + 0.92f * b
        val uavScaledZ = snap.uavZ * zScale

        gridKey.begin()
        gridKey.add(if (hub.showGroundGrid) 1f else 0f)
        gridKey.add(floor(camFocusX / 4f) * 4f)
        gridKey.add(floor(camFocusY / 4f) * 4f)
        gridKey.add(b)
        gridKey.add(curGridRgb)
        gridKey.add(curGridMajorAlpha)
        gridKey.add(curGridMinorAlpha)
        gridCacheHit = geometryCachingEnabled && gridKey.unchanged()
        if (gridCacheHit) {
            groundLineVerts = cachedGridVerts
        } else {
            // 1. 透视地面网格与世界原点三维 XYZ 参考轴
            if (hub.showGroundGrid) {
                val anchorX = floor(camFocusX / 4f) * 4f
                val anchorY = floor(camFocusY / 4f) * 4f
                val halfGrid = 14
                for (g in -halfGrid..halfGrid) {
                    val gx = anchorX + g
                    val gy = anchorY + g
                    val majorX = (abs(gx.toInt()) % 4 == 0)
                    val majorY = (abs(gy.toInt()) % 4 == 0)
                    val fade = (1f - (abs(g) / 15.0f) * (abs(g) / 15.0f)).coerceAtLeast(0.12f)
                    val aX = (if (majorX) curGridMajorAlpha else curGridMinorAlpha) * fade
                    val aY = (if (majorY) curGridMajorAlpha else curGridMinorAlpha) * fade
                    pushGroundLine(gx, anchorY - halfGrid, 0f, gx, anchorY + halfGrid, 0f, curGridRgb[0], curGridRgb[1], curGridRgb[2], aX)
                    pushGroundLine(anchorX - halfGrid, gy, 0f, anchorX + halfGrid, gy, 0f, curGridRgb[0], curGridRgb[1], curGridRgb[2], aY)
                }
                pushGroundLine(0f, 0f, 0.008f, 1.1f, 0f, 0.008f, 1.0f, 0.30f, 0.28f, 0.65f)
                pushGroundLine(0f, 0f, 0.008f, 0f, 1.1f, 0.008f, 0.15f, 0.92f, 0.50f, 0.65f)
                if (b > 0.05f) {
                    pushGroundLine(0f, 0f, 0.008f, 0f, 0f, 1.1f * zScale, 0.25f, 0.68f, 1.0f, 0.65f * b)
                }
            }

            cachedGridVerts = groundLineVerts
            gridKey.commit()
        }

        if (snap.swarmCount > 1) {
            for (i in 0 until snap.swarmCount) {
                val ag = snap.swarmAgents[i]
                val col = SwarmPalette.RGB_FLOAT[i]
                drawHomePadColored3D(ag.homeX, ag.homeY, col[0], col[1], col[2])
            }
        } else {
            drawHomePadColored3D(snap.homeX, snap.homeY, 1.0f, 0.76f, 0.05f)
        }

        pushGroundCircle(snap.uavX, snap.uavY, 0.012f, 2.0f, 36, 0f, 0.90f, 0.46f, 0.22f)
        pushGroundCircle(snap.uavX, snap.uavY, 0.012f, 4.0f, 44, 1f, 1f, 1f, 0.10f)
        val fovL = snap.yaw + 0.56f
        val fovR = snap.yaw - 0.56f
        pushFxTri(
            snap.uavX, snap.uavY, 0.014f,
            snap.uavX + cos(fovL) * 3.8f, snap.uavY + sin(fovL) * 3.8f, 0.014f,
            snap.uavX + cos(fovR) * 3.8f, snap.uavY + sin(fovR) * 3.8f, 0.014f,
            0f, 0.90f, 0.46f, 0.075f
        )

        if (hub.showLocalBox) {
            pushGroundDashedRect(snap.uavX - 4f, snap.uavY - 4f, snap.uavX + 4f, snap.uavY + 4f, 0.018f, 0.09f, 0.56f, 1.0f, 0.45f)
        }

        synchronized(rippleLock) {
            for (i in rippleAge.indices) {
                val age = rippleAge[i]
                if (age >= 0f) {
                    val prog = (age / 1.15f).coerceIn(0f, 1f)
                    val rad = 0.22f + prog * 1.48f
                    val alpha = (1f - prog) * 0.82f
                    pushFxRingXY(rippleX[i], rippleY[i], 0.025f, rad, 0.045f, 28, 0f, 0.88f, 1.0f, alpha)
                }
            }
        }

        var droneBlockedThisFrame = false
        val xrayActive = (b > 0.12f && hub.xrayOcclusionEnabled)

        // 前方轨迹/预览选点第二视线锚点
        val hasAheadTrajProbe = snap.bsplineCount > 6
        val probeIdx = ((snap.bsplineCount * 0.38f).toInt()).coerceIn(0, max(0, snap.bsplineCount - 1)) * 3
        curAheadX = when {
            snap.hasPreviewPin -> snap.previewX
            hasAheadTrajProbe -> snap.bsplinePts[probeIdx]
            else -> snap.uavX
        }
        curAheadY = when {
            snap.hasPreviewPin -> snap.previewY
            hasAheadTrajProbe -> snap.bsplinePts[probeIdx + 1]
            else -> snap.uavY
        }
        curAheadZ = when {
            snap.hasPreviewPin -> snap.previewZ * zScale
            hasAheadTrajProbe -> snap.bsplinePts[probeIdx + 2] * zScale
            else -> uavScaledZ
        }

        curSegDroneX = snap.uavX - curEyeX
        curSegDroneY = snap.uavY - curEyeY
        curSegDroneZ = uavScaledZ - curEyeZ
        val s1Len2 = curSegDroneX * curSegDroneX + curSegDroneY * curSegDroneY + curSegDroneZ * curSegDroneZ
        curInvDroneLen2 = if (s1Len2 > 0.15f) 1.0f / s1Len2 else 0f

        curSegAheadX = curAheadX - curEyeX
        curSegAheadY = curAheadY - curEyeY
        curSegAheadZ = curAheadZ - curEyeZ
        val s2Len2 = curSegAheadX * curSegAheadX + curSegAheadY * curSegAheadY + curSegAheadZ * curSegAheadZ
        curInvAheadLen2 = if ((hasAheadTrajProbe || snap.hasPreviewPin) && s2Len2 > 0.15f) 1.0f / s2Len2 else 0f

        // 随相机视野平滑自适应的透镜内外半径：确保整面挡视野障碍墙开出圆润开阔的观察窗，绝不留锯齿犬牙
        val spanFactor = (worldHalfSpan * 0.06f).coerceIn(0.25f, 0.85f)
        curCutInnerR = 0.58f + spanFactor * 0.45f
        curCutOuterR = 1.55f + spanFactor * 0.85f

        voxelKey.begin()
        voxelKey.add(snap.voxels, snap.voxelCount * 4)
        voxelKey.add(if (hub.showInflate) 1f else 0f)
        voxelKey.add(hub.zSliceMax)
        voxelKey.add(hub.perfMode.toFloat())
        voxelKey.add(if (xrayActive) 1f else 0f)
        voxelKey.add(b)
        voxelKey.add(worldHalfSpan)
        voxelKey.add(camFocusX)
        voxelKey.add(camFocusY)
        voxelKey.add(curEyeX)
        voxelKey.add(curEyeY)
        voxelKey.add(curEyeZ)
        voxelKey.add(curSegDroneX)
        voxelKey.add(curSegDroneY)
        voxelKey.add(curSegDroneZ)
        voxelKey.add(curInvDroneLen2)
        voxelKey.add(curSegAheadX)
        voxelKey.add(curSegAheadY)
        voxelKey.add(curSegAheadZ)
        voxelKey.add(curInvAheadLen2)
        voxelKey.add(curCutOuterR)
        voxelKey.add(vpMatrix)
        voxelKey.add(fxAlphaTriVerts.toFloat())
        voxelCacheHit = geometryCachingEnabled && voxelKey.unchanged() && !previousHadCore
        previousHadCore = snap.obsPrimitiveCount > 0
        if (voxelCacheHit) {
            solidObsTriVerts = cachedVoxelCounts[0]
            solidObsEdgeVerts = cachedVoxelCounts[1]
            glassObsTriVerts = cachedVoxelCounts[2]
            glassObsEdgeVerts = cachedVoxelCounts[3]
            cachedVoxelShadows.copyInto(fxAlphaTriArray, fxAlphaTriVerts * FLOATS_PER_VERT, 0, cachedVoxelShadowVerts * FLOATS_PER_VERT)
            fxAlphaTriVerts += cachedVoxelShadowVerts
            droneBlockedThisFrame = cachedDroneBlocked
        } else {
            val fxStart = fxAlphaTriVerts
            // 2. EGO-Planner 三维膨胀体素地图：分离纯不透明远景批 (`solidObs`) 与外皮透镜消隐批 (`glassObs`)
            val vCnt = snap.voxelCount
            if (hub.showInflate && vCnt > 0) {
                val cullSpan = worldHalfSpan * 2.9f + 4.5f
                val vArr = snap.voxels
                val zMax = hub.zSliceMax
                val pMode = hub.perfMode
                val lodNearSq = if (pMode == 2) 49.0f else 90.25f
                val maxVoxBudget = when (pMode) {
                    1 -> 4096
                    2 -> 1400
                    else -> 2800
                }
                val farStride = if (vCnt > maxVoxBudget) max(1, (vCnt + maxVoxBudget - 1) / maxVoxBudget) else 1

                val m0 = vpMatrix[0]
                val m4 = vpMatrix[4]
                val m8 = vpMatrix[8]
                val m12 = vpMatrix[12]
                val m1 = vpMatrix[1]
                val m5 = vpMatrix[5]
                val m9 = vpMatrix[9]
                val m13 = vpMatrix[13]
                val m3 = vpMatrix[3]
                val m7 = vpMatrix[7]
                val m11 = vpMatrix[11]
                val m15 = vpMatrix[15]

                // 包围盒半径略大于 curCutOuterR，使跨边界体素全部进入逐像素平滑着色器，边界 100% 连续无接缝
                val classifyOuterR = curCutOuterR + 0.32f
                val classifyOuterRSq = classifyOuterR * classifyOuterR

                for (i in 0 until vCnt) {
                    val idx = i * 4
                    val signedPacked = vArr[idx + 3]
                    val inLocal = signedPacked > 0f
                    if (!inLocal && farStride > 1 && (i % farStride != 0)) continue

                    val vx = vArr[idx]
                    val vy = vArr[idx + 1]
                    if (abs(vx - camFocusX) > cullSpan || abs(vy - camFocusY) > cullSpan) continue

                    val vzRaw = vArr[idx + 2]
                    val mag = abs(signedPacked)
                    val isTopCap = mag >= 5.0f
                    val hw = (if (isTopCap) mag - 10.0f else mag).coerceIn(0.04f, 0.20f)
                    val vzCenter = vzRaw * zScale

                    val wClip = m3 * vx + m7 * vy + m11 * vzCenter + m15
                    val padClip = hw * 3.2f + 0.48f
                    if (wClip < -padClip) continue
                    val xClip = m0 * vx + m4 * vy + m8 * vzCenter + m12
                    if (abs(xClip) > wClip + padClip) continue
                    val yClip = m1 * vx + m5 * vy + m9 * vzCenter + m13
                    if (abs(yClip) > wClip + padClip) continue

                    val hh = hw * zScale
                    val z0 = max(0.006f, vzCenter - hh)
                    val z1 = vzCenter + hh
                    val dxFoc = vx - camFocusX
                    val dyFoc = vy - camFocusY
                    val dFocusSq = dxFoc * dxFoc + dyFoc * dyFoc

                    // 底层接触阴影 (v3.1 距离自适应 LOD：仅对近景 <=9.5m 贴地体素生成阴影四边形，削减远景透明混合开销)
                    if (b > 0.12f && dFocusSq <= lodNearSq && vzRaw <= hw * 2.3f) {
                        val sh = hw * 1.32f
                        pushFxQuad(
                            vx - sh, vy - sh, 0.004f,
                            vx + sh, vy - sh, 0.004f,
                            vx + sh, vy + sh, 0.004f,
                            vx - sh, vy + sh, 0.004f,
                            0f, 0f, 0f, 0.20f * b
                        )
                    }

                    computeInflateVoxelColor(vzRaw, zMax, inLocal, tmpVoxelRgb)

                    var intoGlass = false
                    if (xrayActive) {
                        val dDroneSq = distanceSqToCapsuleSeg(vx, vy, vzCenter, curSegDroneX, curSegDroneY, curSegDroneZ, curInvDroneLen2)
                        val blockR = hw * 1.75f + 0.18f
                        if (dDroneSq < blockR * blockR) {
                            droneBlockedThisFrame = true
                        }
                        val dAheadScaledSq = if (curInvAheadLen2 > 0f) {
                            distanceSqToCapsuleSeg(vx, vy, vzCenter, curSegAheadX, curSegAheadY, curSegAheadZ, curInvAheadLen2) * 1.6384f
                        } else {
                            9999f
                        }

                        val dxEye = vx - curEyeX
                        val dyEye = vy - curEyeY
                        val dzEye = vzCenter - curEyeZ
                        val dEyeSq = dxEye * dxEye + dyEye * dyEye + dzEye * dzEye

                        if (min(dDroneSq, dAheadScaledSq) < classifyOuterRSq || dEyeSq < 6.25f) {
                            intoGlass = true
                        }
                    }

                    pushInflatedVoxel3Face(
                        vx, vy, z0, z1, hw,
                        tmpVoxelRgb[0], tmpVoxelRgb[1], tmpVoxelRgb[2],
                        intoGlass, isTopCap && (dFocusSq <= lodNearSq * 1.25f), b
                    )
                }
            }

            cachedVoxelCounts[0] = solidObsTriVerts
            cachedVoxelCounts[1] = solidObsEdgeVerts
            cachedVoxelCounts[2] = glassObsTriVerts
            cachedVoxelCounts[3] = glassObsEdgeVerts
            cachedVoxelShadowVerts = fxAlphaTriVerts - fxStart
            fxAlphaTriArray.copyInto(cachedVoxelShadows, 0, fxStart * FLOATS_PER_VERT, fxAlphaTriVerts * FLOATS_PER_VERT)
            cachedDroneBlocked = droneBlockedThisFrame
            voxelKey.commit()
        }

        // 可选几何内芯参考
        val kAlpha = 1f - exp(-dt * 10.5f)
        val obCnt = snap.obsPrimitiveCount
        val obArr = snap.obsPrimitives
        for (i in 0 until obCnt) {
            val base = i * 6
            val cx = obArr[base]
            val cy = obArr[base + 1]
            val rx = obArr[base + 2]
            val ry = obArr[base + 3]
            val h = obArr[base + 4] * zScale
            val isCyl = obArr[base + 5] > 0.5f

            val dSight = if (xrayActive) {
                distanceFromSightRayToColumn(cx, cy, rx, ry, 0f, h, isCyl, snap.uavX, snap.uavY, uavScaledZ)
            } else {
                99f
            }
            if (dSight < 0.25f) droneBlockedThisFrame = true

            val targetAlpha = if (dSight < 1.15f) 0.35f else 1.0f
            obsSmoothAlpha[i] += (targetAlpha - obsSmoothAlpha[i]) * kAlpha
            val intoGlassPass = obsSmoothAlpha[i] < 0.92f
            if (isCyl) {
                pushLit3DCylinder(cx, cy, rx, h, 18, 1.0f, intoGlassPass, b)
            } else {
                pushLit3DBox(cx, cy, rx, ry, 0f, h, 1.0f, intoGlassPass, b, drawHeightTicks = true)
            }
        }

        hub.droneCurrentlyOccluded = droneBlockedThisFrame
        val targetOccBlend = if (droneBlockedThisFrame) 1f else 0f
        droneOccludedBlend += (targetOccBlend - droneOccludedBlend) * (1f - exp(-dt * 9.0f))

        // 3. 历史航迹与 FAST-LIO 全局路径
        if (hub.showFlightTrail && snap.lioPathCount > 1) {
            pushActorRibbon3D(snap.lioPathPts, snap.lioPathCount, zScale, 0.036f, 0.16f, 0.76f, 1.0f, 0.90f)
        }
        if (hub.showFlightTrail && snap.trailCount > 1) {
            pushActorPolyline3D(snap.trailPts, snap.trailCount, zScale, 0.80f, 0.88f, 0.98f, 0.52f)
        }
        if (snap.astarCount > 1) {
            pushActorPolyline3D(snap.astarPts, snap.astarCount, 0.02f, 0.18f, 0.65f, 1.0f, 0.60f, fixedZ = 0.025f)
        }

        // 4. EGO-Planner 三维 B 样条轨迹 + 地面投影垂幕
        if (snap.bsplineCount > 1) {
            val pts = snap.bsplinePts
            if (hub.showTrajCurtain && b > 0.08f) {
                pushAltitudeCurtain(pts, snap.bsplineCount, zScale, 0.0f, 0.90f, 0.48f, 0.16f * b)
            }
            pushActorRibbon3D(pts, snap.bsplineCount, zScale, 0.054f, 0f, 0.94f, 0.50f, 0.98f)

            val shift = (((animationTimeNanos() / 1_000_000L) / 60L) % 8L).toInt()
            var chv = 4 + shift
            while (chv < snap.bsplineCount - 2) {
                val bp = (chv - 1) * 3
                val bn = (chv + 1) * 3
                val x0 = pts[bp]
                val y0 = pts[bp + 1]
                val x1 = pts[bn]
                val y1 = pts[bn + 1]
                val z1 = pts[bn + 2] * zScale + 0.018f
                val dx = x1 - x0
                val dy = y1 - y0
                val len = hypot(dx, dy)
                if (len > 1e-4f) {
                    val ux = dx / len
                    val uy = dy / len
                    val px = -uy
                    val py = ux
                    val back = 0.16f
                    val wing = 0.12f
                    pushActorQuadSeg3D(x1 - ux * back + px * wing, y1 - uy * back + py * wing, z1, x1, y1, z1, 0.020f, 1f, 1f, 1f, 0.96f, -1f, -1f)
                    pushActorQuadSeg3D(x1 - ux * back - px * wing, y1 - uy * back - py * wing, z1, x1, y1, z1, 0.020f, 1f, 1f, 1f, 0.96f, -1f, -1f)
                }
                chv += 10
            }
        }

        if (snap.ctrlCount > 0) {
            for (c in 0 until snap.ctrlCount) {
                val bi = c * 3
                pushActorDiscXY(snap.ctrlPts[bi], snap.ctrlPts[bi + 1], snap.ctrlPts[bi + 2] * zScale + 0.02f, 0.050f, 8, 1f, 1f, 1f, 0.92f, -1f)
            }
        }

        // 5. 当前目标点三维立柱
        val gz = snap.goalZ * zScale
        if (b > 0.05f) {
            pushActorQuadSeg3D(snap.goalX, snap.goalY, 0.01f, snap.goalX, snap.goalY, gz, 0.022f, 0f, 0.92f, 0.48f, 0.88f, 0f, gz)
        }
        val pulseR = 0.24f + (sin((animationTimeNanos() / 1_000_000L) * 0.006).toFloat() + 1f) * 0.09f
        pushFxRingXY(snap.goalX, snap.goalY, 0.02f, pulseR, 0.035f, 20, 0f, 0.90f, 0.46f, 0.88f)
        val bob = sin(shaderClockSec * 3.4f) * 0.028f * b
        pushActorBeaconBox(snap.goalX, snap.goalY, gz + bob, 0.105f, 0f, 0.94f, 0.50f, 0.96f)

        // 6. 连续多航点航线链
        var lastRouteTipX = snap.uavX
        var lastRouteTipY = snap.uavY
        var lastRouteTipZ = uavScaledZ
        if (snap.waypointQueueCount > 0) {
            val isStaged = snap.isRouteStagedOnly
            var prevX = if (isStaged) snap.uavX else snap.goalX
            var prevY = if (isStaged) snap.uavY else snap.goalY
            var prevZ = if (isStaged) uavScaledZ else gz

            val lr = if (isStaged) 1.0f else 0.41f
            val lg = if (isStaged) 0.78f else 0.75f
            val lb = if (isStaged) 0.14f else 1.0f

            var cumRouteArc = 0f
            val wps = snap.waypointQueuePts
            for (w in 0 until snap.waypointQueueCount) {
                val wx = wps[w * 3]
                val wy = wps[w * 3 + 1]
                val wz = wps[w * 3 + 2] * zScale
                val segLen = hypot(hypot(wx - prevX, wy - prevY), wz - prevZ)
                pushActorQuadSeg3D(prevX, prevY, prevZ, wx, wy, wz, 0.030f, lr, lg, lb, 0.90f, cumRouteArc, cumRouteArc + segLen)
                cumRouteArc += segLen

                if (b > 0.05f) {
                    pushActorLine(wx, wy, 0.01f, wx, wy, wz, lr, lg, lb, 0.75f, 0f, wz)
                }
                pushFxRingXY(wx, wy, 0.02f, 0.21f, 0.032f, 18, lr, lg, lb, 0.85f)
                pushActorBeaconBox(wx, wy, wz, 0.092f, lr, lg, lb, 0.95f)
                prevX = wx
                prevY = wy
                prevZ = wz
            }
            if (snap.routeClosedLoop && snap.waypointQueueCount >= 2) {
                val fx = wps[0]
                val fy = wps[1]
                val fz = wps[2] * zScale
                val closeLen = hypot(hypot(fx - prevX, fy - prevY), fz - prevZ)
                pushActorLine(prevX, prevY, prevZ, fx, fy, fz, lr, lg, lb, 0.58f, cumRouteArc, cumRouteArc + closeLen)
            }
            if (isStaged) {
                lastRouteTipX = prevX
                lastRouteTipY = prevY
                lastRouteTipZ = prevZ
            }
        }

        // 7. 三维选点预览光柱 (全队编队模式下同步渲染全部机位的分色几何编队落位预览柱)
        if (snap.hasPreviewPin) {
            val animS = pinAnimScale
            val px = snap.previewX
            val py = snap.previewY
            val pz = snap.previewZ * zScale * animS.coerceAtLeast(0.1f)
            val pr = if (snap.previewSafe) 0.0f else 1.0f
            val pg = if (snap.previewSafe) 0.88f else 0.30f
            val pb = if (snap.previewSafe) 1.0f else 0.26f

            val linkLen = hypot(hypot(px - lastRouteTipX, py - lastRouteTipY), pz - lastRouteTipZ)
            pushActorQuadSeg3D(lastRouteTipX, lastRouteTipY, lastRouteTipZ, px, py, pz, 0.026f, pr, pg, pb, 0.88f, 0f, linkLen)

            if (b > 0.04f) {
                pushActorQuadSeg3D(px, py, 0.01f, px, py, pz, 0.028f, pr, pg, pb, 0.92f, 0f, pz)
                for (step in 1..3) {
                    val f = step / 4f
                    val rx = lastRouteTipX + (px - lastRouteTipX) * f
                    val ry = lastRouteTipY + (py - lastRouteTipY) * f
                    val rz = lastRouteTipZ + (pz - lastRouteTipZ) * f
                    pushGroundLine(rx, ry, 0.01f, rx, ry, rz, pr, pg, pb, 0.24f * b)
                }
            }
            val pinPulse = (0.26f + (sin((animationTimeNanos() / 1_000_000L) * 0.009).toFloat() + 1f) * 0.11f) * animS.coerceIn(0.4f, 1.2f)
            pushFxRingXY(px, py, 0.022f, pinPulse, 0.040f, 24, pr, pg, pb, 0.92f)
            pushGroundLine(px - 0.42f, py, 0.022f, px + 0.42f, py, 0.022f, pr, pg, pb, 0.78f)
            pushGroundLine(px, py - 0.42f, 0.022f, px, py + 0.42f, 0.022f, pr, pg, pb, 0.78f)
            pushActorBeaconBox(px, py, pz, 0.115f * animS.coerceIn(0.5f, 1.15f), pr, pg, pb, 0.96f)

            if (snap.swarmCount > 1 && snap.swarmCommandAll) {
                for (i in 0 until snap.swarmCount) {
                    val ag = snap.swarmAgents[i]
                    val col = SwarmPalette.RGB_FLOAT[i]
                    val sx = ag.previewSlotX
                    val sy = ag.previewSlotY
                    val sz = ag.previewSlotZ * zScale * animS.coerceAtLeast(0.1f)
                    val sr = if (ag.previewSlotSafe) col[0] else 1.0f
                    val sg = if (ag.previewSlotSafe) col[1] else 0.26f
                    val sb = if (ag.previewSlotSafe) col[2] else 0.24f
                    val slotDist = hypot(sx - px, sy - py)
                    pushGroundDashedSeg(px, py, 0.024f, sx, sy, 0.024f, sr, sg, sb, 0.72f)
                    if (b > 0.04f) {
                        pushActorLine(sx, sy, 0.015f, sx, sy, sz, sr, sg, sb, 0.85f, 0f, sz)
                    }
                    pushFxRingXY(sx, sy, 0.024f, 0.23f, 0.034f, 18, sr, sg, sb, 0.90f)
                    if (slotDist > 0.08f) {
                        pushActorBeaconBox(sx, sy, sz, 0.082f, sr, sg, sb, 0.92f)
                    }
                }
            }
        }

        // 7.5 (v3.1) 当前焦点机三维净空安全光环 + 动力学制动停止点预测线 (辅助林下高速穿缝提前预判刹停裕度)
        val warnClr = hub.warnClearanceMeters.coerceIn(0.35f, 1.80f)
        val curClr = snap.minClearance
        if (curClr < warnClr * 1.35f) {
            val danger = curClr < warnClr * 0.72f
            val hr = if (danger) 1.0f else 1.0f
            val hg = if (danger) 0.24f else 0.72f
            val hb = if (danger) 0.20f else 0.08f
            val haloR = curClr.coerceIn(0.30f, 1.65f)
            pushFxRingXY(snap.uavX, snap.uavY, uavScaledZ, haloR, 0.024f, 26, hr, hg, hb, if (danger) 0.72f else 0.45f)
        }
        val hSpd3D = hypot(snap.vx, snap.vy)
        if (hSpd3D > 0.28f) {
            val maxDecel = max(1.2f, hub.maxAcc * 0.85f)
            val stopDist = (hSpd3D * hSpd3D) / (2.0f * maxDecel)
            val invSpd = 1.0f / hSpd3D
            val stopX = snap.uavX + snap.vx * invSpd * stopDist
            val stopY = snap.uavY + snap.vy * invSpd * stopDist
            val stopZ = (snap.uavZ + snap.vz * (hSpd3D / maxDecel) * 0.5f).coerceAtLeast(0.15f) * zScale
            pushActorLine(snap.uavX, snap.uavY, uavScaledZ, stopX, stopY, stopZ, 0.22f, 0.92f, 1.0f, 0.76f, 0f, stopDist)
            pushActorDiscXY(stopX, stopY, stopZ, 0.058f, 10, 0.22f, 0.92f, 1.0f, 0.88f, -1f)
        }

        // 8. 多机集群分色轨迹、机间安全拓扑连线与三维四旋翼模型
        if (snap.swarmCount > 1) {
            renderSwarmFleet3D(snap, zScale, b)
        } else {
            drawQuadrotor3D(
                snap.uavX, snap.uavY, snap.uavZ * zScale,
                snap.yaw, snap.smoothRoll * b, snap.smoothPitch * b,
                snap.armed, zScale, b,
                armR = 0.88f, armG = 0.92f, armB = 0.98f,
                isSelectedFocus = false,
                isTiltRig = snap.activeIsTiltRig,
                servo1Rad = snap.tiltServo1Rad * b,
                servo2Rad = snap.tiltServo2Rad * b
            )
        }
    }

    /**
     * 高立体感三面朝向剔除膨胀体素构建 (带高程底部 AO 渐变 + 仅在真实柱顶绘制微倒角棱线，杜绝内部多层乱线！)
     */
    private fun pushInflatedVoxel3Face(
        cx: Float,
        cy: Float,
        z0: Float,
        z1: Float,
        hw: Float,
        baseR: Float,
        baseG: Float,
        baseB: Float,
        intoGlass: Boolean,
        isTopCap: Boolean,
        blend3D: Float
    ) {
        val x0 = cx - hw
        val x1 = cx + hw
        val y0 = cy - hw
        val y1 = cy + hw
        val cz = (z0 + z1) * 0.5f

        // 底部环境光遮蔽 (Ground AO)：越靠近地面阴影越深，堆叠体素呈现扎实的三维建筑体量感
        val botAo = (0.58f + 0.34f * (z0 / 1.4f).coerceIn(0f, 1f))
        val topAo = (0.76f + 0.24f * (z1 / 1.4f).coerceIn(0f, 1f))

        // 1. Z 轴朝向面 (顶面最亮，底面背光)
        if (curEyeZ >= cz) {
            pushObsQuadFlat(
                intoGlass,
                x0, y0, z1, x1, y0, z1, x1, y1, z1, x0, y1, z1,
                min(1f, baseR * 1.12f), min(1f, baseG * 1.12f), min(1f, baseB * 1.08f), 1.0f
            )
        } else {
            pushObsQuadFlat(
                intoGlass,
                x0, y1, z0, x1, y1, z0, x1, y0, z0, x0, y0, z0,
                baseR * 0.42f, baseG * 0.42f, baseB * 0.46f, 1.0f
            )
        }

        if (blend3D > 0.04f && z1 - z0 > 0.015f) {
            // 2. X 轴朝向面 (带上下垂直明暗渐变)
            if (curEyeX >= cx) {
                pushObsWallGrad(
                    intoGlass,
                    x1, y0, z0, x1, y1, z0, x1, y1, z1, x1, y0, z1,
                    baseR * 0.84f, baseG * 0.84f, baseB * 0.86f, botAo, topAo
                )
            } else {
                pushObsWallGrad(
                    intoGlass,
                    x0, y1, z0, x0, y0, z0, x0, y0, z1, x0, y1, z1,
                    baseR * 0.60f, baseG * 0.60f, baseB * 0.64f, botAo, topAo
                )
            }

            // 3. Y 轴朝向面 (向阳面与背阴面对比分明，强化三维立体转折)
            if (curEyeY <= cy) {
                pushObsWallGrad(
                    intoGlass,
                    x0, y0, z0, x1, y0, z0, x1, y0, z1, x0, y0, z1,
                    baseR * 0.96f, baseG * 0.96f, baseB * 0.96f, botAo, topAo
                )
            } else {
                pushObsWallGrad(
                    intoGlass,
                    x1, y1, z0, x0, y1, z0, x0, y1, z1, x1, y1, z1,
                    baseR * 0.46f, baseG * 0.46f, baseB * 0.52f, botAo, topAo
                )
            }
        }

        // 仅在暴露柱顶 (isTopCap) 绘制精致微棱线：彻底杜绝 10 层内部线框重叠成一团乱麻！
        if (isTopCap) {
            val zRim = z1 + 0.0015f
            if (intoGlass) {
                pushObsEdge(true, x0, y0, zRim, x1, y0, zRim, 0.30f, 0.92f, 1.0f, 0.24f)
                pushObsEdge(true, x1, y0, zRim, x1, y1, zRim, 0.30f, 0.92f, 1.0f, 0.24f)
                pushObsEdge(true, x1, y1, zRim, x0, y1, zRim, 0.30f, 0.92f, 1.0f, 0.24f)
                pushObsEdge(true, x0, y1, zRim, x0, y0, zRim, 0.30f, 0.92f, 1.0f, 0.24f)
            } else {
                val er = min(1f, baseR * 0.42f)
                val eg = min(1f, baseG * 0.42f)
                val eb = min(1f, baseB * 0.46f)
                pushObsEdge(false, x0, y0, zRim, x1, y0, zRim, er, eg, eb, 0.36f)
                pushObsEdge(false, x1, y0, zRim, x1, y1, zRim, er, eg, eb, 0.36f)
                pushObsEdge(false, x1, y1, zRim, x0, y1, zRim, er, eg, eb, 0.36f)
                pushObsEdge(false, x0, y1, zRim, x0, y0, zRim, er, eg, eb, 0.36f)
            }
        }
    }

    private fun pushObsWallGrad(
        intoGlass: Boolean,
        bx0: Float,
        by0: Float,
        bz0: Float,
        bx1: Float,
        by1: Float,
        bz1: Float,
        tx1: Float,
        ty1: Float,
        tz1: Float,
        tx0: Float,
        ty0: Float,
        tz0: Float,
        r: Float,
        g: Float,
        b: Float,
        botMul: Float,
        topMul: Float
    ) {
        val br = r * botMul
        val bg = g * botMul
        val bb = b * botMul
        val tr = min(1f, r * topMul)
        val tg = min(1f, g * topMul)
        val tb = min(1f, b * topMul)
        if (intoGlass) {
            if (glassObsTriVerts + 6 > MAX_GLASS_TRI_VERTS) return
            pushVert(glassObsTriArray, glassObsTriVerts++, bx0, by0, bz0, br, bg, bb, 1f, 0f)
            pushVert(glassObsTriArray, glassObsTriVerts++, bx1, by1, bz1, br, bg, bb, 1f, 0f)
            pushVert(glassObsTriArray, glassObsTriVerts++, tx1, ty1, tz1, tr, tg, tb, 1f, 0f)
            pushVert(glassObsTriArray, glassObsTriVerts++, bx0, by0, bz0, br, bg, bb, 1f, 0f)
            pushVert(glassObsTriArray, glassObsTriVerts++, tx1, ty1, tz1, tr, tg, tb, 1f, 0f)
            pushVert(glassObsTriArray, glassObsTriVerts++, tx0, ty0, tz0, tr, tg, tb, 1f, 0f)
        } else {
            if (solidObsTriVerts + 6 > MAX_ENV_TRI_VERTS) return
            pushVert(solidObsTriArray, solidObsTriVerts++, bx0, by0, bz0, br, bg, bb, 1f, 0f)
            pushVert(solidObsTriArray, solidObsTriVerts++, bx1, by1, bz1, br, bg, bb, 1f, 0f)
            pushVert(solidObsTriArray, solidObsTriVerts++, tx1, ty1, tz1, tr, tg, tb, 1f, 0f)
            pushVert(solidObsTriArray, solidObsTriVerts++, bx0, by0, bz0, br, bg, bb, 1f, 0f)
            pushVert(solidObsTriArray, solidObsTriVerts++, tx1, ty1, tz1, tr, tg, tb, 1f, 0f)
            pushVert(solidObsTriArray, solidObsTriVerts++, tx0, ty0, tz0, tr, tg, tb, 1f, 0f)
        }
    }

    private fun pushLit3DCylinder(
        cx: Float,
        cy: Float,
        rad: Float,
        height: Float,
        segs: Int,
        alpha: Float,
        intoGlass: Boolean,
        blend3D: Float
    ) {
        val baseR = 0.19f
        val baseG = 0.44f
        val baseB = 0.68f
        val roofR = (baseR * 1.32f).coerceAtMost(1f)
        val roofG = (baseG * 1.32f).coerceAtMost(1f)
        val roofB = (baseB * 1.28f).coerceAtMost(1f)

        var c0 = 1f
        var s0 = 0f
        for (s in 1..segs) {
            val lutIdx = (s * LUT_N) / segs
            val c1 = COS_LUT[lutIdx]
            val s1 = SIN_LUT[lutIdx]
            pushObsTri(intoGlass, cx, cy, height, cx + c0 * rad, cy + s0 * rad, height, cx + c1 * rad, cy + s1 * rad, height, roofR, roofG, roofB, alpha)
            if (blend3D > 0.04f && height > 0.03f) {
                val nx = (c0 + c1) * 0.5f
                val ny = (s0 + s1) * 0.5f
                val shade = (0.64f + 0.36f * (nx * 0.48f - ny * 0.62f)).coerceIn(0.32f, 1.05f)
                pushObsWallGrad(
                    intoGlass,
                    cx + c0 * rad, cy + s0 * rad, 0f,
                    cx + c1 * rad, cy + s1 * rad, 0f,
                    cx + c1 * rad, cy + s1 * rad, height,
                    cx + c0 * rad, cy + s0 * rad, height,
                    baseR * shade, baseG * shade, baseB * shade, 0.56f, 1.0f
                )
            }
            c0 = c1
            s0 = s1
        }
    }

    private fun pushLit3DBox(
        cx: Float,
        cy: Float,
        rx: Float,
        ry: Float,
        z0: Float,
        z1: Float,
        alpha: Float,
        intoGlass: Boolean,
        blend3D: Float,
        drawHeightTicks: Boolean
    ) {
        val x0 = cx - rx
        val x1 = cx + rx
        val y0 = cy - ry
        val y1 = cy + ry
        val baseR = 0.18f
        val baseG = 0.42f
        val baseB = 0.66f

        pushObsQuadFlat(intoGlass, x0, y0, z1, x1, y0, z1, x1, y1, z1, x0, y1, z1, min(1f, baseR * 1.35f), min(1f, baseG * 1.35f), min(1f, baseB * 1.30f), alpha)
        if (blend3D > 0.04f && z1 - z0 > 0.03f) {
            pushObsWallGrad(intoGlass, x0, y0, z0, x1, y0, z0, x1, y0, z1, x0, y0, z1, baseR * 0.98f, baseG * 0.98f, baseB * 0.98f, 0.56f, 1.0f)
            pushObsWallGrad(intoGlass, x1, y0, z0, x1, y1, z0, x1, y1, z1, x1, y0, z1, baseR * 0.84f, baseG * 0.84f, baseB * 0.84f, 0.56f, 1.0f)
            pushObsWallGrad(intoGlass, x0, y1, z0, x0, y0, z0, x0, y0, z1, x0, y1, z1, baseR * 0.56f, baseG * 0.56f, baseB * 0.58f, 0.56f, 1.0f)
            pushObsWallGrad(intoGlass, x1, y1, z0, x0, y1, z0, x0, y1, z1, x1, y1, z1, baseR * 0.44f, baseG * 0.44f, baseB * 0.48f, 0.56f, 1.0f)
            if (drawHeightTicks) {
                val edgeA = if (intoGlass) 0.28f else 0.42f
                pushObsEdge(intoGlass, x0, y0, z0, x0, y0, z1, 0.45f, 0.88f, 1.0f, edgeA)
                pushObsEdge(intoGlass, x1, y0, z0, x1, y0, z1, 0.45f, 0.88f, 1.0f, edgeA)
                pushObsEdge(intoGlass, x1, y1, z0, x1, y1, z1, 0.45f, 0.88f, 1.0f, edgeA)
                pushObsEdge(intoGlass, x0, y1, z0, x0, y1, z1, 0.45f, 0.88f, 1.0f, edgeA)
            }
        }
    }

    private fun pushAltitudeCurtain(pts: FloatArray, count: Int, zScale: Float, r: Float, g: Float, b: Float, topAlpha: Float) {
        for (i in 0 until count - 1) {
            val aIdx = i * 3
            val bIdx = (i + 1) * 3
            val x0 = pts[aIdx]
            val y0 = pts[aIdx + 1]
            val z0 = pts[aIdx + 2] * zScale
            val x1 = pts[bIdx]
            val y1 = pts[bIdx + 1]
            val z1 = pts[bIdx + 2] * zScale

            pushGroundLine(x0, y0, 0.010f, x1, y1, 0.010f, r, g, b, 0.28f)
            pushFxGradientQuad(
                x0, y0, 0.012f, r, g, b, 0.015f,
                x1, y1, 0.012f, r, g, b, 0.015f,
                x1, y1, z1, r, g, b, topAlpha,
                x0, y0, z0, r, g, b, topAlpha
            )
            if (i % 6 == 0) {
                pushGroundLine(x0, y0, 0.012f, x0, y0, z0, r, g, b, topAlpha * 1.45f)
            }
        }
    }

    private fun drawHomePadColored3D(hx: Float, hy: Float, cr: Float, cg: Float, cb: Float) {
        val z = 0.016f
        pushFxDiscXY(hx, hy, z, 0.32f, 20, cr, cg, cb, 0.16f)
        pushFxRingXY(hx, hy, z + 0.002f, 0.32f, 0.034f, 20, cr, cg, cb, 0.84f)
        pushActorQuadSeg3D(hx - 0.10f, hy - 0.12f, z + 0.004f, hx - 0.10f, hy + 0.12f, z + 0.004f, 0.020f, cr, cg, cb, 0.92f, -1f, -1f)
        pushActorQuadSeg3D(hx + 0.10f, hy - 0.12f, z + 0.004f, hx + 0.10f, hy + 0.12f, z + 0.004f, 0.020f, cr, cg, cb, 0.92f, -1f, -1f)
        pushActorQuadSeg3D(hx - 0.10f, hy, z + 0.004f, hx + 0.10f, hy, z + 0.004f, 0.020f, cr, cg, cb, 0.92f, -1f, -1f)
    }

    /**
     * 渲染多机编队全员：分色 B 样条避障轨迹、高程垂幕、目标光柱、机间拓扑距离网与三维四旋翼编队。
     */
    private fun renderSwarmFleet3D(snap: RenderSnapshot, zScale: Float, blend: Float) {
        val cnt = snap.swarmCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
        // 1. 机间协同防撞拓扑连线 (距离过近时自动变琥珀/红色预警)
        if (hub.showSwarmLinks) {
            for (i in 0 until cnt) {
                val ai = snap.swarmAgents[i]
                for (j in i + 1 until cnt) {
                    val aj = snap.swarmAgents[j]
                    val zi = ai.uavZ * zScale
                    val zj = aj.uavZ * zScale
                    val dist = hypot(hypot(ai.uavX - aj.uavX, ai.uavY - aj.uavY), ai.uavZ - aj.uavZ)
                    val lr: Float
                    val lg: Float
                    val lb: Float
                    val la: Float
                    when {
                        dist < 0.85f -> {
                            lr = 1.0f
                            lg = 0.24f
                            lb = 0.22f
                            la = 0.92f
                        }
                        dist < 1.25f -> {
                            lr = 1.0f
                            lg = 0.72f
                            lb = 0.08f
                            la = 0.78f
                        }
                        else -> {
                            lr = 0.28f
                            lg = 0.86f
                            lb = 1.00f
                            la = 0.42f
                        }
                    }
                    pushActorLine(ai.uavX, ai.uavY, zi, aj.uavX, aj.uavY, zj, lr, lg, lb, la, 0f, dist)
                }
            }
        }

        // 2. 僚机 (1..cnt-1) 的分色航迹、B 样条轨迹、目标柱与三维机体
        for (id in 0 until cnt) {
            val ag: SwarmAgentFrame = snap.swarmAgents[id]
            val col = SwarmPalette.RGB_FLOAT[id]
            val isFocus = (id == snap.activeDroneId)

            // 0 号机的轨迹已在上方主通道绘制，此处补充 1..cnt-1 号机的分色轨迹与目标柱
            if (id > 0) {
                if (hub.showFlightTrail && ag.trailCount > 1) {
                    pushActorPolyline3D(ag.trailPts, ag.trailCount, zScale, col[0], col[1], col[2], 0.46f)
                }
                if (ag.bsplineCount > 1) {
                    if (hub.showTrajCurtain && blend > 0.08f) {
                        pushAltitudeCurtain(ag.bsplinePts, ag.bsplineCount, zScale, col[0], col[1], col[2], 0.13f * blend)
                    }
                    pushActorRibbon3D(ag.bsplinePts, ag.bsplineCount, zScale, 0.046f, col[0], col[1], col[2], if (isFocus) 0.98f else 0.86f)
                }
                val gz = ag.goalZ * zScale
                if (blend > 0.05f) {
                    pushActorLine(ag.goalX, ag.goalY, 0.012f, ag.goalX, ag.goalY, gz, col[0], col[1], col[2], 0.76f, 0f, gz)
                }
                pushFxRingXY(ag.goalX, ag.goalY, 0.02f, 0.21f, 0.030f, 18, col[0], col[1], col[2], 0.82f)
                pushActorBeaconBox(ag.goalX, ag.goalY, gz, 0.088f, col[0], col[1], col[2], 0.92f)
            }

            // 地面分色身份光环 (选定主控机呈现双圈高亮呼吸光环)
            val ringR = if (isFocus) 0.46f else 0.30f
            pushFxRingXY(ag.uavX, ag.uavY, 0.015f, ringR, if (isFocus) 0.042f else 0.026f, 22, col[0], col[1], col[2], if (isFocus) 0.94f else 0.65f)

            drawQuadrotor3D(
                ag.uavX, ag.uavY, ag.uavZ * zScale,
                ag.yaw, ag.smoothRoll * blend, ag.smoothPitch * blend,
                ag.armed, zScale, blend,
                armR = 0.35f + col[0] * 0.65f,
                armG = 0.35f + col[1] * 0.65f,
                armB = 0.35f + col[2] * 0.65f,
                isSelectedFocus = isFocus,
                isTiltRig = ag.isTiltRig,
                servo1Rad = ag.tiltServo1Rad * blend,
                servo2Rad = ag.tiltServo2Rad * blend
            )
        }
    }

    private fun drawVectorGlbModels(snap: RenderSnapshot, zScale: Float, occluded: Boolean) {
        if (!vectorGlb.ready) return
        val blend = projBlend
        if (snap.swarmCount > 1) {
            val count = snap.swarmCount.coerceIn(2, SwarmPalette.MAX_SWARM_DRONES)
            for (id in 0 until count) {
                val drone = snap.swarmAgents[id]
                if (!drone.isTiltRig) continue
                vectorGlb.draw(
                    vpMatrix, drone.uavX, drone.uavY, drone.uavZ * zScale,
                    drone.smoothRoll * blend, drone.smoothPitch * blend, drone.yaw,
                    drone.tiltServo1Rad * blend, drone.tiltServo2Rad * blend,
                    if (id == snap.activeDroneId) 0.48f else 0.45f, occluded
                )
            }
        } else if (snap.activeIsTiltRig) {
            vectorGlb.draw(
                vpMatrix, snap.uavX, snap.uavY, snap.uavZ * zScale,
                snap.smoothRoll * blend, snap.smoothPitch * blend, snap.yaw,
                snap.tiltServo1Rad * blend, snap.tiltServo2Rad * blend, 0.45f, occluded
            )
        }
    }

    private fun drawQuadrotor3D(
        ux: Float,
        uy: Float,
        uz: Float,
        yaw: Float,
        roll: Float,
        pitch: Float,
        armed: Boolean,
        zScale: Float,
        blend: Float,
        armR: Float,
        armG: Float,
        armB: Float,
        isSelectedFocus: Boolean,
        isTiltRig: Boolean = true,
        servo1Rad: Float = 0f,
        servo2Rad: Float = 0f
    ) {
        val rig = DroneVectorModel.rigFor(isTiltRig)
        val worldScale = if (isSelectedFocus) 0.48f else 0.45f

        if (blend > 0.05f) {
            pushFxDiscXY(ux, uy, 0.008f, 0.32f, 18, 0f, 0f, 0f, 0.48f * blend)
            pushActorLine(ux, uy, 0.015f, ux, uy, uz, armR, armG, armB, 0.52f, 0f, uz)
        }

        if (isSelectedFocus) {
            pushActorRingXY(ux, uy, uz, 0.36f, 0.024f, 22, armR, armG, armB, 0.85f, -1f)
        }

        if (droneOccludedBlend > 0.03f && isSelectedFocus) {
            val sonarPhase = ((animationTimeNanos() / 1_000_000L) % 1200L) / 1200f
            val sonarR = 0.38f + sonarPhase * 0.48f
            val sonarA = (1f - sonarPhase) * 0.68f * droneOccludedBlend
            pushActorRingXY(ux, uy, uz, sonarR, 0.030f, 24, 0.15f, 0.94f, 1.0f, sonarA, -1f)
        }

        // The GLB is drawn in the same depth passes after these auxiliary rings/lines.
        if (isTiltRig && vectorGlb.ready) return

        for (face in rig.faces) {
            val pts = face.pts
            val n = pts.size / 3
            if (n < 3) continue
            DroneVectorModel.transformBodyToWorld(pts[0], pts[1], pts[2], worldScale, roll, pitch, yaw, uavTmpA)
            val x0 = ux + uavTmpA[0]
            val y0 = uy + uavTmpA[1]
            val z0 = uz + uavTmpA[2]
            for (i in 1 until n - 1) {
                val b1 = i * 3
                val b2 = (i + 1) * 3
                DroneVectorModel.transformBodyToWorld(pts[b1], pts[b1 + 1], pts[b1 + 2], worldScale, roll, pitch, yaw, uavTmpA)
                DroneVectorModel.transformBodyToWorld(pts[b2], pts[b2 + 1], pts[b2 + 2], worldScale, roll, pitch, yaw, uavTmpB)
                val sh = 0.22f * face.shadeBoost
                pushActorTri(
                    x0, y0, z0, -1f,
                    ux + uavTmpA[0], uy + uavTmpA[1], uz + uavTmpA[2], -1f,
                    ux + uavTmpB[0], uy + uavTmpB[1], uz + uavTmpB[2], -1f,
                    sh * (0.65f + armR * 0.55f), sh * (0.75f + armG * 0.55f), sh * (0.90f + armB * 0.55f), 0.98f
                )
            }
        }

        for (seg in rig.segments) {
            DroneVectorModel.transformBodyToWorld(seg.x0, seg.y0, seg.z0, worldScale, roll, pitch, yaw, uavTmpA)
            DroneVectorModel.transformBodyToWorld(seg.x1, seg.y1, seg.z1, worldScale, roll, pitch, yaw, uavTmpB)
            val x0 = ux + uavTmpA[0]
            val y0 = uy + uavTmpA[1]
            val z0 = uz + uavTmpA[2]
            val x1 = ux + uavTmpB[0]
            val y1 = uy + uavTmpB[1]
            val z1 = uz + uavTmpB[2]

            when (seg.role) {
                DroneVectorModel.PartRole.ARM_BEAM ->
                    pushActorQuadSeg3D(x0, y0, z0, x1, y1, z1, 0.022f, armR, armG, armB, 0.98f, -1f, -1f)
                DroneVectorModel.PartRole.GIMBAL_NOSE ->
                    pushActorQuadSeg3D(x0, y0, z0, x1, y1, z1, 0.024f, 0.12f, 0.68f, 1.0f, 0.98f, -1f, -1f)
                DroneVectorModel.PartRole.NAV_FRONT_RED ->
                    pushActorQuadSeg3D(x0, y0, z0, x1, y1, z1, 0.017f, 1.0f, 0.26f, 0.22f, 0.98f, -1f, -1f)
                DroneVectorModel.PartRole.NAV_REAR_GREEN ->
                    pushActorQuadSeg3D(x0, y0, z0, x1, y1, z1, 0.017f, 0.0f, 0.92f, 0.50f, 0.98f, -1f, -1f)
                DroneVectorModel.PartRole.LIDAR_DOME ->
                    pushActorLine(x0, y0, z0, x1, y1, z1, 0.35f, 0.96f, 0.88f, 0.95f, -1f, -1f)
                DroneVectorModel.PartRole.TILT_SERVO_SHAFT ->
                    pushActorQuadSeg3D(x0, y0, z0, x1, y1, z1, 0.019f, 1.0f, 0.72f, 0.05f, 0.98f, -1f, -1f)
                else ->
                    pushActorLine(x0, y0, z0, x1, y1, z1, armR, armG, armB, 0.92f, -1f, -1f)
            }
        }

        val spinPhase = if (armed) shaderClockSec * 22f else 0.4f
        for (rot in rig.rotors) {
            val tiltAlpha = if (!isTiltRig) {
                0f
            } else {
                when (rot.servoGroup) {
                    1 -> servo1Rad
                    2 -> servo2Rad
                    else -> 0f
                }
            }
            if (rot.servoGroup > 0) {
                val shroudR = rot.radius * 1.12f
                val rr = if (rot.isFront) 1.0f else 0.0f
                val rg = if (rot.isFront) 0.26f else 0.92f
                val rb = if (rot.isFront) 0.22f else 0.50f
                for (s in 0 until 10) {
                    val i0 = (s * LUT_N) / 10
                    val i1 = ((s + 1) * LUT_N) / 10
                    DroneVectorModel.transformNacellePointToWorld(
                        rot.cx, rot.cy, rot.cz,
                        COS_LUT[i0] * shroudR, SIN_LUT[i0] * shroudR, 0.018f,
                        tiltAlpha, worldScale, roll, pitch, yaw, uavTmpA
                    )
                    DroneVectorModel.transformNacellePointToWorld(
                        rot.cx, rot.cy, rot.cz,
                        COS_LUT[i1] * shroudR, SIN_LUT[i1] * shroudR, 0.018f,
                        tiltAlpha, worldScale, roll, pitch, yaw, uavTmpB
                    )
                    pushActorQuadSeg3D(
                        ux + uavTmpA[0], uy + uavTmpA[1], uz + uavTmpA[2],
                        ux + uavTmpB[0], uy + uavTmpB[1], uz + uavTmpB[2],
                        0.015f, rr, rg, rb, 0.96f, -1f, -1f
                    )
                }
                if (blend > 0.08f) {
                    DroneVectorModel.transformNacellePointToWorld(
                        rot.cx, rot.cy, rot.cz, 0f, 0f, 0.02f,
                        tiltAlpha, worldScale, roll, pitch, yaw, uavTmpA
                    )
                    DroneVectorModel.transformNacellePointToWorld(
                        rot.cx, rot.cy, rot.cz, 0f, 0f, 0.36f,
                        tiltAlpha, worldScale, roll, pitch, yaw, uavTmpB
                    )
                    val vr = if (rot.servoGroup == 1) 1.0f else 0.18f
                    val vg = if (rot.servoGroup == 1) 0.78f else 0.86f
                    val vb = if (rot.servoGroup == 1) 0.06f else 1.0f
                    pushActorLine(
                        ux + uavTmpA[0], uy + uavTmpA[1], uz + uavTmpA[2],
                        ux + uavTmpB[0], uy + uavTmpB[1], uz + uavTmpB[2],
                        vr, vg, vb, 0.88f, -1f, -1f
                    )
                }
            }
            val ang = spinPhase * rot.spinSign
            val rBlade = rot.radius * 0.88f
            DroneVectorModel.transformNacellePointToWorld(
                rot.cx, rot.cy, rot.cz, cos(ang) * rBlade, sin(ang) * rBlade, 0.028f,
                tiltAlpha, worldScale, roll, pitch, yaw, uavTmpA
            )
            DroneVectorModel.transformNacellePointToWorld(
                rot.cx, rot.cy, rot.cz, -cos(ang) * rBlade, -sin(ang) * rBlade, 0.028f,
                tiltAlpha, worldScale, roll, pitch, yaw, uavTmpB
            )
            pushActorLine(
                ux + uavTmpA[0], uy + uavTmpA[1], uz + uavTmpA[2],
                ux + uavTmpB[0], uy + uavTmpB[1], uz + uavTmpB[2],
                0.92f, 0.96f, 1.0f, 0.75f, -1f, -1f
            )
        }
    }

    // ==================== 底层零分配顶点压入工具函数 ====================

    private fun pushVert(arr: FloatArray, vertIdx: Int, x: Float, y: Float, z: Float, r: Float, g: Float, b: Float, a: Float, arc: Float) {
        var o = vertIdx * FLOATS_PER_VERT
        arr[o++] = x
        arr[o++] = y
        arr[o++] = z
        arr[o++] = r
        arr[o++] = g
        arr[o++] = b
        arr[o++] = a
        arr[o] = arc
    }

    private fun pushGroundLine(x0: Float, y0: Float, z0: Float, x1: Float, y1: Float, z1: Float, r: Float, g: Float, b: Float, a: Float) {
        if (groundLineVerts + 2 > MAX_ENV_LINE_VERTS) return
        pushVert(groundLineArray, groundLineVerts++, x0, y0, z0, r, g, b, a, 0f)
        pushVert(groundLineArray, groundLineVerts++, x1, y1, z1, r, g, b, a, 0f)
    }

    private fun pushGroundCircle(cx: Float, cy: Float, z: Float, rad: Float, segs: Int, r: Float, g: Float, b: Float, a: Float) {
        var px = cx + rad
        var py = cy
        for (s in 1..segs) {
            val lutIdx = (s * LUT_N) / segs
            val nx = cx + COS_LUT[lutIdx] * rad
            val ny = cy + SIN_LUT[lutIdx] * rad
            pushGroundLine(px, py, z, nx, ny, z, r, g, b, a)
            px = nx
            py = ny
        }
    }

    private fun pushGroundDashedRect(x0: Float, y0: Float, x1: Float, y1: Float, z: Float, r: Float, g: Float, b: Float, a: Float) {
        pushGroundDashedSeg(x0, y0, z, x1, y0, z, r, g, b, a)
        pushGroundDashedSeg(x1, y0, z, x1, y1, z, r, g, b, a)
        pushGroundDashedSeg(x1, y1, z, x0, y1, z, r, g, b, a)
        pushGroundDashedSeg(x0, y1, z, x0, y0, z, r, g, b, a)
    }

    private fun pushGroundDashedSeg(x0: Float, y0: Float, z0: Float, x1: Float, y1: Float, z1: Float, r: Float, g: Float, b: Float, a: Float) {
        val steps = 16
        for (i in 0 until steps step 2) {
            val t0 = i.toFloat() / steps
            val t1 = (i + 1).toFloat() / steps
            pushGroundLine(
                x0 + (x1 - x0) * t0, y0 + (y1 - y0) * t0, z0 + (z1 - z0) * t0,
                x0 + (x1 - x0) * t1, y0 + (y1 - y0) * t1, z0 + (z1 - z0) * t1,
                r, g, b, a
            )
        }
    }

    private fun pushObsEdge(intoGlass: Boolean, x0: Float, y0: Float, z0: Float, x1: Float, y1: Float, z1: Float, r: Float, g: Float, b: Float, a: Float) {
        if (intoGlass) {
            if (glassObsEdgeVerts + 2 > MAX_GLASS_LINE_VERTS) return
            pushVert(glassObsEdgeArray, glassObsEdgeVerts++, x0, y0, z0, r, g, b, a, 0f)
            pushVert(glassObsEdgeArray, glassObsEdgeVerts++, x1, y1, z1, r, g, b, a, 0f)
        } else {
            if (solidObsEdgeVerts + 2 > MAX_ENV_LINE_VERTS) return
            pushVert(solidObsEdgeArray, solidObsEdgeVerts++, x0, y0, z0, r, g, b, a, 0f)
            pushVert(solidObsEdgeArray, solidObsEdgeVerts++, x1, y1, z1, r, g, b, a, 0f)
        }
    }

    private fun pushObsTri(
        intoGlass: Boolean,
        x0: Float,
        y0: Float,
        z0: Float,
        x1: Float,
        y1: Float,
        z1: Float,
        x2: Float,
        y2: Float,
        z2: Float,
        r: Float,
        g: Float,
        b: Float,
        a: Float
    ) {
        if (intoGlass) {
            if (glassObsTriVerts + 3 > MAX_GLASS_TRI_VERTS) return
            pushVert(glassObsTriArray, glassObsTriVerts++, x0, y0, z0, r, g, b, a, 0f)
            pushVert(glassObsTriArray, glassObsTriVerts++, x1, y1, z1, r, g, b, a, 0f)
            pushVert(glassObsTriArray, glassObsTriVerts++, x2, y2, z2, r, g, b, a, 0f)
        } else {
            if (solidObsTriVerts + 3 > MAX_ENV_TRI_VERTS) return
            pushVert(solidObsTriArray, solidObsTriVerts++, x0, y0, z0, r, g, b, a, 0f)
            pushVert(solidObsTriArray, solidObsTriVerts++, x1, y1, z1, r, g, b, a, 0f)
            pushVert(solidObsTriArray, solidObsTriVerts++, x2, y2, z2, r, g, b, a, 0f)
        }
    }

    private fun pushObsQuadFlat(
        intoGlass: Boolean,
        x0: Float,
        y0: Float,
        z0: Float,
        x1: Float,
        y1: Float,
        z1: Float,
        x2: Float,
        y2: Float,
        z2: Float,
        x3: Float,
        y3: Float,
        z3: Float,
        r: Float,
        g: Float,
        b: Float,
        a: Float
    ) {
        pushObsTri(intoGlass, x0, y0, z0, x1, y1, z1, x2, y2, z2, r, g, b, a)
        pushObsTri(intoGlass, x0, y0, z0, x2, y2, z2, x3, y3, z3, r, g, b, a)
    }

    // ==================== 主角可遮挡批次 ====================

    private fun pushActorLine(
        x0: Float,
        y0: Float,
        z0: Float,
        x1: Float,
        y1: Float,
        z1: Float,
        r: Float,
        g: Float,
        b: Float,
        a: Float,
        arc0: Float,
        arc1: Float
    ) {
        if (actorLineVerts + 2 > MAX_ACTOR_LINE_VERTS) return
        pushVert(actorLineArray, actorLineVerts++, x0, y0, z0, r, g, b, a, arc0)
        pushVert(actorLineArray, actorLineVerts++, x1, y1, z1, r, g, b, a, arc1)
    }

    private fun pushActorPolyline3D(
        pts: FloatArray,
        count: Int,
        zScale: Float,
        r: Float,
        g: Float,
        b: Float,
        a: Float,
        fixedZ: Float = -1f
    ) {
        var cumArc = 0f
        for (i in 0 until count - 1) {
            val i0 = i * 3
            val i1 = (i + 1) * 3
            val x0 = pts[i0]
            val y0 = pts[i0 + 1]
            val z0 = if (fixedZ >= 0f) fixedZ else pts[i0 + 2] * zScale + 0.012f
            val x1 = pts[i1]
            val y1 = pts[i1 + 1]
            val z1 = if (fixedZ >= 0f) fixedZ else pts[i1 + 2] * zScale + 0.012f
            val seg = hypot(hypot(x1 - x0, y1 - y0), z1 - z0)
            pushActorLine(x0, y0, z0, x1, y1, z1, r, g, b, a, cumArc, cumArc + seg)
            cumArc += seg
        }
    }

    private fun pushActorTri(
        x0: Float,
        y0: Float,
        z0: Float,
        arc0: Float,
        x1: Float,
        y1: Float,
        z1: Float,
        arc1: Float,
        x2: Float,
        y2: Float,
        z2: Float,
        arc2: Float,
        r: Float,
        g: Float,
        b: Float,
        a: Float
    ) {
        if (actorTriVerts + 3 > MAX_ACTOR_TRI_VERTS) return
        pushVert(actorTriArray, actorTriVerts++, x0, y0, z0, r, g, b, a, arc0)
        pushVert(actorTriArray, actorTriVerts++, x1, y1, z1, r, g, b, a, arc1)
        pushVert(actorTriArray, actorTriVerts++, x2, y2, z2, r, g, b, a, arc2)
    }

    private fun pushActorQuadSeg3D(
        x0: Float,
        y0: Float,
        z0: Float,
        x1: Float,
        y1: Float,
        z1: Float,
        halfW: Float,
        r: Float,
        g: Float,
        b: Float,
        a: Float,
        arc0: Float,
        arc1: Float
    ) {
        val dx = x1 - x0
        val dy = y1 - y0
        val len = hypot(dx, dy)
        val nx: Float
        val ny: Float
        if (len > 1e-4f) {
            nx = (-dy / len) * halfW
            ny = (dx / len) * halfW
        } else {
            nx = halfW
            ny = 0f
        }
        pushActorTri(
            x0 - nx, y0 - ny, z0, arc0,
            x0 + nx, y0 + ny, z0, arc0,
            x1 + nx, y1 + ny, z1, arc1,
            r, g, b, a
        )
        pushActorTri(
            x0 - nx, y0 - ny, z0, arc0,
            x1 + nx, y1 + ny, z1, arc1,
            x1 - nx, y1 - ny, z1, arc1,
            r, g, b, a
        )
    }

    private fun pushActorRibbon3D(
        pts: FloatArray,
        count: Int,
        zScale: Float,
        halfW: Float,
        r: Float,
        g: Float,
        b: Float,
        a: Float
    ) {
        var cumArc = 0f
        for (i in 0 until count - 1) {
            val aIdx = i * 3
            val bIdx = (i + 1) * 3
            val x0 = pts[aIdx]
            val y0 = pts[aIdx + 1]
            val z0 = pts[aIdx + 2] * zScale + 0.012f
            val x1 = pts[bIdx]
            val y1 = pts[bIdx + 1]
            val z1 = pts[bIdx + 2] * zScale + 0.012f
            val seg = hypot(hypot(x1 - x0, y1 - y0), z1 - z0)
            pushActorQuadSeg3D(x0, y0, z0, x1, y1, z1, halfW, r, g, b, a, cumArc, cumArc + seg)
            cumArc += seg
        }
    }

    private fun pushActorDiscXY(cx: Float, cy: Float, z: Float, rad: Float, segs: Int, r: Float, g: Float, b: Float, a: Float, arc: Float) {
        var px = cx + rad
        var py = cy
        for (s in 1..segs) {
            val lutIdx = (s * LUT_N) / segs
            val nx = cx + COS_LUT[lutIdx] * rad
            val ny = cy + SIN_LUT[lutIdx] * rad
            pushActorTri(cx, cy, z, arc, px, py, z, arc, nx, ny, z, arc, r, g, b, a)
            px = nx
            py = ny
        }
    }

    private fun pushActorRingXY(cx: Float, cy: Float, z: Float, rad: Float, thick: Float, segs: Int, r: Float, g: Float, b: Float, a: Float, arc: Float) {
        val rIn = max(0.005f, rad - thick * 0.5f)
        val rOut = rad + thick * 0.5f
        var c0 = 1f
        var s0 = 0f
        for (s in 1..segs) {
            val lutIdx = (s * LUT_N) / segs
            val c1 = COS_LUT[lutIdx]
            val s1 = SIN_LUT[lutIdx]
            pushActorTri(cx + c0 * rIn, cy + s0 * rIn, z, arc, cx + c0 * rOut, cy + s0 * rOut, z, arc, cx + c1 * rOut, cy + s1 * rOut, z, arc, r, g, b, a)
            pushActorTri(cx + c0 * rIn, cy + s0 * rIn, z, arc, cx + c1 * rOut, cy + s1 * rOut, z, arc, cx + c1 * rIn, cy + s1 * rIn, z, arc, r, g, b, a)
            c0 = c1
            s0 = s1
        }
    }

    private fun pushActorBeaconBox(cx: Float, cy: Float, cz: Float, hw: Float, r: Float, g: Float, b: Float, a: Float) {
        val x0 = cx - hw
        val x1 = cx + hw
        val y0 = cy - hw
        val y1 = cy + hw
        val z0 = cz - hw
        val z1 = cz + hw
        fun q(ax: Float, ay: Float, az: Float, bx: Float, by: Float, bz: Float, dx: Float, dy: Float, dz: Float, ex: Float, ey: Float, ez: Float, sh: Float) {
            pushActorTri(ax, ay, az, -1f, bx, by, bz, -1f, dx, dy, dz, -1f, r * sh, g * sh, b * sh, a)
            pushActorTri(ax, ay, az, -1f, dx, dy, dz, -1f, ex, ey, ez, -1f, r * sh, g * sh, b * sh, a)
        }
        q(x0, y0, z1, x1, y0, z1, x1, y1, z1, x0, y1, z1, 1.0f)
        q(x0, y0, z0, x1, y0, z0, x1, y0, z1, x0, y0, z1, 0.88f)
        q(x1, y0, z0, x1, y1, z0, x1, y1, z1, x1, y0, z1, 0.76f)
        q(x1, y1, z0, x0, y1, z0, x0, y1, z1, x1, y1, z1, 0.62f)
        q(x0, y1, z0, x0, y0, z0, x0, y0, z1, x0, y1, z1, 0.68f)
    }

    // ==================== 半透明特效批次 ====================

    private fun pushFxTri(x0: Float, y0: Float, z0: Float, x1: Float, y1: Float, z1: Float, x2: Float, y2: Float, z2: Float, r: Float, g: Float, b: Float, a: Float) {
        if (fxAlphaTriVerts + 3 > MAX_FX_TRI_VERTS) return
        pushVert(fxAlphaTriArray, fxAlphaTriVerts++, x0, y0, z0, r, g, b, a, 0f)
        pushVert(fxAlphaTriArray, fxAlphaTriVerts++, x1, y1, z1, r, g, b, a, 0f)
        pushVert(fxAlphaTriArray, fxAlphaTriVerts++, x2, y2, z2, r, g, b, a, 0f)
    }

    private fun pushFxQuad(x0: Float, y0: Float, z0: Float, x1: Float, y1: Float, z1: Float, x2: Float, y2: Float, z2: Float, x3: Float, y3: Float, z3: Float, r: Float, g: Float, b: Float, a: Float) {
        pushFxTri(x0, y0, z0, x1, y1, z1, x2, y2, z2, r, g, b, a)
        pushFxTri(x0, y0, z0, x2, y2, z2, x3, y3, z3, r, g, b, a)
    }

    private fun pushFxGradientQuad(
        x0: Float,
        y0: Float,
        z0: Float,
        r0: Float,
        g0: Float,
        b0: Float,
        a0: Float,
        x1: Float,
        y1: Float,
        z1: Float,
        r1: Float,
        g1: Float,
        b1: Float,
        a1: Float,
        x2: Float,
        y2: Float,
        z2: Float,
        r2: Float,
        g2: Float,
        b2: Float,
        a2: Float,
        x3: Float,
        y3: Float,
        z3: Float,
        r3: Float,
        g3: Float,
        b3: Float,
        a3: Float
    ) {
        if (fxAlphaTriVerts + 6 > MAX_FX_TRI_VERTS) return
        pushVert(fxAlphaTriArray, fxAlphaTriVerts++, x0, y0, z0, r0, g0, b0, a0, 0f)
        pushVert(fxAlphaTriArray, fxAlphaTriVerts++, x1, y1, z1, r1, g1, b1, a1, 0f)
        pushVert(fxAlphaTriArray, fxAlphaTriVerts++, x2, y2, z2, r2, g2, b2, a2, 0f)
        pushVert(fxAlphaTriArray, fxAlphaTriVerts++, x0, y0, z0, r0, g0, b0, a0, 0f)
        pushVert(fxAlphaTriArray, fxAlphaTriVerts++, x2, y2, z2, r2, g2, b2, a2, 0f)
        pushVert(fxAlphaTriArray, fxAlphaTriVerts++, x3, y3, z3, r3, g3, b3, a3, 0f)
    }

    private fun pushFxDiscXY(cx: Float, cy: Float, z: Float, rad: Float, segs: Int, r: Float, g: Float, b: Float, a: Float) {
        var px = cx + rad
        var py = cy
        for (s in 1..segs) {
            val lutIdx = (s * LUT_N) / segs
            val nx = cx + COS_LUT[lutIdx] * rad
            val ny = cy + SIN_LUT[lutIdx] * rad
            pushFxTri(cx, cy, z, px, py, z, nx, ny, z, r, g, b, a)
            px = nx
            py = ny
        }
    }

    private fun pushFxRingXY(cx: Float, cy: Float, z: Float, rad: Float, thick: Float, segs: Int, r: Float, g: Float, b: Float, a: Float) {
        val rIn = max(0.005f, rad - thick * 0.5f)
        val rOut = rad + thick * 0.5f
        var c0 = 1f
        var s0 = 0f
        for (s in 1..segs) {
            val lutIdx = (s * LUT_N) / segs
            val c1 = COS_LUT[lutIdx]
            val s1 = SIN_LUT[lutIdx]
            pushFxQuad(
                cx + c0 * rIn, cy + s0 * rIn, z,
                cx + c0 * rOut, cy + s0 * rOut, z,
                cx + c1 * rOut, cy + s1 * rOut, z,
                cx + c1 * rIn, cy + s1 * rIn, z,
                r, g, b, a
            )
            c0 = c1
            s0 = s1
        }
    }

    // ==================== VBO 提交与多遍渲染工具 ====================

    private fun bindPrimUniforms() {
        GLES20.glUseProgram(progPrim)
        GLES20.glUniformMatrix4fv(uPrimMvp, 1, false, vpMatrix, 0)
        GLES20.glUniform2f(uPrimFocusXY, camFocusX, camFocusY)
        GLES20.glUniform1f(uPrimFogSpan, worldHalfSpan * 2.2f)
        GLES20.glUniform3f(uPrimFogColor, curFogRgb[0], curFogRgb[1], curFogRgb[2])
    }

    private fun flushEnvLines(vboId: Int, src: FloatArray, vertCount: Int, lineWidthPx: Float, upload: Boolean = true, sourceOffset: Int = 0) {
        if (vertCount <= 0) return
        if (upload) {
            envLineBuf.clear()
            envLineBuf.put(src, sourceOffset * FLOATS_PER_VERT, vertCount * FLOATS_PER_VERT)
            envLineBuf.position(0)
        }

        bindPrimUniforms()
        GLES20.glLineWidth(lineWidthPx)

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboId)
        if (upload) GLES20.glBufferSubData(GLES20.GL_ARRAY_BUFFER, 0, vertCount * STRIDE_VERT, envLineBuf)

        GLES20.glEnableVertexAttribArray(aPrimPos)
        GLES20.glVertexAttribPointer(aPrimPos, 3, GLES20.GL_FLOAT, false, STRIDE_VERT, 0)
        GLES20.glEnableVertexAttribArray(aPrimColor)
        GLES20.glVertexAttribPointer(aPrimColor, 4, GLES20.GL_FLOAT, false, STRIDE_VERT, 12)
        if (aPrimArc >= 0) {
            GLES20.glEnableVertexAttribArray(aPrimArc)
            GLES20.glVertexAttribPointer(aPrimArc, 1, GLES20.GL_FLOAT, false, STRIDE_VERT, 28)
        }

        GLES20.glDrawArrays(GLES20.GL_LINES, 0, vertCount)

        GLES20.glDisableVertexAttribArray(aPrimPos)
        GLES20.glDisableVertexAttribArray(aPrimColor)
        if (aPrimArc >= 0) GLES20.glDisableVertexAttribArray(aPrimArc)
    }

    private fun flushEnvTriangles(vboId: Int, src: FloatArray, vertCount: Int, upload: Boolean = true) {
        if (vertCount <= 0) return
        if (upload) {
            envTriBuf.clear()
            envTriBuf.put(src, 0, vertCount * FLOATS_PER_VERT)
            envTriBuf.position(0)
        }

        bindPrimUniforms()

        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboId)
        if (upload) GLES20.glBufferSubData(GLES20.GL_ARRAY_BUFFER, 0, vertCount * STRIDE_VERT, envTriBuf)

        GLES20.glEnableVertexAttribArray(aPrimPos)
        GLES20.glVertexAttribPointer(aPrimPos, 3, GLES20.GL_FLOAT, false, STRIDE_VERT, 0)
        GLES20.glEnableVertexAttribArray(aPrimColor)
        GLES20.glVertexAttribPointer(aPrimColor, 4, GLES20.GL_FLOAT, false, STRIDE_VERT, 12)
        if (aPrimArc >= 0) {
            GLES20.glEnableVertexAttribArray(aPrimArc)
            GLES20.glVertexAttribPointer(aPrimArc, 1, GLES20.GL_FLOAT, false, STRIDE_VERT, 28)
        }

        GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, vertCount)

        GLES20.glDisableVertexAttribArray(aPrimPos)
        GLES20.glDisableVertexAttribArray(aPrimColor)
        if (aPrimArc >= 0) GLES20.glDisableVertexAttribArray(aPrimArc)
    }

    private fun uploadActorVbos() {
        if (actorLineVerts > 0) {
            actorLineBuf.clear()
            actorLineBuf.put(actorLineArray, 0, actorLineVerts * FLOATS_PER_VERT)
            actorLineBuf.position(0)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboActorLineId)
            GLES20.glBufferSubData(GLES20.GL_ARRAY_BUFFER, 0, actorLineVerts * STRIDE_VERT, actorLineBuf)
        }
        if (actorTriVerts > 0) {
            actorTriBuf.clear()
            actorTriBuf.put(actorTriArray, 0, actorTriVerts * FLOATS_PER_VERT)
            actorTriBuf.position(0)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboActorTriId)
            GLES20.glBufferSubData(GLES20.GL_ARRAY_BUFFER, 0, actorTriVerts * STRIDE_VERT, actorTriBuf)
        }
    }

    private fun drawActorBatchesOccluded(lineWidthPx: Float) {
        GLES20.glUseProgram(progOccluded)
        GLES20.glUniformMatrix4fv(uOccMvp, 1, false, vpMatrix, 0)
        GLES20.glUniform1f(uOccTime, shaderClockSec)

        if (actorLineVerts > 0) {
            GLES20.glLineWidth(lineWidthPx)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboActorLineId)
            GLES20.glEnableVertexAttribArray(aOccPos)
            GLES20.glVertexAttribPointer(aOccPos, 3, GLES20.GL_FLOAT, false, STRIDE_VERT, 0)
            GLES20.glEnableVertexAttribArray(aOccColor)
            GLES20.glVertexAttribPointer(aOccColor, 4, GLES20.GL_FLOAT, false, STRIDE_VERT, 12)
            GLES20.glEnableVertexAttribArray(aOccArc)
            GLES20.glVertexAttribPointer(aOccArc, 1, GLES20.GL_FLOAT, false, STRIDE_VERT, 28)
            GLES20.glDrawArrays(GLES20.GL_LINES, 0, actorLineVerts)
        }

        if (actorTriVerts > 0) {
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboActorTriId)
            GLES20.glEnableVertexAttribArray(aOccPos)
            GLES20.glVertexAttribPointer(aOccPos, 3, GLES20.GL_FLOAT, false, STRIDE_VERT, 0)
            GLES20.glEnableVertexAttribArray(aOccColor)
            GLES20.glVertexAttribPointer(aOccColor, 4, GLES20.GL_FLOAT, false, STRIDE_VERT, 12)
            GLES20.glEnableVertexAttribArray(aOccArc)
            GLES20.glVertexAttribPointer(aOccArc, 1, GLES20.GL_FLOAT, false, STRIDE_VERT, 28)
            GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, actorTriVerts)
        }

        GLES20.glDisableVertexAttribArray(aOccPos)
        GLES20.glDisableVertexAttribArray(aOccColor)
        GLES20.glDisableVertexAttribArray(aOccArc)
    }

    private fun drawActorBatchesVisible(lineWidthPx: Float) {
        bindPrimUniforms()

        if (actorLineVerts > 0) {
            GLES20.glLineWidth(lineWidthPx)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboActorLineId)
            GLES20.glEnableVertexAttribArray(aPrimPos)
            GLES20.glVertexAttribPointer(aPrimPos, 3, GLES20.GL_FLOAT, false, STRIDE_VERT, 0)
            GLES20.glEnableVertexAttribArray(aPrimColor)
            GLES20.glVertexAttribPointer(aPrimColor, 4, GLES20.GL_FLOAT, false, STRIDE_VERT, 12)
            if (aPrimArc >= 0) {
                GLES20.glEnableVertexAttribArray(aPrimArc)
                GLES20.glVertexAttribPointer(aPrimArc, 1, GLES20.GL_FLOAT, false, STRIDE_VERT, 28)
            }
            GLES20.glDrawArrays(GLES20.GL_LINES, 0, actorLineVerts)
        }

        if (actorTriVerts > 0) {
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboActorTriId)
            GLES20.glEnableVertexAttribArray(aPrimPos)
            GLES20.glVertexAttribPointer(aPrimPos, 3, GLES20.GL_FLOAT, false, STRIDE_VERT, 0)
            GLES20.glEnableVertexAttribArray(aPrimColor)
            GLES20.glVertexAttribPointer(aPrimColor, 4, GLES20.GL_FLOAT, false, STRIDE_VERT, 12)
            if (aPrimArc >= 0) {
                GLES20.glEnableVertexAttribArray(aPrimArc)
                GLES20.glVertexAttribPointer(aPrimArc, 1, GLES20.GL_FLOAT, false, STRIDE_VERT, 28)
            }
            GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, actorTriVerts)
        }

        GLES20.glDisableVertexAttribArray(aPrimPos)
        GLES20.glDisableVertexAttribArray(aPrimColor)
        if (aPrimArc >= 0) GLES20.glDisableVertexAttribArray(aPrimArc)
    }

    private fun buildProgram(vsSrc: String, fsSrc: String): Int {
        val vs = compileShader(GLES20.GL_VERTEX_SHADER, vsSrc)
        val fs = compileShader(GLES20.GL_FRAGMENT_SHADER, fsSrc)
        val prog = GLES20.glCreateProgram()
        GLES20.glAttachShader(prog, vs)
        GLES20.glAttachShader(prog, fs)
        GLES20.glLinkProgram(prog)
        return prog
    }

    private fun compileShader(type: Int, src: String): Int {
        val sh = GLES20.glCreateShader(type)
        GLES20.glShaderSource(sh, src)
        GLES20.glCompileShader(sh)
        return sh
    }
}
