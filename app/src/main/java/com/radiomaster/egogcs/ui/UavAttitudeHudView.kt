package com.radiomaster.egogcs.ui

import android.animation.ValueAnimator
import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.graphics.Typeface
import android.os.SystemClock
import android.view.View
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.ui.DroneVectorModel.PartRole
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sin

/**
 * 紧凑防重叠、自适应字号零截断版右下角「无人机三维实时姿态视窗」(`UavAttitudeHudView`)：
 * 与左下角 96dp HSI 避障姿态球严格等高底边对齐，头尾文字区采用实测宽度自适应与 FontMetrics 居中杜绝任何裁剪。
 */
class UavAttitudeHudView(context: Context, private val hub: GcsSharedHub) : View(context) {

    companion object {
        private const val RING_SEGS = 24
        private val UNIT_COS = FloatArray(RING_SEGS + 1) { i -> cos((i.toFloat() / RING_SEGS) * (PI.toFloat() * 2f)) }
        private val UNIT_SIN = FloatArray(RING_SEGS + 1) { i -> sin((i.toFloat() / RING_SEGS) * (PI.toFloat() * 2f)) }
        private val BANK_TICKS = intArrayOf(-30, -15, 0, 15, 30)
        private val CARD_WX = floatArrayOf(0f, 1.10f, 0f, -1.10f)
        private val CARD_WY = floatArrayOf(1.10f, 0f, -1.10f, 0f)
        private val CARD_LBL = arrayOf("N", "E", "S", "W")
    }

    var inspectMode: Int = 0
        private set

    private var chaseFollowBlend: Float = 1.0f
    private var frontFlipBlend: Float = 0.0f
    private var camElevRad: Float = 0.33f
    private var camTransAnim: ValueAnimator? = null

    private var projCx = 0f
    private var projCy = 0f
    private var projCosAz = 1f
    private var projSinAz = 0f
    private var projCosEl = 1f
    private var projSinEl = 0f
    private var projFocal = 100f
    private val projCamDist = 3.35f

    private var lastRoll = -999f
    private var lastPitch = -999f
    private var lastYaw = -999f
    private var lastTiltS1 = -999f
    private var lastTiltS2 = -999f
    private var lastTiltEnabled = true
    private var lastThemeId = -1
    private var lastQuatFlag = false
    private var cachedRollTenths = Int.MIN_VALUE
    private var cachedPitchTenths = Int.MIN_VALUE
    private var cachedHdgInt = Int.MIN_VALUE
    private var cachedRollStr = "R +0.0°"
    private var cachedPitchStr = "P +0.0°"
    private var cachedYawStr = "Y 000°"

    private val d = resources.displayMetrics.density

    private val cardRect = RectF()
    private val badgeRect = RectF()
    private val clipStageRect = RectF()
    private val arcRect = RectF()
    private val polyPath = Path()
    private val reticlePath = Path()

    private val tmpWorld0 = FloatArray(3)
    private val tmpWorld1 = FloatArray(3)
    private val tmpProj0 = FloatArray(3)
    private val tmpProj1 = FloatArray(3)

    private val bgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val borderPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE }
    private val horizonPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE }
    private val dashPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        pathEffect = DashPathEffect(floatArrayOf(2.8f * d, 2.8f * d), 0f)
    }
    private val rigFacePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val rigLinePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
    }
    private val rotorFillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val titlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 8.4f * d
        isFakeBoldText = true
    }
    private val badgePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 7.4f * d
        textAlign = Paint.Align.CENTER
        typeface = Typeface.MONOSPACE
        isFakeBoldText = true
    }
    private val cardinalPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 7.6f * d
        textAlign = Paint.Align.CENTER
        isFakeBoldText = true
    }
    private val valPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 8.2f * d
        textAlign = Paint.Align.CENTER
        typeface = Typeface.MONOSPACE
        isFakeBoldText = true
    }

    fun cycleInspectCamera(): String {
        inspectMode = (inspectMode + 1) % 3
        val startFollow = chaseFollowBlend
        val startFront = frontFlipBlend
        val startElev = camElevRad

        val targetFollow: Float
        val targetFront: Float
        val targetElev: Float
        val tip: String
        when (inspectMode) {
            1 -> {
                targetFollow = 0f
                targetFront = 0f
                targetElev = 0.50f
                tip = "姿态视窗：世界系等轴视角"
            }
            2 -> {
                targetFollow = 1f
                targetFront = 1f
                targetElev = 0.24f
                tip = "姿态视窗：机头迎面正视视角"
            }
            else -> {
                targetFollow = 1f
                targetFront = 0f
                targetElev = 0.33f
                tip = "姿态视窗：机尾追尾地平视角"
            }
        }

        camTransAnim?.cancel()
        camTransAnim = ValueAnimator.ofFloat(0f, 1f).apply {
            duration = 250L
            interpolator = AnimHelper.EASE_OUT_EXPO
            addUpdateListener { a ->
                val t = a.animatedValue as Float
                chaseFollowBlend = startFollow + (targetFollow - startFollow) * t
                frontFlipBlend = startFront + (targetFront - startFront) * t
                camElevRad = startElev + (targetElev - startElev) * t
                invalidate()
            }
            start()
        }
        return tip
    }

    fun tickAndInvalidate() {
        if (visibility != VISIBLE) return
        val s = hub.latestSnapshot
        val tid = hub.themeId
        val tiltOn = s.activeIsTiltRig
        val attChanged = abs(s.smoothRoll - lastRoll) > 0.0025f ||
            abs(s.smoothPitch - lastPitch) > 0.0025f ||
            abs(s.yaw - lastYaw) > 0.003f ||
            abs(s.tiltServo1Rad - lastTiltS1) > 0.003f ||
            abs(s.tiltServo2Rad - lastTiltS2) > 0.003f ||
            tiltOn != lastTiltEnabled ||
            tid != lastThemeId ||
            s.hasLiveAttitudeQuat != lastQuatFlag

        if (attChanged || (s.armed && hub.perfMode != 2)) {
            lastRoll = s.smoothRoll
            lastPitch = s.smoothPitch
            lastYaw = s.yaw
            lastTiltS1 = s.tiltServo1Rad
            lastTiltS2 = s.tiltServo2Rad
            lastTiltEnabled = tiltOn
            lastThemeId = tid
            lastQuatFlag = s.hasLiveAttitudeQuat
            invalidate()
        }
    }

    private fun projectWorldPoint(wx: Float, wy: Float, wz: Float, out: FloatArray) {
        val rx = -wx * projSinAz + wy * projCosAz
        val uy = -wx * projSinEl * projCosAz - wy * projSinEl * projSinAz + wz * projCosEl
        val depth = (projCamDist - (wx * projCosEl * projCosAz + wy * projCosEl * projSinAz + wz * projSinEl)).coerceAtLeast(0.8f)
        val k = projFocal / depth
        out[0] = projCx + rx * k
        out[1] = projCy - uy * k
        out[2] = depth
    }

    private fun baselineCenteredY(centerY: Float, paint: Paint): Float {
        val fm = paint.fontMetrics
        return centerY - (fm.ascent + fm.descent) * 0.5f
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val w = width.toFloat()
        val h = height.toFloat()
        if (w <= 8f || h <= 8f) return

        val snap = hub.latestSnapshot
        val pal = GcsTheme.of(hub.themeId)
        val tiltOn = snap.activeIsTiltRig
        val rig = DroneVectorModel.rigFor(tiltOn)

        val pad = 1.0f * d
        cardRect.set(pad, pad, w - pad, h - pad)
        val rCorner = 8.0f * d
        bgPaint.color = pal.glassFill
        canvas.drawRoundRect(cardRect, rCorner, rCorner, bgPaint)

        borderPaint.color = pal.glassStroke
        borderPaint.strokeWidth = max(1f, 0.9f * d)
        canvas.drawRoundRect(cardRect, rCorner, rCorner, borderPaint)

        drawCornerReticles(canvas, cardRect, pal.accent, 5.5f * d)

        val headerBottom = 18.0f * d
        val prefixTitle = if (tiltOn) "倾转" else "姿态"
        val modeShort = when (inspectMode) {
            1 -> "$prefixTitle·等轴"
            2 -> "$prefixTitle·正视"
            else -> "$prefixTitle·追尾"
        }
        titlePaint.color = pal.textSecondary
        canvas.drawText(modeShort, 7.0f * d, baselineCenteredY(headerBottom * 0.52f, titlePaint), titlePaint)

        val srcTag = when {
            tiltOn && snap.hasLiveTiltTelemetry -> "INDI"
            snap.hasLiveAttitudeQuat -> "QUAT"
            tiltOn -> "4M2S"
            hub.isRosMode -> "ODOM"
            else -> "SIM"
        }
        val bw = max(28f * d, badgePaint.measureText(srcTag) + 8.5f * d)
        val bh = 12.0f * d
        badgeRect.set(w - bw - 6.0f * d, (headerBottom - bh) * 0.5f + 0.5f * d, w - 6.0f * d, (headerBottom + bh) * 0.5f + 0.5f * d)
        bgPaint.color = pal.accentSoftFill
        canvas.drawRoundRect(badgeRect, 3.5f * d, 3.5f * d, bgPaint)
        badgePaint.color = if (snap.hasLiveAttitudeQuat) pal.secondaryAccent else pal.accentBright
        canvas.drawText(srcTag, badgeRect.centerX(), baselineCenteredY(badgeRect.centerY(), badgePaint), badgePaint)

        val footerTop = h - 19.0f * d
        clipStageRect.set(3f * d, headerBottom, w - 3f * d, footerTop)
        projCx = clipStageRect.centerX()
        projCy = clipStageRect.centerY() + 1.5f * d
        val stageRadius = min(clipStageRect.width() * 0.42f, clipStageRect.height() * 0.47f)
        projFocal = stageRadius * 2.55f

        val roll = snap.smoothRoll
        val pitch = snap.smoothPitch
        val yaw = snap.yaw

        val bodyAlignedAz = yaw + PI.toFloat() * (1.0f - frontFlipBlend)
        val isoWorldAz = -0.88f
        var azDiff = bodyAlignedAz - isoWorldAz
        while (azDiff > PI.toFloat()) azDiff -= (PI * 2.0).toFloat()
        while (azDiff < -PI.toFloat()) azDiff += (PI * 2.0).toFloat()
        val camAz = isoWorldAz + azDiff * chaseFollowBlend

        projCosAz = cos(camAz)
        projSinAz = sin(camAz)
        projCosEl = cos(camElevRad)
        projSinEl = sin(camElevRad)

        canvas.save()
        canvas.clipRect(clipStageRect)

        if (chaseFollowBlend < 0.55f) {
            drawGyroWorldRing(canvas, pal)
        } else {
            drawArtificialHorizonAndBankScale(canvas, projCx, projCy, stageRadius, roll, pitch, frontFlipBlend, pal)
        }

        projectWorldPoint(0f, 0f, -0.34f, tmpProj0)
        projectWorldPoint(0f, 0f, 0f, tmpProj1)
        dashPaint.color = pal.hsiTickMinor
        dashPaint.strokeWidth = 0.8f * d
        canvas.drawLine(tmpProj0[0], tmpProj0[1], tmpProj1[0], tmpProj1[1], dashPaint)
        bgPaint.color = Color.argb(if (pal.isLight) 28 else 55, 0, 0, 0)
        canvas.drawOval(
            tmpProj0[0] - stageRadius * 0.33f,
            tmpProj0[1] - stageRadius * 0.10f,
            tmpProj0[0] + stageRadius * 0.33f,
            tmpProj0[1] + stageRadius * 0.10f,
            bgPaint
        )

        val modelScale = 0.88f
        val faces = rig.faces
        for (fIdx in 0 until faces.size) {
            val face = faces[fIdx]
            val nPts = face.pts.size / 3
            if (nPts < 3) continue
            polyPath.reset()
            for (i in 0 until nPts) {
                val bi = i * 3
                DroneVectorModel.transformBodyToWorld(
                    face.pts[bi],
                    face.pts[bi + 1],
                    face.pts[bi + 2],
                    modelScale,
                    roll,
                    pitch,
                    yaw,
                    tmpWorld0
                )
                projectWorldPoint(tmpWorld0[0], tmpWorld0[1], tmpWorld0[2], tmpProj0)
                if (i == 0) {
                    polyPath.moveTo(tmpProj0[0], tmpProj0[1])
                } else {
                    polyPath.lineTo(tmpProj0[0], tmpProj0[1])
                }
            }
            polyPath.close()
            val shade = face.shadeBoost
            rigFacePaint.color = if (pal.isLight) {
                Color.argb((185 * shade).roundToInt().coerceIn(60, 230), 42, 62, 92)
            } else {
                Color.argb((178 * shade).roundToInt().coerceIn(60, 235), 28, 38, 54)
            }
            canvas.drawPath(polyPath, rigFacePaint)
        }

        val propSpinRad = if (snap.armed) {
            ((SystemClock.uptimeMillis() % 720L) / 720f) * (PI.toFloat() * 2f) * 3f
        } else {
            0.4f
        }

        val rotors = rig.rotors
        for (rIdx in 0 until rotors.size) {
            val rot = rotors[rIdx]
            val tiltAlpha = when (rot.servoGroup) {
                1 -> snap.tiltServo1Rad
                2 -> snap.tiltServo2Rad
                else -> 0f
            }
            val shroudR = rot.radius * 1.12f
            polyPath.reset()
            var s = 0
            while (s <= RING_SEGS) {
                val lx = UNIT_COS[s] * rot.radius
                val ly = UNIT_SIN[s] * rot.radius
                DroneVectorModel.transformNacellePointToWorld(
                    rot.cx, rot.cy, rot.cz, lx, ly, 0.024f,
                    tiltAlpha, modelScale, roll, pitch, yaw, tmpWorld1
                )
                projectWorldPoint(tmpWorld1[0], tmpWorld1[1], tmpWorld1[2], tmpProj1)
                if (s == 0) {
                    polyPath.moveTo(tmpProj1[0], tmpProj1[1])
                } else {
                    polyPath.lineTo(tmpProj1[0], tmpProj1[1])
                }
                s += 2
            }
            polyPath.close()
            rotorFillPaint.color = if (rot.isFront) {
                Color.argb(if (pal.isLight) 42 else 36, 255, 68, 58)
            } else {
                Color.argb(if (pal.isLight) 42 else 36, 0, 220, 118)
            }
            canvas.drawPath(polyPath, rotorFillPaint)

            // 若为倾转旋翼短舱，绘制随舵机实时偏转的涵道外护圈与三维推力矢量箭
            if (rot.servoGroup > 0) {
                polyPath.reset()
                var k = 0
                while (k <= RING_SEGS) {
                    DroneVectorModel.transformNacellePointToWorld(
                        rot.cx, rot.cy, rot.cz,
                        UNIT_COS[k] * shroudR, UNIT_SIN[k] * shroudR, 0.02f,
                        tiltAlpha, modelScale, roll, pitch, yaw, tmpWorld1
                    )
                    projectWorldPoint(tmpWorld1[0], tmpWorld1[1], tmpWorld1[2], tmpProj1)
                    if (k == 0) {
                        polyPath.moveTo(tmpProj1[0], tmpProj1[1])
                    } else {
                        polyPath.lineTo(tmpProj1[0], tmpProj1[1])
                    }
                    k += 2
                }
                polyPath.close()
                rigLinePaint.style = Paint.Style.STROKE
                rigLinePaint.color = if (rot.isFront) Color.rgb(255, 76, 66) else Color.rgb(0, 224, 122)
                rigLinePaint.strokeWidth = 1.05f * d
                canvas.drawPath(polyPath, rigLinePaint)

                val mIdx = (rot.motorId - 1).coerceIn(0, 3)
                val vecLen = 0.24f + snap.motorNorm4[mIdx].coerceIn(0.15f, 1.0f) * 0.24f
                DroneVectorModel.transformNacellePointToWorld(
                    rot.cx, rot.cy, rot.cz, 0f, 0f, 0.02f,
                    tiltAlpha, modelScale, roll, pitch, yaw, tmpWorld0
                )
                DroneVectorModel.transformNacellePointToWorld(
                    rot.cx, rot.cy, rot.cz, 0f, 0f, vecLen,
                    tiltAlpha, modelScale, roll, pitch, yaw, tmpWorld1
                )
                projectWorldPoint(tmpWorld0[0], tmpWorld0[1], tmpWorld0[2], tmpProj0)
                projectWorldPoint(tmpWorld1[0], tmpWorld1[1], tmpWorld1[2], tmpProj1)
                rigLinePaint.color = if (rot.servoGroup == 1) Color.rgb(255, 196, 0) else Color.rgb(41, 182, 246)
                rigLinePaint.strokeWidth = 1.35f * d
                canvas.drawLine(tmpProj0[0], tmpProj0[1], tmpProj1[0], tmpProj1[1], rigLinePaint)
            }

            val bladeAng = propSpinRad * rot.spinSign
            val ca = cos(bladeAng) * rot.radius * 0.92f
            val sa = sin(bladeAng) * rot.radius * 0.92f

            DroneVectorModel.transformNacellePointToWorld(
                rot.cx, rot.cy, rot.cz, ca, sa, 0.03f,
                tiltAlpha, modelScale, roll, pitch, yaw, tmpWorld0
            )
            DroneVectorModel.transformNacellePointToWorld(
                rot.cx, rot.cy, rot.cz, -ca, -sa, 0.03f,
                tiltAlpha, modelScale, roll, pitch, yaw, tmpWorld1
            )
            projectWorldPoint(tmpWorld0[0], tmpWorld0[1], tmpWorld0[2], tmpProj0)
            projectWorldPoint(tmpWorld1[0], tmpWorld1[1], tmpWorld1[2], tmpProj1)

            rigLinePaint.color = if (pal.isLight) Color.argb(175, 25, 42, 68) else Color.argb(180, 225, 238, 255)
            rigLinePaint.strokeWidth = 1.0f * d
            canvas.drawLine(tmpProj0[0], tmpProj0[1], tmpProj1[0], tmpProj1[1], rigLinePaint)
        }

        val segments = rig.segments
        for (sIdx in 0 until segments.size) {
            val seg = segments[sIdx]
            DroneVectorModel.transformBodyToWorld(seg.x0, seg.y0, seg.z0, modelScale, roll, pitch, yaw, tmpWorld0)
            DroneVectorModel.transformBodyToWorld(seg.x1, seg.y1, seg.z1, modelScale, roll, pitch, yaw, tmpWorld1)
            projectWorldPoint(tmpWorld0[0], tmpWorld0[1], tmpWorld0[2], tmpProj0)
            projectWorldPoint(tmpWorld1[0], tmpWorld1[1], tmpWorld1[2], tmpProj1)

            val meanDepth = (tmpProj0[2] + tmpProj1[2]) * 0.5f
            val depthCue = ((4.1f - meanDepth) / 1.5f).coerceIn(0.52f, 1.18f)

            val baseCol = when (seg.role) {
                PartRole.NAV_FRONT_RED -> -0x00bbdd
                PartRole.NAV_REAR_GREEN -> if (pal.isLight) -0xff57a8 else -0xff198a
                PartRole.GIMBAL_NOSE -> pal.accent
                PartRole.LIDAR_DOME -> if (pal.isLight) -0xff7d2e else -0x9b0026
                PartRole.ARM_BEAM -> pal.textPrimary
                PartRole.FUSELAGE_EDGE -> pal.accentBright
                PartRole.LANDING_SKID -> pal.textMuted
                PartRole.FUSELAGE_SHELL -> pal.textSecondary
                PartRole.TILT_SERVO_SHAFT -> Color.rgb(255, 179, 0)
                PartRole.THRUST_VECTOR -> pal.secondaryAccent
            }

            val alpha = (Color.alpha(baseCol) * (0.58f + 0.42f * depthCue)).roundToInt().coerceIn(75, 255)
            rigLinePaint.color = (alpha shl 24) or (baseCol and 0x00FFFFFF)
            rigLinePaint.strokeWidth = (1.10f * seg.widthFactor * depthCue.coerceAtLeast(0.72f)) * d
            canvas.drawLine(tmpProj0[0], tmpProj0[1], tmpProj1[0], tmpProj1[1], rigLinePaint)
        }

        drawBodyTriadAxes(canvas, roll, pitch, yaw)
        if (tiltOn) {
            val s1Deg = (snap.tiltServo1Rad * 57.29578f).roundToInt()
            val s2Deg = (snap.tiltServo2Rad * 57.29578f).roundToInt()
            val sTag = "a1:${if (s1Deg >= 0) "+" else ""}$s1Deg° a2:${if (s2Deg >= 0) "+" else ""}$s2Deg°"
            badgePaint.color = if (snap.tiltServoSaturated) Color.rgb(255, 145, 0) else pal.secondaryAccent
            val prevAlign = badgePaint.textAlign
            badgePaint.textAlign = Paint.Align.LEFT
            canvas.drawText(sTag, 6.5f * d, headerBottom + 9.2f * d, badgePaint)
            badgePaint.textAlign = prevAlign
        }
        canvas.restore()

        borderPaint.color = pal.divider
        borderPaint.strokeWidth = 1f
        canvas.drawLine(5.5f * d, footerTop, w - 5.5f * d, footerTop, borderPaint)

        val rollDeg = roll * 57.29578f
        val pitchDeg = pitch * 57.29578f
        var hdgDeg = (90f - yaw * 57.29578f) % 360f
        if (hdgDeg < 0f) hdgDeg += 360f

        val rTenths = (rollDeg * 10f).roundToInt()
        if (rTenths != cachedRollTenths) {
            cachedRollTenths = rTenths
            cachedRollStr = FastHudFormatter.formatSignedTenthsDeg("R", rTenths)
        }
        val pTenths = (pitchDeg * 10f).roundToInt()
        if (pTenths != cachedPitchTenths) {
            cachedPitchTenths = pTenths
            cachedPitchStr = FastHudFormatter.formatSignedTenthsDeg("P", pTenths)
        }
        val hInt = hdgDeg.roundToInt() % 360
        if (hInt != cachedHdgInt) {
            cachedHdgInt = hInt
            cachedYawStr = FastHudFormatter.formatHeading3Digits(hInt)
        }

        val innerFooterW = w - 8.0f * d
        val colW = innerFooterW / 3f
        val baseValSize = 8.2f * d
        valPaint.textSize = baseValSize
        val widest = max(
            valPaint.measureText(cachedRollStr),
            max(valPaint.measureText(cachedPitchStr), valPaint.measureText(cachedYawStr))
        )
        if (widest > colW - 3.0f * d && widest > 1f) {
            valPaint.textSize = (baseValSize * ((colW - 3.0f * d) / widest)).coerceAtLeast(6.6f * d)
        }

        val textY = baselineCenteredY((footerTop + h - pad) * 0.5f, valPaint)
        val warnCol = Color.rgb(255, 145, 0)
        val startX = 4.0f * d

        valPaint.color = if (abs(rollDeg) >= 22f) warnCol else pal.textPrimary
        canvas.drawText(cachedRollStr, startX + colW * 0.5f, textY, valPaint)

        valPaint.color = if (abs(pitchDeg) >= 22f) warnCol else pal.textPrimary
        canvas.drawText(cachedPitchStr, startX + colW * 1.5f, textY, valPaint)

        valPaint.color = pal.accentBright
        canvas.drawText(cachedYawStr, startX + colW * 2.5f, textY, valPaint)
    }

    private fun drawArtificialHorizonAndBankScale(
        canvas: Canvas,
        cx: Float,
        cy: Float,
        radius: Float,
        roll: Float,
        pitch: Float,
        frontFlip: Float,
        pal: GcsTheme.Palette
    ) {
        val visualRollSign = 1.0f - 2.0f * frontFlip
        val horizonRollDeg = (-roll * visualRollSign) * 57.29578f
        val pitchShiftPx = (pitch / 0.55f).coerceIn(-1f, 1f) * (radius * 0.48f) * (1.0f - 2.0f * frontFlip)

        val arcR = radius * 0.94f
        arcRect.set(cx - arcR, cy - arcR, cx + arcR, cy + arcR)
        horizonPaint.color = pal.hsiTickMinor
        horizonPaint.strokeWidth = 0.8f * d
        canvas.drawArc(arcRect, -125f, 70f, false, horizonPaint)

        for (i in BANK_TICKS.indices) {
            val bt = BANK_TICKS[i]
            val rad = (-90 + bt) * 0.017453292f
            val cr = cos(rad)
            val sr = sin(rad)
            val rIn = arcR - (if (bt == 0) 3.8f else 2.5f) * d
            horizonPaint.color = if (bt == 0) pal.accent else pal.hsiTickMajor
            horizonPaint.strokeWidth = (if (bt == 0) 1.2f else 0.85f) * d
            canvas.drawLine(cx + cr * rIn, cy + sr * rIn, cx + cr * arcR, cy + sr * arcR, horizonPaint)
        }

        canvas.save()
        canvas.translate(cx, cy)
        canvas.rotate(horizonRollDeg)

        horizonPaint.color = if (pal.isLight) Color.argb(145, 0, 128, 72) else Color.argb(135, 0, 230, 118)
        horizonPaint.strokeWidth = 1.1f * d
        val hw = radius * 0.86f
        val gap = radius * 0.22f
        canvas.drawLine(-hw, pitchShiftPx, -gap, pitchShiftPx, horizonPaint)
        canvas.drawLine(gap, pitchShiftPx, hw, pitchShiftPx, horizonPaint)

        horizonPaint.color = pal.hsiTickMinor
        horizonPaint.strokeWidth = 0.75f * d
        val stepP = radius * 0.26f
        canvas.drawLine(-radius * 0.34f, pitchShiftPx - stepP, radius * 0.34f, pitchShiftPx - stepP, horizonPaint)
        canvas.drawLine(-radius * 0.34f, pitchShiftPx + stepP, radius * 0.34f, pitchShiftPx + stepP, horizonPaint)

        val ptrY = -arcR + 4.8f * d
        polyPath.reset()
        polyPath.moveTo(0f, -arcR + 1.0f * d)
        polyPath.lineTo(-2.6f * d, ptrY)
        polyPath.lineTo(2.6f * d, ptrY)
        polyPath.close()
        bgPaint.color = pal.secondaryAccent
        canvas.drawPath(polyPath, bgPaint)

        canvas.restore()
    }

    private fun drawGyroWorldRing(canvas: Canvas, pal: GcsTheme.Palette) {
        val ringR = 0.98f
        polyPath.reset()
        for (s in 0..RING_SEGS) {
            projectWorldPoint(UNIT_COS[s] * ringR, UNIT_SIN[s] * ringR, -0.12f, tmpProj0)
            if (s == 0) {
                polyPath.moveTo(tmpProj0[0], tmpProj0[1])
            } else {
                polyPath.lineTo(tmpProj0[0], tmpProj0[1])
            }
        }
        horizonPaint.color = pal.hsiRingStroke
        horizonPaint.strokeWidth = 0.9f * d
        canvas.drawPath(polyPath, horizonPaint)

        for (i in 0..3) {
            projectWorldPoint(CARD_WX[i], CARD_WY[i], -0.12f, tmpProj0)
            cardinalPaint.color = if (i == 0) Color.rgb(240, 68, 68) else pal.textMuted
            canvas.drawText(CARD_LBL[i], tmpProj0[0], baselineCenteredY(tmpProj0[1], cardinalPaint), cardinalPaint)
        }
    }

    private fun drawBodyTriadAxes(canvas: Canvas, roll: Float, pitch: Float, yaw: Float) {
        projectWorldPoint(0f, 0f, 0.06f, tmpProj0)
        val axisLen = 0.36f

        DroneVectorModel.transformBodyToWorld(axisLen, 0f, 0.06f, 1f, roll, pitch, yaw, tmpWorld1)
        projectWorldPoint(tmpWorld1[0], tmpWorld1[1], tmpWorld1[2], tmpProj1)
        rigLinePaint.color = Color.rgb(255, 82, 82)
        rigLinePaint.strokeWidth = 1.25f * d
        canvas.drawLine(tmpProj0[0], tmpProj0[1], tmpProj1[0], tmpProj1[1], rigLinePaint)

        DroneVectorModel.transformBodyToWorld(0f, axisLen, 0.06f, 1f, roll, pitch, yaw, tmpWorld1)
        projectWorldPoint(tmpWorld1[0], tmpWorld1[1], tmpWorld1[2], tmpProj1)
        rigLinePaint.color = Color.rgb(0, 220, 118)
        canvas.drawLine(tmpProj0[0], tmpProj0[1], tmpProj1[0], tmpProj1[1], rigLinePaint)

        DroneVectorModel.transformBodyToWorld(0f, 0f, 0.06f + axisLen, 1f, roll, pitch, yaw, tmpWorld1)
        projectWorldPoint(tmpWorld1[0], tmpWorld1[1], tmpWorld1[2], tmpProj1)
        rigLinePaint.color = Color.rgb(64, 178, 255)
        canvas.drawLine(tmpProj0[0], tmpProj0[1], tmpProj1[0], tmpProj1[1], rigLinePaint)
    }

    private fun drawCornerReticles(canvas: Canvas, r: RectF, color: Int, len: Float) {
        reticlePath.reset()
        val inset = 2.2f * d
        val l = r.left + inset
        val t = r.top + inset
        val rt = r.right - inset
        val b = r.bottom - inset

        reticlePath.moveTo(l, t + len)
        reticlePath.lineTo(l, t)
        reticlePath.lineTo(l + len, t)
        reticlePath.moveTo(rt - len, t)
        reticlePath.lineTo(rt, t)
        reticlePath.lineTo(rt, t + len)
        reticlePath.moveTo(l, b - len)
        reticlePath.lineTo(l, b)
        reticlePath.lineTo(l + len, b)
        reticlePath.moveTo(rt - len, b)
        reticlePath.lineTo(rt, b)
        reticlePath.lineTo(rt, b - len)

        borderPaint.color = (145 shl 24) or (color and 0x00FFFFFF)
        borderPaint.strokeWidth = 1.05f * d
        canvas.drawPath(reticlePath, borderPaint)
    }
}
