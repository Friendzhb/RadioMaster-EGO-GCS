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
import android.view.View
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.SwarmPalette
import java.util.Locale
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * 零分配、主题自适应 + 实测文字胶囊宽度零裁剪版商业航空全向避障姿态球 (HSI)
 */
class DjiHsiRadarView(context: Context, private val hub: GcsSharedHub) : View(context) {

    companion object {
        private const val COL_RED = -0x00c4d0 // #FF3B30
        private const val COL_AMBER = -0x005500 // #FFAB00
        private const val COL_CARD_N = -0x00b2b1 // #FF4D4F

        private val BIN_WORLD_ANG = FloatArray(60) { i ->
            -PI.toFloat() + (i + 0.5f) * (PI.toFloat() * 2f / 60f)
        }
        private val BEZEL_WORLD_ANG = FloatArray(36) { idx ->
            Math.toRadians((90 - idx * 10).toDouble()).toFloat()
        }
    }

    var rangeMeters: Float = 4.0f
        private set
    var northUpMode: Boolean = false
        private set
    private var rotBlend: Float = 0f
    private var dispRangeAnim: Float = 4.0f
    private var modeAnim: ValueAnimator? = null
    private var rangeAnim: ValueAnimator? = null

    private var curRefYaw: Float = 0f
    private var lastYaw = -999f
    private var lastRoll = -999f
    private var lastPitch = -999f
    private var lastRadarChecksum = -999f
    private var lastThemeId = -1

    private var cachedPillKey = Int.MIN_VALUE
    private var cachedPillTxt = "机头 4m"

    private val d = resources.displayMetrics.density

    private val fillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val strokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeCap = Paint.Cap.BUTT
    }
    private val dashPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        pathEffect = DashPathEffect(floatArrayOf(3.5f * d, 3.5f * d), 0f)
    }
    private val cardinalPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 9.0f * d
        textAlign = Paint.Align.CENTER
        isFakeBoldText = true
    }
    private val pillTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 8.2f * d
        textAlign = Paint.Align.CENTER
        isFakeBoldText = true
    }

    private val clipPath = Path()
    private val workPath = Path()
    private val rectOuter = RectF()
    private val rectInner = RectF()
    private val pillRect = RectF()

    private fun worldToCanvasRad(wAng: Float): Float = -PI.toFloat() * 0.5f - (wAng - curRefYaw)

    fun invalidateIfChanged() {
        val s = hub.latestSnapshot
        val tid = hub.themeId
        var sum = 0f
        val r = s.radarSmooth60
        var i = 0
        while (i < 60) {
            sum += r[i]
            i += 4
        }
        if (tid != lastThemeId ||
            abs(s.yaw - lastYaw) > 0.004f ||
            abs(s.smoothRoll - lastRoll) > 0.003f ||
            abs(s.smoothPitch - lastPitch) > 0.003f ||
            abs(sum - lastRadarChecksum) > 0.02f
        ) {
            lastThemeId = tid
            lastYaw = s.yaw
            lastRoll = s.smoothRoll
            lastPitch = s.smoothPitch
            lastRadarChecksum = sum
            invalidate()
        }
    }

    fun cycleMode(): String {
        val label: String
        val targetRange: Float
        if (!northUpMode && rangeMeters == 4.0f) {
            targetRange = 8.0f
            label = "HSI 姿态球：机头朝上 · 雷达量程平滑扩至 8m"
        } else if (!northUpMode && rangeMeters == 8.0f) {
            northUpMode = true
            targetRange = 6.0f
            animateNorthBlend(1f)
            label = "HSI 姿态球：北向锁定模式 · 量程 6m"
        } else {
            northUpMode = false
            targetRange = 4.0f
            animateNorthBlend(0f)
            label = "HSI 姿态球：机头朝上 · 雷达量程 4m"
        }
        rangeMeters = targetRange
        animateRangeScale(targetRange)
        lastYaw = -999f
        invalidate()
        return label
    }

    private fun animateNorthBlend(target: Float) {
        modeAnim?.cancel()
        modeAnim = ValueAnimator.ofFloat(rotBlend, target).apply {
            duration = 320L
            interpolator = AnimHelper.EASE_OUT_EXPO
            addUpdateListener { a ->
                rotBlend = a.animatedValue as Float
                invalidate()
            }
            start()
        }
    }

    private fun animateRangeScale(target: Float) {
        rangeAnim?.cancel()
        rangeAnim = ValueAnimator.ofFloat(dispRangeAnim, target).apply {
            duration = 280L
            interpolator = AnimHelper.EASE_OUT_EXPO
            addUpdateListener { a ->
                dispRangeAnim = a.animatedValue as Float
                invalidate()
            }
            start()
        }
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val snap = hub.latestSnapshot
        val pal = GcsTheme.of(hub.themeId)
        val cx = width * 0.5f
        val cy = height * 0.5f
        val rFull = min(width, height) * 0.48f
        val rBezelIn = rFull * 0.79f
        val rRadarMax = rFull * 0.76f
        val rAttCore = rFull * 0.34f
        val activeRange = dispRangeAnim.coerceAtLeast(1.0f)

        var dYaw = (PI.toFloat() * 0.5f) - snap.yaw
        while (dYaw > PI.toFloat()) dYaw -= (PI * 2.0).toFloat()
        while (dYaw < -PI.toFloat()) dYaw += (PI * 2.0).toFloat()
        curRefYaw = snap.yaw + dYaw * rotBlend

        fillPaint.color = pal.hsiOuterBg
        canvas.drawCircle(cx, cy, rFull, fillPaint)
        fillPaint.color = pal.hsiInnerBg
        canvas.drawCircle(cx, cy, rBezelIn, fillPaint)

        strokePaint.color = pal.hsiRingStroke
        strokePaint.strokeWidth = 1.1f * d
        canvas.drawCircle(cx, cy, rFull, strokePaint)
        strokePaint.color = pal.hsiTickMinor
        strokePaint.strokeWidth = 0.85f * d
        canvas.drawCircle(cx, cy, rBezelIn, strokePaint)

        val rMidRing = rAttCore + (rRadarMax - rAttCore) * 0.5f
        dashPaint.color = pal.hsiTickMinor
        dashPaint.strokeWidth = 0.75f * d
        canvas.drawCircle(cx, cy, rMidRing, dashPaint)

        val noseRad = worldToCanvasRad(snap.yaw)
        val noseDeg = noseRad * (180f / PI.toFloat())
        rectOuter.set(cx - rRadarMax, cy - rRadarMax, cx + rRadarMax, cy + rRadarMax)
        rectInner.set(cx - rAttCore, cy - rAttCore, cx + rAttCore, cy + rAttCore)
        workPath.reset()
        workPath.arcTo(rectOuter, noseDeg - 42f, 84f)
        workPath.arcTo(rectInner, noseDeg + 42f, -84f)
        workPath.close()
        fillPaint.color = Color.argb(if (pal.isLight) 32 else 22, Color.red(pal.accent), Color.green(pal.accent), Color.blue(pal.accent))
        canvas.drawPath(workPath, fillPaint)

        val bins = snap.radarSmooth60
        val nBins = bins.size
        var nearestHit = 9.9f
        val spanR = rRadarMax - rAttCore
        val sweepDeg = 360f / nBins + 0.65f
        val fillRed = Color.argb(if (pal.isLight) 105 else 88, 255, 59, 48)
        val fillAmber = Color.argb(if (pal.isLight) 78 else 54, 255, 171, 0)
        val fillSafe = Color.argb(if (pal.isLight) 70 else 54, Color.red(pal.secondaryAccent), Color.green(pal.secondaryAccent), Color.blue(pal.secondaryAccent))
        val invRangeSpan = 1.0f / max(0.5f, activeRange - 0.25f)
        rectOuter.set(cx - rRadarMax, cy - rRadarMax, cx + rRadarMax, cy + rRadarMax)

        for (i in 0 until nBins) {
            val dist = bins[i]
            if (dist < nearestHit) nearestHit = dist
            if (dist > activeRange) continue

            val cDeg = worldToCanvasRad(BIN_WORLD_ANG[i]) * (180f / PI.toFloat())
            val norm = ((dist - 0.25f) * invRangeSpan).coerceIn(0f, 1f)
            val rHit = rAttCore + 2.5f * d + norm * (spanR - 3.5f * d)

            val strokeCol: Int
            val fillCol: Int
            if (dist < 1.25f) {
                strokeCol = COL_RED
                fillCol = fillRed
            } else if (dist < 2.35f) {
                strokeCol = COL_AMBER
                fillCol = fillAmber
            } else {
                strokeCol = pal.secondaryAccent
                fillCol = fillSafe
            }

            rectInner.set(cx - rHit, cy - rHit, cx + rHit, cy + rHit)
            workPath.reset()
            workPath.arcTo(rectOuter, cDeg - sweepDeg * 0.5f, sweepDeg)
            workPath.arcTo(rectInner, cDeg + sweepDeg * 0.5f, -sweepDeg)
            workPath.close()
            fillPaint.color = fillCol
            canvas.drawPath(workPath, fillPaint)

            strokePaint.color = strokeCol
            strokePaint.strokeWidth = 2.0f * d
            canvas.drawArc(rectInner, cDeg - sweepDeg * 0.46f, sweepDeg * 0.92f, false, strokePaint)
        }

        canvas.save()
        clipPath.reset()
        clipPath.addCircle(cx, cy, rAttCore, Path.Direction.CW)
        canvas.clipPath(clipPath)

        canvas.translate(cx, cy)
        canvas.rotate(snap.smoothRoll * (180f / PI.toFloat()))
        val pitchOffset = (snap.smoothPitch / 0.42f).coerceIn(-1f, 1f) * (rAttCore * 0.62f)

        fillPaint.color = pal.hsiSky
        canvas.drawRect(-rAttCore * 1.5f, -rAttCore * 2f, rAttCore * 1.5f, pitchOffset, fillPaint)
        fillPaint.color = pal.hsiGround
        canvas.drawRect(-rAttCore * 1.5f, pitchOffset, rAttCore * 1.5f, rAttCore * 2f, fillPaint)

        strokePaint.color = if (pal.isLight) Color.WHITE else Color.argb(215, 0, 230, 118)
        strokePaint.strokeWidth = 1.25f * d
        canvas.drawLine(-rAttCore * 1.2f, pitchOffset, rAttCore * 1.2f, pitchOffset, strokePaint)

        strokePaint.color = Color.argb(135, 255, 255, 255)
        strokePaint.strokeWidth = 0.85f * d
        val rungGap = rAttCore * 0.30f
        canvas.drawLine(-rAttCore * 0.38f, pitchOffset - rungGap, rAttCore * 0.38f, pitchOffset - rungGap, strokePaint)
        canvas.drawLine(-rAttCore * 0.38f, pitchOffset + rungGap, rAttCore * 0.38f, pitchOffset + rungGap, strokePaint)
        canvas.restore()

        strokePaint.color = pal.hsiRingStroke
        strokePaint.strokeWidth = 1.1f * d
        canvas.drawCircle(cx, cy, rAttCore, strokePaint)

        val cardFm = cardinalPaint.fontMetrics
        val cardCenterOffset = -(cardFm.ascent + cardFm.descent) * 0.5f

        for (idx in 0 until 36) {
            val deg = idx * 10
            val cRad = worldToCanvasRad(BEZEL_WORLD_ANG[idx])
            val cr = cos(cRad)
            val sr = sin(cRad)
            val isCard = (deg % 90 == 0)
            val is30 = (deg % 30 == 0)

            if (!isCard) {
                val r0 = if (is30) rBezelIn + 1.4f * d else rBezelIn + 2.8f * d
                val r1 = if (is30) rFull - 2.2f * d else rFull - 3.6f * d
                strokePaint.color = if (is30) pal.hsiTickMajor else pal.hsiTickMinor
                strokePaint.strokeWidth = (if (is30) 1.1f else 0.75f) * d
                canvas.drawLine(cx + cr * r0, cy + sr * r0, cx + cr * r1, cy + sr * r1, strokePaint)
            } else {
                val rTxt = (rBezelIn + rFull) * 0.5f
                val tx = cx + cr * rTxt
                val ty = cy + sr * rTxt + cardCenterOffset
                val lbl = when (deg) {
                    0 -> "N"
                    90 -> "E"
                    180 -> "S"
                    else -> "W"
                }
                cardinalPaint.color = if (deg == 0) COL_CARD_N else pal.textPrimary
                canvas.drawText(lbl, tx, ty, cardinalPaint)
            }
        }

        workPath.reset()
        workPath.moveTo(cx, cy - rBezelIn + 1.6f * d)
        workPath.lineTo(cx - 3.0f * d, cy - rBezelIn - 3.0f * d)
        workPath.lineTo(cx + 3.0f * d, cy - rBezelIn - 3.0f * d)
        workPath.close()
        fillPaint.color = pal.accent
        canvas.drawPath(workPath, fillPaint)

        val goalAng = worldToCanvasRad(atan2(snap.goalY - snap.uavY, snap.goalX - snap.uavX))
        val cg = cos(goalAng)
        val sg = sin(goalAng)
        val gx = cx + cg * (rBezelIn - 1.2f * d)
        val gy = cy + sg * (rBezelIn - 1.2f * d)

        dashPaint.color = pal.secondaryAccent
        dashPaint.strokeWidth = 1.2f * d
        canvas.drawLine(
            cx + cg * (rAttCore * 0.65f),
            cy + sg * (rAttCore * 0.65f),
            gx,
            gy,
            dashPaint
        )

        canvas.save()
        canvas.translate(gx, gy)
        canvas.rotate(45f)
        val dm = 3.0f * d
        fillPaint.color = pal.secondaryAccent
        canvas.drawRect(-dm, -dm, dm, dm, fillPaint)
        strokePaint.color = Color.WHITE
        strokePaint.strokeWidth = 1.0f * d
        canvas.drawRect(-dm, -dm, dm, dm, strokePaint)
        canvas.restore()

        // v3.9: 多机协同模式下在 HSI 避障球同屏标绘其余僚机的分色方位距离光点
        if (snap.swarmCount > 1) {
            for (id in 0 until snap.swarmCount) {
                if (id == snap.activeDroneId) continue
                val ag = snap.swarmAgents[id]
                val dxP = ag.uavX - snap.uavX
                val dyP = ag.uavY - snap.uavY
                val distP = sqrt(dxP * dxP + dyP * dyP)
                if (distP > 0.05f && distP <= activeRange * 1.15f) {
                    val pAng = worldToCanvasRad(atan2(dyP, dxP))
                    val pNorm = ((distP - 0.25f) * invRangeSpan).coerceIn(0.05f, 1.0f)
                    val pR = rAttCore + 2.5f * d + pNorm * (spanR - 4.0f * d)
                    val px = cx + cos(pAng) * pR
                    val py = cy + sin(pAng) * pR
                    fillPaint.color = SwarmPalette.COLOR_INT[id]
                    canvas.drawCircle(px, py, 2.7f * d, fillPaint)
                    strokePaint.color = Color.WHITE
                    strokePaint.strokeWidth = 0.8f * d
                    canvas.drawCircle(px, py, 2.7f * d, strokePaint)
                }
            }
        }

        val hSpd = sqrt(snap.vx * snap.vx + snap.vy * snap.vy)
        if (hSpd > 0.15f) {
            val wVelAng = atan2(snap.vy, snap.vx)
            val vAng = worldToCanvasRad(wVelAng)
            val stopDist = (hSpd * hSpd) / (2f * max(1.2f, hub.maxAcc * 0.85f))
            val stopNorm = ((stopDist - 0.15f) * invRangeSpan).coerceIn(0.08f, 0.96f)
            val vLen = rAttCore + stopNorm * (rRadarMax - rAttCore)
            val tipX = cx + cos(vAng) * vLen
            val tipY = cy + sin(vAng) * vLen
            // 检查速度方向扇区的障碍物距离，若刹停距离逼近障碍则将预测制动点染为琥珀/红色预警
            val sectorIdx = (((wVelAng + PI.toFloat()) / (PI.toFloat() * 2f)) * 60f).toInt().coerceIn(0, 59)
            val sectorObsDist = bins[sectorIdx]
            val stopRisk = stopDist + 0.45f >= sectorObsDist
            val vecCol = if (stopRisk) COL_AMBER else pal.accent
            strokePaint.color = vecCol
            strokePaint.strokeWidth = 1.6f * d
            canvas.drawLine(cx, cy, tipX, tipY, strokePaint)
            val stopDotR = 2.4f * d
            fillPaint.color = if (stopDist + 0.20f >= sectorObsDist) COL_RED else vecCol
            canvas.drawCircle(tipX, tipY, stopDotR, fillPaint)
        }

        canvas.save()
        canvas.translate(cx, cy)
        canvas.rotate((noseRad + PI.toFloat() * 0.5f) * (180f / PI.toFloat()))
        workPath.reset()
        workPath.moveTo(0f, -8.5f * d)
        workPath.lineTo(6.4f * d, 5.8f * d)
        workPath.lineTo(0f, 2.5f * d)
        workPath.lineTo(-6.4f * d, 5.8f * d)
        workPath.close()
        fillPaint.color = Color.WHITE
        canvas.drawPath(workPath, fillPaint)
        strokePaint.color = Color.rgb(12, 20, 32)
        strokePaint.strokeWidth = 1.15f * d
        canvas.drawPath(workPath, strokePaint)
        canvas.restore()

        val hitTenths = if (nearestHit <= rangeMeters) (nearestHit * 10f).roundToInt() else -1
        val pillKey = if (hitTenths >= 0) {
            0x10000 or hitTenths
        } else {
            (if (northUpMode) 0x20000 else 0x30000) or rangeMeters.toInt()
        }
        if (pillKey != cachedPillKey) {
            cachedPillKey = pillKey
            cachedPillTxt = if (hitTenths >= 0) {
                FastHudFormatter.formatUnsignedTenths("避障 ", hitTenths, "m")
            } else {
                "${if (northUpMode) "北向" else "机头"} ${rangeMeters.toInt()}m"
            }
        }

        pillTextPaint.color = when {
            nearestHit < 1.25f -> COL_RED
            nearestHit < 2.35f -> COL_AMBER
            else -> pal.textPrimary
        }
        val pw = max(46f * d, pillTextPaint.measureText(cachedPillTxt) + 10.0f * d)
        val ph = 13.5f * d
        pillRect.set(cx - pw * 0.5f, cy + rBezelIn - ph - 1.5f * d, cx + pw * 0.5f, cy + rBezelIn - 1.5f * d)
        fillPaint.color = pal.hsiPillBg
        canvas.drawRoundRect(pillRect, 4.0f * d, 4.0f * d, fillPaint)
        val pFm = pillTextPaint.fontMetrics
        val pTextY = pillRect.centerY() - (pFm.ascent + pFm.descent) * 0.5f
        canvas.drawText(cachedPillTxt, cx, pTextY, pillTextPaint)
    }
}

/**
 * 零分配、实测中心徽章宽度自适应版顶部水平航向刻度带
 */
class DjiCompassTapeView(context: Context, private val hub: GcsSharedHub) : View(context) {

    companion object {
        private val DEG_LABELS = Array(12) { idx ->
            when (val deg = idx * 30) {
                0 -> "N"
                90 -> "E"
                180 -> "S"
                270 -> "W"
                else -> String.format(Locale.US, "%03d", deg)
            }
        }
        private val OCTANTS = arrayOf("N", "NE", "E", "SE", "S", "SW", "W", "NW")
    }

    private var lastYaw = -999f
    private var lastGoalAng = -999f
    private var lastThemeId = -1
    private var cachedHdgBadge = -1
    private var cachedHdgBadgeTxt = "000° N"

    private val density = resources.displayMetrics.density
    private val tickPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE }
    private val txtPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 9.2f * density
        textAlign = Paint.Align.CENTER
        isFakeBoldText = true
    }
    private val readoutBgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }
    private val readoutStrokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 1.0f * density
    }
    private val readoutTxtPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 10.2f * density
        textAlign = Paint.Align.CENTER
        typeface = Typeface.MONOSPACE
        isFakeBoldText = true
    }
    private val lubberPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }
    private val goalCaretPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }
    private val lubberPath = Path()
    private val badgeRect = RectF()

    fun invalidateIfChanged() {
        val s = hub.latestSnapshot
        val tid = hub.themeId
        val ga = atan2(s.goalY - s.uavY, s.goalX - s.uavX)
        if (tid != lastThemeId || abs(s.yaw - lastYaw) > 0.0025f || abs(ga - lastGoalAng) > 0.01f) {
            lastThemeId = tid
            lastYaw = s.yaw
            lastGoalAng = ga
            invalidate()
        }
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val snap = hub.latestSnapshot
        val pal = GcsTheme.of(hub.themeId)
        val w = width.toFloat()
        val h = height.toFloat()
        if (w <= 4f || h <= 4f) return

        val yawDeg = snap.yaw * (180f / PI.toFloat())
        var heading = (90f - yawDeg) % 360f
        if (heading < 0f) heading += 360f

        val center = w * 0.5f
        val pxPerDeg = 1.75f * density
        val padX = 5f * density

        val baseR = Color.red(pal.textPrimary)
        val baseG = Color.green(pal.textPrimary)
        val baseB = Color.blue(pal.textPrimary)
        val secR = Color.red(pal.textSecondary)
        val secG = Color.green(pal.textSecondary)
        val secB = Color.blue(pal.textSecondary)

        val hdgInt = ((heading.roundToInt() % 360) + 360) % 360
        if (hdgInt != cachedHdgBadge) {
            cachedHdgBadge = hdgInt
            val octant = OCTANTS[((hdgInt + 22) % 360) / 45]
            cachedHdgBadgeTxt = FastHudFormatter.formatCompassBadge(hdgInt, octant)
        }

        val boxW = max(58f * density, readoutTxtPaint.measureText(cachedHdgBadgeTxt) + 12f * density)
        val maskHalfW = boxW * 0.5f + 3f * density

        canvas.save()
        canvas.clipRect(padX, 1f, w - padX, h - 1f)

        val halfSpanDeg = ((w * 0.5f) / pxPerDeg).toInt() + 10
        val startD = ((floor((heading - halfSpanDeg) / 10f)).toInt()) * 10
        val endD = ((ceil((heading + halfSpanDeg) / 10f)).toInt()) * 10

        var deg = startD
        while (deg <= endD) {
            val x = center + (deg - heading) * pxPerDeg
            if (x >= padX && x <= w - padX) {
                val edgeDist = min(x - padX, (w - padX) - x)
                val edgeFade = (edgeDist / (18f * density)).coerceIn(0f, 1f)
                val norm = ((deg % 360) + 360) % 360
                val cardinal = (norm % 90 == 0)
                val is30 = (norm % 30 == 0)

                val tickBaseAlpha = if (cardinal) {
                    225
                } else if (is30) {
                    155
                } else {
                    85
                }
                tickPaint.color = Color.argb((tickBaseAlpha * edgeFade).roundToInt(), baseR, baseG, baseB)
                tickPaint.strokeWidth = (if (cardinal) 1.4f else 0.95f) * density
                val tickTop = h - (if (is30) 6.8f else 3.8f) * density
                canvas.drawLine(x, tickTop, x, h - 1.2f * density, tickPaint)

                if (is30) {
                    val distFromCenter = abs(x - center)
                    val centerMask = ((distFromCenter - maskHalfW) / (9f * density)).coerceIn(0f, 1f)
                    val finalAlpha = edgeFade * centerMask
                    if (finalAlpha > 0.04f) {
                        val label = DEG_LABELS[(norm / 30) % 12]
                        txtPaint.color = when {
                            norm == 0 -> Color.argb((255 * finalAlpha).roundToInt(), 235, 58, 58)
                            cardinal -> Color.argb((245 * finalAlpha).roundToInt(), baseR, baseG, baseB)
                            else -> Color.argb((190 * finalAlpha).roundToInt(), secR, secG, secB)
                        }
                        canvas.drawText(label, x, 13.5f * density, txtPaint)
                    }
                }
            }
            deg += 10
        }

        val goalWorldDeg = atan2(snap.goalY - sOrZero(snap.uavY), snap.goalX - sOrZero(snap.uavX)) * (180f / PI.toFloat())
        var goalHeading = (90f - goalWorldDeg) % 360f
        if (goalHeading < 0f) goalHeading += 360f
        var diffGoal = goalHeading - heading
        while (diffGoal > 180f) diffGoal -= 360f
        while (diffGoal < -180f) diffGoal += 360f
        val gx = (center + diffGoal * pxPerDeg).coerceIn(padX + 4f * density, w - padX - 4f * density)
        goalCaretPaint.color = pal.accent
        lubberPath.reset()
        lubberPath.moveTo(gx, h - 1.2f * density)
        lubberPath.lineTo(gx - 2.8f * density, h - 5.2f * density)
        lubberPath.lineTo(gx + 2.8f * density, h - 5.2f * density)
        lubberPath.close()
        canvas.drawPath(lubberPath, goalCaretPaint)

        canvas.restore()

        val boxH = 18.0f * density
        readoutBgPaint.color = pal.compassReadoutBg
        readoutStrokePaint.color = pal.accentStroke
        readoutTxtPaint.color = if (pal.isLight) pal.accent else pal.secondaryAccent
        lubberPaint.color = if (pal.isLight) pal.accent else pal.secondaryAccent

        badgeRect.set(center - boxW * 0.5f, 2.0f * density, center + boxW * 0.5f, 2.0f * density + boxH)
        canvas.drawRoundRect(badgeRect, 4.5f * density, 4.5f * density, readoutBgPaint)
        canvas.drawRoundRect(badgeRect, 4.5f * density, 4.5f * density, readoutStrokePaint)
        val rFm = readoutTxtPaint.fontMetrics
        val rTextY = badgeRect.centerY() - (rFm.ascent + rFm.descent) * 0.5f
        canvas.drawText(cachedHdgBadgeTxt, center, rTextY, readoutTxtPaint)

        lubberPath.reset()
        lubberPath.moveTo(center, badgeRect.bottom + 4.2f * density)
        lubberPath.lineTo(center - 3.2f * density, badgeRect.bottom + 0.4f * density)
        lubberPath.lineTo(center + 3.2f * density, badgeRect.bottom + 0.4f * density)
        lubberPath.close()
        canvas.drawPath(lubberPath, lubberPaint)
    }

    private fun sOrZero(v: Float): Float = if (v.isNaN()) 0f else v
}
