package com.radiomaster.egogcs.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.LinearGradient
import android.graphics.Paint
import android.graphics.Path
import android.graphics.Shader
import android.graphics.Typeface
import android.graphics.drawable.GradientDrawable
import android.text.TextUtils
import android.util.TypedValue
import android.view.Gravity
import android.view.View
import android.view.ViewGroup.LayoutParams.MATCH_PARENT
import android.view.ViewGroup.LayoutParams.WRAP_CONTENT
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import com.radiomaster.egogcs.R
import com.radiomaster.egogcs.db.MissionAnalysisReport
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.ui.GcsIcons.Type
import com.radiomaster.egogcs.ui.GcsTheme.Role
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * 飞行任务战后多维分析浮层面板 (`PostFlightAnalysisSheet`)：
 * 从本地 SQLite 数据库聚合单架次时序遥测点，绘制三通道归一化时序剖面图 (高度 Z / 地速 V / 障碍净空 Clr)
 * 及 8 宫格关键性能与电池能耗指标，支持一键跳转 3D 实景轨迹回放或导出科研 CSV。
 */
class PostFlightAnalysisSheet(
    private val context: Context,
    private val hub: GcsSharedHub,
    private val onStartReplay: (Long) -> Unit,
    private val onExportCsv: (Long) -> Unit,
    private val onVisibilityChanged: ((Boolean) -> Unit)? = null
) {
    private val d = context.resources.displayMetrics.density
    private fun dp(v: Float): Int = (v * d).roundToInt()

    var isOpen: Boolean = false
        private set

    private var currentMissionId: Long = 0L
    private lateinit var scrimRoot: TouchChainGuard.IconSlopFrameLayout
    private lateinit var cardWrap: LinearLayout
    private lateinit var tvHeaderTitle: TextView
    private lateinit var tvHeaderMeta: TextView
    private lateinit var profileChart: FlightProfileChartView
    private val metricValueViews = ArrayList<TextView>(8)

    fun buildView(): View {
        val dm = context.resources.displayMetrics
        val modalW = min((dm.widthPixels * 0.92f).roundToInt(), dp(660f))
        val modalH = min((dm.heightPixels * 0.90f).roundToInt(), dp(420f))

        scrimRoot = TouchChainGuard.IconSlopFrameLayout(
            context = context,
            containerMode = TouchChainGuard.ContainerMode.MODAL_SCRIM,
            density = d,
            activePredicate = { isOpen }
        ).apply {
            visibility = View.GONE
            isClickable = true
            isFocusable = true
            setBackgroundColor(Color.argb(176, 4, 8, 15))
            setOnClickListener { setVisible(false) }
        }

        cardWrap = LinearLayout(context).apply {
            orientation = LinearLayout.VERTICAL
            isClickable = true
            GcsTheme.markRole(this, Role.MODAL_CARD, cornerDp = 12f)
        }

        val header = LinearLayout(context).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
            setPadding(dp(12f), dp(8f), dp(10f), dp(8f))
            GcsTheme.markRole(this, Role.MODAL_HEADER)
        }
        val titleCol = LinearLayout(context).apply { orientation = LinearLayout.VERTICAL }
        tvHeaderTitle = TextView(context).apply {
            text = "战后任务遥测分析"
            maxLines = 1
            ellipsize = TextUtils.TruncateAt.END
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 12.5f)
            typeface = Typeface.DEFAULT_BOLD
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
        }
        tvHeaderMeta = TextView(context).apply {
            text = "--"
            maxLines = 1
            ellipsize = TextUtils.TruncateAt.END
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 9.6f)
            typeface = Typeface.MONOSPACE
            GcsTheme.markRole(this, Role.TEXT_MUTED)
        }
        titleCol.addView(tvHeaderTitle)
        titleCol.addView(tvHeaderMeta)
        header.addView(titleCol, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))

        val btnClose = TextView(context).apply {
            tag = "flight_analysis_close_btn"
            contentDescription = context.getString(R.string.flight_replay_exit_desc)
            gravity = Gravity.CENTER
            minimumWidth = dp(48f)
            minimumHeight = dp(34f)
            setPadding(dp(8f), dp(4f), dp(8f), dp(4f))
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 10.5f)
            typeface = Typeface.DEFAULT_BOLD
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
            GcsTheme.markRole(this, Role.GLASS_PANEL, cornerDp = 6f)
            GcsIcons.bind(this, Type.CLOSE_X, "关闭", d, iconDp = 11f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                setVisible(false)
            }
        }
        AnimHelper.attachSpringTouch(btnClose)
        header.addView(btnClose)
        cardWrap.addView(header, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT))

        val bodyScroll = ScrollView(context).apply {
            isFillViewport = true
            overScrollMode = View.OVER_SCROLL_NEVER
        }
        val bodyCol = LinearLayout(context).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(12f), dp(8f), dp(12f), dp(8f))
        }

        profileChart = FlightProfileChartView(context, d)
        GcsTheme.markRole(profileChart, Role.CONSOLE_BOX, cornerDp = 8f)
        bodyCol.addView(profileChart, LinearLayout.LayoutParams(MATCH_PARENT, dp(104f)))

        val labels = arrayOf(
            "飞行时长 / 采样帧",
            "三维总航程",
            "峰值 / 均速",
            "最大高度 / 爬升率",
            "最低净空 / 预警帧",
            "最大滚俯 / 舵机倾角",
            "起止电压 / 最大压降",
            "峰值功率 / 估算能耗"
        )
        metricValueViews.clear()
        for (rowIdx in 0 until 2) {
            val row = LinearLayout(context).apply {
                orientation = LinearLayout.HORIZONTAL
            }
            for (colIdx in 0 until 4) {
                val cellIdx = rowIdx * 4 + colIdx
                val cell = LinearLayout(context).apply {
                    orientation = LinearLayout.VERTICAL
                    setPadding(dp(7f), dp(5f), dp(7f), dp(5f))
                    GcsTheme.markRole(this, Role.METRIC_CELL, cornerDp = 6f)
                }
                val lbl = TextView(context).apply {
                    text = labels[cellIdx]
                    maxLines = 1
                    ellipsize = TextUtils.TruncateAt.END
                    setTextSize(TypedValue.COMPLEX_UNIT_SP, 8.5f)
                    GcsTheme.markRole(this, Role.TEXT_MUTED)
                }
                val valTv = TextView(context).apply {
                    text = "--"
                    maxLines = 2
                    setTextSize(TypedValue.COMPLEX_UNIT_SP, 10.2f)
                    GcsTheme.applyTabularReadout(this, bold = true)
                    GcsTheme.markRole(this, Role.TEXT_PRIMARY)
                }
                metricValueViews.add(valTv)
                cell.addView(lbl)
                cell.addView(valTv, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(2f) })
                row.addView(
                    cell,
                    LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply {
                        if (colIdx > 0) leftMargin = dp(5f)
                    }
                )
            }
            bodyCol.addView(row, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dp(6f) })
        }

        bodyScroll.addView(bodyCol)
        cardWrap.addView(bodyScroll, LinearLayout.LayoutParams(MATCH_PARENT, 0, 1f))

        val footer = LinearLayout(context).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.END or Gravity.CENTER_VERTICAL
            setPadding(dp(12f), dp(7f), dp(12f), dp(8f))
            GcsTheme.markRole(this, Role.MODAL_HEADER)
        }

        val btnCsv = TextView(context).apply {
            tag = "flight_analysis_csv_btn"
            contentDescription = context.getString(R.string.flight_db_btn_export_csv)
            gravity = Gravity.CENTER
            minimumHeight = dp(36f)
            minimumWidth = dp(96f)
            setPadding(dp(12f), dp(6f), dp(12f), dp(6f))
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 10.5f)
            typeface = Typeface.DEFAULT_BOLD
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
            GcsTheme.markRole(this, Role.GLASS_PANEL, cornerDp = 6.5f)
            GcsIcons.bind(this, Type.DOWNLOAD, "导出科研 CSV", d, iconDp = 11.5f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                if (currentMissionId > 0L) onExportCsv(currentMissionId)
            }
        }
        AnimHelper.attachSpringTouch(btnCsv)

        val btnReplay = TextView(context).apply {
            tag = "flight_analysis_replay_btn"
            contentDescription = "进入三维轨迹回放"
            gravity = Gravity.CENTER
            minimumHeight = dp(36f)
            minimumWidth = dp(132f)
            setPadding(dp(14f), dp(6f), dp(14f), dp(6f))
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 10.8f)
            typeface = Typeface.DEFAULT_BOLD
            setTextColor(Color.WHITE)
            background = GradientDrawable().apply {
                setColor(Color.parseColor("#00B865"))
                cornerRadius = dp(6.5f).toFloat()
            }
            GcsIcons.bind(this, Type.PLAY, "进入 3D 轨迹回放", d, iconDp = 11.5f)
            setOnClickListener {
                AnimHelper.triggerHaptic(it)
                val id = currentMissionId
                setVisible(false)
                if (id > 0L) onStartReplay(id)
            }
        }
        AnimHelper.attachSpringTouch(btnReplay)

        footer.addView(btnCsv)
        footer.addView(btnReplay, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(8f) })
        cardWrap.addView(footer, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT))

        scrimRoot.addView(cardWrap, FrameLayout.LayoutParams(modalW, modalH, Gravity.CENTER))
        return scrimRoot
    }

    fun showReport(report: MissionAnalysisReport) {
        currentMissionId = report.mission.id
        val startStr = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(Date(report.mission.startedAtEpochMs))
        val rigLabel = if (report.mission.airframeRig == "TILT_4M2S") "双舵倾转 4M2S" else "常规固定 X4"
        tvHeaderTitle.text = "战后分析 · ${report.mission.title}"
        tvHeaderMeta.text = "${report.mission.linkMode} 链路 · $rigLabel · 集群x${report.mission.swarmCount} · 起飞时间 $startStr"

        val durSec = report.durationMs * 0.001f
        metricValueViews[0].text = String.format(Locale.US, "%.1fs · %d帧 (%.1fHz)", durSec, report.sampleCount, report.effectiveHz)
        metricValueViews[1].text = String.format(Locale.US, "%.2f m", report.distance3dMeters)
        metricValueViews[2].text = String.format(Locale.US, "%.2f / %.2f m/s", report.maxGroundSpeedMps, report.avgGroundSpeedMps)
        metricValueViews[3].text = String.format(Locale.US, "%.2fm / %.2fm/s", report.maxAltitudeM, report.maxClimbRateMps)
        metricValueViews[4].text = String.format(Locale.US, "%.2fm (%d帧预警)", report.minClearanceM, report.nearObstacleWarnFrames)
        metricValueViews[5].text = String.format(
            Locale.US,
            "R%.1f° P%.1f° / S%.1f°",
            report.maxAbsRollDeg,
            report.maxAbsPitchDeg,
            report.maxAbsTiltDeg
        )
        metricValueViews[6].text = String.format(
            Locale.US,
            "%.2fV→%.2fV (-%.2fV)",
            report.startVolt,
            report.endVolt,
            report.voltageSagV
        )
        metricValueViews[7].text = String.format(
            Locale.US,
            "%.0fW · %.0fmAh (%.2fWh)",
            report.peakPowerWatts,
            report.consumedMah,
            report.consumedWh
        )

        profileChart.bindReport(report, hub.warnClearanceMeters)
        GcsTheme.applyTree(scrimRoot, GcsTheme.of(hub.themeId), d)
        setVisible(true)
    }

    fun setVisible(open: Boolean) {
        if (isOpen == open) return
        isOpen = open
        if (open) {
            scrimRoot.visibility = View.VISIBLE
            cardWrap.alpha = 0f
            cardWrap.scaleX = 0.94f
            cardWrap.scaleY = 0.94f
            cardWrap.animate().alpha(1f).scaleX(1f).scaleY(1f).setDuration(180L).start()
        } else {
            scrimRoot.visibility = View.GONE
        }
        onVisibilityChanged?.invoke(open)
    }

    private class FlightProfileChartView(context: Context, private val density: Float) : View(context) {
        private var altArr = FloatArray(0)
        private var spdArr = FloatArray(0)
        private var clrArr = FloatArray(0)
        private var warnThresholdM = 0.65f

        private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeWidth = density * 1f
            color = Color.argb(42, 160, 185, 215)
        }
        private val warnPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeWidth = density * 1.1f
            color = Color.argb(150, 255, 82, 82)
            pathEffect = DashPathEffect(floatArrayOf(density * 4f, density * 3f), 0f)
        }
        private val altPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeWidth = density * 1.8f
            color = Color.parseColor("#29B6F6")
        }
        private val spdPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeWidth = density * 1.8f
            color = Color.parseColor("#00E676")
        }
        private val clrPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.STROKE
            strokeWidth = density * 1.6f
            color = Color.parseColor("#FFB300")
        }
        private val fillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            style = Paint.Style.FILL
        }
        private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            textSize = density * 8.8f
            typeface = Typeface.MONOSPACE
            color = Color.argb(215, 200, 214, 232)
        }
        private val linePath = Path()
        private val areaPath = Path()

        fun bindReport(report: MissionAnalysisReport, warnClr: Float) {
            altArr = report.altitudeProfileM
            spdArr = report.speedProfileMps
            clrArr = report.clearanceProfileM
            warnThresholdM = warnClr
            invalidate()
        }

        override fun onDraw(canvas: Canvas) {
            super.onDraw(canvas)
            val w = width.toFloat()
            val h = height.toFloat()
            if (w <= 10f || h <= 10f) return

            val padL = density * 10f
            val padR = density * 10f
            val padT = density * 20f
            val padB = density * 10f
            val plotW = (w - padL - padR).coerceAtLeast(1f)
            val plotH = (h - padT - padB).coerceAtLeast(1f)

            // 水平参考网格线
            for (g in 0..2) {
                val gy = padT + plotH * (g / 2f)
                canvas.drawLine(padL, gy, padL + plotW, gy, gridPaint)
            }

            // 顶部三通道图例
            labelPaint.color = Color.parseColor("#29B6F6")
            canvas.drawText("■ 高度 Z(m)", padL, density * 13f, labelPaint)
            labelPaint.color = Color.parseColor("#00E676")
            canvas.drawText("■ 地速 V(m/s)", padL + density * 78f, density * 13f, labelPaint)
            labelPaint.color = Color.parseColor("#FFB300")
            canvas.drawText("■ 障碍净空 Clr(m)", padL + density * 168f, density * 13f, labelPaint)

            if (altArr.size < 2) {
                labelPaint.color = Color.argb(160, 180, 195, 215)
                canvas.drawText("采样点不足，无法绘制时序曲线", padL + plotW * 0.25f, padT + plotH * 0.55f, labelPaint)
                return
            }

            var maxScale = 2.5f
            for (v in altArr) maxScale = max(maxScale, v)
            for (v in spdArr) maxScale = max(maxScale, v)
            for (v in clrArr) maxScale = max(maxScale, min(v, 5.0f))

            val warnY = padT + plotH * (1f - (warnThresholdM / maxScale).coerceIn(0f, 1f))
            canvas.drawLine(padL, warnY, padL + plotW, warnY, warnPaint)

            drawSeries(canvas, spdArr, maxScale, padL, padT, plotW, plotH, spdPaint, fillAlphaColor = Color.argb(44, 0, 230, 118))
            drawSeries(canvas, altArr, maxScale, padL, padT, plotW, plotH, altPaint, fillAlphaColor = 0)
            drawSeries(canvas, clrArr, maxScale, padL, padT, plotW, plotH, clrPaint, fillAlphaColor = 0, clampCap = 5.0f)
        }

        private fun drawSeries(
            canvas: Canvas,
            arr: FloatArray,
            maxVal: Float,
            padL: Float,
            padT: Float,
            plotW: Float,
            plotH: Float,
            stroke: Paint,
            fillAlphaColor: Int,
            clampCap: Float = Float.MAX_VALUE
        ) {
            val n = arr.size
            if (n < 2) return
            linePath.reset()
            areaPath.reset()
            for (i in 0 until n) {
                val x = padL + plotW * (i.toFloat() / (n - 1).toFloat())
                val v = min(arr[i], clampCap)
                val norm = (v / maxVal).coerceIn(0f, 1f)
                val y = padT + plotH * (1f - norm)
                if (i == 0) {
                    linePath.moveTo(x, y)
                    areaPath.moveTo(x, padT + plotH)
                    areaPath.lineTo(x, y)
                } else {
                    linePath.lineTo(x, y)
                    areaPath.lineTo(x, y)
                }
            }
            if (fillAlphaColor != 0) {
                areaPath.lineTo(padL + plotW, padT + plotH)
                areaPath.close()
                fillPaint.shader = LinearGradient(
                    0f, padT, 0f, padT + plotH,
                    fillAlphaColor, Color.TRANSPARENT, Shader.TileMode.CLAMP
                )
                canvas.drawPath(areaPath, fillPaint)
            }
            canvas.drawPath(linePath, stroke)
        }
    }
}
