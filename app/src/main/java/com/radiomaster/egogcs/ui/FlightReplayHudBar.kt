package com.radiomaster.egogcs.ui

import android.content.Context
import android.graphics.Color
import android.graphics.Typeface
import android.graphics.drawable.GradientDrawable
import android.text.TextUtils
import android.util.TypedValue
import android.view.Gravity
import android.view.View
import android.view.ViewGroup.LayoutParams.MATCH_PARENT
import android.view.ViewGroup.LayoutParams.WRAP_CONTENT
import android.widget.LinearLayout
import android.widget.SeekBar
import android.widget.TextView
import com.radiomaster.egogcs.R
import com.radiomaster.egogcs.db.FlightMissionEntity
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.ui.GcsIcons.Type
import com.radiomaster.egogcs.ui.GcsTheme.Role
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlin.math.roundToInt

/**
 * 3D 飞行轨迹回放底部控制条与抽屉 SQLite 历史架次列表渲染器 (`FlightReplayHudBar`)：
 * - 在回放态悬浮于主座舱底部中央，支持播放/暂停、1000 级平滑进度条拖动擦除、`0.5x~4x` 倍速切换、一键打开战后分析及退出回放；
 * - 统一遵循航空玻璃毛玻璃视觉规范与无障碍最小触控区标准。
 */
class FlightReplayHudBar(
    private val context: Context,
    private val hub: GcsSharedHub,
    private val onTogglePlayPause: () -> Unit,
    private val onSeekProgress: (Float) -> Unit,
    private val onCycleSpeed: () -> Unit,
    private val onOpenCurrentAnalysis: () -> Unit,
    private val onExitReplay: () -> Unit
) {
    private val d = context.resources.displayMetrics.density
    private fun dp(v: Float): Int = (v * d).roundToInt()

    lateinit var rootView: LinearLayout
        private set

    private lateinit var tvTitleBadge: TextView
    private lateinit var tvTimeReadout: TextView
    private lateinit var btnPlayPause: TextView
    private lateinit var seekBar: SeekBar
    private lateinit var btnSpeed: TextView

    private var userScrubbing = false
    private var lastRenderedPlaying: Boolean? = null
    private var lastRenderedSpeed = -1f
    private val wallTimeFmt = SimpleDateFormat("HH:mm:ss.S", Locale.US)

    fun buildView(): LinearLayout {
        val bar = TouchChainGuard.IconSlopLinearLayout(
            context = context,
            containerMode = TouchChainGuard.ContainerMode.SOLID_BAR,
            density = d,
            activePredicate = { hub.isReplayMode }
        ).apply {
            tag = "flight_replay_hud_bar"
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER_HORIZONTAL
            clipChildren = false
            clipToPadding = false
            setPadding(dp(10f), dp(6f), dp(10f), dp(6f))
            visibility = View.GONE
            isClickable = true
            GcsTheme.markRole(this, Role.GLASS_PANEL, cornerDp = 10f)
        }

        val topRow = LinearLayout(context).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }
        tvTitleBadge = TextView(context).apply {
            text = "3D 轨迹回放"
            maxLines = 1
            ellipsize = TextUtils.TruncateAt.END
            maxWidth = dp(215f)
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 9.8f)
            typeface = Typeface.DEFAULT_BOLD
            GcsTheme.markRole(this, Role.TEXT_ACCENT)
        }
        tvTimeReadout = TextView(context).apply {
            text = "00:00.0 / 00:00.0"
            maxLines = 1
            gravity = Gravity.END
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 9.6f)
            GcsTheme.applyTabularReadout(this, bold = true)
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
        }
        topRow.addView(tvTitleBadge, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f))
        topRow.addView(tvTimeReadout, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(8f) })
        bar.addView(topRow, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT))

        val ctrlRow = LinearLayout(context).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }

        btnPlayPause = createTransportBtn(
            testTag = "replay_play_pause_btn",
            desc = context.getString(R.string.flight_replay_play_pause_desc),
            icon = Type.PAUSE,
            label = "暂停",
            accentFill = Color.parseColor("#00B865")
        ) {
            onTogglePlayPause()
        }

        seekBar = SeekBar(context).apply {
            tag = "replay_timeline_seekbar"
            contentDescription = context.getString(R.string.flight_replay_seek_desc)
            max = 1000
            progress = 0
            minimumHeight = dp(32f)
            setPadding(dp(8f), dp(4f), dp(8f), dp(4f))
            setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
                override fun onProgressChanged(sb: SeekBar?, progress: Int, fromUser: Boolean) {
                    if (fromUser) {
                        onSeekProgress(progress / 1000f)
                    }
                }

                override fun onStartTrackingTouch(sb: SeekBar?) {
                    userScrubbing = true
                }

                override fun onStopTrackingTouch(sb: SeekBar?) {
                    userScrubbing = false
                    if (sb != null) {
                        onSeekProgress(sb.progress / 1000f)
                    }
                }
            })
        }

        btnSpeed = createTransportBtn(
            testTag = "replay_speed_btn",
            desc = context.getString(R.string.flight_replay_speed_desc),
            icon = null,
            label = "1.0x",
            accentFill = null
        ) {
            onCycleSpeed()
        }

        val btnAnalyze = createTransportBtn(
            testTag = "replay_open_analysis_btn",
            desc = context.getString(R.string.flight_replay_analysis_desc),
            icon = Type.TELEMETRY_PULSE,
            label = "分析",
            accentFill = null
        ) {
            onOpenCurrentAnalysis()
        }

        val btnExit = createTransportBtn(
            testTag = "replay_exit_btn",
            desc = context.getString(R.string.flight_replay_exit_desc),
            icon = Type.CLOSE_X,
            label = "退出",
            accentFill = Color.parseColor("#FF3B30")
        ) {
            onExitReplay()
        }

        ctrlRow.addView(btnPlayPause)
        ctrlRow.addView(seekBar, LinearLayout.LayoutParams(dp(146f), WRAP_CONTENT).apply { leftMargin = dp(4f) })
        ctrlRow.addView(btnSpeed, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(4f) })
        ctrlRow.addView(btnAnalyze, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(4f) })
        ctrlRow.addView(btnExit, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { leftMargin = dp(4f) })
        bar.addView(ctrlRow, LinearLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT).apply { topMargin = dp(4f) })

        rootView = bar
        return bar
    }

    private fun createTransportBtn(
        testTag: String,
        desc: String,
        icon: Type?,
        label: String,
        accentFill: Int?,
        onClick: () -> Unit
    ): TextView = TextView(context).apply {
        tag = testTag
        contentDescription = desc
        gravity = Gravity.CENTER
        maxLines = 1
        minimumHeight = dp(30f)
        minimumWidth = dp(42f)
        setPadding(dp(7f), dp(4f), dp(7f), dp(4f))
        setTextSize(TypedValue.COMPLEX_UNIT_SP, 9.6f)
        typeface = Typeface.DEFAULT_BOLD
        if (accentFill != null) {
            setTextColor(Color.WHITE)
            background = GradientDrawable().apply {
                setColor(accentFill)
                cornerRadius = dp(5.5f).toFloat()
            }
        } else {
            GcsTheme.markRole(this, Role.TEXT_PRIMARY)
            GcsTheme.markRole(this, Role.WELL_TRACK, cornerDp = 5.5f)
        }
        if (icon != null) {
            GcsIcons.bind(this, icon, label, d, iconDp = 10.5f, gapDp = 3f)
        } else {
            text = label
        }
        setOnClickListener {
            AnimHelper.triggerHaptic(it)
            onClick()
        }
        AnimHelper.attachSpringTouch(this)
    }

    fun syncOnHudTick(): Boolean {
        if (!::rootView.isInitialized) return false
        val active = hub.isReplayMode
        val visibilityChanged = (rootView.visibility == View.VISIBLE) != active
        if (!active) {
            if (visibilityChanged) rootView.visibility = View.GONE
            lastRenderedPlaying = null
            return visibilityChanged
        }
        if (visibilityChanged) {
            rootView.visibility = View.VISIBLE
            GcsTheme.applyTree(rootView, GcsTheme.of(hub.themeId), d)
        }

        val title = hub.replayMissionTitle.ifBlank { "架次 #${hub.replayMissionId}" }
        val wallStr = if (hub.replayWallTimestampMs > 0L) {
            " · " + wallTimeFmt.format(Date(hub.replayWallTimestampMs))
        } else {
            ""
        }
        val titleLine = "回放 · $title$wallStr"
        if (tvTitleBadge.text.toString() != titleLine) {
            tvTitleBadge.text = titleLine
        }

        val elMs = hub.replayElapsedMs
        val totMs = hub.replayTotalMs.coerceAtLeast(1L)
        val timeText = "${formatMinSecTenths(elMs)} / ${formatMinSecTenths(hub.replayTotalMs)}"
        if (tvTimeReadout.text.toString() != timeText) {
            tvTimeReadout.text = timeText
        }

        if (!userScrubbing) {
            val targetProg = ((elMs * 1000L) / totMs).toInt().coerceIn(0, 1000)
            if (seekBar.progress != targetProg) {
                seekBar.progress = targetProg
            }
        }

        val playing = hub.replayPlaying
        if (lastRenderedPlaying != playing) {
            lastRenderedPlaying = playing
            GcsIcons.bind(
                btnPlayPause,
                if (playing) Type.PAUSE else Type.PLAY,
                if (playing) "暂停" else "播放",
                d,
                iconDp = 10.5f,
                gapDp = 3f
            )
            btnPlayPause.background = GradientDrawable().apply {
                setColor(if (playing) Color.parseColor("#FFB300") else Color.parseColor("#00B865"))
                cornerRadius = dp(5.5f).toFloat()
            }
        }

        val spd = hub.replaySpeed
        if (lastRenderedSpeed != spd) {
            lastRenderedSpeed = spd
            btnSpeed.text = String.format(Locale.US, "%.1fx", spd)
        }
        return visibilityChanged
    }

    companion object {
        fun formatMinSecTenths(ms: Long): String {
            val clamped = ms.coerceAtLeast(0L)
            val totalTenths = clamped / 100L
            val tenths = totalTenths % 10L
            val totalSec = totalTenths / 10L
            val sec = totalSec % 60L
            val min = totalSec / 60L
            return String.format(Locale.US, "%02d:%02d.%d", min, sec, tenths)
        }

        /**
         * 在状态抽屉内渲染本地 SQLite 历史飞行任务卡片列表，支持「3D回放 | 分析 | CSV | 删除」。
         */
        fun populateDrawerMissionList(
            context: Context,
            container: LinearLayout,
            missions: List<FlightMissionEntity>,
            activeReplayMissionId: Long,
            hub: GcsSharedHub,
            density: Float,
            onReplay: (FlightMissionEntity) -> Unit,
            onAnalyze: (FlightMissionEntity) -> Unit,
            onExportCsv: (FlightMissionEntity) -> Unit,
            onDelete: (FlightMissionEntity) -> Unit
        ) {
            fun dpLocal(v: Float): Int = (v * density).roundToInt()
            val pal = GcsTheme.of(hub.themeId)
            container.removeAllViews()

            if (missions.isEmpty()) {
                val emptyTv = TextView(context).apply {
                    tag = "flight_db_empty_state"
                    text = context.getString(R.string.flight_db_empty_hint)
                    setTextSize(TypedValue.COMPLEX_UNIT_SP, 9.6f)
                    setTextColor(pal.textMuted)
                    setPadding(dpLocal(6f), dpLocal(6f), dpLocal(6f), dpLocal(6f))
                }
                container.addView(emptyTv)
                return
            }

            val maxShown = missions.size.coerceAtMost(15)
            for (i in 0 until maxShown) {
                val m = missions[i]
                val isReplayingThis = hub.isReplayMode && activeReplayMissionId == m.id
                val itemCard = LinearLayout(context).apply {
                    tag = "mission_history_card_${m.id}"
                    orientation = LinearLayout.VERTICAL
                    setPadding(dpLocal(7.5f), dpLocal(6f), dpLocal(7.5f), dpLocal(6f))
                    background = GradientDrawable().apply {
                        setColor(if (isReplayingThis) pal.accentSoftFill else pal.wellFill)
                        cornerRadius = dpLocal(6.5f).toFloat()
                        setStroke(1, if (isReplayingThis) pal.accent else pal.cardStroke)
                    }
                }

                val titleTv = TextView(context).apply {
                    val badge = if (isReplayingThis) "[回放中] " else ""
                    text = "$badge${m.title} (${m.linkMode})"
                    maxLines = 1
                    ellipsize = TextUtils.TruncateAt.END
                    setTextSize(TypedValue.COMPLEX_UNIT_SP, 10.0f)
                    typeface = Typeface.DEFAULT_BOLD
                    setTextColor(if (isReplayingThis) pal.accentBright else pal.textPrimary)
                }

                val durSec = m.durationMs * 0.001f
                val statsTv = TextView(context).apply {
                    text = String.format(
                        Locale.US,
                        "时长 %.1fs (%d帧) · 航程 %.1fm · 极速 %.1fm/s · 最低净空 %.2fm",
                        durSec,
                        m.sampleCount,
                        m.distanceMeters,
                        m.maxSpeedMps,
                        m.minClearanceM
                    )
                    maxLines = 2
                    setTextSize(TypedValue.COMPLEX_UNIT_SP, 8.8f)
                    typeface = Typeface.MONOSPACE
                    setTextColor(pal.textSecondary)
                }

                val actRow = LinearLayout(context).apply {
                    orientation = LinearLayout.HORIZONTAL
                    gravity = Gravity.CENTER_VERTICAL
                }

                fun miniActionPill(
                    tagStr: String,
                    icon: Type,
                    label: String,
                    highlightColor: Int?,
                    click: () -> Unit
                ): TextView = TextView(context).apply {
                    tag = tagStr
                    contentDescription = "$label ${m.title}"
                    gravity = Gravity.CENTER
                    maxLines = 1
                    minimumHeight = dpLocal(28f)
                    setPadding(dpLocal(5f), dpLocal(3.5f), dpLocal(5f), dpLocal(3.5f))
                    setTextSize(TypedValue.COMPLEX_UNIT_SP, 9.0f)
                    typeface = Typeface.DEFAULT_BOLD
                    if (highlightColor != null) {
                        setTextColor(Color.WHITE)
                        background = GradientDrawable().apply {
                            setColor(highlightColor)
                            cornerRadius = dpLocal(5f).toFloat()
                        }
                    } else {
                        setTextColor(pal.textPrimary)
                        background = GradientDrawable().apply {
                            setColor(pal.glassFill)
                            cornerRadius = dpLocal(5f).toFloat()
                            setStroke(1, pal.cardStroke)
                        }
                    }
                    GcsIcons.bind(this, icon, label, density, iconDp = 9.5f, gapDp = 2.5f)
                    setOnClickListener {
                        AnimHelper.triggerHaptic(it)
                        click()
                    }
                    AnimHelper.attachSpringTouch(this)
                }

                val btnPlay = miniActionPill(
                    "mission_replay_btn_${m.id}",
                    Type.PLAY,
                    "3D回放",
                    Color.parseColor("#00B865")
                ) { onReplay(m) }
                val btnStat = miniActionPill(
                    "mission_analyze_btn_${m.id}",
                    Type.TELEMETRY_PULSE,
                    "分析",
                    null
                ) { onAnalyze(m) }
                val btnCsv = miniActionPill(
                    "mission_csv_btn_${m.id}",
                    Type.DOWNLOAD,
                    "CSV",
                    null
                ) { onExportCsv(m) }
                val btnDel = miniActionPill(
                    "mission_delete_btn_${m.id}",
                    Type.TRASH,
                    "删除",
                    null
                ) { onDelete(m) }

                actRow.addView(btnPlay, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1.25f))
                actRow.addView(btnStat, LinearLayout.LayoutParams(0, WRAP_CONTENT, 1f).apply { leftMargin = dpLocal(4f) })
                actRow.addView(btnCsv, LinearLayout.LayoutParams(0, WRAP_CONTENT, 0.95f).apply { leftMargin = dpLocal(4f) })
                actRow.addView(btnDel, LinearLayout.LayoutParams(0, WRAP_CONTENT, 0.95f).apply { leftMargin = dpLocal(4f) })

                itemCard.addView(titleTv)
                itemCard.addView(statsTv, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dpLocal(2f) })
                itemCard.addView(actRow, LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply { topMargin = dpLocal(5f) })

                container.addView(
                    itemCard,
                    LinearLayout.LayoutParams(MATCH_PARENT, WRAP_CONTENT).apply {
                        if (i > 0) topMargin = dpLocal(5.5f)
                    }
                )
            }
        }
    }
}
