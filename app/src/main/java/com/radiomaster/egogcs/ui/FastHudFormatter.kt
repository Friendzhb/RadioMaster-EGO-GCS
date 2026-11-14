package com.radiomaster.egogcs.ui

import android.widget.TextView
import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * 零装箱、零 Formatter 正则开销的航空仪表数字格式化器 (`FastHudFormatter`)：
 * - 为每个 TextView 挂载单实例原始整型槽位 `QuantSlot`，在数值显示位未变化时执行纯原始 int 比较并直接 O(1) 返回
 *   （彻底杜绝 Integer.valueOf 装箱、StringBuilder 重建及 TextView 重新排版重绘）；
 * - 仅当四舍五入后的显示位跨过刻度阈值时，通过复用缓冲区拼装紧凑 ASCII 字符串。
 */
object FastHudFormatter {

    private class QuantSlot(
        var a: Int = Int.MIN_VALUE,
        var b: Int = Int.MIN_VALUE
    )

    // 顶字节 0xB9 >= 2，完全符合 Android View.setTag(int, Object) 资源键规范
    private const val TAG_QUANT_SLOT = -0x46801357
    private val sb = StringBuilder(64)

    private fun slotOf(tv: TextView): QuantSlot {
        val existing = tv.getTag(TAG_QUANT_SLOT) as? QuantSlot
        if (existing != null) return existing
        val created = QuantSlot()
        tv.setTag(TAG_QUANT_SLOT, created)
        return created
    }

    fun setFixed1(tv: TextView, prefix: String, value: Float, suffix: String) {
        val q = (value * 10f).roundToInt()
        val s = slotOf(tv)
        if (s.a == q) return
        s.a = q

        sb.setLength(0)
        sb.append(prefix)
        appendScaledInt(sb, q, 1, forcePlus = false)
        sb.append(suffix)
        tv.text = sb.toString()
    }

    fun setFixed2(tv: TextView, prefix: String, value: Float, suffix: String) {
        val q = (value * 100f).roundToInt()
        val s = slotOf(tv)
        if (s.a == q) return
        s.a = q

        sb.setLength(0)
        sb.append(prefix)
        appendScaledInt(sb, q, 2, forcePlus = false)
        sb.append(suffix)
        tv.text = sb.toString()
    }

    fun setSingleInt(tv: TextView, prefix: String, v: Int, suffix: String) {
        val s = slotOf(tv)
        if (s.a == v) return
        s.a = v

        sb.setLength(0)
        sb.append(prefix).append(v).append(suffix)
        tv.text = sb.toString()
    }

    fun setIntMetric(tv: TextView, prefix: String, v1: Int, mid: String, v2: Int, suffix: String) {
        val s = slotOf(tv)
        if (s.a == v1 && s.b == v2) return
        s.a = v1
        s.b = v2

        sb.setLength(0)
        sb.append(prefix).append(v1).append(mid).append(v2).append(suffix)
        tv.text = sb.toString()
    }

    fun setTopHzFps(tv: TextView, isRosMode: Boolean, odomHz: Int, renderFps: Int) {
        val keyA = if (isRosMode) (odomHz and 0xFFFF) or 0x10000 else 0
        val s = slotOf(tv)
        if (s.a == keyA && s.b == renderFps) return
        s.a = keyA
        s.b = renderFps

        sb.setLength(0)
        if (isRosMode) {
            sb.append(odomHz).append("Hz·")
        }
        sb.append(renderFps).append("fps")
        tv.text = sb.toString()
    }

    fun setCompactPoints(tv: TextView, prefix: String, pts: Int, suffix: String) {
        val q = if (pts >= 1000) (pts / 100f).roundToInt() + 1_000_000 else pts
        val s = slotOf(tv)
        if (s.a == q) return
        s.a = q

        sb.setLength(0)
        sb.append(prefix)
        if (pts >= 1000) {
            appendScaledInt(sb, (pts / 100f).roundToInt(), 1, forcePlus = false)
            sb.append('k')
        } else {
            sb.append(pts)
        }
        sb.append(suffix)
        tv.text = sb.toString()
    }

    fun setTopBattery(tv: TextView, volt: Float, amps: Float, txPct: Int, compact: Boolean = false) {
        val qv = (volt * 10f).roundToInt()
        val qa = if (compact) 0 else (amps * 10f).roundToInt()
        val packed = ((if (compact) 1 else 0) shl 28) xor (qv shl 16) xor (qa and 0xFFFF)
        val s = slotOf(tv)
        if (s.a == packed && s.b == txPct) return
        s.a = packed
        s.b = txPct

        sb.setLength(0)
        appendScaledInt(sb, qv, 1, forcePlus = false)
        if (compact) {
            sb.append("V·控").append(txPct).append('%')
        } else {
            sb.append("V·")
            appendScaledInt(sb, qa, 1, forcePlus = false)
            sb.append("A · 控").append(txPct).append('%')
        }
        tv.text = sb.toString()
    }

    fun setSubXyzYaw(tv: TextView, x: Float, y: Float, yawDeg: Float) {
        val qx = (x * 100f).roundToInt()
        val qy = (y * 100f).roundToInt()
        val qYaw = yawDeg.roundToInt()
        val keyA = (qx shl 16) xor (qy and 0xFFFF)
        val s = slotOf(tv)
        if (s.a == keyA && s.b == qYaw) return
        s.a = keyA
        s.b = qYaw

        sb.setLength(0)
        sb.append("X ")
        appendScaledInt(sb, qx, 2, forcePlus = false)
        sb.append("  Y ")
        appendScaledInt(sb, qy, 2, forcePlus = false)
        sb.append("  YAW ").append(qYaw).append('°')
        tv.text = sb.toString()
    }

    fun setPowerDetail(tv: TextView, volt: Float, amps: Float, ampsDecimals: Int, pct: Int) {
        val qv = (volt * 100f).roundToInt()
        val qa = if (ampsDecimals == 2) (amps * 100f).roundToInt() else (amps * 10f).roundToInt()
        val keyA = (qv shl 16) xor (qa and 0xFFFF)
        val s = slotOf(tv)
        if (s.a == keyA && s.b == pct) return
        s.a = keyA
        s.b = pct

        sb.setLength(0)
        appendScaledInt(sb, qv, 2, forcePlus = false)
        sb.append("V · ")
        appendScaledInt(sb, qa, ampsDecimals, forcePlus = false)
        sb.append("A (").append(pct).append("%)")
        tv.text = sb.toString()
    }

    fun setTiltHudLine(
        tv: TextView,
        tiltEnabled: Boolean,
        coordMode: Int,
        s1Deg: Float,
        s2Deg: Float,
        pitchBiasDeg: Float,
        rollDeg: Float = 0f,
        pitchDeg: Float = 0f
    ) {
        val q1 = (s1Deg * 10f).roundToInt()
        val q2 = (s2Deg * 10f).roundToInt()
        val qb = (pitchBiasDeg * 10f).roundToInt()
        val qr = (rollDeg * 10f).roundToInt()
        val qp = (pitchDeg * 10f).roundToInt()
        val keyA = if (!tiltEnabled) {
            (1 shl 29) xor ((qr and 0xFFF) shl 12) xor (qp and 0xFFF)
        } else {
            ((coordMode and 0x7) shl 24) xor ((q1 and 0xFFF) shl 12) xor (q2 and 0xFFF)
        }
        val keyB = if (!tiltEnabled) -888888 else qb
        val s = slotOf(tv)
        if (s.a == keyA && s.b == keyB) return
        s.a = keyA
        s.b = keyB

        sb.setLength(0)
        if (!tiltEnabled) {
            sb.append("RIG·普通X4 R ")
            appendScaledInt(sb, qr, 1, forcePlus = true)
            sb.append("° P ")
            appendScaledInt(sb, qp, 1, forcePlus = true)
            sb.append('°')
            tv.text = sb.toString()
            return
        }
        val mStr = when (coordMode) {
            0 -> "平飞"
            1 -> "协同"
            else -> "锁倾"
        }
        sb.append("TILT·").append(mStr).append(" S1 ")
        appendScaledInt(sb, q1, 1, forcePlus = true)
        sb.append("° S2 ")
        appendScaledInt(sb, q2, 1, forcePlus = true)
        sb.append('°')
        if (coordMode == 1 && abs(qb) >= 5) {
            sb.append(" P偏")
            appendScaledInt(sb, qb, 0, forcePlus = true)
            sb.append('°')
        }
        tv.text = sb.toString()
    }

    fun setRollPitchPair(tv: TextView, rollDeg: Float, pitchDeg: Float) {
        val qr = (rollDeg * 10f).roundToInt()
        val qp = (pitchDeg * 10f).roundToInt()
        val s = slotOf(tv)
        if (s.a == qr && s.b == qp) return
        s.a = qr
        s.b = qp

        sb.setLength(0)
        sb.append("R ")
        appendScaledInt(sb, qr, 1, forcePlus = true)
        sb.append("° · P ")
        appendScaledInt(sb, qp, 1, forcePlus = true)
        sb.append('°')
        tv.text = sb.toString()
    }

    fun setQuat4(tv: TextView, qw: Float, qx: Float, qy: Float, qz: Float) {
        val iw = (qw * 100f).roundToInt()
        val ix = (qx * 100f).roundToInt()
        val iy = (qy * 100f).roundToInt()
        val iz = (qz * 100f).roundToInt()
        val keyA = (iw shl 16) xor (ix and 0xFFFF)
        val keyB = (iy shl 16) xor (iz and 0xFFFF)
        val s = slotOf(tv)
        if (s.a == keyA && s.b == keyB) return
        s.a = keyA
        s.b = keyB

        sb.setLength(0)
        appendScaledInt(sb, iw, 2, forcePlus = false)
        sb.append(',')
        appendScaledInt(sb, ix, 2, forcePlus = false)
        sb.append(',')
        appendScaledInt(sb, iy, 2, forcePlus = false)
        sb.append(',')
        appendScaledInt(sb, iz, 2, forcePlus = false)
        tv.text = sb.toString()
    }

    fun formatSignedTenthsDeg(prefix: String, tenths: Int): String {
        sb.setLength(0)
        sb.append(prefix)
        appendScaledInt(sb, tenths, 1, forcePlus = true)
        sb.append('°')
        return sb.toString()
    }

    fun formatUnsignedTenths(prefix: String, tenths: Int, suffix: String): String {
        sb.setLength(0)
        sb.append(prefix)
        appendScaledInt(sb, abs(tenths), 1, forcePlus = false)
        sb.append(suffix)
        return sb.toString()
    }

    fun formatHeading3Digits(hdgInt: Int): String {
        val clean = ((hdgInt % 360) + 360) % 360
        sb.setLength(0)
        sb.append("Y ")
        if (clean < 100) sb.append('0')
        if (clean < 10) sb.append('0')
        sb.append(clean).append('°')
        return sb.toString()
    }

    fun formatCompassBadge(hdgInt: Int, octant: String): String {
        val clean = ((hdgInt % 360) + 360) % 360
        sb.setLength(0)
        if (clean < 100) sb.append('0')
        if (clean < 10) sb.append('0')
        sb.append(clean).append("° ").append(octant)
        return sb.toString()
    }

    private fun appendScaledInt(out: StringBuilder, scaledVal: Int, decimals: Int, forcePlus: Boolean) {
        if (scaledVal < 0) {
            out.append('-')
        } else if (forcePlus) {
            out.append('+')
        }
        val mag = abs(scaledVal)
        if (decimals == 1) {
            out.append(mag / 10).append('.').append(mag % 10)
        } else {
            val frac = mag % 100
            out.append(mag / 100).append('.')
            if (frac < 10) out.append('0')
            out.append(frac)
        }
    }
}
