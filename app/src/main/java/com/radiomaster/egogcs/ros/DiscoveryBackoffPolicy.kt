package com.radiomaster.egogcs.ros

import java.util.Locale
import java.util.concurrent.ThreadLocalRandom
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.roundToLong

/**
 * 触发自动寻机或链路自愈的原因分类：
 * 用于区分冷启动、空扫指数退避、机载电脑已通电但等待 `roscore/rosbridge` 就绪、桥接意外断链、以及物理网卡恢复。
 */
enum class DiscoveryRecoveryReason {
    INITIAL_STARTUP,
    EMPTY_SCAN_BACKOFF,
    HOST_BOOTING_ROSCORE,
    BRIDGE_DISCONNECTED,
    LAN_LINK_RESTORED,
    MANUAL_TRIGGER
}

/**
 * 线程安全的有界抖动指数退避策略 (`ExponentialBackoffPolicy`)：
 *
 * - 计算公式：`delay = clamp(baseDelayMs * multiplier^(attempt - 1) * (1 + jitterRatio * rand[-1, 1]), minDelayMs, maxDelayMs)`
 * - 引入对称随机抖动（默认 ±15%），防止多台地面站平板在同一局域网热点断链恢复时发生 mDNS / ARP 惊群效应；
 * - 支持通过 [jitterSource] 注入确定性随机数，便于单元测试精确断言每次退避窗口；
 * - 在物理 Wi-Fi 重连或成功发现可用无人机后调用 [reset] 立即清零退避阶数，保证下一次瞬断能够毫秒~秒级快速自愈。
 */
class ExponentialBackoffPolicy(
    val baseDelayMs: Long = 2_000L,
    val maxDelayMs: Long = 30_000L,
    val multiplier: Double = 2.0,
    val jitterRatio: Double = 0.15,
    val minDelayMs: Long = 400L,
    val maxAttempts: Int = Int.MAX_VALUE,
    private val jitterSource: () -> Double = { ThreadLocalRandom.current().nextDouble(-1.0, 1.0) }
) {
    init {
        require(baseDelayMs > 0L) { "baseDelayMs must be positive" }
        require(maxDelayMs >= baseDelayMs) { "maxDelayMs must be >= baseDelayMs" }
        require(multiplier >= 1.0) { "multiplier must be >= 1.0" }
        require(jitterRatio in 0.0..0.5) { "jitterRatio must be in 0.0..0.5" }
    }

    private val attempts = AtomicInteger(0)

    val attemptCount: Int
        get() = attempts.get()

    fun hasBudgetRemaining(): Boolean = attempts.get() < maxAttempts

    /**
     * 递增尝试次数并返回下一次等待毫秒数；若已超出 [maxAttempts] 则返回 `-1L`。
     */
    fun nextDelayMs(): Long {
        while (true) {
            val cur = attempts.get()
            if (cur >= maxAttempts) return -1L
            val next = cur + 1
            if (attempts.compareAndSet(cur, next)) {
                return computeDelayForAttempt(next, applyJitter = true)
            }
        }
    }

    /**
     * 无副作用地预览第 [forAttempt] 次（从 1 开始）的无抖动基准退避时长。
     */
    fun peekBaseDelayMs(forAttempt: Int): Long {
        if (forAttempt <= 0) return 0L
        return computeDelayForAttempt(forAttempt, applyJitter = false)
    }

    fun computeDelayForAttempt(attempt: Int, applyJitter: Boolean = true): Long {
        val clampedExp = (attempt - 1).coerceIn(0, 20)
        val raw = (baseDelayMs.toDouble() * multiplier.pow(clampedExp.toDouble()))
            .coerceAtMost(maxDelayMs.toDouble())
        val factor = if (applyJitter && jitterRatio > 0.0) {
            1.0 + jitterRatio * jitterSource().coerceIn(-1.0, 1.0)
        } else {
            1.0
        }
        val effectiveMin = min(minDelayMs, baseDelayMs)
        return (raw * factor).roundToLong().coerceIn(effectiveMin, maxDelayMs)
    }

    fun reset() {
        attempts.set(0)
    }

    companion object {
        fun formatSecondsShort(delayMs: Long): String =
            String.format(Locale.US, "%.1fs", delayMs.coerceAtLeast(0L) / 1000.0)
    }
}
