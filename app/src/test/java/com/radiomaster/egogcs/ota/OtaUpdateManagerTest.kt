package com.radiomaster.egogcs.ota

import java.io.File
import java.nio.file.Files
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

class OtaUpdateManagerTest {

    private val sampleShaA = "a".repeat(64)
    private val sampleShaB = "b".repeat(64)

    @Test
    fun isRemoteReleaseNewer_returnsFalseWhenInstalledApkShaMatchesRemote() {
        // 即使 GitHub Release 发布时间比 APK 编译时间晚 15 分钟（等待模拟器测试），相同 SHA-256 绝不误报更新
        val meta = OtaUpdateManager.ReleaseMeta(
            sizeBytes = 24_500_000L,
            updatedEpochMs = 1_763_200_000_000L + 15 * 60_000L,
            versionCode = 0L,
            versionName = "3.12.0",
            commitSha = "9f8e7d6c5b4a",
            sha256 = sampleShaA
        )

        val newer = OtaUpdateManager.isRemoteReleaseNewer(
            localVersionCode = 63_200_017L,
            localBuildEpochMs = 1_763_200_000_000L,
            localGitSha = "",
            installedApkSha256 = sampleShaA.uppercase(),
            meta = meta
        )
        assertFalse(newer)
    }

    @Test
    fun isRemoteReleaseNewer_usesAuthoritativeVersionCodeForUpgradeAndNoDowngrade() {
        val sameVersionMeta = OtaUpdateManager.ReleaseMeta(
            sizeBytes = 25_000_000L,
            updatedEpochMs = 1_763_200_900_000L,
            versionCode = 63_200_100L,
            versionName = "3.13.50",
            commitSha = "abcdef123456",
            sha256 = sampleShaB
        )
        assertFalse(
            OtaUpdateManager.isRemoteReleaseNewer(
                localVersionCode = 63_200_100L,
                localBuildEpochMs = 1_763_200_000_000L,
                localGitSha = "111111111111",
                installedApkSha256 = sampleShaA,
                meta = sameVersionMeta
            )
        )
        assertFalse(
            OtaUpdateManager.isRemoteReleaseNewer(
                localVersionCode = 63_200_250L,
                localBuildEpochMs = 1_763_200_000_000L,
                localGitSha = "111111111111",
                installedApkSha256 = sampleShaA,
                meta = sameVersionMeta
            )
        )
        assertTrue(
            OtaUpdateManager.isRemoteReleaseNewer(
                localVersionCode = 63_200_050L,
                localBuildEpochMs = 1_763_200_890_000L,
                localGitSha = "111111111111",
                installedApkSha256 = sampleShaA,
                meta = sameVersionMeta
            )
        )
    }

    @Test
    fun isRemoteReleaseNewer_returnsFalseWhenCommitShaMatchesDespiteCiPublishDelay() {
        val metaFromLegacyBuildInfo = OtaUpdateManager.ReleaseMeta(
            sizeBytes = 24_800_000L,
            updatedEpochMs = 1_763_200_000_000L + 18 * 60_000L,
            versionCode = 0L,
            commitSha = "4c3b2a1987654321fedcba9876543210abcdef12",
            sha256 = sampleShaB
        )

        assertFalse(
            OtaUpdateManager.isRemoteReleaseNewer(
                localVersionCode = 63_200_017L,
                localBuildEpochMs = 1_763_200_000_000L,
                localGitSha = "4c3b2a198765",
                installedApkSha256 = "",
                meta = metaFromLegacyBuildInfo
            )
        )
    }

    @Test
    fun isRemoteReleaseNewer_timestampFallbackHonorsFullCiPipelineGraceWindow() {
        val baseCompileMs = 1_763_200_000_000L
        val delayedSameBuildMeta = OtaUpdateManager.ReleaseMeta(
            sizeBytes = 24_800_000L,
            updatedEpochMs = baseCompileMs + 14 * 60_000L
        )
        val genuinelyNewBuildMeta = OtaUpdateManager.ReleaseMeta(
            sizeBytes = 25_100_000L,
            updatedEpochMs = baseCompileMs + 30 * 60_000L
        )

        assertFalse(
            OtaUpdateManager.isRemoteReleaseNewer(
                localVersionCode = 63_200_017L,
                localBuildEpochMs = baseCompileMs,
                localGitSha = "",
                installedApkSha256 = "",
                meta = delayedSameBuildMeta
            )
        )
        assertTrue(
            OtaUpdateManager.isRemoteReleaseNewer(
                localVersionCode = 63_200_017L,
                localBuildEpochMs = baseCompileMs,
                localGitSha = "",
                installedApkSha256 = "",
                meta = genuinelyNewBuildMeta
            )
        )
    }

    @Test
    fun filterAndRankFreshMirrors_dropsFastStaleCdnNodesServingOld312Apk() {
        val oldTimeMs = 1_763_100_000_000L
        val freshTimeMs = 1_763_200_000_000L
        val staleFastCdn = OtaUpdateManager.VerifiedMirrorNode(
            prefix = "https://stale-edge.example.com/",
            hostLabel = "stale-edge.example.com",
            rttMs = 18L,
            sizeBytes = 23_900_000L,
            updatedEpochMs = oldTimeMs
        )
        val freshMirrorA = OtaUpdateManager.VerifiedMirrorNode(
            prefix = "https://fresh-a.example.com/",
            hostLabel = "fresh-a.example.com",
            rttMs = 135L,
            sizeBytes = 25_120_000L,
            updatedEpochMs = freshTimeMs
        )
        val freshOfficial = OtaUpdateManager.VerifiedMirrorNode(
            prefix = "",
            hostLabel = "github.com (官方)",
            rttMs = 240L,
            sizeBytes = 25_120_000L,
            updatedEpochMs = freshTimeMs
        )

        val expectedMeta = OtaUpdateManager.ReleaseMeta(
            sizeBytes = 25_120_000L,
            updatedEpochMs = freshTimeMs,
            versionCode = 63_200_500L,
            versionName = "3.13.88",
            sha256 = sampleShaB
        )

        val rankedWithMeta = OtaUpdateManager.filterAndRankFreshMirrors(
            listOf(staleFastCdn, freshOfficial, freshMirrorA),
            expectedMeta
        )
        assertEquals(listOf("https://fresh-a.example.com/", ""), rankedWithMeta.map { it.prefix })

        // 即使尚未拿到 expectedMeta，也必须按最新 Last-Modified 基准剔除旧缓存节点
        val rankedWithoutMeta = OtaUpdateManager.filterAndRankFreshMirrors(
            listOf(staleFastCdn, freshOfficial, freshMirrorA),
            expectedMeta = null
        )
        assertEquals(listOf("https://fresh-a.example.com/", ""), rankedWithoutMeta.map { it.prefix })
    }

    @Test
    fun parseAndMergeReleaseMetadata_discardsStaleCachedBuildInfoWhenApiHasNewerAsset() {
        val staleBuildInfoRaw = """
            {
              "source_commit": "1111111111111111",
              "version_code": 63100000,
              "version_name": "3.12.0",
              "bytes": 23900000,
              "sha256": "$sampleShaA"
            }
        """.trimIndent()
        val freshApiRaw = """
            {
              "target_commitish": "2222222222222222",
              "assets": [
                {
                  "name": "EGO-Pilot-AX12.apk",
                  "size": 25120000,
                  "updated_at": "2026-11-16T11:30:00Z",
                  "digest": "sha256:$sampleShaB"
                }
              ]
            }
        """.trimIndent()

        val staleBuildInfo = OtaUpdateManager.parseBuildInfoJson(staleBuildInfoRaw)
        val freshApi = OtaUpdateManager.parseGithubReleaseApiJson(freshApiRaw)
        assertNotNull(staleBuildInfo)
        assertNotNull(freshApi)

        val mergedConflict = OtaUpdateManager.mergeReleaseMetadata(freshApi, staleBuildInfo)
        assertNotNull(mergedConflict)
        assertEquals(25_120_000L, mergedConflict!!.sizeBytes)
        assertEquals(sampleShaB, mergedConflict.sha256)
        assertEquals("2222222222222222", mergedConflict.commitSha)
        // 不得混入旧版 build-info 的 versionCode=63100000
        assertEquals(0L, mergedConflict.versionCode)

        // 当两者同属最新同一次发布时，完整合并 versionName / versionCode / updatedEpochMs
        val matchingBuildInfoRaw = """
            {
              "source_commit": "2222222222222222",
              "version_code": 63200888,
              "version_name": "3.13.88",
              "bytes": 25120000,
              "sha256": "$sampleShaB"
            }
        """.trimIndent()
        val matchingBuildInfo = OtaUpdateManager.parseBuildInfoJson(matchingBuildInfoRaw)
        val mergedConsistent = OtaUpdateManager.mergeReleaseMetadata(freshApi, matchingBuildInfo)
        assertNotNull(mergedConsistent)
        assertEquals(63_200_888L, mergedConsistent!!.versionCode)
        assertEquals("3.13.88", mergedConsistent.versionName)
        assertEquals(sampleShaB, mergedConsistent.sha256)
        assertTrue(mergedConsistent.updatedEpochMs > 0L)
    }

    @Test
    fun computeFileSha256Hex_matchesKnownDigest() {
        val tempFile: File = Files.createTempFile("ego_ota_sha_test", ".bin").toFile()
        try {
            tempFile.writeBytes("abc".toByteArray(Charsets.UTF_8))
            val hex = OtaUpdateManager.computeFileSha256Hex(tempFile)
            assertEquals(64, hex.length)
            assertEquals(
                "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
                hex
            )

            // 验证跨 64KB 缓冲区分块的流式哈希一致性
            val largeBytes = ByteArray(150 * 1024) { idx -> (idx % 251).toByte() }
            tempFile.writeBytes(largeBytes)
            val expectedLargeHex = java.security.MessageDigest.getInstance("SHA-256")
                .digest(largeBytes)
                .joinToString("") { "%02x".format(it.toInt() and 0xFF) }
            assertEquals(expectedLargeHex, OtaUpdateManager.computeFileSha256Hex(tempFile))
        } finally {
            tempFile.delete()
        }
    }
}
