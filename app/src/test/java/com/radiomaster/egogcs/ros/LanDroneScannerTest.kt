package com.radiomaster.egogcs.ros

import com.radiomaster.egogcs.engine.EgoSimEngine
import com.radiomaster.egogcs.model.GcsSharedHub
import com.radiomaster.egogcs.model.TopicConfig
import java.io.ByteArrayOutputStream
import java.net.InetAddress
import java.net.ServerSocket
import java.nio.charset.StandardCharsets
import java.util.Collections
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class LanDroneScannerTest {

    @Test
    fun lanNetworkBinderFiltersVirtualCellularSubnetsAndExpandsWideMasks() {
        assertTrue(LanNetworkBinder.isVirtualOrCellularInterfaceName("tun0"))
        assertTrue(LanNetworkBinder.isVirtualOrCellularInterfaceName("wg0"))
        assertTrue(LanNetworkBinder.isVirtualOrCellularInterfaceName("rmnet_data1"))
        assertTrue(LanNetworkBinder.isVirtualOrCellularInterfaceName("ccmni0"))
        assertTrue(LanNetworkBinder.isVirtualOrCellularInterfaceName("v4-rmnet_data0"))
        assertFalse(LanNetworkBinder.isVirtualOrCellularInterfaceName("wlan0"))
        assertFalse(LanNetworkBinder.isVirtualOrCellularInterfaceName("ap0"))
        assertFalse(LanNetworkBinder.isVirtualOrCellularInterfaceName("usb0"))
        assertFalse(LanNetworkBinder.isVirtualOrCellularInterfaceName("eth0"))

        // Reject Clash fake-IP, CGNAT, link-local APIPA, and loopback; accept real RFC1918 LAN IPs
        assertFalse(LanNetworkBinder.isUsableLanIpv4("198.18.0.1"))
        assertFalse(LanNetworkBinder.isUsableLanIpv4("100.84.12.9"))
        assertFalse(LanNetworkBinder.isUsableLanIpv4("169.254.10.5"))
        assertFalse(LanNetworkBinder.isUsableLanIpv4("127.0.0.1"))
        assertTrue(LanNetworkBinder.isUsableLanIpv4("10.42.0.1"))
        assertTrue(LanNetworkBinder.isUsableLanIpv4("192.168.31.108"))
        assertTrue(LanNetworkBinder.isUsableLanIpv4("172.20.10.2"))

        // /22 subnet (e.g., 192.168.2.35/22 with gateway 192.168.0.1) expands all 4 /24 blocks, local & gateway first
        val expanded22 = LanNetworkBinder.expandSubnet24Prefixes(
            localIp = "192.168.2.35",
            prefixLength = 22,
            gatewayIp = "192.168.0.1"
        )
        assertEquals(
            listOf("192.168.2.", "192.168.0.", "192.168.1.", "192.168.3."),
            expanded22
        )
    }

    @Test
    fun probeSingleHostFingerprintsCppXmlRpcMasterAndNeverDropsOpenMasterPort() {
        val scanner = LanDroneScanner()

        // 1. Real loopback XML-RPC server omitting <string> tags (C++ xmlrpcpp format) & keeping socket open after Content-Length
        ServerSocket(0, 8, InetAddress.getByName("127.0.0.1")).use { srv ->
            val port = srv.localPort
            val serverThread = Thread {
                try {
                    srv.accept().use { client ->
                        val inp = client.getInputStream()
                        val reqBuf = ByteArray(1024)
                        inp.read(reqBuf)
                        val xmlBody = """
                            <?xml version="1.0"?>
                            <methodResponse><params><param><value><array><data>
                              <value><i4>1</i4></value>
                              <value>current system state</value>
                              <value><array><data>
                                <value>/vins_fusion/imu_propagate</value>
                                <value><string>/drone_0_ego_planner_node/optimal_list</string></value>
                                <value>/drone_0_ego_planner_node/grid_map/occupancy_inflate</value>
                                <value>/tilt_uav/servo_angles</value>
                              </data></array></value>
                            </data></array></value></param></params></methodResponse>
                        """.trimIndent()
                        val bodyBytes = xmlBody.toByteArray(StandardCharsets.UTF_8)
                        val respHdr = (
                            "HTTP/1.1 200 OK\r\n" +
                                "Content-Type: text/xml\r\n" +
                                "Content-Length: ${bodyBytes.size}\r\n\r\n"
                            ).toByteArray(StandardCharsets.US_ASCII)
                        val out = client.getOutputStream()
                        out.write(respHdr)
                        out.write(bodyBytes)
                        out.flush()
                        // Deliberately hold socket briefly to verify Content-Length reader returns immediately without waiting for EOF
                        Thread.sleep(250)
                    }
                } catch (_: Throwable) {
                }
            }.apply {
                isDaemon = true
                start()
            }

            // First connection is tcpProbePort(Open), second is postXmlRpc: accept multiple connections cleanly
            val acceptLoop = Thread {
                try {
                    srv.accept().use { client ->
                        val inp = client.getInputStream()
                        val reqBuf = ByteArray(1024)
                        inp.read(reqBuf)
                        val xmlBody = """
                            <?xml version="1.0"?>
                            <methodResponse><params><param><value><array><data>
                              <value><i4>1</i4></value>
                              <value>ok</value>
                              <value><array><data>
                                <value>/vins_fusion/imu_propagate</value>
                                <value><string>/drone_0_ego_planner_node/optimal_list</string></value>
                                <value>/drone_0_ego_planner_node/grid_map/occupancy_inflate</value>
                                <value>/tilt_uav/servo_angles</value>
                              </data></array></value>
                            </data></array></value></param></params></methodResponse>
                        """.trimIndent()
                        val bodyBytes = xmlBody.toByteArray(StandardCharsets.UTF_8)
                        val respHdr = (
                            "HTTP/1.1 200 OK\r\n" +
                                "Content-Type: text/xml\r\n" +
                                "Content-Length: ${bodyBytes.size}\r\n\r\n"
                            ).toByteArray(StandardCharsets.US_ASCII)
                        client.getOutputStream().apply {
                            write(respHdr)
                            write(bodyBytes)
                            flush()
                        }
                    }
                } catch (_: Throwable) {
                }
            }.apply {
                isDaemon = true
                start()
            }

            val drone = scanner.probeSingleHost(
                ip = "127.0.0.1",
                baseCfg = TopicConfig(),
                masterPorts = listOf(port),
                wsPorts = emptyList(),
                isPriority = true
            )
            serverThread.join(400)
            acceptLoop.join(400)

            assertNotNull(drone)
            assertTrue(drone!!.rosReady)
            assertEquals("tcpros", drone.protocol)
            assertEquals(port, drone.port)
            assertTrue(drone.stackSummary.contains("VINS-Fusion"))
            assertTrue(drone.stackSummary.contains("Swarm(drone_0)"))
            assertTrue(drone.stackSummary.contains("Tilt-INDI"))
            assertEquals("/vins_fusion/imu_propagate", drone.suggestedConfig.odomTopic)
            assertEquals("/drone_0_ego_planner_node/optimal_list", drone.suggestedConfig.optimalTopic)
            assertEquals("/drone_0_ego_planner_node/grid_map/occupancy_inflate", drone.suggestedConfig.inflateTopic)
        }

        // 2. Open master port that closes XML-RPC immediately still reports a connectable ROS1 Master instead of null
        ServerSocket(0, 8, InetAddress.getByName("127.0.0.1")).use { silentSrv ->
            val port = silentSrv.localPort
            val closer = Thread {
                repeat(3) {
                    try {
                        silentSrv.accept().close()
                    } catch (_: Throwable) {
                        return@Thread
                    }
                }
            }.apply {
                isDaemon = true
                start()
            }

            val fallbackDrone = scanner.probeSingleHost(
                ip = "127.0.0.1",
                baseCfg = TopicConfig(),
                masterPorts = listOf(port),
                wsPorts = emptyList(),
                isPriority = false
            )
            assertNotNull(fallbackDrone)
            assertTrue(fallbackDrone!!.rosReady)
            assertEquals("http://127.0.0.1:$port", fallbackDrone.suggestedConfig.masterUri)
            silentSrv.close()
            closer.join(200)
        }
    }

    @Test
    fun sshBannerIdentifiesOnlineUbuntuHostAndRejectsRouterDropbear() {
        val scanner = LanDroneScanner()

        ServerSocket(0, 4, InetAddress.getByName("127.0.0.1")).use { sshSrv ->
            val port = sshSrv.localPort
            val bannerThread = Thread {
                try {
                    sshSrv.accept().use { sock ->
                        sock.getOutputStream().write(
                            "SSH-2.0-OpenSSH_8.9p1 Ubuntu-3ubuntu0.10\r\n".toByteArray(StandardCharsets.US_ASCII)
                        )
                        sock.getOutputStream().flush()
                    }
                } catch (_: Throwable) {
                }
            }.apply {
                isDaemon = true
                start()
            }

            val diag = scanner.inspectSshLinuxBanner(
                ip = "127.0.0.1",
                sshPort = port,
                isRouterGateway = false,
                baseCfg = TopicConfig()
            )
            bannerThread.join(300)
            assertNotNull(diag)
            assertFalse(diag!!.rosReady)
            assertTrue(diag.stackSummary.contains("Ubuntu"))
        }

        ServerSocket(0, 4, InetAddress.getByName("127.0.0.1")).use { routerSrv ->
            val port = routerSrv.localPort
            val routerThread = Thread {
                try {
                    routerSrv.accept().use { sock ->
                        sock.getOutputStream().write(
                            "SSH-2.0-dropbear_2022.83\r\n".toByteArray(StandardCharsets.US_ASCII)
                        )
                        sock.getOutputStream().flush()
                    }
                } catch (_: Throwable) {
                }
            }.apply {
                isDaemon = true
                start()
            }

            val rejected = scanner.inspectSshLinuxBanner(
                ip = "127.0.0.1",
                sshPort = port,
                isRouterGateway = true,
                baseCfg = TopicConfig()
            )
            routerThread.join(300)
            assertNull(rejected)
        }
    }

    @Test
    fun mdnsPacketBuilderAndARecordParserExtractOnboardIpv4Addresses() {
        val scanner = LanDroneScanner()
        val queryBytes = scanner.buildDnsQueryPacket(
            listOf("_ros._tcp.local" to 12, "jetson.local" to 1)
        )
        assertTrue(queryBytes.size > 32)
        assertEquals(2, ((queryBytes[4].toInt() and 0xFF) shl 8) or (queryBytes[5].toInt() and 0xFF))

        // Build a synthetic mDNS response with 1 question ("jetson.local") and 1 compressed-name A record -> 192.168.31.128
        val resp = ByteArrayOutputStream()
        // Header: ID=0, Flags=0x8400, QDCOUNT=1, ANCOUNT=1, NSCOUNT=0, ARCOUNT=0
        resp.write(byteArrayOf(0, 0, 0x84.toByte(), 0, 0, 1, 0, 1, 0, 0, 0, 0))
        // Question: 6 "jetson" 5 "local" 0, QTYPE=1, QCLASS=1
        resp.write(6)
        resp.write("jetson".toByteArray(StandardCharsets.US_ASCII))
        resp.write(5)
        resp.write("local".toByteArray(StandardCharsets.US_ASCII))
        resp.write(0)
        resp.write(byteArrayOf(0, 1, 0, 1))
        // Answer: compressed pointer 0xC0 0x0C, TYPE=1 (A), CLASS=0x8001, TTL=120, RDLENGTH=4, RDATA=192.168.31.128
        resp.write(byteArrayOf(0xC0.toByte(), 0x0C, 0, 1, 0x80.toByte(), 1, 0, 0, 0, 120, 0, 4))
        resp.write(byteArrayOf(192.toByte(), 168.toByte(), 31, 128.toByte()))

        val raw = resp.toByteArray()
        val ips = scanner.extractDnsARecordIpv4s(raw, raw.size)
        assertEquals(listOf("192.168.31.128"), ips)
    }

    @Test
    fun exponentialBackoffPolicyScalesCapsJittersAndResetsDeterministically() {
        val deterministic = ExponentialBackoffPolicy(
            baseDelayMs = 2_000L,
            maxDelayMs = 30_000L,
            multiplier = 2.0,
            jitterRatio = 0.0,
            maxAttempts = 6
        )
        assertEquals(0, deterministic.attemptCount)
        assertEquals(2_000L, deterministic.nextDelayMs())
        assertEquals(4_000L, deterministic.nextDelayMs())
        assertEquals(8_000L, deterministic.nextDelayMs())
        assertEquals(16_000L, deterministic.nextDelayMs())
        assertEquals(30_000L, deterministic.nextDelayMs())
        assertEquals(30_000L, deterministic.nextDelayMs())
        assertFalse(deterministic.hasBudgetRemaining())
        assertEquals(-1L, deterministic.nextDelayMs())

        deterministic.reset()
        assertEquals(0, deterministic.attemptCount)
        assertTrue(deterministic.hasBudgetRemaining())
        assertEquals(2_000L, deterministic.nextDelayMs())

        // High-exponent overflow guard & symmetric jitter bounds
        val plusJitter = ExponentialBackoffPolicy(
            baseDelayMs = 1_000L,
            maxDelayMs = 30_000L,
            multiplier = 2.0,
            jitterRatio = 0.15,
            jitterSource = { 1.0 }
        )
        assertEquals(1_150L, plusJitter.nextDelayMs())
        assertEquals(30_000L, plusJitter.computeDelayForAttempt(45, applyJitter = true))

        val minusJitter = ExponentialBackoffPolicy(
            baseDelayMs = 1_000L,
            maxDelayMs = 30_000L,
            multiplier = 2.0,
            jitterRatio = 0.15,
            jitterSource = { -1.0 }
        )
        assertEquals(850L, minusJitter.nextDelayMs())
    }

    @Test
    fun scannerAutoBackoffRetriesAfterIntermittentOutageAndRecoversWithoutUserIntervention() {
        // Reserve a free loopback port, then close it initially to simulate an offline / Wi-Fi-disconnected drone
        val reservedPort = ServerSocket(0, 4, InetAddress.getByName("127.0.0.1")).use { it.localPort }

        val scanner = LanDroneScanner(
            standardBackoffPolicy = ExponentialBackoffPolicy(
                baseDelayMs = 40L,
                maxDelayMs = 320L,
                multiplier = 2.0,
                jitterRatio = 0.0,
                minDelayMs = 20L
            ),
            bootingHostBackoffPolicy = ExponentialBackoffPolicy(
                baseDelayMs = 30L,
                maxDelayMs = 200L,
                multiplier = 2.0,
                jitterRatio = 0.0,
                minDelayMs = 20L
            )
        )

        val scheduledDelays = Collections.synchronizedList(mutableListOf<Long>())
        val recoveredDrone = AtomicReference<DiscoveredDrone?>(null)
        val recoveredLatch = CountDownLatch(1)
        val serverStarted = AtomicBoolean(false)
        var liveServer: ServerSocket? = null
        var serverThread: Thread? = null

        try {
            val cfg = TopicConfig(
                protocol = "tcpros",
                masterUri = "http://127.0.0.1:$reservedPort",
                wsUrl = "ws://127.0.0.1:9090",
                swarmPeerIpsCsv = ""
            )

            scanner.startScan(
                baseConfig = cfg,
                manualHintRaw = "127.0.0.1",
                restrictToHintsOnly = true,
                shouldScheduleAutoRetry = { recoveredDrone.get() == null },
                onBackoffScheduled = { attempt, delayMs, reason ->
                    scheduledDelays.add(delayMs)
                    assertEquals(DiscoveryRecoveryReason.EMPTY_SCAN_BACKOFF, reason)
                    // After 2 failed exponential-backoff passes (40ms -> 80ms), bring the onboard ROS master online
                    if (attempt == 2 && serverStarted.compareAndSet(false, true)) {
                        val srv = ServerSocket(reservedPort, 8, InetAddress.getByName("127.0.0.1"))
                        liveServer = srv
                        serverThread = Thread {
                            repeat(4) {
                                try {
                                    srv.accept().use { client ->
                                        val req = ByteArray(512)
                                        val n = runCatching { client.getInputStream().read(req) }.getOrDefault(-1)
                                        if (n > 0) {
                                            val xml = """
                                                <?xml version="1.0"?>
                                                <methodResponse><params><param><value><array><data>
                                                  <value><i4>1</i4></value>
                                                  <value>ok</value>
                                                  <value><array><data>
                                                    <value><string>/Odometry</string></value>
                                                    <value><string>/cloud_registered</string></value>
                                                    <value><string>/ego_planner_node/optimal_list</string></value>
                                                  </data></array></value>
                                                </data></array></value></param></params></methodResponse>
                                            """.trimIndent()
                                            val body = xml.toByteArray(StandardCharsets.UTF_8)
                                            val hdr = (
                                                "HTTP/1.0 200 OK\r\nContent-Type: text/xml\r\n" +
                                                    "Content-Length: ${body.size}\r\n\r\n"
                                                ).toByteArray(StandardCharsets.US_ASCII)
                                            client.getOutputStream().apply {
                                                write(hdr)
                                                write(body)
                                                flush()
                                            }
                                        }
                                    }
                                } catch (_: Throwable) {
                                    return@Thread
                                }
                            }
                        }.apply {
                            isDaemon = true
                            start()
                        }
                    }
                },
                onProgress = { _, _ -> },
                onDroneFound = { drone ->
                    if (drone.rosReady && recoveredDrone.compareAndSet(null, drone)) {
                        recoveredLatch.countDown()
                    }
                },
                onComplete = { _, _ -> }
            )

            assertTrue(
                "Scanner must automatically recover after intermittent outage without manual re-trigger",
                recoveredLatch.await(4, TimeUnit.SECONDS)
            )
            val found = recoveredDrone.get()
            assertNotNull(found)
            assertEquals("127.0.0.1", found!!.ip)
            assertEquals(reservedPort, found.port)
            assertTrue(found.stackSummary.contains("FAST-LIO2"))
            assertTrue("Expected exponential backoff progression [40, 80, ...], got $scheduledDelays", scheduledDelays.size >= 2)
            assertEquals(40L, scheduledDelays[0])
            assertEquals(80L, scheduledDelays[1])

            // Wait briefly for onComplete cleanup to reset backoff state
            Thread.sleep(60L)
            assertEquals(0, scanner.currentBackoffAttempt)
            assertFalse(scanner.isBackoffScheduled)
        } finally {
            scanner.shutdown()
            runCatching { liveServer?.close() }
            serverThread?.join(300)
        }
    }

    @Test
    fun rosConnectionManagerEscalatesBridgeDisconnectionsToDiscoveryRecovery() {
        val deadWsPort = ServerSocket(0, 2, InetAddress.getByName("127.0.0.1")).use { it.localPort }
        val hub = GcsSharedHub()
        val sim = EgoSimEngine(hub)
        val wsClient = RosBridgeClient(
            hub = hub,
            engine = sim,
            onStateToast = {},
            reconnectBackoff = ExponentialBackoffPolicy(
                baseDelayMs = 35L,
                maxDelayMs = 200L,
                multiplier = 2.0,
                jitterRatio = 0.0,
                minDelayMs = 20L
            )
        )
        val tcpClient = TcpRosNativeClient(hub, sim, {})
        val udpClient = RosUdpTelemetryClient(hub, sim)
        val manager = RosConnectionManager(hub, tcpClient, wsClient, udpClient)

        val escalationLatch = CountDownLatch(1)
        val escalatedReason = AtomicReference<DiscoveryRecoveryReason?>(null)
        manager.onRequestDiscoveryRecovery = { reason ->
            if (escalatedReason.compareAndSet(null, reason)) {
                escalationLatch.countDown()
            }
        }

        try {
            manager.connect(
                TopicConfig(
                    protocol = "rosbridge",
                    wsUrl = "ws://127.0.0.1:$deadWsPort",
                    udpTelemetryEnabled = false
                )
            )

            assertTrue(
                "Bridge failures should escalate to autonomous discovery recovery after >= 2 backoff attempts",
                escalationLatch.await(3, TimeUnit.SECONDS)
            )
            assertEquals(DiscoveryRecoveryReason.BRIDGE_DISCONNECTED, escalatedReason.get())
            assertTrue("Expected automatic bridge reconnect attempts >= 1", hub.rosReconnectCount >= 1)

            manager.disconnectAll(silent = true)
            assertEquals(0, wsClient.reconnectBackoff.attemptCount)
        } finally {
            manager.shutdown()
        }
    }
}
