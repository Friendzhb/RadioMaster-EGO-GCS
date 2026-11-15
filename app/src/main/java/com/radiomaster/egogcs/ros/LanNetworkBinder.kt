package com.radiomaster.egogcs.ros

import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.wifi.WifiManager
import android.os.Build
import java.io.BufferedInputStream
import java.io.BufferedOutputStream
import java.io.ByteArrayOutputStream
import java.io.File
import java.net.DatagramSocket
import java.net.Inet4Address
import java.net.InetSocketAddress
import java.net.NetworkInterface
import java.net.Proxy
import java.net.Socket
import java.util.Locale
import javax.net.SocketFactory

/**
 * 局域网物理网卡绑定与跨 Android 9~15 多网段路由器 (`LanNetworkBinder`)：
 *
 * 专门解决无人机地面站在真机外场与实验室环境下“搜不到飞机 / 连不上无外网机载热点”的五大底层网络栈问题：
 * 1. **防 4G/5G 蜂窝网络路由劫持**：当手机/遥控器连接无外网的机载 Wi-Fi 热点 (如 Ubuntu `10.42.0.1`、机载路由器 `192.168.x.x`)
 *    且同时开启移动数据时，Android 默认将无 Internet 的 Wi-Fi 降权并把普通 `Socket()` 路由至 4G/5G 网卡；
 *    本组件通过 `ConnectivityManager` 定位物理 `TRANSPORT_WIFI / ETHERNET / USB` 句柄并执行 `network.bindSocket()`；
 * 2. **防 VPN / Clash 虚拟网卡与系统代理劫持**：严格过滤 `tun* / wg* / ppp* / rmnet* / ccmni*` 及 `198.18.*` Fake-IP 网段，
 *    所有探测与 ROS 通信强制走 `Proxy.NO_PROXY`；
 * 3. **兼容 Android 11~15 (`targetSdk = 34`) Netlink SELinux 限制**：综合 `ConnectivityManager.allNetworks` (`LinkProperties`)、
 *    `WifiManager.dhcpInfo` 与 `NetworkInterface` 三重数据源提取局域网 IP、掩码前缀长度 (`/22~/24`)、网关及 SoftAP 热点网段；
 * 4. **零分块原生 HTTP/1.0 XML-RPC 传输**：规避 `HttpURLConnection` 分块编码与长连接阻塞，完美兼容 Python `roscore` 与 C++ `xmlrpcpp`。
 */
object LanNetworkBinder {

    data class LanInterfaceSnapshot(
        val interfaceName: String,
        val localIp: String,
        val prefixLength: Int,
        val gatewayIp: String?,
        val network: Network?,
        // 0 = 有线/USB网口, 1 = 本机SoftAP热点, 2 = Wi-Fi客户端, 3 = 其它局域网桥接
        val priority: Int
    )

    @Volatile
    private var appContext: Context? = null

    private val IPV4_LITERAL_REGEX = Regex("^(\\d{1,3})\\.(\\d{1,3})\\.(\\d{1,3})\\.(\\d{1,3})$")

    fun init(context: Context) {
        appContext = context.applicationContext
    }

    /**
     * 综合枚举当前设备上所有活动的局域网物理接口（Wi-Fi、USB-RNDIS、有线以太网、手机自身开启的移动热点 SoftAP），
     * 彻底剔除 4G/5G 基带 (`rmnet*`/`ccmni*`) 与 VPN 虚拟隧道 (`tun*`/`wg*`/`198.18.*`)。
     */
    @Suppress("DEPRECATION")
    fun discoverActiveLanInterfaces(): List<LanInterfaceSnapshot> {
        val merged = LinkedHashMap<String, LanInterfaceSnapshot>()

        fun recordCandidate(
            ifNameRaw: String?,
            ip: String?,
            prefixLenRaw: Int,
            gatewayIp: String?,
            network: Network?,
            priorityOverride: Int? = null
        ) {
            val cleanIp = ip?.trim().orEmpty()
            if (!isUsableLanIpv4(cleanIp)) return
            val ifName = ifNameRaw?.trim().orEmpty()
            if (ifName.isNotEmpty() && isVirtualOrCellularInterfaceName(ifName)) return

            val prefix = if (prefixLenRaw in 8..30) prefixLenRaw else 24
            val prio = priorityOverride ?: classifyInterfacePriority(ifName)
            val cleanGw = gatewayIp?.trim()?.takeIf { isUsableLanIpv4(it) && it != cleanIp }
            val existing = merged[cleanIp]
            if (existing == null ||
                (existing.network == null && network != null) ||
                prio < existing.priority ||
                (existing.gatewayIp == null && cleanGw != null)
            ) {
                merged[cleanIp] = LanInterfaceSnapshot(
                    interfaceName = ifName.ifEmpty { existing?.interfaceName ?: "lan" },
                    localIp = cleanIp,
                    prefixLength = if (prefixLenRaw in 8..30) prefix else (existing?.prefixLength ?: 24),
                    gatewayIp = cleanGw ?: existing?.gatewayIp,
                    network = network ?: existing?.network,
                    priority = minOf(prio, existing?.priority ?: prio)
                )
            }
        }

        val ctx = appContext
        if (ctx != null) {
            // 数据源 1: ConnectivityManager.allNetworks + LinkProperties (Android 10~15 targetSdk=34 官方权威接口)
            try {
                val cm = ctx.getSystemService(Context.CONNECTIVITY_SERVICE) as? ConnectivityManager
                val networks = cm?.allNetworks.orEmpty()
                for (net in networks) {
                    val caps = try {
                        cm?.getNetworkCapabilities(net)
                    } catch (_: Throwable) {
                        null
                    } ?: continue
                    if (caps.hasTransport(NetworkCapabilities.TRANSPORT_VPN)) continue

                    val isEth = caps.hasTransport(NetworkCapabilities.TRANSPORT_ETHERNET)
                    val isUsb = Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
                        caps.hasTransport(NetworkCapabilities.TRANSPORT_USB)
                    val isWifi = caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI)
                    val isCellOnly = caps.hasTransport(NetworkCapabilities.TRANSPORT_CELLULAR) && !isWifi && !isEth && !isUsb
                    if (isCellOnly) continue
                    if (!isWifi && !isEth && !isUsb) continue

                    val lp = try {
                        cm?.getLinkProperties(net)
                    } catch (_: Throwable) {
                        null
                    } ?: continue

                    val ifName = lp.interfaceName.orEmpty()
                    if (ifName.isNotEmpty() && isVirtualOrCellularInterfaceName(ifName)) continue

                    var gatewayV4: String? = null
                    for (route in lp.routes) {
                        val gw = route.gateway
                        if (gw is Inet4Address && !gw.isLoopbackAddress) {
                            val gwStr = gw.hostAddress
                            if (isUsableLanIpv4(gwStr)) {
                                gatewayV4 = gwStr
                                break
                            }
                        }
                    }

                    val prio = when {
                        isEth || isUsb -> 0
                        isWifi -> 2
                        else -> 3
                    }
                    for (la in lp.linkAddresses) {
                        val addr = la.address
                        if (addr is Inet4Address && !addr.isLoopbackAddress) {
                            recordCandidate(
                                ifNameRaw = ifName.ifEmpty { if (isWifi) "wlan0" else "eth0" },
                                ip = addr.hostAddress,
                                prefixLenRaw = la.prefixLength,
                                gatewayIp = gatewayV4,
                                network = net,
                                priorityOverride = prio
                            )
                        }
                    }
                }
            } catch (_: Throwable) {
            }

            // 数据源 2: WifiManager.dhcpInfo 兜底 (兼容部分定制系统未验证无外网 Wi-Fi 时隐藏 LinkAddresses 的情况)
            collectWifiDhcpFallback(ctx) { ip, prefixLen, gw ->
                val wifiNet = findFirstWifiOrEthernetNetwork(ctx)
                recordCandidate("wlan0", ip, prefixLen, gw, wifiNet, priorityOverride = 2)
            }
        }

        // 数据源 3: NetworkInterface 枚举 (可捕获手机开启移动热点 ap0/swlan0、AX12 Android 9 RNDIS/以太网及纯 JVM 环境)
        try {
            val nifs = NetworkInterface.getNetworkInterfaces()
            while (nifs != null && nifs.hasMoreElements()) {
                val nif = nifs.nextElement()
                val name = nif.name.orEmpty()
                if (nif.isLoopback || isVirtualOrCellularInterfaceName(name)) continue
                // 部分 Android 固件 SoftAP 接口 isUp 调用受限，允许按 interfaceAddresses 兜底读取
                val up = try {
                    nif.isUp
                } catch (_: Throwable) {
                    true
                }
                if (!up) continue

                val ifAddrs = try {
                    nif.interfaceAddresses
                } catch (_: Throwable) {
                    emptyList()
                }
                if (ifAddrs.isNotEmpty()) {
                    for (ia in ifAddrs) {
                        val a = ia.address
                        if (a is Inet4Address && !a.isLoopbackAddress) {
                            recordCandidate(
                                ifNameRaw = name,
                                ip = a.hostAddress,
                                prefixLenRaw = ia.networkPrefixLength.toInt(),
                                gatewayIp = null,
                                network = null
                            )
                        }
                    }
                } else {
                    val addrs = nif.inetAddresses
                    while (addrs.hasMoreElements()) {
                        val a = addrs.nextElement()
                        if (a is Inet4Address && !a.isLoopbackAddress) {
                            recordCandidate(
                                ifNameRaw = name,
                                ip = a.hostAddress,
                                prefixLenRaw = 24,
                                gatewayIp = null,
                                network = null
                            )
                        }
                    }
                }
            }
        } catch (_: Throwable) {
        }

        return merged.values.sortedWith(
            compareBy<LanInterfaceSnapshot> { it.priority }
                .thenBy { if (it.network != null) 0 else 1 }
                .thenBy { it.interfaceName }
        )
    }

    @Suppress("DEPRECATION")
    private fun collectWifiDhcpFallback(
        ctx: Context,
        onFound: (ip: String, prefixLen: Int, gatewayIp: String?) -> Unit
    ) {
        try {
            val wm = ctx.applicationContext.getSystemService(Context.WIFI_SERVICE) as? WifiManager ?: return
            if (!wm.isWifiEnabled) return
            val dhcp = wm.dhcpInfo
            val rawIp = when {
                dhcp != null && dhcp.ipAddress != 0 -> dhcp.ipAddress
                else -> wm.connectionInfo?.ipAddress ?: 0
            }
            if (rawIp == 0) return
            val ipStr = littleEndianIntToIpv4(rawIp)
            val maskInt = dhcp?.netmask ?: 0
            val prefixLen = if (maskInt != 0) Integer.bitCount(maskInt).coerceIn(16, 30) else 24
            val gwStr = if (dhcp != null && dhcp.gateway != 0) {
                littleEndianIntToIpv4(dhcp.gateway)
            } else if (dhcp != null && dhcp.serverAddress != 0) {
                littleEndianIntToIpv4(dhcp.serverAddress)
            } else {
                null
            }
            onFound(ipStr, prefixLen, gwStr)
        } catch (_: Throwable) {
        }
    }

    @Suppress("DEPRECATION")
    private fun findFirstWifiOrEthernetNetwork(ctx: Context): Network? {
        return try {
            val cm = ctx.getSystemService(Context.CONNECTIVITY_SERVICE) as? ConnectivityManager ?: return null
            var bestWifi: Network? = null
            for (net in cm.allNetworks) {
                val caps = cm.getNetworkCapabilities(net) ?: continue
                if (caps.hasTransport(NetworkCapabilities.TRANSPORT_VPN)) continue
                val hasUsb = Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
                    caps.hasTransport(NetworkCapabilities.TRANSPORT_USB)
                if (caps.hasTransport(NetworkCapabilities.TRANSPORT_ETHERNET) || hasUsb) {
                    return net
                }
                if (caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) && bestWifi == null) {
                    bestWifi = net
                }
            }
            bestWifi
        } catch (_: Throwable) {
            null
        }
    }

    /**
     * 读取内核 ARP / 邻居表已缓存的在线设备 IP（在 Android 9 遥控器如 RadioMaster AX12 及部分平板上可直接秒级命中热点客户端）。
     */
    fun readKnownArpNeighborIps(): Set<String> {
        val result = LinkedHashSet<String>()
        try {
            val arpFile = File("/proc/net/arp")
            if (arpFile.isFile && arpFile.canRead()) {
                arpFile.useLines { lines ->
                    for (line in lines.drop(1)) {
                        val cols = line.trim().split(Regex("\\s+"))
                        if (cols.size >= 6) {
                            val ip = cols[0]
                            val flags = cols[2]
                            val mac = cols[3]
                            val dev = cols[5]
                            if (flags != "0x0" &&
                                mac != "00:00:00:00:00:00" &&
                                !isVirtualOrCellularInterfaceName(dev) &&
                                isUsableLanIpv4(ip)
                            ) {
                                result.add(ip)
                            }
                        }
                    }
                }
            }
        } catch (_: Throwable) {
        }
        return result
    }

    /**
     * 根据网卡 IP 与子网掩码前缀长度生成待扫描的 `/24` 网段前缀集合（如 `"192.168.31."`）：
     * - 支持标准 `/24`；
     * - 支持实验室/企业路由器常见的 `/23` (510 台) 与 `/22` (1022 台) 跨子网自动展开，优先扫描本机与网关所在 `/24` 块！
     */
    fun expandSubnet24Prefixes(localIp: String, prefixLength: Int, gatewayIp: String? = null): List<String> {
        val ipInt = ipv4ToUnsignedInt(localIp) ?: return emptyList()
        val ordered = LinkedHashSet<String>()

        fun addBlockOf(addrInt: Long) {
            val a = (addrInt ushr 24) and 0xFF
            val b = (addrInt ushr 16) and 0xFF
            val c = (addrInt ushr 8) and 0xFF
            ordered.add("$a.$b.$c.")
        }

        addBlockOf(ipInt)
        if (!gatewayIp.isNullOrBlank()) {
            val gwInt = ipv4ToUnsignedInt(gatewayIp)
            if (gwInt != null && isSameSubnet(ipInt, gwInt, prefixLength)) {
                addBlockOf(gwInt)
            }
        }

        if (prefixLength in 22..23) {
            val blockCount = 1 shl (24 - prefixLength) // /23 -> 2 个 /24 块, /22 -> 4 个 /24 块
            val mask = (0xFFFFFFFFL shl (32 - prefixLength)) and 0xFFFFFFFFL
            val baseNet = ipInt and mask
            for (bIdx in 0 until blockCount) {
                addBlockOf(baseNet + (bIdx.toLong() shl 8))
            }
        }
        return ordered.toList()
    }

    /**
     * 为目标局域网主机匹配最佳物理 `Network`（优先子网精确命中，其次回退到活动的以太网/USB/Wi-Fi 网络），
     * 防止无外网无人机 Wi-Fi 下被 Android 路由到 4G/5G 蜂窝网络。
     */
    fun findRouteNetworkForHost(targetHostOrIp: String): Network? {
        val clean = targetHostOrIp.trim()
        if (clean.isEmpty() || clean == "127.0.0.1" || clean.equals("localhost", ignoreCase = true)) {
            return null
        }
        val interfaces = discoverActiveLanInterfaces()
        if (interfaces.isEmpty()) return null

        val targetInt = ipv4ToUnsignedInt(clean)
        if (targetInt != null) {
            for (iface in interfaces) {
                val net = iface.network ?: continue
                val localInt = ipv4ToUnsignedInt(iface.localIp) ?: continue
                if (isSameSubnet(localInt, targetInt, iface.prefixLength)) {
                    return net
                }
            }
            if (!isPrivateOrLanIpv4Int(targetInt)) {
                return null
            }
        }
        return interfaces.firstOrNull { it.network != null }?.network
    }

    /**
     * 创建直连物理局域网且绕过系统/VPN 代理 (`Proxy.NO_PROXY`) 的 TCP `Socket`。
     */
    fun createBoundSocket(targetHostOrIp: String): Socket {
        val sock = Socket(Proxy.NO_PROXY)
        bindSocketToLan(sock, targetHostOrIp)
        return sock
    }

    fun bindSocketToLan(socket: Socket, targetHostOrIp: String) {
        val net = findRouteNetworkForHost(targetHostOrIp) ?: return
        try {
            net.bindSocket(socket)
        } catch (_: Throwable) {
        }
    }

    fun bindDatagramSocketToLan(socket: DatagramSocket, targetHostOrIp: String) {
        val net = findRouteNetworkForHost(targetHostOrIp) ?: return
        try {
            net.bindSocket(socket)
        } catch (_: Throwable) {
        }
    }

    /**
     * 为 OkHttp (`RosBridgeClient`) 提供绑定到目标无人机所属局域网物理网卡的 `SocketFactory`。
     */
    fun socketFactoryForHost(targetHostOrIp: String): SocketFactory {
        val net = findRouteNetworkForHost(targetHostOrIp)
        if (net != null) {
            return net.socketFactory
        }
        return SocketFactory.getDefault()
    }

    /**
     * 探测本机用于与指定机载 ROS Master 通信的真实局域网 IPv4 地址（等价于自动计算 `ROS_IP`），
     * 确保机载节点反向连接地面站 `/move_base_simple/goal` 时不会拿到 `127.0.0.1` 或蜂窝/VPN 虚拟 IP。
     */
    fun detectLocalLanIpForRemote(remoteHost: String, remotePort: Int = 11311): String {
        val cleanHost = remoteHost.trim()
        if (cleanHost == "127.0.0.1" || cleanHost.equals("localhost", ignoreCase = true)) {
            return "127.0.0.1"
        }

        val interfaces = discoverActiveLanInterfaces()
        val remoteInt = ipv4ToUnsignedInt(cleanHost)
        if (remoteInt != null) {
            for (iface in interfaces) {
                val localInt = ipv4ToUnsignedInt(iface.localIp) ?: continue
                if (isSameSubnet(localInt, remoteInt, iface.prefixLength)) {
                    return iface.localIp
                }
            }
        }

        try {
            createBoundSocket(cleanHost).use { probe ->
                probe.connect(InetSocketAddress(cleanHost, remotePort), 1200)
                val addr = probe.localAddress?.hostAddress
                if (!addr.isNullOrBlank() && addr != "127.0.0.1" && addr != "0.0.0.0") {
                    return addr
                }
            }
        } catch (_: Throwable) {
        }

        if (interfaces.isNotEmpty()) {
            return interfaces.first().localIp
        }
        return "127.0.0.1"
    }

    /**
     * 通过绑定到物理局域网的直连 Socket 发送定长 `HTTP/1.0` XML-RPC POST 请求，并按 `Content-Length` 精确读取响应体：
     * - 彻底规避 `HttpURLConnection` 的蜂窝错路由、系统代理劫持、Chunked 分块编码被 `roscore` 拒绝及 Keep-Alive 读超时问题。
     */
    fun postXmlRpc(
        uri: String,
        xmlBody: String,
        connectTimeoutMs: Int = 2500,
        readTimeoutMs: Int = 3000
    ): String {
        val parsed = parseHttpEndpoint(uri)
            ?: throw IllegalArgumentException("无效的 XML-RPC URI: $uri")
        val bodyBytes = xmlBody.toByteArray(Charsets.UTF_8)
        val reqHeader = buildString(192) {
            append("POST ").append(parsed.path).append(" HTTP/1.0\r\n")
            append("Host: ").append(parsed.host).append(':').append(parsed.port).append("\r\n")
            append("User-Agent: EGO-GCS-Android\r\n")
            append("Content-Type: text/xml\r\n")
            append("Content-Length: ").append(bodyBytes.size).append("\r\n")
            append("Connection: close\r\n\r\n")
        }.toByteArray(Charsets.US_ASCII)

        createBoundSocket(parsed.host).use { sock ->
            sock.tcpNoDelay = true
            sock.soTimeout = readTimeoutMs
            sock.connect(InetSocketAddress(parsed.host, parsed.port), connectTimeoutMs)
            val out = BufferedOutputStream(sock.getOutputStream(), reqHeader.size + bodyBytes.size)
            out.write(reqHeader)
            out.write(bodyBytes)
            out.flush()

            val inp = BufferedInputStream(sock.getInputStream(), 16 * 1024)
            return readHttpResponseBody(inp)
        }
    }

    private data class HttpEndpoint(val host: String, val port: Int, val path: String)

    private fun parseHttpEndpoint(rawUri: String): HttpEndpoint? {
        var s = rawUri.trim()
        if (s.isEmpty()) return null
        if (s.startsWith("http://", ignoreCase = true)) {
            s = s.substring(7)
        } else if (s.startsWith("https://", ignoreCase = true)) {
            s = s.substring(8)
        }
        val slashIdx = s.indexOf('/')
        val authority = if (slashIdx >= 0) s.substring(0, slashIdx) else s
        val rawPath = if (slashIdx >= 0) s.substring(slashIdx) else "/"
        if (authority.isBlank()) return null

        val colonIdx = authority.lastIndexOf(':')
        val host: String
        val port: Int
        if (colonIdx > 0) {
            host = authority.substring(0, colonIdx).trim()
            port = authority.substring(colonIdx + 1).trim().toIntOrNull() ?: return null
        } else {
            host = authority.trim()
            port = 11311
        }
        if (host.isEmpty() || port !in 1..65535) return null
        return HttpEndpoint(host, port, rawPath.ifBlank { "/" })
    }

    private fun readHttpResponseBody(inp: BufferedInputStream): String {
        val headerBuf = ByteArrayOutputStream(512)
        var state = 0 // 匹配 \r\n\r\n 或 \n\n
        var sawLfOnly = false
        while (headerBuf.size() < 16 * 1024) {
            val b = inp.read()
            if (b < 0) break
            headerBuf.write(b)
            when {
                b == '\r'.code -> {
                    state = if (state == 2) 3 else 1
                }
                b == '\n'.code -> {
                    if (state == 1) {
                        state = 2
                    } else if (state == 3) {
                        state = 4
                        break
                    } else if (sawLfOnly) {
                        state = 4
                        break
                    }
                    sawLfOnly = true
                }
                else -> {
                    state = 0
                    sawLfOnly = false
                }
            }
        }

        val headerBytes = headerBuf.toByteArray()
        if (state != 4) {
            // 兼容极少数直接裸回 XML 无 HTTP 头的测试桩
            val rest = inp.readBytes()
            val combined = ByteArray(headerBytes.size + rest.size)
            System.arraycopy(headerBytes, 0, combined, 0, headerBytes.size)
            System.arraycopy(rest, 0, combined, headerBytes.size, rest.size)
            return String(combined, Charsets.UTF_8)
        }

        val headerText = String(headerBytes, Charsets.ISO_8859_1)
        var contentLength = -1
        for (line in headerText.split('\n')) {
            val trimmed = line.trim()
            if (trimmed.startsWith("content-length:", ignoreCase = true)) {
                contentLength = trimmed.substringAfter(':').trim().toIntOrNull() ?: -1
                break
            }
        }

        if (contentLength >= 0) {
            val cap = contentLength.coerceAtMost(4 * 1024 * 1024)
            val body = ByteArray(cap)
            var offset = 0
            while (offset < cap) {
                val n = inp.read(body, offset, cap - offset)
                if (n <= 0) break
                offset += n
            }
            return String(body, 0, offset, Charsets.UTF_8)
        }

        // 无 Content-Length 头时读取至 </methodResponse> 或流末尾
        val bodyOut = ByteArrayOutputStream(4096)
        val chunk = ByteArray(4096)
        while (bodyOut.size() < 4 * 1024 * 1024) {
            val n = inp.read(chunk)
            if (n <= 0) break
            bodyOut.write(chunk, 0, n)
            if (bodyOut.size() >= 17) {
                val tail = String(
                    bodyOut.toByteArray(),
                    (bodyOut.size() - minOf(bodyOut.size(), 96)).coerceAtLeast(0),
                    minOf(bodyOut.size(), 96),
                    Charsets.ISO_8859_1
                )
                if (tail.contains("</methodResponse>")) break
            }
        }
        return String(bodyOut.toByteArray(), Charsets.UTF_8)
    }

    internal fun isVirtualOrCellularInterfaceName(rawName: String): Boolean {
        val n = rawName.trim().lowercase(Locale.US)
        if (n.isEmpty()) return false
        return n == "lo" ||
            n.startsWith("tun") ||
            n.startsWith("tap") ||
            n.startsWith("utun") ||
            n.startsWith("ppp") ||
            n.startsWith("pptp") ||
            n.startsWith("l2tp") ||
            n.startsWith("ipsec") ||
            n.startsWith("wg") ||
            n.startsWith("tailscale") ||
            n.startsWith("rmnet") ||
            n.startsWith("ccmni") ||
            n.startsWith("pdp") ||
            n.startsWith("clat") ||
            n.startsWith("v4-") ||
            n.startsWith("dummy") ||
            n.startsWith("sit") ||
            n.startsWith("ip6tnl") ||
            n.startsWith("erspan") ||
            n.startsWith("docker") ||
            n.startsWith("veth")
    }

    private fun classifyInterfacePriority(rawName: String): Int {
        val n = rawName.trim().lowercase(Locale.US)
        return when {
            n.startsWith("eth") || n.startsWith("usb") || n.startsWith("rndis") ||
                n.startsWith("ncm") || n.startsWith("ecm") || n.startsWith("enx") -> 0
            n.startsWith("ap") || n.startsWith("softap") || n.startsWith("swlan") ||
                n == "wlan1" || n.startsWith("p2p") -> 1
            n.startsWith("wlan") || n.startsWith("wifi") || n.startsWith("mlan") -> 2
            else -> 3
        }
    }

    internal fun isUsableLanIpv4(ip: String?): Boolean {
        if (ip.isNullOrBlank()) return false
        val ipInt = ipv4ToUnsignedInt(ip) ?: return false
        val a = ((ipInt ushr 24) and 0xFF).toInt()
        val b = ((ipInt ushr 16) and 0xFF).toInt()
        // 排除 0.*, 127.*, 169.254.* (DHCP 失败自分配地址), 198.18..19.* (Clash/代理 Fake-IP), 100.64..127.* (运营商 CGNAT), 组播/广播 224+
        if (a == 0 || a == 127 || a >= 224) return false
        if (a == 169 && b == 254) return false
        if (a == 198 && (b == 18 || b == 19)) return false
        if (a == 100 && b in 64..127) return false
        return isPrivateOrLanIpv4Int(ipInt)
    }

    private fun isPrivateOrLanIpv4Int(ipInt: Long): Boolean {
        val a = ((ipInt ushr 24) and 0xFF).toInt()
        val b = ((ipInt ushr 16) and 0xFF).toInt()
        return a == 10 ||
            (a == 172 && b in 16..31) ||
            (a == 192 && b == 168)
    }

    internal fun ipv4ToUnsignedInt(ip: String): Long? {
        val m = IPV4_LITERAL_REGEX.matchEntire(ip.trim()) ?: return null
        val o1 = m.groupValues[1].toIntOrNull() ?: return null
        val o2 = m.groupValues[2].toIntOrNull() ?: return null
        val o3 = m.groupValues[3].toIntOrNull() ?: return null
        val o4 = m.groupValues[4].toIntOrNull() ?: return null
        if (o1 !in 0..255 || o2 !in 0..255 || o3 !in 0..255 || o4 !in 0..255) return null
        return ((o1.toLong() shl 24) or (o2.toLong() shl 16) or (o3.toLong() shl 8) or o4.toLong()) and 0xFFFFFFFFL
    }

    private fun isSameSubnet(ipA: Long, ipB: Long, prefixLength: Int): Boolean {
        val p = prefixLength.coerceIn(8, 30)
        val mask = (0xFFFFFFFFL shl (32 - p)) and 0xFFFFFFFFL
        return (ipA and mask) == (ipB and mask)
    }

    private fun littleEndianIntToIpv4(raw: Int): String {
        val b1 = raw and 0xFF
        val b2 = (raw ushr 8) and 0xFF
        val b3 = (raw ushr 16) and 0xFF
        val b4 = (raw ushr 24) and 0xFF
        return "$b1.$b2.$b3.$b4"
    }
}
