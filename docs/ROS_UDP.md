# ROS1 → UDP 遥测

版本 3.12.0 增加可选的 UDP 点云和里程计入口，默认关闭。
航点、参数、倾转指令、飞控状态、电池和其他 ROS 话题继续使用 TCPROS/rosbridge。
无需修改飞控固件。

## 使用

1. AX12 与机载电脑连接同一网络，记下 AX12 的 Wi-Fi IP。
2. 连接页填写机载地址，勾选“UDP 点云/位姿”，保留默认端口 `14560`，然后连接。
3. 将 `scripts/ros_udp_bridge.py` 放到运行 ROS1 的机载电脑上，加载 ROS 环境后运行：

```bash
source /opt/ros/noetic/setup.bash
python3 ros_udp_bridge.py --target <AX12的IP> --port 14560 \
  --odom-topic /Odometry --cloud-topic /cloud_registered
```

桥接只订阅 ROS、发送显示数据，不接收 UDP 指令，也不访问飞控串口。
来源地址必须与地面站填写的 ROS Master/rosbridge 主机一致。
坐标约定与原 TCPROS 入口相同：位置、线速度与注册点云在同一世界坐标系，Z 朝上；桥接不做 TF 变换。

默认点云最高 10 Hz、每次最多 8000 点；里程计最高 50 Hz。
这些是发送上限，不会将原生 10 Hz 数据变成 50 Hz 测量。
`--cloud-hz` 可设到 15，`--odom-hz` 可设到 100，`--max-points` 最大 8000。
待编码点云仅保留最新一条，采样覆盖整幅输入，支持行填充、大小端、FLOAT32/FLOAT64 坐标。

## 超时与切回

- UDP 位姿在最近 500 ms 内有效时优先使用；超时后原 TCPROS/rosbridge 位姿自动接续。可靠位姿订阅始终保留。
- 启用 UDP 后不再自动订阅主机的 TCP/WebSocket 全局点云，避免重复传输。关闭选项并重新连接即可恢复原点云入口。
- 接收会话在同一工作执行器上串行切换，旧接收循环退出后才启动新监听；界面不等待网络线程退出。
- 连接页显示 UDP 点云是否实时、位姿来源及丢弃计数。750 ms 未收到完整点云即显示等待/过期。
- 不完整或校验失败的点云整帧丢弃，不重传；旧地图保留。UDP 不能保证比 TCP 更快，高丢包网络可能降低有效点云频率。
- 新鲜度使用本机单调时钟。发送时间字段不用于假定两台设备时钟已同步，也不显示未经同步验证的端到端延迟。
- 此入口用于可信局域网，没有加密或身份认证。来源 IP 检查和 CRC 用于筛选与完整性检查。

## 二进制协议 v1

所有整数、浮点均为小端；单个 UDP 数据报最多 1200 字节。
固定 36 字节头对应 Python `struct.Struct("<4sBBHIIHHIIQ")`：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| magic / version / kind / reserved | 4字节 / u8 / u8 / u16 | `EGUD` / 1 / 1位姿或2点云 / 0 |
| session / sequence | u32 / u32 | 启动随机会话、各类消息独立递增序号 |
| fragment_index / fragment_count | u16 / u16 | 从0开始的分片编号和总片数 |
| total_bytes / crc32 | u32 / u32 | 完整载荷长度、完整载荷CRC32 |
| sender_monotonic_ms | u64 | 发送方单调时间，整帧各片一致 |

位姿载荷为10个 float32：`x y z vx vy vz qw qx qy qz`。
点云载荷为连续的 float32 `x y z intensity`，最多8000点。
每片载荷最多1164字节；片数和各片长度必须与完整长度精确匹配。
接收端每种消息最多保留一帧待组装数据，150 ms 未组齐即拒绝；更新序号可取代旧帧。
已完成帧和退役会话不会重复应用；序号支持 u32 回绕。

## 验证范围

Python 自测可在没有 ROS 的机器上运行：

```bash
python3 -B -m unittest discover -s scripts -p 'test_*.py' -v
```

Kotlin 测试覆盖 Python 位姿协议样本、乱序/重复/丢片、CRC、超时、过大载荷及会话切换。
Android 集成测试使用真实 UDP 回环套接字接入生产接收器和生产引擎，验证位姿与超时回退。
这些测试不替代 AX12 与机载 ROS、实际 Wi-Fi 点云的联调；本次没有在 N2 自动安装或启动桥接。
