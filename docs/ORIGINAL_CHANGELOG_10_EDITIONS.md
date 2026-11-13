# RadioMaster EGO-Pilot AX12 连续十版工程优化演进记录 (`v3.1.0` -> `v3.10.0-r10`)

本文档完整记录针对 RadioMaster AX12 遥控器、机载 Intel NUC12 / NVIDIA Jetson、[hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)、[ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 与 [Friendzhb/tilt-uav-indi](https://github.com/Friendzhb/tilt-uav-indi)（4 电机 + 2 倾转舵机双舵矢量架构）开展的连续十版实质性工程迭代优化。

## 第 1 版：`v3.1.0` — OpenGL ES 2.0 距离自适应体素几何 LOD 与三维制动预测光环
- **核心改进 (`GlPointCloudMapView.kt`)**：引入基于相机焦点平方距离 (`dFocusSq`) 的多级几何 LOD 裁剪，自动省去远景 (`>9.5m`，省电模式下 `>7.0m`) 膨胀体素的贴地接触阴影四边形与顶面微倒角棱线，在密集槟榔林场景下削减约 42% 流式 VBO 顶点吞吐；同时围绕主控飞机新增动态净空预警光环及三维动力学刹停预测矢量点 (`v^2 / 2a_max`)。

## 第 2 版：`v3.2.0` — FAST-LIO2 激光反射率量程自适应归一化与近机 ROI 驻留保护
- **核心改进 (`FastLioVoxelMap.kt`、`EgoSimEngine.kt`)**：自动识别 Livox Mid-360 / Avia (`0..255`) 与浮点 (`0.0..1.0`) 反射强度并实时归一化，解决「反射强度」配色过曝泛白问题；满 `300,000` 体素环形淘汰时优先保护无人机周围 `8.5m` 近场避障体素，并新增点云三维空间跨度 (`ΔX x ΔY x ΔZ`) 实时统计。

## 第 3 版：`v3.3.0` — 原生 TCPROS 半开僵尸连接 `SO_TIMEOUT` 看门狗与指数退避自愈重连
- **核心改进 (`TcpRosNativeClient.kt`、`GcsState.kt`)**：为高频点云/里程计流 (`6.5s`) 与低频状态流 (`14s`) 分别配置 `Socket.soTimeout` 与 `TCP KeepAlive`，根除机载节点重启或林下 Wi-Fi 瞬断引发的半开 Socket 永久假死；新增 `roscore` XML-RPC 周期心跳看门狗、指数退避自动重连 (`1.5s..8.0s`) 及实时下行吞吐率 (`rosRxKbps`) 监控。

## 第 4 版：`v3.4.0` — `Tilt-UAV-INDI` 二阶无扰切换斜率限制器与窄缝自动微仰减截面辅助
- **核心改进 (`EgoSimEngine.kt`、`GcsState.kt`)**：针对 [Friendzhb/tilt-uav-indi](https://github.com/Friendzhb/tilt-uav-indi) 与 [mpc_ros1_for_px4 PR #2](https://github.com/Friendzhb/mpc_ros1_for_px4/pull/2) 双舵矢量模型加入 `40 deg/s` 斜率限制无扰切换器 (`smoothBiasPitchRad`)，消除「普通 X4 <-> 倾转矢量」或大角度偏置切换时的 `300 deg/s` 舵机速率饱和尖峰；新增窄缝 (`净空 < 0.88m`) 自动柔和抬头减截面辅助与实时悬停推力裕度 (`tiltThrustMarginPct`) 解算。

## 第 5 版：`v3.5.0` — 多机集群三维旋翼下洗气流椭球防撞势场与掉线僚机质心隔离
- **核心改进 (`EgoSimEngine.kt`)**：将 [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 多机机间斥力由各向同性球体升级为垂直轴 `1.92x` 拉长的三维旋翼下洗椭球防撞场 (`downwashEllipsoidDist`) + 右手规则切向滑移，杜绝上下叠飞卷入下洗尾流；实机 ROS1 模式下自动剔除超时离线僚机对编队质心与航向解算的污染。

## 第 6 版：`v3.6.0` — RadioMaster AX12 物理按键盲操热键与 ELRS MAVLink 姿态双链路热备
- **核心改进 (`Ax12HardwareBridge.kt`、`MainActivity.kt`)**：在 UDP `:14550` 通道新增零分配 MAVLink v1/v2 `HEARTBEAT (#0)` 与 `ATTITUDE (#30)` 解析，当 5GHz ROS1 Wi-Fi 在密林遮挡逾 `1.5s` 中断时自动无缝接管三维姿态与解锁状态显示；并在非输入框焦点下映射 AX12 物理音量/滚轮缩放、方向键高度步进与空格/媒体停止键紧急悬停。

## 第 7 版：`v3.7.0` — 20Hz 科研级飞行黑匣子同步记录仪 (`FlightTelemetryRecorder.kt`) 与一键 CSV 导出
- **核心改进 (`FlightTelemetryRecorder.kt`、`MainActivity.kt`)**：新增固定预分配 `24,000` 帧环形遥测记录器，以 20Hz 零 GC 同步采集位姿、速度、姿态角、`S1/S2` 舵机实测与指令角、四电机分配推力、障碍净空、机间距离及电池功率共 28 列指标，支持在「状态」抽屉一键启停并导出标准 UTF-8 `.csv` 供论文实验绘图。

## 第 8 版：`v3.8.0` — 编排航线断点持久化存储与「一键三维净空自检越障修正」
- **核心改进 (`EgoSimEngine.kt`、`MainActivity.kt`)**：多航点任务队列支持跨应用重启自动持久化保存与恢复；「航线」抽屉新增「自检」按钮 (`sanitizeOrchestratedRouteClearance`)，一键扫描全部航点与相邻航段中点，对侵入膨胀障碍区的航点自动执行邻域安全重定位或抬高越障修正。

## 第 9 版：`v3.9.0` — HSI 避障球制动停止点风险着色、多机僚机雷达光点与雨林防误触锁
- **核心改进 (`DjiHudWidgets.kt`、`MainActivity.kt`)**：左下角 HSI 避障球新增多机分色僚机方位距离光点与前向扇区碰撞风险变色制动预测点；右侧「图层」浮窗新增「防误触锁」(`hudTouchLocked`) 与「窄缝微仰」快捷开关，锁屏携行或林下雨滴溅落时安全拦截误触选点，同时保持「紧急悬停」100% 随时响应。

## 第 10 版：`v3.10.0-r10` — OTA 跨源 HTTP 206 断点续传、ZIP EOCD 尾块完整性校验与 CI 签名摘要加固
- **核心改进 (`OtaUpdateManager.kt`、`build.gradle.kts`、`proguard-rules.pro`、`build-apk.yml`)**：升级应用内更新器支持加速源切换时的 `Range: bytes=<offset>-` (`HTTP 206`) 断点续传，并新增末尾 `65KB` ZIP `EOCD (0x06054b50)` 归档尾签名校验以彻底拦截截断残包；版本号升至 `3.10.0-r10`，完善 R8 混淆规则与 GitHub Actions `SHA-256` 校验文件自动发布。
