# 智航 (RadioMaster AX12 激光避障 · 双舵倾转 · 多机集群原生地面站)

本工程为 **100% 纯原生 Kotlin + [OpenGL ES](https://developer.android.com/develop/ui/views/graphics/opengl/about-opengl) 硬件加速安卓应用（无任何 WebView、无 HTML 套壳文件、全界面零 Emoji 表情符号）**，专为 [RadioMaster AX12](https://radiomasterrc.com/products/ax12-android-radio-controller)（MediaTek MT8788 八核处理器、ARM Mali-G72 GPU、4GB RAM、5.5 英寸 `1280×720` 1000nits 高亮触控屏，见 [AX12 规格手册](https://manuals.plus/radiomaster/ax12-android-radio-controller-manual)）打造，界面与设置逻辑参照 [DJI Pilot 2](https://dl.djicdn.com/downloads/matrice-30-series/20230922UM/Matrice30_Series_User_Manual_v2.0_EN.pdf)、[DJI Fly 系统设置规范](https://support.dji.com/help/content?customId=en-us03400006562&spaceId=34&re=US&lang=en&documentType=artical&paperDocType=paper) 与 [QGroundControl Settings View](https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-user-guide/settings_view/settings_view.html)。

## 1. 成熟商业引擎级「单层外皮深度预通透镜 + 双色对比虚线 X-Ray」透视管线 (`GlPointCloudMapView.kt`)

- **外表面深度预通单层光学透镜（Depth-Only Prepass + `GL_EQUAL` Single-Skin Cutaway，参照 [Depth Prepass 半透明外表面消隐方案](https://medium.com/design-bootcamp/transparency-the-rasterization-nightmare-71ceb98a8e3c) 与 [Visibility Equalizer Cutaway](https://pmc.ncbi.nlm.nih.gov/articles/PMC5364803/)）**：先绘制 100% 不透明远景体素与前景实体飞机/航线，再对视线走廊内的遮挡膨胀体素执行 `glColorMask(false)` 硬件深度预通 + `GL_EQUAL` 单层外皮着色，100% 硬件剔除相邻体素内部所有共享内壁与多层混浊叠影；片元着色器逐像素计算胶囊视线距离与全息边缘光晕，消除整块体素跳变，配合点云 4×4 Bayer 有序抖动消隐 ([Ordered Dithering](https://blog.maximeheckel.com/posts/the-art-of-dithering-and-retro-shading-web/)) 实现丝滑通透的立体视野。
- **硬件 `GL_GREATER` 双色抗锯齿流动虚线与高程接触阴影**：严格遵循 [OpenGL Depth Testing](https://learnopengl.com/Advanced-OpenGL/Depth-testing) 逆深度测试，被实体墙或透视薄壳遮挡的规划轨迹自动切换为「连续微光底轨 + 高对比向前流动胶囊虚线」，被遮挡机体呈现稳态全息轮廓，并结合 51° 自然透视视场、柱顶微倒角与底部接触阴影强化三维空间纵深感。

## 2. 全界面响应式五区无碰撞对齐、三维实时姿态与全兼容原生 ROS1 话题监控

- **五区零重叠无截断对齐与四大飞控设置 (`MainActivity.kt` + `GcsSettingsModal.kt`)**：锁定 `fontScale = 1.0f` 防系统大字体撑破座舱，重构 36dp 顶栏、顶部居中流式堆栈、对称双仪表底栏与空间避让协调器，并将设置收敛为「飞行安全 · 感知与飞控 · 通信链路 · 显示与通用」四大实战分类。
- **三维实时姿态视窗与兼容 C++ `roscpp` / `rospy` 的原生 ROS1 话题监控 (`UavAttitudeHudView.kt` + `RosTopicInspector.kt`)**：实时解算 `/Odometry` 四元数姿态；严格兼容 [ROS xmlrpcpp XmlRpcValue](https://docs.ros.org/en/jade/api/xmlrpcpp/html/XmlRpcValue_8cpp_source.html) 无 `<string>` 标签协议、[ROS Master XML-RPC API](https://wiki.ros.org/ROS/Master_API) 与 [TCPROS Connection Header](https://wiki.ros.org/ROS/Connection%20Header)，支持核心数据流零额外带宽旁路直读、自动重写非 IPv4 主机名至 `roscore` IP、话题单次截取、持续读取与一键绑定通道；零串口冲突（不触碰 `/dev/ttyS0`，见 [ax12-research](https://github.com/rmeadomavic/ax12-research)），兼容 [ELRS Backpack](https://github.com/rmeadomavic/ax12-research/blob/main/docs/hardware/elrs-backpack.md) [MAVLink](https://mavlink.io/en/messages/common.html) `UDP :14550`、原生 [TCPROS](https://wiki.ros.org/ROS/TCPROS)、[EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner/issues/106) 与 [FAST-LIO2](https://github.com/hku-mars/fast_lio)，上传仓库即可由 [GitHub Actions](https://docs.github.com/actions) 自动构建发布 APK。

## 3. 云端实时加速源自动同步与真包测速优选 OTA 在线更新系统 (`OtaUpdateManager.kt`)

- **云端镜像注册表实时同步 + 并发 APK 魔数真伪测速 + 多节点故障自动切换**：默认启用「智能实时加速」模式，动态从定时自动巡检的 [MirrorHub 加速节点仓库](https://github.com/mihsian77/MirrorHub) 多路 CDN、[GHProxy 备用域名索引](https://ghproxy.link/) 与 [GitHub Proxy 聚合站](https://github.akams.cn/) 拉取最新活跃镜像列表，合并 [GH-Proxy](https://gh-proxy.com/)、[GHFast](https://ghfast.top/) 等高可用节点与官方最新构建直链 ([EGO-Pilot-AX12.apk](https://github.com/Friendzhb/RadioMaster-EGO-GCS/releases/download/latest-apk/EGO-Pilot-AX12.apk)) 并发发起 4 字节 `Range: bytes=0-3` 探测，严格校验 `PK\x03\x04` 归档魔数以剔除失效域名与认证网页，按实测端到端延迟优选最快节点并在断流时自动无缝切换备选源；同时支持 [GitHub Releases REST API](https://docs.github.com/en/rest/releases/releases) 镜像加速比对与自定义镜像前缀/直链输入。
- **固定签名证书、包体完整性校验与飞行安全互锁**：工程内置固定签名证书 (`app/keystore/ego_pilot_ota.jks`) 与编译时间自动递增 `versionCode`；下载完成后二次校验 APK 魔数与包名，通过 [Android FileProvider](https://developer.android.com/training/secure-file-sharing/setup-sharing) 与 [REQUEST_INSTALL_PACKAGES](https://support.google.com/googleplay/android-developer/answer/12085295?hl=en) 安全拉起系统覆盖安装，并在实机解锁飞行状态下自动锁定安装以确保安全。

## 4. EGO-Planner-Swarm 多机协同编队模式 (`EgoSimEngine.kt` + `GlPointCloudMapView.kt` + `TcpRosNativeClient.kt`)

- **单机 / 2~4 架多机一键切换与四队形避障集结**：支持「单机模式」与「2 / 3 / 4 架多机集群」无缝切换，内置「三角楔形、横向一字、穿林纵队（适配高密果园/林下行间窄通道）、菱形方阵」四种编队几何解算与机间互避速度势场，支持 3D 场景直接点选无人机切换主控视角、分色 B 样条轨迹、机间最近净空拓扑网与全队一键编队飞往/返航/急停。
- **双拓扑实机 ROS1 集群直连**：同时支持 [ZJU-FAST-Lab EGO-Planner / EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner) 标准同 Master 多命名空间（`/drone_{id}_visual_slam/odom`、`/drone_{id}_ego_planner_node/optimal_list`）与多机载电脑独立 IP `roscore :11311` 并行直连，局域网扫描发现的多台无人机可直接一键编入集群。

## 5. `Tilt-UAV-INDI` 双舵倾转四旋翼全栈专属适配 (`DroneVectorModel.kt` + `EgoSimEngine.kt` + `TcpRosNativeClient.kt`)

- **真实 4 电机 + 2 沿 $Y_b$ 轴倾转舵机物理构型与三维实时矢量渲染**：严格对照用户仓库 [Friendzhb/tilt-uav-indi](https://github.com/Friendzhb/tilt-uav-indi)（实装变体 `g1_indi_xyz`，见 [firmware/current-source](https://github.com/Friendzhb/tilt-uav-indi/tree/main/firmware/current-source) 与 [actuator_model_bank_20261109](https://github.com/Friendzhb/tilt-uav-indi/blob/main/reports/actuator_model_bank_20261109/ARCHIVE_RECEIPT.json)）、[Friendzhb/mpc_ros1_for_px4 PR #2](https://github.com/Friendzhb/mpc_ros1_for_px4/pull/2) 及 [Friendzhb/TerrainScan-NMPC PR #10](https://github.com/Friendzhb/TerrainScan-NMPC/pull/10) 构建 `TILT-INDI 4M2S` 机体模型——舵机 1 (`S1`) 横向连杆控制前组电机 `M1/M3` 前倾/后倾，舵机 2 (`S2`) 横向连杆控制后组电机 `M2/M4` 前倾/后倾；右下角三维姿态视窗与主界面 3D OpenGL 场景同步渲染实时偏转的旋翼护圈、桨盘及三维推力矢量箭，并内置标定硬件动力学（电机一阶 $\tau_m=28\text{ ms}$、二阶欠阻尼舵机 $\tau_s=75\text{ ms}, \zeta=0.68$、`300 deg/s` 速率限幅重分配与可操作度指标 $w$）。
- **三模态姿态-倾转协同、槟榔窄缝场景与 ROS1 原生接口**：支持「零俯仰矢量平飞（锁定水平机身稳激光雷达视场）、主动俯仰-倾转协同（适配 [active_pitch_planner_node.cpp](https://github.com/Friendzhb/tilt-uav-indi/blob/main/src/active_pitch_planner_node.cpp) 与 [px4_ego_live_bridge.cpp](https://github.com/Friendzhb/tilt-uav-indi/blob/main/src/px4_ego_live_bridge.cpp) Jerk 超前补偿及 `-25°..+25°` 窄缝主动俯仰偏置）、倾转归零常规模式」一键切换与「槟榔窄缝」穿林仿真场景；原生 `TCPROS` 与 `rosbridge` 双栈支持一键加载 `Tilt-INDI` 话题模板、局域网自动指纹识别，以及自适应解码 `Float32MultiArray` / `Float64MultiArray` / `Vector3Stamped` / `JointState` / `mavros_msgs/ActuatorControl` 倾转遥测并实时下发 `/active_pitch_planner/pitch_tilt_cmd` 控制指令。

## 6. 单机（矢量 / 普通）与多机（全普通 / 全矢量 / 异构混编）全兼容热切换架构

- **五模态运行时期无缝热切换**：座舱顶栏同时提供独立的「单机 / 多机 N 架」胶囊与「矢量 4M2S / 普通 X4（多机下循环切换全矢量 / 全普通 / 异构混编）」胶囊，抽屉「航线」页顶部内置 `单机·普通` / `单机·矢量` / `多机·普通` / `多机·矢量` 四宫格一键切换矩阵及按机位掩码独立分配 (`#0..#3` 各自可独立指定为双舵倾转矢量机或普通固定 X4 四旋翼)，左下角遥测栏亦支持四态闭环点按切换且绝不锁死。
- **三维渲染、动力学状态机与 ROS1 话题完全解耦**：切换为普通 `X4` 构型的单机或集群成员在 `EgoSimEngine.kt` 内立即清零残留舵机倾角、角速度与饱和标志并恢复标准欠驱动滚转/俯仰耦合，在 `GlPointCloudMapView.kt` 与 `UavAttitudeHudView.kt` 中实时切换为无倾转连杆的标准四旋翼模型，同时自动屏蔽 `/mavros/actuator_control` 辅助通道干扰；多机模式下原生 `TCPROS` 与 `rosbridge` 自动按各机构型按需订阅 `/drone_{id}_tilt_uav/servo_angles` 并定向下发 `/drone_{id}_active_pitch_planner/pitch_tilt_cmd`（兼容 [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 与 [Friendzhb/tilt-uav-indi](https://github.com/Friendzhb/tilt-uav-indi) 混合编队联飞）。

## 7. `v3.10.0-r10` 连续十版工程迭代优化摘要 (详见 `CHANGELOG_10_EDITIONS.md`)

- 已依次完成从 `v3.1.0` 至 `v3.10.0-r10` 的十轮全栈实质性优化：涵盖 OpenGL ES 2.0 距离自适应体素 LOD 与三维制动预测光环、FAST-LIO2 反射率量程自适应与 `8.5m` 近机体素驻留保护、TCPROS 半开僵尸流 `SO_TIMEOUT` 看门狗与指数退避自愈重连、`Tilt-UAV-INDI` 二阶无扰切换斜率限制与窄缝自动微仰减截面、多机三维旋翼下洗椭球防撞势场、AX12 物理按键盲操与 ELRS MAVLink 姿态热备、20Hz 科研 CSV 黑匣子记录仪、航线持久化与净空自检修正、HSI 僚机光点与雨林防误触锁，以及 OTA `HTTP 206` 断点续传与 ZIP `EOCD` 尾签名校验。
