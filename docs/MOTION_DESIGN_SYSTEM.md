# 智航 (RadioMaster EGO GCS) 交互动效设计系统与后期迭代规范

本文档定义 **RadioMaster EGO GCS (`智航`)** 的全局交互动效架构、物理参数字典、全量操作映射与后期开发治理红线。所有新增或修改的界面控件、面板过渡、三维视口交互及安全确认流程，必须严格遵循本规范并通过唯一入口 `GcsMotionSpec` 与 `AnimHelper` 编排实现。

---

## 1. 设计目标与航空级交互五大原则

面向 RadioMaster AX12 遥控器户外强光、单手盲操与飞行高专注度作业场景，地面站动效严禁娱乐化炫技，必须服务于**态势感知、空间定位与安全防呆**：

1. **低认知负荷与零视线干扰（Zero Visual Distraction）**
   - 常规交互反馈控制在 `125ms ~ 255ms` 内收敛，杜绝长尾拖影遮挡 3D 点云障碍物或飞行姿态读数。
   - 姿态球、HSI 罗盘与高频遥测文本严禁附加花哨透明闪烁；仅在用户主动切换模式/量程时执行受控平滑过渡。
2. **速度连续与可打断物理弹簧（Interruptible Physics）**
   - 所有按压、抽屉开合、滑块位移、底部指令坞升降均采用 `SpringAnimation` 并经由 `(View, ViewProperty)` 弱引用池单例复用。
   - 用户高频连点、中途反向滑动或快速切换标签时，自动调用 `animateToFinalPosition()` 继承瞬时速度，严禁出现跳帧重置或多动画实例抢占抖动。
3. **空间因果与方向一致性（Spatial & Directional Causality）**
   - 容器展开必须从触发源锚点生长（如图层浮窗锚定右上工具按钮、指令坞自底栏弹出、抽屉自右缘滑入）。
   - 横向页签（4 大作业页签）、纵向分类（设置左侧 6 大分组）、二级钻取（ROS1 话题列表 ↔ 实时终端详情）以及高度 `+/-` 微调读数，移动方向必须与索引差值或数值增减方向严格一致。
4. **安全分级多模态反馈（Safety-Graded Multimodal Confirmation）**
   - 按操作风险将视觉形变与触觉震动划分为五级按压角色（`PressRole`）与六级语义震动（`HapticRole`）。
   - **高危指令**（返航、清空建图、恢复出厂）首次点击触发待命警示弹跳（`WARNING_ARM`），二次确认触发提交震动；**急停悬停**采用高刚度初速度压缩制动回弹；**拦截拒绝**（防误触锁开启、障碍物内选点、空航线启动）统一采用阻尼衰减水平摇头抖动（`animateRejectShake`）。
5. **功耗与系统无障碍自适应降级（Power & Accessibility Adaptation）**
   - 内置三档全局动效策略，当渲染处于「省电 30帧」(`perfMode == 2`) 或系统关闭动画（`ValueAnimator.areAnimatorsEnabled() == false`）时自动降级至极简临界阻尼档位，关闭级联瀑布与微脉冲以节约 CPU/GPU 合成开销。

---

## 2. 全局动效架构与三档运行策略

### 2.1 分层职责

| 层级 | 核心文件 | 职责边界 |
| :--- | :--- | :--- |
| **Token 与物理引擎层** | `ui/GcsMotionSpec.kt` | 唯一事实源（Single Source of Truth）：统一管理时长阶梯、标准缓动曲线、物理弹簧刚度/阻尼、按压角色、触觉分级、档位缩放公式及 `WeakHashMap` 弹簧复用池。 |
| **交互编排层** | `ui/AnimHelper.kt` | 封装面向业务语义的标准动效原语（按压绑定、开关流体滑块、分段锁入、二次确认脉冲、拦截抖动、方向翻页、抽屉宽屏伸缩、HUD 空间避让等）。 |
| **渲染视口插值层** | `GlPointCloudMapView.kt` / `DjiHudWidgets.kt` / `UavAttitudeHudView.kt` | OpenGL / Canvas 内部相机环绕、2D/3D 投影平滑变形、指点光柱弹落与 HSI 量程/航向融合，时长与曲线统一读取 `GcsMotionSpec`。 |
| **状态持久化与设置层** | `model/GcsState.kt` / `ui/GcsSettingsModal.kt` | 维护 `hub.animStyle` 与 `hub.perfMode` 持久化，启动及变更时调用 `AnimHelper.syncMotionContext(hub.animStyle, hub.perfMode)` 同步全局策略与 3D 相机阻尼倍率。 |

### 2.2 三档全局动效策略

用户可在 **「设置 → 显示与通用 → 全局交互动效档位」** 实时切换并持久化保存：

| 档位常量 | UI 名称 | 时长倍率 | 弹簧刚度倍率 | 阻尼修正 | 列表错峰步进 | 3D 相机阻尼倍率 | 适用场景与自动联动规则 |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| `PROFILE_STANDARD (0)` | **标准物理** | `1.00x` | `1.00x` | 基准阻尼 | `18ms` | `1.00x` | 默认档位；60FPS 自然物理回弹 + 容器卡片微错峰入场。 |
| `PROFILE_SNAPPY (1)` | **敏捷利落** | `0.68x` | `1.45x` | `+0.08` (≤`0.96`) | `12ms` | `1.32x` | 户外强光高频操控；高刚度紧凑收敛，不拖泥带水。 |
| `PROFILE_MINIMAL (2)` | **极简省电** | `0.34x` | `2.45x` | 固定 `0.98` | `0ms`（关闭） | `1.65x` | 临界阻尼瞬态到位，关闭微脉冲与级联瀑布；当 `perfMode == 2`（省电 30帧）且未手动锁定敏捷档，或系统关闭动画时**自动切换至此档**。 |

---

## 3. 标准动效 Token 字典

### 3.1 时长阶梯（`GcsMotionSpec.DurationToken`）

| Token | 标准档基准时长 | 最小下限保护 | 标准用途 |
| :--- | :---: | :---: | :--- |
| `INSTANT` | `65ms` | `35ms` | 极短瞬态保护与微状态切变 |
| `MICRO` | `125ms` | `45ms` | 顶部悬浮徽章离场、HUD 防碰撞退让淡出、折叠区收起、指令坞收起 |
| `FAST` | `160ms` | `55ms` | 抽屉/模态框/图层浮窗收起、HUD 避让恢复、悬浮徽章/指令坞入场透明渐显 |
| `STANDARD` | `195ms` | `70ms` | 抽屉/模态框/浮窗展开、横向页签与纵向分类切换、二级页面推入弹出、宽屏宽度插值、Toast 入场 |
| `SPATIAL` | `255ms` | `85ms` | 三维矢量姿态视窗四视角环绕、3D 地图指点光柱弹落、HSI 雷达量程切换 |
| `EMPHASIZED` | `340ms` | `110ms` | 3D 透视 ↔ 2D 正交投影连续变形、HSI 机头朝上 ↔ 正北朝上旋转融合 |

### 3.2 标准缓动曲线（`GcsMotionSpec`）

| 曲线常量 | 参数定义 | 适用规则 |
| :--- | :--- | :--- |
| `EASE_SPATIAL_OUT` | `PathInterpolator(0.22f, 1.0f, 0.36f, 1.0f)` | **入场与空间展开唯一标准曲线**：初速充沛、末端平滑减速，用于面板展开、列表错峰、相机视角环绕、HSI 融合。 |
| `EASE_EXIT_IN` | `PathInterpolator(0.40f, 0.0f, 1.0f, 1.0f)` | **离场与收起唯一标准曲线**：加速撤离屏幕，避免半透明残影滞留遮挡地图视野。 |
| `EASE_STANDARD` | `PathInterpolator(0.20f, 0.0f, 0.0f, 1.0f)` | **原地属性过渡曲线**：用于日夜/战术主题切换交叉溶解等无位移状态过渡。 |
| `EASE_PIN_DROP` | `OvershootInterpolator(1.42f)` | **3D 空间落点锚定曲线**：专用于地图双击/指点光柱与航点信标落地的空间弹性确认。 |

### 3.3 物理弹簧矩阵（`GcsMotionSpec.SpringToken`）

| Token | 基准刚度 (`stiffness`) | 基准阻尼比 (`dampingRatio`) | 物理质感与适用对象 |
| :--- | :---: | :---: | :--- |
| `PRESS_PILL` | `660f` | `0.72f` | 标准胶囊功能按钮按压与松手回弹 |
| `PRESS_CHIP` | `720f` | `0.76f` | 分段选择器、图层过滤小芯片、步进微调键、滑块读数抬升 |
| `PRESS_ICON` | `740f` | `0.68f` | 紧凑图标按钮（顶栏设置齿轮、关闭 `✕`、缩放 `+/-`、航点上移/下移/删除） |
| `PRESS_INSTRUMENT` | `560f` | `0.80f` | 大面积仪表组件（左下 HSI 雷达、右下三维姿态窗、链路遥测框、ROS 话题行）沉稳微缩 |
| `PRESS_SAFETY` | `800f` | `0.64f` | 安全关键按钮（紧急悬停、智能返航、清空建图、恢复出厂）高刚度清脆反馈 |
| `DETENT_POP` | `620f` | `0.66f` | 分段档位锁入微弹（`animateStateFlip`）、高度读数步进微跳（`animateValueStepNudge`） |
| `WARNING_ARM` | `540f` | `0.56f` | 高危二次确认首次点击待命态警示脉冲（`1.06x → 1.00x`） |
| `SWITCH_THUMB` | `580f` | `0.76f` | 拨动开关滑块水平位移 + 运动瞬态 `1.16x` 流体拉伸归位 |
| `GLIDER_TAB` | `500f` | `0.82f` | 抽屉顶栏 4 页签水平滑块、设置左侧导航指示条纵向伸展 |
| `SPATIAL_DOCK` | `460f` | `0.82f` | 底部上下文指令坞弹出与缩放定位 |
| `SPATIAL_MODAL` | `450f` | `0.82f` | 居中全局设置模态卡片缩放展开 |
| `SPATIAL_DRAWER` | `420f` | `0.85f` | 右侧抽屉滑入滑出、顶部居中状态堆栈防碰撞联动平移 |
| `GEAR_ROTATE` | `360f` | `0.76f` | 顶栏设置按钮开合 `0° ↔ 75°` 机械旋转啮合 |

### 3.4 五级按压角色（`GcsMotionSpec.PressRole`）与六级触觉反馈（`GcsMotionSpec.HapticRole`）

| `PressRole` | 标准按压缩放 | 极简档缩放 | 绑定弹簧 | 按下自动触觉 (`HapticRole`) | 典型控件 |
| :--- | :---: | :---: | :--- | :--- | :--- |
| `PILL_BUTTON` | `0.94f` | `0.970f` | `PRESS_PILL` | `DETENT_SELECT` (`VIRTUAL_KEY`) | 巡航/演示、顶栏胶囊、立即飞往、加入航线、主操作按钮 |
| `SEGMENT_CHIP` | `0.95f` | `0.975f` | `PRESS_CHIP` | `TICK_STEP` (`CLOCK_TICK`) | 3D/跟随/透视切换、分段单选按钮、图层芯片、高度 `+/-` |
| `ICON_CONTROL` | `0.89f` | `0.950f` | `PRESS_ICON` | `TAP_LIGHT` (`KEYBOARD_TAP`) | 齿轮按钮、关闭按钮、视距缩放键、航点行内排序/删除键 |
| `HUD_INSTRUMENT` | `0.968f` | `0.985f` | `PRESS_INSTRUMENT` | `TAP_LIGHT` (`KEYBOARD_TAP`) | HSI 雷达盘、三维姿态球视窗、图传链路遥测框、话题列表行 |
| `CRITICAL_SAFETY` | `0.91f` | `0.955f` | `PRESS_SAFETY` | `WARNING_ARM` (`CONTEXT_CLICK`) | 紧急悬停、智能返航、清空建图、恢复出厂默认 |

补充语义触觉：
- `CRITICAL_COMMIT` (`LONG_PRESS`)：紧急悬停立即触发、二次确认最终执行、立即起飞/启动航线提交。
- `REJECT_BLOCKED` (`REJECT` / API < 30 回退 `LONG_PRESS`)：防误触锁拦截、障碍物内无效落点、空航线启动拒绝。

---

## 4. 全量用户操作—动效映射总表

### 4.1 顶栏与左侧飞行安全栏

| 用户操作 | 动效编排函数 / 机制 | 视觉与触觉表现 |
| :--- | :--- | :--- |
| 点击顶栏「⚙ 设置」打开/关闭设置 | `attachSpringTouch(ICON_CONTROL)` + `animateGearRotation` + `animateSettingsModal` | 图标弹性按压并旋转 `75°`，背景遮罩渐显，居中卡片以 `SPATIAL_MODAL` 弹簧自 `0.93x` 抬升展开；关闭时 `EASE_EXIT_IN` 加速收起且齿轮归零。 |
| 点击顶栏「触控锁 🔒」 | `attachSpringTouch(PILL_BUTTON)` + `animateStateFlip` | 胶囊档位锁入微弹 + 状态色切换 + Toast 横幅滑入提示。 |
| 触控锁开启时误触左侧安全指令/图层开关/双击地图 | `animateRejectShake` + `showDjiToast` | 顶栏触控锁胶囊（或目标控件）执行 2 周期阻尼衰减水平摇头抖动 + `REJECT_BLOCKED` 震动预警。 |
| 点击顶栏「链路状态胶囊 / 作业抽屉开关」 | `attachSpringTouch(PILL_BUTTON)` + `animateDrawer` | 右侧抽屉 `SPATIAL_DRAWER` 弹簧滑入（硬件层加速，结束自动释放），右侧视口工具组向右加速避让淡出，顶部居中状态堆栈同步左移防重叠，当前页签卡片错峰浮现。 |
| 点击左侧「急停 (紧急悬停)」 | `attachSpringTouch(CRITICAL_SAFETY)` + `animateEmergencyBrake` | `0.88x` 强压缩配合 `startVelocity = 3.2f` 高刚度瞬时回弹 + `CRITICAL_COMMIT` 强震动确认。 |
| 点击左侧「返航」首次待命 / 二次确认 | `animateArmedConfirm(armed = true/false)` | 首次点击放大至 `1.06x` 欠阻尼警示回弹（琥珀待命态 + `WARNING_ARM` 震动）；3.5s 内二次点击执行返航并触发 `CRITICAL_COMMIT`；超时自动柔和复位。 |
| 点击左侧「巡航 / 演示环境」切换 | `attachSpringTouch(PILL_BUTTON)` + `animateStateFlip` | 档位锁入弹性缩放 + 激活描边高亮；若空航线启动巡航则触发按钮 `animateRejectShake` 摇头拦截。 |

### 4.2 3D 点云地图、HSI 雷达与三维矢量姿态视窗

| 用户操作 | 动效编排函数 / 机制 | 视觉与触觉表现 |
| :--- | :--- | :--- |
| 单指旋转 / 双指捏合平移 / 惯性滑行 | `GlPointCloudMapView` 阻尼滤波 (`cameraDampingScale`) | 相机方位角、俯仰角、缩放距离按当前动效档位阻尼平滑收敛；自由漫游偏离机体时顶部「回中提示胶囊」通过 `animateFloatingBadge` 滑入。 |
| 点击顶部「回中提示胶囊」或右侧「跟随」 | `attachSpringTouch` + `animateStateFlip` + `animateFloatingBadge(show = false)` | 相机平滑追回无人机机体焦点，悬浮回中胶囊加速上滑淡出。 |
| 切换「3D 透视 ↔ 2D 俯视」 | `GlPointCloudMapView.animateProjectionTo` + `animateStateFlip` | `EMPHASIZED (340ms)` + `EASE_SPATIAL_OUT` 连续插值透视/正交矩阵与俯仰角，无硬切跳变。 |
| 地图双击选点 / 航点落点 | `GlPointCloudMapView.triggerPinSpringAnim` + `animateCommandDock` | 有效落点：`SPATIAL (255ms)` + `EASE_PIN_DROP` 光柱弹落，底部上下文指令坞 `SPATIAL_DOCK` 弹簧升起（底部 HUD 两侧自动 `animateHudAvoidance` 退让）；若点选在障碍物内：底部栏/指令坞执行 `animateRejectShake` 拒绝抖动。 |
| 点击左下「HSI 综合航向仪」切换机头/北向、长按切换量程 | `attachSpringTouch(HUD_INSTRUMENT)` + `DjiHsiRadarView.animateNorthBlend` / `animateRangeScale` | 罗盘刻度盘 `EMPHASIZED` 平滑旋转融合；量程圈 `SPATIAL` 连续缩放过渡 + `DETENT_SELECT` 触觉刻度。 |
| 点击右下「三维矢量姿态视窗」切换环绕视角 / 倾转模式胶囊 | `attachSpringTouch(HUD_INSTRUMENT)` + `UavAttitudeHudView.cycleInspectCamera` | 四预设机位（后上追尾 / 前向迎头 / 侧向检视 / 垂直俯视）以 `SPATIAL` + `EASE_SPATIAL_OUT` 平滑环绕运镜；前后倾转短轴实时驱动旋翼倾转角。 |
| 飞机被高耸点云遮挡视线时 | `animateFloatingBadge(occlusionBadge, show)` | 顶部「遮挡透视已介入」徽章自动微滑浮现，遮挡解除后加速收起。 |

### 4.3 右侧工具栈、图层浮窗与底部上下文指令坞

| 用户操作 | 动效编排函数 / 机制 | 视觉与触觉表现 |
| :--- | :--- | :--- |
| 点击右侧「图层」按钮展开/收起浮窗 | `attachSpringTouch(SEGMENT_CHIP)` + `animatePopover` | 以右上角为锚点（`pivotX/Y`）执行缩放 + 微横移 + 透明度展开；收起或点击外部自动 `EASE_EXIT_IN` 收拢。 |
| 图层浮窗内切换「高度彩虹/强度/战术纯青」或点云元素芯片 | `attachSpringTouch(SEGMENT_CHIP)` + `animateStateFlip` | 选中芯片执行 `DETENT_POP` 档位微弹。 |
| 图层浮窗内点击「清空三维建图缓存」（二次确认） | `attachSpringTouch(CRITICAL_SAFETY)` + `animateArmedConfirm` | 首次点击进入琥珀警示脉冲待命态，二次点击触发清图与 `CRITICAL_COMMIT` 震动。 |
| 点击右侧视距「`-` / `+`」按钮 | `attachSpringTouch(ICON_CONTROL)` + 相机目标距离平滑插值 | 紧凑图标回弹 + 3D 视距连续推拉。 |
| 底部指令坞内切换「定点指点 ↔ 多点航线」子模式 | `pulseViewSubtle(commandDock)` | 指令坞容器轻微呼吸微弹过渡新操作按钮组，消除突兀跳变。 |
| 底部指令坞点击高度「`-` / `+`」或拨动遥控滚轮 | `attachSpringTouch(SEGMENT_CHIP)` + `animateValueStepNudge` + `triggerRoleHaptic(TICK_STEP)` | 高度数字按增减方向向上或向下轻跳 `3dp` 后弹簧归位，伴随清脆刻度震动。 |
| 点击「立即飞往 / 加入航线 / 取消选点」 | `attachSpringTouch` + `animateCommandDock` | 执行飞控指令或追加航点（追加航点时指令坞轻脉冲确认，起飞或取消时指令坞加速下沉收起，两侧 HUD 自动归位）。 |

### 4.4 右侧作业抽屉（4 页签）与全局设置模态框

| 用户操作 | 动效编排函数 / 机制 | 视觉与触觉表现 |
| :--- | :--- | :--- |
| 切换抽屉顶部「连接 / 话题 / 集群 / 航线」页签 | `animateTabSwitch(prevTabIndex -> tabIndex)` | 底部高亮滑块 `GLIDER_TAB` 物理弹簧横移，新面板按左右相对方向滑入并执行前 6 个子卡片微错峰浮现。 |
| 切换抽屉顶栏「标准 / 宽屏」终端宽度 | `animateDrawerWidthMorph` + `animateStateFlip` | 抽屉宽度 `ValueAnimator` 连续插值伸缩，顶部居中状态堆栈通过 `SPATIAL_DRAWER` 弹簧同步平移防碰撞。 |
| 拨动任意开关（UDP 极速通道、自动重连、震动、平滑等） | `animateSwitchThumb` | 滑块执行 `SWITCH_THUMB` 水平弹簧位移，运动瞬间产生 `1.16x × 0.92y` 流体拉伸并回弹归圆。 |
| 展开/收起「高级话题映射手风琴 / OTA 自定义源地址 / 下载进度条」 | `animateCollapsibleSection` | 展开时自上微滑渐显（可选子项错峰），收起时加速上滑淡出并安全置 `GONE`。 |
| 点击 ROS1 话题行进入实时详情 / 点击「返回列表」退出 | `animateHierarchicalPage(enterDetail = true/false)` | 进入详情从右侧 `+16dp` 推入并错峰展示监控卡片；返回列表从左侧 `-14dp` 弹回主列表，明确层级纵深。 |
| 集群编队切换拓扑/队形、倾转模式切换、航点行上移/下移/删除 | `attachSpringTouch` + `animateStateFlip` / `animateStaggeredChildren` | 按钮具备精确角色按压回弹，状态切换具备档位微弹确认。 |
| 设置面板切换左侧 6 大纵向分类 | `animateVerticalCategorySwitch(newIndex, oldIndex)` | 左侧青色指示条 `scaleY` 弹簧伸展，右侧设置页按上下方向因果滑入并错峰呈现卡片。 |
| 拖拽设置面板内任意 `SeekBar` 滑块 | `animateSliderBadgeFocus` + `triggerSliderStepTick` | 按住拖拽时右侧数值胶囊放大至 `1.08x` 抬升聚焦，滑动步进触发 `42ms` 节流刻度微震，松手平滑归位。 |
| 切换「显示主题（深空黑 / 户外高亮白 / 战术夜视绿）」 | `animateThemeCrossfade` | 根视图与模态卡片柔和透明度交叉过渡，消除高对比反色刺眼闪烁。 |
| 点击「恢复出厂默认参数」（二次确认） | `attachSpringTouch(CRITICAL_SAFETY)` + `animateArmedConfirm` | 首次点击警示弹跳待命，二次确认恢复默认并同步动效上下文。 |

---

## 5. 后期开发与迭代准入规范（Governance & PR Checklist）

为防止后续版本迭代出现散乱硬编码动画、风格割裂或帧率回退，所有后续功能开发与代码审查必须遵守以下刚性准则：

### 5.1 四条绝对红线（Must Not）

1. **严禁硬编码时长与曲线**：业务层（`MainActivity.kt`、新增面板或自定义 View）禁止直接字面量调用 `.setDuration(200)`、`OvershootInterpolator()` 或 `PathInterpolator(...)`；必须统一使用 `AnimHelper.dur(DurationToken.*)` 与 `GcsMotionSpec.EASE_*`。
2. **严禁直接 `new SpringAnimation(...)`**：任何基于 `SpringAnimation` 的位移、缩放、旋转必须通过 `GcsMotionSpec.springTo(...)` 派发，以确保档位自动缩放与 `(View, ViewProperty)` 弱引用缓存池复用。
3. **严禁无反馈裸点击与突兀 `VISIBLE/GONE` 跳变**：
   - 新增可点击按钮/卡片必须按面积与风险等级挂载 `AnimHelper.attachSpringTouch(view, PressRole.*)`；
   - 新增悬浮提示条、折叠区域、浮窗或二级子页面，必须分别接入 `animateFloatingBadge`、`animateCollapsibleSection`、`animatePopover` 或 `animateHierarchicalPage`，禁止直接硬切 `visibility`。
4. **严禁危险指令一步裸执行或静默失败**：
   - 涉及飞行中断、清库、重置等破坏性操作必须接入二次确认 `AnimHelper.animateArmedConfirm`（急停除外，急停使用 `animateEmergencyBrake` 零延迟触发）；
   - 当用户操作因前置条件不满足被拒绝时，必须调用 `AnimHelper.animateRejectShake` 搭配明确 Toast 文案提示原因。

### 5.2 新增交互控件标准接入模板

```kotlin
// 1. 常规胶囊动作按钮
val btnAction = TextView(context).apply { /* 样式初始化 */ }
AnimHelper.attachSpringTouch(btnAction, GcsMotionSpec.PressRole.PILL_BUTTON)
btnAction.setOnClickListener {
    if (hub.touchLocked) {
        AnimHelper.animateRejectShake(btnTouchLock, density)
        showToast("防误触锁已开启")
        return@setOnClickListener
    }
    AnimHelper.animateStateFlip(btnAction)
    // 执行业务逻辑...
}

// 2. 条件展开/折叠容器
AnimHelper.animateCollapsibleSection(advancedWrap, expand = isExpanded, density = d, staggerChildren = true)

// 3. 自定义 Canvas / GL 视图内部插值动画
ValueAnimator.ofFloat(startVal, targetVal).apply {
    duration = GcsMotionSpec.scaleDurationMs(GcsMotionSpec.DurationToken.SPATIAL)
    interpolator = GcsMotionSpec.EASE_SPATIAL_OUT
    addUpdateListener { /* 更新渲染状态并按需请求重绘 */ }
    start()
}
```

### 5.3 性能与生命周期守则

- **硬件加速层管理**：仅在大面积容器平移（如侧边抽屉滑入）期间临时开启 `LAYER_TYPE_HARDWARE`，且必须在动画结束回调中恢复 `LAYER_TYPE_NONE`，严禁常驻占用显存。
- **渲染按需唤醒**：自定义 OpenGL / TextureView 视口动画必须遵守 `RenderScheduling` 脏标记与 `onPause` / `onStop` 停帧约定，后台状态下不得持有无限循环 `ValueAnimator`。
- **纯 JVM 单元测试友好性**：新增 `GcsMotionSpec` Token 时，若涉及 Android Framework 类（如 `Interpolator`），须采用 `lazy` 惰性初始化，确保 `:app:testDebugUnitTest` 在无模拟器环境下毫秒级通过。

### 5.4 提交前强制验证清单

每次修改交互或动效相关代码后，提交前必须依次执行并通过：

```bash
./gradlew spotlessApply
./gradlew spotlessCheck :app:testDebugUnitTest :app:lintDebug :app:compileDebugAndroidTestKotlin :app:assembleDebug
```
