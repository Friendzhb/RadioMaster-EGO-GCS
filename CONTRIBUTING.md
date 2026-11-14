# 开发约定

- 直接提交源码。不要将工程 ZIP、APK、Gradle 缓存、IDE 文件或签名材料加入仓库。
- Android 模块位于 `app/`；修改源码后运行 `./gradlew spotlessApply`。
- 提交前运行 `./gradlew spotlessCheck :app:testDebugUnitTest :app:lintDebug :app:assembleDebug`。
- 修改渲染或通信链路时，运行 `:app:connectedDebugAndroidTest`；UDP 桥接另跑 `python3 -B -m unittest discover -s scripts -p 'test_*.py'`。
- Kotlin 与 Gradle Kotlin 脚本的格式由 `.editorconfig` 和固定版本的 Spotless/ktlint 管理。
  现有 Android 通配符导入保留；不为格式整理改动 ROS 消息、渲染算法或控制逻辑。
- 新增或修改交互控件、面板过渡、视口运镜与触觉反馈时，必须遵循 [交互动效规范](docs/MOTION_DESIGN_SYSTEM.md)，统一通过 `GcsMotionSpec` 与 `AnimHelper` 编排，严禁在业务代码中散落硬编码动画时长、曲线或裸 `SpringAnimation` 实例。
- 一个提交聚焦一个问题；在描述中写明触发条件、改动和实际执行的验证。
- CI 对分支和拉取请求进行构建检查。只有主仓库 `main` 的成功构建更新 OTA 下载入口。
- 不移动已有源码历史或强推。旧上传 ZIP 可以从迁移前的 Git 提交中恢复。
