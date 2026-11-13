# RadioMaster EGO GCS

面向 RadioMaster AX12 的原生 Kotlin Android 地面站，包含 OpenGL ES 点云显示、
ROS1/TCPROS 与 rosbridge 通信、MAVLink 遥测、EGO 航线显示及本地仿真。

仓库直接维护 Android 源码。应用功能说明的原始版本见
[原工程说明](docs/ORIGINAL_PROJECT_README.md)，此次源码迁移及构建修复见
[构建修复记录](docs/BUILD_FIX_20261113.md)。

## 构建环境

- JDK 17
- Android SDK Platform 34、Build Tools 34.0.0
- Gradle 8.7，由仓库内的 Wrapper 下载并校验
- Android Gradle Plugin 8.5.2、Kotlin 1.9.24

设置 `ANDROID_HOME` 或在未跟踪的 `local.properties` 中填写 `sdk.dir`，然后执行：

```bash
./gradlew spotlessApply
./gradlew spotlessCheck :app:lintDebug :app:assembleDebug
```

Windows 使用 `gradlew.bat`。APK 输出到 `app/build/outputs/apk/debug/app-debug.apk`。
没有配置 OTA 签名的本地构建使用标准 Android debug 签名。

## CI 与 APK 下载

GitHub Actions 对提交和拉取请求执行格式检查、Android Lint 和 APK 编译。
在 Actions 成功运行页面下载 `RadioMaster-EGO-Pilot-Native-APK` 产物。
主仓库 `main` 成功后更新 [OTA 下载入口](https://github.com/Friendzhb/RadioMaster-EGO-GCS/releases/tag/latest-apk)，
保持已有 `EGO-Pilot-AX12.apk` 地址和开发证书兼容。

源码提交、工作流运行编号和 APK 哈希见发布产物 `build-info.json`。
编译与静态检查不能替代 AX12 真机上的通信、渲染及飞行联调。

## 目录

```text
app/src/main/java/       Kotlin 应用代码
app/src/main/res/        Android 资源
app/build.gradle.kts    Android 模块与环境变量签名配置
gradle/wrapper/         固定版本的 Gradle Wrapper
docs/                   迁移、构建与原始功能说明
scripts/                构建产物元数据工具
.github/workflows/      构建检查及主分支 APK 发布
```

## 后续提交

直接修改并提交 `app/`、资源和构建脚本，不再上传多个版本的源码 ZIP。
格式及提交约定见 [CONTRIBUTING.md](CONTRIBUTING.md)。
签名材料通过 CI Secrets 或本地 `EGO_SIGNING_*` 环境变量提供，不提交到仓库。
