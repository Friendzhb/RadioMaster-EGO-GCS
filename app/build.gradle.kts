import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone
import java.util.concurrent.TimeUnit
import org.gradle.api.tasks.compile.JavaCompile
import org.gradle.api.tasks.testing.Test
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

val otaStoreFile = providers.environmentVariable("EGO_SIGNING_STORE_FILE").orNull
val otaStorePassword = providers.environmentVariable("EGO_SIGNING_STORE_PASSWORD").orNull
val otaKeyAlias = providers.environmentVariable("EGO_SIGNING_KEY_ALIAS").orNull
val otaKeyPassword = providers.environmentVariable("EGO_SIGNING_KEY_PASSWORD").orNull
val resolvedOtaKeystore = otaStoreFile
    ?.takeIf { it.isNotBlank() }
    ?.let { path ->
        val direct = File(path)
        when {
            direct.isAbsolute -> direct
            file(path).isFile -> file(path)
            else -> rootProject.file(path)
        }
    }
val hasOtaSigning = resolvedOtaKeystore?.isFile == true &&
    !otaStorePassword.isNullOrBlank() &&
    !otaKeyAlias.isNullOrBlank() &&
    !otaKeyPassword.isNullOrBlank()
val aiStudioDebugKeystore = rootProject.file("debug.keystore")
val hasAiStudioDebugKeystore = aiStudioDebugKeystore.isFile

// 对齐构建时间戳粒度（OTA 正式发版按分钟、常规 CI/开发构建按小时分桶），防止每次毫秒变化击穿 Gradle buildCache
val explicitEpochSec = providers.environmentVariable("SOURCE_DATE_EPOCH").orNull?.toLongOrNull()
val rawEpochMs = explicitEpochSec?.times(1000L) ?: System.currentTimeMillis()
val cacheBucketMs = if (hasOtaSigning) 60_000L else 3_600_000L
val buildEpochMs = (rawEpochMs / cacheBucketMs) * cacheBucketMs
val autoVersionCode = (((buildEpochMs / 1000L) - 1_700_000_000L).toInt() + 17).coerceAtLeast(317)
val buildUtcStamp = SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.US).apply {
    timeZone = TimeZone.getTimeZone("UTC")
}.format(Date(buildEpochMs)) + " UTC"

// 依赖解析与远端元数据本地缓存优化：避免 GitHub Actions 每次构建重复轮询校验远端 Maven 元数据导致超时
configurations.configureEach {
    resolutionStrategy {
        cacheDynamicVersionsFor(24, TimeUnit.HOURS)
        cacheChangingModulesFor(24, TimeUnit.HOURS)
    }
}

android {
    namespace = "com.radiomaster.egogcs"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.radiomaster.egogcs"
        minSdk = 26 // 原生兼容 RadioMaster AX12 (Android 9.0 / API 28)
        targetSdk = 34
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        versionCode = autoVersionCode
        versionName = "3.12.0"
        resValue("string", "ego_build_stamp", buildUtcStamp)
        resValue("string", "ego_build_epoch_ms", buildEpochMs.toString())
    }

    signingConfigs {
        if (hasAiStudioDebugKeystore) {
            create("debugConfig") {
                storeFile = aiStudioDebugKeystore
                storePassword = "android"
                keyAlias = "androiddebugkey"
                keyPassword = "android"
            }
        }
        if (hasOtaSigning) {
            create("otaShared") {
                storeFile = requireNotNull(resolvedOtaKeystore)
                storePassword = otaStorePassword
                keyAlias = otaKeyAlias
                keyPassword = otaKeyPassword
                enableV1Signing = true
                enableV2Signing = true
            }
        }
    }

    val activeSigningConfig = when {
        hasOtaSigning -> signingConfigs.getByName("otaShared")
        hasAiStudioDebugKeystore -> signingConfigs.getByName("debugConfig")
        else -> signingConfigs.getByName("debug")
    }

    buildTypes {
        release {
            signingConfig = activeSigningConfig
            isMinifyEnabled = true
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
        debug {
            // Priority: CI OTA certificate -> AI Studio root debug.keystore -> standard Android debug key.
            signingConfig = activeSigningConfig
            isMinifyEnabled = false
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }

    lint {
        checkReleaseBuilds = false
        checkDependencies = false
        abortOnError = true
    }

    androidResources {
        // openFd maps the original model directly from the APK without a second heap-sized copy.
        noCompress += "glb"
    }

    packaging {
        resources {
            excludes += "/META-INF/{AL2.0,LGPL2.1}"
            excludes += "/META-INF/LICENSE*"
        }
    }
}

// 显式开启 Kotlin / Java 增量编译与 Gradle 任务级构建缓存 (buildCache)
tasks.withType<KotlinCompile>().configureEach {
    incremental = true
    outputs.cacheIf { true }
}

tasks.withType<JavaCompile>().configureEach {
    options.isIncremental = true
    options.isFork = true
    outputs.cacheIf { true }
}

tasks.withType<Test>().configureEach {
    maxParallelForks = (Runtime.getRuntime().availableProcessors() / 2).coerceAtLeast(1)
    outputs.cacheIf { true }
}

dependencies {
    // 统一锁定 Kotlin 标准库族版本至 1.9.24，消除 OkHttp/Okio 传递引入的 kotlin-stdlib-jdk7/jdk8 1.9.10 版本分裂
    implementation(platform("org.jetbrains.kotlin:kotlin-bom:1.9.24"))
    androidTestImplementation(platform("org.jetbrains.kotlin:kotlin-bom:1.9.24"))

    // 消除主类路径对 debugAndroidTestRuntimeClasspath 的 {strictly} 反向降级冲突
    constraints {
        implementation("androidx.annotation:annotation:1.8.0")
        implementation("androidx.tracing:tracing:1.2.0")
        implementation("org.jetbrains.kotlinx:kotlinx-coroutines-core:1.7.3")
        implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.7.3")
    }

    testImplementation("junit:junit:4.13.2")
    testImplementation("org.json:json:20240303")
    androidTestImplementation("androidx.test:runner:1.6.2")
    androidTestImplementation("androidx.test.ext:junit:1.2.1")
    implementation("androidx.core:core-ktx:1.13.1")
    // Google 官方物理弹簧动画库：提供抽屉阻尼滑出、标签切换与按键回弹 60FPS 物理动效
    implementation("androidx.dynamicanimation:dynamicanimation:1.0.0")
    // OkHttp 原生异步网络栈：支持 rosbridge WebSocket 与 GitHub Releases 302 重定向流式 OTA 下载
    implementation("com.squareup.okhttp3:okhttp:4.12.0")
}
