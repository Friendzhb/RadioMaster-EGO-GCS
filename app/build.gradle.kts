import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone

plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

val buildEpochMs = System.currentTimeMillis()
val autoVersionCode = (((buildEpochMs / 1000L) - 1_700_000_000L).toInt() + 17).coerceAtLeast(317)
val buildUtcStamp = SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.US).apply {
    timeZone = TimeZone.getTimeZone("UTC")
}.format(Date(buildEpochMs)) + " UTC"

val otaStoreFile = providers.environmentVariable("EGO_SIGNING_STORE_FILE").orNull
val otaStorePassword = providers.environmentVariable("EGO_SIGNING_STORE_PASSWORD").orNull
val otaKeyAlias = providers.environmentVariable("EGO_SIGNING_KEY_ALIAS").orNull
val otaKeyPassword = providers.environmentVariable("EGO_SIGNING_KEY_PASSWORD").orNull
val hasOtaSigning = listOf(otaStoreFile, otaStorePassword, otaKeyAlias, otaKeyPassword)
    .all { !it.isNullOrBlank() }

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
        create("debugConfig") {
            storeFile = file("${rootDir}/debug.keystore")
            storePassword = "android"
            keyAlias = "androiddebugkey"
            keyPassword = "android"
        }
        if (hasOtaSigning) {
            create("otaShared") {
                storeFile = file(requireNotNull(otaStoreFile))
                storePassword = otaStorePassword
                keyAlias = otaKeyAlias
                keyPassword = otaKeyPassword
                enableV1Signing = true
                enableV2Signing = true
            }
        }
    }

    buildTypes {
        release {
            if (hasOtaSigning) {
                signingConfig = signingConfigs.getByName("otaShared")
            }
            isMinifyEnabled = true
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
        debug {
            signingConfig = signingConfigs.getByName("debugConfig")
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
}

dependencies {
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test:runner:1.6.2")
    androidTestImplementation("androidx.test.ext:junit:1.2.1")
    implementation("androidx.core:core-ktx:1.13.1")
    // Google 官方物理弹簧动画库：提供抽屉阻尼滑出、标签切换与按键回弹 60FPS 物理动效
    implementation("androidx.dynamicanimation:dynamicanimation:1.0.0")
    // OkHttp 原生异步网络栈：支持 rosbridge WebSocket 与 GitHub Releases 302 重定向流式 OTA 下载
    implementation("com.squareup.okhttp3:okhttp:4.12.0")
}
