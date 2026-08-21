plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.cartographer.demo"
    compileSdk = 34

    ndkVersion = "27.0.12077973"

    defaultConfig {
        applicationId = "com.cartographer.demo"
        minSdk = 26
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"

        ndk {
            // 与 cartographer/build-android 的 libcartographer.a 架构一致；仅有 arm64 产物时不要编 armeabi-v7a
            abiFilters += listOf("arm64-v8a")
        }

        externalNativeBuild {
            cmake {
                cppFlags += listOf("-std=c++17", "-frtti", "-fexceptions")
                arguments += listOf(
                    "-DANDROID_STL=c++_shared",
                    "-DANDROID_PLATFORM=android-26"
                )
            }
        }
    }

    buildTypes {
        debug {
            isMinifyEnabled = false
            isDebuggable = true
        }
        release {
            isMinifyEnabled = true
            isShrinkResources = true
            isDebuggable = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }

    externalNativeBuild {
        cmake {
            // 使用相对路径，兼容 Windows / macOS / Linux
            path = file("../cartographer-android/CMakeLists.txt")
            version = "3.22.1"
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }

    buildFeatures {
        viewBinding = true
        compose = true
        buildConfig = true
    }

    composeOptions {
        kotlinCompilerExtensionVersion = "1.5.10"
    }

    sourceSets {
        getByName("main") {
            java.srcDirs("src/main/java")
            jniLibs.srcDirs("../MyApplication/app/src/main/cpp/opencv/libs")
        }
    }

    lint {
        // 致命错误：必须在发布前修复
        fatal.add("HardcodedDebugMode")
        fatal.add("AllowBackup")
        fatal.add("SetJavaScriptEnabled")
        fatal.add("ExportedContentProvider")
        fatal.add("ExportedReceiver")
        fatal.add("ExportedService")
        fatal.add("AllowBackup")
        fatal.add("UnsafeNativeCodeLocation")

        // 禁用项（已知误报或暂不适用）
        disable.add("InvalidPackage")
        disable.add("KotlinInternalInPublicApi")
        disable.add("ParcelCreator")
        disable.add("ObsoleteSdkInt")

        // 输出配置
        textReport = true
        htmlReport = true
        xmlReport = true
        warningsAsErrors = false
    }
}

dependencies {
    // AndroidX Core
    implementation("androidx.core:core-ktx:1.12.0")
    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.11.0")

    // Jetpack Compose BOM — 统一管理所有 Compose 库版本
    val composeBom = platform("androidx.compose:compose-bom:2024.02.00")
    implementation(composeBom)
    implementation("androidx.compose.ui:ui")
    implementation("androidx.compose.ui:ui-graphics")
    implementation("androidx.compose.ui:ui-tooling-preview")
    implementation("androidx.compose.material3:material3")
    implementation("androidx.compose.material:material-icons-extended")

    // Lifecycle + ViewModel
    implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.7.0")
    implementation("androidx.lifecycle:lifecycle-viewmodel-compose:2.7.0")
    implementation("androidx.lifecycle:lifecycle-runtime-compose:2.7.0")

    // Activity Compose
    implementation("androidx.activity:activity-compose:1.8.2")

    // Navigation Compose
    implementation("androidx.navigation:navigation-compose:2.7.7")

    // Coroutines
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.8.0")

    // ========================================
    // 🚀 新增：USB 串口通信库 (用来读取雷达数据)
    // ========================================
    implementation("com.github.mik3y:usb-serial-for-android:3.4.6")

    // ========================================
    // 测试依赖
    // ========================================
    testImplementation("junit:junit:4.13.2")
    testImplementation("org.jetbrains.kotlinx:kotlinx-coroutines-test:1.8.0")
    testImplementation("io.mockk:mockk:1.13.10")

    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")

    // Compose 测试
    androidTestImplementation(composeBom)
    androidTestImplementation("androidx.compose.ui:ui-test-junit4")
    debugImplementation("androidx.compose.ui:ui-tooling")
    debugImplementation("androidx.compose.ui:ui-test-manifest")
    implementation(files("libs/cartographer-sdk-release.aar"))
    implementation("com.github.mik3y:usb-serial-for-android:3.4.6")
    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.11.0")
    implementation("org.jetbrains.kotlin:kotlin-stdlib:1.9.22")




}
