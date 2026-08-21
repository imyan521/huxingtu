import java.security.MessageDigest
import java.util.zip.ZipFile

plugins {
    id("com.android.library")
    id("org.jetbrains.kotlin.android")
    id("maven-publish")
}

group = "com.cartographer"
version = "1.0.0"

android {
    namespace = "com.cartographer.demo"
    compileSdk = 34
    ndkVersion = "27.0.12077973"

    defaultConfig {
        minSdk = 26
        consumerProguardFiles("consumer-rules.pro")
        ndk { abiFilters += "arm64-v8a" }
        externalNativeBuild {
            cmake {
                cppFlags += listOf("-std=c++17", "-frtti", "-fexceptions", "-fvisibility=hidden", "-ffunction-sections", "-fdata-sections")
                arguments += listOf("-DANDROID_STL=c++_shared", "-DANDROID_PLATFORM=android-26")
            }
        }
    }

    buildTypes {
        debug { isMinifyEnabled = false }
        release {
            isMinifyEnabled = true
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
        }
    }

    externalNativeBuild {
        cmake {
            path = file("../cartographer-android/CMakeLists.txt")
            version = "3.22.1"
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions { jvmTarget = "17" }

    sourceSets {
        getByName("main") {
            java.srcDir(layout.buildDirectory.dir("generated/internalSources"))
            assets.srcDir(layout.buildDirectory.dir("generated/encryptedAssets"))
            res.srcDir("../app/src/main/res")
            jniLibs.srcDir("../MyApplication/app/src/main/cpp/opencv/libs")
        }
    }

    packaging {
        jniLibs {
            excludes += listOf(
                "lib/armeabi-v7a/**",
                "lib/x86/**",
                "lib/x86_64/**"
            )
        }
    }

    publishing { singleVariant("release") }
}

val assetKey = byteArrayOf(0x43, 0x61, 0x72, 0x74, 0x6f, 0x53, 0x44, 0x4b)
val encryptSdkAssets by tasks.registering {
    val inputDir = file("../app/src/main/assets")
    val outputDir = layout.buildDirectory.dir("generated/encryptedAssets/carto_cfg")
    inputs.dir(inputDir)
    outputs.dir(outputDir)
    doLast {
        val destination = outputDir.get().asFile
        destination.deleteRecursively()
        destination.mkdirs()
        inputDir.listFiles().orEmpty().filter { it.isFile && it.extension == "lua" }.forEach { source ->
            val relative = source.name
            val bytes = source.readBytes()
            for (index in bytes.indices) bytes[index] = (bytes[index].toInt() xor assetKey[index % assetKey.size].toInt()).toByte()
            destination.resolve("$relative.bin").writeBytes(bytes)
        }
    }
}

val prepareInternalSources by tasks.registering(Copy::class) {
    from("../app/src/main/java") {
        include("com/cartographer/demo/*.kt")
        exclude("com/cartographer/demo/CartographerApp.kt")
    }
    into(layout.buildDirectory.dir("generated/internalSources"))
}

tasks.named("preBuild").configure { dependsOn(encryptSdkAssets, prepareInternalSources) }

dependencies {
    api("com.github.mik3y:usb-serial-for-android:3.4.6")
    implementation("androidx.core:core-ktx:1.12.0")
    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.11.0")
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.8.0")
    testImplementation("junit:junit:4.13.2")
}

publishing {
    publications {
        register<MavenPublication>("release") {
            afterEvaluate { from(components["release"]) }
            groupId = "com.cartographer"
            artifactId = "device-sdk"
            version = project.version.toString()
        }
    }
}

val verifyReleaseAar by tasks.registering {
    dependsOn("bundleReleaseAar")
    doLast {
        val aar = layout.buildDirectory.file("outputs/aar/cartographer-sdk-release.aar").get().asFile
        require(aar.isFile) { "Release AAR was not produced: $aar" }
        val forbidden = listOf(".kt", ".java", ".cpp", ".cc", ".c", ".h", ".hpp", ".a", "CMakeLists.txt", ".lua")
        ZipFile(aar).use { zip ->
            val bad = zip.entries().asSequence().map { it.name }.filter { name -> forbidden.any { name.endsWith(it) } }.toList()
            require(bad.isEmpty()) { "Forbidden files in release AAR: $bad" }
            require(zip.getEntry("jni/arm64-v8a/libcartographer-jni.so") != null) { "Missing arm64 Cartographer native library" }
            require(zip.getEntry("jni/arm64-v8a/libfloorplan-jni.so") != null) { "Missing arm64 floor-plan native library" }
            val unexpectedAbis = zip.entries().asSequence().map { it.name }
                .filter { it.startsWith("jni/") && !it.startsWith("jni/arm64-v8a/") && it.endsWith(".so") }
                .toList()
            require(unexpectedAbis.isEmpty()) { "Unexpected native ABIs in release AAR: $unexpectedAbis" }
        }
        val digest = MessageDigest.getInstance("SHA-256").digest(aar.readBytes()).joinToString("") { "%02x".format(it) }
        aar.resolveSibling("${aar.name}.sha256").writeText("$digest  ${aar.name}\n")
        println("Verified release AAR: ${aar.absolutePath}\nSHA-256: $digest")
    }
}
