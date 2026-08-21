# Cartographer Device SDK 1.0.0

Closed-source Android SDK for the supported CH340 lidar. The SDK owns USB permission/serial I/O,
phone IMU collection, lidar frame parsing, SLAM, map persistence, relocalization primitives and
floor-plan generation. It does not provide an Activity or UI.

## Compatibility

- Android API 26 or newer
- `arm64-v8a` only
- USB host device with accelerometer and gyroscope
- Lidar USB VID/PID `1A86:7523`, serial settings `230400 8N1`

## Gradle

For a local delivery, copy `cartographer-sdk-release.aar` into the customer's `app/libs` directory:

```kotlin
dependencies {
    implementation(files("libs/cartographer-sdk-release.aar"))
    implementation("com.github.mik3y:usb-serial-for-android:3.4.6")
    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.11.0")
    implementation("org.jetbrains.kotlin:kotlin-stdlib:1.9.22")
}
```

When published to the private Maven repository, use:

```kotlin
implementation("com.cartographer:device-sdk:1.0.0")
```

## Lifecycle

To launch the complete reference experience (UI, mapping, continuation, measurement, magnetic
north alignment, export and floor-plan generation):

```kotlin
startActivity(CartographerSdk.createFullExperienceIntent(this))
```

For a custom UI, use the lower-level lifecycle below.

1. Create one SDK instance with `CartographerSdk.initialize(applicationContext, ..., listener)`.
2. Call `requestUsbPermission(activity)` from a visible Activity.
3. After permission succeeds, call `connect()`.
4. Call `startMapping()` and consume listener snapshots/scans on the main thread.
5. Call `stopMapping()`, then save/load maps or generate a floor plan as needed.
6. Call `close()` when the owning application scope is destroyed.

The SDK dynamically registers USB receivers and does not install an Activity or Application class.
All listener methods and file-operation callbacks run on the Android main thread.
Java callers can extend `CartographerListenerAdapter` instead of implementing every callback.

If mapping must continue while the customer app is backgrounded, the customer app must own a
foreground service and call the SDK from that service. The SDK deliberately does not inject a
service or request notification permissions. Customers already shipping OpenCV or `libc++_shared`
must align those native versions to avoid duplicate-library packaging conflicts.

## Release

```shell
./gradlew :cartographer-sdk:verifyReleaseAar
```

The verification task rejects source, native headers/static archives, CMake files and plaintext Lua
from the AAR, checks required ARM64 libraries, and writes a `.sha256` file next to the release AAR.

The SDK is binary-obfuscated, not cryptographically impossible to reverse engineer. Contractual
confidentiality and no-reverse-engineering terms remain necessary.
