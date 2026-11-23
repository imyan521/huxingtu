# Cartographer Device SDK 1.3.0

Closed-source Android SDK for the supported CH340 lidar. The SDK owns USB permission/serial I/O,
phone IMU collection, lidar frame parsing, SLAM, map persistence, relocalization primitives and
floor-plan generation. A packaged full-experience Activity is also available.

## Compatibility

- Android API 26 or newer
- `arm64-v8a` only
- USB host device; accelerometer and gyroscope are required for the default IMU mode
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
implementation("com.cartographer:device-sdk:1.3.0")
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
5. During mapping, watch `SdkSnapshot.magneticCalibration`. Walk through at least one clear corner.
6. Call `stopMapping()`. It returns `MAGNETIC_CALIBRATION_INCOMPLETE` while more valid motion is needed.
7. After `FINISHED`, read `MagneticNorthAlignment`, then save/load maps or generate a floor plan.
8. Call `close()` when the owning application scope is destroyed.

The SDK dynamically registers USB receivers and declares FullExperienceActivity; it does not install an Application class.
All listener methods and file-operation callbacks run on the Android main thread.
Java callers can extend `CartographerListenerAdapter` instead of implementing every callback.

If mapping must continue while the customer app is backgrounded, the customer app must own a
foreground service and call the SDK from that service. The SDK deliberately does not inject a
service or request notification permissions. Customers already shipping OpenCV or `libc++_shared`
must align those native versions to avoid duplicate-library packaging conflicts.

## Heat-map coverage

Record Wi-Fi RSSI together with the current `SdkSnapshot.pose`, then pass those measurements when
generating the floor plan:

```kotlin
sdk.generateFloorPlan(output, workDir, heatMapSamples) { result, floorPlan ->
    val coverage = floorPlan?.heatMapCoverage
    Log.i("HeatMap", "coverage=${coverage?.coveragePercent}")
}
```

`HeatMapSample` uses SLAM world coordinates in metres. Coverage is the unique usable indoor area
within 1.5 metres of the valid sampled route divided by the usable area inside the fitted outline.
Occupied wall cells block propagation, samples separated by more than three seconds or three metres
are not joined, and repeated passes do not increase the covered area.

## Magnetic north alignment

For a new map, the SDK collects magnetometer, gravity, and SLAM poses while the operator walks. It
fits the fixed magnetic bias from the lidar magnet, then refits against the final optimized
trajectory when mapping ends. No separate compass-button step is required.

```kotlin
override fun onSnapshot(snapshot: SdkSnapshot) {
    val calibration = snapshot.magneticCalibration
    statusText.text = "${calibration.reason}, coverage=${calibration.headingCoverageDegrees}°"
}

override fun onMagneticNorthAlignment(alignment: MagneticNorthAlignment) {
    // Clockwise image rotation matching the reference UI: left=south, right=north.
    val floorPlanRotation = alignment.floorPlanRotationDegrees
}
```

`stopMapping()` keeps the trajectory running when calibration is not ready and returns
`SdkError.MAGNETIC_CALIBRATION_INCOMPLETE`. A normal corner with roughly 90 degrees of heading
coverage is sufficient. Moderate interference is reported through `isHighInterference` but does
not prevent completion. Once a usable fit exists, short later disturbances retain that fit.

`saveMap()` writes a sibling `*.metadata.json` file containing the magnetic-north direction.
`loadMap()` restores it, and continuation mapping keeps the existing map direction. Use
`getMagneticNorthAlignment()` to read it at any time after `FINISHED` or a successful load.

Timestamped final optimized nodes are available through `getTrajectoryNodePoseSamples()`; each
`TrajectoryNodePoseSample` contains `timestampNs`, `x`, `y`, and `theta`.

## Release

```shell
./gradlew :cartographer-sdk:verifyReleaseAar
```

The verification task rejects source, native headers/static archives, CMake files and plaintext Lua
from the AAR, checks required ARM64 libraries, and writes a `.sha256` file next to the release AAR.

The SDK is binary-obfuscated, not cryptographically impossible to reverse engineer. Contractual
confidentiality and no-reverse-engineering terms remain necessary.

## Optional lidar-only mapping (1.3.0)

Existing `SdkConfig` constructors and `initialize(context, config, listener)` calls remain valid and use IMU fusion.
To use lidar-only mapping, select the mode when creating a new SDK instance:

```kotlin
val sdk = CartographerSdk.initialize(
    applicationContext, SdkConfig(), MappingInputMode.LIDAR_ONLY, listener
)
// Wait for MappingState.READY, then use the normal permission/connect/startMapping flow.
```

The mode stays fixed for that instance. Close it before creating an instance with another mode.
The full UI also has a switch (off on first launch, then remembered):

```kotlin
startActivity(CartographerSdk.createFullExperienceIntent(this, MappingInputMode.LIDAR_ONLY))
```

The explicit Intent mode overrides the remembered UI choice for that launch.
Lidar-only profiles disable IMU input, track `base_link`, use a 15-degree angular search
window and a Ceres rotation weight of 10. They support new maps and frozen-map supplement
collection. `loadMap` must be called before `startMapping` for supplement collection.
The original SDK IMU profile assets and phone IMU configuration are preserved.

In lidar-only mode, a missing magnetometer or incomplete magnetic calibration does not block
start/stop/save. North-alignment callbacks and metadata are produced only when calibration is
ready (or restored from a loaded map). Otherwise maps retain their SLAM orientation.
Default IMU mode retains its existing magnetic-calibration requirement.

Supported hardware remains the existing `1A86:7523` CH340 lidar with `230400 8N1`.
The PDA KLD42 USB driver, A-class protocol and board-IMU startup are separate hardware
adaptations and are not part of this port. Phone scan timing and outlier filtering are retained.
Switch modes only after finishing or cancelling mapping; load the original map again to supplement it.
