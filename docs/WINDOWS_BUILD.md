# Windows APK build

## Requirements

- Android Studio with JDK 17
- Android SDK 34
- Android NDK `27.0.12077973`

Clone the repository with submodules:

```bat
git clone --recurse-submodules git@github.com:imyan521/huxingtu.git
cd huxingtu
```

Create `local.properties` with the SDK path for the Windows account:

```properties
sdk.dir=C\:\\Users\\YOUR_NAME\\AppData\\Local\\Android\\Sdk
```

Build the APK using the checked-in ARM64 JNI libraries:

```bat
gradlew.bat :app:assembleDebug
```

The APK is written to `app\build\outputs\apk\debug\app-debug.apk`.

The default build does not rebuild Cartographer or the floor-plan C++ code. To
rebuild native code, use Linux/WSL with the Android dependencies installed under
`third_party/install`, then run:

```bash
./gradlew :app:assembleDebug -PbuildNativeFromSource=true
```
