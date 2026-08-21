# ============================================================
# ProGuard Rules — Cartographer-Android
# ============================================================
# 如果启用了 minifyEnabled = true，这些规则会被应用。

# ---- Kotlin / Coroutines ----
-keepattributes *Annotation*, InnerClasses
-dontnote kotlinx.serialization.AnnotationsKt

-keepclassmembers class kotlinx.coroutines.** {
    volatile <fields>;
}
-keepclassmembers class kotlin.coroutines.SafeContinuation {
    volatile <fields>;
}
-keepclassmembers class kotlin.coroutines.intrinsics.IntrinsicsKt {
    *;
}

# ---- Jetpack Compose ----
-dontwarn androidx.compose.**
-keep class androidx.compose.** { *; }
-keepclassmembers class androidx.compose.** { *; }
-dontwarn org.jetbrains.compose.**

# ---- AndroidX ----
-keep class androidx.** { *; }
-keep interface androidx.** { *; }

# ---- Cartographer Native Interface ----
# 确保 JNI 方法名不被混淆
-keep class com.cartographer.demo.CartographerNative {
    native <methods>;
}
-keep class com.cartographer.demo.Pose3D { *; }
-keep class com.cartographer.demo.Pose2D { *; }
-keep class com.cartographer.demo.Quaternion { *; }

# ---- Google Protobuf ----
-dontwarn google.protobuf.**
-keep class * extends com.google.protobuf.GeneratedMessageLite { *; }

# ---- Native Libraries ----
# 不混淆 .so 文件中的符号，保持 JNI 签名完整
-keepnames class * implements java.io.Serializable
-keepclassmembers class * implements java.io.Serializable {
    static final long serialVersionUID;
    private static final java.io.ObjectStreamField[] serialPersistentFields;
    private void writeObject(java.io.ObjectOutputStream);
    private void readObject(java.io.ObjectInputStream);
    java.lang.Object writeReplace();
    java.lang.Object readResolve();
}

# ---- Resource shrinker (R8) ----
# 保留被反射访问的资源
-keepclassmembers class **.R$* {
    public static <fields>;
}

# ---- General ----
-keepattributes SourceFile,LineNumberTable
-renamesourcefileattribute SourceFile
