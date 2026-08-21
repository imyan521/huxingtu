/*
 * Cartographer Android - JNI接口头文件
 * 定义Java/Kotlin与C++之间的接口
 */

#ifndef CARTOGRAPHER_ANDROID_JNI_H
#define CARTOGRAPHER_ANDROID_JNI_H

#include <jni.h>
#include <android/log.h>

// Android日志宏
#define LOG_TAG "CartographerJNI"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jlong JNICALL
Java_com_cartographer_demo_CartographerNative_nativeInit(
        JNIEnv* env, jobject thiz, jstring config_directory, jstring config_basename);

JNIEXPORT jint JNICALL
Java_com_cartographer_demo_CartographerNative_nativeStartTrajectory(
        JNIEnv* env, jobject thiz, jlong native_handle);

JNIEXPORT void JNICALL
Java_com_cartographer_demo_CartographerNative_nativeAddRangefinderData(
        JNIEnv* env, jobject thiz, jlong native_handle, jint trajectory_id,
        jlong timestamp_ns, jfloat scan_duration_seconds,
        jfloatArray ranges, jfloatArray angles);

JNIEXPORT void JNICALL
Java_com_cartographer_demo_CartographerNative_nativeAddImuData(
        JNIEnv* env, jobject thiz, jlong native_handle, jint trajectory_id,
        jlong timestamp_ns,
        jfloat linear_acceleration_x, jfloat linear_acceleration_y, jfloat linear_acceleration_z,
        jfloat angular_velocity_x, jfloat angular_velocity_y, jfloat angular_velocity_z);

JNIEXPORT jdoubleArray JNICALL
Java_com_cartographer_demo_CartographerNative_nativeGetPose(
        JNIEnv* env, jobject thiz, jlong native_handle, jint trajectory_id);

JNIEXPORT jlongArray JNICALL
Java_com_cartographer_demo_CartographerNative_nativeGetStatus(
        JNIEnv* env, jobject thiz, jlong native_handle);

JNIEXPORT jlongArray JNICALL
Java_com_cartographer_demo_CartographerNative_nativeGetRelocalizationStatus(
        JNIEnv* env, jobject thiz, jlong native_handle);

JNIEXPORT jintArray JNICALL
Java_com_cartographer_demo_CartographerNative_nativeGetLatestSubmapTexture(
        JNIEnv* env, jobject thiz, jlong native_handle);

JNIEXPORT jobjectArray JNICALL
Java_com_cartographer_demo_CartographerNative_nativeGetSubmapTextures(
        JNIEnv* env, jobject thiz, jlong native_handle);

JNIEXPORT jdoubleArray JNICALL
Java_com_cartographer_demo_CartographerNative_nativeGetTrajectoryNodePoses(
        JNIEnv* env, jobject thiz, jlong native_handle);

JNIEXPORT jboolean JNICALL
Java_com_cartographer_demo_CartographerNative_nativeSerializeState(
        JNIEnv* env, jobject thiz, jlong native_handle, jstring filename,
        jboolean include_unfinished_submaps);

JNIEXPORT jboolean JNICALL
Java_com_cartographer_demo_CartographerNative_nativeLoadMap(
        JNIEnv* env, jobject thiz, jlong native_handle, jstring filename,
        jboolean load_frozen_state);

JNIEXPORT void JNICALL
Java_com_cartographer_demo_CartographerNative_nativeFinishTrajectory(
        JNIEnv* env, jobject thiz, jlong native_handle, jint trajectory_id);

JNIEXPORT void JNICALL
Java_com_cartographer_demo_CartographerNative_nativeDestroy(
        JNIEnv* env, jobject thiz, jlong native_handle);

#ifdef __cplusplus
}
#endif

#endif // CARTOGRAPHER_ANDROID_JNI_H
