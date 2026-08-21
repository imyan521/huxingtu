#include <jni.h>

#include <android/log.h>

#include <exception>
#include <string>
#include <vector>

#include "floor_plan.h"

namespace {

constexpr const char* kTag = "FloorPlanJNI";

std::string ToString(JNIEnv* env, jstring value) {
    if (value == nullptr) return {};
    const char* chars = env->GetStringUTFChars(value, nullptr);
    if (chars == nullptr) return {};
    std::string out(chars);
    env->ReleaseStringUTFChars(value, chars);
    return out;
}

}  // namespace

extern "C" JNIEXPORT jdoubleArray JNICALL
FloorPlanNativeGenerate(
        JNIEnv* env,
        jobject /* thiz */,
        jstring input_path,
        jstring visual_input_path,
        jstring semantic_input_path,
        jstring output_path,
        jstring work_dir,
        jdouble meters_per_pixel,
        jdoubleArray trajectory_pixels) {
    try {
        floorplan::PipelineOptions options;
        options.thresh = 200;
        options.min_branch_length = -1;
        options.restore = false;
        options.auto_branch = true;
        options.meters_per_pixel = meters_per_pixel;
        options.visual_input_path = ToString(env, visual_input_path);
        options.semantic_input_path = ToString(env, semantic_input_path);
        if (trajectory_pixels != nullptr) {
            const jsize value_count = env->GetArrayLength(trajectory_pixels);
            if (value_count >= 2 && value_count % 2 == 0) {
                std::vector<jdouble> values(static_cast<size_t>(value_count));
                env->GetDoubleArrayRegion(
                        trajectory_pixels, 0, value_count, values.data());
                options.trajectory_points_px.reserve(values.size() / 2);
                for (size_t index = 0; index + 1 < values.size(); index += 2) {
                    options.trajectory_points_px.emplace_back(
                            static_cast<float>(values[index]),
                            static_cast<float>(values[index + 1]));
                }
            }
        }

        const std::string input = ToString(env, input_path);
        const std::string output = ToString(env, output_path);
        const std::string work = ToString(env, work_dir);
        const std::string debug = work.empty() ? std::string() : (work + "/debug");

        const auto result = floorplan::RunPipeline(input, output, work, debug, options);
        __android_log_print(
                ANDROID_LOG_INFO,
                kTag,
                "floorplan generated closed=%d vertices=%d size=%.1fx%.1f support=%.3f output=%s",
                result.outline_closed ? 1 : 0,
                result.outline_vertex_count,
                result.outline_width_px,
                result.outline_height_px,
                result.outline_support_ratio,
                result.output_path.c_str());
        if (!result.outline_closed || result.outline_width_px <= 0.0 ||
            result.outline_height_px <= 0.0 || result.dimension_long_size_px <= 0.0 ||
            result.dimension_short_size_px <= 0.0) {
            __android_log_print(ANDROID_LOG_ERROR, kTag, "fitted wall bounds validation failed");
            return nullptr;
        }
        std::vector<jdouble> values = {
                result.outline_closed ? 1.0 : 0.0,
                result.outline_width_px,
                result.outline_height_px,
                result.outline_left_px,
                result.outline_top_px,
                result.outline_right_px,
                result.outline_bottom_px,
                result.outline_rotation_degrees,
                result.outline_support_ratio,
                static_cast<double>(result.outline_vertex_count),
                static_cast<double>(result.outline_close_size),
                result.dimension_center_x_px,
                result.dimension_center_y_px,
                result.dimension_long_axis_x,
                result.dimension_long_axis_y,
                result.dimension_short_axis_x,
                result.dimension_short_axis_y,
                result.dimension_long_size_px,
                result.dimension_short_size_px,
        };
        values.reserve(values.size() + result.outline_polygon_px.size() * 2 + 2);
        for (const auto& point : result.outline_polygon_px) {
            values.push_back(point.x);
            values.push_back(point.y);
        }
        values.push_back(result.footprint_area_px2);
        values.push_back(result.footprint_perimeter_px);
        jdoubleArray output_values = env->NewDoubleArray(
                static_cast<jsize>(values.size()));
        if (output_values == nullptr) return nullptr;
        env->SetDoubleArrayRegion(
                output_values,
                0,
                static_cast<jsize>(values.size()),
                values.data());
        return output_values;
    } catch (const std::exception& e) {
        __android_log_print(ANDROID_LOG_ERROR, kTag, "generate failed: %s", e.what());
        return nullptr;
    } catch (...) {
        __android_log_print(ANDROID_LOG_ERROR, kTag, "generate failed: unknown error");
        return nullptr;
    }
}

extern "C" JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void*) {
    JNIEnv* env = nullptr;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) return JNI_ERR;
    jclass type = env->FindClass("com/cartographer/demo/FloorPlanNative");
    if (type == nullptr) return JNI_ERR;
    const JNINativeMethod method = {
            const_cast<char*>("nativeGenerateFloorPlan"),
            const_cast<char*>("(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;D[D)[D"),
            reinterpret_cast<void*>(FloorPlanNativeGenerate)};
    const jint result = env->RegisterNatives(type, &method, 1);
    env->DeleteLocalRef(type);
    return result == JNI_OK ? JNI_VERSION_1_6 : JNI_ERR;
}
