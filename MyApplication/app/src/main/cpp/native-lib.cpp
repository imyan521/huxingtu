#include <jni.h>
#include <android/log.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "map_clean.h"
#include "linefitter.h"

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "TEST", __VA_ARGS__)

extern "C"
JNIEXPORT void JNICALL
Java_com_example_myapplication_MainActivity_runPipeline(JNIEnv *env, jobject thiz, jstring input, jstring output) {

    LOGI("✅ 进入 C++ 算法处理");

    const char *in_path = env->GetStringUTFChars(input, 0);
    const char *out_path = env->GetStringUTFChars(output, 0);

    // 读取图片
    cv::Mat img = cv::imread(in_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        LOGI("❌ 图片读取失败");
        return;
    }
    LOGI("✅ 图片读取成功：%d x %d", img.cols, img.rows);

    // 你的算法
    MapCleaner cleaner;
    LineFitter fitter;

    cv::Mat clean = cleaner.process(img, 180, 3, false);
    cv::imwrite(std::string(out_path) + "_cleaned.png", clean);
    LOGI("✅ 预处理图已保存");

    auto lines = fitter.fit(clean);
    cv::Mat result = cv::Mat::zeros(img.size(), CV_8UC1);
    for (const auto& ln : lines) {
        cv::line(result, ln.p1, ln.p2, cv::Scalar(255), 2);
    }
    cv::imwrite(out_path, result);
    LOGI("✅ 最终结果图已保存");

    env->ReleaseStringUTFChars(input, in_path);
    env->ReleaseStringUTFChars(output, out_path);
}
