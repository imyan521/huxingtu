#include "opencv/include/opencv2/opencv.hpp"
#include "map_clean.h"
#include "linefitter.h"
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "用法: " << argv[0] << " 输入图 输出纯直线图.png" << std::endl;
        return -1;
    }

    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "错误：无法读取图像 " << argv[1] << std::endl;
        return -1;
    }

    MapCleaner cleaner;
    LineFitter fitter;

    // 预处理：二值化 -> 骨架化 -> 剪枝 -> 端点连接
    cv::Mat clean = cleaner.process(img, 200, 5, false);   // 输出骨架图

    // 保存预处理后的骨架图（方便调试）
    std::string clean_path = std::string(argv[2]) + "_cleaned.png";
    if (!cv::imwrite(clean_path, clean)) {
        std::cerr << "警告：无法保存预处理骨架图 " << clean_path << std::endl;
    } else {
        std::cout << "已保存预处理（骨架）图: " << clean_path << std::endl;
    }

    // 直线拟合
    auto lines = fitter.fit(clean);

    // 绘制结果（纯黑背景，白色直线，线宽2）
    cv::Mat result = cv::Mat::zeros(img.size(), CV_8UC1);
    for (const auto& line : lines) {
        cv::line(result, line.p1, line.p2, cv::Scalar(255), 2);
    }

    if (!cv::imwrite(argv[2], result)) {
        std::cerr << "错误：无法保存结果图像 " << argv[2] << std::endl;
        return -1;
    }
    std::cout << "已保存纯墙面直线图: " << argv[2] << std::endl;
    std::cout << "原始线段数: " << fitter.extractHoughSegments(clean).size()
              << " → 最终直线数: " << lines.size() << std::endl;

    return 0;
}