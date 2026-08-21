#ifndef LINE_FITTER_H
#define LINE_FITTER_H

#include "opencv/include/opencv2/opencv.hpp"
#include <vector>

class LineFitter {
public:
    struct Line {
        cv::Point2f p1, p2;
    };

    // 主接口：输入二值图像，输出拟合后的墙面直线
    std::vector<Line> fit(const cv::Mat& binary);

    // 调试用：霍夫阶段原始线段（与 fit 内部一致；单独调用会重复做霍夫）
    std::vector<cv::Vec4i> extractHoughSegments(const cv::Mat& binary);

private:
    // ---------- 参数常量（与 Python 完全一致） ----------
    static constexpr double ANGLE_TOL = 0.8;               // 角度聚类容差（度）
    static constexpr double CONNECT_GAP = 25.0;            // 合并线段最大间隔（像素）
    static constexpr double MIN_LINE_LEN = 6.0;            // 最小线段长度（像素）
    static constexpr int    MIN_GROUP_SEG_NUM = 2;         // DBSCAN 最小簇内点数
    static constexpr double COLINEAR_OFFSET_TOL = 3.0;     // 共线偏移聚类容差（像素）
    static constexpr double CORNER_SNAP_GAP = 12.0;        // 角点吸附最大距离（像素）
    static constexpr double MIN_CORNER_ANGLE = 20.0;       // 吸附时最小夹角（度）

    // ---------- 核心函数 ----------
    std::vector<cv::Vec4i> deduplicateSegments(const std::vector<cv::Vec4i>& segs);
    std::vector<std::vector<cv::Vec4i>> clusterByOrientation(const std::vector<cv::Vec4i>& segs);
    std::vector<cv::Vec4i> mergeCollinearGroup(const std::vector<cv::Vec4i>& segs);
    std::vector<cv::Vec4i> snapCorners(const std::vector<cv::Vec4i>& segs);

    // ---------- 几何辅助 ----------
    static double segmentLength(const cv::Vec4i& seg);
    static double lineAngle(const cv::Vec4i& seg);
    static double angleDistance(double a, double b);
    static cv::Vec4i normalizeSegment(const cv::Vec4i& seg);
    static cv::Point2f directionFromAngle(double theta_deg);
    static double dominantOrientation(const std::vector<cv::Vec4i>& segs);
    static cv::Vec4i buildSegment(double axisStart, double axisEnd, double offset, const cv::Point2f& dir);
    static cv::Point2f lineIntersection(const cv::Vec4i& a, const cv::Vec4i& b);

    // ---------- DBSCAN 实现 ----------
    // 二维点（用于方向聚类，特征为 (cos2θ, sin2θ)）
    static std::vector<int> dbscan2D(const std::vector<cv::Point2f>& points, double eps, int minPts);
    // 一维点（用于偏移量聚类）
    static std::vector<int> dbscan1D(const std::vector<double>& values, double eps, int minPts);
};

#endif