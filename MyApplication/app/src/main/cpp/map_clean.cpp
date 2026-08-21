#include "map_clean.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <queue>
#include <set>
#include <cmath>

using namespace cv;

// =========================
// 手动实现 Zhang-Suen 骨架提取（替代 ximgproc::thinning）
// =========================
void thinningZhangSuen(cv::Mat& im) {
    cv::Mat prev = cv::Mat::zeros(im.size(), CV_8UC1);
    cv::Mat diff;

    do {
        cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);
        for (int y = 1; y < im.rows - 1; y++) {
            for (int x = 1; x < im.cols - 1; x++) {
                uchar p2 = im.at<uchar>(y - 1, x);
                uchar p3 = im.at<uchar>(y - 1, x + 1);
                uchar p4 = im.at<uchar>(y, x + 1);
                uchar p5 = im.at<uchar>(y + 1, x + 1);
                uchar p6 = im.at<uchar>(y + 1, x);
                uchar p7 = im.at<uchar>(y + 1, x - 1);
                uchar p8 = im.at<uchar>(y, x - 1);
                uchar p9 = im.at<uchar>(y - 1, x - 1);

                int A = (p2 == 0 && p3 != 0) +
                        (p3 == 0 && p4 != 0) +
                        (p4 == 0 && p5 != 0) +
                        (p5 == 0 && p6 != 0) +
                        (p6 == 0 && p7 != 0) +
                        (p7 == 0 && p8 != 0) +
                        (p8 == 0 && p9 != 0) +
                        (p9 == 0 && p2 != 0);
                int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                bool m1 = ((p2 & p3 & p4) == 0);
                bool m2 = ((p4 & p5 & p6) == 0);

                if (im.at<uchar>(y, x) == 255 && A == 1 && B >= 2 && B <= 6 && m1 && m2)
                    marker.at<uchar>(y, x) = 1;
            }
        }
        im &= ~marker;

        marker.setTo(0);
        for (int y = 1; y < im.rows - 1; y++) {
            for (int x = 1; x < im.cols - 1; x++) {
                uchar p2 = im.at<uchar>(y - 1, x);
                uchar p3 = im.at<uchar>(y - 1, x + 1);
                uchar p4 = im.at<uchar>(y, x + 1);
                uchar p5 = im.at<uchar>(y + 1, x + 1);
                uchar p6 = im.at<uchar>(y + 1, x);
                uchar p7 = im.at<uchar>(y + 1, x - 1);
                uchar p8 = im.at<uchar>(y, x - 1);
                uchar p9 = im.at<uchar>(y - 1, x - 1);

                int A = (p2 == 0 && p3 != 0) +
                        (p3 == 0 && p4 != 0) +
                        (p4 == 0 && p5 != 0) +
                        (p5 == 0 && p6 != 0) +
                        (p6 == 0 && p7 != 0) +
                        (p7 == 0 && p8 != 0) +
                        (p8 == 0 && p9 != 0) +
                        (p9 == 0 && p2 != 0);
                int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                bool m1 = ((p2 & p3 & p5) == 0);
                bool m2 = ((p2 & p4 & p6) == 0);

                if (im.at<uchar>(y, x) == 255 && A == 1 && B >= 2 && B <= 6 && m1 && m2)
                    marker.at<uchar>(y, x) = 1;
            }
        }
        im &= ~marker;

        cv::absdiff(im, prev, diff);
        prev = im.clone();
    } while (cv::countNonZero(diff) > 0);
}

// =========================
// 1. binarization + noise remove
// =========================
cv::Mat MapCleaner::binarizeMap(const cv::Mat& img, int thresh) {
    cv::Mat gray, binary;

    if (img.channels() == 3)
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else
        gray = img.clone();

    cv::threshold(gray, binary, thresh, 255, cv::THRESH_BINARY_INV);

    cv::Mat labels, stats, centroids;
    int num = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8);

    for (int i = 1; i < num; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area <= 2) {
            binary.setTo(0, labels == i);
        }
    }

    return binary;
}

// =========================
// 2. skeleton + pruning
// =========================
cv::Mat MapCleaner::skeletonPruning(const cv::Mat& binary, int min_branch_length) {
    cv::Mat skel = binary.clone();
    thinningZhangSuen(skel);  // 使用手动实现的函数
    skel = pruneSkeletonIterative(skel, min_branch_length);
    skel = connectEndpoints(skel, 10.0);
    return skel;
}

// =========================
// 3. endpoints
// =========================
std::vector<cv::Point> MapCleaner::findEndpoints(const cv::Mat& skel) {
    std::vector<cv::Point> endpoints;

    for (int y = 1; y < skel.rows - 1; y++) {
        for (int x = 1; x < skel.cols - 1; x++) {
            if (skel.at<uchar>(y, x) == 0) continue;

            int count = 0;
            for (int j = -1; j <= 1; j++)
                for (int i = -1; i <= 1; i++)
                    if (!(i == 0 && j == 0))
                        count += skel.at<uchar>(y + j, x + i) > 0;

            if (count == 1)
                endpoints.emplace_back(x, y);
        }
    }
    return endpoints;
}

// =========================
// 4. neighbors
// =========================
std::vector<cv::Point> MapCleaner::getNeighbors(const cv::Mat& skel, int y, int x) {
    std::vector<cv::Point> nbs;

    for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
            if (i == 0 && j == 0) continue;

            int ny = y + j, nx = x + i;
            if (ny >= 0 && ny < skel.rows && nx >= 0 && nx < skel.cols) {
                if (skel.at<uchar>(ny, nx) > 0)
                    nbs.emplace_back(nx, ny);
            }
        }
    }
    return nbs;
}

// =========================
// 5. trace branch (BFS)
// =========================
std::vector<cv::Point> MapCleaner::traceBranch(const cv::Mat& skel, cv::Point start) {
    std::vector<cv::Point> branch;
    std::set<std::pair<int,int>> visited;

    std::queue<cv::Point> q;
    q.push(start);

    while (!q.empty()) {
        auto p = q.front(); q.pop();

        if (visited.count({p.y, p.x})) continue;
        visited.insert({p.y, p.x});
        branch.push_back(p);

        auto nbs = getNeighbors(skel, p.y, p.x);

        if (nbs.size() >= 3 && p != start)
            break;

        for (auto& nb : nbs)
            if (!visited.count({nb.y, nb.x}))
                q.push(nb);
    }

    return branch;
}

// =========================
// 6. iterative pruning
// =========================
cv::Mat MapCleaner::pruneSkeletonIterative(cv::Mat skel, int min_length) {
    bool changed = true;

    while (changed) {
        changed = false;

        auto endpoints = findEndpoints(skel);

        for (auto& ep : endpoints) {
            if (skel.at<uchar>(ep.y, ep.x) == 0) continue;

            auto branch = traceBranch(skel, ep);

            if ((int)branch.size() < min_length) {
                for (auto& p : branch)
                    skel.at<uchar>(p.y, p.x) = 0;
                changed = true;
            }
        }
    }

    return skel;
}

// =========================
// 7. connect endpoints
// =========================
cv::Mat MapCleaner::connectEndpoints(cv::Mat skel, double max_dist) {
    auto eps = findEndpoints(skel);

    for (size_t i = 0; i < eps.size(); i++) {
        for (size_t j = i + 1; j < eps.size(); j++) {
            double dist = cv::norm(eps[i] - eps[j]);

            if (dist < max_dist) {
                cv::line(skel, eps[i], eps[j], 255, 1);
            }
        }
    }
    return skel;
}

// =========================
// 8. restore thickness
// =========================
cv::Mat MapCleaner::restoreThickness(const cv::Mat& skel, int ksize, int iter) {
    cv::Mat out;
    cv::dilate(skel, out, cv::Mat::ones(ksize, ksize, CV_8U), cv::Point(-1,-1), iter);
    return out;
}

// =========================
// 9. full pipeline
// =========================
cv::Mat MapCleaner::process(const cv::Mat& input,
                            int thresh,
                            int min_branch_length,
                            bool restore) {
    auto binary = binarizeMap(input, thresh);
    auto skel = skeletonPruning(binary, min_branch_length);

    if (restore)
        return restoreThickness(skel);

    return skel;
}
