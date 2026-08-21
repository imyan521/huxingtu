#ifndef MAP_CLEANER_H
#define MAP_CLEANER_H

#include "opencv/include/opencv2/opencv.hpp"

#include <vector>
#include <queue>
#include <cmath>

class MapCleaner {
public:
    cv::Mat process(const cv::Mat& input,
                    int thresh = 200,
                    int min_branch_length = 8,
                    bool restore = false);

private:
    cv::Mat binarizeMap(const cv::Mat& img, int thresh);
    cv::Mat skeletonPruning(const cv::Mat& binary, int min_branch_length);
    std::vector<cv::Point> findEndpoints(const cv::Mat& skel);
    std::vector<cv::Point> getNeighbors(const cv::Mat& skel, int y, int x);
    std::vector<cv::Point> traceBranch(const cv::Mat& skel, cv::Point start);
    cv::Mat pruneSkeletonIterative(cv::Mat skel, int min_length);
    cv::Mat connectEndpoints(cv::Mat skel, double max_dist = 10.0);
    cv::Mat restoreThickness(const cv::Mat& skel, int ksize = 3, int iter = 1);
};

#endif