#include "floor_plan.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>

namespace fs = std::filesystem;

namespace floorplan {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Parameters copied from 0702/ortho_floor_plan.py.
constexpr double ANGLE_TOL = 30.0;
constexpr double CONNECT_GAP = 44.0;
constexpr double MIN_LINE_LEN = 12.0;
constexpr double COLINEAR_OFFSET_TOL = 3.0;
constexpr double CORNER_SNAP_GAP = 25.0;
constexpr double CORNER_SNAP_MAX_MOVE = 20.0;
constexpr double CORNER_MERGE_DIST = 15.0;
constexpr double MIN_CORNER_ANGLE = 20.0;
constexpr double RED_FILL_REMOVE_MAX_LEN = 80.0;
constexpr double RED_FILL_ENDPOINT_DIST = 20.0;
constexpr double RED_FILL_MAX_GAP = 120.0;
constexpr int OUTER_WALL_CLOSE_SIZE = 45;
constexpr int OUTER_WALL_BAND_WIDTH = 10;
const int OUTER_WALL_DETECT_CLOSE_SIZES[] = {45, 65, 85, 121};
constexpr int OUTER_REGION_MIN_AREA = 2000;
constexpr int INNER_CLUTTER_MARGIN = 10;
constexpr int INNER_CLUTTER_MIN_PIXELS = 25;
constexpr int INNER_CLUTTER_MIN_COMPONENTS = 1;
constexpr int INNER_CLUTTER_MIN_COMPONENT_AREA = 8;
constexpr double INNER_CLUTTER_MIN_RATIO = 0.01;
constexpr double INNER_CLUTTER_PROTECT_LINE_ASPECT = 5.0;
constexpr int INNER_CLUTTER_PROTECT_LINE_MIN_LENGTH = 25;
constexpr int INNER_CLUTTER_PROTECT_LINE_MAX_LENGTH = 60;
constexpr int INNER_CLUTTER_PROTECT_NEAR_WALL_MAX_AREA = 80;
constexpr double INNER_CLUTTER_PROTECT_NEAR_WALL_DIST = 8.0;
constexpr double ATTACHED_DIAGONAL_MIN_LEN = 12.0;
constexpr double ATTACHED_DIAGONAL_MAX_LEN = 90.0;
constexpr double ATTACHED_DIAGONAL_MIN_AXIS_ANGLE = 15.0;
constexpr double ATTACHED_DIAGONAL_WALL_DIST = 17.0;
constexpr int ATTACHED_DIAGONAL_ERASE_THICKNESS = 3;
constexpr double ATTACHED_DIAGONAL_ENDPOINT_CONNECT_DIST = 8.0;
constexpr double GAP_THRESHOLD = 50.0;
constexpr double SHORT_LINE_THRESHOLD = 30.0;

struct ClutterInfo {
    int inner_pixels = 0;
    int component_count = 0;
    double inner_ratio = 0.0;
    int close_size = 0;
    double outer_fill_ratio = 0.0;
    cv::Mat mask;
};

double SegmentLength(const cv::Vec4i& seg) {
    return std::hypot(seg[2] - seg[0], seg[3] - seg[1]);
}

double LineAngle(const cv::Vec4i& seg) {
    double angle = std::atan2(seg[3] - seg[1], seg[2] - seg[0]) * 180.0 / kPi;
    angle = std::fmod(angle, 180.0);
    if (angle < 0) angle += 180.0;
    return angle;
}

double AngleDistance(double a, double b) {
    double diff = std::fabs(a - b);
    diff = std::fmod(diff, 180.0);
    return std::min(diff, 180.0 - diff);
}

int PythonRound(double value) {
    const double lower = std::floor(value);
    const double fraction = value - lower;
    if (fraction < 0.5) return static_cast<int>(lower);
    if (fraction > 0.5) return static_cast<int>(lower + 1.0);
    const auto lower_int = static_cast<int64_t>(lower);
    return static_cast<int>((lower_int % 2 == 0) ? lower : lower + 1.0);
}

uint64_t Rot64(uint64_t value, int bits) {
    return (value << bits) | (value >> (64 - bits));
}

int64_t PythonTupleHash4(const cv::Vec4i& seg) {
    constexpr uint64_t kPrime1 = 11400714785074694791ULL;
    constexpr uint64_t kPrime2 = 14029467366897019727ULL;
    constexpr uint64_t kPrime5 = 2870177450012600261ULL;
    uint64_t acc = kPrime5;
    const int values[4] = {seg[0], seg[1], seg[2], seg[3]};
    for (int value : values) {
        const int64_t lane = value == -1 ? -2 : static_cast<int64_t>(value);
        acc += static_cast<uint64_t>(lane) * kPrime2;
        acc = Rot64(acc, 31);
        acc *= kPrime1;
    }
    acc += 4ULL ^ (kPrime5 ^ 3527539ULL);
    const int64_t signed_hash = static_cast<int64_t>(acc);
    return signed_hash == -1 ? 1546275796 : signed_hash;
}

std::vector<cv::Vec4i> DeduplicateSegmentsPythonSetOrder(const std::vector<cv::Vec4i>& segs) {
    struct Slot {
        bool occupied = false;
        cv::Vec4i seg;
        int64_t hash = 0;
    };

    auto insert_slot = [](std::vector<Slot>* table, const cv::Vec4i& seg, int64_t hash) -> bool {
        const size_t mask = table->size() - 1;
        size_t index = static_cast<uint64_t>(hash) & mask;
        uint64_t perturb = static_cast<uint64_t>(hash);
        while ((*table)[index].occupied) {
            if ((*table)[index].hash == hash && (*table)[index].seg == seg) return false;
            index = (index * 5 + 1 + perturb) & mask;
            perturb >>= 5;
        }
        (*table)[index].occupied = true;
        (*table)[index].seg = seg;
        (*table)[index].hash = hash;
        return true;
    };

    std::vector<Slot> table(8);
    size_t used = 0;
    size_t fill = 0;
    for (const auto& seg : segs) {
        const int64_t hash = PythonTupleHash4(seg);
        const bool inserted = insert_slot(&table, seg, hash);
        if (!inserted) continue;
        ++used;
        ++fill;
        const size_t mask = table.size() - 1;
        if (fill * 5 >= mask * 3) {
            std::vector<Slot> old_table = table;
            size_t new_size = 8;
            const size_t min_used = used * 4;
            while (new_size <= min_used) new_size <<= 1;
            table.assign(new_size, Slot{});
            used = 0;
            fill = 0;
            for (const auto& slot : old_table) {
                if (!slot.occupied) continue;
                if (insert_slot(&table, slot.seg, slot.hash)) {
                    ++used;
                    ++fill;
                }
            }
        }
    }

    std::vector<cv::Vec4i> out;
    for (const auto& slot : table) {
        if (slot.occupied) out.push_back(slot.seg);
    }
    return out;
}

cv::Point2d DirectionFromAngle(double theta_deg) {
    const double rad = theta_deg * kPi / 180.0;
    cv::Point2d dir(std::cos(rad), std::sin(rad));
    if (dir.x < 0.0 || (std::fabs(dir.x) < 1e-8 && dir.y < 0.0)) dir *= -1.0;
    return dir;
}

double DominantOrientation(const std::vector<cv::Vec4i>& segs) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double total_weight = 0.0;
    for (const auto& seg : segs) {
        const double angle = LineAngle(seg) * kPi / 180.0;
        const double weight = std::max(SegmentLength(seg), 1.0);
        sum_x += weight * std::cos(2.0 * angle);
        sum_y += weight * std::sin(2.0 * angle);
        total_weight += weight;
    }
    if (total_weight <= 1e-9) return 0.0;
    double theta = 0.5 * std::atan2(sum_y / total_weight, sum_x / total_weight);
    if (theta < 0) theta += kPi;
    return theta * 180.0 / kPi;
}

cv::Vec4i BuildSegment(double axis_start, double axis_end, double offset, const cv::Point2d& dir) {
    const cv::Point2d normal(-dir.y, dir.x);
    const cv::Point2d p1 = dir * axis_start + normal * offset;
    const cv::Point2d p2 = dir * axis_end + normal * offset;
    return cv::Vec4i(PythonRound(p1.x), PythonRound(p1.y), PythonRound(p2.x), PythonRound(p2.y));
}

std::vector<int> Dbscan1D(const std::vector<double>& values, double eps) {
    const int n = static_cast<int>(values.size());
    std::vector<int> labels(n, -1);
    int cluster_id = 0;
    for (int i = 0; i < n; ++i) {
        if (labels[i] != -1) continue;
        labels[i] = cluster_id;
        std::vector<int> stack{i};
        while (!stack.empty()) {
            const int cur = stack.back();
            stack.pop_back();
            for (int j = 0; j < n; ++j) {
                if (labels[j] == -1 && std::fabs(values[j] - values[cur]) <= eps) {
                    labels[j] = cluster_id;
                    stack.push_back(j);
                }
            }
        }
        ++cluster_id;
    }
    return labels;
}

std::vector<int> Dbscan2D(const std::vector<cv::Point2d>& values, double eps) {
    const int n = static_cast<int>(values.size());
    std::vector<int> labels(n, -1);
    int cluster_id = 0;
    for (int i = 0; i < n; ++i) {
        if (labels[i] != -1) continue;
        labels[i] = cluster_id;
        std::vector<int> stack{i};
        while (!stack.empty()) {
            const int cur = stack.back();
            stack.pop_back();
            for (int j = 0; j < n; ++j) {
                if (labels[j] == -1 && cv::norm(values[j] - values[cur]) <= eps) {
                    labels[j] = cluster_id;
                    stack.push_back(j);
                }
            }
        }
        ++cluster_id;
    }
    return labels;
}

cv::Mat Binary255(const cv::Mat& input) {
    cv::Mat out;
    cv::threshold(input, out, 0, 255, cv::THRESH_BINARY);
    return out;
}

cv::Mat BinarizeMap(const cv::Mat& img, int thresh, bool invert = true) {
    cv::Mat gray;
    if (img.channels() == 3) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img.clone();
    }

    cv::Mat binary;
    cv::threshold(gray, binary, thresh, 255, invert ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY);

    cv::Mat labels, stats, centroids;
    const int num = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8);
    for (int i = 1; i < num; ++i) {
        if (stats.at<int>(i, cv::CC_STAT_AREA) <= 2) {
            binary.setTo(0, labels == i);
        }
    }
    return binary;
}

cv::Mat Skeletonize(const cv::Mat& binary) {
    // Port of skimage 0.25.2 _fast_skeletonize from
    // skimage/morphology/_skeletonize_various_cy.pyx.
    static constexpr uchar lut[256] = {
            0, 0, 0, 1, 0, 0, 1, 3, 0, 0, 3, 1, 1, 0, 1, 3,
            0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 3, 0, 3, 3,
            0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 3, 0, 2, 2,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            2, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0,
            3, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 3, 0, 2, 0,
            0, 0, 3, 1, 0, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
            3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            2, 3, 1, 3, 0, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            2, 3, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
            3, 3, 0, 1, 0, 0, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0,
    };

    cv::Mat image01;
    cv::threshold(binary, image01, 0, 1, cv::THRESH_BINARY);

    cv::Mat skeleton = cv::Mat::zeros(image01.rows + 2, image01.cols + 2, CV_8UC1);
    image01.copyTo(skeleton(cv::Rect(1, 1, image01.cols, image01.rows)));
    cv::Mat cleaned = skeleton.clone();

    bool pixel_removed = true;
    while (pixel_removed) {
        pixel_removed = false;
        for (int pass_num = 0; pass_num < 2; ++pass_num) {
            const bool first_pass = (pass_num == 0);
            for (int row = 1; row < skeleton.rows - 1; ++row) {
                for (int col = 1; col < skeleton.cols - 1; ++col) {
                    if (!skeleton.at<uchar>(row, col)) continue;
                    const int code =
                            skeleton.at<uchar>(row - 1, col - 1) +
                            2 * skeleton.at<uchar>(row - 1, col) +
                            4 * skeleton.at<uchar>(row - 1, col + 1) +
                            8 * skeleton.at<uchar>(row, col + 1) +
                            16 * skeleton.at<uchar>(row + 1, col + 1) +
                            32 * skeleton.at<uchar>(row + 1, col) +
                            64 * skeleton.at<uchar>(row + 1, col - 1) +
                            128 * skeleton.at<uchar>(row, col - 1);
                    const uchar neighbors = lut[code];
                    if (neighbors == 0) continue;
                    if (neighbors == 3 ||
                        (neighbors == 1 && first_pass) ||
                        (neighbors == 2 && !first_pass)) {
                        cleaned.at<uchar>(row, col) = 0;
                        pixel_removed = true;
                    }
                }
            }
            cleaned.copyTo(skeleton);
        }
    }

    cv::Mat inner = skeleton(cv::Rect(1, 1, image01.cols, image01.rows)).clone();
    cv::Mat out;
    inner.convertTo(out, CV_8UC1, 255);
    return out;
}

std::vector<cv::Point> FindEndpoints(const cv::Mat& skel) {
    std::vector<cv::Point> endpoints;
    for (int y = 1; y < skel.rows - 1; ++y) {
        for (int x = 1; x < skel.cols - 1; ++x) {
            if (skel.at<uchar>(y, x) == 0) continue;
            int count = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    if (skel.at<uchar>(y + dy, x + dx) > 0) ++count;
                }
            }
            if (count == 1) endpoints.emplace_back(x, y);
        }
    }
    return endpoints;
}

std::vector<cv::Point> GetNeighbors(const cv::Mat& skel, int y, int x) {
    std::vector<cv::Point> out;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const int ny = y + dy;
            const int nx = x + dx;
            if (ny >= 0 && ny < skel.rows && nx >= 0 && nx < skel.cols &&
                skel.at<uchar>(ny, nx) > 0) {
                out.emplace_back(nx, ny);
            }
        }
    }
    return out;
}

std::vector<cv::Point> TraceBranch(const cv::Mat& skel, const cv::Point& start) {
    std::set<std::pair<int, int>> visited;
    std::vector<cv::Point> branch;
    std::vector<cv::Point> stack{start};
    while (!stack.empty()) {
        const cv::Point p = stack.back();
        stack.pop_back();
        const auto key = std::make_pair(p.y, p.x);
        if (visited.count(key)) continue;
        visited.insert(key);
        branch.push_back(p);
        const auto neighbors = GetNeighbors(skel, p.y, p.x);
        if (neighbors.size() >= 3 && p != start) break;
        for (const auto& nb : neighbors) {
            if (!visited.count(std::make_pair(nb.y, nb.x))) stack.push_back(nb);
        }
    }
    return branch;
}

cv::Mat PruneSkeletonIterative(cv::Mat skel, int min_length) {
    bool changed = true;
    while (changed) {
        changed = false;
        const auto endpoints = FindEndpoints(skel);
        for (const auto& ep : endpoints) {
            if (skel.at<uchar>(ep.y, ep.x) == 0) continue;
            const auto branch = TraceBranch(skel, ep);
            if (static_cast<int>(branch.size()) < min_length) {
                for (const auto& p : branch) skel.at<uchar>(p.y, p.x) = 0;
                changed = true;
            }
        }
    }
    return skel;
}

cv::Mat ConnectEndpoints(cv::Mat skel, double max_dist) {
    const auto endpoints = FindEndpoints(skel);
    for (size_t i = 0; i < endpoints.size(); ++i) {
        for (size_t j = i + 1; j < endpoints.size(); ++j) {
            if (cv::norm(endpoints[i] - endpoints[j]) < max_dist) {
                cv::line(skel, endpoints[i], endpoints[j], cv::Scalar(255), 1);
            }
        }
    }
    return skel;
}

cv::Mat SkeletonPruning(const cv::Mat& binary, int min_branch_length) {
    cv::Mat closed;
    cv::morphologyEx(binary, closed, cv::MORPH_CLOSE, cv::Mat::ones(3, 3, CV_8U));
    cv::Mat skel = Skeletonize(closed);
    skel = PruneSkeletonIterative(skel, min_branch_length);
    skel = ConnectEndpoints(skel, 10.0);
    return skel;
}

cv::Mat RestoreThickness(const cv::Mat& skel, int kernel_size = 3, int iterations = 1) {
    cv::Mat out;
    cv::dilate(skel, out, cv::Mat::ones(kernel_size, kernel_size, CV_8U), cv::Point(-1, -1), iterations);
    return out;
}

void BuildOuterWallMasks(const cv::Mat& binary,
                         int band_width,
                         int close_size,
                         cv::Mat* src,
                         cv::Mat* inner_region,
                         cv::Mat* outer_band) {
    *src = Binary255(binary);
    close_size = std::max(3, close_size);
    if (close_size % 2 == 0) ++close_size;
    cv::Mat merged;
    cv::morphologyEx(*src, merged, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(close_size, close_size)));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(merged, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        inner_region->release();
        outer_band->release();
        return;
    }
    auto max_it = std::max_element(contours.begin(), contours.end(), [](const auto& a, const auto& b) {
        return cv::contourArea(a) < cv::contourArea(b);
    });
    cv::Mat outer_region = cv::Mat::zeros(src->size(), CV_8UC1);
    cv::drawContours(outer_region, std::vector<std::vector<cv::Point>>{*max_it}, -1, 255, cv::FILLED);
    band_width = std::max(3, band_width);
    cv::erode(outer_region, *inner_region,
              cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                         cv::Size(band_width * 2 + 1, band_width * 2 + 1)));
    cv::subtract(outer_region, *inner_region, *outer_band);
}

bool DetectInternalClutter(const cv::Mat& binary, ClutterInfo* best_info) {
    cv::Mat src = Binary255(binary);
    best_info->mask = cv::Mat::zeros(src.size(), CV_8UC1);

    const int detect_margin = std::max(OUTER_WALL_BAND_WIDTH + 2, INNER_CLUTTER_MARGIN);
    for (int close_size : OUTER_WALL_DETECT_CLOSE_SIZES) {
        cv::Mat inner_region, outer_band;
        BuildOuterWallMasks(binary, detect_margin, close_size, &src, &inner_region, &outer_band);
        if (inner_region.empty() || outer_band.empty()) continue;

        cv::Mat outer_region;
        cv::bitwise_or(inner_region, outer_band, outer_region);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(outer_region, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty()) continue;
        auto max_it = std::max_element(contours.begin(), contours.end(), [](const auto& a, const auto& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });
        const double outer_area = cv::contourArea(*max_it);
        const cv::Rect rect = cv::boundingRect(*max_it);
        const double outer_fill_ratio = outer_area / std::max(1.0, static_cast<double>(rect.area()));
        if (outer_area < OUTER_REGION_MIN_AREA) continue;

        cv::Mat labels, stats, centroids;
        const int num = cv::connectedComponentsWithStats(src, labels, stats, centroids, 8);
        cv::Mat wall_band_inv;
        cv::threshold(outer_band, wall_band_inv, 0, 1, cv::THRESH_BINARY_INV);
        cv::Mat wall_band_dist;
        cv::distanceTransform(wall_band_inv, wall_band_dist, cv::DIST_L2, 3);

        cv::Mat remove_mask = cv::Mat::zeros(src.size(), CV_8UC1);
        int component_count = 0;
        int kept_area = 0;

        for (int label = 1; label < num; ++label) {
            const int area = stats.at<int>(label, cv::CC_STAT_AREA);
            const int x = stats.at<int>(label, cv::CC_STAT_LEFT);
            const int y = stats.at<int>(label, cv::CC_STAT_TOP);
            const int w = stats.at<int>(label, cv::CC_STAT_WIDTH);
            const int h = stats.at<int>(label, cv::CC_STAT_HEIGHT);
            const double aspect = std::max(w, h) / std::max(1.0, static_cast<double>(std::min(w, h)));
            const int long_side = std::max(w, h);
            const bool protected_wall_fragment =
                    aspect >= INNER_CLUTTER_PROTECT_LINE_ASPECT &&
                    long_side >= INNER_CLUTTER_PROTECT_LINE_MIN_LENGTH &&
                    long_side <= INNER_CLUTTER_PROTECT_LINE_MAX_LENGTH;

            cv::Mat component = labels == label;
            double min_dist = 1e9;
            for (int yy = y; yy < y + h; ++yy) {
                for (int xx = x; xx < x + w; ++xx) {
                    if (component.at<uchar>(yy, xx)) {
                        min_dist = std::min(min_dist, static_cast<double>(wall_band_dist.at<float>(yy, xx)));
                    }
                }
            }
            const bool protected_near_wall =
                    area <= INNER_CLUTTER_PROTECT_NEAR_WALL_MAX_AREA &&
                    min_dist <= INNER_CLUTTER_PROTECT_NEAR_WALL_DIST;

            const int inner_count = cv::countNonZero(component & inner_region);
            const int wall_band_count = cv::countNonZero(component & outer_band);
            if (area >= INNER_CLUTTER_MIN_COMPONENT_AREA &&
                inner_count > 0 &&
                wall_band_count == 0 &&
                !protected_wall_fragment &&
                !protected_near_wall) {
                ++component_count;
                kept_area += area;
                remove_mask.setTo(255, component);
            }
        }

        const double inner_ratio = kept_area / std::max(1.0, static_cast<double>(cv::countNonZero(src)));
        if (kept_area > best_info->inner_pixels) {
            best_info->inner_pixels = kept_area;
            best_info->component_count = component_count;
            best_info->inner_ratio = inner_ratio;
            best_info->close_size = close_size;
            best_info->outer_fill_ratio = outer_fill_ratio;
            best_info->mask = remove_mask;
        }
    }

    // A mask that classifies nearly the complete structural raster as
    // "internal clutter" means the provisional closed outer region was
    // fitted to the wrong topology.  Applying it leaves Hough/partition
    // recovery with almost no measured wall evidence (one regression scene
    // removed more than 99% of the wall pixels).  Furniture clutter can be
    // substantial, but it cannot legitimately account for most of every
    // observed wall.  Keep the original structural raster in that case and
    // let the later semantic/two-sided-free checks reject individual objects.
    constexpr double kMaximumSafeClutterRemovalRatio = 0.78;
    return best_info->inner_pixels >= INNER_CLUTTER_MIN_PIXELS &&
           best_info->component_count >= INNER_CLUTTER_MIN_COMPONENTS &&
           best_info->inner_ratio >= INNER_CLUTTER_MIN_RATIO &&
           best_info->inner_ratio <= kMaximumSafeClutterRemovalRatio;
}

cv::Mat RemoveInternalClutter(const cv::Mat& binary, const cv::Mat& clutter_mask) {
    cv::Mat src = Binary255(binary);
    if (clutter_mask.empty()) return src;
    cv::Mat keep_mask;
    cv::bitwise_not(Binary255(clutter_mask), keep_mask);
    cv::Mat out;
    cv::bitwise_and(src, keep_mask, out);
    return out;
}

int CountConnectedEndpoints(const cv::Vec4i& seg, const std::vector<cv::Vec4i>& segs, double connect_dist) {
    int connected = 0;
    const cv::Point2d endpoints[] = {cv::Point2d(seg[0], seg[1]), cv::Point2d(seg[2], seg[3])};
    for (const auto& endpoint : endpoints) {
        bool has_connection = false;
        for (const auto& other : segs) {
            if (other == seg) continue;
            if (cv::norm(endpoint - cv::Point2d(other[0], other[1])) <= connect_dist ||
                cv::norm(endpoint - cv::Point2d(other[2], other[3])) <= connect_dist) {
                has_connection = true;
                break;
            }
        }
        if (has_connection) ++connected;
    }
    return connected;
}

bool KeepAfterShortFilter(const cv::Vec4i& seg,
                          const std::vector<cv::Vec4i>& segs,
                          double min_len,
                          double connected_min_len,
                          double connect_dist) {
    const double length = SegmentLength(seg);
    if (length >= min_len) return true;

    const int connected = CountConnectedEndpoints(seg, segs, connect_dist);
    if (connected >= 2) return true;
    if (connected >= 1 && length >= connected_min_len) return true;
    return false;
}

std::pair<cv::Mat, std::vector<cv::Vec4i>> RemoveAttachedDiagonalClutter(const cv::Mat& binary) {
    cv::Mat src = Binary255(binary);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(src, lines, 1, kPi / 180.0, 12, ATTACHED_DIAGONAL_MIN_LEN, 4);
    if (lines.empty()) return {src, {}};

    cv::Mat axis_mask = cv::Mat::zeros(src.size(), CV_8UC1);
    std::vector<cv::Vec4i> diagonal_candidates;
    for (const auto& seg : lines) {
        const double length = SegmentLength(seg);
        if (length < ATTACHED_DIAGONAL_MIN_LEN) continue;
        const double angle = LineAngle(seg);
        const double axis_dist = std::min(AngleDistance(angle, 0.0), AngleDistance(angle, 90.0));
        if (axis_dist <= ATTACHED_DIAGONAL_MIN_AXIS_ANGLE) {
            cv::line(axis_mask, {seg[0], seg[1]}, {seg[2], seg[3]}, 255, 3);
        } else if (length <= ATTACHED_DIAGONAL_MAX_LEN) {
            diagonal_candidates.push_back(seg);
        }
    }
    if (diagonal_candidates.empty() || cv::countNonZero(axis_mask) == 0) return {src, {}};

    cv::Mat axis_inv;
    cv::threshold(axis_mask, axis_inv, 0, 1, cv::THRESH_BINARY_INV);
    cv::Mat wall_dist;
    cv::distanceTransform(axis_inv, wall_dist, cv::DIST_L2, 3);
    cv::Mat restored = src.clone();
    std::vector<cv::Vec4i> removed;
    for (const auto& seg : diagonal_candidates) {
        if (CountConnectedEndpoints(seg, lines, ATTACHED_DIAGONAL_ENDPOINT_CONNECT_DIST) >= 2) continue;
        cv::Mat line_mask = cv::Mat::zeros(src.size(), CV_8UC1);
        cv::line(line_mask, {seg[0], seg[1]}, {seg[2], seg[3]}, 255, 1);
        const int total = cv::countNonZero(line_mask);
        if (total == 0) continue;
        int near_count = 0;
        for (int y = 0; y < line_mask.rows; ++y) {
            for (int x = 0; x < line_mask.cols; ++x) {
                if (line_mask.at<uchar>(y, x) > 0 && wall_dist.at<float>(y, x) <= ATTACHED_DIAGONAL_WALL_DIST) {
                    ++near_count;
                }
            }
        }
        const double near_ratio = near_count / static_cast<double>(total);
        if (near_ratio < 0.35) continue;
        cv::line(restored, {seg[0], seg[1]}, {seg[2], seg[3]}, 0, ATTACHED_DIAGONAL_ERASE_THICKNESS);
        removed.push_back(seg);
    }
    return {restored, removed};
}

std::vector<std::vector<cv::Vec4i>> ClusterByOrientation(const std::vector<cv::Vec4i>& segs) {
    if (segs.empty()) return {};
    if (segs.size() == 1) return {segs};
    std::vector<cv::Point2d> features;
    features.reserve(segs.size());
    for (const auto& seg : segs) {
        const double angle = LineAngle(seg) * kPi / 180.0;
        features.emplace_back(std::cos(2.0 * angle), std::sin(2.0 * angle));
    }
    const double eps = std::max(1e-3, 2.0 * std::sin(ANGLE_TOL * kPi / 180.0));
    const auto labels = Dbscan2D(features, eps);
    std::map<int, std::vector<cv::Vec4i>> groups;
    for (size_t i = 0; i < segs.size(); ++i) groups[labels[i]].push_back(segs[i]);
    std::vector<std::vector<cv::Vec4i>> out;
    for (auto& kv : groups) out.push_back(kv.second);
    return out;
}

std::pair<std::vector<cv::Vec4i>, std::vector<cv::Vec4i>> MergeColinearGroup(
        const std::vector<cv::Vec4i>& segs,
        double maximum_gap,
        double minimum_line_length) {
    struct Proj {
        cv::Vec4i seg;
        double start;
        double end;
        double offset;
        double len;
    };

    const double theta = DominantOrientation(segs);
    const cv::Point2d dir = DirectionFromAngle(theta);
    const cv::Point2d normal(-dir.y, dir.x);
    std::vector<Proj> proj_data;
    std::vector<double> offsets;
    for (const auto& seg : segs) {
        const cv::Point2d p1(seg[0], seg[1]);
        const cv::Point2d p2(seg[2], seg[3]);
        const double a1 = p1.dot(dir);
        const double a2 = p2.dot(dir);
        const double offset = 0.5 * (p1.dot(normal) + p2.dot(normal));
        proj_data.push_back({seg, std::min(a1, a2), std::max(a1, a2), offset, SegmentLength(seg)});
        offsets.push_back(offset);
    }

    const auto labels = Dbscan1D(offsets, COLINEAR_OFFSET_TOL);
    std::map<int, std::vector<Proj>> clusters;
    for (size_t i = 0; i < proj_data.size(); ++i) clusters[labels[i]].push_back(proj_data[i]);

    std::vector<cv::Vec4i> green;
    std::vector<cv::Vec4i> red;
    for (auto& kv : clusters) {
        auto cluster = kv.second;
        std::sort(cluster.begin(), cluster.end(), [](const Proj& a, const Proj& b) { return a.start < b.start; });
        double weight_sum = 0.0;
        double weighted_offset = 0.0;
        for (const auto& item : cluster) {
            weight_sum += item.len;
            weighted_offset += item.offset * item.len;
        }
        const double fitted_offset = weight_sum > 1e-9 ? weighted_offset / weight_sum : cluster.front().offset;

        double curr_start = cluster.front().start;
        double curr_end = cluster.front().end;
        for (size_t i = 1; i < cluster.size(); ++i) {
            const double gap = cluster[i].start - curr_end;
            if (gap <= maximum_gap) {
                curr_end = std::max(curr_end, cluster[i].end);
            } else {
                if (curr_end - curr_start >= minimum_line_length) green.push_back(BuildSegment(curr_start, curr_end, fitted_offset, dir));
                red.push_back(BuildSegment(curr_end, cluster[i].start, fitted_offset, dir));
                curr_start = cluster[i].start;
                curr_end = cluster[i].end;
            }
        }
        if (curr_end - curr_start >= minimum_line_length) green.push_back(BuildSegment(curr_start, curr_end, fitted_offset, dir));
    }
    return {green, red};
}

cv::Point2d LineIntersection(const cv::Vec4i& a, const cv::Vec4i& b, bool* ok) {
    const cv::Point2d p(a[0], a[1]);
    const cv::Point2d r(a[2] - a[0], a[3] - a[1]);
    const cv::Point2d q(b[0], b[1]);
    const cv::Point2d s(b[2] - b[0], b[3] - b[1]);
    const double cross = r.x * s.y - r.y * s.x;
    if (std::fabs(cross) < 1e-6) {
        *ok = false;
        return {};
    }
    const cv::Point2d qp = q - p;
    const double t = (qp.x * s.y - qp.y * s.x) / cross;
    *ok = true;
    return p + r * t;
}

cv::Point2d LineIntersectionPoints(const cv::Point2d& p1,
                                   const cv::Point2d& p2,
                                   const cv::Point2d& q1,
                                   const cv::Point2d& q2,
                                   bool* ok) {
    const cv::Point2d r = p2 - p1;
    const cv::Point2d s = q2 - q1;
    const double cross = r.x * s.y - r.y * s.x;
    if (std::fabs(cross) < 1e-6) {
        *ok = false;
        return {};
    }
    const cv::Point2d qp = q1 - p1;
    const double t = (qp.x * s.y - qp.y * s.x) / cross;
    *ok = true;
    return p1 + r * t;
}

std::vector<cv::Vec4i> SnapCorners(
        const std::vector<cv::Vec4i>& segs,
        double snap_gap,
        double maximum_move,
        double merge_distance,
        double minimum_line_length) {
    std::vector<std::pair<cv::Point2d, cv::Point2d>> snapped;
    std::vector<double> angles;
    for (const auto& seg : segs) {
        snapped.push_back({cv::Point2d(seg[0], seg[1]), cv::Point2d(seg[2], seg[3])});
        angles.push_back(LineAngle(seg));
    }

    for (size_t i = 0; i < snapped.size(); ++i) {
        for (size_t j = i + 1; j < snapped.size(); ++j) {
            if (AngleDistance(angles[i], angles[j]) < MIN_CORNER_ANGLE) continue;
            bool ok = false;
            const cv::Point2d inter = LineIntersectionPoints(snapped[i].first, snapped[i].second,
                                                             snapped[j].first, snapped[j].second,
                                                             &ok);
            if (!ok || !std::isfinite(inter.x) || !std::isfinite(inter.y)) continue;
            const double di0 = cv::norm(snapped[i].first - inter);
            const double di1 = cv::norm(snapped[i].second - inter);
            const double dj0 = cv::norm(snapped[j].first - inter);
            const double dj1 = cv::norm(snapped[j].second - inter);
            if (std::min(di0, di1) > snap_gap || std::min(dj0, dj1) > snap_gap) continue;
            if (std::min(di0, di1) > maximum_move || std::min(dj0, dj1) > maximum_move) continue;
            (di1 < di0 ? snapped[i].second : snapped[i].first) = inter;
            (dj1 < dj0 ? snapped[j].second : snapped[j].first) = inter;
        }
    }

    std::vector<cv::Point2d> corners;
    for (const auto& pair : snapped) {
        corners.push_back(pair.first);
        corners.push_back(pair.second);
    }
    std::vector<int> labels(corners.size(), -1);
    int label = 0;
    for (size_t i = 0; i < corners.size(); ++i) {
        if (labels[i] != -1) continue;
        labels[i] = label;
        for (size_t j = i + 1; j < corners.size(); ++j) {
            if (labels[j] == -1 && cv::norm(corners[i] - corners[j]) < merge_distance) {
                labels[j] = label;
            }
        }
        ++label;
    }
    std::vector<cv::Point2d> sums(label, cv::Point2d(0, 0));
    std::vector<int> counts(label, 0);
    for (size_t i = 0; i < corners.size(); ++i) {
        sums[labels[i]] += corners[i];
        ++counts[labels[i]];
    }
    for (size_t i = 0; i < corners.size(); ++i) corners[i] = sums[labels[i]] * (1.0 / counts[labels[i]]);

    std::vector<cv::Vec4i> out;
    for (size_t i = 0; i + 1 < corners.size(); i += 2) {
        cv::Vec4i seg(PythonRound(corners[i].x), PythonRound(corners[i].y),
                      PythonRound(corners[i + 1].x), PythonRound(corners[i + 1].y));
        if (SegmentLength(seg) >= minimum_line_length) out.push_back(seg);
    }
    return out;
}

bool IsWallCorner(const cv::Point2d& corner, const std::vector<cv::Vec4i>& green_segs, double snap_dist = 15.0) {
    std::set<int> angle_bins;
    for (const auto& seg : green_segs) {
        if (cv::norm(corner - cv::Point2d(seg[0], seg[1])) < snap_dist ||
            cv::norm(corner - cv::Point2d(seg[2], seg[3])) < snap_dist) {
            angle_bins.insert(static_cast<int>(std::round((LineAngle(seg) / 10.0)) * 10));
        }
    }
    return angle_bins.size() >= 2;
}

bool HasBranchingCornerNearGapEndpoint(const cv::Point2d& corner,
                                       double candidate_theta,
                                       const std::vector<cv::Vec4i>& green_segs,
                                       double snap_dist = 15.0,
                                       double angle_tol = 15.0) {
    std::vector<double> connected_angles;
    for (const auto& seg : green_segs) {
        if (cv::norm(corner - cv::Point2d(seg[0], seg[1])) < snap_dist ||
            cv::norm(corner - cv::Point2d(seg[2], seg[3])) < snap_dist) {
            connected_angles.push_back(LineAngle(seg));
        }
    }
    if (connected_angles.size() < 2) return false;
    bool colinear = false;
    bool branch = false;
    for (double angle : connected_angles) {
        colinear = colinear || AngleDistance(angle, candidate_theta) <= angle_tol;
        branch = branch || AngleDistance(angle, candidate_theta) > std::max(MIN_CORNER_ANGLE, angle_tol);
    }
    return colinear && branch;
}

bool SegmentsOverlapOnSameLine(const cv::Vec4i& a, const cv::Vec4i& b,
                              double angle_tol = 5.0,
                              double offset_tol = 3.0,
                              double min_overlap = 1.0) {
    if (AngleDistance(LineAngle(a), LineAngle(b)) > angle_tol) return false;
    const double theta = DominantOrientation({a, b});
    const cv::Point2d dir = DirectionFromAngle(theta);
    const cv::Point2d normal(-dir.y, dir.x);
    const cv::Point2d a1(a[0], a[1]), a2(a[2], a[3]), b1(b[0], b[1]), b2(b[2], b[3]);
    const double offset_a = 0.5 * (a1.dot(normal) + a2.dot(normal));
    const double offset_b = 0.5 * (b1.dot(normal) + b2.dot(normal));
    if (std::fabs(offset_a - offset_b) > offset_tol) return false;
    std::array<double, 2> as{a1.dot(dir), a2.dot(dir)};
    std::array<double, 2> bs{b1.dot(dir), b2.dot(dir)};
    std::sort(as.begin(), as.end());
    std::sort(bs.begin(), bs.end());
    const double overlap = std::min(as[1], bs[1]) - std::max(as[0], bs[0]);
    return overlap > min_overlap;
}

std::pair<std::vector<cv::Vec4i>, std::set<std::array<int, 4>>> GenerateRedLinesAndFilters(
        const std::vector<cv::Vec4i>& segs) {
    std::vector<cv::Vec4i> red_lines;
    std::set<std::array<int, 4>> to_remove;
    if (segs.size() < 2) return {red_lines, to_remove};

    for (const auto& group : ClusterByOrientation(segs)) {
        const double theta = DominantOrientation(group);
        const cv::Point2d dir = DirectionFromAngle(theta);
        const cv::Point2d normal(-dir.y, dir.x);
        struct Proj {
            cv::Vec4i seg;
            double start;
            double end;
            double offset;
        };
        std::vector<Proj> proj;
        std::vector<double> offsets;
        for (const auto& seg : group) {
            const cv::Point2d p1(seg[0], seg[1]), p2(seg[2], seg[3]);
            const double a1 = p1.dot(dir);
            const double a2 = p2.dot(dir);
            const double offset = 0.5 * (p1.dot(normal) + p2.dot(normal));
            proj.push_back({seg, std::min(a1, a2), std::max(a1, a2), offset});
            offsets.push_back(offset);
        }
        const auto labels = Dbscan1D(offsets, COLINEAR_OFFSET_TOL);
        std::map<int, std::vector<Proj>> clusters;
        for (size_t i = 0; i < proj.size(); ++i) clusters[labels[i]].push_back(proj[i]);
        for (auto& kv : clusters) {
            auto cluster = kv.second;
            if (cluster.size() < 2) continue;
            std::sort(cluster.begin(), cluster.end(), [](const Proj& a, const Proj& b) { return a.start < b.start; });
            for (size_t i = 0; i + 1 < cluster.size(); ++i) {
                const double gap = cluster[i + 1].start - cluster[i].end;
                if (gap <= GAP_THRESHOLD || gap > RED_FILL_MAX_GAP) continue;
                const cv::Vec4i curr = cluster[i].seg;
                const cv::Vec4i next = cluster[i + 1].seg;
                const cv::Point2d c1(curr[2], curr[3]);
                const cv::Point2d c2(next[0], next[1]);
                if (IsWallCorner(c1, segs) && IsWallCorner(c2, segs)) continue;
                if (HasBranchingCornerNearGapEndpoint(c1, theta, segs) ||
                    HasBranchingCornerNearGapEndpoint(c2, theta, segs)) continue;
                const cv::Vec4i red(curr[2], curr[3], next[0], next[1]);
                bool overlaps = false;
                for (const auto& seg : segs) {
                    if (SegmentsOverlapOnSameLine(red, seg)) {
                        overlaps = true;
                        break;
                    }
                }
                if (overlaps) continue;
                red_lines.push_back(red);
                for (const auto& seg : segs) {
                    if (SegmentLength(seg) > RED_FILL_REMOVE_MAX_LEN) continue;
                    const cv::Point2d p1(seg[0], seg[1]);
                    const cv::Point2d p2(seg[2], seg[3]);
                    const bool from_c1 = cv::norm(p1 - c1) < RED_FILL_ENDPOINT_DIST || cv::norm(p2 - c1) < RED_FILL_ENDPOINT_DIST;
                    const bool from_c2 = cv::norm(p1 - c2) < RED_FILL_ENDPOINT_DIST || cv::norm(p2 - c2) < RED_FILL_ENDPOINT_DIST;
                    if (!from_c1 && !from_c2) continue;
                    if (AngleDistance(LineAngle(seg), theta) > ANGLE_TOL) continue;
                    const double seg_offset = 0.5 * (p1.dot(normal) + p2.dot(normal));
                    if (std::fabs(seg_offset - cluster[i].offset) > COLINEAR_OFFSET_TOL) {
                        to_remove.insert({seg[0], seg[1], seg[2], seg[3]});
                    }
                }
            }
        }
    }
    return {red_lines, to_remove};
}

double SegmentSupportRatio(const cv::Mat& wall_binary, const cv::Vec4i& seg, int thickness = 5) {
    cv::Mat mask = cv::Mat::zeros(wall_binary.size(), CV_8UC1);
    cv::line(mask, {seg[0], seg[1]}, {seg[2], seg[3]}, cv::Scalar(255), thickness);
    cv::Mat supported;
    cv::bitwise_and(mask, wall_binary, supported);
    const int mask_pixels = cv::countNonZero(mask);
    if (mask_pixels <= 0) return 0.0;
    return static_cast<double>(cv::countNonZero(supported)) / mask_pixels;
}

std::string PathJoin(const std::string& dir, const std::string& name) {
    return (fs::path(dir) / name).string();
}

void EnsureDir(const std::string& dir) {
    if (!dir.empty()) fs::create_directories(dir);
}

double ProjectionOverlapRatio(const cv::Vec4i& first, const cv::Vec4i& second) {
    const cv::Point2d direction = DirectionFromAngle(LineAngle(first));
    auto span = [&](const cv::Vec4i& segment) {
        std::array<double, 2> values{
                cv::Point2d(segment[0], segment[1]).dot(direction),
                cv::Point2d(segment[2], segment[3]).dot(direction)};
        std::sort(values.begin(), values.end());
        return values;
    };
    const auto first_span = span(first);
    const auto second_span = span(second);
    const double overlap = std::max(
            0.0,
            std::min(first_span[1], second_span[1]) -
                    std::max(first_span[0], second_span[0]));
    return overlap / std::max(
            1.0,
            std::min(first_span[1] - first_span[0],
                     second_span[1] - second_span[0]));
}

std::vector<cv::Vec4i> TraceSkeletonSegments(
        const cv::Mat& skeleton,
        double minimum_segment_length,
        double simplify_epsilon) {
    if (skeleton.empty()) return {};
    auto active_neighbors = [&](int index) {
        std::vector<int> neighbors;
        const int x = index % skeleton.cols;
        const int y = index / skeleton.cols;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                const int next_x = x + dx;
                const int next_y = y + dy;
                if (next_x < 0 || next_x >= skeleton.cols ||
                    next_y < 0 || next_y >= skeleton.rows ||
                    skeleton.at<uchar>(next_y, next_x) == 0) {
                    continue;
                }
                neighbors.push_back(next_y * skeleton.cols + next_x);
            }
        }
        return neighbors;
    };
    auto edge_key = [](int first, int second) {
        const uint32_t low = static_cast<uint32_t>(std::min(first, second));
        const uint32_t high = static_cast<uint32_t>(std::max(first, second));
        return (static_cast<uint64_t>(low) << 32) | high;
    };
    std::set<uint64_t> visited_edges;
    std::vector<cv::Vec4i> segments;
    auto trace_from = [&](int start, int next) {
        std::vector<cv::Point> path{
                cv::Point(start % skeleton.cols, start / skeleton.cols)};
        int previous = start;
        int current = next;
        visited_edges.insert(edge_key(previous, current));
        while (true) {
            path.emplace_back(
                    current % skeleton.cols,
                    current / skeleton.cols);
            const auto neighbors = active_neighbors(current);
            if (neighbors.size() != 2) break;
            const int following =
                    neighbors[0] == previous ? neighbors[1] : neighbors[0];
            if (visited_edges.count(edge_key(current, following)) > 0) break;
            previous = current;
            current = following;
            visited_edges.insert(edge_key(previous, current));
        }
        if (path.size() < 2) return;
        std::vector<cv::Point> simplified;
        cv::approxPolyDP(path, simplified, simplify_epsilon, false);
        for (size_t index = 1; index < simplified.size(); ++index) {
            const cv::Vec4i segment(
                    simplified[index - 1].x,
                    simplified[index - 1].y,
                    simplified[index].x,
                    simplified[index].y);
            if (SegmentLength(segment) >= minimum_segment_length) {
                segments.push_back(segment);
            }
        }
    };
    // Trace branches from endpoints and junctions first.
    for (int index = 0; index < skeleton.rows * skeleton.cols; ++index) {
        if (skeleton.at<uchar>(index / skeleton.cols,
                               index % skeleton.cols) == 0) {
            continue;
        }
        const auto neighbors = active_neighbors(index);
        if (neighbors.size() == 2) continue;
        for (int next : neighbors) {
            if (visited_edges.count(edge_key(index, next)) == 0) {
                trace_from(index, next);
            }
        }
    }
    // Preserve closed loops (columns/fixed cabinets) that contain no endpoint
    // or junction and therefore were not reached above.
    for (int index = 0; index < skeleton.rows * skeleton.cols; ++index) {
        if (skeleton.at<uchar>(index / skeleton.cols,
                               index % skeleton.cols) == 0) {
            continue;
        }
        for (int next : active_neighbors(index)) {
            if (visited_edges.count(edge_key(index, next)) == 0) {
                trace_from(index, next);
            }
        }
    }
    return segments;
}

std::vector<cv::Vec4i> DetectInternalWallSegments(
        const cv::Mat& stable_wall_mask,
        const std::vector<cv::Point>& outer_polygon,
        double meters_per_pixel,
        const std::string& debug_dir,
        const cv::Mat& semantic_bgr = {},
        const std::vector<cv::Point2f>& trajectory_points_px = {},
        const cv::Mat& observed_support_mask = {}) {
    if (stable_wall_mask.empty() || outer_polygon.size() < 3) return {};
    const double resolution =
            std::isfinite(meters_per_pixel) && meters_per_pixel > 1e-4
                    ? meters_per_pixel
                    : 0.05;
    // Red strokes represent structural/obstacle centerlines.  Interior
    // partitions in places such as toilet cubicles and storage bays are often
    // substantially shorter than a normal room wall.  Requiring 0.80 m here
    // made those lines visible in the map raster but absent from the fitted
    // floor plan.  Keep a conservative pixel floor, while scaling the useful
    // physical minimum down to 0.35 m; support and semantic checks below still
    // reject isolated skeleton twigs.
    const double minimum_length =
            std::clamp(0.35 / resolution, 8.0, 72.0);
    // Only exclude the fitted exterior stroke itself.  The old 2.5%-of-map
    // clearance could exceed a metre on a large map and removed legitimate
    // partitions running close to an outside wall.
    const double boundary_clearance = std::clamp(
            0.12 / resolution,
            2.0,
            8.0);
    const double endpoint_tolerance =
            std::clamp(0.10 / resolution, 2.0, 12.0);
    const int support_radius =
            std::clamp(static_cast<int>(std::round(0.07 / resolution)), 1, 4);
    const int support_thickness =
            std::clamp(support_radius * 2 + 1, 3, 9);
    const double hough_gap =
            std::clamp(0.10 / resolution, 2.0, 10.0);

    cv::Mat stable = Binary255(stable_wall_mask);
    cv::Mat observed =
            !observed_support_mask.empty() &&
                    observed_support_mask.size() == stable.size()
            ? Binary255(observed_support_mask)
            : stable;
    cv::Mat supported_wall;
    cv::dilate(
            observed,
            supported_wall,
            cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                cv::Size(support_radius * 2 + 1,
                             support_radius * 2 + 1)));
    cv::Mat semantic_free;
    if (!semantic_bgr.empty() && semantic_bgr.size() == stable.size()) {
        cv::inRange(
                semantic_bgr,
                cv::Scalar(245, 245, 245),
                cv::Scalar(255, 255, 255),
                semantic_free);
        const int free_tolerance = std::clamp(
                static_cast<int>(std::round(0.10 / resolution)), 1, 5);
        cv::dilate(
                semantic_free,
                semantic_free,
                cv::getStructuringElement(
                        cv::MORPH_ELLIPSE,
                        cv::Size(free_tolerance * 2 + 1,
                                 free_tolerance * 2 + 1)));
    }

    std::vector<cv::Vec4i> candidates;
    cv::HoughLinesP(
            stable,
            candidates,
            1.0,
            kPi / 360.0,
            std::max(10, static_cast<int>(std::round(minimum_length * 0.55))),
            minimum_length,
            hough_gap);
    const std::vector<cv::Vec4i> topology_candidates =
            TraceSkeletonSegments(
                    stable,
                    std::clamp(0.70 / resolution, 10.0, 112.0),
                    std::clamp(0.12 / resolution, 1.5, 7.0));
    candidates.insert(
            candidates.end(),
            topology_candidates.begin(),
            topology_candidates.end());

    std::vector<cv::Point2d> wall_points;
    wall_points.reserve(cv::countNonZero(observed));
    for (int y = 0; y < observed.rows; ++y) {
        for (int x = 0; x < observed.cols; ++x) {
            if (observed.at<uchar>(y, x) == 0) continue;
            if (cv::pointPolygonTest(
                        outer_polygon,
                        cv::Point2f(static_cast<float>(x),
                                    static_cast<float>(y)),
                        false) >= 0.0) {
                wall_points.emplace_back(x, y);
            }
        }
    }

    cv::Mat candidate_debug;
    cv::cvtColor(observed, candidate_debug, cv::COLOR_GRAY2BGR);

    // Estimate the two dominant building axes from the already cleaned green
    // outline. Lines close to those axes are snapped exactly straight. This
    // removes small Hough/skeleton angle errors while retaining genuinely
    // diagonal internal structures.
    double orientation_cosine = 0.0;
    double orientation_sine = 0.0;
    for (size_t index = 0; index < outer_polygon.size(); ++index) {
        const cv::Point2d start(outer_polygon[index]);
        const cv::Point2d end(
                outer_polygon[(index + 1) % outer_polygon.size()]);
        const cv::Point2d edge = end - start;
        const double length = cv::norm(edge);
        if (length < 1.0) continue;
        const double angle = std::atan2(edge.y, edge.x);
        orientation_cosine += length * std::cos(4.0 * angle);
        orientation_sine += length * std::sin(4.0 * angle);
    }
    const double primary_axis_degrees =
            0.25 * std::atan2(orientation_sine, orientation_cosine) *
            180.0 / kPi;
    auto straighten_to_building_axis = [&](const cv::Vec4i& input) {
        const double input_angle = LineAngle(input);
        double best_axis = primary_axis_degrees;
        if (AngleDistance(input_angle, primary_axis_degrees + 90.0) <
            AngleDistance(input_angle, primary_axis_degrees)) {
            best_axis = primary_axis_degrees + 90.0;
        }
        if (AngleDistance(input_angle, best_axis) > 12.0) return input;
        const cv::Point2d direction = DirectionFromAngle(best_axis);
        const cv::Point2d normal(-direction.y, direction.x);
        const cv::Point2d first(input[0], input[1]);
        const cv::Point2d second(input[2], input[3]);
        const double start_projection = first.dot(direction);
        const double end_projection = second.dot(direction);
        const double offset = 0.5 * (first.dot(normal) + second.dot(normal));
        return BuildSegment(
                std::min(start_projection, end_projection),
                std::max(start_projection, end_projection),
                offset,
                direction);
    };

    std::vector<cv::Vec4i> accepted;
    for (const auto& segment : candidates) {
        const double length = SegmentLength(segment);
        if (length < minimum_length) continue;
        const cv::Point2f midpoint(
                0.5f * (segment[0] + segment[2]),
                0.5f * (segment[1] + segment[3]));
        const double midpoint_distance =
                cv::pointPolygonTest(outer_polygon, midpoint, true);
        if (midpoint_distance < boundary_clearance) continue;

        double nearest_edge_distance =
                std::numeric_limits<double>::infinity();
        double nearest_edge_angle = 0.0;
        for (size_t edge_index = 0;
             edge_index < outer_polygon.size();
             ++edge_index) {
            const cv::Point2d edge_start(
                    outer_polygon[edge_index].x,
                    outer_polygon[edge_index].y);
            const cv::Point2d edge_end(
                    outer_polygon[(edge_index + 1) % outer_polygon.size()].x,
                    outer_polygon[(edge_index + 1) % outer_polygon.size()].y);
            const cv::Point2d edge = edge_end - edge_start;
            const double edge_length_squared = edge.dot(edge);
            if (edge_length_squared <= 1e-6) continue;
            const cv::Point2d midpoint_double(midpoint.x, midpoint.y);
            const double fraction = std::clamp(
                    (midpoint_double - edge_start).dot(edge) /
                            edge_length_squared,
                    0.0,
                    1.0);
            const double distance = cv::norm(
                    midpoint_double - (edge_start + edge * fraction));
            if (distance < nearest_edge_distance) {
                nearest_edge_distance = distance;
                nearest_edge_angle = LineAngle(cv::Vec4i(
                        static_cast<int>(std::round(edge_start.x)),
                        static_cast<int>(std::round(edge_start.y)),
                        static_cast<int>(std::round(edge_end.x)),
                        static_cast<int>(std::round(edge_end.y))));
            }
        }
        const double parallel_outer_band = std::clamp(
                0.18 / resolution,
                3.0,
                8.0);
        if (nearest_edge_distance <= parallel_outer_band &&
            AngleDistance(LineAngle(segment), nearest_edge_angle) <= 15.0) {
            continue;
        }

        int inside_samples = 0;
        constexpr int kSampleCount = 7;
        for (int sample = 0; sample < kSampleCount; ++sample) {
            const double fraction =
                    sample / static_cast<double>(kSampleCount - 1);
            const cv::Point2f point(
                    static_cast<float>(
                            segment[0] + fraction * (segment[2] - segment[0])),
                    static_cast<float>(
                            segment[1] + fraction * (segment[3] - segment[1])));
            if (cv::pointPolygonTest(outer_polygon, point, true) >=
                -endpoint_tolerance) {
                ++inside_samples;
            }
        }
        if (inside_samples < kSampleCount - 1) continue;

        const double support =
                SegmentSupportRatio(supported_wall, segment, support_thickness);
        if (support < 0.50) continue;

        const double axis_distance = std::min(
                AngleDistance(LineAngle(segment), primary_axis_degrees),
                AngleDistance(LineAngle(segment), primary_axis_degrees + 90.0));
        const double strong_diagonal_length =
                std::clamp(2.0 / resolution, 28.0, 180.0);
        if (axis_distance > 15.0 &&
            (length < strong_diagonal_length || support < 0.75)) {
            continue;
        }

        // A reliable internal partition has explored free space on both
        // sides. Exterior walls and most furniture/obstacle outlines have
        // free space on only one side, even when Hough finds a long stroke.
        // Apply this semantic test only on Android's full export; legacy
        // desktop callers without a semantic raster keep the old behaviour.
        bool candidate_two_sided_free = semantic_free.empty();
        if (!semantic_free.empty()) {
            const cv::Point2d segment_direction =
                    DirectionFromAngle(LineAngle(segment));
            const cv::Point2d side_normal(
                    -segment_direction.y, segment_direction.x);
            const double side_distance = std::clamp(
                    0.30 / resolution, 4.0, 14.0);
            int positive_free = 0;
            int negative_free = 0;
            constexpr int kSideSamples = 11;
            for (int sample = 1; sample < kSideSamples - 1; ++sample) {
                const double fraction =
                        sample / static_cast<double>(kSideSamples - 1);
                const cv::Point2d center(
                        segment[0] + fraction * (segment[2] - segment[0]),
                        segment[1] + fraction * (segment[3] - segment[1]));
                auto is_free = [&](const cv::Point2d& point) {
                    const int x = static_cast<int>(std::round(point.x));
                    const int y = static_cast<int>(std::round(point.y));
                    return x >= 0 && x < semantic_free.cols &&
                            y >= 0 && y < semantic_free.rows &&
                            semantic_free.at<uchar>(y, x) != 0;
                };
                if (is_free(center + side_normal * side_distance)) {
                    ++positive_free;
                }
                if (is_free(center - side_normal * side_distance)) {
                    ++negative_free;
                }
            }
            const double positive_ratio = positive_free / 9.0;
            const double negative_ratio = negative_free / 9.0;
            candidate_two_sided_free =
                    std::min(positive_ratio, negative_ratio) >= 0.18 &&
                    std::max(positive_ratio, negative_ratio) >= 0.50;
            // Semantic free-space is an auxiliary rejection signal, not an
            // absolute requirement. A dense, straight wall may have unknown
            // cells on one side because the scan never entered every room.
            const double long_partition_length = std::clamp(
                    1.80 / resolution,
                    28.0,
                    180.0);
            if (!candidate_two_sided_free && support < 0.68 &&
                length < long_partition_length) {
                continue;
            }
        }

        // A true exterior wall is an extreme line in its normal direction,
        // even when the fitted green envelope sits well outside its centreline.
        // This removes exterior walls without using an ever-growing polygon
        // clearance that would also delete nearby real partitions.
        const cv::Point2d direction =
                DirectionFromAngle(LineAngle(segment));
        const cv::Point2d normal(-direction.y, direction.x);
        std::vector<double> wall_offsets;
        wall_offsets.reserve(wall_points.size());
        for (const auto& point : wall_points) {
            wall_offsets.push_back(point.dot(normal));
        }
        if (wall_offsets.size() >= 20) {
            std::sort(wall_offsets.begin(), wall_offsets.end());
            // Ignore the outermost 2% isolated returns. A fixed small trim is
            // insufficient on dense maps and lets a cloud of remote speckles
            // hide the real exterior wall extreme.
            const size_t trim = std::min(
                    wall_offsets.size() / 50,
                    (wall_offsets.size() - 1) / 4);
            const double minimum_offset = wall_offsets[trim];
            const double maximum_offset =
                    wall_offsets[wall_offsets.size() - 1 - trim];
            const double segment_offset =
                    0.5 * (cv::Point2d(segment[0], segment[1]).dot(normal) +
                           cv::Point2d(segment[2], segment[3]).dot(normal));
            const double extreme_band = std::max(
                    0.25 / resolution,
                    (maximum_offset - minimum_offset) * 0.025);
            const bool is_offset_extreme =
                    segment_offset - minimum_offset <= extreme_band ||
                    maximum_offset - segment_offset <= extreme_band;
            // An offset extreme is only sufficient evidence for an exterior
            // wall when it is also close to the fitted envelope and lacks
            // explored space on both sides. Concave/L-shaped plans and missing
            // facade returns can make a genuine central partition globally
            // extreme in one direction.
            const double trusted_exterior_depth = std::clamp(
                    0.75 / resolution,
                    8.0,
                    24.0);
            if (is_offset_extreme &&
                midpoint_distance <= trusted_exterior_depth &&
                !candidate_two_sided_free) {
                continue;
            }
        }
        const cv::Vec4i straightened =
                straighten_to_building_axis(segment);
        accepted.push_back(straightened);
        cv::line(candidate_debug,
                 {straightened[0], straightened[1]},
                 {straightened[2], straightened[3]},
                 cv::Scalar(0, 165, 255),
                 1,
                 cv::LINE_AA);
    }
    cv::imwrite(
            PathJoin(debug_dir, "internal_wall_candidates.png"),
            candidate_debug);

    std::sort(
            accepted.begin(),
            accepted.end(),
            [](const cv::Vec4i& first, const cv::Vec4i& second) {
                return SegmentLength(first) > SegmentLength(second);
            });
    std::vector<cv::Vec4i> deduplicated;
    int centered_parallel_segments = 0;
    const double parallel_wall_band =
            std::clamp(0.35 / resolution, 4.0, 18.0);
    for (const auto& candidate : accepted) {
        bool duplicate = false;
        for (auto& retained : deduplicated) {
            if (AngleDistance(
                        LineAngle(candidate),
                        LineAngle(retained)) > 8.0) {
                continue;
            }
            // A wall with measurable thickness commonly produces one Hough
            // line on each face.  Treat strongly overlapping parallel lines
            // as one wall and move the retained line to the middle of both
            // faces instead of arbitrarily keeping either edge.
            const cv::Point2d direction =
                    DirectionFromAngle(LineAngle(retained));
            const cv::Point2d normal(-direction.y, direction.x);
            const double candidate_offset =
                    0.5 * (cv::Point2d(candidate[0], candidate[1]).dot(normal) +
                           cv::Point2d(candidate[2], candidate[3]).dot(normal));
            const double retained_offset =
                    0.5 * (cv::Point2d(retained[0], retained[1]).dot(normal) +
                           cv::Point2d(retained[2], retained[3]).dot(normal));
            if (std::fabs(candidate_offset - retained_offset) >
                parallel_wall_band) {
                continue;
            }
            if (ProjectionOverlapRatio(candidate, retained) >= 0.52) {
                auto projected_span = [&](const cv::Vec4i& segment) {
                    std::array<double, 2> values{
                            cv::Point2d(segment[0], segment[1]).dot(direction),
                            cv::Point2d(segment[2], segment[3]).dot(direction)};
                    std::sort(values.begin(), values.end());
                    return values;
                };
                const auto candidate_span = projected_span(candidate);
                const auto retained_span = projected_span(retained);
                // Equal weighting deliberately locates the result halfway
                // between the two observed wall faces.
                retained = BuildSegment(
                        std::min(candidate_span[0], retained_span[0]),
                        std::max(candidate_span[1], retained_span[1]),
                        0.5 * (candidate_offset + retained_offset),
                        direction);
                duplicate = true;
                ++centered_parallel_segments;
                break;
            }
        }
        if (!duplicate) deduplicated.push_back(candidate);
    }

    // A doorway or a short unobserved strip commonly splits one partition
    // into two collinear strokes. Connectivity filtering below prevents rows
    // of furniture from becoming walls, so allow wall-sized gaps here.
    const double maximum_internal_merge_gap =
            std::clamp(0.90 / resolution, 7.0, 40.0);
    const double maximum_internal_offset =
            std::clamp(0.28 / resolution, 4.0, 15.0);
    const double unconditional_short_gap =
            std::clamp(0.22 / resolution, 2.0, 12.0);
    int bridged_internal_gaps = 0;
    bool merged_internal_segment = true;
    while (merged_internal_segment) {
        merged_internal_segment = false;
        for (size_t first_index = 0;
             first_index < deduplicated.size() && !merged_internal_segment;
             ++first_index) {
            for (size_t second_index = first_index + 1;
                 second_index < deduplicated.size();
                 ++second_index) {
                const cv::Vec4i first = deduplicated[first_index];
                const cv::Vec4i second = deduplicated[second_index];
                if (AngleDistance(LineAngle(first), LineAngle(second)) > 8.0) {
                    continue;
                }
                const cv::Point2d direction =
                        DirectionFromAngle(LineAngle(first));
                const cv::Point2d normal(-direction.y, direction.x);
                const double first_offset =
                        0.5 * (cv::Point2d(first[0], first[1]).dot(normal) +
                               cv::Point2d(first[2], first[3]).dot(normal));
                const double second_offset =
                        0.5 * (cv::Point2d(second[0], second[1]).dot(normal) +
                               cv::Point2d(second[2], second[3]).dot(normal));
                if (std::fabs(first_offset - second_offset) >
                    maximum_internal_offset) {
                    continue;
                }
                auto span = [&](const cv::Vec4i& segment) {
                    std::array<double, 2> values{
                            cv::Point2d(segment[0], segment[1]).dot(direction),
                            cv::Point2d(segment[2], segment[3]).dot(direction)};
                    std::sort(values.begin(), values.end());
                    return values;
                };
                const auto first_span = span(first);
                const auto second_span = span(second);
                const double gap = std::max(
                        0.0,
                        std::max(first_span[0], second_span[0]) -
                                std::min(first_span[1], second_span[1]));
                if (gap > maximum_internal_merge_gap) continue;
                const double merged_start =
                        std::min(first_span[0], second_span[0]);
                const double merged_end =
                        std::max(first_span[1], second_span[1]);
                const double merged_offset =
                        (first_offset * SegmentLength(first) +
                         second_offset * SegmentLength(second)) /
                        std::max(1.0,
                                 SegmentLength(first) + SegmentLength(second));
                const cv::Vec4i merged = BuildSegment(
                        merged_start,
                        merged_end,
                        merged_offset,
                        direction);
                if (gap > unconditional_short_gap) {
                    double gap_start = 0.0;
                    double gap_end = 0.0;
                    if (first_span[1] < second_span[0]) {
                        gap_start = first_span[1];
                        gap_end = second_span[0];
                    } else if (second_span[1] < first_span[0]) {
                        gap_start = second_span[1];
                        gap_end = first_span[0];
                    }
                    if (gap_end > gap_start) {
                        const cv::Vec4i gap_segment = BuildSegment(
                                gap_start,
                                gap_end,
                                merged_offset,
                                direction);
                        // Inspect only the missing interval.  A small amount
                        // of stable-wall evidence is sufficient for a scan
                        // dropout, while an unsupported doorway remains open.
                        const double gap_support = SegmentSupportRatio(
                                supported_wall,
                                gap_segment,
                                std::max(support_thickness,
                                         static_cast<int>(std::ceil(
                                                 0.12 / resolution))));
                        if (gap_support < 0.15) continue;
                    }
                }
                if (gap > 1.0) ++bridged_internal_gaps;
                deduplicated[first_index] = merged;
                deduplicated.erase(
                        deduplicated.begin() +
                        static_cast<long>(second_index));
                merged_internal_segment = true;
                break;
            }
        }
    }
    std::cout << "[floorplan] internal wall centerline merges="
              << centered_parallel_segments
              << " bridged gaps=" << bridged_internal_gaps << std::endl;

    // Join T/cross junctions after collinear consolidation. Hough frequently
    // stops a horizontal partition a few pixels before the centreline of a
    // vertical wall, leaving visually obvious floating red strokes.
    const double maximum_junction_extension =
            std::clamp(0.48 / resolution, 5.0, 22.0);
    const double unconditional_junction_extension =
            std::clamp(0.18 / resolution, 2.0, 9.0);
    int connected_internal_junctions = 0;
    for (size_t first_index = 0; first_index < deduplicated.size(); ++first_index) {
        for (size_t second_index = first_index + 1;
             second_index < deduplicated.size(); ++second_index) {
            cv::Vec4i& first = deduplicated[first_index];
            cv::Vec4i& second = deduplicated[second_index];
            if (AngleDistance(LineAngle(first), LineAngle(second)) < 55.0) {
                continue;
            }
            bool intersection_ok = false;
            const cv::Point2d intersection =
                    LineIntersection(first, second, &intersection_ok);
            if (!intersection_ok || !std::isfinite(intersection.x) ||
                !std::isfinite(intersection.y) ||
                cv::pointPolygonTest(
                        outer_polygon,
                        cv::Point2f(
                                static_cast<float>(intersection.x),
                                static_cast<float>(intersection.y)),
                        true) < -endpoint_tolerance) {
                continue;
            }
            struct JunctionRelation {
                bool contains = false;
                int endpoint = -1;
                double distance = std::numeric_limits<double>::infinity();
            };
            auto relation = [&](const cv::Vec4i& segment) {
                JunctionRelation value;
                const cv::Point2d start(segment[0], segment[1]);
                const cv::Point2d end(segment[2], segment[3]);
                const cv::Point2d delta = end - start;
                const double length_squared = delta.dot(delta);
                if (length_squared <= 1e-6) return value;
                const double fraction =
                        (intersection - start).dot(delta) / length_squared;
                value.contains = fraction >= -0.02 && fraction <= 1.02;
                const double start_distance = cv::norm(intersection - start);
                const double end_distance = cv::norm(intersection - end);
                value.endpoint = start_distance <= end_distance ? 0 : 1;
                value.distance = std::min(start_distance, end_distance);
                return value;
            };
            auto first_relation = relation(first);
            auto second_relation = relation(second);
            if (!(first_relation.contains ||
                  first_relation.distance <= maximum_junction_extension) ||
                !(second_relation.contains ||
                  second_relation.distance <= maximum_junction_extension)) {
                continue;
            }
            auto extend = [&](cv::Vec4i* segment,
                              const JunctionRelation& value) {
                if (value.contains || value.endpoint < 0 ||
                    value.distance > maximum_junction_extension) return false;
                const cv::Point endpoint = value.endpoint == 0
                        ? cv::Point((*segment)[0], (*segment)[1])
                        : cv::Point((*segment)[2], (*segment)[3]);
                const cv::Vec4i extension(
                        endpoint.x,
                        endpoint.y,
                        static_cast<int>(std::round(intersection.x)),
                        static_cast<int>(std::round(intersection.y)));
                if (value.distance > unconditional_junction_extension &&
                    SegmentSupportRatio(
                            supported_wall,
                            extension,
                            support_thickness) < 0.10) {
                    return false;
                }
                if (value.endpoint == 0) {
                    (*segment)[0] = extension[2];
                    (*segment)[1] = extension[3];
                } else {
                    (*segment)[2] = extension[2];
                    (*segment)[3] = extension[3];
                }
                return true;
            };
            const bool extended_first = extend(&first, first_relation);
            const bool extended_second = extend(&second, second_relation);
            if (extended_first || extended_second) {
                ++connected_internal_junctions;
            }
        }
    }

    // Merging can leave tiny residual fragments when a noisy wall is split
    // into several branches. Enforce the structural threshold once more on
    // the actual lines that will be rendered.
    deduplicated.erase(
            std::remove_if(
                    deduplicated.begin(),
                    deduplicated.end(),
                    [&](const cv::Vec4i& segment) {
                        return SegmentLength(segment) < minimum_length;
                    }),
            deduplicated.end());

    const double maximum_outline_extension =
            std::clamp(0.80 / resolution, 6.0, 36.0);
    const double unconditional_short_extension =
            std::clamp(0.45 / resolution, 5.0, 18.0);
    int extended_endpoint_count = 0;
    auto extend_endpoint_to_outline = [&](cv::Vec4i* segment,
                                          bool extend_first) {
        const cv::Point2d endpoint(
                extend_first ? (*segment)[0] : (*segment)[2],
                extend_first ? (*segment)[1] : (*segment)[3]);
        const cv::Point2d other(
                extend_first ? (*segment)[2] : (*segment)[0],
                extend_first ? (*segment)[3] : (*segment)[1]);
        cv::Point2d ray = endpoint - other;
        const double ray_norm = cv::norm(ray);
        if (ray_norm < minimum_length) return;
        ray *= 1.0 / ray_norm;

        double best_distance = std::numeric_limits<double>::infinity();
        cv::Point2d best_intersection;
        for (size_t index = 0; index < outer_polygon.size(); ++index) {
            const cv::Point& polygon_start = outer_polygon[index];
            const cv::Point& polygon_end =
                    outer_polygon[(index + 1) % outer_polygon.size()];
            const cv::Point2d edge_start(polygon_start.x, polygon_start.y);
            const cv::Point2d edge_end(polygon_end.x, polygon_end.y);
            const cv::Point2d edge = edge_end - edge_start;
            const double cross = ray.x * edge.y - ray.y * edge.x;
            if (std::fabs(cross) < 1e-6) continue;
            const cv::Point2d delta = edge_start - endpoint;
            const double ray_distance =
                    (delta.x * edge.y - delta.y * edge.x) / cross;
            const double edge_fraction =
                    (delta.x * ray.y - delta.y * ray.x) / cross;
            if (ray_distance <= 0.5 ||
                ray_distance > maximum_outline_extension ||
                edge_fraction < -0.02 || edge_fraction > 1.02) {
                continue;
            }
            const cv::Vec4i polygon_edge(
                    polygon_start.x,
                    polygon_start.y,
                    polygon_end.x,
                    polygon_end.y);
            if (AngleDistance(
                        LineAngle(*segment),
                        LineAngle(polygon_edge)) < 28.0) {
                continue;
            }
            if (ray_distance < best_distance) {
                best_distance = ray_distance;
                best_intersection = endpoint + ray * ray_distance;
            }
        }
        if (!std::isfinite(best_distance)) return;
        const cv::Vec4i extension(
                static_cast<int>(std::round(endpoint.x)),
                static_cast<int>(std::round(endpoint.y)),
                static_cast<int>(std::round(best_intersection.x)),
                static_cast<int>(std::round(best_intersection.y)));
        const double extension_support =
                SegmentSupportRatio(
                        supported_wall,
                        extension,
                        support_thickness);
        if (best_distance > unconditional_short_extension &&
            extension_support < 0.28) {
            return;
        }
        if (extend_first) {
            (*segment)[0] = extension[2];
            (*segment)[1] = extension[3];
        } else {
            (*segment)[2] = extension[2];
            (*segment)[3] = extension[3];
        }
        ++extended_endpoint_count;
    };
    for (auto& segment : deduplicated) {
        extend_endpoint_to_outline(&segment, true);
        extend_endpoint_to_outline(&segment, false);
    }

    // Connectivity is useful evidence, but must not be mandatory: short
    // obstacle walls and cubicle partitions can be disconnected by doorways
    // at both ends.  Keep connected lines readily and retain floating lines
    // when the source raster itself provides dense support.  This makes the
    // fitted plan describe the obstacle geometry actually present in the map
    // instead of only the globally connected room-wall network.
    const double outline_anchor_distance =
            std::clamp(0.24 / resolution, 3.0, 12.0);
    const double junction_anchor_distance =
            std::clamp(0.24 / resolution, 3.0, 12.0);
    const double single_anchor_minimum_length =
            std::clamp(0.45 / resolution, minimum_length, 80.0);
    const double floating_wall_minimum_length =
            std::clamp(1.20 / resolution, minimum_length, 140.0);
    auto point_to_segment_distance = [](const cv::Point2d& point,
                                        const cv::Vec4i& segment) {
        const cv::Point2d start(segment[0], segment[1]);
        const cv::Point2d end(segment[2], segment[3]);
        const cv::Point2d delta = end - start;
        const double length_squared = delta.dot(delta);
        if (length_squared <= 1e-9) return cv::norm(point - start);
        const double fraction = std::clamp(
                (point - start).dot(delta) / length_squared, 0.0, 1.0);
        return cv::norm(point - (start + delta * fraction));
    };
    std::vector<int> connection_counts(deduplicated.size(), 0);
    for (size_t index = 0; index < deduplicated.size(); ++index) {
        const cv::Vec4i& segment = deduplicated[index];
        for (const cv::Point2f& endpoint : {
                     cv::Point2f(segment[0], segment[1]),
                     cv::Point2f(segment[2], segment[3])}) {
            const double signed_distance =
                    cv::pointPolygonTest(outer_polygon, endpoint, true);
            if (std::fabs(signed_distance) <= outline_anchor_distance) {
                ++connection_counts[index];
            }
        }
    }
    for (size_t first_index = 0; first_index < deduplicated.size(); ++first_index) {
        for (size_t second_index = first_index + 1;
             second_index < deduplicated.size(); ++second_index) {
            const cv::Vec4i& first = deduplicated[first_index];
            const cv::Vec4i& second = deduplicated[second_index];
            if (AngleDistance(LineAngle(first), LineAngle(second)) < 35.0) continue;
            bool intersection_ok = false;
            const cv::Point2d intersection =
                    LineIntersection(first, second, &intersection_ok);
            if (!intersection_ok || !std::isfinite(intersection.x) ||
                !std::isfinite(intersection.y)) {
                continue;
            }
            if (point_to_segment_distance(intersection, first) <=
                    junction_anchor_distance &&
                point_to_segment_distance(intersection, second) <=
                    junction_anchor_distance) {
                ++connection_counts[first_index];
                ++connection_counts[second_index];
            }
        }
    }
    int rejected_floating_segments = 0;
    int rejected_trajectory_crossings = 0;
    std::vector<cv::Vec4i> structural_segments;
    structural_segments.reserve(deduplicated.size());
    for (size_t index = 0; index < deduplicated.size(); ++index) {
        const cv::Vec4i& segment = deduplicated[index];
        const double length = SegmentLength(segment);
        const int connections = connection_counts[index];
        const double support = SegmentSupportRatio(
                supported_wall, segment, support_thickness);
        bool has_two_sided_free_space = semantic_free.empty();
        bool has_one_sided_free_space = semantic_free.empty();
        double positive_free_ratio = semantic_free.empty() ? 1.0 : 0.0;
        double negative_free_ratio = semantic_free.empty() ? 1.0 : 0.0;
        if (!semantic_free.empty()) {
            const cv::Point2d direction =
                    DirectionFromAngle(LineAngle(segment));
            const cv::Point2d normal(-direction.y, direction.x);
            const double side_distance = std::clamp(
                    0.30 / resolution, 4.0, 14.0);
            int positive_free = 0;
            int negative_free = 0;
            constexpr int kStructuralSideSamples = 13;
            for (int sample = 1; sample < kStructuralSideSamples - 1; ++sample) {
                const double fraction = sample /
                        static_cast<double>(kStructuralSideSamples - 1);
                const cv::Point2d center(
                        segment[0] + fraction * (segment[2] - segment[0]),
                        segment[1] + fraction * (segment[3] - segment[1]));
                auto is_free = [&](const cv::Point2d& point) {
                    const int x = static_cast<int>(std::round(point.x));
                    const int y = static_cast<int>(std::round(point.y));
                    return x >= 0 && x < semantic_free.cols &&
                            y >= 0 && y < semantic_free.rows &&
                            semantic_free.at<uchar>(y, x) != 0;
                };
                if (is_free(center + normal * side_distance)) ++positive_free;
                if (is_free(center - normal * side_distance)) ++negative_free;
            }
            const double sample_count = kStructuralSideSamples - 2.0;
            positive_free_ratio = positive_free / sample_count;
            negative_free_ratio = negative_free / sample_count;
            has_two_sided_free_space =
                    positive_free_ratio >= 0.45 &&
                    negative_free_ratio >= 0.45;
            has_one_sided_free_space =
                    std::max(positive_free_ratio, negative_free_ratio) >= 0.55;
        }
        // Two connections define an unambiguous part of the wall network and
        // one connection is sufficient for a normal branch wall.  A floating
        // obstacle segment is also valid when its own occupied-pixel support
        // is strong; two-sided explored free space lowers that support bar.
        bool trajectory_crosses_wall = false;
        if (trajectory_points_px.size() >= 2) {
            const cv::Point2d wall_start(segment[0], segment[1]);
            const cv::Point2d wall_end(segment[2], segment[3]);
            const cv::Point2d wall_delta = wall_end - wall_start;
            const double wall_length_squared = wall_delta.dot(wall_delta);
            for (size_t trajectory_index = 1;
                 trajectory_index < trajectory_points_px.size() &&
                 !trajectory_crosses_wall;
                 ++trajectory_index) {
                const cv::Point2d path_start(
                        trajectory_points_px[trajectory_index - 1]);
                const cv::Point2d path_end(
                        trajectory_points_px[trajectory_index]);
                const cv::Point2d path_delta = path_end - path_start;
                // Ignore pose discontinuities caused by separate sessions or
                // relocalization; they are not a driven path through a wall.
                if (cv::norm(path_delta) > 1.20 / resolution) continue;
                const double denominator =
                        wall_delta.x * path_delta.y -
                        wall_delta.y * path_delta.x;
                if (std::fabs(denominator) < 1e-6 ||
                    wall_length_squared <= 1e-6) {
                    continue;
                }
                const cv::Point2d between = path_start - wall_start;
                const double wall_fraction =
                        (between.x * path_delta.y -
                         between.y * path_delta.x) / denominator;
                const double path_fraction =
                        (between.x * wall_delta.y -
                         between.y * wall_delta.x) / denominator;
                if (wall_fraction > 0.04 && wall_fraction < 0.96 &&
                    path_fraction >= 0.0 && path_fraction <= 1.0) {
                    // A geometric intersection alone is not a reason to erase
                    // a partition: normal survey routes cross its doorway and
                    // the fitted wall centreline spans that opening. Only veto
                    // a weak floating candidate when the semantic raster and
                    // local occupied evidence both confirm an open corridor.
                    if (semantic_free.empty()) {
                        trajectory_crosses_wall = true;
                        continue;
                    }
                    const double path_length = cv::norm(path_delta);
                    if (path_length <= 1e-6) continue;
                    const cv::Point2d intersection =
                            wall_start + wall_delta * wall_fraction;
                    const cv::Point2d path_direction =
                            path_delta * (1.0 / path_length);
                    auto semantic_free_at = [&](const cv::Point2d& point) {
                        const int x = static_cast<int>(std::round(point.x));
                        const int y = static_cast<int>(std::round(point.y));
                        return x >= 0 && x < semantic_free.cols &&
                                y >= 0 && y < semantic_free.rows &&
                                semantic_free.at<uchar>(y, x) != 0;
                    };
                    const double corridor_probe = std::clamp(
                            0.12 / resolution, 2.0, 6.0);
                    const bool open_corridor =
                            semantic_free_at(intersection) &&
                            semantic_free_at(
                                    intersection + path_direction * corridor_probe) &&
                            semantic_free_at(
                                    intersection - path_direction * corridor_probe);
                    const double wall_length = std::sqrt(wall_length_squared);
                    const cv::Point2d wall_direction =
                            wall_delta * (1.0 / wall_length);
                    const double local_half_length = std::clamp(
                            0.22 / resolution, 3.0, 9.0);
                    const cv::Point2d local_start =
                            intersection - wall_direction * local_half_length;
                    const cv::Point2d local_end =
                            intersection + wall_direction * local_half_length;
                    const cv::Vec4i local_segment(
                            static_cast<int>(std::round(local_start.x)),
                            static_cast<int>(std::round(local_start.y)),
                            static_cast<int>(std::round(local_end.x)),
                            static_cast<int>(std::round(local_end.y)));
                    const double local_support = SegmentSupportRatio(
                            observed,
                            local_segment,
                            std::max(1, support_thickness / 2));
                    const bool weak_floating_candidate =
                            connections == 0 && support < 0.68;
                    trajectory_crosses_wall =
                            open_corridor && local_support < 0.24 &&
                            weak_floating_candidate;
                }
            }
        }
        const bool connected_wall = !trajectory_crosses_wall &&
                (connections >= 2 ||
                 (connections == 1 &&
                  length >= single_anchor_minimum_length &&
                  (has_two_sided_free_space || support >= 0.62)));
        const bool strong_floating_partition = connections == 0 &&
                !trajectory_crosses_wall &&
                length >= floating_wall_minimum_length &&
                support >= 0.68 &&
                (has_two_sided_free_space ||
                 (has_one_sided_free_space &&
                  length >= std::clamp(1.50 / resolution, 24.0, 150.0) &&
                  support >= 0.82));
        const double dominant_axis_distance = std::min(
                AngleDistance(LineAngle(segment), primary_axis_degrees),
                AngleDistance(LineAngle(segment), primary_axis_degrees + 90.0));
        const cv::Point2f segment_midpoint(
                0.5f * (segment[0] + segment[2]),
                0.5f * (segment[1] + segment[3]));
        const double interior_depth = cv::pointPolygonTest(
                outer_polygon,
                segment_midpoint,
                true);
        // A long, deep, dominant-axis wall is a main room divider. Door gaps,
        // incomplete scans or an unentered room may keep it from satisfying
        // endpoint/two-sided tests, but those weaknesses must not erase an
        // otherwise obvious central partition.
        const bool strong_long_partition = !trajectory_crosses_wall &&
                length >= std::clamp(1.80 / resolution, 28.0, 180.0) &&
                support >= 0.48 &&
                interior_depth >= std::clamp(0.55 / resolution, 6.0, 18.0) &&
                (dominant_axis_distance <= 15.0 || support >= 0.78);
        if (!connected_wall && !strong_floating_partition &&
            !strong_long_partition) {
            if (trajectory_crosses_wall) {
                ++rejected_trajectory_crossings;
            }
            ++rejected_floating_segments;
            continue;
        }
        structural_segments.push_back(segment);
    }
    deduplicated = std::move(structural_segments);

    cv::Mat final_debug;
    cv::cvtColor(observed, final_debug, cv::COLOR_GRAY2BGR);
    for (const auto& segment : deduplicated) {
        cv::line(final_debug,
                 {segment[0], segment[1]},
                 {segment[2], segment[3]},
                 cv::Scalar(0, 0, 255),
                 2,
                 cv::LINE_AA);
    }
    cv::polylines(
            final_debug,
            std::vector<std::vector<cv::Point>>{outer_polygon},
            true,
            cv::Scalar(0, 255, 0),
            2,
            cv::LINE_AA);
    cv::imwrite(
            PathJoin(debug_dir, "internal_wall_final.png"),
            final_debug);
    std::cout << "[INFO] 内部红线检测 candidates=" << candidates.size()
              << " topology=" << topology_candidates.size()
              << " supported=" << accepted.size()
              << " final=" << deduplicated.size()
              << " junctions=" << connected_internal_junctions
              << " outline_extensions=" << extended_endpoint_count
              << " rejected_floating=" << rejected_floating_segments
              << " rejected_trajectory=" << rejected_trajectory_crossings
              << " min_length_px=" << minimum_length
              << " boundary_clearance_px=" << boundary_clearance
              << "\n";
    return deduplicated;
}

// Recovers a common architectural case that the normal room-separator test
// intentionally treats conservatively: a short partition with one end fixed
// to the exterior wall and the other end open inside the room.  Such a wall
// may have unknown cells on one side and only one graph connection, so it is
// neither a two-sided separator nor a floating long wall.  Requiring a near-
// perpendicular exterior junction, inward depth and direct occupied support
// keeps this rescue distinct from furniture and facade-parallel clutter.
std::vector<cv::Vec4i> DetectOutlineAnchoredPartitionStubs(
        const cv::Mat& stable_wall_mask,
        const cv::Mat& observed_wall_mask,
        const cv::Mat& semantic_bgr,
        const std::vector<cv::Point>& outer_polygon,
        double meters_per_pixel,
        const std::string& debug_dir) {
    std::vector<cv::Vec4i> rescued;
    if (stable_wall_mask.empty() || outer_polygon.size() < 4) return rescued;
    const double resolution =
            std::isfinite(meters_per_pixel) && meters_per_pixel > 1e-4
                    ? meters_per_pixel
                    : 0.05;
    const cv::Mat stable = Binary255(stable_wall_mask);
    const cv::Mat observed =
            !observed_wall_mask.empty() &&
                    observed_wall_mask.size() == stable.size()
            ? Binary255(observed_wall_mask)
            : stable;
    const double minimum_length = std::clamp(
            0.50 / resolution, 8.0, 40.0);
    const double maximum_gap = std::clamp(
            0.18 / resolution, 2.0, 8.0);
    std::vector<cv::Vec4i> candidates;
    cv::HoughLinesP(
            stable,
            candidates,
            1.0,
            kPi / 360.0,
            std::max(5, static_cast<int>(std::round(minimum_length * 0.45))),
            minimum_length,
            maximum_gap);
    const std::vector<cv::Vec4i> traced = TraceSkeletonSegments(
            stable,
            minimum_length,
            std::clamp(0.08 / resolution, 1.2, 4.0));
    candidates.insert(candidates.end(), traced.begin(), traced.end());

    cv::Mat semantic_free;
    if (!semantic_bgr.empty() && semantic_bgr.size() == stable.size()) {
        cv::inRange(
                semantic_bgr,
                cv::Scalar(245, 245, 245),
                cv::Scalar(255, 255, 255),
                semantic_free);
        cv::dilate(
                semantic_free,
                semantic_free,
                cv::getStructuringElement(
                        cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    }
    const double anchor_tolerance = std::clamp(
            0.85 / resolution, 8.0, 22.0);
    const double minimum_inward_depth = std::clamp(
            0.42 / resolution, 6.0, 18.0);
    const int support_thickness = std::clamp(
            static_cast<int>(std::round(0.10 / resolution)), 2, 5);
    const double side_probe = std::clamp(
            0.22 / resolution, 3.0, 10.0);
    int rejected_depth = 0;
    int rejected_anchor = 0;
    int rejected_angle = 0;
    int rejected_support = 0;
    int rejected_semantic = 0;
    cv::Mat raw_candidate_debug;
    cv::cvtColor(observed, raw_candidate_debug, cv::COLOR_GRAY2BGR);

    auto nearest_outline_edge = [&](const cv::Point2d& point,
                                    size_t* best_index,
                                    cv::Point2d* closest) {
        double best_distance = std::numeric_limits<double>::infinity();
        for (size_t index = 0; index < outer_polygon.size(); ++index) {
            const cv::Point2d start(outer_polygon[index]);
            const cv::Point2d end(
                    outer_polygon[(index + 1) % outer_polygon.size()]);
            const cv::Point2d delta = end - start;
            const double squared = delta.dot(delta);
            if (squared <= 1e-9) continue;
            const double fraction = std::clamp(
                    (point - start).dot(delta) / squared, 0.0, 1.0);
            const cv::Point2d projected = start + delta * fraction;
            const double distance = cv::norm(point - projected);
            if (distance < best_distance) {
                best_distance = distance;
                *best_index = index;
                *closest = projected;
            }
        }
        return best_distance;
    };
    auto free_at = [&](const cv::Point2d& point) {
        if (semantic_free.empty()) return true;
        const int x = static_cast<int>(std::round(point.x));
        const int y = static_cast<int>(std::round(point.y));
        return x >= 0 && x < semantic_free.cols &&
                y >= 0 && y < semantic_free.rows &&
                semantic_free.at<uchar>(y, x) != 0;
    };

    for (const cv::Vec4i& raw : candidates) {
        const double raw_length = SegmentLength(raw);
        if (raw_length < minimum_length) continue;
        cv::line(raw_candidate_debug,
                 cv::Point(raw[0], raw[1]),
                 cv::Point(raw[2], raw[3]),
                 cv::Scalar(0, 165, 255), 1, cv::LINE_AA);
        const cv::Point2d endpoints[2] = {
                cv::Point2d(raw[0], raw[1]),
                cv::Point2d(raw[2], raw[3])};
        const double signed_depth[2] = {
                cv::pointPolygonTest(
                        outer_polygon,
                        cv::Point2f(endpoints[0]),
                        true),
                cv::pointPolygonTest(
                        outer_polygon,
                        cv::Point2f(endpoints[1]),
                        true)};
        const int anchor_endpoint =
                std::fabs(signed_depth[0]) <= std::fabs(signed_depth[1])
                ? 0 : 1;
        const int inner_endpoint = 1 - anchor_endpoint;
        if (std::fabs(signed_depth[anchor_endpoint]) > anchor_tolerance ||
            signed_depth[inner_endpoint] < minimum_inward_depth) {
            ++rejected_depth;
            continue;
        }
        size_t edge_index = 0;
        cv::Point2d anchor_on_outline;
        if (nearest_outline_edge(
                    endpoints[anchor_endpoint],
                    &edge_index,
                    &anchor_on_outline) > anchor_tolerance) {
            ++rejected_anchor;
            continue;
        }
        const cv::Point2d edge_start(outer_polygon[edge_index]);
        const cv::Point2d edge_end(
                outer_polygon[(edge_index + 1) % outer_polygon.size()]);
        cv::Point2d edge_direction = edge_end - edge_start;
        const double edge_length = cv::norm(edge_direction);
        if (edge_length <= 1e-6) continue;
        edge_direction *= 1.0 / edge_length;
        cv::Point2d inward_normal(-edge_direction.y, edge_direction.x);
        if ((endpoints[inner_endpoint] - anchor_on_outline).dot(
                    inward_normal) < 0.0) {
            inward_normal *= -1.0;
        }
        if (AngleDistance(
                    LineAngle(raw),
                    std::atan2(inward_normal.y, inward_normal.x) *
                            180.0 / kPi) > 18.0) {
            ++rejected_angle;
            continue;
        }
        const double inward_span =
                (endpoints[inner_endpoint] - anchor_on_outline).dot(
                        inward_normal);
        if (inward_span < minimum_length) {
            ++rejected_depth;
            continue;
        }
        const cv::Point2d fitted_inner =
                anchor_on_outline + inward_normal * inward_span;
        const cv::Vec4i fitted(
                static_cast<int>(std::round(anchor_on_outline.x)),
                static_cast<int>(std::round(anchor_on_outline.y)),
                static_cast<int>(std::round(fitted_inner.x)),
                static_cast<int>(std::round(fitted_inner.y)));
        const double raw_support = SegmentSupportRatio(
                observed, raw, support_thickness);
        const double fitted_support = SegmentSupportRatio(
                observed, fitted, support_thickness);
        // The connection to the exterior may be an unobserved doorway-size
        // gap. Judge the physical stub primarily on its own black evidence;
        // retain a smaller support floor on the extended fitted line.
        if (raw_support < 0.30 || fitted_support < 0.14) {
            ++rejected_support;
            continue;
        }

        int positive_free = 0;
        int negative_free = 0;
        constexpr int kSamples = 9;
        const cv::Point2d tangent = inward_normal;
        const cv::Point2d side_normal(-tangent.y, tangent.x);
        for (int sample = 2; sample < kSamples; ++sample) {
            const double fraction = sample /
                    static_cast<double>(kSamples);
            const cv::Point2d center =
                    anchor_on_outline + tangent * inward_span * fraction;
            if (free_at(center + side_normal * side_probe)) ++positive_free;
            if (free_at(center - side_normal * side_probe)) ++negative_free;
        }
        const bool two_sided_room_evidence = semantic_free.empty() ||
                (positive_free >= 2 && negative_free >= 2);
        const bool exceptionally_strong_stub =
                raw_support >= 0.72 && inward_span >= 0.75 / resolution;
        if (!two_sided_room_evidence && !exceptionally_strong_stub) {
            ++rejected_semantic;
            continue;
        }

        bool duplicate = false;
        for (const cv::Vec4i& retained : rescued) {
            if (AngleDistance(LineAngle(fitted), LineAngle(retained)) <= 8.0 &&
                ProjectionOverlapRatio(fitted, retained) >= 0.55) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) rescued.push_back(fitted);
    }

    if (!debug_dir.empty()) {
        cv::imwrite(
                PathJoin(debug_dir, "outline_anchored_stub_candidates.png"),
                raw_candidate_debug);
        cv::Mat debug;
        cv::cvtColor(observed, debug, cv::COLOR_GRAY2BGR);
        cv::polylines(
                debug,
                std::vector<std::vector<cv::Point>>{outer_polygon},
                true,
                cv::Scalar(0, 255, 0),
                1,
                cv::LINE_AA);
        for (const auto& segment : rescued) {
            cv::line(
                    debug,
                    cv::Point(segment[0], segment[1]),
                    cv::Point(segment[2], segment[3]),
                    cv::Scalar(255, 0, 255),
                    2,
                    cv::LINE_AA);
        }
        cv::imwrite(
                PathJoin(debug_dir, "outline_anchored_stubs.png"),
                debug);
    }
    std::cout << "[INFO] 外墙锚定隔墙诊断 raw=" << candidates.size()
              << " accepted=" << rescued.size()
              << " reject_depth=" << rejected_depth
              << " reject_anchor=" << rejected_anchor
              << " reject_angle=" << rejected_angle
              << " reject_support=" << rejected_support
              << " reject_semantic=" << rejected_semantic << "\n";
    return rescued;
}

std::vector<int> DefaultBranchCandidates(
        const cv::Mat& img,
        double meters_per_pixel) {
    const double resolution =
            std::isfinite(meters_per_pixel) && meters_per_pixel > 1e-4
                    ? meters_per_pixel
                    : 0.05;
    const int maximum_branch = std::clamp(
            static_cast<int>(std::round(std::min(img.rows, img.cols) * 0.035)),
            12,
            32);
    std::vector<int> candidates;
    // Search physically meaningful pruning lengths instead of treating 28 px
    // as universal. At 4 cm/px these are roughly 5, 9, 13 and 18 pixels.
    // Keep desktop regression and the Android JNI path on the same physical
    // candidate set.  At the app export resolution (5 cm/px) this is exactly
    // 4/7/10/12; using 0.70 m here silently introduced a branch-14 candidate
    // on device and could make the APK select a different floor plan from the
    // one validated offline on the same pbstream.
    for (const double length_m : {0.20, 0.35, 0.50, 0.60}) {
        candidates.push_back(std::clamp(
                static_cast<int>(std::round(length_m / resolution)),
                4,
                maximum_branch));
    }
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(
            std::unique(candidates.begin(), candidates.end()),
            candidates.end());
    return candidates;
}

double ScorePipelineResult(const PipelineResult& result, int preferred_branch) {
    const double wall_pixels = std::max(1, result.wall_pixel_count);
    const double exterior_coverage = result.green_total_length / wall_pixels;
    const double internal_coverage =
            result.internal_wall_pixel_count / wall_pixels;
    const double total_length =
            std::max(1.0, result.green_total_length + result.red_total_length);
    const double exterior_fragmentation =
            (result.green_line_count + result.red_line_count) / total_length;
    const double internal_fragmentation =
            result.internal_wall_line_count /
            std::max(1.0, static_cast<double>(result.internal_wall_pixel_count));
    double score = 0.0;
    score += std::min(exterior_coverage, 1.5) * 90.0;
    // Red lines are verified internal structure, not an error condition. Reward
    // their supported length while penalizing fragmentation rather than count.
    score += std::min(internal_coverage, 0.9) * 80.0;
    score -= exterior_fragmentation * 180.0;
    score -= internal_fragmentation * 45.0;
    if (result.green_line_count < 2) score -= 200.0;
    if (result.raw_line_count < result.green_line_count) score -= 30.0;
    if (result.outline_closed) {
        score += 60.0 + result.outline_support_ratio * 60.0;
    } else {
        score -= 120.0;
    }
    // Prefer a footprint that contains the stable explored room area even
    // when part of its facade is weak or absent.  This makes branch selection
    // depend on whole-room evidence instead of rewarding a smaller polygon
    // merely because its remaining walls are dense.
    score += result.free_space_containment_ratio * 120.0;
    if (result.free_space_containment_ratio > 0.0 &&
        result.free_space_containment_ratio < 0.90) {
        score -= (0.90 - result.free_space_containment_ratio) * 260.0;
    }
    // A Manhattan footprint normally needs only a handful of corners.  Keep
    // genuine L/T-shaped wings, but stop raster teeth from winning because
    // they happen to hug more individual wall pixels.
    score -= std::max(0, result.outline_vertex_count - 16) * 2.5;
    score -= std::abs(result.branch - preferred_branch) * 0.05;
    return score;
}

double MedianValue(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const size_t middle = values.size() / 2;
    std::nth_element(
            values.begin(),
            values.begin() + static_cast<long>(middle),
            values.end());
    const double upper = values[middle];
    if (values.size() % 2 != 0) return upper;
    const auto lower = std::max_element(
            values.begin(),
            values.begin() + static_cast<long>(middle));
    return 0.5 * (*lower + upper);
}

double OutlineIntersectionOverUnion(const PipelineResult& first,
                                    const PipelineResult& second,
                                    const cv::Size& image_size) {
    if (first.outline_polygon_px.size() < 3 ||
        second.outline_polygon_px.size() < 3 ||
        image_size.width <= 0 || image_size.height <= 0) {
        return 0.0;
    }
    auto integer_polygon = [](const std::vector<cv::Point2f>& polygon) {
        std::vector<cv::Point> output;
        output.reserve(polygon.size());
        for (const auto& point : polygon) {
            output.emplace_back(
                    static_cast<int>(std::round(point.x)),
                    static_cast<int>(std::round(point.y)));
        }
        return output;
    };
    cv::Mat first_mask = cv::Mat::zeros(image_size, CV_8UC1);
    cv::Mat second_mask = cv::Mat::zeros(image_size, CV_8UC1);
    cv::fillPoly(
            first_mask,
            std::vector<std::vector<cv::Point>>{
                    integer_polygon(first.outline_polygon_px)},
            cv::Scalar(255));
    cv::fillPoly(
            second_mask,
            std::vector<std::vector<cv::Point>>{
                    integer_polygon(second.outline_polygon_px)},
            cv::Scalar(255));
    cv::Mat intersection, union_mask;
    cv::bitwise_and(first_mask, second_mask, intersection);
    cv::bitwise_or(first_mask, second_mask, union_mask);
    return cv::countNonZero(intersection) /
            std::max(1.0, static_cast<double>(cv::countNonZero(union_mask)));
}

void ApplyConsensusScores(std::vector<PipelineResult>* candidates,
                          const cv::Size& image_size,
                          int preferred_branch) {
    if (candidates == nullptr || candidates->empty()) return;
    if (candidates->size() == 1) {
        candidates->front().score = ScorePipelineResult(
                candidates->front(), preferred_branch);
        return;
    }
    std::vector<double> long_sizes;
    std::vector<double> short_sizes;
    std::vector<double> areas;
    std::vector<double> perimeters;
    std::vector<double> vertices;
    for (const auto& candidate : *candidates) {
        long_sizes.push_back(candidate.dimension_long_size_px);
        short_sizes.push_back(candidate.dimension_short_size_px);
        areas.push_back(candidate.footprint_area_px2);
        perimeters.push_back(candidate.footprint_perimeter_px);
        vertices.push_back(candidate.outline_vertex_count);
    }
    const double median_long = std::max(1.0, MedianValue(long_sizes));
    const double median_short = std::max(1.0, MedianValue(short_sizes));
    const double median_area = std::max(1.0, MedianValue(areas));
    const double median_perimeter = std::max(1.0, MedianValue(perimeters));
    const double median_vertices = MedianValue(vertices);
    double supported_minimum_vertices = median_vertices;
    for (const auto& candidate : *candidates) {
        if (candidate.wall_containment_ratio >= 0.90 &&
            candidate.free_space_containment_ratio >= 0.90) {
            supported_minimum_vertices = std::min(
                    supported_minimum_vertices,
                    static_cast<double>(candidate.outline_vertex_count));
        }
    }
    int maximum_verified_internal_pixels = 1;
    for (const auto& candidate : *candidates) {
        maximum_verified_internal_pixels = std::max(
                maximum_verified_internal_pixels,
                candidate.internal_wall_pixel_count);
    }

    for (size_t index = 0; index < candidates->size(); ++index) {
        PipelineResult& candidate = (*candidates)[index];
        double overlap_sum = 0.0;
        for (size_t other = 0; other < candidates->size(); ++other) {
            if (other == index) continue;
            overlap_sum += OutlineIntersectionOverUnion(
                    candidate, (*candidates)[other], image_size);
        }
        const double mean_overlap = overlap_sum /
                std::max(1.0,
                         static_cast<double>(candidates->size() - 1));
        const double dimension_deviation =
                std::fabs(candidate.dimension_long_size_px - median_long) /
                        median_long +
                std::fabs(candidate.dimension_short_size_px - median_short) /
                        median_short;
        const double area_deviation =
                std::fabs(candidate.footprint_area_px2 - median_area) /
                median_area;
        const double perimeter_deviation =
                std::fabs(candidate.footprint_perimeter_px -
                          median_perimeter) /
                median_perimeter;
        const double vertex_deviation =
                std::fabs(candidate.outline_vertex_count - median_vertices);
        const double unsupported_complexity =
                std::max(0.0,
                         candidate.outline_vertex_count -
                                 supported_minimum_vertices);
        const double quality_score = ScorePipelineResult(
                candidate, preferred_branch);
        const double internal_information_retention =
                candidate.internal_wall_pixel_count /
                static_cast<double>(maximum_verified_internal_pixels);
        // Consensus is primary: a candidate must agree spatially and
        // metrically with the other pruning branches. The original line
        // quality remains useful only to break ties inside that common group.
        candidate.score =
                quality_score * 0.35 +
                mean_overlap * 150.0 -
                area_deviation * 130.0 -
                dimension_deviation * 70.0 -
                perimeter_deviation * 35.0 -
                vertex_deviation * 0.8 -
                // Repeated scans often differ only by small unsupported
                // facade stairs. If another high-containment branch explains
                // the same map with fewer corners, require a material quality
                // gain before choosing the more complex outline.
                unsupported_complexity * 4.5 -
                // Prefer the least destructive pruning branch when topology,
                // footprint and wall support are otherwise close. Dense maps
                // can still select a stronger branch when its quality gain is
                // material (as in map3), but ordinary rooms no longer lose a
                // main partition merely for a tiny consensus advantage.
                std::abs(candidate.branch - preferred_branch) * 1.75 +
                candidate.wall_containment_ratio * 250.0 +
                candidate.free_space_containment_ratio * 140.0 +
                // After segment-level facade rejection and topology pruning,
                // these pixels are verified room separators. Reward retaining
                // the common long-wall information across branches; the
                // quality score above still penalizes fragmented overdraw.
                internal_information_retention * 110.0;
        std::cout << "[INFO] 共识评分 branch=" << candidate.branch
                  << " quality=" << std::fixed << std::setprecision(2)
                  << quality_score
                  << " overlap=" << mean_overlap
                  << " area_dev=" << area_deviation
                  << " dimension_dev=" << dimension_deviation
                  << " perimeter_dev=" << perimeter_deviation
                  << " wall_containment="
                  << candidate.wall_containment_ratio
                  << " free_containment="
                  << candidate.free_space_containment_ratio
                  << " internal_retention="
                  << internal_information_retention
                  << " consensus=" << candidate.score
                  << std::defaultfloat << "\n";
    }
}

struct ClosedOutlineResult {
    bool valid = false;
    std::vector<cv::Point> original_polygon;
    int vertex_count = 0;
    int close_size = 0;
    double support_ratio = 0.0;
    double free_space_containment_ratio = 0.0;
    double rotation_degrees = 0.0;
    cv::Point2f dimension_center;
    cv::Point2f long_axis = {1.f, 0.f};
    cv::Point2f short_axis = {0.f, 1.f};
    double long_size_px = 0.0;
    double short_size_px = 0.0;
    double footprint_area_px2 = 0.0;
    double footprint_perimeter_px = 0.0;
};

void FloatBounds(const std::vector<cv::Point2f>& points,
                 float* min_x,
                 float* min_y,
                 float* max_x,
                 float* max_y) {
    *min_x = std::numeric_limits<float>::infinity();
    *min_y = std::numeric_limits<float>::infinity();
    *max_x = -std::numeric_limits<float>::infinity();
    *max_y = -std::numeric_limits<float>::infinity();
    for (const auto& point : points) {
        *min_x = std::min(*min_x, point.x);
        *min_y = std::min(*min_y, point.y);
        *max_x = std::max(*max_x, point.x);
        *max_y = std::max(*max_y, point.y);
    }
}

double EstimateManhattanRotationDegrees(const cv::Mat& wall_mask) {
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(
            Binary255(wall_mask),
            lines,
            1.0,
            kPi / 180.0,
            std::max(18, std::min(wall_mask.rows, wall_mask.cols) / 24),
            std::max(12.0, std::min(wall_mask.rows, wall_mask.cols) * 0.045),
            10.0);
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (const auto& line : lines) {
        const double dx = line[2] - line[0];
        const double dy = line[3] - line[1];
        const double length = std::hypot(dx, dy);
        if (length < 10.0) continue;
        const double angle = std::atan2(dy, dx);
        // Fourfold angle makes horizontal and vertical walls vote for the
        // same Manhattan frame.
        sum_x += length * std::cos(4.0 * angle);
        sum_y += length * std::sin(4.0 * angle);
    }
    if (std::hypot(sum_x, sum_y) < 1e-6) return 0.0;
    double angle = std::atan2(sum_y, sum_x) * 0.25 * 180.0 / kPi;
    while (angle >= 45.0) angle -= 90.0;
    while (angle < -45.0) angle += 90.0;
    return angle;
}

std::vector<cv::Point> OrthogonalizeContour(
        const std::vector<cv::Point>& contour,
        double meters_per_pixel) {
    if (contour.size() < 4) return {};
    const double resolution = std::isfinite(meters_per_pixel) &&
            meters_per_pixel > 1e-4 ? meters_per_pixel : 0.05;
    std::vector<cv::Point> points;
    const double perimeter = cv::arcLength(contour, true);
    cv::approxPolyDP(
            contour,
            points,
            // Architectural footprints should not inherit every occupancy
            // raster wobble.  A stronger physical epsilon removes shallow
            // facade teeth while preserving normal vestibules/recesses.
            std::max(1.5, std::max(0.34 / resolution, perimeter * 0.005)),
            true);
    if (points.size() < 4) return {};

    auto horizontal = [](const cv::Point& first, const cv::Point& second) {
        return std::abs(second.x - first.x) >= std::abs(second.y - first.y);
    };
    bool changed = true;
    while (changed && points.size() > 4) {
        changed = false;
        for (size_t index = 0; index < points.size(); ++index) {
            const size_t next = (index + 1) % points.size();
            const size_t after = (index + 2) % points.size();
            const double first_length = cv::norm(points[next] - points[index]);
            const double second_length = cv::norm(points[after] - points[next]);
            const bool same_axis =
                    horizontal(points[index], points[next]) ==
                    horizontal(points[next], points[after]);
            // Remove only short raster turns here. Building-scale steps are
            // retained directly from the connected broad-free-space mask;
            // there is no later heuristic chain that can remove and then
            // recreate them differently for each scan.
            const bool tiny_step =
                    std::min(first_length, second_length) < 0.40 / resolution;
            if (same_axis || tiny_step) {
                points.erase(points.begin() + static_cast<long>(next));
                changed = true;
                break;
            }
        }
    }
    if (points.size() < 4) return {};

    struct AxisLine {
        bool horizontal = false;
        double coordinate = 0.0;
    };
    std::vector<AxisLine> edges(points.size());
    for (size_t index = 0; index < points.size(); ++index) {
        const cv::Point& first = points[index];
        const cv::Point& second = points[(index + 1) % points.size()];
        edges[index].horizontal = horizontal(first, second);
        edges[index].coordinate = edges[index].horizontal
                ? (first.y + second.y) * 0.5
                : (first.x + second.x) * 0.5;
        if (index > 0 &&
            edges[index].horizontal == edges[index - 1].horizontal) {
            return {};
        }
    }
    if (edges.front().horizontal == edges.back().horizontal) return {};

    std::vector<cv::Point> orthogonal;
    orthogonal.reserve(points.size());
    for (size_t index = 0; index < points.size(); ++index) {
        const AxisLine& previous =
                edges[(index + edges.size() - 1) % edges.size()];
        const AxisLine& current = edges[index];
        const double x = previous.horizontal
                ? current.coordinate : previous.coordinate;
        const double y = previous.horizontal
                ? previous.coordinate : current.coordinate;
        orthogonal.emplace_back(
                static_cast<int>(std::round(x)),
                static_cast<int>(std::round(y)));
    }
    orthogonal.erase(
            std::unique(orthogonal.begin(), orthogonal.end()),
            orthogonal.end());
    if (orthogonal.size() < 4 ||
        std::fabs(cv::contourArea(orthogonal)) < 1.0) return {};
    return orthogonal;
}

double HorizontalWallCoverage(const cv::Mat& walls,
                              int y,
                              int left,
                              int right,
                              int tolerance) {
    left = std::clamp(left, 0, walls.cols - 1);
    right = std::clamp(right, left + 1, walls.cols);
    const int top = std::max(0, y - tolerance);
    const int bottom = std::min(walls.rows - 1, y + tolerance);
    int supported = 0;
    for (int x = left; x < right; ++x) {
        bool found = false;
        for (int sample_y = top; sample_y <= bottom; ++sample_y) {
            if (walls.at<uchar>(sample_y, x) != 0) {
                found = true;
                break;
            }
        }
        if (found) ++supported;
    }
    return supported / std::max(1.0, static_cast<double>(right - left));
}

double VerticalWallCoverage(const cv::Mat& walls,
                            int x,
                            int top,
                            int bottom,
                            int tolerance) {
    top = std::clamp(top, 0, walls.rows - 1);
    bottom = std::clamp(bottom, top + 1, walls.rows);
    const int left = std::max(0, x - tolerance);
    const int right = std::min(walls.cols - 1, x + tolerance);
    int supported = 0;
    for (int y = top; y < bottom; ++y) {
        bool found = false;
        for (int sample_x = left; sample_x <= right; ++sample_x) {
            if (walls.at<uchar>(y, sample_x) != 0) {
                found = true;
                break;
            }
        }
        if (found) ++supported;
    }
    return supported / std::max(1.0, static_cast<double>(bottom - top));
}

// Collapse a directional wall band onto one robust centreline per connected
// run. Occupancy-grid walls are commonly two to several pixels thick and
// wander by a few pixels because adjacent scans/submaps do not register
// perfectly. Using those raw rows/columns for the final snap makes a straight
// facade inherit the raster wobble. The fitted mask is only a geometric
// proposal; downstream support checks still use the original observations.
cv::Mat FitDirectionalWallCenterlines(const cv::Mat& directional_walls,
                                      bool horizontal,
                                      double meters_per_pixel) {
    if (directional_walls.empty()) return {};
    const double resolution =
            std::isfinite(meters_per_pixel) && meters_per_pixel > 1e-4
            ? meters_per_pixel
            : 0.05;
    cv::Mat binary = Binary255(directional_walls);
    cv::Mat labels, stats, centroids;
    const int component_count = cv::connectedComponentsWithStats(
            binary, labels, stats, centroids, 8, CV_32S);
    const int minimum_run = std::clamp(
            static_cast<int>(std::round(0.45 / resolution)), 6, 24);
    const int maximum_band = std::clamp(
            static_cast<int>(std::round(0.45 / resolution)), 5, 16);
    cv::Mat fitted = cv::Mat::zeros(binary.size(), CV_8UC1);
    for (int label = 1; label < component_count; ++label) {
        const int left = stats.at<int>(label, cv::CC_STAT_LEFT);
        const int top = stats.at<int>(label, cv::CC_STAT_TOP);
        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        const int long_size = horizontal ? width : height;
        const int band_size = horizontal ? height : width;
        if (long_size < minimum_run || band_size > maximum_band ||
            long_size < band_size * 2) {
            continue;
        }

        std::vector<int> normal_coordinates;
        normal_coordinates.reserve(
                static_cast<size_t>(stats.at<int>(label, cv::CC_STAT_AREA)));
        int span_min = std::numeric_limits<int>::max();
        int span_max = std::numeric_limits<int>::min();
        for (int y = top; y < top + height; ++y) {
            for (int x = left; x < left + width; ++x) {
                if (labels.at<int>(y, x) != label) continue;
                normal_coordinates.push_back(horizontal ? y : x);
                const int tangent = horizontal ? x : y;
                span_min = std::min(span_min, tangent);
                span_max = std::max(span_max, tangent);
            }
        }
        if (normal_coordinates.empty() || span_max <= span_min) continue;
        const size_t middle = normal_coordinates.size() / 2;
        std::nth_element(
                normal_coordinates.begin(),
                normal_coordinates.begin() + static_cast<long>(middle),
                normal_coordinates.end());
        const int coordinate = normal_coordinates[middle];
        if (horizontal) {
            cv::line(fitted,
                     cv::Point(span_min, coordinate),
                     cv::Point(span_max, coordinate),
                     cv::Scalar(255), 1, cv::LINE_8);
        } else {
            cv::line(fitted,
                     cv::Point(coordinate, span_min),
                     cv::Point(coordinate, span_max),
                     cv::Scalar(255), 1, cv::LINE_8);
        }
    }
    return fitted;
}

// Recover room-separating walls as complete architectural runs instead of
// depending on a single local Hough segment.  In the Manhattan frame a real
// partition remains a long horizontal/vertical occupied band with explored
// floor beside it. Directional closing spans doors and short scan dropouts;
// the original raster still supplies the support score, so the operation does
// not invent a wall across an empty room.
std::vector<cv::Vec4i> DetectRegionSeparatorSegments(
        const cv::Mat& observed_wall_binary,
        const cv::Mat& semantic_bgr,
        const std::vector<cv::Point>& outer_polygon,
        double meters_per_pixel,
        const std::string& debug_dir) {
    std::vector<cv::Vec4i> result;
    if (observed_wall_binary.empty() || semantic_bgr.empty() ||
        observed_wall_binary.size() != semantic_bgr.size() ||
        outer_polygon.size() < 4) {
        return result;
    }
    const double resolution =
            std::isfinite(meters_per_pixel) && meters_per_pixel > 1e-4
            ? meters_per_pixel : 0.05;
    cv::Mat free_mask;
    cv::inRange(
            semantic_bgr,
            cv::Scalar(245, 245, 245),
            cv::Scalar(255, 255, 255),
            free_mask);
    const int free_tolerance = std::clamp(
            static_cast<int>(std::round(0.08 / resolution)), 1, 4);
    cv::dilate(
            free_mask,
            free_mask,
            cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(free_tolerance * 2 + 1,
                             free_tolerance * 2 + 1)));

    // The already-fitted exterior is the most stable estimate of the
    // building frame. Re-estimating it from cluttered wall pixels can be off
    // by 10–20 degrees in narrow/vertical plans and makes every directional
    // partition disappear.
    double frame_cosine = 0.0;
    double frame_sine = 0.0;
    for (size_t index = 0; index < outer_polygon.size(); ++index) {
        const cv::Point2d edge =
                cv::Point2d(outer_polygon[(index + 1) % outer_polygon.size()]) -
                cv::Point2d(outer_polygon[index]);
        const double length = cv::norm(edge);
        if (length < 2.0) continue;
        const double angle = std::atan2(edge.y, edge.x);
        frame_cosine += length * std::cos(4.0 * angle);
        frame_sine += length * std::sin(4.0 * angle);
    }
    double alignment_degrees =
            0.25 * std::atan2(frame_sine, frame_cosine) * 180.0 / kPi;
    while (alignment_degrees >= 45.0) alignment_degrees -= 90.0;
    while (alignment_degrees < -45.0) alignment_degrees += 90.0;
    const cv::Point2f image_center(
            observed_wall_binary.cols * 0.5f,
            observed_wall_binary.rows * 0.5f);
    const cv::Mat rotation = cv::getRotationMatrix2D(
            image_center, alignment_degrees, 1.0);
    cv::Mat inverse_rotation;
    cv::invertAffineTransform(rotation, inverse_rotation);
    cv::Mat aligned_walls;
    cv::Mat aligned_free;
    cv::warpAffine(
            Binary255(observed_wall_binary),
            aligned_walls,
            rotation,
            observed_wall_binary.size(),
            cv::INTER_NEAREST,
            cv::BORDER_CONSTANT,
            cv::Scalar(0));
    cv::warpAffine(
            free_mask,
            aligned_free,
            rotation,
            free_mask.size(),
            cv::INTER_NEAREST,
            cv::BORDER_CONSTANT,
            cv::Scalar(0));
    std::vector<cv::Point2f> polygon_float;
    polygon_float.reserve(outer_polygon.size());
    for (const auto& point : outer_polygon) polygon_float.emplace_back(point);
    std::vector<cv::Point2f> aligned_polygon_float;
    cv::transform(polygon_float, aligned_polygon_float, rotation);
    std::vector<cv::Point> aligned_polygon;
    aligned_polygon.reserve(aligned_polygon_float.size());
    for (const auto& point : aligned_polygon_float) {
        aligned_polygon.emplace_back(
                static_cast<int>(std::round(point.x)),
                static_cast<int>(std::round(point.y)));
    }

    const int minimum_run = std::clamp(
            static_cast<int>(std::round(0.55 / resolution)), 7, 28);
    int bridge_gap = std::clamp(
            static_cast<int>(std::round(0.85 / resolution)), 7, 35);
    if (bridge_gap % 2 == 0) ++bridge_gap;
    cv::Mat horizontal;
    cv::Mat vertical;
    cv::morphologyEx(
            aligned_walls,
            horizontal,
            cv::MORPH_OPEN,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(minimum_run, 1)));
    cv::morphologyEx(
            aligned_walls,
            vertical,
            cv::MORPH_OPEN,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(1, minimum_run)));
    cv::morphologyEx(
            horizontal,
            horizontal,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(bridge_gap, 3)));
    cv::morphologyEx(
            vertical,
            vertical,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(3, bridge_gap)));
    horizontal = FitDirectionalWallCenterlines(
            horizontal, true, resolution);
    vertical = FitDirectionalWallCenterlines(
            vertical, false, resolution);

    const double minimum_length = std::clamp(
            0.70 / resolution, 10.0, 70.0);
    const double strong_length = std::clamp(
            1.40 / resolution, 22.0, 140.0);
    const double minimum_depth = std::clamp(
            0.22 / resolution, 3.0, 10.0);
    const double side_distance = std::clamp(
            0.28 / resolution, 4.0, 12.0);
    const double endpoint_anchor_distance = std::clamp(
            0.45 / resolution, 5.0, 18.0);
    const int support_thickness = std::clamp(
            static_cast<int>(std::round(0.10 / resolution)), 2, 5);

    auto extract = [&](const cv::Mat& fitted, bool is_horizontal) {
        cv::Mat labels, stats, centroids;
        const int component_count = cv::connectedComponentsWithStats(
                fitted, labels, stats, centroids, 8, CV_32S);
        for (int label = 1; label < component_count; ++label) {
            const int left = stats.at<int>(label, cv::CC_STAT_LEFT);
            const int top = stats.at<int>(label, cv::CC_STAT_TOP);
            const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
            const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
            cv::Vec4i segment = is_horizontal
                    ? cv::Vec4i(left,
                                static_cast<int>(std::round(
                                        centroids.at<double>(label, 1))),
                                left + width - 1,
                                static_cast<int>(std::round(
                                        centroids.at<double>(label, 1))))
                    : cv::Vec4i(
                                static_cast<int>(std::round(
                                        centroids.at<double>(label, 0))),
                                top,
                                static_cast<int>(std::round(
                                        centroids.at<double>(label, 0))),
                                top + height - 1);
            const double length = SegmentLength(segment);
            if (length < minimum_length) continue;
            const cv::Point2f midpoint(
                    0.5f * (segment[0] + segment[2]),
                    0.5f * (segment[1] + segment[3]));
            if (cv::pointPolygonTest(
                        aligned_polygon, midpoint, true) < minimum_depth) {
                continue;
            }
            const double support = SegmentSupportRatio(
                    aligned_walls, segment, support_thickness);
            if (support < 0.30) continue;
            const cv::Point2d tangent = is_horizontal
                    ? cv::Point2d(1.0, 0.0)
                    : cv::Point2d(0.0, 1.0);
            const cv::Point2d normal(-tangent.y, tangent.x);
            int positive_free = 0;
            int negative_free = 0;
            constexpr int kSamples = 15;
            for (int sample = 1; sample < kSamples - 1; ++sample) {
                const double fraction =
                        sample / static_cast<double>(kSamples - 1);
                const cv::Point2d center(
                        segment[0] + fraction * (segment[2] - segment[0]),
                        segment[1] + fraction * (segment[3] - segment[1]));
                auto is_free = [&](const cv::Point2d& point) {
                    const int x = static_cast<int>(std::round(point.x));
                    const int y = static_cast<int>(std::round(point.y));
                    return x >= 0 && x < aligned_free.cols &&
                            y >= 0 && y < aligned_free.rows &&
                            aligned_free.at<uchar>(y, x) != 0;
                };
                if (is_free(center + normal * side_distance)) ++positive_free;
                if (is_free(center - normal * side_distance)) ++negative_free;
            }
            const double positive_ratio = positive_free / 13.0;
            const double negative_ratio = negative_free / 13.0;
            const bool two_sided =
                    std::min(positive_ratio, negative_ratio) >= 0.30 &&
                    std::max(positive_ratio, negative_ratio) >= 0.55;
            int anchored_endpoints = 0;
            for (const cv::Point2f& endpoint : {
                         cv::Point2f(segment[0], segment[1]),
                         cv::Point2f(segment[2], segment[3])}) {
                if (std::fabs(cv::pointPolygonTest(
                            aligned_polygon, endpoint, true)) <=
                    endpoint_anchor_distance) {
                    ++anchored_endpoints;
                }
            }
            const bool strong_one_sided =
                    length >= strong_length && support >= 0.55 &&
                    std::max(positive_ratio, negative_ratio) >= 0.60 &&
                    anchored_endpoints >= 1;
            if (!two_sided && !strong_one_sided) continue;

            std::vector<cv::Point2f> aligned_endpoints{
                    cv::Point2f(segment[0], segment[1]),
                    cv::Point2f(segment[2], segment[3])};
            std::vector<cv::Point2f> source_endpoints;
            cv::transform(
                    aligned_endpoints, source_endpoints, inverse_rotation);
            result.emplace_back(
                    static_cast<int>(std::round(source_endpoints[0].x)),
                    static_cast<int>(std::round(source_endpoints[0].y)),
                    static_cast<int>(std::round(source_endpoints[1].x)),
                    static_cast<int>(std::round(source_endpoints[1].y)));
        }
    };
    extract(horizontal, true);
    extract(vertical, false);

    if (!debug_dir.empty()) {
        cv::Mat debug;
        cv::cvtColor(Binary255(observed_wall_binary), debug, cv::COLOR_GRAY2BGR);
        for (const auto& segment : result) {
            cv::line(debug,
                     cv::Point(segment[0], segment[1]),
                     cv::Point(segment[2], segment[3]),
                     cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
        }
        cv::imwrite(PathJoin(debug_dir, "region_separator_segments.png"), debug);
    }
    return result;
}

double BinaryRectDensity(const cv::Mat& integral,
                         int center_x,
                         int center_y,
                         int half_width,
                         int half_height) {
    const int cols = integral.cols - 1;
    const int rows = integral.rows - 1;
    const int left = std::clamp(center_x - half_width, 0, cols);
    const int right = std::clamp(center_x + half_width + 1, 0, cols);
    const int top = std::clamp(center_y - half_height, 0, rows);
    const int bottom = std::clamp(center_y + half_height + 1, 0, rows);
    if (right <= left || bottom <= top) return 0.0;
    const int count = integral.at<int>(bottom, right) -
            integral.at<int>(top, right) -
            integral.at<int>(bottom, left) +
            integral.at<int>(top, left);
    return count / static_cast<double>((right - left) * (bottom - top));
}

// Exterior walls separate a broad scanned indoor face from mostly unknown
// space. Internal partitions generally have broad free space on both sides.
// Keeping this evidence separate prevents a long internal wall from winning
// the final outline snap merely because its point cloud is cleaner.
cv::Mat BuildExteriorWallEvidence(const cv::Mat& aligned_walls,
                                  const cv::Mat& horizontal_walls,
                                  const cv::Mat& vertical_walls,
                                  const cv::Mat& aligned_free,
                                  double meters_per_pixel) {
    const double resolution =
            std::isfinite(meters_per_pixel) && meters_per_pixel > 1e-4
                    ? meters_per_pixel
                    : 0.05;
    cv::Mat free01;
    cv::threshold(aligned_free, free01, 0, 1, cv::THRESH_BINARY);
    cv::Mat free_integral;
    cv::integral(free01, free_integral, CV_32S);

    const int normal_offset = std::clamp(
            static_cast<int>(std::round(0.32 / resolution)), 4, 14);
    const int normal_half_size = std::clamp(
            static_cast<int>(std::round(0.14 / resolution)), 2, 6);
    const int tangent_half_size = std::clamp(
            static_cast<int>(std::round(0.22 / resolution)), 3, 9);
    cv::Mat horizontal_evidence = cv::Mat::zeros(
            aligned_walls.size(), CV_8UC1);
    cv::Mat vertical_evidence = cv::Mat::zeros(
            aligned_walls.size(), CV_8UC1);
    for (int y = 0; y < aligned_walls.rows; ++y) {
        for (int x = 0; x < aligned_walls.cols; ++x) {
            if (horizontal_walls.at<uchar>(y, x) != 0) {
                const double above = BinaryRectDensity(
                        free_integral,
                        x,
                        y - normal_offset,
                        tangent_half_size,
                        normal_half_size);
                const double below = BinaryRectDensity(
                        free_integral,
                        x,
                        y + normal_offset,
                        tangent_half_size,
                        normal_half_size);
                const double inside = std::max(above, below);
                const double outside = std::min(above, below);
                if (inside >= 0.40 && outside <= 0.38 &&
                    inside - outside >= 0.18) {
                    horizontal_evidence.at<uchar>(y, x) = 255;
                }
            }
            if (vertical_walls.at<uchar>(y, x) != 0) {
                const double left = BinaryRectDensity(
                        free_integral,
                        x - normal_offset,
                        y,
                        normal_half_size,
                        tangent_half_size);
                const double right = BinaryRectDensity(
                        free_integral,
                        x + normal_offset,
                        y,
                        normal_half_size,
                        tangent_half_size);
                const double inside = std::max(left, right);
                const double outside = std::min(left, right);
                if (inside >= 0.40 && outside <= 0.38 &&
                    inside - outside >= 0.18) {
                    vertical_evidence.at<uchar>(y, x) = 255;
                }
            }
        }
    }

    const int evidence_gap = std::clamp(
            static_cast<int>(std::round(0.45 / resolution)), 5, 19);
    cv::morphologyEx(
            horizontal_evidence,
            horizontal_evidence,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(evidence_gap | 1, 3)));
    cv::morphologyEx(
            vertical_evidence,
            vertical_evidence,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(3, evidence_gap | 1)));
    cv::Mat evidence;
    cv::bitwise_or(horizontal_evidence, vertical_evidence, evidence);
    cv::dilate(
            evidence,
            evidence,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    cv::bitwise_and(evidence, aligned_walls, evidence);
    return evidence;
}

// Move each orthogonal footprint edge onto the outside of the nearby observed
// exterior wall. The free-space mask describes the inside face of a room, but
// the report's green geometry is a building envelope: black exterior-wall
// observations must lie on or inside it. Adjacent snapped lines are intersected
// afterwards so missing/noisy corners remain closed.
bool SnapOrthogonalOutlineToWalls(const cv::Mat& aligned_walls,
                                  const cv::Mat& exterior_wall_evidence,
                                  double meters_per_pixel,
                                  std::vector<cv::Point>* polygon) {
    if (polygon == nullptr || polygon->size() < 4 ||
        aligned_walls.empty()) {
        return false;
    }
    // Rounding line intersections can leave a duplicate point, which turns
    // the next two nominally orthogonal edges into two consecutive edges on
    // the same axis. Normalize that raster artifact before fitting.
    bool normalized = true;
    while (normalized && polygon->size() > 4) {
        normalized = false;
        for (size_t index = 0; index < polygon->size(); ++index) {
            const size_t previous_index =
                    (index + polygon->size() - 1) % polygon->size();
            const size_t next_index = (index + 1) % polygon->size();
            const cv::Point& previous = (*polygon)[previous_index];
            const cv::Point& current = (*polygon)[index];
            const cv::Point& next = (*polygon)[next_index];
            const bool duplicate = current == previous || current == next;
            const bool previous_horizontal = previous.y == current.y;
            const bool previous_vertical = previous.x == current.x;
            const bool next_horizontal = current.y == next.y;
            const bool next_vertical = current.x == next.x;
            const bool collinear =
                    (previous_horizontal && next_horizontal) ||
                    (previous_vertical && next_vertical);
            if (duplicate || collinear) {
                polygon->erase(
                        polygon->begin() + static_cast<long>(index));
                normalized = true;
                break;
            }
        }
    }
    const double resolution =
            std::isfinite(meters_per_pixel) && meters_per_pixel > 1e-4
                    ? meters_per_pixel
                    : 0.05;
    // The reachable-floor contour is closed with a 0.65 m kernel before it
    // reaches this registration step.  Its provisional edge can therefore
    // sit more than 0.5 m inside a real facade even when the wall observation
    // itself is clean.  Search at least that preprocessing displacement;
    // continuous wall coverage below still decides the winning coordinate.
    const int search_radius = std::clamp(
            static_cast<int>(std::round(0.85 / resolution)), 6, 30);
    const int wall_tolerance = std::clamp(
            static_cast<int>(std::round(0.06 / resolution)), 1, 2);
    const int endpoint_inset = std::clamp(
            static_cast<int>(std::round(0.08 / resolution)), 1, 4);
    const int minimum_snap_length = std::clamp(
            static_cast<int>(std::round(0.45 / resolution)), 5, 16);
    const int inward_search_radius = std::clamp(
            static_cast<int>(std::round(0.80 / resolution)), 6, 28);

    struct AxisLine {
        bool horizontal = false;
        int coordinate = 0;
        double support = 0.0;
        int begin = 0;
        int end = 0;
    };
    std::vector<AxisLine> edges(polygon->size());
    int snapped_edges = 0;
    for (size_t index = 0; index < polygon->size(); ++index) {
        const cv::Point& first = (*polygon)[index];
        const cv::Point& second = (*polygon)[(index + 1) % polygon->size()];
        const bool horizontal = first.y == second.y;
        const bool vertical = first.x == second.x;
        if (!horizontal && !vertical) {
            std::cout << "[DEBUG] 外墙吸附拒绝: 非正交边\n";
            return false;
        }

        const int original_coordinate = horizontal ? first.y : first.x;
        int begin = horizontal
                ? std::min(first.x, second.x)
                : std::min(first.y, second.y);
        int end = horizontal
                ? std::max(first.x, second.x)
                : std::max(first.y, second.y);
        if (end - begin < minimum_snap_length) {
            edges[index] = {
                    horizontal, original_coordinate, 0.0, begin, end};
            continue;
        }
        if (end - begin > endpoint_inset * 2 + 2) {
            begin += endpoint_inset;
            end -= endpoint_inset;
        }

        const cv::Point2f midpoint(
                (first.x + second.x) * 0.5f,
                (first.y + second.y) * 0.5f);
        const float probe_distance = 2.5f;
        const cv::Point2f positive_probe = horizontal
                ? midpoint + cv::Point2f(0.f, probe_distance)
                : midpoint + cv::Point2f(probe_distance, 0.f);
        const cv::Point2f negative_probe = horizontal
                ? midpoint - cv::Point2f(0.f, probe_distance)
                : midpoint - cv::Point2f(probe_distance, 0.f);
        const double positive_inside = cv::pointPolygonTest(
                *polygon, positive_probe, false);
        const double negative_inside = cv::pointPolygonTest(
                *polygon, negative_probe, false);
        int interior_sign = 0;
        if (positive_inside >= 0.0 && negative_inside < 0.0) {
            interior_sign = 1;
        } else if (negative_inside >= 0.0 && positive_inside < 0.0) {
            interior_sign = -1;
        } else {
            const cv::Moments moments = cv::moments(*polygon);
            const double center_x = moments.m00 != 0.0
                    ? moments.m10 / moments.m00
                    : midpoint.x;
            const double center_y = moments.m00 != 0.0
                    ? moments.m01 / moments.m00
                    : midpoint.y;
            interior_sign = horizontal
                    ? (center_y >= original_coordinate ? 1 : -1)
                    : (center_x >= original_coordinate ? 1 : -1);
        }

        struct CoordinateCandidate {
            int coordinate = 0;
            int distance = 0;
            double support = 0.0;
            double center_support = 0.0;
            double exterior_support = 0.0;
        };
        std::vector<CoordinateCandidate> candidates;
        int nearest_supported_distance = search_radius + 1;
        for (int delta = -search_radius; delta <= search_radius; ++delta) {
            // This provisional polygon comes from reachable free space, so it
            // normally lies on the indoor face of the wall. Search freely
            // toward the exterior and only a short distance toward the room;
            // otherwise a parallel internal partition can steal this edge.
            if (delta * interior_sign > inward_search_radius) continue;
            const int coordinate = original_coordinate + delta;
            if ((horizontal &&
                 (coordinate < 0 || coordinate >= aligned_walls.rows)) ||
                (!horizontal &&
                 (coordinate < 0 || coordinate >= aligned_walls.cols))) {
                continue;
            }
            const double support = horizontal
                    ? HorizontalWallCoverage(
                            aligned_walls,
                            coordinate,
                            begin,
                            end,
                            wall_tolerance)
                    : VerticalWallCoverage(
                            aligned_walls,
                            coordinate,
                            begin,
                            end,
                            wall_tolerance);
            const double center_support = horizontal
                    ? HorizontalWallCoverage(
                            aligned_walls,
                            coordinate,
                            begin,
                            end,
                            0)
                    : VerticalWallCoverage(
                            aligned_walls,
                            coordinate,
                            begin,
                            end,
                            0);
            const double exterior_support =
                    exterior_wall_evidence.empty()
                    ? 0.0
                    : (horizontal
                        ? HorizontalWallCoverage(
                                exterior_wall_evidence,
                                coordinate,
                                begin,
                                end,
                                wall_tolerance)
                        : VerticalWallCoverage(
                                exterior_wall_evidence,
                                coordinate,
                                begin,
                                end,
                                wall_tolerance));
            // A tolerance-only hit can be a parallel line one or two pixels
            // outside the actual wall. Require pixels on the candidate centre
            // coordinate when establishing the nearest wall position.
            const bool coordinate_hits_wall =
                    center_support >= 0.05 ||
                    (center_support >= 0.025 &&
                     exterior_support >= 0.10);
            if (support >= 0.10 && coordinate_hits_wall) {
                nearest_supported_distance = std::min(
                        nearest_supported_distance, std::abs(delta));
                candidates.push_back({
                        coordinate,
                        std::abs(delta),
                        support,
                        center_support,
                        exterior_support});
            }
        }

        // Estimate the facade position independently at many tangent samples.
        // A long probability-map wall is rarely one pixel wide: submap
        // interpolation makes it slightly curved/thicker and a single global
        // coverage maximum can be pulled toward one dense patch.  Per-sample
        // band centres followed by a median give the architectural centre of
        // the visible black wall while ignoring doors, missing returns and
        // isolated outdoor rays.
        std::vector<int> sampled_wall_centres;
        const int tangent_step = std::max(1, (end - begin) / 80);
        auto has_wall_pixel = [&](int coordinate, int tangent) {
            for (int tangent_offset = -1; tangent_offset <= 1;
                 ++tangent_offset) {
                const int sample_tangent = tangent + tangent_offset;
                const int x = horizontal ? sample_tangent : coordinate;
                const int y = horizontal ? coordinate : sample_tangent;
                if (x >= 0 && x < aligned_walls.cols &&
                    y >= 0 && y < aligned_walls.rows &&
                    aligned_walls.at<uchar>(y, x) != 0) {
                    return true;
                }
            }
            return false;
        };
        for (int tangent = begin; tangent <= end;
             tangent += tangent_step) {
            std::vector<std::pair<int, int>> bands;
            int band_start = std::numeric_limits<int>::max();
            int previous_coordinate = std::numeric_limits<int>::min();
            for (int delta = -search_radius; delta <= search_radius;
                 ++delta) {
                if (delta * interior_sign > inward_search_radius) continue;
                const int coordinate = original_coordinate + delta;
                if ((horizontal &&
                     (coordinate < 0 || coordinate >= aligned_walls.rows)) ||
                    (!horizontal &&
                     (coordinate < 0 || coordinate >= aligned_walls.cols))) {
                    continue;
                }
                if (has_wall_pixel(coordinate, tangent)) {
                    if (band_start == std::numeric_limits<int>::max() ||
                        coordinate > previous_coordinate + 1) {
                        if (band_start != std::numeric_limits<int>::max()) {
                            bands.emplace_back(
                                    band_start, previous_coordinate);
                        }
                        band_start = coordinate;
                    }
                    previous_coordinate = coordinate;
                }
            }
            if (band_start != std::numeric_limits<int>::max()) {
                bands.emplace_back(band_start, previous_coordinate);
            }
            if (bands.empty()) continue;
            auto best_band = bands.front();
            double best_band_score =
                    std::numeric_limits<double>::infinity();
            for (const auto& band : bands) {
                const double centre = 0.5 * (band.first + band.second);
                const double signed_offset =
                        (centre - original_coordinate) * interior_sign;
                // Inward candidates can be internal partitions. They remain
                // available for a genuinely inside provisional contour, but
                // receive a modest penalty when an equally close exterior
                // wall band exists.
                const double inward_penalty =
                        signed_offset > 0.0 ? signed_offset * 0.35 : 0.0;
                const double score =
                        std::fabs(centre - original_coordinate) +
                        inward_penalty;
                if (score < best_band_score) {
                    best_band_score = score;
                    best_band = band;
                }
            }
            sampled_wall_centres.push_back(static_cast<int>(std::round(
                    0.5 * (best_band.first + best_band.second))));
        }

        int robust_wall_coordinate = original_coordinate;
        const int minimum_sampled_sections = std::max(
                4, (end - begin) / std::max(1, tangent_step) / 5);
        const bool has_robust_wall_coordinate =
                static_cast<int>(sampled_wall_centres.size()) >=
                minimum_sampled_sections;
        if (has_robust_wall_coordinate) {
            const size_t middle = sampled_wall_centres.size() / 2;
            std::nth_element(
                    sampled_wall_centres.begin(),
                    sampled_wall_centres.begin() +
                            static_cast<long>(middle),
                    sampled_wall_centres.end());
            robust_wall_coordinate = sampled_wall_centres[middle];
        }

        int best_coordinate = original_coordinate;
        double best_support = 0.0;
        double best_score = -std::numeric_limits<double>::infinity();
        for (const auto& candidate : candidates) {
            const double score = candidate.support +
                    0.35 * candidate.center_support +
                    0.85 * candidate.exterior_support -
                    0.02 * candidate.distance -
                    (has_robust_wall_coordinate
                         ? 0.035 * std::fabs(
                               candidate.coordinate -
                               robust_wall_coordinate)
                         : 0.0);
            if (score > best_score) {
                best_score = score;
                best_support = candidate.support;
                best_coordinate = candidate.coordinate;
            }
        }
        if (has_robust_wall_coordinate) {
            const double robust_support = horizontal
                    ? HorizontalWallCoverage(
                            aligned_walls,
                            robust_wall_coordinate,
                            begin,
                            end,
                            wall_tolerance)
                    : VerticalWallCoverage(
                            aligned_walls,
                            robust_wall_coordinate,
                            begin,
                            end,
                            wall_tolerance);
            // The edge-wide coverage score can be won by a sparse row near
            // the provisional free-space contour (or by an interpolation
            // halo on the outside of a thick wall).  The median of many
            // independently observed wall-band centres is a better estimate
            // of the visible facade.  Permit it to move more than one raster
            // cell when it has materially stronger support; this keeps the
            // Manhattan direction while removing the common one-wall-thick
            // green/black offset.
            const bool materially_better_wall_band =
                    robust_support >= 0.14 &&
                    robust_support >= best_support + 0.08;
            if (robust_support >= 0.10 &&
                (best_support < 0.10 ||
                 std::fabs(best_coordinate - robust_wall_coordinate) <=
                         wall_tolerance + 1 ||
                 materially_better_wall_band)) {
                best_coordinate = robust_wall_coordinate;
                best_support = std::max(best_support, robust_support);
            }
        }
        edges[index] = {
                horizontal, original_coordinate, best_support, begin, end};
        if (best_support >= 0.10) {
            // `aligned_walls` is now extracted from the same continuous
            // probability raster that is drawn behind the annotations.  Keep
            // the fitted line on the maximum-support wall coordinate. Walking
            // further toward the exterior face used to shift the green stroke
            // several pixels outside the visible black boundary, especially
            // where a thick wall was composed from multiple submaps.
            edges[index].coordinate = best_coordinate;
            if (best_coordinate != original_coordinate) ++snapped_edges;
        }
    }
    if (snapped_edges == 0) {
        std::cout << "[DEBUG] 外墙吸附跳过: 无可吸附边\n";
        return false;
    }

    // Snap candidates are evaluated edge by edge. Two nearly coincident runs
    // can therefore land on adjacent raster rows/columns and create a small
    // staircase even though they describe one facade. Cluster such runs onto
    // the better-supported/longer line before intersecting adjacent edges.
    // A larger real recess is retained; only a very shallow offset, or a
    // shallow short return, is collapsed.
    // Only collapse offsets that are plausibly caused by wall thickness or
    // raster quantisation. The previous 0.38--0.65 m allowance could move a
    // correctly snapped facade onto a neighbouring parallel run, visibly
    // leaving the black exterior point cloud behind.
    const double unconditional_line_cluster =
            std::clamp(0.12 / resolution, 2.0, 4.0);
    const double conditional_line_cluster =
            std::clamp(0.24 / resolution, 3.0, 7.0);
    const double maximum_short_run =
            std::clamp(1.50 / resolution, 16.0, 54.0);
    int clustered_outline_steps = 0;
    for (size_t index = 0; index < edges.size(); ++index) {
        const size_t middle = (index + 1) % edges.size();
        const size_t next_parallel = (index + 2) % edges.size();
        if (edges[index].horizontal != edges[next_parallel].horizontal ||
            edges[index].horizontal == edges[middle].horizontal) {
            continue;
        }
        const cv::Point first_start = (*polygon)[index];
        const cv::Point first_end =
                (*polygon)[(index + 1) % polygon->size()];
        const cv::Point second_start =
                (*polygon)[next_parallel];
        const cv::Point second_end =
                (*polygon)[(next_parallel + 1) % polygon->size()];
        const cv::Point first_direction = first_end - first_start;
        const cv::Point second_direction = second_end - second_start;
        if (first_direction.dot(second_direction) <= 0) continue;
        const double offset = std::fabs(
                edges[index].coordinate -
                edges[next_parallel].coordinate);
        const double first_length = cv::norm(first_direction);
        const double second_length = cv::norm(second_direction);
        const bool merge_lines = offset <= unconditional_line_cluster ||
                (offset <= conditional_line_cluster &&
                 std::min(first_length, second_length) <= maximum_short_run);
        if (!merge_lines) continue;
        const double first_score = first_length *
                (0.50 + edges[index].support);
        const double second_score = second_length *
                (0.50 + edges[next_parallel].support);
        const int common_coordinate = first_score >= second_score
                ? edges[index].coordinate
                : edges[next_parallel].coordinate;
        auto support_at = [&](const AxisLine& edge, int coordinate) {
            return edge.horizontal
                    ? HorizontalWallCoverage(
                            aligned_walls,
                            coordinate,
                            edge.begin,
                            edge.end,
                            wall_tolerance)
                    : VerticalWallCoverage(
                            aligned_walls,
                            coordinate,
                            edge.begin,
                            edge.end,
                            wall_tolerance);
        };
        const double first_common_support =
                support_at(edges[index], common_coordinate);
        const double second_common_support =
                support_at(edges[next_parallel], common_coordinate);
        // Geometry cleanup must never override actual facade registration.
        // Both shifted runs need independent black-wall support, except for a
        // genuinely missing short return whose original fit also had none.
        const bool first_supported = first_common_support >= 0.08 ||
                (edges[index].support < 0.08 &&
                 first_length <= maximum_short_run);
        const bool second_supported = second_common_support >= 0.08 ||
                (edges[next_parallel].support < 0.08 &&
                 second_length <= maximum_short_run);
        if (!first_supported || !second_supported) continue;
        edges[index].coordinate = common_coordinate;
        edges[next_parallel].coordinate = common_coordinate;
        edges[index].support = std::max(
                edges[index].support, first_common_support);
        edges[next_parallel].support = std::max(
                edges[next_parallel].support, second_common_support);
        ++clustered_outline_steps;
    }
    if (clustered_outline_steps > 0) {
        std::cout << "[INFO] 外轮廓近似共线合并="
                  << clustered_outline_steps << "\n";
    }

    std::vector<cv::Point> snapped;
    snapped.reserve(edges.size());
    for (size_t index = 0; index < edges.size(); ++index) {
        const AxisLine& previous =
                edges[(index + edges.size() - 1) % edges.size()];
        const AxisLine& current = edges[index];
        if (previous.horizontal == current.horizontal) {
            std::cout << "[DEBUG] 外墙吸附拒绝: 相邻边同向\n";
            return false;
        }
        snapped.emplace_back(
                previous.horizontal
                        ? current.coordinate
                        : previous.coordinate,
                previous.horizontal
                        ? previous.coordinate
                        : current.coordinate);
    }

    bool removed_short_edge = true;
    while (removed_short_edge && snapped.size() > 4) {
        removed_short_edge = false;
        for (size_t index = 0; index < snapped.size(); ++index) {
            const size_t next = (index + 1) % snapped.size();
            // Removing one endpoint of a one-pixel orthogonal step joins its
            // neighbours diagonally. That invalidates the Manhattan polygon
            // and used to roll back every otherwise valid facade snap. Only
            // erase a truly collapsed (duplicate) vertex here; shallow steps
            // remain valid orthogonal geometry.
            if (snapped[next] == snapped[index]) {
                snapped.erase(
                        snapped.begin() + static_cast<long>(next));
                removed_short_edge = true;
                break;
            }
        }
    }
    // A collapsed one-pixel step leaves two collinear runs. Merge them so the
    // result remains a valid alternating orthogonal polygon.
    bool merged_collinear = true;
    while (merged_collinear && snapped.size() > 4) {
        merged_collinear = false;
        for (size_t index = 0; index < snapped.size(); ++index) {
            const cv::Point& previous = snapped[
                    (index + snapped.size() - 1) % snapped.size()];
            const cv::Point& current = snapped[index];
            const cv::Point& next = snapped[(index + 1) % snapped.size()];
            if ((previous.y == current.y && current.y == next.y) ||
                (previous.x == current.x && current.x == next.x)) {
                snapped.erase(
                        snapped.begin() + static_cast<long>(index));
                merged_collinear = true;
                break;
            }
        }
    }

    const double original_area = std::fabs(cv::contourArea(*polygon));
    const double snapped_area = std::fabs(cv::contourArea(snapped));
    if (original_area < 1.0 || snapped_area < 1.0 ||
        snapped_area / original_area < 0.70 ||
        snapped_area / original_area > 1.30) {
        std::cout << "[DEBUG] 外墙吸附拒绝: 面积比例="
                  << snapped_area / std::max(1.0, original_area) << "\n";
        return false;
    }
    if (snapped.size() < 4 || snapped.size() % 2 != 0) return false;
    *polygon = std::move(snapped);
    return true;
}

struct OutlineWallAlignmentQuality {
    double mean_distance_px =
            std::numeric_limits<double>::infinity();
    double p90_distance_px =
            std::numeric_limits<double>::infinity();
    double supported_ratio = 0.0;
};

// Measure the visible registration of a candidate polygon against the final
// fused occupied raster.  Distances are clipped so a doorway or an unobserved
// corner cannot dominate an otherwise well-supported long facade.  This is a
// presentation-space quality gate: it does not invent topology, it only
// decides whether a local wall-line refit is safer than the orthogonal prior.
OutlineWallAlignmentQuality EvaluateOutlineWallAlignment(
        const cv::Mat& observed_walls,
        const std::vector<cv::Point>& polygon,
        double meters_per_pixel) {
    OutlineWallAlignmentQuality quality;
    if (observed_walls.empty() || polygon.size() < 3) return quality;
    const double resolution =
            std::isfinite(meters_per_pixel) && meters_per_pixel > 1e-4
                    ? meters_per_pixel
                    : 0.05;
    const cv::Mat walls = Binary255(observed_walls);
    cv::Mat non_wall;
    cv::bitwise_not(walls, non_wall);
    cv::Mat distance;
    cv::distanceTransform(non_wall, distance, cv::DIST_L2, 3);

    const double support_distance = std::clamp(
            0.16 / resolution, 2.0, 5.0);
    const double clipped_distance = std::clamp(
            0.55 / resolution, 7.0, 18.0);
    std::vector<float> distances;
    for (size_t index = 0; index < polygon.size(); ++index) {
        const cv::Point2d start(polygon[index]);
        const cv::Point2d end(
                polygon[(index + 1) % polygon.size()]);
        const cv::Point2d delta = end - start;
        const double length = cv::norm(delta);
        if (length < 2.0) continue;
        const cv::Point2d direction = delta * (1.0 / length);
        const double inset = std::min(
                std::clamp(0.10 / resolution, 1.0, 4.0),
                length * 0.15);
        for (double along = inset; along <= length - inset;
             along += 1.5) {
            const cv::Point2d point = start + direction * along;
            const int x = std::clamp(
                    static_cast<int>(std::round(point.x)),
                    0,
                    distance.cols - 1);
            const int y = std::clamp(
                    static_cast<int>(std::round(point.y)),
                    0,
                    distance.rows - 1);
            distances.push_back(std::min(
                    distance.at<float>(y, x),
                    static_cast<float>(clipped_distance)));
        }
    }
    if (distances.empty()) return quality;
    double distance_sum = 0.0;
    int supported = 0;
    for (const float value : distances) {
        distance_sum += value;
        if (value <= support_distance) ++supported;
    }
    quality.mean_distance_px = distance_sum / distances.size();
    quality.supported_ratio = supported /
            static_cast<double>(distances.size());
    const size_t p90_index = std::min(
            distances.size() - 1,
            static_cast<size_t>(std::floor(
                    (distances.size() - 1) * 0.90)));
    std::nth_element(
            distances.begin(),
            distances.begin() + static_cast<long>(p90_index),
            distances.end());
    quality.p90_distance_px = distances[p90_index];
    return quality;
}

// Recover a room-sized orthogonal step that lies outside an otherwise valid
// footprint.  The topology mask can omit such a wing when its doorway is
// weak, even though the fused occupancy still contains the outer wall and
// both perpendicular returns.  Requiring that U-shaped wall evidence and an
// actual improvement in whole-outline registration keeps outdoor lidar rays
// and isolated furniture from changing the footprint.
bool RecoverExteriorWallBracketedStep(
        const cv::Mat& observed_walls,
        const cv::Mat& horizontal_walls,
        const cv::Mat& vertical_walls,
        double meters_per_pixel,
        std::vector<cv::Point>* polygon) {
    if (polygon == nullptr || polygon->size() < 4 ||
        observed_walls.empty() || horizontal_walls.empty() ||
        vertical_walls.empty()) {
        return false;
    }
    const double resolution = std::isfinite(meters_per_pixel) &&
            meters_per_pixel > 1e-4 ? meters_per_pixel : 0.05;
    const OutlineWallAlignmentQuality original_quality =
            EvaluateOutlineWallAlignment(
                    observed_walls, *polygon, resolution);
    // High-quality outlines should be stable across repeated scans.  Exterior
    // step recovery is a targeted fallback for the visibly under-fitted case.
    if (original_quality.supported_ratio >= 0.85 &&
        original_quality.p90_distance_px <= 0.25 / resolution) {
        return false;
    }

    const int minimum_depth = std::clamp(
            static_cast<int>(std::round(0.35 / resolution)), 5, 14);
    const int maximum_depth = std::clamp(
            static_cast<int>(std::round(2.40 / resolution)), 18, 72);
    const int normal_tolerance = std::clamp(
            static_cast<int>(std::round(0.10 / resolution)), 1, 4);
    const int tangent_gap = std::clamp(
            static_cast<int>(std::round(0.30 / resolution)), 3, 10);
    const int endpoint_search = std::clamp(
            static_cast<int>(std::round(0.35 / resolution)), 4, 12);
    const int minimum_run = std::clamp(
            static_cast<int>(std::round(0.80 / resolution)), 10, 32);

    struct Candidate {
        size_t edge_index = 0;
        bool horizontal = false;
        int outer_coordinate = 0;
        int tangent_begin = 0;
        int tangent_end = 0;
        double wall_score = 0.0;
        std::vector<cv::Point> polygon;
        OutlineWallAlignmentQuality quality;
    };
    Candidate best;
    bool found_candidate = false;

    auto pixel_near = [&](const cv::Mat& mask,
                          bool horizontal,
                          int coordinate,
                          int tangent) {
        for (int offset = -normal_tolerance;
             offset <= normal_tolerance; ++offset) {
            const int x = horizontal ? tangent : coordinate + offset;
            const int y = horizontal ? coordinate + offset : tangent;
            if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows &&
                mask.at<uchar>(y, x) != 0) {
                return true;
            }
        }
        return false;
    };
    auto perpendicular_support = [&](bool horizontal,
                                     int tangent,
                                     int inner_coordinate,
                                     int outer_coordinate) {
        double best_support = 0.0;
        for (int offset = -endpoint_search;
             offset <= endpoint_search; ++offset) {
            if (horizontal) {
                best_support = std::max(
                        best_support,
                        VerticalWallCoverage(
                                vertical_walls,
                                tangent + offset,
                                std::min(inner_coordinate, outer_coordinate),
                                std::max(inner_coordinate, outer_coordinate) + 1,
                                normal_tolerance));
            } else {
                best_support = std::max(
                        best_support,
                        HorizontalWallCoverage(
                                horizontal_walls,
                                tangent + offset,
                                std::min(inner_coordinate, outer_coordinate),
                                std::max(inner_coordinate, outer_coordinate) + 1,
                                normal_tolerance));
            }
        }
        return best_support;
    };

    for (size_t edge_index = 0; edge_index < polygon->size(); ++edge_index) {
        const cv::Point start = (*polygon)[edge_index];
        const cv::Point end = (*polygon)[(edge_index + 1) % polygon->size()];
        const bool horizontal = start.y == end.y;
        const bool vertical = start.x == end.x;
        if (!horizontal && !vertical) continue;
        const int inner_coordinate = horizontal ? start.y : start.x;
        const int tangent_min = horizontal
                ? std::min(start.x, end.x) : std::min(start.y, end.y);
        const int tangent_max = horizontal
                ? std::max(start.x, end.x) : std::max(start.y, end.y);
        if (tangent_max - tangent_min < minimum_run) continue;

        const cv::Point2f midpoint(
                (start.x + end.x) * 0.5f,
                (start.y + end.y) * 0.5f);
        cv::Point2f positive_probe = midpoint;
        if (horizontal) positive_probe.y += 2.f;
        else positive_probe.x += 2.f;
        const int interior_sign = cv::pointPolygonTest(
                *polygon, positive_probe, false) >= 0.0 ? 1 : -1;
        const int exterior_sign = -interior_sign;
        const cv::Mat& parallel_walls = horizontal
                ? horizontal_walls : vertical_walls;

        for (int depth = minimum_depth; depth <= maximum_depth; ++depth) {
            const int outer_coordinate =
                    inner_coordinate + exterior_sign * depth;
            if ((horizontal &&
                 (outer_coordinate < 0 || outer_coordinate >= observed_walls.rows)) ||
                (vertical &&
                 (outer_coordinate < 0 || outer_coordinate >= observed_walls.cols))) {
                continue;
            }
            int run_begin = -1;
            int last_supported = -1;
            int support_count = 0;
            auto evaluate_run = [&]() {
                if (run_begin < 0 || last_supported < run_begin) return;
                const int span = last_supported - run_begin + 1;
                if (span < minimum_run ||
                    support_count / static_cast<double>(span) < 0.34) {
                    return;
                }
                const double first_return = perpendicular_support(
                        horizontal, run_begin,
                        inner_coordinate, outer_coordinate);
                const double second_return = perpendicular_support(
                        horizontal, last_supported,
                        inner_coordinate, outer_coordinate);
                if (first_return < 0.28 || second_return < 0.28) return;

                const int ordered_first = horizontal
                        ? (start.x <= end.x ? run_begin : last_supported)
                        : (start.y <= end.y ? run_begin : last_supported);
                const int ordered_second = horizontal
                        ? (start.x <= end.x ? last_supported : run_begin)
                        : (start.y <= end.y ? last_supported : run_begin);
                std::vector<cv::Point> stepped;
                stepped.reserve(polygon->size() + 4);
                for (size_t index = 0; index < polygon->size(); ++index) {
                    stepped.push_back((*polygon)[index]);
                    if (index != edge_index) continue;
                    if (horizontal) {
                        stepped.emplace_back(ordered_first, inner_coordinate);
                        stepped.emplace_back(ordered_first, outer_coordinate);
                        stepped.emplace_back(ordered_second, outer_coordinate);
                        stepped.emplace_back(ordered_second, inner_coordinate);
                    } else {
                        stepped.emplace_back(inner_coordinate, ordered_first);
                        stepped.emplace_back(outer_coordinate, ordered_first);
                        stepped.emplace_back(outer_coordinate, ordered_second);
                        stepped.emplace_back(inner_coordinate, ordered_second);
                    }
                }
                stepped.erase(
                        std::unique(stepped.begin(), stepped.end()),
                        stepped.end());
                const double original_area = std::fabs(
                        cv::contourArea(*polygon));
                const double stepped_area = std::fabs(
                        cv::contourArea(stepped));
                if (stepped.size() < 8 || stepped.size() % 2 != 0 ||
                    stepped_area <= original_area ||
                    stepped_area / std::max(1.0, original_area) > 1.28) {
                    return;
                }
                const OutlineWallAlignmentQuality quality =
                        EvaluateOutlineWallAlignment(
                                observed_walls, stepped, resolution);
                const double improvement = quality.supported_ratio -
                        original_quality.supported_ratio;
                if (improvement < 0.012 ||
                    quality.mean_distance_px >
                            original_quality.mean_distance_px + 0.35 ||
                    quality.p90_distance_px >
                            original_quality.p90_distance_px + 1.0) {
                    return;
                }
                const double wall_score = improvement * 100.0 +
                        0.01 * span + first_return + second_return;
                if (!found_candidate || wall_score > best.wall_score) {
                    found_candidate = true;
                    best.edge_index = edge_index;
                    best.horizontal = horizontal;
                    best.outer_coordinate = outer_coordinate;
                    best.tangent_begin = run_begin;
                    best.tangent_end = last_supported;
                    best.wall_score = wall_score;
                    best.polygon = std::move(stepped);
                    best.quality = quality;
                }
            };

            for (int tangent = tangent_min; tangent <= tangent_max; ++tangent) {
                const bool supported = pixel_near(
                        parallel_walls,
                        horizontal,
                        outer_coordinate,
                        tangent);
                if (supported) {
                    if (run_begin < 0) run_begin = tangent;
                    last_supported = tangent;
                    ++support_count;
                } else if (run_begin >= 0 &&
                           tangent - last_supported > tangent_gap) {
                    evaluate_run();
                    run_begin = -1;
                    last_supported = -1;
                    support_count = 0;
                }
            }
            evaluate_run();
        }
    }

    if (!found_candidate) return false;
    std::cout << "[INFO] 墙体约束外轮廓台阶恢复 edge="
              << best.edge_index
              << " span=" << best.tangent_begin << "-"
              << best.tangent_end
              << " coordinate=" << best.outer_coordinate
              << " support=" << original_quality.supported_ratio
              << "->" << best.quality.supported_ratio << "\n";
    *polygon = std::move(best.polygon);
    return true;
}

// A closed rectangular room should not inherit small steps caused by wall
// thickness, door openings or free-space raster noise.  Only snap when all
// four sides have independent wall support; area rectangularity alone would
// incorrectly flatten a real L-shaped room or a corridor branch.
ClosedOutlineResult BuildTrajectoryConnectedOutline(
        const cv::Mat& wall_binary,
        const cv::Mat& observed_wall_binary,
        const cv::Mat& semantic_bgr,
        const std::vector<cv::Point2f>& trajectory_points,
        double meters_per_pixel,
        const std::string& debug_dir) {
    ClosedOutlineResult result;
    if (wall_binary.empty() || observed_wall_binary.empty() ||
        semantic_bgr.empty() ||
        wall_binary.size() != semantic_bgr.size() ||
        observed_wall_binary.size() != wall_binary.size() ||
        trajectory_points.empty()) {
        return result;
    }
    const double resolution = std::isfinite(meters_per_pixel) &&
            meters_per_pixel > 1e-4 ? meters_per_pixel : 0.05;

    cv::Mat free_mask;
    cv::inRange(
            semantic_bgr,
            cv::Scalar(245, 245, 245),
            cv::Scalar(255, 255, 255),
            free_mask);
    if (cv::countNonZero(free_mask) < 100 ||
        cv::countNonZero(wall_binary) < 80) return result;

    // The report is drawn on the final fused probability raster, so its
    // Manhattan frame must come from that same raster whenever the estimate
    // agrees with the denoised structural topology.  Using only the pruned
    // centreline can leave a several-degree residual yaw: the green polygon
    // is then perfectly orthogonal in its own frame but slowly drifts away
    // from a long black facade.  The structural estimate remains a guard
    // against detached outdoor rays dominating the visual Hough vote.
    const double structural_alignment_degrees =
            EstimateManhattanRotationDegrees(wall_binary);
    const double visual_alignment_degrees =
            EstimateManhattanRotationDegrees(observed_wall_binary);
    const double alignment_disagreement = std::fabs(
            structural_alignment_degrees - visual_alignment_degrees);
    const double wrapped_alignment_disagreement = std::min(
            alignment_disagreement,
            std::fabs(90.0 - alignment_disagreement));
    const bool visual_frame_is_consistent =
            cv::countNonZero(observed_wall_binary) >= 80 &&
            wrapped_alignment_disagreement <= 25.0;
    const double alignment_degrees = visual_frame_is_consistent
            ? visual_alignment_degrees
            : structural_alignment_degrees;
    std::cout << "[INFO] 统一点云方向 structural="
              << structural_alignment_degrees
              << " visual=" << visual_alignment_degrees
              << " selected=" << alignment_degrees
              << "\n";
    const cv::Point2f image_center(
            wall_binary.cols * 0.5f, wall_binary.rows * 0.5f);
    const cv::Mat rotation =
            cv::getRotationMatrix2D(image_center, alignment_degrees, 1.0);
    cv::Mat aligned_walls;
    cv::Mat aligned_observed_walls;
    cv::Mat aligned_free;
    cv::warpAffine(
            Binary255(wall_binary),
            aligned_walls,
            rotation,
            wall_binary.size(),
            cv::INTER_NEAREST,
            cv::BORDER_CONSTANT,
            cv::Scalar(0));
    cv::warpAffine(
            Binary255(observed_wall_binary),
            aligned_observed_walls,
            rotation,
            observed_wall_binary.size(),
            cv::INTER_NEAREST,
            cv::BORDER_CONSTANT,
            cv::Scalar(0));
    cv::warpAffine(
            free_mask,
            aligned_free,
            rotation,
            free_mask.size(),
            cv::INTER_NEAREST,
            cv::BORDER_CONSTANT,
            cv::Scalar(0));

    // Extract Manhattan wall strokes. Directional closing bridges door-sized
    // gaps. Do not dilate every stroke along its long axis: doing so expands a
    // closed room at all four corners and turns wall/door noise into false
    // exterior steps. An open corridor is capped where its two real parallel
    // side walls share reliable support.
    const int minimum_wall_run = std::clamp(
            static_cast<int>(std::round(0.35 / resolution)), 5, 21);
    const int door_gap = std::clamp(
            static_cast<int>(std::round(1.50 / resolution)), 7, 61);
    cv::Mat horizontal_walls;
    cv::Mat vertical_walls;
    cv::morphologyEx(
            aligned_walls,
            horizontal_walls,
            cv::MORPH_OPEN,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(minimum_wall_run, 1)));
    cv::morphologyEx(
            aligned_walls,
            vertical_walls,
            cv::MORPH_OPEN,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(1, minimum_wall_run)));
    cv::morphologyEx(
            horizontal_walls,
            horizontal_walls,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(door_gap, 3)));
    cv::morphologyEx(
            vertical_walls,
            vertical_walls,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(3, door_gap)));

    cv::Mat observed_horizontal_walls;
    cv::Mat observed_vertical_walls;
    cv::morphologyEx(
            aligned_observed_walls,
            observed_horizontal_walls,
            cv::MORPH_OPEN,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(minimum_wall_run, 1)));
    cv::morphologyEx(
            aligned_observed_walls,
            observed_vertical_walls,
            cv::MORPH_OPEN,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(1, minimum_wall_run)));
    cv::morphologyEx(
            observed_horizontal_walls,
            observed_horizontal_walls,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(door_gap, 3)));
    cv::morphologyEx(
            observed_vertical_walls,
            observed_vertical_walls,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(
                    cv::MORPH_RECT, cv::Size(3, door_gap)));
    cv::Mat fitted_horizontal_walls = FitDirectionalWallCenterlines(
            observed_horizontal_walls, true, resolution);
    cv::Mat fitted_vertical_walls = FitDirectionalWallCenterlines(
            observed_vertical_walls, false, resolution);
    if (cv::countNonZero(fitted_horizontal_walls) < 20) {
        fitted_horizontal_walls = observed_horizontal_walls.clone();
    }
    if (cv::countNonZero(fitted_vertical_walls) < 20) {
        fitted_vertical_walls = observed_vertical_walls.clone();
    }
    // Room topology must follow the black Cartographer wall raster and the
    // branch-independent structural source, not a branch-specific pruned
    // skeleton.  Fitted visual centrelines restore weak/thick wall faces while
    // avoiding raw directional clutter becoming the nearest wall of a room.
    // Semantic free-space and trajectory connectivity below still prevent
    // detached outdoor returns from becoming part of the green footprint.
    cv::bitwise_or(
            horizontal_walls,
            fitted_horizontal_walls,
            horizontal_walls);
    cv::bitwise_or(
            vertical_walls,
            fitted_vertical_walls,
            vertical_walls);
    cv::Mat fitted_observed_walls;
    cv::bitwise_or(
            fitted_horizontal_walls,
            fitted_vertical_walls,
            fitted_observed_walls);
    const cv::Mat fitted_exterior_wall_evidence = BuildExteriorWallEvidence(
            fitted_observed_walls,
            fitted_horizontal_walls,
            fitted_vertical_walls,
            aligned_free,
            resolution);
    const cv::Mat observed_exterior_wall_evidence =
            BuildExteriorWallEvidence(
                    aligned_observed_walls,
                    observed_horizontal_walls,
                    observed_vertical_walls,
                    aligned_free,
                    resolution);
    cv::Mat exterior_wall_evidence;
    cv::bitwise_or(
            fitted_exterior_wall_evidence,
            observed_exterior_wall_evidence,
            exterior_wall_evidence);

    const int minimum_span = std::max(
            4, static_cast<int>(std::round(0.55 / resolution)));
    const int maximum_span = std::max(
            minimum_span + 1,
            static_cast<int>(std::round(20.0 / resolution)));
    cv::Mat between_vertical_pair =
            cv::Mat::zeros(aligned_walls.size(), CV_8UC1);
    cv::Mat between_horizontal_pair =
            cv::Mat::zeros(aligned_walls.size(), CV_8UC1);

    // Between a left/right pair of vertical walls.
    std::vector<int> nearest_left(aligned_walls.cols, -1);
    std::vector<int> nearest_right(aligned_walls.cols, -1);
    for (int y = 0; y < aligned_walls.rows; ++y) {
        int last = -1;
        const uchar* wall_row = vertical_walls.ptr<uchar>(y);
        for (int x = 0; x < aligned_walls.cols; ++x) {
            if (wall_row[x] != 0) last = x;
            nearest_left[x] = last;
        }
        last = -1;
        for (int x = aligned_walls.cols - 1; x >= 0; --x) {
            if (wall_row[x] != 0) last = x;
            nearest_right[x] = last;
        }
        const uchar* free_row = aligned_free.ptr<uchar>(y);
        uchar* output_row = between_vertical_pair.ptr<uchar>(y);
        for (int x = 0; x < aligned_walls.cols; ++x) {
            if (free_row[x] == 0 || nearest_left[x] < 0 ||
                nearest_right[x] < 0) continue;
            const int span = nearest_right[x] - nearest_left[x];
            if (span >= minimum_span && span <= maximum_span) {
                output_row[x] = 255;
            }
        }
    }

    // Between an upper/lower pair of horizontal walls.
    std::vector<int> nearest_top(aligned_walls.rows, -1);
    std::vector<int> nearest_bottom(aligned_walls.rows, -1);
    for (int x = 0; x < aligned_walls.cols; ++x) {
        int last = -1;
        for (int y = 0; y < aligned_walls.rows; ++y) {
            if (horizontal_walls.at<uchar>(y, x) != 0) last = y;
            nearest_top[y] = last;
        }
        last = -1;
        for (int y = aligned_walls.rows - 1; y >= 0; --y) {
            if (horizontal_walls.at<uchar>(y, x) != 0) last = y;
            nearest_bottom[y] = last;
        }
        for (int y = 0; y < aligned_walls.rows; ++y) {
            if (aligned_free.at<uchar>(y, x) == 0 ||
                nearest_top[y] < 0 || nearest_bottom[y] < 0) continue;
            const int span = nearest_bottom[y] - nearest_top[y];
            if (span >= minimum_span && span <= maximum_span) {
                between_horizontal_pair.at<uchar>(y, x) = 255;
            }
        }
    }

    cv::Mat between_parallel;
    cv::bitwise_or(
            between_vertical_pair,
            between_horizontal_pair,
            between_parallel);

    cv::Mat trajectory_mask =
            cv::Mat::zeros(aligned_walls.size(), CV_8UC1);
    std::vector<cv::Point2f> aligned_trajectory;
    cv::transform(trajectory_points, aligned_trajectory, rotation);
    const int trajectory_radius = std::clamp(
            static_cast<int>(std::round(0.45 / resolution)), 4, 18);
    for (const auto& point : aligned_trajectory) {
        const int x = static_cast<int>(std::round(point.x));
        const int y = static_cast<int>(std::round(point.y));
        if (x >= 0 && x < trajectory_mask.cols &&
            y >= 0 && y < trajectory_mask.rows) {
            cv::circle(
                    trajectory_mask,
                    cv::Point(x, y),
                    trajectory_radius,
                    cv::Scalar(255),
                    cv::FILLED,
                    cv::LINE_8);
        }
    }

    // A room does not have to contain a trajectory node to belong to the
    // scanned floor plan. Select every raw free-space component reachable
    // from the driven trajectory, then use that topology when choosing the
    // more conservative between-parallel-walls components below. This keeps
    // an observed room corner from disappearing merely because the robot
    // scanned it through a doorway instead of driving into it.
    cv::Mat free_labels;
    cv::Mat free_stats;
    cv::Mat free_centroids;
    const int free_component_count = cv::connectedComponentsWithStats(
            aligned_free,
            free_labels,
            free_stats,
            free_centroids,
            8,
            CV_32S);
    std::vector<uchar> reachable_free_labels(
            static_cast<size_t>(free_component_count), 0);
    for (int y = 0; y < free_labels.rows; ++y) {
        const int* label_row = free_labels.ptr<int>(y);
        const uchar* trajectory_row = trajectory_mask.ptr<uchar>(y);
        for (int x = 0; x < free_labels.cols; ++x) {
            if (trajectory_row[x] == 0) continue;
            const int label = label_row[x];
            if (label > 0 && label < free_component_count) {
                reachable_free_labels[static_cast<size_t>(label)] = 1;
            }
        }
    }
    cv::Mat reachable_free = cv::Mat::zeros(aligned_free.size(), CV_8UC1);
    for (int y = 0; y < free_labels.rows; ++y) {
        const int* label_row = free_labels.ptr<int>(y);
        uchar* reachable_row = reachable_free.ptr<uchar>(y);
        for (int x = 0; x < free_labels.cols; ++x) {
            const int label = label_row[x];
            if (label > 0 &&
                reachable_free_labels[static_cast<size_t>(label)] != 0) {
                reachable_row[x] = 255;
            }
        }
    }
    if (cv::countNonZero(reachable_free) == 0) {
        // Preserve the former direct-touch behavior for sparse/legacy
        // semantic exports whose pose samples do not land on a white cell.
        reachable_free = trajectory_mask.clone();
    }

    cv::Mat navigable;
    cv::bitwise_and(between_parallel, aligned_free, navigable);
    const int connectivity_gap = std::clamp(
            static_cast<int>(std::round(0.45 / resolution)), 5, 21);
    cv::morphologyEx(
            navigable,
            navigable,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(connectivity_gap | 1, connectivity_gap | 1)));

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int component_count = cv::connectedComponentsWithStats(
            navigable, labels, stats, centroids, 8, CV_32S);
    cv::Mat selected = cv::Mat::zeros(navigable.size(), CV_8UC1);
    const int minimum_component_area = std::max(
            80, static_cast<int>(std::round(0.40 /
                    (resolution * resolution))));
    int selected_components = 0;
    std::vector<uchar> selected_labels(
            static_cast<size_t>(component_count), 0);
    for (int label = 1; label < component_count; ++label) {
        if (stats.at<int>(label, cv::CC_STAT_AREA) <
            minimum_component_area) continue;
        cv::Mat component = labels == label;
        cv::Mat touched;
        cv::bitwise_and(component, reachable_free, touched);
        if (cv::countNonZero(touched) == 0) continue;
        selected.setTo(255, component);
        selected_labels[static_cast<size_t>(label)] = 1;
        ++selected_components;
    }
    if (selected_components == 0) return result;
    // Add broad room faces that the between-parallel test missed, but first
    // remove narrow lidar spokes. This restores rooms scanned through a door
    // without admitting the long, thin free-space rays outside the building.
    cv::Mat free_distance;
    cv::distanceTransform(aligned_free, free_distance, cv::DIST_L2, 3);
    const double minimum_free_radius = std::clamp(
            0.20 / resolution, 3.0, 8.0);
    cv::Mat broad_free_core;
    cv::compare(
            free_distance,
            minimum_free_radius,
            broad_free_core,
            cv::CMP_GE);
    const int reconstruct_radius = std::clamp(
            static_cast<int>(std::ceil(minimum_free_radius)), 3, 8);
    cv::Mat broad_free;
    cv::dilate(
            broad_free_core,
            broad_free,
            cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(
                            reconstruct_radius * 2 + 1,
                            reconstruct_radius * 2 + 1)));
    cv::bitwise_and(broad_free, aligned_free, broad_free);

    cv::Mat dominant_reachable_free;
    cv::bitwise_and(broad_free, reachable_free, dominant_reachable_free);
    const int dominant_join_radius = std::clamp(
            static_cast<int>(std::round(0.22 / resolution)), 2, 7);
    cv::morphologyEx(
            dominant_reachable_free,
            dominant_reachable_free,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(dominant_join_radius * 2 + 1,
                             dominant_join_radius * 2 + 1)));
    cv::bitwise_and(
            dominant_reachable_free,
            reachable_free,
            dominant_reachable_free);

    // Do not union the complete trajectory-reachable free component here.
    // Outdoor lidar returns form broad, connected fans near the building and
    // would otherwise become part of the green footprint.  The recovery step
    // below admits only broad free space close to a wall-bracketed room seed.
    cv::Mat inverse_selected;
    cv::bitwise_not(selected, inverse_selected);
    cv::Mat distance_to_selected;
    cv::distanceTransform(
            inverse_selected,
            distance_to_selected,
            cv::DIST_L2,
            3);
    // This is only a seam-recovery allowance around an already
    // wall-bracketed room seed.  A wider band crosses facade gaps and turns
    // nearby outdoor lidar fans into indoor footprint (most visibly beside
    // long exterior walls).  Doorway-to-room transitions are connected by
    // the trajectory-aware step below, so keep this isotropic expansion
    // deliberately smaller than a normal doorway.
    const double room_recovery_distance = std::clamp(
            0.30 / resolution, 4.0, 16.0);
    cv::Mat near_selected;
    cv::compare(
            distance_to_selected,
            room_recovery_distance,
            near_selected,
            cv::CMP_LE);
    cv::Mat recovered_room_faces;
    cv::bitwise_and(broad_free, reachable_free, recovered_room_faces);
    cv::bitwise_and(recovered_room_faces, near_selected, recovered_room_faces);
    cv::bitwise_or(selected, recovered_room_faces, selected);
    // Candidate scoring must measure retained indoor space, not the complete
    // reachable semantic component: that component also contains outdoor
    // lidar fans connected through doors and other openings.
    const cv::Mat trusted_room_free = selected.clone();

    // Join only a confirmed transition between two trajectory-touched free
    // components. This repairs a doorway/semantic seam without growing a
    // corridor along the complete trajectory or inventing unvisited rooms.
    const int label_search_radius = std::clamp(
            static_cast<int>(std::round(0.35 / resolution)), 3, 14);
    auto nearest_selected_label = [&](const cv::Point2f& point) {
        const int center_x = static_cast<int>(std::round(point.x));
        const int center_y = static_cast<int>(std::round(point.y));
        int best_label = 0;
        int best_distance_squared = std::numeric_limits<int>::max();
        for (int dy = -label_search_radius; dy <= label_search_radius; ++dy) {
            const int y = center_y + dy;
            if (y < 0 || y >= labels.rows) continue;
            for (int dx = -label_search_radius; dx <= label_search_radius; ++dx) {
                const int distance_squared = dx * dx + dy * dy;
                if (distance_squared > label_search_radius * label_search_radius ||
                    distance_squared >= best_distance_squared) continue;
                const int x = center_x + dx;
                if (x < 0 || x >= labels.cols) continue;
                const int label = labels.at<int>(y, x);
                if (label <= 0 || label >= component_count ||
                    selected_labels[static_cast<size_t>(label)] == 0) continue;
                best_label = label;
                best_distance_squared = distance_squared;
            }
        }
        return best_label;
    };
    const double maximum_transition_step =
            std::clamp(1.20 / resolution, 12.0, 48.0);
    const int connector_thickness = std::clamp(
            static_cast<int>(std::round(0.28 / resolution)), 3, 12);
    int connected_component_transitions = 0;
    int previous_label = 0;
    cv::Point2f previous_point;
    for (const auto& point : aligned_trajectory) {
        const int label = nearest_selected_label(point);
        if (label > 0 && previous_label > 0 && label != previous_label &&
            cv::norm(point - previous_point) <= maximum_transition_step) {
            const cv::Vec4i transition(
                    static_cast<int>(std::round(previous_point.x)),
                    static_cast<int>(std::round(previous_point.y)),
                    static_cast<int>(std::round(point.x)),
                    static_cast<int>(std::round(point.y)));
            const double crossing_wall_support = SegmentSupportRatio(
                    aligned_walls,
                    transition,
                    std::clamp(
                            static_cast<int>(std::round(0.08 / resolution)),
                            2,
                            5));
            if (crossing_wall_support < 0.35) {
                cv::circle(
                        selected,
                        cv::Point(transition[0], transition[1]),
                        label_search_radius,
                        cv::Scalar(255),
                        cv::FILLED,
                        cv::LINE_8);
                cv::line(
                        selected,
                        cv::Point(transition[0], transition[1]),
                        cv::Point(transition[2], transition[3]),
                        cv::Scalar(255),
                        connector_thickness,
                        cv::LINE_8);
                cv::circle(
                        selected,
                        cv::Point(transition[2], transition[3]),
                        label_search_radius,
                        cv::Scalar(255),
                        cv::FILLED,
                        cv::LINE_8);
                ++connected_component_transitions;
            }
        }
        if (label > 0) {
            previous_label = label;
            previous_point = point;
        }
    }

    const int union_gap = std::clamp(
            static_cast<int>(std::round(0.65 / resolution)), 5, 27);
    cv::morphologyEx(
            selected,
            selected,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(union_gap | 1, union_gap | 1)));
    // Only bridge the remaining raster gap to the observed wall. A larger
    // dilation puts the contour on the outside face of thick/noisy walls.
    const int wall_margin = std::clamp(
            static_cast<int>(std::round(0.06 / resolution)), 1, 3);
    cv::dilate(
            selected,
            selected,
            cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(wall_margin * 2 + 1, wall_margin * 2 + 1)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(
            selected.clone(),
            contours,
            cv::RETR_EXTERNAL,
            cv::CHAIN_APPROX_NONE);
    if (contours.empty()) return result;
    size_t best_index = 0;
    double best_area = 0.0;
    for (size_t index = 0; index < contours.size(); ++index) {
        const double area = std::fabs(cv::contourArea(contours[index]));
        if (area > best_area) {
            best_area = area;
            best_index = index;
        }
    }
    if (best_area < minimum_component_area) return result;
    std::vector<cv::Point> aligned_polygon =
            OrthogonalizeContour(contours[best_index], resolution);
    if (aligned_polygon.size() < 4 || aligned_polygon.size() > 96) {
        return result;
    }
    const bool snapped_to_walls = SnapOrthogonalOutlineToWalls(
            aligned_observed_walls,
            exterior_wall_evidence,
            resolution,
            &aligned_polygon);
    const bool recovered_exterior_step = RecoverExteriorWallBracketedStep(
            aligned_observed_walls,
            observed_horizontal_walls,
            observed_vertical_walls,
            resolution,
            &aligned_polygon);
    // The connected broad-free-space mask already encodes the competitor's
    // room topology. Keep that single topology decision and only register its
    // edges to observed exterior walls. The former chain of recess filling,
    // detour suppression, rectangle recovery and appendage recovery allowed
    // later heuristics to undo earlier ones and made similar scans diverge.
    cv::Mat aligned_footprint = cv::Mat::zeros(
            dominant_reachable_free.size(), CV_8UC1);
    cv::fillPoly(
            aligned_footprint,
            std::vector<std::vector<cv::Point>>{aligned_polygon},
            cv::Scalar(255));
    cv::Mat contained_dominant_free;
    cv::bitwise_and(
            trusted_room_free,
            aligned_footprint,
            contained_dominant_free);
    const double free_space_containment_ratio =
            cv::countNonZero(contained_dominant_free) /
            std::max(1.0, static_cast<double>(
                    cv::countNonZero(trusted_room_free)));

    cv::Mat inverse_rotation;
    cv::invertAffineTransform(rotation, inverse_rotation);
    std::vector<cv::Point2f> aligned_float;
    aligned_float.reserve(aligned_polygon.size());
    for (const auto& point : aligned_polygon) {
        aligned_float.emplace_back(point);
    }
    std::vector<cv::Point2f> source_float;
    cv::transform(aligned_float, source_float, inverse_rotation);
    std::vector<cv::Point> source_polygon;
    source_polygon.reserve(source_float.size());
    for (const auto& point : source_float) {
        source_polygon.emplace_back(
                std::clamp(
                        static_cast<int>(std::round(point.x)),
                        0,
                        wall_binary.cols - 1),
                std::clamp(
                        static_cast<int>(std::round(point.y)),
                        0,
                        wall_binary.rows - 1));
    }

    // Keep the competitor-style Manhattan topology strict. The aligned-space
    // snap above already moves every horizontal/vertical edge onto the final
    // fused wall raster. A source-space free-angle refit may reduce point
    // distance by a fraction of a pixel, but it also reintroduces several
    // different local wall angles and makes the final report look skewed.
    // Measure registration here for diagnostics only; never change an edge's
    // direction after the common Manhattan frame has been selected.
    const OutlineWallAlignmentQuality alignment_before =
            EvaluateOutlineWallAlignment(
                    observed_wall_binary,
                    source_polygon,
                    resolution);
    std::cout << "[INFO] 正交外轮廓点云注册 mean="
              << alignment_before.mean_distance_px
              << " p90=" << alignment_before.p90_distance_px
              << " support=" << alignment_before.supported_ratio
              << "\n";
    if (source_polygon.size() < 4 ||
        std::fabs(cv::contourArea(source_polygon)) <
                0.50 / (resolution * resolution)) {
        return result;
    }

    int included_trajectory = 0;
    int valid_trajectory = 0;
    const double trajectory_tolerance = std::max(2.0, 0.35 / resolution);
    for (const auto& point : trajectory_points) {
        if (point.x < 0 || point.y < 0 ||
            point.x >= wall_binary.cols || point.y >= wall_binary.rows) continue;
        ++valid_trajectory;
        if (cv::pointPolygonTest(source_polygon, point, true) >=
            -trajectory_tolerance) {
            ++included_trajectory;
        }
    }
    const double trajectory_support = included_trajectory /
            std::max(1.0, static_cast<double>(valid_trajectory));
    if (trajectory_support < 0.55) return result;

    const cv::RotatedRect box = cv::minAreaRect(source_polygon);
    cv::Point2f box_points[4];
    box.points(box_points);
    double longest = 0.0;
    double shortest = std::numeric_limits<double>::infinity();
    cv::Point2f long_axis(1.f, 0.f);
    for (int index = 0; index < 4; ++index) {
        const cv::Point2f delta =
                box_points[(index + 1) % 4] - box_points[index];
        const double length = cv::norm(delta);
        if (length > longest && length > 1e-6) {
            longest = length;
            long_axis = delta * static_cast<float>(1.0 / length);
        }
        if (length > 1e-6) shortest = std::min(shortest, length);
    }
    if (long_axis.x < -1e-6f ||
        (std::fabs(long_axis.x) <= 1e-6f && long_axis.y < 0.f)) {
        long_axis *= -1.f;
    }
    const cv::Point2f short_axis(-long_axis.y, long_axis.x);
    result.valid = true;
    result.original_polygon = std::move(source_polygon);
    result.vertex_count = static_cast<int>(result.original_polygon.size());
    result.close_size = door_gap;
    result.support_ratio = trajectory_support;
    result.free_space_containment_ratio = free_space_containment_ratio;
    result.rotation_degrees =
            std::atan2(long_axis.y, long_axis.x) * 180.0 / kPi;
    result.dimension_center = box.center;
    result.long_axis = long_axis;
    result.short_axis = short_axis;
    result.long_size_px = longest;
    result.short_size_px = shortest;
    result.footprint_area_px2 =
            std::fabs(cv::contourArea(result.original_polygon));
    result.footprint_perimeter_px =
            cv::arcLength(result.original_polygon, true);

    if (!debug_dir.empty()) {
        EnsureDir(debug_dir);
        cv::imwrite(PathJoin(debug_dir, "topology_free.png"), free_mask);
        cv::imwrite(PathJoin(debug_dir, "topology_aligned_walls.png"), aligned_walls);
        cv::imwrite(PathJoin(debug_dir, "topology_aligned_observed_walls.png"), aligned_observed_walls);
        cv::imwrite(PathJoin(debug_dir, "topology_fitted_wall_evidence.png"), fitted_observed_walls);
        cv::imwrite(PathJoin(debug_dir, "topology_horizontal_walls.png"), horizontal_walls);
        cv::imwrite(PathJoin(debug_dir, "topology_vertical_walls.png"), vertical_walls);
        cv::imwrite(PathJoin(debug_dir, "topology_exterior_walls.png"), exterior_wall_evidence);
        cv::imwrite(PathJoin(debug_dir, "topology_between_parallel.png"), between_parallel);
        cv::imwrite(
                PathJoin(debug_dir, "topology_between_vertical_pair.png"),
                between_vertical_pair);
        cv::imwrite(
                PathJoin(debug_dir, "topology_between_horizontal_pair.png"),
                between_horizontal_pair);
        cv::imwrite(PathJoin(debug_dir, "topology_trajectory.png"), trajectory_mask);
        cv::imwrite(PathJoin(debug_dir, "topology_reachable_free.png"), reachable_free);
        cv::imwrite(PathJoin(debug_dir, "topology_broad_free.png"), broad_free);
        cv::imwrite(
                PathJoin(debug_dir, "topology_dominant_reachable_free.png"),
                dominant_reachable_free);
        cv::imwrite(PathJoin(debug_dir, "topology_selected.png"), selected);
        cv::Mat aligned_debug;
        cv::cvtColor(Binary255(fitted_observed_walls), aligned_debug, cv::COLOR_GRAY2BGR);
        cv::polylines(
                aligned_debug,
                std::vector<std::vector<cv::Point>>{aligned_polygon},
                true,
                cv::Scalar(0, 255, 0),
                2,
                cv::LINE_AA);
        cv::imwrite(PathJoin(debug_dir, "topology_aligned_outline.png"), aligned_debug);
        cv::Mat source_debug;
        cv::cvtColor(Binary255(observed_wall_binary), source_debug, cv::COLOR_GRAY2BGR);
        cv::polylines(
                source_debug,
                std::vector<std::vector<cv::Point>>{result.original_polygon},
                true,
                cv::Scalar(0, 255, 0),
                2,
                cv::LINE_AA);
        cv::imwrite(PathJoin(debug_dir, "topology_source_outline.png"), source_debug);
        cv::Mat visual_registration_debug;
        cv::cvtColor(
                Binary255(observed_wall_binary),
                visual_registration_debug,
                cv::COLOR_GRAY2BGR);
        cv::polylines(
                visual_registration_debug,
                std::vector<std::vector<cv::Point>>{
                        result.original_polygon},
                true,
                cv::Scalar(0, 255, 0),
                2,
                cv::LINE_AA);
        cv::imwrite(
                PathJoin(
                        debug_dir,
                        "topology_visual_registered_outline.png"),
                visual_registration_debug);
    }
    std::cout << "[INFO] 轨迹连通主体完成 components="
              << selected_components
              << " vertices=" << result.vertex_count
              << " trajectory_support=" << trajectory_support
              << " free_containment=" << free_space_containment_ratio
              << " wall_snapped=" << snapped_to_walls
              << " exterior_step_recovered=" << recovered_exterior_step
              << " component_transitions="
              << connected_component_transitions
              << " rotation=" << alignment_degrees << "\n";
    return result;
}


}  // namespace

static cv::Mat ProcessMapImage(
        const cv::Mat& input,
        int thresh,
        int min_branch_length,
        bool restore) {
    if (input.empty()) throw std::runtime_error("Input image is empty");
    cv::Mat binary = BinarizeMap(input, thresh, true);
    cv::Mat pruned = SkeletonPruning(binary, min_branch_length);
    return restore ? RestoreThickness(pruned) : pruned;
}

static PipelineResult FitFloorPlan(const std::string& clean_map_path,
                                   const std::string& original_map_path,
                                   const std::string& internal_structure_map_path,
                                   const std::string& output_path,
                                   const std::string& debug_dir,
                                   double meters_per_pixel,
                                   const std::string& semantic_map_path,
                                   const std::vector<cv::Point2f>& trajectory_points_px) {
    EnsureDir(debug_dir);
    cv::Mat img = cv::imread(clean_map_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) throw std::runtime_error("Missing image at " + clean_map_path);
    // The visual occupancy map is also the final report canvas. Keep it in
    // BGR so green exterior and red interior annotations remain colored;
    // loading it as grayscale silently collapsed every annotation to black.
    cv::Mat original_map = cv::imread(original_map_path, cv::IMREAD_COLOR);
    if (original_map.empty()) throw std::runtime_error("Missing image at " + original_map_path);
    cv::Mat internal_structure_map = cv::imread(
            internal_structure_map_path,
            cv::IMREAD_GRAYSCALE);
    if (internal_structure_map.empty()) {
        throw std::runtime_error(
                "Missing internal-structure image at " +
                internal_structure_map_path);
    }
    if (internal_structure_map.size() != original_map.size()) {
        throw std::runtime_error(
                "Internal-structure and visual map sizes do not match");
    }
    cv::Mat semantic_map = cv::imread(
            semantic_map_path,
            cv::IMREAD_COLOR);
    if (semantic_map.empty()) {
        throw std::runtime_error("Missing semantic image at " + semantic_map_path);
    }
    if (semantic_map.size() != original_map.size()) {
        throw std::runtime_error("Semantic and visual map sizes do not match");
    }

    // Exterior fitting must use the same wall evidence the user sees.  The
    // algorithm input is deliberately denoised and skeletonized for stable
    // topology; its one-pixel centreline can be displaced from the continuous
    // Cartographer probability raster by several pixels.  Extract dark wall
    // returns from the final visual raster (gray unknown is safely above this
    // threshold) and use them only for facade position snapping.  Internal
    // wall detection continues to use the cleaner structural input.
    cv::Mat visual_gray;
    cv::cvtColor(original_map, visual_gray, cv::COLOR_BGR2GRAY);
    cv::Mat visual_wall_binary;
    cv::threshold(
            visual_gray,
            visual_wall_binary,
            125,
            255,
            cv::THRESH_BINARY_INV);
    cv::morphologyEx(
            visual_wall_binary,
            visual_wall_binary,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(
                    cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    cv::imwrite(
            PathJoin(debug_dir, "visual_wall_evidence.png"),
            visual_wall_binary);

    // Keep a full-width, gap-closed copy of the structural evidence. The
    // pruning branch below is useful for line fitting, but it must not define
    // the outside edge of the building: skeletonization can move that edge
    // onto the indoor face of a thick wall.
    cv::Mat internal_structure_binary =
            BinarizeMap(internal_structure_map, 200, true);
    cv::morphologyEx(
            internal_structure_binary,
            internal_structure_binary,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    cv::Mat binary;
    cv::threshold(img, binary, 127, 255, cv::THRESH_BINARY);
    const double safe_resolution =
            std::isfinite(meters_per_pixel) && meters_per_pixel > 1e-4
                    ? meters_per_pixel
                    : 0.05;
    // Every geometric threshold below represents a physical distance. The
    // previous 12/30/44-pixel constants changed meaning whenever export
    // resolution changed and could bridge more than two metres at 5 cm/px.
    const double hough_minimum_length = std::clamp(
            0.25 / safe_resolution, 5.0, 18.0);
    const double hough_maximum_gap = std::clamp(
            0.16 / safe_resolution, 2.0, 8.0);
    const int hough_votes = std::clamp(
            static_cast<int>(std::round(hough_minimum_length * 0.8)),
            6,
            16);
    const double collinear_merge_gap = std::clamp(
            0.35 / safe_resolution, 4.0, 14.0);
    const double corner_snap_gap = std::clamp(
            0.35 / safe_resolution, 5.0, 14.0);
    const double corner_maximum_move = std::clamp(
            0.25 / safe_resolution, 4.0, 10.0);
    const double corner_merge_distance = std::clamp(
            0.16 / safe_resolution, 2.0, 7.0);
    const double short_line_threshold = std::clamp(
            0.45 / safe_resolution, 7.0, 20.0);
    const double connected_minimum_length = std::clamp(
            0.25 / safe_resolution, 4.0, 12.0);
    const double endpoint_connect_distance = std::clamp(
            0.25 / safe_resolution, 4.0, 10.0);

    ClutterInfo clutter;
    const bool has_inner_clutter = DetectInternalClutter(binary, &clutter);
    cv::imwrite(PathJoin(debug_dir, "inner_clutter_mask.png"), clutter.mask);
    std::cout << "[DEBUG] 内部杂线检测: "
              << "has_clutter=" << (has_inner_clutter ? "True" : "False")
              << ", pixels=" << clutter.inner_pixels
              << ", components=" << clutter.component_count
              << ", ratio=" << std::fixed << std::setprecision(4) << clutter.inner_ratio
              << ", close_size="
              << (clutter.close_size == 0 ? std::string("None") : std::to_string(clutter.close_size))
              << ", outer_fill=" << std::setprecision(3) << clutter.outer_fill_ratio
              << std::defaultfloat << "\n";
    cv::Mat wall_binary = has_inner_clutter ? RemoveInternalClutter(binary, clutter.mask) : binary.clone();
    if (has_inner_clutter) {
        std::cout << "[DEBUG] 内部杂线清除: "
                  << cv::countNonZero(binary) << " -> " << cv::countNonZero(wall_binary)
                  << " 像素\n";
    } else {
        std::cout << "[DEBUG] 未检测到明显内部杂线，跳过外围墙清理\n";
    }
    cv::imwrite(PathJoin(debug_dir, "outer_wall_cleaned.png"), wall_binary);

    auto diagonal_result = RemoveAttachedDiagonalClutter(wall_binary);
    wall_binary = diagonal_result.first;
    std::cout << "[DEBUG] 贴墙斜线清理: 删除 " << diagonal_result.second.size() << " 条\n";

    std::vector<cv::Vec4i> raw_segs;
    cv::HoughLinesP(
            wall_binary,
            raw_segs,
            1,
            kPi / 180.0,
            hough_votes,
            hough_minimum_length,
            hough_maximum_gap);
    std::cout << "霍夫检测线段: " << raw_segs.size() << " 条\n";

    std::vector<cv::Vec4i> final_green;
    for (const auto& group : ClusterByOrientation(raw_segs)) {
        auto merged = MergeColinearGroup(
                group,
                collinear_merge_gap,
                hough_minimum_length).first;
        final_green.insert(final_green.end(), merged.begin(), merged.end());
    }

    final_green = DeduplicateSegmentsPythonSetOrder(final_green);

    const size_t before_snap = final_green.size();
    final_green = SnapCorners(
            final_green,
            corner_snap_gap,
            corner_maximum_move,
            corner_merge_distance,
            hough_minimum_length);
    const size_t after_snap = final_green.size();
    std::cout << "[DEBUG] snap_corners: " << before_snap << " -> " << after_snap
              << " (删除了 " << (before_snap - after_snap) << " 条)\n";
    const size_t before_short = final_green.size();
    const std::vector<cv::Vec4i> before_short_filter = final_green;
    final_green.erase(std::remove_if(final_green.begin(), final_green.end(),
                                     [&](const cv::Vec4i& seg) {
                                         return !KeepAfterShortFilter(
                                                 seg,
                                                 before_short_filter,
                                                 short_line_threshold,
                                                 connected_minimum_length,
                                                 endpoint_connect_distance);
                                     }),
                      final_green.end());
    const size_t after_short = final_green.size();
    std::cout << "[DEBUG] 短线过滤: " << before_short << " -> " << after_short
              << " (删除了 " << (before_short - after_short) << " 条)\n";

    auto red_filter = GenerateRedLinesAndFilters(final_green);
    std::vector<cv::Vec4i> final_red = red_filter.first;
    const auto& to_remove = red_filter.second;
    std::cout << "[DEBUG] generate_red_lines_and_filters 标记删除: " << to_remove.size() << " 条\n";

    const size_t before_remove = final_green.size();
    final_green.erase(std::remove_if(final_green.begin(), final_green.end(), [&](const cv::Vec4i& seg) {
        return to_remove.count({seg[0], seg[1], seg[2], seg[3]}) > 0;
    }), final_green.end());
    const size_t after_remove = final_green.size();
    std::cout << "[DEBUG] 删除多余线段: " << before_remove << " -> " << after_remove
              << " (删除了 " << (before_remove - after_remove) << " 条)\n";

    cv::Mat structural_mask = cv::Mat::zeros(wall_binary.size(), CV_8UC1);
    const int structural_thickness = std::clamp(
            static_cast<int>(std::round(0.12 / safe_resolution)), 3, 7);
    for (const auto& seg : final_green) {
        cv::line(structural_mask, {seg[0], seg[1]}, {seg[2], seg[3]},
                 cv::Scalar(255), structural_thickness, cv::LINE_8);
    }
    for (const auto& seg : final_red) {
        cv::line(structural_mask, {seg[0], seg[1]}, {seg[2], seg[3]},
                 cv::Scalar(255), structural_thickness, cv::LINE_8);
    }
    std::vector<cv::Vec4i> structural_segments = final_green;
    structural_segments.insert(
            structural_segments.end(), final_red.begin(), final_red.end());
    const double max_corner_extension = std::clamp(
            2.60 / safe_resolution, 18.0, 65.0);
    const double max_collinear_gap = std::clamp(
            3.20 / safe_resolution, 18.0, 70.0);
    for (size_t i = 0; i < structural_segments.size(); ++i) {
        for (size_t j = i + 1; j < structural_segments.size(); ++j) {
            const double angle_distance = AngleDistance(
                    LineAngle(structural_segments[i]), LineAngle(structural_segments[j]));
            if (angle_distance < 12.0) {
                const cv::Point2d endpoints_i[] = {
                        cv::Point2d(structural_segments[i][0], structural_segments[i][1]),
                        cv::Point2d(structural_segments[i][2], structural_segments[i][3])};
                const cv::Point2d endpoints_j[] = {
                        cv::Point2d(structural_segments[j][0], structural_segments[j][1]),
                        cv::Point2d(structural_segments[j][2], structural_segments[j][3])};
                double closest = std::numeric_limits<double>::infinity();
                cv::Point2d closest_i, closest_j;
                for (const auto& first : endpoints_i) {
                    for (const auto& second : endpoints_j) {
                        const double distance = cv::norm(first - second);
                        if (distance < closest) {
                            closest = distance;
                            closest_i = first;
                            closest_j = second;
                        }
                    }
                }
                if (closest <= max_collinear_gap) {
                    cv::line(structural_mask,
                             cv::Point(static_cast<int>(std::round(closest_i.x)),
                                       static_cast<int>(std::round(closest_i.y))),
                             cv::Point(static_cast<int>(std::round(closest_j.x)),
                                       static_cast<int>(std::round(closest_j.y))),
                             cv::Scalar(255), structural_thickness, cv::LINE_8);
                }
                continue;
            }
            if (angle_distance < MIN_CORNER_ANGLE) {
                continue;
            }
            bool intersection_ok = false;
            const cv::Point2d intersection = LineIntersection(
                    structural_segments[i], structural_segments[j], &intersection_ok);
            if (!intersection_ok || !std::isfinite(intersection.x) ||
                !std::isfinite(intersection.y) || intersection.x < -max_corner_extension ||
                intersection.y < -max_corner_extension ||
                intersection.x >= structural_mask.cols + max_corner_extension ||
                intersection.y >= structural_mask.rows + max_corner_extension) {
                continue;
            }
            auto nearest_endpoint = [&](const cv::Vec4i& segment) {
                const cv::Point2d first(segment[0], segment[1]);
                const cv::Point2d second(segment[2], segment[3]);
                return cv::norm(first - intersection) <= cv::norm(second - intersection)
                        ? first : second;
            };
            const cv::Point2d first = nearest_endpoint(structural_segments[i]);
            const cv::Point2d second = nearest_endpoint(structural_segments[j]);
            if (cv::norm(first - intersection) > max_corner_extension ||
                cv::norm(second - intersection) > max_corner_extension) {
                continue;
            }
            const cv::Point corner(
                    static_cast<int>(std::round(intersection.x)),
                    static_cast<int>(std::round(intersection.y)));
            cv::line(structural_mask,
                     cv::Point(static_cast<int>(std::round(first.x)),
                               static_cast<int>(std::round(first.y))),
                     corner, cv::Scalar(255), structural_thickness, cv::LINE_8);
            cv::line(structural_mask,
                     cv::Point(static_cast<int>(std::round(second.x)),
                               static_cast<int>(std::round(second.y))),
                     corner, cv::Scalar(255), structural_thickness, cv::LINE_8);
        }
    }
    cv::imwrite(PathJoin(debug_dir, "structural_wall_mask.png"), structural_mask);
    // Exterior topology must be branch-independent. Skeleton pruning and
    // fitted line bridging are useful for red internal walls, but allowing
    // them into this mask made each branch propose a different green outline
    // for the same exported map. Use the fixed fused occupancy raster that is
    // also presented to the user; semantic free-space connectivity below
    // prevents its detached lidar rays from entering the footprint.
    cv::Mat outline_source_mask = visual_wall_binary.clone();
    cv::imwrite(PathJoin(debug_dir, "outline_source_mask.png"), outline_source_mask);
    ClosedOutlineResult outline = BuildTrajectoryConnectedOutline(
            outline_source_mask,
            visual_wall_binary,
            semantic_map,
            trajectory_points_px,
            meters_per_pixel,
            debug_dir);

    // Keep every black map detail, but use the single trustworthy exterior
    // polygon as the only green overlay. `closed=true` guarantees that the last
    // vertex is connected back to the first with the same stroke.
    if (!outline.valid || outline.original_polygon.size() < 3) {
        throw std::runtime_error("Unable to extract a trustworthy closed outer contour");
    }

    // A valid exterior should contain the fused walls that border measured
    // indoor free space.  This catches a branch that closes around only part
    // of the building while avoiding a reward for detached outdoor lidar
    // rays.  A small polygon dilation admits the physical wall thickness.
    cv::Mat relevant_building_walls = internal_structure_binary.clone();
    if (!semantic_map.empty()) {
        cv::Mat measured_free;
        cv::inRange(
                semantic_map,
                cv::Scalar(245, 245, 245),
                cv::Scalar(255, 255, 255),
                measured_free);
        const int free_neighbourhood = std::clamp(
                static_cast<int>(std::round(0.45 / safe_resolution)),
                5,
                19);
        cv::dilate(
                measured_free,
                measured_free,
                cv::getStructuringElement(
                        cv::MORPH_ELLIPSE,
                        cv::Size(free_neighbourhood * 2 + 1,
                                 free_neighbourhood * 2 + 1)));
        cv::bitwise_and(
                relevant_building_walls,
                measured_free,
                relevant_building_walls);
    }
    cv::Mat outline_interior = cv::Mat::zeros(
            internal_structure_binary.size(), CV_8UC1);
    cv::fillPoly(
            outline_interior,
            std::vector<std::vector<cv::Point>>{outline.original_polygon},
            cv::Scalar(255));
    const int wall_tolerance = std::clamp(
            static_cast<int>(std::round(0.18 / safe_resolution)), 2, 7);
    cv::dilate(
            outline_interior,
            outline_interior,
            cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(wall_tolerance * 2 + 1,
                             wall_tolerance * 2 + 1)));
    cv::Mat contained_building_walls;
    cv::bitwise_and(
            relevant_building_walls,
            outline_interior,
            contained_building_walls);
    const double wall_containment_ratio = cv::countNonZero(
            contained_building_walls) /
            std::max(1.0, static_cast<double>(cv::countNonZero(
                    relevant_building_walls)));

    const cv::Rect polygon_bounds = cv::boundingRect(outline.original_polygon);
    const int content_left = std::clamp(polygon_bounds.x - 2, 0, original_map.cols);
    const int content_top = std::clamp(polygon_bounds.y - 2, 0, original_map.rows);
    const int content_right = std::clamp(
            polygon_bounds.x + polygon_bounds.width + 2, 0, original_map.cols);
    const int content_bottom = std::clamp(
            polygon_bounds.y + polygon_bounds.height + 2, 0, original_map.rows);
    if (content_right - content_left <= 1 || content_bottom - content_top <= 1) {
        throw std::runtime_error("Fitted wall bounds are invalid");
    }

    // The thresholded structural export can retain only the two short faces
    // of a thick/weak wall while the continuous visual probability raster
    // still contains its complete long axis.  Detect candidates from their
    // union; the semantic, topology and trajectory checks below remain the
    // authority that distinguishes partitions from ordinary clutter.
    cv::Mat partition_wall_evidence;
    cv::bitwise_or(
            internal_structure_binary,
            visual_wall_binary,
            partition_wall_evidence);
    cv::Mat internal_structure_skeleton =
            Skeletonize(partition_wall_evidence);
    cv::imwrite(
            PathJoin(debug_dir, "internal_structure_source.png"),
            internal_structure_skeleton);
    std::vector<cv::Vec4i> internal_wall_segments =
            DetectInternalWallSegments(
                    internal_structure_skeleton,
                    outline.original_polygon,
                    meters_per_pixel,
                    debug_dir,
                    semantic_map,
                    trajectory_points_px,
                    partition_wall_evidence);

    // Region-first recovery complements the local Hough/skeleton detector.
    // It contributes complete Manhattan partition runs across small scan gaps,
    // then merges them into the existing graph so one physical wall still
    // produces exactly one red centreline.
    // The structural export is stable for topology, while the continuous
    // probability raster may retain a wall that thinning/thresholding lost.
    // Use their union only to propose room separators; semantic side tests
    // inside DetectRegionSeparatorSegments still reject outdoor rays and most
    // isolated obstacles.
    const std::vector<cv::Vec4i> region_separator_segments =
            DetectRegionSeparatorSegments(
                    partition_wall_evidence,
                    semantic_map,
                    outline.original_polygon,
                    meters_per_pixel,
                    debug_dir);
    int region_separator_additions = 0;
    int region_separator_merges = 0;
    const double separator_merge_offset = std::clamp(
            0.30 / safe_resolution, 4.0, 14.0);
    const double separator_merge_gap = std::clamp(
            0.55 / safe_resolution, 5.0, 24.0);
    for (const auto& candidate : region_separator_segments) {
        bool merged = false;
        for (auto& retained : internal_wall_segments) {
            if (AngleDistance(LineAngle(candidate), LineAngle(retained)) >
                10.0) {
                continue;
            }
            const cv::Point2d direction =
                    DirectionFromAngle(LineAngle(candidate));
            const cv::Point2d normal(-direction.y, direction.x);
            const double candidate_offset =
                    0.5 * (cv::Point2d(candidate[0], candidate[1]).dot(normal) +
                           cv::Point2d(candidate[2], candidate[3]).dot(normal));
            const double retained_offset =
                    0.5 * (cv::Point2d(retained[0], retained[1]).dot(normal) +
                           cv::Point2d(retained[2], retained[3]).dot(normal));
            if (std::fabs(candidate_offset - retained_offset) >
                separator_merge_offset) {
                continue;
            }
            auto span = [&](const cv::Vec4i& segment) {
                std::array<double, 2> values{
                        cv::Point2d(segment[0], segment[1]).dot(direction),
                        cv::Point2d(segment[2], segment[3]).dot(direction)};
                std::sort(values.begin(), values.end());
                return values;
            };
            const auto candidate_span = span(candidate);
            const auto retained_span = span(retained);
            const double gap = std::max(
                    0.0,
                    std::max(candidate_span[0], retained_span[0]) -
                            std::min(candidate_span[1], retained_span[1]));
            if (gap > separator_merge_gap) continue;
            // Prefer the region separator's robust band centre and extend it
            // over the union of both observed spans.
            retained = BuildSegment(
                    std::min(candidate_span[0], retained_span[0]),
                    std::max(candidate_span[1], retained_span[1]),
                    candidate_offset,
                    direction);
            merged = true;
            ++region_separator_merges;
            break;
        }
        if (!merged) {
            internal_wall_segments.push_back(candidate);
            ++region_separator_additions;
        }
    }
    if (!region_separator_segments.empty()) {
        std::cout << "[INFO] 区域分隔墙恢复 candidates="
                  << region_separator_segments.size()
                  << " added=" << region_separator_additions
                  << " merged=" << region_separator_merges << "\n";
    }

    // Rescue a long observed partition whose doorway/noisy endpoint kept it
    // out of the junction-based detector. It must lie well inside the fitted
    // footprint, be supported by the original wall raster, and have scanned
    // free space on both sides at multiple longitudinal samples. These checks
    // reject facade returns and most furniture edges while recovering the
    // clean room dividers retained by region-first competitor floor plans.
    int raw_partition_rescues = 0;
    if (!semantic_map.empty() && semantic_map.size() == wall_binary.size()) {
        const double partition_resolution =
                std::max(1e-4, std::isfinite(meters_per_pixel)
                        ? meters_per_pixel : 0.05);
        cv::Mat raw_partition_free;
        cv::inRange(
                semantic_map,
                cv::Scalar(245, 245, 245),
                cv::Scalar(255, 255, 255),
                raw_partition_free);
        const int free_expansion = std::clamp(
                static_cast<int>(std::round(0.08 / partition_resolution)),
                1,
                4);
        cv::dilate(
                raw_partition_free,
                raw_partition_free,
                cv::getStructuringElement(
                        cv::MORPH_ELLIPSE,
                        cv::Size(free_expansion * 2 + 1,
                                 free_expansion * 2 + 1)));
        const double minimum_rescue_length = std::clamp(
                1.00 / partition_resolution, 14.0, 80.0);
        const double minimum_rescue_depth = std::clamp(
                0.35 / partition_resolution, 4.0, 14.0);
        const double side_probe_distance = std::clamp(
                0.28 / partition_resolution, 4.0, 12.0);
        const int support_thickness = std::clamp(
                static_cast<int>(std::round(0.10 / partition_resolution)),
                2,
                5);
        const double long_axis_angle = std::atan2(
                outline.long_axis.y,
                outline.long_axis.x) * 180.0 / kPi;
        auto semantic_free_at = [&](const cv::Point2d& point) {
            const int x = static_cast<int>(std::round(point.x));
            const int y = static_cast<int>(std::round(point.y));
            return x >= 0 && x < raw_partition_free.cols &&
                    y >= 0 && y < raw_partition_free.rows &&
                    raw_partition_free.at<uchar>(y, x) != 0;
        };
        for (const auto& segment : raw_segs) {
            const double length = SegmentLength(segment);
            if (length < minimum_rescue_length) continue;
            const cv::Point2d first(segment[0], segment[1]);
            const cv::Point2d second(segment[2], segment[3]);
            const cv::Point2d delta = second - first;
            if (length <= 1e-6) continue;
            const cv::Point2d tangent = delta * (1.0 / length);
            const cv::Point2d normal(-tangent.y, tangent.x);
            const double segment_angle = std::atan2(
                    tangent.y, tangent.x) * 180.0 / kPi;
            if (std::min(
                        AngleDistance(segment_angle, long_axis_angle),
                        AngleDistance(segment_angle, long_axis_angle + 90.0)) >
                15.0) {
                continue;
            }
            int positive_free_samples = 0;
            int negative_free_samples = 0;
            int deep_inside_samples = 0;
            for (const double fraction : {0.20, 0.35, 0.50, 0.65, 0.80}) {
                const cv::Point2d center = first + delta * fraction;
                if (cv::pointPolygonTest(
                            outline.original_polygon,
                            cv::Point2f(
                                    static_cast<float>(center.x),
                                    static_cast<float>(center.y)),
                            true) >= minimum_rescue_depth) {
                    ++deep_inside_samples;
                }
                if (semantic_free_at(
                            center + normal * side_probe_distance)) {
                    ++positive_free_samples;
                }
                if (semantic_free_at(
                            center - normal * side_probe_distance)) {
                    ++negative_free_samples;
                }
            }
            const double observed_support = SegmentSupportRatio(
                    internal_structure_skeleton,
                    segment,
                    support_thickness);
            const bool strong_two_sided_partition =
                    deep_inside_samples >= 3 &&
                    positive_free_samples >= 3 &&
                    negative_free_samples >= 3 &&
                    observed_support >= 0.22;
            const bool long_deep_partition =
                    length >= 1.50 / partition_resolution &&
                    deep_inside_samples >= 4 &&
                    std::max(positive_free_samples, negative_free_samples) >= 3 &&
                    observed_support >= 0.40;
            if (!strong_two_sided_partition && !long_deep_partition) continue;
            internal_wall_segments.push_back(segment);
            ++raw_partition_rescues;
        }
    }
    if (raw_partition_rescues > 0) {
        std::cout << "[INFO] 双侧自由空间长隔墙补全="
                  << raw_partition_rescues << "\n";
    }

    // Normalize all accepted evidence into one sparse architectural graph.
    // Local Hough, skeleton tracing and region separators may describe the
    // same physical partition with slightly different angle/offset/span.
    // Snap them to the exterior's Manhattan frame and merge only overlapping
    // or source-supported gaps. This produces the competitor-style set of
    // long straight dividers instead of several short parallel red strokes.
    const double graph_primary_angle = std::atan2(
            outline.long_axis.y,
            outline.long_axis.x) * 180.0 / kPi;
    std::vector<cv::Vec4i> normalized_internal_segments;
    for (const auto& segment : internal_wall_segments) {
        const double length = SegmentLength(segment);
        if (length < 0.40 / safe_resolution) continue;
        const double first_axis_distance =
                AngleDistance(LineAngle(segment), graph_primary_angle);
        const double second_axis_distance =
                AngleDistance(LineAngle(segment), graph_primary_angle + 90.0);
        const double best_axis = first_axis_distance <= second_axis_distance
                ? graph_primary_angle
                : graph_primary_angle + 90.0;
        if (std::min(first_axis_distance, second_axis_distance) > 18.0) {
            // Keep an intentional diagonal only when it is long and densely
            // observed. Most short diagonal candidates are furniture edges.
            const double diagonal_support = SegmentSupportRatio(
                    internal_structure_binary, segment, 3);
            if (length < 1.50 / safe_resolution ||
                diagonal_support < 0.68) {
                continue;
            }
            normalized_internal_segments.push_back(segment);
            continue;
        }
        const cv::Point2d direction = DirectionFromAngle(best_axis);
        const cv::Point2d normal(-direction.y, direction.x);
        const cv::Point2d first(segment[0], segment[1]);
        const cv::Point2d second(segment[2], segment[3]);
        normalized_internal_segments.push_back(BuildSegment(
                std::min(first.dot(direction), second.dot(direction)),
                std::max(first.dot(direction), second.dot(direction)),
                0.5 * (first.dot(normal) + second.dot(normal)),
                direction));
    }
    const double graph_offset_tolerance = std::clamp(
            0.24 / safe_resolution, 3.0, 11.0);
    // Long partitions are often split into several Hough runs by a doorway,
    // a weak scan band, or a dark wall face.  Keep the short unconditional
    // allowance below, but permit a wider join when the intervening span is
    // itself supported by the structural raster.  Judging the fragments
    // independently later makes a genuine room-scale wall look like several
    // isolated furniture edges.
    const double graph_maximum_gap = std::clamp(
            1.35 / safe_resolution, 10.0, 54.0);
    const double graph_unconditional_gap = std::clamp(
            0.18 / safe_resolution, 2.0, 8.0);
    bool graph_merged = true;
    int graph_merge_count = 0;
    while (graph_merged) {
        graph_merged = false;
        for (size_t first_index = 0;
             first_index < normalized_internal_segments.size() &&
                     !graph_merged;
             ++first_index) {
            for (size_t second_index = first_index + 1;
                 second_index < normalized_internal_segments.size();
                 ++second_index) {
                const cv::Vec4i first =
                        normalized_internal_segments[first_index];
                const cv::Vec4i second =
                        normalized_internal_segments[second_index];
                if (AngleDistance(LineAngle(first), LineAngle(second)) > 6.0) {
                    continue;
                }
                const cv::Point2d direction =
                        DirectionFromAngle(LineAngle(first));
                const cv::Point2d normal(-direction.y, direction.x);
                const double first_offset =
                        0.5 * (cv::Point2d(first[0], first[1]).dot(normal) +
                               cv::Point2d(first[2], first[3]).dot(normal));
                const double second_offset =
                        0.5 * (cv::Point2d(second[0], second[1]).dot(normal) +
                               cv::Point2d(second[2], second[3]).dot(normal));
                if (std::fabs(first_offset - second_offset) >
                    graph_offset_tolerance) {
                    continue;
                }
                auto span = [&](const cv::Vec4i& value) {
                    std::array<double, 2> output{
                            cv::Point2d(value[0], value[1]).dot(direction),
                            cv::Point2d(value[2], value[3]).dot(direction)};
                    std::sort(output.begin(), output.end());
                    return output;
                };
                const auto first_span = span(first);
                const auto second_span = span(second);
                const double gap = std::max(
                        0.0,
                        std::max(first_span[0], second_span[0]) -
                                std::min(first_span[1], second_span[1]));
                if (gap > graph_maximum_gap) continue;
                const double merged_offset =
                        (first_offset * SegmentLength(first) +
                         second_offset * SegmentLength(second)) /
                        std::max(1.0,
                                 SegmentLength(first) + SegmentLength(second));
                if (gap > graph_unconditional_gap) {
                    double gap_start = 0.0;
                    double gap_end = 0.0;
                    if (first_span[1] < second_span[0]) {
                        gap_start = first_span[1];
                        gap_end = second_span[0];
                    } else if (second_span[1] < first_span[0]) {
                        gap_start = second_span[1];
                        gap_end = first_span[0];
                    }
                    if (gap_end > gap_start) {
                        const cv::Vec4i gap_segment = BuildSegment(
                                gap_start, gap_end, merged_offset, direction);
                        const int gap_support_radius = std::clamp(
                                static_cast<int>(std::round(
                                        0.16 / safe_resolution)),
                                3,
                                7);
                        const double gap_support = SegmentSupportRatio(
                                partition_wall_evidence,
                                gap_segment,
                                gap_support_radius);
                        if (gap_support < 0.18) {
                            continue;
                        }
                    }
                }
                normalized_internal_segments[first_index] = BuildSegment(
                        std::min(first_span[0], second_span[0]),
                        std::max(first_span[1], second_span[1]),
                        merged_offset,
                        direction);
                normalized_internal_segments.erase(
                        normalized_internal_segments.begin() +
                        static_cast<long>(second_index));
                graph_merged = true;
                ++graph_merge_count;
                break;
            }
        }
    }
    internal_wall_segments = std::move(normalized_internal_segments);
    // Keep the complete normalized pool before facade rejection. A genuine
    // room divider can run close and parallel to an indented exterior wall;
    // distance-only facade filtering may remove it even though it has rooms
    // on both sides. The later room-graph recovery has stronger semantic and
    // connectivity tests and must be allowed to reconsider such candidates.
    const std::vector<cv::Vec4i> all_normalized_partition_candidates =
            internal_wall_segments;
    std::cout << "[INFO] 内墙线网归一化 merges="
              << graph_merge_count
              << " final=" << internal_wall_segments.size() << "\n";

    // Classify exterior-vs-partition per complete fitted segment. Pixel-wise
    // removal below used distance to outside unknown space and could punch
    // dozens of holes through a real wall beside an unentered room. A facade
    // segment must be parallel and close to the green envelope and must not
    // have explored floor on both sides. Perpendicular room dividers are kept
    // through their junction with the facade.
    cv::Mat segment_free_mask;
    cv::Mat segment_raw_free_mask;
    if (!semantic_map.empty()) {
        cv::inRange(
                semantic_map,
                cv::Scalar(245, 245, 245),
                cv::Scalar(255, 255, 255),
                segment_free_mask);
        // Preserve the undilated semantic floor mask for room-scale checks.
        // The dilated copy is useful for tolerating registration noise next
        // to a wall, but it can leak across a narrow unknown/obstacle strip
        // and make a cabinet or stair opening appear to have rooms on both
        // sides.
        segment_raw_free_mask = segment_free_mask.clone();
        const int free_radius = std::clamp(
                static_cast<int>(std::round(0.08 / safe_resolution)),
                1, 4);
        cv::dilate(
                segment_free_mask,
                segment_free_mask,
                cv::getStructuringElement(
                        cv::MORPH_ELLIPSE,
                        cv::Size(free_radius * 2 + 1,
                                 free_radius * 2 + 1)));
    }
    const double facade_distance = std::clamp(
            0.85 / safe_resolution, 8.0, 24.0);
    const double unconditional_facade_distance = std::clamp(
            0.30 / safe_resolution, 3.0, 9.0);
    const double facade_side_probe = std::clamp(
            0.24 / safe_resolution, 3.0, 10.0);
    int exterior_segments_removed = 0;
    internal_wall_segments.erase(
            std::remove_if(
                    internal_wall_segments.begin(),
                    internal_wall_segments.end(),
                    [&](const cv::Vec4i& segment) {
                        const cv::Point2d midpoint(
                                0.5 * (segment[0] + segment[2]),
                                0.5 * (segment[1] + segment[3]));
                        double nearest_distance =
                                std::numeric_limits<double>::infinity();
                        double nearest_angle = 0.0;
                        for (size_t edge_index = 0;
                             edge_index < outline.original_polygon.size();
                             ++edge_index) {
                            const cv::Point2d edge_start(
                                    outline.original_polygon[edge_index]);
                            const cv::Point2d edge_end(
                                    outline.original_polygon[
                                            (edge_index + 1) %
                                            outline.original_polygon.size()]);
                            const cv::Point2d edge = edge_end - edge_start;
                            const double length_squared = edge.dot(edge);
                            if (length_squared <= 1e-6) continue;
                            const double fraction = std::clamp(
                                    (midpoint - edge_start).dot(edge) /
                                            length_squared,
                                    0.0, 1.0);
                            const double distance = cv::norm(
                                    midpoint - (edge_start + edge * fraction));
                            if (distance < nearest_distance) {
                                nearest_distance = distance;
                                nearest_angle = std::atan2(
                                        edge.y, edge.x) * 180.0 / kPi;
                            }
                        }
                        if (nearest_distance > facade_distance ||
                            AngleDistance(
                                    LineAngle(segment), nearest_angle) > 18.0) {
                            return false;
                        }
                        bool two_sided_free = false;
                        if (!segment_free_mask.empty()) {
                            const cv::Point2d start(segment[0], segment[1]);
                            const cv::Point2d end(segment[2], segment[3]);
                            const cv::Point2d delta = end - start;
                            const double length = cv::norm(delta);
                            if (length > 1e-6) {
                                const cv::Point2d tangent = delta * (1.0 / length);
                                const cv::Point2d normal(-tangent.y, tangent.x);
                                int positive = 0;
                                int negative = 0;
                                for (const double fraction :
                                     {0.20, 0.35, 0.50, 0.65, 0.80}) {
                                    const cv::Point2d center =
                                            start + delta * fraction;
                                    auto is_free = [&](const cv::Point2d& point) {
                                        const int x = static_cast<int>(
                                                std::round(point.x));
                                        const int y = static_cast<int>(
                                                std::round(point.y));
                                        return x >= 0 &&
                                                x < segment_free_mask.cols &&
                                                y >= 0 &&
                                                y < segment_free_mask.rows &&
                                                segment_free_mask.at<uchar>(y, x) != 0;
                                    };
                                    if (is_free(center + normal * facade_side_probe)) {
                                        ++positive;
                                    }
                                    if (is_free(center - normal * facade_side_probe)) {
                                        ++negative;
                                    }
                                }
                                two_sided_free = positive >= 3 && negative >= 3;
                            }
                        }
                        if (two_sided_free &&
                            nearest_distance > unconditional_facade_distance) {
                            return false;
                        }
                        ++exterior_segments_removed;
                        return true;
                    }),
            internal_wall_segments.end());
    if (exterior_segments_removed > 0) {
        std::cout << "[INFO] 整段外墙排除="
                  << exterior_segments_removed << "\n";
    }

    // Keep the complete normalized graph for a later main-wall recovery.
    // Room topology is intentionally conservative and may reject a long wall
    // beside an unscanned room; once discarded, an anchored-stub pass cannot
    // reconstruct a divider whose endpoints both stop inside the footprint.
    const std::vector<cv::Vec4i> pre_topology_internal_segments =
            internal_wall_segments;

    // Build a room-scale partition label map. Local side pixels alone cannot
    // distinguish a true wall from a U-shaped cabinet or stair opening: both
    // may have free pixels on two sides. Close doorway-sized gaps in the
    // observed Manhattan wall network, then label the navigable faces inside
    // the fitted footprint. A normal architectural divider should border two
    // different faces whose areas are large enough to represent rooms.
    cv::Mat room_partition_labels;
    cv::Mat room_partition_stats;
    if (!segment_free_mask.empty()) {
        const int directional_run = std::clamp(
                static_cast<int>(std::round(0.35 / safe_resolution)),
                5, 18);
        int doorway_close = std::clamp(
                static_cast<int>(std::round(1.05 / safe_resolution)),
                9, 43);
        if (doorway_close % 2 == 0) ++doorway_close;
        cv::Mat horizontal_barriers;
        cv::Mat vertical_barriers;
        cv::morphologyEx(
                internal_structure_binary,
                horizontal_barriers,
                cv::MORPH_OPEN,
                cv::getStructuringElement(
                        cv::MORPH_RECT,
                        cv::Size(directional_run, 1)));
        cv::morphologyEx(
                internal_structure_binary,
                vertical_barriers,
                cv::MORPH_OPEN,
                cv::getStructuringElement(
                        cv::MORPH_RECT,
                        cv::Size(1, directional_run)));
        cv::morphologyEx(
                horizontal_barriers,
                horizontal_barriers,
                cv::MORPH_CLOSE,
                cv::getStructuringElement(
                        cv::MORPH_RECT,
                        cv::Size(doorway_close, 3)));
        cv::morphologyEx(
                vertical_barriers,
                vertical_barriers,
                cv::MORPH_CLOSE,
                cv::getStructuringElement(
                        cv::MORPH_RECT,
                        cv::Size(3, doorway_close)));
        cv::Mat room_barriers;
        cv::bitwise_or(
                horizontal_barriers,
                vertical_barriers,
                room_barriers);
        // Include the normalized candidate graph: a fragmented but verified
        // divider must participate in the room split even if its source wall
        // contains a doorway-sized gap.
        const int graph_barrier_thickness = std::clamp(
                static_cast<int>(std::round(0.12 / safe_resolution)),
                3, 7);
        for (const auto& segment : internal_wall_segments) {
            cv::line(
                    room_barriers,
                    cv::Point(segment[0], segment[1]),
                    cv::Point(segment[2], segment[3]),
                    cv::Scalar(255),
                    graph_barrier_thickness,
                    cv::LINE_8);
        }
        cv::Mat footprint_for_rooms = cv::Mat::zeros(
                internal_structure_binary.size(), CV_8UC1);
        cv::fillPoly(
                footprint_for_rooms,
                std::vector<std::vector<cv::Point>>{
                        outline.original_polygon},
                cv::Scalar(255));
        cv::Mat navigable_faces;
        cv::bitwise_and(
                segment_free_mask,
                footprint_for_rooms,
                navigable_faces);
        navigable_faces.setTo(cv::Scalar(0), room_barriers);
        cv::morphologyEx(
                navigable_faces,
                navigable_faces,
                cv::MORPH_OPEN,
                cv::getStructuringElement(
                        cv::MORPH_ELLIPSE, cv::Size(3, 3)));
        cv::Mat room_centroids;
        cv::connectedComponentsWithStats(
                navigable_faces,
                room_partition_labels,
                room_partition_stats,
                room_centroids,
                8,
                CV_32S);
        if (!debug_dir.empty()) {
            cv::Mat room_debug(
                    navigable_faces.size(), CV_8UC3,
                    cv::Scalar(0, 0, 0));
            const std::array<cv::Vec3b, 8> colors{{
                    {64, 64, 64}, {180, 80, 80}, {80, 180, 80},
                    {80, 80, 180}, {180, 180, 80}, {180, 80, 180},
                    {80, 180, 180}, {150, 120, 80}}};
            for (int y = 0; y < room_debug.rows; ++y) {
                cv::Vec3b* output = room_debug.ptr<cv::Vec3b>(y);
                const int* labels = room_partition_labels.ptr<int>(y);
                for (int x = 0; x < room_debug.cols; ++x) {
                    if (labels[x] > 0) {
                        output[x] = colors[
                                static_cast<size_t>(labels[x]) %
                                colors.size()];
                    }
                }
            }
            cv::imwrite(
                    PathJoin(debug_dir, "room_partition_faces.png"),
                    room_debug);
            cv::imwrite(
                    PathJoin(debug_dir, "room_partition_barriers.png"),
                    room_barriers);
        }
    }

    // Form the final room-partition graph. A true partition normally has
    // explored room area on both sides. Allow a one-sided wall only when it is
    // long, strongly observed and connected to the exterior/another wall;
    // this preserves walls of rooms scanned through a doorway while rejecting
    // furniture returns and isolated facade fragments.
    const double topology_anchor_distance = std::clamp(
            0.40 / safe_resolution, 4.0, 16.0);
    const double topology_side_distance_near = std::clamp(
            0.24 / safe_resolution, 3.0, 10.0);
    const double topology_side_distance_far = std::clamp(
            0.42 / safe_resolution, 5.0, 16.0);
    auto point_segment_distance = [](const cv::Point2d& point,
                                     const cv::Vec4i& segment) {
        const cv::Point2d start(segment[0], segment[1]);
        const cv::Point2d end(segment[2], segment[3]);
        const cv::Point2d delta = end - start;
        const double length_squared = delta.dot(delta);
        if (length_squared <= 1e-9) return cv::norm(point - start);
        const double fraction = std::clamp(
                (point - start).dot(delta) / length_squared, 0.0, 1.0);
        return cv::norm(point - (start + delta * fraction));
    };

    // Connect nearly meeting perpendicular runs before computing graph degree.
    int topology_junctions = 0;
    for (size_t first_index = 0;
         first_index < internal_wall_segments.size(); ++first_index) {
        for (size_t second_index = first_index + 1;
             second_index < internal_wall_segments.size(); ++second_index) {
            cv::Vec4i& first = internal_wall_segments[first_index];
            cv::Vec4i& second = internal_wall_segments[second_index];
            if (AngleDistance(LineAngle(first), LineAngle(second)) < 55.0) {
                continue;
            }
            bool valid_intersection = false;
            const cv::Point2d intersection = LineIntersection(
                    first, second, &valid_intersection);
            if (!valid_intersection ||
                cv::pointPolygonTest(
                        outline.original_polygon,
                        cv::Point2f(intersection),
                        true) < -2.0) {
                continue;
            }
            auto extend_nearest_endpoint = [&](cv::Vec4i* segment) {
                const cv::Point2d start((*segment)[0], (*segment)[1]);
                const cv::Point2d end((*segment)[2], (*segment)[3]);
                const double start_distance = cv::norm(start - intersection);
                const double end_distance = cv::norm(end - intersection);
                if (std::min(start_distance, end_distance) >
                    topology_anchor_distance) {
                    return false;
                }
                if (start_distance <= end_distance) {
                    (*segment)[0] = static_cast<int>(std::round(intersection.x));
                    (*segment)[1] = static_cast<int>(std::round(intersection.y));
                } else {
                    (*segment)[2] = static_cast<int>(std::round(intersection.x));
                    (*segment)[3] = static_cast<int>(std::round(intersection.y));
                }
                return true;
            };
            const bool first_extended = extend_nearest_endpoint(&first);
            const bool second_extended = extend_nearest_endpoint(&second);
            if (first_extended || second_extended) ++topology_junctions;
        }
    }

    int topology_pruned_segments = 0;
    int room_partition_verified_segments = 0;
    std::vector<cv::Vec4i> topology_segments;
    for (size_t index = 0; index < internal_wall_segments.size(); ++index) {
        const cv::Vec4i& segment = internal_wall_segments[index];
        const double length = SegmentLength(segment);
        if (length < 0.70 / safe_resolution) {
            ++topology_pruned_segments;
            continue;
        }
        const cv::Point2d start(segment[0], segment[1]);
        const cv::Point2d end(segment[2], segment[3]);
        const cv::Point2d delta = end - start;
        if (length <= 1e-6) continue;
        const cv::Point2d tangent = delta * (1.0 / length);
        const cv::Point2d normal(-tangent.y, tangent.x);
        int positive_free = 0;
        int negative_free = 0;
        constexpr int kTopologySamples = 11;
        for (int sample = 1; sample < kTopologySamples - 1; ++sample) {
            const double fraction =
                    sample / static_cast<double>(kTopologySamples - 1);
            const cv::Point2d center = start + delta * fraction;
            auto is_free = [&](const cv::Point2d& point) {
                if (segment_free_mask.empty()) return false;
                const int x = static_cast<int>(std::round(point.x));
                const int y = static_cast<int>(std::round(point.y));
                return x >= 0 && x < segment_free_mask.cols &&
                        y >= 0 && y < segment_free_mask.rows &&
                        segment_free_mask.at<uchar>(y, x) != 0;
            };
            if (is_free(center + normal * topology_side_distance_near) ||
                is_free(center + normal * topology_side_distance_far)) {
                ++positive_free;
            }
            if (is_free(center - normal * topology_side_distance_near) ||
                is_free(center - normal * topology_side_distance_far)) {
                ++negative_free;
            }
        }
        const double positive_ratio = positive_free / 9.0;
        const double negative_ratio = negative_free / 9.0;
        const bool two_sided_partition =
                std::min(positive_ratio, negative_ratio) >= 0.34 &&
                std::max(positive_ratio, negative_ratio) >= 0.56;

        // Require sustained, room-scale free space on both sides using the
        // original semantic mask. Probe at several distances so a real thick
        // wall is not rejected, while a U-shaped obstacle boundary whose
        // opposite side remains unknown cannot be promoted by dilation.
        int broad_positive_free = 0;
        int broad_negative_free = 0;
        int broad_samples = 0;
        if (!segment_raw_free_mask.empty()) {
            const double room_probe_distance = std::clamp(
                    0.26 / safe_resolution, 3.0, 11.0);
            auto is_raw_free = [&](const cv::Point2d& point) {
                const int x = static_cast<int>(std::round(point.x));
                const int y = static_cast<int>(std::round(point.y));
                return x >= 0 && x < segment_raw_free_mask.cols &&
                        y >= 0 && y < segment_raw_free_mask.rows &&
                        segment_raw_free_mask.at<uchar>(y, x) != 0;
            };
            for (const double fraction :
                 {0.18, 0.31, 0.44, 0.56, 0.69, 0.82}) {
                const cv::Point2d center = start + delta * fraction;
                // A room must begin immediately outside the wall band. Do
                // not accept a far free pixel after crossing an unknown gap:
                // that is the characteristic signature of a cabinet, stair
                // opening or unobserved pocket surrounded by explored floor.
                // The probe is outside the maximum accepted wall half width.
                const bool positive_has_room = is_raw_free(
                        center + normal * room_probe_distance);
                const bool negative_has_room = is_raw_free(
                        center - normal * room_probe_distance);
                broad_positive_free += positive_has_room ? 1 : 0;
                broad_negative_free += negative_has_room ? 1 : 0;
                ++broad_samples;
            }
        }
        const double broad_positive_ratio = broad_samples > 0
                ? broad_positive_free / static_cast<double>(broad_samples)
                : 0.0;
        const double broad_negative_ratio = broad_samples > 0
                ? broad_negative_free / static_cast<double>(broad_samples)
                : 0.0;
        const bool broad_two_sided_room =
                std::min(broad_positive_ratio, broad_negative_ratio) >= 0.58 &&
                std::max(broad_positive_ratio, broad_negative_ratio) >= 0.70;

        bool separates_room_faces = false;
        if (!room_partition_labels.empty() &&
            !room_partition_stats.empty()) {
            std::map<int, int> positive_labels;
            std::map<int, int> negative_labels;
            const double label_probe = std::clamp(
                    0.34 / safe_resolution, 4.0, 13.0);
            const int label_search = std::clamp(
                    static_cast<int>(std::round(0.12 / safe_resolution)),
                    2, 5);
            auto collect_label = [&](const cv::Point2d& point,
                                     std::map<int, int>* votes) {
                int best_label = 0;
                int best_distance_squared = std::numeric_limits<int>::max();
                const int center_x = static_cast<int>(std::round(point.x));
                const int center_y = static_cast<int>(std::round(point.y));
                for (int dy = -label_search; dy <= label_search; ++dy) {
                    for (int dx = -label_search; dx <= label_search; ++dx) {
                        const int x = center_x + dx;
                        const int y = center_y + dy;
                        if (x < 0 || x >= room_partition_labels.cols ||
                            y < 0 || y >= room_partition_labels.rows) {
                            continue;
                        }
                        const int label =
                                room_partition_labels.at<int>(y, x);
                        const int distance_squared = dx * dx + dy * dy;
                        if (label > 0 &&
                            distance_squared < best_distance_squared) {
                            best_label = label;
                            best_distance_squared = distance_squared;
                        }
                    }
                }
                if (best_label > 0) ++(*votes)[best_label];
            };
            for (const double fraction :
                 {0.18, 0.30, 0.42, 0.58, 0.70, 0.82}) {
                const cv::Point2d center = start + delta * fraction;
                collect_label(
                        center + normal * label_probe,
                        &positive_labels);
                collect_label(
                        center - normal * label_probe,
                        &negative_labels);
            }
            auto dominant_label = [](const std::map<int, int>& votes) {
                int label = 0;
                int count = 0;
                for (const auto& [candidate, candidate_count] : votes) {
                    if (candidate_count > count) {
                        label = candidate;
                        count = candidate_count;
                    }
                }
                return std::pair<int, int>(label, count);
            };
            const auto positive_label = dominant_label(positive_labels);
            const auto negative_label = dominant_label(negative_labels);
            const int minimum_room_area_px = static_cast<int>(std::round(
                    0.75 / (safe_resolution * safe_resolution)));
            auto room_area = [&](int label) {
                return label > 0 && label < room_partition_stats.rows
                        ? room_partition_stats.at<int>(
                                label, cv::CC_STAT_AREA)
                        : 0;
            };
            separates_room_faces =
                    positive_label.first > 0 &&
                    negative_label.first > 0 &&
                    positive_label.first != negative_label.first &&
                    positive_label.second >= 2 &&
                    negative_label.second >= 2 &&
                    room_area(positive_label.first) >= minimum_room_area_px &&
                    room_area(negative_label.first) >= minimum_room_area_px;
        }

        int connections = 0;
        for (const cv::Point2d& endpoint : {start, end}) {
            if (std::fabs(cv::pointPolygonTest(
                        outline.original_polygon,
                        cv::Point2f(endpoint), true)) <=
                topology_anchor_distance) {
                ++connections;
            }
            for (size_t other = 0;
                 other < internal_wall_segments.size(); ++other) {
                if (other == index) continue;
                if (point_segment_distance(
                            endpoint, internal_wall_segments[other]) <=
                    topology_anchor_distance) {
                    ++connections;
                    break;
                }
            }
        }
        const double support = SegmentSupportRatio(
                internal_structure_binary, segment, 3);
        const cv::Point2f midpoint(
                static_cast<float>(0.5 * (segment[0] + segment[2])),
                static_cast<float>(0.5 * (segment[1] + segment[3])));
        const double interior_depth = cv::pointPolygonTest(
                outline.original_polygon, midpoint, true);
        const bool strong_connected_main_wall =
                length >= 1.15 / safe_resolution &&
                support >= 0.42 &&
                // A connected edge around a cabinet/stair void has room on
                // only one side.  Require some sustained room evidence on
                // both sides before graph connectivity can promote it to a
                // main partition. Very long, exceptionally well-supported
                // dividers may still survive through the dedicated long-wall
                // recovery path earlier in the pipeline.
                std::min(positive_ratio, negative_ratio) >= 0.20 &&
                std::max(positive_ratio, negative_ratio) >= 0.55 &&
                connections >= 1 &&
                interior_depth >= 0.25 / safe_resolution;
        const bool verified_room_partition =
                separates_room_faces &&
                length >= 0.70 / safe_resolution &&
                support >= 0.24;
        // Door openings intentionally keep a floor navigable; therefore a
        // valid wall may not split the free mask into two fully disconnected
        // components. Treat the room-face test as strong positive evidence,
        // while retaining the established large two-sided connected-wall
        // path for doorway-bearing partitions.
        if (!verified_room_partition &&
            !(two_sided_partition && broad_two_sided_room) &&
            !(strong_connected_main_wall && broad_two_sided_room)) {
            ++topology_pruned_segments;
            continue;
        }
        if (verified_room_partition) {
            ++room_partition_verified_segments;
        }
        topology_segments.push_back(segment);
    }
    internal_wall_segments = std::move(topology_segments);
    std::cout << "[INFO] 内墙拓扑筛选 junctions="
              << topology_junctions
              << " pruned=" << topology_pruned_segments
              << " room_verified=" << room_partition_verified_segments
              << " final=" << internal_wall_segments.size() << "\n";

    // Topology filtering deliberately favors walls that separate two rooms.
    // Add back verified one-ended partition stubs afterwards so they are not
    // immediately removed again for having only one graph connection.
    const std::vector<cv::Vec4i> outline_anchored_stubs =
            DetectOutlineAnchoredPartitionStubs(
                    internal_structure_skeleton,
                    internal_structure_binary,
                    semantic_map,
                    outline.original_polygon,
                    meters_per_pixel,
                    debug_dir);
    int anchored_stub_additions = 0;
    for (const cv::Vec4i& stub : outline_anchored_stubs) {
        bool duplicate = false;
        for (const cv::Vec4i& retained : internal_wall_segments) {
            if (AngleDistance(LineAngle(stub), LineAngle(retained)) <= 8.0 &&
                ProjectionOverlapRatio(stub, retained) >= 0.45) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            internal_wall_segments.push_back(stub);
            ++anchored_stub_additions;
        }
    }
    if (!outline_anchored_stubs.empty()) {
        std::cout << "[INFO] 外墙锚定内向隔墙 candidates="
                  << outline_anchored_stubs.size()
                  << " added=" << anchored_stub_additions << "\n";
    }

    // Build the final competitor-style partition skeleton from semantic
    // sources, instead of merely appending them to every local Hough line.
    // Region separators describe walls between room faces; outline-anchored
    // stubs describe genuine one-ended dividers. Local candidates are allowed
    // back only as exceptionally long, continuously observed two-sided walls.
    std::vector<cv::Vec4i> semantic_partition_skeleton;
    const double semantic_merge_offset = std::clamp(
            0.26 / safe_resolution, 3.0, 10.0);
    const double semantic_merge_gap = std::clamp(
            0.48 / safe_resolution, 5.0, 20.0);
    auto add_semantic_partition = [&](const cv::Vec4i& candidate) {
        if (SegmentLength(candidate) < 0.55 / safe_resolution) return;
        for (cv::Vec4i& retained : semantic_partition_skeleton) {
            if (AngleDistance(LineAngle(candidate), LineAngle(retained)) >
                10.0) {
                continue;
            }
            const cv::Point2d direction =
                    DirectionFromAngle(LineAngle(retained));
            const cv::Point2d normal(-direction.y, direction.x);
            auto span = [&](const cv::Vec4i& segment) {
                std::array<double, 2> values{{
                        cv::Point2d(segment[0], segment[1]).dot(direction),
                        cv::Point2d(segment[2], segment[3]).dot(direction)}};
                std::sort(values.begin(), values.end());
                return values;
            };
            const double candidate_offset = 0.5 * (
                    cv::Point2d(candidate[0], candidate[1]).dot(normal) +
                    cv::Point2d(candidate[2], candidate[3]).dot(normal));
            const double retained_offset = 0.5 * (
                    cv::Point2d(retained[0], retained[1]).dot(normal) +
                    cv::Point2d(retained[2], retained[3]).dot(normal));
            if (std::fabs(candidate_offset - retained_offset) >
                semantic_merge_offset) {
                continue;
            }
            const auto candidate_span = span(candidate);
            const auto retained_span = span(retained);
            const double gap = std::max(
                    0.0,
                    std::max(candidate_span[0], retained_span[0]) -
                            std::min(candidate_span[1], retained_span[1]));
            if (gap > semantic_merge_gap) continue;
            const double combined_offset =
                    (candidate_offset * SegmentLength(candidate) +
                     retained_offset * SegmentLength(retained)) /
                    std::max(1.0,
                             SegmentLength(candidate) +
                             SegmentLength(retained));
            retained = BuildSegment(
                    std::min(candidate_span[0], retained_span[0]),
                    std::max(candidate_span[1], retained_span[1]),
                    combined_offset,
                    direction);
            return;
        }
        semantic_partition_skeleton.push_back(candidate);
    };
    for (const auto& segment : region_separator_segments) {
        add_semantic_partition(segment);
    }
    for (const auto& segment : outline_anchored_stubs) {
        add_semantic_partition(segment);
    }

    int long_partition_supplements = 0;
    // Region separators are the evidence that semantic room decomposition is
    // reliable for this scene. Anchored stubs alone are only supplements; in
    // narrow/partly scanned plans they cannot replace the central wall graph.
    // Without any region separator, retain the topology-filtered long walls
    // and let the anchored stubs fill their missing ends.
    if (!region_separator_segments.empty() &&
        semantic_partition_skeleton.size() >= 2 &&
        !segment_raw_free_mask.empty()) {
        const double long_partition_threshold = std::clamp(
                2.00 / safe_resolution, 28.0, 85.0);
        const double raw_room_probe = std::clamp(
                0.28 / safe_resolution, 4.0, 11.0);
        auto raw_free_at = [&](const cv::Point2d& point) {
            const int x = static_cast<int>(std::round(point.x));
            const int y = static_cast<int>(std::round(point.y));
            return x >= 0 && x < segment_raw_free_mask.cols &&
                    y >= 0 && y < segment_raw_free_mask.rows &&
                    segment_raw_free_mask.at<uchar>(y, x) != 0;
        };
        const std::vector<cv::Vec4i>& local_candidates =
                pre_topology_internal_segments;
        for (const auto& candidate : local_candidates) {
            const double length = SegmentLength(candidate);
            if (length < long_partition_threshold) continue;
            const int long_wall_support_radius = std::clamp(
                    static_cast<int>(std::round(
                            0.16 / safe_resolution)),
                    3,
                    7);
            const double source_support = SegmentSupportRatio(
                    partition_wall_evidence,
                    candidate,
                    long_wall_support_radius);
            if (source_support < 0.30) continue;
            const cv::Point2d start(candidate[0], candidate[1]);
            const cv::Point2d end(candidate[2], candidate[3]);
            const cv::Point2d delta = end - start;
            if (length <= 1e-6) continue;
            const cv::Point2d tangent = delta * (1.0 / length);
            const cv::Point2d normal(-tangent.y, tangent.x);
            int positive_room = 0;
            int negative_room = 0;
            for (const double fraction :
                 {0.18, 0.31, 0.44, 0.56, 0.69, 0.82}) {
                const cv::Point2d center = start + delta * fraction;
                positive_room += raw_free_at(
                        center + normal * raw_room_probe) ? 1 : 0;
                negative_room += raw_free_at(
                        center - normal * raw_room_probe) ? 1 : 0;
            }
            bool connects_semantic_skeleton = false;
            const double semantic_connection_distance = std::clamp(
                    0.42 / safe_resolution, 5.0, 16.0);
            for (const cv::Point2d& endpoint : {start, end}) {
                for (const auto& retained : semantic_partition_skeleton) {
                    if (point_segment_distance(endpoint, retained) <=
                        semantic_connection_distance) {
                        connects_semantic_skeleton = true;
                        break;
                    }
                }
                if (connects_semantic_skeleton) break;
            }
            const bool complete_two_sided_partition =
                    positive_room >= 4 && negative_room >= 4;
            const bool connected_one_sided_main_partition =
                    length >= 1.75 / safe_resolution &&
                    source_support >= 0.48 &&
                    std::max(positive_room, negative_room) >= 5 &&
                    std::min(positive_room, negative_room) >= 1 &&
                    connects_semantic_skeleton;
            const bool strong_partly_scanned_main_partition =
                    length >= 2.60 / safe_resolution &&
                    source_support >= 0.38 &&
                    std::max(positive_room, negative_room) >= 4 &&
                    connects_semantic_skeleton;
            if (!complete_two_sided_partition &&
                !connected_one_sided_main_partition &&
                !strong_partly_scanned_main_partition) {
                continue;
            }
            const size_t before = semantic_partition_skeleton.size();
            add_semantic_partition(candidate);
            if (semantic_partition_skeleton.size() > before) {
                ++long_partition_supplements;
            }
        }
        internal_wall_segments = semantic_partition_skeleton;
        std::cout << "[INFO] 语义主分区重建 region="
                  << region_separator_segments.size()
                  << " anchored=" << outline_anchored_stubs.size()
                  << " long_supplements=" << long_partition_supplements
                  << " final=" << internal_wall_segments.size() << "\n";
    }


    int recovered_unscanned_main_walls = 0;
    if (region_separator_segments.empty() &&
        !pre_topology_internal_segments.empty()) {
        const double recovery_length = std::clamp(
                1.45 / safe_resolution, 22.0, 65.0);
        const double independent_recovery_length = std::clamp(
                2.10 / safe_resolution, 34.0, 90.0);
        const double recovery_connection_distance = std::clamp(
                0.42 / safe_resolution, 5.0, 16.0);
        // Merge fragmented collinear runs before applying a room-scale length
        // threshold. A doorway or scan dropout often splits the main wall
        // into two individually short pieces; judging each piece first loses
        // the very wall the competitor retains.
        std::vector<cv::Vec4i> recovery_candidates =
                pre_topology_internal_segments;
        const double recovery_merge_offset = std::clamp(
                1.60 / safe_resolution, 14.0, 36.0);
        const double recovery_merge_gap = std::clamp(
                0.85 / safe_resolution, 8.0, 34.0);
        bool recovery_merged = true;
        while (recovery_merged) {
            recovery_merged = false;
            for (size_t first_index = 0;
                 first_index < recovery_candidates.size() &&
                         !recovery_merged;
                 ++first_index) {
                for (size_t second_index = first_index + 1;
                     second_index < recovery_candidates.size();
                     ++second_index) {
                    const cv::Vec4i first =
                            recovery_candidates[first_index];
                    const cv::Vec4i second =
                            recovery_candidates[second_index];
                    if (AngleDistance(LineAngle(first), LineAngle(second)) >
                        10.0) {
                        continue;
                    }
                    const cv::Point2d direction =
                            DirectionFromAngle(LineAngle(first));
                    const cv::Point2d normal(-direction.y, direction.x);
                    const double first_offset = 0.5 * (
                            cv::Point2d(first[0], first[1]).dot(normal) +
                            cv::Point2d(first[2], first[3]).dot(normal));
                    const double second_offset = 0.5 * (
                            cv::Point2d(second[0], second[1]).dot(normal) +
                            cv::Point2d(second[2], second[3]).dot(normal));
                    if (std::fabs(first_offset - second_offset) >
                        recovery_merge_offset) {
                        continue;
                    }
                    auto span = [&](const cv::Vec4i& segment) {
                        std::array<double, 2> values{{
                                cv::Point2d(segment[0], segment[1]).dot(
                                        direction),
                                cv::Point2d(segment[2], segment[3]).dot(
                                        direction)}};
                        std::sort(values.begin(), values.end());
                        return values;
                    };
                    const auto first_span = span(first);
                    const auto second_span = span(second);
                    const double overlap = std::max(
                            0.0,
                            std::min(first_span[1], second_span[1]) -
                                    std::max(first_span[0], second_span[0]));
                    const double shorter_span = std::max(
                            1.0,
                            std::min(first_span[1] - first_span[0],
                                     second_span[1] - second_span[0]));
                    // A large lateral allowance is safe only for sequential
                    // pieces of one drifted wall. Strongly overlapping runs
                    // are distinct parallel walls and must stay separate.
                    if (overlap / shorter_span > 0.28 &&
                        std::fabs(first_offset - second_offset) >
                                0.30 / safe_resolution) {
                        continue;
                    }
                    const double gap = std::max(
                            0.0,
                            std::max(first_span[0], second_span[0]) -
                                    std::min(first_span[1], second_span[1]));
                    if (gap > recovery_merge_gap) continue;
                    const double first_source_support = SegmentSupportRatio(
                            internal_structure_binary,
                            first,
                            std::clamp(
                                    static_cast<int>(std::round(
                                            0.18 / safe_resolution)),
                                    3,
                                    8));
                    const double second_source_support = SegmentSupportRatio(
                            internal_structure_binary,
                            second,
                            std::clamp(
                                    static_cast<int>(std::round(
                                            0.18 / safe_resolution)),
                                    3,
                                    8));
                    if (std::min(first_source_support,
                                 second_source_support) < 0.10) {
                        continue;
                    }
                    const double offset =
                            (first_offset * SegmentLength(first) +
                             second_offset * SegmentLength(second)) /
                            std::max(1.0,
                                     SegmentLength(first) +
                                     SegmentLength(second));
                    recovery_candidates[first_index] = BuildSegment(
                            std::min(first_span[0], second_span[0]),
                            std::max(first_span[1], second_span[1]),
                            offset,
                            direction);
                    recovery_candidates.erase(
                            recovery_candidates.begin() +
                            static_cast<long>(second_index));
                    recovery_merged = true;
                    break;
                }
            }
        }
        for (const auto& candidate : recovery_candidates) {
            const double length = SegmentLength(candidate);
            const double support = SegmentSupportRatio(
                    internal_structure_binary,
                    candidate,
                    std::clamp(
                            static_cast<int>(std::round(
                                    0.85 / safe_resolution)),
                            8,
                            20));
            if (length < recovery_length || support < 0.06) continue;
            bool already_present = false;
            for (auto& retained : internal_wall_segments) {
                if (AngleDistance(LineAngle(candidate), LineAngle(retained)) <=
                        9.0 &&
                    ProjectionOverlapRatio(candidate, retained) >= 0.40) {
                    if (length > SegmentLength(retained) * 1.20) {
                        retained = candidate;
                        ++recovered_unscanned_main_walls;
                    }
                    already_present = true;
                    break;
                }
            }
            if (already_present) continue;

            int graph_connections = 0;
            for (const cv::Point2d& endpoint : {
                    cv::Point2d(candidate[0], candidate[1]),
                    cv::Point2d(candidate[2], candidate[3])}) {
                bool connected = false;
                for (const auto& retained : internal_wall_segments) {
                    if (point_segment_distance(endpoint, retained) <=
                        recovery_connection_distance) {
                        connected = true;
                        break;
                    }
                }
                if (!connected) {
                    for (const auto& other :
                         pre_topology_internal_segments) {
                        if (&other == &candidate ||
                            AngleDistance(LineAngle(candidate),
                                          LineAngle(other)) < 40.0) {
                            continue;
                        }
                        if (point_segment_distance(endpoint, other) <=
                            recovery_connection_distance) {
                            connected = true;
                            break;
                        }
                    }
                }
                graph_connections += connected ? 1 : 0;
            }
            if (graph_connections == 0 &&
                length < independent_recovery_length) {
                continue;
            }
            internal_wall_segments.push_back(candidate);
            ++recovered_unscanned_main_walls;
        }
    }
    if (recovered_unscanned_main_walls > 0) {
        std::cout << "[INFO] 未扫描房间侧主墙恢复="
                  << recovered_unscanned_main_walls << "\n";
    }

    // Revisit every normalized pre-topology candidate, even when a few
    // region separators were already found. The previous mutually-exclusive
    // recovery path treated "some semantic walls exist" as "the semantic
    // wall graph is complete" and therefore dropped other real dividers.
    // Competitor-style plans instead keep any Manhattan run that either
    // separates two room-scale faces or has sustained free space on both
    // sides and connects to the accepted wall graph. Short scan dropouts are
    // represented by the fitted run; two wall faces still collapse through
    // the collinear merge below.
    int room_graph_partition_recoveries = 0;
    if (!all_normalized_partition_candidates.empty() &&
        !segment_raw_free_mask.empty()) {
        const double minimum_partition_length = std::clamp(
                0.58 / safe_resolution, 8.0, 28.0);
        const double independent_partition_length = std::clamp(
                1.20 / safe_resolution, 18.0, 52.0);
        const double room_probe_near = std::clamp(
                0.28 / safe_resolution, 4.0, 11.0);
        const double room_probe_far = std::clamp(
                0.48 / safe_resolution, 6.0, 17.0);
        const double graph_anchor = std::clamp(
                0.45 / safe_resolution, 5.0, 17.0);
        const double merge_offset = std::clamp(
                0.22 / safe_resolution, 3.0, 9.0);
        const double merge_gap = std::clamp(
                0.72 / safe_resolution, 7.0, 28.0);
        const int minimum_room_area = static_cast<int>(std::round(
                0.70 / (safe_resolution * safe_resolution)));
        auto raw_free_at = [&](const cv::Point2d& point) {
            const int x = static_cast<int>(std::round(point.x));
            const int y = static_cast<int>(std::round(point.y));
            return x >= 0 && x < segment_raw_free_mask.cols &&
                    y >= 0 && y < segment_raw_free_mask.rows &&
                    segment_raw_free_mask.at<uchar>(y, x) != 0;
        };
        auto room_label_near = [&](const cv::Point2d& point) {
            if (room_partition_labels.empty()) return 0;
            const int center_x = static_cast<int>(std::round(point.x));
            const int center_y = static_cast<int>(std::round(point.y));
            const int radius = std::clamp(
                    static_cast<int>(std::round(0.14 / safe_resolution)),
                    2,
                    5);
            int best_label = 0;
            int best_distance_squared = std::numeric_limits<int>::max();
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    const int x = center_x + dx;
                    const int y = center_y + dy;
                    if (x < 0 || x >= room_partition_labels.cols ||
                        y < 0 || y >= room_partition_labels.rows) {
                        continue;
                    }
                    const int label = room_partition_labels.at<int>(y, x);
                    const int distance_squared = dx * dx + dy * dy;
                    if (label > 0 &&
                        distance_squared < best_distance_squared) {
                        best_label = label;
                        best_distance_squared = distance_squared;
                    }
                }
            }
            return best_label;
        };
        auto label_area = [&](int label) {
            return label > 0 && !room_partition_stats.empty() &&
                    label < room_partition_stats.rows
                    ? room_partition_stats.at<int>(
                            label, cv::CC_STAT_AREA)
                    : 0;
        };

        for (const cv::Vec4i& candidate :
             all_normalized_partition_candidates) {
            const double length = SegmentLength(candidate);
            if (length < minimum_partition_length) continue;
            const cv::Point2d start(candidate[0], candidate[1]);
            const cv::Point2d end(candidate[2], candidate[3]);
            const cv::Point2d delta = end - start;
            if (length <= 1e-6) continue;
            const cv::Point2d tangent = delta * (1.0 / length);
            const cv::Point2d normal(-tangent.y, tangent.x);
            const cv::Point2f midpoint(
                    static_cast<float>(0.5 * (candidate[0] + candidate[2])),
                    static_cast<float>(0.5 * (candidate[1] + candidate[3])));
            if (cv::pointPolygonTest(
                        outline.original_polygon, midpoint, true) <
                0.18 / safe_resolution) {
                continue;
            }

            int near_positive = 0;
            int near_negative = 0;
            int far_positive = 0;
            int far_negative = 0;
            std::map<int, int> positive_label_votes;
            std::map<int, int> negative_label_votes;
            for (const double fraction :
                 {0.14, 0.26, 0.38, 0.50, 0.62, 0.74, 0.86}) {
                const cv::Point2d center = start + delta * fraction;
                near_positive += raw_free_at(
                        center + normal * room_probe_near) ? 1 : 0;
                near_negative += raw_free_at(
                        center - normal * room_probe_near) ? 1 : 0;
                far_positive += raw_free_at(
                        center + normal * room_probe_far) ? 1 : 0;
                far_negative += raw_free_at(
                        center - normal * room_probe_far) ? 1 : 0;
                const int positive_label = room_label_near(
                        center + normal * room_probe_far);
                const int negative_label = room_label_near(
                        center - normal * room_probe_far);
                if (positive_label > 0) {
                    ++positive_label_votes[positive_label];
                }
                if (negative_label > 0) {
                    ++negative_label_votes[negative_label];
                }
            }
            auto dominant_label = [](const std::map<int, int>& votes) {
                std::pair<int, int> best(0, 0);
                for (const auto& value : votes) {
                    if (value.second > best.second) best = value;
                }
                return best;
            };
            const auto positive_label = dominant_label(
                    positive_label_votes);
            const auto negative_label = dominant_label(
                    negative_label_votes);
            const bool separates_room_faces =
                    positive_label.first > 0 &&
                    negative_label.first > 0 &&
                    positive_label.first != negative_label.first &&
                    positive_label.second >= 2 &&
                    negative_label.second >= 2 &&
                    label_area(positive_label.first) >= minimum_room_area &&
                    label_area(negative_label.first) >= minimum_room_area;
            const bool sustained_two_sided_free =
                    std::min(near_positive, near_negative) >= 4 &&
                    std::min(far_positive, far_negative) >= 3;

            int graph_connections = 0;
            for (const cv::Point2d& endpoint : {start, end}) {
                bool connected = std::fabs(cv::pointPolygonTest(
                        outline.original_polygon,
                        cv::Point2f(endpoint), true)) <= graph_anchor;
                for (const auto& retained : internal_wall_segments) {
                    if (!connected && point_segment_distance(
                                endpoint, retained) <= graph_anchor) {
                        connected = true;
                    }
                }
                graph_connections += connected ? 1 : 0;
            }
            const double source_support = SegmentSupportRatio(
                    internal_structure_binary,
                    candidate,
                    std::clamp(
                            static_cast<int>(std::round(
                                    0.12 / safe_resolution)),
                            2,
                            6));
            const bool connected_two_sided_partition =
                    sustained_two_sided_free &&
                    source_support >= 0.28 &&
                    (graph_connections >= 1 ||
                     length >= independent_partition_length);
            if (!separates_room_faces &&
                !connected_two_sided_partition) {
                continue;
            }

            bool merged_or_present = false;
            for (cv::Vec4i& retained : internal_wall_segments) {
                if (AngleDistance(
                            LineAngle(candidate), LineAngle(retained)) > 6.0) {
                    continue;
                }
                const cv::Point2d direction = DirectionFromAngle(
                        LineAngle(retained));
                const cv::Point2d retained_normal(
                        -direction.y, direction.x);
                auto span = [&](const cv::Vec4i& segment) {
                    std::array<double, 2> values{{
                            cv::Point2d(segment[0], segment[1]).dot(direction),
                            cv::Point2d(segment[2], segment[3]).dot(direction)}};
                    std::sort(values.begin(), values.end());
                    return values;
                };
                const double candidate_offset = 0.5 * (
                        start.dot(retained_normal) +
                        end.dot(retained_normal));
                const double retained_offset = 0.5 * (
                        cv::Point2d(retained[0], retained[1]).dot(
                                retained_normal) +
                        cv::Point2d(retained[2], retained[3]).dot(
                                retained_normal));
                if (std::fabs(candidate_offset - retained_offset) >
                    merge_offset) {
                    continue;
                }
                const auto candidate_span = span(candidate);
                const auto retained_span = span(retained);
                const double gap = std::max(
                        0.0,
                        std::max(candidate_span[0], retained_span[0]) -
                                std::min(candidate_span[1], retained_span[1]));
                if (gap > merge_gap) continue;
                retained = BuildSegment(
                        std::min(candidate_span[0], retained_span[0]),
                        std::max(candidate_span[1], retained_span[1]),
                        (candidate_offset * length +
                         retained_offset * SegmentLength(retained)) /
                                std::max(1.0,
                                         length + SegmentLength(retained)),
                        direction);
                merged_or_present = true;
                break;
            }
            if (!merged_or_present) {
                internal_wall_segments.push_back(candidate);
                ++room_graph_partition_recoveries;
            }
        }
    }
    if (room_graph_partition_recoveries > 0) {
        std::cout << "[INFO] 房间分隔主墙统一恢复="
                  << room_graph_partition_recoveries << "\n";
    }

    // Dense scenes can contain many individually plausible short wall pieces
    // (door frames, alcove returns and obstacle sides). Competitor-style
    // plans retain the connected room-partition skeleton. Apply this graph
    // reduction only when the accepted set is already dense; sparse scenes
    // keep all verified partitions.
    int partition_graph_pruned = 0;
    if (internal_wall_segments.size() >= 7) {
        const bool very_dense_partition_graph =
                internal_wall_segments.size() >= 13;
        const double graph_connection_distance = std::clamp(
                0.38 / safe_resolution, 5.0, 15.0);
        const double medium_partition_length = std::clamp(
                0.85 / safe_resolution, 11.0, 34.0);
        const double main_partition_length = std::clamp(
                1.20 / safe_resolution, 17.0, 50.0);
        const double independent_partition_length = std::clamp(
                2.00 / safe_resolution, 28.0, 80.0);
        std::vector<cv::Vec4i> skeleton_segments;
        for (size_t index = 0;
             index < internal_wall_segments.size(); ++index) {
            const cv::Vec4i& segment = internal_wall_segments[index];
            const double length = SegmentLength(segment);
            int connected_endpoints = 0;
            for (const cv::Point2d& endpoint : {
                    cv::Point2d(segment[0], segment[1]),
                    cv::Point2d(segment[2], segment[3])}) {
                bool connected = std::fabs(cv::pointPolygonTest(
                        outline.original_polygon,
                        cv::Point2f(endpoint), true)) <=
                        graph_connection_distance;
                for (size_t other = 0;
                     other < internal_wall_segments.size() && !connected;
                     ++other) {
                    if (other == index) continue;
                    if (point_segment_distance(
                                endpoint,
                                internal_wall_segments[other]) <=
                        graph_connection_distance) {
                        connected = true;
                    }
                }
                connected_endpoints += connected ? 1 : 0;
            }
            // In a very dense graph, short two-ended loops are usually door
            // frames or the remaining sides of a small wall band. Keep only
            // room-scale runs. Less complex scenes still retain medium
            // two-ended partitions.
            const bool keep = very_dense_partition_graph
                    ? (length >= independent_partition_length ||
                       (length >= 1.35 / safe_resolution &&
                        connected_endpoints >= 1))
                    : (length >= independent_partition_length ||
                       (length >= main_partition_length &&
                        connected_endpoints >= 1) ||
                       (length >= medium_partition_length &&
                        connected_endpoints >= 2));
            if (keep) {
                skeleton_segments.push_back(segment);
            } else {
                ++partition_graph_pruned;
            }
        }
        internal_wall_segments = std::move(skeleton_segments);
    }
    if (partition_graph_pruned > 0) {
        std::cout << "[INFO] 内墙主分区骨架筛选 removed="
                  << partition_graph_pruned
                  << " final=" << internal_wall_segments.size() << "\n";
    }

    // Semantic recovery runs after the first facade rejection and can add a
    // complete occupied run that is actually the inner face of an exterior
    // wall. Enforce the final graph invariant once more: a red segment close
    // and parallel to the green envelope is retained only when explored room
    // space is sustained on both sides. Perpendicular partitions are never
    // removed here, so genuine wall-to-facade junctions remain available for
    // endpoint extension below.
    int final_facade_segments_removed = 0;
    if (!segment_raw_free_mask.empty()) {
        const double near_facade = std::clamp(
                0.75 / safe_resolution, 8.0, 24.0);
        const double unconditional_facade = std::clamp(
                0.25 / safe_resolution, 3.0, 9.0);
        const double wall_band_facade = std::clamp(
                0.40 / safe_resolution, 5.0, 13.0);
        const double side_probe = std::clamp(
                0.28 / safe_resolution, 4.0, 11.0);
        auto free_at = [&](const cv::Point2d& point) {
            const int x = static_cast<int>(std::round(point.x));
            const int y = static_cast<int>(std::round(point.y));
            return x >= 0 && x < segment_raw_free_mask.cols &&
                    y >= 0 && y < segment_raw_free_mask.rows &&
                    segment_raw_free_mask.at<uchar>(y, x) != 0;
        };
        internal_wall_segments.erase(
                std::remove_if(
                        internal_wall_segments.begin(),
                        internal_wall_segments.end(),
                        [&](const cv::Vec4i& segment) {
                            const cv::Point2d start(
                                    segment[0], segment[1]);
                            const cv::Point2d end(
                                    segment[2], segment[3]);
                            const cv::Point2d delta = end - start;
                            const double length = cv::norm(delta);
                            if (length <= 1e-6) return true;
                            const cv::Point2d tangent =
                                    delta * (1.0 / length);
                            const cv::Point2d normal(
                                    -tangent.y, tangent.x);
                            constexpr std::array<double, 7>
                                    kFacadeSamples{{
                                            0.10, 0.23, 0.36, 0.50,
                                            0.64, 0.77, 0.90}};
                            std::array<double, 7> parallel_distances;
                            parallel_distances.fill(
                                    std::numeric_limits<double>::infinity());
                            for (size_t edge_index = 0;
                                 edge_index <
                                         outline.original_polygon.size();
                                 ++edge_index) {
                                const cv::Point2d edge_start(
                                        outline.original_polygon[edge_index]);
                                const cv::Point2d edge_end(
                                        outline.original_polygon[
                                                (edge_index + 1) %
                                                outline.original_polygon.size()]);
                                const cv::Vec4i edge_segment(
                                        static_cast<int>(edge_start.x),
                                        static_cast<int>(edge_start.y),
                                        static_cast<int>(edge_end.x),
                                        static_cast<int>(edge_end.y));
                                if (AngleDistance(
                                            LineAngle(segment),
                                            LineAngle(edge_segment)) > 12.0) {
                                    continue;
                                }
                                for (size_t sample = 0;
                                     sample < kFacadeSamples.size();
                                     ++sample) {
                                    const cv::Point2d point = start +
                                            delta * kFacadeSamples[sample];
                                    parallel_distances[sample] = std::min(
                                            parallel_distances[sample],
                                            point_segment_distance(
                                                    point,
                                                    edge_segment));
                                }
                            }
                            const int near_facade_samples =
                                    static_cast<int>(std::count_if(
                                            parallel_distances.begin(),
                                            parallel_distances.end(),
                                            [&](double distance) {
                                                return distance <= near_facade;
                                            }));
                            const int coincident_facade_samples =
                                    static_cast<int>(std::count_if(
                                            parallel_distances.begin(),
                                            parallel_distances.end(),
                                            [&](double distance) {
                                                return distance <=
                                                        unconditional_facade;
                                            }));
                            const int wall_band_facade_samples =
                                    static_cast<int>(std::count_if(
                                            parallel_distances.begin(),
                                            parallel_distances.end(),
                                            [&](double distance) {
                                                return distance <=
                                                        wall_band_facade;
                                            }));
                            // An internal wall is allowed to terminate at the
                            // facade. Only classify a line as exterior when a
                            // clear majority of its full tangent span follows
                            // the same nearby green-envelope run.
                            if (near_facade_samples < 4) {
                                return false;
                            }
                            int positive_free = 0;
                            int negative_free = 0;
                            for (const double fraction :
                                 {0.16, 0.28, 0.40, 0.50,
                                  0.60, 0.72, 0.84}) {
                                const cv::Point2d center =
                                        start + delta * fraction;
                                positive_free += free_at(
                                        center + normal * side_probe) ? 1 : 0;
                                negative_free += free_at(
                                        center - normal * side_probe) ? 1 : 0;
                            }
                            const bool sustained_two_sided_room =
                                    std::min(
                                            positive_free,
                                            negative_free) >= 4 &&
                                    std::max(
                                            positive_free,
                                            negative_free) >= 5;
                            const bool facade =
                                    coincident_facade_samples >= 4 ||
                                    wall_band_facade_samples >= 6 ||
                                    !sustained_two_sided_room;
                            if (facade) ++final_facade_segments_removed;
                            return facade;
                        }),
                internal_wall_segments.end());
    }
    if (final_facade_segments_removed > 0) {
        std::cout << "[INFO] 最终内外墙互斥移除="
                  << final_facade_segments_removed << "\n";
    }

    // Every recovery stage above works on architectural wall runs and may
    // deliberately bridge a short unsupported interval.  That is correct for
    // scan dropouts, but it can also close a real doorway after the initial
    // candidate-level trajectory veto has already run.  Build one final,
    // authoritative doorway mask from the optimized driven path.  A geometric
    // crossing alone is not enough: the path must be continuous, the semantic
    // grid must show an open corridor before/at/after the crossing, and the
    // final fused point cloud must have weak wall support locally.  This keeps
    // localization jumps and occasional pose-through-wall errors from cutting
    // holes in strongly observed walls.
    cv::Mat confirmed_trajectory_doorway_mask = cv::Mat::zeros(
            internal_structure_binary.size(), CV_8UC1);
    int confirmed_trajectory_doorways = 0;
    int rejected_strong_wall_crossings = 0;
    if (!semantic_map.empty() && trajectory_points_px.size() >= 2 &&
        !internal_wall_segments.empty()) {
        cv::Mat exact_semantic_free;
        cv::inRange(
                semantic_map,
                cv::Scalar(245, 245, 245),
                cv::Scalar(255, 255, 255),
                exact_semantic_free);
        auto free_near = [&](const cv::Point2d& point) {
            const int center_x = static_cast<int>(std::round(point.x));
            const int center_y = static_cast<int>(std::round(point.y));
            int free_pixels = 0;
            int valid_pixels = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int x = center_x + dx;
                    const int y = center_y + dy;
                    if (x < 0 || x >= exact_semantic_free.cols ||
                        y < 0 || y >= exact_semantic_free.rows) {
                        continue;
                    }
                    ++valid_pixels;
                    free_pixels += exact_semantic_free.at<uchar>(y, x) != 0
                            ? 1 : 0;
                }
            }
            return valid_pixels > 0 &&
                    free_pixels * 2 >= valid_pixels;
        };
        const double maximum_driven_step = std::clamp(
                1.20 / safe_resolution, 16.0, 60.0);
        const double minimum_driven_step = std::clamp(
                0.025 / safe_resolution, 0.35, 1.5);
        const double corridor_probe = std::clamp(
                0.16 / safe_resolution, 2.0, 7.0);
        const double local_wall_half_length = std::clamp(
                0.24 / safe_resolution, 3.0, 10.0);
        const int local_wall_thickness = std::clamp(
                static_cast<int>(std::round(0.07 / safe_resolution)),
                1,
                3);
        const int doorway_radius = std::clamp(
                static_cast<int>(std::round(0.31 / safe_resolution)),
                4,
                13);
        std::vector<cv::Point2d> accepted_doorway_centers;
        for (const cv::Vec4i& wall : internal_wall_segments) {
            const cv::Point2d wall_start(wall[0], wall[1]);
            const cv::Point2d wall_end(wall[2], wall[3]);
            const cv::Point2d wall_delta = wall_end - wall_start;
            const double wall_length = cv::norm(wall_delta);
            if (wall_length <= 1e-6) continue;
            const cv::Point2d wall_direction =
                    wall_delta * (1.0 / wall_length);
            for (size_t path_index = 1;
                 path_index < trajectory_points_px.size();
                 ++path_index) {
                const cv::Point2d path_start(
                        trajectory_points_px[path_index - 1]);
                const cv::Point2d path_end(
                        trajectory_points_px[path_index]);
                const cv::Point2d path_delta = path_end - path_start;
                const double path_length = cv::norm(path_delta);
                if (path_length < minimum_driven_step ||
                    path_length > maximum_driven_step) {
                    continue;
                }
                const double denominator =
                        wall_delta.x * path_delta.y -
                        wall_delta.y * path_delta.x;
                if (std::fabs(denominator) < 1e-6) continue;
                const double crossing_sine = std::fabs(denominator) /
                        (wall_length * path_length);
                // A path grazing along a wall is not a doorway crossing.
                if (crossing_sine < std::sin(25.0 * kPi / 180.0)) continue;
                const cv::Point2d between = path_start - wall_start;
                const double wall_fraction =
                        (between.x * path_delta.y -
                         between.y * path_delta.x) / denominator;
                const double path_fraction =
                        (between.x * wall_delta.y -
                         between.y * wall_delta.x) / denominator;
                if (wall_fraction <= 0.04 || wall_fraction >= 0.96 ||
                    path_fraction < 0.0 || path_fraction > 1.0) {
                    continue;
                }
                const cv::Point2d intersection =
                        wall_start + wall_delta * wall_fraction;
                const cv::Point2d path_direction =
                        path_delta * (1.0 / path_length);
                int free_corridor_samples = 0;
                for (const double offset :
                     {-corridor_probe, -0.5 * corridor_probe, 0.0,
                       0.5 * corridor_probe, corridor_probe}) {
                    free_corridor_samples += free_near(
                            intersection + path_direction * offset) ? 1 : 0;
                }
                if (free_corridor_samples < 4) continue;

                const cv::Point2d support_start = intersection -
                        wall_direction * local_wall_half_length;
                const cv::Point2d support_end = intersection +
                        wall_direction * local_wall_half_length;
                const cv::Vec4i local_wall(
                        static_cast<int>(std::round(support_start.x)),
                        static_cast<int>(std::round(support_start.y)),
                        static_cast<int>(std::round(support_end.x)),
                        static_cast<int>(std::round(support_end.y)));
                const double observed_support = SegmentSupportRatio(
                        visual_wall_binary,
                        local_wall,
                        local_wall_thickness);
                if (observed_support >= 0.30) {
                    ++rejected_strong_wall_crossings;
                    continue;
                }

                bool duplicate_doorway = false;
                for (const cv::Point2d& accepted :
                     accepted_doorway_centers) {
                    if (cv::norm(accepted - intersection) <=
                        doorway_radius * 0.55) {
                        duplicate_doorway = true;
                        break;
                    }
                }
                if (duplicate_doorway) continue;
                cv::circle(
                        confirmed_trajectory_doorway_mask,
                        cv::Point(
                                static_cast<int>(std::round(intersection.x)),
                                static_cast<int>(std::round(intersection.y))),
                        doorway_radius,
                        cv::Scalar(255),
                        cv::FILLED,
                        cv::LINE_8);
                accepted_doorway_centers.push_back(intersection);
                ++confirmed_trajectory_doorways;
            }
        }
        if (!debug_dir.empty()) {
            cv::Mat doorway_debug = original_map.clone();
            for (size_t index = 1;
                 index < trajectory_points_px.size(); ++index) {
                const cv::Point2f first = trajectory_points_px[index - 1];
                const cv::Point2f second = trajectory_points_px[index];
                if (cv::norm(second - first) > maximum_driven_step) continue;
                cv::line(
                        doorway_debug,
                        first,
                        second,
                        cv::Scalar(255, 220, 0),
                        1,
                        cv::LINE_AA);
            }
            doorway_debug.setTo(
                    cv::Scalar(0, 180, 255),
                    confirmed_trajectory_doorway_mask);
            cv::imwrite(
                    PathJoin(debug_dir,
                             "confirmed_trajectory_doorways.png"),
                    doorway_debug);
            cv::imwrite(
                    PathJoin(debug_dir,
                             "confirmed_trajectory_doorway_mask.png"),
                    confirmed_trajectory_doorway_mask);
        }
    }
    if (confirmed_trajectory_doorways > 0 ||
        rejected_strong_wall_crossings > 0) {
        std::cout << "[INFO] 轨迹门洞最终校验 opened="
                  << confirmed_trajectory_doorways
                  << " kept_strong_wall="
                  << rejected_strong_wall_crossings << "\n";
    }

    // Build a wall-only visual base from the accepted structural graph. Raw
    // long Hough lines and elongated connected components still admit desks,
    // shelves and dense scan tangles, especially in cluttered rooms. Keep the
    // observed black pixels only in a narrow corridor around the fitted outer
    // boundary or an accepted internal partition. The pixels remain genuine
    // observations, but unrelated obstacle returns no longer dominate the
    // final report.
    const double visual_resolution =
            std::max(1e-4, std::isfinite(meters_per_pixel)
                    ? meters_per_pixel : 0.05);
    const int exterior_visual_radius = std::clamp(
            static_cast<int>(std::round(0.20 / visual_resolution)), 3, 9);
    const int internal_visual_radius = std::clamp(
            static_cast<int>(std::round(0.16 / visual_resolution)), 2, 7);
    cv::Mat visual_wall_support = cv::Mat::zeros(
            internal_structure_binary.size(), CV_8UC1);
    cv::polylines(
            visual_wall_support,
            std::vector<std::vector<cv::Point>>{outline.original_polygon},
            true,
            cv::Scalar(255),
            exterior_visual_radius * 2 + 1,
            cv::LINE_8);
    for (const auto& segment : internal_wall_segments) {
        cv::line(
                visual_wall_support,
                {segment[0], segment[1]},
                {segment[2], segment[3]},
                cv::Scalar(255),
                internal_visual_radius * 2 + 1,
                cv::LINE_8);
    }
    cv::Mat wall_only_binary;
    cv::bitwise_and(
            internal_structure_binary,
            visual_wall_support,
            wall_only_binary);
    cv::morphologyEx(
            wall_only_binary,
            wall_only_binary,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    // The green polygon is the fitted building boundary, while the underlying
    // raster remains the complete Cartographer point cloud.  Geometry filters
    // decide which observations define the footprint; presentation must not
    // erase other measured returns or truncate the bottom of the scan.
    cv::Mat footprint_mask = cv::Mat::zeros(
            wall_only_binary.size(), CV_8UC1);
    cv::fillPoly(
            footprint_mask,
            std::vector<std::vector<cv::Point>>{outline.original_polygon},
            cv::Scalar(255));
    cv::imwrite(
            PathJoin(debug_dir, "wall_only_visual_unclipped.png"),
            wall_only_binary);
    cv::bitwise_and(wall_only_binary, footprint_mask, wall_only_binary);
    cv::imwrite(
            PathJoin(debug_dir, "wall_only_visual_mask.png"),
            wall_only_binary);
    // Keep the complete Cartographer occupancy image as both the report base
    // and a diagnostic artifact.  Do not gray pixels outside the fitted
    // polygon: users need to compare the red/green reconstruction against all
    // original black observations.
    cv::Mat rendered = original_map.clone();
    if (rendered.empty()) {
        rendered = cv::Mat(
                internal_structure_binary.size(),
                CV_8UC3,
                cv::Scalar(154, 154, 154));
        rendered.setTo(cv::Scalar(0, 0, 0), internal_structure_binary);
    }
    const std::string raw_diagnostic_path = PathJoin(
            fs::path(output_path).parent_path().string(),
            "raw_occupancy_unclipped.png");
    cv::imwrite(raw_diagnostic_path, rendered);
    // Keep the legacy filename for Android compatibility; its content is now
    // intentionally the full, unclipped occupancy raster.
    cv::imwrite(
            PathJoin(fs::path(output_path).parent_path().string(),
                     "occupancy_clipped.png"),
            rendered);
    const int internal_line_thickness = std::clamp(
            static_cast<int>(std::round(
                    0.06 / std::max(
                            1e-4,
                            std::isfinite(meters_per_pixel)
                                    ? meters_per_pixel
                                    : 0.05))),
            2,
            5);
    // The visible wall raster and the red annotation share one source of
    // truth, but a thick wall region must still become exactly one red
    // centerline.  Merge returns from the two faces of an ordinary wall before
    // skeletonization.  The physical merge width remains well below a normal
    // corridor/door width, so separate walls and door openings stay separate.
    // Unconditionally clear only the pixels actually occupied by the green
    // stroke. A wider unconditional band would erase a real partition that
    // terminates perpendicularly at an exterior wall.
    const int exterior_clearance_radius = std::clamp(
            static_cast<int>(std::round(0.12 / visual_resolution)),
            2,
            5);
    cv::Mat exterior_band_mask = cv::Mat::zeros(
            wall_only_binary.size(), CV_8UC1);
    cv::polylines(
            exterior_band_mask,
            std::vector<std::vector<cv::Point>>{outline.original_polygon},
            true,
            cv::Scalar(255),
            exterior_clearance_radius * 2 + 1,
            cv::LINE_8);
    // The black raster remains useful visual evidence, but it is intentionally
    // not copied wholesale into the structural annotation.  Competitor-style
    // annotations describe room-separating structural walls, not every chair,
    // cabinet, wall return or isolated obstacle visible in the occupancy map.
    cv::Mat internal_wall_region = cv::Mat::zeros(
            wall_only_binary.size(), CV_8UC1);
    for (const auto& segment : internal_wall_segments) {
        cv::line(
                internal_wall_region,
                {segment[0], segment[1]},
                {segment[2], segment[3]},
                cv::Scalar(255),
                internal_line_thickness,
                cv::LINE_8);
    }
    cv::bitwise_and(
            internal_wall_region,
            footprint_mask,
            internal_wall_region);
    internal_wall_region.setTo(
            cv::Scalar(0), confirmed_trajectory_doorway_mask);

    // Classify the true building exterior from semantic topology, not merely
    // from distance to the fitted polygon.  The gray unknown region connected
    // to an image border is outside the explored building. Any wall evidence
    // touching that exterior (within normal wall thickness) is therefore an
    // exterior wall, even when the green fit is offset by noisy returns. An
    // internal wall close to the facade remains separated from this mask by
    // explored white floor and is preserved.
    cv::Mat semantic_exterior_region = cv::Mat::zeros(
            internal_wall_region.size(), CV_8UC1);
    cv::Mat semantic_exterior_wall_mask = cv::Mat::zeros(
            internal_wall_region.size(), CV_8UC1);
    if (!semantic_map.empty() &&
        semantic_map.size() == internal_wall_region.size()) {
        cv::Mat semantic_unknown;
        cv::inRange(
                semantic_map,
                cv::Scalar(80, 80, 80),
                cv::Scalar(200, 200, 200),
                semantic_unknown);
        cv::Mat unknown_labels;
        const int unknown_component_count = cv::connectedComponents(
                semantic_unknown,
                unknown_labels,
                8,
                CV_32S);
        std::vector<uchar> exterior_component(
                static_cast<size_t>(unknown_component_count), 0);
        auto mark_exterior_label = [&](int x, int y) {
            if (semantic_unknown.at<uchar>(y, x) == 0) return;
            const int label = unknown_labels.at<int>(y, x);
            if (label > 0 && label < unknown_component_count) {
                exterior_component[static_cast<size_t>(label)] = 1;
            }
        };
        for (int x = 0; x < semantic_unknown.cols; ++x) {
            mark_exterior_label(x, 0);
            mark_exterior_label(x, semantic_unknown.rows - 1);
        }
        for (int y = 0; y < semantic_unknown.rows; ++y) {
            mark_exterior_label(0, y);
            mark_exterior_label(semantic_unknown.cols - 1, y);
        }
        for (int y = 0; y < unknown_labels.rows; ++y) {
            for (int x = 0; x < unknown_labels.cols; ++x) {
                const int label = unknown_labels.at<int>(y, x);
                if (label > 0 &&
                    exterior_component[static_cast<size_t>(label)] != 0) {
                    semantic_exterior_region.at<uchar>(y, x) = 255;
                }
            }
        }
        // Exterior walls border the outside unknown component directly. Use
        // its distance field as a hard classifier as well, so isolated white
        // lidar rays cannot make an exterior wall appear to have free space
        // on both sides.
        cv::Mat exterior_inverse;
        cv::bitwise_not(semantic_exterior_region, exterior_inverse);
        cv::Mat exterior_distance;
        cv::distanceTransform(
                exterior_inverse,
                exterior_distance,
                cv::DIST_L2,
                3);
        // This is a wall-contact classifier, not a room-depth classifier.
        // 0.70 m swallowed real partitions beside unentered rooms. Limit the
        // hard exterior band to ordinary wall thickness; farther candidates
        // are handled by orientation/two-sided-free checks below.
        const double direct_exterior_distance = std::clamp(
                0.22 / visual_resolution,
                3.0,
                8.0);
        cv::Mat direct_exterior_mask;
        cv::compare(
                exterior_distance,
                direct_exterior_distance,
                direct_exterior_mask,
                cv::CMP_LE);
        cv::Mat semantic_free_for_exterior;
        cv::inRange(
                semantic_map,
                cv::Scalar(245, 245, 245),
                cv::Scalar(255, 255, 255),
                semantic_free_for_exterior);
        cv::Mat exterior_passable;
        cv::bitwise_not(semantic_free_for_exterior, exterior_passable);
        // Geodesic expansion may cross gray unknown cells and the black outer
        // wall itself, but never crosses explored white floor. This allows a
        // generous search distance for a badly offset green fit without
        // reaching a nearby internal wall across a narrow perimeter room.
        const int exterior_wall_contact_distance = std::clamp(
                static_cast<int>(std::round(0.28 / visual_resolution)),
                3,
                10);
        cv::Mat expanded_exterior = semantic_exterior_region.clone();
        const cv::Mat geodesic_kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(3, 3));
        for (int step = 0; step < exterior_wall_contact_distance; ++step) {
            cv::dilate(
                    expanded_exterior,
                    expanded_exterior,
                    geodesic_kernel);
            cv::bitwise_and(
                    expanded_exterior,
                    exterior_passable,
                    expanded_exterior);
        }
        // Exterior candidates were already classified as complete segments
        // above. Keep these masks for diagnostics only; applying them per
        // pixel here would fragment a retained partition beside unknown
        // space and undo the segment-level decision.
        cv::bitwise_and(
                internal_wall_region,
                expanded_exterior,
                semantic_exterior_wall_mask);
        cv::Mat directly_exposed_wall_mask;
        cv::bitwise_and(
                internal_wall_region,
                direct_exterior_mask,
                directly_exposed_wall_mask);
        cv::bitwise_or(
                semantic_exterior_wall_mask,
                directly_exposed_wall_mask,
                semantic_exterior_wall_mask);
    }
    internal_wall_region.setTo(cv::Scalar(0), exterior_band_mask);

    // internal_wall_region is drawn from consolidated centreline segments,
    // not raw wall faces. A 0.35 m isotropic close followed by skeletonization
    // bent those straight runs back into noisy curves and joined nearby but
    // unrelated partitions. Thin the fitted line raster directly; explicit
    // collinear/T-junction gap repair below handles only supported gaps.
    cv::Mat internal_centerline = Skeletonize(internal_wall_region);
    const int maximum_spur_length = std::clamp(
            static_cast<int>(std::round(0.18 / visual_resolution)),
            3,
            8);
    internal_centerline = PruneSkeletonIterative(
            internal_centerline,
            maximum_spur_length);
    // Re-apply the exact exterior exclusion after morphology/skeletonization
    // so no red stroke can overlap the green line.
    internal_centerline.setTo(cv::Scalar(0), exterior_band_mask);

    // Complete-segment classification above is authoritative. The former
    // per-pixel exterior classifier was permanently disabled because it
    // perforated accepted long partitions.
    int removed_exterior_centerline_pixels = 0;
    // Do not reapply semantic_exterior_wall_mask to retained line segments.
    // It is intentionally diagnostic after the segment-level classifier.

    // Exterior classification and skeleton pruning run after the vector wall
    // candidates were consolidated. They can therefore punch a small hole in
    // the middle of an otherwise accepted straight partition. Repair only a
    // gap that is bounded by the same structural segment on both sides, lies
    // safely inside the footprint, and is either physically tiny or still
    // supported by occupied pixels. A normal doorway remains open because it
    // is longer and has no occupied evidence through its centre.
    const int centerline_probe_radius = std::clamp(
            static_cast<int>(std::round(0.10 / visual_resolution)),
            1,
            4);
    cv::Mat centerline_proximity;
    cv::dilate(
            internal_centerline,
            centerline_proximity,
            cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(centerline_probe_radius * 2 + 1,
                             centerline_probe_radius * 2 + 1)));
    const int source_support_radius = std::clamp(
            static_cast<int>(std::round(0.08 / visual_resolution)),
            1,
            3);
    cv::Mat centerline_gap_support;
    cv::dilate(
            internal_structure_binary,
            centerline_gap_support,
            cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(source_support_radius * 2 + 1,
                             source_support_radius * 2 + 1)));
    const double maximum_centerline_gap = std::clamp(
            0.40 / visual_resolution, 4.0, 18.0);
    const double unconditional_centerline_gap = std::clamp(
            0.12 / visual_resolution, 2.0, 6.0);
    const double minimum_bridge_interior_depth = std::clamp(
            0.30 / visual_resolution, 3.0, 12.0);
    const int gap_support_thickness = std::clamp(
            static_cast<int>(std::round(0.14 / visual_resolution)),
            3,
            9);
    int bridged_internal_centerline_gaps = 0;
    for (const auto& structural_segment : internal_wall_segments) {
        cv::Point guide_start(structural_segment[0], structural_segment[1]);
        cv::Point guide_end(structural_segment[2], structural_segment[3]);
        if (!cv::clipLine(
                    cv::Rect(0, 0,
                             internal_centerline.cols,
                             internal_centerline.rows),
                    guide_start,
                    guide_end)) {
            continue;
        }
        cv::LineIterator iterator(
                internal_centerline,
                guide_start,
                guide_end,
                8);
        if (iterator.count < 3) continue;
        std::vector<cv::Point> samples;
        samples.reserve(static_cast<size_t>(iterator.count));
        for (int sample = 0; sample < iterator.count; ++sample, ++iterator) {
            samples.push_back(iterator.pos());
        }
        int last_supported = -1;
        for (int sample = 0; sample < static_cast<int>(samples.size()); ++sample) {
            const cv::Point& point = samples[static_cast<size_t>(sample)];
            if (centerline_proximity.at<uchar>(point.y, point.x) == 0) {
                continue;
            }
            if (last_supported >= 0 && sample - last_supported > 1) {
                const cv::Point bridge_start =
                        samples[static_cast<size_t>(last_supported)];
                const cv::Point bridge_end = point;
                const double gap_length = cv::norm(bridge_end - bridge_start);
                if (gap_length <= maximum_centerline_gap) {
                    const cv::Point2f midpoint(
                            0.5f * (bridge_start.x + bridge_end.x),
                            0.5f * (bridge_start.y + bridge_end.y));
                    const double interior_depth = cv::pointPolygonTest(
                            outline.original_polygon,
                            midpoint,
                            true);
                    const cv::Vec4i bridge(
                            bridge_start.x,
                            bridge_start.y,
                            bridge_end.x,
                            bridge_end.y);
                    const double bridge_support = SegmentSupportRatio(
                            centerline_gap_support,
                            bridge,
                            gap_support_thickness);
                    if (interior_depth >= minimum_bridge_interior_depth &&
                        (gap_length <= unconditional_centerline_gap ||
                         bridge_support >= 0.12)) {
                        cv::line(
                                internal_centerline,
                                bridge_start,
                                bridge_end,
                                cv::Scalar(255),
                                1,
                                cv::LINE_8);
                        ++bridged_internal_centerline_gaps;
                    }
                }
            }
            last_supported = sample;
        }
    }
    // Gap repair above may only restore occupied, sub-wall-size dropouts.  A
    // trajectory-confirmed open corridor is authoritative and is carved after
    // that repair so no later morphology can silently close the doorway.
    internal_centerline.setTo(
            cv::Scalar(0), confirmed_trajectory_doorway_mask);

    cv::Mat internal_annotation_mask;
    const int annotation_stroke_size =
            internal_line_thickness % 2 == 0
                    ? internal_line_thickness + 1
                    : internal_line_thickness;
    cv::dilate(
            internal_centerline,
            internal_annotation_mask,
            cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(annotation_stroke_size,
                             annotation_stroke_size)));
    internal_annotation_mask.setTo(cv::Scalar(0), exterior_band_mask);
    cv::Mat annotation_labels, annotation_stats, annotation_centroids;
    const int annotation_component_count = cv::connectedComponentsWithStats(
            internal_annotation_mask,
            annotation_labels,
            annotation_stats,
            annotation_centroids,
            8,
            CV_32S);
    const int minimum_annotation_area = annotation_stroke_size * std::clamp(
            static_cast<int>(std::round(0.12 / visual_resolution)),
            2,
            6);
    for (int label = 1; label < annotation_component_count; ++label) {
        if (annotation_stats.at<int>(label, cv::CC_STAT_AREA) <
            minimum_annotation_area) {
            internal_annotation_mask.setTo(
                    cv::Scalar(0),
                    annotation_labels == label);
        }
    }

    // Restore genuine perpendicular wall-to-facade junctions after the
    // exterior suppression pass. That pass correctly removes red exterior
    // faces, but its unconditional green protection band also used to leave a
    // visible white gap at the end of a real room divider. Extend only along
    // an already accepted partition, only to a nearby intersected polygon
    // edge, and only when the two directions are close to perpendicular.
    cv::Mat outline_junction_mask = cv::Mat::zeros(
            internal_annotation_mask.size(), CV_8UC1);
    const double maximum_junction_extension = std::clamp(
            0.55 / visual_resolution, 5.0, 22.0);
    int restored_outline_junctions = 0;
    for (const auto& segment : internal_wall_segments) {
        const double segment_angle = LineAngle(segment);
        for (const bool first_endpoint : {true, false}) {
            const cv::Point2d endpoint(
                    first_endpoint ? segment[0] : segment[2],
                    first_endpoint ? segment[1] : segment[3]);
            const cv::Point2d other(
                    first_endpoint ? segment[2] : segment[0],
                    first_endpoint ? segment[3] : segment[1]);
            cv::Point2d ray = endpoint - other;
            const double ray_norm = cv::norm(ray);
            if (ray_norm < 1.0) continue;
            ray *= 1.0 / ray_norm;

            double best_distance = std::numeric_limits<double>::infinity();
            cv::Point2d best_intersection;
            for (size_t edge_index = 0;
                 edge_index < outline.original_polygon.size();
                 ++edge_index) {
                const cv::Point polygon_start =
                        outline.original_polygon[edge_index];
                const cv::Point polygon_end = outline.original_polygon[
                        (edge_index + 1) % outline.original_polygon.size()];
                const cv::Point2d edge_start(polygon_start);
                const cv::Point2d edge_end(polygon_end);
                const cv::Point2d edge = edge_end - edge_start;
                const double cross = ray.x * edge.y - ray.y * edge.x;
                if (std::fabs(cross) < 1e-6) continue;
                const cv::Point2d delta = edge_start - endpoint;
                const double ray_distance =
                        (delta.x * edge.y - delta.y * edge.x) / cross;
                const double edge_fraction =
                        (delta.x * ray.y - delta.y * ray.x) / cross;
                if (ray_distance < -1.0 ||
                    ray_distance > maximum_junction_extension ||
                    edge_fraction < -0.02 || edge_fraction > 1.02) {
                    continue;
                }
                const cv::Vec4i polygon_edge(
                        polygon_start.x,
                        polygon_start.y,
                        polygon_end.x,
                        polygon_end.y);
                if (AngleDistance(segment_angle, LineAngle(polygon_edge)) <
                    55.0) {
                    continue;
                }
                if (std::max(0.0, ray_distance) < best_distance) {
                    best_distance = std::max(0.0, ray_distance);
                    best_intersection = endpoint + ray * ray_distance;
                }
            }
            if (!std::isfinite(best_distance)) continue;
            cv::line(
                    outline_junction_mask,
                    cv::Point(static_cast<int>(std::round(endpoint.x)),
                              static_cast<int>(std::round(endpoint.y))),
                    cv::Point(static_cast<int>(std::round(best_intersection.x)),
                              static_cast<int>(std::round(best_intersection.y))),
                    cv::Scalar(255),
                    annotation_stroke_size,
                    cv::LINE_8);
            ++restored_outline_junctions;
        }
    }
    cv::bitwise_and(
            outline_junction_mask,
            footprint_mask,
            outline_junction_mask);
    cv::bitwise_or(
            internal_annotation_mask,
            outline_junction_mask,
            internal_annotation_mask);
    internal_annotation_mask.setTo(
            cv::Scalar(0), confirmed_trajectory_doorway_mask);
    if (restored_outline_junctions > 0) {
        std::cout << "[INFO] 内墙与外轮廓交点恢复="
                  << restored_outline_junctions << "\n";
    }
    cv::imwrite(
            PathJoin(debug_dir, "internal_wall_region.png"),
            internal_wall_region);
    cv::imwrite(
            PathJoin(debug_dir, "semantic_exterior_region.png"),
            semantic_exterior_region);
    cv::imwrite(
            PathJoin(debug_dir, "semantic_exterior_wall_mask.png"),
            semantic_exterior_wall_mask);
    cv::imwrite(
            PathJoin(debug_dir, "internal_wall_centerline.png"),
            internal_centerline);
    cv::imwrite(
            PathJoin(debug_dir, "internal_annotation_mask.png"),
            internal_annotation_mask);
    rendered.setTo(cv::Scalar(0, 0, 255), internal_annotation_mask);

    // Draw the exterior last.  Fitted Hough lines remain available in debug
    // output, but are deliberately not drawn over the canonical skeleton: two
    // independent renderers were the cause of duplicate red wall lines.
    cv::polylines(
            rendered,
            std::vector<std::vector<cv::Point>>{outline.original_polygon},
            true,
            cv::Scalar(0, 255, 0),
            2,
            cv::LINE_AA);

    // Export a noise-free structural raster in exactly the same pixel space
    // as the algorithm input. Android uses this for the finalized map shown
    // after saving; the floor-plan geometry above is therefore the single
    // source of truth for both views.
    cv::Mat structural_map(
            original_map.size(), CV_8UC1, cv::Scalar(255));
    structural_map.setTo(cv::Scalar(0), internal_annotation_mask);
    cv::polylines(
            structural_map,
            std::vector<std::vector<cv::Point>>{outline.original_polygon},
            true,
            cv::Scalar(0),
            2,
            cv::LINE_AA);
    const std::string structural_path =
            PathJoin(fs::path(output_path).parent_path().string(),
                     "structural_map.png");
    if (!cv::imwrite(structural_path, structural_map)) {
        throw std::runtime_error("Unable to write finalized structural map");
    }

    EnsureDir(fs::path(output_path).parent_path().string());
    if (!cv::imwrite(output_path, rendered)) {
        throw std::runtime_error("Unable to write fitted floor-plan image");
    }
    std::cout << "闭合绿色外边界完成！顶点=" << outline.original_polygon.size()
              << " size=" << (content_right - content_left)
              << "x" << (content_bottom - content_top)
              << " internal_red=" << internal_wall_segments.size()
              << " internal_red_pixels="
              << cv::countNonZero(internal_annotation_mask)
              << " bridged_red_gaps="
              << bridged_internal_centerline_gaps
              << " exterior_red_removed="
              << removed_exterior_centerline_pixels
              << std::defaultfloat << "\n";

    PipelineResult result;
    result.output_path = output_path;
    result.structural_path = structural_path;
    result.debug_dir = debug_dir;
    result.raw_line_count = static_cast<int>(raw_segs.size());
    result.green_line_count = static_cast<int>(final_green.size());
    result.red_line_count = static_cast<int>(final_red.size());
    result.internal_wall_line_count =
            static_cast<int>(internal_wall_segments.size());
    result.internal_wall_pixel_count =
            cv::countNonZero(internal_annotation_mask);
    result.wall_pixel_count = cv::countNonZero(wall_binary);
    result.wall_containment_ratio = wall_containment_ratio;
    result.free_space_containment_ratio =
            outline.free_space_containment_ratio;
    result.outline_closed = outline.valid;
    result.outline_vertex_count = outline.vertex_count;
    result.outline_close_size = outline.close_size;
    result.outline_support_ratio = outline.support_ratio;
    result.outline_rotation_degrees = outline.rotation_degrees;
    result.outline_width_px = content_right - content_left;
    result.outline_height_px = content_bottom - content_top;
    result.outline_left_px = content_left;
    result.outline_top_px = content_top;
    result.outline_right_px = content_right;
    result.outline_bottom_px = content_bottom;
    result.dimension_center_x_px = outline.dimension_center.x;
    result.dimension_center_y_px = outline.dimension_center.y;
    result.dimension_long_axis_x = outline.long_axis.x;
    result.dimension_long_axis_y = outline.long_axis.y;
    result.dimension_short_axis_x = outline.short_axis.x;
    result.dimension_short_axis_y = outline.short_axis.y;
    result.dimension_long_size_px = outline.long_size_px;
    result.dimension_short_size_px = outline.short_size_px;
    result.footprint_area_px2 = outline.footprint_area_px2;
    result.footprint_perimeter_px = outline.footprint_perimeter_px;
    result.outline_polygon_px.reserve(outline.original_polygon.size());
    for (const auto& point : outline.original_polygon) {
        result.outline_polygon_px.emplace_back(point);
    }
    for (const auto& seg : final_green) {
        result.green_total_length += SegmentLength(seg);
    }
    for (const auto& seg : final_red) {
        result.red_total_length += SegmentLength(seg);
    }
    return result;
}

static PipelineResult RunSingleBranch(const std::string& input_path,
                               const std::string& output_path,
                               const std::string& work_dir,
                               int thresh,
                               int min_branch_length,
                               bool restore,
                               const std::string& debug_dir,
                               double meters_per_pixel,
                               const std::string& visual_input_path,
                               const std::string& semantic_input_path,
                               const std::vector<cv::Point2f>& trajectory_points_px) {
    EnsureDir(work_dir);
    EnsureDir(debug_dir);

    cv::Mat img = cv::imread(input_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) throw std::runtime_error("Missing image at " + input_path);

    const cv::Mat cleaned = ProcessMapImage(img, thresh, min_branch_length, restore);
    const std::string clean_path = PathJoin(work_dir, "clean_map.png");
    cv::imwrite(clean_path, cleaned);
    std::cout << "[INFO] 预处理完成 branch=" << min_branch_length << ": " << clean_path << "\n";

    PipelineResult result = FitFloorPlan(
            clean_path,
            visual_input_path.empty() ? input_path : visual_input_path,
            // Use the connected pre-thinning structural raster for wall
            // extraction. The visual raster remains a presentation-only base
            // so the on-screen/final rendering keeps its existing thin style.
            input_path,
            output_path,
            debug_dir,
            meters_per_pixel,
            semantic_input_path,
            trajectory_points_px);
    result.branch = min_branch_length;
    result.clean_path = clean_path;
    return result;
}

PipelineResult RunPipeline(const std::string& input_path,
                           const std::string& output_path,
                           const std::string& work_dir,
                           const std::string& debug_dir,
                           const PipelineOptions& options) {
    if (options.visual_input_path.empty() ||
        options.semantic_input_path.empty() ||
        options.trajectory_points_px.empty()) {
        throw std::invalid_argument(
                "Floor-plan generation requires visual map, semantic map, and trajectory");
    }
    const std::string actual_work_dir = work_dir.empty() ? fs::path(output_path).parent_path().string() : work_dir;
    const std::string actual_debug_dir = debug_dir.empty() ? PathJoin(actual_work_dir, "debug") : debug_dir;
    EnsureDir(actual_work_dir);
    EnsureDir(actual_debug_dir);

    cv::Mat img = cv::imread(input_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) throw std::runtime_error("Missing image at " + input_path);

    std::vector<int> branches;
    if (options.auto_branch) {
        branches = options.branch_candidates.empty()
                ? DefaultBranchCandidates(img, options.meters_per_pixel)
                : options.branch_candidates;
        if (options.min_branch_length > 0 &&
            std::find(branches.begin(), branches.end(), options.min_branch_length) == branches.end()) {
            branches.push_back(options.min_branch_length);
        }
        std::sort(branches.begin(), branches.end());
        branches.erase(std::unique(branches.begin(), branches.end()), branches.end());
    } else {
        branches = {options.min_branch_length > 0 ? options.min_branch_length : 8};
    }

    const int preferred_branch = options.min_branch_length > 0
            ? options.min_branch_length
            : std::max(4, static_cast<int>(std::round(std::min(img.rows, img.cols) * 0.01)));

    std::vector<PipelineResult> valid_results;
    if (options.auto_branch) {
        std::cout << "[INFO] 自动搜索 branch: [";
        for (size_t i = 0; i < branches.size(); ++i) {
            if (i > 0) std::cout << ", ";
            std::cout << branches[i];
        }
        std::cout << "]\n";
    }
    for (int branch : branches) {
        const std::string branch_dir = options.auto_branch ? PathJoin(actual_work_dir, "branch_" + std::to_string(branch)) : actual_work_dir;
        const std::string branch_debug = options.auto_branch ? PathJoin(branch_dir, "debug") : actual_debug_dir;
        const std::string branch_output = options.auto_branch ? PathJoin(branch_dir, "final.png") : output_path;
        PipelineResult result;
        try {
            result = RunSingleBranch(
                    input_path,
                    branch_output,
                    branch_dir,
                    options.thresh,
                    branch,
                    options.restore,
                    branch_debug,
                    options.meters_per_pixel,
                    options.visual_input_path,
                    options.semantic_input_path,
                    options.trajectory_points_px);
        } catch (const std::exception& error) {
            std::cout << "[WARN] branch=" << branch
                      << " failed: " << error.what() << "\n";
            continue;
        }
        result.score = ScorePipelineResult(result, preferred_branch);
        if (options.auto_branch) {
            std::cout << "[INFO] branch=" << branch
                      << " score=" << std::fixed << std::setprecision(2) << result.score
                      << " green=" << result.green_line_count
                      << " red=" << result.red_line_count
                      << " internal=" << result.internal_wall_line_count
                      << "/" << result.internal_wall_pixel_count << "px"
                      << " raw=" << result.raw_line_count
                      << std::defaultfloat << "\n";
        }

        valid_results.push_back(std::move(result));
    }

    if (valid_results.empty()) {
        throw std::runtime_error("All floor-plan pruning branches failed");
    }

    if (options.auto_branch) {
        ApplyConsensusScores(&valid_results, img.size(), preferred_branch);
    }
    const auto best_iterator = std::max_element(
            valid_results.begin(),
            valid_results.end(),
            [](const PipelineResult& first, const PipelineResult& second) {
                return first.score < second.score;
            });
    PipelineResult best = *best_iterator;

    if (options.auto_branch) {
        // Select exterior topology independently from internal-wall detail.
        // Skeleton pruning is allowed to decide which verified red partitions
        // survive, but it must not buy a noisier green facade merely by
        // retaining more red pixels.  Among candidates that are close to the
        // best wall/free/trajectory support, use the simplest footprint. This
        // removes scan-specific shallow steps while keeping every genuinely
        // supported wing needed to contain the explored room.
        double maximum_wall_containment = 0.0;
        double maximum_free_containment = 0.0;
        double maximum_outline_support = 0.0;
        for (const auto& candidate : valid_results) {
            maximum_wall_containment = std::max(
                    maximum_wall_containment,
                    candidate.wall_containment_ratio);
            maximum_free_containment = std::max(
                    maximum_free_containment,
                    candidate.free_space_containment_ratio);
            maximum_outline_support = std::max(
                    maximum_outline_support,
                    candidate.outline_support_ratio);
        }
        const PipelineResult* outline_choice = nullptr;
        for (const auto& candidate : valid_results) {
            if (!candidate.outline_closed ||
                candidate.outline_polygon_px.size() < 4 ||
                // Consensus score already combines cross-branch overlap,
                // dimensions, wall containment and retained room detail. A
                // materially weaker candidate must not replace that result
                // merely because it has fewer corners. This was the source
                // of scan-to-scan footprint shrinkage in otherwise identical
                // rooms.
                candidate.score < best.score - 8.0 ||
                // Simplicity is only a tie-breaker between geometrically
                // equivalent outlines.  The previous 15%/8% allowances let a
                // four-corner main rectangle beat a slightly more complex
                // outline that retained a real narrow room wing.  Keep the
                // candidate near the best wall and indoor-free-space
                // coverage before preferring fewer vertices.
                candidate.wall_containment_ratio <
                        maximum_wall_containment - 0.08 ||
                candidate.free_space_containment_ratio <
                        maximum_free_containment - 0.025 ||
                candidate.outline_support_ratio <
                        maximum_outline_support - 0.06) {
                continue;
            }
            if (outline_choice == nullptr ||
                candidate.outline_vertex_count <
                        outline_choice->outline_vertex_count ||
                (candidate.outline_vertex_count ==
                         outline_choice->outline_vertex_count &&
                 candidate.score > outline_choice->score)) {
                outline_choice = &candidate;
            }
        }
        if (outline_choice == nullptr) outline_choice = &best;

        const std::string selected_branch_dir =
                fs::path(best.output_path).parent_path().string();
        const std::string best_clean = PathJoin(actual_work_dir, "best_clean_map.png");
        fs::copy_file(best.clean_path, best_clean, fs::copy_options::overwrite_existing);
        best.clean_path = best_clean;
        const std::string best_structural =
                PathJoin(actual_work_dir, "best_structural_map.png");

        const std::string presentation_input =
                options.visual_input_path.empty()
                        ? input_path
                        : options.visual_input_path;
        cv::Mat recomposed = cv::imread(
                presentation_input, cv::IMREAD_COLOR);
        cv::Mat internal_mask = cv::imread(
                PathJoin(best.debug_dir, "internal_annotation_mask.png"),
                cv::IMREAD_GRAYSCALE);
        if (recomposed.empty() || internal_mask.empty() ||
            recomposed.size() != internal_mask.size()) {
            throw std::runtime_error(
                    "Unable to load selected branch annotation mask");
        }
        std::vector<cv::Point> outline_polygon;
        outline_polygon.reserve(outline_choice->outline_polygon_px.size());
        for (const auto& point : outline_choice->outline_polygon_px) {
            outline_polygon.emplace_back(
                    static_cast<int>(std::round(point.x)),
                    static_cast<int>(std::round(point.y)));
        }
        cv::Mat footprint = cv::Mat::zeros(internal_mask.size(), CV_8UC1);
        cv::fillPoly(
                footprint,
                std::vector<std::vector<cv::Point>>{outline_polygon},
                cv::Scalar(255));
        cv::bitwise_and(internal_mask, footprint, internal_mask);
        recomposed.setTo(cv::Scalar(0, 0, 255), internal_mask);
        cv::polylines(
                recomposed,
                std::vector<std::vector<cv::Point>>{outline_polygon},
                true,
                cv::Scalar(0, 255, 0),
                2,
                cv::LINE_AA);
        if (!cv::imwrite(output_path, recomposed)) {
            throw std::runtime_error("Unable to write recomposed floor-plan image");
        }

        // Export architectural annotations independently in the same native
        // SLAM pixel frame. Android can then toggle this layer without an
        // opaque presentation bitmap hiding the heat map or trajectory.
        cv::Mat floorplan_overlay(
                internal_mask.size(), CV_8UC4, cv::Scalar(0, 0, 0, 0));
        floorplan_overlay.setTo(
                cv::Scalar(0, 0, 255, 255), internal_mask);
        cv::polylines(
                floorplan_overlay,
                std::vector<std::vector<cv::Point>>{outline_polygon},
                true,
                cv::Scalar(0, 255, 0, 255),
                2,
                cv::LINE_AA);
        const std::string floorplan_overlay_path =
                PathJoin(actual_work_dir, "floorplan_overlay.png");
        if (!cv::imwrite(floorplan_overlay_path, floorplan_overlay)) {
            throw std::runtime_error("Unable to write transparent floor-plan overlay");
        }

        cv::Mat structural(internal_mask.size(), CV_8UC1, cv::Scalar(255));
        structural.setTo(cv::Scalar(0), internal_mask);
        cv::polylines(
                structural,
                std::vector<std::vector<cv::Point>>{outline_polygon},
                true,
                cv::Scalar(0),
                2,
                cv::LINE_AA);
        if (!cv::imwrite(best_structural, structural)) {
            throw std::runtime_error("Unable to write recomposed structural map");
        }

        // Geometry exported to Android must describe the selected exterior,
        // while verified internal-wall statistics continue to come from the
        // independently selected detail branch.
        best.output_path = output_path;
        best.structural_path = best_structural;
        best.wall_containment_ratio =
                outline_choice->wall_containment_ratio;
        best.free_space_containment_ratio =
                outline_choice->free_space_containment_ratio;
        best.outline_closed = outline_choice->outline_closed;
        best.outline_vertex_count =
                outline_choice->outline_vertex_count;
        best.outline_close_size = outline_choice->outline_close_size;
        best.outline_support_ratio =
                outline_choice->outline_support_ratio;
        best.outline_rotation_degrees =
                outline_choice->outline_rotation_degrees;
        best.outline_width_px = outline_choice->outline_width_px;
        best.outline_height_px = outline_choice->outline_height_px;
        best.outline_left_px = outline_choice->outline_left_px;
        best.outline_top_px = outline_choice->outline_top_px;
        best.outline_right_px = outline_choice->outline_right_px;
        best.outline_bottom_px = outline_choice->outline_bottom_px;
        best.dimension_center_x_px =
                outline_choice->dimension_center_x_px;
        best.dimension_center_y_px =
                outline_choice->dimension_center_y_px;
        best.dimension_long_axis_x =
                outline_choice->dimension_long_axis_x;
        best.dimension_long_axis_y =
                outline_choice->dimension_long_axis_y;
        best.dimension_short_axis_x =
                outline_choice->dimension_short_axis_x;
        best.dimension_short_axis_y =
                outline_choice->dimension_short_axis_y;
        best.dimension_long_size_px =
                outline_choice->dimension_long_size_px;
        best.dimension_short_size_px =
                outline_choice->dimension_short_size_px;
        best.footprint_area_px2 = outline_choice->footprint_area_px2;
        best.footprint_perimeter_px =
                outline_choice->footprint_perimeter_px;
        best.outline_polygon_px = outline_choice->outline_polygon_px;
        best.outline_vertex_count = static_cast<int>(
                best.outline_polygon_px.size());
        const std::string branch_raw_diagnostic = PathJoin(
                selected_branch_dir,
                "raw_occupancy_unclipped.png");
        if (fs::exists(branch_raw_diagnostic)) {
            fs::copy_file(
                    branch_raw_diagnostic,
                    PathJoin(actual_work_dir,
                             "raw_occupancy_unclipped.png"),
                    fs::copy_options::overwrite_existing);
        }
        const std::string branch_clipped_occupancy = PathJoin(
                selected_branch_dir,
                "occupancy_clipped.png");
        if (fs::exists(branch_clipped_occupancy)) {
            fs::copy_file(
                    branch_clipped_occupancy,
                    PathJoin(actual_work_dir, "occupancy_clipped.png"),
                    fs::copy_options::overwrite_existing);
        }
        std::cout << "[INFO] 自动选择 branch=" << best.branch
                  << " outline_branch=" << outline_choice->branch
                  << " score=" << std::fixed << std::setprecision(2) << best.score
                  << std::defaultfloat << "\n";
    }
    std::cout << "[INFO] 端到端处理完成: " << output_path << "\n";
    return best;
}

}  // namespace floorplan
