#include "linefitter.h"
#include <cmath>
#include <algorithm>
#include <set>
#include <queue>
#include <map>
#include <tuple>

// 静态常量定义（C++11 要求）
constexpr double LineFitter::ANGLE_TOL;
constexpr double LineFitter::CONNECT_GAP;
constexpr double LineFitter::MIN_LINE_LEN;
constexpr int    LineFitter::MIN_GROUP_SEG_NUM;
constexpr double LineFitter::COLINEAR_OFFSET_TOL;
constexpr double LineFitter::CORNER_SNAP_GAP;
constexpr double LineFitter::MIN_CORNER_ANGLE;

// ================= 辅助函数实现 =================
double LineFitter::segmentLength(const cv::Vec4i& seg) {
    return std::hypot(seg[2] - seg[0], seg[3] - seg[1]);
}

double LineFitter::lineAngle(const cv::Vec4i& seg) {
    double ang = std::atan2(seg[3] - seg[1], seg[2] - seg[0]) * 180.0 / CV_PI;
    ang = std::fmod(ang, 180.0);
    if (ang < 0) ang += 180.0;
    return ang;
}

double LineFitter::angleDistance(double a, double b) {
    double d = std::fabs(a - b);
    d = std::fmod(d, 180.0);
    return std::min(d, 180.0 - d);
}

cv::Vec4i LineFitter::normalizeSegment(const cv::Vec4i& seg) {
    int x1 = seg[0], y1 = seg[1], x2 = seg[2], y2 = seg[3];
    // 按字典序排序：先比较 x，若相同比较 y
    if (x2 < x1 || (x2 == x1 && y2 < y1)) {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }
    return cv::Vec4i(x1, y1, x2, y2);
}

cv::Point2f LineFitter::directionFromAngle(double theta_deg) {
    double rad = theta_deg * CV_PI / 180.0;
    cv::Point2f dir(std::cos(rad), std::sin(rad));
    if (dir.x < 0 || (std::abs(dir.x) < 1e-8 && dir.y < 0)) {
        dir = -dir;
    }
    return dir;
}

double LineFitter::dominantOrientation(const std::vector<cv::Vec4i>& segs) {
    double sum_cos2 = 0.0, sum_sin2 = 0.0;
    double total_weight = 0.0;
    for (const auto& seg : segs) {
        double len = std::max(segmentLength(seg), 1.0);
        double ang_rad = lineAngle(seg) * CV_PI / 180.0;
        sum_cos2 += len * std::cos(2.0 * ang_rad);
        sum_sin2 += len * std::sin(2.0 * ang_rad);
        total_weight += len;
    }
    if (total_weight < 1e-6) return 0.0;
    double theta = 0.5 * std::atan2(sum_sin2, sum_cos2);
    if (theta < 0) theta += CV_PI;
    return theta * 180.0 / CV_PI;
}

cv::Vec4i LineFitter::buildSegment(double axisStart, double axisEnd, double offset, const cv::Point2f& dir) {
    cv::Point2f normal(-dir.y, dir.x);
    cv::Point2f p1 = dir * (float)axisStart + normal * (float)offset;
    cv::Point2f p2 = dir * (float)axisEnd   + normal * (float)offset;
    return cv::Vec4i(cvRound(p1.x), cvRound(p1.y), cvRound(p2.x), cvRound(p2.y));
}

cv::Point2f LineFitter::lineIntersection(const cv::Vec4i& seg_a, const cv::Vec4i& seg_b) {
    cv::Point2f p(seg_a[0], seg_a[1]);
    cv::Point2f r(seg_a[2] - seg_a[0], seg_a[3] - seg_a[1]);
    cv::Point2f q(seg_b[0], seg_b[1]);
    cv::Point2f s(seg_b[2] - seg_b[0], seg_b[3] - seg_b[1]);

    double cross = r.x * s.y - r.y * s.x;
    if (std::abs(cross) < 1e-6) return cv::Point2f(1e9, 1e9);  // 平行或重合

    cv::Point2f qp = q - p;
    double t = (qp.x * s.y - qp.y * s.x) / cross;
    return p + r * (float)t;
}

// ================= DBSCAN 实现 =================
std::vector<int> LineFitter::dbscan2D(const std::vector<cv::Point2f>& points, double eps, int minPts) {
    int n = points.size();
    std::vector<int> labels(n, -1);
    int clusterId = 0;

    // 预计算距离矩阵？为简单，实时计算
    auto dist = [&](int i, int j) {
        double dx = points[i].x - points[j].x;
        double dy = points[i].y - points[j].y;
        return std::hypot(dx, dy);
    };

    for (int i = 0; i < n; ++i) {
        if (labels[i] != -1) continue;

        // 查找邻域
        std::vector<int> neighbors;
        for (int j = 0; j < n; ++j) {
            if (dist(i, j) < eps) {
                neighbors.push_back(j);
            }
        }

        if ((int)neighbors.size() < minPts) {
            labels[i] = -1;   // 暂时标记为噪声
            continue;
        }

        // 扩展新簇
        labels[i] = clusterId;
        std::queue<int> q;
        for (int nb : neighbors) {
            if (labels[nb] == -1) {
                labels[nb] = clusterId;
                q.push(nb);
            } else if (labels[nb] != -1 && labels[nb] != clusterId) {
                // 已属于其他簇，忽略
                continue;
            }
        }
        while (!q.empty()) {
            int idx = q.front(); q.pop();
            // 查找 idx 的邻域
            std::vector<int> nb2;
            for (int j = 0; j < n; ++j) {
                if (dist(idx, j) < eps) {
                    nb2.push_back(j);
                }
            }
            if ((int)nb2.size() >= minPts) {
                for (int k : nb2) {
                    if (labels[k] == -1) {
                        labels[k] = clusterId;
                        q.push(k);
                    }
                }
            }
        }
        clusterId++;
    }
    return labels;
}

std::vector<int> LineFitter::dbscan1D(const std::vector<double>& values, double eps, int minPts) {
    int n = values.size();
    std::vector<int> labels(n, -1);
    int clusterId = 0;

    for (int i = 0; i < n; ++i) {
        if (labels[i] != -1) continue;
        // 查找邻域
        std::vector<int> neighbors;
        for (int j = 0; j < n; ++j) {
            if (std::abs(values[i] - values[j]) < eps) {
                neighbors.push_back(j);
            }
        }
        if ((int)neighbors.size() < minPts) {
            labels[i] = -1;
            continue;
        }
        labels[i] = clusterId;
        std::queue<int> q;
        for (int nb : neighbors) {
            if (labels[nb] == -1) {
                labels[nb] = clusterId;
                q.push(nb);
            }
        }
        while (!q.empty()) {
            int idx = q.front(); q.pop();
            std::vector<int> nb2;
            for (int j = 0; j < n; ++j) {
                if (std::abs(values[idx] - values[j]) < eps) {
                    nb2.push_back(j);
                }
            }
            if ((int)nb2.size() >= minPts) {
                for (int k : nb2) {
                    if (labels[k] == -1) {
                        labels[k] = clusterId;
                        q.push(k);
                    }
                }
            }
        }
        clusterId++;
    }
    return labels;
}

// ================= 核心算法实现 =================
std::vector<cv::Vec4i> LineFitter::extractHoughSegments(const cv::Mat& binary) {
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(binary, lines, 1, CV_PI/180, MIN_LINE_LEN, MIN_LINE_LEN, CONNECT_GAP);
    return lines;
}

std::vector<cv::Vec4i> LineFitter::deduplicateSegments(const std::vector<cv::Vec4i>& segs) {
    std::set<std::tuple<int, int, int, int>> seen;
    std::vector<cv::Vec4i> unique;
    for (const auto& seg : segs) {
        cv::Vec4i norm = normalizeSegment(seg);
        auto key = std::make_tuple(norm[0], norm[1], norm[2], norm[3]);
        if (seen.insert(key).second)
            unique.push_back(norm);
    }
    return unique;
}

std::vector<std::vector<cv::Vec4i>> LineFitter::clusterByOrientation(const std::vector<cv::Vec4i>& segs) {
    if (segs.empty()) return {};
    if (segs.size() == 1) return {segs};

    int n = segs.size();
    std::vector<cv::Point2f> features(n);
    for (int i = 0; i < n; ++i) {
        double angRad = lineAngle(segs[i]) * CV_PI / 180.0;
        features[i] = cv::Point2f(std::cos(2.0 * angRad), std::sin(2.0 * angRad));
    }
    double eps = std::max(1e-3, 2.0 * std::sin(ANGLE_TOL * CV_PI / 180.0));
    int minPts = std::min(MIN_GROUP_SEG_NUM, n);
    std::vector<int> labels = dbscan2D(features, eps, minPts);

    // 将噪声（-1）单独成组，其他同一label为一组
    std::map<int, std::vector<cv::Vec4i>> groups;
    for (int i = 0; i < n; ++i) {
        int lbl = labels[i];
        if (lbl == -1) {
            // 每个噪声单独一个组（使用负索引避免冲突）
            static int noiseId = -1000;
            groups[noiseId--].push_back(segs[i]);
        } else {
            groups[lbl].push_back(segs[i]);
        }
    }
    std::vector<std::vector<cv::Vec4i>> result;
    for (auto& kv : groups) {
        result.push_back(kv.second);
    }
    return result;
}

std::vector<cv::Vec4i> LineFitter::mergeCollinearGroup(const std::vector<cv::Vec4i>& segs) {
    // 1. 计算主导方向
    double theta = dominantOrientation(segs);
    cv::Point2f dir = directionFromAngle(theta);
    cv::Point2f normal(-dir.y, dir.x);

    // 2. 计算每个线段的投影区间和偏移量
    struct Proj {
        double t_min, t_max;
        double offset;
        double length;
    };
    std::vector<Proj> projs;
    for (const auto& seg : segs) {
        cv::Point2f p1(seg[0], seg[1]), p2(seg[2], seg[3]);
        double t1 = p1.dot(dir);
        double t2 = p2.dot(dir);
        double off = 0.5 * (p1.dot(normal) + p2.dot(normal));
        double len = segmentLength(seg);
        projs.push_back({std::min(t1, t2), std::max(t1, t2), off, len});
    }

    // 3. 对偏移量进行 DBSCAN 聚类
    std::vector<double> offsets(projs.size());
    for (size_t i = 0; i < projs.size(); ++i) offsets[i] = projs[i].offset;
    std::vector<int> offsetLabels = dbscan1D(offsets, COLINEAR_OFFSET_TOL, 1);  // min_samples=1

    // 4. 每个偏移簇内分别合并投影区间
    std::map<int, std::vector<Proj>> clusters;
    for (size_t i = 0; i < projs.size(); ++i) {
        clusters[offsetLabels[i]].push_back(projs[i]);
    }

    std::vector<cv::Vec4i> merged;
    for (auto& kv : clusters) {
        auto& cluster = kv.second;
        // 按 t_min 排序
        std::sort(cluster.begin(), cluster.end(),
                  [](const Proj& a, const Proj& b) { return a.t_min < b.t_min; });

        // 计算加权平均偏移量
        double totalLen = 0.0, weightedOffset = 0.0;
        for (const auto& p : cluster) {
            totalLen += p.length;
            weightedOffset += p.offset * p.length;
        }
        double avgOffset = (totalLen > 1e-6) ? weightedOffset / totalLen : 0.0;

        // 合并重叠或接近的区间
        double curStart = cluster[0].t_min;
        double curEnd   = cluster[0].t_max;
        for (size_t i = 1; i < cluster.size(); ++i) {
            if (cluster[i].t_min <= curEnd + CONNECT_GAP) {
                curEnd = std::max(curEnd, cluster[i].t_max);
            } else {
                if (curEnd - curStart >= MIN_LINE_LEN) {
                    merged.push_back(buildSegment(curStart, curEnd, avgOffset, dir));
                }
                curStart = cluster[i].t_min;
                curEnd   = cluster[i].t_max;
            }
        }
        if (curEnd - curStart >= MIN_LINE_LEN) {
            merged.push_back(buildSegment(curStart, curEnd, avgOffset, dir));
        }
    }
    return merged;
}

std::vector<cv::Vec4i> LineFitter::snapCorners(const std::vector<cv::Vec4i>& segs) {
    int n = segs.size();
    // 转换为可修改的端点对
    std::vector<std::pair<cv::Point2f, cv::Point2f>> snapped;
    std::vector<double> angles(n);
    for (int i = 0; i < n; ++i) {
        snapped.push_back({cv::Point2f(segs[i][0], segs[i][1]),
                           cv::Point2f(segs[i][2], segs[i][3])});
        angles[i] = lineAngle(segs[i]);
    }

    for (int i = 0; i < n; ++i) {
        for (int j = i+1; j < n; ++j) {
            double angDiff = angleDistance(angles[i], angles[j]);
            if (angDiff < MIN_CORNER_ANGLE) continue;

            cv::Point2f inter = lineIntersection(segs[i], segs[j]);
            if (inter.x == 1e9) continue;
            if (!std::isfinite(inter.x) || !std::isfinite(inter.y)) continue;

            double dist_i0 = cv::norm(snapped[i].first - inter);
            double dist_i1 = cv::norm(snapped[i].second - inter);
            double dist_j0 = cv::norm(snapped[j].first - inter);
            double dist_j1 = cv::norm(snapped[j].second - inter);

            if (std::min(dist_i0, dist_i1) > CORNER_SNAP_GAP) continue;
            if (std::min(dist_j0, dist_j1) > CORNER_SNAP_GAP) continue;

            // 移动 i 中较近的端点
            if (dist_i0 < dist_i1)
                snapped[i].first = inter;
            else
                snapped[i].second = inter;
            // 移动 j 中较近的端点
            if (dist_j0 < dist_j1)
                snapped[j].first = inter;
            else
                snapped[j].second = inter;
        }
    }

    // 重新构造线段并过滤长度不足的
    std::vector<cv::Vec4i> result;
    for (int i = 0; i < n; ++i) {
        cv::Vec4i seg(cvRound(snapped[i].first.x), cvRound(snapped[i].first.y),
                      cvRound(snapped[i].second.x), cvRound(snapped[i].second.y));
        if (segmentLength(seg) >= MIN_LINE_LEN) {
            result.push_back(seg);
        }
    }
    return result;
}

// ================= 对外接口 =================
std::vector<LineFitter::Line> LineFitter::fit(const cv::Mat& binary) {
    // 1. 霍夫线段提取
    std::vector<cv::Vec4i> rawSegments = extractHoughSegments(binary);
    if (rawSegments.empty()) return {};

    // 2. 按方向聚类
    std::vector<std::vector<cv::Vec4i>> groups = clusterByOrientation(rawSegments);

    // 3. 每组内共线合并
    std::vector<cv::Vec4i> mergedSegments;
    for (const auto& group : groups) {
        auto merged = mergeCollinearGroup(group);
        mergedSegments.insert(mergedSegments.end(), merged.begin(), merged.end());
    }

    // 4. 去重
    mergedSegments = deduplicateSegments(mergedSegments);

    // 5. 角点吸附
    mergedSegments = snapCorners(mergedSegments);

    // 6. 再次去重并过滤长度
    mergedSegments = deduplicateSegments(mergedSegments);
    std::vector<Line> finalLines;
    for (const auto& seg : mergedSegments) {
        if (segmentLength(seg) >= MIN_LINE_LEN) {
            finalLines.push_back({cv::Point2f(seg[0], seg[1]), cv::Point2f(seg[2], seg[3])});
        }
    }
    return finalLines;
}