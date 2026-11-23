#ifndef CARTOGRAPHER_ANDROID_LIDAR_POINT_TIMING_H_
#define CARTOGRAPHER_ANDROID_LIDAR_POINT_TIMING_H_

#include <cmath>
#include <cstdint>
#include <limits>

// Parser revolutions begin/end at the zero-degree crossing. Missing returns
// must keep their share of the revolution time instead of stretching the
// remaining points over the full period.
inline float LidarPointOffsetFromBoundary(float angle_degrees, float period_seconds) {
    if (!std::isfinite(angle_degrees) || angle_degrees < 0.f || angle_degrees >= 360.f ||
        !std::isfinite(period_seconds) || period_seconds <= 0.f) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return period_seconds * (angle_degrees / 360.f - 1.f);
}

// Cartographer's cloud timestamp must denote the last retained point, whose
// relative time is zero. Rebasing both timestamp and offsets preserves the
// absolute acquisition times even if filtering removes the scan's last sector.
template <typename PointContainer>
int64_t RebaseLidarPointTimes(int64_t boundary_ns, PointContainer* points) {
    if (points->empty()) return boundary_ns;
    const float last_offset = points->back().time;
    for (auto& point : *points) point.time -= last_offset;
    return boundary_ns + static_cast<int64_t>(std::llround(last_offset * 1e9));
}

#endif
