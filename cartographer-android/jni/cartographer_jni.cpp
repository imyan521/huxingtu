#include "cartographer_jni.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <jni.h>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

using namespace cartographer;
using SensorId = mapping::TrajectoryBuilderInterface::SensorId;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr float kDefaultLidarScanDurationSeconds = 0.1f;
constexpr float kMinLidarScanDurationSeconds = 0.04f;
constexpr float kMaxLidarScanDurationSeconds = 0.4f;
constexpr float kMinUsableRangeMeters = 0.10f;
constexpr float kMaxUsableRangeMeters = 8.0f;
// Display filtering only. Cartographer's probability grid and serialized map
// remain untouched. Four consistent hits are normally required to cross 0.68,
// which removes transient returns without erasing real walls.
constexpr float kMinimumDisplayedOccupiedProbability = 0.68f;
constexpr float kNeighborOccupiedProbability = 0.62f;
constexpr float kReliableOccupiedProbability = 0.76f;
constexpr int kMinimumOccupiedNeighbors = 3;

uint8_t InterpolateByte(
        const float value,
        const float lower_value,
        const float upper_value,
        const uint8_t lower_output,
        const uint8_t upper_output) {
    if (value <= lower_value) return lower_output;
    if (value >= upper_value) return upper_output;
    const float ratio = (value - lower_value) / (upper_value - lower_value);
    return static_cast<uint8_t>(std::lround(
            lower_output + ratio * (upper_output - lower_output)));
}

float DecodeTextureProbability(const uint8_t intensity, const uint8_t alpha) {
    // ProbabilityGrid::DrawToSubmapTexture() stores
    //   delta = 128 - ProbabilityToLogOddsInteger(probability)
    // as premultiplied-alpha texture bytes. Reconstruct that log-odds integer
    // and invert Cartographer's ProbabilityToLogOddsInteger() mapping.
    int log_odds_integer = 128;
    if (intensity > 0) {
        log_odds_integer = 128 - intensity;
    } else if (alpha > 1) {
        log_odds_integer = 128 + alpha;
    }
    log_odds_integer = std::max(1, std::min(255, log_odds_integer));

    const float min_log_odds =
            std::log(mapping::kMinProbability / (1.f - mapping::kMinProbability));
    const float max_log_odds =
            std::log(mapping::kMaxProbability / (1.f - mapping::kMaxProbability));
    const float normalized = (log_odds_integer - 1) / 254.f;
    const float log_odds =
            min_log_odds + normalized * (max_log_odds - min_log_odds);
    return 1.f / (1.f + std::exp(-log_odds));
}

float YawFromQuaternion(const Eigen::Quaterniond& q) {
    return static_cast<float>(std::atan2(
            2.0 * (q.w() * q.z() + q.x() * q.y()),
            1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())));
}

jint FloatBits(const float value) {
    static_assert(sizeof(jint) == sizeof(float), "Unexpected jint size");
    jint bits;
    std::memcpy(&bits, &value, sizeof(float));
    return bits;
}

jintArray BuildSubmapTextureArray(
        JNIEnv* env,
        const mapping::SubmapId& submap_id,
        const mapping::proto::SubmapQuery::Response& response,
        const transform::Rigid3d& global_submap_pose) {
    if (response.textures_size() == 0) {
        return env->NewIntArray(0);
    }

    const auto& texture = response.textures(0);
    const int width = texture.width();
    const int height = texture.height();
    const int num_pixels = width * height;
    if (width <= 0 || height <= 0 || num_pixels <= 0 || num_pixels > 2'000'000) {
        return env->NewIntArray(0);
    }

    std::string cells;
    common::FastGunzipString(texture.cells(), &cells);
    if (static_cast<int>(cells.size()) != 2 * num_pixels) {
        return env->NewIntArray(0);
    }

    const transform::Rigid3d slice_pose = transform::ToRigid3(texture.slice_pose());
    const transform::Rigid3d texture_pose = global_submap_pose * slice_pose;
    const int header_size = 9;
    // First plane keeps the existing highlighted ARGB live-map pixels. The
    // second plane preserves Cartographer's untouched texture bytes so the
    // finalized black occupancy image can be composited without reconstructing
    // probabilities from an already thresholded display bitmap.
    jintArray arr = env->NewIntArray(num_pixels * 2 + header_size);
    if (arr == nullptr) {
        return nullptr;
    }

    std::vector<jint> output(num_pixels * 2 + header_size);
    output[0] = width;
    output[1] = height;
    output[2] = response.submap_version();
    output[3] = submap_id.trajectory_id;
    output[4] = submap_id.submap_index;
    output[5] = FloatBits(static_cast<float>(texture.resolution()));
    output[6] = FloatBits(static_cast<float>(texture_pose.translation().x()));
    output[7] = FloatBits(static_cast<float>(texture_pose.translation().y()));
    output[8] = FloatBits(YawFromQuaternion(texture_pose.rotation()));

    std::vector<float> probabilities(num_pixels, 0.f);
    std::vector<uint8_t> known(num_pixels, 0);
    for (int i = 0; i < num_pixels; ++i) {
        const uint8_t intensity = static_cast<uint8_t>(cells[i * 2]);
        const uint8_t alpha = static_cast<uint8_t>(cells[i * 2 + 1]);
        output[header_size + num_pixels + i] =
                (static_cast<jint>(intensity) << 8) |
                static_cast<jint>(alpha);
        if (intensity != 0 || alpha != 0) {
            known[i] = 1;
            probabilities[i] = DecodeTextureProbability(intensity, alpha);
        }
    }

    auto has_occupied_neighborhood = [&](const int index) {
        const int x = index % width;
        const int y = index / width;
        int occupied_neighbors = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            const int sample_y = y + dy;
            if (sample_y < 0 || sample_y >= height) continue;
            for (int dx = -1; dx <= 1; ++dx) {
                const int sample_x = x + dx;
                if (sample_x < 0 || sample_x >= width) continue;
                const int sample_index = sample_y * width + sample_x;
                if (known[sample_index] != 0 &&
                    probabilities[sample_index] >=
                            kNeighborOccupiedProbability) {
                    ++occupied_neighbors;
                }
            }
        }
        return occupied_neighbors >= kMinimumOccupiedNeighbors;
    };

    for (int i = 0; i < num_pixels; ++i) {
        if (known[i] == 0) {
            output[i + header_size] = 0;
            continue;
        }
        const float probability = probabilities[i];
        uint8_t out_alpha;
        uint8_t red;
        uint8_t green;
        uint8_t blue;
        if (probability < 0.5f) {
            // Free-space evidence remains deliberately subtle. Stronger
            // repeated misses make the explored interior slightly clearer.
            out_alpha = InterpolateByte(
                    0.5f - probability,
                    0.f,
                    0.5f - mapping::kMinProbability,
                    12,
                    42);
            red = 42;
            green = 62;
            blue = 78;
        } else if (probability < kMinimumDisplayedOccupiedProbability ||
                   !has_occupied_neighborhood(i)) {
            // Weak occupancy and spatially isolated returns are not walls.
            // A one-cell-wide continuous wall still has three supported cells
            // in its 3x3 neighborhood and is preserved.
            output[i + header_size] = 0;
            continue;
        } else if (probability < kReliableOccupiedProbability) {
            // Repeated, spatially continuous occupancy.
            out_alpha = InterpolateByte(
                    probability,
                    kMinimumDisplayedOccupiedProbability,
                    kReliableOccupiedProbability,
                    90,
                    190);
            red = 170;
            green = 225;
            blue = 214;
        } else {
            // 0.72-0.90: reliable structure up to Cartographer's probability
            // clamp. Stable walls are bright; weaker occupied cells are not.
            out_alpha = InterpolateByte(
                    probability,
                    kReliableOccupiedProbability,
                    mapping::kMaxProbability,
                    200,
                    255);
            red = 215;
            green = 255;
            blue = 244;
        }

        output[i + header_size] = (out_alpha << 24) |
                                  (red << 16) |
                                  (green << 8) |
                                  blue;
    }

    env->SetIntArrayRegion(arr, 0, output.size(), output.data());
    return arr;
}

common::Time FromMonotonicAndroidElapsedRealtimeNanos(
        const int64_t timestamp_ns, int64_t* last_universal_ticks) {
    int64_t universal_ticks = timestamp_ns / 100;
    if (universal_ticks <= *last_universal_ticks) {
        universal_ticks = *last_universal_ticks + 1;
    }
    *last_universal_ticks = universal_ticks;
    return common::FromUniversal(universal_ticks);
}

}  // namespace

struct CartographerWrapper {
    std::unique_ptr<mapping::MapBuilderInterface> map_builder;
    mapping::proto::TrajectoryBuilderOptions trajectory_options;
    std::mutex data_mutex;
    std::mutex pose_mutex;
    bool has_latest_pose = false;
    bool use_imu_data = false;
    transform::Rigid3d lidar_to_tracking = transform::Rigid3d::Identity();
    transform::Rigid3d imu_to_tracking = transform::Rigid3d::Identity();
    std::atomic<int> active_trajectory_id{-1};
    std::set<int> frozen_trajectory_ids;
    int64_t last_imu_universal_ticks = 0;
    int64_t last_range_universal_ticks = 0;
    std::atomic<int64_t> range_frame_count{0};
    std::atomic<int64_t> imu_sample_count{0};
    std::atomic<int64_t> local_slam_result_count{0};
    std::atomic<int64_t> inserted_node_count{0};
    size_t relocalization_scanned_constraint_count = 0;
    int64_t relocalization_inter_constraint_count = 0;
    std::set<int> relocalization_matched_active_nodes;
    std::set<mapping::SubmapId> relocalization_matched_old_submaps;
    transform::Rigid3d latest_local_pose = transform::Rigid3d::Identity();
};

std::string jstring_to_string(JNIEnv* env, jstring jstr) {
    if (!jstr) return "";
    const char* c = env->GetStringUTFChars(jstr, nullptr);
    std::string s(c);
    env->ReleaseStringUTFChars(jstr, c);
    return s;
}

extern "C" JNIEXPORT jlong JNICALL
CartographerNativeInit(
        JNIEnv* env, jobject thiz, jstring config_dir, jstring config_base) {
    try {
        auto wrapper = std::make_unique<CartographerWrapper>();
        auto file_resolver = std::make_unique<common::ConfigurationFileResolver>(
                std::vector<std::string>{jstring_to_string(env, config_dir)});

        auto lua_dict = common::LuaParameterDictionary::NonReferenceCounted(
                file_resolver->GetFileContentOrDie(jstring_to_string(env, config_base)),
                std::move(file_resolver));

        wrapper->map_builder = mapping::CreateMapBuilder(
                mapping::CreateMapBuilderOptions(lua_dict->GetDictionary("map_builder").get()));

        wrapper->trajectory_options = mapping::CreateTrajectoryBuilderOptions(
                lua_dict->GetDictionary("trajectory_builder").get());
        if (wrapper->trajectory_options.has_trajectory_builder_2d_options()) {
            wrapper->use_imu_data =
                    wrapper->trajectory_options.trajectory_builder_2d_options().use_imu_data();
        }
        if (lua_dict->HasKey("lidar_to_tracking")) {
            wrapper->lidar_to_tracking =
                    transform::FromDictionary(lua_dict->GetDictionary("lidar_to_tracking").get());
        }
        if (lua_dict->HasKey("imu_to_tracking")) {
            wrapper->imu_to_tracking =
                    transform::FromDictionary(lua_dict->GetDictionary("imu_to_tracking").get());
        }
        const Eigen::Vector3d& lidar_translation = wrapper->lidar_to_tracking.translation();
        LOGI("Loaded android_2d.lua use_imu_data=%d lidar_to_tracking=(%.3f, %.3f, %.3f, yaw=%.3f) imu_to_tracking_yaw=%.3f",
             wrapper->use_imu_data ? 1 : 0,
             lidar_translation.x(),
             lidar_translation.y(),
             lidar_translation.z(),
             YawFromQuaternion(wrapper->lidar_to_tracking.rotation()),
             YawFromQuaternion(wrapper->imu_to_tracking.rotation()));

        return reinterpret_cast<jlong>(wrapper.release());
    } catch (...) {
        return 0;
    }
}

extern "C" JNIEXPORT jint JNICALL
CartographerNativeStartTrajectory(
        JNIEnv* env, jobject thiz, jlong handle) {
    auto* w = reinterpret_cast<CartographerWrapper*>(handle);
    if (!w) return -1;

    std::set<SensorId> sensor_ids;
    // 🌟 注意：这里你注册的名字是 "rangefinder"
    sensor_ids.insert(SensorId{SensorId::SensorType::RANGE, "rangefinder"});
    if (w->use_imu_data) {
        sensor_ids.insert(SensorId{SensorId::SensorType::IMU, "imu"});
    }

    const int trajectory_id = w->map_builder->AddTrajectoryBuilder(
            sensor_ids,
            w->trajectory_options,
            [w](int callback_trajectory_id,
                common::Time,
                transform::Rigid3d local_pose,
                sensor::RangeData,
                std::unique_ptr<const mapping::TrajectoryBuilderInterface::InsertionResult> insertion_result) {
                const int64_t local_slam_count = ++w->local_slam_result_count;
                if (insertion_result != nullptr) {
                    ++w->inserted_node_count;
                }
                {
                    std::lock_guard<std::mutex> lock(w->pose_mutex);
                    w->active_trajectory_id = callback_trajectory_id;
                    w->latest_local_pose = local_pose;
                    w->has_latest_pose = true;
                }
                if (local_slam_count % 10 == 0) {
                    LOGI("Local SLAM results=%lld inserted_nodes=%lld",
                         static_cast<long long>(local_slam_count),
                         static_cast<long long>(w->inserted_node_count.load()));
                }
            });

    {
        std::lock_guard<std::mutex> lock(w->pose_mutex);
        w->active_trajectory_id = trajectory_id;
    }
    return trajectory_id;
}

extern "C" JNIEXPORT void JNICALL
CartographerNativeAddRangefinderData(
        JNIEnv* env, jobject thiz, jlong handle, jint tid, jlong ts,
        jfloat scan_duration_seconds, jfloatArray ranges, jfloatArray angles)
{
    auto* w = reinterpret_cast<CartographerWrapper*>(handle);
    if (!w) return;

    jsize count = std::min(env->GetArrayLength(ranges), env->GetArrayLength(angles));
    if (count <= 0) return;

    jfloat* ranges_ptr = env->GetFloatArrayElements(ranges, nullptr);
    jfloat* angles_ptr = env->GetFloatArrayElements(angles, nullptr);
    if (ranges_ptr == nullptr || angles_ptr == nullptr) {
        if (ranges_ptr != nullptr) env->ReleaseFloatArrayElements(ranges, ranges_ptr, JNI_ABORT);
        if (angles_ptr != nullptr) env->ReleaseFloatArrayElements(angles, angles_ptr, JNI_ABORT);
        return;
    }

    const float scan_duration =
            std::isfinite(scan_duration_seconds)
                    ? std::clamp(scan_duration_seconds,
                                 kMinLidarScanDurationSeconds,
                                 kMaxLidarScanDurationSeconds)
                    : kDefaultLidarScanDurationSeconds;

    cartographer::sensor::TimedPointCloud point_cloud;
    point_cloud.reserve(count);

    // Ranges outside the usable interval have already been omitted by the
    // Android parser. Derive acquisition progress from the original lidar
    // angles instead of the compacted array index: otherwise a missing sector
    // is assigned almost no time and deskew bends walls while turning. D6
    // samples arrive in increasing counter-clockwise angle order.
    std::vector<float> acquisition_progress(count, 0.f);
    float total_angular_travel = 0.f;
    for (int i = 1; i < count; ++i) {
        float delta_degrees = angles_ptr[i] - angles_ptr[i - 1];
        while (delta_degrees < 0.f) delta_degrees += 360.f;
        while (delta_degrees >= 360.f) delta_degrees -= 360.f;
        // Isolated corrupt angles must not consume most of a revolution.
        if (delta_degrees <= 45.f) total_angular_travel += delta_degrees;
        acquisition_progress[i] = total_angular_travel;
    }
    const bool has_reliable_angular_progress = total_angular_travel > 180.f;

    for (int i = 0; i < count; ++i) {
        float r = ranges_ptr[i];
        if (!std::isfinite(r) || r < kMinUsableRangeMeters || r > kMaxUsableRangeMeters) {
            continue;
        }
        float a = static_cast<float>(angles_ptr[i] * kPi / 180.0);

        const Eigen::Vector3d point_lidar(
                static_cast<double>(r * std::cos(a)),
                static_cast<double>(r * std::sin(a)),
                0.0);
        const Eigen::Vector3d point_tracking = w->lidar_to_tracking * point_lidar;

        // Cartographer expects the cloud time to be the last point timestamp.
        const float ratio = has_reliable_angular_progress
                ? acquisition_progress[i] / total_angular_travel
                : (count > 1 ? i / static_cast<float>(count - 1) : 1.f);
        const float point_time_offset = -scan_duration + ratio * scan_duration;

        point_cloud.push_back({point_tracking.cast<float>(), point_time_offset});
    }

    env->ReleaseFloatArrayElements(ranges, ranges_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(angles, angles_ptr, JNI_ABORT);
    if (point_cloud.empty()) return;

    std::lock_guard<std::mutex> lock(w->data_mutex);
    auto* traj = w->map_builder->GetTrajectoryBuilder(tid);
    if (!traj) return;

    // 组装点云数据
    cartographer::sensor::TimedPointCloudData timed_point_cloud_data{
            FromMonotonicAndroidElapsedRealtimeNanos(ts, &w->last_range_universal_ticks),
            w->lidar_to_tracking.translation().cast<float>(),
            point_cloud
    };

    ++w->range_frame_count;
    traj->AddSensorData("rangefinder", timed_point_cloud_data);
}

extern "C" JNIEXPORT void JNICALL
CartographerNativeAddImuData(
        JNIEnv* env, jobject thiz, jlong handle, jint tid, jlong ts,
        jfloat ax, jfloat ay, jfloat az, jfloat wx, jfloat wy, jfloat wz)
{
    auto* w = reinterpret_cast<CartographerWrapper*>(handle);
    if (!w) return;
    if (!w->use_imu_data) return;

    const Eigen::Vector3d linear_acceleration =
            w->imu_to_tracking.rotation() * Eigen::Vector3d(ax, ay, az);
    const Eigen::Vector3d angular_velocity =
            w->imu_to_tracking.rotation() * Eigen::Vector3d(wx, wy, wz);

    std::lock_guard<std::mutex> lock(w->data_mutex);
    auto* traj = w->map_builder->GetTrajectoryBuilder(tid);
    if (!traj) return;

    ++w->imu_sample_count;
    traj->AddSensorData(
            "imu",
            sensor::ImuData{
                    FromMonotonicAndroidElapsedRealtimeNanos(ts, &w->last_imu_universal_ticks),
                    linear_acceleration,
                    angular_velocity
            });
}

extern "C" JNIEXPORT jdoubleArray JNICALL
CartographerNativeGetPose(
        JNIEnv* env, jobject thiz, jlong handle, jint tid)
{
    auto* w = reinterpret_cast<CartographerWrapper*>(handle);
    transform::Rigid3d pose = transform::Rigid3d::Identity();
    if (w) {
        transform::Rigid3d local_pose = transform::Rigid3d::Identity();
        bool has_pose = false;
        {
            std::lock_guard<std::mutex> lock(w->pose_mutex);
            const int active_trajectory_id = w->active_trajectory_id.load();
            if (w->has_latest_pose &&
                (active_trajectory_id == tid || active_trajectory_id < 0)) {
                local_pose = w->latest_local_pose;
                has_pose = true;
            }
        }

        if (has_pose) {
            std::lock_guard<std::mutex> lock(w->data_mutex);
            auto* pose_graph = w->map_builder ? w->map_builder->pose_graph() : nullptr;
            pose = pose_graph == nullptr
                    ? local_pose
                    : pose_graph->GetLocalToGlobalTransform(tid) * local_pose;
        }
    }

    jdoubleArray arr = env->NewDoubleArray(7);
    const auto& translation = pose.translation();
    const auto& rotation = pose.rotation();
    jdouble val[7] = {
            translation.x(), translation.y(), translation.z(),
            rotation.w(), rotation.x(), rotation.y(), rotation.z()
    };
    env->SetDoubleArrayRegion(arr, 0, 7, val);
    return arr;
}

extern "C" JNIEXPORT jlongArray JNICALL
CartographerNativeGetStatus(
        JNIEnv* env, jobject thiz, jlong handle)
{
    auto* w = reinterpret_cast<CartographerWrapper*>(handle);
    jlongArray arr = env->NewLongArray(5);
    jlong val[5] = {0, 0, 0, 0, 0};
    if (w) {
        val[0] = w->range_frame_count.load();
        val[1] = w->imu_sample_count.load();
        val[2] = w->local_slam_result_count.load();
        val[3] = w->inserted_node_count.load();
        {
            std::lock_guard<std::mutex> lock(w->pose_mutex);
            val[4] = w->has_latest_pose ? 1 : 0;
        }
    }
    env->SetLongArrayRegion(arr, 0, 5, val);
    return arr;
}

extern "C" JNIEXPORT jlongArray JNICALL
CartographerNativeGetRelocalizationStatus(
        JNIEnv* env, jobject thiz, jlong handle)
{
    auto* w = reinterpret_cast<CartographerWrapper*>(handle);
    // old trajectories, active trajectory id, active nodes, inter-trajectory
    // constraints, matched active nodes, matched old submaps, relocalized.
    jlong values[7] = {0, -1, 0, 0, 0, 0, 0};
    if (w && w->map_builder) {
        std::lock_guard<std::mutex> lock(w->data_mutex);
        const int active_id = w->active_trajectory_id.load();
        values[0] = static_cast<jlong>(w->frozen_trajectory_ids.size());
        values[1] = active_id;
        auto* pose_graph = w->map_builder->pose_graph();
        if (pose_graph != nullptr && active_id >= 0) {
            values[2] = static_cast<jlong>(w->inserted_node_count.load());
            const auto& constraints = pose_graph->constraints();
            if (constraints.size() < w->relocalization_scanned_constraint_count) {
                w->relocalization_scanned_constraint_count = 0;
                w->relocalization_inter_constraint_count = 0;
                w->relocalization_matched_active_nodes.clear();
                w->relocalization_matched_old_submaps.clear();
            }
            for (size_t index = w->relocalization_scanned_constraint_count;
                 index < constraints.size(); ++index) {
                const auto& constraint = constraints[index];
                if (constraint.tag != mapping::PoseGraphInterface::Constraint::INTER_SUBMAP) {
                    continue;
                }
                if (constraint.node_id.trajectory_id == active_id &&
                    w->frozen_trajectory_ids.count(constraint.submap_id.trajectory_id) > 0) {
                    ++w->relocalization_inter_constraint_count;
                    w->relocalization_matched_active_nodes.insert(
                            constraint.node_id.node_index);
                    w->relocalization_matched_old_submaps.insert(
                            constraint.submap_id);
                }
            }
            w->relocalization_scanned_constraint_count = constraints.size();
            values[3] = static_cast<jlong>(
                    w->relocalization_inter_constraint_count);
            values[4] = static_cast<jlong>(
                    w->relocalization_matched_active_nodes.size());
            values[5] = static_cast<jlong>(
                    w->relocalization_matched_old_submaps.size());
            // Multiple independent new nodes agreeing with a frozen submap are
            // substantially safer than accepting the first ambiguous match.
            values[6] = w->relocalization_matched_active_nodes.size() >= 3 &&
                        !w->relocalization_matched_old_submaps.empty() ? 1 : 0;
        }
    }

    jlongArray result = env->NewLongArray(7);
    if (result != nullptr) env->SetLongArrayRegion(result, 0, 7, values);
    return result;
}

extern "C" JNIEXPORT jintArray JNICALL
CartographerNativeGetLatestSubmapTexture(
        JNIEnv* env, jobject thiz, jlong handle)
{
    auto* w = reinterpret_cast<CartographerWrapper*>(handle);
    if (!w) {
        return env->NewIntArray(0);
    }

    std::lock_guard<std::mutex> lock(w->data_mutex);
    auto* pose_graph = w->map_builder->pose_graph();
    if (pose_graph == nullptr) {
        return env->NewIntArray(0);
    }

    const auto submap_data = pose_graph->GetAllSubmapData();
    if (submap_data.empty()) {
        return env->NewIntArray(0);
    }

    mapping::SubmapId latest_submap_id(0, 0);
    bool has_submap = false;
    for (const auto& id_data : submap_data) {
        if (!has_submap || latest_submap_id < id_data.id) {
            latest_submap_id = id_data.id;
            has_submap = true;
        }
    }
    if (!has_submap) {
        return env->NewIntArray(0);
    }

    mapping::proto::SubmapQuery::Response response;
    const std::string error = w->map_builder->SubmapToProto(latest_submap_id, &response);
    if (!error.empty() || response.textures_size() == 0) {
        return env->NewIntArray(0);
    }

    const auto latest_submap_data = submap_data.at(latest_submap_id);
    return BuildSubmapTextureArray(env, latest_submap_id, response, latest_submap_data.pose);
}

extern "C" JNIEXPORT jobjectArray JNICALL
CartographerNativeGetActiveSubmapTextures(
        JNIEnv* env, jobject thiz, jlong handle)
{
    auto* w = reinterpret_cast<CartographerWrapper*>(handle);
    jclass int_array_class = env->FindClass("[I");
    if (!w || !w->map_builder) {
        return env->NewObjectArray(0, int_array_class, nullptr);
    }

    struct TextureSnapshot {
        mapping::SubmapId id;
        mapping::proto::SubmapQuery::Response response;
        transform::Rigid3d pose;
    };
    struct SubmapMetadata {
        mapping::SubmapId id;
        transform::Rigid3d pose;
    };
    std::vector<SubmapMetadata> active_submaps;
    std::vector<TextureSnapshot> snapshots;
    {
        std::lock_guard<std::mutex> lock(w->data_mutex);
        auto* pose_graph = w->map_builder->pose_graph();
        if (pose_graph == nullptr) {
            return env->NewObjectArray(0, int_array_class, nullptr);
        }
        const int active_trajectory_id = w->active_trajectory_id.load();
        const auto submap_data = pose_graph->GetAllSubmapData();
        std::vector<mapping::SubmapId> active_ids;
        for (const auto& id_data : submap_data) {
            if (id_data.id.trajectory_id == active_trajectory_id) {
                active_ids.push_back(id_data.id);
            }
        }
        std::sort(active_ids.begin(), active_ids.end());
        // Include the just-finished predecessor as well as the two actively
        // receiving submaps. This lets the incremental display archive its
        // final texture without querying every historical submap.
        constexpr size_t kLiveDisplaySubmapCount = 3;
        const size_t first = active_ids.size() > kLiveDisplaySubmapCount
                ? active_ids.size() - kLiveDisplaySubmapCount
                : 0;
        for (size_t index = first; index < active_ids.size(); ++index) {
            active_submaps.push_back(SubmapMetadata{
                    active_ids[index],
                    submap_data.at(active_ids[index]).pose});
        }
    }
    for (const auto& metadata : active_submaps) {
        mapping::proto::SubmapQuery::Response response;
        std::string error;
        {
            // Do not block sensor insertion while decompressing/converting a
            // different submap, and yield between the two active queries.
            std::lock_guard<std::mutex> lock(w->data_mutex);
            error = w->map_builder->SubmapToProto(metadata.id, &response);
        }
        if (!error.empty() || response.textures_size() == 0) continue;
        snapshots.push_back(TextureSnapshot{
                metadata.id, std::move(response), metadata.pose});
    }

    jobjectArray result = env->NewObjectArray(
            snapshots.size(), int_array_class, nullptr);
    if (result == nullptr) return nullptr;
    for (size_t index = 0; index < snapshots.size(); ++index) {
        jintArray texture_array = BuildSubmapTextureArray(
                env,
                snapshots[index].id,
                snapshots[index].response,
                snapshots[index].pose);
        env->SetObjectArrayElement(
                result, static_cast<jsize>(index), texture_array);
        env->DeleteLocalRef(texture_array);
    }
    env->DeleteLocalRef(int_array_class);
    return result;
}

extern "C" JNIEXPORT jobjectArray JNICALL
CartographerNativeGetSubmapTextures(
        JNIEnv* env, jobject thiz, jlong handle)
{
    auto* w = reinterpret_cast<CartographerWrapper*>(handle);
    jclass int_array_class = env->FindClass("[I");
    if (!w) {
        return env->NewObjectArray(0, int_array_class, nullptr);
    }

    struct TextureSnapshot {
        mapping::SubmapId id;
        mapping::proto::SubmapQuery::Response response;
        transform::Rigid3d pose;
    };
    struct SubmapMetadata {
        mapping::SubmapId id;
        transform::Rigid3d pose;
    };
    std::vector<SubmapMetadata> all_submaps;
    std::vector<TextureSnapshot> snapshots;
    {
        std::lock_guard<std::mutex> lock(w->data_mutex);
        auto* pose_graph = w->map_builder->pose_graph();
        if (pose_graph == nullptr) {
            return env->NewObjectArray(0, int_array_class, nullptr);
        }
        const auto submap_data = pose_graph->GetAllSubmapData();
        all_submaps.reserve(submap_data.size());
        for (const auto& id_data : submap_data) {
            all_submaps.push_back(SubmapMetadata{
                    id_data.id, id_data.data.pose});
        }
    }
    snapshots.reserve(all_submaps.size());
    for (const auto& metadata : all_submaps) {
        mapping::proto::SubmapQuery::Response response;
        std::string error;
        {
            // A full display refresh may contain many historical submaps.
            // Yield the SLAM mutex between queries so incoming scans are not
            // stalled for the duration of the complete export.
            std::lock_guard<std::mutex> lock(w->data_mutex);
            error = w->map_builder->SubmapToProto(metadata.id, &response);
        }
        if (!error.empty() || response.textures_size() == 0) continue;
        snapshots.push_back(TextureSnapshot{
                metadata.id, std::move(response), metadata.pose});
    }

    jobjectArray result = env->NewObjectArray(
            snapshots.size(), int_array_class, nullptr);
    if (result == nullptr) {
        return nullptr;
    }

    for (size_t index = 0; index < snapshots.size(); ++index) {
        jintArray texture_array = BuildSubmapTextureArray(
                env,
                snapshots[index].id,
                snapshots[index].response,
                snapshots[index].pose);
        env->SetObjectArrayElement(
                result, static_cast<jsize>(index), texture_array);
        env->DeleteLocalRef(texture_array);
    }
    env->DeleteLocalRef(int_array_class);
    return result;
}

extern "C" JNIEXPORT jdoubleArray JNICALL
CartographerNativeGetTrajectoryNodePoses(
        JNIEnv* env, jobject thiz, jlong handle)
{
    auto* w = reinterpret_cast<CartographerWrapper*>(handle);
    if (!w || !w->map_builder) {
        return env->NewDoubleArray(0);
    }

    std::lock_guard<std::mutex> lock(w->data_mutex);
    auto* pose_graph = w->map_builder->pose_graph();
    if (pose_graph == nullptr) {
        return env->NewDoubleArray(0);
    }

    const auto node_poses = pose_graph->GetTrajectoryNodePoses();
    std::vector<jdouble> output;
    output.reserve(node_poses.size() * 2);
    for (const auto& id_data : node_poses) {
        const int active_trajectory_id = w->active_trajectory_id.load();
        if (active_trajectory_id >= 0 &&
            id_data.id.trajectory_id != active_trajectory_id) {
            continue;
        }
        const auto& translation = id_data.data.global_pose.translation();
        output.push_back(translation.x());
        output.push_back(translation.y());
    }

    jdoubleArray arr = env->NewDoubleArray(output.size());
    if (arr == nullptr || output.empty()) {
        return arr;
    }
    env->SetDoubleArrayRegion(arr, 0, output.size(), output.data());
    return arr;
}

extern "C" JNIEXPORT void JNICALL
CartographerNativeFinishTrajectory(
        JNIEnv* env, jobject thiz, jlong handle, jint tid)
{
    auto* w = reinterpret_cast<CartographerWrapper*>(handle);
    if (!w || !w->map_builder) return;

    std::lock_guard<std::mutex> lock(w->data_mutex);
    w->map_builder->FinishTrajectory(tid);
    w->map_builder->pose_graph()->RunFinalOptimization();
    {
        std::lock_guard<std::mutex> pose_lock(w->pose_mutex);
        if (w->active_trajectory_id == tid) {
            w->active_trajectory_id = -1;
        }
    }
}

extern "C" JNIEXPORT jboolean JNICALL
CartographerNativeSerializeState(
        JNIEnv* env, jobject thiz, jlong handle, jstring filename,
        jboolean include_unfinished_submaps)
{
    auto* w = reinterpret_cast<CartographerWrapper*>(handle);
    if (!w || !w->map_builder || !filename) {
        return JNI_FALSE;
    }

    const std::string path = jstring_to_string(env, filename);
    std::lock_guard<std::mutex> lock(w->data_mutex);
    const bool ok = w->map_builder->SerializeStateToFile(
            include_unfinished_submaps == JNI_TRUE, path);
    LOGI("SerializeStateToFile path=%s ok=%d", path.c_str(), ok ? 1 : 0);
    return ok ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jboolean JNICALL
CartographerNativeLoadMap(
        JNIEnv* env, jobject thiz, jlong handle, jstring filename,
        jboolean load_frozen_state)
{
    auto* w = reinterpret_cast<CartographerWrapper*>(handle);
    if (!w || !w->map_builder || !filename) {
        return JNI_FALSE;
    }

    const std::string path = jstring_to_string(env, filename);
    std::ifstream input(path, std::ios::binary);
    if (path.empty() || !input.good()) {
        LOGE("LoadStateFromFile cannot read path=%s", path.c_str());
        return JNI_FALSE;
    }
    input.close();

    try {
        std::lock_guard<std::mutex> lock(w->data_mutex);
        const auto trajectory_remapping = w->map_builder->LoadStateFromFile(
                path, load_frozen_state == JNI_TRUE);
        w->frozen_trajectory_ids.clear();
        if (load_frozen_state == JNI_TRUE) {
            for (const auto& id_remapping : trajectory_remapping) {
                w->frozen_trajectory_ids.insert(id_remapping.second);
            }
        }
        w->relocalization_inter_constraint_count = 0;
        w->relocalization_matched_active_nodes.clear();
        w->relocalization_matched_old_submaps.clear();
        auto* pose_graph = w->map_builder->pose_graph();
        w->relocalization_scanned_constraint_count =
                pose_graph == nullptr ? 0 : pose_graph->constraints().size();
        LOGI("LoadStateFromFile path=%s frozen=%d trajectories=%zu",
             path.c_str(), load_frozen_state == JNI_TRUE ? 1 : 0,
             trajectory_remapping.size());
        return JNI_TRUE;
    } catch (const std::exception& e) {
        LOGE("LoadStateFromFile failed path=%s error=%s", path.c_str(), e.what());
    } catch (...) {
        LOGE("LoadStateFromFile failed path=%s unknown error", path.c_str());
    }
    return JNI_FALSE;
}

extern "C" JNIEXPORT void JNICALL
CartographerNativeDestroy(
        JNIEnv* env, jobject thiz, jlong handle)
{
    delete reinterpret_cast<CartographerWrapper*>(handle);
}

namespace {

const JNINativeMethod kCartographerMethods[] = {
        {const_cast<char*>("nativeInit"), const_cast<char*>("(Ljava/lang/String;Ljava/lang/String;)J"), reinterpret_cast<void*>(CartographerNativeInit)},
        {const_cast<char*>("nativeStartTrajectory"), const_cast<char*>("(J)I"), reinterpret_cast<void*>(CartographerNativeStartTrajectory)},
        {const_cast<char*>("nativeAddImuData"), const_cast<char*>("(JIJFFFFFF)V"), reinterpret_cast<void*>(CartographerNativeAddImuData)},
        {const_cast<char*>("nativeAddRangefinderData"), const_cast<char*>("(JIJF[F[F)V"), reinterpret_cast<void*>(CartographerNativeAddRangefinderData)},
        {const_cast<char*>("nativeGetPose"), const_cast<char*>("(JI)[D"), reinterpret_cast<void*>(CartographerNativeGetPose)},
        {const_cast<char*>("nativeGetStatus"), const_cast<char*>("(J)[J"), reinterpret_cast<void*>(CartographerNativeGetStatus)},
        {const_cast<char*>("nativeGetRelocalizationStatus"), const_cast<char*>("(J)[J"), reinterpret_cast<void*>(CartographerNativeGetRelocalizationStatus)},
        {const_cast<char*>("nativeGetLatestSubmapTexture"), const_cast<char*>("(J)[I"), reinterpret_cast<void*>(CartographerNativeGetLatestSubmapTexture)},
        {const_cast<char*>("nativeGetActiveSubmapTextures"), const_cast<char*>("(J)[[I"), reinterpret_cast<void*>(CartographerNativeGetActiveSubmapTextures)},
        {const_cast<char*>("nativeGetSubmapTextures"), const_cast<char*>("(J)[[I"), reinterpret_cast<void*>(CartographerNativeGetSubmapTextures)},
        {const_cast<char*>("nativeGetTrajectoryNodePoses"), const_cast<char*>("(J)[D"), reinterpret_cast<void*>(CartographerNativeGetTrajectoryNodePoses)},
        {const_cast<char*>("nativeSerializeState"), const_cast<char*>("(JLjava/lang/String;Z)Z"), reinterpret_cast<void*>(CartographerNativeSerializeState)},
        {const_cast<char*>("nativeLoadMap"), const_cast<char*>("(JLjava/lang/String;Z)Z"), reinterpret_cast<void*>(CartographerNativeLoadMap)},
        {const_cast<char*>("nativeFinishTrajectory"), const_cast<char*>("(JI)V"), reinterpret_cast<void*>(CartographerNativeFinishTrajectory)},
        {const_cast<char*>("nativeDestroy"), const_cast<char*>("(J)V"), reinterpret_cast<void*>(CartographerNativeDestroy)},
};

}  // namespace

extern "C" JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void*) {
    JNIEnv* env = nullptr;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) return JNI_ERR;
    jclass type = env->FindClass("com/cartographer/demo/CartographerNative");
    if (type == nullptr) return JNI_ERR;
    if (env->RegisterNatives(type, kCartographerMethods,
                             sizeof(kCartographerMethods) / sizeof(kCartographerMethods[0])) != JNI_OK) {
        env->DeleteLocalRef(type);
        return JNI_ERR;
    }
    env->DeleteLocalRef(type);
    return JNI_VERSION_1_6;
}
