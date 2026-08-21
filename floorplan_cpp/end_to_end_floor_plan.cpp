#include "floor_plan.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {

std::vector<int> ParseBranchCandidates(const std::string& text) {
    std::vector<int> out;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty()) out.push_back(std::stoi(item));
    }
    return out;
}

std::vector<cv::Point2f> ReadTrajectory(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Unable to open trajectory file " + path);
    std::vector<cv::Point2f> points;
    float x = 0.f;
    float y = 0.f;
    while (input >> x >> y) points.emplace_back(x, y);
    return points;
}

void CanonicalizeDesktopReport(
        const std::string& output_path,
        const floorplan::PipelineResult& result,
        double meters_per_pixel) {
    if (result.outline_polygon_px.size() < 3) return;
    cv::Mat source = cv::imread(output_path, cv::IMREAD_COLOR);
    if (source.empty()) return;
    const cv::Point2f center(
            static_cast<float>(result.dimension_center_x_px),
            static_cast<float>(result.dimension_center_y_px));
    const double long_axis_angle = std::atan2(
            result.dimension_long_axis_y,
            result.dimension_long_axis_x) * 180.0 / 3.14159265358979323846;
    // Use one canonical report frame regardless of arbitrary SLAM yaw.
    const double correction = long_axis_angle;
    cv::Mat transform = cv::getRotationMatrix2D(center, correction, 1.0);
    // Preserve the complete occupancy raster. Cropping to the fitted green
    // polygon hid legitimate black observations at the bottom and outside an
    // imperfect outline. Rotate the four source corners and allocate a canvas
    // large enough for every source pixel instead.
    std::vector<cv::Point2f> source_corners{
            {0.f, 0.f},
            {static_cast<float>(source.cols), 0.f},
            {static_cast<float>(source.cols), static_cast<float>(source.rows)},
            {0.f, static_cast<float>(source.rows)}};
    std::vector<cv::Point2f> transformed;
    cv::transform(source_corners, transformed, transform);
    float min_x = transformed.front().x;
    float max_x = transformed.front().x;
    float min_y = transformed.front().y;
    float max_y = transformed.front().y;
    for (const auto& point : transformed) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    const int padding = 2;
    const int width = std::max(
            3,
            static_cast<int>(std::ceil(max_x - min_x)) + padding * 2 + 1);
    const int height = std::max(
            3,
            static_cast<int>(std::ceil(max_y - min_y)) + padding * 2 + 1);
    transform.at<double>(0, 2) += padding - min_x;
    transform.at<double>(1, 2) += padding - min_y;
    cv::Mat canonical;
    // Occupancy reports use gray unknown space.  Preserve that canvas color
    // through canonical rotation instead of introducing white corner wedges.
    // Wall-only legacy inputs still have a white top-left pixel and therefore
    // retain their former white border automatically.
    const cv::Vec3b corner = source.at<cv::Vec3b>(0, 0);
    cv::warpAffine(
            source,
            canonical,
            transform,
            cv::Size(width, height),
            cv::INTER_LINEAR,
            cv::BORDER_CONSTANT,
            cv::Scalar(corner[0], corner[1], corner[2]));
    if (!cv::imwrite(output_path, canonical)) {
        throw std::runtime_error("Unable to write canonical desktop report");
    }
}

void PrintUsage(const char* argv0) {
    std::cerr
            << "Usage: " << argv0 << " --input in.png --visual visual.png "
            << "--semantic semantic.png --trajectory trajectory.txt "
            << "--output out.png "
            << "[--work-dir dir] [--debug-dir dir] [--thresh 200] "
            << "[--branch 8] [--branch-candidates 4,6,8,12] "
            << "[--resolution 0.05] [--no-auto-branch] [--restore]\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string input;
    std::string output;
    std::string work_dir;
    std::string debug_dir;
    floorplan::PipelineOptions options;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + name);
            return argv[++i];
        };
        if (arg == "--input") input = require_value("--input");
        else if (arg == "--output") output = require_value("--output");
        else if (arg == "--visual") {
            options.visual_input_path = require_value("--visual");
        }
        else if (arg == "--semantic") {
            options.semantic_input_path = require_value("--semantic");
        }
        else if (arg == "--trajectory") {
            options.trajectory_points_px =
                    ReadTrajectory(require_value("--trajectory"));
        }
        else if (arg == "--work-dir") work_dir = require_value("--work-dir");
        else if (arg == "--debug-dir") debug_dir = require_value("--debug-dir");
        else if (arg == "--thresh") options.thresh = std::stoi(require_value("--thresh"));
        else if (arg == "--branch") options.min_branch_length = std::stoi(require_value("--branch"));
        else if (arg == "--branch-candidates") options.branch_candidates = ParseBranchCandidates(require_value("--branch-candidates"));
        else if (arg == "--resolution") options.meters_per_pixel = std::stod(require_value("--resolution"));
        else if (arg == "--no-auto-branch") options.auto_branch = false;
        else if (arg == "--restore") options.restore = true;
        else {
            PrintUsage(argv[0]);
            return 2;
        }
    }

    if (input.empty() || output.empty() || options.visual_input_path.empty() ||
        options.semantic_input_path.empty() ||
        options.trajectory_points_px.empty()) {
        PrintUsage(argv[0]);
        return 2;
    }

    try {
        const auto result = floorplan::RunPipeline(input, output, work_dir, debug_dir, options);
        // Desktop regression images use the same presentation-only Manhattan
        // correction as Android. Native geometry itself remains in the SLAM
        // pixel frame, so map overlays and measurements stay registered.
        CanonicalizeDesktopReport(output, result, options.meters_per_pixel);
        std::cout << std::fixed << std::setprecision(2)
                  << "[INFO] selected branch=" << result.branch
                  << " score=" << result.score
                  << " green=" << result.green_line_count
                  << " red=" << result.red_line_count
                  << " raw=" << result.raw_line_count
                  << " length_px=" << result.dimension_long_size_px
                  << " width_px=" << result.dimension_short_size_px
                  << " area_px2=" << result.footprint_area_px2
                  << " vertices=" << result.outline_vertex_count
                  << " output=" << result.output_path
                  << std::defaultfloat << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }
}
