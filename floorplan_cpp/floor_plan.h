#pragma once

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

namespace floorplan {

struct PipelineOptions {
    int thresh = 200;
    // Python default is None. Use <= 0 to mean "not specified".
    int min_branch_length = -1;
    bool restore = false;
    bool auto_branch = true;
    std::vector<int> branch_candidates;
    // Resolution of the algorithm input. Used to choose physically meaningful
    // outer-wall gap closing sizes. Falls back to 0.05 m/px when invalid.
    double meters_per_pixel = 0.05;
    // Optional Android inputs sharing the exact same export geometry.
    std::string visual_input_path;
    std::string semantic_input_path;
    std::vector<cv::Point2f> trajectory_points_px;
};

struct PipelineResult {
    std::string output_path;
    std::string clean_path;
    // White background structural map in the original export coordinates.
    // Contains only the fitted closed exterior and reliable internal walls.
    std::string structural_path;
    std::string debug_dir;
    int branch = 8;
    double score = 0.0;
    int raw_line_count = 0;
    int green_line_count = 0;
    int red_line_count = 0;
    // Final verified room-partition output. These are distinct from the early
    // red gap-fill segments above and drive automatic branch selection.
    int internal_wall_line_count = 0;
    int internal_wall_pixel_count = 0;
    double green_total_length = 0.0;
    double red_total_length = 0.0;
    int wall_pixel_count = 0;
    // Fraction of fused building-wall evidence enclosed by (or immediately
    // touching) the final green polygon. Detached lidar rays are excluded.
    double wall_containment_ratio = 0.0;
    // Fraction of broad, trajectory-reachable explored free space contained
    // by the final footprint.  This is deliberately independent of wall
    // continuity: a doorway or weak facade must not make a scanned room
    // disappear from branch selection.
    double free_space_containment_ratio = 0.0;
    bool outline_closed = false;
    int outline_vertex_count = 0;
    int outline_close_size = 0;
    double outline_support_ratio = 0.0;
    // Native geometry stays in the original SLAM orientation. Presentation
    // layers may rotate the complete report to the nearest Manhattan axis;
    // these values always continue to describe the native pixel frame.
    double outline_rotation_degrees = 0.0;
    double outline_width_px = 0.0;
    double outline_height_px = 0.0;
    double outline_left_px = 0.0;
    double outline_top_px = 0.0;
    double outline_right_px = 0.0;
    double outline_bottom_px = 0.0;
    double dimension_center_x_px = 0.0;
    double dimension_center_y_px = 0.0;
    double dimension_long_axis_x = 1.0;
    double dimension_long_axis_y = 0.0;
    double dimension_short_axis_x = 0.0;
    double dimension_short_axis_y = 1.0;
    double dimension_long_size_px = 0.0;
    double dimension_short_size_px = 0.0;
    double footprint_area_px2 = 0.0;
    double footprint_perimeter_px = 0.0;
    // Closed outline vertices in the original algorithm-input pixel space.
    // Android converts these exact points back to SLAM world coordinates so
    // the map overlay and the exported floor plan share one geometry source.
    std::vector<cv::Point2f> outline_polygon_px;
};

PipelineResult RunPipeline(const std::string& input_path,
                           const std::string& output_path,
                           const std::string& work_dir,
                           const std::string& debug_dir,
                           const PipelineOptions& options);

}  // namespace floorplan
