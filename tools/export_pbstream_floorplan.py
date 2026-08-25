#!/usr/bin/env python3
"""Offline pbstream -> fused rasters/trajectory for the floor-plan pipeline.

Uses only Python's standard library and Pillow.  The protobuf wire fields used
by Cartographer are decoded directly so this tool does not depend on protoc or
a host Cartographer installation.
"""

import argparse
import gzip
import math
import struct
from pathlib import Path

from PIL import Image


MAGIC = 0x7B1D1F7B5BF501DB


def varint(data, offset=0):
    value = 0
    shift = 0
    while offset < len(data):
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if byte < 0x80:
            return value, offset
        shift += 7
        if shift > 70:
            raise ValueError("invalid protobuf varint")
    raise ValueError("truncated protobuf varint")


def fields(data):
    result = []
    offset = 0
    while offset < len(data):
        key, offset = varint(data, offset)
        number, wire = key >> 3, key & 7
        if wire == 0:
            value, offset = varint(data, offset)
        elif wire == 1:
            if offset + 8 > len(data):
                raise ValueError("truncated fixed64")
            value = data[offset:offset + 8]
            offset += 8
        elif wire == 2:
            size, offset = varint(data, offset)
            if offset + size > len(data):
                raise ValueError("truncated length-delimited field")
            value = data[offset:offset + size]
            offset += size
        elif wire == 5:
            if offset + 4 > len(data):
                raise ValueError("truncated fixed32")
            value = data[offset:offset + 4]
            offset += 4
        else:
            raise ValueError(f"unsupported protobuf wire type {wire}")
        result.append((number, wire, value))
    return result


def values(message, number, wire=None):
    return [value for field, actual_wire, value in fields(message)
            if field == number and (wire is None or actual_wire == wire)]


def first(message, number, wire=None, default=None):
    found = values(message, number, wire)
    return found[0] if found else default


def fixed_double(value, default=0.0):
    return struct.unpack("<d", value)[0] if value is not None else default


def fixed_float(value, default=0.0):
    return struct.unpack("<f", value)[0] if value is not None else default


def parse_vector3(message):
    return (
        fixed_double(first(message, 1, 1)),
        fixed_double(first(message, 2, 1)),
        fixed_double(first(message, 3, 1)),
    )


def parse_pose(message):
    translation = parse_vector3(first(message, 1, 2, b""))
    quaternion = first(message, 2, 2, b"")
    qx = fixed_double(first(quaternion, 1, 1))
    qy = fixed_double(first(quaternion, 2, 1))
    qz = fixed_double(first(quaternion, 3, 1))
    qw = fixed_double(first(quaternion, 4, 1), 1.0)
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return translation[0], translation[1], yaw


def inverse_transform(point, pose):
    dx = point[0] - pose[0]
    dy = point[1] - pose[1]
    cosine = math.cos(pose[2])
    sine = math.sin(pose[2])
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def transform(point, pose):
    cosine = math.cos(pose[2])
    sine = math.sin(pose[2])
    return (
        pose[0] + cosine * point[0] - sine * point[1],
        pose[1] + sine * point[0] + cosine * point[1],
    )


def local_to_global(point, local_pose, global_pose):
    return transform(inverse_transform(point, local_pose), global_pose)


def read_records(path):
    with path.open("rb") as stream:
        magic_data = stream.read(8)
        if len(magic_data) != 8 or struct.unpack("<Q", magic_data)[0] != MAGIC:
            raise ValueError("not a Cartographer proto stream")
        while True:
            size_data = stream.read(8)
            if not size_data:
                break
            if len(size_data) != 8:
                raise ValueError("truncated proto stream record size")
            size = struct.unpack("<Q", size_data)[0]
            compressed = stream.read(size)
            if len(compressed) != size:
                raise ValueError("truncated proto stream record")
            yield gzip.decompress(compressed)


def parse_pose_graph(message):
    submap_poses = {}
    trajectory = []
    for trajectory_message in values(message, 4, 2):
        trajectory_id = first(trajectory_message, 3, 0, 0)
        for node in values(trajectory_message, 1, 2):
            pose_message = first(node, 5, 2)
            if pose_message is not None:
                trajectory.append(parse_pose(pose_message)[:2])
        for submap in values(trajectory_message, 2, 2):
            index = first(submap, 2, 0, 0)
            pose_message = first(submap, 1, 2)
            if pose_message is not None:
                submap_poses[(trajectory_id, index)] = parse_pose(pose_message)
    return submap_poses, trajectory


def unpack_packed_varints(chunks):
    output = []
    for chunk in chunks:
        offset = 0
        while offset < len(chunk):
            value, offset = varint(chunk, offset)
            output.append(value)
    return output


def parse_submap(message, global_pose):
    submap_id = first(message, 1, 2, b"")
    trajectory_id = first(submap_id, 1, 0, 0)
    submap_index = first(submap_id, 2, 0, 0)
    submap_2d = first(message, 2, 2)
    if submap_2d is None:
        return None
    local_pose = parse_pose(first(submap_2d, 1, 2, b""))
    grid = first(submap_2d, 4, 2)
    if grid is None:
        return None
    limits = first(grid, 1, 2, b"")
    resolution = fixed_double(first(limits, 1, 1))
    maximum = first(limits, 2, 2, b"")
    max_x = fixed_double(first(maximum, 1, 1))
    max_y = fixed_double(first(maximum, 2, 1))
    cell_limits = first(limits, 3, 2, b"")
    rows = first(cell_limits, 1, 0, 0)
    cols = first(cell_limits, 2, 0, 0)
    cells = unpack_packed_varints(values(grid, 2, 2))
    # Older writers may encode repeated primitive fields unpacked.
    cells.extend(values(grid, 2, 0))
    if resolution <= 0 or rows <= 0 or cols <= 0 or len(cells) < rows * cols:
        return None
    box = first(grid, 3, 2)
    if box is None:
        return None
    max_row = first(box, 1, 0, rows - 1)
    max_col = first(box, 2, 0, cols - 1)
    min_row = first(box, 3, 0, 0)
    min_col = first(box, 4, 0, 0)
    minimum_cost = fixed_float(first(grid, 6, 5), 0.1)
    maximum_cost = fixed_float(first(grid, 7, 5), 0.9)
    return {
        "id": (trajectory_id, submap_index),
        "global_pose": global_pose,
        "local_pose": local_pose,
        "resolution": resolution,
        "max_x": max_x,
        "max_y": max_y,
        "rows": rows,
        "cols": cols,
        "cells": cells,
        "box": (min_row, min_col, max_row, max_col),
        "minimum_cost": minimum_cost,
        "maximum_cost": maximum_cost,
    }


def cell_probability(submap, value):
    value &= 0x7FFF
    if value == 0:
        return None
    low = submap["minimum_cost"]
    high = submap["maximum_cost"]
    scale = (high - low) / 32766.0
    cost = value * scale + (low - scale)
    return 1.0 - cost


def occupied_alpha(probability):
    if probability < 0.68:
        return 0.0
    if probability < 0.76:
        return (90.0 + (probability - 0.68) / 0.08 * 100.0) / 255.0
    return (200.0 + (probability - 0.76) / 0.14 * 55.0) / 255.0


def free_alpha(probability):
    if probability >= 0.5:
        return 0.0
    return 12.0 + (0.5 - probability) / 0.4 * 30.0


def grid_cell_world(submap, row, col):
    resolution = submap["resolution"]
    local = (
        submap["max_x"] - resolution * (col + 0.5),
        submap["max_y"] - resolution * (row + 0.5),
    )
    return local_to_global(local, submap["local_pose"], submap["global_pose"])


def global_to_grid_local(point, submap):
    # Inverse of local_to_global(): global_pose * inverse(local_pose).
    return transform(
        inverse_transform(point, submap["global_pose"]),
        submap["local_pose"],
    )


def texture_channels(submap):
    """Builds the exact intensity/alpha pair used by DrawTexture()."""
    minimum_log_odds = math.log(0.1 / 0.9)
    maximum_log_odds = math.log(0.9 / 0.1)
    output = [None] * len(submap["cells"])
    for index, value in enumerate(submap["cells"]):
        probability = cell_probability(submap, value)
        if probability is None:
            continue
        probability = min(0.9, max(0.1, probability))
        log_odds = math.log(probability / (1.0 - probability))
        normalized = (log_odds - minimum_log_odds) / (
            maximum_log_odds - minimum_log_odds
        )
        log_odds_integer = max(1, min(255, round(1.0 + normalized * 254.0)))
        delta = 128 - log_odds_integer
        output[index] = (max(0, delta), max(0, -delta))
    return output


def compose_cartographer_occupancy(
        submaps, width, height, resolution, minimum_x, maximum_y):
    """Reproduces PaintSubmapSlices' two-channel SOURCE_OVER painting."""
    color = [128.0] * (width * height)
    observed = [0.0] * (width * height)
    for submap in sorted(submaps, key=lambda item: item["id"]):
        channels = texture_channels(submap)
        min_row, min_col, max_row, max_col = submap["box"]
        corners = []
        for row, col in (
                (min_row, min_col), (min_row, max_col + 1),
                (max_row + 1, min_col), (max_row + 1, max_col + 1)):
            local = (
                submap["max_x"] - submap["resolution"] * col,
                submap["max_y"] - submap["resolution"] * row,
            )
            world_x, world_y = local_to_global(
                local, submap["local_pose"], submap["global_pose"])
            corners.append((
                (world_x - minimum_x) / resolution,
                (maximum_y - world_y) / resolution,
            ))
        target_min_x = max(0, math.floor(min(point[0] for point in corners)) - 2)
        target_max_x = min(width - 1, math.ceil(max(point[0] for point in corners)) + 2)
        target_min_y = max(0, math.floor(min(point[1] for point in corners)) - 2)
        target_max_y = min(height - 1, math.ceil(max(point[1] for point in corners)) + 2)

        for target_y in range(target_min_y, target_max_y + 1):
            world_y = maximum_y - (target_y + 0.5) * resolution
            for target_x in range(target_min_x, target_max_x + 1):
                world_x = minimum_x + (target_x + 0.5) * resolution
                local_x, local_y = global_to_grid_local(
                    (world_x, world_y), submap)
                source_col = (
                    (submap["max_x"] - local_x) / submap["resolution"] - 0.5
                )
                source_row = (
                    (submap["max_y"] - local_y) / submap["resolution"] - 0.5
                )
                base_col = math.floor(source_col)
                base_row = math.floor(source_row)
                fraction_col = source_col - base_col
                fraction_row = source_row - base_row
                sampled_intensity = 0.0
                sampled_alpha = 0.0
                sampled_observed = 0.0
                for row_offset in (0, 1):
                    row = base_row + row_offset
                    if row < 0 or row >= submap["rows"]:
                        continue
                    row_weight = 1.0 - fraction_row if row_offset == 0 else fraction_row
                    for col_offset in (0, 1):
                        col = base_col + col_offset
                        if col < 0 or col >= submap["cols"]:
                            continue
                        col_weight = 1.0 - fraction_col if col_offset == 0 else fraction_col
                        weight = row_weight * col_weight
                        channel = channels[col * submap["rows"] + row]
                        if channel is None or weight <= 0.0:
                            continue
                        sampled_intensity += channel[0] * weight
                        sampled_alpha += channel[1] * weight
                        sampled_observed += 255.0 * weight
                if sampled_observed < 0.5:
                    continue
                index = target_y * width + target_x
                inverse_alpha = 1.0 - max(0.0, min(1.0, sampled_alpha / 255.0))
                color[index] = max(0.0, min(
                    255.0, sampled_intensity + color[index] * inverse_alpha))
                observed[index] = max(0.0, min(
                    255.0, sampled_observed + observed[index] * inverse_alpha))
    return [
        max(0, min(255, round(color[index]))) if observed[index] >= 0.5 else 154
        for index in range(width * height)
    ]


def smooth_walls(source, width, height):
    current = source[:]
    for _ in range(2):
        output = current[:]
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                index = y * width + x
                neighbors = sum(
                    current[(y + dy) * width + x + dx]
                    for dy in (-1, 0, 1) for dx in (-1, 0, 1)
                    if dx or dy
                )
                if current[index]:
                    output[index] = neighbors >= 1
                else:
                    output[index] = (
                        (current[index - 1] and current[index + 1]) or
                        (current[index - width] and current[index + width]) or
                        (current[index - width - 1] and current[index + width + 1]) or
                        (current[index - width + 1] and current[index + width - 1])
                    )
        current = output
    return current


def thin_walls(source, width, height):
    result = bytearray(source)
    for _ in range(16):
        changed = False
        for step in (0, 1):
            remove = []
            for y in range(1, height - 1):
                for x in range(1, width - 1):
                    index = y * width + x
                    if not result[index]:
                        continue
                    p = [result[index - width], result[index - width + 1],
                         result[index + 1], result[index + width + 1],
                         result[index + width], result[index + width - 1],
                         result[index - 1], result[index - width - 1]]
                    count = sum(bool(value) for value in p)
                    transitions = sum(not p[i] and p[(i + 1) % 8] for i in range(8))
                    if count < 2 or count > 6 or transitions != 1:
                        continue
                    if step == 0:
                        blocked = (p[0] and p[2] and p[4]) or (p[2] and p[4] and p[6])
                    else:
                        blocked = (p[0] and p[2] and p[6]) or (p[0] and p[4] and p[6])
                    if not blocked:
                        remove.append(index)
            for index in remove:
                result[index] = 0
            changed |= bool(remove)
        if not changed:
            break
    return result


def smooth_free(source, walls, width, height):
    current = bytearray(source)
    for _ in range(2):
        output = bytearray(current)
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                index = y * width + x
                if walls[index]:
                    output[index] = 0
                    continue
                neighbors = sum(
                    bool(current[(y + dy) * width + x + dx])
                    for dy in (-1, 0, 1) for dx in (-1, 0, 1)
                    if dx or dy
                )
                output[index] = neighbors >= (2 if current[index] else 5)
        current = output
    for index, wall in enumerate(walls):
        if wall:
            current[index] = 0
    return current


def render(pbstream, output_dir):
    records = list(read_records(pbstream))
    if len(records) < 2:
        raise ValueError("pbstream contains no serialized mapping data")
    submap_poses = {}
    trajectory = []
    serialized_records = []
    for record in records[1:]:
        wrapped_pose_graph = first(record, 1, 2)
        if wrapped_pose_graph is not None and not submap_poses:
            submap_poses, trajectory = parse_pose_graph(wrapped_pose_graph)
        serialized_records.append(record)
    if not submap_poses:
        raise ValueError("pbstream pose graph is missing")
    submaps = []
    for record in serialized_records:
        wrapped_submap = first(record, 3, 2)
        if wrapped_submap is None:
            continue
        submap_id = first(wrapped_submap, 1, 2, b"")
        key = (first(submap_id, 1, 0, 0), first(submap_id, 2, 0, 0))
        pose = submap_poses.get(key)
        if pose is None:
            continue
        parsed = parse_submap(wrapped_submap, pose)
        if parsed is not None:
            submaps.append(parsed)
    if not submaps:
        raise ValueError("pbstream contains no usable 2D submaps")

    resolution = min(submap["resolution"] for submap in submaps)
    bounds = [float("inf"), float("inf"), -float("inf"), -float("inf")]
    for submap in submaps:
        min_row, min_col, max_row, max_col = submap["box"]
        for row, col in ((min_row, min_col), (min_row, max_col + 1),
                         (max_row + 1, min_col), (max_row + 1, max_col + 1)):
            local = (submap["max_x"] - submap["resolution"] * col,
                     submap["max_y"] - submap["resolution"] * row)
            x, y = local_to_global(local, submap["local_pose"], submap["global_pose"])
            bounds[0] = min(bounds[0], x)
            bounds[1] = min(bounds[1], y)
            bounds[2] = max(bounds[2], x)
            bounds[3] = max(bounds[3], y)
    padding = 6
    width = math.ceil((bounds[2] - bounds[0]) / resolution) + padding * 2
    height = math.ceil((bounds[3] - bounds[1]) / resolution) + padding * 2
    maximum_side = max(width, height)
    if maximum_side > 1600:
        resolution *= maximum_side / 1600.0
        width = math.ceil((bounds[2] - bounds[0]) / resolution) + padding * 2
        height = math.ceil((bounds[3] - bounds[1]) / resolution) + padding * 2
    minimum_x = bounds[0] - padding * resolution
    maximum_y = bounds[3] + padding * resolution
    count = width * height
    occupied = [0.0] * count
    strongest = [0.0] * count
    free = [0.0] * count
    latest = {}
    for submap in submaps:
        latest[submap["id"][0]] = max(latest.get(submap["id"][0], -1), submap["id"][1])

    for submap in submaps:
        # This exporter represents the finalized Android path. Once final
        # optimization has completed FusedMapRenderer gives every submap the
        # same authority; live-view age weights must not leak into the saved
        # floor-plan inputs.
        weight = 1.0
        min_row, min_col, max_row, max_col = submap["box"]
        for row in range(max(0, min_row), min(submap["rows"] - 1, max_row) + 1):
            row_offset = row * submap["rows"]
            # Grid2D flattening uses num_x_cells * y + x. Here row=x, col=y.
            for col in range(max(0, min_col), min(submap["cols"] - 1, max_col) + 1):
                probability = cell_probability(submap, submap["cells"][col * submap["rows"] + row])
                if probability is None:
                    continue
                wall_alpha = occupied_alpha(probability)
                miss_alpha = free_alpha(probability)
                if wall_alpha <= 0 and miss_alpha <= 0:
                    continue
                world_x, world_y = grid_cell_world(submap, row, col)
                target_x = (world_x - minimum_x) / resolution
                target_y = (maximum_y - world_y) / resolution
                base_x = math.floor(target_x)
                base_y = math.floor(target_y)
                fraction_x = target_x - base_x
                fraction_y = target_y - base_y
                for offset_y in (0, 1):
                    y = base_y + offset_y
                    if y < 0 or y >= height:
                        continue
                    wy = 1.0 - fraction_y if offset_y == 0 else fraction_y
                    for offset_x in (0, 1):
                        x = base_x + offset_x
                        if x < 0 or x >= width:
                            continue
                        wx = 1.0 - fraction_x if offset_x == 0 else fraction_x
                        interpolation = wx * wy
                        if interpolation <= 0.001:
                            continue
                        index = y * width + x
                        if wall_alpha > 0:
                            contribution = weight * wall_alpha * interpolation
                            occupied[index] += contribution
                            strongest[index] = max(strongest[index], contribution)
                        elif miss_alpha > 0:
                            free[index] += weight * 0.30 * (miss_alpha / 54.0) * interpolation

    stable = bytearray(count)
    for y in range(height):
        for x in range(width):
            index = y * width + x
            if occupied[index] <= 0:
                continue
            neighborhood = occupied[index]
            for dy in (-1, 0, 1):
                sy = y + dy
                if sy < 0 or sy >= height:
                    continue
                for dx in (-1, 0, 1):
                    sx = x + dx
                    if (not dx and not dy) or sx < 0 or sx >= width:
                        continue
                    neighborhood += occupied[sy * width + sx] * (0.16 if not dx or not dy else 0.08)
            other = occupied[index] - strongest[index]
            if (other >= 0.16 and neighborhood >= 0.86) or (
                    occupied[index] >= 0.52 and neighborhood >= 0.94):
                stable[index] = 1
    walls = thin_walls(smooth_walls(stable, width, height), width, height)
    free_mask = smooth_free(
        bytearray(free[index] > 0.08 and not walls[index] for index in range(count)),
        walls, width, height)

    output_dir.mkdir(parents=True, exist_ok=True)
    algorithm = Image.new("RGB", (width, height), "white")
    semantic = Image.new("RGB", (width, height), (154, 154, 154))
    probability_image = Image.new("RGB", (width, height), (154, 154, 154))
    algorithm_pixels = algorithm.load()
    semantic_pixels = semantic.load()
    probability_pixels = probability_image.load()
    painted_occupancy = compose_cartographer_occupancy(
        submaps, width, height, resolution, minimum_x, maximum_y)
    for y in range(height):
        for x in range(width):
            index = y * width + x
            if walls[index]:
                algorithm_pixels[x, y] = (0, 0, 0)
                semantic_pixels[x, y] = (0, 0, 0)
            elif free_mask[index]:
                semantic_pixels[x, y] = (255, 255, 255)
            value = painted_occupancy[index]
            probability_pixels[x, y] = (value, value, value)
    algorithm.save(output_dir / "floorplan_input.png")
    probability_image.save(output_dir / "floorplan_probability.png")
    # Final black point-cloud presentation: continuous Cartographer occupancy
    # probability, not the semantic mask and not a fitted/vectorized polygon.
    probability_image.save(output_dir / "floorplan_visual.png")
    semantic.save(output_dir / "floorplan_semantic.png")
    semantic.save(output_dir / "floorplan_preview.png")
    with (output_dir / "trajectory.txt").open("w", encoding="utf-8") as stream:
        for x, y in trajectory:
            px = (x - minimum_x) / resolution
            py = (maximum_y - y) / resolution
            if 0 <= px < width and 0 <= py < height:
                stream.write(f"{px:.6f} {py:.6f}\n")
    (output_dir / "geometry.txt").write_text(
        f"resolution_m_per_pixel={resolution:.9f}\n"
        f"world_min_x={minimum_x:.9f}\nworld_max_y={maximum_y:.9f}\n"
        f"width_px={width}\nheight_px={height}\n"
        f"submaps={len(submaps)}\ntrajectory_nodes={len(trajectory)}\n",
        encoding="utf-8",
    )
    return resolution, len(submaps), len(trajectory), width, height


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pbstream", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    result = render(args.pbstream, args.output_dir)
    print("resolution=%.9f submaps=%d trajectory_nodes=%d size=%dx%d" % result)


if __name__ == "__main__":
    main()
