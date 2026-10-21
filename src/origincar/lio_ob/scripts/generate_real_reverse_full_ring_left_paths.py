#!/usr/bin/env python3

"""Generate left-side obstacle-bypass variants of the real full-ring routes."""

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

import generate_real_reverse_full_ring_straight_paths as common


APPROACH_POINTS = 40
RADIUS = 0.45
LEFT_X = 1.80
CHANNEL_X = 2.65
CHANNEL_TOP = (2.65, 2.72, math.radians(-90.0))


def clockwise_arc(start, start_heading, end_heading, count):
    center = (
        start[0] + RADIUS * math.sin(start_heading),
        start[1] - RADIUS * math.cos(start_heading),
    )
    change = common.normalize_angle(end_heading - start_heading)
    if change > 0.0:
        change -= 2.0 * math.pi
    result = []
    for index in range(count):
        heading = start_heading + change * index / (count - 1)
        result.append((
            center[0] - RADIUS * math.sin(heading),
            center[1] + RADIUS * math.cos(heading),
        ))
    return result


def counterclockwise_arc(start, start_heading, end_heading, count):
    center = (
        start[0] - RADIUS * math.sin(start_heading),
        start[1] + RADIUS * math.cos(start_heading),
    )
    change = common.normalize_angle(end_heading - start_heading)
    if change < 0.0:
        change += 2.0 * math.pi
    result = []
    for index in range(count):
        heading = start_heading + change * index / (count - 1)
        result.append((
            center[0] + RADIUS * math.sin(heading),
            center[1] - RADIUS * math.cos(heading),
        ))
    return result


def make_left_approach():
    start_heading = common.REAL_START[2] + math.pi
    west_heading = math.pi
    north_heading = 0.5 * math.pi

    # Settle from the measured southwest reverse heading to west, then travel
    # left underneath the possible centre obstacle.
    settle = clockwise_arc(common.REAL_START, start_heading, west_heading, 201)
    west_end = (LEFT_X + RADIUS, settle[-1][1])
    west_line = [settle[-1], west_end]

    # A quarter circle brings the vehicle up on the obstacle's left side.
    rise = clockwise_arc(west_end, west_heading, north_heading, 301)

    # Two equal-radius arcs shift the northbound path right into the channel
    # without changing its final heading or exceeding the curvature limit.
    lateral_shift = CHANNEL_X - LEFT_X
    cosine = 1.0 - lateral_shift / (2.0 * RADIUS)
    if not -1.0 <= cosine <= 1.0:
        raise ValueError("left bypass cannot reach channel at configured radius")
    shift_angle = math.acos(cosine)
    diagonal_heading = north_heading - shift_angle
    shift_right = clockwise_arc(rise[-1], north_heading, diagonal_heading, 301)
    straighten = counterclockwise_arc(
        shift_right[-1], diagonal_heading, north_heading, 301)

    if math.hypot(straighten[-1][0] - CHANNEL_X, 0.0) > 1.0e-6:
        raise ValueError("left bypass did not reach channel centre")
    channel = [straighten[-1], CHANNEL_TOP[:2]]
    dense = (
        settle + common.resample_polyline(west_line, 301)[1:] + rise[1:] +
        shift_right[1:] + straighten[1:] +
        common.resample_polyline(channel, 201)[1:]
    )
    points = common.resample_polyline(dense, APPROACH_POINTS)
    return common.rows_from_polyline(
        points, common.REAL_START[2], CHANNEL_TOP[2], direction=-1)


def make_left_route(base):
    if len(base) != common.PATH_POINTS:
        raise ValueError(f"base route must contain {common.PATH_POINTS} points")
    route = make_left_approach() + [row.copy() for row in base[APPROACH_POINTS:]]
    if route[APPROACH_POINTS:] != base[APPROACH_POINTS:]:
        raise ValueError("points 41..200 differ from the real-ring base")
    return route


def make_panel(mask, rows, title, stats):
    panel = mask.convert("RGB")
    overlay = Image.new("RGBA", panel.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    width, height = panel.size
    switch = common.REVERSE_END_INDEX + 1
    common.draw_polyline(draw, rows[:switch], width, height, common.REVERSE_COLOR, 7)
    common.draw_polyline(
        draw, rows[switch - 1:], width, height, common.FORWARD_COLOR, 7)
    common.draw_pose_arrow(draw, rows[0], width, height)

    font = ImageFont.load_default()
    draw.rectangle((12, 12, 560, 126), fill=(0, 0, 0, 190))
    draw.text((24, 22), title, fill=(255, 255, 255, 255), font=font)
    draw.text((24, 44), "RED ARROW: vehicle nose at real start (3.60, 1.10)",
              fill=(255, 255, 255, 255), font=font)
    draw.text((24, 64), "BLUE: reverse around the obstacle's left side",
              fill=(255, 255, 255, 255), font=font)
    draw.text((24, 84), "GREEN: unchanged forward exit after direction switch",
              fill=(255, 255, 255, 255), font=font)
    draw.text((24, 104),
              f"bypass radius: {RADIUS:.2f} m; max curvature: {stats[0]:.2f} 1/m",
              fill=(255, 255, 255, 255), font=font)
    return Image.alpha_composite(panel.convert("RGBA"), overlay).convert("RGB")


def generate(clockwise_base_path, counterclockwise_base_path, mask_path, output_directory):
    clockwise_base = common.parse_path(clockwise_base_path)
    counterclockwise_base = common.parse_path(counterclockwise_base_path)
    clockwise = make_left_route(clockwise_base)
    counterclockwise = make_left_route(counterclockwise_base)

    clockwise_stats = common.verify_route(clockwise, clockwise_base, "clockwise left")
    counterclockwise_stats = common.verify_route(
        counterclockwise, counterclockwise_base, "counterclockwise left")

    output_directory.mkdir(parents=True, exist_ok=True)
    clockwise_output = (
        output_directory / "path_post_qr_reverse_full_real_ring_clockwise_left.txt")
    counterclockwise_output = (
        output_directory /
        "path_post_qr_reverse_full_real_ring_counterclockwise_left.txt")
    preview_output = output_directory / "path_qr_reverse_full_real_ring_left_preview.png"
    common.write_path(clockwise_output, clockwise)
    common.write_path(counterclockwise_output, counterclockwise)

    mask = Image.open(mask_path).convert("L")
    clockwise_white = sum(common.footprint_is_white(mask, row) for row in clockwise)
    counterclockwise_white = sum(
        common.footprint_is_white(mask, row) for row in counterclockwise)
    left = make_panel(
        mask, clockwise, "LEFT BYPASS FULL REVERSE / SIGN 3 CLOCKWISE",
        clockwise_stats)
    right = make_panel(
        mask, counterclockwise,
        "LEFT BYPASS FULL REVERSE / SIGN 4 COUNTERCLOCKWISE",
        counterclockwise_stats)
    preview = Image.new("RGB", (left.width + right.width, left.height), (20, 20, 20))
    preview.paste(left, (0, 0))
    preview.paste(right, (left.width, 0))
    preview.save(preview_output)

    print(f"clockwise: max curvature={clockwise_stats[0]:.3f} 1/m, "
          f"white footprints={clockwise_white}/{len(clockwise)}")
    print(f"counterclockwise: max curvature={counterclockwise_stats[0]:.3f} 1/m, "
          f"white footprints={counterclockwise_white}/{len(counterclockwise)}")
    print(clockwise_output)
    print(counterclockwise_output)
    print(preview_output)


def main():
    script_directory = Path(__file__).resolve().parent
    package_directory = script_directory.parent
    paths = package_directory / "paths"
    parser = argparse.ArgumentParser(
        description="Generate left-obstacle-bypass real full-ring routes")
    parser.add_argument(
        "--clockwise-base", type=Path,
        default=paths / "path_post_qr_reverse_full_real_ring_clockwise.txt")
    parser.add_argument(
        "--counterclockwise-base", type=Path,
        default=paths / "path_post_qr_reverse_full_real_ring_counterclockwise.txt")
    parser.add_argument(
        "--mask", type=Path,
        default=package_directory / "config" / "field_keepout_mask.pgm")
    parser.add_argument("--output-directory", type=Path, default=paths)
    args = parser.parse_args()
    generate(
        args.clockwise_base, args.counterclockwise_base,
        args.mask, args.output_directory)


if __name__ == "__main__":
    main()
