#!/usr/bin/env python3

"""Generate no-obstacle straight-entry variants of the real ring routes."""

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


FIELD_SIZE_M = 5.0
PATH_POINTS = 200
APPROACH_POINTS = 40
REVERSE_END_INDEX = 162
MAX_CURVATURE = 4.0
ROBOT_HALF_LENGTH = 0.1375
ROBOT_HALF_WIDTH = 0.0925

REAL_START = (3.60, 1.10, math.radians(30.2))
CHANNEL_TOP = (2.65, 2.72, math.radians(-90.0))

REVERSE_COLOR = (55, 115, 255, 255)
FORWARD_COLOR = (25, 205, 105, 255)
START_COLOR = (245, 75, 55, 255)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_path(path):
    rows = []
    for line in path.read_text(encoding="ascii").splitlines():
        fields = line.rstrip(",").split(",")
        if len(fields) < 8:
            raise ValueError(f"malformed path row: {line}")
        rows.append([float(value) for value in fields[:7]] + [int(fields[7])])
    return rows


def write_path(path, rows):
    with path.open("w", encoding="ascii", newline="\n") as stream:
        for x, y, z, qx, qy, qz, qw, direction in rows:
            stream.write(
                f"{x:.4f},{y:.4f},{z:.4f},{qx:.4f},{qy:.4f},"
                f"{qz:.4f},{qw:.4f},{direction},\n"
            )


def path_row(x, y, vehicle_yaw, direction):
    return [
        x, y, 0.0, 0.0, 0.0,
        math.sin(0.5 * vehicle_yaw), math.cos(0.5 * vehicle_yaw), direction,
    ]


def polyline_lengths(points):
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(
            cumulative[-1] + math.hypot(second[0] - first[0], second[1] - first[1])
        )
    return cumulative


def resample_polyline(points, count):
    cumulative = polyline_lengths(points)
    total = cumulative[-1]
    result = []
    segment = 0
    for index in range(count):
        target = total * index / (count - 1)
        while segment + 1 < len(cumulative) - 1 and cumulative[segment + 1] < target:
            segment += 1
        span = cumulative[segment + 1] - cumulative[segment]
        fraction = 0.0 if span <= 0.0 else (target - cumulative[segment]) / span
        first = points[segment]
        second = points[segment + 1]
        result.append((
            first[0] + fraction * (second[0] - first[0]),
            first[1] + fraction * (second[1] - first[1]),
        ))
    return result


def rows_from_polyline(points, start_yaw, end_yaw, direction=-1):
    rows = []
    for index, point in enumerate(points):
        if index == 0:
            travel_yaw = start_yaw + math.pi
        elif index == len(points) - 1:
            travel_yaw = end_yaw + math.pi
        else:
            travel_yaw = math.atan2(
                points[index + 1][1] - points[index - 1][1],
                points[index + 1][0] - points[index - 1][0],
            )
        vehicle_yaw = travel_yaw + math.pi if direction < 0 else travel_yaw
        rows.append(path_row(point[0], point[1], normalize_angle(vehicle_yaw), direction))
    return rows


def make_straight_approach():
    """One broad reverse arc followed by the channel centreline."""
    start_heading = REAL_START[2] + math.pi
    north_heading = 0.5 * math.pi
    denominator = math.sin(start_heading) - math.sin(north_heading)
    radius = (CHANNEL_TOP[0] - REAL_START[0]) / denominator
    if radius <= 1.0 / MAX_CURVATURE:
        raise ValueError("straight-entry radius violates maximum curvature")

    center = (
        REAL_START[0] + radius * math.sin(start_heading),
        REAL_START[1] - radius * math.cos(start_heading),
    )
    heading_change = normalize_angle(north_heading - start_heading)
    arc = []
    for index in range(601):
        heading = start_heading + heading_change * index / 600.0
        arc.append((
            center[0] - radius * math.sin(heading),
            center[1] + radius * math.cos(heading),
        ))
    channel = [arc[-1], CHANNEL_TOP[:2]]
    dense = arc + resample_polyline(channel, 401)[1:]
    points = resample_polyline(dense, APPROACH_POINTS)
    return rows_from_polyline(points, REAL_START[2], CHANNEL_TOP[2]), radius


def make_straight_route(base):
    if len(base) != PATH_POINTS:
        raise ValueError(f"base route must contain {PATH_POINTS} points")
    approach, radius = make_straight_approach()
    route = approach + [row.copy() for row in base[APPROACH_POINTS:]]
    return route, radius


def yaw(row):
    return 2.0 * math.atan2(row[5], row[6])


def curvature_statistics(rows):
    curvatures = []
    for previous, current, following in zip(rows, rows[1:], rows[2:]):
        first_distance = math.hypot(current[0] - previous[0], current[1] - previous[1])
        second_distance = math.hypot(following[0] - current[0], following[1] - current[1])
        distance = 0.5 * (first_distance + second_distance)
        if distance <= 1.0e-6:
            continue
        heading_change = normalize_angle(yaw(following) - yaw(previous))
        curvatures.append(abs(heading_change) / (2.0 * distance))
    return max(curvatures), sum(curvatures) / len(curvatures)


def verify_route(rows, base, name):
    if len(rows) != PATH_POINTS:
        raise ValueError(f"{name}: expected {PATH_POINTS} points")
    if rows[0][7] != -1 or rows[REVERSE_END_INDEX][7] != -1:
        raise ValueError(f"{name}: reverse section is malformed")
    if rows[REVERSE_END_INDEX + 1][7] != 1:
        raise ValueError(f"{name}: forward section is malformed")
    if rows[APPROACH_POINTS:] != base[APPROACH_POINTS:]:
        raise ValueError(f"{name}: points 41..200 differ from the real-ring base")
    maximum, average = curvature_statistics(rows)
    if maximum > MAX_CURVATURE + 0.05:
        raise ValueError(f"{name}: curvature {maximum:.3f} exceeds limit")
    return maximum, average


def pixel(point, width, height):
    return (
        int(round(point[0] * width / FIELD_SIZE_M)),
        int(round((FIELD_SIZE_M - point[1]) * height / FIELD_SIZE_M)),
    )


def footprint_is_white(mask, row):
    vehicle_yaw = yaw(row)
    cos_yaw = math.cos(vehicle_yaw)
    sin_yaw = math.sin(vehicle_yaw)
    for longitudinal in (-ROBOT_HALF_LENGTH, 0.0, ROBOT_HALF_LENGTH):
        for lateral in (-ROBOT_HALF_WIDTH, 0.0, ROBOT_HALF_WIDTH):
            x = row[0] + longitudinal * cos_yaw - lateral * sin_yaw
            y = row[1] + longitudinal * sin_yaw + lateral * cos_yaw
            column, image_row = pixel((x, y), mask.width, mask.height)
            if not (0 <= column < mask.width and 0 <= image_row < mask.height):
                return False
            if mask.getpixel((column, image_row)) < 128:
                return False
    return True


def draw_polyline(draw, rows, width, height, color, line_width):
    draw.line(
        [pixel(row, width, height) for row in rows],
        fill=color, width=line_width, joint="curve",
    )


def draw_pose_arrow(draw, row, width, height):
    x, y = pixel(row, width, height)
    heading = yaw(row)
    length = 0.32 * width / FIELD_SIZE_M
    tip = (x + length * math.cos(heading), y - length * math.sin(heading))
    draw.line((x, y, tip[0], tip[1]), fill=START_COLOR, width=7)
    for offset in (-0.55, 0.55):
        wing = heading + math.pi + offset
        wing_tip = (
            tip[0] + 0.11 * width / FIELD_SIZE_M * math.cos(wing),
            tip[1] - 0.11 * height / FIELD_SIZE_M * math.sin(wing),
        )
        draw.line((tip[0], tip[1], wing_tip[0], wing_tip[1]), fill=START_COLOR, width=7)


def make_panel(mask, rows, title, stats, radius):
    panel = mask.convert("RGB")
    overlay = Image.new("RGBA", panel.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    width, height = panel.size
    switch = REVERSE_END_INDEX + 1
    draw_polyline(draw, rows[:switch], width, height, REVERSE_COLOR, 7)
    draw_polyline(draw, rows[switch - 1:], width, height, FORWARD_COLOR, 7)
    draw_pose_arrow(draw, rows[0], width, height)

    font = ImageFont.load_default()
    draw.rectangle((12, 12, 540, 126), fill=(0, 0, 0, 190))
    draw.text((24, 22), title, fill=(255, 255, 255, 255), font=font)
    draw.text((24, 44), "RED ARROW: vehicle nose at real start (3.60, 1.10)",
              fill=(255, 255, 255, 255), font=font)
    draw.text((24, 64), "BLUE: no-obstacle reverse entry and complete ring lap",
              fill=(255, 255, 255, 255), font=font)
    draw.text((24, 84), "GREEN: unchanged forward exit after direction switch",
              fill=(255, 255, 255, 255), font=font)
    draw.text((24, 104),
              f"entry radius: {radius:.3f} m; max curvature: {stats[0]:.2f} 1/m",
              fill=(255, 255, 255, 255), font=font)
    return Image.alpha_composite(panel.convert("RGBA"), overlay).convert("RGB")


def generate(clockwise_base_path, counterclockwise_base_path, mask_path, output_directory):
    clockwise_base = parse_path(clockwise_base_path)
    counterclockwise_base = parse_path(counterclockwise_base_path)
    clockwise, radius = make_straight_route(clockwise_base)
    counterclockwise, other_radius = make_straight_route(counterclockwise_base)
    if abs(radius - other_radius) > 1.0e-9:
        raise ValueError("straight entry radii differ")

    clockwise_stats = verify_route(clockwise, clockwise_base, "clockwise straight")
    counterclockwise_stats = verify_route(
        counterclockwise, counterclockwise_base, "counterclockwise straight")

    output_directory.mkdir(parents=True, exist_ok=True)
    clockwise_output = (
        output_directory / "path_post_qr_reverse_full_real_ring_clockwise_straight.txt")
    counterclockwise_output = (
        output_directory /
        "path_post_qr_reverse_full_real_ring_counterclockwise_straight.txt")
    preview_output = output_directory / "path_qr_reverse_full_real_ring_straight_preview.png"
    write_path(clockwise_output, clockwise)
    write_path(counterclockwise_output, counterclockwise)

    mask = Image.open(mask_path).convert("L")
    clockwise_white = sum(footprint_is_white(mask, row) for row in clockwise)
    counterclockwise_white = sum(footprint_is_white(mask, row) for row in counterclockwise)
    left = make_panel(
        mask, clockwise, "STRAIGHT FULL REVERSE / SIGN 3 CLOCKWISE",
        clockwise_stats, radius)
    right = make_panel(
        mask, counterclockwise, "STRAIGHT FULL REVERSE / SIGN 4 COUNTERCLOCKWISE",
        counterclockwise_stats, radius)
    preview = Image.new("RGB", (left.width + right.width, left.height), (20, 20, 20))
    preview.paste(left, (0, 0))
    preview.paste(right, (left.width, 0))
    preview.save(preview_output)

    print(f"entry radius={radius:.3f} m, curvature={1.0 / radius:.3f} 1/m")
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
        description="Generate no-obstacle straight-entry real full-ring routes")
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
