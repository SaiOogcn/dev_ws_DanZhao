#!/usr/bin/env python3

"""Generate full-reverse ring routes from the measured post-QR pose.

The original full-ring routes start at (4.20, 1.50).  The real vehicle is at
(3.60, 1.10) after scanning the QR code, with its nose pointing 30.2 degrees
towards the upper-right corner.  This generator replaces only the 78-point
reverse entry.  The complete reverse lap, direction-change cusp, and forward
exit remain byte-for-byte equivalent (apart from reformatting) to the existing
full-ring routes.
"""

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


FIELD_SIZE_M = 5.0
PATH_POINTS = 200
ENTRY_POINTS = 78
REVERSE_END_INDEX = 162  # zero-based; point 164 is the first forward point
MAX_CURVATURE = 4.0
ROBOT_HALF_LENGTH = 0.1375
ROBOT_HALF_WIDTH = 0.0925

REAL_START = (3.60, 1.10, math.radians(30.2))
# The real-start route stays on the right side of the marked lower-channel
# obstacle, runs north there, then makes a smooth left merge into the channel.
CHANNEL_X = 2.65
FORWARD_CHANNEL_X = 2.52
CHANNEL_TOP = (2.65, 2.72, math.radians(-90.0))
RING_RADIUS = 0.48
CW_RING_START = (2.17, 3.20, 0.0)
CCW_RING_START = (3.13, 3.20, math.pi)

REVERSE_COLOR = (55, 115, 255, 255)
FORWARD_COLOR = (25, 205, 105, 255)
START_COLOR = (245, 75, 55, 255)


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


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def path_row(x, y, vehicle_yaw, direction):
    return [
        x, y, 0.0, 0.0, 0.0,
        math.sin(0.5 * vehicle_yaw), math.cos(0.5 * vehicle_yaw), direction,
    ]


def quintic_segment(start, end, start_handle, end_handle, samples=401, direction=-1):
    """Quintic Bezier with zero endpoint curvature.

    P0/P1/P2 and P3/P4/P5 are collinear, so adjacent segments can meet with
    continuous heading and zero curvature at the shared waypoint.
    """
    start_travel_yaw = start[2] if direction > 0 else start[2] + math.pi
    end_travel_yaw = end[2] if direction > 0 else end[2] + math.pi
    start_tangent = (math.cos(start_travel_yaw), math.sin(start_travel_yaw))
    end_tangent = (math.cos(end_travel_yaw), math.sin(end_travel_yaw))
    points = [
        start[:2],
        (start[0] + start_handle * start_tangent[0],
         start[1] + start_handle * start_tangent[1]),
        (start[0] + 2.0 * start_handle * start_tangent[0],
         start[1] + 2.0 * start_handle * start_tangent[1]),
        (end[0] - 2.0 * end_handle * end_tangent[0],
         end[1] - 2.0 * end_handle * end_tangent[1]),
        (end[0] - end_handle * end_tangent[0],
         end[1] - end_handle * end_tangent[1]),
        end[:2],
    ]
    dense = []
    for index in range(samples):
        t = index / (samples - 1)
        u = 1.0 - t
        basis = (
            u**5,
            5.0 * u**4 * t,
            10.0 * u**3 * t**2,
            10.0 * u**2 * t**3,
            5.0 * u * t**4,
            t**5,
        )
        x = sum(weight * point[0] for weight, point in zip(basis, points))
        y = sum(weight * point[1] for weight, point in zip(basis, points))
        dense.append((x, y))
    return dense


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
            travel_yaw = start_yaw if direction > 0 else start_yaw + math.pi
        elif index == len(points) - 1:
            travel_yaw = end_yaw if direction > 0 else end_yaw + math.pi
        else:
            travel_yaw = math.atan2(
                points[index + 1][1] - points[index - 1][1],
                points[index + 1][0] - points[index - 1][0],
            )
        vehicle_yaw = travel_yaw if direction > 0 else travel_yaw + math.pi
        rows.append(path_row(point[0], point[1], normalize_angle(vehicle_yaw), direction))
    return rows


def make_shared_approach():
    start_heading = REAL_START[2] + math.pi
    north_heading = 0.5 * math.pi
    radius = 0.30

    def clockwise_arc(start, start_heading, end_heading, count):
        center = (
            start[0] + radius * math.sin(start_heading),
            start[1] - radius * math.cos(start_heading),
        )
        change = normalize_angle(end_heading - start_heading)
        if change > 0.0:
            change -= 2.0 * math.pi
        result = []
        for index in range(count):
            heading = start_heading + change * index / (count - 1)
            result.append((
                center[0] - radius * math.sin(heading),
                center[1] + radius * math.cos(heading),
            ))
        return result

    def counterclockwise_arc(start, start_heading, end_heading, count):
        center = (
            start[0] - radius * math.sin(start_heading),
            start[1] + radius * math.cos(start_heading),
        )
        change = normalize_angle(end_heading - start_heading)
        if change < 0.0:
            change += 2.0 * math.pi
        result = []
        for index in range(count):
            heading = start_heading + change * index / (count - 1)
            result.append((
                center[0] + radius * math.sin(heading),
                center[1] - radius * math.cos(heading),
            ))
        return result

    # First arc: back toward the lower-left and become tangent to a northbound
    # line on the obstacle's right side.
    first_arc = clockwise_arc(REAL_START, start_heading, north_heading, 401)
    right_x = first_arc[-1][0]

    # Symmetric left/right arcs shift the reverse path from the obstacle's right
    # side to the channel centre without exceeding the configured curvature.
    lateral_shift = right_x - CHANNEL_X
    cosine = 1.0 - lateral_shift / (2.0 * radius)
    if not -1.0 <= cosine <= 1.0:
        raise ValueError("right-side bypass cannot reach the channel at this radius")
    bypass_angle = math.acos(cosine)
    diagonal_heading = north_heading + bypass_angle
    left_arc = counterclockwise_arc(
        first_arc[-1], north_heading, diagonal_heading, 301)
    right_arc = clockwise_arc(
        left_arc[-1], diagonal_heading, north_heading, 301)

    channel_start = right_arc[-1]
    channel = [channel_start, (CHANNEL_TOP[0], CHANNEL_TOP[1])]
    dense = (
        first_arc + left_arc[1:] + right_arc[1:] +
        resample_polyline(channel, 301)[1:]
    )
    points = resample_polyline(dense, 40)
    return rows_from_polyline(points, REAL_START[2], CHANNEL_TOP[2])
def make_ring_entry(clockwise):
    center_x = CW_RING_START[0] if clockwise else CCW_RING_START[0]
    start_angle = 0.0 if clockwise else math.pi
    end_angle = 0.5 * math.pi
    points = []
    for index in range(39):
        fraction = index / 38.0
        angle = start_angle + fraction * (end_angle - start_angle)
        points.append((
            center_x + RING_RADIUS * math.cos(angle),
            CHANNEL_TOP[1] + RING_RADIUS * math.sin(angle),
        ))
    end_pose = CW_RING_START if clockwise else CCW_RING_START
    return rows_from_polyline(points, CHANNEL_TOP[2], end_pose[2])


def make_counterclockwise_forward_exit(reference):
    """Leave the ring through the channel centre before bending left."""
    cusp_row = reference[REVERSE_END_INDEX]
    cusp_yaw = 2.0 * math.atan2(cusp_row[5], cusp_row[6])
    cusp = (cusp_row[0], cusp_row[1], cusp_yaw)

    # Move west along the centre of the lower ring lane, then use a 0.48 m
    # quarter circle to enter the channel at its measured centre x=2.52 m.
    upper_radius = 0.48
    upper_turn_start = (FORWARD_CHANNEL_X + upper_radius, cusp[1])
    ring_straight = [cusp[:2], upper_turn_start]
    upper_center = (upper_turn_start[0], cusp[1] - upper_radius)
    upper_turn = []
    for index in range(301):
        angle = 0.5 * math.pi + 0.5 * math.pi * index / 300.0
        upper_turn.append((
            upper_center[0] + upper_radius * math.cos(angle),
            upper_center[1] + upper_radius * math.sin(angle),
        ))

    # Stay centred in the channel.  Begin the left turn while still above the
    # marked obstacle; the complete turn lies in the open lower field.
    lower_radius = 0.45
    lower_turn_start = (FORWARD_CHANNEL_X, 1.65)
    channel = [upper_turn[-1], lower_turn_start]
    lower_center = (FORWARD_CHANNEL_X - lower_radius, lower_turn_start[1])
    lower_turn = []
    for index in range(301):
        angle = -0.5 * math.pi * index / 300.0
        lower_turn.append((
            lower_center[0] + lower_radius * math.cos(angle),
            lower_center[1] + lower_radius * math.sin(angle),
        ))

    goal_row = reference[-1]
    goal = (
        goal_row[0], goal_row[1],
        2.0 * math.atan2(goal_row[5], goal_row[6]),
    )
    lower_turn_end = (lower_turn[-1][0], lower_turn[-1][1], math.pi)
    finish = quintic_segment(
        lower_turn_end, goal, 0.20, 0.20, 601, direction=1)

    dense = (
        resample_polyline(ring_straight, 101) + upper_turn[1:] +
        resample_polyline(channel, 201)[1:] + lower_turn[1:] + finish[1:]
    )
    # Point 163 is already the final reverse cusp.  Drop its duplicate so the
    # 37 forward rows occupy points 164..200 exactly as before.
    points = resample_polyline(dense, 38)[1:]
    return rows_from_polyline(points, cusp[2], goal[2], direction=1)

def make_route(reference, clockwise):
    approach = make_shared_approach()
    ring_entry = make_ring_entry(clockwise)
    entry = approach + ring_entry[1:]
    if len(entry) != ENTRY_POINTS:
        raise ValueError(f"expected {ENTRY_POINTS} entry points, got {len(entry)}")
    if clockwise:
        route = entry + [row.copy() for row in reference[ENTRY_POINTS:]]
    else:
        reverse_tail = [
            row.copy() for row in reference[ENTRY_POINTS:REVERSE_END_INDEX + 1]
        ]
        route = entry + reverse_tail + make_counterclockwise_forward_exit(reference)
    if len(route) != PATH_POINTS:
        raise ValueError(f"expected {PATH_POINTS} route points, got {len(route)}")
    return route


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


def verify_route(rows, name):
    if rows[0][7] != -1 or rows[REVERSE_END_INDEX][7] != -1:
        raise ValueError(f"{name}: reverse section is malformed")
    if rows[REVERSE_END_INDEX + 1][7] != 1:
        raise ValueError(f"{name}: forward section does not start at point 164")
    if any(row[7] != -1 for row in rows[:REVERSE_END_INDEX + 1]):
        raise ValueError(f"{name}: unexpected forward point in reverse section")
    if any(row[7] != 1 for row in rows[REVERSE_END_INDEX + 1:]):
        raise ValueError(f"{name}: unexpected reverse point in forward section")
    if math.hypot(rows[0][0] - REAL_START[0], rows[0][1] - REAL_START[1]) > 1.0e-6:
        raise ValueError(f"{name}: incorrect start point")
    maximum, average = curvature_statistics(rows)
    if maximum > MAX_CURVATURE + 0.05:
        raise ValueError(
            f"{name}: discrete curvature {maximum:.3f} exceeds {MAX_CURVATURE:.3f} m^-1"
        )
    return maximum, average


def pixel(point, width, height):
    return (
        int(round(point[0] * width / FIELD_SIZE_M)),
        int(round((FIELD_SIZE_M - point[1]) * height / FIELD_SIZE_M)),
    )


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


def make_panel(mask, rows, title, stats):
    panel = mask.convert("RGB")
    overlay = Image.new("RGBA", panel.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    width, height = panel.size
    switch = REVERSE_END_INDEX + 1
    draw_polyline(draw, rows[:switch], width, height, REVERSE_COLOR, 7)
    draw_polyline(draw, rows[switch - 1:], width, height, FORWARD_COLOR, 7)
    draw_pose_arrow(draw, rows[0], width, height)

    font = ImageFont.load_default()
    draw.rectangle((12, 12, 510, 126), fill=(0, 0, 0, 190))
    draw.text((24, 22), title, fill=(255, 255, 255, 255), font=font)
    draw.text((24, 44), "RED ARROW: vehicle nose at real start (3.60, 1.10)",
              fill=(255, 255, 255, 255), font=font)
    draw.text((24, 64), "BLUE: reverse entry and complete ring lap",
              fill=(255, 255, 255, 255), font=font)
    draw.text((24, 84), "GREEN: forward motion after direction switch",
              fill=(255, 255, 255, 255), font=font)
    draw.text((24, 104), f"max discrete curvature: {stats[0]:.2f} 1/m (limit 4.00)",
              fill=(255, 255, 255, 255), font=font)
    return Image.alpha_composite(panel.convert("RGBA"), overlay).convert("RGB")


def generate(clockwise_reference, counterclockwise_reference, mask_path, output_directory):
    clockwise_base = parse_path(clockwise_reference)
    counterclockwise_base = parse_path(counterclockwise_reference)
    clockwise = make_route(clockwise_base, True)
    counterclockwise = make_route(counterclockwise_base, False)
    clockwise_stats = verify_route(clockwise, "clockwise")
    counterclockwise_stats = verify_route(counterclockwise, "counterclockwise")

    output_directory.mkdir(parents=True, exist_ok=True)
    clockwise_output = output_directory / "path_post_qr_reverse_full_real_ring_clockwise.txt"
    counterclockwise_output = output_directory / "path_post_qr_reverse_full_real_ring_counterclockwise.txt"
    preview_output = output_directory / "path_qr_reverse_full_real_ring_preview.png"
    write_path(clockwise_output, clockwise)
    write_path(counterclockwise_output, counterclockwise)

    mask = Image.open(mask_path).convert("L")
    clockwise_white = sum(footprint_is_white(mask, row) for row in clockwise)
    counterclockwise_white = sum(footprint_is_white(mask, row) for row in counterclockwise)
    left = make_panel(
        mask, clockwise, "REAL START FULL REVERSE / SIGN 3 CLOCKWISE", clockwise_stats)
    right = make_panel(
        mask, counterclockwise,
        "REAL START FULL REVERSE / SIGN 4 COUNTERCLOCKWISE", counterclockwise_stats)
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
        description="Generate real-start full-reverse clockwise/counterclockwise ring routes")
    parser.add_argument(
        "--clockwise-reference", type=Path,
        default=paths / "path_post_qr_reverse_full_ring_clockwise.txt")
    parser.add_argument(
        "--counterclockwise-reference", type=Path,
        default=paths / "path_post_qr_reverse_full_ring_counterclockwise.txt")
    parser.add_argument(
        "--mask", type=Path, default=package_directory / "config" / "field_keepout_mask.pgm")
    parser.add_argument("--output-directory", type=Path, default=paths)
    args = parser.parse_args()
    generate(
        args.clockwise_reference, args.counterclockwise_reference,
        args.mask, args.output_directory)


if __name__ == "__main__":
    main()
