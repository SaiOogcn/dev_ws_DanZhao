#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


PATH_POINTS = 200
FIXED_ENTRY_END_POINT = 72
FIELD_SIZE_M = 5.0
QR_GOAL = (4.8, 1.8, math.radians(45.0))
POST_PATH_HEAD = (4.2, 1.5, math.radians(30.2))
PARKING_FORWARD_CUSP = (4.3, 1.56, math.radians(30.2))
CHANNEL_ENTRY = (2.7, 0.95, math.radians(90.0))
REVERSE_RING_LOWER_APPROACH = (3.15, 0.90, 0.0)
REVERSE_RING_CHANNEL = (2.65, 1.40, math.radians(-90.0))
REVERSE_RING_TOP = (2.65, 2.72, math.radians(-90.0))
REVERSE_RING_TURN_EXIT = (2.15, 3.22, 0.0)
REVERSE_RING_CUSP = (1.75, 3.20, 0.0)
REVERSE_RING_CORNER_RADIUS = 0.35
# A/B route: move the reverse channel left and start each bend earlier.
REVERSE_RING_INNER_LOWER_APPROACH = (3.00, 0.90, 0.0)
REVERSE_RING_INNER_CHANNEL = (2.45, 1.32, math.radians(-90.0))
REVERSE_RING_INNER_TOP = (2.45, 2.60, math.radians(-90.0))
REVERSE_RING_INNER_TURN_EXIT = (2.00, 3.20, 0.0)
REVERSE_RING_INNER_CUSP = (1.62, 3.20, 0.0)
REVERSE_RING_INNER_TOP_Y = 4.30
REVERSE_RING_CLOCKWISE_TURN_EXIT = (3.15, 3.22, math.pi)
REVERSE_RING_CLOCKWISE_CUSP = (3.55, 3.20, math.pi)
REVERSE_RING_INNER_CLOCKWISE_TURN_EXIT = (2.90, 3.20, math.pi)
REVERSE_RING_INNER_CLOCKWISE_CUSP = (3.28, 3.20, math.pi)
REVERSE_FULL_RING_CCW_START = (3.13, 3.20, math.pi)
REVERSE_FULL_RING_CW_START = (2.17, 3.20, 0.0)
REVERSE_FULL_RING_CCW_EXIT_CUSP = (3.45, 3.20, math.pi)
REVERSE_FULL_RING_CW_EXIT_CUSP = (1.85, 3.20, 0.0)


def parse_path(path):
    rows = []
    for line in path.read_text(encoding="ascii").splitlines():
        fields = line.rstrip(",").split(",")
        if len(fields) < 8:
            raise ValueError(f"malformed path row: {line}")
        rows.append([float(value) for value in fields[:7]] + [int(fields[7])])
    return rows


def path_row(x, y, yaw, direction):
    return [
        x, y, 0.0, 0.0, 0.0,
        math.sin(0.5 * yaw), math.cos(0.5 * yaw), direction,
    ]


def cubic_bezier(p0, p1, p2, p3, count, direction):
    rows = []
    for index in range(count):
        t = index / (count - 1)
        u = 1.0 - t
        x = u**3 * p0[0] + 3.0 * u**2 * t * p1[0] + 3.0 * u * t**2 * p2[0] + t**3 * p3[0]
        y = u**3 * p0[1] + 3.0 * u**2 * t * p1[1] + 3.0 * u * t**2 * p2[1] + t**3 * p3[1]
        dx = 3.0 * (u**2 * (p1[0] - p0[0]) + 2.0 * u * t * (p2[0] - p1[0]) + t**2 * (p3[0] - p2[0]))
        dy = 3.0 * (u**2 * (p1[1] - p0[1]) + 2.0 * u * t * (p2[1] - p1[1]) + t**2 * (p3[1] - p2[1]))
        travel_yaw = math.atan2(dy, dx)
        vehicle_yaw = travel_yaw if direction > 0 else travel_yaw + math.pi
        rows.append(path_row(x, y, vehicle_yaw, direction))
    return rows


def cubic_between(start, end, count, direction, start_tangent, end_tangent):
    start_travel_yaw = start[2] if direction > 0 else start[2] + math.pi
    end_travel_yaw = end[2] if direction > 0 else end[2] + math.pi
    p0 = start[:2]
    p3 = end[:2]
    p1 = (
        p0[0] + start_tangent * math.cos(start_travel_yaw),
        p0[1] + start_tangent * math.sin(start_travel_yaw),
    )
    p2 = (
        p3[0] - end_tangent * math.cos(end_travel_yaw),
        p3[1] - end_tangent * math.sin(end_travel_yaw),
    )
    return cubic_bezier(p0, p1, p2, p3, count, direction)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def right_arc(start, turn_angle, radius, count):
    center = (
        start[0] + radius * math.sin(start[2]),
        start[1] - radius * math.cos(start[2]),
    )
    rows = []
    for index in range(count):
        t = index / (count - 1)
        yaw = normalize_angle(start[2] - t * turn_angle)
        x = center[0] - radius * math.sin(yaw)
        y = center[1] + radius * math.cos(yaw)
        rows.append(path_row(x, y, yaw, 1))
    return rows


def make_qr_sprint(start_row):
    p0 = (start_row[0], start_row[1])
    start_yaw = 2.0 * math.atan2(start_row[5], start_row[6])
    p3 = QR_GOAL[:2]
    p1 = (p0[0] + 1.45 * math.cos(start_yaw), p0[1] + 1.45 * math.sin(start_yaw))
    p2 = (p3[0] - 1.15 * math.cos(QR_GOAL[2]), p3[1] - 1.15 * math.sin(QR_GOAL[2]))
    return cubic_bezier(p0, p1, p2, p3, 47, 1)


def make_parking_entry(base_rows):
    fixed_entry_end = base_rows[FIXED_ENTRY_END_POINT - 1]
    entry_yaw = 2.0 * math.atan2(fixed_entry_end[5], fixed_entry_end[6])
    approach = cubic_between(
        POST_PATH_HEAD, PARKING_FORWARD_CUSP, 4, 1, 0.08, 0.08)
    original_reverse_head = base_rows[32]
    original_reverse_pose = (
        original_reverse_head[0], original_reverse_head[1],
        2.0 * math.atan2(original_reverse_head[5], original_reverse_head[6]),
    )
    reverse_lead = cubic_between(
        PARKING_FORWARD_CUSP, original_reverse_pose, 13, -1, 0.42, 0.35)
    reverse_tail = [row.copy() for row in base_rows[33:47]]
    channel = cubic_between(
        CHANNEL_ENTRY,
        (fixed_entry_end[0], fixed_entry_end[1], entry_yaw),
        42,
        1,
        0.62,
        0.62,
    )
    entry = approach + reverse_lead[1:] + reverse_tail + channel
    if len(entry) != FIXED_ENTRY_END_POINT:
        raise ValueError(f"parking entry must have 72 points, got {len(entry)}")
    return entry


def make_right_turn_entry(base_rows):
    fixed_entry_end = base_rows[FIXED_ENTRY_END_POINT - 1]
    entry_yaw = 2.0 * math.atan2(fixed_entry_end[5], fixed_entry_end[6])
    rear_turn = right_arc(POST_PATH_HEAD, math.radians(210.0), 0.40, 30)
    rear_end = rear_turn[-1]
    rear_end_pose = (
        rear_end[0], rear_end[1],
        2.0 * math.atan2(rear_end[5], rear_end[6]),
    )
    lower_straight_pose = (3.20, rear_end[1], math.radians(180.0))
    lower_straight = cubic_between(
        rear_end_pose, lower_straight_pose, 12, 1, 0.42, 0.42)
    channel_turn = right_arc(
        lower_straight_pose, math.radians(90.0), 0.40, 14)
    channel_turn_end = channel_turn[-1]
    channel_turn_end_pose = (
        channel_turn_end[0], channel_turn_end[1],
        2.0 * math.atan2(channel_turn_end[5], channel_turn_end[6]),
    )
    channel = cubic_between(
        channel_turn_end_pose,
        (fixed_entry_end[0], fixed_entry_end[1], entry_yaw),
        19,
        1,
        0.62,
        0.62,
    )
    entry = rear_turn + lower_straight[1:] + channel_turn[1:] + channel[1:]
    if len(entry) != FIXED_ENTRY_END_POINT:
        raise ValueError(f"right-turn entry must have 72 points, got {len(entry)}")
    return entry


def make_reverse_ring_counterclockwise(base_rows):
    """Reverse directly into the lower-left ring, then drive forward CCW."""
    ring_join_row = base_rows[77]
    ring_join = (
        ring_join_row[0], ring_join_row[1],
        2.0 * math.atan2(ring_join_row[5], ring_join_row[6]),
    )

    leave_qr = cubic_between(
        POST_PATH_HEAD, REVERSE_RING_LOWER_APPROACH, 16, -1, 0.70, 0.70)
    enter_channel = cubic_between(
        REVERSE_RING_LOWER_APPROACH, REVERSE_RING_CHANNEL,
        10, -1, 0.276, 0.276)
    reverse_channel = cubic_between(
        REVERSE_RING_CHANNEL, REVERSE_RING_TOP, 16, -1, 0.40, 0.40)
    reverse_into_ring = cubic_between(
        REVERSE_RING_TOP, REVERSE_RING_TURN_EXIT,
        10, -1, 0.276, 0.276)
    reverse_ring_straight = cubic_between(
        REVERSE_RING_TURN_EXIT, REVERSE_RING_CUSP, 7, -1, 0.15, 0.15)
    forward_on_ring = cubic_between(
        REVERSE_RING_CUSP, ring_join, 24, 1, 0.55, 0.55)

    prefix = (
        leave_qr + enter_channel[1:] + reverse_channel[1:] +
        reverse_into_ring[1:] + reverse_ring_straight[1:] +
        forward_on_ring[1:]
    )
    if len(prefix) != 78:
        raise ValueError(f"reverse-ring prefix must have 78 points, got {len(prefix)}")

    post = prefix + make_tight_reverse_ring(base_rows) + [row.copy() for row in base_rows[158:]]
    if len(post) != PATH_POINTS:
        raise ValueError(f"reverse-ring path must have {PATH_POINTS} points, got {len(post)}")
    return post


def make_reverse_ring_counterclockwise_inner(base_rows):
    """A/B route with a left-shifted reverse entry and an inner upper loop."""
    ring_join_row = base_rows[77]
    ring_join = (
        ring_join_row[0], ring_join_row[1],
        2.0 * math.atan2(ring_join_row[5], ring_join_row[6]),
    )

    leave_qr = cubic_between(
        POST_PATH_HEAD, REVERSE_RING_INNER_LOWER_APPROACH, 16, -1, 0.78, 0.34)
    enter_channel = cubic_between(
        REVERSE_RING_INNER_LOWER_APPROACH, REVERSE_RING_INNER_CHANNEL,
        10, -1, 0.32, 0.32)
    reverse_channel = cubic_between(
        REVERSE_RING_INNER_CHANNEL, REVERSE_RING_INNER_TOP,
        16, -1, 0.40, 0.40)
    reverse_into_ring = cubic_between(
        REVERSE_RING_INNER_TOP, REVERSE_RING_INNER_TURN_EXIT,
        10, -1, 0.32, 0.32)
    reverse_ring_straight = cubic_between(
        REVERSE_RING_INNER_TURN_EXIT, REVERSE_RING_INNER_CUSP, 7, -1, 0.15, 0.15)
    forward_on_ring = cubic_between(
        REVERSE_RING_INNER_CUSP, ring_join, 24, 1, 0.55, 0.55)

    prefix = (
        leave_qr + enter_channel[1:] + reverse_channel[1:] +
        reverse_into_ring[1:] + reverse_ring_straight[1:] +
        forward_on_ring[1:]
    )
    if len(prefix) != 78:
        raise ValueError(f"inner reverse-ring prefix must have 78 points, got {len(prefix)}")

    post = prefix + make_tight_reverse_ring(
        base_rows, top_y=REVERSE_RING_INNER_TOP_Y) + [row.copy() for row in base_rows[158:]]
    if len(post) != PATH_POINTS:
        raise ValueError(f"inner reverse-ring path must have {PATH_POINTS} points, got {len(post)}")
    return post


def make_reverse_ring_clockwise(base_rows):
    """Reverse through the channel, then drive forward clockwise."""
    return make_reverse_ring_clockwise_variant(
        base_rows,
        REVERSE_RING_LOWER_APPROACH,
        REVERSE_RING_CHANNEL,
        REVERSE_RING_TOP,
        REVERSE_RING_CLOCKWISE_TURN_EXIT,
        REVERSE_RING_CLOCKWISE_CUSP,
        4.42,
        0.70,
        0.70,
        0.276,
    )


def make_reverse_ring_clockwise_inner(base_rows):
    """Clockwise A/B route with the same left-shifted inner geometry."""
    return make_reverse_ring_clockwise_variant(
        base_rows,
        REVERSE_RING_INNER_LOWER_APPROACH,
        REVERSE_RING_INNER_CHANNEL,
        REVERSE_RING_INNER_TOP,
        REVERSE_RING_INNER_CLOCKWISE_TURN_EXIT,
        REVERSE_RING_INNER_CLOCKWISE_CUSP,
        REVERSE_RING_INNER_TOP_Y,
        0.78,
        0.34,
        0.32,
    )


def make_reverse_ring_clockwise_variant(
        base_rows, lower_approach, channel_pose, channel_top, turn_exit,
        cusp, top_y, leave_start_tangent, leave_end_tangent,
        turn_tangent):
    ring_join_row = base_rows[77]
    ring_join = (
        ring_join_row[0], ring_join_row[1],
        2.0 * math.atan2(ring_join_row[5], ring_join_row[6]),
    )

    leave_qr = cubic_between(
        POST_PATH_HEAD, lower_approach, 16, -1,
        leave_start_tangent, leave_end_tangent)
    enter_channel = cubic_between(
        lower_approach, channel_pose, 10, -1, turn_tangent, turn_tangent)
    reverse_channel = cubic_between(
        channel_pose, channel_top, 16, -1, 0.40, 0.40)
    reverse_into_ring = cubic_between(
        channel_top, turn_exit, 10, -1, turn_tangent, turn_tangent)
    reverse_ring_straight = cubic_between(
        turn_exit, cusp, 7, -1, 0.15, 0.15)
    forward_on_ring = cubic_between(
        cusp, ring_join, 24, 1, 0.55, 0.55)

    prefix = (
        leave_qr + enter_channel[1:] + reverse_channel[1:] +
        reverse_into_ring[1:] + reverse_ring_straight[1:] +
        forward_on_ring[1:]
    )
    if len(prefix) != 78:
        raise ValueError(f"clockwise reverse-ring prefix must have 78 points, got {len(prefix)}")

    post = prefix + make_tight_ring(
        base_rows, clockwise=True, top_y=top_y) + [row.copy() for row in base_rows[158:]]
    if len(post) != PATH_POINTS:
        raise ValueError(f"clockwise reverse-ring path must have {PATH_POINTS} points, got {len(post)}")
    return post


def sample_track_segments(segments, count, direction):
    lengths = []
    for segment in segments:
        kind = segment[0]
        if kind == "line":
            _, first, second = segment
            lengths.append(math.hypot(second[0] - first[0], second[1] - first[1]))
        else:
            _, _, first, second, radius = segment
            lengths.append(radius * abs(second - first))
    total_length = sum(lengths)

    rows = []
    for index in range(count):
        distance = total_length * index / (count - 1)
        segment_index = 0
        while segment_index < len(lengths) - 1 and distance > lengths[segment_index]:
            distance -= lengths[segment_index]
            segment_index += 1
        segment = segments[segment_index]
        kind = segment[0]
        if kind == "line":
            _, first, second = segment
            length = lengths[segment_index]
            fraction = distance / length if length > 0.0 else 0.0
            x = first[0] + fraction * (second[0] - first[0])
            y = first[1] + fraction * (second[1] - first[1])
            travel_yaw = math.atan2(second[1] - first[1], second[0] - first[0])
        else:
            _, center, start_angle, end_angle, radius = segment
            turn_sign = 1.0 if end_angle > start_angle else -1.0
            angle = start_angle + turn_sign * distance / radius
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            travel_yaw = normalize_angle(angle + turn_sign * 0.5 * math.pi)
        vehicle_yaw = travel_yaw if direction > 0 else travel_yaw + math.pi
        rows.append(path_row(x, y, vehicle_yaw, direction))
    return rows


def ring_segments(start_x, end_x, clockwise, top_y):
    radius = REVERSE_RING_CORNER_RADIUS
    right_x = 4.25
    left_x = 0.75
    bottom_y = 3.20

    if clockwise:
        return [
            ("line", (start_x, bottom_y), (left_x + radius, bottom_y)),
            ("arc", (left_x + radius, bottom_y + radius), -0.5 * math.pi, -math.pi, radius),
            ("line", (left_x, bottom_y + radius), (left_x, top_y - radius)),
            ("arc", (left_x + radius, top_y - radius), math.pi, 0.5 * math.pi, radius),
            ("line", (left_x + radius, top_y), (right_x - radius, top_y)),
            ("arc", (right_x - radius, top_y - radius), 0.5 * math.pi, 0.0, radius),
            ("line", (right_x, top_y - radius), (right_x, bottom_y + radius)),
            ("arc", (right_x - radius, bottom_y + radius), 0.0, -0.5 * math.pi, radius),
            ("line", (right_x - radius, bottom_y), (end_x, bottom_y)),
        ]
    return [
        ("line", (start_x, bottom_y), (right_x - radius, bottom_y)),
        ("arc", (right_x - radius, bottom_y + radius), -0.5 * math.pi, 0.0, radius),
        ("line", (right_x, bottom_y + radius), (right_x, top_y - radius)),
        ("arc", (right_x - radius, top_y - radius), 0.0, 0.5 * math.pi, radius),
        ("line", (right_x - radius, top_y), (left_x + radius, top_y)),
        ("arc", (left_x + radius, top_y - radius), 0.5 * math.pi, math.pi, radius),
        ("line", (left_x, top_y - radius), (left_x, bottom_y + radius)),
        ("arc", (left_x + radius, bottom_y + radius), math.pi, 1.5 * math.pi, radius),
        ("line", (left_x + radius, bottom_y), (end_x, bottom_y)),
    ]


def make_tight_ring(base_rows, clockwise=False, top_y=4.42):
    """Resample a forward loop with 0.35 m-radius corners."""
    start = base_rows[78]
    end = base_rows[157]
    return sample_track_segments(
        ring_segments(start[0], end[0], clockwise, top_y), 80, 1)


def make_tight_reverse_ring(base_rows, top_y=4.42):
    return make_tight_ring(base_rows, clockwise=False, top_y=top_y)


def make_reverse_full_ring(base_rows, clockwise):
    """Reverse through the entry and one complete ring lap, then drive out."""
    ring_start = REVERSE_FULL_RING_CW_START if clockwise else REVERSE_FULL_RING_CCW_START
    exit_cusp = (
        REVERSE_FULL_RING_CW_EXIT_CUSP if clockwise else
        REVERSE_FULL_RING_CCW_EXIT_CUSP
    )

    leave_qr = cubic_between(
        POST_PATH_HEAD, REVERSE_RING_LOWER_APPROACH, 16, -1, 0.70, 0.70)
    enter_channel = cubic_between(
        REVERSE_RING_LOWER_APPROACH, REVERSE_RING_CHANNEL,
        10, -1, 0.276, 0.276)
    reverse_channel = cubic_between(
        REVERSE_RING_CHANNEL, REVERSE_RING_TOP, 16, -1, 0.40, 0.40)
    entry_turn_radius = 0.48
    if clockwise:
        entry_turn = [
            ("arc", (ring_start[0], REVERSE_RING_TOP[1]),
             0.0, 0.5 * math.pi, entry_turn_radius),
        ]
    else:
        entry_turn = [
            ("arc", (ring_start[0], REVERSE_RING_TOP[1]),
             math.pi, 0.5 * math.pi, entry_turn_radius),
        ]
    reverse_into_lap = sample_track_segments(entry_turn, 39, -1)
    entry = (
        leave_qr + enter_channel[1:] + reverse_channel[1:] +
        reverse_into_lap[1:]
    )
    if len(entry) != 78:
        raise ValueError(f"full-reverse entry must have 78 points, got {len(entry)}")

    lap = sample_track_segments(
        ring_segments(ring_start[0], ring_start[0], clockwise, 4.42),
        81,
        -1,
    )
    reverse_overshoot = cubic_between(
        ring_start, exit_cusp, 6, -1, 0.15, 0.15)

    exit_join_row = base_rows[174]
    exit_join = (
        exit_join_row[0], exit_join_row[1],
        2.0 * math.atan2(exit_join_row[5], exit_join_row[6]),
    )
    forward_turn_into_channel = cubic_between(
        exit_cusp, exit_join, 13, 1, 0.55, 0.40)
    post = (
        entry + lap[1:] + reverse_overshoot[1:] +
        forward_turn_into_channel[1:] +
        [row.copy() for row in base_rows[175:]]
    )
    if len(post) != PATH_POINTS:
        raise ValueError(f"full-reverse path must have {PATH_POINTS} points, got {len(post)}")
    return post


def make_post_path(base_rows, mode):
    fixed_entry = (
        make_parking_entry(base_rows) if mode == "parking" else
        make_right_turn_entry(base_rows)
    )
    post = fixed_entry + [row.copy() for row in base_rows[FIXED_ENTRY_END_POINT:]]
    if len(post) != PATH_POINTS:
        raise ValueError(f"post-QR path must have {PATH_POINTS} points, got {len(post)}")
    return post


def write_path(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="\n") as stream:
        for x, y, z, qx, qy, qz, qw, direction in rows:
            stream.write(
                f"{x:.4f},{y:.4f},{z:.4f},{qx:.4f},{qy:.4f},"
                f"{qz:.4f},{qw:.4f},{direction},\n")


def pixel(point, width, height):
    return (
        int(round(point[0] * width / FIELD_SIZE_M)),
        int(round((FIELD_SIZE_M - point[1]) * height / FIELD_SIZE_M)),
    )


def draw_polyline(draw, rows, width, height, color, line_width=6):
    points = [pixel(row, width, height) for row in rows]
    if len(points) >= 2:
        draw.line(points, fill=color, width=line_width, joint="curve")


def make_panel(mask, sprint, post, title, ring_color, entry_label):
    panel = mask.convert("RGB")
    overlay = Image.new("RGBA", panel.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    width, height = panel.size

    draw_polyline(draw, sprint, width, height, (245, 75, 55, 255), 7)
    draw_polyline(draw, post[:72], width, height, (25, 185, 90, 255), 7)
    draw_polyline(draw, post[71:160], width, height, ring_color, 6)
    draw_polyline(draw, post[159:], width, height, (120, 120, 120, 255), 4)

    for fraction in (0.55, 0.70, 0.85, 1.0):
        index = min(len(sprint) - 1, round(fraction * (len(sprint) - 1)))
        x, y = pixel(sprint[index], width, height)
        draw.ellipse((x - 7, y - 7, x + 7, y + 7), fill=(255, 210, 0, 255))

    font = ImageFont.load_default()
    draw.rectangle((12, 12, 425, 126), fill=(0, 0, 0, 190))
    draw.text((24, 22), title, fill=(255, 255, 255, 255), font=font)
    draw.text((24, 44), "RED: QR sprint to (4.8, 1.8, 45 deg)", fill=(255, 255, 255, 255), font=font)
    draw.text((24, 64), "YELLOW: possible immediate QR switch poses", fill=(255, 255, 255, 255), font=font)
    draw.text((24, 84), entry_label, fill=(255, 255, 255, 255), font=font)
    draw.text((24, 104), "Built-in local avoidance handles blocked sections", fill=(255, 255, 255, 255), font=font)
    return Image.alpha_composite(panel.convert("RGBA"), overlay).convert("RGB")


def make_reverse_ring_panel(mask, sprint, post, title, full_reverse=False):
    panel = mask.convert("RGB")
    overlay = Image.new("RGBA", panel.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    width, height = panel.size

    switch_index = next(
        index for index, row in enumerate(post[1:], start=1)
        if row[7] > 0
    )
    draw_polyline(draw, sprint, width, height, (245, 75, 55, 255), 7)
    draw_polyline(draw, post[:switch_index], width, height, (55, 115, 255, 255), 7)
    if full_reverse:
        draw_polyline(draw, post[switch_index - 1:], width, height, (25, 205, 105, 255), 7)
    else:
        draw_polyline(draw, post[switch_index - 1:78], width, height, (25, 205, 105, 255), 7)
        draw_polyline(draw, post[77:160], width, height, (0, 190, 225, 255), 6)
        draw_polyline(draw, post[159:], width, height, (120, 120, 120, 255), 4)

    font = ImageFont.load_default()
    draw.rectangle((12, 12, 470, 126), fill=(0, 0, 0, 190))
    draw.text((24, 22), title,
              fill=(255, 255, 255, 255), font=font)
    draw.text((24, 44), "RED: QR sprint", fill=(255, 255, 255, 255), font=font)
    draw.text((24, 64), "BLUE: reverse motion",
              fill=(255, 255, 255, 255), font=font)
    draw.text((24, 84), "GREEN: forward motion after the direction switch",
              fill=(255, 255, 255, 255), font=font)
    draw.text((24, 104), "CYAN: forward ring lap (standard routes only)",
              fill=(255, 255, 255, 255), font=font)
    return Image.alpha_composite(panel.convert("RGBA"), overlay).convert("RGB")


def generate(counterclockwise_path, clockwise_path, mask_path, output_directory):
    counterclockwise = parse_path(counterclockwise_path)
    clockwise = parse_path(clockwise_path)
    if len(counterclockwise) != PATH_POINTS or len(clockwise) != PATH_POINTS:
        raise ValueError("both competition paths must contain exactly 200 points")
    if counterclockwise[:71] != clockwise[:71]:
        raise ValueError("clockwise and counterclockwise paths must share the parking/ring-entry prefix")

    sprint = make_qr_sprint(counterclockwise[0])
    parking_counterclockwise = make_post_path(counterclockwise, "parking")
    parking_clockwise = make_post_path(clockwise, "parking")
    right_turn_counterclockwise = make_post_path(counterclockwise, "right_turn")
    right_turn_clockwise = make_post_path(clockwise, "right_turn")
    reverse_ring_counterclockwise = make_reverse_ring_counterclockwise(counterclockwise)
    reverse_ring_counterclockwise_inner = make_reverse_ring_counterclockwise_inner(counterclockwise)
    reverse_ring_clockwise = make_reverse_ring_clockwise(clockwise)
    reverse_ring_clockwise_inner = make_reverse_ring_clockwise_inner(clockwise)
    reverse_full_ring_counterclockwise = make_reverse_full_ring(counterclockwise, False)
    reverse_full_ring_clockwise = make_reverse_full_ring(clockwise, True)

    output_directory.mkdir(parents=True, exist_ok=True)
    write_path(output_directory / "path_qr_sprint.txt", sprint)
    write_path(output_directory / "path_post_qr_parking_counterclockwise.txt", parking_counterclockwise)
    write_path(output_directory / "path_post_qr_parking_clockwise.txt", parking_clockwise)
    write_path(output_directory / "path_post_qr_right_turn_counterclockwise.txt", right_turn_counterclockwise)
    write_path(output_directory / "path_post_qr_right_turn_clockwise.txt", right_turn_clockwise)
    write_path(
        output_directory / "path_post_qr_reverse_ring_counterclockwise.txt",
        reverse_ring_counterclockwise)
    write_path(
        output_directory / "path_post_qr_reverse_ring_counterclockwise_inner.txt",
        reverse_ring_counterclockwise_inner)
    write_path(
        output_directory / "path_post_qr_reverse_ring_clockwise.txt",
        reverse_ring_clockwise)
    write_path(
        output_directory / "path_post_qr_reverse_ring_clockwise_inner.txt",
        reverse_ring_clockwise_inner)
    write_path(
        output_directory / "path_post_qr_reverse_full_ring_counterclockwise.txt",
        reverse_full_ring_counterclockwise)
    write_path(
        output_directory / "path_post_qr_reverse_full_ring_clockwise.txt",
        reverse_full_ring_clockwise)
    # Keep the old filenames as aliases for the default right-turn plan.
    write_path(output_directory / "path_post_qr_counterclockwise.txt", right_turn_counterclockwise)
    write_path(output_directory / "path_post_qr_clockwise.txt", right_turn_clockwise)
    write_path(output_directory / "path_competition_clockwise.txt", clockwise)

    mask = Image.open(mask_path).convert("L")
    right_label = "GREEN: forward right-turn entry from (4.2, 1.5, 30.2 deg)"
    left = make_panel(
        mask, sprint, right_turn_clockwise, "RIGHT TURN / SIGN 3 CLOCKWISE",
        (210, 55, 220, 255), right_label)
    right = make_panel(
        mask, sprint, right_turn_counterclockwise, "RIGHT TURN / SIGN 4 COUNTERCLOCKWISE",
        (0, 190, 225, 255), right_label)
    preview = Image.new("RGB", (left.width + right.width, left.height), (20, 20, 20))
    preview.paste(left, (0, 0))
    preview.paste(right, (left.width, 0))
    preview.save(output_directory / "path_qr_split_preview.png")

    parking_label = "GREEN: forward-reverse-forward parking entry from (4.2, 1.5)"
    parking_left = make_panel(
        mask, sprint, parking_clockwise, "PARKING / SIGN 3 CLOCKWISE",
        (210, 55, 220, 255), parking_label)
    parking_right = make_panel(
        mask, sprint, parking_counterclockwise, "PARKING / SIGN 4 COUNTERCLOCKWISE",
        (0, 190, 225, 255), parking_label)
    variants = Image.new("RGB", (2 * left.width, 2 * left.height), (20, 20, 20))
    variants.paste(parking_left, (0, 0))
    variants.paste(parking_right, (left.width, 0))
    variants.paste(left, (0, left.height))
    variants.paste(right, (left.width, left.height))
    variants.save(output_directory / "path_qr_post_variants_preview.png")

    reverse_ring_preview = make_reverse_ring_panel(
        mask, sprint, reverse_ring_counterclockwise,
        "REVERSE ENTRY / SIGN 4 COUNTERCLOCKWISE")
    reverse_ring_preview.save(output_directory / "path_qr_reverse_ring_preview.png")
    reverse_ring_inner_preview = make_reverse_ring_panel(
        mask, sprint, reverse_ring_counterclockwise_inner,
        "INNER REVERSE ENTRY / SIGN 4 COUNTERCLOCKWISE")
    reverse_ring_inner_preview.save(output_directory / "path_qr_reverse_ring_inner_preview.png")

    standard_panels = [
        make_reverse_ring_panel(
            mask, sprint, reverse_ring_clockwise,
            "REVERSE ENTRY / SIGN 3 CLOCKWISE"),
        reverse_ring_preview,
        make_reverse_ring_panel(
            mask, sprint, reverse_ring_clockwise_inner,
            "INNER REVERSE ENTRY / SIGN 3 CLOCKWISE"),
        reverse_ring_inner_preview,
    ]
    reverse_variants = Image.new(
        "RGB", (2 * mask.width, 2 * mask.height), (20, 20, 20))
    for index, panel in enumerate(standard_panels):
        reverse_variants.paste(
            panel, ((index % 2) * mask.width, (index // 2) * mask.height))
    reverse_variants.save(
        output_directory / "path_qr_reverse_ring_variants_preview.png")

    full_reverse_left = make_reverse_ring_panel(
        mask, sprint, reverse_full_ring_clockwise,
        "FULL REVERSE LAP / SIGN 3 CLOCKWISE", True)
    full_reverse_right = make_reverse_ring_panel(
        mask, sprint, reverse_full_ring_counterclockwise,
        "FULL REVERSE LAP / SIGN 4 COUNTERCLOCKWISE", True)
    full_reverse_preview = Image.new(
        "RGB", (2 * mask.width, mask.height), (20, 20, 20))
    full_reverse_preview.paste(full_reverse_left, (0, 0))
    full_reverse_preview.paste(full_reverse_right, (mask.width, 0))
    full_reverse_preview.save(
        output_directory / "path_qr_reverse_full_ring_preview.png")


def main():
    parser = argparse.ArgumentParser(description="Generate two-stage QR competition paths")
    parser.add_argument("counterclockwise_path", type=Path)
    parser.add_argument("clockwise_path", type=Path)
    parser.add_argument("mask", type=Path)
    parser.add_argument("output_directory", type=Path)
    args = parser.parse_args()
    generate(args.counterclockwise_path, args.clockwise_path, args.mask, args.output_directory)


if __name__ == "__main__":
    main()
