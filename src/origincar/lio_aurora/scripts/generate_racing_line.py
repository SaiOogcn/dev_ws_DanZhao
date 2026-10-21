#!/usr/bin/env python3

import argparse
import math
from pathlib import Path


FIRST_REPLACED_POINT = 79
LAST_REPLACED_POINT = 158
BOTTOM_Y = 3.19
TOP_Y = 4.39
RIGHT_JOIN_X = 3.38
LEFT_JOIN_X = 1.66


def parse_path(path):
    rows = []
    for line in path.read_text(encoding="ascii").splitlines():
        fields = line.rstrip(",").split(",")
        if len(fields) < 8:
            raise ValueError(f"malformed path row: {line}")
        rows.append([float(value) for value in fields[:7]] + [int(fields[7])])
    return rows


def integrate_sine_curvature_cap(start_x, start_y, end_y, initial_yaw, samples=2001):
    # k(s) is proportional to sin(pi*s/L). Curvature is zero at both straight
    # joins, and its integral is pi radians for one end of the oval.
    ds = 1.0 / (samples - 1)
    normalization = math.pi * math.pi / 2.0
    normalized = [(0.0, 0.0, initial_yaw)]
    x = 0.0
    y = 0.0
    yaw = initial_yaw
    for index in range(1, samples):
        s0 = (index - 1) * ds
        s1 = index * ds
        k0 = normalization * math.sin(math.pi * s0)
        k1 = normalization * math.sin(math.pi * s1)
        next_yaw = yaw + 0.5 * (k0 + k1) * ds
        x += 0.5 * (math.cos(yaw) + math.cos(next_yaw)) * ds
        y += 0.5 * (math.sin(yaw) + math.sin(next_yaw)) * ds
        yaw = next_yaw
        normalized.append((x, y, yaw))

    scale = (end_y - start_y) / normalized[-1][1]
    return [
        (start_x + scale * x, start_y + scale * y)
        for x, y, _ in normalized
    ]


def append_line(points, start, end, samples):
    for index in range(samples):
        ratio = index / (samples - 1)
        points.append((
            start[0] + ratio * (end[0] - start[0]),
            start[1] + ratio * (end[1] - start[1]),
        ))


def build_dense_ring(start_x, end_x):
    points = []
    append_line(points, (start_x, BOTTOM_Y), (RIGHT_JOIN_X, BOTTOM_Y), 81)
    points.extend(integrate_sine_curvature_cap(
        RIGHT_JOIN_X, BOTTOM_Y, TOP_Y, 0.0)[1:])
    append_line(points, (RIGHT_JOIN_X, TOP_Y), (LEFT_JOIN_X, TOP_Y), 401)
    points.extend(integrate_sine_curvature_cap(
        LEFT_JOIN_X, TOP_Y, BOTTOM_Y, math.pi)[1:])
    append_line(points, (LEFT_JOIN_X, BOTTOM_Y), (end_x, BOTTOM_Y), 201)
    return points


def resample_by_distance(points, count):
    lengths = [0.0]
    for previous, current in zip(points, points[1:]):
        lengths.append(lengths[-1] + math.hypot(
            current[0] - previous[0], current[1] - previous[1]))

    result = []
    segment = 1
    for index in range(count):
        target = lengths[-1] * index / (count - 1)
        while segment < len(lengths) - 1 and lengths[segment] < target:
            segment += 1
        low = segment - 1
        span = lengths[segment] - lengths[low]
        ratio = 0.0 if span <= 1e-12 else (target - lengths[low]) / span
        result.append((
            points[low][0] + ratio * (points[segment][0] - points[low][0]),
            points[low][1] + ratio * (points[segment][1] - points[low][1]),
        ))
    return result


def generate(source, destination):
    if source.resolve() == destination.resolve():
        raise ValueError("source and destination must differ; the original path is preserved")

    rows = parse_path(source)
    if len(rows) != 200:
        raise ValueError(f"expected the 200-point competition path, got {len(rows)} points")

    first = FIRST_REPLACED_POINT - 1
    last = LAST_REPLACED_POINT - 1
    dense = build_dense_ring(rows[first][0], rows[last][0])
    replacement = resample_by_distance(dense, last - first + 1)
    for offset, (x, y) in enumerate(replacement):
        rows[first + offset][0] = x
        rows[first + offset][1] = y

    for index in range(first, last + 1):
        previous = rows[max(0, index - 1)]
        following = rows[min(len(rows) - 1, index + 1)]
        yaw = math.atan2(following[1] - previous[1], following[0] - previous[0])
        rows[index][5] = math.sin(0.5 * yaw)
        rows[index][6] = math.cos(0.5 * yaw)

    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="ascii", newline="\n") as stream:
        for x, y, z, qx, qy, qz, qw, direction in rows:
            stream.write(
                f"{x:.4f},{y:.4f},{z:.4f},{qx:.4f},{qy:.4f},"
                f"{qz:.4f},{qw:.4f},{direction},\n")


def main():
    parser = argparse.ArgumentParser(description="Generate the competition ring racing line")
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    generate(args.source, args.destination)


if __name__ == "__main__":
    main()
