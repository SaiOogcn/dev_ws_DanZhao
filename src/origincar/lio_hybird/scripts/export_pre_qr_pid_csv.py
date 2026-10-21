#!/usr/bin/env python3
"""Merge an origincar_control terminal log with a pre-QR rosbag2 recording."""

import argparse
import csv
import math
import re
from bisect import bisect_right
from decimal import Decimal
from pathlib import Path

import rosbag2_py
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from std_msgs.msg import Bool


LOG_PATTERN = re.compile(
    r"^\[(?P<level>INFO|WARN|ERROR)] \[(?P<time>\d+\.\d+)] "
    r"\[(?P<logger>[^]]+)]\s*:?\s*(?P<message>.*)$"
)
PID_PATTERN = re.compile(
    r"error[：:]\s*(?P<error>[-+\d.eE]+).*?"
    r"integral[：:]\s*(?P<integral>[-+\d.eE]+).*?"
    r"derivative[：:]\s*(?P<derivative>[-+\d.eE]+).*?"
    r"v\s*(?P<output>[-+\d.eE]+)"
)


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def load_bag(bag_directory):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    starts = []
    releases = []
    commands = []
    odometry = []
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic == "/origincar_control/started":
            if deserialize_message(data, Bool).data:
                starts.append(timestamp_ns)
        elif topic == "/origincar_control/control_released":
            if deserialize_message(data, Bool).data:
                releases.append(timestamp_ns)
        elif topic == "/cmd_vel":
            message = deserialize_message(data, Twist)
            commands.append((timestamp_ns, message.linear.x, message.angular.z))
        elif topic == "/lio_hybird/odom":
            message = deserialize_message(data, Odometry)
            pose = message.pose.pose
            twist = message.twist.twist
            odometry.append((
                timestamp_ns,
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
                twist.linear.x,
                twist.angular.z,
            ))

    if not starts or not releases:
        raise RuntimeError("bag missing first-stage start or control-release markers")
    start_ns = starts[-1]
    release_ns = next((stamp for stamp in releases if stamp >= start_ns), None)
    if release_ns is None:
        raise RuntimeError("bag has no control-release marker after first-stage start")
    return start_ns, release_ns, commands, odometry


def load_terminal_log(log_path):
    events = []
    pid_rows = []
    for line_number, line in enumerate(log_path.read_text(encoding="utf-8").splitlines(), 1):
        match = LOG_PATTERN.match(line)
        if not match:
            continue
        timestamp_ns = int(Decimal(match["time"]) * Decimal(1_000_000_000))
        event = (
            timestamp_ns,
            match["level"],
            match["logger"],
            match["message"],
            line_number,
        )
        events.append(event)
        if match["logger"] != "PID":
            continue
        pid_match = PID_PATTERN.search(match["message"])
        if pid_match:
            pid_rows.append((
                timestamp_ns,
                float(pid_match["error"]),
                float(pid_match["integral"]),
                float(pid_match["derivative"]),
                float(pid_match["output"]),
            ))
    return events, pid_rows


def nearest_sample(samples, timestamp_ns):
    if not samples:
        return None
    index = bisect_right(samples, timestamp_ns, key=lambda item: item[0]) - 1
    if index < 0:
        return samples[0]
    if index + 1 >= len(samples):
        return samples[index]
    previous = samples[index]
    following = samples[index + 1]
    return previous if timestamp_ns - previous[0] <= following[0] - timestamp_ns else following


def write_csv(output_directory, start_ns, release_ns, commands, odometry, events, pid_rows):
    output_directory.mkdir(parents=True, exist_ok=True)
    window_events = [event for event in events if start_ns <= event[0] <= release_ns]
    window_pid = [row for row in pid_rows if start_ns <= row[0] <= release_ns]

    with (output_directory / "origincar_control_events.csv").open(
            "w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t_ros_ns", "t_from_started_s", "level", "logger", "message", "source_line"])
        for stamp, level, logger, message, line_number in window_events:
            writer.writerow([stamp, f"{(stamp - start_ns) / 1e9:.9f}", level, logger, message, line_number])

    with (output_directory / "pid_aligned.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "t_ros_ns", "t_from_started_s", "error_px", "integral_px_s",
            "derivative_px_per_s", "pid_output", "cmd_t_ros_ns", "cmd_linear_x",
            "cmd_angular_z", "odom_t_ros_ns", "odom_x_m", "odom_y_m",
            "odom_yaw_rad", "odom_linear_x_mps", "odom_angular_z_radps",
        ])
        for stamp, error, integral, derivative, output in window_pid:
            command = nearest_sample(commands, stamp)
            pose = nearest_sample(odometry, stamp)
            writer.writerow([
                stamp, f"{(stamp - start_ns) / 1e9:.9f}", error, integral,
                derivative, output,
                "" if command is None else command[0],
                "" if command is None else command[1],
                "" if command is None else command[2],
                "" if pose is None else pose[0],
                "" if pose is None else pose[1],
                "" if pose is None else pose[2],
                "" if pose is None else pose[3],
                "" if pose is None else pose[4],
                "" if pose is None else pose[5],
            ])
    return len(window_events), len(window_pid)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_directory", type=Path)
    parser.add_argument("terminal_log", type=Path)
    parser.add_argument("output_directory", type=Path)
    args = parser.parse_args()

    start_ns, release_ns, commands, odometry = load_bag(args.bag_directory)
    events, pid_rows = load_terminal_log(args.terminal_log)
    event_count, pid_count = write_csv(
        args.output_directory, start_ns, release_ns, commands, odometry, events, pid_rows)
    print(
        f"window={(release_ns - start_ns) / 1e9:.3f}s "
        f"events={event_count} pid_rows={pid_count} output={args.output_directory}"
    )


if __name__ == "__main__":
    main()
