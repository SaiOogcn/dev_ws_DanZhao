"""ROS 2 adapter for real localization/laser data, with explicit activation.

No simulator topics, models, contacts or privileged obstacle positions are
used. Preview is the default; physical drive output is an explicit setting.
"""
from collections import deque
from dataclasses import replace
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

from .pose_source import field_body_pose
from .race_course import START, audit_course, build_course, path_yaw, wrap
from .race_navigation import RaceMission
from .race_tuning import tuning_for
from .obstacle_cloud import ObstacleCloud
from .sensor_pipeline import (
    ObservationMemory, inputs_reason, project_scan, steering_to_twist,
)


class FullRaceNavigation(Node):
    def __init__(self):
        super().__init__('zsh_navigation')
        defaults = {
            'direction': 'CW', 'profile': 'sprint', 'publish_drive': False,
            'hardware_calibrated': False, 'command_topic': '/cmd_vel',
            'pose_topic': '/lio_hybird/odom', 'pose_frame': 'lio_hybird_odom',
            'pose_child_frame': 'base_footprint', 'pose_tf_topic': '',
            'pose_tf_child': 'base', 'motion_topic': '/odom',
            'motion_child_frame': 'base_footprint',
            'scan_topic': '/scan', 'scan_frame': 'laser', 'field_frame': 'field',
            'field_offset': [0., 0., 0.], 'body_to_pose_sensor': [0., 0., 0.],
            'laser_in_body': [-.03, 0., 0.], 'scan_stamp_is_end': True,
            'scan_index_time_reversed': True, 'pose_max_age': .30,
            'motion_max_age': .30, 'scan_max_age': .40,
            'interpolation_max_gap': .20, 'scan_max_range': 3.,
            'observation_lifetime': 2., 'observation_maximum': 1200,
            'cone_section_radius': .030, 'cone_base_half_width': .11,
            'fallback_inflation': .14, 'maximum_speed': .30,
            'maximum_lateral_acceleration': .50, 'braking': .60,
            'steering_rate': 1.5, 'steering_acceleration': 5.,
            'forward_gain': .896, 'forward_damping': 20.90,
            'forward_frequency2': 176.84, 'reverse_gain': 1.180,
            'reverse_damping': 3.29, 'reverse_frequency2': 16.42,
            'speed_time_constant': .16, 'speed_acceleration': 1.6,
            'maximum_control_seconds': .15, 'maximum_duration': 300.,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        # Configuration is intentionally read once. Restart after editing it;
        # live changes must not silently disagree with a running prediction.
        self.p = {name: self.get_parameter(name).value for name in defaults}
        positive = ('pose_max_age', 'motion_max_age', 'scan_max_age',
                    'interpolation_max_gap', 'scan_max_range',
                    'cone_section_radius', 'cone_base_half_width',
                    'fallback_inflation', 'maximum_speed',
                    'maximum_lateral_acceleration', 'braking', 'steering_rate',
                    'steering_acceleration', 'forward_gain', 'forward_damping',
                    'forward_frequency2', 'reverse_gain', 'reverse_damping',
                    'reverse_frequency2', 'speed_time_constant',
                    'speed_acceleration', 'maximum_control_seconds',
                    'maximum_duration')
        if any(not math.isfinite(self.p[k]) or self.p[k] <= 0 for k in positive):
            raise ValueError('model, timing and limit parameters must be positive and finite')
        for name in ('field_offset', 'body_to_pose_sensor', 'laser_in_body'):
            if len(self.p[name]) != 3 or not all(math.isfinite(x) for x in self.p[name]):
                raise ValueError(name + ' requires a finite SE(2) triple')
        for name in ('pose_frame', 'scan_frame', 'field_frame', 'motion_child_frame'):
            if not self.p[name]:
                raise ValueError(name + ' must be explicit')
        share = get_package_share_directory('zsh_navigation')
        self.stages = build_course(share, self.p['direction'])
        report = audit_course(self.stages)
        if report['illegal_reference_poses'] or report['stage_gaps']:
            raise ValueError('course geometry audit failed')
        for stage in self.stages:
            stage.max_speed = min(stage.max_speed, self.p['maximum_speed'])
        base = tuning_for(self.p['profile'])
        model = {k: self.p[k] for k in (
            'braking', 'steering_rate', 'steering_acceleration',
            'forward_gain', 'forward_damping', 'forward_frequency2',
            'reverse_gain', 'reverse_damping', 'reverse_frequency2',
            'speed_time_constant', 'speed_acceleration')}
        self.tuning = replace(
            base, maximum_speed=self.p['maximum_speed'],
            ring_speed=min(base.ring_speed, self.p['maximum_speed']),
            channel_speed=min(base.channel_speed, self.p['maximum_speed']),
            lateral_acceleration=self.p['maximum_lateral_acceleration'],
            ring_lateral_acceleration=self.p['maximum_lateral_acceleration'], **model)
        self.pose = None
        self.v = self.omega = 0.
        self.stamps = dict(pose=None, motion=None, scan=None)
        self.max_ages = {key: self.p[key + '_max_age'] for key in self.stamps}
        self.history = deque(maxlen=2000)
        self.pending_scans = deque(maxlen=2)
        self.memory = ObservationMemory(self.p['observation_lifetime'],
                                        maximum=self.p['observation_maximum'])
        self.mission = None
        self.enabled = False
        self.failure = ''
        self.started = None
        self.last_clock = None
        self.last_mode = ''
        self.elapsed_control = 0.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.candidate_pub = self.create_publisher(Twist, '/zsh_navigation/candidate_cmd_vel', 1)
        self.drive_pub = (self.create_publisher(Twist, self.p['command_topic'], 1)
                          if self.p['publish_drive'] else None)
        self.status_pub = self.create_publisher(String, '/zsh_navigation/status', latched)
        self.reference_pub = self.create_publisher(Path, '/zsh_navigation/reference', latched)
        self.path_pub = self.create_publisher(Path, '/zsh_navigation/local_plan', 1)
        qos = qos_profile_sensor_data
        if self.p['pose_tf_topic']:
            self.create_subscription(TFMessage, self.p['pose_tf_topic'], self.on_tf, qos)
        else:
            self.create_subscription(Odometry, self.p['pose_topic'], self.on_pose, qos)
        self.create_subscription(Odometry, self.p['motion_topic'], self.on_motion, qos)
        self.create_subscription(LaserScan, self.p['scan_topic'], self.on_scan, qos)
        self.create_service(SetBool, '/zsh_navigation/enable', self.on_enable)
        self.create_timer(.05, self.tick)
        self.create_timer(1., self.publish_paths)
        self.get_logger().info('Ready, disabled; drive output=' + str(bool(self.drive_pub)))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def stamp(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def fail(self, reason):
        if not self.failure:
            self.failure = reason
            self.get_logger().error(reason)

    def accept_pose(self, header, position, orientation):
        q = orientation
        values = (position.x, position.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(x) for x in values):
            self.fail('INVALID_POSE')
            return
        if header.frame_id != self.p['pose_frame']:
            self.fail('LOCALIZATION_FRAME_MISMATCH')
            return
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > .05:
            self.fail('INVALID_QUATERNION')
            return
        stamp = self.stamp(header)
        if self.stamps['pose'] is not None and stamp <= self.stamps['pose']:
            return
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        pose = field_body_pose((position.x, position.y, yaw),
                               self.p['field_offset'], self.p['body_to_pose_sensor'])
        if self.pose is not None and math.dist(pose[:2], self.pose[:2]) > .50:
            self.fail('LOCALIZATION_JUMP')
            return
        self.pose, self.stamps['pose'] = pose, stamp
        self.history.append((stamp, pose))

    def on_pose(self, message):
        if message.child_frame_id != self.p['pose_child_frame']:
            self.fail('LOCALIZATION_CHILD_FRAME_MISMATCH')
            return
        self.accept_pose(message.header, message.pose.pose.position, message.pose.pose.orientation)

    def on_tf(self, message):
        for transform in message.transforms:
            if (transform.header.frame_id == self.p['pose_frame'] and
                    transform.child_frame_id == self.p['pose_tf_child']):
                self.accept_pose(transform.header, transform.transform.translation,
                                 transform.transform.rotation)

    def on_motion(self, message):
        if message.child_frame_id != self.p['motion_child_frame']:
            self.fail('MOTION_FRAME_MISMATCH')
            return
        v, omega = message.twist.twist.linear.x, message.twist.twist.angular.z
        if not math.isfinite(v) or not math.isfinite(omega):
            self.fail('INVALID_MOTION')
            return
        stamp = self.stamp(message.header)
        if self.stamps['motion'] is not None and stamp <= self.stamps['motion']:
            return
        self.v, self.omega, self.stamps['motion'] = v, omega, stamp

    def on_scan(self, message):
        if message.header.frame_id != self.p['scan_frame']:
            self.fail('LASER_FRAME_MISMATCH')
            return
        self.pending_scans.append(message)

    def consume_scans(self, now):
        for _ in range(len(self.pending_scans)):
            message = self.pending_scans.popleft()
            stamp = self.stamp(message.header)
            duration = max(0, len(message.ranges) - 1) * message.time_increment
            end = stamp if self.p['scan_stamp_is_end'] else stamp + duration
            if now - end > self.p['scan_max_age']:
                continue
            if self.stamps['scan'] is not None and end <= self.stamps['scan']:
                continue
            if end > now + .05:
                self.fail('SCAN_FROM_FUTURE')
                continue
            if end > now:
                self.pending_scans.append(message)
                continue
            points = project_scan(
                message.ranges, message.angle_min, message.angle_increment,
                stamp, message.time_increment, self.history,
                laser_in_body=self.p['laser_in_body'],
                stamp_is_end=self.p['scan_stamp_is_end'],
                reversed_time=self.p['scan_index_time_reversed'],
                range_min=message.range_min,
                range_max=min(message.range_max, self.p['scan_max_range']),
                max_gap=self.p['interpolation_max_gap'])
            if points is None:
                self.pending_scans.append(message)
                continue
            self.memory.update(points, end)
            self.stamps['scan'] = end

    def on_enable(self, request, response):
        if not request.data:
            if self.enabled and self.drive_pub is not None:
                self.drive_pub.publish(Twist())
            self.enabled = False
            response.success, response.message = True, 'DISABLED'
            return response
        reason = self.failure or inputs_reason(self.now(), self.stamps, self.max_ages)
        if self.enabled:
            response.success, response.message = True, 'ALREADY_ENABLED'
            return response
        if self.drive_pub is not None and not self.p['hardware_calibrated']:
            reason = reason or 'HARDWARE_CALIBRATION_REQUIRED'
        if self.pose is not None:
            if (math.dist(self.pose[:2], START[:2]) > .25 or
                    abs(wrap(self.pose[2] - START[2])) > .40):
                reason = reason or 'START_POSE_MISMATCH'
        if abs(self.v) > .025:
            reason = reason or 'VEHICLE_NOT_STOPPED'
        if reason:
            response.success, response.message = False, reason
            return response
        self.mission = RaceMission(self.stages, background_planning=True,
                                   profile=self.p['profile'], tuning=self.tuning)
        self.started, self.enabled = self.now(), True
        response.success = True
        response.message = 'DRIVE_ENABLED' if self.drive_pub else 'PREVIEW_ENABLED'
        return response

    def tick(self):
        now = self.now()
        if self.last_clock is not None and now < self.last_clock:
            self.fail('CLOCK_REVERSED')
            self.history.clear()
            self.pending_scans.clear()
        self.last_clock = now
        try:
            self.consume_scans(now)
        except (ValueError, ArithmeticError) as error:
            self.fail(str(error))
        reason = inputs_reason(now, self.stamps, self.max_ages)
        mode = self.failure or reason or 'READY'
        command = Twist()
        if self.enabled and not self.failure and not reason:
            begin = time.perf_counter()
            try:
                if now - self.started > self.p['maximum_duration']:
                    self.fail('MISSION_TIMEOUT')
                else:
                    cloud = ObstacleCloud(self.memory.current(now),
                                          inflation=self.p['fallback_inflation'],
                                          section_radius=self.p['cone_section_radius'],
                                          base_half_width=self.p['cone_base_half_width'])
                    control = self.mission.command(self.pose, self.v, cloud,
                                                   now - self.started, self.omega)
                    speed, omega = steering_to_twist(control.speed, control.steering)
                    if abs(speed) > self.p['maximum_speed'] + 1e-8 or abs(control.steering) > .520001:
                        raise ValueError('COMMAND_LIMIT_VIOLATION')
                    command.linear.x, command.angular.z = speed, omega
                    mode = control.mode
                    if self.mission.failure:
                        self.fail(self.mission.failure)
            except Exception as error:
                self.fail('CONTROL_ERROR: ' + str(error))
            self.elapsed_control = time.perf_counter() - begin
            if self.elapsed_control > self.p['maximum_control_seconds']:
                self.fail('CONTROL_DEADLINE_EXCEEDED')
            reason = inputs_reason(self.now(), self.stamps, self.max_ages)
        elif self.enabled and reason:
            # Do not silently resume the old mission after a sensor outage.
            self.fail(reason)
        if self.failure or reason:
            command = Twist()
            mode = self.failure or reason
        if not self.enabled and not self.failure and not reason:
            mode = 'DISABLED'
        self.candidate_pub.publish(command)
        # Disabled preview never contends with another controller's /cmd_vel.
        if self.enabled and self.drive_pub is not None:
            self.drive_pub.publish(command)
        status = {
            'mode': mode, 'enabled': self.enabled, 'drive_output': bool(self.drive_pub),
            'failure': self.failure, 'pose': self.pose, 'speed': self.v,
            'stage': self.mission.stage.name if self.mission else '',
            'finished': bool(self.mission and self.mission.finished and not self.failure),
            'command_speed': command.linear.x, 'command_yaw_rate': command.angular.z,
            'control_seconds': self.elapsed_control,
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        if mode != self.last_mode:
            self.get_logger().info(mode)
            self.last_mode = mode

    def path_message(self, points, direction=1):
        result = Path()
        result.header.frame_id = self.p['field_frame']
        result.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = result.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            yaw = path_yaw(points, i, direction)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            result.poses.append(pose)
        return result

    def publish_paths(self):
        if self.mission is not None:
            stage, points = self.mission.stage, self.mission.follower.points
        else:
            stage, points = self.stages[0], self.stages[0].points
        self.reference_pub.publish(self.path_message(stage.points, stage.direction))
        self.path_pub.publish(self.path_message(points, stage.direction))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FullRaceNavigation()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.enabled and node.drive_pub is not None and rclpy.ok():
                node.drive_pub.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        from .race_navigation import shutdown_planners
        shutdown_planners()
