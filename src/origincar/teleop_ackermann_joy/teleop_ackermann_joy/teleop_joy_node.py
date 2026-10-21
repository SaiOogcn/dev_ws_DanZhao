"""
Xbox gamepad teleop for Ackermann vehicle (小鸡启明星1).

Mapping:
  Left Stick X (axes[0]): steering, 1=left, -1=right
  Left Stick Y (axes[1]): direction, 1=forward, -1=backward
  RT (axes[4]): throttle, -1=full press, 1=released
  LT (axes[5]): (unused)
  D-pad X (axes[6]): 1=left, -1=right
  D-pad Y (axes[7]): 1=up, -1=down

  A=button0, B=button1, X=button3, Y=button4, LB=button6, RB=button7

Controls:
  Left Stick     -> direction + steering combined
  RT             -> throttle (0 to speed_limit m/s, default 1.2)
  RB             -> increase speed limit (+0.3 m/s)
  LB             -> decrease speed limit (-0.3 m/s)
  A              -> trigger take_photo node
  Y              -> toggle reverse mode
  D-pad Up/Down  -> fine-tune speed limit (+/-0.1 m/s)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JoyFeedback
from std_msgs.msg import Bool


class TeleopAckermannJoy(Node):
    def __init__(self) -> None:
        super().__init__('teleop_ackermann_joy')

        # --- Parameters ---
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_speed', 1.2)
        self.declare_parameter('max_angular', 4.0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('speed_step', 0.3)
        self.declare_parameter('fine_step', 0.1)
        self.declare_parameter('joy_timeout', 0.5)
        self.declare_parameter('take_photo_trigger_topic', '/take_photo/trigger')

        self._joy_topic = self._s('joy_topic')
        self._cmd_vel_topic = self._s('cmd_vel_topic')
        self._max_speed = self._d('max_speed')
        self._max_angular = self._d('max_angular')
        self._deadzone = self._d('deadzone')
        self._speed_step = self._d('speed_step')
        self._fine_step = self._d('fine_step')
        self._joy_timeout = self._d('joy_timeout')
        self._take_photo_trigger_topic = self._s('take_photo_trigger_topic')

        # --- State ---
        self._speed_limit = self._max_speed
        self._reverse = False
        self._last_joy_time = self.get_clock().now()
        self._prev_buttons: list[int] = []
        self._dpad_cd = 0.0

        # --- Pub / Sub / Safety timer ---
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._feedback_pub = self.create_publisher(JoyFeedback, '/joy/set_feedback', 10)
        self._trigger_pub = self.create_publisher(Bool, self._take_photo_trigger_topic, 10)
        self._joy_sub = self.create_subscription(Joy, self._joy_topic, self._on_joy, 10)
        self._timer = self.create_timer(0.1, self._safety_check)

        self.get_logger().info(
            f'Teleop ready | joy={self._joy_topic} cmd_vel={self._cmd_vel_topic} '
            f'max_speed={self._max_speed:.1f} m/s max_angular={self._max_angular:.1f} rad/s '
            f'take_photo_trigger={self._take_photo_trigger_topic}'
        )

    # ------------------------------------------------------------------
    def _s(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _d(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    # ------------------------------------------------------------------
    def _on_joy(self, msg: Joy) -> None:
        axes = list(msg.axes)
        buttons = [int(b) for b in msg.buttons]

        # --- Read axes (index-safe) ---
        def axis(i: int) -> float:
            return float(axes[i]) if i < len(axes) else 0.0

        steer_raw = axis(0)   # 1=left, -1=right
        dir_raw = axis(1)     # 1=up(forward), -1=down(backward)
        rt_raw = axis(4)      # -1=full press, 1=released

        # --- Button edge detection ---
        if self._prev_buttons:
            if self._edge(7, buttons):   # RB: speed up
                self._speed_limit = min(3.0, self._speed_limit + self._speed_step)
                self.get_logger().info(f'Speed limit: {self._speed_limit:.1f} m/s')
                self._pulse_rumble(0.4, 0.08)
            if self._edge(6, buttons):   # LB: speed down
                self._speed_limit = max(0.3, self._speed_limit - self._speed_step)
                self.get_logger().info(f'Speed limit: {self._speed_limit:.1f} m/s')
                self._pulse_rumble(0.4, 0.08)
            if self._edge(0, buttons):   # A: trigger take_photo
                trigger_msg = Bool()
                trigger_msg.data = True
                self._trigger_pub.publish(trigger_msg)
                self.get_logger().info('Take photo triggered')
                self._pulse_rumble(0.6, 0.15)
            if self._edge(4, buttons):   # Y: toggle reverse
                self._reverse = not self._reverse
                self.get_logger().info(f'Reverse: {"ON" if self._reverse else "OFF"}')
                self._pulse_rumble(0.3, 0.08)

        # --- D-pad fine adjust ---
        dpad_y = axis(7)
        if self._dpad_cd <= 0.0 and abs(dpad_y) > 0.5:
            delta = self._fine_step * (-1.0 if dpad_y > 0 else 1.0)
            self._speed_limit = max(0.3, min(3.0, self._speed_limit + delta))
            self._dpad_cd = 0.3
            self.get_logger().info(f'Speed limit: {self._speed_limit:.1f} m/s')
            self._pulse_rumble(0.2, 0.05)

        self._prev_buttons = buttons

        # --- Deadzone ---
        steer = self._dz(steer_raw, self._deadzone)
        direct = self._dz(dir_raw, self._deadzone)

        # --- RT throttle: -1(full) -> 1, 1(released) -> 0 ---
        throttle = (1.0 - rt_raw) / 2.0
        throttle = max(0.0, min(1.0, throttle))

        # --- Compute cmd_vel ---
        direction = -1.0 if self._reverse else 1.0
        linear = direct * throttle * self._speed_limit * direction
        angular = steer * self._max_angular

        self._publish(linear, angular)
        self._last_joy_time = self.get_clock().now()

    # ------------------------------------------------------------------
    def _edge(self, idx: int, buttons: list[int]) -> bool:
        if idx < 0 or idx >= len(buttons) or idx >= len(self._prev_buttons):
            return False
        return buttons[idx] == 1 and self._prev_buttons[idx] == 0

    def _dz(self, val: float, dz: float) -> float:
        if abs(val) < dz:
            return 0.0
        return (val - dz * (1.0 if val > 0 else -1.0)) / (1.0 - dz)

    # ------------------------------------------------------------------
    def _safety_check(self) -> None:
        dt = (self.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
        if dt > self._joy_timeout:
            self._publish(0.0, 0.0)
        if self._dpad_cd > 0.0:
            self._dpad_cd -= 0.1

    # ------------------------------------------------------------------
    # Rumble helpers
    # ------------------------------------------------------------------
    def _send_rumble(self, intensity: float) -> None:
        fb = JoyFeedback()
        fb.type = JoyFeedback.TYPE_RUMBLE
        fb.id = 0
        fb.intensity = max(0.0, min(1.0, intensity))
        self._feedback_pub.publish(fb)

    def _stop_rumble(self) -> None:
        self._send_rumble(0.0)

    def _pulse_rumble(self, intensity: float, duration: float) -> None:
        self._send_rumble(intensity)

        def _stop_and_cancel() -> None:
            self._stop_rumble()
            t.cancel()

        t = self.create_timer(duration, _stop_and_cancel)

    # ------------------------------------------------------------------
    def _publish(self, linear: float, angular: float) -> None:
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self._cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = TeleopAckermannJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
