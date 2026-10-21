import rclpy
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node


class FollowPathClient(Node):
    """FollowPath 教学骨架。

    保留 ROS 2 action 的连接、反馈和结果接口；发送 goal 的核心策略已挖空。
    即使 launch 参数 send_goal:=true，student_ready 默认为 false，不会接管车。
    """

    def __init__(self):
        super().__init__('demo2_follow_path_client')
        self.declare_parameter('path_topic', '/demo2/reference_path')
        self.declare_parameter('action_name', '/follow_path')
        self.declare_parameter('controller_id', 'TEB')
        self.declare_parameter('goal_checker_id', '')
        self.declare_parameter('send_goal', True)
        self.declare_parameter('student_ready', False)
        self.path = None
        self.sent = False
        self.path_sub = self.create_subscription(
            Path, self.get_parameter('path_topic').value, self.on_path, 1)
        self.client = ActionClient(
            self, FollowPath, self.get_parameter('action_name').value)

    def on_path(self, msg):
        if not msg.poses:
            self.get_logger().warn('收到空路径，等待学员检查 map/odom/frame_id')
            return
        self.path = msg
        if (self.get_parameter('send_goal').value and
                self.get_parameter('student_ready').value and not self.sent):
            self.send_goal()

    def send_goal(self):
        # TODO(学员): 完成 action server 等待、路径坐标系检查、起点对齐、
        # controller_id/goal_checker_id 选择、取消与超时策略，然后删除
        # student_ready 安全门并在低速测试中验证 /cmd_vel 的唯一发布者。
        self.get_logger().error(
            'FollowPath 核心发送逻辑未完成；请先实现 send_goal() 和安全检查')
        return

    def on_goal_response(self, future):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('FollowPath goal was rejected')
            self.sent = False
            return
        self.get_logger().info('FollowPath goal accepted')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        # TODO(学员): 记录 distance_remaining、速度和重规划状态，
        # 设计“卡住/偏离/障碍物”时的取消与恢复。
        # 注意：这个回调只在 send_goal() 里用 SendGoalOptions.feedback_callback
        # 注册之后才会被调用，骨架没有替你注册。
        feedback = feedback_msg.feedback
        self.get_logger().debug('distance remaining: %.3f', feedback.distance_to_goal)

    def on_result(self, future):
        # TODO(学员): 根据 result.status 处理成功、取消、失败和重试。
        self.get_logger().info('FollowPath finished with status %s', future.result().status)


def main(args=None):
    rclpy.init(args=args)
    node = FollowPathClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
