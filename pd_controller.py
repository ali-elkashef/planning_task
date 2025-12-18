import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from math import atan2, sqrt, sin, cos
import tf_transformations
from PDcontroller.action import FollowPath

class PDController(LifecycleNode):

    def __init__(self):
        super().__init__('pd_controller')

        self.path = None
        self.current_pose = None
        self.current_goal_index = 0
        self.prev_angle_error = 0.0

        self.cmd_pub = None
        self.pose_sub = None
        self.path_sub = None
        self.action_server = None
        self.control_timer = None
        self.active_goal = None

        self.get_logger().info('controller has begun')

    def on_configure(self, state: LifecycleState):
        self.get_logger().info('IN configuration mode')
        self.pose_sub = self.create_subscription(PoseStamped, '/current_pose', self.pose_callback, 10)
        self.path_sub = self.create_subscription(Path, 'planned_path', self.path_callback, 10)
        self.cmd_pub = self.create_lifecycle_publisher(Twist, 'cmd_vel', 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info('IN activate mode')
        self.action_server = ActionServer(self, FollowPath, 'follow_path', self.execute_callback)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info('IN deactivate mode')
        self.control_timer.cancel()
        self.action_server.destroy()
        self.active_goal = None

        return super().on_deactivate(state)

    def path_callback(self, msg: Path):
        self.path = msg
        self.current_goal_index = 0
        self.get_logger().info('path is received')

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose

    def execute_callback(self, goal_handle):
        self.get_logger().info('goal is accepted')

        if self.path is None:
            self.get_logger().warn('no path is received')
            goal_handle.abort()
            return FollowPath.Result(success=False)

        self.active_goal = goal_handle
        return FollowPath.Result(success=True)

    def control_loop(self):
        if (self.active_goal is None
            or self.path is None
            or self.current_pose is None
            or self.current_goal_index >= len(self.path.poses)):
            return

        goal_pose = self.path.poses[self.current_goal_index].pose

        x = self.current_pose.position.x
        y = self.current_pose.position.y

        q = self.current_pose.orientation
        _, _, theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        dx = goal_pose.position.x - x
        dy = goal_pose.position.y - y

        distance_error = sqrt(dx**2 + dy**2)
        angle_to_goal = atan2(dy, dx)
        angle_error = angle_to_goal - theta
        angle_error = atan2(sin(angle_error), cos(angle_error))

       # feedback
        feedback = FollowPath.Feedback()
        feedback.distance_to_goal = distance_error
        self.active_goal.publish_feedback(feedback)

        # PD gains
        Kp_ang = 2.0
        Kd_ang = 0.5
        Kp_lin = 1.5

        angular = Kp_ang * angle_error + Kd_ang * (angle_error - self.prev_angle_error)
        linear = Kp_lin * distance_error
        self.prev_angle_error = angle_error

        cmd = Twist()
        cmd.linear.x = min(linear, 2.0)
        cmd.angular.z = max(min(angular, 4.0), -4.0)

        if self.cmd_pub.is_activated:
            self.cmd_pub.publish(cmd)

        if distance_error < 0.2:
            self.current_goal_index += 1

            if self.current_goal_index >= len(self.path.poses):
                self.stop_robot()
                self.active_goal.succeed()
                self.active_goal = None

    def stop_robot(self):
        cmd = Twist()
        if self.cmd_pub.is_activated:
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PDController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
