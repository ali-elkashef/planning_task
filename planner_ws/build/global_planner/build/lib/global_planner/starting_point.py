import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion

class StartingPointPublisher(Node):
    def __init__(self):
        super().__init__("starting_point")

        self.publisher_ = self.create_publisher(
            Odometry,
            "/sim_ground_truth_pose",
            10
        )

        self.timer = self.create_timer(1.0, self.publish_start)

        self.start_x = 1.0
        self.start_y = 1.0

    def publish_start(self):
        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"

        msg.pose.pose = Pose(
            position=Point(x=self.start_x, y=self.start_y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        self.publisher_.publish(msg)
        self.get_logger().info(f"Start pose published at ({self.start_x}, {self.start_y})")


def main(args=None):
    rclpy.init(args=args)
    node = StartingPointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()