import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class RandomMapPublisher(Node):
    def __init__(self):
        super().__init__("random_map")

        qos_prof = rclpy.qos.QoSProfile(depth=1)
        qos_prof.durability = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.publisher_ = self.create_publisher(OccupancyGrid, "/map", qos_prof)

        self.width = 50
        self.height = 50
        self.resolution = 0.1
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = "map"
        self.map_msg.info.width = self.width
        self.map_msg.info.height = self.height
        self.map_msg.info.resolution = self.resolution
        self.map_msg.info.origin.position.x = self.origin_x
        self.map_msg.info.origin.position.y = self.origin_y

        data = np.random.choice([0, 100], size=(self.height, self.width), p=[0.8, 0.2])
        self.map_msg.data = data.flatten().tolist()


        self.timer = self.create_timer(0.5, self.publish_map)

    def publish_map(self):

        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.map_msg)
        self.get_logger().info("Map published")

def main(args=None):
    rclpy.init(args=args)
    node = RandomMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
