import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
import numpy as np
import heapq

class GlobalPlannerNode(Node):
    
    def __init__(self):
        super().__init__("global_planner")
        self.get_logger().info("Global Planner Node Initialized")

        qos_prof = QoSProfile(
            history = QoSHistoryPolicy.KEEP_LAST,
            depth= 10,
            durability= QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        qos = QoSProfile(depth = 10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.path_publisher = self.create_publisher(Path, "global_planner", 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, "/sim_ground_truth_pose", self.odom_callback, qos
        )
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, "/map" , self.map_callback, qos_prof
        )
        self.map_data= None
        self.map_width= None
        self.map_height= None
        self.map_resolution= None
        self.map_origin_x= None
        self.map_origin_y= None
        self.current_x= None
        self.current_y= None
        self.goal_x, self.goal_y= 4, 4

        self.orthogonal_step_cost = 1.0
        self.diagonal_step_cost = self.orthogonal_step_cost * 1.4142

    def map_callback(self,msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        self.get_logger().info("Map is received and stored ")

    def odom_callback(self,msg):
        if self.map_data is None:
            self.get_logger().warn("Map isn't received. Path computation is skipped")
            return

        x,y = msg.pose.pose.position.x , msg.pose.pose.position.y
        self.current_x , self.current_y = self.meter_to_grid(
            msg.pose.pose.position.x , msg.pose.pose.position.y
        )

        goal_x, goal_y = self.meter_to_grid(self.goal_x, self.goal_y)

        path = self.A_algorithm(self.current_x, self.current_y, goal_x, goal_y)
        if path:
            self.publish_path(path)
            self.get_logger().info("path published")
        else:
            self.get_logger().warn("No vaild path is found")
        
    def meter_to_grid(self, x_meters, y_meters):
        if self.map_origin_x is not None and self.map_resolution is not None:
            x_grid = int((x_meters - self.map_origin_x) // self.map_resolution)
            y_grid = int((y_meters - self.map_origin_y) // self.map_resolution)
            return x_grid , y_grid
        else:
            self.get_logger().warn("resolution and map origin aren't received yet")
            return
        
    def grid_to_meter(self, x_grid, y_grid):
            if self.map_origin_x is not None and self.map_resolution is not None:
                  x_meters = (self.map_resolution * x_grid) + self.map_origin_x
                  y_meters = (self.map_resolution * y_grid) + self.map_origin_y
                  return x_meters, y_meters
            else:
                self.get_logger().warn("resolution and map origin aren't received yet")
                return
        
    def h(self, x, y, goal_x, goal_y):

        return np.sqrt((x - goal_x) ** 2 + (y - goal_y) ** 2)
    
    def valid_nodes(self, current):
        neighbors = []
        x, y = current
        lethal_cost = 50
        moves = [
            (1,0,self.orthogonal_step_cost), (-1,0,self.orthogonal_step_cost),
            (0,1,self.orthogonal_step_cost), (0,-1,self.orthogonal_step_cost),
            (1,1,self.diagonal_step_cost), (1,-1,self.diagonal_step_cost),
            (-1,1,self.diagonal_step_cost), (-1,-1,self.diagonal_step_cost),
        ]

        for dx, dy, step_cost in moves:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                if self.map_data[ny, nx] < lethal_cost:
                    adjusted_cost = step_cost + self.map_data[ny, nx] / 255
                    neighbors.append(((nx, ny), adjusted_cost))
        return neighbors

    
    def A_algorithm(self, start_x, start_y, goal_x, goal_y):

        self.get_logger().info(f"starting from {start_x, start_y} to {goal_x,goal_y}")

        open_set = []
        heapq.heappush(open_set, (0,(start_x, start_y)))
        came_from  = {}
        g_score = {(start_x,start_y) : 0}
        f_score = {(start_x,start_y) : self.h(start_x,start_y, goal_x, goal_y)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == (goal_x,goal_y):
                return self.reconstruct_path(came_from, current)
            for neighbor, cost in self.valid_nodes(current):
                temp_g_score = g_score[current] + cost
                if neighbor not in g_score or temp_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g_score
                    x_new, y_new = neighbor
                    f_score[neighbor] = temp_g_score + self.h(
                        x_new, y_new, goal_x, goal_y
                    )
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
            
        return []
    
    def reconstruct_path(self, came_from,current):

        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]

        path.reverse()

        return path
    
    def smooth_path_maker(self, path, samples= 40):

        if not path or len(path) < 3:
            return path
        
        smoothed = []
        n = len(path)

        smoothed.append(path[0])

        for i in range (1, n-1):
            p0 = path[ i - 1]
            p1 = path[i]
            p2 = path[i+1]

            for t in np.linspace(0,1,samples):
                x = (1-t) ** 2 * p0[0] + 2 * (1-t) * t * p1[0] + t**2 * p2[0]
                y = (1-t) ** 2 * p0[1] + 2 * (1-t) * t * p1[1] + t**2 * p2[1]
                smoothed.append((x,y))
                
        smoothed.append(path[-1])

        return smoothed
    
    def publish_path(self, path):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        if not path:
            return
        smoothed_path = self.smooth_path_maker(path)

        path_meters = [self.grid_to_meter(x,y) for x,y in smoothed_path]
        if len(path) < 1:
            return
        
        pose = PoseStamped()
        pose.pose.position.x = float(path_meters[0][0])
        pose.pose.position.y = float(path_meters[0][1])
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)

        for i in range(1, len(path_meters)):
            pose = PoseStamped()
            pose.pose.position.x = float(path_meters[i][0])
            pose.pose.position.y = float(path_meters[i][1])

            dx = path_meters[i][0] - path_meters[i-1][0]
            dy = path_meters[i][1] - path_meters[i-1][1]
            theta = np.arctan2(dy,dx)

            cy = np.cos(theta * 0.5)
            sy = np.sin(theta * 0.5)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = sy
            pose.pose.orientation.w = cy

            msg.poses.append(pose)
        self.path_publisher.publish(msg)

def main(args= None):
    rclpy.init(args=args)
    planner = GlobalPlannerNode()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()