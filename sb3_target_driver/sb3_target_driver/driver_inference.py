import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import numpy as np
import tf2_ros
from stable_baselines3 import PPO
import math
import time
import argparse

class SB3TargetDriverInference(Node):
    FACTOR = 2.4
    def __init__(self, suppress_driving:bool = False):
        super().__init__('sb3_driver_inference')
        # Set up QoS profile for LiDAR
        self.get_logger().info("Setting up...")
        self.qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # subscribe to 
        self.lidar_subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            self.qos_profile,
        )

        self.clicked_point_subscription = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_point_callback,
            self.qos_profile
        )

        self.reset_target_subscription = self.create_subscription(
            Empty,
            "/reset_target",
            self.reset_target_callback,
            1
        )
        self.target_point = None
        self.start_time = 0.0
        self.target_pub = self.create_publisher(Marker, "/target_pos", 1)

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.model = PPO.load("/home/turtle1/turtlebot3_ws/mt_driver_fixed_999840_steps.zip")

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.twist_vel = 0.0
        self.twist_ang = 0.0
        # timer for inference tick
        self.create_timer(0.26, self.inference_tick)
        self.create_timer(0.1, self.twist_pub_loop)
        
        self.ball_dir_vec = (0,0)
        self.get_logger().info("Setup finished...")
        self.suppress_driving: bool = suppress_driving
        if suppress_driving:
            self.get_logger().warn("Driving supressed! Drop the -sd or --suppress_driving flag to resume driving!")
        self.reset_target_marker()

    def lidar_callback(self, msg):
        # Suppose msg.ranges is your array of float values.
        lidar_val_count = len(msg.ranges)
        step = int(lidar_val_count / 8)  # approximate spacing
        OFFSET = 10
        # Initialize the list to store 8 chosen values.
        lidar_scan = []

        for i in range(8):
            # Calculate the index to sample from.
            index = i * step
            value = msg.ranges[index]
            
            # If the value is nan, attempt to find a valid neighbor.
            if math.isnan(value):
                found_valid = False
                # Try neighbours by increasing offset until a valid index is found.
                # You can decide on a maximum offset limit. Here, we try up to 'step' values.
                for offset in range(1, step):
                    left_index = index - offset
                    right_index = index + offset
                    
                    # Check left neighbour if within bounds and valid.
                    if left_index >= 0 and not math.isnan(msg.ranges[left_index]):
                        value = msg.ranges[left_index]
                        found_valid = True
                        break
                    # Check right neighbour if within bounds and valid.
                    if right_index < lidar_val_count and not math.isnan(msg.ranges[right_index]):
                        value = msg.ranges[right_index]
                        found_valid = True
                        break
                
                # If no valid neighbour was found, set the value to 0.
                if not found_valid:
                    value = 0.0

            # Append the (possibly corrected) value to our result list.
            lidar_scan.append(value)

        # Save the result
        self.lidar_scan = lidar_scan
        # self.get_logger().info(f"Lidar callback calculated {self.lidar_scan}")

    def clicked_point_callback(self, msg):
        self.target_point = msg
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.pose.position.x = msg.point.x
        marker.pose.position.y = msg.point.y
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        self.target_pub.publish(marker)
        self.get_logger().info(f"New target set at: ({msg.point})")
        self.start_time = time.time()

    def reset_target_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 0
        marker.type = Marker.DELETE
        self.target_pub.publish(marker)

    def calc_vec_dist(self, point):
        return math.sqrt(point.x * point.x + point.y * point.y)

    def pub_stop_twist(self):
        self.twist_vel = 0.0
        self.twist_ang = 0.0

    def twist_pub_loop(self):
        twist = Twist()
        twist.linear.x = self.twist_vel
        twist.angular.z = self.twist_ang
        if not self.suppress_driving:
            self.twist_pub.publish(twist)

    def reset_target_callback(self, msg):
        self.get_logger().warn("Reset target signal receieved! Resetting...")
        self.target_point = None
        self.reset_target_marker()

    def inference_tick(self):
        if self.target_point == None:
            self.pub_stop_twist()
            return
        try:
            tf_to_local = self.tf_buffer.lookup_transform("base_link", "map", rclpy.time.Time())
        except:
            self.get_logger().error("Transform error...")
            return
        # check if target reached
        target_local = do_transform_point(self.target_point, tf_to_local)
        if self.calc_vec_dist(target_local.point) < 0.25:
            # point reached
            time_taken = time.time() - self.start_time
            self.get_logger().info(f"Target point reached in {time_taken:.2f}s! Waiting for further input...")
            self.target_point = None
            self.reset_target_marker()
            self.pub_stop_twist()
        obs = {
            "obs": [
                # target area pos
                - target_local.point.y / self.FACTOR,
                - target_local.point.x / self.FACTOR, 
            ]
        }
        obs["obs"].extend(self.lidar_scan)
        # self.get_logger().info(f"Observation: ({obs['obs'][2]}|{obs['obs'][3]})")
        action, _states = self.model.predict(obs)
        # swap action places
        tmp = action[0]
        action[0] = action[1]
        action[1] = tmp
        obsv = obs["obs"]
        self.get_logger().info(f"\nTarget: ({target_local.point.x:.4f}|{target_local.point.y:.4f}) \nObs: ({obsv[0]:.4f}|{obsv[1]:.4f}) \nAction: ({action[0]:.4f}|{action[1]:.4f})")
        # self.get_logger().info(f"Action: {action}")
        # TODO: implement twist lerp
        self.twist_vel = np.clip(action[0] * 0.20, -0.10, 0.20)
        self.twist_ang = action[1] * 2.0


def main(args=None):
    rclpy.init(args=args)
    # Argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument('-sd', '--suppress_driving', action='store_true', help='Suppress driving functionality')
    parsed_args, unknown = parser.parse_known_args()
    inference = SB3TargetDriverInference(suppress_driving=parsed_args.suppress_driving)
    rclpy.spin(inference)
    inference.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()