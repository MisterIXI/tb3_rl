import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Point
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import numpy as np
import tf2_ros
from stable_baselines3 import PPO
import math
import time
import argparse

class StableBaselines3Inference(Node):
    FACTOR = 2.4
    TWIST_PUB_FREQ = 0.1
    TWIST_ANG_RATE = 2.5 * TWIST_PUB_FREQ
    TWIST_VEL_RATE = 0.5 * TWIST_PUB_FREQ
    def __init__(self, suppress_driving:bool = False):
        super().__init__('stable_baselines3_inference')
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
        # subscribe to ball_angle
        self.ball_angle_subscription = self.create_subscription(
            Float32,
            '/ball_angle',
            self.ball_angle_callback,
            10,
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
        # Subscription to the SonarSensorInfo topic
        self.sonar_subscription = self.create_subscription(
            Float32,
            'SonarSensorInfo',
            self.sonar_distance_callback,
            self.qos_profile
        )
        self.sonar_distance = None  # Initialize distance variable to None
        
        self.ball_angle = 0.0
        self.ball_visible = 0
        self.ball_pos = PointStamped()
        
        # debug publis ball pos
        self.ball_pos_pub = self.create_publisher(PointStamped, "/ball_pos", 1)

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.model = PPO.load("/home/turtle1/turtlebot3_ws/pusher_lerp_2499600_steps.zip")

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.twist_vel = 0.0
        self.vel_target = 0.0
        self.twist_ang = 0.0
        self.ang_target = 0.0
        # timer for inference tick
        self.create_timer(0.26, self.inference_tick)
        self.create_timer(self.TWIST_PUB_FREQ, self.twist_pub_loop)
        
        self.ball_dir_vec = (0,0)
        self.get_logger().info("Setup finished...")
        self.suppress_driving: bool = suppress_driving
        if suppress_driving:
            self.get_logger().warn("Driving supressed! Drop the -sd or --suppress_driving flag to resume driving!")
        self.reset_target_marker()

    def sonar_distance_callback(self, msg):
        self.sonar_distance = msg.data
        #self.get_logger().info(f"Distance callback calculated {self.sonar_distance}")


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
            # Adjust value to be like the godot values
            value = (4.215 - value) / 4.215
            # Append the (possibly corrected) value to our result list.
            lidar_scan.append(value)

        # Save the result
        self.lidar_scan = lidar_scan
        # self.get_logger().info(f"Lidar callback calculated {self.lidar_scan}")

    def ball_angle_callback(self, msg):
        self.ball_angle = msg.data
        if math.isnan(self.ball_angle):
            self.ball_angle = 0.0
            self.ball_visible = 0
        else: 
            self.ball_visible = 1
            angle_deg = self.ball_angle * 60
            rad = math.radians((self.ball_angle * 60) - 90)
            
            cos_r = math.cos(rad)
            sin_r = math.sin(rad)
            
            def normalized(x, y):
                magnitute = math.hypot(x, y)
                if magnitute == 0:
                    return x, y
                return x / magnitute, y / magnitute
            
            self.ball_dir_vec = normalized(cos_r, sin_r)
            if angle_deg < 10 and angle_deg > -10:
                
                try:
                    # map_tf = self.tf_buffer.lookup_transform("base_link", "map", rclpy.time.Time())
                    map_tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                except:
                    self.get_logger().error("Transform error...")
                    return
                dir_stamped = PointStamped()
                dir_stamped.header.frame_id = "base_link"
                dir_stamped.header.stamp = self.get_clock().now().to_msg()
                dir_stamped.point = Point()
                dir_stamped.point.x = self.sonar_distance + 0.09
                self.ball_pos = do_transform_point(dir_stamped, map_tf)
                self.ball_pos_pub.publish(self.ball_pos)

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
        self.vel_target = 0.0
        self.ang_target = 0.0
        self.twist_vel = 0.0
        self.twist_ang = 0.0

    def move_toward(self, src, tgt, step):
        if src < tgt:
            if step >= tgt - src:
                return tgt
            else:
                return src + step
        else:
            if step >= src - tgt:
                return tgt
            else:
                return src - step
            
    def twist_pub_loop(self):
        # move toward on vel and ang
        self.twist_vel = self.move_toward(self.twist_vel, self.vel_target, self.TWIST_VEL_RATE)
        self.twist_ang = self.move_toward(self.twist_ang, self.ang_target, self.TWIST_ANG_RATE)
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
        # if self.ball_pos is None:
        #     self.get_logger().warning("No ball position received yet")
        try:
            tb_pos = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except:
            self.get_logger().error("Transform error...")
            return
        point_target = self.target_point
        # point_target = PointStamped()
        # point_target.header.frame_id = "base_link"
        # point_target.header.stamp = self.get_clock().now().to_msg()
        # point_target.point = Point()
        # point_target.point.x = 0.0
        # point_target.point.y = 0.0
        point_target = do_transform_point(point_target, tf_to_local)
        ball_pos_rel = do_transform_point(self.ball_pos, tf_to_local)

 #^       self.get_logger().info(f"""ball vec: {str((
 #^           self.ball_dir_vec[0],
 #^           self.ball_dir_vec[1],
 #^           - ball_pos_rel.point.y / 3,
 #^           - ball_pos_rel.point.x / 3,))}""")
        
        obs = {
            "obs": [
                # target area pos
                - point_target.point.y / 2.4,
                - point_target.point.x / 2.4, 
                # ball information
                #self.ball_angle,
                self.ball_dir_vec[0],
                self.ball_dir_vec[1],
                # - ball_pos_rel.point.y / 2.4,
                # - ball_pos_rel.point.x / 2.4,
                0.0,
                0.0,
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
        self.get_logger().info(f"""
                               \nTarget: ({point_target.point.x:.4f}|{point_target.point.y:.4f}) 
                               \nObs: ({obsv[0]:.4f}|{obsv[1]:.4f}) 
                               \nBall Dir: ({obsv[2]:.4f}|{obsv[3]:.4f})
                               \nBall Pos: ({obsv[4]:.4f}|{obsv[5]:.4f})
                               \nAction: ({action[0]:.4f}|{action[1]:.4f})""")
        # mystr = "\n"
        # for val in self.lidar_scan:
        #     mystr += f"{val:.6f}\n"
        # self.get_logger().info(mystr)
        # self.get_logger().info(f"Action: {action}")
        self.vel_target = np.clip(action[0] * 0.20, -0.10, 0.20)
        self.ang_target = action[1] * 2.0
        
        self.ball_dir_vec = (0,0)
        
        pass    


def main(args=None):
    rclpy.init(args=args)
    # Argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument('-sd', '--suppress_driving', action='store_true', help='Suppress driving functionality')
    parsed_args, unknown = parser.parse_known_args()
    inference = StableBaselines3Inference(suppress_driving=parsed_args.suppress_driving)
    rclpy.spin(inference)
    inference.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()