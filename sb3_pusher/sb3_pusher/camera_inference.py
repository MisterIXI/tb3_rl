import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Point
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import numpy as np
import tf2_ros
from stable_baselines3 import PPO
import math

class StableBaselines3Inference(Node):
    def __init__(self):
        super().__init__('stable_baselines3_inference')
        # Set up QoS profile for LiDAR
        
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

        self.model = PPO.load("/home/turtle1/turtlebot3_ws/cardinal_locator_1999920_steps.zip")

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # timer for inference tick
        self.create_timer(0.1, self.inference_tick)
        
        self.ball_dir_vec = (0,0)

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
            if angle_deg < 10 and angle_deg > -10:
                
                rad = math.radians((self.ball_angle * 60) - 90)
                
                cos_r = math.cos(rad)
                sin_r = math.sin(rad)
                
                def normalized(x, y):
                    magnitute = math.hypot(x, y)
                    if magnitute == 0:
                        return x, y
                    return x / magnitute, y / magnitute
                
                self.ball_dir_vec = normalized(cos_r, sin_r)
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
                
                        
        # self.get_logger().info(f"Ball Angle/Visible: ({self.ball_angle}|{self.ball_visible})")

    def inference_tick(self):
        # if self.ball_pos is None:
        #     self.get_logger().warning("No ball position received yet")
        try:
            tb_pos = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except:
            self.get_logger().error("Transform error...")
            return
        point_target = PointStamped()
        point_target.header.frame_id = "base_link"
        point_target.header.stamp = self.get_clock().now().to_msg()
        point_target.point = Point()
        point_target.point.x = 0.0
        point_target.point.y = 0.0
        point_target = do_transform_point(point_target, tb_pos)
        ball_pos_rel = do_transform_point(self.ball_pos, tb_pos)

 #^       self.get_logger().info(f"""ball vec: {str((
 #^           self.ball_dir_vec[0],
 #^           self.ball_dir_vec[1],
 #^           - ball_pos_rel.point.y / 3,
 #^           - ball_pos_rel.point.x / 3,))}""")
        
        obs = {
            "obs": [
                # target area pos
                - point_target.point.y / 3,
                - point_target.point.x / 3, 
                # ball information
                #self.ball_angle,
                self.ball_dir_vec[0],
                self.ball_dir_vec[1],
                - ball_pos_rel.point.y / 3,
                - ball_pos_rel.point.x / 3,
            ]
        }
        obs["obs"].extend(self.lidar_scan)
        # self.get_logger().info(f"Observation: ({obs['obs'][2]}|{obs['obs'][3]})")
        action, _states = self.model.predict(obs)
        self.get_logger().info(f"Action: {action}")
        twist = Twist()

        twist.linear.x = - action[0] * 0.20
        twist.angular.z = action[1] * 2.0

        self.twist_pub.publish(twist)
        
        self.ball_dir_vec = (0,0)
        
        pass    


def main(args=None):
    rclpy.init(args=args)
    inference = StableBaselines3Inference()
    rclpy.spin(inference)
    inference.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()