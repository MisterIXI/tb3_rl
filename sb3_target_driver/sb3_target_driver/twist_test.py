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

## Findings:
# ideal twist vel change per second seems to be 0.5/s
# ideal twist ang change per second seems to be 2.5/s
class TwistTestNode(Node):
    VEL_MAX = 0.20
    ANG_MAX = 2.8
    def __init__(self):
        super().__init__("twist_test")
        twist_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', twist_qos)
        self.create_timer(0.1, self.twist_callback)
        self.switch = False
        self.target_vel = 0.0
        self.target_ang = 0.0
        self.twist_vel = 0.0
        self.twist_ang = 0.0
        self.last_time = time.time()
        pass

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
            
    def twist_callback(self):
        msg = Twist()
        diff = time.time() - self.last_time
        if diff > 3.0:
            self.last_time = time.time()
            self.switch = not self.switch
        
        # if self.switch:
        #     if diff < 1.0:
        #         msg.linear.x = self.VEL_MAX * diff
        #         msg.angular.z = self.ANG_MAX * diff
        #     else:
        #         msg.linear.x = self.VEL_MAX
        #         msg.angular.z = self.ANG_MAX
        CHANGE = 0.05
        if self.switch:
            self.twist_vel = self.move_toward(self.twist_vel, self.VEL_MAX, CHANGE)
            msg.linear.x = self.twist_vel
            self.twist_ang = self.move_toward(self.twist_ang, self.ANG_MAX, CHANGE * 5)
            msg.angular.z = self.twist_ang
        else:
            self.twist_vel = self.move_toward(self.twist_vel, self.VEL_MAX, CHANGE)
            msg.linear.x = self.twist_vel
            self.twist_ang = self.move_toward(self.twist_ang, -self.ANG_MAX, CHANGE * 5)
            msg.angular.z = self.twist_ang
            

        self.twist_pub.publish(msg)
        pass

def main(args=None):
    rclpy.init(args=args)
    # Argument parsing
    inference = TwistTestNode()
    rclpy.spin(inference)
    inference.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()