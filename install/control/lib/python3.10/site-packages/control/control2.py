import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from nav_msgs.msg import Path
import numpy as np
import time
from control.utils.control_ import (
    takeoff,
    move_local,
    land,
    pub_command,
    wait_command,
    hold_position,
    left_matrix_rotate
)

class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')
        
        self.zero_alt = 0.0
        self.current_velocity = None
        self.current_pose = None
        self.current_state = None
        
        # All drone poses (for formation operations)
        self.uav1_pose = None
        self.uav2_pose = None
        self.uav3_pose = None

        # ROS Publishers
        self.vel_pub = self.create_publisher(
            Twist, '/uav2/setpoint_velocity/cmd_vel_unstamped', 10)
        self.local_pos_pub = self.create_publisher(
            PoseStamped, '/uav2/setpoint_position/local', 10)
        self.smooth_path_pub = self.create_publisher(
            Path, '/smooth_path', 10)

        # ROS Subscribers
        self.vel_sub = self.create_subscription(
            TwistStamped, '/uav2/local_position/velocity_local', 
            self.vel_callback, rclpy.qos.qos_profile_sensor_data)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/uav2/local_position/pose', 
            self.pose_callback, rclpy.qos.qos_profile_sensor_data)
        self.status_sub = self.create_subscription(
            State, '/uav2/state', self.status_callback, 10)
        
        # Subscribe to all 3 drone positions (for formation rotation)
        self.uav1_sub = self.create_subscription(
            PoseStamped, '/uav1/local_position/pose',
            self.uav1_callback, rclpy.qos.qos_profile_sensor_data)
        self.uav2_sub = self.create_subscription(
            PoseStamped, '/uav2/local_position/pose',
            self.uav2_callback, rclpy.qos.qos_profile_sensor_data)
        self.uav3_sub = self.create_subscription(
            PoseStamped, '/uav3/local_position/pose',
            self.uav3_callback, rclpy.qos.qos_profile_sensor_data)

        # ROS Clients
        self.setmode_client = self.create_client(SetMode, '/uav2/set_mode')
        self.arming_client = self.create_client(CommandBool, '/uav2/cmd/arming')
        
        self.get_logger().info("Drone Control Node initialized")
    
    def vel_callback(self, msg):
        self.current_velocity = msg
    
    def pose_callback(self, msg):
        self.current_pose = msg

    def status_callback(self, msg):
        self.current_state = msg
    
    def uav1_callback(self, msg):
        self.uav1_pose = msg
    
    def uav2_callback(self, msg):
        self.uav2_pose = msg
    
    def uav3_callback(self, msg):
        self.uav3_pose = msg


def main():
    rclpy.init()
    node = DroneControlNode()
    
    # Takeoff
    node.get_logger().info("TAKEOFF")
    if not takeoff(node, target_altitude=3.3):
        node.get_logger().error("TAKEOFF FAILED!")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    hold_position(node, 5.0)
    left_matrix_rotate(node, drone_id=2, angle_deg=120.0, tolerance=0.3, timeout=20.0, inter_drone_distance=3.0)
    hold_position(node, 2.0)
    
    node.get_logger().info("LANDING")
    land(node)

    node.get_logger().info("MISSION COMPLETED")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
