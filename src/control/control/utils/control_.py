import rclpy
from rclpy.node import Node
from scipy.interpolate import splprep, splev
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
import time
import math
import numpy as np
from geometry_msgs.msg import Quaternion

def takeoff(node, target_altitude=1.0, timeout=60.0):
    # Wait for FCU connection
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.current_pose is not None:
            break
        node.get_logger().info("Waiting for FCU connection...")
        time.sleep(1)

    node.zero_alt = node.current_pose.pose.position.z
    node.get_logger().info(f"Zero altitude set to: {node.zero_alt}")

    # Send initial setpoints before OFFBOARD mode (PX4 requirement)
    takeoff_pose = PoseStamped()
    takeoff_pose.pose.position.x = node.current_pose.pose.position.x
    takeoff_pose.pose.position.y = node.current_pose.pose.position.y
    takeoff_pose.pose.position.z = node.zero_alt + target_altitude
    takeoff_pose.pose.orientation = node.current_pose.pose.orientation
    
    node.get_logger().info("Sending initial setpoints")
    for _ in range(100):
        node.local_pos_pub.publish(takeoff_pose)
        rclpy.spin_once(node)
        time.sleep(0.01)

    # Set to OFFBOARD mode (PX4)
    node.get_logger().info("Setting mode to OFFBOARD...")
    while rclpy.ok():
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        future = node.setmode_client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        if future.result().mode_sent:
            node.get_logger().info("Set mode to OFFBOARD")
            break
        else:
            node.get_logger().error("Failed to set OFFBOARD mode, retrying...")
            time.sleep(1)

    time.sleep(0.5)

    # Arm the drone
    node.get_logger().info("Arming vehicle...")
    while rclpy.ok():
        req = CommandBool.Request()
        req.value = True
        future = node.arming_client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        if future.result().success:
            node.get_logger().info("Vehicle armed")
            break
        else:
            node.get_logger().error("Failed to arm vehicle, retrying...")
            time.sleep(1)

    time.sleep(0.5)

    # Wait to reach altitude (continue publishing setpoint)
    node.get_logger().info(f"Waiting to reach altitude {target_altitude}m (timeout: {timeout}s)...")
    start_time = time.time()
    reached = False

    while rclpy.ok():
        current_time = time.time()
        elapsed = current_time - start_time
        
        # Continue publishing setpoint for PX4 OFFBOARD
        node.local_pos_pub.publish(takeoff_pose)
        
        current_alt = node.current_pose.pose.position.z - node.zero_alt
        alt_error = abs(target_altitude - current_alt)
        
        node.get_logger().info(f"Current altitude: {current_alt:.2f}m / {target_altitude}m (error: {alt_error:.2f}m)")

        if alt_error <= 0.25:
            reached = True
            node.get_logger().info("Reached takeoff altitude!")
            break

        if elapsed > timeout:
            node.get_logger().warn(f"Timeout waiting to reach takeoff altitude after {timeout}s")
            break

        rclpy.spin_once(node)
        time.sleep(0.1)

    if not reached:
        node.get_logger().warn("Did not reach takeoff altitude within timeout")
        return False

    # Hold position for stability after reaching altitude
    node.get_logger().info("HOLD POSITION")
    hold_start = time.time()
    while rclpy.ok() and (time.time() - hold_start) < 2.0:
        node.local_pos_pub.publish(takeoff_pose)
        rclpy.spin_once(node)
        time.sleep(0.1)

    node.get_logger().info(f"Takeoff to {target_altitude} meters complete.")
    return True

def move_local(node, x, y, z, tolerance=0.5, timeout=30.0):
    """
    Move drone to target position in local frame.
    
    Args:
        node: ROS2 node with local_pos_pub and current_pose
        x: Target X position (meters)
        y: Target Y position (meters)
        z: Target Z position (meters, absolute altitude from ground)
        tolerance: Position tolerance to consider target reached (meters)
        timeout: Maximum time to wait for reaching target (seconds)
    
    Returns:
        True if target reached, False if timeout
    """
    if node.current_pose is None:
        node.get_logger().error("Current pose not available")
        return False
    
    node.get_logger().info(f"[MOVE_LOCAL] Target: [{x:.2f}, {y:.2f}, {z:.2f}]")
    
    # Create target pose
    target_pose = PoseStamped()
    target_pose.pose.position.x = x
    target_pose.pose.position.y = y
    target_pose.pose.position.z = z
    target_pose.pose.orientation = node.current_pose.pose.orientation
    
    # Publish setpoint and wait for arrival
    start_time = time.time()
    
    while rclpy.ok():
        elapsed = time.time() - start_time
        
        # Get current position
        current_x = node.current_pose.pose.position.x
        current_y = node.current_pose.pose.position.y
        current_z = node.current_pose.pose.position.z
        
        # Calculate error
        error = math.sqrt((x - current_x)**2 + (y - current_y)**2 + (z - current_z)**2)
        
        node.get_logger().info(f"[MOVE_LOCAL] Current: [{current_x:.2f}, {current_y:.2f}, {current_z:.2f}] Error: {error:.2f}m")
        
        if error < tolerance:
            node.get_logger().info(f"[MOVE_LOCAL] Target reached! (error: {error:.2f}m)")
            return True
        
        if elapsed > timeout:
            node.get_logger().warn(f"[MOVE_LOCAL] Timeout! Did not reach target after {timeout}s (error: {error:.2f}m)")
            return False
        
        # Publish setpoint
        node.local_pos_pub.publish(target_pose)
        rclpy.spin_once(node)
        time.sleep(0.1)  # 10Hz update rate

def land(node, descend_timeout=30.0):
    """
    Land the drone using AUTO.LAND mode.
    Works with both PX4 (AUTO.LAND) and ArduPilot (AUTO.LAND).
    """
    node.get_logger().info("Initiating landing sequence...")
    
    while rclpy.ok():
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'       
        future = node.setmode_client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        if future.result().mode_sent:
            node.get_logger().info("AUTO.LAND enabled")
            break
        else:
            node.get_logger().info("Failed to set AUTO.LAND mode, retrying...")
            time.sleep(1)

    return 0

def hold_position(node, hold_duration=5.0, target_pose=None):
    """
    Hold drone at current position for specified duration.
    Continuously publishes position setpoint to keep drone stable.
    
    Args:
        node: ROS2 node
        hold_duration: Duration to hold position in seconds (default: 5.0)
        target_pose: Optional specific pose to hold (if None, uses current pose)
    
    Returns:
        True if completed successfully
    """
    node.get_logger().info(f"Holding position for {hold_duration}s...")
    
    # If no target pose provided, use current pose
    if target_pose is None:
        if node.current_pose is None:
            node.get_logger().error("Current pose is None!")
            return False
        
        target_pose = PoseStamped()
        target_pose.pose.position.x = node.current_pose.pose.position.x
        target_pose.pose.position.y = node.current_pose.pose.position.y
        target_pose.pose.position.z = node.current_pose.pose.position.z
        target_pose.pose.orientation = node.current_pose.pose.orientation
    
    start_time = time.time()
    last_log_time = start_time
    
    while rclpy.ok():
        current_time = time.time()
        elapsed = current_time - start_time
        
        # Publish hold position setpoint
        target_pose.header.stamp = node.get_clock().now().to_msg()
        node.local_pos_pub.publish(target_pose)
        
        # Log every 1 second
        if current_time - last_log_time >= 1.0:
            remaining = hold_duration - elapsed
            node.get_logger().info(f"Holding... {remaining:.1f}s remaining")
            last_log_time = current_time
        
        # Check if hold duration completed
        if elapsed >= hold_duration:
            node.get_logger().info(f"Hold complete({elapsed:.2f}s held)")
            return True
        
        rclpy.spin_once(node)
        time.sleep(0.05)  # 20Hz update rate for smooth holding
    
    return False


def pub_command(node, command_text, topic="/mission"):
    from std_msgs.msg import String

    if not hasattr(node, '_command_pub'):
        node._command_pub = node.create_publisher(String, topic, 10)
    
    msg = String()
    msg.data = command_text
    node._command_pub.publish(msg)
    node.get_logger().info(f"Published: '{command_text}' to topic {topic}")


def wait_command(node, expected_command="GO", topic="/mission", timeout=60.0, hold_position=True):
    from std_msgs.msg import String
    
    received_command = {'data': None}
    hold_pose = None
    
    def command_callback(msg):
        received_command['data'] = msg.data
    
    # Create subscription for mission topic (shared by all UAVs)
    sub = node.create_subscription(String, topic, command_callback, 10)
    
    node.get_logger().info(f"Waiting for '{expected_command}' on {topic} (timeout: {timeout}s)...")
    
    # Capture current pose for holding position
    if hold_position and node.current_pose is not None:
        hold_pose = PoseStamped()
        hold_pose.pose.position.x = node.current_pose.pose.position.x
        hold_pose.pose.position.y = node.current_pose.pose.position.y
        hold_pose.pose.position.z = node.current_pose.pose.position.z
        hold_pose.pose.orientation = node.current_pose.pose.orientation
        node.get_logger().info(f"Holding position at ({hold_pose.pose.position.x:.2f}, {hold_pose.pose.position.y:.2f}, {hold_pose.pose.position.z:.2f})")
    
    start_time = time.time()
    
    while rclpy.ok():
        current_time = time.time()
        elapsed = current_time - start_time
        
        # Publish hold position setpoint to keep drone stable
        if hold_position and node.current_pose is not None:
            if hold_pose is None:
                hold_pose = PoseStamped()
                hold_pose.pose.position.x = node.current_pose.pose.position.x
                hold_pose.pose.position.y = node.current_pose.pose.position.y
                hold_pose.pose.position.z = node.current_pose.pose.position.z
                hold_pose.pose.orientation = node.current_pose.pose.orientation
            hold_pose.header.stamp = node.get_clock().now().to_msg()
            node.local_pos_pub.publish(hold_pose)
        
        # Check if command received
        if received_command['data'] is not None:
            recv_cmd = received_command['data'].strip().upper()
            exp_cmd = expected_command.strip().upper()
            
            if recv_cmd == exp_cmd:
                node.get_logger().info(f"Received command: {recv_cmd}")
                node.destroy_subscription(sub)
                return True
            else:
                node.get_logger().warn(f"Received unexpected command: {recv_cmd} (expected: {exp_cmd})")
                received_command['data'] = None  # Reset and wait for correct command
        
        # Check timeout
        if elapsed > timeout:
            node.get_logger().warn(f"Timeout waiting for '{expected_command}' after {timeout}s")
            node.destroy_subscription(sub)
            return False
        
        rclpy.spin_once(node)
        time.sleep(0.1)
    
    node.destroy_subscription(sub)
    return False


def get_formation_rotation_target(node, drone_id, angle_deg=90.0, inter_drone_distance=3.0):
    """
    Compute rotated target for a drone in 3-drone inline formation using drone 1 as pivot.

    Args:
        node: Node containing uav1_pose and current_pose.
        drone_id: Drone id (1, 2, or 3).
        angle_deg: Positive angle rotates left (CCW) in local XY frame.
        inter_drone_distance: Desired spacing between neighboring drones in local coordinates.

    Returns:
        (x, y, z) rotated target tuple, or None if required pose data is unavailable.
    """
    if drone_id not in (1, 2, 3):
        raise ValueError(f"drone_id must be 1, 2, or 3, got {drone_id}")

    if node.uav1_pose is None:
        node.get_logger().warn("Pivot pose (uav1) not available, cannot compute rotation target yet")
        return None

    pivot = np.array([
        node.uav1_pose.pose.position.x,
        node.uav1_pose.pose.position.y
    ], dtype=float)

    # Deterministic inline offsets by drone id (drone1 pivot, drone2 +d, drone3 +2d along +X).
    # This avoids noisy/non-synchronized live geometry and enforces requested spacing.
    base_offsets = {
        1: np.array([0.0, 0.0], dtype=float),
        2: np.array([inter_drone_distance, 0.0], dtype=float),
        3: np.array([2.0 * inter_drone_distance, 0.0], dtype=float),
    }

    theta = math.radians(angle_deg)
    rotation_matrix = np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)]
    ], dtype=float)

    rel_vec = base_offsets[drone_id]
    rotated_rel = rotation_matrix @ rel_vec
    rotated_xy = pivot + rotated_rel

    # Keep current drone altitude unchanged during horizontal formation rotation.
    if node.current_pose is None:
        node.get_logger().warn("Current pose not available for altitude reference")
        return None

    z_target = node.current_pose.pose.position.z

    node.get_logger().info(
        "[ROTATE] angle=%.1f deg, pivot=(%.2f, %.2f), spacing=%.2f, drone=%d, "
        "offset_before=(%.2f, %.2f), offset_after=(%.2f, %.2f)" % (
            angle_deg, pivot[0], pivot[1], inter_drone_distance, drone_id,
            rel_vec[0], rel_vec[1], rotated_rel[0], rotated_rel[1]
        )
    )

    return float(rotated_xy[0]), float(rotated_xy[1]), float(z_target)


def left_matrix_rotate(node, drone_id, angle_deg=90.0, tolerance=0.5, timeout=30.0, inter_drone_distance=3.0):
    """
    Rotate 3-drone formation by angle_deg around drone 1 pivot axis.

    Drone indexing convention:
      - drone 1 : pivot (stays in place in XY)
      - drone 2 : rotated around drone 1
      - drone 3 : rotated around drone 1
    """
    target = get_formation_rotation_target(
        node=node,
        drone_id=drone_id,
        angle_deg=angle_deg,
        inter_drone_distance=inter_drone_distance
    )
    if target is None:
        return False

    tx, ty, tz = target
    node.get_logger().info(
        f"[ROTATE] angle={angle_deg:.1f} deg, Drone {drone_id} target -> x: {tx:.2f}, y: {ty:.2f}, z: {tz:.2f}"
    )

    return move_local(node, tx, ty, tz, tolerance=tolerance, timeout=timeout)