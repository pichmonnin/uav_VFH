import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Point
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from collections import deque
from sensor_msgs.msg import PointCloud2, LaserScan
import math
import numpy as np 

# Global variables
current_state = State()
raw_pose = None
filtered_pose = PoseStamped()
current_pose = PoseStamped()
waypoints = [
    [10.0, 0.0, 7.0],
    [20.0, 0.0, 7.0], 
    [20.0, 20.0, 7.0], 
    [20.0, 0.0, 7.0]
]
current_waypoint_index = 0 
position_tolerance = 0.5
filter_window = 20
x_buffer = deque(maxlen=filter_window)
y_buffer = deque(maxlen=filter_window)
z_buffer = deque(maxlen=filter_window)

# Obstacle avoidance variables
avoid = False
avoidance_vector_x = 0.0
avoidance_vector_y = 0.0
d0 = 6.0  # Detection threshold (meters)
k = 2.0   # Increased gain for stronger avoidance
original_target = None
avoidance_timeout = 5.0  # Reduced timeout
last_avoidance_time = None
min_safe_distance = 1.0  # Minimum safe distance from obstacles

def state_callback(msg):
    global current_state 
    current_state = msg

def pose_callback(msg):
    global raw_pose, filtered_pose
    raw_pose = msg
    x_buffer.append(msg.pose.position.x)
    y_buffer.append(msg.pose.position.y)
    z_buffer.append(msg.pose.position.z)
    
    if len(x_buffer) >= filter_window:
        filtered_pose.pose.position.x = sum(x_buffer) / len(x_buffer)
        filtered_pose.pose.position.y = sum(y_buffer) / len(y_buffer)
        filtered_pose.pose.position.z = sum(z_buffer) / len(z_buffer)
        filtered_pose.header = msg.header

def publish_setpoint(node, pub, x, y, z):
    pose = PoseStamped()
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    # Keep current orientation
    if raw_pose:
        pose.pose.orientation = raw_pose.pose.orientation
    pub.publish(pose)
    node.get_logger().info(f"Publishing setpoint ({x:.2f}, {y:.2f}, {z:.2f})")

def arm_and_takeoff(node, arm_client, mode_client, pose_pub):
    # Send initial setpoints
    for _ in range(50):
        publish_setpoint(node, pose_pub, 0.0, 0.0, 7.0)
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # Arm the drone
    arm_cmd = CommandBool.Request()
    arm_cmd.value = True
    arm_future = arm_client.call_async(arm_cmd)
    rclpy.spin_until_future_complete(node, arm_future)
    
    if arm_future.result() and arm_future.result().success:
        node.get_logger().info("Armed successfully")
    else:
        node.get_logger().error("Failed to arm")
        return False
        
    # Set OFFBOARD mode
    mode_cmd = SetMode.Request()
    mode_cmd.custom_mode = "OFFBOARD"
    mode_future = mode_client.call_async(mode_cmd)
    rclpy.spin_until_future_complete(node, mode_future)
    
    if mode_future.result() and mode_future.result().mode_sent:
        node.get_logger().info("OFFBOARD mode set successfully")
        return True
    else:
        node.get_logger().error("Failed to set OFFBOARD mode")
        return False

def get_current_heading():
    if raw_pose is None:
        return 0.0
    
    q = raw_pose.pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return yaw

def lidar_callback(msg, node):
    global avoid, avoidance_vector_x, avoidance_vector_y, last_avoidance_time
    
    try:
        # Reset avoidance state
        avoid = False
        avoidance_vector_x = 0.0
        avoidance_vector_y = 0.0
        
        if not isinstance(msg, LaserScan):
            node.get_logger().error(f"Expected LaserScan, got {type(msg)}")
            return
            
        # Get current position for relative calculations
        if raw_pose is None:
            return
            
        current_x = raw_pose.pose.position.x
        current_y = raw_pose.pose.position.y
        current_heading = get_current_heading()
        
        obstacle_count = 0
        total_repulsion_x = 0.0
        total_repulsion_y = 0.0
        min_distance = float('inf')
        
        # Process laser scan data
        for i, distance in enumerate(msg.ranges):
            # Skip invalid readings
            if math.isinf(distance) or math.isnan(distance) or distance <= 0.1:
                continue
                
            # Calculate angle in world frame
            scan_angle = msg.angle_min + i * msg.angle_increment
            world_angle = current_heading + scan_angle
            
            # Convert to world coordinates
            obstacle_x = current_x + distance * math.cos(world_angle)
            obstacle_y = current_y + distance * math.sin(world_angle)
            
            min_distance = min(min_distance, distance)
            
            # Check if obstacle is within detection range
            if distance < d0 and distance > 0.3:  # Avoid very close noise
                avoid = True
                obstacle_count += 1
                
                # Calculate repulsive force using potential field
                # Force magnitude inversely proportional to distance squared
                force_magnitude = k * (1.0/distance - 1.0/d0) * (1.0/(distance**2))
                
                # Force direction: away from obstacle
                repulsion_x = -math.cos(world_angle) * force_magnitude
                repulsion_y = -math.sin(world_angle) * force_magnitude
                
                total_repulsion_x += repulsion_x
                total_repulsion_y += repulsion_y
                
                if obstacle_count <= 3:  # Log first few obstacles
                    node.get_logger().info(
                        f"Obstacle {obstacle_count}: dist={distance:.2f}m, "
                        f"angle={math.degrees(scan_angle):.1f}°, "
                        f"force=({repulsion_x:.2f}, {repulsion_y:.2f})"
                    )
        
        # Update global avoidance vectors
        if avoid:
            avoidance_vector_x = total_repulsion_x
            avoidance_vector_y = total_repulsion_y
            
            # Normalize and scale the avoidance vector
            magnitude = math.sqrt(avoidance_vector_x**2 + avoidance_vector_y**2)
            if magnitude > 0.1:
                # Scale to reasonable avoidance distance (2-5 meters)
                max_avoidance = min(5.0, max(2.0, magnitude))
                scale_factor = max_avoidance / magnitude
                avoidance_vector_x *= scale_factor
                avoidance_vector_y *= scale_factor
            else:
                # Default avoidance if calculation fails
                avoidance_vector_x = 3.0 * math.cos(current_heading + math.pi/2)
                avoidance_vector_y = 3.0 * math.sin(current_heading + math.pi/2)
            
            last_avoidance_time = node.get_clock().now().seconds_nanoseconds()[0]
            
            node.get_logger().warn(
                f"OBSTACLE DETECTED! Count: {obstacle_count}, Min dist: {min_distance:.2f}m, "
                f"Avoidance vector: ({avoidance_vector_x:.2f}, {avoidance_vector_y:.2f})"
            )
        else:
            node.get_logger().info(f"Clear path - Min distance: {min_distance:.2f}m", 
                                 throttle_duration_sec=2.0)
            
    except Exception as e:
        node.get_logger().error(f"LaserScan callback error: {str(e)}")
        import traceback
        node.get_logger().error(f"Traceback: {traceback.format_exc()}")

def follow_waypoint(node, mode_client):
    global current_waypoint_index, avoid, avoidance_vector_x, avoidance_vector_y
    global original_target, last_avoidance_time
    
    # Check mission completion
    if current_waypoint_index >= len(waypoints):
        node.get_logger().info("Mission Complete - Landing")
        land_mode = SetMode.Request()
        land_mode.custom_mode = "AUTO.LAND"
        land_future = mode_client.call_async(land_mode)
        rclpy.spin_until_future_complete(node, land_future)
        if land_future.result() and land_future.result().mode_sent:
            node.get_logger().info("Landing initiated")
        else:
            node.get_logger().error("Failed to initiate landing")
        node.timer.cancel()
        return
    
    # Wait for stable pose data
    if raw_pose is None or len(x_buffer) < filter_window:
        node.get_logger().info("Waiting for stable pose data", throttle_duration_sec=1.0)
        return
    
    # Get current position
    current_x = filtered_pose.pose.position.x
    current_y = filtered_pose.pose.position.y
    current_z = filtered_pose.pose.position.z
    
    # Get target waypoint
    target_x, target_y, target_z = waypoints[current_waypoint_index]
    
    # Handle obstacle avoidance
    if avoid:
        # Store original target when starting avoidance
        if original_target is None:
            original_target = [target_x, target_y, target_z]
            node.get_logger().warn(f"Starting avoidance - Original target: {original_target}")
        
        # Apply avoidance - move to avoidance position
        avoid_target_x = current_x + avoidance_vector_x
        avoid_target_y = current_y + avoidance_vector_y
        
        node.get_logger().warn(
            f"AVOIDING OBSTACLE: Current ({current_x:.2f}, {current_y:.2f}) -> "
            f"Avoidance target ({avoid_target_x:.2f}, {avoid_target_y:.2f})"
        )
        
        publish_setpoint(node, node.pose_pub, avoid_target_x, avoid_target_y, target_z)
        
    else:
        # No obstacle - check if we should return to original path
        if original_target is not None:
            current_time = node.get_clock().now().seconds_nanoseconds()[0]
            if (last_avoidance_time is not None and 
                (current_time - last_avoidance_time) > avoidance_timeout):
                
                node.get_logger().info("Returning to original waypoint path")
                target_x, target_y, target_z = original_target
                original_target = None
                last_avoidance_time = None
        
        # Normal waypoint following
        node.get_logger().info(
            f"Following waypoint {current_waypoint_index}: "
            f"Current ({current_x:.2f}, {current_y:.2f}, {current_z:.2f}) -> "
            f"Target ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})"
        )
        
        publish_setpoint(node, node.pose_pub, target_x, target_y, target_z)
        
        # Check if waypoint is reached
        distance = math.sqrt((target_x - current_x)**2 + 
                           (target_y - current_y)**2 + 
                           (target_z - current_z)**2)
        
        node.get_logger().info(f"Distance to waypoint: {distance:.2f}m")
        
        if distance < position_tolerance:
            node.get_logger().info(f"Reached waypoint {current_waypoint_index}")
            current_waypoint_index += 1
            original_target = None  # Reset for next waypoint

def main():
    rclpy.init()
    
    node = Node("drone_controller_node")

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT, 
        durability=QoSDurabilityPolicy.VOLATILE, 
        history=QoSHistoryPolicy.KEEP_LAST, 
        depth=10
    )
    
    # Subscriptions
    node.drone_state = node.create_subscription(
        State, 
        '/mavros/state', 
        state_callback, 
        10
    )
    
    # Use LaserScan topic (not PointCloud2)
    node.lidar = node.create_subscription(
        LaserScan,
        "/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan",
        lambda msg: lidar_callback(msg, node),
        qos_profile
    )
    
    node.pose_sub = node.create_subscription(
        PoseStamped, 
        "/mavros/local_position/pose", 
        pose_callback, 
        qos_profile
    )
    
    # Service clients
    node.arm_client = node.create_client(CommandBool, "/mavros/cmd/arming")
    node.mode_client = node.create_client(SetMode, "/mavros/set_mode")
    
    # Publisher
    node.pose_pub = node.create_publisher(
        PoseStamped, 
        "/mavros/setpoint_position/local",
        10
    )   
    
    # Wait for services
    while not node.arm_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for arming service...")
    while not node.mode_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for mode service...")
    
    # Arm and takeoff
    if not arm_and_takeoff(node, node.arm_client, node.mode_client, node.pose_pub):
        node.get_logger().error("Failed to arm and takeoff")
        return
    
    # Start waypoint following timer
    node.timer = node.create_timer(0.2, lambda: follow_waypoint(node, node.mode_client))
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()