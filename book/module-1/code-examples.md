---
title: Code Examples - Introduction to Physical AI
sidebar_label: Code Examples
---

# Code Examples - Introduction to Physical AI

## Overview

This section provides practical code examples that demonstrate fundamental concepts in Physical AI. All examples follow safety guidelines and are designed to run in simulation environments before deployment to physical robots.

## Example 1: Basic Robot State Publisher

This example demonstrates how to publish robot state information, which is fundamental to Physical AI systems:

```python
#!/usr/bin/env python3
# Basic Robot State Publisher
# Demonstrates publishing robot state information

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import random

class BasicRobotStatePublisher(Node):
    def __init__(self):
        super().__init__('basic_robot_state_publisher')

        # Safety parameters
        self.declare_parameter('max_joint_velocity', 1.0)
        self.declare_parameter('max_joint_effort', 10.0)

        # Publishers and broadcasters
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.broadcaster = TransformBroadcaster(self)

        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz

        # Joint names and initial positions
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4',
            'joint5', 'joint6', 'left_wheel', 'right_wheel'
        ]
        self.joint_positions = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_efforts = [0.0] * len(self.joint_names)

        self.get_logger().info('Basic Robot State Publisher initialized')

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Publish joint states
        self.joint_pub.publish(msg)

        # Publish transforms
        self.publish_transforms()

    def publish_transforms(self):
        # Example transform for a simple robot
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Simple movement pattern for demonstration
        time = self.get_clock().now().nanoseconds / 1e9
        t.transform.translation.x = math.sin(time * 0.5) * 0.5
        t.transform.translation.y = math.cos(time * 0.5) * 0.5
        t.transform.translation.z = 0.1
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(time * 0.25)
        t.transform.rotation.w = math.cos(time * 0.25)

        self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    node = BasicRobotStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 2: Safe Movement Controller

This example demonstrates safe movement control with validation and safety limits:

```python
#!/usr/bin/env python3
# Safe Movement Controller
# Demonstrates safe movement with validation and limits

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
import math

class SafeMovementController(Node):
    def __init__(self):
        super().__init__('safe_movement_controller')

        # Safety parameters with validation
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('min_obstacle_distance', 0.5)
        self.declare_parameter('safety_margin', 0.3)

        # Get parameters with safety validation
        max_lin_vel = self.get_parameter('max_linear_velocity').value
        max_ang_vel = self.get_parameter('max_angular_velocity').value
        min_obs_dist = self.get_parameter('min_obstacle_distance').value

        # Validate parameters are within safe ranges
        if max_lin_vel > 1.0:  # Safety limit
            self.get_logger().warn('Linear velocity limited for safety')
            max_lin_vel = 1.0
        if max_ang_vel > 2.0:  # Safety limit
            self.get_logger().warn('Angular velocity limited for safety')
            max_ang_vel = 2.0

        # Store validated parameters
        self.max_linear_velocity = max_lin_vel
        self.max_angular_velocity = max_ang_vel
        self.min_obstacle_distance = min_obs_dist
        self.safety_margin = self.get_parameter('safety_margin').value

        # Publishers and subscribers
        qos_profile = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile)

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        # State variables
        self.obstacle_distances = []
        self.safe_to_move = True
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0

        self.get_logger().info('Safe Movement Controller initialized')

    def scan_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        # Filter valid range readings
        valid_ranges = [r for r in msg.ranges if not (math.isnan(r) or math.isinf(r))]

        if valid_ranges:
            # Find minimum distance
            min_distance = min(valid_ranges) if valid_ranges else float('inf')
            self.obstacle_distances = valid_ranges
            self.safe_to_move = min_distance > (self.min_obstacle_distance + self.safety_margin)

            if not self.safe_to_move:
                self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m, stopping')
        else:
            # No valid readings - stop for safety
            self.safe_to_move = False

    def control_loop(self):
        """Main control loop with safety checks"""
        cmd_msg = Twist()

        if self.safe_to_move:
            # Simple movement pattern - forward with occasional turns
            time = self.get_clock().now().nanoseconds / 1e9
            cmd_msg.linear.x = self.max_linear_velocity * 0.5  # Move forward at half speed
            cmd_msg.angular.z = math.sin(time) * 0.3  # Gentle turning
        else:
            # Emergency stop
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0

        # Apply velocity limits
        cmd_msg.linear.x = max(-self.max_linear_velocity,
                              min(self.max_linear_velocity, cmd_msg.linear.x))
        cmd_msg.angular.z = max(-self.max_angular_velocity,
                               min(self.max_angular_velocity, cmd_msg.angular.z))

        # Publish command
        self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)

    node = SafeMovementController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Safe Movement Controller')
    finally:
        # Emergency stop on shutdown
        emergency_stop = Twist()
        emergency_stop.linear.x = 0.0
        emergency_stop.angular.z = 0.0
        node.cmd_vel_pub.publish(emergency_stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example 3: Perception Data Processing

This example demonstrates processing of sensor data for perception:

```python
#!/usr/bin/env python3
# Perception Data Processing
# Demonstrates basic perception with sensor data processing

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np
import math

class PerceptionProcessor(Node):
    def __init__(self):
        super().__init__('perception_processor')

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'perception_markers', 10)
        self.object_pub = self.create_publisher(Marker, 'detected_objects', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # Parameters
        self.declare_parameter('min_distance_threshold', 0.3)
        self.declare_parameter('max_distance_threshold', 5.0)
        self.declare_parameter('cluster_min_points', 3)

        self.min_dist_thresh = self.get_parameter('min_distance_threshold').value
        self.max_dist_thresh = self.get_parameter('max_distance_threshold').value
        self.cluster_min_points = self.get_parameter('cluster_min_points').value

        # Timer for visualization
        self.viz_timer = self.create_timer(0.5, self.publish_visualization)  # 2Hz

        # Internal state
        self.latest_scan = None
        self.detected_clusters = []

        self.get_logger().info('Perception Processor initialized')

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        self.latest_scan = msg

        # Convert polar coordinates to Cartesian
        points = []
        for i, range_val in enumerate(msg.ranges):
            if not (math.isnan(range_val) or math.isinf(range_val)):
                if self.min_dist_thresh <= range_val <= self.max_dist_thresh:
                    angle = msg.angle_min + i * msg.angle_increment
                    x = range_val * math.cos(angle)
                    y = range_val * math.sin(angle)
                    points.append((x, y))

        # Simple clustering algorithm
        self.detected_clusters = self.cluster_points(points)

    def cluster_points(self, points):
        """Simple clustering algorithm to group nearby points"""
        if len(points) < self.cluster_min_points:
            return []

        clusters = []
        unassigned = set(range(len(points)))

        while unassigned:
            # Start a new cluster with a random point
            start_idx = unassigned.pop()
            cluster = [points[start_idx]]
            cluster_indices = {start_idx}

            # Find all points within distance threshold
            changed = True
            while changed:
                changed = False
                for i in unassigned.copy():
                    point = points[i]
                    # Check distance to any point in current cluster
                    for cluster_point in cluster:
                        dist = math.sqrt((point[0] - cluster_point[0])**2 +
                                       (point[1] - cluster_point[1])**2)
                        if dist < 0.5:  # 50cm threshold
                            cluster.append(point)
                            cluster_indices.add(i)
                            unassigned.remove(i)
                            changed = True
                            break

            # Only keep clusters with minimum number of points
            if len(cluster) >= self.cluster_min_points:
                clusters.append(cluster)

        return clusters

    def publish_visualization(self):
        """Publish visualization markers for detected clusters"""
        if not self.detected_clusters:
            return

        marker_array = MarkerArray()

        for i, cluster in enumerate(self.detected_clusters):
            # Create a marker for the cluster centroid
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "clusters"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Calculate centroid
            centroid_x = sum(p[0] for p in cluster) / len(cluster)
            centroid_y = sum(p[1] for p in cluster) / len(cluster)

            marker.pose.position.x = centroid_x
            marker.pose.position.y = centroid_y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            # Size based on cluster size
            marker.scale.x = min(0.5, len(cluster) * 0.1)  # Max 50cm
            marker.scale.y = marker.scale.x
            marker.scale.z = 0.2

            # Color based on cluster size
            marker.color.r = min(1.0, len(cluster) / 10.0)
            marker.color.g = 0.5
            marker.color.b = 1.0 - marker.color.r
            marker.color.a = 0.7

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    node = PerceptionProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety Guidelines for Code Examples

⚠️ **IMPORTANT SAFETY NOTES**:

1. **Simulation First**: All examples should be tested in simulation before deployment to physical robots
2. **Parameter Validation**: Always validate parameters to ensure they're within safe operating ranges
3. **Emergency Stops**: Implement emergency stop mechanisms that can halt robot motion immediately
4. **Obstacle Detection**: Include obstacle detection and avoidance in all movement code
5. **Velocity Limits**: Always enforce reasonable velocity and acceleration limits
6. **Error Handling**: Implement proper error handling and graceful degradation

## Running the Examples

### Prerequisites
- ROS 2 Humble Hawksbill installed
- Gazebo Garden or compatible simulator
- Basic ROS 2 workspace setup

### Execution Steps

1. **Create a new ROS 2 package**:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --dependencies rclpy sensor_msgs geometry_msgs visualization_msgs -- python_pkg
   ```

2. **Copy the example code** to the appropriate location in your package

3. **Build the workspace**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select python_pkg
   source install/setup.bash
   ```

4. **Run the example** in a simulated environment:
   ```bash
   ros2 run python_pkg basic_robot_state_publisher
   ```

### Testing in Gazebo

For complete testing, set up a Gazebo environment:

1. **Launch Gazebo** with a simple world:
   ```bash
   ros2 launch gazebo_ros empty_world.launch.py
   ```

2. **Spawn a robot model** (e.g., differential drive robot)

3. **Run the example nodes** to control the robot safely

## Summary

These code examples demonstrate fundamental Physical AI concepts:
- State publishing and TF transforms
- Safe movement with obstacle avoidance
- Basic perception and clustering

All examples include safety features and are designed for simulation-first development. Remember to always test in simulation before deploying to physical robots, and implement additional safety measures as needed for your specific application.