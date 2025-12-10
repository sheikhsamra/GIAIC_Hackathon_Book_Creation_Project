# Code Examples: Introduction to Physical AI

## 1. Basic ROS 2 Node Structure

### 1.1 Simple Publisher Node

```python
#!/usr/bin/env python3
"""
Basic ROS 2 publisher node for Physical AI systems
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PhysicalAIStatusPublisher(Node):
    def __init__(self):
        super().__init__('physical_ai_status_publisher')
        self.publisher_ = self.create_publisher(String, 'ai_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Physical AI System Operational: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    physical_ai_publisher = PhysicalAIStatusPublisher()

    try:
        rclpy.spin(physical_ai_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        physical_ai_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 1.2 Simple Subscriber Node

```python
#!/usr/bin/env python3
"""
Basic ROS 2 subscriber node for Physical AI systems
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PhysicalAIStatusListener(Node):
    def __init__(self):
        super().__init__('physical_ai_status_listener')
        self.subscription = self.create_subscription(
            String,
            'ai_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received AI Status: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    physical_ai_listener = PhysicalAIStatusListener()

    try:
        rclpy.spin(physical_ai_listener)
    except KeyboardInterrupt:
        pass
    finally:
        physical_ai_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 2. Sensor Data Processing

### 2.1 Processing Camera Data

```python
#!/usr/bin/env python3
"""
Camera data processing for Physical AI systems
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/processed', 10)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process the image (example: edge detection)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Convert back to ROS Image message
        processed_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')

        # Publish processed image
        self.publisher.publish(processed_msg)

        # Display the result
        cv2.imshow('Original', cv_image)
        cv2.imshow('Edges', edges)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    camera_processor = CameraProcessor()

    try:
        rclpy.spin(camera_processor)
    except KeyboardInterrupt:
        pass
    finally:
        camera_processor.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2.2 Processing IMU Data

```python
#!/usr/bin/env python3
"""
IMU data processing for Physical AI systems
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np


class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        self.orientation_publisher = self.create_publisher(Vector3, '/robot/orientation', 10)
        self.linear_acceleration_publisher = self.create_publisher(Vector3, '/robot/acceleration', 10)

    def imu_callback(self, msg):
        # Extract orientation (in quaternion format)
        orientation_q = msg.orientation
        # Convert to Euler angles (simplified - in practice use proper conversion)
        roll = np.arctan2(2.0 * (orientation_q.w * orientation_q.x + orientation_q.y * orientation_q.z),
                          1.0 - 2.0 * (orientation_q.x * orientation_q.x + orientation_q.y * orientation_q.y))
        pitch = np.arcsin(2.0 * (orientation_q.w * orientation_q.y - orientation_q.z * orientation_q.x))
        yaw = np.arctan2(2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y),
                         1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z))

        # Create orientation message
        orientation_msg = Vector3()
        orientation_msg.x = roll
        orientation_msg.y = pitch
        orientation_msg.z = yaw
        self.orientation_publisher.publish(orientation_msg)

        # Extract linear acceleration
        acceleration_msg = Vector3()
        acceleration_msg.x = msg.linear_acceleration.x
        acceleration_msg.y = msg.linear_acceleration.y
        acceleration_msg.z = msg.linear_acceleration.z
        self.linear_acceleration_publisher.publish(acceleration_msg)

        # Log data
        self.get_logger().info(f'Orientation - Roll: {roll:.3f}, Pitch: {pitch:.3f}, Yaw: {yaw:.3f}')
        self.get_logger().info(f'Acceleration - X: {acceleration_msg.x:.3f}, Y: {acceleration_msg.y:.3f}, Z: {acceleration_msg.z:.3f}')


def main(args=None):
    rclpy.init(args=args)
    imu_processor = IMUProcessor()

    try:
        rclpy.spin(imu_processor)
    except KeyboardInterrupt:
        pass
    finally:
        imu_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 3. Safety Monitoring System

### 3.1 Basic Safety Monitor

```python
#!/usr/bin/env python3
"""
Safety monitoring system for Physical AI applications
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import threading
import time


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Subscriptions
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Publishers
        self.safety_status_publisher = self.create_publisher(Bool, '/safety_status', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, '/emergency_stop', 10)

        # Parameters
        self.safety_distance = 0.5  # meters
        self.is_safe = True
        self.cmd_vel_buffer = Twist()

        # Timer for safety checks
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info('Safety Monitor initialized')

    def laser_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        if len(msg.ranges) > 0:
            # Find minimum distance in front of robot (±30 degrees)
            front_ranges = msg.ranges[:30] + msg.ranges[-30:]
            front_ranges = [r for r in front_ranges if r != float('inf') and not r != r]  # Remove inf and NaN

            if front_ranges:
                min_distance = min(front_ranges)
                if min_distance < self.safety_distance:
                    self.is_safe = False
                    self.get_logger().warn(f'OBSTACLE DETECTED: {min_distance:.2f}m (threshold: {self.safety_distance}m)')
                else:
                    self.is_safe = True

    def cmd_vel_callback(self, msg):
        """Store the latest velocity command"""
        self.cmd_vel_buffer = msg

    def safety_check(self):
        """Periodic safety check"""
        safety_msg = Bool()
        safety_msg.data = self.is_safe
        self.safety_status_publisher.publish(safety_msg)

        # If not safe, publish emergency stop
        if not self.is_safe:
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_publisher.publish(emergency_msg)
        else:
            # Publish safe status
            emergency_msg = Bool()
            emergency_msg.data = False
            self.emergency_stop_publisher.publish(emergency_msg)

    def get_safety_status(self):
        """Return current safety status"""
        return self.is_safe


def main(args=None):
    rclpy.init(args=args)
    safety_monitor = SafetyMonitor()

    try:
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        safety_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 4. Basic Control System

### 4.1 Simple Go-to-Goal Controller

```python
#!/usr/bin/env python3
"""
Simple go-to-goal controller for mobile robots
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math


class GoToGoalController(Node):
    def __init__(self):
        super().__init__('go_to_goal_controller')

        # Subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Goal parameters
        self.goal_x = 5.0
        self.goal_y = 5.0
        self.goal_tolerance = 0.1

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Control parameters
        self.linear_kp = 1.0
        self.angular_kp = 2.0

        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'Go-to-goal controller initialized. Goal: ({self.goal_x}, {self.goal_y})')

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to euler (simplified - in practice use tf2)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """Main control loop"""
        # Calculate distance to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        # Check if goal reached
        if distance < self.goal_tolerance:
            self.get_logger().info(f'Goal reached! Position: ({self.current_x:.2f}, {self.current_y:.2f})')
            # Stop the robot
            stop_msg = Twist()
            self.cmd_vel_publisher.publish(stop_msg)
            return

        # Calculate desired angle to goal
        desired_theta = math.atan2(dy, dx)

        # Calculate angle error
        angle_error = desired_theta - self.current_theta
        # Normalize angle error to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Create velocity command
        cmd_vel = Twist()

        # If robot is not aligned with goal direction, rotate first
        if abs(angle_error) > 0.1:  # 0.1 rad = ~5.7 degrees
            cmd_vel.angular.z = self.angular_kp * angle_error
        else:
            # Move forward toward goal
            cmd_vel.linear.x = min(self.linear_kp * distance, 0.5)  # Limit max speed
            cmd_vel.angular.z = self.angular_kp * angle_error

        # Publish command
        self.cmd_vel_publisher.publish(cmd_vel)

        self.get_logger().info(f'Distance to goal: {distance:.2f}m, Angle error: {math.degrees(angle_error):.2f}°')


def main(args=None):
    rclpy.init(args=args)
    controller = GoToGoalController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 5. Environment Setup Script

### 5.1 Physical AI Development Environment Setup

```bash
#!/bin/bash
# Physical AI Development Environment Setup Script

set -e  # Exit on any error

echo "Setting up Physical AI Development Environment..."

# Update system packages
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble Hawksbill
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
fi

sudo apt install ros-humble-desktop ros-humble-ros-base -y
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

# Initialize rosdep
sudo rosdep init || true
rosdep update

# Install Gazebo Garden
if [ ! -f /etc/apt/sources.list.d/gazebo.list ]; then
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo.list'
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt update
fi

sudo apt install gazebo -y

# Install Python dependencies
pip3 install --user -U colcon-common-extensions
pip3 install --user -U rosdep rosinstall_generator wstool vcstool

# Install additional Python packages for Physical AI
pip3 install --user numpy scipy matplotlib opencv-python transforms3d

# Install NVIDIA Isaac Sim (if NVIDIA GPU is available)
if nvidia-smi > /dev/null 2>&1; then
    echo "NVIDIA GPU detected. Installing Isaac Sim prerequisites..."
    sudo apt install nvidia-driver-470-server nvidia-utils-470-server -y
    echo "Please visit https://developer.nvidia.com/isaac-sim to download and install Isaac Sim"
else
    echo "No NVIDIA GPU detected. Skipping Isaac Sim installation."
fi

# Create workspace
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws

echo "Physical AI Development Environment setup complete!"
echo "Please run 'source /opt/ros/humble/setup.bash' or add it to your ~/.bashrc"
```

These code examples provide a foundation for understanding Physical AI concepts through practical implementation. Each example demonstrates key aspects of Physical AI systems, from basic ROS 2 communication to safety monitoring and control systems.