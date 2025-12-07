---
title: Lab Exercises - Introduction to Physical AI
sidebar_label: Lab Exercises
---

# Lab Exercises - Introduction to Physical AI

## Overview

This lab exercises section provides hands-on activities to reinforce the concepts learned in Module 1. Each lab includes detailed instructions, expected outcomes, and safety considerations.

## Lab Exercise 1: Setting Up the Physical AI Development Environment

### Objective
Set up a complete development environment for Physical AI projects using ROS 2, Gazebo, and necessary tools.

### Prerequisites
- Ubuntu 22.04 LTS (recommended)
- At least 8GB RAM (16GB recommended)
- 50GB free disk space
- Internet connection

### Required Software
- ROS 2 Humble Hawksbill
- Gazebo Garden
- Python 3.10 or 3.11
- Git
- Docker (optional)

### Instructions

#### Step 1: Install ROS 2 Humble
```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2 python3-colcon-common-extensions
sudo rosdep init
rosdep update

# Source ROS 2 in your bashrc to make it available by default
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Step 2: Install Gazebo
```bash
curl -sSL http://get.gazebosim.org | sh
```

#### Step 3: Create a ROS 2 Workspace
```bash
# Create workspace directory
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws

# Source ROS 2 and build workspace (even though it's empty)
source /opt/ros/humble/setup.bash
colcon build

# Add workspace to bashrc
echo "source ~/physical_ai_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Step 4: Test the Installation
```bash
# Test ROS 2
ros2 topic list

# Test Gazebo
gz sim --verbose
```

### Expected Outcome
- ROS 2 Humble installed and sourced
- Gazebo Garden installed and running
- ROS 2 workspace created successfully
- Basic ROS 2 and Gazebo commands working

### Safety Considerations
- Ensure adequate ventilation when running simulations
- Monitor system resources to prevent overheating
- Save work frequently during extended sessions

### Evaluation Criteria
- [ ] ROS 2 installation verified with `ros2 --version`
- [ ] Gazebo runs without errors
- [ ] ROS 2 workspace builds without errors
- [ ] Basic ROS 2 commands execute successfully

---

## Lab Exercise 2: Basic Robot State Publisher and TF Tree

### Objective
Create a simple robot state publisher and visualize the robot's TF tree to understand spatial relationships in Physical AI systems.

### Prerequisites
- Completed Lab Exercise 1
- Basic Python knowledge

### Instructions

#### Step 1: Create a New Package
```bash
cd ~/physical_ai_ws/src
ros2 pkg create --dependencies rclpy std_msgs sensor_msgs geometry_msgs tf2_ros -- python_robot_state_publisher
```

#### Step 2: Create the State Publisher Node
Create the file `~/physical_ai_ws/src/python_robot_state_publisher/python_robot_state_publisher/state_publisher.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Publishers and broadcasters
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.broadcaster = TransformBroadcaster(self)

        # Timer for periodic updates (10Hz)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Joint names and initial positions
        self.joint_names = ['joint1', 'joint2', 'left_wheel', 'right_wheel']
        self.joint_positions = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_efforts = [0.0] * len(self.joint_names)

        self.get_logger().info('Robot State Publisher initialized')

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
        # Publish base_link to odom transform (simple movement pattern)
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

    node = RobotStatePublisher()

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

#### Step 3: Update setup.py
Edit the `setup.py` file in the package directory to make the script executable:

```python
from setuptools import find_packages, setup

package_name = 'python_robot_state_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Basic robot state publisher for Physical AI lab',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = python_robot_state_publisher.state_publisher:main',
        ],
    },
)
```

#### Step 4: Build and Run
```bash
cd ~/physical_ai_ws
colcon build --packages-select python_robot_state_publisher
source install/setup.bash

# Run the state publisher
ros2 run python_robot_state_publisher state_publisher
```

#### Step 5: Visualize TF Tree
In a new terminal:
```bash
# Install and run tf2 tools
sudo apt install ros-humble-tf2-tools
rviz2  # Then add TF display to visualize the tree
```

Or use command line:
```bash
ros2 run tf2_tools view_frames
```

### Expected Outcome
- Robot state publisher node running
- TF tree visualized showing spatial relationships
- Joint states published at 10Hz
- Understanding of coordinate frames in Physical AI

### Safety Considerations
- No physical safety risks (simulation only)
- Ensure adequate system resources for visualization

### Evaluation Criteria
- [ ] State publisher node runs without errors
- [ ] Joint states published to `/joint_states` topic
- [ ] TF tree visualized correctly
- [ ] Understanding of frame relationships demonstrated

---

## Lab Exercise 3: Perception and Obstacle Detection

### Objective
Implement a basic perception system that detects obstacles using simulated laser scan data.

### Prerequisites
- Completed Lab Exercises 1 and 2
- Understanding of ROS 2 topics and messages

### Instructions

#### Step 1: Create Perception Package
```bash
cd ~/physical_ai_ws/src
ros2 pkg create --dependencies rclpy sensor_msgs geometry_msgs visualization_msgs -- python_perception
```

#### Step 2: Create Obstacle Detection Node
Create the file `~/physical_ai_ws/src/python_perception/python_perception/obstacle_detector.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # Parameters
        self.declare_parameter('min_distance', 0.5)
        self.declare_parameter('max_distance', 3.0)
        self.declare_parameter('min_angle', -1.57)  # -90 degrees
        self.declare_parameter('max_angle', 1.57)   # 90 degrees

        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value

        # Publishers and subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'obstacle_markers', 10)

        # Internal state
        self.obstacles_detected = []

        self.get_logger().info('Obstacle Detector initialized')

    def scan_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        obstacles = []

        # Process each range reading
        for i, range_val in enumerate(msg.ranges):
            if not (math.isnan(range_val) or math.isinf(range_val)):
                if self.min_distance <= range_val <= self.max_distance:
                    angle = msg.angle_min + i * msg.angle_increment

                    # Only consider angles in specified range
                    if self.min_angle <= angle <= self.max_angle:
                        x = range_val * math.cos(angle)
                        y = range_val * math.sin(angle)
                        obstacles.append((x, y, range_val))

        self.obstacles_detected = obstacles
        self.publish_visualization()

    def publish_visualization(self):
        """Publish visualization markers for detected obstacles"""
        if not self.obstacles_detected:
            return

        marker_array = MarkerArray()

        for i, (x, y, distance) in enumerate(self.obstacles_detected):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            # Size based on distance (closer = larger)
            size = max(0.1, 0.3 / (distance + 0.1))
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = 0.1

            # Color based on distance (red = close, green = far)
            intensity = min(1.0, max(0.0, (distance - self.min_distance) /
                           (self.max_distance - self.min_distance)))
            marker.color.r = 1.0 - intensity
            marker.color.g = intensity
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    node = ObstacleDetector()

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

#### Step 3: Update setup.py
Edit the `setup.py` file:

```python
from setuptools import find_packages, setup

package_name = 'python_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Obstacle detection for Physical AI lab',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector = python_perception.obstacle_detector:main',
        ],
    },
)
```

#### Step 4: Test with Gazebo Simulation
```bash
# Terminal 1: Start Gazebo with a simple world
gz sim -r -v 1 empty.sdf

# Terminal 2: Spawn a robot with laser scanner
# You can use a pre-built model or create your own URDF

# Terminal 3: Run the obstacle detector
cd ~/physical_ai_ws
colcon build --packages-select python_perception
source install/setup.bash
ros2 run python_perception obstacle_detector
```

### Expected Outcome
- Obstacle detection node processing laser scan data
- Visualization of detected obstacles in RViz or as markers
- Understanding of perception in Physical AI systems

### Safety Considerations
- Simulation-only exercise, no physical safety risks
- Ensure proper resource management during simulation

### Evaluation Criteria
- [ ] Obstacle detector node processes scan data correctly
- [ ] Obstacles visualized as markers
- [ ] Distance and angle filtering working properly
- [ ] Understanding of perception challenges demonstrated

---

## Lab Exercise 4: Safe Movement Controller

### Objective
Implement a safe movement controller that integrates perception and action with safety constraints.

### Prerequisites
- Completed previous lab exercises
- Understanding of ROS 2 topics and safety concepts

### Instructions

#### Step 1: Create Control Package
```bash
cd ~/physical_ai_ws/src
ros2 pkg create --dependencies rclpy sensor_msgs geometry_msgs -- python_safe_controller
```

#### Step 2: Create Safe Controller Node
Create the file `~/physical_ai_ws/src/python_safe_controller/python_safe_controller/safe_controller.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import math

class SafeController(Node):
    def __init__(self):
        super().__init__('safe_controller')

        # Safety parameters
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 0.5)
        self.declare_parameter('safe_distance', 0.6)
        self.declare_parameter('safety_margin', 0.2)

        self.max_lin_vel = min(1.0, self.get_parameter('max_linear_velocity').value)  # Safety cap
        self.max_ang_vel = min(1.5, self.get_parameter('max_angular_velocity').value)  # Safety cap
        self.safe_distance = self.get_parameter('safe_distance').value
        self.safety_margin = self.get_parameter('safety_margin').value

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        # State variables
        self.obstacle_distances = []
        self.safe_to_move = True
        self.emergency_stop = False

        self.get_logger().info('Safe Controller initialized')

    def scan_callback(self, msg):
        """Process laser scan to detect obstacles"""
        # Filter valid range readings
        valid_ranges = [r for r in msg.ranges if not (math.isnan(r) or math.isinf(r))]

        if valid_ranges:
            min_distance = min(valid_ranges) if valid_ranges else float('inf')
            self.obstacle_distances = valid_ranges
            # Check if any obstacle is within safe distance + margin
            self.safe_to_move = min_distance > (self.safe_distance + self.safety_margin)

            if not self.safe_to_move:
                self.get_logger().warn(f'Obstacle at {min_distance:.2f}m, stopping')
        else:
            # No valid readings - safest to stop
            self.safe_to_move = False
            self.get_logger().warn('No valid sensor readings, stopping')

    def control_loop(self):
        """Main control loop with safety checks"""
        cmd_msg = Twist()

        if self.emergency_stop or not self.safe_to_move:
            # Emergency stop - no movement
            cmd_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
            cmd_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        else:
            # Safe to move - implement a simple behavior (move forward with gentle turns)
            time = self.get_clock().now().nanoseconds / 1e9
            cmd_msg.linear.x = self.max_lin_vel * 0.6  # 60% of max speed
            cmd_msg.angular.z = math.sin(time) * 0.2  # Gentle turning

        # Apply velocity limits
        cmd_msg.linear.x = max(-self.max_lin_vel, min(self.max_lin_vel, cmd_msg.linear.x))
        cmd_msg.angular.z = max(-self.max_ang_vel, min(self.max_ang_vel, cmd_msg.angular.z))

        # Publish command
        self.cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)

    node = SafeController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Safe Controller')
    finally:
        # Emergency stop on shutdown
        emergency_stop = Twist()
        emergency_stop.linear = Vector3(x=0.0, y=0.0, z=0.0)
        emergency_stop.angular = Vector3(x=0.0, y=0.0, z=0.0)
        node.cmd_pub.publish(emergency_stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 3: Update setup.py
```python
from setuptools import find_packages, setup

package_name = 'python_safe_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Safe movement controller for Physical AI lab',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safe_controller = python_safe_controller.safe_controller:main',
        ],
    },
)
```

#### Step 4: Test the Safe Controller
```bash
# Terminal 1: Start Gazebo
gz sim -r -v 1 empty.sdf

# Terminal 2: Run the safe controller
cd ~/physical_ai_ws
colcon build --packages-select python_safe_controller
source install/setup.bash
ros2 run python_safe_controller safe_controller

# Terminal 3: Monitor the robot's movement and safety behavior
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist
```

### Expected Outcome
- Safe controller responding to obstacles
- Robot stops when obstacles are detected
- Understanding of safety in Physical AI systems

### Safety Considerations
- The controller implements multiple safety layers
- Emergency stop functionality
- Velocity limits to prevent dangerous movements

### Evaluation Criteria
- [ ] Safe controller responds to obstacle detection
- [ ] Robot stops when obstacles are within safe distance
- [ ] Velocity limits enforced properly
- [ ] Emergency stop functionality working
- [ ] Understanding of safety-critical systems demonstrated

---

## Summary

These lab exercises provide hands-on experience with:

1. **Environment Setup**: Complete development environment for Physical AI
2. **State Publishing**: Understanding TF trees and coordinate frames
3. **Perception**: Basic obstacle detection and visualization
4. **Safe Control**: Integration of perception and action with safety constraints

Each lab includes safety considerations, evaluation criteria, and expected outcomes to ensure proper learning and understanding of Physical AI concepts. The progression from basic setup to safe control demonstrates the complexity and safety-critical nature of Physical AI systems.