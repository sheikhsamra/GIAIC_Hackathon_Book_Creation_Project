# Code Examples: Gazebo Simulation and Physics Engines

## 1. Robot Model Definitions

### 1.1 Simple Mobile Robot URDF

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.141" ixy="0.0" ixz="0.0" iyy="0.141" iyz="0.0" izz="0.225"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.05" rpy="1.570796 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 -0.05" rpy="1.570796 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Gazebo extensions -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>

  <!-- Differential drive plugin -->
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <odom_frame>odom</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_wheel_tf>false</publish_wheel_tf>
    </plugin>
  </gazebo>

</robot>
```

### 1.2 Robot with Sensors (SDF Format)

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="sensor_robot">
    <pose>0 0 0.1 0 0 0</pose>

    <!-- Chassis -->
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.4</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.4</iyy>
          <iyz>0</iyz>
          <izz>0.2</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.2</size>
          </box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.8 1</ambient>
          <diffuse>0.4 0.4 1 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Camera -->
    <link name="camera_link">
      <pose>0.2 0 0.1 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>

      <collision name="camera_collision">
        <geometry>
          <box>
            <size>0.05 0.05 0.05</size>
          </box>
        </geometry>
      </collision>

      <visual name="camera_visual">
        <geometry>
          <box>
            <size>0.05 0.05 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Camera joint -->
    <joint name="camera_joint" type="fixed">
      <parent>chassis</parent>
      <child>camera_link</child>
      <pose>0.2 0 0 0 0 0</pose>
    </joint>

    <!-- Camera sensor -->
    <sensor name="camera" type="camera">
      <pose>0 0 0 0 0 0</pose>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
    </sensor>

    <!-- IMU -->
    <sensor name="imu" type="imu">
      <pose>0 0 0 0 0 0</pose>
      <always_on>1</always_on>
      <update_rate>100</update_rate>
    </sensor>

    <!-- Differential drive plugin -->
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <odom_frame>odom</odom_frame>
      <robot_base_frame>chassis</robot_base_frame>
    </plugin>

    <!-- Camera plugin -->
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <camera_name>camera</camera_name>
      <image_topic_name>image_raw</image_topic_name>
      <camera_info_topic_name>camera_info</camera_info_topic_name>
      <frame_name>camera_link</frame_name>
    </plugin>

  </model>
</sdf>
```

## 2. World File Examples

### 2.1 Simple World with Obstacles

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.0 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Simple robot -->
    <include>
      <uri>model://simple_robot</uri>
      <pose>0 0 0.1 0 0 0</pose>
    </include>

    <!-- Obstacles -->
    <model name="box1">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.3 0.1 1</ambient>
            <diffuse>1.0 0.5 0.3 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="cylinder1">
      <pose>-2 -2 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.1 0.8 0.3 1</ambient>
            <diffuse>0.3 1.0 0.5 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.291667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.291667</iyy>
            <iyz>0</iyz>
            <izz>0.125</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Wall -->
    <model name="wall">
      <pose>0 3 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>5 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>5 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>8.58333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>10.41667</iyy>
            <iyz>0</iyz>
            <izz>2.08333</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## 3. ROS 2 Integration Examples

### 3.1 Launch File for Gazebo Integration

```python
#!/usr/bin/env python3
"""
Launch file for Gazebo simulation with ROS 2 integration
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_world.sdf',
        description='Choose one of the world files from `/my_robot_gazebo/worlds`'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='simple_robot',
        description='Choose one of the robot models from `/my_robot_description/robots`'
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('my_robot_gazebo'),
                'worlds',
                LaunchConfiguration('world')
            ])
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'urdf',
                LaunchConfiguration('model') + '.urdf'
            ])
        ]
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', LaunchConfiguration('model'),
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # Return the complete launch description
    return LaunchDescription([
        world_arg,
        model_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

### 3.2 ROS 2 Node for Simulation Control

```python
#!/usr/bin/env python3
"""
ROS 2 node for controlling robot in Gazebo simulation
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class SimulationController(Node):
    def __init__(self):
        super().__init__('simulation_controller')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Internal state
        self.current_pose = Pose()
        self.current_twist = Twist()
        self.cv_bridge = CvBridge()

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Navigation parameters
        self.target_x = 5.0
        self.target_y = 5.0
        self.goal_tolerance = 0.2
        self.avoid_distance = 0.5

        self.get_logger().info('Simulation Controller initialized')

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def scan_callback(self, msg):
        """Process laser scan data"""
        # Check for obstacles in front of robot
        if len(msg.ranges) > 0:
            # Front 30 degrees
            front_ranges = msg.ranges[:15] + msg.ranges[-15:]
            front_ranges = [r for r in front_ranges if 0.1 < r < 10.0]

            if front_ranges:
                min_front_dist = min(front_ranges)
                if min_front_dist < self.avoid_distance:
                    self.get_logger().warn(f'Obstacle detected: {min_front_dist:.2f}m')

    def image_callback(self, msg):
        """Process camera image"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Simple color detection example (detect red objects)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower_red, upper_red)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 100:  # Minimum area threshold
                    self.get_logger().info('Red object detected in camera view')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def control_loop(self):
        """Main control loop"""
        # Calculate distance to goal
        dx = self.target_x - self.current_pose.position.x
        dy = self.target_y - self.current_pose.position.y
        distance_to_goal = np.sqrt(dx**2 + dy**2)

        cmd_vel = Twist()

        if distance_to_goal < self.goal_tolerance:
            # Goal reached, stop
            self.get_logger().info('Goal reached!')
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        else:
            # Navigate towards goal
            target_angle = np.arctan2(dy, dx)

            # Get current orientation (simplified - in practice use proper quaternion conversion)
            current_yaw = 0.0  # This would need proper conversion from quaternion

            # Simple proportional controller
            angle_error = target_angle - current_yaw
            cmd_vel.linear.x = min(0.5, distance_to_goal * 0.5)  # Move forward
            cmd_vel.angular.z = angle_error * 1.0  # Turn towards goal

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

        self.get_logger().info(
            f'Pos: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}), '
            f'Dist to goal: {distance_to_goal:.2f}m'
        )


def main(args=None):
    """Main function to initialize and run the controller"""
    rclpy.init(args=args)
    controller = SimulationController()

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

## 4. Sensor Processing Examples

### 4.1 Laser Scan Processing Node

```python
#!/usr/bin/env python3
"""
Node for processing laser scan data from Gazebo simulation
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np


class ScanProcessor(Node):
    def __init__(self):
        super().__init__('scan_processor')

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Publishers
        self.obstacle_pub = self.create_publisher(Float32MultiArray, 'obstacle_distances', 10)
        self.free_space_pub = self.create_publisher(Float32MultiArray, 'free_space_segments', 10)

        # Parameters
        self.min_distance_threshold = 0.5  # meters
        self.angle_increment = 0.0174533  # 1 degree in radians

        self.get_logger().info('Scan Processor initialized')

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        # Convert ranges to numpy array
        ranges = np.array(msg.ranges)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min

        # Filter out invalid ranges
        valid_indices = (ranges > msg.range_min) & (ranges < msg.range_max)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]

        # Detect obstacles
        obstacle_indices = valid_ranges < self.min_distance_threshold
        obstacle_distances = valid_ranges[obstacle_indices]
        obstacle_angles = valid_angles[obstacle_indices]

        # Publish obstacle information
        obstacle_msg = Float32MultiArray()
        obstacle_msg.data = [float(len(obstacle_distances))] + obstacle_distances.tolist()
        self.obstacle_pub.publish(obstacle_msg)

        # Detect free space segments
        free_space_indices = valid_ranges >= self.min_distance_threshold
        free_space_ranges = valid_ranges[free_space_indices]
        free_space_angles = valid_angles[free_space_indices]

        # Group consecutive free space points
        free_segments = self.group_consecutive_points(free_space_ranges, free_space_angles)

        # Publish free space information
        free_space_msg = Float32MultiArray()
        free_space_msg.data = [float(len(free_segments))] + [item for sublist in free_segments for item in sublist]
        self.free_space_pub.publish(free_space_msg)

        # Log summary
        if len(obstacle_distances) > 0:
            self.get_logger().info(
                f'Detected {len(obstacle_distances)} obstacles, '
                f'min distance: {np.min(obstacle_distances):.2f}m'
            )
        else:
            self.get_logger().info('No obstacles detected')

    def group_consecutive_points(self, ranges, angles):
        """Group consecutive points into segments"""
        if len(ranges) == 0:
            return []

        segments = []
        current_segment = [(ranges[0], angles[0])]

        for i in range(1, len(ranges)):
            # Check if this point is close to the previous one in angle
            if abs(angles[i] - angles[i-1]) < 0.1:  # 0.1 rad = ~5.7 degrees
                current_segment.append((ranges[i], angles[i]))
            else:
                # Current segment ended, start new one
                if len(current_segment) > 2:  # Only keep segments with multiple points
                    segments.append(current_segment)
                current_segment = [(ranges[i], angles[i])]

        # Add the last segment
        if len(current_segment) > 2:
            segments.append(current_segment)

        return segments


def main(args=None):
    """Main function to initialize and run the scan processor"""
    rclpy.init(args=args)
    processor = ScanProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 5. Simulation Testing and Validation

### 5.1 Test Node for Simulation Validation

```python
#!/usr/bin/env python3
"""
Node for validating simulation behavior against expected results
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import time
import math


class SimulationValidator(Node):
    def __init__(self):
        super().__init__('simulation_validator')

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.test_result_pub = self.create_publisher(Float32, 'test_result', 10)

        # Internal state
        self.start_time = self.get_clock().now()
        self.start_pose = None
        self.current_pose = None
        self.test_stage = 0  # 0: not started, 1: moving forward, 2: turning, 3: completed
        self.test_completed = False

        # Test parameters
        self.forward_distance = 2.0  # meters
        self.turn_angle = math.pi / 2  # 90 degrees in radians
        self.test_duration = 30.0  # seconds

        # Test results
        self.test_passed = True
        self.error_count = 0

        # Timer for test execution
        self.test_timer = self.create_timer(0.1, self.test_execution)

        self.get_logger().info('Simulation Validator initialized')

    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose

        if self.start_pose is None:
            self.start_pose = msg.pose.pose
            self.get_logger().info('Test started - recorded initial pose')

    def test_execution(self):
        """Execute the validation test"""
        if self.current_pose is None or self.start_pose is None:
            return

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed_time > self.test_duration:
            self.test_failed("Test timed out")
            return

        cmd_vel = Twist()

        if self.test_stage == 0:  # Initialize test
            cmd_vel.linear.x = 0.5  # Start moving forward
            cmd_vel.angular.z = 0.0
            self.test_stage = 1
            self.get_logger().info('Test stage 1: Moving forward')

        elif self.test_stage == 1:  # Moving forward
            # Calculate distance traveled
            dx = self.current_pose.position.x - self.start_pose.position.x
            dy = self.current_pose.position.y - self.start_pose.position.y
            distance_traveled = math.sqrt(dx*dx + dy*dy)

            if distance_traveled >= self.forward_distance:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.5  # Start turning
                self.test_stage = 2
                self.get_logger().info('Test stage 2: Turning 90 degrees')
            else:
                cmd_vel.linear.x = 0.5
                cmd_vel.angular.z = 0.0

        elif self.test_stage == 2:  # Turning
            # Calculate current orientation change (simplified)
            # In a real test, you'd properly convert quaternions to angles
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5

            # For this example, just run turn for a fixed time
            time_in_stage = elapsed_time - self.get_time_at_stage(1)
            if time_in_stage > 3.0:  # Approximately 90 degrees at 0.5 rad/s
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.test_stage = 3
                self.test_completed = True
                self.get_logger().info('Test completed successfully')

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

        # Validate physics behavior
        self.validate_physics()

    def validate_physics(self):
        """Validate that physics are behaving as expected"""
        if self.current_pose is None:
            return

        # Check for unrealistic accelerations or velocities
        # This is a simplified check - in practice, you'd have more sophisticated validation

        # Example: Check if position is reasonable
        if abs(self.current_pose.position.x) > 100 or abs(self.current_pose.position.y) > 100:
            self.test_failed("Robot position is unrealistic")

        # Example: Check if robot is underground
        if self.current_pose.position.z < -0.1:  # Assuming ground is at z=0
            self.test_failed("Robot is below ground level")

    def test_failed(self, reason):
        """Handle test failure"""
        if not self.test_completed:
            self.test_completed = True
            self.test_passed = False
            self.get_logger().error(f'Test failed: {reason}')

            # Publish test result
            result_msg = Float32()
            result_msg.data = 0.0  # Failed
            self.test_result_pub.publish(result_msg)

    def get_time_at_stage(self, stage):
        """Get time when entering a specific stage (simplified)"""
        # This would be implemented with proper time tracking in a real test
        return 0.0


def main(args=None):
    """Main function to initialize and run the validator"""
    rclpy.init(args=args)
    validator = SimulationValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish final result
        if validator.test_completed:
            result_msg = Float32()
            result_msg.data = 1.0 if validator.test_passed else 0.0
            validator.test_result_pub.publish(result_msg)

        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 6. Advanced Simulation Features

### 6.1 Custom Gazebo Plugin (C++)

```cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace gazebo
{
  class PhysicalAISimulatorPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the model pointer for later use
      this->model = _model;

      // Initialize ROS if not already initialized
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
                 ros::init_options::NoSigintHandler);
      }

      // Create ROS node handle
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Subscribe to command velocity topic
      this->rosSub = this->rosNode->subscribe(
          "/" + this->model->GetName() + "/cmd_vel",
          1, &PhysicalAISimulatorPlugin::OnRosCmdVel, this);

      // Create ROS callback queue thread
      this->rosQueueThread =
          std::thread(std::bind(&PhysicalAISimulatorPlugin::QueueThread, this));

      // Listen to the update event (every simulation iteration)
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PhysicalAISimulatorPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      // Apply velocity to the model
      this->model->SetLinearVel(this->targetVel);
    }

    private: void OnRosCmdVel(const geometry_msgs::Twist::ConstPtr& _msg)
    {
      // Convert ROS message to Gazebo velocity
      this->targetVel.Set(_msg->linear.x, _msg->linear.y, 0);
    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    private: ignition::math::Vector3d targetVel;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PhysicalAISimulatorPlugin)
}
```

These code examples demonstrate fundamental Gazebo simulation concepts essential for Physical AI systems, including robot modeling, sensor simulation, ROS 2 integration, and simulation validation. Each example builds upon the previous ones to show how to create realistic, validated simulations for Physical AI applications.