# Code Examples: Humanoid Robotics and Locomotion

## 1. Humanoid Robot Control Framework

### 1.1 Humanoid Robot Model (URDF)

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="left_shin">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Leg -->
  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="right_shin">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
  </joint>

  <!-- Left Leg Joints -->
  <joint name="left_hip" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="-0.1 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <joint name="left_knee" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="100" velocity="1"/>
  </joint>

  <joint name="left_ankle" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <!-- Right Leg Joints -->
  <joint name="right_hip" type="revolute">
    <parent link="base_link"/>
    <child link="right_thigh"/>
    <origin xyz="0.1 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <joint name="right_knee" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="100" velocity="1"/>
  </joint>

  <joint name="right_ankle" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <!-- Gazebo extensions -->
  <gazebo reference="base_link">
    <material>Gazebo/Gray</material>
  </gazebo>

  <gazebo reference="torso">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="left_thigh">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="left_shin">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="left_foot">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="right_thigh">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_shin">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_foot">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- Joint state publisher -->
  <gazebo>
    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
      <robot_namespace>/humanoid</robot_namespace>
      <joint_name>left_hip</joint_name>
      <joint_name>left_knee</joint_name>
      <joint_name>left_ankle</joint_name>
      <joint_name>right_hip</joint_name>
      <joint_name>right_knee</joint_name>
      <joint_name>right_ankle</joint_name>
    </plugin>
  </gazebo>

  <!-- Joint position controllers -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid</robotNamespace>
    </plugin>
  </gazebo>

</robot>
```

### 1.2 Humanoid Robot Controller Node

```python
#!/usr/bin/env python3
"""
Humanoid robot controller for balance and locomotion
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float64MultiArray
import numpy as np
import math


class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Publishers
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.balance_status_pub = self.create_publisher(Float64MultiArray, '/balance_status', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Internal state
        self.joint_positions = {}
        self.joint_velocities = {}
        self.current_imu = Imu()
        self.desired_joints = {}

        # Balance control parameters
        self.balance_gain = 1.0
        self.com_height = 0.7  # Center of mass height
        self.gravity = 9.81

        # Walking parameters
        self.step_length = 0.2
        self.step_width = 0.1
        self.walk_speed = 0.5
        self.swing_height = 0.05

        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info('Humanoid Controller initialized')

    def joint_state_callback(self, msg):
        """Update joint state"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    def imu_callback(self, msg):
        """Update IMU data"""
        self.current_imu = msg

    def control_loop(self):
        """Main control loop"""
        # Calculate current CoM and ZMP
        com_pos = self.calculate_center_of_mass()
        zmp_pos = self.calculate_zero_moment_point(com_pos)

        # Check balance status
        balance_ok = self.check_balance(zmp_pos)

        # Generate walking pattern if walking
        if self.walk_speed > 0:
            walking_pattern = self.generate_walking_pattern()
        else:
            walking_pattern = self.generate_balancing_pattern()

        # Apply balance control
        balance_correction = self.balance_control(com_pos, zmp_pos)

        # Combine walking and balance commands
        final_commands = self.combine_commands(walking_pattern, balance_correction)

        # Publish commands
        self.publish_joint_commands(final_commands)

        # Publish balance status
        self.publish_balance_status(com_pos, zmp_pos, balance_ok)

    def calculate_center_of_mass(self):
        """Calculate center of mass position"""
        # Simplified CoM calculation based on joint positions
        # In practice, this would use a more sophisticated kinematic model
        com_x = 0.0
        com_y = 0.0
        com_z = self.com_height

        # Add contributions from different body parts
        # This is a simplified example - real implementation would use full kinematics
        if 'left_foot' in self.joint_positions:
            # Add foot position contribution
            pass

        return Point(x=com_x, y=com_y, z=com_z)

    def calculate_zero_moment_point(self, com_pos):
        """Calculate Zero Moment Point"""
        # Simplified ZMP calculation
        # ZMP_x = CoM_x - (CoM_height / g) * CoM_x_ddot
        # ZMP_y = CoM_y - (CoM_height / g) * CoM_y_ddot

        # For this example, we'll return a simplified calculation
        # In practice, this would use actual acceleration data
        zmp_x = com_pos.x  # Simplified
        zmp_y = com_pos.y  # Simplified

        return Point(x=zmp_x, y=zmp_y, z=0.0)

    def check_balance(self, zmp_pos):
        """Check if robot is balanced"""
        # Define support polygon (simplified as rectangle under feet)
        support_width = 0.2  # Width of support polygon
        support_length = 0.3  # Length of support polygon

        # Check if ZMP is within support polygon
        is_balanced = (
            abs(zmp_pos.x) < support_length / 2 and
            abs(zmp_pos.y) < support_width / 2
        )

        return is_balanced

    def generate_walking_pattern(self):
        """Generate walking gait pattern"""
        # Simplified walking pattern generation
        # In practice, this would use inverse kinematics and trajectory planning

        # Calculate step timing based on walk speed
        step_duration = self.step_length / self.walk_speed if self.walk_speed > 0 else 1.0

        # Current time for phase calculation
        current_time = self.get_clock().now().nanoseconds / 1e9
        phase = (current_time % step_duration) / step_duration

        # Generate joint commands for walking
        commands = Float64MultiArray()

        # Simplified walking gait
        left_hip = math.sin(phase * 2 * math.pi) * 0.1  # Hip swing
        right_hip = math.sin(phase * 2 * math.pi + math.pi) * 0.1  # Opposite hip

        left_knee = math.sin(phase * 2 * math.pi) * 0.05  # Knee bend
        right_knee = math.sin(phase * 2 * math.pi + math.pi) * 0.05  # Opposite knee

        # Swing foot trajectory (simplified)
        if phase < 0.5:  # Left foot swings
            left_ankle = math.sin(phase * 4 * math.pi) * self.swing_height
            right_ankle = 0.0
        else:  # Right foot swings
            left_ankle = 0.0
            right_ankle = math.sin((phase - 0.5) * 4 * math.pi) * self.swing_height

        # Create command structure
        commands.data = [
            left_hip, left_knee, left_ankle,
            right_hip, right_knee, right_ankle
        ]

        return commands

    def generate_balancing_pattern(self):
        """Generate balancing pattern when stationary"""
        # Return neutral positions for all joints
        commands = Float64MultiArray()
        commands.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Neutral positions
        return commands

    def balance_control(self, com_pos, zmp_pos):
        """Apply balance control corrections"""
        # Calculate error between desired and actual ZMP
        desired_zmp_x = 0.0  # Center of support polygon
        desired_zmp_y = 0.0

        zmp_error_x = desired_zmp_x - zmp_pos.x
        zmp_error_y = desired_zmp_y - zmp_pos.y

        # Generate corrective joint commands based on ZMP error
        correction_commands = Float64MultiArray()

        # Simple proportional control
        hip_correction_x = zmp_error_x * self.balance_gain
        hip_correction_y = zmp_error_y * self.balance_gain

        correction_commands.data = [
            hip_correction_x, 0.0, 0.0,  # Left leg correction
            hip_correction_y, 0.0, 0.0   # Right leg correction
        ]

        return correction_commands

    def combine_commands(self, walking_commands, balance_commands):
        """Combine walking and balance commands"""
        combined_commands = Float64MultiArray()

        # Add walking and balance commands together
        for i in range(min(len(walking_commands.data), len(balance_commands.data))):
            combined_commands.data.append(walking_commands.data[i] + balance_commands.data[i])

        # Add remaining commands if lengths differ
        if len(walking_commands.data) > len(balance_commands.data):
            combined_commands.data.extend(walking_commands.data[len(balance_commands.data):])
        elif len(balance_commands.data) > len(walking_commands.data):
            combined_commands.data.extend(balance_commands.data[len(walking_commands.data):])

        return combined_commands

    def publish_joint_commands(self, commands):
        """Publish joint commands to robot"""
        self.joint_cmd_pub.publish(commands)

    def publish_balance_status(self, com_pos, zmp_pos, is_balanced):
        """Publish balance status information"""
        status_msg = Float64MultiArray()
        status_msg.data = [
            com_pos.x, com_pos.y, com_pos.z,
            zmp_pos.x, zmp_pos.y, zmp_pos.z,
            float(is_balanced)
        ]
        self.balance_status_pub.publish(status_msg)


def main(args=None):
    """Main function to initialize and run the controller"""
    rclpy.init(args=args)
    controller = HumanoidController()

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

## 2. Balance Control Systems

### 2.1 ZMP-Based Balance Controller

```python
#!/usr/bin/env python3
"""
ZMP-based balance controller for humanoid robots
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Point, WrenchStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy import signal
import math


class ZMPBalanceController(Node):
    def __init__(self):
        super().__init__('zmp_balance_controller')

        # Publishers
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.zmp_pub = self.create_publisher(Point, '/zmp_estimate', 10)
        self.com_pub = self.create_publisher(Point, '/com_estimate', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.force_torque_sub = self.create_subscription(
            WrenchStamped,
            '/left_foot/force_torque',
            self.left_foot_force_callback,
            10
        )

        self.force_torque_sub_right = self.create_subscription(
            WrenchStamped,
            '/right_foot/force_torque',
            self.right_foot_force_callback,
            10
        )

        # Internal state
        self.joint_positions = {}
        self.joint_velocities = {}
        self.current_imu = Imu()
        self.left_foot_force = WrenchStamped()
        self.right_foot_force = WrenchStamped()

        # Robot parameters
        self.mass = 30.0  # kg
        self.com_height = 0.7  # m
        self.gravity = 9.81

        # Control parameters
        self.zmp_p_gain = 10.0
        self.zmp_d_gain = 2.0
        self.com_p_gain = 5.0
        self.com_d_gain = 1.0

        # Previous values for derivative calculation
        self.prev_zmp_error = Point()
        self.prev_com_error = Point()
        self.prev_time = self.get_clock().now()

        # Timer for control loop
        self.control_timer = self.create_timer(0.005, self.balance_control_loop)

        self.get_logger().info('ZMP Balance Controller initialized')

    def joint_state_callback(self, msg):
        """Update joint state"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    def imu_callback(self, msg):
        """Update IMU data"""
        self.current_imu = msg

    def left_foot_force_callback(self, msg):
        """Update left foot force/torque data"""
        self.left_foot_force = msg

    def right_foot_force_callback(self, msg):
        """Update right foot force/torque data"""
        self.right_foot_force = msg

    def balance_control_loop(self):
        """Main balance control loop"""
        # Calculate current CoM and ZMP
        current_com = self.calculate_center_of_mass()
        current_zmp = self.calculate_zero_moment_point()

        # Calculate desired CoM and ZMP
        desired_com = self.calculate_desired_com()
        desired_zmp = self.calculate_desired_zmp()

        # Calculate errors
        com_error = Point(
            x=desired_com.x - current_com.x,
            y=desired_com.y - current_com.y,
            z=desired_com.z - current_com.z
        )

        zmp_error = Point(
            x=desired_zmp.x - current_zmp.x,
            y=desired_zmp.y - current_zmp.y,
            z=0.0
        )

        # Calculate derivatives for PD control
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        if dt > 0:
            com_derivative = Point(
                x=(com_error.x - self.prev_com_error.x) / dt,
                y=(com_error.y - self.prev_com_error.y) / dt,
                z=(com_error.z - self.prev_com_error.z) / dt
            )

            zmp_derivative = Point(
                x=(zmp_error.x - self.prev_zmp_error.x) / dt,
                y=(zmp_error.y - self.prev_zmp_error.y) / dt,
                z=0.0
            )
        else:
            com_derivative = Point()
            zmp_derivative = Point()

        # Store current errors for next iteration
        self.prev_com_error = com_error
        self.prev_zmp_error = zmp_error

        # Apply PD control
        com_control = Point(
            x=self.com_p_gain * com_error.x + self.com_d_gain * com_derivative.x,
            y=self.com_p_gain * com_error.y + self.com_d_gain * com_derivative.y,
            z=self.com_p_gain * com_error.z + self.com_d_gain * com_derivative.z
        )

        zmp_control = Point(
            x=self.zmp_p_gain * zmp_error.x + self.zmp_d_gain * zmp_derivative.x,
            y=self.zmp_p_gain * zmp_error.y + self.zmp_d_gain * zmp_derivative.y,
            z=0.0
        )

        # Combine control efforts
        total_control = Point(
            x=com_control.x + zmp_control.x,
            y=com_control.y + zmp_control.y,
            z=com_control.z + zmp_control.z
        )

        # Generate joint commands based on control effort
        joint_commands = self.generate_joint_commands(total_control, current_com)

        # Publish results
        self.publish_results(current_com, current_zmp, joint_commands)

    def calculate_center_of_mass(self):
        """Calculate center of mass position using joint positions"""
        # This is a simplified calculation
        # In practice, this would use full kinematic model with link masses
        com_x = 0.0
        com_y = 0.0
        com_z = self.com_height

        # Example calculation using joint positions
        # This would be replaced with actual kinematic calculations
        if 'left_hip' in self.joint_positions:
            com_x += self.joint_positions['left_hip'] * 0.1
        if 'right_hip' in self.joint_positions:
            com_x += self.joint_positions['right_hip'] * 0.1

        return Point(x=com_x, y=com_y, z=com_z)

    def calculate_zero_moment_point(self):
        """Calculate Zero Moment Point from force/torque measurements"""
        # Calculate total forces and moments from both feet
        total_fx = self.left_foot_force.wrench.force.x + self.right_foot_force.wrench.force.x
        total_fy = self.left_foot_force.wrench.force.y + self.right_foot_force.wrench.force.y
        total_fz = self.left_foot_force.wrench.force.z + self.right_foot_force.wrench.force.z

        # Calculate moments about ankle (simplified)
        left_moment_x = self.left_foot_force.wrench.torque.x
        left_moment_y = self.left_foot_force.wrench.torque.y
        right_moment_x = self.right_foot_force.wrench.torque.x
        right_moment_y = self.right_foot_force.wrench.torque.y

        total_mx = left_moment_x + right_moment_x
        total_my = left_moment_y + right_moment_y

        # Calculate ZMP (when Fz is not zero)
        if abs(total_fz) > 0.1:  # Threshold to avoid division by zero
            zmp_x = -total_my / total_fz
            zmp_y = total_mx / total_fz
        else:
            zmp_x = 0.0
            zmp_y = 0.0

        return Point(x=zmp_x, y=zmp_y, z=0.0)

    def calculate_desired_com(self):
        """Calculate desired center of mass position"""
        # For now, return a simple desired position
        # In practice, this would come from walking pattern generator
        return Point(x=0.0, y=0.0, z=self.com_height)

    def calculate_desired_zmp(self):
        """Calculate desired Zero Moment Point"""
        # For balance, typically want ZMP at center of support polygon
        # For walking, follows pre-planned trajectory
        return Point(x=0.0, y=0.0, z=0.0)

    def generate_joint_commands(self, control_effort, current_com):
        """Generate joint commands based on control effort"""
        commands = Float64MultiArray()

        # Simplified joint command generation
        # In practice, this would use inverse kinematics or whole-body control

        # Map control effort to joint commands
        left_hip_cmd = control_effort.x * 0.5 + control_effort.y * 0.2
        right_hip_cmd = control_effort.x * 0.5 - control_effort.y * 0.2

        left_knee_cmd = -abs(control_effort.x) * 0.3
        right_knee_cmd = -abs(control_effort.x) * 0.3

        left_ankle_cmd = control_effort.y * 0.4
        right_ankle_cmd = -control_effort.y * 0.4

        commands.data = [
            left_hip_cmd, left_knee_cmd, left_ankle_cmd,
            right_hip_cmd, right_knee_cmd, right_ankle_cmd
        ]

        return commands

    def publish_results(self, com_pos, zmp_pos, joint_commands):
        """Publish control results"""
        # Publish CoM estimate
        self.com_pub.publish(com_pos)

        # Publish ZMP estimate
        self.zmp_pub.publish(zmp_pos)

        # Publish joint commands
        self.joint_cmd_pub.publish(joint_commands)

        # Log control status
        self.get_logger().info(
            f'CoM: ({com_pos.x:.3f}, {com_pos.y:.3f}), '
            f'ZMP: ({zmp_pos.x:.3f}, {zmp_pos.y:.3f})'
        )


def main(args=None):
    """Main function to initialize and run the ZMP controller"""
    rclpy.init(args=args)
    controller = ZMPBalanceController()

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

## 3. Walking Gait Generation

### 3.1 Inverted Pendulum Walking Generator

```python
#!/usr/bin/env python3
"""
Inverted pendulum based walking gait generator
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Point
import numpy as np
import math
from scipy import signal


class WalkingPatternGenerator(Node):
    def __init__(self):
        super().__init__('walking_pattern_generator')

        # Publishers
        self.trajectory_pub = self.create_publisher(Float64MultiArray, '/walking_trajectory', 10)
        self.step_status_pub = self.create_publisher(Bool, '/step_status', 10)

        # Subscribers
        self.com_sub = self.create_subscription(
            Point,
            '/com_estimate',
            self.com_callback,
            10
        )

        self.zmp_sub = self.create_subscription(
            Point,
            '/zmp_estimate',
            self.zmp_callback,
            10
        )

        # Internal state
        self.current_com = Point()
        self.current_zmp = Point()
        self.walk_enabled = False
        self.step_phase = 0.0
        self.step_count = 0

        # Walking parameters
        self.step_length = 0.2  # meters
        self.step_width = 0.15  # meters (distance between feet)
        self.step_height = 0.05  # meters (foot clearance)
        self.step_duration = 1.0  # seconds
        self.dsp_ratio = 0.2  # Double Support Phase ratio
        self.walk_speed = 0.0  # m/s

        # Inverted pendulum parameters
        self.pendulum_height = 0.7  # CoM height
        self.omega = math.sqrt(self.gravity / self.pendulum_height)

        # Support polygon
        self.support_polygon = {
            'left': {'x': 0.0, 'y': self.step_width/2},
            'right': {'x': 0.0, 'y': -self.step_width/2}
        }

        # Timer for walking pattern generation
        self.pattern_timer = self.create_timer(0.01, self.generate_walking_pattern)

        self.get_logger().info('Walking Pattern Generator initialized')

    def com_callback(self, msg):
        """Update CoM estimate"""
        self.current_com = msg

    def zmp_callback(self, msg):
        """Update ZMP estimate"""
        self.current_zmp = msg

    def generate_walking_pattern(self):
        """Generate walking pattern using inverted pendulum model"""
        if not self.walk_enabled:
            return

        # Update step phase
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.step_phase = (current_time / self.step_duration) % 1.0

        # Determine support foot based on step phase
        is_left_support = self.is_left_foot_supporting()

        # Generate foot trajectories
        left_foot_pos = self.calculate_foot_trajectory('left', is_left_support)
        right_foot_pos = self.calculate_foot_trajectory('right', not is_left_support)

        # Generate CoM trajectory using inverted pendulum model
        com_trajectory = self.calculate_com_trajectory()

        # Generate joint commands for the walking pattern
        joint_commands = self.calculate_joint_commands(
            left_foot_pos, right_foot_pos, com_trajectory
        )

        # Publish the walking trajectory
        self.trajectory_pub.publish(joint_commands)

        # Publish step status
        step_msg = Bool()
        step_msg.data = True
        self.step_status_pub.publish(step_msg)

    def is_left_foot_supporting(self):
        """Determine if left foot is in support phase"""
        # In a simple alternating pattern
        # Left foot supports during first half of step cycle
        return self.step_phase < 0.5

    def calculate_foot_trajectory(self, foot_side, is_supporting):
        """Calculate foot trajectory for a given foot"""
        foot_pos = Point()

        if is_supporting:
            # Supporting foot stays in place
            if foot_side == 'left':
                foot_pos.x = self.step_count * self.step_length
                foot_pos.y = self.step_width / 2
                foot_pos.z = 0.0
            else:  # right foot
                foot_pos.x = self.step_count * self.step_length
                foot_pos.y = -self.step_width / 2
                foot_pos.z = 0.0
        else:
            # Swing foot trajectory
            if foot_side == 'left':
                # Left foot swings forward
                phase_offset = self.step_phase if self.step_count % 2 == 0 else self.step_phase + 0.5
                phase_offset %= 1.0

                # Calculate swing trajectory
                foot_pos.x = self.step_count * self.step_length + self.step_length * phase_offset
                foot_pos.y = self.step_width / 2  # Stay on centerline during swing

                # Vertical trajectory (parabolic for foot clearance)
                swing_phase = phase_offset * 2 if phase_offset < 0.5 else (1 - phase_offset) * 2
                foot_pos.z = self.step_height * math.sin(math.pi * swing_phase)
            else:
                # Right foot swings forward
                phase_offset = self.step_phase + 0.5 if self.step_count % 2 == 0 else self.step_phase
                phase_offset %= 1.0

                # Calculate swing trajectory
                foot_pos.x = self.step_count * self.step_length + self.step_length * phase_offset
                foot_pos.y = -self.step_width / 2  # Stay on centerline during swing

                # Vertical trajectory (parabolic for foot clearance)
                swing_phase = phase_offset * 2 if phase_offset < 0.5 else (1 - phase_offset) * 2
                foot_pos.z = self.step_height * math.sin(math.pi * swing_phase)

        return foot_pos

    def calculate_com_trajectory(self):
        """Calculate CoM trajectory using inverted pendulum model"""
        # Use preview control to generate CoM trajectory
        # This is a simplified version - full implementation would use
        # model predictive control with preview of future ZMP reference

        com_ref = Point()

        # Simple sinusoidal CoM trajectory to maintain balance
        # In practice, this would be generated using ZMP preview control

        # Lateral CoM movement to maintain balance over supporting foot
        support_side = 'left' if self.is_left_foot_supporting() else 'right'
        target_y = self.step_width / 2 if support_side == 'left' else -self.step_width / 2

        # Add small oscillation for natural walking motion
        oscillation = 0.01 * math.sin(2 * math.pi * self.step_phase)
        com_ref.y = target_y + oscillation

        # Forward CoM movement following the step
        com_ref.x = self.step_count * self.step_length + self.step_length * self.step_phase * 0.9

        # Maintain relatively constant height
        com_ref.z = self.pendulum_height

        return com_ref

    def calculate_joint_commands(self, left_foot_pos, right_foot_pos, com_trajectory):
        """Calculate joint commands from desired foot and CoM positions"""
        commands = Float64MultiArray()

        # This is a simplified kinematic mapping
        # In practice, this would use inverse kinematics solver

        # Map desired foot positions to joint angles
        # Using simplified sagittal plane model for demonstration

        # Left leg commands
        left_hip_angle = math.atan2(left_foot_pos.z, left_foot_pos.x) if left_foot_pos.x != 0 else 0
        left_knee_angle = math.pi / 3  # Fixed knee bend for simplicity
        left_ankle_angle = -left_hip_angle - left_knee_angle  # Keep foot level

        # Right leg commands
        right_hip_angle = math.atan2(right_foot_pos.z, right_foot_pos.x) if right_foot_pos.x != 0 else 0
        right_knee_angle = math.pi / 3  # Fixed knee bend for simplicity
        right_ankle_angle = -right_hip_angle - right_knee_angle  # Keep foot level

        commands.data = [
            left_hip_angle, left_knee_angle, left_ankle_angle,
            right_hip_angle, right_knee_angle, right_ankle_angle
        ]

        return commands

    def start_walking(self):
        """Start the walking pattern generation"""
        self.walk_enabled = True
        self.get_logger().info('Walking started')

    def stop_walking(self):
        """Stop the walking pattern generation"""
        self.walk_enabled = False
        self.get_logger().info('Walking stopped')

    def set_walk_parameters(self, step_length, step_height, step_duration, walk_speed):
        """Set walking parameters"""
        self.step_length = step_length
        self.step_height = step_height
        self.step_duration = step_duration
        self.walk_speed = walk_speed
        self.get_logger().info(
            f'Walk parameters updated: step_len={step_length}, '
            f'step_dur={step_duration}, speed={walk_speed}'
        )


def main(args=None):
    """Main function to initialize and run the walking pattern generator"""
    rclpy.init(args=args)
    walker = WalkingPatternGenerator()

    try:
        # Start walking after initialization
        walker.start_walking()
        rclpy.spin(walker)
    except KeyboardInterrupt:
        walker.stop_walking()
        pass
    finally:
        walker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 4. AI-Based Locomotion Control

### 4.1 Reinforcement Learning for Gait Optimization

```python
#!/usr/bin/env python3
"""
Reinforcement Learning for gait optimization in humanoid robots
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
import math
import random
from collections import deque


class RLGaitOptimizer(Node):
    def __init__(self):
        super().__init__('rl_gait_optimizer')

        # Publishers
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.reward_pub = self.create_publisher(Float64MultiArray, '/rl_reward', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.com_sub = self.create_subscription(
            Point,
            '/com_estimate',
            self.com_callback,
            10
        )

        # Internal state
        self.joint_positions = {}
        self.joint_velocities = {}
        self.current_imu = Imu()
        self.current_com = Point()
        self.episode_step = 0
        self.total_reward = 0.0

        # RL parameters
        self.learning_rate = 0.001
        self.discount_factor = 0.99
        self.epsilon = 1.0  # Exploration rate
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.01

        # Action space (simplified gait parameters)
        # Actions: [step_length_change, step_width_change, step_height_change, timing_change]
        self.action_space = 4
        self.action_bounds = [-0.1, 0.1]  # Bounds for each action

        # State space (simplified)
        # State: [com_x, com_y, com_z, roll, pitch, yaw, com_vel_x, com_vel_y, com_vel_z]
        self.state_space = 9

        # Q-Network weights (simplified linear approximation)
        self.q_weights = np.random.randn(self.state_space, self.action_space) * 0.1

        # Replay buffer
        self.replay_buffer = deque(maxlen=10000)

        # Current gait parameters
        self.base_step_length = 0.2
        self.base_step_width = 0.15
        self.base_step_height = 0.05
        self.base_timing = 1.0

        # Timer for RL control loop
        self.rl_timer = self.create_timer(0.05, self.rl_control_loop)

        self.get_logger().info('RL Gait Optimizer initialized')

    def joint_state_callback(self, msg):
        """Update joint state"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    def imu_callback(self, msg):
        """Update IMU data"""
        self.current_imu = msg

    def com_callback(self, msg):
        """Update CoM estimate"""
        self.current_com = msg

    def rl_control_loop(self):
        """Main RL control loop"""
        # Get current state
        state = self.get_state()

        # Select action using epsilon-greedy policy
        action = self.select_action(state)

        # Apply action to modify gait parameters
        self.apply_gait_modification(action)

        # Calculate reward
        reward = self.calculate_reward()

        # Get next state
        next_state = self.get_state()

        # Store experience in replay buffer
        self.replay_buffer.append((state, action, reward, next_state))

        # Update Q-network (simplified)
        self.update_q_network(state, action, reward, next_state)

        # Publish reward
        reward_msg = Float64MultiArray()
        reward_msg.data = [reward, self.total_reward]
        self.reward_pub.publish(reward_msg)

        # Update episode counters
        self.episode_step += 1
        self.total_reward += reward

        # Decay exploration rate
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

        self.get_logger().info(
            f'Step: {self.episode_step}, Reward: {reward:.3f}, '
            f'Total: {self.total_reward:.3f}, Epsilon: {self.epsilon:.3f}'
        )

    def get_state(self):
        """Get current state vector"""
        # Simplified state representation
        state = np.zeros(self.state_space)

        # CoM position
        state[0] = self.current_com.x
        state[1] = self.current_com.y
        state[2] = self.current_com.z

        # IMU orientation (simplified)
        state[3] = self.current_imu.orientation.x  # roll component
        state[4] = self.current_imu.orientation.y  # pitch component
        state[5] = self.current_imu.orientation.z  # yaw component

        # Assume CoM velocities (would need proper differentiation in practice)
        state[6] = 0.0  # com_vel_x
        state[7] = 0.0  # com_vel_y
        state[8] = 0.0  # com_vel_z

        return state

    def select_action(self, state):
        """Select action using epsilon-greedy policy"""
        if random.random() < self.epsilon:
            # Explore: random action
            action_idx = random.randint(0, self.action_space - 1)
            action_value = random.uniform(*self.action_bounds)
        else:
            # Exploit: best action according to Q-network
            q_values = np.dot(state, self.q_weights)
            action_idx = np.argmax(q_values)
            action_value = q_values[action_idx]

        # Create action vector
        action = np.zeros(self.action_space)
        action[action_idx] = action_value

        return action

    def apply_gait_modification(self, action):
        """Apply gait parameter modifications based on action"""
        # Modify gait parameters based on action
        step_length_change = action[0] if len(action) > 0 else 0.0
        step_width_change = action[1] if len(action) > 1 else 0.0
        step_height_change = action[2] if len(action) > 2 else 0.0
        timing_change = action[3] if len(action) > 3 else 0.0

        # Apply changes with bounds checking
        new_step_length = np.clip(
            self.base_step_length + step_length_change,
            0.1, 0.4
        )
        new_step_width = np.clip(
            self.base_step_width + step_width_change,
            0.1, 0.3
        )
        new_step_height = np.clip(
            self.base_step_height + step_height_change,
            0.02, 0.1
        )
        new_timing = np.clip(
            self.base_timing + timing_change,
            0.5, 2.0
        )

        # Update gait parameters
        self.base_step_length = new_step_length
        self.base_step_width = new_step_width
        self.base_step_height = new_step_height
        self.base_timing = new_timing

    def calculate_reward(self):
        """Calculate reward based on walking performance"""
        reward = 0.0

        # Reward for forward progress
        forward_progress = self.current_com.x
        reward += forward_progress * 10.0

        # Penalty for falling (deviation from upright)
        roll_deviation = abs(self.current_imu.orientation.x)
        pitch_deviation = abs(self.current_imu.orientation.y)
        balance_penalty = (roll_deviation + pitch_deviation) * 5.0
        reward -= balance_penalty

        # Penalty for CoM deviation from center
        lateral_deviation = abs(self.current_com.y)
        center_penalty = lateral_deviation * 20.0
        reward -= center_penalty

        # Reward for smooth motion (low joint velocities)
        if self.joint_velocities:
            avg_velocity = np.mean(list(self.joint_velocities.values()))
            smoothness_reward = max(0, 1.0 - abs(avg_velocity))
            reward += smoothness_reward * 5.0

        # Small time penalty to encourage efficiency
        reward -= 0.01

        return reward

    def update_q_network(self, state, action, reward, next_state):
        """Update Q-network weights (simplified)"""
        # Convert action to index (find the action with maximum value)
        action_idx = np.argmax(np.abs(action))

        # Calculate target Q-value
        current_q = np.dot(state, self.q_weights)[action_idx]
        next_max_q = np.max(np.dot(next_state, self.q_weights))
        target_q = reward + self.discount_factor * next_max_q

        # Calculate temporal difference error
        td_error = target_q - current_q

        # Update weights using gradient descent
        self.q_weights[:, action_idx] += self.learning_rate * td_error * state

    def reset_episode(self):
        """Reset episode counters"""
        self.episode_step = 0
        self.total_reward = 0.0


def main(args=None):
    """Main function to initialize and run the RL optimizer"""
    rclpy.init(args=args)
    optimizer = RLGaitOptimizer()

    try:
        rclpy.spin(optimizer)
    except KeyboardInterrupt:
        pass
    finally:
        optimizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 5. Simulation Integration

### 5.1 Humanoid Robot Simulator Node

```python
#!/usr/bin/env python3
"""
Humanoid robot simulator with physics-based control
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float64MultiArray
import numpy as np
import math


class HumanoidSimulator(Node):
    def __init__(self):
        super().__init__('humanoid_simulator')

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.joint_command_callback,
            10
        )

        # Internal state
        self.current_joint_positions = {
            'left_hip': 0.0, 'left_knee': 0.0, 'left_ankle': 0.0,
            'right_hip': 0.0, 'right_knee': 0.0, 'right_ankle': 0.0
        }

        self.current_joint_velocities = {key: 0.0 for key in self.current_joint_positions}
        self.commanded_joint_positions = self.current_joint_positions.copy()

        # Robot state
        self.com_position = Point(x=0.0, y=0.0, z=0.7)
        self.com_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        self.orientation = Vector3(x=0.0, y=0.0, z=0.0)  # roll, pitch, yaw
        self.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)

        # Physics parameters
        self.simulation_dt = 0.001  # 1ms simulation timestep
        self.integration_steps = 10  # Number of integration steps per control cycle

        # Control parameters
        self.control_dt = 0.01  # 10ms control timestep
        self.joint_stiffness = 100.0  # N*m/rad
        self.joint_damping = 10.0     # N*m*s/rad

        # Timer for simulation
        self.sim_timer = self.create_timer(self.simulation_dt, self.simulation_step)
        self.pub_timer = self.create_timer(self.control_dt, self.publish_state)

        self.get_logger().info('Humanoid Simulator initialized')

    def joint_command_callback(self, msg):
        """Update joint commands"""
        joint_names = [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle'
        ]

        if len(msg.data) >= len(joint_names):
            for i, name in enumerate(joint_names):
                self.commanded_joint_positions[name] = msg.data[i]

    def simulation_step(self):
        """Perform one simulation step"""
        # Update joint positions based on commands (with compliance)
        for joint_name in self.current_joint_positions:
            commanded_pos = self.commanded_joint_positions[joint_name]
            current_pos = self.current_joint_positions[joint_name]

            # Simple spring-damper model for joint compliance
            restoring_force = self.joint_stiffness * (commanded_pos - current_pos)
            damping_force = self.joint_damping * (0 - self.current_joint_velocities[joint_name])

            # Calculate acceleration
            # For simplicity, assuming unit inertia for each joint
            angular_acceleration = restoring_force + damping_force

            # Update velocity and position using Euler integration
            self.current_joint_velocities[joint_name] += angular_acceleration * self.simulation_dt
            self.current_joint_positions[joint_name] += self.current_joint_velocities[joint_name] * self.simulation_dt

        # Update robot dynamics (simplified physics)
        self.update_robot_dynamics()

    def update_robot_dynamics(self):
        """Update robot dynamics based on joint positions"""
        # Simplified dynamics update
        # In a real simulator, this would involve complex multibody dynamics

        # Calculate approximate CoM based on joint positions
        self.update_com_position()

        # Calculate orientation based on joint configuration
        self.update_orientation()

        # Calculate velocity based on position changes
        # This would be more accurately computed with proper differentiation

    def update_com_position(self):
        """Update CoM position based on joint configuration"""
        # Simplified CoM calculation
        # In reality, this would use full kinematic model with link masses

        # Base CoM position
        self.com_position.x = 0.0
        self.com_position.y = 0.0
        self.com_position.z = 0.7  # Default standing height

        # Add influence from joint positions
        # This is a very simplified model
        hip_influence = 0.05
        knee_influence = 0.02
        ankle_influence = 0.01

        # Left leg influence
        self.com_position.x += (
            self.current_joint_positions['left_hip'] * hip_influence +
            self.current_joint_positions['left_knee'] * knee_influence +
            self.current_joint_positions['left_ankle'] * ankle_influence
        )

        # Right leg influence
        self.com_position.x += (
            self.current_joint_positions['right_hip'] * hip_influence +
            self.current_joint_positions['right_knee'] * knee_influence +
            self.current_joint_positions['right_ankle'] * ankle_influence
        ) / 2  # Average influence from both legs

    def update_orientation(self):
        """Update robot orientation based on joint configuration"""
        # Simplified orientation update
        # In reality, this would involve complex kinematic calculations

        # Calculate approximate roll and pitch based on leg configuration
        left_leg_effect = (
            self.current_joint_positions['left_hip'] +
            self.current_joint_positions['left_knee'] +
            self.current_joint_positions['left_ankle']
        )

        right_leg_effect = (
            self.current_joint_positions['right_hip'] +
            self.current_joint_positions['right_knee'] +
            self.current_joint_positions['right_ankle']
        )

        # Calculate roll (lean side to side)
        self.orientation.x = (right_leg_effect - left_leg_effect) * 0.1

        # Calculate pitch (lean forward/backward)
        self.orientation.y = (
            self.current_joint_positions['left_hip'] +
            self.current_joint_positions['right_hip']
        ) * 0.05

    def publish_state(self):
        """Publish current state to ROS topics"""
        # Publish joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = 'base_link'

        joint_state_msg.name = list(self.current_joint_positions.keys())
        joint_state_msg.position = list(self.current_joint_positions.values())
        joint_state_msg.velocity = list(self.current_joint_velocities.values())

        # Calculate effort (torque) based on spring-damper model
        effort = []
        for joint_name in self.current_joint_positions:
            pos_error = self.commanded_joint_positions[joint_name] - self.current_joint_positions[joint_name]
            vel = self.current_joint_velocities[joint_name]
            torque = self.joint_stiffness * pos_error - self.joint_damping * vel
            effort.append(torque)

        joint_state_msg.effort = effort
        self.joint_state_pub.publish(joint_state_msg)

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Convert roll, pitch, yaw to quaternion
        # Using a simplified conversion for demonstration
        roll, pitch, yaw = self.orientation.x, self.orientation.y, self.orientation.z

        # Convert Euler to Quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy

        # Angular velocity (simplified)
        imu_msg.angular_velocity.x = self.angular_velocity.x
        imu_msg.angular_velocity.y = self.angular_velocity.y
        imu_msg.angular_velocity.z = self.angular_velocity.z

        # Linear acceleration (simplified)
        imu_msg.linear_acceleration.x = self.com_velocity.x / self.control_dt
        imu_msg.linear_acceleration.y = self.com_velocity.y / self.control_dt
        imu_msg.linear_acceleration.z = (self.com_velocity.z - 9.81) / self.control_dt  # Subtract gravity

        self.imu_pub.publish(imu_msg)

        # Log state occasionally
        if self.get_clock().now().nanoseconds % 1000000000 < 1000000:  # Every second approximately
            self.get_logger().info(
                f'CoM: ({self.com_position.x:.3f}, {self.com_position.y:.3f}, {self.com_position.z:.3f}), '
                f'Orient: ({math.degrees(self.orientation.x):.1f}°, {math.degrees(self.orientation.y):.1f}°)'
            )


def main(args=None):
    """Main function to initialize and run the simulator"""
    rclpy.init(args=args)
    simulator = HumanoidSimulator()

    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

These code examples demonstrate fundamental humanoid robotics concepts essential for Physical AI systems, including robot modeling, balance control using ZMP, walking gait generation using inverted pendulum models, AI-based locomotion optimization, and simulation integration. Each example builds upon the previous ones to show how to create stable, balanced humanoid robots capable of bipedal locomotion.