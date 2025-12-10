# Lab Exercises: Humanoid Robotics and Locomotion

## Lab 5.1: Humanoid Robot Model Construction and Simulation

### Objective
Construct a humanoid robot model using URDF, simulate it in Gazebo, and verify its basic functionality and joint configurations.

### Prerequisites
- Completed Modules 1-4 (Physical AI, ROS 2, Gazebo, NVIDIA Isaac)
- Understanding of URDF and robot modeling
- Gazebo simulation experience

### Estimated Time
3 hours

### Steps

#### Step 1: Create Humanoid Robot Package
1. Navigate to your workspace:
   ```bash
   cd ~/humanoid_ws/src
   ```

2. Create a new package for the humanoid robot:
   ```bash
   ros2 pkg create --build-type ament_python humanoid_robot_description --dependencies urdf xacro
   ```

3. Navigate to the package directory:
   ```bash
   cd humanoid_robot_description
   ```

4. Create the directory structure:
   ```bash
   mkdir -p urdf meshes launch config
   ```

#### Step 2: Create Humanoid Robot URDF Model
1. Create a humanoid robot URDF file `humanoid_robot_description/urdf/humanoid_robot.urdf.xacro`:
   ```xml
   <?xml version="1.0"?>
   <robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

     <!-- Constants -->
     <xacro:property name="pi" value="3.1415926535897931" />
     <xacro:property name="mass_body" value="20.0" />
     <xacro:property name="mass_limb" value="5.0" />
     <xacro:property name="mass_foot" value="2.0" />

     <!-- Materials -->
     <material name="black">
       <color rgba="0.0 0.0 0.0 1.0"/>
     </material>
     <material name="blue">
       <color rgba="0.0 0.0 0.8 1.0"/>
     </material>
     <material name="green">
       <color rgba="0.0 0.8 0.0 1.0"/>
     </material>
     <material name="grey">
       <color rgba="0.5 0.5 0.5 1.0"/>
     </material>
     <material name="orange">
       <color rgba="1.0 0.4235294117647059 0.0392156862745098 1.0"/>
     </material>
     <material name="brown">
       <color rgba="0.8705882352941177 0.8117647058823529 0.7647058823529411 1.0"/>
     </material>
     <material name="red">
       <color rgba="0.8 0.0 0.0 1.0"/>
     </material>
     <material name="white">
       <color rgba="1.0 1.0 1.0 1.0"/>
     </material>

     <!-- Base link -->
     <link name="base_link">
       <visual>
         <geometry>
           <cylinder length="0.6" radius="0.1"/>
         </geometry>
         <material name="grey"/>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.6" radius="0.1"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="${mass_body}"/>
         <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
       </inertial>
     </link>

     <!-- Torso -->
     <link name="torso">
       <visual>
         <geometry>
           <box size="0.3 0.2 0.5"/>
         </geometry>
         <material name="blue"/>
       </visual>
       <collision>
         <geometry>
           <box size="0.3 0.2 0.5"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="${mass_body}"/>
         <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5"/>
       </inertial>
     </link>

     <!-- Head -->
     <link name="head">
       <visual>
         <geometry>
           <sphere radius="0.1"/>
         </geometry>
         <material name="white"/>
       </visual>
       <collision>
         <geometry>
           <sphere radius="0.1"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="2.0"/>
         <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
       </inertial>
     </link>

     <!-- Left Leg -->
     <link name="left_thigh">
       <visual>
         <geometry>
           <cylinder length="0.4" radius="0.05"/>
         </geometry>
         <material name="red"/>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.4" radius="0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="${mass_limb}"/>
         <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.0025"/>
       </inertial>
     </link>

     <link name="left_shin">
       <visual>
         <geometry>
           <cylinder length="0.4" radius="0.05"/>
         </geometry>
         <material name="red"/>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.4" radius="0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="${mass_limb}"/>
         <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.0025"/>
       </inertial>
     </link>

     <link name="left_foot">
       <visual>
         <geometry>
           <box size="0.2 0.1 0.05"/>
         </geometry>
         <material name="black"/>
       </visual>
       <collision>
         <geometry>
           <box size="0.2 0.1 0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="${mass_foot}"/>
         <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
       </inertial>
     </link>

     <!-- Right Leg -->
     <link name="right_thigh">
       <visual>
         <geometry>
           <cylinder length="0.4" radius="0.05"/>
         </geometry>
         <material name="red"/>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.4" radius="0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="${mass_limb}"/>
         <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.0025"/>
       </inertial>
     </link>

     <link name="right_shin">
       <visual>
         <geometry>
           <cylinder length="0.4" radius="0.05"/>
         </geometry>
         <material name="red"/>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.4" radius="0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="${mass_limb}"/>
         <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.0025"/>
       </inertial>
     </link>

     <link name="right_foot">
       <visual>
         <geometry>
           <box size="0.2 0.1 0.05"/>
         </geometry>
         <material name="black"/>
       </visual>
       <collision>
         <geometry>
           <box size="0.2 0.1 0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="${mass_foot}"/>
         <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
       </inertial>
     </link>

     <!-- Left Arm -->
     <link name="left_upper_arm">
       <visual>
         <geometry>
           <cylinder length="0.3" radius="0.04"/>
         </geometry>
         <material name="green"/>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.3" radius="0.04"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1.5"/>
         <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
       </inertial>
     </link>

     <link name="left_lower_arm">
       <visual>
         <geometry>
           <cylinder length="0.3" radius="0.03"/>
         </geometry>
         <material name="green"/>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.3" radius="0.03"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1.0"/>
         <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
       </inertial>
     </link>

     <!-- Right Arm -->
     <link name="right_upper_arm">
       <visual>
         <geometry>
           <cylinder length="0.3" radius="0.04"/>
         </geometry>
         <material name="green"/>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.3" radius="0.04"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1.5"/>
         <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
       </inertial>
     </link>

     <link name="right_lower_arm">
       <visual>
         <geometry>
           <cylinder length="0.3" radius="0.03"/>
         </geometry>
         <material name="green"/>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.3" radius="0.03"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1.0"/>
         <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
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
       <limit lower="${-pi/2}" upper="${pi/2}" effort="100" velocity="1"/>
     </joint>

     <joint name="left_knee" type="revolute">
       <parent link="left_thigh"/>
       <child link="left_shin"/>
       <origin xyz="0 0 -0.4" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="0" upper="${pi/2}" effort="100" velocity="1"/>
     </joint>

     <joint name="left_ankle" type="revolute">
       <parent link="left_shin"/>
       <child link="left_foot"/>
       <origin xyz="0 0 -0.4" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-pi/4}" upper="${pi/4}" effort="50" velocity="1"/>
     </joint>

     <!-- Right Leg Joints -->
     <joint name="right_hip" type="revolute">
       <parent link="base_link"/>
       <child link="right_thigh"/>
       <origin xyz="0.1 0 -0.1" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-pi/2}" upper="${pi/2}" effort="100" velocity="1"/>
     </joint>

     <joint name="right_knee" type="revolute">
       <parent link="right_thigh"/>
       <child link="right_shin"/>
       <origin xyz="0 0 -0.4" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="0" upper="${pi/2}" effort="100" velocity="1"/>
     </joint>

     <joint name="right_ankle" type="revolute">
       <parent link="right_shin"/>
       <child link="right_foot"/>
       <origin xyz="0 0 -0.4" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-pi/4}" upper="${pi/4}" effort="50" velocity="1"/>
     </joint>

     <!-- Left Arm Joints -->
     <joint name="left_shoulder" type="revolute">
       <parent link="torso"/>
       <child link="left_upper_arm"/>
       <origin xyz="0.1 0.1 0.1" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-pi/2}" upper="${pi/2}" effort="50" velocity="1"/>
     </joint>

     <joint name="left_elbow" type="revolute">
       <parent link="left_upper_arm"/>
       <child link="left_lower_arm"/>
       <origin xyz="0 0 -0.3" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-pi/2}" upper="0" effort="30" velocity="1"/>
     </joint>

     <!-- Right Arm Joints -->
     <joint name="right_shoulder" type="revolute">
       <parent link="torso"/>
       <child link="right_upper_arm"/>
       <origin xyz="0.1 -0.1 0.1" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-pi/2}" upper="${pi/2}" effort="50" velocity="1"/>
     </joint>

     <joint name="right_elbow" type="revolute">
       <parent link="right_upper_arm"/>
       <child link="right_lower_arm"/>
       <origin xyz="0 0 -0.3" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-pi/2}" upper="0" effort="30" velocity="1"/>
     </joint>

     <!-- Gazebo extensions -->
     <gazebo reference="base_link">
       <material>Gazebo/Grey</material>
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

     <gazebo reference="left_upper_arm">
       <material>Gazebo/Green</material>
     </gazebo>

     <gazebo reference="left_lower_arm">
       <material>Gazebo/Green</material>
     </gazebo>

     <gazebo reference="right_upper_arm">
       <material>Gazebo/Green</material>
     </gazebo>

     <gazebo reference="right_lower_arm">
       <material>Gazebo/Green</material>
     </gazebo>

     <!-- Gazebo plugins -->
     <gazebo>
       <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
         <robot_namespace>/humanoid</robot_namespace>
         <joint_name>left_hip</joint_name>
         <joint_name>left_knee</joint_name>
         <joint_name>left_ankle</joint_name>
         <joint_name>right_hip</joint_name>
         <joint_name>right_knee</joint_name>
         <joint_name>right_ankle</joint_name>
         <joint_name>left_shoulder</joint_name>
         <joint_name>left_elbow</joint_name>
         <joint_name>right_shoulder</joint_name>
         <joint_name>right_elbow</joint_name>
       </plugin>
     </gazebo>

     <gazebo>
       <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
         <robotNamespace>/humanoid</robotNamespace>
       </plugin>
     </gazebo>

   </robot>
   ```

#### Step 3: Create Robot State Publisher Launch File
1. Create a launch file `humanoid_robot_description/launch/display.launch.py`:
   ```python
   #!/usr/bin/env python3
   """
   Launch file to display the humanoid robot model
   """
   import os
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory


   def generate_launch_description():
       # Launch arguments
       model_arg = DeclareLaunchArgument(
           'model',
           default_value='humanoid_robot.urdf.xacro',
           description='Robot model file'
       )

       # Robot state publisher
       robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           name='robot_state_publisher',
           output='screen',
           parameters=[{
               'use_sim_time': False
           }],
           arguments=[
               os.path.join(
                   get_package_share_directory('humanoid_robot_description'),
                   'urdf',
                   LaunchConfiguration('model')
               )
           ]
       )

       # Joint state publisher GUI
       joint_state_publisher_gui = Node(
           package='joint_state_publisher_gui',
           executable='joint_state_publisher_gui',
           name='joint_state_publisher_gui',
           output='screen'
       )

       # RViz2
       rviz = Node(
           package='rviz2',
           executable='rviz2',
           name='rviz2',
           output='screen',
           arguments=[
               '-d', os.path.join(
                   get_package_share_directory('humanoid_robot_description'),
                   'config',
                   'view_humanoid.rviz'
               )
           ]
       )

       return LaunchDescription([
           model_arg,
           robot_state_publisher,
           joint_state_publisher_gui,
           rviz
       ])
   ```

2. Create a config directory and RViz configuration:
   ```bash
   mkdir -p humanoid_robot_description/config
   ```

3. Create `humanoid_robot_description/config/view_humanoid.rviz`:
   ```yaml
   Panels:
     - Class: rviz_common/Displays
       Name: Displays
     - Class: rviz_common/Views
       Name: Views
     - Class: rviz_common/Selection
       Name: Selection
   Visualization Manager:
     Displays:
       - Alpha: 0.5
         Cell Size: 1
         Class: rviz_default_plugins/Grid
         Name: Grid
         Plane: XY
         Plane Cell Count: 10
         Reference Frame: <Fixed Frame>
         Value: true
       - Class: rviz_default_plugins/RobotModel
         Description Topic:
           Value: /robot_description
         Enabled: true
         Name: RobotModel
         TF Prefix: ""
         Update Interval: 0
         Value: true
         Visual Enabled: true
       - Class: rviz_default_plugins/TF
         Enabled: true
         Name: TF
         Marker Scale: 0.5
         Show Arrows: true
         Show Names: true
         Value: true
     Global Options:
       Fixed Frame: base_link
       Frame Rate: 30
     Name: root
     Tools:
       - Class: rviz_default_plugins/Interact
         Hide Inactive Objects: true
       - Class: rviz_default_plugins/MoveCamera
       - Class: rviz_default_plugins/Select
       - Class: rviz_default_plugins/FocusCamera
     Transformation:
       Current:
         Class: rviz_default_plugins/TF
     Value: true
     Views:
       Current:
         Class: rviz_default_plugins/Orbit
         Name: Orbit
         Target Frame: base_link
       Saved: ~
   Window Geometry:
     Displays:
       collapsed: false
     Height: 800
     QMainWindow State: 000000ff00000000fd000000010000000000000156000002f6fc0200000001fb000000100044006900730070006c006100790073010000003d000001e6000000c900ffffff000002d8000002f600000004000000040000000800000008fc0000000100000002000000010000000a00560069006500770073000000003d000000a4000000a400ffffff000002d7000002f600000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
     Selection:
       collapsed: false
     Views:
       collapsed: false
     Width: 1200
     X: 60
     Y: 60
   ```

#### Step 4: Build and Test the Robot Model
1. Build the package:
   ```bash
   cd ~/humanoid_ws
   colcon build --packages-select humanoid_robot_description
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Test the robot model in RViz:
   ```bash
   ros2 launch humanoid_robot_description display.launch.py
   ```

4. In another terminal, check the robot description:
   ```bash
   ros2 param get /robot_state_publisher robot_description | head -n 20
   ```

5. Check the joint states:
   ```bash
   ros2 topic echo /joint_states --field name | head -n 10
   ```

### Expected Results
- Robot model displays correctly in RViz with all links and joints
- Joint state publisher works and publishes joint information
- All 12 joints are properly defined (6 leg joints + 4 arm joints + 2 torso joints)
- Robot appears in correct proportions with appropriate materials

### Analysis Questions
1. How does the kinematic structure of the humanoid robot compare to human anatomy?
2. What are the degrees of freedom in each limb?
3. How would you modify the model to add more realistic joint limits?

---

## Lab 5.2: Balance Control Implementation

### Objective
Implement a basic balance control system using the Zero Moment Point (ZMP) concept for the humanoid robot model.

### Prerequisites
- Completed Lab 5.1
- Understanding of kinematics and dynamics
- Basic control theory knowledge

### Estimated Time
4 hours

### Steps

#### Step 1: Create Balance Control Package
1. Navigate to your workspace:
   ```bash
   cd ~/humanoid_ws/src
   ```

2. Create a new package for balance control:
   ```bash
   ros2 pkg create --build-type ament_python humanoid_balance_control --dependencies rclpy sensor_msgs geometry_msgs std_msgs message_filters tf2_ros
   ```

3. Navigate to the package directory:
   ```bash
   cd humanoid_balance_control
   ```

#### Step 2: Create ZMP-Based Balance Controller
1. Create a balance controller script `humanoid_balance_control/humanoid_balance_control/zmp_balance_controller.py`:
   ```python
   #!/usr/bin/env python3
   """
   ZMP-based balance controller for humanoid robot
   """
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import JointState, Imu
   from geometry_msgs.msg import Point, Vector3, WrenchStamped
   from std_msgs.msg import Float64MultiArray, Bool
   from tf2_ros import TransformException
   from tf2_ros.buffer import Buffer
   from tf2_ros.transform_listener import TransformListener
   import numpy as np
   import math


   class ZMPBalanceController(Node):
       def __init__(self):
           super().__init__('zmp_balance_controller')

           # Publishers
           self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
           self.zmp_pub = self.create_publisher(Point, '/zmp_estimate', 10)
           self.com_pub = self.create_publisher(Point, '/com_estimate', 10)
           self.balance_status_pub = self.create_publisher(Bool, '/balance_status', 10)

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

           # TF buffer and listener
           self.tf_buffer = Buffer()
           self.tf_listener = TransformListener(self.tf_buffer, self)

           # Internal state
           self.joint_positions = {}
           self.joint_velocities = {}
           self.current_imu = Imu()
           self.current_com = Point(x=0.0, y=0.0, z=0.7)  # Default CoM height
           self.current_zmp = Point()
           self.balance_active = False

           # Robot parameters
           self.mass = 30.0  # kg
           self.com_height = 0.7  # m (default)
           self.gravity = 9.81
           self.support_polygon_width = 0.15  # Distance between feet
           self.support_polygon_length = 0.20  # Foot length

           # Control parameters
           self.zmp_p_gain = 10.0
           self.zmp_d_gain = 2.0
           self.com_p_gain = 5.0
           self.com_d_gain = 1.0

           # Previous values for derivative calculation
           self.prev_zmp_error = Point()
           self.prev_com_error = Point()
           self.prev_time = self.get_clock().now()

           # Balance thresholds
           self.zmp_threshold = 0.05  # meters
           self.com_threshold = 0.1   # meters

           # Timer for control loop
           self.control_timer = self.create_timer(0.01, self.balance_control_loop)

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

       def balance_control_loop(self):
           """Main balance control loop"""
           if not self.balance_active:
               return

           # Estimate current CoM and ZMP
           self.current_com = self.estimate_center_of_mass()
           self.current_zmp = self.estimate_zero_moment_point()

           # Calculate desired CoM and ZMP (for balance, keep at center)
           desired_com = Point(x=0.0, y=0.0, z=self.com_height)
           desired_zmp = Point(x=0.0, y=0.0)

           # Calculate errors
           com_error = Point(
               x=desired_com.x - self.current_com.x,
               y=desired_com.y - self.current_com.y,
               z=desired_com.z - self.current_com.z
           )

           zmp_error = Point(
               x=desired_zmp.x - self.current_zmp.x,
               y=desired_zmp.y - self.current_zmp.y
           )

           # Calculate derivatives for PD control
           current_time = self.get_clock().now()
           dt = (current_time - self.prev_time).nanoseconds / 1e9
           self.prev_time = current_time

           if dt > 0 and dt < 1.0:  # Valid time difference
               com_derivative = Point(
                   x=(com_error.x - self.prev_com_error.x) / dt,
                   y=(com_error.y - self.prev_com_error.y) / dt,
                   z=(com_error.z - self.prev_com_error.z) / dt
               )

               zmp_derivative = Point(
                   x=(zmp_error.x - self.prev_zmp_error.x) / dt,
                   y=(zmp_error.y - self.prev_zmp_error.y) / dt
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
               y=self.zmp_p_gain * zmp_error.y + self.zmp_d_gain * zmp_derivative.y
           )

           # Combine control efforts
           total_control = Point(
               x=com_control.x + zmp_control.x,
               y=com_control.y + zmp_control.y,
               z=com_control.z + zmp_control.z
           )

           # Generate joint commands based on control effort
           joint_commands = self.generate_joint_commands(total_control)

           # Publish results
           self.publish_results(joint_commands)

           # Check balance status
           balance_ok = self.check_balance_status()
           balance_msg = Bool()
           balance_msg.data = balance_ok
           self.balance_status_pub.publish(balance_msg)

           # Log balance status
           self.get_logger().info(
               f'ZMP: ({self.current_zmp.x:.3f}, {self.current_zmp.y:.3f}), '
               f'CoM: ({self.current_com.x:.3f}, {self.current_com.y:.3f}), '
               f'Balance: {"OK" if balance_ok else "UNSTABLE"}'
           )

       def estimate_center_of_mass(self):
           """Estimate CoM position based on joint configuration"""
           # Simplified CoM estimation
           # In practice, this would use full kinematic model with link masses
           com = Point()

           # Base CoM position
           com.x = 0.0
           com.y = 0.0
           com.z = self.com_height

           # Add small adjustments based on joint positions
           if 'left_hip' in self.joint_positions:
               com.y += self.joint_positions['left_hip'] * 0.02
           if 'right_hip' in self.joint_positions:
               com.y += self.joint_positions['right_hip'] * -0.02
           if 'left_ankle' in self.joint_positions:
               com.y += self.joint_positions['left_ankle'] * 0.01
           if 'right_ankle' in self.joint_positions:
               com.y += self.joint_positions['right_ankle'] * -0.01

           return com

       def estimate_zero_moment_point(self):
           """Estimate ZMP based on IMU and joint data"""
           # Simplified ZMP estimation
           # In practice, this would use force/torque sensors on feet
           zmp = Point()

           # Use IMU orientation as approximation of ZMP deviation
           # This is a very simplified approach
           roll = self.current_imu.orientation.x  # Simplified
           pitch = self.current_imu.orientation.y  # Simplified

           # Convert orientation to approximate ZMP deviation
           zmp.x = pitch * 0.1  # Scale factor
           zmp.y = roll * 0.1   # Scale factor

           return zmp

       def generate_joint_commands(self, control_effort):
           """Generate joint commands to correct balance errors"""
           commands = Float64MultiArray()

           # Simplified balance control strategy
           # In practice, this would use inverse kinematics or whole-body control

           # Calculate corrective joint angles based on balance errors
           left_hip_cmd = -control_effort.y * 0.5  # Correct lateral balance
           right_hip_cmd = control_effort.y * 0.5  # Correct lateral balance

           left_ankle_cmd = -control_effort.y * 0.3  # Ankle correction
           right_ankle_cmd = control_effort.y * 0.3  # Ankle correction

           # Knee adjustments for forward/backward balance
           left_knee_cmd = -control_effort.x * 0.2
           right_knee_cmd = -control_effort.x * 0.2

           # Shoulder adjustments for upper body balance
           left_shoulder_cmd = -control_effort.y * 0.1
           right_shoulder_cmd = control_effort.y * 0.1

           # Create command array
           commands.data = [
               left_hip_cmd, left_knee_cmd, left_ankle_cmd,
               right_hip_cmd, right_knee_cmd, right_ankle_cmd,
               left_shoulder_cmd, 0.0,  # Left elbow (fixed for now)
               right_shoulder_cmd, 0.0  # Right elbow (fixed for now)
           ]

           return commands

       def publish_results(self, joint_commands):
           """Publish control results"""
           # Publish CoM estimate
           self.com_pub.publish(self.current_com)

           # Publish ZMP estimate
           self.zmp_pub.publish(self.current_zmp)

           # Publish joint commands
           self.joint_cmd_pub.publish(joint_commands)

       def check_balance_status(self):
           """Check if robot is balanced"""
           # Check if ZMP is within support polygon
           zmp_in_polygon = (
               abs(self.current_zmp.x) < self.support_polygon_length / 2 and
               abs(self.current_zmp.y) < self.support_polygon_width / 2
           )

           # Check if CoM is within acceptable range
           com_stable = (
               abs(self.current_com.x) < self.com_threshold and
               abs(self.current_com.y) < self.com_threshold
           )

           return zmp_in_polygon and com_stable

       def activate_balance_control(self):
           """Activate balance control"""
           self.balance_active = True
           self.get_logger().info('Balance control activated')

       def deactivate_balance_control(self):
           """Deactivate balance control"""
           self.balance_active = False
           self.get_logger().info('Balance control deactivated')


   def main(args=None):
       """Main function to initialize and run the balance controller"""
       rclpy.init(args=args)
       controller = ZMPBalanceController()

       try:
           # Activate balance control after initialization
           controller.activate_balance_control()
           rclpy.spin(controller)
       except KeyboardInterrupt:
           controller.deactivate_balance_control()
           pass
       finally:
           controller.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Make the script executable:
   ```bash
   chmod +x humanoid_balance_control/humanoid_balance_control/zmp_balance_controller.py
   ```

#### Step 3: Create Balance Control Launch File
1. Create a launch directory:
   ```bash
   mkdir -p humanoid_balance_control/launch
   ```

2. Create a launch file `humanoid_balance_control/launch/balance_control.launch.py`:
   ```python
   #!/usr/bin/env python3
   """
   Launch file for humanoid balance control
   """
   from launch import LaunchDescription
   from launch_ros.actions import Node


   def generate_launch_description():
       # Balance controller node
       balance_controller = Node(
           package='humanoid_balance_control',
           executable='zmp_balance_controller',
           name='zmp_balance_controller',
           output='screen',
           parameters=[
               {'use_sim_time': False}
           ]
       )

       return LaunchDescription([
           balance_controller
       ])
   ```

#### Step 4: Update Setup Files
1. Update `setup.py` to include the new executable:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   package_name = 'humanoid_balance_control'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='Balance control for humanoid robots',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'zmp_balance_controller = humanoid_balance_control.zmp_balance_controller:main',
           ],
       },
   )
   ```

#### Step 5: Build and Test Balance Control
1. Build the package:
   ```bash
   cd ~/humanoid_ws
   colcon build --packages-select humanoid_balance_control
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Test the balance controller (this will run but won't have real sensor data yet):
   ```bash
   ros2 run humanoid_balance_control zmp_balance_controller
   ```

4. In another terminal, check the published topics:
   ```bash
   ros2 topic list | grep balance
   ros2 topic echo /zmp_estimate
   ```

### Expected Results
- Balance controller runs without errors
- Publishes CoM and ZMP estimates
- Generates joint commands for balance correction
- Reports balance status

### Analysis Questions
1. How does the ZMP concept help maintain balance in humanoid robots?
2. What are the limitations of the simplified CoM estimation used here?
3. How would you improve the balance controller to handle real sensor data?

---

## Lab 5.3: Walking Gait Generation

### Objective
Implement a walking gait generator using inverted pendulum models and demonstrate basic bipedal locomotion.

### Prerequisites
- Completed Lab 5.1 and 5.2
- Understanding of balance control concepts
- Basic knowledge of gait patterns

### Estimated Time
5 hours

### Steps

#### Step 1: Create Walking Gait Package
1. Navigate to your workspace:
   ```bash
   cd ~/humanoid_ws/src
   ```

2. Create a new package for walking gait generation:
   ```bash
   ros2 pkg create --build-type ament_python humanoid_walking_gait --dependencies rclpy sensor_msgs geometry_msgs std_msgs builtin_interfaces
   ```

3. Navigate to the package directory:
   ```bash
   cd humanoid_walking_gait
   ```

#### Step 2: Create Walking Pattern Generator
1. Create a walking pattern generator script `humanoid_walking_gait/humanoid_walking_gait/walking_pattern_generator.py`:
   ```python
   #!/usr/bin/env python3
   """
   Walking pattern generator for humanoid robot using inverted pendulum model
   """
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import JointState
   from geometry_msgs.msg import Point, Twist
   from std_msgs.msg import Float64MultiArray, Bool
   from builtin_interfaces.msg import Time
   import numpy as np
   import math
   from scipy import signal


   class WalkingPatternGenerator(Node):
       def __init__(self):
           super().__init__('walking_pattern_generator')

           # Publishers
           self.trajectory_pub = self.create_publisher(Float64MultiArray, '/walking_trajectory', 10)
           self.step_status_pub = self.create_publisher(Bool, '/step_status', 10)
           self.com_trajectory_pub = self.create_publisher(Point, '/com_trajectory', 10)
           self.zmp_trajectory_pub = self.create_publisher(Point, '/zmp_trajectory', 10)

           # Subscribers
           self.joint_state_sub = self.create_subscription(
               JointState,
               '/joint_states',
               self.joint_state_callback,
               10
           )

           self.balance_status_sub = self.create_subscription(
               Bool,
               '/balance_status',
               self.balance_status_callback,
               10
           )

           # Internal state
           self.joint_positions = {}
           self.balance_ok = True
           self.walk_enabled = False
           self.step_count = 0
           self.step_phase = 0.0
           self.walk_state = 'standing'  # standing, left_support, right_support, double_support

           # Walking parameters
           self.step_length = 0.20  # meters
           self.step_width = 0.15  # meters (distance between feet)
           self.step_height = 0.05 # meters (foot clearance)
           self.step_duration = 1.0 # seconds
           self.dsp_ratio = 0.1     # Double Support Phase ratio
           self.walk_speed = 0.0    # m/s
           self.walk_direction = 'forward'  # forward, backward, left, right

           # Inverted pendulum parameters
           self.com_height = 0.7    # CoM height (m)
           self.gravity = 9.81
           self.omega = math.sqrt(self.gravity / self.com_height)  # Natural frequency

           # Support polygon
           self.left_foot_pos = Point(x=0.0, y=self.step_width/2, z=0.0)
           self.right_foot_pos = Point(x=0.0, y=-self.step_width/2, z=0.0)
           self.com_pos = Point(x=0.0, y=0.0, z=self.com_height)

           # Preview control parameters
           self.preview_horizon = 10  # steps to preview
           self.zmp_reference = [Point() for _ in range(self.preview_horizon)]

           # Timer for walking pattern generation
           self.pattern_timer = self.create_timer(0.01, self.generate_walking_pattern)

           self.get_logger().info('Walking Pattern Generator initialized')

       def joint_state_callback(self, msg):
           """Update joint state"""
           for i, name in enumerate(msg.name):
               if i < len(msg.position):
                   self.joint_positions[name] = msg.position[i]

       def balance_status_callback(self, msg):
           """Update balance status"""
           self.balance_ok = msg.data

       def generate_walking_pattern(self):
           """Generate walking pattern using inverted pendulum model"""
           if not self.walk_enabled or not self.balance_ok:
               # If not walking or balance is bad, return to standing position
               if self.walk_enabled:
                   self.get_logger().warn('Balance compromised, stopping walk')
                   self.stop_walking()
               return

           # Update step phase based on time
           current_time = self.get_clock().now().nanoseconds / 1e9
           self.step_phase = ((current_time / self.step_duration) % 1.0)

           # Determine current walk state based on step phase
           self.update_walk_state()

           # Generate CoM trajectory using inverted pendulum model
           self.generate_com_trajectory()

           # Generate foot trajectories
           self.generate_foot_trajectories()

           # Generate ZMP trajectory
           self.generate_zmp_trajectory()

           # Generate joint commands from trajectories
           joint_commands = self.generate_joint_commands()

           # Publish results
           self.publish_walking_pattern(joint_commands)

           # Log walking status
           self.get_logger().info(
               f'Step: {self.step_count}, Phase: {self.step_phase:.2f}, '
               f'State: {self.walk_state}, CoM: ({self.com_pos.x:.3f}, {self.com_pos.y:.3f})'
           )

       def update_walk_state(self):
           """Update current walk state based on step phase"""
           dsp_start = self.dsp_ratio / 2
           dsp_end = 1.0 - self.dsp_ratio / 2

           if self.step_phase < dsp_start:
               # Double support phase at beginning
               self.walk_state = 'double_support'
           elif self.step_phase < 0.5 - self.dsp_ratio / 2:
               # Left foot support
               self.walk_state = 'left_support'
           elif self.step_phase < 0.5 + self.dsp_ratio / 2:
               # Double support phase at middle
               self.walk_state = 'double_support'
           elif self.step_phase < 1.0 - self.dsp_ratio / 2:
               # Right foot support
               self.walk_state = 'right_support'
           else:
               # Double support phase at end
               self.walk_state = 'double_support'

           # Update step count at transition points
           if self.step_phase < 0.01:  # Beginning of cycle
               self.step_count += 1

       def generate_com_trajectory(self):
           """Generate CoM trajectory using inverted pendulum model"""
           # Use preview control to generate CoM trajectory
           # This is a simplified implementation

           # For this example, generate a simple sinusoidal CoM trajectory
           # that moves forward and maintains balance over the supporting foot

           # Forward progression
           forward_progress = self.step_count * self.step_length + self.step_length * self.step_phase

           # Lateral movement to maintain balance over supporting foot
           if self.walk_state == 'left_support':
               target_y = self.step_width / 2
           elif self.walk_state == 'right_support':
               target_y = -self.step_width / 2
           else:  # double support - center between feet
               target_y = 0.0

           # Add small oscillation for natural walking motion
           lateral_oscillation = 0.01 * math.sin(2 * math.pi * self.step_phase)

           self.com_pos.x = forward_progress * 0.9  # Slightly behind for stability
           self.com_pos.y = target_y + lateral_oscillation
           self.com_pos.z = self.com_height  # Maintain constant height

       def generate_foot_trajectories(self):
           """Generate foot trajectories for walking"""
           # Left foot trajectory
           if self.walk_state == 'left_support':
               # Left foot is supporting, stays in place or moves slightly
               self.left_foot_pos.x = self.step_count * self.step_length
               self.left_foot_pos.y = self.step_width / 2
               self.left_foot_pos.z = 0.0
           else:
               # Left foot is swinging forward
               # Calculate swing trajectory based on step phase
               if self.step_phase < 0.5:
                   # Left foot swings forward in this half-cycle
                   swing_progress = self.step_phase * 2  # 0 to 1 during swing
                   self.left_foot_pos.x = self.step_count * self.step_length + self.step_length * swing_progress

                   # Add vertical clearance
                   swing_arc = math.sin(math.pi * swing_progress)
                   self.left_foot_pos.z = self.step_height * swing_arc

                   # Lateral movement back to center
                   self.left_foot_pos.y = self.step_width / 2 * (1 - swing_progress)
               else:
                   # Left foot is in support or preparation
                   self.left_foot_pos.x = (self.step_count + 1) * self.step_length
                   self.left_foot_pos.y = 0.0
                   self.left_foot_pos.z = 0.0

           # Right foot trajectory
           if self.walk_state == 'right_support':
               # Right foot is supporting, stays in place or moves slightly
               self.right_foot_pos.x = self.step_count * self.step_length
               self.right_foot_pos.y = -self.step_width / 2
               self.right_foot_pos.z = 0.0
           else:
               # Right foot is swinging forward
               if self.step_phase >= 0.5:
                   # Right foot swings forward in this half-cycle
                   swing_progress = (self.step_phase - 0.5) * 2  # 0 to 1 during swing
                   self.right_foot_pos.x = self.step_count * self.step_length + self.step_length * swing_progress

                   # Add vertical clearance
                   swing_arc = math.sin(math.pi * swing_progress)
                   self.right_foot_pos.z = self.step_height * swing_arc

                   # Lateral movement back to center
                   self.right_foot_pos.y = -self.step_width / 2 * (1 - swing_progress)
               else:
                   # Right foot is in support or preparation
                   self.right_foot_pos.x = (self.step_count + 1) * self.step_length
                   self.right_foot_pos.y = 0.0
                   self.right_foot_pos.z = 0.0

       def generate_zmp_trajectory(self):
           """Generate ZMP trajectory for stable walking"""
           # Simplified ZMP trajectory that follows the support polygon
           zmp = Point()

           if self.walk_state == 'left_support':
               # ZMP under left foot
               zmp.x = self.left_foot_pos.x
               zmp.y = self.left_foot_pos.y
           elif self.walk_state == 'right_support':
               # ZMP under right foot
               zmp.x = self.right_foot_pos.x
               zmp.y = self.right_foot_pos.y
           else:  # double support
               # ZMP between feet
               zmp.x = (self.left_foot_pos.x + self.right_foot_pos.x) / 2
               zmp.y = (self.left_foot_pos.y + self.right_foot_pos.y) / 2

           # Add small forward offset for dynamic walking
           zmp.x += 0.02  # Small forward offset

           self.zmp_reference[0] = zmp  # Current ZMP reference

       def generate_joint_commands(self):
           """Generate joint commands from desired trajectories"""
           commands = Float64MultiArray()

           # This is a simplified kinematic mapping
           # In practice, this would use inverse kinematics

           # Map desired foot positions to joint angles using simplified model
           # For demonstration purposes, we'll use a simplified approach

           # Calculate required joint angles to achieve desired foot positions
           # This is a very simplified model - real implementation would use
           # proper inverse kinematics

           # Left leg commands (simplified)
           left_hip_angle = math.atan2(
               self.left_foot_pos.z,
               math.sqrt((self.left_foot_pos.x)**2 + (self.left_foot_pos.y - 0.1)**2)
           ) if (self.left_foot_pos.x**2 + (self.left_foot_pos.y - 0.1)**2 + self.left_foot_pos.z**2) > 0.01 else 0.0

           left_knee_angle = math.pi / 3  # Fixed bend for simplicity
           left_ankle_angle = -left_hip_angle - left_knee_angle  # Keep foot level

           # Right leg commands (simplified)
           right_hip_angle = math.atan2(
               self.right_foot_pos.z,
               math.sqrt((self.right_foot_pos.x)**2 + (self.right_foot_pos.y + 0.1)**2)
           ) if (self.right_foot_pos.x**2 + (self.right_foot_pos.y + 0.1)**2 + self.right_foot_pos.z**2) > 0.01 else 0.0

           right_knee_angle = math.pi / 3  # Fixed bend for simplicity
           right_ankle_angle = -right_hip_angle - right_knee_angle  # Keep foot level

           # Upper body adjustments for balance
           left_shoulder_angle = -self.com_pos.y * 0.5  # Counteract lateral lean
           right_shoulder_angle = self.com_pos.y * 0.5
           left_elbow_angle = -0.2  # Default elbow bend
           right_elbow_angle = -0.2

           commands.data = [
               left_hip_angle, left_knee_angle, left_ankle_angle,
               right_hip_angle, right_knee_angle, right_ankle_angle,
               left_shoulder_angle, left_elbow_angle,
               right_shoulder_angle, right_elbow_angle
           ]

           return commands

       def publish_walking_pattern(self, joint_commands):
           """Publish walking pattern and related information"""
           # Publish joint commands
           self.trajectory_pub.publish(joint_commands)

           # Publish CoM trajectory
           self.com_trajectory_pub.publish(self.com_pos)

           # Publish ZMP trajectory
           self.zmp_trajectory_pub.publish(self.zmp_reference[0])

           # Publish step status
           step_msg = Bool()
           step_msg.data = self.walk_enabled
           self.step_status_pub.publish(step_msg)

       def start_walking(self, speed=0.2, step_length=0.2, step_width=0.15, step_height=0.05):
           """Start the walking pattern generation"""
           self.walk_speed = speed
           self.step_length = step_length
           self.step_width = step_width
           self.step_height = step_height
           self.walk_enabled = True
           self.get_logger().info(
               f'Walking started: speed={speed}m/s, step_len={step_length}m, '
               f'step_width={step_width}m, step_height={step_height}m'
           )

       def stop_walking(self):
           """Stop the walking pattern generation"""
           self.walk_enabled = False
           self.get_logger().info('Walking stopped')

       def set_walk_direction(self, direction):
           """Set walking direction"""
           if direction in ['forward', 'backward', 'left', 'right']:
               self.walk_direction = direction
               self.get_logger().info(f'Walking direction set to: {direction}')


   def main(args=None):
       """Main function to initialize and run the walking pattern generator"""
       rclpy.init(args=args)
       walker = WalkingPatternGenerator()

       try:
           # Start walking after initialization
           walker.start_walking(speed=0.2, step_length=0.2, step_width=0.15, step_height=0.05)
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

2. Make the script executable:
   ```bash
   chmod +x humanoid_walking_gait/humanoid_walking_gait/walking_pattern_generator.py
   ```

#### Step 3: Create Walking Control Launch File
1. Create a launch directory if it doesn't exist:
   ```bash
   mkdir -p humanoid_walking_gait/launch
   ```

2. Create a launch file `humanoid_walking_gait/launch/walking_control.launch.py`:
   ```python
   #!/usr/bin/env python3
   """
   Launch file for humanoid walking control
   """
   from launch import LaunchDescription
   from launch_ros.actions import Node


   def generate_launch_description():
       # Walking pattern generator node
       walking_generator = Node(
           package='humanoid_walking_gait',
           executable='walking_pattern_generator',
           name='walking_pattern_generator',
           output='screen',
           parameters=[
               {'use_sim_time': False}
           ]
       )

       return LaunchDescription([
           walking_generator
       ])
   ```

#### Step 4: Update Setup Files
1. Update `setup.py` to include the new executable:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   package_name = 'humanoid_walking_gait'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='Walking gait generation for humanoid robots',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'walking_pattern_generator = humanoid_walking_gait.walking_pattern_generator:main',
           ],
       },
   )
   ```

#### Step 5: Build and Test Walking System
1. Build the package:
   ```bash
   cd ~/humanoid_ws
   colcon build --packages-select humanoid_walking_gait
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Test the walking pattern generator:
   ```bash
   ros2 run humanoid_walking_gait walking_pattern_generator
   ```

4. In another terminal, check the published topics:
   ```bash
   ros2 topic list | grep walk
   ros2 topic echo /com_trajectory
   ros2 topic echo /zmp_trajectory
   ```

### Expected Results
- Walking pattern generator runs without errors
- Publishes CoM and ZMP trajectories
- Generates joint commands for walking
- Shows proper step phase transitions
- Maintains balance during walking simulation

### Analysis Questions
1. How does the inverted pendulum model contribute to stable walking?
2. What are the key differences between single and double support phases?
3. How would you improve the walking pattern to handle different terrains?

---

## Lab 5.4: AI-Based Locomotion Control

### Objective
Implement a reinforcement learning-based approach for optimizing humanoid robot locomotion and demonstrate adaptive gait control.

### Prerequisites
- Completed Labs 5.1-5.3
- Basic understanding of machine learning concepts
- Experience with ROS 2 control systems

### Estimated Time
6 hours

### Steps

#### Step 1: Create AI Locomotion Package
1. Navigate to your workspace:
   ```bash
   cd ~/humanoid_ws/src
   ```

2. Create a new package for AI-based locomotion:
   ```bash
   ros2 pkg create --build-type ament_python humanoid_ai_locomotion --dependencies rclpy sensor_msgs geometry_msgs std_msgs builtin_interfaces
   ```

3. Navigate to the package directory:
   ```bash
   cd humanoid_ai_locomotion
   ```

#### Step 2: Create Reinforcement Learning Controller
1. Create an RL controller script `humanoid_ai_locomotion/humanoid_ai_locomotion/rl_locomotion_controller.py`:
   ```python
   #!/usr/bin/env python3
   """
   Reinforcement Learning controller for humanoid locomotion optimization
   """
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import JointState, Imu
   from geometry_msgs.msg import Point, Twist
   from std_msgs.msg import Float64MultiArray, Bool, Float32
   import numpy as np
   import math
   import random
   from collections import deque


   class RLLocomotionController(Node):
       def __init__(self):
           super().__init__('rl_locomotion_controller')

           # Publishers
           self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
           self.reward_pub = self.create_publisher(Float32, '/rl_reward', 10)
           self.action_pub = self.create_publisher(Float64MultiArray, '/rl_action', 10)
           self.gait_param_pub = self.create_publisher(Float64MultiArray, '/gait_parameters', 10)

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
               '/com_trajectory',
               self.com_callback,
               10
           )

           self.zmp_sub = self.create_subscription(
               Point,
               '/zmp_trajectory',
               self.zmp_callback,
               10
           )

           # Internal state
           self.joint_positions = {}
           self.joint_velocities = {}
           self.current_imu = Imu()
           self.current_com = Point()
           self.current_zmp = Point()
           self.episode_step = 0
           self.total_reward = 0.0
           self.learning_active = True

           # RL parameters
           self.learning_rate = 0.001
           self.discount_factor = 0.95
           self.epsilon = 1.0  # Exploration rate
           self.epsilon_decay = 0.995
           self.epsilon_min = 0.05

           # Action space (gait parameter adjustments)
           # Actions: [step_length_delta, step_width_delta, step_height_delta, step_timing_delta, com_adjustment]
           self.action_space_size = 5
           self.action_bounds = [-0.05, 0.05]  # Bounds for each action (meters or seconds)

           # State space (simplified)
           # State: [com_x, com_y, com_z, roll, pitch, yaw, zmp_x, zmp_y, joint_pos_mean, joint_vel_mean]
           self.state_space_size = 10

           # Q-Network weights (using linear function approximation)
           self.q_weights = np.random.randn(self.state_space_size, self.action_space_size) * 0.1

           # Replay buffer for experience replay
           self.replay_buffer = deque(maxlen=5000)

           # Gait parameters to optimize
           self.base_step_length = 0.20
           self.base_step_width = 0.15
           self.base_step_height = 0.05
           self.base_step_duration = 1.0
           self.base_com_height = 0.70

           # Current gait parameters (will be adjusted by RL)
           self.current_step_length = self.base_step_length
           self.current_step_width = self.base_step_width
           self.current_step_height = self.base_step_height
           self.current_step_duration = self.base_step_duration
           self.current_com_height = self.base_com_height

           # Performance tracking
           self.forward_progress = 0.0
           self.balance_errors = 0.0
           self.energy_consumption = 0.0

           # Timer for RL control loop
           self.rl_timer = self.create_timer(0.05, self.rl_control_loop)

           self.get_logger().info('RL Locomotion Controller initialized')

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
           """Update CoM data"""
           self.current_com = msg

       def zmp_callback(self, msg):
           """Update ZMP data"""
           self.current_zmp = msg

       def rl_control_loop(self):
           """Main RL control loop"""
           if not self.learning_active:
               return

           # Get current state
           state = self.get_state()

           # Select action using epsilon-greedy policy
           action = self.select_action(state)

           # Apply action to modify gait parameters
           self.apply_action(action)

           # Calculate reward based on performance
           reward = self.calculate_reward()

           # Get next state
           next_state = self.get_state()

           # Store experience in replay buffer
           self.replay_buffer.append((state, action, reward, next_state))

           # Update Q-network using experience replay
           if len(self.replay_buffer) > 100:
               self.train_network()

           # Publish results
           self.publish_results(reward, action)

           # Update episode counters
           self.episode_step += 1
           self.total_reward += reward

           # Decay exploration rate
           if self.epsilon > self.epsilon_min:
               self.epsilon *= self.epsilon_decay

           # Log performance periodically
           if self.episode_step % 100 == 0:
               self.get_logger().info(
                   f'Episode Step: {self.episode_step}, '
                   f'Reward: {reward:.3f}, Total: {self.total_reward:.3f}, '
                   f'Epsilon: {self.epsilon:.3f}, '
                   f'Step Len: {self.current_step_length:.3f}, '
                   f'Progress: {self.forward_progress:.3f}m'
               )

       def get_state(self):
           """Get current state vector"""
           state = np.zeros(self.state_space_size)

           # CoM position (relative to starting position)
           state[0] = self.current_com.x
           state[1] = self.current_com.y
           state[2] = self.current_com.z

           # IMU orientation (roll, pitch, yaw)
           # Simplified conversion from quaternion (in practice, use proper conversion)
           state[3] = self.current_imu.orientation.x  # roll approximation
           state[4] = self.current_imu.orientation.y  # pitch approximation
           state[5] = self.current_imu.orientation.z  # yaw approximation

           # ZMP position relative to CoM
           state[6] = self.current_zmp.x - self.current_com.x
           state[7] = self.current_zmp.y - self.current_com.y

           # Average joint positions and velocities
           if self.joint_positions:
               state[8] = np.mean(list(self.joint_positions.values()))
           if self.joint_velocities:
               state[9] = np.mean(list(np.abs(np.array(list(self.joint_velocities.values())))))

           return state

       def select_action(self, state):
           """Select action using epsilon-greedy policy"""
           if random.random() < self.epsilon:
               # Explore: random action
               action = np.random.uniform(
                   self.action_bounds[0],
                   self.action_bounds[1],
                   self.action_space_size
               )
           else:
               # Exploit: best action according to Q-network
               q_values = np.dot(state, self.q_weights)
               best_action_idx = np.argmax(q_values)

               # Create action vector with impulse at best action
               action = np.zeros(self.action_space_size)
               action[best_action_idx] = np.random.uniform(-0.02, 0.02)  # Small adjustment

           return action

       def apply_action(self, action):
           """Apply action to modify gait parameters"""
           # Apply changes with bounds checking
           self.current_step_length = np.clip(
               self.base_step_length + action[0],
               0.1, 0.3
           )
           self.current_step_width = np.clip(
               self.base_step_width + action[1],
               0.1, 0.25
           )
           self.current_step_height = np.clip(
               self.base_step_height + action[2],
               0.02, 0.1
           )
           self.current_step_duration = np.clip(
               self.base_step_duration + action[3],
               0.5, 1.5
           )
           self.current_com_height = np.clip(
               self.base_com_height + action[4],
               0.6, 0.8
           )

           # Publish updated gait parameters
           gait_params = Float64MultiArray()
           gait_params.data = [
               self.current_step_length,
               self.current_step_width,
               self.current_step_height,
               self.current_step_duration,
               self.current_com_height
           ]
           self.gait_param_pub.publish(gait_params)

       def calculate_reward(self):
           """Calculate reward based on walking performance"""
           reward = 0.0

           # Reward for forward progress
           forward_reward = self.current_com.x * 10.0
           reward += forward_reward

           # Penalty for balance errors
           # Calculate deviation from upright position
           roll_deviation = abs(self.current_imu.orientation.x)
           pitch_deviation = abs(self.current_imu.orientation.y)
           balance_penalty = (roll_deviation + pitch_deviation) * 20.0
           reward -= balance_penalty

           # Penalty for CoM deviation from center
           lateral_deviation = abs(self.current_com.y)
           center_penalty = lateral_deviation * 15.0
           reward -= center_penalty

           # Penalty for ZMP leaving support polygon
           zmp_margin = 0.05  # Safety margin
           if (abs(self.current_zmp.x) > self.current_step_length/2 + zmp_margin or
               abs(self.current_zmp.y) > self.current_step_width/2 + zmp_margin):
               reward -= 5.0

           # Reward for smooth motion (low joint velocities)
           if self.joint_velocities:
               avg_velocity = np.mean(np.abs(list(self.joint_velocities.values())))
               smoothness_reward = max(0, 1.0 - avg_velocity) * 2.0
               reward += smoothness_reward

           # Penalty for high energy consumption (estimated by joint torques)
           if self.joint_positions and self.joint_velocities:
               # Simplified energy estimation
               energy_estimate = np.mean(np.abs(list(self.joint_positions.values()))) * 0.1
               energy_penalty = energy_estimate * 1.0
               reward -= energy_penalty

           # Small time penalty to encourage efficiency
           reward -= 0.01

           # Track performance metrics
           self.forward_progress = self.current_com.x
           self.balance_errors = balance_penalty
           self.energy_consumption = energy_estimate if 'energy_estimate' in locals() else 0.0

           return reward

       def train_network(self):
           """Train Q-network using experience replay"""
           # Sample random batch from replay buffer
           batch_size = 32
           if len(self.replay_buffer) < batch_size:
               return

           batch_indices = np.random.choice(len(self.replay_buffer), batch_size, replace=False)
           batch = [self.replay_buffer[i] for i in batch_indices]

           # Update Q-network for each sample in batch
           for state, action, reward, next_state in batch:
               # Find the action index with maximum Q-value for next state
               next_q_values = np.dot(next_state, self.q_weights)
               max_next_q = np.max(next_q_values)

               # Calculate target Q-value
               target_q = reward + self.discount_factor * max_next_q

               # Find action index that was taken
               action_idx = np.argmin(np.abs(action - np.array([a if isinstance(a, (int, float)) else 0 for a in action])))

               # Calculate current Q-value for the taken action
               current_q = np.dot(state, self.q_weights)[action_idx]

               # Calculate temporal difference error
               td_error = target_q - current_q

               # Update weights using gradient descent
               self.q_weights[:, action_idx] += self.learning_rate * td_error * state

       def publish_results(self, reward, action):
           """Publish RL results"""
           # Publish reward
           reward_msg = Float32()
           reward_msg.data = reward
           self.reward_pub.publish(reward_msg)

           # Publish action
           action_msg = Float64MultiArray()
           action_msg.data = action.tolist() if isinstance(action, np.ndarray) else [action]
           self.action_pub.publish(action_msg)

       def start_learning(self):
           """Start the learning process"""
           self.learning_active = True
           self.get_logger().info('RL learning started')

       def stop_learning(self):
           """Stop the learning process"""
           self.learning_active = False
           self.get_logger().info('RL learning stopped')


   def main(args=None):
       """Main function to initialize and run the RL controller"""
       rclpy.init(args=args)
       controller = RLLocomotionController()

       try:
           controller.start_learning()
           rclpy.spin(controller)
       except KeyboardInterrupt:
           controller.stop_learning()
           pass
       finally:
           controller.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Make the script executable:
   ```bash
   chmod +x humanoid_ai_locomotion/humanoid_ai_locomotion/rl_locomotion_controller.py
   ```

#### Step 3: Create AI Locomotion Launch File
1. Create a launch directory if it doesn't exist:
   ```bash
   mkdir -p humanoid_ai_locomotion/launch
   ```

2. Create a launch file `humanoid_ai_locomotion/launch/ai_locomotion.launch.py`:
   ```python
   #!/usr/bin/env python3
   """
   Launch file for AI-based humanoid locomotion
   """
   from launch import LaunchDescription
   from launch_ros.actions import Node


   def generate_launch_description():
       # RL locomotion controller node
       rl_controller = Node(
           package='humanoid_ai_locomotion',
           executable='rl_locomotion_controller',
           name='rl_locomotion_controller',
           output='screen',
           parameters=[
               {'use_sim_time': False}
           ]
       )

       return LaunchDescription([
           rl_controller
       ])
   ```

#### Step 4: Update Setup Files
1. Update `setup.py` to include the new executable:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   package_name = 'humanoid_ai_locomotion'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='AI-based locomotion control for humanoid robots',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'rl_locomotion_controller = humanoid_ai_locomotion.rl_locomotion_controller:main',
           ],
       },
   )
   ```

#### Step 5: Build and Test AI Locomotion System
1. Build the package:
   ```bash
   cd ~/humanoid_ws
   colcon build --packages-select humanoid_ai_locomotion
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Test the AI locomotion controller:
   ```bash
   ros2 run humanoid_ai_locomotion rl_locomotion_controller
   ```

4. In another terminal, check the published topics:
   ```bash
   ros2 topic list | grep rl
   ros2 topic echo /rl_reward
   ros2 topic echo /gait_parameters
   ```

### Expected Results
- RL controller runs without errors
- Learns to optimize gait parameters over time
- Improves walking performance based on rewards
- Adapts gait parameters to maintain balance and forward progress
- Publishes reward signals and gait parameter adjustments

### Analysis Questions
1. How does reinforcement learning improve locomotion compared to fixed gait patterns?
2. What are the challenges in applying RL to physical humanoid robots?
3. How would you modify the reward function to handle different terrains?

### Safety Considerations
- Always test AI controllers in simulation first
- Implement safety limits on gait parameter adjustments
- Monitor robot behavior during learning
- Include emergency stop mechanisms
- Validate learned behaviors before deployment