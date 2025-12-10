# Lab Exercises: Gazebo Simulation and Physics Engines

## Lab 3.1: Setting Up Gazebo Environment and Basic Robot Model

### Objective
Install and configure Gazebo Garden, create a basic robot model using URDF, and spawn it in a simulation environment.

### Prerequisites
- Completed Module 1 (Introduction to Physical AI)
- Completed Module 2 (ROS 2 Fundamentals)
- Ubuntu 22.04 LTS with ROS 2 Humble Hawksbill installed

### Estimated Time
2 hours

### Steps

#### Step 1: Install Gazebo Garden
1. Add the Gazebo repository:
   ```bash
   sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo.list'
   wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   sudo apt update
   ```

2. Install Gazebo Garden:
   ```bash
   sudo apt install gazebo
   ```

3. Install ROS 2 Gazebo packages:
   ```bash
   sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros-gz
   ```

4. Test Gazebo installation:
   ```bash
   gazebo --version
   ```

#### Step 2: Create a Robot Description Package
1. Navigate to your workspace:
   ```bash
   cd ~/physical_ai_ws/src
   ```

2. Create a robot description package:
   ```bash
   ros2 pkg create --build-type ament_cmake robot_description --dependencies urdf xacro
   ```

3. Navigate to the package directory:
   ```bash
   cd robot_description
   ```

4. Create the directory structure:
   ```bash
   mkdir -p urdf meshes models launch
   ```

#### Step 3: Create a Simple Robot Model (URDF)
1. Create a simple robot URDF file `robot_description/urdf/simple_robot.urdf`:
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
       <limit effort="100" velocity="100"/>
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

#### Step 4: Create a Launch File for Robot Spawn
1. Create a launch file `robot_description/launch/spawn_robot.launch.py`:
   ```python
   #!/usr/bin/env python3
   """
   Launch file to spawn a robot in Gazebo
   """
   import os
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
   from launch_ros.actions import Node
   from launch_ros.substitutions import FindPackageShare


   def generate_launch_description():
       # Launch arguments
       model_arg = DeclareLaunchArgument(
           'model',
           default_value='simple_robot.urdf',
           description='Choose one of the robot models from `/robot_description/urdf`'
       )

       # Launch Gazebo
       gazebo = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               PathJoinSubstitution([
                   FindPackageShare('gazebo_ros'),
                   'launch',
                   'gazebo.launch.py'
               ])
           ])
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
                   FindPackageShare('robot_description'),
                   'urdf',
                   LaunchConfiguration('model')
               ])
           ]
       )

       # Spawn entity
       spawn_entity = Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=[
               '-topic', 'robot_description',
               '-entity', 'simple_robot',
               '-x', '0', '-y', '0', '-z', '0.1'
           ],
           output='screen'
       )

       # Return the complete launch description
       return LaunchDescription([
           model_arg,
           gazebo,
           robot_state_publisher,
           spawn_entity
       ])
   ```

#### Step 5: Build and Test the Robot Model
1. Build the package:
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select robot_description
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Launch the simulation:
   ```bash
   ros2 launch robot_description spawn_robot.launch.py
   ```

4. In another terminal, send velocity commands to test the robot:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
   ```

### Expected Results
- Gazebo launches with the simple robot model
- Robot appears in the simulation environment
- Robot moves forward when velocity commands are sent
- Robot state is published via ROS 2 topics

### Analysis Questions
1. What happens when you change the friction values (mu1, mu2) in the URDF?
2. How does changing the wheel diameter affect the robot's movement?
3. What is the significance of the inertia parameters in the URDF?

---

## Lab 3.2: Adding Sensors to the Robot Model

### Objective
Add various sensors to the robot model (camera, LIDAR, IMU) and verify that they publish data in the simulation.

### Prerequisites
- Completed Lab 3.1
- Understanding of URDF/SDF formats
- Working Gazebo installation

### Estimated Time
2.5 hours

### Steps

#### Step 1: Create an Enhanced Robot Model with Sensors
1. Navigate to the robot description package:
   ```bash
   cd ~/physical_ai_ws/src/robot_description
   ```

2. Create an enhanced robot URDF file with sensors `robot_description/urdf/sensor_robot.urdf`:
   ```xml
   <?xml version="1.0"?>
   <robot name="sensor_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

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

     <!-- Camera link -->
     <link name="camera_link">
       <visual>
         <geometry>
           <box size="0.05 0.05 0.05"/>
         </geometry>
         <material name="red">
           <color rgba="0.8 0.2 0.2 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.05 0.05 0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.1"/>
         <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
       </inertial>
     </link>

     <!-- IMU link -->
     <link name="imu_link">
       <visual>
         <geometry>
           <box size="0.02 0.02 0.02"/>
         </geometry>
         <material name="green">
           <color rgba="0.2 0.8 0.2 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.02 0.02 0.02"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.01"/>
         <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
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

     <joint name="camera_joint" type="fixed">
       <parent link="base_link"/>
       <child link="camera_link"/>
       <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
     </joint>

     <joint name="imu_joint" type="fixed">
       <parent link="base_link"/>
       <child link="imu_link"/>
       <origin xyz="0 0 0.05" rpy="0 0 0"/>
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

     <gazebo reference="camera_link">
       <material>Gazebo/Red</material>
     </gazebo>

     <gazebo reference="imu_link">
       <material>Gazebo/Green</material>
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

     <!-- Camera sensor -->
     <gazebo reference="camera_link">
       <sensor name="camera" type="camera">
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
     </gazebo>

     <!-- Camera plugin -->
     <gazebo>
       <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
         <camera_name>camera</camera_name>
         <image_topic_name>image_raw</image_topic_name>
         <camera_info_topic_name>camera_info</camera_info_topic_name>
         <frame_name>camera_link</frame_name>
       </plugin>
     </gazebo>

     <!-- IMU sensor -->
     <gazebo reference="imu_link">
       <sensor name="imu_sensor" type="imu">
         <always_on>1</always_on>
         <update_rate>100</update_rate>
         <visualize>false</visualize>
       </sensor>
     </gazebo>

     <!-- IMU plugin -->
     <gazebo>
       <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
         <robotNamespace>/</robotNamespace>
         <topicName>imu</topicName>
         <bodyName>imu_link</bodyName>
         <frameName>imu_link</frameName>
         <serviceName>imu_service</serviceName>
         <gaussianNoise>0.01</gaussianNoise>
         <updateRateHZ>100.0</updateRateHZ>
       </plugin>
     </gazebo>

   </robot>
   ```

#### Step 2: Create a Sensor Test Launch File
1. Create a launch file `robot_description/launch/sensor_robot.launch.py`:
   ```python
   #!/usr/bin/env python3
   """
   Launch file to spawn a robot with sensors in Gazebo
   """
   import os
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
   from launch_ros.actions import Node
   from launch_ros.substitutions import FindPackageShare


   def generate_launch_description():
       # Launch arguments
       model_arg = DeclareLaunchArgument(
           'model',
           default_value='sensor_robot.urdf',
           description='Choose one of the robot models from `/robot_description/urdf`'
       )

       # Launch Gazebo
       gazebo = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               PathJoinSubstitution([
                   FindPackageShare('gazebo_ros'),
                   'launch',
                   'gazebo.launch.py'
               ])
           ])
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
                   FindPackageShare('robot_description'),
                   'urdf',
                   LaunchConfiguration('model')
               ])
           ]
       )

       # Spawn entity
       spawn_entity = Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=[
               '-topic', 'robot_description',
               '-entity', 'sensor_robot',
               '-x', '0', '-y', '0', '-z', '0.1'
           ],
           output='screen'
       )

       # Return the complete launch description
       return LaunchDescription([
           model_arg,
           gazebo,
           robot_state_publisher,
           spawn_entity
       ])
   ```

#### Step 3: Test Sensor Data Publication
1. Build the package:
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select robot_description
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Launch the simulation with sensors:
   ```bash
   ros2 launch robot_description sensor_robot.launch.py
   ```

4. In another terminal, check available topics:
   ```bash
   ros2 topic list
   ```

5. View sensor data:
   ```bash
   # Check camera images
   ros2 topic echo /image_raw --field data | head -n 5

   # Check IMU data
   ros2 topic echo /imu

   # Check odometry
   ros2 topic echo /odom
   ```

6. Send commands to move the robot and observe sensor changes:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
   ```

### Expected Results
- Robot with sensors spawns in Gazebo
- Camera, IMU, and odometry topics are published
- Sensor data changes as the robot moves
- All sensor data is properly synchronized

### Analysis Questions
1. How does the camera's field of view affect the captured image?
2. What is the impact of sensor update rates on simulation performance?
3. How would you calibrate the IMU sensor data to match real-world values?

---

## Lab 3.3: Creating Complex Environments and World Files

### Objective
Create complex simulation environments with multiple objects, different terrains, and lighting conditions to test robot capabilities.

### Prerequisites
- Completed Lab 3.1 and 3.2
- Understanding of SDF world file format
- Working Gazebo installation

### Estimated Time
3 hours

### Steps

#### Step 1: Create a Custom World File
1. Navigate to the robot description package:
   ```bash
   cd ~/physical_ai_ws/src/robot_description
   ```

2. Create a worlds directory:
   ```bash
   mkdir worlds
   ```

3. Create a complex world file `robot_description/worlds/obstacle_course.sdf`:
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="obstacle_course">
       <!-- Physics -->
       <physics type="ode">
         <max_step_size>0.001</max_step_size>
         <real_time_factor>1</real_time_factor>
         <real_time_update_rate>1000</real_time_update_rate>
         <gravity>0 0 -9.8</gravity>
         <ode>
           <solver>
             <type>quick</type>
             <iters>10</iters>
             <sor>1.0</sor>
           </solver>
           <constraints>
             <cfm>0.0</cfm>
             <erp>0.2</erp>
             <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
             <contact_surface_layer>0.001</contact_surface_layer>
           </constraints>
         </ode>
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
         <uri>model://sensor_robot</uri>
         <pose>0 0 0.1 0 0 0</pose>
       </include>

       <!-- Obstacles -->
       <!-- Wall obstacles -->
       <model name="wall1">
         <pose>2 0 0.5 0 0 0</pose>
         <link name="link">
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.1 4 1</size>
               </box>
             </geometry>
             <material>
               <ambient>0.8 0.8 0.8 1</ambient>
               <diffuse>0.9 0.9 0.9 1</diffuse>
             </material>
           </visual>
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.1 4 1</size>
               </box>
             </geometry>
           </collision>
           <inertial>
             <mass>10.0</mass>
             <inertia>
               <ixx>13.37</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>12.54</iyy>
               <iyz>0</iyz>
               <izz>0.87</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <model name="wall2">
         <pose>-2 0 0.5 0 0 0</pose>
         <link name="link">
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.1 4 1</size>
               </box>
             </geometry>
             <material>
               <ambient>0.8 0.8 0.8 1</ambient>
               <diffuse>0.9 0.9 0.9 1</diffuse>
             </material>
           </visual>
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.1 4 1</size>
               </box>
             </geometry>
           </collision>
           <inertial>
             <mass>10.0</mass>
             <inertia>
               <ixx>13.37</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>12.54</iyy>
               <iyz>0</iyz>
               <izz>0.87</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <model name="wall3">
         <pose>0 2 0.5 0 0 1.5707</pose>
         <link name="link">
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.1 4 1</size>
               </box>
             </geometry>
             <material>
               <ambient>0.8 0.8 0.8 1</ambient>
               <diffuse>0.9 0.9 0.9 1</diffuse>
             </material>
           </visual>
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.1 4 1</size>
               </box>
             </geometry>
           </collision>
           <inertial>
             <mass>10.0</mass>
             <inertia>
               <ixx>13.37</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>12.54</iyy>
               <iyz>0</iyz>
               <izz>0.87</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <!-- Cylinder obstacles -->
       <model name="cylinder1">
         <pose>1 1 0.5 0 0 0</pose>
         <link name="link">
           <visual name="visual">
             <geometry>
               <cylinder>
                 <radius>0.3</radius>
                 <length>1.0</length>
               </cylinder>
             </geometry>
             <material>
               <ambient>0.8 0.3 0.1 1</ambient>
               <diffuse>1.0 0.5 0.3 1</diffuse>
             </material>
           </visual>
           <collision name="collision">
             <geometry>
               <cylinder>
                 <radius>0.3</radius>
                 <length>1.0</length>
               </cylinder>
             </geometry>
           </collision>
           <inertial>
             <mass>5.0</mass>
             <inertia>
               <ixx>0.625</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.625</iyy>
               <iyz>0</iyz>
               <izz>0.225</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <model name="cylinder2">
         <pose>-1 -1 0.5 0 0 0</pose>
         <link name="link">
           <visual name="visual">
             <geometry>
               <cylinder>
                 <radius>0.3</radius>
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
                 <radius>0.3</radius>
                 <length>1.0</length>
               </cylinder>
             </geometry>
           </collision>
           <inertial>
             <mass>5.0</mass>
             <inertia>
               <ixx>0.625</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.625</iyy>
               <iyz>0</iyz>
               <izz>0.225</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <!-- Boxes for navigation challenge -->
       <model name="box1">
         <pose>0 1.5 0.3 0 0 0</pose>
         <link name="link">
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.4 0.4 0.6</size>
               </box>
             </geometry>
             <material>
               <ambient>0.1 0.1 0.8 1</ambient>
               <diffuse>0.3 0.3 1.0 1</diffuse>
             </material>
           </visual>
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.4 0.4 0.6</size>
               </box>
             </geometry>
           </collision>
           <inertial>
             <mass>2.0</mass>
             <inertia>
               <ixx>0.067</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.067</iyy>
               <iyz>0</iyz>
               <izz>0.067</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <model name="box2">
         <pose>1.5 0 0.3 0 0 0</pose>
         <link name="link">
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.4 0.4 0.6</size>
               </box>
             </geometry>
             <material>
               <ambient>0.8 0.1 0.1 1</ambient>
               <diffuse>1.0 0.3 0.3 1</diffuse>
             </material>
           </visual>
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.4 0.4 0.6</size>
               </box>
             </geometry>
           </collision>
           <inertial>
             <mass>2.0</mass>
             <inertia>
               <ixx>0.067</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.067</iyy>
               <iyz>0</iyz>
               <izz>0.067</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <!-- Goal marker -->
       <model name="goal">
         <pose>2 -2 0.1 0 0 0</pose>
         <link name="link">
           <visual name="visual">
             <geometry>
               <cylinder>
                 <radius>0.1</radius>
                 <length>0.2</length>
               </cylinder>
             </geometry>
             <material>
               <ambient>0.0 0.8 0.0 1</ambient>
               <diffuse>0.2 1.0 0.2 1</diffuse>
             </material>
           </visual>
           <collision name="collision">
             <geometry>
               <cylinder>
                 <radius>0.1</radius>
                 <length>0.2</length>
               </cylinder>
             </geometry>
           </collision>
           <inertial>
             <mass>0.1</mass>
             <inertia>
               <ixx>0.001</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.001</iyy>
               <iyz>0</iyz>
               <izz>0.0005</izz>
             </inertia>
           </inertial>
         </link>
       </model>
     </world>
   </sdf>
   ```

#### Step 2: Create a Launch File for the Custom World
1. Create a launch file `robot_description/launch/obstacle_course.launch.py`:
   ```python
   #!/usr/bin/env python3
   """
   Launch file to run robot in obstacle course world
   """
   import os
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
           default_value='obstacle_course.sdf',
           description='Choose one of the world files from `/robot_description/worlds`'
       )

       # Launch Gazebo with custom world
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
                   FindPackageShare('robot_description'),
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
                   FindPackageShare('robot_description'),
                   'urdf',
                   'sensor_robot.urdf'
               ])
           ]
       )

       # Spawn entity
       spawn_entity = Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=[
               '-topic', 'robot_description',
               '-entity', 'sensor_robot',
               '-x', '0', '-y', '0', '-z', '0.1'
           ],
           output='screen'
       )

       # Return the complete launch description
       return LaunchDescription([
           world_arg,
           gazebo,
           robot_state_publisher,
           spawn_entity
       ])
   ```

#### Step 3: Create Navigation Test Node
1. Create a navigation test script in `robot_description/robot_description/navigation_test.py`:
   ```python
   #!/usr/bin/env python3
   """
   Navigation test for obstacle course
   """
   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Twist, Pose
   from nav_msgs.msg import Odometry
   from sensor_msgs.msg import LaserScan
   import math


   class NavigationTest(Node):
       def __init__(self):
           super().__init__('navigation_test')

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

           # Internal state
           self.current_pose = Pose()
           self.current_scan = LaserScan()
           self.nav_state = 'searching'  # searching, avoiding, approaching_goal
           self.goal_x = 2.0
           self.goal_y = -2.0
           self.goal_tolerance = 0.3

           # Timer for navigation
           self.nav_timer = self.create_timer(0.1, self.navigation_loop)

           self.get_logger().info('Navigation Test Node initialized')

       def odom_callback(self, msg):
           """Update current pose from odometry"""
           self.current_pose = msg.pose.pose

       def scan_callback(self, msg):
           """Update laser scan data"""
           self.current_scan = msg

       def navigation_loop(self):
           """Main navigation loop"""
           cmd_vel = Twist()

           # Calculate distance to goal
           dx = self.goal_x - self.current_pose.position.x
           dy = self.goal_y - self.current_pose.position.y
           distance_to_goal = math.sqrt(dx*dx + dy*dy)

           # Check if goal reached
           if distance_to_goal < self.goal_tolerance:
               self.get_logger().info('Goal reached!')
               cmd_vel.linear.x = 0.0
               cmd_vel.angular.z = 0.0
               self.nav_state = 'completed'
           else:
               # Simple obstacle avoidance
               if len(self.current_scan.ranges) > 0:
                   # Check front, left, and right ranges
                   front_range = min(self.current_scan.ranges[0:10] + self.current_scan.ranges[-10:])
                   left_range = min(self.current_scan.ranges[45:90])
                   right_range = min(self.current_scan.ranges[-90:-45])

                   # If obstacle detected in front, turn away
                   if front_range < 0.8:
                       self.nav_state = 'avoiding'
                       if left_range > right_range:
                           cmd_vel.angular.z = 0.5  # Turn left
                           cmd_vel.linear.x = 0.0
                       else:
                           cmd_vel.angular.z = -0.5  # Turn right
                           cmd_vel.linear.x = 0.0
                   else:
                       # Navigate toward goal
                       self.nav_state = 'approaching_goal'
                       target_angle = math.atan2(dy, dx)

                       # Simple proportional controller for orientation
                       cmd_vel.linear.x = 0.5  # Move forward
                       cmd_vel.angular.z = target_angle * 1.0  # Turn toward goal

           self.cmd_vel_pub.publish(cmd_vel)
           self.get_logger().info(
               f'Navigation state: {self.nav_state}, '
               f'Pos: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}), '
               f'Dist to goal: {distance_to_goal:.2f}m'
           )


   def main(args=None):
       rclpy.init(args=args)
       nav_test = NavigationTest()

       try:
           rclpy.spin(nav_test)
       except KeyboardInterrupt:
           pass
       finally:
           nav_test.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Make the script executable:
   ```bash
   chmod +x robot_description/robot_description/navigation_test.py
   ```

#### Step 4: Update Setup Files
1. Update `setup.py` to include the new script:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   from ament_index_python.packages import get_package_share_directory

   package_name = 'robot_description'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
           ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
           ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='Robot description for Physical AI',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'navigation_test = robot_description.navigation_test:main',
           ],
       },
   )
   ```

#### Step 5: Test the Complex Environment
1. Build the package:
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select robot_description
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Launch the obstacle course:
   ```bash
   ros2 launch robot_description obstacle_course.launch.py
   ```

4. In another terminal, run the navigation test:
   ```bash
   ros2 run robot_description navigation_test
   ```

### Expected Results
- Robot spawns in the complex obstacle course environment
- Robot navigates around obstacles to reach the goal
- Sensor data helps with obstacle detection
- Robot successfully completes the course

### Analysis Questions
1. How do the physics parameters in the world file affect simulation behavior?
2. What challenges arise when navigating complex environments compared to simple ones?
3. How would you improve the navigation algorithm for better obstacle avoidance?

---

## Lab 3.4: Simulation Validation and Reality Gap Analysis

### Objective
Validate simulation accuracy by comparing simulation behavior with expected physical behavior and analyze the reality gap.

### Prerequisites
- Completed previous labs
- Understanding of simulation concepts
- Basic Python programming skills

### Estimated Time
2.5 hours

### Steps

#### Step 1: Create a Validation Test Suite
1. Create a validation script in `robot_description/robot_description/validation_test.py`:
   ```python
   #!/usr/bin/env python3
   """
   Validation test for simulation accuracy
   """
   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Twist, Pose
   from nav_msgs.msg import Odometry
   import time
   import math


   class SimulationValidator(Node):
       def __init__(self):
           super().__init__('simulation_validator')

           # Publishers
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

           # Subscribers
           self.odom_sub = self.create_subscription(
               Odometry,
               'odom',
               self.odom_callback,
               10
           )

           # Internal state
           self.start_pose = Pose()
           self.current_pose = Pose()
           self.initial_time = self.get_clock().now()
           self.start_time = None
           self.test_stage = 0  # 0: idle, 1: moving forward, 2: turning, 3: validation
           self.test_results = {'position_error': [], 'velocity_error': [], 'timing_error': []}
           self.test_complete = False

           # Test parameters
           self.forward_distance = 2.0  # meters
           self.turn_angle = math.pi / 2  # 90 degrees
           self.linear_speed = 0.5  # m/s
           self.angular_speed = 0.2  # rad/s
           self.test_duration = 20.0  # seconds

           # Timer for validation
           self.validation_timer = self.create_timer(0.1, self.validation_loop)

           self.get_logger().info('Simulation Validator initialized')

       def odom_callback(self, msg):
           """Update current pose from odometry"""
           self.current_pose = msg.pose.pose

       def validation_loop(self):
           """Main validation loop"""
           if self.start_time is None:
               # Initialize test
               self.start_pose = self.current_pose
               self.start_time = self.get_clock().now()
               self.test_stage = 1
               self.get_logger().info('Starting validation test')

           current_time = self.get_clock().now()
           elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

           if elapsed_time > self.test_duration:
               self.test_complete = True
               self.publish_results()
               return

           cmd_vel = Twist()

           if self.test_stage == 1:  # Move forward test
               # Calculate expected position
               expected_x = self.start_pose.position.x + self.linear_speed * elapsed_time
               expected_y = self.start_pose.position.y

               # Calculate actual position
               actual_x = self.current_pose.position.x
               actual_y = self.current_pose.position.y

               # Check if target distance reached
               dx = actual_x - self.start_pose.position.x
               dy = actual_y - self.start_pose.position.y
               distance_traveled = math.sqrt(dx*dx + dy*dy)

               if distance_traveled >= self.forward_distance:
                   cmd_vel.linear.x = 0.0
                   cmd_vel.angular.z = 0.0
                   self.test_stage = 2
                   self.get_logger().info('Forward movement test completed')
                   # Log position error
                   expected_pos_error = abs((expected_x - self.start_pose.position.x) - distance_traveled)
                   self.test_results['position_error'].append(expected_pos_error)
                   self.get_logger().info(f'Position error: {expected_pos_error:.3f}m')
               else:
                   cmd_vel.linear.x = self.linear_speed
                   cmd_vel.angular.z = 0.0

           elif self.test_stage == 2:  # Turn test
               cmd_vel.linear.x = 0.0
               cmd_vel.angular.z = self.angular_speed

               # Simulate turning for a fixed time to achieve 90 degrees
               time_in_turn = elapsed_time - self.get_time_at_stage(1)
               if time_in_turn > self.turn_angle / self.angular_speed:
                   cmd_vel.linear.x = 0.0
                   cmd_vel.angular.z = 0.0
                   self.test_stage = 3
                   self.get_logger().info('Turning test completed')

           elif self.test_stage == 3:  # Validation
               cmd_vel.linear.x = 0.0
               cmd_vel.angular.z = 0.0
               self.test_complete = True
               self.publish_results()

           # Publish command
           self.cmd_vel_pub.publish(cmd_vel)

       def get_time_at_stage(self, stage):
           """Get time when entering a specific stage (simplified)"""
           return 0.0

       def publish_results(self):
           """Publish validation results"""
           if len(self.test_results['position_error']) > 0:
               avg_pos_error = sum(self.test_results['position_error']) / len(self.test_results['position_error'])
               self.get_logger().info(f'Average position error: {avg_pos_error:.3f}m')

               if avg_pos_error < 0.1:  # Less than 10cm error
                   self.get_logger().info('Simulation validation: PASSED')
               else:
                   self.get_logger().info('Simulation validation: FAILED - High position error')


   def main(args=None):
       rclpy.init(args=args)
       validator = SimulationValidator()

       try:
           rclpy.spin(validator)
       except KeyboardInterrupt:
           pass
       finally:
           validator.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Make the script executable:
   ```bash
   chmod +x robot_description/robot_description/validation_test.py
   ```

#### Step 2: Update Setup Files
1. Update `setup.py` to include the validation script:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   from ament_index_python.packages import get_package_share_directory

   package_name = 'robot_description'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
           ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
           ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='Robot description for Physical AI',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'navigation_test = robot_description.navigation_test:main',
               'validation_test = robot_description.validation_test:main',
           ],
       },
   )
   ```

#### Step 3: Build and Test Validation
1. Build the package:
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select robot_description
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Launch the basic simulation:
   ```bash
   ros2 launch robot_description spawn_robot.launch.py
   ```

4. In another terminal, run the validation test:
   ```bash
   ros2 run robot_description validation_test
   ```

### Expected Results
- Validation test runs through predetermined movements
- Position errors are calculated and reported
- Simulation accuracy is assessed
- Pass/fail status is determined based on error thresholds

### Analysis Questions
1. What are the main sources of simulation error you observed?
2. How could you improve the physics parameters to reduce the reality gap?
3. What other validation tests would you design for more comprehensive assessment?

### Safety Considerations
- Always validate simulation behavior before applying to real robots
- Document any significant differences between simulation and reality
- Use multiple validation methods to ensure comprehensive assessment
- Regularly update simulation models based on real-world performance