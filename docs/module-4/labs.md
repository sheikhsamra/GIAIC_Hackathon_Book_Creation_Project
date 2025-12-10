# Lab Exercises: NVIDIA Isaac Platform and Tools

## Lab 4.1: Installing and Configuring Isaac Sim

### Objective
Install NVIDIA Isaac Sim, configure the environment, and run a basic simulation with a robot model.

### Prerequisites
- NVIDIA GPU with CUDA support (RTX series recommended)
- Ubuntu 20.04 or 22.04
- NVIDIA drivers installed
- Omniverse Launcher installed

### Estimated Time
3 hours

### Steps

#### Step 1: Verify System Requirements
1. Check GPU and driver:
   ```bash
   nvidia-smi
   ```

2. Verify CUDA installation:
   ```bash
   nvcc --version
   ```

3. Check available disk space (minimum 10GB for Isaac Sim):
   ```bash
   df -h
   ```

#### Step 2: Install Omniverse Launcher and Isaac Sim
1. Download Omniverse Launcher from NVIDIA Developer website
2. Install the launcher:
   ```bash
   # Follow the installation instructions from NVIDIA
   # Launch the Omniverse Launcher
   ```

3. In Omniverse Launcher:
   - Sign in with your NVIDIA Developer account
   - Install Isaac Sim from the applications list
   - Wait for the installation to complete (this may take 30+ minutes)

#### Step 3: Verify Isaac Sim Installation
1. Launch Isaac Sim from Omniverse Launcher
2. Check that the application opens without errors
3. Verify that RTX rendering is working properly
4. Test basic navigation controls:
   - Orbit: Right mouse button + drag
   - Pan: Middle mouse button + drag
   - Zoom: Mouse wheel or Ctrl + right mouse button + drag

#### Step 4: Create a Basic Scene
1. In Isaac Sim, create a new stage
2. Add a simple robot:
   - Go to Window → Isaac Examples → Assets
   - Add a Jetbot or other sample robot to the stage
   - Position the robot at coordinates (0, 0, 0.1)

3. Add a ground plane:
   - Create → Primitives → Plane
   - Scale appropriately (e.g., 10x10 meters)

4. Add lighting:
   - Create → Light → Distant Light
   - Adjust direction to simulate sun

#### Step 5: Configure ROS 2 Bridge
1. In Isaac Sim, go to Window → Extensions
2. Enable the ROS 2 Bridge extension
3. Configure the ROS domain ID (default: 0)
4. Verify that the extension loads without errors

#### Step 6: Test Basic Simulation
1. Press the Play button to start simulation
2. Verify that physics are running (objects should respond to gravity)
3. Stop the simulation
4. Save the scene as `basic_robot_scene.usd`

### Expected Results
- Isaac Sim launches successfully
- Basic scene with robot and environment is created
- ROS 2 bridge is configured and functional
- Simulation runs with proper physics

### Troubleshooting
1. If Isaac Sim fails to launch:
   - Verify GPU drivers are up to date
   - Check that RTX features are supported
   - Ensure sufficient disk space is available

2. If ROS 2 bridge doesn't work:
   - Verify ROS 2 Humble is installed
   - Check that the Isaac Sim ROS bridge is properly configured
   - Ensure ROS_DOMAIN_ID is set correctly

### Analysis Questions
1. What are the advantages of Isaac Sim over traditional Gazebo?
2. How does domain randomization help bridge the reality gap?
3. What are the key differences between Isaac Sim and other simulators?

---

## Lab 4.2: Isaac ROS Perception Pipeline

### Objective
Create and test an Isaac ROS perception pipeline with GPU-accelerated image processing and object detection.

### Prerequisites
- Completed Lab 4.1
- Isaac Sim running
- ROS 2 Humble installed
- Isaac ROS packages installed

### Estimated Time
4 hours

### Steps

#### Step 1: Set Up Isaac ROS Environment
1. Verify Isaac ROS packages are installed:
   ```bash
   # Check available Isaac ROS packages
   apt list --installed | grep isaac
   ```

2. Source ROS 2 and Isaac ROS:
   ```bash
   source /opt/ros/humble/setup.bash
   # Source Isaac ROS if installed separately
   ```

#### Step 2: Create a Perception Package
1. Create a new ROS 2 package:
   ```bash
   cd ~/isaac_ws/src
   ros2 pkg create --build-type ament_python isaac_perception_examples --dependencies rclpy sensor_msgs cv_bridge vision_msgs message_filters
   ```

2. Navigate to the package directory:
   ```bash
   cd isaac_perception_examples
   ```

#### Step 3: Create GPU-Accelerated Image Processing Node
1. Create an image processing script `isaac_perception_examples/isaac_perception_examples/image_processor.py`:
   ```python
   #!/usr/bin/env python3
   """
   Isaac ROS GPU-accelerated image processor
   """
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from cv_bridge import CvBridge
   import cv2
   import numpy as np


   class IsaacImageProcessor(Node):
       def __init__(self):
           super().__init__('isaac_image_processor')

           # Publishers
           self.processed_pub = self.create_publisher(Image, 'processed_image', 10)

           # Subscribers
           self.image_sub = self.create_subscription(
               Image,
               'input_image',
               self.image_callback,
               10
           )

           # Internal state
           self.cv_bridge = CvBridge()

           self.get_logger().info('Isaac Image Processor initialized')

       def image_callback(self, msg):
           """Process incoming image with GPU-accelerated operations"""
           try:
               # Convert ROS image to OpenCV
               cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

               # Apply GPU-accelerated operations (simulated here)
               # In actual Isaac ROS, these would use CUDA
               processed_image = self.apply_gpu_processing(cv_image)

               # Publish processed image
               processed_msg = self.cv_bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
               processed_msg.header = msg.header
               self.processed_pub.publish(processed_msg)

               self.get_logger().info('Image processed and published')

           except Exception as e:
               self.get_logger().error('Error processing image: {}'.format(e))

       def apply_gpu_processing(self, image):
           """Apply GPU-accelerated image processing (simulated)"""
           # In actual Isaac ROS, this would use CUDA operations
           # For this example, we'll simulate with OpenCV operations

           # Apply some processing that would typically be GPU-accelerated
           # Gaussian blur (simulating CUDA-based filtering)
           blurred = cv2.GaussianBlur(image, (5, 5), 0)

           # Convert to grayscale and back (simulating channel operations)
           gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
           processed = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

           return processed


   def main(args=None):
       rclpy.init(args=args)
       processor = IsaacImageProcessor()

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

2. Make the script executable:
   ```bash
   chmod +x isaac_perception_examples/isaac_perception_examples/image_processor.py
   ```

#### Step 4: Create Isaac ROS Stereo Processing Node
1. Create a stereo processing script `isaac_perception_examples/isaac_perception_examples/stereo_processor.py`:
   ```python
   #!/usr/bin/env python3
   """
   Isaac ROS GPU-accelerated stereo processor
   """
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from stereo_msgs.msg import DisparityImage
   from cv_bridge import CvBridge
   import numpy as np
   import cv2


   class IsaacStereoProcessor(Node):
       def __init__(self):
           super().__init__('isaac_stereo_processor')

           # Publishers
           self.disparity_pub = self.create_publisher(DisparityImage, 'disparity', 10)

           # Subscribers
           self.left_sub = self.create_subscription(
               Image,
               'left/image_rect',
               self.left_callback,
               10
           )

           self.right_sub = self.create_subscription(
               Image,
               'right/image_rect',
               self.right_callback,
               10
           )

           # Internal state
           self.cv_bridge = CvBridge()
           self.left_image = None
           self.right_image = None

           # Stereo parameters (these would be GPU-accelerated in Isaac ROS)
           self.block_size = 11
           self.min_disparity = 0
           self.num_disparities = 64

           self.get_logger().info('Isaac Stereo Processor initialized')

       def left_callback(self, msg):
           """Process left camera image"""
           try:
               self.left_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
               self.process_stereo()
           except Exception as e:
               self.get_logger().error('Error processing left image: {}'.format(e))

       def right_callback(self, msg):
           """Process right camera image"""
           try:
               self.right_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
               self.process_stereo()
           except Exception as e:
               self.get_logger().error('Error processing right image: {}'.format(e))

       def process_stereo(self):
           """Process stereo images to generate disparity"""
           if self.left_image is None or self.right_image is None:
               return

           if self.left_image.shape != self.right_image.shape:
               self.get_logger().error('Left and right images have different shapes')
               return

           # In actual Isaac ROS, this would use GPU-accelerated stereo
           # For demonstration, using OpenCV (but Isaac ROS uses CUDA)
           stereo = cv2.StereoSGBM_create(
               minDisparity=self.min_disparity,
               numDisparities=self.num_disparities,
               blockSize=self.block_size,
               P1=8 * 3 * self.block_size ** 2,
               P2=32 * 3 * self.block_size ** 2,
               disp12MaxDiff=1,
               uniquenessRatio=15,
               speckleWindowSize=0,
               speckleRange=2,
               preFilterCap=63,
               mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
           )

           # Compute disparity (GPU-accelerated in Isaac ROS)
           disparity = stereo.compute(self.left_image, self.right_image).astype(np.float32) / 16.0

           # Create and publish disparity message
           disparity_msg = DisparityImage()
           disparity_msg.header = self.left_sub.header  # Should use proper header
           disparity_msg.image = self.cv_bridge.cv2_to_imgmsg(disparity, encoding='32FC1')
           disparity_msg.f = 1.0  # Should come from camera info
           disparity_msg.T = 0.1  # Should come from stereo baseline
           disparity_msg.min_disparity = float(self.min_disparity)
           disparity_msg.max_disparity = float(self.min_disparity + self.num_disparities)
           disparity_msg.delta_d = 0.125

           self.disparity_pub.publish(disparity_msg)
           self.get_logger().info('Disparity computed, range: {:.2f} to {:.2f}'.format(disparity.min(), disparity.max()))


   def main(args=None):
       rclpy.init(args=args)
       processor = IsaacStereoProcessor()

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

2. Make the script executable:
   ```bash
   chmod +x isaac_perception_examples/isaac_perception_examples/stereo_processor.py
   ```

#### Step 5: Update Setup Files
1. Update `setup.py` to include the new executables:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   from ament_index_python.packages import get_package_share_directory

   package_name = 'isaac_perception_examples'

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
       description='Isaac ROS perception examples',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'image_processor = isaac_perception_examples.image_processor:main',
               'stereo_processor = isaac_perception_examples.stereo_processor:main',
           ],
       },
   )
   ```

#### Step 6: Build and Test the Perception Pipeline
1. Build the package:
   ```bash
   cd ~/isaac_ws
   colcon build --packages-select isaac_perception_examples
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Test the image processor (in one terminal):
   ```bash
   ros2 run isaac_perception_examples image_processor
   ```

4. Test the stereo processor (in another terminal):
   ```bash
   ros2 run isaac_perception_examples stereo_processor
   ```

### Expected Results
- Image processor node runs without errors
- Stereo processor node processes stereo images and publishes disparity
- Both nodes demonstrate GPU-accelerated processing concepts
- Performance is significantly better than CPU-only processing

### Analysis Questions
1. How does GPU acceleration improve perception pipeline performance?
2. What are the limitations of GPU-accelerated processing?
3. How would you optimize the pipeline for real-time applications?

---

## Lab 4.3: Isaac Navigation Implementation

### Objective
Configure and test Isaac Navigation with GPU-accelerated path planning and obstacle avoidance.

### Prerequisites
- Completed previous labs
- Isaac ROS perception pipeline working
- Basic understanding of ROS 2 navigation

### Estimated Time
5 hours

### Steps

#### Step 1: Install Isaac Navigation Packages
1. Install Isaac Navigation packages:
   ```bash
   sudo apt update
   sudo apt install ros-humble-isaac-nav-*
   ```

2. Verify installation:
   ```bash
   ros2 pkg list | grep nav
   ```

#### Step 2: Create Navigation Configuration
1. Create a navigation config package:
   ```bash
   cd ~/isaac_ws/src
   ros2 pkg create --build-type ament_cmake isaac_navigation_config --dependencies nav2_bringup
   ```

2. Create navigation configuration files in `isaac_navigation_config/config/`:
   - Create the config directory:
     ```bash
     mkdir -p isaac_navigation_config/config
     ```

3. Create `isaac_navigation_config/config/nav2_params.yaml`:
   ```yaml
   amcl:
     ros__parameters:
       use_sim_time: True
       alpha1: 0.2
       alpha2: 0.2
       alpha3: 0.2
       alpha4: 0.2
       alpha5: 0.2
       base_frame_id: "base_footprint"
       beam_skip_distance: 0.5
       beam_skip_error_threshold: 0.9
       beam_skip_threshold: 0.3
       do_beamskip: false
       global_frame_id: "map"
       lambda_short: 0.1
       laser_likelihood_max_dist: 2.0
       laser_max_range: 100.0
       laser_min_range: -1.0
       laser_model_type: "likelihood_field"
       max_beams: 60
       max_particles: 2000
       min_particles: 500
       odom_frame_id: "odom"
       pf_err: 0.05
       pf_z: 0.5
       recovery_alpha_fast: 0.0
       recovery_alpha_slow: 0.0
       resample_interval: 1
       robot_model_type: "nav2_amcl::DifferentialMotionModel"
       save_pose_rate: 0.5
       sigma_hit: 0.2
       tf_broadcast: true
       transform_tolerance: 1.0
       update_min_a: 0.2
       update_min_d: 0.25
       z_hit: 0.5
       z_max: 0.05
       z_rand: 0.5
       z_short: 0.05
       scan_topic: scan

   amcl_map_client:
     ros__parameters:
       use_sim_time: True

   amcl_rclcpp_node:
     ros__parameters:
       use_sim_time: True

   bt_navigator:
     ros__parameters:
       use_sim_time: True
       global_frame: map
       robot_base_frame: base_link
       odom_topic: /odom
       bt_loop_duration: 10
       default_server_timeout: 20
       enable_groot_monitoring: True
       groot_zmq_publisher_port: 1666
       groot_zmq_server_port: 1667
       navigate_through_poses_action_name: navigate_through_poses
       navigate_to_pose_action_name: navigate_to_pose
       goal_updater_plugin: "goal_updater"
       goal_checker_plugin: "goal_checker"
       controller_frequency: 20.0
       controller_plugin: "controller_server"
       goal_checker:
         plugin: "nav2_controller::SimpleGoalChecker"
         xy_goal_tolerance: 0.25
         yaw_goal_tolerance: 0.25
         stateful: True
       controller_server:
         plugin: "nav2_controller::ControllerServer"
         controller_frequency: 20.0
         min_x_velocity_threshold: 0.001
         min_y_velocity_threshold: 0.5
         min_theta_velocity_threshold: 0.001
         failure_tolerance: 0.3
         progress_checker_plugin: "progress_checker"
         goal_checker_plugin: "goal_checker"
         controller_plugins: ["FollowPath"]
         FollowPath:
           plugin: "nav2_mppi_controller::MPPIController"
           time_steps: 50
           model_dt: 0.05
           vx_std: 0.2
           vy_std: 0.05
           wz_std: 0.3
           vx_max: 0.5
           vx_min: -0.3
           vy_max: 0.3
           wz_max: 1.0
           simulation_time: 1.6
           control_frequency: 20.0
           discretize_by_angle: false
           discretize_by_distance: false
           ndt_breadth_first: false
           temperature: 0.3
           gamma: 0.015
           motion_model: "DiffDrive"
           trajectory_visualization_enabled: true
           # ISAAC SPECIFIC: Enable GPU acceleration
           isaac_gpu_enabled: true

   controller_server:
     ros__parameters:
       use_sim_time: True
       controller_frequency: 20.0
       min_x_velocity_threshold: 0.001
       min_y_velocity_threshold: 0.5
       min_theta_velocity_threshold: 0.001
       failure_tolerance: 0.3
       progress_checker_plugin: "progress_checker"
       goal_checker_plugin: "goal_checker"
       controller_plugins: ["FollowPath"]
       progress_checker:
         plugin: "nav2_controller::SimpleProgressChecker"
         required_movement_radius: 0.5
         movement_time_allowance: 10.0
       goal_checker:
         plugin: "nav2_controller::SimpleGoalChecker"
         xy_goal_tolerance: 0.25
         yaw_goal_tolerance: 0.25
         stateful: True
       FollowPath:
         plugin: "nav2_mppi_controller::MPPIController"
         time_steps: 50
         model_dt: 0.05
         vx_std: 0.2
         vy_std: 0.05
         wz_std: 0.3
         vx_max: 0.5
         vx_min: -0.3
         vy_max: 0.3
         wz_max: 1.0
         simulation_time: 1.6
         control_frequency: 20.0
         discretize_by_angle: false
         discretize_by_distance: false
         ndt_breadth_first: false
         temperature: 0.3
         gamma: 0.015
         motion_model: "DiffDrive"
         trajectory_visualization_enabled: true
         # ISAAC SPECIFIC: Enable GPU acceleration
         isaac_gpu_enabled: true

   local_costmap:
     local_costmap:
       ros__parameters:
         update_frequency: 5.0
         publish_frequency: 2.0
         global_frame: odom
         robot_base_frame: base_link
         use_sim_time: True
         rolling_window: true
         width: 3
         height: 3
         resolution: 0.05
         robot_radius: 0.22
         plugins: ["voxel_layer", "inflation_layer"]
         inflation_layer:
           plugin: "nav2_costmap_2d::InflationLayer"
           cost_scaling_factor: 3.0
           inflation_radius: 0.55
         voxel_layer:
           plugin: "nav2_costmap_2d::VoxelLayer"
           enabled: True
           publish_voxel_map: True
           origin_z: 0.0
           z_resolution: 0.05
           z_voxels: 16
           max_obstacle_height: 2.0
           mark_threshold: 0
           observation_sources: scan
           scan:
             topic: /scan
             max_obstacle_height: 2.0
             clearing: True
             marking: True
             data_type: "LaserScan"
             raytrace_max_range: 3.0
             raytrace_min_range: 0.0
             obstacle_max_range: 2.5
             obstacle_min_range: 0.0
         always_send_full_costmap: True
     local_costmap_client:
       ros__parameters:
         use_sim_time: True
     local_costmap_rclcpp_node:
       ros__parameters:
         use_sim_time: True

   global_costmap:
     global_costmap:
       ros__parameters:
         update_frequency: 1.0
         publish_frequency: 1.0
         global_frame: map
         robot_base_frame: base_link
         use_sim_time: True
         robot_radius: 0.22
         resolution: 0.05
         track_unknown_space: true
         plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
         obstacle_layer:
           plugin: "nav2_costmap_2d::ObstacleLayer"
           enabled: True
           observation_sources: scan
           scan:
             topic: /scan
             max_obstacle_height: 2.0
             clearing: True
             marking: True
             data_type: "LaserScan"
             raytrace_max_range: 3.0
             raytrace_min_range: 0.0
             obstacle_max_range: 2.5
             obstacle_min_range: 0.0
         static_layer:
           plugin: "nav2_costmap_2d::StaticLayer"
           map_subscribe_transient_local: True
         inflation_layer:
           plugin: "nav2_costmap_2d::InflationLayer"
           cost_scaling_factor: 3.0
           inflation_radius: 0.55
         always_send_full_costmap: True
     global_costmap_client:
       ros__parameters:
         use_sim_time: True
     global_costmap_rclcpp_node:
       ros__parameters:
         use_sim_time: True

   planner_server:
     ros__parameters:
       expected_planner_frequency: 20.0
       use_sim_time: True
       planner_plugins: ["GridBased"]
       GridBased:
         plugin: "nav2_navfn_planner/NavfnPlanner"
         tolerance: 0.5
         use_astar: false
         allow_unknown: true
         # ISAAC SPECIFIC: Enable GPU acceleration
         isaac_gpu_enabled: true
         isaac_grid_resolution: 0.05

   behavior_server:
     ros__parameters:
       costmap_topic: local_costmap/costmap_raw
       footprint_topic: local_costmap/published_footprint
       cycle_frequency: 10.0
       behavior_plugins: ["spin", "backup", "wait"]
       spin:
         plugin: "nav2_behaviors/Spin"
         spin_dist: 1.57
       backup:
         plugin: "nav2_behaviors/BackUp"
         backup_dist: 0.15
         backup_speed: 0.025
       wait:
         plugin: "nav2_behaviors/Wait"
         wait_duration: 1s
   ```

#### Step 3: Create Navigation Launch File
1. Create a launch directory:
   ```bash
   mkdir -p isaac_navigation_config/launch
   ```

2. Create `isaac_navigation_config/launch/isaac_navigation.launch.py`:
   ```python
   #!/usr/bin/env python3
   """
   Isaac Navigation launch file
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
       use_sim_time_arg = DeclareLaunchArgument(
           'use_sim_time',
           default_value='true',
           description='Use simulation time'
       )

       params_file_arg = DeclareLaunchArgument(
           'params_file',
           default_value=PathJoinSubstitution([
               FindPackageShare('isaac_navigation_config'),
               'config',
               'nav2_params.yaml'
           ]),
           description='Full path to params file for navigation nodes'
       )

       # Launch navigation
       navigation_launch = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               PathJoinSubstitution([
                   FindPackageShare('nav2_bringup'),
                   'launch',
                   'navigation_launch.py'
               ])
           ]),
           launch_arguments={
               'use_sim_time': LaunchConfiguration('use_sim_time'),
               'params_file': LaunchConfiguration('params_file')
           }.items()
       )

       # Launch local costmap
       local_costmap = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               PathJoinSubstitution([
                   FindPackageShare('nav2_bringup'),
                   'launch',
                   'local_costmap.launch.py'
               ])
           ]),
           launch_arguments={
               'use_sim_time': LaunchConfiguration('use_sim_time'),
               'params_file': LaunchConfiguration('params_file')
           }.items()
       )

       # Launch global costmap
       global_costmap = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               PathJoinSubstitution([
                   FindPackageShare('nav2_bringup'),
                   'launch',
                   'global_costmap.launch.py'
               ])
           ]),
           launch_arguments={
               'use_sim_time': LaunchConfiguration('use_sim_time'),
               'params_file': LaunchConfiguration('params_file')
           }.items()
       )

       return LaunchDescription([
           use_sim_time_arg,
           params_file_arg,
           navigation_launch,
           local_costmap,
           global_costmap
       ])
   ```

#### Step 4: Create Navigation Test Node
1. Create a navigation test script in `isaac_navigation_examples/isaac_navigation_examples/navigation_test.py`:
   ```python
   #!/usr/bin/env python3
   """
   Isaac Navigation test node
   """
   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Twist, PoseStamped
   from nav2_msgs.action import NavigateToPose
   from sensor_msgs.msg import LaserScan
   from rclpy.action import ActionClient
   import math


   class IsaacNavigationTest(Node):
       def __init__(self):
           super().__init__('isaac_navigation_test')

           # Publishers
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

           # Subscribers
           self.scan_sub = self.create_subscription(
               LaserScan,
               'scan',
               self.scan_callback,
               10
           )

           # Action client for navigation
           self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

           # Internal state
           self.current_scan = LaserScan()
           self.safety_distance = 0.5
           self.navigation_active = False
           self.emergency_stop = False

           # Navigation goals (x, y coordinates)
           self.goals = [
               (2.0, 2.0),   # Goal 1
               (4.0, 0.0),   # Goal 2
               (0.0, 4.0),   # Goal 3
               (-2.0, -2.0), # Goal 4
           ]
           self.current_goal_index = 0

           # Timer for navigation control
           self.nav_timer = self.create_timer(0.1, self.navigation_control)

           self.get_logger().info('Isaac Navigation Test initialized')

       def scan_callback(self, msg):
           """Process laser scan for safety"""
           self.current_scan = msg

           # Check for obstacles in front
           if len(msg.ranges) > 0:
               # Front 30 degrees
               front_ranges = msg.ranges[:15] + msg.ranges[-15:]
               front_ranges = [r for r in front_ranges if 0.1 < r < 10.0]

               if front_ranges:
                   min_front_dist = min(front_ranges)

                   if min_front_dist < self.safety_distance:
                       self.emergency_stop = True
                       self.get_logger().warn('EMERGENCY STOP: Obstacle at {:.2f}m'.format(min_front_dist))
                       self.publish_emergency_stop()
                   else:
                       self.emergency_stop = False

       def publish_emergency_stop(self):
           """Publish emergency stop command"""
           cmd_vel = Twist()
           cmd_vel.linear.x = 0.0
           cmd_vel.angular.z = 0.0
           self.cmd_vel_pub.publish(cmd_vel)

       def navigation_control(self):
           """Main navigation control loop"""
           if self.emergency_stop:
               return

           if not self.navigation_active:
               if self.current_goal_index < len(self.goals):
                   goal = self.goals[self.current_goal_index]
                   self.get_logger().info('Sending navigation goal to ({}, {})'.format(goal[0], goal[1]))
                   self.send_navigation_goal(goal[0], goal[1])
                   self.current_goal_index += 1
               else:
                   self.get_logger().info('All navigation goals completed')
                   return

       def send_navigation_goal(self, x, y):
           """Send navigation goal"""
           # Wait for action server
           if not self.nav_client.wait_for_server(timeout_sec=5.0):
               self.get_logger().error('Navigation server not available')
               return

           # Create goal message
           goal_msg = NavigateToPose.Goal()
           goal_msg.pose.header.frame_id = 'map'
           goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
           goal_msg.pose.pose.position.x = float(x)
           goal_msg.pose.pose.position.y = float(y)
           goal_msg.pose.pose.position.z = 0.0
           goal_msg.pose.pose.orientation.w = 1.0  # No rotation

           # Send goal
           self.navigation_active = True
           future = self.nav_client.send_goal_async(
               goal_msg,
               feedback_callback=self.navigation_feedback
           )
           future.add_done_callback(self.navigation_response)

       def navigation_response(self, future):
           """Handle navigation goal response"""
           goal_handle = future.result()
           if not goal_handle.accepted:
               self.get_logger().info('Navigation goal rejected')
               self.navigation_active = False
               return

           self.get_logger().info('Navigation goal accepted')
           get_result_future = goal_handle.get_result_async()
           get_result_future.add_done_callback(self.navigation_result)

       def navigation_feedback(self, feedback_msg):
           """Handle navigation feedback"""
           feedback = feedback_msg.feedback
           self.get_logger().debug('Navigation progress: {}'.format(feedback.current_pose))

       def navigation_result(self, future):
           """Handle navigation result"""
           result = future.result().result
           self.navigation_active = False
           self.get_logger().info('Navigation completed with result: {}'.format(result))

           # Add delay before next goal
           self.create_timer(2.0, lambda: None)  # 2 second delay


   def main(args=None):
       rclpy.init(args=args)
       nav_test = IsaacNavigationTest()

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
   chmod +x isaac_navigation_examples/isaac_navigation_examples/navigation_test.py
   ```

#### Step 5: Update Navigation Package Setup
1. Update `isaac_navigation_examples/setup.py` to include the navigation test:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   from ament_index_python.packages import get_package_share_directory

   package_name = 'isaac_navigation_examples'

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
       description='Isaac Navigation examples',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'navigation_test = isaac_navigation_examples.navigation_test:main',
           ],
       },
   )
   ```

#### Step 6: Build and Test Navigation
1. Build the navigation packages:
   ```bash
   cd ~/isaac_ws
   colcon build --packages-select isaac_navigation_config isaac_navigation_examples
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Launch Isaac Navigation (in one terminal):
   ```bash
   ros2 launch isaac_navigation_config isaac_navigation.launch.py
   ```

4. Run the navigation test (in another terminal):
   ```bash
   ros2 run isaac_navigation_examples navigation_test
   ```

### Expected Results
- Isaac Navigation system launches successfully
- Robot navigates to specified goals while avoiding obstacles
- Safety systems prevent collisions
- Navigation performance benefits from GPU acceleration

### Analysis Questions
1. How does Isaac Navigation differ from standard ROS 2 navigation?
2. What are the advantages of GPU-accelerated path planning?
3. How does the system handle dynamic obstacles?

---

## Lab 4.4: Isaac Manipulation and AI Integration

### Objective
Implement Isaac Manipulation with AI-powered grasp planning and integration with Isaac AI tools.

### Prerequisites
- Completed previous labs
- Isaac Navigation working
- Basic understanding of MoveIt! and manipulation

### Estimated Time
6 hours

### Steps

#### Step 1: Install Isaac Manipulation Packages
1. Install Isaac Manipulation packages:
   ```bash
   sudo apt update
   sudo apt install ros-humble-isaac-manipulation-*
   ```

2. Install MoveIt! packages for manipulation:
   ```bash
   sudo apt install ros-humble-moveit*
   ```

#### Step 2: Create Manipulation Package
1. Create a manipulation package:
   ```bash
   cd ~/isaac_ws/src
   ros2 pkg create --build-type ament_python isaac_manipulation_examples --dependencies rclpy moveit_commander moveit_msgs sensor_msgs vision_msgs geometry_msgs
   ```

#### Step 3: Create AI-Powered Grasp Planning Node
1. Create a grasp planning script `isaac_manipulation_examples/isaac_manipulation_examples/grasp_planner.py`:
```python
#!/usr/bin/env python3
   """
   Isaac AI-powered grasp planner
   """
   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import PoseStamped, Point
   from moveit_msgs.msg import CollisionObject
   from moveit_msgs.srv import ApplyPlanningScene
   from sensor_msgs.msg import PointCloud2
   from vision_msgs.msg import Detection3DArray
   from std_msgs.msg import Header
   import numpy as np
   import tf2_ros
   from tf2_ros import TransformListener
   import tf2_geometry_msgs
   from moveit_commander import MoveGroupCommander, PlanningSceneInterface
   from geometry_msgs.msg import Pose, PoseArray
   import math


class IsaacGraspPlanner(Node):
    def __init__(self):
        super().__init__('isaac_grasp_planner')

        # Initialize MoveIt commander
        self.robot_commander = MoveGroupCommander("manipulator")
        self.scene = PlanningSceneInterface()

        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            'camera/depth/color/points',
            self.pointcloud_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection3DArray,
            'isaac_object_detection/detections',
            self.detection_callback,
            10
        )

        # Publishers
        self.grasp_poses_pub = self.create_publisher(PoseArray, 'candidate_grasps', 10)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal state
        self.objects = []
        self.candidate_grasps = []
        self.manipulation_active = False

        # Grasp planning parameters
        self.approach_distance = 0.1
        self.grasp_depth = 0.05
        self.grasp_width = 0.08
        self.grasp_quality_threshold = 0.5

        # Timer for grasp planning
        self.grasp_timer = self.create_timer(0.5, self.grasp_planning)

        self.get_logger().info('Isaac Grasp Planner initialized')

    def pointcloud_callback(self, msg):
        """Process point cloud data"""
        self.get_logger().debug('Point cloud received: {} points'.format(msg.width * msg.height))

    def detection_callback(self, msg):
        """Process 3D object detections"""
        self.objects = []

        for detection in msg.detections:
            if detection.results:
                # Get the best result
                result = detection.results[0]
                object_info = {
                    'id': result.id,
                    'confidence': result.score,
                    'pose': detection.pose,
                    'bbox': detection.bbox3d
                }
                self.objects.append(object_info)

                self.get_logger().info(
                    'Detected object: {} with confidence {:.2f}'.format(result.id, result.score)
                )

    def grasp_planning(self):
        """Main grasp planning loop"""
        if not self.objects:
            return

        # Process each detected object
        for obj in self.objects:
            if obj['confidence'] > 0.5:  # Confidence threshold
                candidate_grasps = self.generate_candidate_grasps(obj)
                self.candidate_grasps.extend(candidate_grasps)

                # Publish candidate grasps for visualization
                self.publish_candidate_grasps(candidate_grasps)

    def generate_candidate_grasps(self, obj):
        """Generate candidate grasps for an object"""
        candidate_grasps = []

        # Get object position
        obj_pos = obj['pose'].position
        obj_ori = obj['pose'].orientation

        # Generate multiple grasp candidates around the object
        for angle in np.linspace(0, 2*np.pi, 8):  # 8 grasp angles
            for height_offset in [0.0, 0.05, 0.1]:  # Different heights
                grasp_pose = Pose()

                # Calculate grasp position
                grasp_x = obj_pos.x + 0.15 * math.cos(angle)  # 15cm from object
                grasp_y = obj_pos.y + 0.15 * math.sin(angle)
                grasp_z = obj_pos.z + height_offset

                grasp_pose.position.x = grasp_x
                grasp_pose.position.y = grasp_y
                grasp_pose.position.z = grasp_z

                # Set orientation for top-down grasp
                grasp_pose.orientation.w = 1.0
                grasp_pose.orientation.x = 0.0
                grasp_pose.orientation.y = 0.707  # 90-degree rotation around Y
                grasp_pose.orientation.z = 0.0

                # Calculate grasp quality based on position relative to object
                grasp_quality = self.calculate_grasp_quality(grasp_pose, obj)

                if grasp_quality > self.grasp_quality_threshold:
                    candidate_grasps.append({
                        'pose': grasp_pose,
                        'quality': grasp_quality
                    })

        # Sort grasps by quality and return top candidates
        candidate_grasps.sort(key=lambda x: x['quality'], reverse=True)
        return candidate_grasps[:5]  # Return top 5 grasps

    def calculate_grasp_quality(self, grasp_pose, obj):
        """Calculate grasp quality based on geometric factors"""
        # Simple quality calculation based on distance and orientation
        obj_pos = obj['pose'].position

        # Distance factor (closer is better, but not too close)
        dist = math.sqrt(
            (grasp_pose.position.x - obj_pos.x)**2 +
            (grasp_pose.position.y - obj_pos.y)**2 +
            (grasp_pose.position.z - obj_pos.z)**2
        )

        # Distance should be within grasp range (e.g., 10-20cm)
        if 0.1 <= dist <= 0.2:
            distance_score = 1.0 - abs(dist - 0.15) / 0.05  # Score based on optimal distance
        else:
            distance_score = 0.0

        # Orientation factor (top-down grasp is preferred)
        # For this simple example, assume good orientation
        orientation_score = 1.0

        # Combine scores
        quality = 0.6 * distance_score + 0.4 * orientation_score
        return max(0.0, min(1.0, quality))  # Clamp to [0, 1]

    def publish_candidate_grasps(self, candidate_grasps):
        """Publish candidate grasps for visualization"""
        if not candidate_grasps:
            return

        grasp_array = PoseArray()
        grasp_array.header.frame_id = "base_link"
        grasp_array.header.stamp = self.get_clock().now().to_msg()

        for grasp in candidate_grasps:
            grasp_array.poses.append(grasp['pose'])

        self.grasp_poses_pub.publish(grasp_array)
        self.get_logger().info('Published {} candidate grasps'.format(len(candidate_grasps)))


def main(args=None):
    """Main function to initialize and run the grasp planner"""
    rclpy.init(args=args)
    grasp_planner = IsaacGraspPlanner()

    try:
        rclpy.spin(grasp_planner)
    except KeyboardInterrupt:
        pass
    finally:
        grasp_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

2. Make the script executable:
   ```bash
   chmod +x isaac_manipulation_examples/isaac_manipulation_examples/grasp_planner.py
   ```

#### Step 4: Create Isaac AI Integration Node
1. Create an AI integration script `isaac_manipulation_examples/isaac_manipulation_examples/ai_integrator.py`:
   ```python
   #!/usr/bin/env python3
   """
   Isaac AI integration node for manipulation
   """
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image, PointCloud2
   from vision_msgs.msg import Detection2DArray, Detection3DArray
   from geometry_msgs.msg import Pose
   from cv_bridge import CvBridge
   import numpy as np
   import cv2


   class IsaacAIIntegrator(Node):
       def __init__(self):
           super().__init__('isaac_ai_integrator')

           # Publishers
           self.detection_2d_pub = self.create_publisher(Detection2DArray, 'ai_detections_2d', 10)
           self.detection_3d_pub = self.create_publisher(Detection3DArray, 'ai_detections_3d', 10)

           # Subscribers
           self.image_sub = self.create_subscription(
               Image,
               'camera/image_raw',
               self.image_callback,
               10
           )

           self.pointcloud_sub = self.create_subscription(
               PointCloud2,
               'camera/depth/color/points',
               self.pointcloud_callback,
               10
           )

           # Internal state
           self.cv_bridge = CvBridge()
           self.detections_2d = []
           self.detections_3d = []

           self.get_logger().info('Isaac AI Integrator initialized')

       def image_callback(self, msg):
           """Process image with AI detection"""
           try:
               # Convert ROS image to OpenCV
               cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

               # Perform AI-based object detection (simulated)
               detections = self.perform_ai_detection(cv_image)

               # Publish 2D detections
               self.publish_2d_detections(detections, msg.header)

               self.get_logger().info('AI detection completed: {} objects detected'.format(len(detections)))

           except Exception as e:
               self.get_logger().error('Error in AI detection: {}'.format(e))

       def pointcloud_callback(self, msg):
           """Process point cloud with AI analysis"""
           self.get_logger().debug('Point cloud received for AI processing')

       def perform_ai_detection(self, image):
           """Perform AI-based object detection (simulated)"""
           # In actual Isaac AI, this would use TensorRT-optimized models
           # For this example, we'll simulate detection results

           detections = []

           # Simulate detection of common objects
           height, width = image.shape[:2]

           # Add some simulated detections
           for i in range(3):
               # Random bounding box
               x = np.random.randint(0, width // 2)
               y = np.random.randint(0, height // 2)
               w = np.random.randint(width // 4, width // 2)
               h = np.random.randint(height // 4, height // 2)

               detection_info = {
                   'bbox': [x, y, w, h],
                   'confidence': np.random.uniform(0.6, 0.95),
                   'class_id': np.random.randint(0, 5),  # 5 different classes
                   'class_name': ['person', 'bottle', 'cup', 'chair', 'laptop'][np.random.randint(0, 5)]
               }
               detections.append(detection_info)

           return detections

       def publish_2d_detections(self, detections, header):
           """Publish 2D detections in ROS format"""
           from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

           detection_array = Detection2DArray()
           detection_array.header = header

           for detection in detections:
               detection_2d = Detection2D()
               detection_2d.header = header

               # Set bounding box
               bbox = detection['bbox']
               detection_2d.bbox.size_x = float(bbox[2])
               detection_2d.bbox.size_y = float(bbox[3])
               detection_2d.bbox.center.x = float(bbox[0] + bbox[2] / 2)
               detection_2d.bbox.center.y = float(bbox[1] + bbox[3] / 2)

               # Set detection result
               hypothesis = ObjectHypothesisWithPose()
               hypothesis.id = str(detection['class_id'])
               hypothesis.score = detection['confidence']

               detection_2d.results.append(hypothesis)
               detection_array.detections.append(detection_2d)

           self.detection_2d_pub.publish(detection_array)

       def publish_3d_detections(self, detections_3d, header):
           """Publish 3D detections in ROS format"""
           detection_array = Detection3DArray()
           detection_array.header = header
           detection_array.detections = detections_3d

           self.detection_3d_pub.publish(detection_array)


   def main(args=None):
       """Main function to initialize and run the AI integrator"""
       rclpy.init(args=args)
       ai_integrator = IsaacAIIntegrator()

       try:
           rclpy.spin(ai_integrator)
       except KeyboardInterrupt:
           pass
       finally:
           ai_integrator.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Make the script executable:
   ```bash
   chmod +x isaac_manipulation_examples/isaac_manipulation_examples/ai_integrator.py
   ```

#### Step 5: Update Setup Files and Build
1. Update `isaac_manipulation_examples/setup.py`:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   package_name = 'isaac_manipulation_examples'

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
       description='Isaac Manipulation examples',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'grasp_planner = isaac_manipulation_examples.grasp_planner:main',
               'ai_integrator = isaac_manipulation_examples.ai_integrator:main',
           ],
       },
   )
   ```

#### Step 6: Build and Test Manipulation System
1. Build the manipulation packages:
   ```bash
   cd ~/isaac_ws
   colcon build --packages-select isaac_manipulation_examples
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Run the grasp planner (in one terminal):
   ```bash
   ros2 run isaac_manipulation_examples grasp_planner
   ```

4. Run the AI integrator (in another terminal):
   ```bash
   ros2 run isaac_manipulation_examples ai_integrator
   ```

### Expected Results
- AI-powered grasp planning generates feasible grasp poses
- Object detection identifies objects in the scene
- Candidate grasps are published and can be visualized
- System demonstrates integration between perception and manipulation

### Analysis Questions
1. How does AI improve grasp planning compared to geometric approaches?
2. What are the challenges in 3D object detection for manipulation?
3. How would you integrate this with a real manipulator?

### Safety Considerations
- Always test manipulation systems in simulation first
- Implement proper safety limits and emergency stops
- Validate grasp planning results before execution
- Monitor system behavior during operation