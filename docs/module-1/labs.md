# Lab Exercises: Introduction to Physical AI

## Lab 1.1: Setting Up the Physical AI Development Environment

### Objective
Set up a complete Physical AI development environment with ROS 2, Gazebo, and basic tools.

### Prerequisites
- Computer with Ubuntu 22.04 LTS (or Windows with WSL2)
- At least 8GB RAM (16GB recommended)
- Compatible GPU for NVIDIA Isaac Sim (optional for this lab)

### Estimated Time
1-2 hours

### Steps

#### Step 1: System Preparation
1. Update your system packages:
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

2. Install basic development tools:
   ```bash
   sudo apt install python3-pip python3-colcon-common-extensions build-essential
   ```

#### Step 2: Install ROS 2 Humble Hawksbill
1. Add ROS 2 repository:
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

2. Install ROS 2 packages:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop ros-humble-ros-base
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

3. Initialize rosdep:
   ```bash
   sudo rosdep init
   rosdep update
   ```

4. Source ROS 2 in your shell:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

#### Step 3: Install Gazebo Garden
1. Add Gazebo repository:
   ```bash
   sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo.list'
   wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   sudo apt update
   ```

2. Install Gazebo:
   ```bash
   sudo apt install gazebo
   ```

#### Step 4: Create a ROS 2 Workspace
1. Create workspace directory:
   ```bash
   mkdir -p ~/physical_ai_ws/src
   cd ~/physical_ai_ws
   ```

2. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```

3. Source the workspace:
   ```bash
   echo "source ~/physical_ai_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

#### Step 5: Verify Installation
1. Test ROS 2 installation:
   ```bash
   ros2 topic list
   ```

2. Test Gazebo installation:
   ```bash
   gazebo --version
   ```

### Expected Results
- ROS 2 Humble Hawksbill is installed and working
- Gazebo Garden is installed and working
- ROS 2 workspace is created and sourced
- Basic ROS 2 commands work correctly

### Troubleshooting
- If `rosdep init` fails, try running it with `sudo` and ensure your internet connection is working
- If Gazebo fails to start, check if you have a compatible graphics driver installed
- If workspace build fails, ensure all dependencies are installed correctly

### Safety Considerations
- Always work in a safe environment when dealing with physical systems
- Ensure proper ventilation when running intensive simulations
- Monitor system resources to prevent overheating

---

## Lab 1.2: Basic ROS 2 Communication with Physical AI Concepts

### Objective
Create and run basic ROS 2 publisher and subscriber nodes to understand communication in Physical AI systems.

### Prerequisites
- Completed Lab 1.1
- Basic Python programming knowledge

### Estimated Time
1 hour

### Steps

#### Step 1: Create a New Package
1. Navigate to your workspace:
   ```bash
   cd ~/physical_ai_ws/src
   ```

2. Create a new ROS 2 package:
   ```bash
   ros2 pkg create --build-type ament_python physical_ai_examples
   ```

#### Step 2: Create Publisher Node
1. Navigate to the package directory:
   ```bash
   cd physical_ai_examples
   ```

2. Create the publisher script in `physical_ai_examples/physical_ai_examples/status_publisher.py`:
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   import time


   class PhysicalAIStatusPublisher(Node):
       def __init__(self):
           super().__init__('physical_ai_status_publisher')
           self.publisher_ = self.create_publisher(String, 'ai_status', 10)
           timer_period = 1  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.status_counter = 0

       def timer_callback(self):
           msg = String()
           msg.data = f'Physical AI System Status: Operational - Cycle {self.status_counter}'
           self.publisher_.publish(msg)
           self.get_logger().info(f'Published: {msg.data}')
           self.status_counter += 1


   def main(args=None):
       rclpy.init(args=args)
       publisher = PhysicalAIStatusPublisher()

       try:
           rclpy.spin(publisher)
       except KeyboardInterrupt:
           pass
       finally:
           publisher.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

3. Make the script executable:
   ```bash
   chmod +x physical_ai_examples/status_publisher.py
   ```

#### Step 3: Create Subscriber Node
1. Create the subscriber script in `physical_ai_examples/physical_ai_examples/status_subscriber.py`:
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class PhysicalAIStatusSubscriber(Node):
       def __init__(self):
           super().__init__('physical_ai_status_subscriber')
           self.subscription = self.create_subscription(
               String,
               'ai_status',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, msg):
           self.get_logger().info(f'Subscribed to AI Status: {msg.data}')


   def main(args=None):
       rclpy.init(args=args)
       subscriber = PhysicalAIStatusSubscriber()

       try:
           rclpy.spin(subscriber)
       except KeyboardInterrupt:
           pass
       finally:
           subscriber.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Make the script executable:
   ```bash
   chmod +x physical_ai_examples/status_subscriber.py
   ```

#### Step 4: Update Setup Files
1. Update `setup.py` to include your scripts:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   package_name = 'physical_ai_examples'

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
       description='Examples for Physical AI concepts',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'status_publisher = physical_ai_examples.status_publisher:main',
               'status_subscriber = physical_ai_examples.status_subscriber:main',
           ],
       },
   )
   ```

#### Step 5: Build and Run
1. Build the package:
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select physical_ai_examples
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Run the publisher in one terminal:
   ```bash
   ros2 run physical_ai_examples status_publisher
   ```

4. In another terminal, run the subscriber:
   ```bash
   ros2 run physical_ai_examples status_subscriber
   ```

### Expected Results
- Publisher node publishes status messages every second
- Subscriber node receives and logs the status messages
- Communication occurs over the `ai_status` topic
- Both nodes run without errors

### Analysis Questions
1. How does this simple publisher-subscriber pattern relate to Physical AI systems?
2. What safety considerations should be taken into account when designing communication in Physical AI systems?
3. How could you extend this pattern to handle sensor data from a physical robot?

---

## Lab 1.3: Environment Perception Simulation

### Objective
Create a simple simulation environment and implement basic perception capabilities using Gazebo.

### Prerequisites
- Completed Labs 1.1 and 1.2
- Understanding of ROS 2 concepts

### Estimated Time
2 hours

### Steps

#### Step 1: Create a Simulation Package
1. Navigate to your workspace:
   ```bash
   cd ~/physical_ai_ws/src
   ```

2. Create a new package for simulation:
   ```bash
   ros2 pkg create --build-type ament_python physical_ai_simulation
   ```

#### Step 2: Create a Simple World File
1. Create a worlds directory:
   ```bash
   mkdir -p physical_ai_simulation/worlds
   ```

2. Create a simple world file `physical_ai_simulation/worlds/simple_world.sdf`:
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="simple_world">
       <!-- Ground plane -->
       <include>
         <uri>model://ground_plane</uri>
       </include>

       <!-- Sun light -->
       <include>
         <uri>model://sun</uri>
       </include>

       <!-- Simple box obstacle -->
       <model name="box">
         <pose>2 2 0.5 0 0 0</pose>
         <link name="link">
           <visual name="visual">
             <geometry>
               <box>
                 <size>1 1 1</size>
               </box>
             </geometry>
             <material>
               <ambient>0.5 0.5 0.5 1</ambient>
               <diffuse>0.8 0.3 0.1 1</diffuse>
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

       <!-- Simple robot model -->
       <model name="simple_robot">
         <pose>0 0 0.2 0 0 0</pose>
         <link name="chassis">
           <visual name="chassis_visual">
             <geometry>
               <cylinder>
                 <radius>0.3</radius>
                 <length>0.2</length>
               </cylinder>
             </geometry>
             <material>
               <ambient>0.1 0.1 0.8 1</ambient>
               <diffuse>0.2 0.2 1.0 1</diffuse>
             </material>
           </visual>
           <collision name="chassis_collision">
             <geometry>
               <cylinder>
                 <radius>0.3</radius>
                 <length>0.2</length>
               </cylinder>
             </geometry>
           </collision>
           <inertial>
             <mass>5.0</mass>
             <inertia>
               <ixx>0.125</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.125</iyy>
               <iyz>0</iyz>
               <izz>0.225</izz>
             </inertia>
           </inertial>
         </link>
       </model>
     </world>
   </sdf>
   ```

#### Step 3: Launch Gazebo with Your World
1. Create a launch directory:
   ```bash
   mkdir -p physical_ai_simulation/launch
   ```

2. Create a launch file `physical_ai_simulation/launch/simple_world.launch.py`:
   ```python
   import os
   from launch import LaunchDescription
   from launch.actions import ExecuteProcess
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory


   def generate_launch_description():
       package_dir = get_package_share_directory('physical_ai_simulation')
       world_file = os.path.join(package_dir, 'worlds', 'simple_world.sdf')

       return LaunchDescription([
           # Launch Gazebo with the custom world
           ExecuteProcess(
               cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
               output='screen'
           )
       ])
   ```

#### Step 4: Build and Run Simulation
1. Build the package:
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select physical_ai_simulation
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Launch the simulation:
   ```bash
   ros2 launch physical_ai_simulation simple_world.launch.py
   ```

### Expected Results
- Gazebo launches with your custom world
- A simple robot model appears in the simulation
- An obstacle box is present in the environment
- Simulation runs without errors

### Extension Activities
1. Add sensors to your robot model (camera, LIDAR, IMU)
2. Create a perception node that processes sensor data
3. Implement basic obstacle detection
4. Add safety checks to prevent the robot from colliding with obstacles

### Safety Considerations
- Always validate simulation results before applying to real robots
- Implement proper safety checks in simulation to mirror real-world requirements
- Consider fail-safe mechanisms in your simulation design

---

## Lab 1.4: Safety Monitoring Implementation

### Objective
Implement a basic safety monitoring system for Physical AI applications.

### Prerequisites
- Completed previous labs
- Understanding of ROS 2 topics and messages

### Estimated Time
1.5 hours

### Steps

#### Step 1: Create Safety Package
1. Create a new package for safety components:
   ```bash
   cd ~/physical_ai_ws/src
   ros2 pkg create --build-type ament_python physical_ai_safety
   ```

#### Step 2: Create Safety Monitor Node
1. Create the safety monitor script in `physical_ai_safety/physical_ai_safety/safety_monitor.py`:
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Bool, Float32
   from geometry_msgs.msg import Twist
   import threading
   import time


   class SafetyMonitor(Node):
       def __init__(self):
           super().__init__('safety_monitor')

           # Subscriptions
           self.safety_status_sub = self.create_subscription(
               Bool,
               '/robot/safety_status',
               self.safety_status_callback,
               10)

           self.velocity_sub = self.create_subscription(
               Twist,
               '/cmd_vel',
               self.velocity_callback,
               10)

           # Publishers
           self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
           self.safety_violation_pub = self.create_publisher(Bool, '/safety_violation', 10)

           # Parameters
           self.max_linear_velocity = 1.0  # m/s
           self.max_angular_velocity = 1.0  # rad/s
           self.is_safe = True
           self.safety_timer = self.create_timer(0.1, self.safety_check)

           self.get_logger().info('Safety Monitor initialized')

       def safety_status_callback(self, msg):
           """Handle safety status updates"""
           self.is_safe = msg.data
           if not self.is_safe:
               self.get_logger().warn('SAFETY VIOLATION DETECTED!')

       def velocity_callback(self, msg):
           """Monitor velocity commands for safety violations"""
           linear_violation = abs(msg.linear.x) > self.max_linear_velocity
           angular_violation = abs(msg.angular.z) > self.max_angular_velocity

           if linear_violation or angular_violation:
               self.get_logger().warn(f'SAFETY VIOLATION: Linear: {msg.linear.x}, Angular: {msg.angular.z}')
               self.is_safe = False

       def safety_check(self):
           """Periodic safety checks"""
           safety_msg = Bool()
           safety_msg.data = self.is_safe
           self.safety_violation_pub.publish(safety_msg)

           # Emergency stop if not safe
           if not self.is_safe:
               emergency_msg = Bool()
               emergency_msg.data = True
               self.emergency_stop_pub.publish(emergency_msg)
           else:
               emergency_msg = Bool()
               emergency_msg.data = False
               self.emergency_stop_pub.publish(emergency_msg)


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

2. Make the script executable:
   ```bash
   chmod +x physical_ai_safety/physical_ai_safety/safety_monitor.py
   ```

#### Step 3: Create Safety Test Node
1. Create a test script in `physical_ai_safety/physical_ai_safety/safety_test.py`:
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Bool
   from geometry_msgs.msg import Twist
   import time


   class SafetyTest(Node):
       def __init__(self):
           super().__init__('safety_test')
           self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
           self.safety_status_pub = self.create_publisher(Bool, '/robot/safety_status', 10)

           # Timer to send test commands
           self.test_timer = self.create_timer(2.0, self.send_test_commands)
           self.command_count = 0

       def send_test_commands(self):
           """Send various test commands to trigger safety checks"""
           if self.command_count == 0:
               # Safe command
               cmd = Twist()
               cmd.linear.x = 0.5
               cmd.angular.z = 0.2
               self.cmd_vel_pub.publish(cmd)
               self.get_logger().info('Sent safe command: linear=0.5, angular=0.2')

               # Send safe status
               safe_msg = Bool()
               safe_msg.data = True
               self.safety_status_pub.publish(safe_msg)

           elif self.command_count == 1:
               # Unsafe command (too fast)
               cmd = Twist()
               cmd.linear.x = 2.0  # Exceeds max velocity
               cmd.angular.z = 0.5
               self.cmd_vel_pub.publish(cmd)
               self.get_logger().info('Sent unsafe command: linear=2.0 (too fast)')

           elif self.command_count == 2:
               # Safe command again
               cmd = Twist()
               cmd.linear.x = 0.3
               cmd.angular.z = 0.1
               self.cmd_vel_pub.publish(cmd)
               self.get_logger().info('Sent safe command again: linear=0.3, angular=0.1')

           self.command_count = (self.command_count + 1) % 3


   def main(args=None):
       rclpy.init(args=args)
       safety_test = SafetyTest()

       try:
           rclpy.spin(safety_test)
       except KeyboardInterrupt:
           pass
       finally:
           safety_test.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Make the script executable:
   ```bash
   chmod +x physical_ai_safety/physical_ai_safety/safety_test.py
   ```

#### Step 4: Build and Test Safety System
1. Build the package:
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select physical_ai_safety
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Run the safety monitor in one terminal:
   ```bash
   ros2 run physical_ai_safety safety_monitor
   ```

4. In another terminal, run the safety test:
   ```bash
   ros2 run physical_ai_safety safety_test
   ```

### Expected Results
- Safety monitor detects velocity violations
- Emergency stop is triggered when safety is violated
- Safety status messages are published correctly
- System responds appropriately to different safety conditions

### Discussion Questions
1. How would you extend this basic safety system for a real Physical AI application?
2. What additional safety checks would be necessary for a mobile robot operating in human environments?
3. How could you integrate this safety system with higher-level planning and control?

### Safety Considerations
- Safety systems must be designed with multiple layers of protection
- Fail-safe mechanisms should default to safe states
- Safety systems should be tested extensively in simulation before real-world deployment
- Consider both hardware and software safety measures