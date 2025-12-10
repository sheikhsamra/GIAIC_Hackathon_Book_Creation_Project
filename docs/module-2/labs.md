# Lab Exercises: ROS 2 Fundamentals for Physical AI

## Lab 2.1: Creating Your First ROS 2 Package and Nodes

### Objective
Create a ROS 2 package with publisher and subscriber nodes to understand the basic ROS 2 communication model.

### Prerequisites
- Completed Module 1
- Working ROS 2 Humble Hawksbill installation
- Basic Python programming knowledge

### Estimated Time
1.5 hours

### Steps

#### Step 1: Create a New ROS 2 Package
1. Navigate to your workspace source directory:
   ```bash
   cd ~/physical_ai_ws/src
   ```

2. Create a new ROS 2 package:
   ```bash
   ros2 pkg create --build-type ament_python ros2_fundamentals_examples --dependencies rclpy std_msgs geometry_msgs sensor_msgs
   ```

3. Navigate to the package directory:
   ```bash
   cd ros2_fundamentals_examples
   ```

#### Step 2: Create Publisher Node
1. Create a publisher script in `ros2_fundamentals_examples/ros2_fundamentals_examples/status_publisher.py`:
   ```python
   #!/usr/bin/env python3
   """
   ROS 2 publisher node for Physical AI status messages
   """
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   import time


   class StatusPublisher(Node):
       def __init__(self):
           super().__init__('status_publisher')

           # Create publisher
           self.publisher_ = self.create_publisher(String, 'physical_ai_status', 10)

           # Create timer for periodic publishing
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = f'Physical AI System Status: Operational - Cycle {self.i}'
           self.publisher_.publish(msg)
           self.get_logger().info(f'Publishing: "{msg.data}"')
           self.i += 1


   def main(args=None):
       rclpy.init(args=args)
       status_publisher = StatusPublisher()

       try:
           rclpy.spin(status_publisher)
       except KeyboardInterrupt:
           pass
       finally:
           status_publisher.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Make the script executable:
   ```bash
   chmod +x ros2_fundamentals_examples/status_publisher.py
   ```

#### Step 3: Create Subscriber Node
1. Create a subscriber script in `ros2_fundamentals_examples/ros2_fundamentals_examples/status_subscriber.py`:
   ```python
   #!/usr/bin/env python3
   """
   ROS 2 subscriber node for Physical AI status messages
   """
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class StatusSubscriber(Node):
       def __init__(self):
           super().__init__('status_subscriber')

           # Create subscription
           self.subscription = self.create_subscription(
               String,
               'physical_ai_status',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, msg):
           self.get_logger().info(f'Received status: {msg.data}')


   def main(args=None):
       rclpy.init(args=args)
       status_subscriber = StatusSubscriber()

       try:
           rclpy.spin(status_subscriber)
       except KeyboardInterrupt:
           pass
       finally:
           status_subscriber.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Make the script executable:
   ```bash
   chmod +x ros2_fundamentals_examples/status_subscriber.py
   ```

#### Step 4: Update Setup Files
1. Update `setup.py` to include your executables:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   from ament_index_python.packages import get_package_share_directory

   package_name = 'ros2_fundamentals_examples'

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
       description='ROS 2 fundamentals examples for Physical AI',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'status_publisher = ros2_fundamentals_examples.status_publisher:main',
               'status_subscriber = ros2_fundamentals_examples.status_subscriber:main',
           ],
       },
   )
   ```

#### Step 5: Build and Test
1. Build the package:
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select ros2_fundamentals_examples
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Run the publisher in one terminal:
   ```bash
   ros2 run ros2_fundamentals_examples status_publisher
   ```

4. In another terminal, run the subscriber:
   ```bash
   ros2 run ros2_fundamentals_examples status_subscriber
   ```

### Expected Results
- Publisher node publishes status messages every 0.5 seconds
- Subscriber node receives and logs the status messages
- Communication occurs over the `physical_ai_status` topic
- Both nodes run without errors

### Analysis Questions
1. What happens when you start the subscriber after the publisher has been running for a while?
2. How does the queue size affect message delivery?
3. What would happen if you changed the topic name in one of the nodes?

---

## Lab 2.2: Implementing Services for Physical AI Operations

### Objective
Create and use ROS 2 services to implement request-response communication patterns for Physical AI operations.

### Prerequisites
- Completed Lab 2.1
- Understanding of ROS 2 nodes and topics

### Estimated Time
1.5 hours

### Steps

#### Step 1: Create Service Definition
1. Create a services directory in your package:
   ```bash
   mkdir -p ros2_fundamentals_examples/srv
   ```

2. Create a service definition file `ros2_fundamentals_examples/srv/SafetyCheck.srv`:
   ```
   # Request: No input parameters needed for basic safety check
   ---
   # Response: Success flag and message
   bool success
   string message
   ```

#### Step 2: Create Service Server
1. Create a service server script in `ros2_fundamentals_examples/ros2_fundamentals_examples/safety_service.py`:
   ```python
   #!/usr/bin/env python3
   """
   ROS 2 service server for Physical AI safety checks
   """
   import rclpy
   from rclpy.node import Node
   from ros2_fundamentals_examples.srv import SafetyCheck  # Custom service
   import random


   class SafetyService(Node):
       def __init__(self):
           super().__init__('safety_service')

           # Create service server
           self.srv = self.create_service(
               SafetyCheck,
               'safety_check',
               self.safety_check_callback
           )

           self.get_logger().info('Safety service server created')

       def safety_check_callback(self, request, response):
           # Simulate safety check logic
           # In a real system, this would check sensor data, environment, etc.

           # For this example, we'll simulate with 90% success rate
           is_safe = random.random() > 0.1

           if is_safe:
               response.success = True
               response.message = 'System is safe to operate'
               self.get_logger().info('Safety check passed')
           else:
               response.success = False
               response.message = 'Safety check failed - system unsafe'
               self.get_logger().warn('Safety check failed')

           return response


   def main(args=None):
       rclpy.init(args=args)
       safety_service = SafetyService()

       try:
           rclpy.spin(safety_service)
       except KeyboardInterrupt:
           pass
       finally:
           safety_service.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Make the script executable:
   ```bash
   chmod +x ros2_fundamentals_examples/safety_service.py
   ```

#### Step 3: Create Service Client
1. Create a service client script in `ros2_fundamentals_examples/ros2_fundamentals_examples/safety_client.py`:
   ```python
   #!/usr/bin/env python3
   """
   ROS 2 service client for Physical AI safety checks
   """
   import sys
   import rclpy
   from rclpy.node import Node
   from ros2_fundamentals_examples.srv import SafetyCheck  # Custom service


   class SafetyClient(Node):
       def __init__(self):
           super().__init__('safety_client')

           # Create client
           self.cli = self.create_client(SafetyCheck, 'safety_check')

           # Wait for service to be available
           while not self.cli.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('Waiting for safety_check service...')

           self.get_logger().info('Safety client created')

       def send_request(self):
           # Create request (empty for this service)
           request = SafetyCheck.Request()

           # Call service asynchronously
           self.future = self.cli.call_async(request)
           return self.future


   def main(args=None):
       rclpy.init(args=args)
       safety_client = SafetyClient()

       # Send request and wait for response
       future = safety_client.send_request()

       try:
           # Wait for response
           rclpy.spin_until_future_complete(safety_client, future)

           # Process response
           if future.result() is not None:
               response = future.result()
               if response.success:
                   safety_client.get_logger().info(f'Safety Check: {response.message}')
               else:
                   safety_client.get_logger().error(f'Safety Check Failed: {response.message}')
           else:
               safety_client.get_logger().error('Exception occurred while calling service')
       except KeyboardInterrupt:
           pass
       finally:
           safety_client.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Make the script executable:
   ```bash
   chmod +x ros2_fundamentals_examples/safety_client.py
   ```

#### Step 4: Update Package Configuration
1. Update `package.xml` to include the service definition:
   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>ros2_fundamentals_examples</name>
     <version>0.0.0</version>
     <description>ROS 2 fundamentals examples for Physical AI</description>
     <maintainer email="your_email@example.com">your_name</maintainer>
     <license>TODO: License declaration</license>

     <depend>rclpy</depend>
     <depend>std_msgs</depend>
     <depend>geometry_msgs</depend>
     <depend>sensor_msgs</depend>

     <exec_depend>rosidl_default_runtime</exec_depend>
     <member_of_group>rosidl_interface_packages</member_of_group>

     <test_depend>ament_copyright</test_depend>
     <test_depend>ament_flake8</test_depend>
     <test_depend>ament_pep257</test_depend>
     <test_depend>python3-pytest</test_depend>

     <export>
       <build_type>ament_python</build_type>
     </export>
   </package>
   ```

2. Update `setup.py` to include the service definition:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   from ament_index_python.packages import get_package_share_directory
   from setuptools import setup
   from setuptools import find_packages
   from setuptools import setup_from_arguments

   package_name = 'ros2_fundamentals_examples'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           ('share/' + package_name + '/srv', ['srv/SafetyCheck.srv']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='ROS 2 fundamentals examples for Physical AI',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'status_publisher = ros2_fundamentals_examples.status_publisher:main',
               'status_subscriber = ros2_fundamentals_examples.status_subscriber:main',
               'safety_service = ros2_fundamentals_examples.safety_service:main',
               'safety_client = ros2_fundamentals_examples.safety_client:main',
           ],
       },
   )
   ```

#### Step 5: Build and Test Service
1. Build the package with the service definition:
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select ros2_fundamentals_examples --symlink-install
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Run the service server in one terminal:
   ```bash
   ros2 run ros2_fundamentals_examples safety_service
   ```

4. In another terminal, run the service client:
   ```bash
   ros2 run ros2_fundamentals_examples safety_client
   ```

### Expected Results
- Service server waits for safety check requests
- Service client sends a request and receives a response
- Safety status is logged in both nodes
- Service communication works without errors

### Analysis Questions
1. How is service communication different from topic communication?
2. When would you use a service instead of a topic?
3. What happens if the service server is not running when the client makes a request?

---

## Lab 2.3: Parameter Management and Launch Files

### Objective
Implement parameter management in ROS 2 nodes and create launch files to coordinate complex system startup.

### Prerequisites
- Completed Labs 2.1 and 2.2
- Understanding of ROS 2 nodes and services

### Estimated Time
2 hours

### Steps

#### Step 1: Create Parameter-Enabled Node
1. Create a parameter-enabled node script in `ros2_fundamentals_examples/ros2_fundamentals_examples/parameter_node.py`:
   ```python
   #!/usr/bin/env python3
   """
   ROS 2 node demonstrating parameter management
   """
   import rclpy
   from rclpy.node import Node
   from rclpy.parameter import Parameter
   from rcl_interfaces.msg import SetParametersResult


   class ParameterNode(Node):
       def __init__(self):
           super().__init__('parameter_node')

           # Declare parameters with default values
           self.declare_parameter('max_velocity', 1.0)
           self.declare_parameter('safety_distance', 0.5)
           self.declare_parameter('control_frequency', 50)
           self.declare_parameter('robot_name', 'physical_ai_robot')
           self.declare_parameter('operation_mode', 'autonomous')

           # Get parameter values
           self.max_velocity = self.get_parameter('max_velocity').value
           self.safety_distance = self.get_parameter('safety_distance').value
           self.control_frequency = self.get_parameter('control_frequency').value
           self.robot_name = self.get_parameter('robot_name').value
           self.operation_mode = self.get_parameter('operation_mode').value

           self.get_logger().info(f'Robot: {self.robot_name}')
           self.get_logger().info(f'Max Velocity: {self.max_velocity} m/s')
           self.get_logger().info(f'Safety Distance: {self.safety_distance} m')
           self.get_logger().info(f'Control Frequency: {self.control_frequency} Hz')
           self.get_logger().info(f'Operation Mode: {self.operation_mode}')

           # Set up parameter change callback
           self.add_on_set_parameters_callback(self.parameter_callback)

           # Timer to periodically check parameters
           self.timer = self.create_timer(5.0, self.timer_callback)

       def parameter_callback(self, params):
           """Callback for parameter changes"""
           for param in params:
               if param.name == 'max_velocity':
                   if param.value > 5.0:
                       self.get_logger().warn(f'High velocity requested: {param.value}')
                   self.max_velocity = param.value
               elif param.name == 'safety_distance':
                   if param.value < 0.1:
                       self.get_logger().error(f'Dangerous safety distance: {param.value}')
                   self.safety_distance = param.value
               elif param.name == 'control_frequency':
                   self.control_frequency = param.value
               elif param.name == 'robot_name':
                   self.robot_name = param.value
               elif param.name == 'operation_mode':
                   self.operation_mode = param.value

           return SetParametersResult(successful=True)

       def timer_callback(self):
           """Periodically log current parameter values"""
           self.get_logger().info(
               f'Current params - Vel: {self.max_velocity}, '
               f'Safe dist: {self.safety_distance}, '
               f'Freq: {self.control_frequency}, '
               f'Mode: {self.operation_mode}'
           )


   def main(args=None):
       rclpy.init(args=args)
       param_node = ParameterNode()

       try:
           rclpy.spin(param_node)
       except KeyboardInterrupt:
           pass
       finally:
           param_node.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Make the script executable:
   ```bash
   chmod +x ros2_fundamentals_examples/parameter_node.py
   ```

#### Step 2: Create Launch Files
1. Create a launch directory:
   ```bash
   mkdir -p ros2_fundamentals_examples/launch
   ```

2. Create a launch file `ros2_fundamentals_examples/launch/physical_ai_system.launch.py`:
   ```python
   #!/usr/bin/env python3
   """
   Launch file for Physical AI system with multiple nodes
   """
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node


   def generate_launch_description():
       # Declare launch arguments
       namespace_arg = DeclareLaunchArgument(
           'namespace',
           default_value='physical_ai',
           description='Namespace for the nodes'
       )

       use_sim_time_arg = DeclareLaunchArgument(
           'use_sim_time',
           default_value='false',
           description='Use simulation time'
       )

       # Get launch configurations
       namespace = LaunchConfiguration('namespace')
       use_sim_time = LaunchConfiguration('use_sim_time')

       # Create parameter node
       parameter_node = Node(
           package='ros2_fundamentals_examples',
           executable='parameter_node',
           name='parameter_node',
           namespace=namespace,
           parameters=[
               {'max_velocity': 1.5},
               {'safety_distance': 0.8},
               {'control_frequency': 100},
               {'robot_name': 'ros2_physical_ai_robot'},
               {'operation_mode': 'autonomous'},
               {'use_sim_time': use_sim_time}
           ],
           output='screen'
       )

       # Create status publisher node
       status_publisher = Node(
           package='ros2_fundamentals_examples',
           executable='status_publisher',
           name='status_publisher',
           namespace=namespace,
           parameters=[{'use_sim_time': use_sim_time}],
           output='screen'
       )

       # Create safety service node
       safety_service = Node(
           package='ros2_fundamentals_examples',
           executable='safety_service',
           name='safety_service',
           namespace=namespace,
           parameters=[{'use_sim_time': use_sim_time}],
           output='screen'
       )

       # Return the complete launch description
       return LaunchDescription([
           namespace_arg,
           use_sim_time_arg,
           parameter_node,
           status_publisher,
           safety_service
       ])
   ```

3. Make the launch file executable:
   ```bash
   chmod +x ros2_fundamentals_examples/launch/physical_ai_system.launch.py
   ```

#### Step 3: Update Setup Files
1. Update `setup.py` to include launch files:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   package_name = 'ros2_fundamentals_examples'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           ('share/' + package_name + '/srv', glob('srv/*.srv')),
           ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='ROS 2 fundamentals examples for Physical AI',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'status_publisher = ros2_fundamentals_examples.status_publisher:main',
               'status_subscriber = ros2_fundamentals_examples.status_subscriber:main',
               'safety_service = ros2_fundamentals_examples.safety_service:main',
               'safety_client = ros2_fundamentals_examples.safety_client:main',
               'parameter_node = ros2_fundamentals_examples.parameter_node:main',
           ],
       },
   )
   ```

#### Step 4: Build and Test Launch System
1. Build the package:
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select ros2_fundamentals_examples --symlink-install
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Launch the complete system:
   ```bash
   ros2 launch ros2_fundamentals_examples physical_ai_system.launch.py
   ```

4. In another terminal, check the running nodes:
   ```bash
   ros2 node list
   ```

5. Check the parameters of the parameter node:
   ```bash
   ros2 param list
   ros2 param get /physical_ai/parameter_node robot_name
   ```

### Expected Results
- Launch file starts multiple nodes with specified parameters
- Parameter node initializes with configured values
- All nodes run within the specified namespace
- Parameters can be queried and modified at runtime

### Analysis Questions
1. How do launch files simplify system startup?
2. What are the benefits of parameter management in ROS 2?
3. How does namespacing help organize complex systems?

---

## Lab 2.4: Safety Monitor with Real-time Monitoring

### Objective
Create a comprehensive safety monitoring system that integrates multiple sensors and implements fail-safe mechanisms.

### Prerequisites
- Completed previous labs
- Understanding of ROS 2 topics, services, and parameters

### Estimated Time
2.5 hours

### Steps

#### Step 1: Create Safety Monitor Node
1. Create a safety monitor script in `ros2_fundamentals_examples/ros2_fundamentals_examples/safety_monitor.py`:
   ```python
   #!/usr/bin/env python3
   """
   Comprehensive safety monitor for Physical AI systems
   """
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Bool, String, Float32
   from geometry_msgs.msg import Twist
   from sensor_msgs.msg import LaserScan
   from builtin_interfaces.msg import Time
   import threading
   import time
   from collections import deque


   class SafetyMonitor(Node):
       def __init__(self):
           super().__init__('safety_monitor')

           # Safety state variables
           self.system_safe = True
           self.emergency_stop_active = False
           self.last_velocity_cmd = Twist()
           self.scan_buffer = deque(maxlen=5)

           # Safety thresholds
           self.max_linear_velocity = 1.0  # m/s
           self.max_angular_velocity = 1.0  # rad/s
           self.min_obstacle_distance = 0.5  # m
           self.max_operation_time = 3600.0  # seconds

           # Publishers
           self.safety_status_pub = self.create_publisher(Bool, 'safety_status', 10)
           self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
           self.safety_violation_pub = self.create_publisher(String, 'safety_violation', 10)
           self.cmd_vel_filtered_pub = self.create_publisher(Twist, 'cmd_vel_filtered', 10)

           # Subscriptions
           self.cmd_vel_sub = self.create_subscription(
               Twist,
               'cmd_vel',
               self.cmd_vel_callback,
               10
           )

           self.scan_sub = self.create_subscription(
               LaserScan,
               'scan',
               self.scan_callback,
               10
           )

           # Timer for periodic safety checks
           self.safety_timer = self.create_timer(0.1, self.periodic_safety_check)

           self.get_logger().info('Safety Monitor initialized')

       def cmd_vel_callback(self, msg):
           """Monitor velocity commands for safety violations"""
           # Check linear velocity limits
           linear_speed = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
           if linear_speed > self.max_linear_velocity:
               self.log_safety_violation(f'VELOCITY VIOLATION: {linear_speed:.2f} m/s > {self.max_linear_velocity} m/s')
               self.system_safe = False
               # Publish filtered command to limit velocity
               filtered_cmd = self.limit_velocity(msg, self.max_linear_velocity)
               self.cmd_vel_filtered_pub.publish(filtered_cmd)
               return

           # Check angular velocity limits
           angular_speed = (msg.angular.x**2 + msg.angular.y**2 + msg.angular.z**2)**0.5
           if angular_speed > self.max_angular_velocity:
               self.log_safety_violation(f'ANGULAR VELOCITY VIOLATION: {angular_speed:.2f} rad/s > {self.max_angular_velocity} rad/s')
               self.system_safe = False
               # Publish filtered command to limit angular velocity
               filtered_cmd = self.limit_angular_velocity(msg, self.max_angular_velocity)
               self.cmd_vel_filtered_pub.publish(filtered_cmd)
               return

           # If command is safe, publish it
           self.cmd_vel_filtered_pub.publish(msg)
           self.last_velocity_cmd = msg

       def scan_callback(self, msg):
           """Monitor laser scan for obstacle detection"""
           if len(msg.ranges) > 0:
               # Find minimum distance in front of robot
               valid_ranges = [r for r in msg.ranges if 0.1 < r < 10.0]
               if valid_ranges:
                   min_distance = min(valid_ranges)
                   self.scan_buffer.append(min_distance)

                   if min_distance < self.min_obstacle_distance:
                       self.log_safety_violation(f'OBSTACLE TOO CLOSE: {min_distance:.2f} m < {self.min_obstacle_distance} m')
                       self.system_safe = False

       def periodic_safety_check(self):
           """Perform periodic safety checks"""
           # Publish safety status
           safety_msg = Bool()
           safety_msg.data = self.system_safe
           self.safety_status_pub.publish(safety_msg)

           # Publish emergency stop if needed
           emergency_msg = Bool()
           emergency_msg.data = not self.system_safe
           self.emergency_stop_pub.publish(emergency_msg)

           # Log status
           if not self.system_safe:
               self.get_logger().warn('SAFETY SYSTEM ACTIVE - SYSTEM NOT SAFE')
           else:
               self.get_logger().info('Safety check passed')

       def log_safety_violation(self, violation_msg):
           """Log safety violation"""
           violation = String()
           violation.data = violation_msg
           self.safety_violation_pub.publish(violation)
           self.get_logger().error(violation_msg)

       def limit_velocity(self, cmd, max_vel):
           """Limit linear velocity in command"""
           current_speed = (cmd.linear.x**2 + cmd.linear.y**2 + cmd.linear.z**2)**0.5
           if current_speed > max_vel:
               scale = max_vel / current_speed
               cmd.linear.x *= scale
               cmd.linear.y *= scale
               cmd.linear.z *= scale
           return cmd

       def limit_angular_velocity(self, cmd, max_angular_vel):
           """Limit angular velocity in command"""
           current_angular_speed = (cmd.angular.x**2 + cmd.angular.y**2 + cmd.angular.z**2)**0.5
           if current_angular_speed > max_angular_vel:
               scale = max_angular_vel / current_angular_speed
               cmd.angular.x *= scale
               cmd.angular.y *= scale
               cmd.angular.z *= scale
           return cmd

       def reset_safety_system(self):
           """Reset the safety system (would require manual intervention in real systems)"""
           self.system_safe = True
           self.get_logger().info('Safety system reset')


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
   chmod +x ros2_fundamentals_examples/safety_monitor.py
   ```

#### Step 2: Update Package Configuration
1. Update `setup.py` to include the new node:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   package_name = 'ros2_fundamentals_examples'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           ('share/' + package_name + '/srv', glob('srv/*.srv')),
           ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='ROS 2 fundamentals examples for Physical AI',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'status_publisher = ros2_fundamentals_examples.status_publisher:main',
               'status_subscriber = ros2_fundamentals_examples.status_subscriber:main',
               'safety_service = ros2_fundamentals_examples.safety_service:main',
               'safety_client = ros2_fundamentals_examples.safety_client:main',
               'parameter_node = ros2_fundamentals_examples.parameter_node:main',
               'safety_monitor = ros2_fundamentals_examples.safety_monitor:main',
           ],
       },
   )
   ```

#### Step 3: Create Safety Test Launch File
1. Create a safety test launch file `ros2_fundamentals_examples/launch/safety_test.launch.py`:
   ```python
   #!/usr/bin/env python3
   """
   Launch file for safety monitoring test
   """
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node


   def generate_launch_description():
       # Declare launch arguments
       namespace_arg = DeclareLaunchArgument(
           'namespace',
           default_value='physical_ai',
           description='Namespace for the nodes'
       )

       use_sim_time_arg = DeclareLaunchArgument(
           'use_sim_time',
           default_value='false',
           description='Use simulation time'
       )

       # Get launch configurations
       namespace = LaunchConfiguration('namespace')
       use_sim_time = LaunchConfiguration('use_sim_time')

       # Create safety monitor node
       safety_monitor = Node(
           package='ros2_fundamentals_examples',
           executable='safety_monitor',
           name='safety_monitor',
           namespace=namespace,
           parameters=[{'use_sim_time': use_sim_time}],
           output='screen'
       )

       # Create status publisher for testing
       status_publisher = Node(
           package='ros2_fundamentals_examples',
           executable='status_publisher',
           name='status_publisher',
           namespace=namespace,
           parameters=[{'use_sim_time': use_sim_time}],
           output='screen'
       )

       # Return the complete launch description
       return LaunchDescription([
           namespace_arg,
           use_sim_time_arg,
           safety_monitor,
           status_publisher
       ])
   ```

2. Make the launch file executable:
   ```bash
   chmod +x ros2_fundamentals_examples/launch/safety_test.launch.py
   ```

#### Step 4: Build and Test Safety System
1. Build the package:
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select ros2_fundamentals_examples --symlink-install
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Launch the safety test system:
   ```bash
   ros2 launch ros2_fundamentals_examples safety_test.launch.py
   ```

4. Test the safety system by publishing unsafe commands:
   ```bash
   ros2 topic pub /physical_ai/cmd_vel geometry_msgs/Twist '{linear: {x: 3.0}, angular: {z: 0.0}}'
   ```

5. Monitor safety status:
   ```bash
   ros2 topic echo /physical_ai/safety_status
   ```

### Expected Results
- Safety monitor detects unsafe commands and obstacles
- Emergency stop is triggered when safety is violated
- Filtered commands are published with velocity limits
- Safety violations are logged and published

### Analysis Questions
1. How does the safety monitor provide defense in depth?
2. What additional safety checks could be implemented?
3. How would you integrate this with hardware safety systems?

### Safety Considerations
- Always test safety systems thoroughly in simulation before real-world deployment
- Implement multiple layers of safety protection
- Ensure fail-safe behavior in all scenarios
- Regularly verify safety system functionality