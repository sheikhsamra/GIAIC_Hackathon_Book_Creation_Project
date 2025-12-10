# Code Examples: ROS 2 Fundamentals for Physical AI

## 1. Basic Node Structure

### 1.1 Simple Publisher Node (Python)

```python
#!/usr/bin/env python3
"""
Basic ROS 2 publisher node for Physical AI systems
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Header
import time


class PhysicalAIStatusPublisher(Node):
    def __init__(self):
        super().__init__('physical_ai_status_publisher')

        # Create publisher with QoS settings appropriate for status messages
        self.publisher_ = self.create_publisher(
            String,
            'ai_status',
            10  # queue size
        )

        # Timer to publish messages at regular intervals
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.status_counter = 0

        self.get_logger().info('Physical AI Status Publisher initialized')

    def timer_callback(self):
        """Callback function that publishes status messages"""
        msg = String()
        msg.data = f'Physical AI System Operational - Cycle: {self.status_counter}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Published: {msg.data}')

        # Increment counter
        self.status_counter += 1


def main(args=None):
    """Main function to initialize and run the node"""
    rclpy.init(args=args)
    publisher = PhysicalAIStatusPublisher()

    try:
        # Spin the node to keep it alive
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 1.2 Simple Subscriber Node (Python)

```python
#!/usr/bin/env python3
"""
Basic ROS 2 subscriber node for Physical AI systems
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PhysicalAIStatusSubscriber(Node):
    def __init__(self):
        super().__init__('physical_ai_status_subscriber')

        # Create subscription to receive status messages
        self.subscription = self.create_subscription(
            String,
            'ai_status',
            self.listener_callback,
            10  # queue size
        )

        # Prevent unused variable warning
        self.subscription  # type: ignore

        self.get_logger().info('Physical AI Status Subscriber initialized')

    def listener_callback(self, msg):
        """Callback function to process received messages"""
        self.get_logger().info(f'Received AI Status: {msg.data}')


def main(args=None):
    """Main function to initialize and run the node"""
    rclpy.init(args=args)
    subscriber = PhysicalAIStatusSubscriber()

    try:
        # Spin the node to keep it alive
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 2. Service Implementation

### 2.1 Service Server (Python)

```python
#!/usr/bin/env python3
"""
ROS 2 service server for Physical AI safety checks
"""
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger


class SafetyCheckService(Node):
    def __init__(self):
        super().__init__('safety_check_service')

        # Create service server
        self.srv = self.create_service(
            Trigger,
            'safety_check',
            self.safety_check_callback
        )

        self.get_logger().info('Safety Check Service initialized')

    def safety_check_callback(self, request, response):
        """Callback function to handle safety check requests"""
        # Perform safety check logic here
        # This is a simplified example - in real systems, this would check
        # sensor data, system status, environment, etc.

        # For this example, we'll simulate a safety check
        import random
        is_safe = random.random() > 0.1  # 90% chance of being safe

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
    """Main function to initialize and run the service"""
    rclpy.init(args=args)
    service = SafetyCheckService()

    try:
        # Spin the service to keep it alive
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2.2 Service Client (Python)

```python
#!/usr/bin/env python3
"""
ROS 2 service client for Physical AI safety checks
"""
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger


class SafetyCheckClient(Node):
    def __init__(self):
        super().__init__('safety_check_client')

        # Create client for the safety check service
        self.cli = self.create_client(Trigger, 'safety_check')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for safety_check service...')

        self.get_logger().info('Safety Check Client initialized')

    def send_request(self):
        """Send a request to the safety check service"""
        request = Trigger.Request()

        # Call the service asynchronously
        self.future = self.cli.call_async(request)

        return self.future


def main(args=None):
    """Main function to initialize and run the client"""
    rclpy.init(args=args)
    client = SafetyCheckClient()

    # Send a request and wait for response
    future = client.send_request()

    try:
        # Spin until the future is complete
        rclpy.spin_until_future_complete(client, future)

        # Process the response
        if future.result() is not None:
            response = future.result()
            if response.success:
                client.get_logger().info(f'Safety Check: {response.message}')
            else:
                client.get_logger().error(f'Safety Check Failed: {response.message}')
        else:
            client.get_logger().error('Exception occurred while calling service')
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 3. Action Implementation

### 3.1 Action Server (Python)

```python
#!/usr/bin/env python3
"""
ROS 2 action server for Physical AI navigation tasks
"""
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose


class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Navigation Action Server initialized')

    def destroy(self):
        """Clean up the action server"""
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Handle incoming navigation goals"""
        self.get_logger().info('Received navigation goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancellation requests"""
        self.get_logger().info('Received request to cancel navigation goal')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the navigation task"""
        self.get_logger().info('Executing navigation task...')

        # Get target pose from goal
        target_pose = goal_handle.request.pose
        self.get_logger().info(f'Navigating to: ({target_pose.pose.position.x}, {target_pose.pose.position.y})')

        # Simulate navigation progress
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        # Simulate navigation steps
        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Navigation goal canceled')
                result.error_code = -1
                return result

            # Publish feedback
            feedback_msg.current_pose = target_pose  # Simplified feedback
            feedback_msg.distance_remaining = 10.0 - i  # Simplified distance
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Navigation progress: {i+1}/10')

            # Simulate navigation time
            time.sleep(0.5)

        # Complete successfully
        goal_handle.succeed()
        result.result = target_pose
        self.get_logger().info('Navigation task completed successfully')

        return result


def main(args=None):
    """Main function to initialize and run the action server"""
    rclpy.init(args=args)

    action_server = NavigationActionServer()

    try:
        # Use multi-threaded executor to handle multiple goals
        executor = MultiThreadedExecutor()
        rclpy.spin(action_server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        action_server.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 4. Parameter Management

### 4.1 Parameter Server Node (Python)

```python
#!/usr/bin/env python3
"""
ROS 2 node demonstrating parameter management for Physical AI
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class ParameterExampleNode(Node):
    def __init__(self):
        super().__init__('parameter_example_node')

        # Declare parameters with default values
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('control_frequency', 50)
        self.declare_parameter('robot_name', 'physical_ai_robot')

        # Get parameter values
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.robot_name = self.get_parameter('robot_name').value

        self.get_logger().info(f'Robot: {self.robot_name}')
        self.get_logger().info(f'Max Velocity: {self.max_velocity} m/s')
        self.get_logger().info(f'Safety Distance: {self.safety_distance} m')
        self.get_logger().info(f'Control Frequency: {self.control_frequency} Hz')

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
                self.robot_name = param.name

        return SetParametersResult(successful=True)

    def timer_callback(self):
        """Periodically log current parameter values"""
        self.get_logger().info(
            f'Current params - Vel: {self.max_velocity}, '
            f'Safe dist: {self.safety_distance}, '
            f'Freq: {self.control_frequency}'
        )


def main(args=None):
    """Main function to initialize and run the parameter node"""
    rclpy.init(args=args)
    node = ParameterExampleNode()

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

## 5. Launch File Examples

### 5.1 Basic Launch File (Python)

```python
#!/usr/bin/env python3
"""
Launch file for starting multiple ROS 2 nodes for Physical AI system
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the Physical AI system"""

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

    # Create nodes
    status_publisher_node = Node(
        package='physical_ai_examples',
        executable='status_publisher',
        name='status_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    safety_service_node = Node(
        package='physical_ai_examples',
        executable='safety_check_service',
        name='safety_service',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    parameter_node = Node(
        package='physical_ai_examples',
        executable='parameter_example',
        name='parameter_node',
        namespace=namespace,
        parameters=[
            {'max_velocity': 1.0},
            {'safety_distance': 0.5},
            {'control_frequency': 50},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Return the complete launch description
    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        status_publisher_node,
        safety_service_node,
        parameter_node
    ])
```

## 6. Sensor Integration Example

### 6.1 Sensor Data Processing Node (Python)

```python
#!/usr/bin/env python3
"""
Sensor data processing node for Physical AI systems
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
from collections import deque


class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Publishers for processed data
        self.obstacle_distance_pub = self.create_publisher(Float32, 'obstacle_distance', 10)
        self.safety_status_pub = self.create_publisher(Twist, 'safety_twist', 10)

        # Subscriptions for sensor data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )

        # Data buffers
        self.scan_buffer = deque(maxlen=5)
        self.imu_buffer = deque(maxlen=5)

        # Safety parameters
        self.safety_distance = 0.5  # meters
        self.emergency_stop_active = False

        self.get_logger().info('Sensor Processor initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        # Filter out invalid ranges
        valid_ranges = [r for r in msg.ranges if 0.1 < r < 10.0]

        if valid_ranges:
            min_distance = min(valid_ranges)
            self.scan_buffer.append(min_distance)

            # Calculate average distance
            avg_distance = np.mean(self.scan_buffer) if self.scan_buffer else float('inf')

            # Publish obstacle distance
            distance_msg = Float32()
            distance_msg.data = float(avg_distance)
            self.obstacle_distance_pub.publish(distance_msg)

            # Check for safety violations
            if avg_distance < self.safety_distance and not self.emergency_stop_active:
                self.get_logger().warn(f'OBSTACLE TOO CLOSE: {avg_distance:.2f}m')
                self.emergency_stop_active = True
                self.publish_emergency_stop()

    def imu_callback(self, msg):
        """Process IMU data"""
        self.imu_buffer.append({
            'linear_acceleration': msg.linear_acceleration,
            'angular_velocity': msg.angular_velocity,
            'orientation': msg.orientation
        })

        # Check for dangerous accelerations
        lin_acc_mag = np.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )

        if lin_acc_mag > 10.0:  # 10 m/s^2 threshold
            self.get_logger().warn(f'DANGEROUS ACCELERATION: {lin_acc_mag:.2f} m/s^2')

    def joint_callback(self, msg):
        """Process joint state data"""
        # Check joint positions and velocities for safety
        for i, name in enumerate(msg.name):
            if i < len(msg.position) and i < len(msg.velocity):
                pos = msg.position[i]
                vel = msg.velocity[i]

                # Check for dangerous velocities
                if abs(vel) > 5.0:  # 5 rad/s threshold
                    self.get_logger().warn(f'DANGEROUS VELOCITY on {name}: {vel:.2f} rad/s')

    def publish_emergency_stop(self):
        """Publish emergency stop command"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0

        self.safety_status_pub.publish(stop_cmd)
        self.get_logger().info('EMERGENCY STOP PUBLISHED')

    def reset_emergency_stop(self):
        """Reset emergency stop status"""
        self.emergency_stop_active = False
        self.get_logger().info('Emergency stop reset')


def main(args=None):
    """Main function to initialize and run the sensor processor"""
    rclpy.init(args=args)
    processor = SensorProcessor()

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

## 7. Safety Monitor Implementation

### 7.1 Comprehensive Safety Monitor (Python)

```python
#!/usr/bin/env python3
"""
Comprehensive safety monitor for Physical AI systems
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Duration
import threading
import time


class ComprehensiveSafetyMonitor(Node):
    def __init__(self):
        super().__init__('comprehensive_safety_monitor')

        # Safety state variables
        self.system_safe = True
        self.emergency_stop_requested = False
        self.last_velocity_cmd = Twist()
        self.last_pose = None

        # Safety thresholds
        self.max_linear_velocity = 1.0  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        self.min_obstacle_distance = 0.5  # m
        self.max_operation_time = 3600.0  # seconds

        # Start time for operation time limit
        self.start_time = self.get_clock().now()

        # Publishers
        self.safety_status_pub = self.create_publisher(Bool, 'safety_status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self.safety_violation_pub = self.create_publisher(String, 'safety_violation', 10)

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

        self.pose_sub = self.create_subscription(
            PoseStamped,
            'current_pose',
            self.pose_callback,
            10
        )

        # Timer for periodic safety checks
        self.safety_timer = self.create_timer(0.1, self.periodic_safety_check)

        self.get_logger().info('Comprehensive Safety Monitor initialized')

    def cmd_vel_callback(self, msg):
        """Monitor velocity commands for safety violations"""
        # Check linear velocity limits
        linear_speed = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
        if linear_speed > self.max_linear_velocity:
            violation_msg = String()
            violation_msg.data = f'VELOCITY VIOLATION: {linear_speed:.2f} m/s > {self.max_linear_velocity} m/s'
            self.safety_violation_pub.publish(violation_msg)
            self.get_logger().error(violation_msg.data)
            self.system_safe = False

        # Check angular velocity limits
        angular_speed = (msg.angular.x**2 + msg.angular.y**2 + msg.angular.z**2)**0.5
        if angular_speed > self.max_angular_velocity:
            violation_msg = String()
            violation_msg.data = f'ANGULAR VELOCITY VIOLATION: {angular_speed:.2f} rad/s > {self.max_angular_velocity} rad/s'
            self.safety_violation_pub.publish(violation_msg)
            self.get_logger().error(violation_msg.data)
            self.system_safe = False

        # Store last command for monitoring
        self.last_velocity_cmd = msg

    def scan_callback(self, msg):
        """Monitor laser scan for obstacle detection"""
        if len(msg.ranges) > 0:
            # Find minimum distance in front of robot (simplified)
            valid_ranges = [r for r in msg.ranges if 0.1 < r < 10.0]
            if valid_ranges:
                min_distance = min(valid_ranges)

                if min_distance < self.min_obstacle_distance:
                    violation_msg = String()
                    violation_msg.data = f'OBSTACLE TOO CLOSE: {min_distance:.2f} m < {self.min_obstacle_distance} m'
                    self.safety_violation_pub.publish(violation_msg)
                    self.get_logger().error(violation_msg.data)
                    self.system_safe = False

    def pose_callback(self, msg):
        """Monitor robot pose for safety"""
        self.last_pose = msg.pose

    def periodic_safety_check(self):
        """Perform periodic safety checks"""
        # Check operation time limit
        current_time = self.get_clock().now()
        operation_duration = (current_time - self.start_time).nanoseconds / 1e9

        if operation_duration > self.max_operation_time:
            violation_msg = String()
            violation_msg.data = f'OPERATION TIME LIMIT EXCEEDED: {operation_duration:.0f} s > {self.max_operation_time} s'
            self.safety_violation_pub.publish(violation_msg)
            self.get_logger().error(violation_msg.data)
            self.system_safe = False

        # Publish safety status
        safety_msg = Bool()
        safety_msg.data = self.system_safe
        self.safety_status_pub.publish(safety_msg)

        # Publish emergency stop if needed
        emergency_msg = Bool()
        emergency_msg.data = not self.system_safe
        self.emergency_stop_pub.publish(emergency_msg)

        # Reset system safety if it was unsafe and conditions are now safe
        if not self.system_safe:
            self.get_logger().warn('SAFETY SYSTEM ACTIVE - SYSTEM NOT SAFE')
        else:
            self.get_logger().info('Safety check passed')

    def reset_safety_system(self):
        """Reset the safety system (typically requires manual intervention)"""
        self.system_safe = True
        self.get_logger().info('Safety system reset - VERIFY CONDITIONS BEFORE CONTINUING')


def main(args=None):
    """Main function to initialize and run the safety monitor"""
    rclpy.init(args=args)
    safety_monitor = ComprehensiveSafetyMonitor()

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

These code examples demonstrate fundamental ROS 2 concepts essential for Physical AI systems, including node creation, communication patterns, parameter management, and safety considerations. Each example builds upon the previous ones to show how to create robust, safe Physical AI applications using ROS 2.