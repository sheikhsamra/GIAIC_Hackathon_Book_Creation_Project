# Lab Exercises: Vision-Language-Action Systems

## Lab 6.1: Vision-Language Integration for Physical AI

### Objective
Implement a vision-language integration system that can understand natural language commands and ground them to visual objects in a simulated environment.

### Prerequisites
- Completed Module 5 (Humanoid Robotics and Locomotion)
- Basic understanding of computer vision and natural language processing
- Familiarity with ROS 2 and Gazebo

### Estimated Time
4 hours

### Steps

#### Step 1: Create Vision-Language Package
1. Navigate to your workspace:
   ```bash
   cd ~/humanoid_ws/src
   ```

2. Create a new package for vision-language integration:
   ```bash
   ros2 pkg create --build-type ament_python vision_language_integration --dependencies rclpy sensor_msgs geometry_msgs std_msgs cv_bridge message_filters tf2_ros
   ```

3. Navigate to the package directory:
   ```bash
   cd vision_language_integration
   ```

#### Step 2: Create Vision-Language Grounding Node
1. Create the main vision-language grounding node `vision_language_integration/vision_language_integration/vision_language_grounding.py`:
   ```python
   #!/usr/bin/env python3
   """
   Vision-Language integration node for Physical AI systems
   """
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image, CameraInfo
   from geometry_msgs.msg import Point, Pose, Vector3
   from std_msgs.msg import String, Float64MultiArray
   from cv_bridge import CvBridge
   import cv2
   import numpy as np
   import torch
   import torch.nn as nn
   import torchvision.transforms as transforms
   from typing import Dict, List, Tuple


   class VisionLanguageGrounding(Node):
       def __init__(self):
           super().__init__('vision_language_grounding')

           # Publishers
           self.grounded_objects_pub = self.create_publisher(Float64MultiArray, '/grounded_objects', 10)
           self.visual_debug_pub = self.create_publisher(Image, '/vision_language_debug', 10)
           self.command_status_pub = self.create_publisher(String, '/command_status', 10)

           # Subscribers
           self.image_sub = self.create_subscription(
               Image,
               '/camera/image_raw',
               self.image_callback,
               10
           )

           self.command_sub = self.create_subscription(
               String,
               '/natural_language_command',
               self.command_callback,
               10
           )

           # Internal state
           self.cv_bridge = CvBridge()
           self.current_image = None
           self.current_command = None
           self.object_detections = []
           self.language_embeddings = None

           # Vision processing components
           self.image_transform = transforms.Compose([
               transforms.ToTensor(),
               transforms.Resize((224, 224)),
               transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
           ])

           # Simple object detection (in practice, use a pre-trained model)
           self.object_detector = SimpleObjectDetector()

           # Language processing (simplified for this example)
           self.vocabulary = {
               'red': 0, 'blue': 1, 'green': 2, 'yellow': 3,
               'cup': 4, 'bottle': 5, 'box': 6, 'ball': 7,
               'table': 8, 'chair': 9, 'couch': 10,
               'pick': 11, 'grasp': 12, 'take': 13, 'grab': 14,
               'move': 15, 'go': 16, 'navigate': 17, 'approach': 18
           }

           self.color_names = {v: k for k, v in self.vocabulary.items() if v < 4}
           self.object_names = {v: k for k, v in self.vocabulary.items() if 4 <= v < 11}
           self.action_names = {v: k for k, v in self.vocabulary.items() if v >= 11}

           self.get_logger().info('Vision-Language Grounding node initialized')

       def image_callback(self, msg):
           """Process incoming camera image"""
           try:
               # Convert ROS image to OpenCV
               cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
               self.current_image = cv_image

               # Detect objects in the image
               self.object_detections = self.object_detector.detect(cv_image)

               # If there's a pending command, process it
               if self.current_command:
                   self.process_command_with_vision()

           except Exception as e:
               self.get_logger().error(f'Error processing image: {e}')

       def command_callback(self, msg):
           """Process natural language command"""
           self.current_command = msg.data
           self.get_logger().info(f'Received command: {msg.data}')

           # If we have an image, process immediately
           if self.current_image:
               self.process_command_with_vision()

       def process_command_with_vision(self):
           """Process command with current vision data"""
           if not self.current_command or not self.current_image:
               return

           # Parse the command to extract objects and actions
           command_objects = self.parse_command_objects(self.current_command)
           command_actions = self.parse_command_actions(self.current_command)

           # Ground language to visual objects
           grounded_objects = self.ground_language_to_objects(
               command_objects, self.object_detections
           )

           # Generate visual debug output
           debug_image = self.draw_grounding_results(
               self.current_image.copy(), grounded_objects
           )

           # Publish results
           self.publish_grounding_results(grounded_objects, debug_image)

           # Clear command after processing
           self.current_command = None

       def parse_command_objects(self, command: str) -> List[Dict]:
           """Parse objects mentioned in the command"""
           tokens = command.lower().split()
           objects = []

           for i, token in enumerate(tokens):
               if token in self.vocabulary:
                   token_id = self.vocabulary[token]
                   if 4 <= token_id <= 10:  # Object category
                       # Look for color descriptors before object
                       color = None
                       if i > 0 and tokens[i-1] in ['red', 'blue', 'green', 'yellow']:
                           color = tokens[i-1]

                       objects.append({
                           'category': token,
                           'color': color,
                           'confidence': 1.0  # Simplified
                       })

           return objects

       def parse_command_actions(self, command: str) -> List[str]:
           """Parse actions mentioned in the command"""
           tokens = command.lower().split()
           actions = []

           for token in tokens:
               if token in self.vocabulary:
                   token_id = self.vocabulary[token]
                   if token_id >= 11:  # Action category
                       actions.append(token)

           return actions

       def ground_language_to_objects(self, command_objects: List[Dict],
                                    detected_objects: List[Dict]) -> List[Dict]:
           """Ground language objects to visual detections"""
           grounded_results = []

           for cmd_obj in command_objects:
               best_match = None
               best_score = 0.0

               for det_obj in detected_objects:
                   score = self.calculate_matching_score(cmd_obj, det_obj)

                   if score > best_score:
                       best_score = score
                       best_match = {
                           'command_object': cmd_obj,
                           'detected_object': det_obj,
                           'matching_score': score,
                           'position_3d': self.estimate_3d_position(det_obj)
                       }

               if best_match and best_score > 0.5:  # Threshold for matching
                   grounded_results.append(best_match)

           return grounded_results

       def calculate_matching_score(self, cmd_obj: Dict, det_obj: Dict) -> float:
           """Calculate matching score between command object and detected object"""
           score = 0.0

           # Category matching
           if cmd_obj['category'] == det_obj['category']:
               score += 0.6

           # Color matching
           if (cmd_obj['color'] and det_obj.get('color') and
               cmd_obj['color'] == det_obj['color']):
               score += 0.4

           # Size/shape considerations (simplified)
           if 'size' in det_obj and 'category' in cmd_obj:
               expected_size = self.estimate_expected_size(cmd_obj['category'])
               actual_size = det_obj['size']
               size_similarity = 1.0 - abs(expected_size - actual_size) / max(expected_size, actual_size)
               score += 0.2 * size_similarity

           return min(score, 1.0)

       def estimate_expected_size(self, category: str) -> float:
           """Estimate expected size for object category"""
           size_map = {
               'cup': 0.1, 'bottle': 0.2, 'box': 0.3,
               'ball': 0.15, 'table': 1.0, 'chair': 0.5, 'couch': 2.0
           }
           return size_map.get(category, 0.2)

       def estimate_3d_position(self, detection: Dict) -> Point:
           """Estimate 3D position from 2D detection (simplified)"""
           # This is a simplified approach - in practice, use depth information
           # or stereo vision to get accurate 3D positions

           bbox = detection['bbox']
           center_x = (bbox[0] + bbox[2]) / 2  # Center of bounding box
           center_y = (bbox[1] + bbox[3]) / 2

           # Convert to 3D position (simplified - assumes known camera parameters)
           # In practice, use camera info and depth data
           z_distance = 1.0  # Simplified distance estimate
           x_3d = (center_x - 320) * z_distance / 600  # Simplified conversion
           y_3d = (center_y - 240) * z_distance / 600  # Simplified conversion

           return Point(x=x_3d, y=y_3d, z=z_distance)

       def draw_grounding_results(self, image: np.ndarray, grounded_objects: List[Dict]) -> np.ndarray:
           """Draw grounding results on image for debugging"""
           debug_image = image.copy()

           for i, result in enumerate(grounded_objects):
               det_obj = result['detected_object']
               bbox = det_obj['bbox']

               # Draw bounding box
               cv2.rectangle(debug_image, (int(bbox[0]), int(bbox[1])),
                           (int(bbox[2]), int(bbox[3])), (0, 255, 0), 2)

               # Draw label
               label = f"{det_obj['category']} ({result['matching_score']:.2f})"
               cv2.putText(debug_image, label, (int(bbox[0]), int(bbox[1])-10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

               # Draw 3D position estimate
               pos_3d = result['position_3d']
               pos_text = f"[{pos_3d.x:.2f}, {pos_3d.y:.2f}, {pos_3d.z:.2f}]"
               cv2.putText(debug_image, pos_text, (int(bbox[0]), int(bbox[1])+20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

           return debug_image

       def publish_grounding_results(self, grounded_objects: List[Dict], debug_image: np.ndarray):
           """Publish grounding results"""
           # Publish grounded objects
           objects_msg = Float64MultiArray()
           objects_data = []

           for result in grounded_objects:
               pos_3d = result['position_3d']
               objects_data.extend([
                   pos_3d.x, pos_3d.y, pos_3d.z,  # 3D position
                   result['matching_score'],       # Confidence
                   self.vocabulary.get(result['command_object']['category'], 0),  # Category ID
                   self.vocabulary.get(result['command_object'].get('color', ''), 0)  # Color ID
               ])

           objects_msg.data = objects_data
           self.grounded_objects_pub.publish(objects_msg)

           # Publish debug image
           debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
           self.visual_debug_pub.publish(debug_msg)

           # Publish status
           status_msg = String()
           status_msg.data = f'Grounded {len(grounded_objects)} objects'
           self.command_status_pub.publish(status_msg)

           self.get_logger().info(f'Grounded {len(grounded_objects)} objects from command')


   class SimpleObjectDetector:
       """Simple object detector for demonstration (in practice, use YOLO, SSD, etc.)"""
       def __init__(self):
           # In practice, load a pre-trained model here
           pass

       def detect(self, image: np.ndarray) -> List[Dict]:
           """Detect objects in image (simplified implementation)"""
           # This is a simplified detection - in practice, use a real detector
           detections = []

           # Convert to HSV for color-based detection
           hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

           # Define color ranges for simple color-based detection
           color_ranges = {
               'red': ([0, 50, 50], [10, 255, 255]),
               'blue': ([100, 50, 50], [130, 255, 255]),
               'green': ([40, 50, 50], [80, 255, 255]),
               'yellow': ([20, 50, 50], [30, 255, 255])
           }

           # Simple shape detection based on contour analysis
           gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
           blurred = cv2.GaussianBlur(gray, (5, 5), 0)
           edged = cv2.Canny(blurred, 50, 150)

           # Find contours
           contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

           for contour in contours:
               # Filter by area (remove very small or very large contours)
               area = cv2.contourArea(contour)
               if 100 < area < 10000:  # Reasonable size for objects
                   # Get bounding box
                   x, y, w, h = cv2.boundingRect(contour)
                   aspect_ratio = float(w) / h

                   # Estimate object category based on shape and size
                   if 0.8 < aspect_ratio < 1.2:
                       category = 'ball' if area < 5000 else 'box'
                   elif aspect_ratio > 1.5:
                       category = 'bottle'
                   else:
                       category = 'cup'

                   # Determine dominant color in bounding box
                   roi = hsv[y:y+h, x:x+w]
                   color = self.identify_dominant_color(roi, color_ranges)

                   detections.append({
                       'bbox': [x, y, x+w, y+h],
                       'category': category,
                       'color': color,
                       'size': area / (image.shape[0] * image.shape[1]),  # Normalized size
                       'confidence': 0.7  # Simplified confidence
                   })

           return detections

       def identify_dominant_color(self, roi_hsv: np.ndarray, color_ranges: Dict) -> str:
           """Identify dominant color in region of interest"""
           max_pixels = 0
           dominant_color = None

           for color_name, (lower, upper) in color_ranges.items():
               mask = cv2.inRange(roi_hsv, np.array(lower), np.array(upper))
               pixels = cv2.countNonZero(mask)

               if pixels > max_pixels:
                   max_pixels = pixels
                   dominant_color = color_name

           return dominant_color if dominant_color else 'unknown'


   def main(args=None):
       """Main function to initialize and run the vision-language grounding node"""
       rclpy.init(args=args)
       node = VisionLanguageGrounding()

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

2. Make the script executable:
   ```bash
   chmod +x vision_language_integration/vision_language_integration/vision_language_grounding.py
   ```

#### Step 3: Create Vision-Language Integration Test Node
1. Create a test node `vision_language_integration/vision_language_integration/vision_language_test.py`:
   ```python
   #!/usr/bin/env python3
   """
   Test node for vision-language integration
   """
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from std_msgs.msg import String
   from geometry_msgs.msg import Point
   from cv_bridge import CvBridge
   import cv2
   import numpy as np


   class VisionLanguageTest(Node):
       def __init__(self):
           super().__init__('vision_language_test')

           # Publishers
           self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
           self.command_pub = self.create_publisher(String, '/natural_language_command', 10)

           # Subscribers
           self.result_sub = self.create_subscription(
               String,
               '/command_status',
               self.result_callback,
               10
           )

           self.cv_bridge = CvBridge()

           # Timer to send test data
           self.test_timer = self.create_timer(5.0, self.send_test_data)

           self.test_commands = [
               "Pick up the red cup",
               "Move to the blue bottle",
               "Grasp the yellow box",
               "Navigate to the green ball"
           ]
           self.command_index = 0

           self.get_logger().info('Vision-Language Test node initialized')

       def send_test_data(self):
           """Send test image and command"""
           # Create a synthetic test image with colored objects
           test_image = self.create_test_image()
           image_msg = self.cv_bridge.cv2_to_imgmsg(test_image, encoding='bgr8')
           self.image_pub.publish(image_msg)

           # Send a test command
           command_msg = String()
           command_msg.data = self.test_commands[self.command_index]
           self.command_pub.publish(command_msg)

           self.get_logger().info(f'Sent test command: {command_msg.data}')
           self.command_index = (self.command_index + 1) % len(self.test_commands)

       def create_test_image(self) -> np.ndarray:
           """Create a synthetic test image with colored objects"""
           # Create a blank image
           img = np.ones((480, 640, 3), dtype=np.uint8) * 240  # Light gray background

           # Draw colored objects
           # Red cup
           cv2.circle(img, (100, 100), 30, (0, 0, 255), -1)  # Red circle
           cv2.circle(img, (100, 100), 30, (0, 0, 0), 2)     # Black outline

           # Blue bottle
           cv2.rectangle(img, (200, 150), (230, 250), (255, 0, 0), -1)  # Blue rectangle
           cv2.rectangle(img, (200, 150), (230, 250), (0, 0, 0), 2)     # Black outline

           # Yellow box
           cv2.rectangle(img, (350, 100), (400, 150), (0, 255, 255), -1)  # Yellow rectangle
           cv2.rectangle(img, (350, 100), (400, 150), (0, 0, 0), 2)      # Black outline

           # Green ball
           cv2.circle(img, (500, 200), 25, (0, 255, 0), -1)  # Green circle
           cv2.circle(img, (500, 200), 25, (0, 0, 0), 2)     # Black outline

           # Add table
           cv2.rectangle(img, (50, 300), (600, 450), (139, 69, 19), -1)  # Brown table
           cv2.rectangle(img, (50, 300), (600, 450), (0, 0, 0), 2)       # Black outline

           return img

       def result_callback(self, msg):
           """Handle result from vision-language system"""
           self.get_logger().info(f'Received result: {msg.data}')


   def main(args=None):
       """Main function to initialize and run the test node"""
       rclpy.init(args=args)
       node = VisionLanguageTest()

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

2. Make the test script executable:
   ```bash
   chmod +x vision_language_integration/vision_language_integration/vision_language_test.py
   ```

#### Step 4: Create Launch File
1. Create a launch directory:
   ```bash
   mkdir -p vision_language_integration/launch
   ```

2. Create a launch file `vision_language_integration/launch/vision_language_demo.launch.py`:
   ```python
   #!/usr/bin/env python3
   """
   Launch file for vision-language integration demo
   """
   from launch import LaunchDescription
   from launch_ros.actions import Node


   def generate_launch_description():
       # Vision-language grounding node
       vision_language_node = Node(
           package='vision_language_integration',
           executable='vision_language_grounding',
           name='vision_language_grounding',
           output='screen',
           parameters=[
               {'use_sim_time': False}
           ]
       )

       # Test node
       test_node = Node(
           package='vision_language_integration',
           executable='vision_language_test',
           name='vision_language_test',
           output='screen',
           parameters=[
               {'use_sim_time': False}
           ]
       )

       return LaunchDescription([
           vision_language_node,
           test_node
       ])
   ```

#### Step 5: Update Setup Files
1. Update `setup.py` to include the new executables:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   package_name = 'vision_language_integration'

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
       description='Vision-language integration for Physical AI',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'vision_language_grounding = vision_language_integration.vision_language_grounding:main',
               'vision_language_test = vision_language_integration.vision_language_test:main',
           ],
       },
   )
   ```

#### Step 6: Build and Test Vision-Language System
1. Build the package:
   ```bash
   cd ~/humanoid_ws
   colcon build --packages-select vision_language_integration
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Run the vision-language integration demo:
   ```bash
   ros2 launch vision_language_integration vision_language_demo.launch.py
   ```

4. In another terminal, check the published topics:
   ```bash
   ros2 topic list | grep vision
   ros2 topic echo /grounded_objects
   ros2 topic echo /command_status
   ```

### Expected Results
- Vision-language system processes commands and images
- Objects are detected and matched to language commands
- Grounded object positions are published
- Visual debugging shows bounding boxes and labels
- System successfully connects language to visual objects

### Analysis Questions
1. How does the system handle ambiguous language commands?
2. What are the limitations of the simple object detection approach?
3. How would you improve the grounding accuracy for complex scenes?
4. What challenges arise when scaling to real-world environments?

---

## Lab 6.2: Action Generation and Execution

### Objective
Implement an action generation system that translates grounded language-vision information into executable robot commands for physical interaction.

### Prerequisites
- Completed Lab 6.1
- Understanding of robot kinematics and control
- Experience with motion planning

### Estimated Time
5 hours

### Steps

#### Step 1: Create Action Generation Package
1. Create a new package for action generation:
   ```bash
   cd ~/humanoid_ws/src
   ros2 pkg create --build-type ament_python action_generation_system --dependencies rclpy sensor_msgs geometry_msgs nav_msgs std_msgs trajectory_msgs control_msgs action_msgs
   ```

2. Navigate to the package directory:
   ```bash
   cd action_generation_system
   ```

#### Step 2: Create Action Generation Node
1. Create the main action generation node `action_generation_system/action_generation_system/action_generator.py`:
   ```python
   #!/usr/bin/env python3
   """
   Action generation system for VLA (Vision-Language-Action) systems
   """
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String, Float64MultiArray
   from geometry_msgs.msg import Point, Pose, Twist
   from sensor_msgs.msg import JointState
   from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
   from action_msgs.msg import GoalStatus
   from builtin_interfaces.msg import Duration
   import numpy as np
   import math
   from enum import Enum
   from typing import Dict, List, Optional


   class ActionType(Enum):
       NAVIGATION = "navigation"
       MANIPULATION = "manipulation"
       INTERACTION = "interaction"
       INSPECTION = "inspection"


   class ActionGenerator(Node):
       def __init__(self):
           super().__init__('action_generator')

           # Publishers
           self.joint_trajectory_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
           self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
           self.action_status_pub = self.create_publisher(String, '/action_status', 10)
           self.trajectory_debug_pub = self.create_publisher(Float64MultiArray, '/trajectory_debug', 10)

           # Subscribers
           self.grounded_objects_sub = self.create_subscription(
               Float64MultiArray,
               '/grounded_objects',
               self.grounded_objects_callback,
               10
           )

           self.command_sub = self.create_subscription(
               String,
               '/natural_language_command',
               self.command_callback,
               10
           )

           self.joint_state_sub = self.create_subscription(
               JointState,
               '/joint_states',
               self.joint_state_callback,
               10
           )

           # Internal state
           self.grounded_objects = []
           self.current_command = ""
           self.current_joints = JointState()
           self.action_queue = []
           self.active_action = None
           self.execution_status = "idle"

           # Robot parameters
           self.arm_joint_names = [
               'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
               'left_elbow_pitch', 'left_wrist_pitch', 'left_wrist_yaw'
           ]
           self.wheel_joint_names = ['left_wheel', 'right_wheel']

           # Action parameters
           self.grasp_approach_distance = 0.1  # meters
           self.grasp_height_offset = 0.05    # meters
           self.navigation_tolerance = 0.2    # meters
           self.manipulation_speed = 0.5      # normalized speed (0-1)

           # Timer for action execution
           self.execution_timer = self.create_timer(0.1, self.execute_action)

           self.get_logger().info('Action Generation System initialized')

       def grounded_objects_callback(self, msg):
           """Process grounded objects from vision-language system"""
           # Parse object data from multi-array message
           # Format: [x, y, z, confidence, category_id, color_id, ...]
           if len(msg.data) % 6 != 0:
               self.get_logger().warn('Invalid grounded objects data format')
               return

           self.grounded_objects = []
           for i in range(0, len(msg.data), 6):
               obj_data = msg.data[i:i+6]
               obj = {
                   'position': Point(x=obj_data[0], y=obj_data[1], z=obj_data[2]),
                   'confidence': obj_data[3],
                   'category_id': int(obj_data[4]),
                   'color_id': int(obj_data[5])
               }
               self.grounded_objects.append(obj)

           self.get_logger().info(f'Updated {len(self.grounded_objects)} grounded objects')

       def command_callback(self, msg):
           """Process natural language command and generate actions"""
           self.current_command = msg.data
           self.get_logger().info(f'Received command: {msg.data}')

           # Parse command and generate appropriate actions
           actions = self.parse_command_to_actions(msg.data)

           # Add actions to queue
           self.action_queue.extend(actions)

           # If no action is currently executing, start the next one
           if self.active_action is None and self.action_queue:
               self.start_next_action()

       def joint_state_callback(self, msg):
           """Update current joint states"""
           self.current_joints = msg

       def parse_command_to_actions(self, command: str) -> List[Dict]:
           """Parse natural language command to executable actions"""
           command_lower = command.lower()
           actions = []

           # Identify action type based on keywords
           if any(keyword in command_lower for keyword in ['pick', 'grasp', 'take', 'grab']):
               action_type = ActionType.MANIPULATION
           elif any(keyword in command_lower for keyword in ['go', 'move', 'navigate', 'approach', 'walk']):
               action_type = ActionType.NAVIGATION
           elif any(keyword in command_lower for keyword in ['look', 'inspect', 'examine', 'check']):
               action_type = ActionType.INSPECTION
           else:
               action_type = ActionType.INTERACTION

           # Extract target object if specified
           target_object = self.identify_target_object(command_lower)

           # Generate action based on type and target
           if action_type == ActionType.MANIPULATION and target_object:
               action = self.create_manipulation_action(target_object, command_lower)
               if action:
                   actions.append(action)
           elif action_type == ActionType.NAVIGATION and target_object:
               action = self.create_navigation_action(target_object, command_lower)
               if action:
                   actions.append(action)
           elif action_type == ActionType.INSPECTION and target_object:
               action = self.create_inspection_action(target_object, command_lower)
               if action:
                   actions.append(action)
           elif action_type == ActionType.INTERACTION:
               action = self.create_interaction_action(command_lower)
               if action:
                   actions.append(action)

           return actions

       def identify_target_object(self, command: str) -> Optional[Dict]:
           """Identify target object from grounded objects based on command"""
           if not self.grounded_objects:
               return None

           # Simple keyword matching for object categories
           object_keywords = {
               4: ['cup'], 5: ['bottle'], 6: ['box'], 7: ['ball'],
               8: ['table'], 9: ['chair'], 10: ['couch']
           }

           color_keywords = {
               0: ['red'], 1: ['blue'], 2: ['green'], 3: ['yellow']
           }

           best_match = None
           best_score = 0.0

           for obj in self.grounded_objects:
               score = 0.0

               # Check for color match
               for color_id, color_words in color_keywords.items():
                   if color_id == obj['color_id'] and any(word in command for word in color_words):
                       score += 0.5

               # Check for category match
               for cat_id, cat_words in object_keywords.items():
                   if cat_id == obj['category_id'] and any(word in command for word in cat_words):
                       score += 0.5

               # Consider confidence
               score *= obj['confidence']

               if score > best_score:
                   best_score = score
                   best_match = obj

           return best_match if best_score > 0.3 else None  # Threshold for valid match

       def create_manipulation_action(self, target_object: Dict, command: str) -> Optional[Dict]:
           """Create manipulation action for target object"""
           try:
               # Determine specific manipulation type
               if 'pick' in command or 'grasp' in command or 'take' in command:
                   manipulation_type = 'grasp'
               elif 'place' in command or 'put' in command:
                   manipulation_type = 'place'
               elif 'lift' in command:
                   manipulation_type = 'lift'
               else:
                   manipulation_type = 'grasp'  # Default

               # Calculate approach position
               target_pos = target_object['position']
               approach_pos = Point(
                   x=target_pos.x - self.grasp_approach_distance,  # Approach from front
                   y=target_pos.y,
                   z=target_pos.z + self.grasp_height_offset
               )

               # Generate trajectory for manipulation
               trajectory = self.generate_manipulation_trajectory(
                   approach_pos, target_pos, manipulation_type
               )

               action = {
                   'type': ActionType.MANIPULATION,
                   'manipulation_type': manipulation_type,
                   'target_object': target_object,
                   'approach_position': approach_pos,
                   'target_position': target_pos,
                   'trajectory': trajectory,
                   'status': 'pending'
               }

               return action

           except Exception as e:
               self.get_logger().error(f'Error creating manipulation action: {e}')
               return None

       def create_navigation_action(self, target_object: Dict, command: str) -> Optional[Dict]:
           """Create navigation action to target object"""
           try:
               # Calculate navigation destination
               target_pos = target_object['position']
               navigation_dest = Point(
                   x=target_pos.x - 0.5,  # Stay at distance from object
                   y=target_pos.y,
                   z=0.0  # Navigation is 2D
               )

               # Generate path to destination
               path = self.generate_navigation_path(navigation_dest)

               action = {
                   'type': ActionType.NAVIGATION,
                   'destination': navigation_dest,
                   'path': path,
                   'target_object': target_object,
                   'status': 'pending'
               }

               return action

           except Exception as e:
               self.get_logger().error(f'Error creating navigation action: {e}')
               return None

       def create_inspection_action(self, target_object: Dict, command: str) -> Optional[Dict]:
           """Create inspection action for target object"""
           try:
               # For inspection, navigate to object and perform visual inspection
               target_pos = target_object['position']

               # Position for inspection
               inspection_pos = Point(
                   x=target_pos.x - 0.3,  # Close for inspection
                   y=target_pos.y,
                   z=target_pos.z + 0.2  # Slightly elevated view
               )

               action = {
                   'type': ActionType.INSPECTION,
                   'inspection_position': inspection_pos,
                   'target_object': target_object,
                   'status': 'pending'
               }

               return action

           except Exception as e:
               self.get_logger().error(f'Error creating inspection action: {e}')
               return None

       def create_interaction_action(self, command: str) -> Optional[Dict]:
           """Create general interaction action"""
           try:
               # Parse interaction command for specific behavior
               if 'wave' in command:
                   behavior = 'wave'
               elif 'nod' in command:
                   behavior = 'nod'
               elif 'point' in command:
                   behavior = 'point'
               else:
                   behavior = 'acknowledge'  # Default interaction

               action = {
                   'type': ActionType.INTERACTION,
                   'behavior': behavior,
                   'command': command,
                   'status': 'pending'
               }

               return action

           except Exception as e:
               self.get_logger().error(f'Error creating interaction action: {e}')
               return None

       def generate_manipulation_trajectory(self, approach_pos: Point, target_pos: Point,
                                          manipulation_type: str) -> List[JointTrajectoryPoint]:
           """Generate joint trajectory for manipulation"""
           trajectory_points = []

           # Calculate joint angles for approach position
           approach_angles = self.inverse_kinematics(approach_pos, 'arm')
           target_angles = self.inverse_kinematics(target_pos, 'arm')

           # Create trajectory points
           num_points = 20
           for i in range(num_points + 1):
               t = i / num_points

               # Interpolate between approach and target positions
               point = JointTrajectoryPoint()

               # Interpolate joint positions
               interpolated_positions = []
               for j in range(len(approach_angles)):
                   pos = approach_angles[j] + t * (target_angles[j] - approach_angles[j])
                   interpolated_positions.append(pos)

               point.positions = interpolated_positions

               # Set timing (assuming 2 seconds total)
               point.time_from_start.sec = int(t * 2.0)
               point.time_from_start.nanosec = int((t * 2.0 - int(t * 2.0)) * 1e9)

               trajectory_points.append(point)

           return trajectory_points

       def generate_navigation_path(self, destination: Point) -> List[Point]:
           """Generate navigation path to destination (simplified)"""
           path = []

           # Current position (simplified - in practice get from odometry)
           current_pos = Point(x=0.0, y=0.0, z=0.0)

           # Simple straight-line path
           steps = 10
           for i in range(steps + 1):
               t = i / steps
               intermediate_point = Point(
                   x=current_pos.x + t * (destination.x - current_pos.x),
                   y=current_pos.y + t * (destination.y - current_pos.y),
                   z=0.0
               )
               path.append(intermediate_point)

           return path

       def inverse_kinematics(self, position: Point, limb: str = 'arm') -> List[float]:
           """Simple inverse kinematics (simplified for demonstration)"""
           # This is a very simplified IK implementation
           # In practice, use proper IK solvers like KDL or MoveIt!

           if limb == 'arm':
               # Simplified 3DOF arm IK
               x, y, z = position.x, position.y, position.z

               # Calculate joint angles (very simplified)
               shoulder_yaw = math.atan2(y, x)
               shoulder_pitch = math.atan2(z, math.sqrt(x*x + y*y)) - 0.5  # Offset for arm length
               elbow_pitch = 1.0  # Fixed elbow angle for simplicity

               # Return joint angles for a 6DOF arm (filling with dummy values)
               return [shoulder_pitch, 0.0, shoulder_yaw, elbow_pitch, 0.0, 0.0]
           else:
               return [0.0] * 6  # Default joint angles

       def execute_action(self):
           """Execute the current action in the queue"""
           if self.active_action is None:
               if self.action_queue:
                   self.start_next_action()
               return

           # Check action status and continue execution
           if self.active_action['status'] == 'executing':
               # For this example, we'll simulate action completion
               # In practice, monitor actual robot execution

               # Publish trajectory for manipulation actions
               if self.active_action['type'] == ActionType.MANIPULATION:
                   self.publish_manipulation_trajectory(self.active_action['trajectory'])

               # Simulate completion after a certain time
               if self.get_clock().now().nanoseconds / 1e9 > self.action_start_time + 2.0:
                   self.complete_action()
           elif self.active_action['status'] == 'pending':
               # Start action execution
               self.active_action['status'] = 'executing'
               self.action_start_time = self.get_clock().now().nanoseconds / 1e9

       def start_next_action(self):
           """Start executing the next action in the queue"""
           if self.action_queue:
               self.active_action = self.action_queue.pop(0)
               self.active_action['status'] = 'executing'
               self.action_start_time = self.get_clock().now().nanoseconds / 1e9

               self.get_logger().info(f'Starting action: {self.active_action["type"].value}')

               # Publish action status
               status_msg = String()
               status_msg.data = f'Starting {self.active_action["type"].value} action'
               self.action_status_pub.publish(status_msg)

       def complete_action(self):
           """Complete the current action"""
           if self.active_action:
               self.get_logger().info(f'Completed action: {self.active_action["type"].value}')

               # Publish completion status
               status_msg = String()
               status_msg.data = f'Completed {self.active_action["type"].value} action'
               self.action_status_pub.publish(status_msg)

               # Clear active action
               self.active_action = None

               # Start next action if available
               if self.action_queue:
                   self.start_next_action()

       def publish_manipulation_trajectory(self, trajectory_points: List[JointTrajectoryPoint]):
           """Publish joint trajectory for manipulation"""
           if not trajectory_points:
               return

           traj_msg = JointTrajectory()
           traj_msg.joint_names = self.arm_joint_names
           traj_msg.points = trajectory_points

           self.joint_trajectory_pub.publish(traj_msg)

       def publish_navigation_command(self, destination: Point):
           """Publish navigation command"""
           # Simplified navigation command
           cmd_vel = Twist()

           # Calculate direction to destination
           current_pos = Point(x=0.0, y=0.0, z=0.0)  # Simplified
           dx = destination.x - current_pos.x
           dy = destination.y - current_pos.y

           # Simple proportional controller
           cmd_vel.linear.x = min(0.5, max(-0.5, dx * 0.5))  # Limit speed
           cmd_vel.angular.z = min(1.0, max(-1.0, math.atan2(dy, dx) * 2.0))

           self.cmd_vel_pub.publish(cmd_vel)


   def main(args=None):
       """Main function to initialize and run the action generator"""
       rclpy.init(args=args)
       node = ActionGenerator()

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

2. Make the script executable:
   ```bash
   chmod +x action_generation_system/action_generation_system/action_generator.py
   ```

#### Step 3: Create Action Execution Test Node
1. Create a test node `action_generation_system/action_generation_system/action_test.py`:
   ```python
   #!/usr/bin/env python3
   """
   Test node for action generation system
   """
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String, Float64MultiArray
   from geometry_msgs.msg import Point


   class ActionTest(Node):
       def __init__(self):
           super().__init__('action_test')

           # Publishers
           self.command_pub = self.create_publisher(String, '/natural_language_command', 10)
           self.grounded_objects_pub = self.create_publisher(Float64MultiArray, '/grounded_objects', 10)

           # Subscribers
           self.status_sub = self.create_subscription(
               String,
               '/action_status',
               self.status_callback,
               10
           )

           # Timer to send test commands
           self.test_timer = self.create_timer(8.0, self.send_test_commands)

           self.test_commands = [
               "Pick up the red cup",
               "Navigate to the blue bottle",
               "Inspect the yellow box",
               "Move to the green ball"
           ]
           self.command_index = 0

           # Create some simulated grounded objects
           self.create_simulated_objects()

           self.get_logger().info('Action Test node initialized')

       def create_simulated_objects(self):
           """Create simulated grounded objects for testing"""
           objects_msg = Float64MultiArray()

           # Format: [x, y, z, confidence, category_id, color_id]
           objects_data = [
               # Red cup at (1.0, 0.5, 0.1)
               1.0, 0.5, 0.1, 0.9, 4.0, 0.0,  # Category 4 = cup, Color 0 = red
               # Blue bottle at (2.0, -0.5, 0.2)
               2.0, -0.5, 0.2, 0.85, 5.0, 1.0,  # Category 5 = bottle, Color 1 = blue
               # Yellow box at (0.5, 1.0, 0.05)
               0.5, 1.0, 0.05, 0.95, 6.0, 3.0,  # Category 6 = box, Color 3 = yellow
               # Green ball at (-0.5, -1.0, 0.1)
               -0.5, -1.0, 0.1, 0.8, 7.0, 2.0,  # Category 7 = ball, Color 2 = green
           ]

           objects_msg.data = objects_data
           self.grounded_objects_pub.publish(objects_msg)

       def send_test_commands(self):
           """Send test commands to action system"""
           if self.command_index < len(self.test_commands):
               command_msg = String()
               command_msg.data = self.test_commands[self.command_index]
               self.command_pub.publish(command_msg)

               self.get_logger().info(f'Sent test command: {command_msg.data}')
               self.command_index += 1
           else:
               # Reset after all commands sent
               self.command_index = 0

       def status_callback(self, msg):
           """Handle action status updates"""
           self.get_logger().info(f'Action status: {msg.data}')


   def main(args=None):
       """Main function to initialize and run the test node"""
       rclpy.init(args=args)
       node = ActionTest()

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

2. Make the test script executable:
   ```bash
   chmod +x action_generation_system/action_generation_system/action_test.py
   ```

#### Step 4: Create Action Generation Launch File
1. Create a launch directory:
   ```bash
   mkdir -p action_generation_system/launch
   ```

2. Create a launch file `action_generation_system/launch/action_demo.launch.py`:
   ```python
   #!/usr/bin/env python3
   """
   Launch file for action generation demo
   """
   from launch import LaunchDescription
   from launch_ros.actions import Node


   def generate_launch_description():
       # Action generation node
       action_generator = Node(
           package='action_generation_system',
           executable='action_generator',
           name='action_generator',
           output='screen',
           parameters=[
               {'use_sim_time': False}
           ]
       )

       # Test node
       action_test = Node(
           package='action_generation_system',
           executable='action_test',
           name='action_test',
           output='screen',
           parameters=[
               {'use_sim_time': False}
           ]
       )

       return LaunchDescription([
           action_generator,
           action_test
       ])
   ```

#### Step 5: Update Action Generation Setup Files
1. Update `setup.py` for the action generation package:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   package_name = 'action_generation_system'

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
       description='Action generation for VLA systems',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'action_generator = action_generation_system.action_generator:main',
               'action_test = action_generation_system.action_test:main',
           ],
       },
   )
   ```

#### Step 6: Build and Test Action Generation System
1. Build the package:
   ```bash
   cd ~/humanoid_ws
   colcon build --packages-select action_generation_system
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Run the action generation demo:
   ```bash
   ros2 launch action_generation_system action_demo.launch.py
   ```

4. In another terminal, check the published topics:
   ```bash
   ros2 topic list | grep action
   ros2 topic echo /action_status
   ros2 topic echo /joint_trajectory
   ```

### Expected Results
- Action generation system processes language commands
- System generates appropriate actions based on grounded objects
- Joint trajectories are published for manipulation tasks
- Navigation commands are generated for mobility tasks
- Action execution is tracked and reported

### Analysis Questions
1. How does the system handle conflicting commands?
2. What are the limitations of the simplified inverse kinematics?
3. How would you extend this to handle multiple simultaneous goals?
4. What safety considerations should be included in action execution?

---

## Lab 6.3: AI-Integrated Control System

### Objective
Implement an AI-integrated control system that uses machine learning to optimize action selection and execution based on environmental feedback.

### Prerequisites
- Completed Labs 6.1 and 6.2
- Understanding of basic machine learning concepts
- Experience with reinforcement learning fundamentals

### Estimated Time
6 hours

### Steps

#### Step 1: Create AI Control Package
1. Create a new package for AI control:
   ```bash
   cd ~/humanoid_ws/src
   ros2 pkg create --build-type ament_python ai_control_system --dependencies rclpy sensor_msgs geometry_msgs std_msgs message_filters tf2_ros builtin_interfaces
   ```

2. Navigate to the package directory:
   ```bash
   cd ai_control_system
   ```

#### Step 2: Create AI Control Node
1. Create the main AI control node `ai_control_system/ai_control_system/ai_controller.py`:
   ```python
   #!/usr/bin/env python3
   """
   AI-integrated controller for Physical AI systems
   Uses reinforcement learning for adaptive action selection and optimization
   """
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import JointState, Imu, LaserScan
   from geometry_msgs.msg import Point, Twist, Pose
   from std_msgs.msg import String, Float64MultiArray, Bool
   from builtin_interfaces.msg import Time
   import numpy as np
   import math
   import random
   from collections import deque
   from typing import Dict, List, Tuple, Optional


   class AIController(Node):
       def __init__(self):
           super().__init__('ai_controller')

           # Publishers
           self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
           self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
           self.ai_status_pub = self.create_publisher(String, '/ai_status', 10)
           self.learning_stats_pub = self.create_publisher(Float64MultiArray, '/learning_stats', 10)

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

           self.scan_sub = self.create_subscription(
               LaserScan,
               '/scan',
               self.scan_callback,
               10
           )

           self.command_sub = self.create_subscription(
               String,
               '/natural_language_command',
               self.command_callback,
               10
           )

           self.grounded_objects_sub = self.create_subscription(
               Float64MultiArray,
               '/grounded_objects',
               self.grounded_objects_callback,
               10
           )

           # Internal state
           self.joint_positions = {}
           self.joint_velocities = {}
           self.current_imu = None
           self.current_scan = None
           self.current_command = ""
           self.grounded_objects = []
           self.current_pose = Point(x=0.0, y=0.0, z=0.0)
           self.current_twist = Twist()
           self.ai_enabled = True

           # AI controller parameters
           self.state_size = 12  # Dimension of state space
           self.action_size = 6  # Dimension of action space
           self.learning_rate = 0.001
           self.discount_factor = 0.95
           self.epsilon = 1.0  # Exploration rate
           self.epsilon_decay = 0.995
           self.epsilon_min = 0.01

           # Neural network weights (simplified linear approximation)
           self.actor_weights = np.random.randn(self.state_size, self.action_size) * 0.1
           self.critic_weights = np.random.randn(self.state_size + self.action_size, 1) * 0.1

           # Experience replay buffer
           self.replay_buffer = deque(maxlen=10000)

           # Learning parameters
           self.batch_size = 32
           self.update_frequency = 4
           self.step_count = 0

           # Performance tracking
           self.episode_rewards = []
           self.current_episode_reward = 0.0
           self.episode_steps = 0

           # Task-specific parameters
           self.target_position = Point(x=0.0, y=0.0, z=0.0)
           self.task_type = "navigation"  # navigation, manipulation, etc.
           self.task_completed = False

           # Timer for AI control loop
           self.ai_timer = self.create_timer(0.1, self.ai_control_loop)

           self.get_logger().info('AI Controller initialized')

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

       def scan_callback(self, msg):
           """Update laser scan data"""
           self.current_scan = msg

       def command_callback(self, msg):
           """Update command and set task"""
           self.current_command = msg.data
           self.parse_command_for_task(msg.data)

       def grounded_objects_callback(self, msg):
           """Update grounded objects"""
           # Parse grounded objects from message
           if len(msg.data) % 6 == 0:
               self.grounded_objects = []
               for i in range(0, len(msg.data), 6):
                   obj_data = msg.data[i:i+6]
                   obj = {
                       'position': Point(x=obj_data[0], y=obj_data[1], z=obj_data[2]),
                       'confidence': obj_data[3],
                       'category_id': int(obj_data[4]),
                       'color_id': int(obj_data[5])
                   }
                   self.grounded_objects.append(obj)

       def parse_command_for_task(self, command: str):
           """Parse command to set task type and target"""
           command_lower = command.lower()

           if 'navigate' in command_lower or 'go to' in command_lower:
               self.task_type = "navigation"
               # Find target object in grounded objects
               target_obj = self.find_target_object(command_lower)
               if target_obj:
                   self.target_position = target_obj['position']
           elif 'pick' in command_lower or 'grasp' in command_lower:
               self.task_type = "manipulation"
               target_obj = self.find_target_object(command_lower)
               if target_obj:
                   self.target_position = target_obj['position']
           else:
               self.task_type = "navigation"  # Default task type

       def find_target_object(self, command: str) -> Optional[Dict]:
           """Find target object based on command keywords"""
           for obj in self.grounded_objects:
               # Simple keyword matching based on category IDs
               if 'cup' in command and obj['category_id'] == 4:
                   return obj
               elif 'bottle' in command and obj['category_id'] == 5:
                   return obj
               elif 'box' in command and obj['category_id'] == 6:
                   return obj
               elif 'ball' in command and obj['category_id'] == 7:
                   return obj

           # If no specific object found, return the closest one
           if self.grounded_objects:
               return self.grounded_objects[0]

           return None

       def ai_control_loop(self):
           """Main AI control loop"""
           if not self.ai_enabled:
               return

           # Get current state
           state = self.get_state()

           # Select action using AI policy
           action = self.select_action(state)

           # Execute action
           self.execute_action(action)

           # Calculate reward
           reward = self.calculate_reward()

           # Get next state
           next_state = self.get_state()

           # Store experience
           self.replay_buffer.append((state, action, reward, next_state, self.task_completed))

           # Update AI model
           if len(self.replay_buffer) > self.batch_size and self.step_count % self.update_frequency == 0:
               self.update_model()

           # Update counters
           self.step_count += 1
           self.current_episode_reward += reward
           self.episode_steps += 1

           # Check if task is completed
           if self.is_task_completed():
               self.end_episode()

           # Publish AI status
           self.publish_ai_status(reward, action)

           # Log performance periodically
           if self.step_count % 100 == 0:
               avg_reward = np.mean(self.episode_rewards[-10:]) if self.episode_rewards else 0.0
               self.get_logger().info(
                   f'Step: {self.step_count}, '
                   f'Avg Reward: {avg_reward:.3f}, '
                   f'Epsilon: {self.epsilon:.3f}, '
                   f'Task: {self.task_type}'
               )

       def get_state(self) -> np.ndarray:
           """Get current state representation"""
           state = np.zeros(self.state_size)

           # Position information
           state[0] = self.current_pose.x
           state[1] = self.current_pose.y
           state[2] = self.current_pose.z

           # Velocity information
           state[3] = self.current_twist.linear.x
           state[4] = self.current_twist.linear.y
           state[5] = self.current_twist.angular.z

           # Target information
           state[6] = self.target_position.x - self.current_pose.x
           state[7] = self.target_position.y - self.current_pose.y
           state[8] = math.sqrt(state[6]**2 + state[7]**2)  # Distance to target

           # Sensor information (simplified)
           if self.current_scan and len(self.current_scan.ranges) > 0:
               # Get minimum distance to obstacles
               valid_ranges = [r for r in self.current_scan.ranges if 0.1 < r < 10.0]
               if valid_ranges:
                   state[9] = min(valid_ranges)  # Closest obstacle
               else:
                   state[9] = 10.0  # No obstacles detected

               # Get front distance
               front_ranges = self.current_scan.ranges[:10] + self.current_scan.ranges[-10:]
               front_distances = [r for r in front_ranges if 0.1 < r < 10.0]
               if front_distances:
                   state[10] = min(front_distances)  # Front obstacle distance
               else:
                    state[10] = 10.0
           else:
               state[9] = 5.0
               state[10] = 5.0

           # Balance information (from IMU if available)
           if self.current_imu:
               state[11] = self.current_imu.orientation.z  # Simplified balance measure
           else:
               state[11] = 0.0

           return state

       def select_action(self, state: np.ndarray) -> np.ndarray:
           """Select action using epsilon-greedy policy"""
           if random.random() < self.epsilon:
               # Explore: random action
               action = np.random.uniform(-1, 1, self.action_size)
           else:
               # Exploit: best action according to policy
               action = self.get_best_action(state)

           # Decay epsilon
           if self.epsilon > self.epsilon_min:
               self.epsilon *= self.epsilon_decay

           return action

       def get_best_action(self, state: np.ndarray) -> np.ndarray:
           """Get best action according to current policy"""
           # Simple linear policy for demonstration
           # In practice, use a neural network
           action = np.tanh(np.dot(state, self.actor_weights))

           # Scale action based on task type
           if self.task_type == "navigation":
               # Navigation: prioritize forward movement and turning
               action[0] = np.clip(action[0], -0.5, 0.5)  # Linear velocity
               action[1] = np.clip(action[1], -1.0, 1.0)  # Angular velocity
           elif self.task_type == "manipulation":
               # Manipulation: prioritize precise positioning
               action[0] = np.clip(action[0], -0.2, 0.2)  # Small movements
               action[1] = np.clip(action[1], -0.5, 0.5)  # Controlled turns

           return action

       def execute_action(self, action: np.ndarray):
           """Execute the selected action"""
           if self.task_type == "navigation":
               self.execute_navigation_action(action)
           elif self.task_type == "manipulation":
               self.execute_manipulation_action(action)

       def execute_navigation_action(self, action: np.ndarray):
           """Execute navigation action"""
           cmd_vel = Twist()
           cmd_vel.linear.x = action[0] * 0.5  # Scale linear velocity
           cmd_vel.angular.z = action[1] * 1.0  # Scale angular velocity

           # Add some components for other DOFs if needed
           if len(action) > 2:
               cmd_vel.linear.y = action[2] * 0.2
           if len(action) > 3:
               cmd_vel.linear.z = action[3] * 0.1
           if len(action) > 4:
               cmd_vel.angular.x = action[4] * 0.1
           if len(action) > 5:
               cmd_vel.angular.y = action[5] * 0.1

           self.cmd_vel_pub.publish(cmd_vel)

       def execute_manipulation_action(self, action: np.ndarray):
           """Execute manipulation action"""
           # For manipulation, publish joint commands
           joint_cmd = Float64MultiArray()
           joint_cmd.data = action.tolist()  # Simple mapping for demonstration
           self.joint_cmd_pub.publish(joint_cmd)

       def calculate_reward(self) -> float:
           """Calculate reward based on current state and action"""
           reward = 0.0

           # Distance to target reward
           distance_to_target = math.sqrt(
               (self.target_position.x - self.current_pose.x)**2 +
               (self.target_position.y - self.current_pose.y)**2
           )
           distance_reward = -distance_to_target * 0.1  # Negative reward for distance
           reward += distance_reward

           # Progress toward target
           if hasattr(self, 'prev_distance_to_target'):
               if distance_to_target < self.prev_distance_to_target:
                   reward += 0.05  # Positive reward for progress
               else:
                   reward -= 0.02  # Small penalty for moving away

           self.prev_distance_to_target = distance_to_target

           # Obstacle avoidance
           if self.current_scan:
               front_distance = self.get_state()[10]  # Get front distance from state
               if front_distance < 0.3:  # Too close to obstacle
                   reward -= 1.0
               elif front_distance < 0.5:  # Getting close
                   reward -= 0.1

           # Balance maintenance
           if self.current_imu:
               # Penalize excessive tilt
               roll = abs(self.current_imu.orientation.x)
               pitch = abs(self.current_imu.orientation.y)
               tilt_penalty = (roll + pitch) * 2.0
               reward -= tilt_penalty

           # Task completion bonus
           if distance_to_target < 0.3 and self.task_type == "navigation":
               reward += 10.0  # Significant bonus for reaching target
               self.task_completed = True

           # Small time penalty to encourage efficiency
           reward -= 0.01

           return reward

       def is_task_completed(self) -> bool:
           """Check if current task is completed"""
           if self.task_type == "navigation":
               distance = math.sqrt(
                   (self.target_position.x - self.current_pose.x)**2 +
                   (self.target_position.y - self.current_pose.y)**2
               )
               return distance < 0.3
           elif self.task_type == "manipulation":
               # Simplified manipulation completion check
               distance = math.sqrt(
                   (self.target_position.x - self.current_pose.x)**2 +
                   (self.target_position.y - self.current_pose.y)**2
               )
               return distance < 0.1  # Closer for manipulation

           return False

       def end_episode(self):
           """End current episode and reset"""
           self.episode_rewards.append(self.current_episode_reward)
           self.current_episode_reward = 0.0
           self.episode_steps = 0
           self.task_completed = False

           # Reset target position for next episode
           if self.grounded_objects:
               # Choose a new target object
               import random
               target_obj = random.choice(self.grounded_objects)
               self.target_position = target_obj['position']

           self.get_logger().info(f'Episode ended with reward: {self.current_episode_reward:.3f}')

       def update_model(self):
           """Update AI model using experience replay"""
           # Sample batch from replay buffer
           batch_indices = np.random.choice(len(self.replay_buffer), self.batch_size, replace=False)
           batch = [self.replay_buffer[i] for i in batch_indices]

           # Separate batch into components
           states = np.array([exp[0] for exp in batch])
           actions = np.array([exp[1] for exp in batch])
           rewards = np.array([exp[2] for exp in batch])
           next_states = np.array([exp[3] for exp in batch])
           dones = np.array([exp[4] for exp in batch])

           # Update critic (value function)
           next_actions = np.array([self.get_best_action(ns) for ns in next_states])
           next_q_values = np.array([np.dot(np.concatenate([ns, na]), self.critic_weights)[0]
                                   for ns, na in zip(next_states, next_actions)])

           target_q_values = rewards + self.discount_factor * next_q_values * (1 - dones.astype(int))

           # Simple gradient update for critic
           for i in range(self.batch_size):
               state_action = np.concatenate([states[i], actions[i]])
               current_q = np.dot(state_action, self.critic_weights)[0]
               td_error = target_q_values[i] - current_q
               self.critic_weights += self.learning_rate * td_error * state_action.reshape(-1, 1)

           # Update actor (policy)
           for i in range(self.batch_size):
               # Calculate policy gradient
               state = states[i]
               action = self.get_best_action(state)
               state_action = np.concatenate([state, action])

               # Get Q-value gradient with respect to action
               q_grad = self.critic_weights[len(state):]  # Gradient w.r.t. action

               # Update actor weights
               self.actor_weights += self.learning_rate * q_grad.reshape(1, -1) * state.reshape(-1, 1)

       def publish_ai_status(self, reward: float, action: np.ndarray):
           """Publish AI status information"""
           # Publish status message
           status_msg = String()
           status_msg.data = f'AI Active - Task: {self.task_type}, Reward: {reward:.3f}, Steps: {self.episode_steps}'
           self.ai_status_pub.publish(status_msg)

           # Publish learning statistics
           stats_msg = Float64MultiArray()
           stats_msg.data = [
               reward,
               self.epsilon,
               self.current_episode_reward,
               float(len(self.episode_rewards)),
               np.mean(self.episode_rewards[-10:]) if len(self.episode_rewards) >= 10 else 0.0,
               self.step_count
           ]
           self.learning_stats_pub.publish(stats_msg)


   def main(args=None):
       """Main function to initialize and run the AI controller"""
       rclpy.init(args=args)
       controller = AIController()

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

2. Make the script executable:
   ```bash
   chmod +x ai_control_system/ai_control_system/ai_controller.py
   ```

#### Step 3: Create AI Control Test Node
1. Create a test node `ai_control_system/ai_control_system/ai_control_test.py`:
   ```python
   #!/usr/bin/env python3
   """
   Test node for AI control system
   """
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String, Float64MultiArray
   from sensor_msgs.msg import JointState, Imu, LaserScan
   from geometry_msgs.msg import Point, Twist
   import numpy as np


   class AIControlTest(Node):
       def __init__(self):
           super().__init__('ai_control_test')

           # Publishers
           self.command_pub = self.create_publisher(String, '/natural_language_command', 10)
           self.grounded_objects_pub = self.create_publisher(Float64MultiArray, '/grounded_objects', 10)
           self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
           self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
           self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

           # Subscribers
           self.ai_status_sub = self.create_subscription(
               String,
               '/ai_status',
               self.ai_status_callback,
               10
           )

           # Timer for publishing test data
           self.test_timer = self.create_timer(0.1, self.publish_test_data)

           # Simulated data
           self.time_counter = 0.0
           self.robot_x = 0.0
           self.robot_y = 0.0
           self.robot_theta = 0.0

           # Test commands sequence
           self.test_commands = [
               "Navigate to the red cup",
               "Go to the blue bottle",
               "Move toward the yellow box",
               "Approach the green ball"
           ]
           self.command_index = 0
           self.command_sent = False

           # Create simulated environment
           self.create_simulated_environment()

           self.get_logger().info('AI Control Test node initialized')

       def create_simulated_environment(self):
           """Create simulated environment with objects"""
           # Simulated objects in the environment
           self.simulated_objects = [
               {'pos': Point(x=2.0, y=1.0, z=0.0), 'category': 4, 'color': 0},  # Red cup
               {'pos': Point(x=3.0, y=-1.0, z=0.0), 'category': 5, 'color': 1},  # Blue bottle
               {'pos': Point(x=1.0, y=2.0, z=0.0), 'category': 6, 'color': 3},  # Yellow box
               {'pos': Point(x=-1.0, y=-2.0, z=0.0), 'category': 7, 'color': 2},  # Green ball
           ]

       def publish_test_data(self):
           """Publish test data to simulate robot sensors and environment"""
           self.time_counter += 0.1

           # Publish grounded objects
           self.publish_grounded_objects()

           # Publish joint states
           self.publish_joint_states()

           # Publish IMU data
           self.publish_imu_data()

           # Publish laser scan
           self.publish_laser_scan()

           # Send command periodically
           if not self.command_sent and int(self.time_counter) % 20 == 0:
               self.send_test_command()

       def publish_grounded_objects(self):
           """Publish simulated grounded objects"""
           objects_msg = Float64MultiArray()
           objects_data = []

           for obj in self.simulated_objects:
               objects_data.extend([
                   obj['pos'].x, obj['pos'].y, obj['pos'].z,  # Position
                   0.9,  # Confidence
                   float(obj['category']),  # Category ID
                   float(obj['color'])  # Color ID
               ])

           objects_msg.data = objects_data
           self.grounded_objects_pub.publish(objects_msg)

       def publish_joint_states(self):
           """Publish simulated joint states"""
           joint_msg = JointState()
           joint_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

           # Simulate changing joint positions
           positions = []
           for i in range(6):
               pos = np.sin(self.time_counter + i) * 0.5
               positions.append(pos)

           joint_msg.position = positions
           joint_msg.velocity = [0.0] * 6
           joint_msg.effort = [0.0] * 6

           self.joint_state_pub.publish(joint_msg)

       def publish_imu_data(self):
           """Publish simulated IMU data"""
           imu_msg = Imu()

           # Simulate slight tilting
           imu_msg.orientation.x = np.sin(self.time_counter * 0.1) * 0.01
           imu_msg.orientation.y = np.cos(self.time_counter * 0.1) * 0.01
           imu_msg.orientation.z = 0.0
           imu_msg.orientation.w = 1.0

           # Angular velocity
           imu_msg.angular_velocity.x = np.sin(self.time_counter * 0.2) * 0.01
           imu_msg.angular_velocity.y = np.cos(self.time_counter * 0.2) * 0.01
           imu_msg.angular_velocity.z = 0.01

           # Linear acceleration
           imu_msg.linear_acceleration.x = np.sin(self.time_counter * 0.3) * 0.1
           imu_msg.linear_acceleration.y = np.cos(self.time_counter * 0.3) * 0.1
           imu_msg.linear_acceleration.z = 9.81  # Gravity

           self.imu_pub.publish(imu_msg)

       def publish_laser_scan(self):
           """Publish simulated laser scan"""
           scan_msg = LaserScan()

           # Set scan parameters
           scan_msg.angle_min = -np.pi / 2
           scan_msg.angle_max = np.pi / 2
           scan_msg.angle_increment = np.pi / 180  # 1 degree
           scan_msg.time_increment = 0.0
           scan_msg.scan_time = 0.1
           scan_msg.range_min = 0.1
           scan_msg.range_max = 10.0

           # Generate ranges with some obstacles
           num_ranges = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
           ranges = []

           for i in range(num_ranges):
               angle = scan_msg.angle_min + i * scan_msg.angle_increment

               # Add some simulated obstacles
               distance = 5.0  # Default far distance

               # Add a few obstacles at specific angles
               if abs(angle - 0.0) < 0.2:  # Front
                   distance = 2.0 + np.sin(self.time_counter) * 0.5
               elif abs(angle - np.pi/4) < 0.1:  # Front-right
                   distance = 3.0
               elif abs(angle + np.pi/4) < 0.1:  # Front-left
                   distance = 2.5

               ranges.append(distance)

           scan_msg.ranges = ranges
           scan_msg.intensities = [100.0] * len(ranges)  # Dummy intensities

           self.scan_pub.publish(scan_msg)

       def send_test_command(self):
           """Send test command to AI system"""
           if not self.command_sent:
               command_msg = String()
               command_msg.data = self.test_commands[self.command_index]
               self.command_pub.publish(command_msg)

               self.get_logger().info(f'Sent command: {command_msg.data}')
               self.command_sent = True
               self.command_index = (self.command_index + 1) % len(self.test_commands)

       def ai_status_callback(self, msg):
           """Handle AI status updates"""
           self.get_logger().info(f'AI Status: {msg.data}')
           self.command_sent = False  # Allow next command after some time


   def main(args=None):
       """Main function to initialize and run the test node"""
       rclpy.init(args=args)
       test_node = AIControlTest()

       try:
           rclpy.spin(test_node)
       except KeyboardInterrupt:
           pass
       finally:
           test_node.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Make the test script executable:
   ```bash
   chmod +x ai_control_system/ai_control_system/ai_control_test.py
   ```

#### Step 4: Create AI Control Launch File
1. Create a launch directory:
   ```bash
   mkdir -p ai_control_system/launch
   ```

2. Create a launch file `ai_control_system/launch/ai_control_demo.launch.py`:
   ```python
   #!/usr/bin/env python3
   """
   Launch file for AI control system demo
   """
   from launch import LaunchDescription
   from launch_ros.actions import Node


   def generate_launch_description():
       # AI controller node
       ai_controller = Node(
           package='ai_control_system',
           executable='ai_controller',
           name='ai_controller',
           output='screen',
           parameters=[
               {'use_sim_time': False}
           ]
       )

       # Test node
       ai_test = Node(
           package='ai_control_system',
           executable='ai_control_test',
           name='ai_control_test',
           output='screen',
           parameters=[
               {'use_sim_time': False}
           ]
       )

       return LaunchDescription([
           ai_controller,
           ai_test
       ])
   ```

#### Step 5: Update AI Control Setup Files
1. Update `setup.py` for the AI control package:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   from setuptools import find_packages

   package_name = 'ai_control_system'

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
       description='AI-integrated control for Physical AI systems',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'ai_controller = ai_control_system.ai_controller:main',
               'ai_control_test = ai_control_system.ai_control_test:main',
           ],
       },
   )
   ```

#### Step 6: Build and Test AI Control System
1. Build the package:
   ```bash
   cd ~/humanoid_ws
   colcon build --packages-select ai_control_system
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Run the AI control demo:
   ```bash
   ros2 launch ai_control_system ai_control_demo.launch.py
   ```

4. In another terminal, check the published topics:
   ```bash
   ros2 topic list | grep ai
   ros2 topic echo /ai_status
   ros2 topic echo /learning_stats
   ```

### Expected Results
- AI controller learns to navigate to targets
- Learning statistics show improvement over time
- Robot successfully reaches specified objects
- AI adapts behavior based on environmental feedback
- Reward function guides learning toward successful outcomes

### Analysis Questions
1. How does the AI controller adapt its behavior over time?
2. What are the limitations of the simple neural network approach used?
3. How would you improve the reward function for better learning?
4. What challenges arise when deploying this to a real robot?

---

## Quiz 5.4: Simulation Integration and Validation

### Question 1
What is the main purpose of the Gazebo transport layer?

A) To store robot models
B) To handle communication between server and clients
C) To render 3D graphics
D) To control robot joints

**Correct Answer: B**

### Question 2
Which physics engine is NOT supported by Gazebo?

A) ODE
B) Bullet
C) PhysX
D) DART

**Correct Answer: C**

### Question 3
What does SDF stand for in Gazebo?

A) Simulation Description Format
B) Simple Definition Format
C) System Description Framework
D) Sensor Data Format

**Correct Answer: A**

### Question 4
What is the purpose of the ZMP (Zero Moment Point) in humanoid robotics?

A) To control camera focus
B) To ensure dynamic balance during locomotion
C) To measure sensor accuracy
D) To control joint velocities

**Correct Answer: B**

### Question 5
What is domain randomization used for in simulation?

A) To make the simulation run faster
B) To reduce the reality gap between simulation and real-world performance
C) To decrease the quality of graphics
D) To limit the number of objects in the simulation

**Correct Answer: B**

### Question 6
What is the main advantage of using URDF over SDF for robot description?

A) URDF is more visually appealing
B) URDF is better integrated with ROS
C) URDF supports more complex physics
D) URDF has better rendering capabilities

**Correct Answer: B**

### Question 7
What is the typical update rate for IMU sensors in simulation?

A) 1 Hz
B) 10 Hz
C) 100 Hz
D) 1000 Hz

**Correct Answer: C**

### Question 8
What is the purpose of the ros_gz_bridge package?

A) To bridge ROS 1 and ROS 2
B) To bridge ROS and Gazebo Harmonic
C) To bridge different robot models
D) To bridge sensor data formats

**Correct Answer: B**

---

## Quiz 5.5: Safety and Human-Robot Interaction

### Question 1
What is the primary safety concern in Physical AI systems?

A) Data privacy
B) Physical harm to humans or environment
C) Network security
D) Sensor calibration

**Correct Answer: B**

### Question 2
What does "fail-safe" mean in robotics?

A) The system fails to work
B) The system defaults to a safe state when problems occur
C) The system stops working permanently
D) The system requires manual restart

**Correct Answer: B**

### Question 3
What is the purpose of an emergency stop system?

A) To save power
B) To immediately halt robot operation in dangerous situations
C) To calibrate sensors
D) To update software

**Correct Answer: B**

### Question 4
What is the "safety envelope" in robotics?

A) The area where the robot can operate safely
B) The physical casing of the robot
C) The software safety protocols
D) The network security measures

**Correct Answer: A**

### Question 5
What is the typical response time requirement for safety systems?

A) 1 second
B) 100 milliseconds
C) 10 milliseconds
D) 1 millisecond

**Correct Answer: C**

### Question 6
What is "safety-rated" in robotics?

A) A marketing rating
B) Certified to meet specific safety standards
C) A color coding system
D) A performance metric

**Correct Answer: B**

### Question 7
What is the purpose of safety-rated monitoring?

A) To monitor robot performance
B) To continuously check for safety violations
C) To track productivity
D) To monitor battery levels

**Correct Answer: B**

### Question 8
What does ISO 10218 specify?

A) Industrial robot safety requirements
B) Medical device standards
C) Automotive safety standards
D) Consumer electronics standards

**Correct Answer: A**

---

## Quiz 5.6: Performance Optimization

### Question 1
What is the main benefit of using GPU acceleration in Physical AI?

A) Cheaper hardware
B) Faster processing of vision and AI tasks
C) Simpler programming
D) Better networking

**Correct Answer: B**

### Question 2
What does "real-time factor" measure in simulation?

A) How fast the robot moves
B) The ratio of simulation time to real time
C) The accuracy of sensors
D) The number of objects in the simulation

**Correct Answer: B**

### Question 3
What is the purpose of multi-threading in robot control?

A) To use multiple CPUs
B) To handle multiple tasks concurrently
C) To increase memory usage
D) To improve graphics

**Correct Answer: B**

### Question 4
What is "deterministic execution" in robotics?

A) Execution that varies randomly
B) Execution with predictable timing and behavior
C) Execution that is always the same
D) Execution that is very fast

**Correct Answer: B**

### Question 5
What is the typical control frequency for high-performance robot control?

A) 10 Hz
B) 50 Hz
C) 100 Hz
D) 1 kHz

**Correct Answer: D**

### Question 6
What is "latency" in robot control?

A) The time to execute a command
B) The delay between command and response
C) The processing speed
D) The memory usage

**Correct Answer: B**

### Question 7
What is "bandwidth optimization" in robotics?

A) Reducing network costs
B) Optimizing data transmission efficiency
C) Improving graphics quality
D) Reducing power consumption

**Correct Answer: B**

### Question 8
What is "resource contention" in multi-robot systems?

A) Competition for the same physical space
B) Competition for shared computational resources
C) Competition for the same tasks
D) Competition for the same sensors

**Correct Answer: B**

---

## Answers Summary

### Quiz 5.1 Answers:
1. B
2. B
3. C
4. D
5. B
6. A
7. C
8. C

### Quiz 5.2 Answers:
1. C
2. B
3. B
4. B

### Quiz 5.3 Answers:
1. A
2. B
3. B
4. C
5. C
6. B
7. B
8. A

### Quiz 5.4 Answers:
1. B
2. C
3. A
4. B
5. B
6. B
7. C
8. B

### Quiz 5.5 Answers:
1. B
2. B
3. B
4. A
5. C
6. B
7. B
8. A

### Quiz 5.6 Answers:
1. B
2. B
3. B
4. B
5. D
6. B
7. B
8. B