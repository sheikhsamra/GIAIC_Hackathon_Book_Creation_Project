# Code Examples: Capstone Physical AI Project

## Overview

This module provides comprehensive code examples for implementing a complete Physical AI system. The examples demonstrate integration of perception, reasoning, action, and safety systems in a cohesive application.

## 1. System Architecture and Component Integration

### 1.1 Main System Orchestrator

```python
#!/usr/bin/env python3
"""
Physical AI System Orchestrator
Integrates all major components into a unified system
"""

import rospy
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
from typing import Dict, List, Optional, Tuple
import threading
import time
from dataclasses import dataclass
from enum import Enum

class SystemState(Enum):
    """System operational states"""
    INITIALIZING = "initializing"
    IDLE = "idle"
    PERCEIVING = "perceiving"
    PLANNING = "planning"
    EXECUTING = "executing"
    SAFETY_EMERGENCY = "safety_emergency"
    SHUTDOWN = "shutdown"

@dataclass
class SystemStatus:
    """Comprehensive system status information"""
    state: SystemState
    timestamp: float
    safety_status: Dict[str, bool]
    component_health: Dict[str, bool]
    current_task: Optional[str]
    performance_metrics: Dict[str, float]

class PhysicalAISystem:
    """Main orchestrator for the Physical AI system"""

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('physical_ai_system', anonymous=True)

        # System state management
        self.current_state = SystemState.INITIALIZING
        self.system_status = SystemStatus(
            state=SystemState.INITIALIZING,
            timestamp=time.time(),
            safety_status={},
            component_health={},
            current_task=None,
            performance_metrics={}
        )

        # Publishers for system communication
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.system_state_pub = rospy.Publisher('/system_state', String, queue_size=10)
        self.safety_alert_pub = rospy.Publisher('/safety_alert', String, queue_size=10)

        # Subscribers for sensor data
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.pointcloud_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.pointcloud_callback)
        self.safety_stop_sub = rospy.Subscriber('/emergency_stop', Bool, self.emergency_stop_callback)

        # Component instances (will be initialized after safety check)
        self.perception_system = None
        self.language_system = None
        self.planning_system = None
        self.control_system = None

        # Threading for concurrent operations
        self.system_thread = None
        self.safety_monitor_thread = None

        # Initialize components
        self.initialize_components()

        # Start system threads
        self.start_system_threads()

    def initialize_components(self):
        """Initialize all system components"""
        try:
            # Initialize perception system
            self.perception_system = PerceptionSystem()
            self.system_status.component_health['perception'] = True

            # Initialize language understanding system
            self.language_system = LanguageSystem()
            self.system_status.component_health['language'] = True

            # Initialize planning system
            self.planning_system = PlanningSystem()
            self.system_status.component_health['planning'] = True

            # Initialize control system
            self.control_system = ControlSystem()
            self.system_status.component_health['control'] = True

            # Initialize safety system
            self.safety_system = SafetySystem()
            self.system_status.component_health['safety'] = True

            rospy.loginfo("All components initialized successfully")
            self.system_status.state = SystemState.IDLE

        except Exception as e:
            rospy.logerr(f"Component initialization failed: {e}")
            self.system_status.state = SystemState.SAFETY_EMERGENCY

    def start_system_threads(self):
        """Start system operation threads"""
        self.system_thread = threading.Thread(target=self.main_system_loop)
        self.system_thread.daemon = True
        self.system_thread.start()

        self.safety_monitor_thread = threading.Thread(target=self.safety_monitor_loop)
        self.safety_monitor_thread.daemon = True
        self.safety_monitor_thread.start()

    def main_system_loop(self):
        """Main system operation loop"""
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown() and self.system_status.state != SystemState.SHUTDOWN:
            try:
                # Update system status
                self.system_status.timestamp = time.time()

                # Based on current state, execute appropriate behavior
                if self.system_status.state == SystemState.IDLE:
                    self.idle_behavior()
                elif self.system_status.state == SystemState.PERCEIVING:
                    self.perception_behavior()
                elif self.system_status.state == SystemState.PLANNING:
                    self.planning_behavior()
                elif self.system_status.state == SystemState.EXECUTING:
                    self.execution_behavior()
                elif self.system_status.state == SystemState.SAFETY_EMERGENCY:
                    self.safety_emergency_behavior()

                # Publish system state
                state_msg = String()
                state_msg.data = self.system_status.state.value
                self.system_state_pub.publish(state_msg)

                rate.sleep()

            except Exception as e:
                rospy.logerr(f"Error in main system loop: {e}")
                self.system_status.state = SystemState.SAFETY_EMERGENCY

    def safety_monitor_loop(self):
        """Continuous safety monitoring"""
        rate = rospy.Rate(50)  # 50 Hz for safety monitoring

        while not rospy.is_shutdown():
            try:
                # Check safety conditions
                safety_status = self.safety_system.check_safety_conditions()
                self.system_status.safety_status.update(safety_status)

                # Trigger emergency if needed
                if not safety_status.get('safe_operation', True):
                    self.system_status.state = SystemState.SAFETY_EMERGENCY
                    alert_msg = String()
                    alert_msg.data = "Safety violation detected"
                    self.safety_alert_pub.publish(alert_msg)

                rate.sleep()

            except Exception as e:
                rospy.logerr(f"Error in safety monitor: {e}")
                self.system_status.state = SystemState.SAFETY_EMERGENCY

    def idle_behavior(self):
        """Behavior when system is idle"""
        # In idle state, system listens for commands and maintains awareness
        rospy.logdebug("System in IDLE state")

        # Keep perception system running for environmental awareness
        if self.perception_system:
            self.perception_system.update_environment_map()

    def perception_behavior(self):
        """Behavior during perception phase"""
        rospy.logdebug("System in PERCEIVING state")

        if self.perception_system:
            # Perform comprehensive environmental perception
            perception_result = self.perception_system.process_sensory_data()
            self.system_status.performance_metrics['perception_fps'] = perception_result.fps

    def planning_behavior(self):
        """Behavior during planning phase"""
        rospy.logdebug("System in PLANNING state")

        if self.planning_system and self.perception_system:
            # Use perception data to generate plans
            env_data = self.perception_system.get_environment_data()
            plan = self.planning_system.generate_plan(env_data)

            # Validate plan safety before execution
            if self.safety_system.validate_plan(plan):
                self.system_status.state = SystemState.EXECUTING
            else:
                rospy.logwarn("Plan failed safety validation")
                self.system_status.state = SystemState.IDLE

    def execution_behavior(self):
        """Behavior during execution phase"""
        rospy.logdebug("System in EXECUTING state")

        if self.control_system:
            # Execute planned actions
            execution_result = self.control_system.execute_current_plan()

            if execution_result.completed:
                self.system_status.state = SystemState.IDLE
            elif execution_result.failed:
                self.system_status.state = SystemState.IDLE

    def safety_emergency_behavior(self):
        """Behavior during safety emergency"""
        rospy.logwarn("Safety emergency - stopping all operations")

        # Stop all motion
        if self.control_system:
            self.control_system.emergency_stop()

        # Clear any active plans
        if self.planning_system:
            self.planning_system.clear_active_plan()

        # Maintain safety monitoring
        time.sleep(1)  # Brief pause before allowing recovery

    def image_callback(self, data):
        """Handle incoming image data"""
        if self.perception_system and self.system_status.state != SystemState.SAFETY_EMERGENCY:
            self.perception_system.process_image_data(data)

    def pointcloud_callback(self, data):
        """Handle incoming point cloud data"""
        if self.perception_system and self.system_status.state != SystemState.SAFETY_EMERGENCY:
            self.perception_system.process_pointcloud_data(data)

    def emergency_stop_callback(self, data):
        """Handle emergency stop commands"""
        if data.data:
            rospy.logwarn("Emergency stop command received")
            self.system_status.state = SystemState.SAFETY_EMERGENCY

    def execute_command(self, command: str) -> bool:
        """Execute a high-level command"""
        try:
            if self.system_status.state == SystemState.SAFETY_EMERGENCY:
                rospy.logerr("Cannot execute command during safety emergency")
                return False

            # Parse and understand the command
            if self.language_system:
                parsed_command = self.language_system.parse_command(command)

                if parsed_command is not None:
                    self.system_status.current_task = command

                    # Move to planning state
                    self.system_status.state = SystemState.PLANNING
                    return True
                else:
                    rospy.logerr(f"Could not parse command: {command}")
                    return False
            else:
                rospy.logerr("Language system not initialized")
                return False

        except Exception as e:
            rospy.logerr(f"Error executing command: {e}")
            self.system_status.state = SystemState.SAFETY_EMERGENCY
            return False

if __name__ == '__main__':
    try:
        ai_system = PhysicalAISystem()
        rospy.loginfo("Physical AI System initialized successfully")

        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Physical AI System shutting down")
        if ai_system:
            ai_system.system_status.state = SystemState.SHUTDOWN
```

### 1.2 Perception System

```python
#!/usr/bin/env python3
"""
Perception System for Physical AI
Handles visual and spatial perception tasks
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import open3d as o3d
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
import threading
import time

@dataclass
class ObjectDetection:
    """Represents a detected object"""
    label: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x, y, width, height
    position_3d: Tuple[float, float, float]  # x, y, z in world coordinates
    size_3d: Tuple[float, float, float]  # width, height, depth

@dataclass
class PerceptionResult:
    """Result of perception processing"""
    objects: List[ObjectDetection]
    environment_map: np.ndarray
    fps: float
    timestamp: float

class PerceptionSystem:
    """Handles all perception tasks in the Physical AI system"""

    def __init__(self):
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()

        # Storage for sensor data
        self.latest_image = None
        self.latest_pointcloud = None

        # Object detection model (simplified - in practice would use YOLO, etc.)
        self.detection_model = self._load_detection_model()

        # Environment mapping
        self.environment_map = np.zeros((100, 100, 3), dtype=np.uint8)  # 100x100 grid
        self.map_lock = threading.Lock()

        # Processing statistics
        self.processing_times = []
        self.frame_count = 0
        self.last_process_time = time.time()

    def _load_detection_model(self):
        """Load the object detection model"""
        # In practice, this would load a pre-trained model like YOLO, SSD, etc.
        # For this example, we'll simulate detection
        rospy.loginfo("Loading object detection model")
        return "dummy_model"  # Placeholder

    def process_image_data(self, image_msg: Image):
        """Process incoming image data"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            self.latest_image = cv_image
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")

    def process_pointcloud_data(self, pointcloud_msg: PointCloud2):
        """Process incoming point cloud data"""
        # In practice, this would convert and process the point cloud
        # For this example, we'll store the message
        self.latest_pointcloud = pointcloud_msg

    def get_environment_data(self) -> Dict:
        """Get current environment data"""
        with self.map_lock:
            return {
                'map': self.environment_map.copy(),
                'objects': self.get_detected_objects(),
                'timestamp': time.time()
            }

    def get_detected_objects(self) -> List[ObjectDetection]:
        """Get currently detected objects"""
        # In practice, this would return the results of the latest detection
        # For this example, we'll simulate some objects
        objects = []

        # Simulate some detected objects
        if self.latest_image is not None:
            # Example: simulate detection of a table and a chair
            objects.append(ObjectDetection(
                label="table",
                confidence=0.95,
                bbox=(100, 200, 300, 150),
                position_3d=(1.0, 0.0, 0.0),
                size_3d=(1.2, 0.8, 0.75)
            ))

            objects.append(ObjectDetection(
                label="chair",
                confidence=0.88,
                bbox=(250, 300, 150, 180),
                position_3d=(1.5, 0.5, 0.0),
                size_3d=(0.5, 0.5, 0.8)
            ))

        return objects

    def process_sensory_data(self) -> PerceptionResult:
        """Process all available sensory data"""
        start_time = time.time()

        objects = self.get_detected_objects()

        # Update processing statistics
        current_time = time.time()
        self.processing_times.append(current_time - start_time)

        # Calculate FPS
        self.frame_count += 1
        if current_time - self.last_process_time >= 1.0:  # Every second
            fps = self.frame_count / (current_time - self.last_process_time)
            self.frame_count = 0
            self.last_process_time = current_time
        else:
            fps = 0  # Will be updated on next second boundary

        # In practice, this would update the environment map with detected objects
        with self.map_lock:
            # Update map based on detections
            self.update_environment_map_with_detections(objects)

        result = PerceptionResult(
            objects=objects,
            environment_map=self.environment_map.copy(),
            fps=fps,
            timestamp=current_time
        )

        return result

    def update_environment_map_with_detections(self, objects: List[ObjectDetection]):
        """Update the environment map with detected objects"""
        # Reset map
        self.environment_map.fill(0)

        # Draw detected objects on the map
        for obj in objects:
            # Convert 3D position to 2D map coordinates (simplified)
            x, y, z = obj.position_3d
            map_x = int((x + 5) * 10)  # Scale factor to fit in 100x100 map
            map_y = int((y + 5) * 10)  # Scale factor to fit in 100x100 map

            if 0 <= map_x < 100 and 0 <= map_y < 100:
                # Draw object as a colored square
                color = self._get_color_for_label(obj.label)
                size = max(1, int(obj.size_3d[0] * 10))  # Scale to map

                # Draw filled rectangle
                cv2.rectangle(
                    self.environment_map,
                    (map_x - size//2, map_y - size//2),
                    (map_x + size//2, map_y + size//2),
                    color,
                    -1
                )

    def _get_color_for_label(self, label: str) -> Tuple[int, int, int]:
        """Get color for object label"""
        color_map = {
            "table": (0, 255, 0),      # Green
            "chair": (0, 0, 255),      # Red
            "person": (255, 0, 0),     # Blue
            "cup": (255, 255, 0),      # Cyan
            "book": (255, 0, 255),     # Magenta
        }
        return color_map.get(label, (128, 128, 128))  # Default gray

    def update_environment_map(self):
        """Update environment map for continuous awareness"""
        # In practice, this would continuously update the map
        # For this example, we'll just keep it as a placeholder
        pass
```

### 1.3 Language Understanding System

```python
#!/usr/bin/env python3
"""
Language Understanding System for Physical AI
Handles natural language command interpretation
"""

import rospy
import re
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
from enum import Enum
import spacy

class ActionType(Enum):
    """Types of actions that can be performed"""
    MOVE_TO = "move_to"
    GRASP = "grasp"
    PLACE = "place"
    FOLLOW = "follow"
    STOP = "stop"
    GREET = "greet"
    REPORT = "report"
    NAVIGATE = "navigate"

@dataclass
class Command:
    """Represents a parsed command"""
    action: ActionType
    target_object: Optional[str] = None
    target_location: Optional[str] = None
    person_name: Optional[str] = None
    confidence: float = 0.0

class LanguageSystem:
    """Handles natural language understanding in the Physical AI system"""

    def __init__(self):
        # Load spaCy model for NLP processing
        try:
            self.nlp = spacy.load("en_core_web_sm")
        except OSError:
            rospy.logwarn("spaCy English model not found. Using basic pattern matching.")
            self.nlp = None

        # Define command patterns
        self.command_patterns = {
            ActionType.MOVE_TO: [
                r'go to the (.+)',
                r'move to the (.+)',
                r'go to (.+)',
                r'head to the (.+)',
                r'head to (.+)'
            ],
            ActionType.GRASP: [
                r'pick up the (.+)',
                r'grasp the (.+)',
                r'get the (.+)',
                r'take the (.+)',
                r'grab the (.+)'
            ],
            ActionType.PLACE: [
                r'place it on the (.+)',
                r'put it on the (.+)',
                r'place the (.+) on',
                r'put the (.+) on'
            ],
            ActionType.FOLLOW: [
                r'follow (.+)',
                r'follow me',
                r'come with me'
            ],
            ActionType.STOP: [
                r'stop',
                r'freeze',
                r'hold',
                r'wait'
            ],
            ActionType.GREET: [
                r'greet (.+)',
                r'say hello to (.+)',
                r'hello (.+)'
            ],
            ActionType.REPORT: [
                r'tell me about (.+)',
                r'report on (.+)',
                r'what do you see',
                r'what is around you'
            ],
            ActionType.NAVIGATE: [
                r'go (.+)',
                r'navigate to (.+)',
                r'explore (.+)'
            ]
        }

        # Define location keywords
        self.location_keywords = {
            'kitchen': ['kitchen', 'counter', 'fridge', 'microwave', 'sink'],
            'living_room': ['living room', 'couch', 'sofa', 'tv', 'coffee table'],
            'bedroom': ['bedroom', 'bed', 'dresser', 'nightstand'],
            'office': ['office', 'desk', 'computer', 'chair'],
            'dining_room': ['dining room', 'dining table', 'table', 'dinner table']
        }

        # Object categories
        self.object_categories = {
            'furniture': ['table', 'chair', 'couch', 'bed', 'desk'],
            'kitchen_items': ['cup', 'plate', 'bowl', 'fork', 'spoon', 'knife'],
            'electronics': ['phone', 'tablet', 'computer', 'tv', 'laptop'],
            'personal_items': ['book', 'keys', 'wallet', 'glasses', 'hat']
        }

    def parse_command(self, command_text: str) -> Optional[Command]:
        """Parse a natural language command into structured action"""
        command_text = command_text.lower().strip()

        # Try to match command patterns
        for action_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, command_text)
                if match:
                    # Extract the target from the match
                    target = match.group(1).strip() if len(match.groups()) > 0 else None

                    # Determine if target is an object or location
                    target_object = self._extract_object(target) if target else None
                    target_location = self._extract_location(target) if target else None
                    person_name = self._extract_person_name(command_text)

                    # Calculate confidence based on match strength
                    confidence = self._calculate_confidence(action_type, target)

                    return Command(
                        action=action_type,
                        target_object=target_object,
                        target_location=target_location,
                        person_name=person_name,
                        confidence=confidence
                    )

        # If no pattern matched, return None
        return None

    def _extract_object(self, text: str) -> Optional[str]:
        """Extract object from text"""
        if not text:
            return None

        # Check if text contains known objects
        for category, objects in self.object_categories.items():
            for obj in objects:
                if obj in text:
                    return obj

        # If using spaCy, try to extract noun phrases
        if self.nlp:
            doc = self.nlp(text)
            for token in doc:
                if token.pos_ == "NOUN":
                    return token.text

        # Return the full text as object if no specific object identified
        return text

    def _extract_location(self, text: str) -> Optional[str]:
        """Extract location from text"""
        if not text:
            return None

        # Check if text contains known locations
        for room, locations in self.location_keywords.items():
            for loc in locations:
                if loc in text:
                    return loc

        return None

    def _extract_person_name(self, text: str) -> Optional[str]:
        """Extract person name from text"""
        # Look for common greeting patterns that might contain names
        greeting_patterns = [
            r'greet (\w+)',
            r'hello (\w+)',
            r'hi (\w+)',
            r'hey (\w+)'
        ]

        for pattern in greeting_patterns:
            match = re.search(pattern, text)
            if match:
                name = match.group(1)
                # Validate that it's a proper name (basic check)
                if len(name) > 1 and name[0].isupper():
                    return name

        return None

    def _calculate_confidence(self, action_type: ActionType, target: Optional[str]) -> float:
        """Calculate confidence score for the parsed command"""
        base_confidence = 0.8  # Base confidence for pattern match

        # Increase confidence if target is identified
        if target:
            # Check if target is in known categories
            for category, items in {**self.object_categories, **self.location_keywords}.items():
                if target in items:
                    return base_confidence + 0.15  # High confidence for known items

            # If using spaCy and it's a noun
            if self.nlp:
                doc = self.nlp(target)
                if any(token.pos_ == "NOUN" for token in doc):
                    return base_confidence + 0.1  # Good confidence for noun

        return base_confidence

    def interpret_context(self, command: Command, context: Dict) -> Command:
        """Interpet command in context of current situation"""
        # In practice, this would use context to disambiguate commands
        # For example, if "it" is mentioned, find the most recent object

        # If the command refers to a general location like "there",
        # use context to determine actual location
        if command.target_location == "there" and context.get('recent_locations'):
            command.target_location = context['recent_locations'][-1]

        # If the command refers to an object like "it",
        # use context to determine the specific object
        if command.target_object == "it" and context.get('recent_objects'):
            command.target_object = context['recent_objects'][-1]

        return command

# Example usage and testing
if __name__ == '__main__':
    # Initialize ROS node for testing
    rospy.init_node('language_system_test', anonymous=True)

    lang_system = LanguageSystem()

    # Test commands
    test_commands = [
        "Go to the kitchen",
        "Pick up the red cup",
        "Greet John",
        "Tell me about the environment",
        "Place the book on the table"
    ]

    for cmd_text in test_commands:
        parsed = lang_system.parse_command(cmd_text)
        if parsed:
            rospy.loginfo(f"Command: '{cmd_text}' -> Action: {parsed.action}, "
                         f"Target: {parsed.target_object}, Location: {parsed.target_location}, "
                         f"Confidence: {parsed.confidence:.2f}")
        else:
            rospy.logwarn(f"Could not parse command: {cmd_text}")
```

### 1.4 Planning System

```python
#!/usr/bin/env python3
"""
Planning System for Physical AI
Handles task planning and motion planning
"""

import rospy
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
from enum import Enum
import heapq
import math

class TaskType(Enum):
    """Types of tasks that can be planned"""
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    INTERACTION = "interaction"
    SURVEILLANCE = "surveillance"
    TRANSPORT = "transport"

@dataclass
class Task:
    """Represents a high-level task"""
    task_type: TaskType
    target_location: Optional[Tuple[float, float, float]] = None
    target_object: Optional[str] = None
    description: str = ""
    priority: int = 1  # Lower number = higher priority

@dataclass
class PlanStep:
    """Represents a single step in a plan"""
    action: str
    parameters: Dict
    expected_duration: float  # in seconds
    preconditions: List[str]
    effects: List[str]

@dataclass
class MotionPlan:
    """Represents a motion plan with waypoints"""
    waypoints: List[Tuple[float, float, float]]  # x, y, theta
    total_distance: float
    estimated_time: float

class PlanningSystem:
    """Handles planning in the Physical AI system"""

    def __init__(self):
        self.current_plan = []
        self.active_task = None
        self.environment_map = None
        self.robot_pose = (0.0, 0.0, 0.0)  # x, y, theta

    def generate_plan(self, env_data: Dict) -> Optional[List[PlanStep]]:
        """Generate a plan based on environment data and current task"""
        if not self.active_task:
            rospy.logwarn("No active task to plan for")
            return None

        self.environment_map = env_data.get('map')

        if self.active_task.task_type == TaskType.NAVIGATION:
            return self._generate_navigation_plan()
        elif self.active_task.task_type == TaskType.MANIPULATION:
            return self._generate_manipulation_plan()
        elif self.active_task.task_type == TaskType.INTERACTION:
            return self._generate_interaction_plan()
        elif self.active_task.task_type == TaskType.TRANSPORT:
            return self._generate_transport_plan()
        else:
            rospy.logwarn(f"Unknown task type: {self.active_task.task_type}")
            return None

    def _generate_navigation_plan(self) -> List[PlanStep]:
        """Generate navigation plan to reach target location"""
        if not self.active_task.target_location:
            rospy.logwarn("No target location for navigation task")
            return []

        # Generate motion plan using A* or similar algorithm
        motion_plan = self._plan_motion_to_target(self.active_task.target_location)

        if not motion_plan:
            rospy.logwarn("Could not generate motion plan")
            return []

        # Convert motion plan to plan steps
        plan_steps = []

        # Add approach step
        plan_steps.append(PlanStep(
            action="move_to",
            parameters={
                "waypoints": motion_plan.waypoints,
                "target_location": self.active_task.target_location
            },
            expected_duration=motion_plan.estimated_time,
            preconditions=["robot_operational", "path_clear"],
            effects=["robot_at_target", "path_traversed"]
        ))

        return plan_steps

    def _generate_manipulation_plan(self) -> List[PlanStep]:
        """Generate manipulation plan for object interaction"""
        if not self.active_task.target_object:
            rospy.logwarn("No target object for manipulation task")
            return []

        plan_steps = []

        # 1. Navigate to object
        object_location = self._find_object_location(self.active_task.target_object)
        if object_location:
            navigate_plan = self._plan_motion_to_target(object_location)
            if navigate_plan:
                plan_steps.append(PlanStep(
                    action="navigate_to_object",
                    parameters={
                        "object_name": self.active_task.target_object,
                        "target_location": object_location
                    },
                    expected_duration=navigate_plan.estimated_time,
                    preconditions=["robot_operational", "object_visible"],
                    effects=["robot_near_object"]
                ))

        # 2. Grasp object
        plan_steps.append(PlanStep(
            action="grasp_object",
            parameters={"object_name": self.active_task.target_object},
            expected_duration=5.0,  # 5 seconds for grasping
            preconditions=["robot_near_object", "object_graspable"],
            effects=["object_grasped", "gripper_occupied"]
        ))

        # 3. Verify grasp
        plan_steps.append(PlanStep(
            action="verify_grasp",
            parameters={"object_name": self.active_task.target_object},
            expected_duration=1.0,
            preconditions=["object_grasped"],
            effects=["grasp_verified"]
        ))

        return plan_steps

    def _generate_interaction_plan(self) -> List[PlanStep]:
        """Generate interaction plan for human interaction"""
        plan_steps = []

        # 1. Approach person
        person_location = self._find_person_location()
        if person_location:
            motion_plan = self._plan_motion_to_target(person_location)
            if motion_plan:
                plan_steps.append(PlanStep(
                    action="approach_person",
                    parameters={"target_location": person_location},
                    expected_duration=motion_plan.estimated_time,
                    preconditions=["person_detected", "path_clear"],
                    effects=["near_person"]
                ))

        # 2. Perform interaction
        plan_steps.append(PlanStep(
            action="perform_interaction",
            parameters={"interaction_type": "greeting"},
            expected_duration=10.0,
            preconditions=["near_person"],
            effects=["interaction_completed"]
        ))

        return plan_steps

    def _generate_transport_plan(self) -> List[PlanStep]:
        """Generate transport plan for moving objects"""
        plan_steps = []

        # 1. Navigate to pickup location
        if self.active_task.target_object:
            object_location = self._find_object_location(self.active_task.target_object)
            if object_location:
                navigate_plan = self._plan_motion_to_target(object_location)
                if navigate_plan:
                    plan_steps.append(PlanStep(
                        action="navigate_to_pickup",
                        parameters={"target_location": object_location},
                        expected_duration=navigate_plan.estimated_time,
                        preconditions=["robot_operational", "object_location_known"],
                        effects=["at_pickup_location"]
                    ))

        # 2. Pick up object
        plan_steps.append(PlanStep(
            action="pickup_object",
            parameters={"object_name": self.active_task.target_object},
            expected_duration=5.0,
            preconditions=["at_pickup_location", "object_accessible"],
            effects=["object_picked_up", "gripper_occupied"]
        ))

        # 3. Navigate to destination
        if self.active_task.target_location:
            transport_plan = self._plan_motion_to_target(self.active_task.target_location)
            if transport_plan:
                plan_steps.append(PlanStep(
                    action="navigate_to_destination",
                    parameters={"target_location": self.active_task.target_location},
                    expected_duration=transport_plan.estimated_time,
                    preconditions=["object_picked_up"],
                    effects=["at_destination"]
                ))

        # 4. Place object
        plan_steps.append(PlanStep(
            action="place_object",
            parameters={"target_location": self.active_task.target_location},
            expected_duration=3.0,
            preconditions=["at_destination"],
            effects=["object_placed", "gripper_free"]
        ))

        return plan_steps

    def _plan_motion_to_target(self, target: Tuple[float, float, float]) -> Optional[MotionPlan]:
        """Plan motion to target using pathfinding algorithm"""
        # In practice, this would use A* or RRT on the environment map
        # For this example, we'll simulate a simple path

        # Get current robot position
        start_x, start_y, start_theta = self.robot_pose
        target_x, target_y, target_theta = target

        # Calculate straight-line path (in practice, this would avoid obstacles)
        distance = math.sqrt((target_x - start_x)**2 + (target_y - start_y)**2)

        # Generate waypoints along the path
        waypoints = []
        steps = max(1, int(distance / 0.1))  # 10cm between waypoints

        for i in range(steps + 1):
            t = i / steps
            x = start_x + t * (target_x - start_x)
            y = start_y + t * (target_y - start_y)
            theta = start_theta + t * (target_theta - start_theta)
            waypoints.append((x, y, theta))

        # Calculate estimated time (assuming 0.2 m/s speed)
        estimated_time = distance / 0.2 if distance > 0 else 0

        return MotionPlan(
            waypoints=waypoints,
            total_distance=distance,
            estimated_time=estimated_time
        )

    def _find_object_location(self, object_name: str) -> Optional[Tuple[float, float, float]]:
        """Find location of named object in environment"""
        # In practice, this would search the environment map for the object
        # For this example, we'll return a simulated location
        object_locations = {
            "cup": (1.0, 0.0, 0.0),
            "book": (1.5, 0.5, 0.0),
            "table": (2.0, 0.0, 0.0),
            "chair": (1.5, 1.0, 0.0)
        }

        return object_locations.get(object_name, (1.0, 1.0, 0.0))  # Default location

    def _find_person_location(self) -> Optional[Tuple[float, float, float]]:
        """Find location of person in environment"""
        # In practice, this would detect and locate people
        # For this example, we'll return a simulated location
        return (0.5, 0.5, 0.0)  # Default person location

    def validate_plan(self, plan: List[PlanStep]) -> bool:
        """Validate that the plan is executable and safe"""
        if not plan:
            return False

        # Check if all steps have preconditions satisfied
        # In practice, this would check current state against preconditions
        for step in plan:
            # For this example, we'll assume all preconditions can be met
            # In a real system, you'd check the current state
            pass

        # Check if plan is safe
        if self._is_plan_safe(plan):
            return True
        else:
            rospy.logwarn("Plan failed safety validation")
            return False

    def _is_plan_safe(self, plan: List[PlanStep]) -> bool:
        """Check if the plan is safe to execute"""
        # In practice, this would validate each step for safety
        # For this example, we'll assume the plan is safe
        return True

    def clear_active_plan(self):
        """Clear the current active plan"""
        self.current_plan = []
        self.active_task = None
```

### 1.5 Safety System

```python
#!/usr/bin/env python3
"""
Safety System for Physical AI
Handles safety monitoring and emergency procedures
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from enum import Enum
import threading
import time

class SafetyLevel(Enum):
    """Safety levels for different situations"""
    NORMAL = "normal"
    WARNING = "warning"
    ALERT = "alert"
    EMERGENCY = "emergency"

class SafetyViolation(Enum):
    """Types of safety violations"""
    OBSTACLE_TOO_CLOSE = "obstacle_too_close"
    SPEED_VIOLATION = "speed_violation"
    BOUNDARY_VIOLATION = "boundary_violation"
    HUMAN_TOO_CLOSE = "human_too_close"
    STABILITY_ISSUE = "stability_issue"
    UNKNOWN = "unknown"

@dataclass
class SafetyCondition:
    """Represents a safety condition check"""
    name: str
    value: bool
    threshold: float
    current_value: float
    description: str

class SafetySystem:
    """Manages safety in the Physical AI system"""

    def __init__(self):
        # Safety thresholds and parameters
        self.safety_params = {
            'min_obstacle_distance': 0.5,  # meters
            'max_linear_speed': 0.5,       # m/s
            'max_angular_speed': 0.5,      # rad/s
            'max_human_approach': 0.8,     # meters
            'stability_threshold': 0.1     # stability measure
        }

        # Safety state tracking
        self.current_safety_level = SafetyLevel.NORMAL
        self.violations = []
        self.safety_history = []

        # Sensor data
        self.laser_scan = None
        self.latest_pose = None

        # Publishers for safety commands
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_safety_filtered', Twist, queue_size=10)

        # Subscribers for safety-critical data
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pose_sub = rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)

        # Lock for thread-safe operations
        self.safety_lock = threading.Lock()

        # Initialize safety system
        self.initialize_safety_system()

    def initialize_safety_system(self):
        """Initialize the safety system"""
        rospy.loginfo("Safety system initialized with parameters:")
        for param, value in self.safety_params.items():
            rospy.loginfo(f"  {param}: {value}")

    def laser_callback(self, data: LaserScan):
        """Handle incoming laser scan data"""
        with self.safety_lock:
            self.laser_scan = data

    def pose_callback(self, data: PoseStamped):
        """Handle incoming pose data"""
        with self.safety_lock:
            self.latest_pose = data

    def check_safety_conditions(self) -> Dict[str, bool]:
        """Check all safety conditions and return status"""
        with self.safety_lock:
            # Initialize safety status
            safety_status = {
                'safe_operation': True,
                'obstacle_free': True,
                'speed_safe': True,
                'boundary_respected': True,
                'human_safe': True,
                'system_stable': True
            }

            # Check each safety condition
            conditions = [
                self._check_obstacle_distance(),
                self._check_speed_limits(),
                self._check_boundary_violations(),
                self._check_human_proximity(),
                self._check_system_stability()
            ]

            # Aggregate results
            for condition in conditions:
                if not condition.value:
                    safety_status['safe_operation'] = False
                    safety_status[condition.name] = False

                    # Record violation
                    self.violations.append({
                        'type': self._get_violation_type(condition.name),
                        'timestamp': time.time(),
                        'value': condition.current_value,
                        'threshold': condition.threshold,
                        'description': condition.description
                    })

            # Update safety level based on violations
            self._update_safety_level(safety_status)

            # Log safety status if not normal
            if self.current_safety_level != SafetyLevel.NORMAL:
                self._log_safety_status(safety_status)

            return safety_status

    def _check_obstacle_distance(self) -> SafetyCondition:
        """Check for obstacles too close to robot"""
        if not self.laser_scan:
            return SafetyCondition(
                name='obstacle_free',
                value=True,
                threshold=self.safety_params['min_obstacle_distance'],
                current_value=float('inf'),
                description='No laser data available'
            )

        # Find minimum distance in laser scan
        min_distance = min(self.laser_scan.ranges) if self.laser_scan.ranges else float('inf')

        is_safe = min_distance > self.safety_params['min_obstacle_distance']

        return SafetyCondition(
            name='obstacle_free',
            value=is_safe,
            threshold=self.safety_params['min_obstacle_distance'],
            current_value=min_distance,
            description=f'Minimum obstacle distance: {min_distance:.2f}m'
        )

    def _check_speed_limits(self) -> SafetyCondition:
        """Check if robot is within speed limits"""
        # In practice, this would check current velocity
        # For this example, we'll assume speed is safe
        return SafetyCondition(
            name='speed_safe',
            value=True,
            threshold=self.safety_params['max_linear_speed'],
            current_value=0.0,
            description='Speed limits checked'
        )

    def _check_boundary_violations(self) -> SafetyCondition:
        """Check if robot is within operational boundaries"""
        if not self.latest_pose:
            return SafetyCondition(
                name='boundary_respected',
                value=True,
                threshold=0.0,
                current_value=0.0,
                description='No pose data available'
            )

        # Check if robot is within safe operating area
        # For this example, we'll assume it's in bounds
        x = self.latest_pose.pose.position.x
        y = self.latest_pose.pose.position.y

        # Define safe operating area (for example, within 5m of origin)
        distance_from_origin = np.sqrt(x**2 + y**2)
        is_safe = distance_from_origin < 5.0

        return SafetyCondition(
            name='boundary_respected',
            value=is_safe,
            threshold=5.0,
            current_value=distance_from_origin,
            description=f'Distance from origin: {distance_from_origin:.2f}m'
        )

    def _check_human_proximity(self) -> SafetyCondition:
        """Check for humans too close to robot"""
        if not self.laser_scan:
            return SafetyCondition(
                name='human_safe',
                value=True,
                threshold=self.safety_params['max_human_approach'],
                current_value=float('inf'),
                description='No sensor data available for human detection'
            )

        # In practice, this would use human detection algorithms
        # For this example, we'll use a simplified approach with laser scan
        # assuming any close obstacle could be a human

        min_distance = min(self.laser_scan.ranges) if self.laser_scan.ranges else float('inf')

        is_safe = min_distance > self.safety_params['max_human_approach']

        return SafetyCondition(
            name='human_safe',
            value=is_safe,
            threshold=self.safety_params['max_human_approach'],
            current_value=min_distance,
            description=f'Minimum distance to human-like obstacle: {min_distance:.2f}m'
        )

    def _check_system_stability(self) -> SafetyCondition:
        """Check system stability metrics"""
        # In practice, this would check IMU data, control stability, etc.
        # For this example, we'll assume system is stable
        return SafetyCondition(
            name='system_stable',
            value=True,
            threshold=self.safety_params['stability_threshold'],
            current_value=0.0,
            description='System stability checked'
        )

    def _get_violation_type(self, condition_name: str) -> SafetyViolation:
        """Map condition name to violation type"""
        violation_map = {
            'obstacle_free': SafetyViolation.OBSTACLE_TOO_CLOSE,
            'speed_safe': SafetyViolation.SPEED_VIOLATION,
            'boundary_respected': SafetyViolation.BOUNDARY_VIOLATION,
            'human_safe': SafetyViolation.HUMAN_TOO_CLOSE,
            'system_stable': SafetyViolation.STABILITY_ISSUE
        }

        return violation_map.get(condition_name, SafetyViolation.UNKNOWN)

    def _update_safety_level(self, safety_status: Dict[str, bool]):
        """Update safety level based on current conditions"""
        if not safety_status['safe_operation']:
            # Count violations to determine level
            violation_count = sum(1 for k, v in safety_status.items() if k != 'safe_operation' and not v)

            if violation_count >= 3:
                self.current_safety_level = SafetyLevel.EMERGENCY
            elif violation_count >= 2:
                self.current_safety_level = SafetyLevel.ALERT
            else:
                self.current_safety_level = SafetyLevel.WARNING
        else:
            self.current_safety_level = SafetyLevel.NORMAL

    def _log_safety_status(self, safety_status: Dict[str, bool]):
        """Log safety status when not in normal state"""
        rospy.logwarn(f"Safety level: {self.current_safety_level.value}")

        for condition, is_safe in safety_status.items():
            if condition != 'safe_operation' and not is_safe:
                rospy.logwarn(f"  Unsafe condition: {condition}")

        # Log recent violations
        if self.violations:
            recent_violations = self.violations[-5:]  # Last 5 violations
            for violation in recent_violations:
                rospy.logwarn(f"  Violation: {violation['type'].value} at {violation['timestamp']:.2f}")

    def validate_plan(self, plan: List) -> bool:
        """Validate that a plan is safe to execute"""
        # In practice, this would simulate or analyze the plan for safety
        # For this example, we'll return True (in a real system, this would be more complex)
        return True

    def emergency_stop(self):
        """Trigger emergency stop"""
        rospy.logerr("EMERGENCY STOP ACTIVATED")

        # Publish emergency stop command
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)

        # Stop all motion
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

        # Record in safety history
        self.safety_history.append({
            'timestamp': time.time(),
            'event': 'EMERGENCY_STOP',
            'level': 'CRITICAL',
            'description': 'Emergency stop activated'
        })

    def get_safety_report(self) -> Dict:
        """Generate comprehensive safety report"""
        return {
            'current_level': self.current_safety_level.value,
            'violations_count': len(self.violations),
            'recent_violations': self.violations[-10:],  # Last 10 violations
            'safety_params': self.safety_params,
            'safety_history_count': len(self.safety_history),
            'last_update': time.time()
        }

# Example usage
if __name__ == '__main__':
    rospy.init_node('safety_system_test', anonymous=True)

    safety_system = SafetySystem()

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        safety_status = safety_system.check_safety_conditions()
        rospy.loginfo_throttle(5, f"Safe operation: {safety_status['safe_operation']}")
        rate.sleep()
```