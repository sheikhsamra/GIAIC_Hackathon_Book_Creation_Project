# Quizzes: ROS 2 Fundamentals for Physical AI

## Quiz 2.1: ROS 2 Architecture and Concepts

### Question 1
What does DDS stand for in the context of ROS 2?

A) Distributed Data System
B) Data Distribution Service
C) Dynamic Discovery System
D) Direct Data Sharing

**Correct Answer: B**

### Question 2
Which of the following is NOT a key difference between ROS 1 and ROS 2?

A) DDS-based communication in ROS 2
B) Improved security features in ROS 2
C) Better real-time support in ROS 2
D) ROS 1 has better multi-robot support

**Correct Answer: D**

### Question 3
What is the primary purpose of Quality of Service (QoS) policies in ROS 2?

A) To make the system run faster
B) To provide guarantees about message delivery and timing
C) To encrypt messages
D) To compress data

**Correct Answer: B**

### Question 4
Which build system is used in ROS 2?

A) catkin
B) colcon
C) cmake
D) make

**Correct Answer: B**

### Question 5
What are the three main communication patterns in ROS 2?

A) Topics, Services, Actions
B) Publish, Subscribe, Request
C) Nodes, Topics, Messages
D) Publisher, Subscriber, Server

**Correct Answer: A**

### Question 6
Which QoS policy determines whether all messages are delivered?

A) Durability
B) History
C) Reliability
D) Deadline

**Correct Answer: C**

### Question 7
What does the "Transient Local" durability policy mean?

A) Messages are only delivered to current subscribers
B) Late-joining subscribers receive the last known value
C) Messages are stored permanently
D) Messages are delivered with maximum reliability

**Correct Answer: B**

### Question 8
Which client libraries are available for ROS 2?

A) rclcpp and rclpy
B) rospy and roscpp
C) Both A and B
D) Only rclcpp

**Correct Answer: A**

---

## Quiz 2.2: Nodes, Packages, and Communication

### Question 1
What is a ROS 2 node?

A) A message type
B) An executable process that performs computation
C) A communication channel
D) A package dependency

**Correct Answer: B**

### Question 2
Which command is used to create a new ROS 2 package?

A) catkin_create_pkg
B) ros2 pkg create
C) ros2 create package
D) ros2 new package

**Correct Answer: B**

### Question 3
In the publisher-subscriber pattern, what is the role of the publisher?

A) To receive messages from other nodes
B) To send messages to subscribers
C) To store messages permanently
D) To manage communication

**Correct Answer: B**

### Question 4
What happens when you use "reliable" QoS policy for a topic?

A) Messages may be lost but delivery is faster
B) All messages are guaranteed to be delivered
C) Only the last message is delivered
D) Messages are delivered with minimal latency

**Correct Answer: B**

### Question 5
Which file contains metadata about a ROS 2 package?

A) CMakeLists.txt
B) setup.py
C) package.xml
D) manifest.xml

**Correct Answer: C**

### Question 6
What is the purpose of a service in ROS 2?

A) To provide continuous data streams
B) To provide request-response communication
C) To store configuration parameters
D) To manage node lifecycles

**Correct Answer: B**

### Question 7
How are ROS 2 packages built?

A) Using catkin_make
B) Using colcon build
C) Using cmake
D) Using make

**Correct Answer: B**

### Question 8
What is the default queue size for ROS 2 publishers and subscribers?

A) 1
B) 10
C) 100
D) Unlimited

**Correct Answer: B**

---

## Quiz 2.3: Services and Actions

### Question 1
What is the main difference between a service and an action in ROS 2?

A) Services are faster than actions
B) Actions provide feedback during execution and support preemption
C) Services can handle multiple requests simultaneously
D) Actions use different message types

**Correct Answer: B**

### Question 2
Which of the following is NOT a component of an action goal?

A) Goal
B) Feedback
C) Result
D) Request

**Correct Answer: D**

### Question 3
In a service call, who sends the request?

A) Service server
B) Service client
C) Both client and server
D) Neither client nor server

**Correct Answer: B**

### Question 4
What does the "feedback" component of an action provide?

A) The final result of the action
B) Information about the action's progress during execution
C) Error messages from the action
D) Configuration parameters

**Correct Answer: B**

### Question 5
Which interface type would be most appropriate for a navigation task that takes several minutes?

A) Topic
B) Service
C) Action
D) Parameter

**Correct Answer: C**

### Question 6
What is the purpose of action preemption?

A) To speed up action execution
B) To cancel an action before completion
C) To improve communication reliability
D) To reduce memory usage

**Correct Answer: B**

### Question 7
In the service request-response pattern, what happens after the server processes the request?

A) The server sends a response back to the client
B) The server publishes a message to a topic
C) The server creates a new service
D) The server shuts down

**Correct Answer: A**

### Question 8
Which command can be used to list available services in ROS 2?

A) ros2 topic list
B) ros2 service list
C) ros2 node list
D) ros2 param list

**Correct Answer: B**

---

## Quiz 2.4: Parameters and Launch Systems

### Question 1
How are parameters typically declared in a ROS 2 node?

A) Using the declare_parameter() method
B) Through command line arguments
C) In the package.xml file
D) By creating a separate configuration file

**Correct Answer: A**

### Question 2
What is the purpose of launch files in ROS 2?

A) To compile source code
B) To coordinate complex system startup with multiple nodes
C) To create new message types
D) To manage hardware drivers

**Correct Answer: B**

### Question 3
Which command is used to run a launch file in ROS 2?

A) ros2 run
B) ros2 launch
C) ros2 start
D) ros2 execute

**Correct Answer: B**

### Question 4
What happens when you change a parameter value at runtime in ROS 2?

A) The node must be restarted
B) The parameter change takes effect immediately if the node supports it
C) The parameter value is locked
D) The node crashes

**Correct Answer: B**

### Question 5
Which file is used to define launch arguments in a Python launch file?

A) package.xml
B) CMakeLists.txt
C) Using DeclareLaunchArgument action
D) setup.py

**Correct Answer: C**

### Question 6
What is the purpose of namespaces in ROS 2?

A) To organize nodes into logical groups
B) To prevent naming conflicts in multi-robot systems
C) To provide hierarchical organization
D) All of the above

**Correct Answer: D**

### Question 7
How can you set parameter values when running a launch file?

A) Through launch arguments
B) By defining them directly in the launch file
C) Through command line parameters
D) All of the above

**Correct Answer: D**

### Question 8
What is node composition in ROS 2?

A) Combining multiple nodes into a single process
B) Creating a hierarchy of nodes
C) Compressing node executables
D) Sharing parameters between nodes

**Correct Answer: A**

---

## Quiz 2.5: Tools and Safety

### Question 1
Which command is used to inspect topics in ROS 2?

A) ros2 node list
B) ros2 topic list
C) ros2 service list
D) ros2 param list

**Correct Answer: B**

### Question 2
What does the ros2 doctor command do?

A) Fixes system errors automatically
B) Provides system health check and diagnostics
C) Updates ROS 2 packages
D) Creates new nodes

**Correct Answer: B**

### Question 3
Which tool is commonly used for ROS 2 visualization?

A) rqt
B) rviz
C) Both A and B
D) Neither A nor B

**Correct Answer: C**

### Question 4
What is the primary purpose of a safety monitor in Physical AI systems?

A) To improve system performance
B) To ensure safe operation by monitoring and limiting dangerous behaviors
C) To reduce computational requirements
D) To simplify system architecture

**Correct Answer: B**

### Question 5
Which QoS policy would be most appropriate for emergency stop commands?

A) Best effort
B) Reliable
C) Volatile
D) Keep last with depth 1

**Correct Answer: B**

### Question 6
What should a safety system do when a dangerous condition is detected?

A) Continue normal operation
B) Shut down immediately without warning
C) Take protective action (e.g., emergency stop) and alert operators
D) Ignore the condition

**Correct Answer: C**

### Question 7
Which command is used to record ROS 2 topics to a bag file?

A) ros2 record
B) ros2 bag record
C) ros2 log
D) ros2 capture

**Correct Answer: B**

### Question 8
What is the importance of fail-safe design in Physical AI systems?

A) Systems should default to safe states when problems occur
B) Systems should continue operating under all conditions
C) Systems should prioritize performance over safety
D) Systems should require manual intervention for all operations

**Correct Answer: A**

---

## Quiz 2.6: Integration and Best Practices

### Question 1
What is the main advantage of using ROS 2 for Physical AI systems?

A) It provides a standardized communication framework
B) It offers extensive hardware abstraction
C) It enables rapid prototyping and testing
D) All of the above

**Correct Answer: D**

### Question 2
Which pattern is recommended for sensor data that can tolerate some message loss?

A) Reliable QoS
B) Best effort QoS
C) Transient local durability
D) Keep all history

**Correct Answer: B**

### Question 3
What is the purpose of the ros2 bag tool?

A) To manage package dependencies
B) To record and replay ROS 2 data for testing and analysis
C) To create new message types
D) To configure network settings

**Correct Answer: B**

### Question 4
In Physical AI applications, why is real-time performance important?

A) For predictable response to environmental changes
B) For safety-critical operations
C) For coordination with other systems
D) All of the above

**Correct Answer: D**

### Question 5
What should be considered when designing ROS 2 nodes for Physical AI systems?

A) Error handling and graceful degradation
B) Resource management and efficiency
C) Safety and security measures
D) All of the above

**Correct Answer: D**

### Question 6
Which approach is recommended for handling critical safety parameters?

A) Hardcoding values in source code
B) Using runtime parameters with validation
C) Storing in external files without checks
D) Using default values only

**Correct Answer: B**

### Question 7
What is the benefit of using launch files for complex Physical AI systems?

A) Simplified system startup
B) Consistent configuration
C) Easier testing and deployment
D) All of the above

**Correct Answer: D**

### Question 8
Why is it important to validate sensor data in ROS 2 nodes?

A) To prevent processing invalid or dangerous inputs
B) To maintain system stability
C) To ensure safe operation
D) All of the above

**Correct Answer: D**

---

## Answers Summary

### Quiz 2.1 Answers:
1. B
2. D
3. B
4. B
5. A
6. C
7. B
8. A

### Quiz 2.2 Answers:
1. B
2. B
3. B
4. B
5. C
6. B
7. B
8. B

### Quiz 2.3 Answers:
1. B
2. D
3. B
4. B
5. C
6. B
7. A
8. B

### Quiz 2.4 Answers:
1. A
2. B
3. B
4. B
5. C
6. D
7. D
8. A

### Quiz 2.5 Answers:
1. B
2. B
3. C
4. B
5. B
6. C
7. B
8. A

### Quiz 2.6 Answers:
1. D
2. B
3. B
4. D
5. D
6. B
7. D
8. D