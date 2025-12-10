# Quizzes: Humanoid Robotics and Locomotion

## Quiz 5.1: Humanoid Robot Fundamentals

### Question 1
What is the primary characteristic that distinguishes humanoid robots from other robot types?

A) Number of wheels
B) Human-like morphology and bipedal locomotion
C) Color of the robot
D) Size of the robot

**Correct Answer: B**

### Question 2
What does "Degrees of Freedom (DOF)" refer to in humanoid robotics?

A) Number of different colors the robot can display
B) Number of independent movements a robot can perform
C) Number of sensors on the robot
D) Number of batteries the robot uses

**Correct Answer: B**

### Question 3
Which of the following is NOT a typical application of humanoid robots?

A) Assisting elderly and disabled individuals
B) Performing tasks in human-designed environments
C) Industrial manufacturing on assembly lines
D) Entertainment and education

**Correct Answer: C**

### Question 4
What is the typical number of degrees of freedom in a humanoid robot?

A) 5-10 DOF
B) 10-20 DOF
C) 20-30 DOF
D) 30+ DOF

**Correct Answer: D**

### Question 5
What is the main challenge in humanoid robotics related to locomotion?

A) Making the robot look attractive
B) Maintaining balance during dynamic motion
C) Painting the robot in bright colors
D) Connecting to Wi-Fi networks

**Correct Answer: B**

### Question 6
What does "CoM" stand for in humanoid robotics?

A) Center of Mass
B) Center of Motion
C) Center of Mechanics
D) Center of Movement

**Correct Answer: A**

### Question 7
Which part of the human body does the "torso" of a humanoid robot represent?

A) Legs
B) Arms
C) Trunk/chest area
D) Head

**Correct Answer: C**

### Question 8
What is the typical CoM height for a humanoid robot?

A) 0.3-0.5 meters
B) 0.5-0.7 meters
C) 0.7-1.0 meters
D) 1.0-1.5 meters

**Correct Answer: B**

---

## Quiz 5.2: Biomechanics and Human Locomotion

### Question 1
What percentage of the gait cycle is typically spent in the stance phase for humans?

A) 20%
B) 40%
C) 60%
D) 80%

**Correct Answer: C**

### Question 2
What is the "swing phase" in human walking?

A) When both feet are on the ground
B) When one foot is off the ground and moving forward
C) When the person is running
D) When the person is sitting

**Correct Answer: B**

### Question 3
What does the "Zero Moment Point (ZMP)" represent?

A) The point where gravity has no effect
B) The point where the net moment of ground reaction forces is zero
C) The center of the foot
D) The height of the CoM

**Correct Answer: B**

### Question 4
What is the "Capture Point" in humanoid balance control?

A) The point where the robot can grab objects
B) The location where a point mass can be brought to rest with one step
C) The center of the robot
D) The point where sensors are located

**Correct Answer: B**

### Question 5
What is the typical walking speed range for humans?

A) 0.1-0.5 m/s
B) 0.5-1.5 m/s
C) 1.5-2.5 m/s
D) 2.5-3.5 m/s

**Correct Answer: B**

### Question 6
What does the "inverted pendulum model" represent in walking?

A) A person walking upside down
B) A model treating the body as a pendulum pivoting at the ankles
C) A physical pendulum hanging from the feet
D) A model of a person swinging on a bar

**Correct Answer: B**

### Question 7
What is the main purpose of the double support phase in walking?

A) To make the robot jump
B) To transfer weight between legs and provide smooth transitions
C) To stop the robot completely
D) To charge the robot's batteries

**Correct Answer: B**

### Question 8
What is the typical step width for human walking?

A) 0.05-0.10 meters
B) 0.10-0.20 meters
C) 0.20-0.30 meters
D) 0.30-0.40 meters

**Correct Answer: B**

---

## Quiz 5.3: Balance Control and Stability

### Question 1
What is the primary purpose of ZMP control in humanoid robots?

A) To make the robot dance
B) To maintain balance by keeping ZMP within support polygon
C) To increase the robot's speed
D) To improve the robot's appearance

**Correct Answer: B**

### Question 2
What is the "support polygon" in humanoid balance?

A) A polygon drawn on the floor
B) The area bounded by ground contact points where ZMP must remain
C) A decorative pattern on the robot
D) A polygon representing the robot's head

**Correct Answer: B**

### Question 3
What is the main difference between static and dynamic balance?

A) Static is faster than dynamic
B) Static balance keeps CoM within support polygon; dynamic uses stepping and momentum
C) Static uses more power than dynamic
D) Static is for sitting, dynamic is for standing

**Correct Answer: B**

### Question 4
What is "ankle strategy" in balance control?

A) Using ankle joints to maintain balance for small disturbances
B) Moving the entire body to maintain balance
C) Taking a step to maintain balance
D) Using arm movements to maintain balance

**Correct Answer: A**

### Question 5
What is "hip strategy" in balance control?

A) Using hip joints for small balance adjustments
B) Using hip joints for medium-range balance corrections
C) Using hip joints for large balance corrections
D) Using hip joints for walking only

**Correct Answer: B**

### Question 6
What is "stepping strategy" in balance control?

A) Using steps to maintain balance for large disturbances
B) Using steps to increase walking speed
C) Using steps to change direction only
D) Using steps to improve appearance

**Correct Answer: A**

### Question 7
What is the typical natural frequency of the human inverted pendulum?

A) 0.5-1.0 Hz
B) 1.0-2.0 Hz
C) 2.0-3.0 Hz
D) 3.0-4.0 Hz

**Correct Answer: B**

### Question 8
What is the main advantage of using PD (Proportional-Derivative) control for balance?

A) It's cheaper to implement
B) It provides both position and velocity feedback for stability
C) It's easier to program
D) It uses less power

**Correct Answer: B**

---

## Quiz 5.4: Walking Gait Generation

### Question 1
What is the main purpose of walking pattern generation?

A) To make the robot look good
B) To create stable, efficient walking patterns that maintain balance
C) To play music while walking
D) To flash lights during walking

**Correct Answer: B**

### Question 2
What is "preview control" in walking?

A) Watching the robot walk in advance
B) Using future reference trajectory to plan current motion
C) Recording video of the walk
D) Planning the next step only

**Correct Answer: B**

### Question 3
What is the typical duration of a single walking step?

A) 0.1-0.3 seconds
B) 0.3-0.6 seconds
C) 0.6-1.0 seconds
D) 1.0-1.5 seconds

**Correct Answer: C**

### Question 4
What is "DSP" in walking terminology?

A) Digital Signal Processing
B) Double Support Phase
C) Dynamic Stability Pattern
D) Direct Servo Position

**Correct Answer: B**

### Question 5
What is "SSP" in walking terminology?

A) Single Support Phase
B) Static Stability Pattern
C) Servo Signal Processing
D) Simple Step Pattern

**Correct Answer: A**

### Question 6
What is the typical duty factor for human walking?

A) 0.1-0.3 (10-30%)
B) 0.3-0.5 (30-50%)
C) 0.5-0.7 (50-70%)
D) 0.7-0.9 (70-90%)

**Correct Answer: C**

### Question 7
What is the main purpose of foot clearance during the swing phase?

A) To look more elegant
B) To avoid obstacles and ensure safe ground contact
C) To increase speed
D) To reduce power consumption

**Correct Answer: B**

### Question 8
What is the typical step length for human walking?

A) 0.3-0.5 meters
B) 0.5-0.7 meters
C) 0.7-0.9 meters
D) 0.9-1.1 meters

**Correct Answer: B**

---

## Quiz 5.5: Control Systems for Humanoid Robots

### Question 1
What is the typical structure of humanoid robot control?

A) Single monolithic controller
B) Hierarchical control with high, mid, and low levels
C) Random control structure
D) No control structure needed

**Correct Answer: B**

### Question 2
What does "operational space control" refer to?

A) Controlling the robot in joint space
B) Controlling the robot in Cartesian task space
C) Controlling the robot's operations
D) Controlling the robot's workspace

**Correct Answer: B**

### Question 3
What is "inverse kinematics" used for?

A) Moving the robot forward
B) Calculating joint angles from desired end-effector positions
C) Calculating end-effector positions from joint angles
D) Inverting the robot's movement

**Correct Answer: B**

### Question 4
What is "whole-body control"?

A) Controlling all parts of the robot simultaneously considering constraints
B) Controlling the robot's entire body separately
C) Controlling the robot while it's upside down
D) Controlling the robot's complete lifecycle

**Correct Answer: A**

### Question 5
What is "impedance control"?

A) Controlling the robot's resistance to motion
B) Controlling the robot's electrical circuits
C) Controlling the robot's communication
D) Controlling the robot's appearance

**Correct Answer: A**

### Question 6
What is "admittance control"?

A) Controlling how the robot responds to external forces
B) Controlling the robot's admission to areas
C) Controlling the robot's admittance to electricity
D) Controlling the robot's admittance to water

**Correct Answer: A**

### Question 7
What is "task prioritization" in whole-body control?

A) Ranking tasks by importance and executing them in order
B) Prioritizing the robot's tasks
C) Ranking the robot's tasks
D) Organizing the robot's tasks

**Correct Answer: A**

### Question 8
What is "constraint handling" in humanoid control?

A) Handling physical constraints like joint limits and collisions
B) Handling the robot's constraints
C) Handling constraints on the robot's operation
D) Handling the robot's behavioral constraints

**Correct Answer: A**

---

## Quiz 5.6: AI and Machine Learning for Locomotion

### Question 1
What is reinforcement learning used for in humanoid locomotion?

A) To make the robot learn optimal walking patterns through interaction with environment
B) To paint the robot
C) To make the robot sing
D) To control the robot's lights

**Correct Answer: A**

### Question 2
What does "exploration vs exploitation" mean in RL?

A) Exploring vs exploiting the robot's features
B) Balancing trying new actions vs using known good actions
C) Exploiting the robot's resources
D) Exploring the robot's capabilities

**Correct Answer: B**

### Question 3
What is "epsilon-greedy" policy?

A) A policy that randomly selects actions
B) A policy that balances random exploration with greedy exploitation
C) A policy that always exploits
D) A policy that always explores

**Correct Answer: B**

### Question 4
What is "imitation learning" in robotics?

A) Learning by copying human demonstrations
B) Learning by imitating other robots
C) Learning by imitation games
D) Learning by imitation of animals

**Correct Answer: A**

### Question 5
What is "inverse reinforcement learning"?

A) Learning the reward function from expert demonstrations
B) Learning by reversing the robot
C) Learning by inverting the process
D) Learning by inverse kinematics

**Correct Answer: A**

### Question 6
What is "model predictive control" (MPC)?

A) Predicting the robot's model
B) Control method that uses model predictions to optimize future behavior
C) Controlling the robot's model
D) Predicting the robot's control

**Correct Answer: B**

### Question 7
What is "deep reinforcement learning"?

A) Learning with deep neural networks for policy/value functions
B) Learning that goes deep into the robot
C) Learning with deep pockets
D) Learning with deep understanding

**Correct Answer: A**

### Question 8
What is "policy gradient" method?

A) Method that directly optimizes the policy parameters
B) Method that gradients the policy
C) Method that polishes the gradient
D) Method that creates policy gradients

**Correct Answer: A**

---

## Quiz 5.7: Safety and Human-Robot Interaction

### Question 1
What is the most important safety consideration for humanoid robots?

A) Making them look nice
B) Ensuring they don't harm humans or themselves
C) Making them fast
D) Making them cheap

**Correct Answer: B**

### Question 2
What is "emergency stop" in humanoid robotics?

A) A way to make the robot stop dancing
B) An immediate shutdown mechanism for safety
C) A way to stop the robot's music
D) A way to stop the robot's lights

**Correct Answer: B**

### Question 3
What is "compliance control"?

A) Making the robot compliant with laws
B) Controlling the robot's compliance to be safe for human interaction
C) Making the robot compliant with standards
D) Making the robot compliant with users

**Correct Answer: B**

### Question 4
What is "safe human-robot interaction"?

A) Interaction that is legally safe
B) Interaction that ensures physical safety of humans
C) Interaction that is financially safe
D) Interaction that is environmentally safe

**Correct Answer: B**

### Question 5
What is "collision detection" in humanoid safety?

A) Detecting collisions with other robots
B) Detecting potential or actual collisions with humans/environment
C) Detecting collision courses
D) Detecting collision damage

**Correct Answer: B**

### Question 6
What is "force limitation" in humanoid safety?

A) Limiting the robot's force to prevent injury to humans
B) Limiting the robot's force for power saving
C) Limiting the robot's force for speed
D) Limiting the robot's force for cost

**Correct Answer: A**

### Question 7
What is "predictable behavior" in safety?

A) Behavior that is predictable for marketing
B) Behavior that humans can anticipate and react to safely
C) Behavior that is predictable for programming
D) Behavior that is predictable for maintenance

**Correct Answer: B**

### Question 8
What is "fail-safe design"?

A) Design that fails safely
B) Design that is safe from failure
C) Design that ensures safe failure modes
D) Design that prevents all failures

**Correct Answer: C**

---

## Quiz 5.8: Simulation and Validation

### Question 1
Why is simulation important for humanoid robots?

A) Because it's fun to watch
B) To test and validate control algorithms safely before real-world deployment
C) Because it's cheaper than real robots
D) Because it's easier to program

**Correct Answer: B**

### Question 2
What is "reality gap"?

A) The gap in the robot's reality
B) The difference between simulation and real-world performance
C) The gap in the robot's knowledge
D) The gap in the robot's hardware

**Correct Answer: B**

### Question 3
What is "domain randomization"?

A) Randomizing the robot's domain
B) Technique of randomizing simulation parameters to improve transfer to reality
C) Randomizing the robot's domain knowledge
D) Randomizing the robot's domain expertise

**Correct Answer: B**

### Question 4
What is "system identification"?

A) Identifying the robot's system
B) Process of determining real robot parameters from experimental data
C) Identifying the system's robot
D) Identifying the robot's system components

**Correct Answer: B**

### Question 5
What is "transfer learning" in robotics?

A) Learning to transfer objects
B) Applying knowledge learned in simulation to real robots
C) Learning to transfer positions
D) Learning to transfer forces

**Correct Answer: B**

### Question 6
What is "hardware-in-the-loop" (HIL) simulation?

A) Simulation with hardware in the loop
B) Simulation that includes real hardware components
C) Simulation with hardware loop
D) Simulation in hardware loop

**Correct Answer: B**

### Question 7
What is "Monte Carlo validation"?

A) Validation in Monte Carlo
B) Statistical validation using multiple random simulations
C) Validation with Monte Carlo methods
D) Validation using Monte Carlo techniques

**Correct Answer: B**

### Question 8
What is "formal verification" in robotics?

A) Formal validation of the robot
B) Mathematical proof of correctness of system properties
C) Formal validation of the system
D) Formal verification of the robot's performance

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
8. B

### Quiz 5.2 Answers:
1. C
2. B
3. B
4. B
5. B
6. B
7. B
8. B

### Quiz 5.3 Answers:
1. B
2. B
3. B
4. A
5. B
6. A
7. B
8. B

### Quiz 5.4 Answers:
1. B
2. B
3. C
4. B
5. A
6. C
7. B
8. B

### Quiz 5.5 Answers:
1. B
2. B
3. B
4. A
5. A
6. A
7. A
8. A

### Quiz 5.6 Answers:
1. A
2. B
3. B
4. A
5. A
6. B
7. A
8. A

### Quiz 5.7 Answers:
1. B
2. B
3. B
4. B
5. B
6. A
7. B
8. C

### Quiz 5.8 Answers:
1. B
2. B
3. B
4. B
5. B
6. B
7. B
8. B