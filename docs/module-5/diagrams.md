# Diagrams: Humanoid Robotics and Locomotion

## 1. Humanoid Robot Architecture

### 1.1 Humanoid Robot Kinematic Structure
```mermaid
graph TD
    subgraph "Humanoid Robot Structure"
        A[Base Link] --> B[Torso]
        B --> C[Head]
        A --> D[Left Hip]
        D --> E[Left Knee]
        E --> F[Left Ankle]
        F --> G[Left Foot]
        A --> H[Right Hip]
        H --> I[Right Knee]
        I --> J[Right Ankle]
        J --> K[Right Foot]
        B --> L[Left Shoulder]
        L --> M[Left Elbow]
        M --> N[Left Wrist]
        B --> O[Right Shoulder]
        O --> P[Right Elbow]
        P --> Q[Right Wrist]
    end

    style A fill:#b3e0ff
    style B fill:#c8e6c9
    style C fill:#e1bee7
    style D fill:#ffccbc
    style E fill:#ffccbc
    style F fill:#ffccbc
    style G fill:#ffccbc
    style H fill:#ffccbc
    style I fill:#ffccbc
    style J fill:#ffccbc
    style K fill:#ffccbc
    style L fill:#fff9c4
    style M fill:#fff9c4
    style N fill:#fff9c4
    style O fill:#fff9c4
    style P fill:#fff9c4
    style Q fill:#fff9c4
```

### 1.2 Degrees of Freedom in Humanoid Robot
```mermaid
graph LR
    subgraph "Lower Body (Legs)"
        LeftHip[Left Hip: 3DOF<br/>Roll, Pitch, Yaw]
        LeftKnee[Left Knee: 1DOF<br/>Pitch]
        LeftAnkle[Left Ankle: 2DOF<br/>Pitch, Roll]
        RightHip[Right Hip: 3DOF<br/>Roll, Pitch, Yaw]
        RightKnee[Right Knee: 1DOF<br/>Pitch]
        RightAnkle[Right Ankle: 2DOF<br/>Pitch, Roll]
    end

    subgraph "Upper Body (Arms)"
        LeftShoulder[Left Shoulder: 3DOF<br/>Roll, Pitch, Yaw]
        LeftElbow[Left Elbow: 1DOF<br/>Pitch]
        LeftWrist[Left Wrist: 3DOF<br/>Roll, Pitch, Yaw]
        RightShoulder[Right Shoulder: 3DOF<br/>Roll, Pitch, Yaw]
        RightElbow[Right Elbow: 1DOF<br/>Pitch]
        RightWrist[Right Wrist: 3DOF<br/>Roll, Pitch, Yaw]
    end

    subgraph "Torso & Head"
        Waist[Waist: 3DOF<br/>Roll, Pitch, Yaw]
        Neck[Neck: 3DOF<br/>Roll, Pitch, Yaw]
    end

    LeftHip --- LeftKnee
    LeftKnee --- LeftAnkle
    RightHip --- RightKnee
    RightKnee --- RightAnkle
    LeftShoulder --- LeftElbow
    LeftElbow --- LeftWrist
    RightShoulder --- RightElbow
    RightElbow --- RightWrist
    Waist --- LeftHip
    Waist --- RightHip
    Waist --- Neck

    style LeftHip fill:#b3e0ff
    style LeftKnee fill:#b3e0ff
    style LeftAnkle fill:#b3e0ff
    style RightHip fill:#b3e0ff
    style RightKnee fill:#b3e0ff
    style RightAnkle fill:#b3e0ff
    style LeftShoulder fill:#c8e6c9
    style LeftElbow fill:#c8e6c9
    style LeftWrist fill:#c8e6c9
    style RightShoulder fill:#c8e6c9
    style RightElbow fill:#c8e6c9
    style RightWrist fill:#c8e6c9
    style Waist fill:#fff9c4
    style Neck fill:#fff9c4
```

## 2. Biomechanics and Human Locomotion

### 2.1 Human Gait Cycle Phases
```mermaid
gantt
    title Human Gait Cycle Phases
    dateFormat X
    axisFormat %s

    section Stance Phase
    Heel Strike           :HS, 0, 0.08
    Loading Response      :LR, 0.08, 0.18
    Mid Stance            :MS, 0.18, 0.30
    Terminal Stance       :TS, 0.30, 0.50

    section Swing Phase
    Pre-swing             :PS, 0.50, 0.60
    Initial Swing         :IS, 0.60, 0.73
    Mid Swing             :MSw, 0.73, 0.87
    Terminal Swing        :TSw, 0.87, 1.00

    section Support Type
    Single Support        :SS, 0.0, 0.60
    Double Support        :DS, 0.60, 0.62
    Single Support 2      :SS2, 0.62, 1.00
```

### 2.2 Inverted Pendulum Model
```mermaid
graph TD
    subgraph "Inverted Pendulum Model"
        CoM[Center of Mass<br/>Mass = m]
        Support[Support Point<br/>Foot Contact]
        ZMP[Zero Moment Point<br/>ZMP = CoM - (h/g)*CoM_ddot]
        CoP[Center of Pressure<br/>Under Foot]
    end

    CoM --- Support
    Support --- ZMP
    Support --- CoP

    subgraph "Balance Control"
        Feedback[Feedback Control<br/>Adjust CoM Position]
        AnkleStrategy[Ankle Strategy<br/>Ankle Torque]
        HipStrategy[Hip Strategy<br/>Hip Torque]
        StepStrategy[Step Strategy<br/>Foot Placement]
    end

    ZMP -.-> Feedback
    Feedback -.-> AnkleStrategy
    Feedback -.-> HipStrategy
    Feedback -.-> StepStrategy

    style CoM fill:#b3e0ff
    style Support fill:#c8e6c9
    style ZMP fill:#ffccbc
    style CoP fill:#ffccbc
    style Feedback fill:#e1bee7
    style AnkleStrategy fill:#fff9c4
    style HipStrategy fill:#fff9c4
    style StepStrategy fill:#fff9c4
```

### 2.3 Capture Point Concept
```mermaid
graph LR
    subgraph "Balance Control with Capture Point"
        CoM[Current CoM<br/>Position and Velocity]
        CoMVelocity[CoM Velocity<br/>v = dx/dt]
        Gravity[Gravity<br/>g = 9.81 m/s²]
        CoMHeight[CoM Height<br/>h]
    end

    subgraph "Capture Point Calculation"
        Omega[w = √(g/h)<br/>Natural Frequency]
        CapturePoint[Capture Point<br/>CP = CoM + v/w]
    end

    subgraph "Balance Strategies"
        StepLocation[Required Step Location<br/>At Capture Point]
        CoMAdjustment[CoM Adjustment<br/>Modify Trajectory]
        AnkleControl[Ankle Control<br/>Maintain Balance]
    end

    CoM --> CapturePoint
    CoMVelocity --> CapturePoint
    Gravity --> Omega
    CoMHeight --> Omega
    Omega --> CapturePoint
    CapturePoint --> StepLocation
    CapturePoint --> CoMAdjustment
    CapturePoint --> AnkleControl

    style CoM fill:#b3e0ff
    style CoMVelocity fill:#b3e0ff
    style Gravity fill:#c8e6c9
    style CoMHeight fill:#c8e6c9
    style Omega fill:#e1bee7
    style CapturePoint fill:#ffccbc
    style StepLocation fill:#fff9c4
    style CoMAdjustment fill:#fff9c4
    style AnkleControl fill:#fff9c4
```

## 3. Balance Control Systems

### 3.1 ZMP-Based Balance Control Architecture
```mermaid
graph TD
    subgraph "Reference Trajectories"
        ZMPRef[ZMP Reference<br/>Trajectory]
        CoMRef[CoM Reference<br/>Trajectory]
    end

    subgraph "State Estimation"
        StateEst[State Estimator<br/>Joint Angles, IMU, FT Sensors]
        CoMEst[CoM Estimator<br/>Kinematic Calculation]
        ZMPEst[ZMP Estimator<br/>Force/Torque Integration]
    end

    subgraph "Balance Controller"
        ZMPController[ZMP Controller<br/>PD Control]
        CoMController[CoM Controller<br/>Feedback Control]
        WholeBodyCtrl[Whole-Body Controller<br/>Inverse Kinematics]
    end

    subgraph "Robot Control"
        InvKinematics[Inverse Kinematics<br/>Joint Angle Commands]
        JointControl[Joint Controllers<br/>Position/Torque Control]
        Robot[Humanoid Robot<br/>Physical System]
    end

    subgraph "Feedback"
        JointSensors[Joint Sensors<br/>Encoders, Torque]
        ForceSensors[Force/Torque Sensors<br/>Foot, Hand Contacts]
        IMUSensors[IMU Sensors<br/>Orientation, Acceleration]
    end

    ZMPRef --> ZMPController
    CoMRef --> CoMController
    StateEst --> ZMPEst
    StateEst --> CoMEst
    ZMPEst --> ZMPController
    CoMEst --> CoMController
    ZMPController --> WholeBodyCtrl
    CoMController --> WholeBodyCtrl
    WholeBodyCtrl --> InvKinematics
    InvKinematics --> JointControl
    JointControl --> Robot
    Robot --> JointSensors
    Robot --> ForceSensors
    Robot --> IMUSensors
    JointSensors --> StateEst
    ForceSensors --> StateEst
    IMUSensors --> StateEst

    style ZMPRef fill:#b3e0ff
    style CoMRef fill:#b3e0ff
    style StateEst fill:#c8e6c9
    style CoMEst fill:#c8e6c9
    style ZMPEst fill:#c8e6c9
    style ZMPController fill:#e1bee7
    style CoMController fill:#e1bee7
    style WholeBodyCtrl fill:#ffccbc
    style InvKinematics fill:#fff9c4
    style JointControl fill:#fff9c4
    style Robot fill:#d1c4e9
    style JointSensors fill:#f8bbd0
    style ForceSensors fill:#f8bbd0
    style IMUSensors fill:#f8bbd0
```

### 3.2 Balance Control Strategies
```mermaid
graph TD
    subgraph "Balance Control Hierarchy"
        HighLevel[High-Level Planner<br/>Walking, Standing, Tasks]
        MidLevel[Mid-Level Controller<br/>Balance, Gait Generation]
        LowLevel[Low-Level Controller<br/>Joint Servo, Safety]
    end

    subgraph "Balance Strategies"
        AnkleStrategy[Ankle Strategy<br/>0-2.5 cm sway]
        HipStrategy[Hip Strategy<br/>2.5-5.0 cm sway]
        StepStrategy[Step Strategy<br/>>5.0 cm sway]
        AnkleStrategy -.-> HipStrategy
        HipStrategy -.-> StepStrategy
    end

    subgraph "Control Modes"
        Passive[Passive Compliance<br/>Spring-Damper]
        Active[Active Control<br/>Feedback Control]
        Predictive[Predictive Control<br/>Feedforward]
    end

    HighLevel --> MidLevel
    MidLevel --> LowLevel
    MidLevel --> AnkleStrategy
    MidLevel --> HipStrategy
    MidLevel --> StepStrategy
    AnkleStrategy --> Passive
    HipStrategy --> Active
    StepStrategy --> Predictive

    style HighLevel fill:#b3e0ff
    style MidLevel fill:#c8e6c9
    style LowLevel fill:#e1bee7
    style AnkleStrategy fill:#ffccbc
    style HipStrategy fill:#ffccbc
    style StepStrategy fill:#ffccbc
    style Passive fill:#fff9c4
    style Active fill:#fff9c4
    style Predictive fill:#fff9c4
```

## 4. Walking Gait Generation

### 4.1 Walking Pattern Generation Process
```mermaid
graph TD
    subgraph "Input Parameters"
        WalkParams[Walking Parameters<br/>Speed, Step Length, Height]
        Terrain[Terrain Information<br/>Type, Slope, Obstacles]
        UserCmd[User Commands<br/>Start, Stop, Turn]
    end

    subgraph "Pattern Generator"
        FootPlacement[Foot Placement<br/>Location and Timing]
        SwingTrajectory[Swing Trajectory<br/>Foot Path Planning]
        ZMPTraj[ZMP Trajectory<br/>Stability Planning]
        CoMTraj[CoM Trajectory<br/>Balance Planning]
    end

    subgraph "Trajectory Smoothing"
        SplineFit[Spline Fitting<br/>Smooth Transitions]
        TimingAdjust[Timing Adjustment<br/>Phase Synchronization]
        StabilityCheck[Stability Check<br/>ZMP Within Support]
    end

    subgraph "Output Generation"
        JointTraj[Joint Trajectories<br/>Inverse Kinematics]
        Timing[Timing Signals<br/>Phase Control]
        ForceCtrl[Force Control<br/>Compliance Planning]
    end

    WalkParams --> FootPlacement
    Terrain --> FootPlacement
    UserCmd --> FootPlacement
    FootPlacement --> SwingTrajectory
    FootPlacement --> ZMPTraj
    SwingTrajectory --> CoMTraj
    ZMPTraj --> CoMTraj
    CoMTraj --> SplineFit
    SwingTrajectory --> SplineFit
    ZMPTraj --> SplineFit
    SplineFit --> TimingAdjust
    TimingAdjust --> StabilityCheck
    StabilityCheck --> JointTraj
    StabilityCheck --> Timing
    StabilityCheck --> ForceCtrl

    style WalkParams fill:#b3e0ff
    style Terrain fill:#b3e0ff
    style UserCmd fill:#b3e0ff
    style FootPlacement fill:#c8e6c9
    style SwingTrajectory fill:#c8e6c9
    style ZMPTraj fill:#c8e6c9
    style CoMTraj fill:#c8e6c9
    style SplineFit fill:#e1bee7
    style TimingAdjust fill:#e1bee7
    style StabilityCheck fill:#ffccbc
    style JointTraj fill:#fff9c4
    style Timing fill:#fff9c4
    style ForceCtrl fill:#fff9c4
```

### 4.2 Walking Gait Phases and Control
```mermaid
sequenceDiagram
    participant C as Controller
    participant R as Robot
    participant L as Left Foot
    participant Rf as Right Foot

    Note over C,R: Double Support Phase (Start)
    C->>R: Start step transition
    R->>L: Maintain contact
    R->>Rf: Prepare to lift

    Note over C,R: Single Support Phase (Left)
    C->>R: Left support, right swing
    R->>L: Maintain ground contact
    R->>Rf: Swing forward trajectory

    Note over C,R: Double Support Phase (Mid)
    C->>R: Shift weight
    R->>L: Begin to lift
    R->>Rf: Make ground contact

    Note over C,R: Single Support Phase (Right)
    C->>R: Right support, left swing
    R->>Rf: Maintain ground contact
    R->>L: Swing forward trajectory

    Note over C,R: Cycle repeats
```

## 5. Control Systems Architecture

### 5.1 Hierarchical Control Architecture
```mermaid
graph TD
    subgraph "High-Level Planning"
        TaskPlanner[Task Planner<br/>Reach, Walk, Manipulate]
        PathPlanner[Path Planner<br/>Navigation, Obstacle Avoidance]
        GaitSelector[Gait Selector<br/>Walk, Run, Climb]
    end

    subgraph "Mid-Level Control"
        WalkGen[Walking Pattern Generator<br/>ZMP, CoM Trajectories]
        BalanceCtrl[Balancing Controller<br/>Stability Maintenance]
        MPC[Model Predictive Control<br/>Optimization-Based]
    end

    subgraph "Low-Level Control"
        InvKinematics[Inverse Kinematics<br/>Joint Angle Commands]
        JointCtrl[Joint Controllers<br/>PID, Impedance]
        SafetyMon[Safety Monitor<br/>Limits, Emergency Stop]
    end

    subgraph "Sensors & Estimation"
        StateEst[State Estimator<br/>Position, Velocity, Contact]
        SensFusion[Sensor Fusion<br/>IMU, Force, Vision]
        EnvPercep[Environment Perception<br/>Terrain, Obstacles]
    end

    TaskPlanner --> WalkGen
    PathPlanner --> WalkGen
    GaitSelector --> WalkGen
    WalkGen --> BalanceCtrl
    WalkGen --> MPC
    BalanceCtrl --> InvKinematics
    MPC --> InvKinematics
    InvKinematics --> JointCtrl
    JointCtrl --> SafetyMon
    StateEst --> BalanceCtrl
    StateEst --> MPC
    SensFusion --> StateEst
    EnvPercep --> TaskPlanner
    EnvPercep --> PathPlanner

    style TaskPlanner fill:#b3e0ff
    style PathPlanner fill:#b3e0ff
    style GaitSelector fill:#b3e0ff
    style WalkGen fill:#c8e6c9
    style BalanceCtrl fill:#c8e6c9
    style MPC fill:#c8e6c9
    style InvKinematics fill:#e1bee7
    style JointCtrl fill:#e1bee7
    style SafetyMon fill:#ffccbc
    style StateEst fill:#fff9c4
    style SensFusion fill:#fff9c4
    style EnvPercep fill:#fff9c4
```

### 5.2 Whole-Body Control Framework
```mermaid
graph LR
    subgraph "Task Prioritization"
        Primary[Primary Tasks<br/>Balance, Safety]
        Secondary[Secondary Tasks<br/>Manipulation, Posture]
        Tertiary[Tertiary Tasks<br/>Comfort, Efficiency]
    end

    subgraph "Constraint Handling"
        JointLimits[Joint Limits<br/>Position, Velocity, Torque]
        CollisionAvoid[Collision Avoidance<br/>Self, Environment]
        DynamicCons[Dynamic Constraints<br/>ZMP, Momentum]
    end

    subgraph "Control Resolution"
        QPOptimizer[Quadratic Programming<br/>Optimization Solver]
        TaskWeights[Task Weights<br/>Priority Assignment]
        ConstraintMatrix[Constraint Matrix<br/>Mathematical Formulation]
    end

    subgraph "Output Generation"
        JointAccel[Joint Accelerations<br/>Resolved Motion]
        JointVel[Joint Velocities<br/>Integrated Motion]
        JointPos[Joint Positions<br/>Integrated Motion]
    end

    Primary --> QPOptimizer
    Secondary --> QPOptimizer
    Tertiary --> QPOptimizer
    JointLimits --> ConstraintMatrix
    CollisionAvoid --> ConstraintMatrix
    DynamicCons --> ConstraintMatrix
    ConstraintMatrix --> QPOptimizer
    TaskWeights --> QPOptimizer
    QPOptimizer --> JointAccel
    JointAccel --> JointVel
    JointVel --> JointPos

    style Primary fill:#b3e0ff
    style Secondary fill:#b3e0ff
    style Tertiary fill:#b3e0ff
    style JointLimits fill:#c8e6c9
    style CollisionAvoid fill:#c8e6c9
    style DynamicCons fill:#c8e6c9
    style QPOptimizer fill:#e1bee7
    style TaskWeights fill:#e1bee7
    style ConstraintMatrix fill:#e1bee7
    style JointAccel fill:#ffccbc
    style JointVel fill:#ffccbc
    style JointPos fill:#ffccbc
```

## 6. AI and Machine Learning Integration

### 6.1 Reinforcement Learning for Gait Optimization
```mermaid
graph TD
    subgraph "Environment"
        RobotState[Robot State<br/>Joint Angles, Velocities, IMU]
        RewardSignal[Reward Signal<br/>Balance, Speed, Efficiency]
        ActionSpace[Action Space<br/>Gait Parameters]
    end

    subgraph "RL Agent"
        PolicyNetwork[Policy Network<br/>Actor Network]
        ValueNetwork[Value Network<br/>Critic Network]
        ExperienceBuffer[Experience Buffer<br/>Replay Memory]
    end

    subgraph "Learning Process"
        ForwardPass[Forward Pass<br/>Action Selection]
        Backprop[Backpropagation<br/>Gradient Update]
        TargetUpdate[Target Network Update<br/>Stability]
    end

    subgraph "Gait Optimization"
        ParameterUpdate[Gait Parameter Update<br/>Step Length, Height, Timing]
        PerformanceEval[Performance Evaluation<br/>Stability, Efficiency]
        Adaptation[Adaptation<br/>Terrain, Load Changes]
    end

    RobotState --> PolicyNetwork
    PolicyNetwork --> ActionSpace
    ActionSpace --> RobotState
    RobotState --> RewardSignal
    RewardSignal --> ValueNetwork
    ValueNetwork --> Backprop
    PolicyNetwork --> Backprop
    ExperienceBuffer --> ForwardPass
    ForwardPass --> PolicyNetwork
    ForwardPass --> ValueNetwork
    Backprop --> TargetUpdate
    TargetUpdate --> PolicyNetwork
    TargetUpdate --> ValueNetwork
    Backprop --> ParameterUpdate
    PerformanceEval --> Adaptation
    ParameterUpdate --> PerformanceEval

    style RobotState fill:#b3e0ff
    style RewardSignal fill:#b3e0ff
    style ActionSpace fill:#b3e0ff
    style PolicyNetwork fill:#c8e6c9
    style ValueNetwork fill:#c8e6c9
    style ExperienceBuffer fill:#e1bee7
    style ForwardPass fill:#e1bee7
    style Backprop fill:#ffccbc
    style TargetUpdate fill:#ffccbc
    style ParameterUpdate fill:#fff9c4
    style PerformanceEval fill:#fff9c4
    style Adaptation fill:#fff9c4
```

### 6.2 Imitation Learning from Human Demonstrations
```mermaid
graph LR
    subgraph "Human Demonstration"
        MotionCapture[Motion Capture<br/>Human Movement Data]
        ForceMeasurement[Force Measurement<br/>Ground Reaction Forces]
        EMG[EMG Signals<br/>Muscle Activation]
    end

    subgraph "Data Processing"
        TrajectoryExtraction[Trajectory Extraction<br/>Joint Angle Sequences]
        FeatureExtraction[Feature Extraction<br/>Key Movement Features]
        Normalization[Normalization<br/>Scale, Timing Adjustment]
    end

    subgraph "Learning Algorithm"
        BehavioralCloning[Behavioral Cloning<br/>Direct Mapping]
        GAIL[Generative Adversarial Imitation Learning<br/>Distribution Matching]
        DAgger[Differentiable Aggregation<br/>Interactive Learning]
    end

    subgraph "Robot Execution"
        PolicyMapping[Policy Mapping<br/>Human to Robot Morphology]
        Adaptation[Adaptation<br/>Dynamics Compensation]
        Execution[Execution<br/>Real Robot Movement]
    end

    MotionCapture --> TrajectoryExtraction
    ForceMeasurement --> FeatureExtraction
    EMG --> FeatureExtraction
    TrajectoryExtraction --> Normalization
    FeatureExtraction --> Normalization
    Normalization --> BehavioralCloning
    Normalization --> GAIL
    Normalization --> DAgger
    BehavioralCloning --> PolicyMapping
    GAIL --> PolicyMapping
    DAgger --> PolicyMapping
    PolicyMapping --> Adaptation
    Adaptation --> Execution

    style MotionCapture fill:#b3e0ff
    style ForceMeasurement fill:#b3e0ff
    style EMG fill:#b3e0ff
    style TrajectoryExtraction fill:#c8e6c9
    style FeatureExtraction fill:#c8e6c9
    style Normalization fill:#c8e6c9
    style BehavioralCloning fill:#e1bee7
    style GAIL fill:#e1bee7
    style DAgger fill:#e1bee7
    style PolicyMapping fill:#ffccbc
    style Adaptation fill:#ffccbc
    style Execution fill:#fff9c4
```

## 7. Safety and Human-Robot Interaction

### 7.1 Safety Architecture for Humanoid Robots
```mermaid
graph TD
    subgraph "Safety Monitoring"
        StateMonitor[State Monitor<br/>Position, Velocity, Torque]
        ContactDetect[Contact Detection<br/>Unexpected Interaction]
        FallDetect[Fall Detection<br/>Orientation, Acceleration]
    end

    subgraph "Safety Responses"
        EmergencyStop[Emergency Stop<br/>Immediate Shutdown]
        ComplianceCtrl[Compliance Control<br/>Reduce Stiffness]
        SafePosture[Safe Posture<br/>Protective Position]
    end

    subgraph "Safety Boundaries"
        JointLimits[Joint Limits<br/>Position, Velocity, Torque]
        ForceLimits[Force Limits<br/>Contact Forces]
        WorkspaceLimits[Workspace Limits<br/>Reachable Area]
    end

    subgraph "Safety Validation"
        SimulationTest[Simulation Testing<br/>Virtual Validation]
        SafetyCert[Certification Process<br/>Standards Compliance]
        ContinuousMon[Continuous Monitoring<br/>Runtime Verification]
    end

    StateMonitor --> EmergencyStop
    ContactDetect --> ComplianceCtrl
    FallDetect --> SafePosture
    JointLimits --> StateMonitor
    ForceLimits --> ContactDetect
    WorkspaceLimits --> StateMonitor
    SimulationTest --> SafetyCert
    SafetyCert --> ContinuousMon
    ContinuousMon --> StateMonitor
    ContinuousMon --> ContactDetect
    ContinuousMon --> FallDetect

    style StateMonitor fill:#b3e0ff
    style ContactDetect fill:#b3e0ff
    style FallDetect fill:#b3e0ff
    style EmergencyStop fill:#c8e6c9
    style ComplianceCtrl fill:#c8e6c9
    style SafePosture fill:#c8e6c9
    style JointLimits fill:#e1bee7
    style ForceLimits fill:#e1bee7
    style WorkspaceLimits fill:#e1bee7
    style SimulationTest fill:#ffccbc
    style SafetyCert fill:#ffccbc
    style ContinuousMon fill:#ffccbc
```

### 7.2 Human-Robot Interaction Framework
```mermaid
graph LR
    subgraph "Human Perception"
        GestureRec[Gesture Recognition<br/>Hand, Body Movements]
        VoiceRec[Voice Recognition<br/>Commands, Questions]
        GazeTracking[Gaze Tracking<br/>Attention Direction]
    end

    subgraph "Robot Response"
        SocialRules[Social Rules<br/>Personal Space, Etiquette]
        SafeMotion[Safe Motion<br/>Predictable, Gentle]
        ExpressiveBehavior[Expressive Behavior<br/>Gestures, Sounds]
    end

    subgraph "Interaction Modalities"
        Physical[Physical Interaction<br/>Touch, Handshake]
        Verbal[Verbal Interaction<br/>Speech, Language]
        Nonverbal[Non-verbal Interaction<br/>Gestures, Expressions]
    end

    subgraph "Adaptation"
        UserModel[User Model<br/>Preferences, Abilities]
        ContextAware[Context Awareness<br/>Environment, Situation]
        Learning[Learning<br/>Improvement Over Time]
    end

    GestureRec --> SocialRules
    VoiceRec --> SafeMotion
    GazeTracking --> ExpressiveBehavior
    SocialRules --> Physical
    SafeMotion --> Verbal
    ExpressiveBehavior --> Nonverbal
    Physical --> UserModel
    Verbal --> ContextAware
    Nonverbal --> Learning
    UserModel --> SocialRules
    ContextAware --> SafeMotion
    Learning --> ExpressiveBehavior

    style GestureRec fill:#b3e0ff
    style VoiceRec fill:#b3e0ff
    style GazeTracking fill:#b3e0ff
    style SocialRules fill:#c8e6c9
    style SafeMotion fill:#c8e6c9
    style ExpressiveBehavior fill:#c8e6c9
    style Physical fill:#e1bee7
    style Verbal fill:#e1bee7
    style Nonverbal fill:#e1bee7
    style UserModel fill:#ffccbc
    style ContextAware fill:#ffccbc
    style Learning fill:#ffccbc
```

These diagrams provide visual representations of key humanoid robotics concepts, including robot architecture, balance control systems, walking gait generation, control architectures, AI integration, and safety frameworks, helping to understand the complex relationships between different components in humanoid robot systems.