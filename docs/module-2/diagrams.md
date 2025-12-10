# Diagrams: ROS 2 Fundamentals for Physical AI

## 1. ROS 2 Architecture Overview

### 1.1 ROS 2 System Architecture
```mermaid
graph TB
    subgraph "ROS 2 Ecosystem"
        subgraph "Client Libraries"
            CL1[rclcpp<br/>C++ Client Library]
            CL2[rclpy<br/>Python Client Library]
            CL3[rcl<br/>Common Client Library]
        end

        subgraph "DDS Implementation"
            DDS1[Fast DDS]
            DDS2[Cyclone DDS]
            DDS3[RTI Connext DDS]
        end

        subgraph "ROS 2 Core"
            RMW[rclcpp/rclpy<br/>Middleware Wrappers]
            RCL[rcl<br/>ROS Client Library]
        end
    end

    subgraph "User Applications"
        Node1[ROS 2 Node 1]
        Node2[ROS 2 Node 2]
        Node3[ROS 2 Node 3]
    end

    Node1 --> CL2
    Node2 --> CL1
    Node3 --> CL2

    CL1 --> RMW
    CL2 --> RMW
    CL3 --> RMW

    RMW --> RCL
    RCL --> DDS1
    RCL --> DDS2
    RCL --> DDS3

    style CL1 fill:#b3e0ff
    style CL2 fill:#b3e0ff
    style CL3 fill:#b3e0ff
    style DDS1 fill:#ffccbc
    style DDS2 fill:#ffccbc
    style DDS3 fill:#ffccbc
    style RMW fill:#c8e6c9
    style RCL fill:#c8e6c9
    style Node1 fill:#fff9c4
    style Node2 fill:#fff9c4
    style Node3 fill:#fff9c4
```

### 1.2 Node Communication Architecture
```mermaid
graph LR
    subgraph "ROS 2 Network"
        NodeA[Node A<br/>Publisher]
        NodeB[Node B<br/>Subscriber]
        NodeC[Node C<br/>Service Server]
        NodeD[Node D<br/>Service Client]
        NodeE[Node E<br/>Action Server]
        NodeF[Node F<br/>Action Client]
    end

    subgraph "DDS Layer"
        DDS[DDS<br/>Data Distribution Service]
    end

    NodeA --> DDS
    NodeB --> DDS
    NodeC --> DDS
    NodeD --> DDS
    NodeE --> DDS
    NodeF --> DDS

    style NodeA fill:#a8e6cf
    style NodeB fill:#a8e6cf
    style NodeC fill:#ffccbc
    style NodeD fill:#ffccbc
    style NodeE fill:#c8e6c9
    style NodeF fill:#c8e6c9
    style DDS fill:#d1c4e9
```

## 2. Communication Patterns

### 2.1 Publisher-Subscriber Pattern
```mermaid
graph LR
    subgraph "Publisher Node"
        P[Publisher<br/>/sensor_data]
    end

    subgraph "DDS Middleware"
        DDS[DDS Network<br/>Topic-based Communication]
    end

    subgraph "Subscriber Nodes"
        S1[Subscriber 1<br/>/sensor_data]
        S2[Subscriber 2<br/>/sensor_data]
        S3[Subscriber 3<br/>/sensor_data]
    end

    P --> DDS
    DDS --> S1
    DDS --> S2
    DDS --> S3

    style P fill:#b3e0ff
    style DDS fill:#e1bee7
    style S1 fill:#ffccbc
    style S2 fill:#ffccbc
    style S3 fill:#ffccbc
```

### 2.2 Service Request-Response Pattern
```mermaid
graph LR
    subgraph "Service Client"
        Client[Client Node<br/>Calls /safety_check]
    end

    subgraph "DDS Middleware"
        DDS[DDS Network<br/>Service Communication]
    end

    subgraph "Service Server"
        Server[Server Node<br/>Provides /safety_check]
    end

    Client -->|Request| DDS
    DDS -->|Request| Server
    Server -->|Response| DDS
    DDS -->|Response| Client

    style Client fill:#b3e0ff
    style Server fill:#c8e6c9
    style DDS fill:#e1bee7
```

### 2.3 Action Goal-Feedback-Result Pattern
```mermaid
graph TD
    subgraph "Action Client"
        AC[Client Node<br/>Sends Goal to /navigate_to_pose]
    end

    subgraph "DDS Middleware"
        DDS[DDS Network<br/>Action Communication]
    end

    subgraph "Action Server"
        AS[Server Node<br/>Handles /navigate_to_pose]
    end

    AC -->|Goal| DDS
    DDS -->|Goal| AS
    AS -->|Feedback| DDS
    DDS -->|Feedback| AC
    AS -->|Result| DDS
    DDS -->|Result| AC

    style AC fill:#b3e0ff
    style AS fill:#c8e6c9
    style DDS fill:#e1bee7
```

## 3. Quality of Service (QoS) Policies

### 3.1 Reliability Policy Comparison
```mermaid
graph LR
    subgraph "Reliable Policy"
        ReliableP[Publisher<br/>All messages delivered]
        ReliableS[Subscriber<br/>All messages received]
    end

    subgraph "Best Effort Policy"
        BestEffortP[Publisher<br/>Messages may be lost]
        BestEffortS[Subscriber<br/>Only received messages]
    end

    ReliableP --> ReliableS
    BestEffortP --> BestEffortS

    style ReliableP fill:#c8e6c9
    style ReliableS fill:#c8e6c9
    style BestEffortP fill:#ffccbc
    style BestEffortS fill:#ffccbc
```

### 3.2 Durability Policy Comparison
```mermaid
graph LR
    subgraph "Transient Local"
        TLP[Publisher<br/>Stores last message]
        TLS[Subscriber<br/>Receives last message immediately]
    end

    subgraph "Volatile"
        VP[Publisher<br/>No message storage]
        VS[Subscriber<br/>Only receives future messages]
    end

    TLP --> TLS
    VP --> VS

    style TLP fill:#c8e6c9
    style TLS fill:#c8e6c9
    style VP fill:#ffccbc
    style VS fill:#ffccbc
```

## 4. Package Structure

### 4.1 ROS 2 Package Organization
```mermaid
graph TD
    subgraph "ROS 2 Package"
        A[package.xml<br/>Package metadata]
        B[CMakeLists.txt<br/>Build configuration]
        C[src/<br/>Source code files]
        D[include/<br/>Header files]
        E[scripts/<br/>Python scripts]
        F[launch/<br/>Launch files]
        G[config/<br/>Configuration files]
        H[test/<br/>Unit tests]
        I[msg/<br/>Custom message definitions]
    end

    A -.-> C
    A -.-> D
    A -.-> E
    A -.-> F
    A -.-> G
    A -.-> H
    A -.-> I

    style A fill:#b3e0ff
    style B fill:#b3e0ff
    style C fill:#dcedc8
    style D fill:#dcedc8
    style E fill:#fff9c4
    style F fill:#e1bee7
    style G fill:#ffccbc
    style H fill:#ffcdd2
    style I fill:#c8e6c9
```

## 5. Launch System Architecture

### 5.1 Launch File Components
```mermaid
graph TB
    subgraph "Launch File"
        A[LaunchDescription<br/>Top-level container]
        B[DeclareLaunchArgument<br/>Parameter definitions]
        C[Node<br/>ROS 2 node definitions]
        D[ExecuteProcess<br/>External processes]
        E[RegisterEventHandler<br/>Event handling]
    end

    subgraph "Launch System"
        F[LaunchService<br/>Launch service]
        G[LaunchContext<br/>Context management]
        H[LaunchIntrospector<br/>System introspection]
    end

    A --> B
    A --> C
    A --> D
    A --> E
    A --> F
    F --> G
    F --> H

    style A fill:#b3e0ff
    style B fill:#dcedc8
    style C fill:#fff9c4
    style D fill:#e1bee7
    style E fill:#ffccbc
    style F fill:#c8e6c9
    style G fill:#d1c4e9
    style H fill:#ffab91
```

## 6. Physical AI System Architecture

### 6.1 Complete Physical AI System with ROS 2
```mermaid
graph TB
    subgraph "Physical AI System"
        subgraph "Perception Layer"
            Cam[Camera Node<br/>sensor_msgs/Image]
            Laser[Laser Scanner Node<br/>sensor_msgs/LaserScan]
            IMU[IMU Node<br/>sensor_msgs/Imu]
            Joint[Joint State Node<br/>sensor_msgs/JointState]
        end

        subgraph "Processing Layer"
            Perception[Perception Node<br/>Object Detection, SLAM]
            Planning[Planning Node<br/>Path Planning, Task Planning]
            Control[Control Node<br/>Motion Control]
        end

        subgraph "Safety Layer"
            SafetyMonitor[Safety Monitor<br/>Emergency Stop Logic]
            SafetyControl[Safety Controller<br/>Safe Commands]
        end

        subgraph "Execution Layer"
            CmdVel[Command Velocity<br/>geometry_msgs/Twist]
            Robot[Physical Robot<br/>Hardware Interface]
        end
    end

    subgraph "ROS 2 Communication"
        DDS[DDS Middleware<br/>Topic/Service/Action Communication]
    end

    Cam --> DDS
    Laser --> DDS
    IMU --> DDS
    Joint --> DDS

    DDS --> Perception
    Perception --> DDS
    DDS --> Planning
    Planning --> DDS
    DDS --> Control
    Control --> DDS

    DDS --> SafetyMonitor
    SafetyMonitor --> DDS
    DDS --> SafetyControl
    SafetyControl --> DDS

    DDS --> CmdVel
    CmdVel --> Robot

    style Cam fill:#a8e6cf
    style Laser fill:#a8e6cf
    style IMU fill:#a8e6cf
    style Joint fill:#a8e6cf
    style Perception fill:#fff9c4
    style Planning fill:#fff9c4
    style Control fill:#fff9c4
    style SafetyMonitor fill:#ffccbc
    style SafetyControl fill:#ffccbc
    style CmdVel fill:#c8e6c9
    style Robot fill:#d1c4e9
    style DDS fill:#e1bee7
```

## 7. Parameter Management

### 7.1 Parameter Server Architecture
```mermaid
graph LR
    subgraph "Parameter Server"
        PS[Parameter Server<br/>Centralized parameter management]
    end

    subgraph "Nodes with Parameters"
        Node1[Node 1<br/>param1, param2]
        Node2[Node 2<br/>param3, param4]
        Node3[Node 3<br/>param5, param6]
    end

    subgraph "Launch Configuration"
        Launch[Launch File<br/>Parameter values]
    end

    Launch --> PS
    PS --> Node1
    PS --> Node2
    PS --> Node3

    style PS fill:#b3e0ff
    style Node1 fill:#ffccbc
    style Node2 fill:#ffccbc
    style Node3 fill:#ffccbc
    style Launch fill:#c8e6c9
```

## 8. Tools and Ecosystem

### 8.1 ROS 2 Tool Ecosystem
```mermaid
graph TB
    subgraph "Development Tools"
        A[ros2 topic<br/>Topic inspection]
        B[ros2 service<br/>Service inspection]
        C[ros2 action<br/>Action inspection]
        D[ros2 node<br/>Node management]
        E[ros2 param<br/>Parameter management]
        F[ros2 bag<br/>Data recording]
        G[ros2 launch<br/>System startup]
    end

    subgraph "Visualization Tools"
        H[rqt_graph<br/>System visualization]
        I[rqt_plot<br/>Data plotting]
        J[rqt_console<br/>Log viewing]
        K[rqt_bag<br/>Bag file viewer]
    end

    subgraph "System Tools"
        L[ros2 doctor<br/>System health]
        M[ros2 multicast<br/>Network diagnostics]
        N[ros2 lifecycle<br/>Node lifecycle]
    end

    style A fill:#b3e0ff
    style B fill:#b3e0ff
    style C fill:#b3e0ff
    style D fill:#b3e0ff
    style E fill:#b3e0ff
    style F fill:#b3e0ff
    style G fill:#b3e0ff
    style H fill:#c8e6c9
    style I fill:#c8e6c9
    style J fill:#c8e6c9
    style K fill:#c8e6c9
    style L fill:#ffccbc
    style M fill:#ffccbc
    style N fill:#ffccbc
```

## 9. Safety Architecture

### 9.1 Multi-layer Safety System with ROS 2
```mermaid
graph TB
    subgraph "Physical AI Application"
        App[Application Nodes<br/>Navigation, Manipulation, etc.]
    end

    subgraph "Safety Monitoring Layer"
        Monitor[ROS 2 Safety Monitor<br/>Obstacle detection, velocity limits]
        Checker[Safety Checker<br/>Rule validation]
    end

    subgraph "Safety Enforcement Layer"
        Supervisor[Safety Supervisor<br/>Emergency stop logic]
        Limiter[Velocity Limiter<br/>Command limiting]
    end

    subgraph "Hardware Safety Layer"
        EStop[Hardware Emergency Stop<br/>Physical safety circuit]
        Robot[Robot Controller<br/>Hardware safety features]
    end

    App --> Monitor
    Monitor --> Checker
    Checker --> Supervisor
    Supervisor --> Limiter
    Limiter --> Robot
    Robot --> EStop

    style App fill:#b3e0ff
    style Monitor fill:#c8e6c9
    style Checker fill:#c8e6c9
    style Supervisor fill:#ffccbc
    style Limiter fill:#ffccbc
    style Robot fill:#d1c4e9
    style EStop fill:#f44336
```

These diagrams provide visual representations of key ROS 2 concepts and architectures relevant to Physical AI systems, helping to understand the communication patterns, system architecture, and safety considerations involved in ROS 2-based Physical AI applications.