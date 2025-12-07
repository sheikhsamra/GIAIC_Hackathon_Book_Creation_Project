# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Prerequisites

Before starting with the textbook, ensure you have:

1. **System Requirements**:
   - Ubuntu 22.04 LTS (recommended for ROS 2 Humble)
   - At least 8GB RAM (16GB recommended for simulation)
   - 50GB free disk space for all tools
   - Modern CPU with hardware virtualization support
   - Graphics card with OpenGL 4.3+ support (for Isaac Sim)

2. **Software Requirements**:
   - ROS 2 Humble Hawksbill installed
   - Gazebo Garden or newer
   - NVIDIA Isaac Sim (if using Isaac platform content)
   - Python 3.10 or 3.11
   - Node.js 18+ and npm/yarn
   - Git
   - Docker (for containerized examples)

## Setting Up Your Environment

### 1. Install ROS 2 Humble
```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2 python3-colcon-common-extensions
sudo rosdep init
rosdep update

# Source ROS 2 in your bashrc to make it available by default
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Gazebo
```bash
curl -sSL http://get.gazebosim.org | sh
```

### 3. Install Isaac Sim (Optional)
Follow NVIDIA's official installation guide for Isaac Sim. Requires NVIDIA GPU with CUDA support.

### 4. Install Node.js and Dependencies
```bash
# Using NodeSource repository (recommended)
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs

# Verify installation
node --version
npm --version
```

### 5. Set Up Textbook Repository
```bash
git clone [repository-url]
cd [repository-name]
source /opt/ros/humble/setup.bash
```

## Getting Started with the Textbook

### 1. Navigate to the Textbook
The textbook is organized in the `docs/` directory with modules:
- `docs/module-1/` - Introduction to Physical AI
- `docs/module-2/` - ROS 2 Fundamentals
- `docs/module-3/` - Gazebo Simulation
- `docs/module-4/` - NVIDIA Isaac Platform
- `docs/module-5/` - Humanoid Robotics
- `docs/module-6/` - Vision-Language-Action (VLA)
- `docs/capstone/` - Capstone Project

### 2. Building and Running the Documentation
```bash
cd [repository-root]
npm install  # Install Docusaurus dependencies
npm run start  # Start development server (default: http://localhost:3000)
```

### 3. Working with Code Examples
Each module contains code examples. To run a Python example:

```bash
source /opt/ros/humble/setup.bash
cd [path-to-example]
python3 [example-file].py
```

### 4. Running Simulation Examples

For Gazebo examples:
```bash
source /opt/ros/humble/setup.bash
# Build the workspace if needed
colcon build --packages-select [package-name]
source install/setup.bash
ros2 launch [package-name] [launch-file].py
```

For Isaac Sim examples:
```bash
# Launch Isaac Sim first, then run Python scripts
source /opt/ros/humble/setup.bash
python3 [isaac-example-script].py
```

### 5. Using the RAG Chatbot
The RAG chatbot is accessible through the documentation interface. To ask questions:
1. Navigate to the chat interface in the textbook
2. Ask questions related to the textbook content
3. The chatbot will respond based only on textbook content

## Personalization Options

The textbook offers content adapted for different experience levels:
- **Beginner**: More detailed explanations, step-by-step instructions, additional examples
- **Intermediate**: Concise explanations with focus on key concepts
- **Advanced**: Brief explanations with emphasis on implementation details
- **Robotics Professional**: Advanced concepts and optimization techniques

To set your experience level:
1. Navigate to the user profile section
2. Select your preferred experience level
3. The content will automatically adjust to your preference

## Multi-language Support

The textbook is available in both English and Urdu:
- English: Default language
- Urdu: Available as alternate language option

Technical terms (ROS 2, Gazebo, Isaac Sim, VSLAM, etc.) are preserved in English across all translations.

To switch languages:
1. Use the language selector in the top navigation bar
2. The entire textbook interface will change to the selected language

## Safety Guidelines

⚠️ **IMPORTANT SAFETY NOTES**:
- Always test code in simulation before attempting on physical robots
- Use appropriate velocity and torque limits (never exceed 50% of maximum initially)
- Implement safety checks in all robot control code
- Follow ROS 2 best practices for robot safety
- Include emergency stop mechanisms in all control code
- Verify all parameters are within safe operating ranges before execution

## Development Workflow

### For Contributors
1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Make your changes
4. Test thoroughly in simulation
5. Commit your changes: `git commit -m 'Add some amazing feature'`
6. Push to the branch: `git push origin feature/amazing-feature`
7. Open a Pull Request

### Content Validation
All content must pass validation against the project constitution:
- Technical accuracy verified
- Safety guidelines followed
- Proper ROS 2/Isaac APIs used
- No hallucinated methods or parameters

## Troubleshooting

### Common Issues
1. **ROS 2 Commands Not Found**: Ensure you've sourced the ROS 2 setup file
2. **Gazebo Not Launching**: Check OpenGL support and graphics drivers
3. **Isaac Sim Errors**: Verify CUDA compatibility and GPU support
4. **Docusaurus Build Failures**: Check Node.js version compatibility

### Getting Help
- Check the FAQ section in each module
- Use the RAG chatbot with questions about textbook content
- Refer to official documentation:
  - ROS 2 Humble: https://docs.ros.org/en/humble/
  - Gazebo: https://gazebosim.org/docs/
  - Isaac ROS: https://nvidia-isaac-ros.github.io/
- Join the community forum for discussions
- Report issues on the GitHub repository

## Next Steps

1. Complete the Introduction to Physical AI module (Module 1)
2. Progress through the modules in order for optimal learning
3. Complete the capstone project to integrate all concepts
4. Use the RAG chatbot for questions as you progress
5. Experiment with the code examples in simulation
6. Consider the personalization options to match your skill level