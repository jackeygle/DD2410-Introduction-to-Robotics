# 🛠️ Setup and Installation Guide

## 📋 Prerequisites

### System Requirements
- **Operating System**: Ubuntu 18.04 LTS (recommended)
- **ROS Version**: ROS Melodic Morenia
- **Python**: Python 3.6+
- **Gazebo**: Version 9.0+
- **Memory**: 8GB RAM minimum, 16GB recommended
- **Storage**: 20GB free space

### Hardware Requirements
- **CPU**: Intel i5 or equivalent (multi-core recommended)
- **GPU**: NVIDIA GPU with CUDA support (optional, for enhanced visualization)
- **Network**: Stable internet connection for package downloads

## 🚀 Installation Steps

### Step 1: ROS Melodic Installation

```bash
# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update package index
sudo apt update

# Install ROS Melodic Desktop Full
sudo apt install ros-melodic-desktop-full

# Initialize rosdep
sudo rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: TIAGo Simulation Dependencies

```bash
# Install TIAGo simulation packages
sudo apt install ros-melodic-tiago-simulation
sudo apt install ros-melodic-tiago-desktop

# Install MoveIt! for motion planning
sudo apt install ros-melodic-moveit
sudo apt install ros-melodic-moveit-planners

# Install additional navigation packages
sudo apt install ros-melodic-navigation
sudo apt install ros-melodic-gmapping
sudo apt install ros-melodic-amcl
```

### Step 3: Workspace Setup

```bash
# Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone the project repository
git clone https://github.com/jackeygle/DD2410-Introduction-to-Robotics.git

# Clone required dependencies
git clone https://github.com/kth-ros-pkg/Robotics_intro.git

# Build the workspace
cd ~/catkin_ws
catkin_make -DCATKIN_ENABLE_TESTING=0

# Source the workspace
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 4: Python Dependencies

```bash
# Install Python packages
pip3 install numpy scipy matplotlib

# Install behavior tree library (if used)
pip3 install py-trees

# Install additional robotics libraries
pip3 install transforms3d
pip3 install rospkg
```

## 🔧 Configuration

### Environment Variables
Add these to your `~/.bashrc`:

```bash
# ROS configuration
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# Gazebo configuration
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/Robotics_intro/tiago_gazebo/models

# TIAGo configuration
export TIAGO_ROBOT=tiago-steel
```

### Gazebo Performance Optimization

```bash
# For better performance, you can disable GUI
export GAZEBO_GUI=false

# Reduce physics update rate if needed
export GAZEBO_PHYSICS_UPDATE_RATE=100
```

## 🚀 Quick Verification

### Test ROS Installation
```bash
# Terminal 1: Start roscore
roscore

# Terminal 2: Check ROS topics
rostopic list

# Terminal 3: Test basic functionality
rosrun turtlesim turtlesim_node
```

### Test TIAGo Simulation
```bash
# Launch basic TIAGo simulation
roslaunch tiago_gazebo tiago_gazebo.launch world:=small_office

# In another terminal, check robot topics
rostopic list | grep tiago
```

## 🐛 Troubleshooting

### Common Issues

#### Issue 1: Package Not Found
```bash
# Symptoms
ERROR: cannot launch node of type [package_name/node_name]: package_name

# Solution
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

#### Issue 2: Gazebo Crashes
```bash
# Symptoms
Gazebo crashes or fails to start

# Solutions
1. Check available memory: free -h
2. Restart Gazebo: killall gazebo && killall gzserver
3. Clear Gazebo cache: rm -rf ~/.gazebo/log/*
```

#### Issue 3: TIAGo Model Not Loading
```bash
# Symptoms
Robot model appears as white/gray boxes

# Solution
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/Robotics_intro/tiago_gazebo/models
source ~/.bashrc
```

#### Issue 4: Python Import Errors
```bash
# Symptoms
ImportError: No module named 'module_name'

# Solution
pip3 install --user module_name
# or
sudo apt install python3-module-name
```

### Performance Issues

#### Low FPS in Gazebo
- Reduce world complexity
- Disable shadows: `export GAZEBO_ENABLE_SHADOWS=false`
- Lower physics update rate
- Use headless mode for pure simulation

#### High CPU Usage
- Monitor with: `htop`
- Reduce sensor frequency in launch files
- Limit number of concurrent processes

## 📊 System Validation

### Hardware Check
```bash
# Check CPU cores
nproc

# Check memory
free -h

# Check GPU (if available)
nvidia-smi
```

### Software Check
```bash
# Check ROS installation
rosversion -d

# Check Python version
python3 --version

# Check Gazebo version
gazebo --version
```

### Network Check
```bash
# Check ROS networking
rostopic list
rosnode list

# Test inter-node communication
rostopic echo /clock
```

## 🎯 Next Steps

After successful installation:

1. **Run the main project**: Follow instructions in main README
2. **Explore examples**: Check the demos/ directory
3. **Read documentation**: Review docs/ for detailed explanations
4. **Customize configuration**: Modify config files as needed

## 📚 Additional Resources

- [ROS Melodic Documentation](http://wiki.ros.org/melodic)
- [TIAGo Tutorials](http://wiki.ros.org/Robots/TIAGo)
- [Gazebo Documentation](http://gazebosim.org/documentation)
- [MoveIt! Tutorials](https://moveit.ros.org/documentation/tutorials/)

## 🆘 Getting Help

If you encounter issues not covered here:

1. Check the [GitHub Issues](https://github.com/jackeygle/DD2410-Introduction-to-Robotics/issues)
2. Consult ROS community forums
3. Review TIAGo documentation
4. Contact course instructors (for academic use)

---

**Estimated Setup Time**: 2-3 hours for first-time installation 