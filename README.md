# Robotics Final Project - Advanced Behavior Tree Control System

> **Core File Location:**
> 
> The main control file, `bt_students.py`, is located at:
> 
> **`robotics_project/scripts/behaviour_trees/bt_students.py`**

## Overview

This project implements a sophisticated **Behavior Tree (BT)** control system for autonomous robot navigation and manipulation tasks. The core of this system is the `bt_students.py` file, which serves as the **central brain** of the robot, orchestrating complex pick-and-place operations through intelligent decision-making and robust error recovery mechanisms.

## What is a Behavior Tree?

A **Behavior Tree** is a mathematical model for plan execution used in AI, robotics, and game development. Unlike traditional finite state machines, behavior trees provide:

- **Hierarchical Structure**: Complex behaviors built from simpler, reusable components
- **Reactive Execution**: Real-time response to environmental changes and failures
- **Modularity**: Each behavior node is self-contained and easily testable
- **Robustness**: Built-in error handling and recovery mechanisms

In this project, the behavior tree enables the robot to make intelligent decisions, handle unexpected situations, and maintain task progress even when facing navigation failures or localization issues.

## 🎯 Core Control File: `bt_students.py`

The `bt_students.py` file is the **heart of the entire robotics system**. This 950-line implementation contains the complete control logic that transforms a TIAGo robot into an intelligent autonomous agent capable of:

- **Autonomous Navigation** with kidnapping detection and recovery
- **Object Detection** using ArUco markers and computer vision
- **Manipulation Tasks** including pick and place operations
- **Intelligent Error Recovery** with state persistence and task resumption
- **Centralized State Management** for coordinated system behavior

### File Location

> **Path:** `robotics_project/scripts/behaviour_trees/bt_students.py`

### System Architecture

The behavior tree follows a hierarchical structure designed for robustness and maintainability:

```
🤖 bt_students.py - MAIN CONTROL SYSTEM
├── 🧠 StateManager - Centralized state coordination
├── 🌳 BehaviourTree - Main behavior tree controller
├── 📍 Navigation System - Robust movement with error recovery
├── 👁️ Detection System - ArUco-based object recognition
├── 🦾 Manipulation System - Pick and place operations
└── 🛡️ Recovery System - Error handling and task resumption
```

## 🏗️ Behavior Tree Structure

### Main Sequence Flow

The primary task execution follows this intelligent sequence:

```
Main Sequence
├── 🚨 Kidnapped Check & Recovery
├── ⚠️ Navigation Failure Check & Recovery
├── 🔧 Arm Tucking (Safety positioning)
├── 👆 Head Up (Wide field of view)
├── 📍 Localization & Navigation to Pick
├── 👇 Head Down (Object detection view)
├── 👁️ Object Detection (ArUco markers)
├── 🤏 Pick Object
├── 📍 Localization & Navigation to Place
├── 🤲 Place Object
└── ✅ Task Verification
```

### Node Types and Functions

#### 🧠 **Composite Nodes**
- **Selector (Fallback)**: Executes children until one succeeds - used for graceful degradation
- **Sequence**: Executes children in order until one fails - used for step-by-step tasks
- **Reactive Sequence**: Enhanced sequence with continuous monitoring

#### 🎯 **Action Nodes**
- **Navigation**: Move_base integration with kidnapping detection
- **Detection**: ArUco marker recognition with TF transformation
- **Manipulation**: Service-based pick/place operations
- **Localization**: AMCL-based robot positioning with spin recovery

#### 🔍 **Condition Nodes**
- **StateCheck**: Verify single state conditions
- **StateChecker**: Advanced state verification with custom messages
- **CheckRobotState**: Multi-condition state validation

## 🔑 Key Components Analysis

### 1. StateManager Class
**Purpose**: Centralized state coordination across all system components

**Key States**:
```python
{
    "arm_tucked": False,           # Arm safety position
    "head_state": "unknown",       # Head orientation (up/down)
    "detected": False,             # Object detection status
    "picked": False,               # Pick operation completion
    "placed": False,               # Place operation completion
    "localized": False,            # Robot localization status
    "navigated_pick": False,       # Navigation to pick location
    "navigated_place": False,      # Navigation to place location
    "kidnapped": False,            # Localization failure detection
    "navigation_failed": False     # Navigation failure flag
}
```

### 2. Intelligent Error Recovery

#### Kidnapping Recovery System
**Scenario**: Robot loses localization during navigation (covariance > 0.07)
**Response**:
1. Immediately stop robot movement
2. Cancel current navigation goals
3. Initiate re-localization sequence (robot spinning)
4. Resume task from appropriate checkpoint

#### Navigation Failure Recovery
**Scenario**: Move_base fails to reach target destination
**Response**:
1. Reset navigation failure state
2. Re-execute localization
3. Retry navigation with fresh path planning
4. Maintain task progress without full restart

### 3. Advanced Navigation System
- **Real-time Kidnapping Detection**: Monitors AMCL covariance during movement
- **Graceful Failure Handling**: Preserves task state during recovery
- **Multi-goal Support**: Handles both pick and place destinations
- **Safety Integration**: Immediate stop on localization loss

### 4. Robust Detection System
- **ArUco Marker Recognition**: `/robotics_intro/aruco_single/pose`
- **TF Transformation**: Converts sensor frame to robot base frame
- **Failure Recovery**: Resets navigation state on detection failure
- **Pose Publishing**: Provides transformed pose for manipulation

### 5. Modular Manipulation System
- **CubeManipulator Base Class**: Generic pick/place operations
- **Service Integration**: Uses ROS services for manipulation commands
- **State Synchronization**: Updates global state after successful operations
- **Error Handling**: Provides detailed feedback on operation status

## 🚀 Running the System

### Prerequisites
```bash
# ROS Noetic environment
source /opt/ros/noetic/setup.bash

# Project workspace
cd /path/to/robotics_project
source devel/setup.bash
```

### Launch Sequence

1. **Start Gazebo Simulation Environment**:
```bash
roslaunch tiago_gazebo tiago_gazebo.launch world:=final_project
```

2. **Launch Navigation Stack**:
```bash
roslaunch tiago_2dnav_gazebo tiago_navigation.launch
```

3. **Run the Core Behavior Tree Controller**:
```bash
rosrun robotics_project bt_students.py
# (File path: robotics_project/scripts/behaviour_trees/bt_students.py)
```

### Alternative: Complete System Launch
```bash
# Single command to launch everything
roslaunch robotics_project launch_project.launch
```

### Required Dependencies
- **ROS Packages**: `py_trees`, `py_trees_ros`, `move_base`, `amcl`
- **Hardware Simulation**: `tiago_gazebo`, `tiago_description`
- **Navigation**: `tiago_navigation`, `tiago_2dnav_gazebo`
- **Computer Vision**: `aruco_ros`, `tf2_geometry_msgs`

## 🎛️ Configuration Parameters

The behavior tree uses ROS parameters for flexible configuration:

```yaml
# Navigation topics
amcl_estimate: "/amcl_pose"
pick_pose_topic: "/pick_pose"
place_pose_topic: "/place_pose"
cmd_vel_topic: "/mobile_base_controller/cmd_vel"

# Services
move_head_srv: "/head_controller/move_head"
pick_srv: "/pick_gui"
place_srv: "/place_gui"

# Robot configuration
robot_base_frame: "base_footprint"
aruco_pose_topic: "/aruco_single/pose"
```

## 🧪 Testing and Verification

### System Validation
1. **Localization Test**: Verify AMCL convergence (covariance < 0.02)
2. **Navigation Test**: Confirm successful path planning and execution
3. **Detection Test**: Validate ArUco marker recognition and pose transformation
4. **Manipulation Test**: Test pick and place service integration
5. **Recovery Test**: Simulate kidnapping and navigation failures

### Performance Monitoring
```bash
# Monitor behavior tree execution
rostopic echo /bt_students/status

# Check state transitions
rostopic echo /bt_students/state_updates

# Verify navigation performance
rostopic echo /move_base/status
```

## 🌟 Advanced Features

### Reactive Behavior Execution
- **Continuous Monitoring**: Real-time environment assessment
- **Dynamic Replanning**: Adaptive response to changing conditions
- **State Persistence**: Maintains progress through failures

### Intelligent Task Resumption
- **Checkpoint System**: Remembers task progress during recovery
- **Context-Aware Recovery**: Different recovery strategies based on current state
- **Minimal Redundancy**: Avoids repeating completed sub-tasks

### Modular Architecture
- **Plugin-Based Design**: Easy addition of new behaviors
- **Hierarchical Composition**: Complex behaviors from simple primitives
- **Reusable Components**: Shared behaviors across different tasks

## 🎯 Why bt_students.py is the Project Core

1. **Central Intelligence**: Contains all decision-making logic and task coordination
2. **System Integration**: Interfaces with navigation, manipulation, and perception systems
3. **Robust Control**: Implements advanced error recovery and fault tolerance
4. **Scalable Design**: Modular architecture allows easy extension and modification
5. **Real-world Ready**: Production-quality implementation with comprehensive error handling

The `bt_students.py` file transforms a standard TIAGo robot into an intelligent autonomous system capable of complex task execution with human-level robustness and adaptability.

---

## 📝 Quick Start Summary

```bash
# 1. Launch simulation
roslaunch tiago_gazebo tiago_gazebo.launch world:=final_project

# 2. Start navigation
roslaunch tiago_2dnav_gazebo tiago_navigation.launch

# 3. Run the core behavior tree (THE BRAIN!)
rosrun robotics_project bt_students.py
# (File path: robotics_project/scripts/behaviour_trees/bt_students.py)
```

**The magic happens in `bt_students.py` - this single file orchestrates the entire robotic system! 🤖✨**
