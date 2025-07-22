# Robotics Final Project - Behavior Tree Implementation

This repository contains a sophisticated behavior tree implementation for autonomous robot navigation and manipulation tasks, specifically designed for pick-and-place operations in a simulated environment.

## Table of Contents
- [Overview](#overview)
- [Behavior Tree Architecture](#behavior-tree-architecture)
- [Key Features](#key-features)
- [System Components](#system-components)
- [Installation](#installation)
- [Usage](#usage)
- [Behavior Tree Structure](#behavior-tree-structure)
- [State Management](#state-management)
- [Error Handling and Recovery](#error-handling-and-recovery)
- [Testing](#testing)
- [Contributing](#contributing)

## Overview

This project implements a robust behavior tree system for controlling a mobile manipulator robot in Gazebo simulation. The robot performs autonomous pick-and-place operations involving object detection, navigation, manipulation, and error recovery.

### What is a Behavior Tree?

A behavior tree is a mathematical model of plan execution used in computer science, robotics, and control systems. It provides:

- **Modularity**: Each behavior is self-contained and reusable
- **Hierarchical Structure**: Complex behaviors built from simpler ones
- **Reactive Execution**: Real-time response to environmental changes
- **Robust Error Handling**: Systematic failure recovery mechanisms

## Behavior Tree Architecture

### Core Design Principles

1. **State-Driven Execution**: Central state manager coordinates all behaviors
2. **Fault Tolerance**: Comprehensive error detection and recovery
3. **Modular Design**: Reusable behavior components
4. **Real-time Responsiveness**: Continuous environment monitoring

### Main Behavior Classes

- **BehaviourTree**: Main coordinator implementing the primary task sequence
- **StateManager**: Centralized state management system
- **Navigation**: Robust navigation with kidnapping detection
- **CubeManipulator**: Generic manipulation behaviors (pick/place)
- **Localization**: AMCL-based robot localization
- **Detection**: ArUco marker-based object detection

## Key Features

### 🤖 Autonomous Navigation
- Move_base integration for path planning
- Real-time localization monitoring
- Kidnapping detection and recovery
- Navigation failure handling

### 🎯 Object Detection
- ArUco marker recognition
- TF transformation to robot base frame
- Robust detection with failure handling

### 🦾 Manipulation
- Pick and place operations
- Arm tucking for navigation
- Head movement for better perception

### 🛡️ Error Recovery
- Kidnapping detection during navigation
- Navigation failure recovery
- State consistency maintenance
- Automatic task resumption

## System Components

### State Management
The `StateManager` class maintains robot state including:
- `arm_tucked`: Arm configuration status
- `head_state`: Head position (up/down)
- `detected`: Object detection status
- `picked`/`placed`: Manipulation progress
- `localized`: Localization status
- `navigated_pick`/`navigated_place`: Navigation progress
- `kidnapped`: Localization failure flag
- `navigation_failed`: Navigation failure flag

### Recovery Mechanisms

#### Kidnapping Recovery
- Detects localization failures via covariance monitoring
- Triggers re-localization sequence
- Resumes appropriate task based on progress

#### Navigation Recovery
- Handles move_base failures
- Resets navigation states
- Retries navigation after re-localization

## Installation

### Prerequisites
- ROS Noetic
- Python 3.8+
- py_trees library
- Gazebo simulation environment
- TIAGo robot packages

### Setup
```bash
# Clone the repository
git clone https://github.com/jackeygle/DD2410-Introduction-to-Robotics.git
cd DD2410-Introduction-to-Robotics

# Install dependencies
pip install py_trees py_trees_ros

# Build the workspace
catkin build

# Source the workspace
source devel/setup.bash
```

## Usage

### Launch the System
```bash
# Start Gazebo simulation
roslaunch robotics_project launch_project.launch

# In a new terminal, run the behavior tree
rosrun robotics_project bt_students.py
```

### Launch Files
- `launch_project.launch`: Complete system launch
- `gazebo_project.launch`: Gazebo environment only
- `launch_robot.launch`: Robot-specific launch

### Parameters
Key ROS parameters (configured in launch files):
- `amcl_estimate`: AMCL pose topic
- `pick_pose_topic`: Pick location pose
- `place_pose_topic`: Place location pose
- `cmd_vel_topic`: Velocity command topic
- `move_head_srv`: Head movement service
- `pick_srv`/`place_srv`: Manipulation services

## Behavior Tree Structure

### Main Sequence
```
Main Sequence
├── Kidnapped Check
│   ├── StateCheck(kidnapped, False)
│   └── Recovery Sequence
├── Navigation Failure Check
│   ├── StateCheck(navigation_failed, False)
│   └── Navigation Recovery Sequence
├── Tuck Arm Fallback
├── Move Head Up
├── Localization (Pick)
├── Move Head Down
├── Detect Object
├── Pick Object
├── Localization (Place)
├── Place Object
└── Check Cube on Table
```

### Fallback Pattern
Each major operation uses a Selector (fallback) pattern:
```
Operation Selector
├── StateCheck(operation_completed)
└── Execute Operation
```

## State Management

### State Flow
1. **Initialization**: Reset all states
2. **Localization**: Establish robot position
3. **Navigation**: Move to pick location
4. **Detection**: Find target object
5. **Manipulation**: Pick object
6. **Navigation**: Move to place location
7. **Manipulation**: Place object
8. **Verification**: Confirm task completion

### Error States
- **Kidnapped**: Localization lost during navigation
- **Navigation Failed**: Path planning/execution failure
- **Detection Failed**: Object not found
- **Manipulation Failed**: Pick/place operation failure

## Error Handling and Recovery

### Kidnapping Detection
```python
def check_covariance_threshold(self, covariance_matrix):
    pos_cov = covariance_matrix[0] + covariance_matrix[7]
    return pos_cov > 0.07  # Threshold for kidnapping detection
```

### Recovery Sequence
1. **Detect Failure**: Monitor covariance during navigation
2. **Stop Robot**: Immediately halt movement
3. **Cancel Goals**: Clear navigation targets
4. **Re-localize**: Spin robot for AMCL convergence
5. **Resume Task**: Continue from appropriate state

## Testing

### Unit Testing
Test individual behavior components:
```bash
# Test state management
python -m pytest tests/test_state_manager.py

# Test navigation behaviors
python -m pytest tests/test_navigation.py
```

### Integration Testing
Test complete behavior tree:
```bash
# Run full system test
rostest robotics_project bt_integration_test.test
```

### Simulation Testing
1. Launch Gazebo environment
2. Place ArUco cube at pick location
3. Run behavior tree
4. Verify pick-and-place completion

## Performance Considerations

### Optimization Features
- **Parallel Execution**: Non-blocking behavior evaluation
- **State Caching**: Avoid redundant state checks
- **Efficient Polling**: Optimized sensor monitoring
- **Memory Management**: Proper cleanup of ROS resources

### Monitoring
- Real-time state visualization
- Performance metrics logging
- Error rate tracking
- Success rate analysis

## Troubleshooting

### Common Issues

#### Robot Not Localizing
- Check AMCL parameters
- Verify map quality
- Ensure sufficient sensor data

#### Navigation Failures
- Verify costmap configuration
- Check path planning parameters
- Ensure clear navigation paths

#### Detection Issues
- Confirm ArUco marker visibility
- Check camera calibration
- Verify lighting conditions

#### Manipulation Failures
- Check arm configuration
- Verify manipulation services
- Ensure proper object positioning

## Contributing

### Development Guidelines
1. Follow PEP 8 style guidelines
2. Add comprehensive docstrings
3. Include unit tests for new features
4. Update documentation for changes

### Code Structure
- Keep behaviors modular and focused
- Use descriptive names for states and behaviors
- Implement proper error handling
- Add logging for debugging

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ROS community for excellent robotics framework
- py_trees developers for behavior tree implementation
- TIAGo robot team for simulation environment
- Course instructors for project guidance

## Contact

For questions and support:
- Create an issue in this repository
- Contact the development team

---

*This implementation demonstrates advanced robotics concepts including autonomous navigation, perception, manipulation, and robust error recovery in a realistic simulation environment.* 