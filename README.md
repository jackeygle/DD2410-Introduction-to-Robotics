Project Overview

This project involves simulating a TIAGo robot in an apartment using ROS and Gazebo. The robot is equipped with sensors and a manipulator for navigation and performing pick, carry, and place tasks. Your goal is to modify and add components to the system to handle increasingly complex tasks.
Key Tasks

    Pick&Carry&Place with visual sensing
    Pick&Carry&Place with sensing and navigation

Setup Instructions

    Download the packages:

cd ~/catkin_ws/src/
git clone https://github.com/kth-ros-pkg/Robotics_intro.git

Build the project:

    cd ~/catkin_ws
    catkin_make -DCATKIN_ENABLE_TESTING=0
    source devel/setup.bash

    Choose between cloning the repository or using a ZIP file:
        Cloning the repo ensures you get updates automatically.
        The ZIP file only receives major updates, and you must merge any changes manually.

Launching the Simulation

    Run the following command to start Gazebo and RVIZ:

    roslaunch robotics_project launch_project.launch

    To speed up the simulation, set the "gzclient" variable to false in the launch file if you don’t need the Gazebo graphical interface.

    If everything works, the robot should move around, fold its arm, approach a chair, and lower its head.

Dealing with Probabilistic Components

Some system components use probabilistic models (e.g., AMCL localization, MoveIt! arm planning) that may fail or behave non-deterministically. This is normal. You should implement feedback and error handling in your state machine to manage these failures. For example, log feedback from action servers or service calls.
Launching the System in Gazebo and RVIZ

    Run the following to launch both components:

    roslaunch robotics_project gazebo_project.launch
    roslaunch robotics_project launch_project.launch

Relaunching the System

    Kill the terminal with launch_project.launch (Ctrl-C).
    Manually delete the tiago_steel and aruco_cube models in Gazebo.
    Relaunch launch_project.launch.

Troubleshooting

If you see this warning:

[WARN] Controller Spawner couldn't find the expected controller_manager ROS interface.

It means something went wrong. In this case, kill both terminals and relaunch as described above.
