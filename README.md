
# Mobile Robot Trajectory Tracking and Visual Servoing

## Overview

This project is developed as part of the Robotics Lab course assignment and aims to control a mobile robot to follow a specific trajectory
in ROS. The project involves creating a Gazebo world, spawning a robot, setting up various goals, and performing autonomous navigation, 
vision-based tasks, and environmental mapping.

## Objectives

The primary objectives of this project are:
1. **Gazebo Simulation Setup**: Construct a simulated world in Gazebo, spawn a mobile robot, and configure its starting position.
2. **Goal Management**: Define multiple goal positions in the environment and enable the robot to autonomously navigate through these goals.
3. **Path Planning and Navigation**: Implement navigation using the `move_base` package and tune various parameters for optimal robot performance.
4. **Vision-Based Navigation**: Integrate a camera with the robot and utilize ArUco markers for navigation tasks.
5. **Mapping and Localization**: Modify the environment and navigate the robot to achieve a complete map of the surroundings.
6. **Data Recording and Analysis**: Record the robot's trajectory data, analyze the path taken, and adjust parameters for improved performance.

## Project Structure

The project is organized into several components, each corresponding to different stages of development and testing. The primary components include:

1. **Gazebo World Construction**: 
    - A custom Gazebo world (`rl_racefield`) is created, with specific obstacles and markers placed at predefined positions.
    - The robot is spawned in this environment with an initial pose set to `(x = -3, y = 5, yaw = -90 degrees)`.

2. **Goal Definition and Management**: 
    - Static transform frames (TF) are defined as goals within the environment. The robot navigates to these goals in a predefined sequence: Goal_3 → Goal_4 → Goal_2 → Goal_1.
    - Multiple listeners and goal-sending functions are implemented to manage the robot's movement from one goal to another.

3. **Path Planning and Navigation**:
    - Using the `move_base` package, the robot navigates between goals. The `move_base` action client protocol is used to track the robot's progress and ensure it reaches each goal.
    - Various configurations of the planner and costmap parameters are tested to optimize the robot's trajectory and obstacle avoidance capabilities.

4. **Vision-Based Navigation**:
    - The robot is equipped with a camera and configured to detect an ArUco marker placed on an obstacle in the environment.
    - Upon detecting the marker, the robot calculates its position relative to the marker and sets a new navigation goal accordingly.

5. **Mapping and Environment Exploration**:
    - The robot explores the environment by following a sequence of goals, which are modified or added to create a complete map.
    - Multiple navigation and mapping strategies are tested by adjusting the robot's speed, acceleration, and costmap configurations.

## Installation and Setup

To set up the project and run the simulation, follow these steps:

1. **Install ROS Noetic**:
    Ensure that ROS Noetic is installed on your system.

2. **Clone the Repository**:
    ```bash
    cd .../catkin_ws/src
    git clone https://github.com/EmanueleCuzzocrea/Homework4.git
    ```
    
3. **Build the Package**:
    Compile the ROS workspace:
    ```bash
    cd .../catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## Conclusion

This project demonstrates the integration of various ROS tools and packages to achieve autonomous navigation in a simulated environment.
Through careful setup, parameter tuning, and implementation of vision-based strategies, the mobile robot was able to follow complex trajectories
and perform mapping tasks effectively.

