
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
5. **Mapping and Localization**: Modify the environment and navigate the robot to achieve a complete map of the surroundings.
6. **Data Recording and Analysis**: Record the robot's trajectory data, analyze the path taken, and adjust parameters for improved performance.
7. **Vision-Based Navigation**: Integrate a camera with the robot and utilize ArUco markers for navigation tasks.


## Project Structure

The primary components include:

1. **Gazebo World Construction**: 
    - A custom Gazebo world (`rl_racefield`) is created, with specific obstacles and markers placed at predefined positions.
    - The robot is spawned in this environment with an initial pose set to `(x = -3, y = 5, yaw = -90 degrees)`.

<p align="center">
  <img src="https://github.com/user-attachments/assets/169f68b2-833e-4ebc-a6b1-77ff37a159c5" alt="Pseudocode" width="400"/>
</p>


<p align="center">
  <img src="https://github.com/user-attachments/assets/f3b5b166-7f15-48f9-8730-1db3a52080ab" alt="Pseudocode" width="400"/>
</p>



2. **Goal Definition and Management**: 
    - Static transform frames (TF) are defined as goals within the environment. The robot navigates to these goals in a predefined sequence: Goal_3 → Goal_4 → Goal_2 → Goal_1.
    - Multiple listeners and goal-sending functions are implemented to manage the robot's movement from one goal to another.

<p align="center">
  <img src="https://github.com/user-attachments/assets/24146d36-6cea-464f-a763-54d2a7b3597d" alt="Pseudocode" width="400"/>
</p>


3. **Path Planning and Navigation**:
    - Using the `move_base` package, the robot navigates between goals. The `move_base` action client protocol is used to track the robot's progress and ensure it reaches each goal.
    - Various configurations of the planner and costmap parameters are tested to optimize the robot's trajectory and obstacle avoidance capabilities.

4. **Mapping and Environment Exploration**:
    - The robot explores the environment by following a sequence of goals, which are modified or added to create a complete map.
    - Multiple navigation and mapping strategies are tested by adjusting the robot's speed, acceleration, and costmap configurations.

<p align="center">
  <img src="https://github.com/user-attachments/assets/1de1a6fa-2949-4f7c-9fb8-3c8b0bade772" alt="Pseudocode" width="400"/>
</p>


5. **Vision-Based Navigation**:
    - The robot is equipped with a camera and configured to detect an ArUco marker placed on an obstacle in the environment.
    - Upon detecting the marker, the robot calculates its position relative to the marker and sets a new navigation goal accordingly.

<p align="center">
  <img src="https://github.com/user-attachments/assets/b65fe457-398d-46e4-ac4d-0836fe40cb29" alt="Pseudocode" width="400"/>
</p>


## Installation

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

## Key Results and Observations

- **Trajectory Control**: The robot successfully navigated through the predefined goals in the correct order, demonstrating effective use of the `move_base` package.
- **Mapping**: The robot successfully mapped the environment by exploring all areas defined by the goals. The final map provided a complete representation of the simulated world.
- **Parameter Tuning**: Multiple configurations were tested to optimize the robot's navigation. The final configuration provided a balance between speed and obstacle avoidance.
- **Vision-Based Navigation**: The integration of the ArUco marker allowed the robot to autonomously detect and adjust its path based on visual input, showcasing the effectiveness of vision-based navigation strategies.


