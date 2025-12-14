# Unity Integration for Humanoid Robot Simulation

This directory contains configuration and documentation for integrating the humanoid robot simulation with Unity.

## Overview

This package provides the necessary components to connect your ROS 2-based humanoid robot simulation with Unity for high-fidelity rendering and simulation.

## Setup Instructions

### Prerequisites

1. Unity 2022.3 LTS or later
2. Unity Robotics Hub package
3. ROS 2 Humble Hawksbill
4. ROS TCP Connector package

### Installation

1. Install the Unity Robotics Hub from the Unity Package Manager
2. Import the ROS TCP Connector package
3. Configure the ROS connection settings in the Unity scene

### Configuration Files

- `unity_robot_bridge_config.json` - Configuration for the ROS-Unity bridge
- `humanoid_robot_prefab.prefab` - Unity prefab for the humanoid robot
- `simulation_scenes/` - Unity scene files for different simulation environments

## ROS-Unity Bridge

The ROS-Unity bridge enables bidirectional communication between ROS 2 nodes and Unity components. The bridge handles:

- Joint state publishing from Unity to ROS 2
- Robot state subscription from ROS 2 to Unity
- Sensor data exchange (IMU, camera, etc.)
- Control command transmission

## Example Scenes

- `empty_unity_scene.unity` - Basic scene with the humanoid robot
- `obstacle_course_scene.unity` - Scene with obstacles for navigation tasks
- `manipulation_scene.unity` - Scene for manipulation tasks

## Running the Simulation

1. Start the ROS 2 network:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 launch robot_description robot_visualize.launch.py
   ```

2. Start the Unity scene with the ROS TCP Connector configured

3. The humanoid robot in Unity will now be synchronized with the ROS 2 TF tree

## Troubleshooting

- Ensure ROS TCP Connector is properly configured with the correct IP and port
- Check that both ROS 2 and Unity are on the same network
- Verify that the robot description matches between ROS 2 and Unity