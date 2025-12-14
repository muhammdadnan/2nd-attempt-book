# NVIDIA Isaac Integration for Humanoid Robot Simulation

This directory contains configuration and documentation for integrating the humanoid robot simulation with NVIDIA Isaac Sim.

## Overview

This package provides the necessary components to connect your ROS 2-based humanoid robot simulation with NVIDIA Isaac Sim for photorealistic rendering and advanced AI training capabilities.

## Setup Instructions

### Prerequisites

1. NVIDIA Isaac Sim (Part of Isaac ROS Developer Kit)
2. NVIDIA GPU with CUDA support
3. ROS 2 Humble Hawksbill
4. Isaac ROS packages

### Installation

1. Install NVIDIA Isaac Sim from NVIDIA Developer Zone
2. Set up the Isaac ROS workspace
3. Configure the ROS bridge settings

### Configuration Files

- `isaac_sim_config.json` - Configuration for Isaac Sim environment
- `humanoid_robot_asset.usd` - USD asset for the humanoid robot
- `training_scenarios/` - Isaac Sim scenario files for AI training

## Isaac ROS Bridge

The Isaac ROS bridge enables high-performance communication between ROS 2 and Isaac Sim. The bridge handles:

- High-frequency sensor data (LiDAR, cameras, IMU)
- Real-time control commands
- Photorealistic rendering for perception training
- Physics simulation with realistic material properties

## Example Scenarios

- `basic_navigation_scenario.usd` - Basic navigation environment
- `manipulation_scenario.usd` - Object manipulation tasks
- `dynamic_obstacles_scenario.usd` - Environments with moving obstacles

## Running the Simulation

1. Start Isaac Sim:
   ```bash
   # Launch Isaac Sim with the humanoid robot scene
   python -m omni.isaac.sim.python_app --config=humanoid_robot_config.json
   ```

2. In a separate terminal, start the ROS 2 bridge:
   ```bash
   ros2 launch simulation isaac_ros_bridge.launch.py
   ```

3. The humanoid robot in Isaac Sim will now be synchronized with ROS 2

## Perception Pipeline

Isaac Sim enables advanced perception pipeline development:

- Synthetic dataset generation
- Domain randomization for robust perception
- Multi-sensor fusion testing
- Visual-inertial odometry validation

## Troubleshooting

- Ensure CUDA drivers are properly installed
- Verify Isaac Sim license is active
- Check that the robot USD asset is properly formatted
- Monitor GPU memory usage during simulation