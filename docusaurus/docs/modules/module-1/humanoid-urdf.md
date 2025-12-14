# Humanoid URDF

In this chapter, we'll create a detailed URDF model for a humanoid robot, applying the URDF fundamentals we learned to create a realistic representation of a bipedal robot with arms, torso, and head.

## Humanoid Robot Design Considerations

When designing a humanoid URDF, consider:

1. **Degrees of Freedom**: How many joints and what range of motion
2. **Kinematic Structure**: How parts connect to form a kinematic chain
3. **Balance**: Center of mass and stability considerations
4. **Actuation**: What joints will be actively controlled
5. **Sensors**: Where to place IMU, cameras, and other sensors

## Complete Humanoid URDF Example

Here's a comprehensive humanoid robot URDF using Xacro:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include other xacro files -->
  <xacro:include filename="humanoid_constants.xacro"/>
  <xacro:include filename="humanoid_materials.xacro"/>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${base_width} ${base_depth} ${base_height}"/>
      </geometry>
      <material name="light_grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${base_width} ${base_depth} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0.0 0.0 ${base_height/2 + torso_height/2}" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
      <material name="light_grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.04"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 ${torso_height/2 + head_radius}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/3}" upper="${M_PI/3}" effort="5" velocity="2"/>
  </joint>

  <link name="head">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${head_radius}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${head_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="${torso_width/2} ${torso_depth/4} 0.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="10" velocity="2"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 ${-upper_arm_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${upper_arm_radius}" length="${upper_arm_length}"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${-upper_arm_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${upper_arm_radius}" length="${upper_arm_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 ${-upper_arm_length/2}" rpy="0 0 0"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0.0 0.0 ${-upper_arm_length}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="${M_PI/2}" effort="8" velocity="2"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <origin xyz="0 0 ${-lower_arm_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${lower_arm_radius}" length="${lower_arm_length}"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${-lower_arm_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${lower_arm_radius}" length="${lower_arm_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 ${-lower_arm_length/2}" rpy="0 0 0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.0008"/>
    </inertial>
  </link>

  <!-- Right Arm -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="${torso_width/2} ${-torso_depth/4} 0.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="10" velocity="2"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 ${-upper_arm_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${upper_arm_radius}" length="${upper_arm_length}"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${-upper_arm_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${upper_arm_radius}" length="${upper_arm_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 ${-upper_arm_length/2}" rpy="0 0 0"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0.0 0.0 ${-upper_arm_length}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="${M_PI/2}" effort="8" velocity="2"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <origin xyz="0 0 ${-lower_arm_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${lower_arm_radius}" length="${lower_arm_length}"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${-lower_arm_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${lower_arm_radius}" length="${lower_arm_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 ${-lower_arm_length/2}" rpy="0 0 0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.0008"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="${-base_width/4} ${base_depth/4} ${-base_height/2}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="20" velocity="1.5"/>
  </joint>

  <link name="left_upper_leg">
    <visual>
      <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${upper_leg_radius}" length="${upper_leg_length}"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${upper_leg_radius}" length="${upper_leg_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0.0 0.0 ${-upper_leg_length}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="${M_PI/2}" effort="18" velocity="1.5"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${lower_leg_radius}" length="${lower_leg_length}"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${lower_leg_radius}" length="${lower_leg_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.9"/>
      <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0"/>
      <inertia ixx="0.009" ixy="0.0" ixz="0.0" iyy="0.009" iyz="0.0" izz="0.0015"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0.0 0.0 ${-lower_leg_length}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="10" velocity="1"/>
  </joint>

  <link name="left_foot">
    <visual>
      <origin xyz="0.05 0.0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0.05 0.0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0.05 0.0 -0.025" rpy="0 0 0"/>
      <inertia ixx="0.0003" ixy="0.0" ixz="0.0" iyy="0.0003" iyz="0.0" izz="0.0003"/>
    </inertial>
  </link>

  <!-- Right Leg -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="${-base_width/4} ${-base_depth/4} ${-base_height/2}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="20" velocity="1.5"/>
  </joint>

  <link name="right_upper_leg">
    <visual>
      <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${upper_leg_radius}" length="${upper_leg_length}"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${upper_leg_radius}" length="${upper_leg_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0.0 0.0 ${-upper_leg_length}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="${M_PI/2}" effort="18" velocity="1.5"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${lower_leg_radius}" length="${lower_leg_length}"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${lower_leg_radius}" length="${lower_leg_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.9"/>
      <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0"/>
      <inertia ixx="0.009" ixy="0.0" ixz="0.0" iyy="0.009" iyz="0.0" izz="0.0015"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0.0 0.0 ${-lower_leg_length}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="10" velocity="1"/>
  </joint>

  <link name="right_foot">
    <visual>
      <origin xyz="0.05 0.0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0.05 0.0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0.05 0.0 -0.025" rpy="0 0 0"/>
      <inertia ixx="0.0003" ixy="0.0" ixz="0.0" iyy="0.0003" iyz="0.0" izz="0.0003"/>
    </inertial>
  </link>

  <!-- Sensors -->
  <joint name="imu_joint" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Transmissions for actuated joints -->
  <xacro:macro name="transmission_block" params="joint_name">
    <transmission name="${joint_name}_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="${joint_name}">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      </joint>
      <actuator name="${joint_name}_motor">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>
  </xacro:macro>

  <!-- Add transmissions for all actuated joints -->
  <xacro:transmission_block joint_name="left_shoulder_joint"/>
  <xacro:transmission_block joint_name="right_shoulder_joint"/>
  <xacro:transmission_block joint_name="left_elbow_joint"/>
  <xacro:transmission_block joint_name="right_elbow_joint"/>
  <xacro:transmission_block joint_name="left_hip_joint"/>
  <xacro:transmission_block joint_name="right_hip_joint"/>
  <xacro:transmission_block joint_name="left_knee_joint"/>
  <xacro:transmission_block joint_name="right_knee_joint"/>
  <xacro:transmission_block joint_name="left_ankle_joint"/>
  <xacro:transmission_block joint_name="right_ankle_joint"/>
  <xacro:transmission_block joint_name="neck_joint"/>

</robot>
```

## Constants File

Create a constants file to define robot dimensions:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Physical constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Robot dimensions -->
  <xacro:property name="base_width" value="0.3" />
  <xacro:property name="base_depth" value="0.2" />
  <xacro:property name="base_height" value="0.15" />

  <xacro:property name="torso_width" value="0.25" />
  <xacro:property name="torso_depth" value="0.15" />
  <xacro:property name="torso_height" value="0.4" />

  <xacro:property name="head_radius" value="0.08" />

  <xacro:property name="upper_arm_length" value="0.3" />
  <xacro:property name="lower_arm_length" value="0.25" />
  <xacro:property name="upper_arm_radius" value="0.04" />
  <xacro:property name="lower_arm_radius" value="0.03" />

  <xacro:property name="upper_leg_length" value="0.4" />
  <xacro:property name="lower_leg_length" value="0.35" />
  <xacro:property name="upper_leg_radius" value="0.05" />
  <xacro:property name="lower_leg_radius" value="0.045" />

  <xacro:property name="foot_size" value="0.15 0.1 0.05" />

</robot>
```

## Materials File

Create a materials file for visual appearance:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>

  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>

  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <material name="light_grey">
    <color rgba="0.7 0.7 0.7 1.0"/>
  </material>

  <material name="orange">
    <color rgba="${255/255} ${108/255} ${10/255} 1.0"/>
  </material>

  <material name="brown">
    <color rgba="${222/207/195} 1.0"/>
  </material>

  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>

  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <material name="yellow">
    <color rgba="0.8 0.8 0.0 1.0"/>
  </material>

</robot>
```

## Kinematic Considerations for Humanoid Robots

### Balance and Center of Mass

For stable bipedal locomotion, consider:

1. **Low center of mass**: Keep heavy components low
2. **Wide stance**: Design feet appropriately sized
3. **Ankle joints**: Enable balance adjustments

### Joint Limits for Safety

```xml
<!-- Safe joint limits for humanoid -->
<limit lower="${-M_PI/3}" upper="${M_PI/3}" effort="10" velocity="2"/>
```

### Redundancy and Dexterity

Humanoid robots often have redundant degrees of freedom:

- Arms with 7+ DOF for dexterous manipulation
- Mobile base with wheels or legs
- Multiple sensors for perception

## Advanced Humanoid Features

### Flexible Spine

For more human-like movement, add spinal joints:

```xml
<joint name="spine_joint" type="revolute">
  <parent link="base_link"/>
  <child link="torso"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="15" velocity="1"/>
</joint>
```

### Hand Models

For manipulation, add detailed hand models:

```xml
<!-- Add finger joints for manipulation -->
<joint name="left_thumb_joint" type="revolute">
  <parent link="left_lower_arm"/>
  <child link="left_thumb"/>
  <origin xyz="0 0 ${-lower_arm_length - 0.02}" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="1.0" effort="2" velocity="1"/>
</joint>
```

## Validation and Testing

Test your humanoid URDF:

```bash
# Check the URDF
check_urdf /path/to/humanoid_robot.urdf.xacro

# Visualize in RViz
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(xacro /path/to/humanoid_robot.urdf.xacro)'

# Use joint state publisher to move joints
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

## Common Humanoid URDF Issues

1. **Balance Problems**: Ensure the center of mass is within the support polygon
2. **Joint Limit Violations**: Set appropriate limits to prevent self-collision
3. **Inertial Issues**: Properly calculate inertial properties for stable simulation
4. **Complexity**: Start simple and add complexity gradually

## Next Steps

In the next chapter, we'll learn how to visualize our humanoid robot in RViz, which is essential for debugging and understanding robot behavior. We'll explore RViz configuration, TF visualization, sensor data visualization, and more.