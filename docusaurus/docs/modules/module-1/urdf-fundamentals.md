# URDF Fundamentals

Unified Robot Description Format (URDF) is the standard for representing robot models in ROS. In this chapter, we'll explore the fundamentals of URDF and how to create accurate representations of humanoid robots.

## Introduction to URDF

URDF is an XML-based format that describes robots in terms of:
- **Links**: Rigid bodies with visual, collision, and inertial properties
- **Joints**: Connections between links that define kinematic relationships
- **Transmissions**: How actuators connect to joints
- **Materials**: Visual appearance properties

## Basic URDF Structure

A basic URDF file has this structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Define materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <!-- Define links -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Define joints -->
  <joint name="example_joint" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="child_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>
</robot>
```

## Links

Links represent rigid bodies in the robot. Each link can have:

### Visual Elements
Define how the link appears in visualization tools:

```xml
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Choose one geometry type -->
      <box size="1 1 1"/>
      <!-- OR -->
      <cylinder radius="0.1" length="0.5"/>
      <!-- OR -->
      <sphere radius="0.1"/>
      <!-- OR -->
      <mesh filename="package://my_robot/meshes/link.stl"/>
    </geometry>
    <material name="material_name"/>
  </visual>
</link>
```

### Collision Elements
Define the collision geometry for physics simulation:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Similar to visual geometry but often simplified -->
    <box size="1 1 1"/>
  </geometry>
</collision>
```

### Inertial Elements
Define the physical properties for dynamics simulation:

```xml
<inertial>
  <origin xyz="0.03 0 0.05" rpy="0 0 0"/>
  <mass value="0.13"/>
  <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
</inertial>
```

## Joints

Joints define how links connect and move relative to each other:

### Joint Types

1. **Revolute**: Rotational joint with limits
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

2. **Continuous**: Rotational joint without limits
```xml
<joint name="joint_name" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

3. **Prismatic**: Linear sliding joint
```xml
<joint name="joint_name" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.1" effort="10" velocity="1"/>
</joint>
```

4. **Fixed**: Rigid connection (no movement)
```xml
<joint name="joint_name" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
```

5. **Floating**: 6 degrees of freedom
```xml
<joint name="joint_name" type="floating">
  <parent link="parent_link"/>
  <child link="child_link"/>
</joint>
```

6. **Planar**: Motion on a plane
```xml
<joint name="joint_name" type="planar">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <axis xyz="0 0 1"/>
</joint>
```

## Materials

Define visual appearance:

```xml
<material name="red">
  <color rgba="0.8 0.0 0.0 1.0"/>
</material>

<material name="green">
  <color rgba="0.0 0.8 0.0 1.0"/>
</material>

<material name="blue">
  <color rgba="0.0 0.0 0.8 1.0"/>
</material>

<material name="white">
  <color rgba="1.0 1.0 1.0 1.0"/>
</material>

<material name="black">
  <color rgba="0.0 0.0 0.0 1.0"/>
</material>
```

## Xacro for Complex URDFs

Xacro (XML Macros) helps manage complex URDFs by allowing:

- Variable definitions
- Macros
- Mathematical expressions
- File inclusion

### Basic Xacro Example

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">

  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />

  <!-- Define a macro for wheels -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <origin xyz="0 0 0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro to create wheels -->
  <xacro:wheel prefix="front_left" parent="base_link" xyz="0.2 0.2 0" rpy="0 0 0"/>
  <xacro:wheel prefix="front_right" parent="base_link" xyz="0.2 -0.2 0" rpy="0 0 0"/>
  <xacro:wheel prefix="rear_left" parent="base_link" xyz="-0.2 0.2 0" rpy="0 0 0"/>
  <xacro:wheel prefix="rear_right" parent="base_link" xyz="-0.2 -0.2 0" rpy="0 0 0"/>

</robot>
```

## Inertial Calculations

Proper inertial properties are crucial for simulation. Here are formulas for basic shapes:

### Box (width x depth x height)
```xml
<inertial>
  <mass value="m"/>
  <inertia ixx="m*(d² + h²)/12" ixy="0" ixz="0"
           iyy="m*(w² + h²)/12" iyz="0"
           izz="m*(w² + d²)/12"/>
</inertial>
```

### Cylinder (radius r, length l, axis along z)
```xml
<inertial>
  <mass value="m"/>
  <inertia ixx="m*(3*r² + l²)/12" ixy="0" ixz="0"
           iyy="m*(3*r² + l²)/12" iyz="0"
           izz="m*r²/2"/>
</inertial>
```

### Sphere (radius r)
```xml
<inertial>
  <mass value="m"/>
  <inertia ixx="2*m*r²/5" ixy="0" ixz="0"
           iyy="2*m*r²/5" iyz="0"
           izz="2*m*r²/5"/>
</inertial>
```

## Best Practices for URDF

1. **Start simple**: Begin with basic shapes and add complexity gradually
2. **Use consistent units**: Stick to meters for length, kilograms for mass
3. **Validate your URDF**: Use tools like `check_urdf` to verify syntax
4. **Include proper inertials**: Essential for accurate simulation
5. **Use appropriate collision geometry**: Simplified versions of visual geometry
6. **Consider mass distribution**: Place inertial origins correctly
7. **Name links and joints consistently**: Use descriptive names
8. **Use Xacro for complex robots**: Avoid duplication with macros

## URDF Validation

Validate your URDF files:

```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Show URDF info
urdf_to_graphiz /path/to/robot.urdf
```

## Visualizing URDF

Test your URDF with RViz:

```bash
# Start robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat robot.urdf)'

# Or with joint states
ros2 run joint_state_publisher joint_state_publisher
ros2 run rviz2 rviz2
```

## Common URDF Issues

1. **Missing parent/child links**: Ensure all joints reference existing links
2. **Disconnected parts**: All links should connect to the tree
3. **Invalid inertial values**: Check that inertial matrices are physically valid
4. **Zero masses**: All links should have positive mass values
5. **Invalid joint limits**: Ensure lower < upper for revolute joints

## Next Steps

In the next chapter, we'll apply these URDF fundamentals specifically to create a detailed humanoid robot model. We'll design the kinematic structure, define appropriate joint limits, and create a complete URDF that represents a realistic humanoid robot for simulation and control.