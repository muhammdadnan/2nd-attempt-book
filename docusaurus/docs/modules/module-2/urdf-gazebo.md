# URDF to Gazebo

In this chapter, we'll learn how to adapt URDF models for Gazebo simulation by adding Gazebo-specific extensions. While URDF describes the robot's structure, Gazebo extensions define how the robot interacts with the simulation environment.

## Gazebo Extensions Overview

Gazebo uses `<gazebo>` tags within URDF files to specify simulation-specific properties:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Standard URDF content -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.15"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Gazebo-specific extensions -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

</robot>
```

## Material Definitions

Define visual materials for Gazebo:

```xml
<!-- Standard Gazebo materials -->
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
</gazebo>

<!-- Or define custom materials -->
<gazebo>
  <material name="custom_material">
    <ambient>0.3 0.3 0.3 1.0</ambient>
    <diffuse>0.7 0.7 0.7 1.0</diffuse>
    <specular>0.1 0.1 0.1 1.0</specular>
    <emissive>0.0 0.0 0.0 1.0</emissive>
  </material>
</gazebo>
```

Common Gazebo materials:
- `Gazebo/Blue`, `Gazebo/Green`, `Gazebo/Red`
- `Gazebo/White`, `Gazebo/Black`, `Gazebo/Grey`
- `Gazebo/Yellow`, `Gazebo/Orange`, `Gazebo/Purple`

## Physics Properties

Define physics properties for accurate simulation:

```xml
<gazebo reference="link_name">
  <!-- Inertial properties (if different from URDF) -->
  <mu1>0.8</mu1>  <!-- Friction coefficient 1 -->
  <mu2>0.8</mu2>  <!-- Friction coefficient 2 -->
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>100000.0</kd>  <!-- Contact damping -->
  <maxVel>100.0</maxVel>  <!-- Maximum contact velocity -->
  <minDepth>0.001</minDepth>  <!-- Minimum contact depth -->

  <!-- Bounce properties -->
  <bounce>
    <restitution_coefficient>0.1</restitution_coefficient>
    <threshold>100000.0</threshold>
  </bounce>
</gazebo>
```

## Complete Humanoid Robot with Gazebo Extensions

Here's a complete example of a humanoid robot URDF with Gazebo extensions:

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

  <!-- Gazebo extensions for base link -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>100000.0</kd>
  </gazebo>

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

  <!-- Gazebo extensions for torso -->
  <gazebo reference="torso">
    <material>Gazebo/Grey</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

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

  <!-- Gazebo extensions for head -->
  <gazebo reference="head">
    <material>Gazebo/White</material>
    <mu1>0.5</mu1>
    <mu2>0.5</mu2>
  </gazebo>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="${torso_width/2} ${torso_depth/4} 0.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="10" velocity="2"/>
    <dynamics damping="0.5" friction="0.1"/>
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

  <!-- Gazebo extensions for left upper arm -->
  <gazebo reference="left_upper_arm">
    <material>Gazebo/Blue</material>
    <mu1>0.6</mu1>
    <mu2>0.6</mu2>
  </gazebo>

  <!-- Left Elbow Joint -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0.0 0.0 ${-upper_arm_length}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="${M_PI/2}" effort="8" velocity="2"/>
    <dynamics damping="0.3" friction="0.1"/>
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

  <!-- Gazebo extensions for left lower arm -->
  <gazebo reference="left_lower_arm">
    <material>Gazebo/Blue</material>
    <mu1>0.6</mu1>
    <mu2>0.6</mu2>
  </gazebo>

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="${-base_width/4} ${base_depth/4} ${-base_height/2}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="20" velocity="1.5"/>
    <dynamics damping="1.0" friction="0.2"/>
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

  <!-- Gazebo extensions for left upper leg -->
  <gazebo reference="left_upper_leg">
    <material>Gazebo/Red</material>
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
    <kp>10000000.0</kp>
    <kd>1000000.0</kd>
  </gazebo>

  <!-- Continue similarly for other links and joints... -->

  <!-- Gazebo plugin for ROS control -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>

  <!-- Gazebo plugin for IMU sensor -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>

</robot>
```

## Gazebo Plugins for Humanoid Robots

### ROS Control Plugin

The ROS Control plugin connects Gazebo to ROS 2:

```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    <controlPeriod>0.001</controlPeriod>  <!-- Match physics update rate -->
  </plugin>
</gazebo>
```

### Joint State Publisher Plugin

Publishes joint states from simulation:

```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
    <jointName>left_shoulder_joint</jointName>
    <updateRate>30</updateRate>
    <alwaysOn>true</alwaysOn>
  </plugin>
</gazebo>
```

## Sensor Integration

### Camera Sensor
```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
      <cameraName>head_camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Sensor
```xml
<gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>
```

## Physics Tuning for Humanoid Robots

### Balance and Stability
For humanoid robots, special attention to physics parameters is crucial:

```xml
<!-- For feet links - high friction for stability -->
<gazebo reference="left_foot">
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
  <kp>10000000.0</kp>  <!-- High stiffness for stable contact -->
  <kd>1000000.0</kd>
</gazebo>
```

### Joint Dynamics
Proper joint damping and friction for realistic movement:

```xml
<joint name="left_knee_joint" type="revolute">
  <parent link="left_upper_leg"/>
  <child link="left_lower_leg"/>
  <origin xyz="0.0 0.0 ${-upper_leg_length}" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="${M_PI/2}" effort="18" velocity="1.5"/>
  <dynamics damping="2.0" friction="0.5"/>  <!-- Higher damping for stability -->
</joint>
```

## Ground Plane and Environment

Define the ground plane with appropriate properties:

```xml
<!-- In world file -->
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>
            <mu2>1.0</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <material>
        <ambient>0.7 0.7 0.7 1</ambient>
        <diffuse>0.7 0.7 0.7 1</diffuse>
        <specular>0.0 0.0 0.0 1</specular>
      </material>
    </visual>
  </link>
</model>
```

## Validation and Testing

Test your Gazebo-extended URDF:

```bash
# Test the URDF with Gazebo
gzsdf check /path/to/robot.urdf.xacro

# Launch in Gazebo
ros2 launch gazebo_ros gazebo.launch.py world:=empty.world
ros2 run gazebo_ros spawn_entity.py -file /path/to/robot.urdf.xacro -entity humanoid_robot
```

## Common Issues and Solutions

### Robot Sinks into Ground
- Increase `kp` (stiffness) and `kd` (damping) values
- Check that collision geometry is properly defined
- Verify inertial properties are realistic

### Unstable Joint Behavior
- Increase joint damping values
- Verify joint limits and effort values
- Check that the robot is properly assembled

### Performance Issues
- Simplify collision geometry where possible
- Use appropriate physics update rates
- Reduce the number of complex contact surfaces

## Best Practices

1. **Start with basic physics**: Add complex properties gradually
2. **Match real hardware**: Use actual robot parameters when available
3. **Validate physics behavior**: Test stability and movement patterns
4. **Optimize for performance**: Balance accuracy with simulation speed
5. **Document changes**: Keep track of Gazebo-specific modifications

## Next Steps

In the next chapter, we'll dive deeper into physics tuning, learning how to fine-tune parameters for realistic humanoid robot behavior in simulation. We'll explore how to match simulation behavior with real-world physics and optimize for specific tasks like walking and manipulation.