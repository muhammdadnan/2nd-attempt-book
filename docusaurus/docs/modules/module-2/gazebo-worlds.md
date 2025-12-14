# Gazebo Worlds

In this chapter, we'll explore how to create complex Gazebo worlds with various environments, obstacles, and scenarios for testing your humanoid robot. These worlds enable comprehensive validation of robot behaviors in diverse and realistic settings.

## World File Structure

A basic Gazebo world file structure:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_test_world">
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- GUI configuration -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0.0 0.0 1.0 0.0 0.0 0.0</pose>
      </camera>
    </gui>

    <!-- Custom models and objects -->
    <!-- Will be added in sections below -->

  </world>
</sdf>
```

## Basic Indoor Environment

### Simple Room World

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_room">
    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Room walls -->
    <model name="wall_1">
      <pose>0 5 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall_2">
      <pose>5 0 1 0 0 1.57</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Furniture for humanoid testing -->
    <model name="table">
      <pose>2 2 0.4 0 0 0</pose>
      <link name="table_top">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.2 0.8 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.2 0.8 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
      <link name="leg_1">
        <pose>-0.5 -0.3 0.2 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.05 0.05 0.4</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.4</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.2 0.0 1</ambient>
            <diffuse>0.4 0.2 0.0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
      <!-- Additional legs would be defined similarly -->
    </model>

    <!-- Obstacles for navigation testing -->
    <model name="obstacle_1">
      <pose>-2 -2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>5</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- GUI -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>5 5 5 0 0.5 1.57</pose>
      </camera>
    </gui>
  </world>
</sdf>
```

## Complex Indoor Environment

### Multi-Room World with Corridors

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="multi_room_house">
    <!-- Standard includes -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
        </constraints>
      </ode>
    </physics>

    <!-- Room 1: Living room -->
    <!-- Walls -->
    <model name="living_room_wall_north">
      <pose>0 5 1 0 0 0</pose>
      <link name="link">
        <collision><geometry><box><size>8 0.2 2</size></box></geometry></collision>
        <visual><geometry><box><size>8 0.2 2</size></box></geometry></visual>
      </link>
    </model>

    <model name="living_room_wall_south">
      <pose>0 -3 1 0 0 0</pose>
      <link name="link">
        <collision><geometry><box><size>8 0.2 2</size></box></geometry></collision>
        <visual><geometry><box><size>8 0.2 2</size></box></geometry></visual>
      </link>
    </model>

    <model name="living_room_wall_west">
      <pose>-4 1 1 0 0 1.57</pose>
      <link name="link">
        <collision><geometry><box><size>8 0.2 2</size></box></geometry></collision>
        <visual><geometry><box><size>8 0.2 2</size></box></geometry></visual>
      </link>
    </model>

    <model name="living_room_wall_east">
      <pose>4 1 1 0 0 1.57</pose>
      <link name="link">
        <collision><geometry><box><size>8 0.2 2</size></box></geometry></collision>
        <visual><geometry><box><size>8 0.2 2</size></box></geometry></visual>
      </link>
    </model>

    <!-- Room 2: Kitchen -->
    <model name="kitchen_wall_north">
      <pose>6 2 1 0 0 0</pose>
      <link name="link">
        <collision><geometry><box><size>4 0.2 2</size></box></geometry></collision>
        <visual><geometry><box><size>4 0.2 2</size></box></geometry></visual>
      </link>
    </model>

    <model name="kitchen_wall_south">
      <pose>6 -2 1 0 0 0</pose>
      <link name="link">
        <collision><geometry><box><size>4 0.2 2</size></box></geometry></collision>
        <visual><geometry><box><size>4 0.2 2</size></box></geometry></visual>
      </link>
    </model>

    <model name="kitchen_wall_west">
      <pose>4 0 1 0 0 1.57</pose>
      <link name="link">
        <collision><geometry><box><size>4 0.2 2</size></box></geometry></collision>
        <visual><geometry><box><size>4 0.2 2</size></box></geometry></visual>
      </link>
    </model>

    <!-- Door opening between rooms -->
    <model name="door_frame">
      <pose>4 0 1 0 0 0</pose>
      <link name="link">
        <collision><geometry><box><size>0.2 2 2</size></box></geometry></collision>
        <visual><geometry><box><size>0.2 2 2</size></box></geometry></visual>
      </link>
    </model>

    <!-- Furniture for humanoid interaction -->
    <model name="sofa">
      <pose>-2 0 0.3 0 0 0</pose>
      <link name="sofa_base">
        <collision><geometry><box><size>2 0.8 0.6</size></box></geometry></collision>
        <visual><geometry><box><size>2 0.8 0.6</size></box></geometry></visual>
      </link>
    </model>

    <model name="dining_table">
      <pose>5 -1 0.4 0 0 0</pose>
      <link name="table_top">
        <collision><geometry><box><size>1.5 0.8 0.05</size></box></geometry></collision>
        <visual><geometry><box><size>1.5 0.8 0.05</size></box></geometry></visual>
      </link>
    </model>

    <!-- Interactive objects -->
    <model name="box_1">
      <pose>0 2 0.5 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <collision><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision>
        <visual><geometry><box><size>0.3 0.3 0.3</size></box></geometry></visual>
        <inertial>
          <mass>2.0</mass>
          <inertia><ixx>0.05</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.05</iyy><iyz>0</iyz><izz>0.05</izz></inertia>
        </inertial>
      </link>
    </model>

    <!-- Stairs for humanoid climbing test -->
    <model name="stairs">
      <pose>-3 -4 0 0 0 0</pose>
      <link name="step_1">
        <pose>0 0 0.1 0 0 0</pose>
        <collision><geometry><box><size>1 0.5 0.2</size></box></geometry></collision>
        <visual><geometry><box><size>1 0.5 0.2</size></box></geometry></visual>
      </link>
      <link name="step_2">
        <pose>0 0 0.3 0 0 0</pose>
        <collision><geometry><box><size>1 0.5 0.2</size></box></geometry></collision>
        <visual><geometry><box><size>1 0.5 0.2</size></box></geometry></visual>
      </link>
      <link name="step_3">
        <pose>0 0 0.5 0 0 0</pose>
        <collision><geometry><box><size>1 0.5 0.2</size></box></geometry></collision>
        <visual><geometry><box><size>1 0.5 0.2</size></box></geometry></visual>
      </link>
    </model>

    <!-- Lighting configuration -->
    <light name="room_light_1" type="point">
      <pose>0 0 3 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <attenuation>
        <range>10</range>
        <linear>0.1</linear>
        <quadratic>0.01</quadratic>
      </attenuation>
    </light>

    <!-- GUI configuration -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>8 8 5 0 0.5 1.57</pose>
      </camera>
    </gui>
  </world>
</sdf>
```

## Outdoor Environments

### Urban Street Scene

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="urban_street">
    <!-- Standard includes -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground surface - more realistic outdoor material -->
    <model name="road">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode><mu>0.8</mu><mu2>0.8</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Sidewalks -->
    <model name="sidewalk_left">
      <pose>-8 0 0.01 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><box><size>2 50 0.1</size></box></geometry></collision>
        <visual><geometry><box><size>2 50 0.1</size></box></geometry></visual>
      </link>
    </model>

    <model name="sidewalk_right">
      <pose>8 0 0.01 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><box><size>2 50 0.1</size></box></geometry></collision>
        <visual><geometry><box><size>2 50 0.1</size></box></geometry></visual>
      </link>
    </model>

    <!-- Buildings -->
    <model name="building_1">
      <pose>-10 10 5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><box><size>8 8 10</size></box></geometry></collision>
        <visual><geometry><box><size>8 8 10</size></box></geometry></visual>
      </link>
    </model>

    <model name="building_2">
      <pose>10 -10 7 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><box><size>10 10 14</size></box></geometry></collision>
        <visual><geometry><box><size>10 10 14</size></box></geometry></visual>
      </link>
    </model>

    <!-- Street elements -->
    <model name="traffic_sign">
      <pose>0 15 2 0 0 0</pose>
      <link name="pole">
        <collision><geometry><cylinder><radius>0.05</radius><length>3</length></cylinder></geometry></collision>
        <visual><geometry><cylinder><radius>0.05</radius><length>3</length></cylinder></geometry></visual>
      </link>
      <link name="sign">
        <pose>0 0 2.8 0 0 0</pose>
        <collision><geometry><box><size>0.8 0.5 0.05</size></box></geometry></collision>
        <visual><geometry><box><size>0.8 0.5 0.05</size></box></geometry></visual>
      </link>
    </model>

    <!-- Trees for realistic environment -->
    <model name="tree_1">
      <pose>5 5 2.5 0 0 0</pose>
      <static>true</static>
      <link name="trunk">
        <collision><geometry><cylinder><radius>0.2</radius><length>4</length></cylinder></geometry></collision>
        <visual><geometry><cylinder><radius>0.2</radius><length>4</length></cylinder></geometry></visual>
      </link>
      <link name="leaves">
        <pose>0 0 4.5 0 0 0</pose>
        <collision><geometry><sphere><radius>1.5</radius></sphere></geometry></collision>
        <visual><geometry><sphere><radius>1.5</radius></sphere></geometry></visual>
      </link>
    </model>

    <!-- Parked cars -->
    <model name="car_1">
      <pose>-3 8 0.7 0 0 0</pose>
      <static>false</static>
      <link name="chassis">
        <collision><geometry><box><size>4 1.8 1.5</size></box></geometry></collision>
        <visual><geometry><box><size>4 1.8 1.5</size></box></geometry></visual>
      </link>
    </model>

    <!-- Pedestrian path markers -->
    <model name="crosswalk_line_1">
      <pose>0 0 0.02 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><box><size>0.5 4 0.01</size></box></geometry></collision>
        <visual><geometry><box><size>0.5 4 0.01</size></box></geometry></visual>
      </link>
    </model>

    <!-- GUI -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>15 15 15 0 0.5 1.57</pose>
      </camera>
    </gui>
  </world>
</sdf>
```

## Specialized Testing Environments

### Balance and Locomotion Test World

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="balance_test">
    <!-- Standard includes -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics optimized for balance testing -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.2</sor>
        </solver>
        <constraints>
          <cfm>1e-5</cfm>
          <erp>0.2</erp>
        </constraints>
      </ode>
    </physics>

    <!-- Main testing platform -->
    <model name="main_platform">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><box><size>20 20 0.1</size></box></geometry></collision>
        <visual><geometry><box><size>20 20 0.1</size></box></geometry></visual>
        <surface>
          <friction>
            <ode><mu>1.0</mu><mu2>1.0</mu2></ode>
          </friction>
        </surface>
      </link>
    </model>

    <!-- Balance beam -->
    <model name="balance_beam">
      <pose>0 -5 0.15 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <collision><geometry><box><size>2 0.1 0.1</size></box></geometry></collision>
        <visual><geometry><box><size>2 0.1 0.1</size></box></geometry></visual>
        <inertial>
          <mass>5.0</mass>
          <inertia><ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.1</iyy><iyz>0</iyz><izz>0.1</izz></inertia>
        </inertial>
      </link>
    </model>

    <!-- Stepping stones -->
    <model name="stepping_stone_1">
      <pose>-2 2 0.1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><cylinder><radius>0.2</radius><length>0.2</length></cylinder></geometry></collision>
        <visual><geometry><cylinder><radius>0.2</radius><length>0.2</length></cylinder></geometry></visual>
      </link>
    </model>

    <model name="stepping_stone_2">
      <pose>0 2 0.1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><cylinder><radius>0.2</radius><length>0.2</length></cylinder></geometry></collision>
        <visual><geometry><cylinder><radius>0.2</radius><length>0.2</length></cylinder></geometry></visual>
      </link>
    </model>

    <model name="stepping_stone_3">
      <pose>2 2 0.1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><cylinder><radius>0.2</radius><length>0.2</length></cylinder></geometry></collision>
        <visual><geometry><cylinder><radius>0.2</radius><length>0.2</length></cylinder></geometry></visual>
      </link>
    </model>

    <!-- Inclined planes for walking tests -->
    <model name="ramp_1">
      <pose>-5 5 0.2 0 0.2 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><box><size>2 1 0.1</size></box></geometry></collision>
        <visual><geometry><box><size>2 1 0.1</size></box></geometry></visual>
      </link>
    </model>

    <model name="ramp_2">
      <pose>5 5 0.2 0 -0.2 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><box><size>2 1 0.1</size></box></geometry></collision>
        <visual><geometry><box><size>2 1 0.1</size></box></geometry></visual>
      </link>
    </model>

    <!-- Obstacle course -->
    <model name="low_barrier_1">
      <pose>-8 0 0.2 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><box><size>0.1 2 0.4</size></box></geometry></collision>
        <visual><geometry><box><size>0.1 2 0.4</size></box></geometry></visual>
      </link>
    </model>

    <model name="low_barrier_2">
      <pose>8 0 0.2 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><box><size>0.1 2 0.4</size></box></geometry></collision>
        <visual><geometry><box><size>0.1 2 0.4</size></box></geometry></visual>
      </link>
    </model>

    <!-- Soft surface for compliance testing -->
    <model name="soft_surface">
      <pose>5 -5 0.05 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><box><size>2 2 0.1</size></box></geometry></collision>
        <visual><geometry><box><size>2 2 0.1</size></box></geometry></visual>
        <surface>
          <friction>
            <ode><mu>0.9</mu><mu2>0.9</mu2></ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.1</restitution_coefficient>
          </bounce>
        </surface>
      </link>
    </model>

    <!-- Measurement markers -->
    <model name="start_line">
      <pose>0 -8 0.02 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision><geometry><box><size>0.1 10 0.01</size></box></geometry></collision>
        <visual><geometry><box><size>0.1 10 0.01</size></box></geometry></visual>
      </link>
    </model>

    <!-- GUI -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 0 10 0 1.57 0</pose>
        <track_visual>
          <name>humanoid_robot</name>
          <type>rotation</type>
        </track_visual>
      </camera>
    </gui>
  </world>
</sdf>
```

## Dynamic and Interactive Elements

### Moving Obstacles World

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="dynamic_obstacles">
    <!-- Standard includes -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Main arena -->
    <model name="arena">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      <link name="floor">
        <collision><geometry><box><size>20 20 0.1</size></box></geometry></collision>
        <visual><geometry><box><size>20 20 0.1</size></box></geometry></visual>
      </link>
      <link name="wall_north">
        <pose>0 10 1 0 0 0</pose>
        <collision><geometry><box><size>20 0.2 2</size></box></geometry></collision>
        <visual><geometry><box><size>20 0.2 2</size></box></geometry></visual>
      </link>
      <link name="wall_south">
        <pose>0 -10 1 0 0 0</pose>
        <collision><geometry><box><size>20 0.2 2</size></box></geometry></collision>
        <visual><geometry><box><size>20 0.2 2</size></box></geometry></visual>
      </link>
      <link name="wall_east">
        <pose>10 0 1 0 0 1.57</pose>
        <collision><geometry><box><size>20 0.2 2</size></box></geometry></collision>
        <visual><geometry><box><size>20 0.2 2</size></box></geometry></visual>
      </link>
      <link name="wall_west">
        <pose>-10 0 1 0 0 1.57</pose>
        <collision><geometry><box><size>20 0.2 2</size></box></geometry></collision>
        <visual><geometry><box><size>20 0.2 2</size></box></geometry></visual>
      </link>
    </model>

    <!-- Moving obstacles -->
    <model name="moving_obstacle_1">
      <pose>-8 0 0.5 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <collision><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>
        <visual><geometry><sphere><radius>0.5</radius></sphere></geometry></visual>
        <inertial>
          <mass>2.0</mass>
          <inertia><ixx>0.2</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.2</iyy><iyz>0</iyz><izz>0.2</izz></inertia>
        </inertial>
      </link>
      <!-- SDF plugin for movement would go here -->
    </model>

    <model name="moving_obstacle_2">
      <pose>8 5 0.5 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <collision><geometry><cylinder><radius>0.3</radius><length>1.0</length></cylinder></geometry></collision>
        <visual><geometry><cylinder><radius>0.3</radius><length>1.0</length></cylinder></geometry></visual>
        <inertial>
          <mass>3.0</mass>
          <inertia><ixx>0.3</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.3</iyy><iyz>0</iyz><izz>0.3</izz></inertia>
        </inertial>
      </link>
    </model>

    <!-- Moving platform -->
    <model name="moving_platform">
      <pose>0 5 0.2 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <collision><geometry><box><size>2 1 0.1</size></box></geometry></collision>
        <visual><geometry><box><size>2 1 0.1</size></box></geometry></visual>
        <inertial>
          <mass>10.0</mass>
          <inertia><ixx>2.0</ixx><ixy>0</ixy><ixz>0</ixz><iyy>2.0</iyy><iyz>0</iyz><izz>2.0</izz></inertia>
        </inertial>
      </link>
    </model>

    <!-- Interactive objects -->
    <model name="pushable_box">
      <pose>-5 -5 0.5 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <collision><geometry><box><size>0.6 0.6 0.6</size></box></geometry></collision>
        <visual><geometry><box><size>0.6 0.6 0.6</size></box></geometry></visual>
        <inertial>
          <mass>5.0</mass>
          <inertia><ixx>0.3</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.3</iyy><iyz>0</iyz><izz>0.3</izz></inertia>
        </inertial>
        <self_collide>false</self_collide>
        <kinematic>false</kinematic>
        <gravity>true</gravity>
      </link>
    </model>

    <!-- GUI -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>15 15 10 0 0.5 1.57</pose>
      </camera>
    </gui>
  </world>
</sdf>
```

## World Customization and Parameterization

### Parameterized World Template

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="parametrized_environment">
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Configurable elements -->
    <!-- This world can be customized through SDF parameters or plugins -->

    <!-- Example: Spawn location markers -->
    <model name="spawn_point_1">
      <pose>0 0 0.1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual><geometry><cylinder><radius>0.1</radius><length>0.01</length></cylinder></geometry></visual>
      </link>
    </model>

    <model name="spawn_point_2">
      <pose>5 5 0.1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual><geometry><cylinder><radius>0.1</radius><length>0.01</length></cylinder></geometry></visual>
      </link>
    </model>

    <!-- GUI -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>10 10 10 0 0.5 1.57</pose>
      </camera>
    </gui>
  </world>
</sdf>
```

## World Loading and Management

### Launch File for World Loading

```python
# launch/humanoid_worlds.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    world_name = LaunchConfiguration('world', default='simple_room')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # World file path
    world_file = os.path.join(
        get_package_share_directory('simulation'),
        'worlds',
        f'{world_name}.world'
    )

    # Gazebo launch
    gazebo = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-world', world_file,
            '-entity', 'humanoid_robot',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='simple_room',
            description='Choose one of the world files from `/simulation/worlds`'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo
    ])
```

## World Performance Optimization

### Large World Optimization Techniques

For large or complex worlds:

1. **Level of Detail (LOD)**: Use simplified models when far from the robot
2. **Occlusion Culling**: Hide objects not visible to sensors
3. **Physics Optimization**: Use simpler collision geometries
4. **Model Instancing**: Reuse similar objects efficiently

### Multi-World Scenarios

```bash
# Load different worlds for different testing scenarios
# Navigation world
ros2 launch simulation load_world.launch.py world:=urban_street

# Manipulation world
ros2 launch simulation load_world.launch.py world:=multi_room_house

# Balance test world
ros2 launch simulation load_world.launch.py world:=balance_test
```

## Best Practices

1. **Start Simple**: Begin with basic worlds and add complexity gradually
2. **Performance Testing**: Monitor simulation performance with complex worlds
3. **Realistic Materials**: Use appropriate friction and collision properties
4. **Modular Design**: Create reusable world components
5. **Documentation**: Comment world files to explain the purpose of elements
6. **Validation**: Test worlds with your robot to ensure proper functionality
7. **Scalability**: Design worlds that can be easily modified for different scenarios

## Next Steps

In the next chapter, we'll explore Unity integration for humanoid robot simulation, focusing on high-fidelity rendering and perception training. Unity provides photorealistic environments that are ideal for developing and testing perception algorithms that require realistic visual data.