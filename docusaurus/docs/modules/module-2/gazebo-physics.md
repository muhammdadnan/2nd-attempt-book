# Gazebo Physics

Gazebo's physics engine is the core component that provides realistic simulation of physical interactions. In this chapter, we'll explore how Gazebo models physics, configure physics parameters, and optimize physics simulation for humanoid robot applications.

## Physics Engine Overview

Gazebo supports multiple physics engines, each with different characteristics:

### ODE (Open Dynamics Engine)
- **Default engine** for Gazebo Classic
- Good balance of accuracy and performance
- Well-suited for ground vehicles and simple robots
- Supports various joint types and collision geometries

### Bullet
- **Default engine** for Ignition Gazebo
- More accurate contact simulation
- Better for complex interactions and manipulations
- Good performance with many objects

### Simbody
- High-accuracy multibody dynamics
- Excellent for complex articulated systems
- More computationally expensive
- Good for biomechanical simulations

## Physics Configuration in World Files

Physics parameters are configured in world files using the `<physics>` tag:

```xml
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>

      <!-- ODE-specific parameters -->
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Rest of world content -->
  </world>
</sdf>
```

## Key Physics Parameters

### Time Step Configuration
- `max_step_size`: Simulation time step (typically 0.001-0.01 seconds)
- `real_time_update_rate`: Updates per second (1/dt)
- `real_time_factor`: Simulation speed relative to real time

```xml
<max_step_size>0.001</max_step_size>  <!-- 1ms time step -->
<real_time_factor>1.0</real_time_factor>  <!-- Real-time simulation -->
<real_time_update_rate>1000.0</real_time_update_rate>  <!-- 1000 Hz -->
```

### Gravity Configuration
```xml
<gravity>0 0 -9.8</gravity>  <!-- Standard Earth gravity -->
```

### Solver Parameters
- **Iterations**: More iterations = more accurate but slower
- **SOR (Successive Over-Relaxation)**: Damping parameter (1.0-1.3 typical)

## Collision Detection

Gazebo uses multiple collision detection algorithms:

### Contact Parameters
```xml
<constraints>
  <cfm>0.0</cfm>  <!-- Constraint Force Mixing -->
  <erp>0.2</erp>  <!-- Error Reduction Parameter -->
  <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
  <contact_surface_layer>0.001</contact_surface_layer>
</constraints>
```

### Collision Properties in URDF/SDF
```xml
<link name="link_name">
  <collision>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>  <!-- Coefficient of friction -->
          <mu2>1.0</mu2>
          <slip1>0.0</slip1>
          <slip2>0.0</slip2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.1</restitution_coefficient>
        <threshold>100000.0</threshold>
      </bounce>
      <contact>
        <ode>
          <kp>1e+16</kp>  <!-- Spring stiffness -->
          <kd>1e+14</kd>  <!-- Damping coefficient -->
          <max_vel>100.0</max_vel>
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
    </surface>
  </collision>
</link>
```

## Physics for Humanoid Robots

Humanoid robots present unique physics challenges:

### Balance and Stability
- Accurate center of mass calculations
- Proper inertial properties
- Appropriate friction coefficients
- Realistic joint dynamics

### Contact Modeling
For feet contacting the ground:
```xml
<surface>
  <friction>
    <ode>
      <mu>0.8</mu>  <!-- High friction for stable walking -->
      <mu2>0.8</mu2>
    </ode>
  </friction>
  <contact>
    <ode>
      <kp>1e+7</kp>  <!-- Stiff contact for stable stance -->
      <kd>1e+5</kd>
    </ode>
  </contact>
</surface>
```

### Joint Dynamics
For realistic joint behavior:
```xml
<joint name="knee_joint" type="revolute">
  <parent link="upper_leg"/>
  <child link="lower_leg"/>
  <limit effort="20" velocity="1.5" lower="-0.1" upper="2.0"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

## Physics Performance Optimization

### Simulation Accuracy vs. Performance
- Smaller time steps = more accurate but slower
- More solver iterations = more accurate but slower
- Balance based on application requirements

### Common Performance Settings
```xml
<physics type="ode">
  <max_step_size>0.002</max_step_size>  <!-- 2ms for humanoid -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>500.0</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>20</iters>  <!-- Balance accuracy and speed -->
      <sor>1.2</sor>
    </solver>
  </ode>
</physics>
```

## Debugging Physics Issues

### Common Physics Problems in Humanoid Simulation

1. **Robot falls through ground**: Check collision geometry and surface parameters
2. **Unstable joints**: Verify inertial properties and joint limits
3. **Slipping feet**: Increase friction coefficients
4. **Oscillating behavior**: Adjust solver parameters

### Physics Debugging Tools

#### Contact Visualization
Enable contact visualization in Gazebo to see contact forces:
```bash
gzserver --verbose --play your_world.world
```

#### Physics Statistics
Monitor physics performance:
```bash
gz stats
```

## Advanced Physics Features

### Custom Physics Plugins

Create custom physics plugins for specialized behaviors:

```cpp
// Example physics plugin for humanoid balance
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

class HumanoidBalancePlugin : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    this->world = _world;
    // Custom balance control code
  }

private:
  gazebo::physics::WorldPtr world;
};

GZ_REGISTER_WORLD_PLUGIN(HumanoidBalancePlugin)
```

### Multi-Body Dynamics

For complex humanoid interactions:
```xml
<model name="humanoid_robot">
  <link name="base_link">
    <inertial>
      <mass>5.0</mass>
      <inertia>
        <ixx>0.1</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.1</iyy>
        <iyz>0</iyz>
        <izz>0.1</izz>
      </inertia>
    </inertial>
  </link>
  <!-- Additional links with proper inertial properties -->
</model>
```

## Physics Validation

Validate physics simulation by comparing with real robot data:

1. **Kinematic validation**: Verify joint positions match expected values
2. **Dynamic validation**: Compare forces and torques
3. **Balance validation**: Test stability under various conditions
4. **Sensor validation**: Ensure simulated sensors match real sensor data

## Best Practices

1. **Start simple**: Begin with basic physics parameters and refine
2. **Match real hardware**: Use actual robot inertial and friction values
3. **Validate regularly**: Compare simulation with real robot behavior
4. **Optimize for performance**: Balance accuracy with simulation speed
5. **Document parameters**: Keep track of physics settings for reproducibility

## Next Steps

In the next chapter, we'll learn how to adapt our URDF models for Gazebo simulation by adding Gazebo-specific tags and plugins. This is crucial for connecting your robot description to the physics simulation environment.