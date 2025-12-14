# Physics Tuning

Physics tuning is crucial for creating realistic simulation behavior that matches real-world robot performance. In this chapter, we'll explore how to fine-tune physics parameters to achieve accurate and stable humanoid robot simulation.

## Understanding Physics Parameters

### Time Step and Update Rate

The simulation time step affects both accuracy and stability:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- 1ms - good for humanoid control -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>  <!-- 1000 Hz -->
</physics>
```

**Guidelines for humanoid robots:**
- Use 1ms time step for precise control
- Match controller update rate to physics rate
- Balance accuracy with performance requirements

### Solver Parameters

Tune solver parameters for optimal performance:

```xml
<ode>
  <solver>
    <type>quick</type>
    <iters>100</iters>  <!-- More iterations = more accurate but slower -->
    <sor>1.3</sor>      <!-- Successive Over-Relaxation parameter -->
  </solver>
  <constraints>
    <cfm>0.000001</cfm>  <!-- Constraint Force Mixing -->
    <erp>0.2</erp>      <!-- Error Reduction Parameter -->
  </constraints>
</ode>
```

## Tuning for Humanoid Stability

### Center of Mass Optimization

Accurate center of mass is critical for balance:

```xml
<link name="torso">
  <inertial>
    <mass value="3.0"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>  <!-- Adjust for actual CoM -->
    <inertia ixx="0.08" ixy="0.0" ixz="0.0"
             iyy="0.08" iyz="0.0" izz="0.04"/>
  </inertial>
</link>
```

### Friction Tuning

For humanoid feet to maintain stable contact:

```xml
<gazebo reference="left_foot">
  <mu1>1.0</mu1>    <!-- High friction for stable walking -->
  <mu2>1.0</mu2>
  <fdir1>1 0 0</fdir1>  <!-- Friction direction -->
</gazebo>
```

### Contact Stiffness and Damping

Balance stability and realistic response:

```xml
<gazebo reference="left_foot">
  <kp>10000000.0</kp>  <!-- High stiffness for stable contact -->
  <kd>1000000.0</kd>   <!-- Appropriate damping to prevent oscillation -->
</gazebo>
```

## Joint-Specific Tuning

### Hip Joint Tuning (Critical for Walking)

```xml
<joint name="left_hip_joint" type="revolute">
  <parent link="base_link"/>
  <child link="left_upper_leg"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="20" velocity="1.5"/>
  <dynamics damping="1.0" friction="0.2"/>  <!-- Higher damping for stability -->
</joint>
```

### Knee Joint Tuning (Critical for Support)

```xml
<joint name="left_knee_joint" type="revolute">
  <parent link="left_upper_leg"/>
  <child link="left_lower_leg"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="${M_PI/2}" effort="18" velocity="1.5"/>
  <dynamics damping="1.5" friction="0.3"/>  <!-- Higher damping for support -->
</joint>
```

### Ankle Joint Tuning (Fine Control)

```xml
<joint name="left_ankle_joint" type="revolute">
  <parent link="left_lower_leg"/>
  <child link="left_foot"/>
  <origin xyz="0 0 -0.35" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="10" velocity="1"/>
  <dynamics damping="0.5" friction="0.1"/>  <!-- Lower damping for fine control -->
</joint>
```

## Systematic Tuning Process

### Step 1: Static Balance Test

Start by testing if the robot can stand:

```xml
<!-- World file with minimal physics -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.2</sor>
    </solver>
  </ode>
</physics>
```

### Step 2: Joint Damping Tuning

Gradually adjust joint damping:

```python
# Test script to evaluate balance
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

class BalanceEvaluator(Node):
    def __init__(self):
        super().__init__('balance_evaluator')

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        self.balance_pub = self.create_publisher(Float32, '/balance_score', 10)

        self.balance_score = 0.0

    def joint_callback(self, msg):
        # Calculate balance score based on joint positions
        # and robot orientation
        pass

def main(args=None):
    rclpy.init(args=args)
    evaluator = BalanceEvaluator()
    rclpy.spin(evaluator)
    evaluator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Walking Gait Validation

Test with a simple walking pattern:

```xml
<!-- Add a simple walking controller plugin -->
<gazebo>
  <plugin name="walking_controller" filename="libwalking_controller.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
    <stepHeight>0.1</stepHeight>
    <stepLength>0.3</stepLength>
    <stepDuration>1.0</stepDuration>
  </plugin>
</gazebo>
```

## Advanced Tuning Techniques

### Adaptive Parameter Tuning

Use parameter servers to adjust physics in real-time:

```python
class PhysicsTuner(Node):
    def __init__(self):
        super().__init__('physics_tuner')

        # Declare parameters for physics tuning
        self.declare_parameter('contact_stiffness', 10000000.0)
        self.declare_parameter('contact_damping', 1000000.0)
        self.declare_parameter('friction_coefficient', 1.0)

        # Timer to update physics parameters
        self.timer = self.create_timer(1.0, self.update_physics_params)

    def update_physics_params(self):
        # Update Gazebo physics parameters via service call
        stiffness = self.get_parameter('contact_stiffness').value
        damping = self.get_parameter('contact_damping').value
        friction = self.get_parameter('friction_coefficient').value

        # Apply parameters to relevant links
        self.apply_physics_params('left_foot', stiffness, damping, friction)
        self.apply_physics_params('right_foot', stiffness, damping, friction)

def main(args=None):
    rclpy.init(args=args)
    tuner = PhysicsTuner()
    rclpy.spin(tuner)
    tuner.destroy_node()
    rclpy.shutdown()
```

### Multi-Objective Optimization

Balance multiple physics objectives:

```python
class PhysicsOptimizer:
    def __init__(self):
        self.objectives = {
            'stability': 0.4,      # Robot stability
            'realism': 0.3,        # Physical realism
            'performance': 0.3     # Simulation performance
        }

    def optimize_parameters(self, current_params):
        # Multi-objective optimization algorithm
        # Consider: stability, computational cost, physical accuracy
        pass

    def evaluate_fitness(self, params):
        # Evaluate how well parameters meet objectives
        stability_score = self.evaluate_stability(params)
        realism_score = self.evaluate_realism(params)
        performance_score = self.evaluate_performance(params)

        fitness = (self.objectives['stability'] * stability_score +
                  self.objectives['realism'] * realism_score +
                  self.objectives['performance'] * performance_score)

        return fitness
```

## Physics Validation Methods

### Comparison with Real Robot Data

Validate simulation against real robot behavior:

```python
class PhysicsValidator(Node):
    def __init__(self):
        super().__init__('physics_validator')

        # Subscribers for both sim and real robot data
        self.sim_joint_sub = self.create_subscription(
            JointState, '/sim/joint_states', self.sim_joint_callback, 10)

        self.real_joint_sub = self.create_subscription(
            JointState, '/real/joint_states', self.real_joint_callback, 10)

    def compare_trajectories(self, sim_traj, real_traj):
        # Calculate error metrics
        position_error = np.mean(np.abs(sim_traj.position - real_traj.position))
        velocity_error = np.mean(np.abs(sim_traj.velocity - real_traj.velocity))

        return {
            'position_error': position_error,
            'velocity_error': velocity_error,
            'total_error': position_error + velocity_error
        }
```

### Sensitivity Analysis

Test parameter sensitivity:

```python
def sensitivity_analysis():
    base_params = {
        'kp': 1e7,
        'kd': 1e6,
        'mu': 1.0,
        'damping': 1.0
    }

    param_variations = {
        'kp': [5e6, 1e7, 2e7],
        'kd': [5e5, 1e6, 2e6],
        'mu': [0.5, 1.0, 1.5],
        'damping': [0.5, 1.0, 1.5]
    }

    results = {}
    for param, values in param_variations.items():
        param_results = []
        for value in values:
            test_params = base_params.copy()
            test_params[param] = value

            error = test_physics_configuration(test_params)
            param_results.append((value, error))

        results[param] = param_results

    return results
```

## Common Tuning Scenarios

### Walking Stability Tuning

For stable bipedal walking:

```xml
<!-- Feet with high friction and appropriate stiffness -->
<gazebo reference="left_foot">
  <mu1>1.2</mu1>
  <mu2>1.2</mu2>
  <kp>15000000.0</kp>  <!-- Higher for walking stability -->
  <kd>1500000.0</kd>
</gazebo>

<!-- Hip joints with moderate damping -->
<joint name="left_hip_joint" type="revolute">
  <dynamics damping="1.2" friction="0.15"/>  <!-- Balance control vs stability -->
</joint>

<!-- Ankle joints with lower damping for compliance -->
<joint name="left_ankle_joint" type="revolute">
  <dynamics damping="0.3" friction="0.05"/>  <!-- Allow some compliance -->
</joint>
```

### Manipulation Tuning

For stable manipulation tasks:

```xml
<!-- Hands/fingers with appropriate friction -->
<gazebo reference="left_hand">
  <mu1>0.8</mu1>  <!-- Good for grasping objects -->
  <mu2>0.8</mu2>
  <kp>5000000.0</kp>  <!-- Lower than feet for manipulation compliance -->
  <kd>500000.0</kd>
</gazebo>

<!-- Arm joints with lower damping for smoother motion -->
<joint name="left_shoulder_joint" type="revolute">
  <dynamics damping="0.3" friction="0.05"/>
</joint>
```

## Performance Optimization

### Adaptive Time Stepping

For mixed real-time and accuracy requirements:

```xml
<physics type="ode">
  <max_step_size>0.002</max_step_size>  <!-- Larger for performance -->
  <real_time_factor>0.8</real_time_factor>  <!-- Slightly slower than real-time -->
  <real_time_update_rate>500.0</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>30</iters>  <!-- Fewer iterations for performance -->
      <sor>1.2</sor>
    </solver>
  </ode>
</physics>
```

### Selective High-Fidelity Regions

Apply high-fidelity physics only where needed:

```xml
<!-- High-fidelity for feet and hands -->
<gazebo reference="left_foot">
  <kp>20000000.0</kp>
  <kd>2000000.0</kd>
  <max_step_size>0.0005</max_step_size>  <!-- Smaller for critical contacts -->
</gazebo>

<!-- Lower fidelity for torso (less critical) -->
<gazebo reference="torso">
  <kp>1000000.0</kp>  <!-- Lower stiffness -->
  <kd>100000.0</kd>
</gazebo>
```

## Debugging Physics Issues

### Common Problems and Solutions

**Problem**: Robot falls through ground
**Solution**: Increase contact stiffness and verify collision geometry

**Problem**: Unstable oscillations
**Solution**: Increase damping, adjust solver parameters

**Problem**: Robot slides unrealistically
**Solution**: Increase friction coefficients

**Problem**: Joints behave erratically
**Solution**: Check joint limits, damping, and inertial properties

### Physics Debugging Tools

```bash
# Monitor physics performance
gz stats

# Visualize contacts (if available)
gz topic -e /gazebo/default/physics/contacts

# Check model states
gz topic -e /gazebo/default/model/robot_name/state
```

## Best Practices

1. **Start conservative**: Begin with stable but potentially over-damped parameters
2. **Iterate gradually**: Make small changes and test thoroughly
3. **Validate continuously**: Compare simulation to real robot behavior
4. **Document parameters**: Keep records of successful configurations
5. **Consider task-specific tuning**: Different tasks may need different parameters
6. **Balance performance and accuracy**: Optimize for your specific use case

## Next Steps

In the next chapter, we'll explore sensor simulation in detail, learning how to model various robot sensors (IMU, cameras, LiDAR, etc.) in Gazebo and ensure they provide realistic data for perception and control systems.