# Parameters and Launch

In this chapter, we'll explore how to configure ROS 2 nodes using parameters and how to launch complex systems using the ROS 2 launch system. These tools are essential for managing humanoid robot systems with many configurable components.

## Parameters

Parameters in ROS 2 allow you to configure node behavior without recompiling code. They can be set at startup or changed at runtime.

### Declaring and Using Parameters

Here's how to declare and use parameters in a ROS 2 node:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterizedHumanoidNode(Node):
    def __init__(self):
        super().__init__('parameterized_humanoid_node')

        # Declare parameters with default values and descriptions
        self.declare_parameter('robot_name', 'humanoid_robot',
                             'Name of the robot')
        self.declare_parameter('control_frequency', 100,
                             'Control loop frequency in Hz')
        self.declare_parameter('max_joint_velocity', 2.0,
                             'Maximum joint velocity in rad/s')
        self.declare_parameter('safety_enabled', True,
                             'Enable safety checks')

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        self.safety_enabled = self.get_parameter('safety_enabled').value

        self.get_logger().info(
            f'Node initialized with parameters:\n'
            f'  Robot name: {self.robot_name}\n'
            f'  Control frequency: {self.control_frequency} Hz\n'
            f'  Max joint velocity: {self.max_joint_velocity} rad/s\n'
            f'  Safety enabled: {self.safety_enabled}'
        )

        # Create a timer with the configured frequency
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop
        )

    def control_loop(self):
        # Use parameters in your control logic
        if self.safety_enabled:
            # Apply safety checks
            pass

        self.get_logger().info(f'Control loop running for {self.robot_name}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterizedHumanoidNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Parameter Callbacks

You can set up callbacks to handle parameter changes at runtime:

```python
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class ParameterCallbackNode(Node):
    def __init__(self):
        super().__init__('parameter_callback_node')

        # Declare parameter with descriptor
        descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Maximum joint velocity',
            additional_constraints='Must be positive',
            floating_point_range=[rcl_interfaces.msg.FloatingPointRange(from_value=0.1, to_value=10.0)]
        )

        self.declare_parameter('max_joint_velocity', 2.0, descriptor)

        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_joint_velocity':
                if param.value <= 0:
                    return SetParametersResult(successful=False, reason='Max velocity must be positive')

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterCallbackNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Setting Parameters from Command Line

Parameters can be set when launching a node:

```bash
# Set parameters from command line
ros2 run my_package my_node --ros-args -p robot_name:=atlas_robot -p control_frequency:=200

# Or from a parameter file
ros2 run my_package my_node --ros-args --params-file config/my_params.yaml
```

### Parameter Files

Create YAML files to store parameter configurations:

```yaml
# config/humanoid_params.yaml
parameterized_humanoid_node:
  ros__parameters:
    robot_name: 'humanoid_robot'
    control_frequency: 100
    max_joint_velocity: 2.0
    safety_enabled: true
    joint_limits:
      left_shoulder:
        min: -1.57
        max: 1.57
      right_shoulder:
        min: -1.57
        max: 1.57
    walking_gait:
      step_height: 0.1
      step_length: 0.3
      step_duration: 1.0
```

## Launch System

The ROS 2 launch system allows you to start multiple nodes with specific configurations in a coordinated way.

### Basic Launch File

Here's a basic launch file structure:

```python
# launch/humanoid_bringup.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_name = LaunchConfiguration('robot_name', default='humanoid_robot')
    config_file = LaunchConfiguration('config_file')

    # Find package share directory
    pkg_share = FindPackageShare('robot_description').find('robot_description')

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(
                os.path.join(pkg_share, 'urdf', 'humanoid_robot.urdf.xacro')
            ).read()
        }]
    )

    # Joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Humanoid controller node
    humanoid_controller = Node(
        package='robot_control',
        executable='humanoid_controller',
        name='humanoid_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_name': robot_name}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='humanoid_robot',
            description='Name of the robot'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to parameter configuration file'
        ),
        robot_state_publisher,
        joint_state_publisher,
        humanoid_controller
    ])
```

### Launch Files with Conditional Logic

Launch files can include conditional logic:

```python
# launch/humanoid_with_gazebo.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gazebo = LaunchConfiguration('use_gazebo', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Include Gazebo launch if requested
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        condition=IfCondition(use_gazebo)
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz node (only if not using Gazebo)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            get_package_share_directory('robot_description'),
            'rviz',
            'robot_config.rviz'
        ])],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='false',
            description='Launch Gazebo simulation if true'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz if true'
        ),
        gazebo_launch,
        robot_state_publisher,
        rviz_node
    ])
```

### Launch Files with Parameter Files

Launch files can load parameters from external files:

```python
# launch/humanoid_with_params.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration(
        'params_file',
        default=[get_package_share_directory('robot_control'), '/config/humanoid_params.yaml']
    )

    # Humanoid controller with parameter file
    humanoid_controller = Node(
        package='robot_control',
        executable='humanoid_controller',
        name='humanoid_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            params_file
        ],
        output='screen'
    )

    # Walking controller with specific parameters
    walking_controller = Node(
        package='robot_control',
        executable='walking_controller',
        name='walking_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'step_height': 0.1},
            {'step_length': 0.3},
            {'step_duration': 1.0}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=[get_package_share_directory('robot_control'), '/config/humanoid_params.yaml'],
            description='Path to parameter file'
        ),
        humanoid_controller,
        walking_controller
    ])
```

## Advanced Launch Concepts

### Event Handling in Launch Files

Launch files can handle events and dependencies:

```python
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    controller_node = Node(
        package='robot_control',
        executable='controller_node',
        name='controller_node'
    )

    monitor_node = Node(
        package='robot_monitoring',
        executable='monitor_node',
        name='monitor_node',
        condition=launch.conditions.IfCondition(LaunchConfiguration('enable_monitoring'))
    )

    # Start monitor after controller
    delayed_monitor = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_node,
            on_start=[
                monitor_node
            ]
        )
    )

    return LaunchDescription([
        controller_node,
        delayed_monitor
    ])
```

### Launch Substitutions

Launch files support various substitutions:

```python
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.actions import LogInfo

def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace', default='')

    # Log the robot namespace
    log_namespace = LogInfo(
        msg=['Robot namespace: ', robot_namespace]
    )

    controller_node = Node(
        package='robot_control',
        executable='controller_node',
        name='controller_node',
        namespace=robot_namespace,
        parameters=[{'robot_namespace': robot_namespace}]
    )

    return LaunchDescription([
        log_namespace,
        controller_node
    ])
```

## Parameter and Launch Best Practices

1. **Use parameter files** for complex configurations instead of command-line arguments
2. **Validate parameters** at startup to catch configuration errors early
3. **Use meaningful parameter names** with clear descriptions
4. **Organize launch files** by functionality (bringup, simulation, etc.)
5. **Use launch arguments** to make launch files flexible
6. **Include conditional logic** to enable/disable components as needed
7. **Use parameter callbacks** for runtime validation and coordination
8. **Document parameters** with descriptions and constraints

## Command Line Tools

ROS 2 provides command-line tools for working with parameters:

```bash
# List parameters of a node
ros2 param list /humanoid_controller

# Get a specific parameter
ros2 param get /humanoid_controller robot_name

# Set a parameter
ros2 param set /humanoid_controller control_frequency 200

# List launch arguments for a launch file
ros2 launch --show-args robot_description humanoid_bringup.launch.py

# Launch with parameters
ros2 launch robot_description humanoid_bringup.launch.py use_sim_time:=true robot_name:=atlas_robot
```

## Next Steps

In the next chapter, we'll explore rclpy (the Python client library) in more detail and how to integrate AI capabilities with ROS 2. This is crucial for creating intelligent humanoid robots that can perceive, reason, and act in complex environments.