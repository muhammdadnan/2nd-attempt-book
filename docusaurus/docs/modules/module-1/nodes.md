# Nodes

Nodes are the fundamental building blocks of ROS 2 applications. In this chapter, we'll explore how to create, structure, and manage ROS 2 nodes for humanoid robot applications.

## Understanding ROS 2 Nodes

A node in ROS 2 is an executable that uses ROS 2 client libraries to communicate with other nodes. Each node:
- Runs in its own process
- Communicates with other nodes through topics, services, or actions
- Can be written in different programming languages
- Has its own set of parameters and lifecycle

## Creating a Basic Node in Python

Here's a simple ROS 2 node structure:

```python
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Simple node has been created')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()

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

## Node Lifecycle

ROS 2 nodes have a well-defined lifecycle:

1. **Unconfigured**: Initial state after creation
2. **Inactive**: After configuration, ready to activate
3. **Active**: Running and processing callbacks
4. **Finalized**: Node is shutting down

## Creating a Humanoid Robot Control Node

Let's create a more complex node for humanoid robot control:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Create publisher for joint states
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Initialize joint positions
        self.joint_names = [
            'left_shoulder_joint', 'right_shoulder_joint',
            'left_elbow_joint', 'right_elbow_joint',
            'left_hip_joint', 'right_hip_joint',
            'left_knee_joint', 'right_knee_joint'
        ]

        self.joint_positions = [0.0] * len(self.joint_names)
        self.time_step = 0.0

        self.get_logger().info('Humanoid controller node initialized')

    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions

        # Update joint positions with a simple oscillating pattern
        self.time_step += 0.1
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = 0.5 * math.sin(self.time_step + i * 0.5)

        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidController()

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

## Node Parameters

Nodes can accept parameters to customize their behavior:

```python
class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('parameterized_node')

        # Declare parameters with default values
        self.declare_parameter('loop_rate', 10)
        self.declare_parameter('robot_name', 'humanoid_robot')

        # Get parameter values
        self.loop_rate = self.get_parameter('loop_rate').value
        self.robot_name = self.get_parameter('robot_name').value

        self.get_logger().info(
            f'Node initialized with robot_name: {self.robot_name}, '
            f'loop_rate: {self.loop_rate}'
        )
```

## Node Composition

For performance-critical applications, multiple nodes can be composed into a single process:

```python
from rclpy.node import Node
from rclpy import executors
from rclpy.executors import MultiThreadedExecutor
import threading

class CompositeNode(Node):
    def __init__(self):
        super().__init__('composite_node')

        # Create multiple internal components
        self.controller = HumanoidController()
        self.sensor_processor = SensorProcessor()

        # Setup timers for each component
        self.create_timer(0.1, self.controller.update)
        self.create_timer(0.05, self.sensor_processor.process)
```

## Best Practices

1. **Use meaningful node names** that reflect their function
2. **Properly handle node destruction** to prevent resource leaks
3. **Use parameters** for configurable behavior
4. **Implement proper error handling** and logging
5. **Follow naming conventions** for topics, services, and parameters
6. **Consider computational requirements** when designing node structure

## Creating Your First Humanoid Node

Let's create a complete example of a humanoid robot node:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import math

class HumanoidBaseController(Node):
    def __init__(self):
        super().__init__('humanoid_base_controller')

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Initialize variables
        self.joint_names = [
            'left_hip_joint', 'right_hip_joint',
            'left_knee_joint', 'right_knee_joint'
        ]
        self.joint_positions = [0.0] * len(self.joint_names)
        self.cmd_vel = Twist()

        self.get_logger().info('Humanoid base controller initialized')

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def control_loop(self):
        # Simple gait control based on velocity command
        current_time = self.get_clock().now()

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = current_time.to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions

        # Update joint positions based on command
        self.joint_positions[0] = 0.2 * math.sin(current_time.nanoseconds * 1e-9)
        self.joint_positions[1] = 0.2 * math.sin(current_time.nanoseconds * 1e-9 + math.pi)
        self.joint_positions[2] = 0.1 * math.sin(current_time.nanoseconds * 1e-9)
        self.joint_positions[3] = 0.1 * math.sin(current_time.nanoseconds * 1e-9 + math.pi)

        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidBaseController()

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

## Next Steps

In the next chapter, we'll explore topics in detail, learning how to implement publisher-subscriber communication patterns that are essential for robot sensor data and control commands.