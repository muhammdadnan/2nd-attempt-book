# Topics

Topics form the backbone of communication in ROS 2, enabling asynchronous, many-to-many communication between nodes. In this chapter, we'll explore how to implement and use topics effectively for humanoid robot applications.

## Understanding Topics

Topics in ROS 2 use a publish-subscribe communication pattern:
- **Publishers** send messages to a topic
- **Subscribers** receive messages from a topic
- Multiple publishers and subscribers can use the same topic
- Communication is asynchronous and non-blocking

## Quality of Service (QoS) for Topics

QoS profiles allow fine-tuning of topic communication behavior:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# For sensor data (high frequency, may lose some messages)
sensor_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# For critical commands (must be delivered reliably)
command_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# For configuration data (keep last value for late-joining nodes)
config_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)
```

## Creating Publishers

Here's how to create a publisher in a ROS 2 node:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher with QoS profile
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10  # Queue size
        )

        # Timer to publish messages at regular intervals
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Initialize joint information
        self.joint_names = [
            'left_shoulder_joint', 'right_shoulder_joint',
            'left_elbow_joint', 'right_elbow_joint',
            'left_hip_joint', 'right_hip_joint',
            'left_knee_joint', 'right_knee_joint'
        ]

        self.time = 0.0
        self.get_logger().info('Joint state publisher started')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = []

        # Generate oscillating joint positions
        for i, _ in enumerate(self.joint_names):
            position = 0.5 * math.sin(self.time + i * 0.5)
            msg.position.append(position)

        self.publisher.publish(msg)
        self.time += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()

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

## Creating Subscribers

Subscribers receive messages from topics:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')

        # Create subscriber with callback
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10  # Queue size
        )

        # Adjust QoS if needed
        # self.subscription = self.create_subscription(
        #     JointState,
        #     '/joint_states',
        #     self.joint_state_callback,
        #     qos_profile=sensor_qos
        # )

        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Joint state subscriber started')

    def joint_state_callback(self, msg):
        self.get_logger().info(
            f'Received joint states: {len(msg.name)} joints, '
            f'first joint position: {msg.position[0] if msg.position else "N/A"}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()

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

## Common Topic Types for Humanoid Robots

### Sensor Data Topics
- `/joint_states` - Joint positions, velocities, efforts
- `/imu/data` - Inertial measurement unit data
- `/camera/image_raw` - Raw camera images
- `/scan` - Laser scan data
- `/tf` and `/tf_static` - Transform data

### Control Command Topics
- `/cmd_vel` - Velocity commands
- `/joint_group_position_controller/commands` - Joint position commands
- `/joint_group_velocity_controller/commands` - Joint velocity commands

## Implementing a Humanoid Sensor Publisher

Let's create a comprehensive example that publishes multiple sensor streams:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import math
import numpy as np

class HumanoidSensorPublisher(Node):
    def __init__(self):
        super().__init__('humanoid_sensor_publisher')

        # Publishers for different sensor types
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Timer for sensor publishing
        self.sensor_timer = self.create_timer(0.02, self.publish_sensors)  # 50 Hz

        # Initialize joint information
        self.joint_names = [
            'left_shoulder_joint', 'right_shoulder_joint',
            'left_elbow_joint', 'right_elbow_joint',
            'left_hip_joint', 'right_hip_joint',
            'left_knee_joint', 'right_knee_joint'
        ]

        self.time = 0.0
        self.get_logger().info('Humanoid sensor publisher initialized')

    def publish_sensors(self):
        # Publish joint states
        self.publish_joint_states()

        # Publish IMU data
        self.publish_imu_data()

        # Update time for next iteration
        self.time += 0.02

    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = []

        # Generate realistic joint positions with some coordination
        for i, _ in enumerate(self.joint_names):
            # Different patterns for different joint types
            if 'shoulder' in self.joint_names[i]:
                position = 0.3 * math.sin(self.time + i * 0.3)
            elif 'elbow' in self.joint_names[i]:
                position = 0.2 * math.cos(self.time + i * 0.3)
            elif 'hip' in self.joint_names[i]:
                position = 0.4 * math.sin(self.time * 0.5 + i * 0.2)
            else:  # knee
                position = 0.3 * math.cos(self.time * 0.5 + i * 0.2)

            msg.position.append(position)

        self.joint_pub.publish(msg)

    def publish_imu_data(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulate IMU data with some noise
        # Orientation (simplified - just showing concept)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(self.time * 0.1) * 0.1
        msg.orientation.w = math.cos(self.time * 0.1) * 0.1

        # Angular velocity
        msg.angular_velocity.x = 0.1 * math.cos(self.time * 2)
        msg.angular_velocity.y = 0.05 * math.sin(self.time * 1.5)
        msg.angular_velocity.z = 0.02 * math.cos(self.time * 1.8)

        # Linear acceleration
        msg.linear_acceleration.x = 0.5 * math.sin(self.time * 3)
        msg.linear_acceleration.y = 0.3 * math.cos(self.time * 2.5)
        msg.linear_acceleration.z = 9.8 + 0.2 * math.sin(self.time * 4)  # Gravity + movement

        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidSensorPublisher()

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

## Topic Monitoring and Debugging

ROS 2 provides tools for monitoring topics:

```bash
# List all topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /joint_states

# Echo with specific fields
ros2 topic echo /joint_states position[0]

# Get topic info
ros2 topic info /joint_states

# Publish to a topic from command line
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
```

## Best Practices for Topics

1. **Use appropriate QoS profiles** for your application needs
2. **Follow naming conventions** (e.g., `/namespace/topic_name`)
3. **Choose appropriate message types** from standard ROS 2 message packages
4. **Consider message frequency** and bandwidth requirements
5. **Use latching** for static configuration data
6. **Monitor topic bandwidth** to avoid network congestion
7. **Implement proper error handling** for communication failures

## Advanced Topic Patterns

### Latching for Static Data
For topics that don't change frequently, use latching:

```python
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

latched_qos = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)

latched_publisher = self.create_publisher(
    String,
    '/robot_description',
    qos_profile=latched_qos
)
```

### Custom Message Types
For humanoid-specific data, you might create custom messages:

```python
# In your package's msg directory, create HumanoidStatus.msg:
# string robot_name
# float64 battery_level
# float64[] joint_temperatures
# bool[] joint_errors
```

## Next Steps

In the next chapter, we'll explore services and actions, which provide synchronous request-response communication and support for long-running tasks with feedback, respectively. These are essential for humanoid robot control systems.