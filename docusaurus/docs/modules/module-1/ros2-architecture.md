# ROS 2 Architecture

Understanding the architecture of ROS 2 is crucial for developing robust robotic applications. This chapter covers the fundamental architectural concepts that make ROS 2 a powerful framework for robotics development.

## DDS-Based Architecture

ROS 2 is built on top of Data Distribution Service (DDS), a middleware standard for real-time, distributed systems. This architecture provides:

- **Decentralized communication**: No central master node as in ROS 1
- **Fault tolerance**: Individual node failures don't bring down the entire system
- **Real-time performance**: Deterministic message delivery with quality of service (QoS) settings
- **Multi-language support**: Native support for C++, Python, Java, and other languages

## Core Components

### Nodes
Nodes are the fundamental execution units in ROS 2. Each node:
- Runs a single process
- Communicates with other nodes through topics, services, or actions
- Can be written in different programming languages
- Manages its own lifecycle

### Communication Primitives

#### Topics (Publish/Subscribe)
- Asynchronous, many-to-many communication
- Used for streaming data like sensor readings
- Quality of Service (QoS) profiles for reliability and performance

#### Services (Request/Response)
- Synchronous, one-to-one communication
- Used for discrete requests like setting parameters
- Request-response pattern

#### Actions (Long-running Tasks)
- Asynchronous communication for long-running tasks
- Includes feedback and goal preemption
- Used for navigation, manipulation, etc.

### Packages
Packages organize ROS 2 code and resources:
- Source code
- Launch files
- Configuration files
- Documentation
- Dependencies

## Quality of Service (QoS) Settings

QoS settings allow fine-tuning communication behavior:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Example QoS profile for sensor data
sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# Example QoS profile for critical commands
command_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)
```

## Launch System

The launch system in ROS 2 provides:
- Declarative startup of multiple nodes
- Parameter configuration
- Conditional startup
- Integration with other tools

## Parameter System

Parameters in ROS 2:
- Can be set at startup or changed at runtime
- Support for hierarchical parameter names
- Automatic parameter validation
- Parameter callbacks

## TF2 (Transform Library)

TF2 handles coordinate frame transformations:
- Maintains tree of coordinate frames
- Provides transformations between frames
- Handles timing and interpolation
- Essential for robot localization and mapping

## Next Steps

In the next chapter, we'll dive into creating and managing ROS 2 nodes. You'll learn how to create your first ROS 2 node and understand its lifecycle management.