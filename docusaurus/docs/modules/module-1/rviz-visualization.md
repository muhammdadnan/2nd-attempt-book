# RViz Visualization

RViz (Robot Visualizer) is the 3D visualization tool for ROS. In this chapter, we'll learn how to effectively visualize our humanoid robot, sensor data, and robot behavior in RViz for debugging, monitoring, and understanding robot systems.

## Introduction to RViz

RViz is a 3D visualization environment for displaying robot models, sensor data, and other ROS topics. It's essential for:
- Visualizing robot models and transformations (TF)
- Monitoring sensor data in real-time
- Debugging robot behavior
- Planning and navigation visualization
- Creating robot dashboards

## Setting Up RViz for Humanoid Robots

### Basic RViz Configuration

Create a basic RViz configuration file for your humanoid robot:

```yaml
# humanoid_robot_config.rviz
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1
        - /TF1
        - /TF1/Frames1
      Splitter Ratio: 0.5
    Tree Height: 605
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Description Topic:
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      Robot Description: robot_description
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 0.5
      Name: TF
      Show Arrows: true
      Show Names: false
      Tree:
        {}
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/2D Goal Pose
      Topic:
        ./topic: /goal_pose
    - Class: rviz_default_plugins/Publish Point
      Single click: true
      Topic:
        ./topic: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002f4fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002f4000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720020006300680061006e0067006500640000000000ffffffff0000000000000000fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740000000186000001060000030c00000261000000010000010f000002f4fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002f4000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d006501000000000000045000000000000000000000023f000002f400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Width: 1200
  X: 72
  Y: 60
```

## Essential RViz Displays for Humanoid Robots

### RobotModel Display

The RobotModel display shows your URDF model:

```python
# In your launch file
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(
                os.path.join(
                    get_package_share_directory('robot_description'),
                    'urdf',
                    'humanoid_robot.urdf.xacro'
                )
            ).read()
        }]
    )

    # Joint state publisher (for moving joints)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('robot_description'),
            'rviz',
            'humanoid_config.rviz'
        )]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])
```

### TF (Transform) Display

The TF display shows all coordinate frames and their relationships:

```xml
<!-- In your URDF, each link creates a TF frame -->
<link name="left_foot">
  <!-- This creates a "left_foot" frame in TF tree -->
</link>
```

Key TF settings for humanoid robots:
- **Show Names**: Enable to see frame names
- **Show Axes**: Enable to see coordinate axes
- **Marker Scale**: Adjust to make transforms visible
- **Frame Timeout**: Increase if frames are published infrequently

### Sensor Data Visualization

Add displays for common humanoid robot sensors:

#### LaserScan Display
```yaml
- Class: rviz_default_plugins/LaserScan
  Enabled: true
  Name: LaserScan
  Topic:
    Depth: 5
    Durability Policy: Volatile
    History Policy: Keep Last
    Reliability Policy: Reliable
    Value: /scan
```

#### Image Display
```yaml
- Class: rviz_default_plugins/Image
  Enabled: true
  Name: CameraImage
  Topic:
    Depth: 5
    Durability Policy: Volatile
    History Policy: Keep Last
    Reliability Policy: Reliable
    Value: /camera/image_raw
```

#### PointCloud2 Display
```yaml
- Class: rviz_default_plugins/PointCloud2
  Enabled: true
  Name: PointCloud
  Topic:
    Depth: 5
    Durability Policy: Volatile
    History Policy: Keep Last
    Reliability Policy: Reliable
    Value: /pointcloud
```

## Advanced RViz Displays for Humanoid Robots

### Path Display

For navigation and trajectory visualization:

```yaml
- Class: rviz_default_plugins/Path
  Enabled: true
  Name: PlannedPath
  Topic:
    Depth: 5
    Durability Policy: Volatile
    History Policy: Keep Last
    Reliability Policy: Reliable
    Value: /plan
  Color:
    Alpha: 1
    Blue: 0.1
    Green: 1
    Red: 0.1
  Line Style: Lines
  Line Width: 0.029999999329447746
```

### Marker Display

For custom visualization of robot behavior:

```python
# In your Python node
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class HumanoidVisualizationNode(Node):
    def __init__(self):
        super().__init__('humanoid_visualization')

        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # Timer to publish markers
        self.timer = self.create_timer(0.1, self.publish_markers)

    def publish_markers(self):
        # Create a marker for center of mass visualization
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "center_of_mass"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position (example: center of base link)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5  # Approximate CoM height
        marker.pose.orientation.w = 1.0

        # Size and color
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0  # Don't forget to set alpha!
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidVisualizationNode()

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

### Interactive Markers

For interactive control of humanoid robot poses:

```python
import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import Point

class HumanoidInteractiveControl(Node):
    def __init__(self):
        super().__init__('humanoid_interactive_control')

        # Create interactive marker server
        self.server = InteractiveMarkerServer(self, 'humanoid_control')
        self.menu_handler = MenuHandler()

        # Create menu options
        self.menu_handler.insert("Move to Position", callback=self.process_feedback)
        self.menu_handler.insert("Reset Position", callback=self.process_feedback)

        self.create_interactive_marker()

    def create_interactive_marker(self):
        # Create an interactive marker for end-effector control
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "end_effector_control"
        int_marker.description = "Left Hand Control"
        int_marker.pose.position.x = 0.5
        int_marker.pose.position.y = 0.2
        int_marker.pose.position.z = 1.0

        # Create a control for movement
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_x_y"

        # Add a sphere to the control
        from visualization_msgs.msg import Marker
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        control.markers.append(marker)
        control.always_visible = True
        int_marker.controls.append(control)

        # Add the interactive marker to the server
        self.server.insert(int_marker, self.process_feedback)
        self.menu_handler.apply(self.server, int_marker.name)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        self.get_logger().info(f'Feedback received: {feedback.marker_name}')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidInteractiveControl()

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

## RViz Configuration Best Practices

### Organizing Displays

Group related displays together:

1. **Robot Structure**: RobotModel, TF
2. **Sensors**: LaserScan, Image, PointCloud2
3. **Navigation**: Path, PoseArray, Marker
4. **Planning**: Trajectory, Waypoints

### Fixed Frame Selection

For humanoid robots, common fixed frames:
- `base_link`: Robot's base coordinate system
- `odom`: Odometry frame for navigation
- `map`: Global map frame for SLAM
- `world`: Global reference frame

### View Configuration

Optimize views for humanoid robot monitoring:

```yaml
Views:
  Current:
    Class: rviz_default_plugins/Orbit
    Distance: 3.0  # Adjust based on robot size
    Enable Stereo Rendering:
      Stereo Eye Separation: 0.05999999865889549
      Stereo Focal Distance: 1
      Swap Stereo Eyes: false
      Value: false
    Focal Point:
      X: 0
      Y: 0
      Z: 0.8  # Center on robot torso height
    Focal Shape Fixed Size: true
    Focal Shape Size: 0.05000000074505806
    Invert Z Axis: false
    Name: Current View
    Near Clip Distance: 0.009999999776482582
    Pitch: 0.5  # Slightly downward view
    Target Frame: <Fixed Frame>
    Value: Orbit (rviz)
    Yaw: 0.5
```

## Custom RViz Panels

Create custom panels for humanoid-specific controls:

```python
# custom_humanoid_panel.py
import rclpy
from rclpy.node import Node
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import String

class HumanoidControlPanel(QWidget):
    def __init__(self):
        super(HumanoidControlPanel, self).__init__()
        # Load UI file if you have one
        # loadUi(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'humanoid_control.ui'), self)

        # Initialize ROS node
        rclpy.init()
        self.node = rclpy.create_node('humanoid_control_panel')

        # Create publishers for humanoid commands
        self.pose_pub = self.node.create_publisher(String, '/humanoid_pose_command', 10)
        self.action_pub = self.node.create_publisher(String, '/humanoid_action_command', 10)

    def send_pose_command(self, pose_name):
        """Send predefined pose commands to humanoid"""
        cmd_msg = String()
        cmd_msg.data = f"POSE:{pose_name}"
        self.pose_pub.publish(cmd_msg)

    def send_action_command(self, action_name):
        """Send action commands to humanoid"""
        cmd_msg = String()
        cmd_msg.data = f"ACTION:{action_name}"
        self.action_pub.publish(cmd_msg)
```

## Performance Optimization

For complex humanoid robots in RViz:

1. **Reduce update rates**: Lower the update rate of high-frequency displays
2. **Limit history**: Use appropriate history depth for topics
3. **Disable unused displays**: Turn off displays you're not actively using
4. **Optimize mesh complexity**: Use simplified meshes for visualization

## Troubleshooting Common Issues

### Robot Model Not Showing

1. Verify `robot_description` parameter is set
2. Check that joint states are being published
3. Ensure TF tree is connected
4. Validate URDF syntax

### TF Issues

1. Make sure `robot_state_publisher` is running
2. Verify joint state messages are being published
3. Check for disconnected TF frames

### Performance Problems

1. Reduce the complexity of visual meshes
2. Lower the rate of high-frequency displays
3. Use appropriate QoS settings for visualization topics

## Command Line Tools

Useful RViz-related command line tools:

```bash
# Start RViz with a specific config
ros2 run rviz2 rviz2 -d /path/to/config.rviz

# Echo TF tree
ros2 run tf2_tools view_frames

# Check TF transforms
ros2 run tf2_ros tf2_echo base_link left_foot

# Visualize a single topic
ros2 run rqt_plot rqt_plot /joint_states/position[0]
```

## Next Steps

In the final chapter of Module 1, we'll summarize what we've learned about ROS 2 and prepare for Module 2, where we'll explore simulation environments for our humanoid robot using Gazebo, Unity, and NVIDIA Isaac.