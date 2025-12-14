# Isaac ROS Architecture

In this chapter, we'll explore the NVIDIA Isaac ROS architecture, which provides optimized ROS 2 packages specifically designed for robotics applications with GPU acceleration. Isaac ROS leverages NVIDIA's hardware capabilities to deliver high-performance perception, navigation, and manipulation capabilities for humanoid robots.

## Isaac ROS Overview

### Architecture Components

Isaac ROS consists of several key components that work together to provide GPU-accelerated robotics capabilities:

- **Isaac ROS Common**: Core utilities and common functionality
- **Isaac ROS Perception**: Computer vision and sensor processing
- **Isaac ROS Navigation**: Path planning and navigation
- **Isaac ROS Manipulation**: Grasping and manipulation
- **Isaac ROS Gems**: GPU-accelerated algorithms and utilities
- **Isaac ROS TensorRT**: Deep learning inference acceleration

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Isaac ROS Applications                   │
├─────────────────────────────────────────────────────────────┤
│  Perception   │  Navigation  │  Manipulation  │  Utilities  │
├─────────────────────────────────────────────────────────────┤
│              Isaac ROS Core Packages                        │
│  Common  │  Messages  │  Interfaces  │  Utilities          │
├─────────────────────────────────────────────────────────────┤
│            GPU-Accelerated Libraries                        │
│ TensorRT │ CUDA │ OptiX │ OpenCV | Open3D | OpenGL         │
├─────────────────────────────────────────────────────────────┤
│                Hardware Layer                              │
│ GPU (RTX/V100/A100) │ CPU │ Memory │ Storage               │
└─────────────────────────────────────────────────────────────┘
```

## Isaac ROS Package Structure

### Core Package Organization

```python
# isaac_ros_packages.py
import os
from ament_index_python.packages import get_package_share_directory

class IsaacROSPackageManager:
    def __init__(self):
        self.packages = {
            'isaac_ros_common': {
                'path': self._get_package_path('isaac_ros_common'),
                'description': 'Common utilities and base functionality',
                'components': ['logging', 'utils', 'messages']
            },
            'isaac_ros_visual_slam': {
                'path': self._get_package_path('isaac_ros_visual_slam'),
                'description': 'GPU-accelerated visual SLAM',
                'components': ['tracking', 'mapping', 'optimization']
            },
            'isaac_ros_pose_estimation': {
                'path': self._get_package_path('isaac_ros_pose_estimation'),
                'description': 'Accurate pose estimation from sensors',
                'components': ['estimation', 'filtering', 'optimization']
            },
            'isaac_ros_apriltag': {
                'path': self._get_package_path('isaac_ros_apriltag'),
                'description': 'GPU-accelerated AprilTag detection',
                'components': ['detection', 'decoding', 'pose_estimation']
            },
            'isaac_ros_detectnet': {
                'path': self._get_package_path('isaac_ros_detectnet'),
                'description': 'Object detection with TensorRT',
                'components': ['inference', 'post_processing', 'tracking']
            },
            'isaac_ros_segmentation': {
                'path': self._get_package_path('isaac_ros_segmentation'),
                'description': 'Semantic segmentation',
                'components': ['inference', 'mask_processing', 'analysis']
            },
            'isaac_ros_gems': {
                'path': self._get_package_path('isaac_ros_gems'),
                'description': 'GPU-accelerated utilities',
                'components': ['image_processing', 'geometry', 'optimization']
            },
            'isaac_ros_tensor_rt': {
                'path': self._get_package_path('isaac_ros_tensor_rt'),
                'description': 'TensorRT integration',
                'components': ['engine_management', 'inference', 'optimization']
            }
        }

    def _get_package_path(self, package_name):
        """Get the share directory path for an Isaac ROS package"""
        try:
            return get_package_share_directory(package_name)
        except:
            return f"/opt/ros/humble/share/{package_name}"

    def get_package_info(self, package_name):
        """Get detailed information about an Isaac ROS package"""
        if package_name in self.packages:
            return self.packages[package_name]
        else:
            raise ValueError(f"Package {package_name} not found in Isaac ROS")

    def list_all_packages(self):
        """List all available Isaac ROS packages"""
        return list(self.packages.keys())

    def get_perception_packages(self):
        """Get all perception-related packages"""
        perception_packages = [
            'isaac_ros_visual_slam',
            'isaac_ros_apriltag',
            'isaac_ros_detectnet',
            'isaac_ros_segmentation'
        ]
        return {pkg: self.packages[pkg] for pkg in perception_packages if pkg in self.packages}

    def get_navigation_packages(self):
        """Get all navigation-related packages"""
        navigation_packages = [
            'isaac_ros_visual_slam',
            'isaac_ros_pose_estimation'
        ]
        return {pkg: self.packages[pkg] for pkg in navigation_packages if pkg in self.packages}

# Example usage
def explore_isaac_ros_packages():
    """Explore available Isaac ROS packages"""
    manager = IsaacROSPackageManager()

    print("Isaac ROS Package Overview:")
    print("=" * 50)

    for pkg_name, pkg_info in manager.packages.items():
        print(f"\nPackage: {pkg_name}")
        print(f"  Description: {pkg_info['description']}")
        print(f"  Components: {', '.join(pkg_info['components'])}")
        print(f"  Path: {pkg_info['path']}")

if __name__ == "__main__":
    explore_isaac_ros_packages()
```

## Isaac ROS Perception Stack

### GPU-Accelerated Computer Vision

```python
# perception_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import numpy as np

class IsaacPerceptionPipeline(Node):
    def __init__(self):
        super().__init__('isaac_perception_pipeline')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Subscribe to camera data
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Publishers for different perception outputs
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/isaac_ros/detections',
            10
        )

        self.segmentation_pub = self.create_publisher(
            Image,
            '/isaac_ros/segmentation',
            10
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/isaac_ros/depth',
            10
        )

        # Isaac ROS perception components
        self.camera_info = None
        self.gpu_initialized = False

        # Initialize Isaac ROS perception components
        self._initialize_perception_components()

    def _initialize_perception_components(self):
        """Initialize Isaac ROS perception components"""
        try:
            # Initialize GPU-accelerated components
            # This would typically involve loading TensorRT engines
            # or initializing CUDA-based algorithms
            self.get_logger().info('Isaac ROS perception components initialized')
            self.gpu_initialized = True
        except Exception as e:
            self.get_logger().error(f'Failed to initialize perception components: {e}')
            self.gpu_initialized = False

    def image_callback(self, msg):
        """Process incoming camera images using Isaac ROS perception"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Perform GPU-accelerated perception
            if self.gpu_initialized:
                results = self._perform_gpu_perception(cv_image)
                self._publish_perception_results(results, msg.header)
            else:
                # Fallback to CPU processing
                results = self._perform_cpu_perception(cv_image)
                self._publish_perception_results(results, msg.header)

        except Exception as e:
            self.get_logger().error(f'Error in perception pipeline: {e}')

    def camera_info_callback(self, msg):
        """Update camera calibration information"""
        self.camera_info = msg

    def _perform_gpu_perception(self, image):
        """Perform perception using GPU acceleration"""
        results = {
            'detections': [],
            'segmentation': None,
            'depth': None,
            'features': []
        }

        # This would use Isaac ROS GPU-accelerated algorithms
        # For example:
        # - Isaac ROS DetectNet for object detection
        # - Isaac ROS Segmentation for semantic segmentation
        # - Isaac ROS AprilTag for fiducial detection

        # Simulate GPU processing results
        if np.random.random() > 0.7:  # Random detection for simulation
            detection = {
                'bbox': [50, 50, 200, 150],  # x, y, width, height
                'class': 'human',
                'confidence': 0.95
            }
            results['detections'].append(detection)

        return results

    def _perform_cpu_perception(self, image):
        """Fallback CPU-based perception"""
        # Implement basic CPU-based perception as fallback
        results = {
            'detections': [],
            'segmentation': None,
            'depth': None,
            'features': []
        }

        # Basic processing for fallback
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        # ... basic processing

        return results

    def _publish_perception_results(self, results, header):
        """Publish perception results to ROS topics"""
        # Publish detections
        if results['detections']:
            detection_msg = Detection2DArray()
            detection_msg.header = header

            for detection in results['detections']:
                vision_detection = Detection2D()
                vision_detection.bbox.center.x = detection['bbox'][0] + detection['bbox'][2] / 2
                vision_detection.bbox.center.y = detection['bbox'][1] + detection['bbox'][3] / 2
                vision_detection.bbox.size_x = detection['bbox'][2]
                vision_detection.bbox.size_y = detection['bbox'][3]

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = detection['class']
                hypothesis.hypothesis.score = detection['confidence']

                vision_detection.results.append(hypothesis)
                detection_msg.detections.append(vision_detection)

            self.detection_pub.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacPerceptionPipeline()

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

## Isaac ROS Navigation Stack

### GPU-Accelerated Path Planning

```python
# navigation_stack.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import MarkerArray
import numpy as np

class IsaacNavigationStack(Node):
    def __init__(self):
        super().__init__('isaac_navigation_stack')

        # Subscribers
        self.odom_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )

        # Publishers
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # Isaac ROS navigation components
        self.current_pose = None
        self.goal_pose = None
        self.map_data = None
        self.path = []

        # GPU-accelerated navigation components
        self._initialize_navigation_components()

    def _initialize_navigation_components(self):
        """Initialize Isaac ROS navigation components"""
        try:
            # Initialize GPU-accelerated path planning
            # This would typically involve:
            # - GPU-based costmap generation
            # - Accelerated A* or Dijkstra planning
            # - CUDA-based trajectory optimization
            self.get_logger().info('Isaac ROS navigation components initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize navigation: {e}')

    def odom_callback(self, msg):
        """Update robot's current pose"""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Process laser scan data for mapping and obstacle detection"""
        # Process scan data using GPU acceleration
        self._process_scan_gpu(msg)

    def goal_callback(self, msg):
        """Process navigation goal and plan path"""
        self.goal_pose = msg.pose
        if self.current_pose:
            self._plan_path_gpu()

    def _process_scan_gpu(self, scan_msg):
        """Process laser scan using GPU acceleration"""
        # Convert scan to point cloud
        ranges = np.array(scan_msg.ranges)
        angles = np.array([scan_msg.angle_min + i * scan_msg.angle_increment
                          for i in range(len(ranges))])

        # Filter valid ranges
        valid_mask = np.isfinite(ranges) & (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        # Convert to Cartesian coordinates
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)

        # GPU-accelerated obstacle detection and mapping would go here
        # This is where Isaac ROS would use CUDA kernels for fast processing

    def _plan_path_gpu(self):
        """Plan path using GPU acceleration"""
        if not self.current_pose or not self.goal_pose:
            return

        # GPU-accelerated path planning
        # This would use Isaac ROS GPU-based planners
        start = (self.current_pose.position.x, self.current_pose.position.y)
        goal = (self.goal_pose.position.x, self.goal_pose.position.y)

        # Simulate GPU path planning
        path_points = self._simulate_gpu_path_planning(start, goal)

        # Publish the path
        self._publish_path(path_points)

    def _simulate_gpu_path_planning(self, start, goal):
        """Simulate GPU-accelerated path planning"""
        # In a real implementation, this would use Isaac ROS GPU planners
        # For simulation, create a simple path
        path = []
        steps = 20
        for i in range(steps + 1):
            t = i / steps
            x = start[0] + t * (goal[0] - start[0])
            y = start[1] + t * (goal[1] - start[1])
            path.append((x, y))

        return path

    def _publish_path(self, path_points):
        """Publish planned path"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0

            path_msg.poses.append(pose_stamped)

        self.path_pub.publish(path_msg)
        self.path = path_points

def main(args=None):
    rclpy.init(args=args)
    node = IsaacNavigationStack()

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

## Isaac ROS Manipulation Stack

### GPU-Accelerated Manipulation

```python
# manipulation_stack.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Float64
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class IsaacManipulationStack(Node):
    def __init__(self):
        super().__init__('isaac_manipulation_stack')

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.target_sub = self.create_subscription(
            PoseStamped,
            '/manipulation_target',
            self.target_callback,
            10
        )

        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.marker_pub = self.create_publisher(MarkerArray, '/manipulation_markers', 10)

        # Isaac ROS manipulation components
        self.current_joint_states = None
        self.target_pose = None
        self.robot_model = None

        # Initialize manipulation components
        self._initialize_manipulation_components()

    def _initialize_manipulation_components(self):
        """Initialize Isaac ROS manipulation components"""
        try:
            # Initialize GPU-accelerated inverse kinematics
            # Initialize grasp planning components
            # Load robot kinematic model
            self.get_logger().info('Isaac ROS manipulation components initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize manipulation: {e}')

    def joint_state_callback(self, msg):
        """Update current joint states"""
        self.current_joint_states = msg

    def target_callback(self, msg):
        """Process manipulation target and plan motion"""
        self.target_pose = msg.pose
        self._plan_manipulation_motion()

    def _plan_manipulation_motion(self):
        """Plan manipulation motion using Isaac ROS components"""
        if not self.target_pose:
            return

        # GPU-accelerated motion planning
        trajectory = self._compute_gpu_motion_trajectory()

        if trajectory:
            self.trajectory_pub.publish(trajectory)

    def _compute_gpu_motion_trajectory(self):
        """Compute motion trajectory using GPU acceleration"""
        # This would use Isaac ROS GPU-accelerated IK and motion planning
        # For simulation, create a simple trajectory

        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # Example joint names

        # Create trajectory points
        for i in range(10):  # 10 points in trajectory
            point = JointTrajectoryPoint()
            point.positions = [0.0 + i * 0.1] * 6  # Example positions
            point.velocities = [0.1] * 6  # Example velocities
            point.time_from_start.sec = i
            point.time_from_start.nanosec = 0

            trajectory.points.append(point)

        return trajectory

    def _compute_gpu_inverse_kinematics(self, target_pose):
        """Compute inverse kinematics using GPU acceleration"""
        # This would use Isaac ROS GPU-accelerated IK solvers
        # For humanoid robots, this might involve whole-body IK
        pass

    def _plan_grasp_motion(self, object_pose):
        """Plan grasp motion for manipulation"""
        # GPU-accelerated grasp planning
        # This would use Isaac ROS grasp planning components
        pass

def main(args=None):
    rclpy.init(args=args)
    node = IsaacManipulationStack()

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

## Isaac ROS TensorRT Integration

### Deep Learning Acceleration

```python
# tensorrt_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

class IsaacTensorRTNode(Node):
    def __init__(self):
        super().__init__('isaac_tensorrt_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Subscribe to camera data
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.fps_pub = self.create_publisher(Float32, '/tensorrt_fps', 10)

        # TensorRT components
        self.engine = None
        self.context = None
        self.input_binding = None
        self.output_binding = None
        self.stream = None

        # Initialize TensorRT
        self._initialize_tensorrt()

        # Performance monitoring
        self.frame_count = 0
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def _initialize_tensorrt(self):
        """Initialize TensorRT engine for GPU inference"""
        try:
            # This would typically load a pre-built TensorRT engine
            # For demonstration, we'll create a placeholder
            self.get_logger().info('TensorRT engine initialized')

            # Create CUDA stream
            self.stream = cuda.Stream()

        except Exception as e:
            self.get_logger().error(f'Failed to initialize TensorRT: {e}')

    def image_callback(self, msg):
        """Process image using TensorRT-accelerated inference"""
        try:
            # Convert ROS Image to numpy array
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Perform TensorRT inference
            if self.engine:
                results = self._perform_tensorrt_inference(cv_image)
                self._publish_results(results, msg.header)
            else:
                # Fallback processing
                results = self._fallback_inference(cv_image)
                self._publish_results(results, msg.header)

            # Update performance metrics
            self.frame_count += 1
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self.start_time >= 1.0:  # Every second
                fps = self.frame_count / (current_time - self.start_time)
                fps_msg = Float32()
                fps_msg.data = fps
                self.fps_pub.publish(fps_msg)

                self.frame_count = 0
                self.start_time = current_time

        except Exception as e:
            self.get_logger().error(f'Error in TensorRT processing: {e}')

    def _perform_tensorrt_inference(self, image):
        """Perform inference using TensorRT"""
        # Preprocess image for TensorRT
        input_tensor = self._preprocess_image(image)

        # Run inference
        # This would involve:
        # - Copying data to GPU
        # - Executing TensorRT engine
        # - Copying results back to CPU

        # For demonstration, return dummy results
        results = {
            'detections': [
                {'bbox': [100, 100, 200, 150], 'class': 'person', 'confidence': 0.92},
                {'bbox': [300, 200, 150, 120], 'class': 'object', 'confidence': 0.87}
            ]
        }

        return results

    def _preprocess_image(self, image):
        """Preprocess image for TensorRT inference"""
        # Resize and normalize image
        # Convert BGR to RGB if needed
        # Normalize pixel values
        # Add batch dimension
        pass

    def _fallback_inference(self, image):
        """Fallback CPU-based inference"""
        # Implement basic inference as fallback
        return {'detections': []}

    def _publish_results(self, results, header):
        """Publish inference results"""
        detection_msg = Detection2DArray()
        detection_msg.header = header

        for detection in results['detections']:
            vision_detection = Detection2D()
            vision_detection.bbox.center.x = detection['bbox'][0] + detection['bbox'][2] / 2
            vision_detection.bbox.center.y = detection['bbox'][1] + detection['bbox'][3] / 2
            vision_detection.bbox.size_x = detection['bbox'][2]
            vision_detection.bbox.size_y = detection['bbox'][3]

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = detection['class']
            hypothesis.hypothesis.score = detection['confidence']

            vision_detection.results.append(hypothesis)
            detection_msg.detections.append(vision_detection)

        self.detection_pub.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacTensorRTNode()

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

## Isaac ROS Launch Files

### Comprehensive Launch Configuration

```python
# isaac_ros_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_perception = LaunchConfiguration('enable_perception', default='true')
    enable_navigation = LaunchConfiguration('enable_navigation', default='true')
    enable_manipulation = LaunchConfiguration('enable_manipulation', default='true')

    # Isaac ROS Perception Container
    perception_container = ComposableNodeContainer(
        name='isaac_perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_perception),
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag',
                parameters=[{
                    'family': 'tag36h11',
                    'max_tags': 128,
                    'tile_size': 2.0,
                    'black_border': 1,
                    'quad_decimate': 2.0,
                    'quad_sigma': 0.0,
                    'refine_edges': True,
                    'decode_sharpening': 0.25,
                    'min_tag_width': 0.02,
                    'max_hamming': 1,
                    'debug': False,
                    'default_bounds': [0, 1000000],
                    'is_big_endian': False,
                    'input_image_width': 640,
                    'input_image_height': 480,
                    'num_cameras': 1,
                    'output_frame': 'camera_link'
                }],
                remappings=[
                    ('image', '/camera/image_rect_color'),
                    ('camera_info', '/camera/camera_info'),
                    ('detections', '/apriltag_detections')
                ]
            ),
            ComposableNode(
                package='isaac_ros_detectnet',
                plugin='nvidia::isaac_ros::detectnet::DetectNetNode',
                name='detectnet',
                parameters=[{
                    'model_name': 'ssd_mobilenet_v2_coco',
                    'input_topic': '/camera/image_rect_color',
                    'max_batch_size': 1,
                    'num_channels': 3,
                    'input_tensor_dimensions': '3,544,960',
                    'mean_pixel_value': [0.0, 0.0, 0.0],
                    'std_pixel_value': [1.0, 1.0, 1.0],
                    'threshold': 0.7,
                    'enable_padding': False,
                    'use_trt_fp16_mode': True,
                    'use_trt_int8_mode': False,
                    'trt_cache_directory': '/tmp/detectnet',
                    'trt_engine_cache_directory': '/tmp/detectnet'
                }],
                remappings=[
                    ('image', '/camera/image_rect_color'),
                    ('detections', '/detectnet_detections')
                ]
            )
        ]
    )

    # Isaac ROS Navigation Container
    navigation_container = ComposableNodeContainer(
        name='isaac_navigation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_navigation),
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'enable_observations_view': True,
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'min_num_landmarks': 100,
                    'max_num_landmarks': 1000,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'init_frame': 'base_link',
                    'publish_tracked_pose': True,
                    'tracked_pose_frame': 'tracked_pose',
                    'rectified_images': True,
                    'force_cpu_based_relocalization': False
                }],
                remappings=[
                    ('/visual_slam/imu', '/imu/data'),
                    ('/visual_slam/camera/left/image_rect', '/camera/image_rect_left'),
                    ('/visual_slam/camera/right/image_rect', '/camera/image_rect_right'),
                    ('/visual_slam/camera/left/camera_info', '/camera/camera_info_left'),
                    ('/visual_slam/camera/right/camera_info', '/camera/camera_info_right'),
                    ('/visual_slam/tracked_pose', '/visual_slam/pose'),
                    ('/visual_slam/visual_odometry', '/visual_slam/odometry'),
                    ('/visual_slam/path', '/visual_slam/path')
                ]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'enable_perception',
            default_value='true',
            description='Enable Isaac ROS perception components'
        ),
        DeclareLaunchArgument(
            'enable_navigation',
            default_value='true',
            description='Enable Isaac ROS navigation components'
        ),
        DeclareLaunchArgument(
            'enable_manipulation',
            default_value='true',
            description='Enable Isaac ROS manipulation components'
        ),
        perception_container,
        navigation_container
    ])
```

## Isaac ROS Best Practices

### Performance Optimization

```python
# optimization_guide.py
class IsaacROSOptimizationGuide:
    def __init__(self):
        self.optimization_tips = {
            'memory_management': [
                "Use CUDA unified memory for efficient GPU-CPU transfers",
                "Pre-allocate tensors to avoid runtime allocation overhead",
                "Use memory pools for frequently allocated/deallocated memory"
            ],
            'gpu_utilization': [
                "Batch inference operations for better GPU utilization",
                "Use TensorRT for optimized deep learning inference",
                "Leverage CUDA streams for overlapping computation and memory transfers"
            ],
            'pipeline_optimization': [
                "Use composable nodes to reduce message passing overhead",
                "Implement multi-threaded containers for parallel processing",
                "Use QoS settings appropriate for real-time requirements"
            ],
            'tensorrt_optimization': [
                "Use FP16 mode when precision allows",
                "Optimize batch sizes for your specific GPU",
                "Use TensorRT optimization tools for model optimization"
            ]
        }

    def get_optimization_recommendations(self, component_type):
        """Get optimization recommendations for specific component type"""
        recommendations = []

        if component_type == 'perception':
            recommendations.extend([
                "Use Isaac ROS DetectNet for object detection",
                "Implement multi-camera processing with CUDA",
                "Use hardware-accelerated image preprocessing"
            ])
        elif component_type == 'navigation':
            recommendations.extend([
                "Use GPU-accelerated costmap generation",
                "Implement CUDA-based path planning algorithms",
                "Use hardware-accelerated point cloud processing"
            ])
        elif component_type == 'manipulation':
            recommendations.extend([
                "Use GPU-accelerated inverse kinematics",
                "Implement CUDA-based trajectory optimization",
                "Use hardware-accelerated grasp planning"
            ])

        return recommendations

def print_optimization_guide():
    """Print Isaac ROS optimization guide"""
    guide = IsaacROSOptimizationGuide()

    print("Isaac ROS Optimization Guide")
    print("=" * 50)

    for category, tips in guide.optimization_tips.items():
        print(f"\n{category.replace('_', ' ').title()}:")
        for tip in tips:
            print(f"  • {tip}")

    # Print component-specific recommendations
    for component in ['perception', 'navigation', 'manipulation']:
        recommendations = guide.get_optimization_recommendations(component)
        print(f"\n{component.title()} Optimization:")
        for rec in recommendations:
            print(f"  • {rec}")

if __name__ == "__main__":
    print_optimization_guide()
```

## Integration with Isaac Sim

### Isaac ROS-Sim Integration

```python
# isaac_ros_sim_integration.py
import omni
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera, Lidar
import carb

class IsaacROSSimIntegration:
    def __init__(self, robot_prim_path):
        self.robot_prim_path = robot_prim_path
        self.stage = omni.usd.get_context().get_stage()

    def setup_ros_sensors(self):
        """Set up ROS-enabled sensors in Isaac Sim"""
        # Add ROS-enabled camera
        self._setup_ros_camera()

        # Add ROS-enabled LiDAR
        self._setup_ros_lidar()

        # Add ROS-enabled IMU
        self._setup_ros_imu()

    def _setup_ros_camera(self):
        """Set up ROS camera with Isaac ROS integration"""
        camera_path = f"{self.robot_prim_path}/head/head_camera"

        # Create camera using Isaac Sim API
        camera = Camera(
            prim_path=camera_path,
            frequency=30,
            resolution=(640, 480)
        )

        # Configure for ROS bridge
        # This would set up the appropriate ROS topic names and formats
        camera.config_camera_for_ros_bridge(
            rgb_topic="/camera/image_raw",
            camera_info_topic="/camera/camera_info"
        )

    def _setup_ros_lidar(self):
        """Set up ROS LiDAR with Isaac ROS integration"""
        lidar_path = f"{self.robot_prim_path}/base/lidar"

        # Create LiDAR sensor
        lidar = Lidar(
            prim_path=lidar_path,
            translation=(0.2, 0.0, 0.1),
            orientation=(0, 0, 0, 1),
            config="Example_Rotary"
        )

        # Configure for ROS
        lidar.config_lidar_for_ros_bridge(
            scan_topic="/scan",
            point_cloud_topic="/point_cloud"
        )

    def _setup_ros_imu(self):
        """Set up ROS IMU with Isaac ROS integration"""
        # IMU setup would go here
        pass

def integrate_isaac_ros_with_sim(robot_prim_path):
    """Integrate Isaac ROS with Isaac Sim"""
    integration = IsaacROSSimIntegration(robot_prim_path)
    integration.setup_ros_sensors()

    carb.log_info("Isaac ROS-Sim integration completed")
```

## Best Practices and Guidelines

1. **Use Composable Nodes**: Leverage component containers for better performance
2. **Optimize GPU Memory**: Pre-allocate and reuse GPU memory buffers
3. **Batch Processing**: Process multiple inputs together for better GPU utilization
4. **TensorRT Optimization**: Use TensorRT for deep learning inference acceleration
5. **QoS Configuration**: Set appropriate QoS for real-time requirements
6. **Error Handling**: Implement robust error handling for GPU operations
7. **Performance Monitoring**: Monitor GPU utilization and memory usage

## Next Steps

In the next chapter, we'll explore the Visual SLAM pipeline in detail, learning how Isaac ROS provides GPU-accelerated simultaneous localization and mapping capabilities specifically optimized for humanoid robot navigation and mapping tasks.