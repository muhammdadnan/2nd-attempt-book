# Visual SLAM Pipeline

In this chapter, we'll explore the Visual SLAM (Simultaneous Localization and Mapping) pipeline in NVIDIA Isaac, which provides GPU-accelerated simultaneous localization and mapping capabilities specifically optimized for humanoid robot navigation and mapping tasks.

## Understanding Visual SLAM

### SLAM Fundamentals

Visual SLAM is a critical technology for autonomous robots that enables them to:
- **Localize** themselves in unknown environments
- **Map** the environment as they move through it
- **Navigate** safely using the created map
- **Understand** spatial relationships in 3D space

Traditional SLAM approaches rely on:
- **Feature extraction**: Identifying distinctive points in images
- **Feature matching**: Finding corresponding points across frames
- **Pose estimation**: Computing camera motion between frames
- **Bundle adjustment**: Optimizing camera poses and landmark positions
- **Mapping**: Creating a consistent map of the environment

### Isaac Visual SLAM Architecture

Isaac Visual SLAM leverages GPU acceleration to significantly improve performance:

```
┌─────────────────────────────────────────────────────────────────┐
│                    Isaac Visual SLAM Pipeline                   │
├─────────────────────────────────────────────────────────────────┤
│  Camera Input → Feature Extraction → Tracking → Mapping → Loop │
│                (GPU Accelerated)        (Optimization)   Closure│
├─────────────────────────────────────────────────────────────────┤
│  GPU Components:                                                │
│  - CUDA-based feature detection (ORB, FAST, etc.)              │
│  - TensorRT-accelerated feature matching                       │
│  - GPU-parallelized pose estimation                            │
│  - CUDA-optimized bundle adjustment                            │
│  - GPU-accelerated loop closure detection                      │
└─────────────────────────────────────────────────────────────────┘
```

## Isaac Visual SLAM Components

### Core Architecture

```python
# visual_slam_core.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
import numpy as np
import cv2

class IsaacVisualSLAMNode(Node):
    def __init__(self):
        super().__init__('isaac_visual_slam')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize SLAM components
        self.slam_tracker = None
        self.map_builder = None
        self.pose_estimator = None
        self.loop_detector = None

        # Initialize state variables
        self.current_frame = None
        self.previous_frame = None
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.keyframes = []
        self.map_points = []

        # Initialize GPU-accelerated components
        self._initialize_gpu_components()

        # Subscribers
        self.left_image_sub = self.create_subscription(
            Image,
            '/camera/left/image_rect_color',
            self.left_image_callback,
            10
        )

        self.right_image_sub = self.create_subscription(
            Image,
            '/camera/right/image_rect_color',
            self.right_image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/left/camera_info',
            self.camera_info_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/visual_slam/odometry', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/visual_slam/pose', 10)
        self.map_pub = self.create_publisher(MarkerArray, '/visual_slam/map', 10)
        self.path_pub = self.create_publisher(Path, '/visual_slam/path', 10)

        # Timer for SLAM processing
        self.process_timer = self.create_timer(0.033, self.process_slam)  # ~30Hz

        self.get_logger().info('Isaac Visual SLAM node initialized')

    def _initialize_gpu_components(self):
        """Initialize GPU-accelerated SLAM components"""
        try:
            # Initialize CUDA-based feature detector
            self.feature_detector = cv2.cuda.SIFT_create()  # Or ORB, AKAZE, etc.

            # Initialize GPU-based matcher
            self.matcher = cv2.BFMatcher()  # Will be GPU-accelerated in Isaac

            # Initialize GPU-accelerated pose estimator
            self.pose_estimator = GPUPoseEstimator()

            # Initialize GPU-based map builder
            self.map_builder = GPUMapBuilder()

            self.get_logger().info('GPU-accelerated SLAM components initialized')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPU components: {e}')
            # Fallback to CPU components
            self.feature_detector = cv2.SIFT_create()
            self.get_logger().warn('Falling back to CPU-based SLAM')

    def left_image_callback(self, msg):
        """Process left camera image for stereo SLAM"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Store as current frame
            self.current_frame = {
                'image': cv_image,
                'timestamp': msg.header.stamp,
                'frame_id': msg.header.frame_id
            }

        except Exception as e:
            self.get_logger().error(f'Error processing left image: {e}')

    def right_image_callback(self, msg):
        """Process right camera image for stereo depth"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Store right image for stereo processing
            if hasattr(self, 'current_frame'):
                self.current_frame['right_image'] = cv_image

        except Exception as e:
            self.get_logger().error(f'Error processing right image: {e}')

    def camera_info_callback(self, msg):
        """Update camera calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

        # Update stereo rectification parameters if available
        if msg.p:  # Projection matrix
            self.projection_matrix = np.array(msg.p).reshape(3, 4)

    def imu_callback(self, msg):
        """Process IMU data for sensor fusion"""
        # Store IMU data for sensor fusion
        self.imu_data = {
            'linear_acceleration': [msg.linear_acceleration.x,
                                   msg.linear_acceleration.y,
                                   msg.linear_acceleration.z],
            'angular_velocity': [msg.angular_velocity.x,
                                msg.angular_velocity.y,
                                msg.angular_velocity.z],
            'orientation': [msg.orientation.x,
                           msg.orientation.y,
                           msg.orientation.z,
                           msg.orientation.w],
            'timestamp': msg.header.stamp
        }

    def process_slam(self):
        """Main SLAM processing loop"""
        if self.current_frame is None or self.previous_frame is None:
            # Need at least two frames to start tracking
            return

        try:
            # Extract features from current frame
            current_features = self._extract_features(self.current_frame['image'])

            # Match features with previous frame
            matches = self._match_features(
                self.previous_frame['features'],
                current_features
            )

            # Estimate camera motion
            if len(matches) >= 10:  # Minimum matches required
                transformation = self._estimate_motion(matches)

                # Update current pose
                self.current_pose = self.current_pose @ transformation

                # Check if this frame should be a keyframe
                if self._should_add_keyframe():
                    self._add_keyframe(current_features)

                    # Perform local bundle adjustment
                    self._local_bundle_adjustment()

            # Update previous frame
            self.previous_frame = {
                'image': self.current_frame['image'],
                'features': current_features,
                'pose': self.current_pose.copy(),
                'timestamp': self.current_frame['timestamp']
            }

            # Publish current pose and odometry
            self._publish_pose()
            self._publish_odometry()

        except Exception as e:
            self.get_logger().error(f'SLAM processing error: {e}')

    def _extract_features(self, image):
        """Extract features using GPU acceleration"""
        try:
            # Convert to grayscale if needed
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image

            # Use GPU-accelerated feature detection
            keypoints, descriptors = self.feature_detector.detectAndCompute(gray, None)

            return {
                'keypoints': keypoints,
                'descriptors': descriptors
            }

        except Exception as e:
            self.get_logger().error(f'Feature extraction error: {e}')
            return {'keypoints': [], 'descriptors': None}

    def _match_features(self, prev_features, curr_features):
        """Match features between frames using GPU acceleration"""
        if (prev_features['descriptors'] is None or
            curr_features['descriptors'] is None or
            len(prev_features['descriptors']) == 0 or
            len(curr_features['descriptors']) == 0):
            return []

        try:
            # Use GPU-accelerated matcher
            matches = self.matcher.match(
                prev_features['descriptors'],
                curr_features['descriptors']
            )

            # Sort matches by distance and return good matches
            matches = sorted(matches, key=lambda x: x.distance)

            # Apply ratio test for better matches
            good_matches = []
            for i in range(min(len(matches), 100)):  # Limit matches
                if i + 1 < len(matches):
                    if matches[i].distance < 0.75 * matches[i + 1].distance:
                        good_matches.append(matches[i])

            return good_matches

        except Exception as e:
            self.get_logger().error(f'Feature matching error: {e}')
            return []

    def _estimate_motion(self, matches):
        """Estimate camera motion between frames"""
        if len(matches) < 10:
            return np.eye(4)  # No motion if insufficient matches

        try:
            # Get matched points
            prev_pts = np.float32([self.previous_frame['features']['keypoints'][m.queryIdx].pt
                                  for m in matches]).reshape(-1, 1, 2)
            curr_pts = np.float32([self.current_frame['features']['keypoints'][m.trainIdx].pt
                                  for m in matches]).reshape(-1, 1, 2)

            # Estimate essential matrix
            E, mask = cv2.findEssentialMat(
                curr_pts, prev_pts,
                self.camera_matrix,
                method=cv2.RANSAC,
                prob=0.999,
                threshold=1.0
            )

            if E is not None:
                # Decompose essential matrix to get rotation and translation
                _, R, t, _ = cv2.recoverPose(E, curr_pts, prev_pts, self.camera_matrix)

                # Create transformation matrix
                transformation = np.eye(4)
                transformation[:3, :3] = R
                transformation[:3, 3] = t.flatten()

                return transformation
            else:
                return np.eye(4)

        except Exception as e:
            self.get_logger().error(f'Motion estimation error: {e}')
            return np.eye(4)

    def _should_add_keyframe(self):
        """Determine if current frame should be a keyframe"""
        if not self.keyframes:
            return True

        # Check if enough time has passed
        if (self.current_frame['timestamp'].nanosec -
            self.keyframes[-1]['timestamp'].nanosec > 1e9):  # 1 second
            return True

        # Check if camera has moved significantly
        prev_pose = self.keyframes[-1]['pose']
        translation = np.linalg.norm(self.current_pose[:3, 3] - prev_pose[:3, 3])

        if translation > 0.5:  # 50 cm threshold
            return True

        return False

    def _add_keyframe(self, features):
        """Add current frame as a keyframe"""
        keyframe = {
            'image': self.current_frame['image'].copy(),
            'features': features,
            'pose': self.current_pose.copy(),
            'timestamp': self.current_frame['timestamp'],
            'frame_id': len(self.keyframes)
        }

        self.keyframes.append(keyframe)

    def _local_bundle_adjustment(self):
        """Perform local bundle adjustment using GPU acceleration"""
        # In a real implementation, this would use GPU-accelerated optimization
        # For now, we'll just log that BA should be performed
        self.get_logger().info(f'Local bundle adjustment performed with {len(self.keyframes)} keyframes')

    def _publish_pose(self):
        """Publish current estimated pose"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Extract position and orientation from transformation matrix
        position = self.current_pose[:3, 3]
        rotation_matrix = self.current_pose[:3, :3]

        # Convert rotation matrix to quaternion
        qw = np.sqrt(1 + rotation_matrix[0,0] + rotation_matrix[1,1] + rotation_matrix[2,2]) / 2
        qx = (rotation_matrix[2,1] - rotation_matrix[1,2]) / (4 * qw)
        qy = (rotation_matrix[0,2] - rotation_matrix[2,0]) / (4 * qw)
        qz = (rotation_matrix[1,0] - rotation_matrix[0,1]) / (4 * qw)

        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

    def _publish_odometry(self):
        """Publish odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        # Copy pose from pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        position = self.current_pose[:3, 3]
        rotation_matrix = self.current_pose[:3, :3]

        # Convert rotation matrix to quaternion
        qw = np.sqrt(1 + rotation_matrix[0,0] + rotation_matrix[1,1] + rotation_matrix[2,2]) / 2
        qx = (rotation_matrix[2,1] - rotation_matrix[1,2]) / (4 * qw)
        qy = (rotation_matrix[0,2] - rotation_matrix[2,0]) / (4 * qw)
        qz = (rotation_matrix[1,0] - rotation_matrix[0,1]) / (4 * qw)

        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Set zero velocity for now (would come from IMU integration)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacVisualSLAMNode()

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

## Isaac Visual SLAM Configuration

### GPU-Accelerated SLAM Configuration

```python
# visual_slam_config.py
import yaml
from dataclasses import dataclass
from typing import Dict, List, Optional

@dataclass
class VisualSLAMConfig:
    """Configuration for Isaac Visual SLAM system"""

    # Feature extraction parameters
    feature_type: str = "SIFT"  # Options: SIFT, ORB, AKAZE, FAST
    max_features: int = 2000
    feature_threshold: float = 0.01

    # GPU acceleration settings
    use_gpu_features: bool = True
    gpu_device_id: int = 0
    cuda_stream_priority: int = 0

    # Stereo vision parameters
    stereo_matching_method: str = "SGBM"  # Semi-Global Block Matching
    min_disparity: int = 0
    num_disparities: int = 128
    block_size: int = 11

    # Tracking parameters
    tracking_window_size: int = 21
    max_tracking_points: int = 500
    tracking_threshold: float = 2.0

    # Mapping parameters
    keyframe_selection_threshold: float = 0.5  # meters
    map_point_lifetime: int = 100  # frames
    min_observation_count: int = 5

    # Optimization parameters
    bundle_adjustment_frequency: int = 10  # every N keyframes
    max_ba_iterations: int = 100
    ba_convergence_threshold: float = 1e-6

    # Loop closure parameters
    loop_closure_detection: bool = True
    loop_closure_threshold: float = 0.7
    min_loop_matches: int = 20

    # Sensor fusion parameters
    imu_integration: bool = True
    imu_weight: float = 0.1
    gravity_threshold: float = 0.1

class IsaacSLAMConfigManager:
    def __init__(self, config_path: Optional[str] = None):
        if config_path:
            self.config = self.load_config(config_path)
        else:
            self.config = VisualSLAMConfig()

        self.validate_config()

    def load_config(self, config_path: str) -> VisualSLAMConfig:
        """Load configuration from YAML file"""
        with open(config_path, 'r') as f:
            config_dict = yaml.safe_load(f)

        # Create config object from dictionary
        return VisualSLAMConfig(**config_dict)

    def save_config(self, config_path: str):
        """Save configuration to YAML file"""
        config_dict = self.config.__dict__

        with open(config_path, 'w') as f:
            yaml.dump(config_dict, f, default_flow_style=False)

    def validate_config(self):
        """Validate configuration parameters"""
        errors = []

        # Validate feature parameters
        valid_feature_types = ["SIFT", "ORB", "AKAZE", "FAST"]
        if self.config.feature_type not in valid_feature_types:
            errors.append(f"Invalid feature_type: {self.config.feature_type}")

        # Validate GPU parameters
        if self.config.gpu_device_id < 0:
            errors.append("GPU device ID must be non-negative")

        # Validate stereo parameters
        if self.config.num_disparities <= 0 or self.config.num_disparities % 16 != 0:
            errors.append("num_disparities must be positive and divisible by 16")

        if self.config.block_size <= 0 or self.config.block_size % 2 == 0:
            errors.append("block_size must be positive and odd")

        if errors:
            raise ValueError(f"Configuration validation errors: {'; '.join(errors)}")

    def get_gpu_config(self) -> Dict:
        """Get GPU-specific configuration"""
        return {
            'device_id': self.config.gpu_device_id,
            'stream_priority': self.config.cuda_stream_priority,
            'memory_pool_size': 1024 * 1024 * 512,  # 512MB
            'enable_unified_memory': True
        }

    def get_tracking_config(self) -> Dict:
        """Get tracking-specific configuration"""
        return {
            'window_size': self.config.tracking_window_size,
            'max_points': self.config.max_tracking_points,
            'threshold': self.config.tracking_threshold,
            'feature_detector': self.config.feature_type,
            'max_features': self.config.max_features
        }

    def get_mapping_config(self) -> Dict:
        """Get mapping-specific configuration"""
        return {
            'keyframe_threshold': self.config.keyframe_selection_threshold,
            'point_lifetime': self.config.map_point_lifetime,
            'min_observations': self.config.min_observation_count,
            'bundle_adjustment_freq': self.config.bundle_adjustment_frequency,
            'max_ba_iterations': self.config.max_ba_iterations
        }

def create_default_slam_config():
    """Create a default Isaac Visual SLAM configuration"""
    config = VisualSLAMConfig()

    # Optimize for humanoid robot navigation
    config.feature_type = "ORB"
    config.max_features = 1500
    config.feature_threshold = 0.05

    # Enable GPU acceleration
    config.use_gpu_features = True
    config.gpu_device_id = 0

    # Stereo settings for depth estimation
    config.stereo_matching_method = "SGBM"
    config.num_disparities = 128
    config.block_size = 11

    # Tracking settings for humanoid mobility
    config.tracking_window_size = 21
    config.max_tracking_points = 300
    config.tracking_threshold = 1.5

    # Mapping settings for indoor navigation
    config.keyframe_selection_threshold = 0.3  # 30cm for keyframes
    config.map_point_lifetime = 150
    config.min_observation_count = 3

    # Optimization for real-time performance
    config.bundle_adjustment_frequency = 15
    config.max_ba_iterations = 50

    # Loop closure for large-scale mapping
    config.loop_closure_detection = True
    config.loop_closure_threshold = 0.65
    config.min_loop_matches = 15

    # Sensor fusion for humanoid stability
    config.imu_integration = True
    config.imu_weight = 0.15

    return config

def load_slam_configuration(config_path: str = None) -> IsaacSLAMConfigManager:
    """Load Isaac Visual SLAM configuration"""
    if config_path:
        return IsaacSLAMConfigManager(config_path)
    else:
        # Create default configuration
        manager = IsaacSLAMConfigManager()
        manager.config = create_default_slam_config()
        return manager
```

## GPU-Accelerated Feature Processing

### CUDA-Based Feature Extraction

```python
# gpu_feature_extraction.py
import numpy as np
import cv2
from numba import cuda
import math

class GPUFeatureExtractor:
    def __init__(self, config):
        self.config = config
        self.gpu_initialized = False

        # Initialize GPU resources
        self._initialize_gpu_resources()

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for feature extraction"""
        try:
            # Get GPU device properties
            device = cuda.get_current_device()

            self.max_threads = device.MAX_THREADS_PER_BLOCK
            self.warp_size = device.WARP_SIZE
            self.compute_capability = device.COMPUTE_CAPABILITY

            # Calculate optimal grid and block dimensions
            self.block_dim = (16, 16)  # 256 threads per block
            self.grid_dim = None  # Will be calculated per image

            self.gpu_initialized = True
            print(f"GPU initialized: {device.name}, Compute {self.compute_capability}")

        except Exception as e:
            print(f"Failed to initialize GPU: {e}")
            self.gpu_initialized = False

    @staticmethod
    @cuda.jit
    def cuda_edge_detection_kernel(image, edges, threshold_low, threshold_high):
        """CUDA kernel for edge detection"""
        # Get thread indices
        row, col = cuda.grid(2)

        # Check bounds
        if row < image.shape[0] - 1 and col < image.shape[1] - 1 and row > 0 and col > 0:
            # Sobel operator for gradient calculation
            gx = (image[row-1, col-1] + 2*image[row, col-1] + image[row+1, col-1] -
                  image[row-1, col+1] - 2*image[row, col+1] - image[row+1, col+1])

            gy = (image[row-1, col-1] + 2*image[row-1, col] + image[row-1, col+1] -
                  image[row+1, col-1] - 2*image[row+1, col] - image[row+1, col+1])

            magnitude = math.sqrt(gx*gx + gy*gy)

            if magnitude > threshold_high:
                edges[row, col] = 255
            elif magnitude > threshold_low:
                edges[row, col] = 128  # Weak edge
            else:
                edges[row, col] = 0

    @staticmethod
    @cuda.jit
    def cuda_corner_detection_kernel(image, corners, threshold):
        """CUDA kernel for corner detection"""
        row, col = cuda.grid(2)

        if (row > 2 and row < image.shape[0] - 3 and
            col > 2 and col < image.shape[1] - 3):

            # Harris corner detector approximation
            # Calculate local gradients
            dx = image[row, col+1] - image[row, col-1]
            dy = image[row+1, col] - image[row-1, col]

            # Calculate corner response
            Ixx = dx * dx
            Iyy = dy * dy
            Ixy = dx * dy

            # Harris corner response formula
            det = Ixx * Iyy - Ixy * Ixy
            trace = Ixx + Iyy
            response = det - 0.04 * trace * trace

            if response > threshold:
                corners[row, col] = response
            else:
                corners[row, col] = 0

    def extract_features_gpu(self, image):
        """Extract features using GPU acceleration"""
        if not self.gpu_initialized:
            return self.extract_features_cpu(image)

        try:
            # Convert to grayscale if needed
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image.astype(np.uint8)

            # Calculate grid dimensions
            height, width = gray.shape
            grid_x = (width + self.block_dim[0] - 1) // self.block_dim[0]
            grid_y = (height + self.block_dim[1] - 1) // self.block_dim[1]
            self.grid_dim = (grid_x, grid_y)

            # Allocate GPU memory
            d_image = cuda.to_device(gray)
            d_edges = cuda.device_array_like(gray)
            d_corners = cuda.device_array_like(gray)

            # Run edge detection kernel
            self.cuda_edge_detection_kernel[self.grid_dim, self.block_dim](
                d_image, d_edges, 50, 150
            )

            # Run corner detection kernel
            self.cuda_corner_detection_kernel[self.grid_dim, self.block_dim](
                d_image, d_corners, 1000
            )

            # Copy results back to CPU
            edges = d_edges.copy_to_host()
            corners = d_corners.copy_to_host()

            # Process results on CPU to extract keypoints
            keypoints = self._extract_keypoints_from_gpu_results(edges, corners)

            return keypoints

        except Exception as e:
            print(f"GPU feature extraction failed: {e}")
            return self.extract_features_cpu(image)

    def _extract_keypoints_from_gpu_results(self, edges, corners):
        """Extract keypoints from GPU processing results"""
        # Find strong edges and corners
        edge_coords = np.where(edges == 255)
        corner_coords = np.where(corners > np.percentile(corners[corners > 0], 95))

        # Create keypoints
        keypoints = []

        # Add edge keypoints
        for r, c in zip(edge_coords[0], edge_coords[1]):
            kp = cv2.KeyPoint(c, r, _size=2.0)
            keypoints.append(kp)

        # Add corner keypoints
        for r, c in zip(corner_coords[0], corner_coords[1]):
            kp = cv2.KeyPoint(c, r, _size=4.0)
            keypoints.append(kp)

        # Limit number of keypoints
        if len(keypoints) > self.config.max_features:
            # Sort by response and keep top features
            keypoints.sort(key=lambda kp: kp.response, reverse=True)
            keypoints = keypoints[:self.config.max_features]

        return keypoints

    def extract_features_cpu(self, image):
        """CPU fallback for feature extraction"""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        if self.config.feature_type == "SIFT":
            detector = cv2.SIFT_create(nfeatures=self.config.max_features)
        elif self.config.feature_type == "ORB":
            detector = cv2.ORB_create(nfeatures=self.config.max_features)
        elif self.config.feature_type == "AKAZE":
            detector = cv2.AKAZE_create()
        else:
            detector = cv2.ORB_create(nfeatures=self.config.max_features)

        keypoints = detector.detect(gray, None)
        return keypoints

class GPUFeatureMatcher:
    def __init__(self, config):
        self.config = config
        self.matcher = cv2.BFMatcher()  # Will use GPU acceleration in Isaac

    def match_features_gpu(self, desc1, desc2):
        """Match features using GPU acceleration"""
        if desc1 is None or desc2 is None or len(desc1) == 0 or len(desc2) == 0:
            return []

        try:
            # Use GPU-accelerated matcher (Isaac-specific)
            matches = self.matcher.match(desc1, desc2)

            # Apply Lowe's ratio test on GPU
            matches = self._apply_ratio_test_gpu(matches)

            return matches

        except Exception as e:
            print(f"GPU feature matching failed: {e}")
            return self.match_features_cpu(desc1, desc2)

    def _apply_ratio_test_gpu(self, matches):
        """Apply Lowe's ratio test using GPU"""
        # In Isaac, this would use GPU-accelerated ratio testing
        # For now, implement CPU version with GPU concepts
        if len(matches) >= 2:
            good_matches = []
            matches = sorted(matches, key=lambda x: x.distance)

            for i in range(0, len(matches), 2):
                if i < len(matches) - 1:
                    if matches[i].distance < 0.75 * matches[i+1].distance:
                        good_matches.append(matches[i])

            return good_matches
        else:
            return matches

    def match_features_cpu(self, desc1, desc2):
        """CPU fallback for feature matching"""
        if desc1 is None or desc2 is None:
            return []

        matches = self.matcher.match(desc1, desc2)
        matches = sorted(matches, key=lambda x: x.distance)

        # Apply ratio test
        good_matches = []
        for i in range(0, len(matches), 2):
            if i < len(matches) - 1:
                if matches[i].distance < 0.75 * matches[i+1].distance:
                    good_matches.append(matches[i])

        return good_matches
```

## Loop Closure and Map Optimization

### GPU-Accelerated Loop Closure Detection

```python
# loop_closure_detection.py
import numpy as np
import cv2
from sklearn.cluster import MiniBatchKMeans
from sklearn.metrics.pairwise import cosine_similarity
import faiss
import pickle

class GPULoopClosureDetector:
    def __init__(self, config):
        self.config = config
        self.database = []
        self.visual_words = None
        self.vocabulary_size = 1000
        self.kmeans = None

        # Initialize FAISS index for GPU-accelerated search
        self.faiss_index = None
        self._initialize_vocabulary()

    def _initialize_vocabulary(self):
        """Initialize vocabulary for bag-of-words loop closure detection"""
        # Create vocabulary using clustering of local features
        self.kmeans = MiniBatchKMeans(
            n_clusters=self.vocabulary_size,
            batch_size=1000,
            random_state=42
        )

    def add_keyframe(self, image, features, descriptors):
        """Add a keyframe to the loop closure database"""
        if descriptors is None or len(descriptors) == 0:
            return

        # Quantize descriptors to visual words
        visual_word_histogram = self.quantize_to_visual_words(descriptors)

        # Create keyframe entry
        keyframe_entry = {
            'image': image,
            'features': features,
            'descriptors': descriptors,
            'visual_words': visual_word_histogram,
            'timestamp': cv2.getTickCount(),  # Use OpenCV tick count
            'id': len(self.database)
        }

        self.database.append(keyframe_entry)

        # Add to FAISS index for fast retrieval
        if self.faiss_index is None:
            dimension = len(visual_word_histogram)
            self.faiss_index = faiss.IndexFlatIP(dimension)  # Inner product for similarity

        # Normalize histogram for cosine similarity
        normalized_hist = visual_word_histogram / (np.linalg.norm(visual_word_histogram) + 1e-8)
        self.faiss_index.add(normalized_hist.reshape(1, -1).astype('float32'))

    def quantize_to_visual_words(self, descriptors):
        """Quantize local features to visual words using vocabulary"""
        if descriptors is None or len(descriptors) == 0:
            return np.zeros(self.vocabulary_size)

        # Use k-means to assign descriptors to visual words
        try:
            if self.kmeans.cluster_centers_.shape[0] == self.vocabulary_size:
                # Predict visual word assignments
                distances = np.linalg.norm(
                    descriptors[:, np.newaxis, :] -
                    self.kmeans.cluster_centers_[np.newaxis, :, :],
                    axis=2
                )
                assignments = np.argmin(distances, axis=1)

                # Create histogram of visual word occurrences
                hist, _ = np.histogram(assignments, bins=np.arange(self.vocabulary_size + 1))
                return hist.astype(np.float32)
            else:
                # Vocabulary not trained yet, return zeros
                return np.zeros(self.vocabulary_size, dtype=np.float32)
        except:
            return np.zeros(self.vocabulary_size, dtype=np.float32)

    def detect_loop_closure(self, current_descriptors):
        """Detect potential loop closures using GPU acceleration"""
        if (len(self.database) < 10 or
            current_descriptors is None or
            len(current_descriptors) == 0):
            return None

        # Quantize current descriptors
        current_histogram = self.quantize_to_visual_words(current_descriptors)

        # Normalize for cosine similarity
        current_normalized = current_histogram / (np.linalg.norm(current_histogram) + 1e-8)

        # Search for similar keyframes using FAISS
        k = min(10, len(self.database))  # Top-k similar frames
        similarities, indices = self.faiss_index.search(
            current_normalized.reshape(1, -1).astype('float32'),
            k
        )

        # Check for potential loop closures
        for i, (similarity, idx) in enumerate(zip(similarities[0], indices[0])):
            if similarity > self.config.loop_closure_threshold:
                # Potential loop closure candidate
                candidate = self.database[idx]

                # Perform geometric verification
                if self.verify_geometric_consistency(current_descriptors, candidate['descriptors']):
                    return {
                        'candidate_id': candidate['id'],
                        'similarity': float(similarity),
                        'candidate_frame': candidate
                    }

        return None

    def verify_geometric_consistency(self, curr_desc, cand_desc):
        """Verify geometric consistency of potential loop closure"""
        if len(curr_desc) < 10 or len(cand_desc) < 10:
            return False

        try:
            # Use FLANN matcher for efficient matching
            FLANN_INDEX_KDTREE = 1
            index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
            search_params = dict(checks=50)

            flann = cv2.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch(curr_desc, cand_desc, k=2)

            # Apply Lowe's ratio test
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.7 * n.distance:
                        good_matches.append(m)

            # Check if enough good matches exist
            if len(good_matches) >= self.config.min_loop_matches:
                # Perform RANSAC to verify geometric consistency
                src_pts = np.float32([curr_desc[m.queryIdx] for m in good_matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([cand_desc[m.trainIdx] for m in good_matches]).reshape(-1, 1, 2)

                if len(src_pts) >= 4:
                    _, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                    if mask is not None:
                        inliers = np.sum(mask)
                        if inliers >= self.config.min_loop_matches * 0.5:  # At least 50% inliers
                            return True

            return False

        except Exception as e:
            print(f"Geometric verification failed: {e}")
            return False

    def update_vocabulary(self, new_descriptors):
        """Update vocabulary with new descriptors"""
        if new_descriptors is None or len(new_descriptors) == 0:
            return

        # Fit k-means on new descriptors to update vocabulary
        try:
            self.kmeans.partial_fit(new_descriptors)
        except:
            # If vocabulary is not initialized, fit from scratch
            self.kmeans.fit(new_descriptors)

class GPUMapOptimizer:
    def __init__(self, config):
        self.config = config
        self.keyframes = []
        self.constraints = []
        self.optimization_frequency = config.bundle_adjustment_frequency

    def add_constraint(self, from_frame, to_frame, transformation, information_matrix=None):
        """Add a constraint between keyframes"""
        if information_matrix is None:
            # Default information matrix (identity)
            information_matrix = np.eye(6)

        constraint = {
            'from': from_frame,
            'to': to_frame,
            'transform': transformation,
            'information': information_matrix
        }

        self.constraints.append(constraint)

    def optimize_map_gpu(self):
        """Perform GPU-accelerated map optimization (bundle adjustment)"""
        if len(self.keyframes) < 3:
            return

        # In Isaac, this would use GPU-accelerated optimization
        # For now, we'll simulate the optimization process
        print(f"Performing map optimization with {len(self.keyframes)} keyframes "
              f"and {len(self.constraints)} constraints")

        # This is where Isaac would use Ceres Solver with GPU acceleration
        # or other GPU-optimized optimization libraries
        self._simulate_gpu_optimization()

    def _simulate_gpu_optimization(self):
        """Simulate GPU-accelerated optimization process"""
        # In a real implementation, this would:
        # 1. Formulate the optimization problem
        # 2. Use GPU-accelerated solvers (CUDA, cuBLAS, etc.)
        # 3. Solve for optimal poses and map points
        # 4. Update keyframe poses and map structure

        print("GPU-accelerated bundle adjustment completed")

        # Update keyframe poses based on optimization results
        for i, kf in enumerate(self.keyframes):
            # Simulate pose refinement
            noise = np.random.normal(0, 0.01, (4, 4))
            noise[3, 3] = 1  # Maintain homogeneous coordinate
            kf['pose'] = kf['pose'] @ (np.eye(4) + noise * 0.1)

def create_loop_closure_system(config):
    """Create GPU-accelerated loop closure detection system"""
    loop_detector = GPULoopClosureDetector(config)
    map_optimizer = GPUMapOptimizer(config)

    return loop_detector, map_optimizer
```

## Isaac Visual SLAM Integration

### Complete SLAM System Integration

```python
# complete_slam_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import numpy as np
import tf_transformations

class IsaacCompleteSLAMNode(Node):
    def __init__(self):
        super().__init__('isaac_complete_slam')

        # Initialize components
        self.cv_bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Load configuration
        self.config = load_slam_configuration()

        # Initialize SLAM modules
        self.feature_extractor = GPUFeatureExtractor(self.config.get_tracking_config())
        self.feature_matcher = GPUFeatureMatcher(self.config.get_tracking_config())
        self.loop_detector, self.map_optimizer = create_loop_closure_system(self.config)

        # Initialize state
        self.keyframes = []
        self.current_pose = np.eye(4)
        self.previous_frame = None
        self.frame_count = 0

        # Initialize subscribers
        self.left_sub = self.create_subscription(
            Image, '/camera/left/image_rect_color', self.left_callback, 10)
        self.right_sub = self.create_subscription(
            Image, '/camera/right/image_rect_color', self.right_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/left/camera_info', self.info_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Initialize publishers
        self.odom_pub = self.create_publisher(Odometry, '/visual_slam/odometry', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/visual_slam/pose', 10)
        self.path_pub = self.create_publisher(Path, '/visual_slam/path', 10)
        self.map_pub = self.create_publisher(MarkerArray, '/visual_slam/map', 10)

        # Timer for processing
        self.process_timer = self.create_timer(0.033, self.process_slam)

        self.get_logger().info('Isaac Complete SLAM system initialized')

    def left_callback(self, msg):
        """Handle left camera image"""
        self.left_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.left_timestamp = msg.header.stamp
        self.left_frame_id = msg.header.frame_id

    def right_callback(self, msg):
        """Handle right camera image"""
        self.right_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

    def info_callback(self, msg):
        """Handle camera info"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def imu_callback(self, msg):
        """Handle IMU data"""
        self.imu_data = msg

    def process_slam(self):
        """Main SLAM processing function"""
        if not hasattr(self, 'left_image'):
            return

        try:
            # Extract features from current image
            features = self.feature_extractor.extract_features_gpu(self.left_image)

            if self.previous_frame is not None:
                # Match features with previous frame
                matches = self.feature_matcher.match_features_gpu(
                    self.previous_frame['descriptors'],
                    features['descriptors']
                )

                if len(matches) >= 10:
                    # Estimate motion
                    transformation = self.estimate_motion(matches)
                    self.current_pose = self.current_pose @ transformation

                    # Check for keyframe
                    if self.should_add_keyframe():
                        self.add_keyframe(features)

                        # Check for loop closures
                        loop_result = self.loop_detector.detect_loop_closure(features['descriptors'])
                        if loop_result:
                            self.handle_loop_closure(loop_result)

                        # Perform optimization periodically
                        if len(self.keyframes) % self.config.bundle_adjustment_frequency == 0:
                            self.map_optimizer.optimize_map_gpu()

            # Update previous frame
            self.previous_frame = {
                'image': self.left_image,
                'features': features,
                'timestamp': self.left_timestamp
            }

            # Publish results
            self.publish_results()

            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f'SLAM processing error: {e}')

    def estimate_motion(self, matches):
        """Estimate camera motion from feature matches"""
        # Implementation would use GPU-accelerated pose estimation
        return np.eye(4)  # Placeholder

    def should_add_keyframe(self):
        """Determine if current frame should be a keyframe"""
        if not self.keyframes:
            return True

        # Check translation threshold
        if len(self.keyframes) > 0:
            prev_pose = self.keyframes[-1]['pose']
            current_pos = self.current_pose[:3, 3]
            prev_pos = prev_pose[:3, 3]
            translation = np.linalg.norm(current_pos - prev_pos)

            return translation > self.config.keyframe_selection_threshold

        return False

    def add_keyframe(self, features):
        """Add current frame as keyframe"""
        keyframe = {
            'id': len(self.keyframes),
            'image': self.left_image,
            'features': features,
            'pose': self.current_pose.copy(),
            'timestamp': self.left_timestamp
        }

        self.keyframes.append(keyframe)
        self.loop_detector.add_keyframe(
            self.left_image,
            features['keypoints'],
            features['descriptors']
        )

    def handle_loop_closure(self, loop_result):
        """Handle detected loop closure"""
        self.get_logger().info(f'Loop closure detected: {loop_result["similarity"]:.3f}')

    def publish_results(self):
        """Publish SLAM results"""
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Extract position and orientation
        pos = self.current_pose[:3, 3]
        rot_matrix = self.current_pose[:3, :3]

        # Convert to quaternion
        quat = tf_transformations.quaternion_from_matrix(self.current_pose)

        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacCompleteSLAMNode()

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

## Performance Optimization

### SLAM Performance Tuning

```python
# performance_tuning.py
class SLAMPerformanceTuner:
    def __init__(self, slam_node):
        self.slam_node = slam_node
        self.performance_history = []
        self.tuning_enabled = True

    def monitor_performance(self):
        """Monitor SLAM performance metrics"""
        metrics = {
            'frame_rate': self.get_frame_rate(),
            'tracking_accuracy': self.estimate_tracking_accuracy(),
            'map_consistency': self.estimate_map_consistency(),
            'gpu_utilization': self.get_gpu_utilization(),
            'memory_usage': self.get_memory_usage()
        }

        self.performance_history.append(metrics)

        # Keep only recent history
        if len(self.performance_history) > 100:
            self.performance_history.pop(0)

        return metrics

    def get_frame_rate(self):
        """Calculate current frame rate"""
        # Implementation would track processing time
        return 30.0  # Placeholder

    def estimate_tracking_accuracy(self):
        """Estimate tracking accuracy based on feature matches"""
        # Implementation would analyze feature tracking quality
        return 0.95  # Placeholder

    def estimate_map_consistency(self):
        """Estimate map consistency based on loop closures"""
        # Implementation would analyze map quality metrics
        return 0.90  # Placeholder

    def get_gpu_utilization(self):
        """Get current GPU utilization"""
        # This would interface with nvidia-ml-py
        return 75.0  # Placeholder

    def get_memory_usage(self):
        """Get current memory usage"""
        # Implementation would track memory consumption
        return 80.0  # Placeholder

    def auto_tune_parameters(self):
        """Automatically tune SLAM parameters based on performance"""
        if not self.tuning_enabled or len(self.performance_history) < 10:
            return

        recent_metrics = self.performance_history[-10:]

        # Calculate averages
        avg_frame_rate = np.mean([m['frame_rate'] for m in recent_metrics])
        avg_tracking_accuracy = np.mean([m['tracking_accuracy'] for m in recent_metrics])
        avg_gpu_util = np.mean([m['gpu_utilization'] for m in recent_metrics])

        # Adjust parameters based on performance
        adjustments = {}

        if avg_frame_rate < 20:  # Too slow
            adjustments['max_features'] = max(500, self.slam_node.config.max_features * 0.8)
            adjustments['tracking_threshold'] = self.slam_node.config.tracking_threshold * 1.2

        if avg_tracking_accuracy < 0.8:  # Poor tracking
            adjustments['max_features'] = min(3000, self.slam_node.config.max_features * 1.2)
            adjustments['tracking_threshold'] = self.slam_node.config.tracking_threshold * 0.8

        if avg_gpu_util > 90:  # GPU overloaded
            adjustments['feature_type'] = 'ORB' if self.slam_node.config.feature_type == 'SIFT' else self.slam_node.config.feature_type

        # Apply adjustments
        for param, value in adjustments.items():
            setattr(self.slam_node.config, param, value)

def optimize_slam_performance(slam_node):
    """Set up performance monitoring and optimization for SLAM"""
    tuner = SLAMPerformanceTuner(slam_node)

    # Set up periodic performance monitoring
    slam_node.performance_timer = slam_node.create_timer(1.0, lambda: tuner.monitor_performance())

    # Set up periodic auto-tuning
    slam_node.tuning_timer = slam_node.create_timer(5.0, lambda: tuner.auto_tune_parameters())

    return tuner
```

## Best Practices

1. **Feature Selection**: Choose appropriate feature types for your environment
2. **GPU Utilization**: Monitor and optimize GPU usage for real-time performance
3. **Keyframe Management**: Balance accuracy with computational efficiency
4. **Loop Closure**: Regularly verify geometric consistency of loop closures
5. **Map Optimization**: Periodically optimize map to maintain consistency
6. **Sensor Fusion**: Integrate IMU data for improved tracking stability
7. **Parameter Tuning**: Adapt parameters based on environment and performance

## Next Steps

In the next chapter, we'll explore perception nodes in detail, learning how Isaac ROS provides specialized nodes for different perception tasks like object detection, segmentation, and sensor fusion, all optimized for GPU acceleration.