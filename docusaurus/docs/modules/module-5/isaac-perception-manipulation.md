# Isaac Perception for Manipulation

In this chapter, we'll explore how to leverage NVIDIA Isaac's perception capabilities specifically for robotic manipulation tasks. Isaac's GPU-accelerated perception systems provide the visual understanding necessary for humanoid robots to perceive objects, estimate their poses, and execute precise manipulation actions.

## Manipulation Perception Overview

### Vision Requirements for Manipulation

Manipulation tasks require specialized perception capabilities:

- **Object Detection**: Identifying graspable objects in the environment
- **Pose Estimation**: Precise 6D pose estimation for manipulation
- **Grasp Point Detection**: Identifying viable grasp locations
- **Shape Understanding**: Understanding object geometry for manipulation
- **Material Properties**: Estimating object properties (weight, fragility, etc.)
- **Multi-view Integration**: Combining information from multiple viewpoints

### Perception Pipeline for Manipulation

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                    Manipulation Perception Pipeline                            │
├─────────────────────────────────────────────────────────────────────────────────┤
│  RGB-D Input → Object Detection → Pose Estimation → Grasp Planning → Action   │
│  (GPU Accel)    (GPU Accel)      (GPU Accel)      (GPU Accel)    (Execution) │
├─────────────────────────────────────────────────────────────────────────────────┤
│  Key Components:                                                              │
│  - Isaac ROS DetectNet: GPU-accelerated object detection                       │
│  - Isaac ROS Segmentation: Semantic understanding for grasping                 │
│  - Isaac ROS Pose Estimation: 6D pose estimation                               │
│  - Isaac ROS Grasp Detection: GPU-accelerated grasp planning                   │
│  - Isaac ROS 3D Perception: Depth and point cloud processing                   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Isaac Object Detection for Manipulation

### GPU-Accelerated Object Detection

```python
# manipulation_object_detection.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import jetson.inference
import jetson.utils

class IsaacManipulationDetectionNode(Node):
    def __init__(self):
        super().__init__('isaac_manipulation_detection')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize Isaac object detection
        self._initialize_isaac_detection()

        # Publishers and subscribers
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

        self.detection_pub = self.create_publisher(Detection2DArray, '/manipulation_objects', 10)
        self.graspable_objects_pub = self.create_publisher(Detection2DArray, '/graspable_objects', 10)

        # Camera calibration
        self.camera_matrix = None
        self.distortion_coeffs = None

        # Object classes relevant for manipulation
        self.manipulation_classes = [
            'bottle', 'cup', 'box', 'ball', 'book', 'phone',
            'laptop', 'mouse', 'keyboard', 'toy', 'food',
            'container', 'utensil', 'tool', 'cloth', 'paper'
        ]

        # Manipulation-specific parameters
        self.min_object_size = 0.02  # Minimum 2cm for manipulation
        self.max_detection_distance = 2.0  # Maximum 2m detection range
        self.graspability_threshold = 0.7  # Confidence threshold for graspable objects

        self.get_logger().info('Isaac Manipulation Detection node initialized')

    def _initialize_isaac_detection(self):
        """Initialize Isaac GPU-accelerated object detection"""
        try:
            # Initialize Isaac DetectNet model optimized for manipulation objects
            self.net = jetson.inference.detectNet(
                model_name="ssd-mobilenet-v2",
                threshold=0.5  # Lower threshold for more detections
            )

            # Set up Isaac perception components
            self.get_logger().info('Isaac manipulation object detection initialized')

        except Exception as e:
            self.get_logger().error(f'Isaac detection initialization error: {e}')
            raise

    def camera_info_callback(self, msg):
        """Update camera calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        """Process image for manipulation object detection"""
        try:
            # Convert ROS Image to CUDA image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cuda_image = jetson.utils.cudaFromNumpy(cv_image)

            # Perform GPU-accelerated object detection
            detections = self.net.Detect(cuda_image)

            # Process detections for manipulation
            manipulation_detections = self._process_manipulation_detections(detections, msg.header)

            # Publish all detected objects
            if manipulation_detections:
                detection_array = Detection2DArray()
                detection_array.header = msg.header
                detection_array.detections = manipulation_detections
                self.detection_pub.publish(detection_array)

                # Filter for graspable objects
                graspable_detections = self._filter_graspable_objects(manipulation_detections)
                if graspable_detections:
                    graspable_array = Detection2DArray()
                    graspable_array.header = msg.header
                    graspable_array.detections = graspable_detections
                    self.graspable_objects_pub.publish(graspable_array)

                    self.get_logger().info(f'Detected {len(graspable_detections)} graspable objects')

        except Exception as e:
            self.get_logger().error(f'Manipulation detection error: {e}')

    def _process_manipulation_detections(self, detections, header):
        """Process Isaac detections for manipulation relevance"""
        manipulation_detections = []

        for detection in detections:
            # Get class description
            class_desc = self.net.GetClassDesc(detection.ClassID)
            confidence = detection.Confidence

            # Check if object is relevant for manipulation
            if class_desc.lower() in self.manipulation_classes and confidence > 0.5:
                detection_msg = Detection2D()

                # Set bounding box
                detection_msg.bbox.center.x = float(detection.Center[0])
                detection_msg.bbox.center.y = float(detection.Center[1])
                detection_msg.bbox.size_x = float(detection.Width)
                detection_msg.bbox.size_y = float(detection.Height)

                # Check object size for manipulation feasibility
                if (detection.Width > self.min_object_size * self.camera_matrix[0, 0] and
                    detection.Height > self.min_object_size * self.camera_matrix[1, 1]):

                    # Add detection result
                    result = ObjectHypothesisWithPose()
                    result.hypothesis.class_id = class_desc
                    result.hypothesis.score = float(confidence)

                    # Estimate 3D position if camera calibration is available
                    if self.camera_matrix is not None:
                        # Convert 2D pixel coordinates to 3D world coordinates
                        # This is a simplified approach - in practice, use depth information
                        world_pos = self._pixel_to_world_coordinates(
                            detection.Center[0], detection.Center[1], 1.0  # 1m assumed depth
                        )

                        if world_pos:
                            result.pose.pose.position.x = world_pos[0]
                            result.pose.pose.position.y = world_pos[1]
                            result.pose.pose.position.z = world_pos[2]

                    detection_msg.results.append(result)
                    manipulation_detections.append(detection_msg)

        return manipulation_detections

    def _filter_graspable_objects(self, detections):
        """Filter detections for graspable objects only"""
        graspable_detections = []

        for detection in detections:
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id.lower()

                # Check if object is typically graspable
                if self._is_graspable_object(class_id):
                    graspable_detections.append(detection)

        return graspable_detections

    def _is_graspable_object(self, class_id):
        """Determine if an object class is typically graspable"""
        # Define which objects are typically graspable
        graspable_classes = [
            'bottle', 'cup', 'box', 'ball', 'book', 'phone',
            'laptop', 'mouse', 'keyboard', 'toy', 'food',
            'container', 'utensil', 'tool', 'cloth', 'paper',
            'can', 'bowl', 'plate', 'spoon', 'fork', 'knife'
        ]

        return class_id in graspable_classes

    def _pixel_to_world_coordinates(self, u, v, depth):
        """Convert pixel coordinates to world coordinates"""
        if self.camera_matrix is None:
            return None

        # Convert pixel to normalized coordinates
        x_norm = (u - self.camera_matrix[0, 2]) / self.camera_matrix[0, 0]
        y_norm = (v - self.camera_matrix[1, 2]) / self.camera_matrix[1, 1]

        # Convert to world coordinates
        world_x = x_norm * depth
        world_y = y_norm * depth
        world_z = depth

        return [world_x, world_y, world_z]

def main(args=None):
    rclpy.init(args=args)
    node = IsaacManipulationDetectionNode()

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

## Isaac 6D Pose Estimation

### GPU-Accelerated Pose Estimation

```python
# pose_estimation_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cupy as cp
from scipy.spatial.transform import Rotation as R

class IsaacPoseEstimationNode(Node):
    def __init__(self):
        super().__init__('isaac_pose_estimation')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize GPU resources
        self._initialize_gpu_resources()

        # Publishers and subscribers
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

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/manipulation_objects',
            self.detection_callback,
            10
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/object_pose', 10)
        self.pose_array_pub = self.create_publisher(PoseArray, '/object_poses', 10)

        # Camera calibration
        self.camera_matrix = None
        self.distortion_coeffs = None

        # Pose estimation parameters
        self.method = 'solvePnP'  # Options: 'solvePnP', 'epnp', 'iterative'
        self.min_points_for_pose = 4
        self.max_iterations = 100
        self.reprojection_error_threshold = 2.0

        # Object models (simplified - in practice, use CAD models)
        self.object_models = {
            'bottle': self._create_bottle_model(),
            'cup': self._create_cup_model(),
            'box': self._create_box_model(),
            'ball': self._create_sphere_model()
        }

        self.get_logger().info('Isaac Pose Estimation node initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for pose estimation"""
        try:
            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for pose estimation')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU initialization failed: {e}, using CPU fallback')

    def camera_info_callback(self, msg):
        """Update camera calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def detection_callback(self, msg):
        """Process detections for 6D pose estimation"""
        for detection in msg.detections:
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id.lower()

                if class_id in self.object_models:
                    # Estimate 6D pose for detected object
                    pose = self._estimate_object_pose(detection, class_id)

                    if pose:
                        pose_msg = PoseStamped()
                        pose_msg.header = msg.header
                        pose_msg.pose = pose

                        self.pose_pub.publish(pose_msg)

    def image_callback(self, msg):
        """Process image for pose estimation"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.gpu_available:
                # Use GPU-accelerated pose estimation
                pose_results = self._estimate_poses_gpu(cv_image)
            else:
                # CPU fallback
                pose_results = self._estimate_poses_cpu(cv_image)

            # Publish pose array
            if pose_results:
                pose_array_msg = PoseArray()
                pose_array_msg.header = msg.header
                pose_array_msg.poses = pose_results
                self.pose_array_pub.publish(pose_array_msg)

        except Exception as e:
            self.get_logger().error(f'Pose estimation error: {e}')

    def _estimate_object_pose(self, detection, class_id):
        """Estimate 6D pose of detected object"""
        try:
            if class_id not in self.object_models:
                return None

            # Get object model
            model_points = self.object_models[class_id]

            # Extract 2D points from detection (this would use keypoint detection in practice)
            # For this example, we'll use bounding box corners as 2D points
            bbox = detection.bbox
            center_x = bbox.center.x
            center_y = bbox.center.y
            width = bbox.size_x
            height = bbox.size_y

            # Create 2D points from bounding box
            image_points = np.array([
                [center_x - width/4, center_y - height/4],  # Top-left
                [center_x + width/4, center_y - height/4],  # Top-right
                [center_x + width/4, center_y + height/4],  # Bottom-right
                [center_x - width/4, center_y + height/4],  # Bottom-left
                [center_x, center_y]  # Center
            ], dtype=np.float32)

            # Use solvePnP for pose estimation
            success, rvec, tvec = cv2.solvePnP(
                model_points,
                image_points,
                self.camera_matrix,
                self.distortion_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success:
                # Convert rotation vector to quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)

                # Create pose message
                pose = Pose()
                pose.position.x = float(tvec[0][0])
                pose.position.y = float(tvec[1][0])
                pose.position.z = float(tvec[2][0])
                pose.orientation.x = float(quaternion[0])
                pose.orientation.y = float(quaternion[1])
                pose.orientation.z = float(quaternion[2])
                pose.orientation.w = float(quaternion[3])

                return pose

        except Exception as e:
            self.get_logger().error(f'Object pose estimation error: {e}')
            return None

    def _rotation_matrix_to_quaternion(self, rotation_matrix):
        """Convert rotation matrix to quaternion"""
        # Use scipy for reliable conversion
        r = R.from_matrix(rotation_matrix)
        return r.as_quat()

    def _create_bottle_model(self):
        """Create 3D model for bottle object"""
        # Simplified cylinder model for bottle
        points = []
        radius = 0.03  # 3cm radius
        height = 0.15  # 15cm height

        # Create points for cylinder
        for i in range(8):  # 8 points around circumference
            angle = 2 * np.pi * i / 8
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)

            # Top and bottom points
            points.append([x, y, height/2])
            points.append([x, y, -height/2])

        return np.array(points, dtype=np.float32)

    def _create_cup_model(self):
        """Create 3D model for cup object"""
        # Simplified cup model (cylinder with handle)
        points = []
        radius = 0.04  # 4cm radius
        height = 0.10  # 10cm height

        # Main body points
        for i in range(8):
            angle = 2 * np.pi * i / 8
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)

            points.append([x, y, height/2])
            points.append([x, y, -height/2])

        # Handle points (simplified)
        for i in range(4):
            angle = np.pi / 2  # Handle at 90 degrees
            x = (radius + 0.01) * np.cos(angle) + 0.02 * i / 4  # Extend handle
            y = (radius + 0.01) * np.sin(angle)
            z = height/4 * (i - 2)  # Vertical handle

            points.append([x, y, z])

        return np.array(points, dtype=np.float32)

    def _create_box_model(self):
        """Create 3D model for box object"""
        # 3D box model (8 corner points)
        size = [0.05, 0.05, 0.05]  # 5cm cube

        points = [
            [-size[0]/2, -size[1]/2, -size[2]/2],  # Bottom-back-left
            [size[0]/2, -size[1]/2, -size[2]/2],   # Bottom-back-right
            [size[0]/2, size[1]/2, -size[2]/2],    # Bottom-front-right
            [-size[0]/2, size[1]/2, -size[2]/2],   # Bottom-front-left
            [-size[0]/2, -size[1]/2, size[2]/2],   # Top-back-left
            [size[0]/2, -size[1]/2, size[2]/2],    # Top-back-right
            [size[0]/2, size[1]/2, size[2]/2],     # Top-front-right
            [-size[0]/2, size[1]/2, size[2]/2],    # Top-front-left
        ]

        return np.array(points, dtype=np.float32)

    def _create_sphere_model(self):
        """Create 3D model for spherical object"""
        # Simplified sphere model (octahedron approximation)
        radius = 0.03  # 3cm radius

        points = [
            [0, 0, radius],      # Top
            [0, 0, -radius],     # Bottom
            [radius, 0, 0],      # Right
            [-radius, 0, 0],     # Left
            [0, radius, 0],      # Front
            [0, -radius, 0],     # Back
            [radius/np.sqrt(2), radius/np.sqrt(2), 0],  # Diagonal
            [radius/np.sqrt(2), -radius/np.sqrt(2), 0], # Diagonal
        ]

        return np.array(points, dtype=np.float32)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacPoseEstimationNode()

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

## Isaac Grasp Detection

### GPU-Accelerated Grasp Point Detection

```python
# grasp_detection_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose, PoseArray
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header, Float32
from cv_bridge import CvBridge
import numpy as np
import cupy as cp
from scipy.spatial.distance import pdist, squareform
import cv2

class IsaacGraspDetectionNode(Node):
    def __init__(self):
        super().__init__('isaac_grasp_detection')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize GPU resources
        self._initialize_gpu_resources()

        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/point_cloud',
            self.pointcloud_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/manipulation_objects',
            self.detection_callback,
            10
        )

        self.grasp_poses_pub = self.create_publisher(PoseArray, '/grasp_poses', 10)
        self.grasp_quality_pub = self.create_publisher(Float32, '/grasp_quality', 10)

        # Grasp detection parameters
        self.min_grasp_score = 0.7
        self.max_grasps_per_object = 5
        self.grasp_approach_distance = 0.1  # 10cm approach
        self.gripper_width = 0.08  # 8cm gripper width
        self.gripper_height = 0.02  # 2cm gripper height

        # Grasp types to consider
        self.grasp_types = [
            'top_grasp',      # Grasp from above
            'side_grasp',     # Grasp from the side
            'pinch_grasp',    # Pinch grasp for small objects
            'power_grasp',    # Power grasp for large objects
            'wrap_grasp'      # Wrap grasp for irregular objects
        ]

        self.get_logger().info('Isaac Grasp Detection node initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for grasp detection"""
        try:
            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for grasp detection')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU initialization failed: {e}, using CPU fallback')

    def image_callback(self, msg):
        """Process image for grasp point detection"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.gpu_available:
                # Use GPU-accelerated grasp detection
                grasp_results = self._detect_grasps_gpu(cv_image)
            else:
                # CPU fallback
                grasp_results = self._detect_grasps_cpu(cv_image)

            # Publish grasp results
            if grasp_results:
                pose_array_msg = PoseArray()
                pose_array_msg.header = msg.header
                pose_array_msg.poses = [grasp['pose'] for grasp in grasp_results if grasp['score'] > self.min_grasp_score]

                self.grasp_poses_pub.publish(pose_array_msg)

                # Publish average grasp quality
                avg_quality = np.mean([grasp['score'] for grasp in grasp_results])
                quality_msg = Float32()
                quality_msg.data = float(avg_quality)
                self.grasp_quality_pub.publish(quality_msg)

                self.get_logger().info(f'Detected {len(pose_array_msg.poses)} high-quality grasps')

        except Exception as e:
            self.get_logger().error(f'Grasp detection error: {e}')

    def pointcloud_callback(self, msg):
        """Process point cloud for 3D grasp detection"""
        try:
            # Convert point cloud to numpy array (simplified)
            points = self._pointcloud_to_array(msg)

            if self.gpu_available:
                grasp_poses_3d = self._detect_3d_grasps_gpu(points)
            else:
                grasp_poses_3d = self._detect_3d_grasps_cpu(points)

            # Publish 3D grasp poses if needed
            if grasp_poses_3d:
                pose_array_msg = PoseArray()
                pose_array_msg.header = msg.header
                pose_array_msg.poses = grasp_poses_3d
                # Would need separate publisher for 3D grasps

        except Exception as e:
            self.get_logger().error(f'3D grasp detection error: {e}')

    def detection_callback(self, msg):
        """Process object detections for targeted grasp detection"""
        for detection in msg.detections:
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id.lower()

                # Only process grasp detection for relevant objects
                if self._is_graspable_object(class_id):
                    grasp_poses = self._generate_grasp_poses_for_object(detection)

                    if grasp_poses:
                        # Publish object-specific grasps
                        grasp_array_msg = PoseArray()
                        grasp_array_msg.header = msg.header
                        grasp_array_msg.poses = grasp_poses
                        # Would need separate publisher for object-specific grasps

    def _detect_grasps_gpu(self, image):
        """Detect grasp points using GPU acceleration"""
        try:
            # Upload image to GPU
            gpu_image = cp.asarray(image)

            # Apply GPU-accelerated edge detection
            edges = self._gpu_edge_detection(gpu_image)

            # Apply GPU-accelerated corner detection for grasp points
            grasp_points = self._gpu_grasp_point_detection(edges)

            # Evaluate grasp quality using GPU
            grasp_results = []
            for point in grasp_points:
                grasp_pose = self._create_grasp_pose(point, gpu_image)
                quality = self._evaluate_grasp_quality_gpu(grasp_pose, gpu_image)

                if quality > self.min_grasp_score:
                    grasp_results.append({
                        'pose': grasp_pose,
                        'score': float(quality),
                        'position': (int(point[0]), int(point[1]))
                    })

            return grasp_results

        except Exception as e:
            self.get_logger().error(f'GPU grasp detection error: {e}')
            return self._detect_grasps_cpu(cp.asnumpy(gpu_image))

    def _gpu_edge_detection(self, image):
        """GPU-accelerated edge detection"""
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cp.dot(image[...,:3], [0.299, 0.587, 0.114])
        else:
            gray = image

        # Apply Gaussian blur
        blurred = cp.zeros_like(gray)
        # This would use GPU convolution in practice
        # For now, return the grayscale image
        return gray

    def _gpu_grasp_point_detection(self, edges):
        """GPU-accelerated grasp point detection"""
        # This would implement GPU-parallelized grasp detection algorithms
        # Such as: GPU-parallelized GPD (Grasp Pose Detection) or other methods

        # For demonstration, find edge points that could be grasp locations
        edge_points = cp.where(edges > 128)  # Simple threshold for edge detection
        edge_coords = cp.column_stack((edge_points[1], edge_points[0]))  # x, y coordinates

        # Sample some edge points as grasp candidates
        if len(edge_coords) > 0:
            # Limit to 50 candidates for performance
            sample_size = min(50, len(edge_coords))
            indices = cp.random.choice(len(edge_coords), size=sample_size, replace=False)
            sampled_coords = edge_coords[indices]
            return cp.asnumpy(sampled_coords)
        else:
            return np.array([])

    def _evaluate_grasp_quality_gpu(self, grasp_pose, image):
        """Evaluate grasp quality using GPU acceleration"""
        try:
            # This would implement GPU-parallelized grasp quality evaluation
            # Based on: surface normals, curvature, object geometry, etc.

            # For this example, return a simple quality score based on image features
            # at the grasp location
            x = int(grasp_pose.position.x)
            y = int(grasp_pose.position.y)

            if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                # Check local image features around grasp point
                region_size = 20
                x_start = max(0, x - region_size//2)
                x_end = min(image.shape[1], x + region_size//2)
                y_start = max(0, y - region_size//2)
                y_end = min(image.shape[0], y + region_size//2)

                if len(image.shape) == 3:
                    region = image[y_start:y_end, x_start:x_end, :]
                else:
                    region = image[y_start:y_end, x_start:x_end]

                # Calculate local variance as a proxy for texture/edge density
                if region.size > 0:
                    variance = cp.var(region)
                    quality = min(1.0, float(variance) / 1000.0)  # Normalize
                    return quality

            return 0.1  # Low quality if out of bounds

        except Exception as e:
            self.get_logger().error(f'GPU grasp quality evaluation error: {e}')
            return 0.5  # Default quality

    def _create_grasp_pose(self, point, image):
        """Create grasp pose from image coordinates"""
        x, y = point

        pose = Pose()
        # Convert pixel coordinates to world coordinates (simplified)
        pose.position.x = x * 0.001  # 1mm per pixel approximation
        pose.position.y = y * 0.001
        pose.position.z = 0.5  # Fixed height for now

        # Set orientation for grasp (simplified - would be more complex in practice)
        # Grasp from above (top grasp)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        return pose

    def _generate_grasp_poses_for_object(self, detection):
        """Generate specific grasp poses for detected object"""
        bbox = detection.bbox
        center_x = int(bbox.center.x)
        center_y = int(bbox.center.y)
        width = int(bbox.size_x)
        height = int(bbox.size_y)

        grasp_poses = []

        # Generate multiple grasp points around the object
        for grasp_type in self.grasp_types:
            if grasp_type == 'top_grasp':
                # Top grasp - from above
                grasp_pose = self._create_top_grasp(center_x, center_y, width, height)
            elif grasp_type == 'side_grasp':
                # Side grasp - from the side
                grasp_pose = self._create_side_grasp(center_x, center_y, width, height)
            elif grasp_type == 'pinch_grasp':
                # Pinch grasp for small objects
                grasp_pose = self._create_pinch_grasp(center_x, center_y, width, height)
            else:
                continue

            if grasp_pose:
                grasp_poses.append(grasp_pose)

                # Limit number of grasps per object
                if len(grasp_poses) >= self.max_grasps_per_object:
                    break

        return grasp_poses

    def _create_top_grasp(self, center_x, center_y, width, height):
        """Create top-down grasp pose"""
        pose = Pose()
        pose.position.x = center_x * 0.001
        pose.position.y = center_y * 0.001
        pose.position.z = 0.6  # Above the object

        # Top-down orientation (Z-axis pointing down)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        return pose

    def _create_side_grasp(self, center_x, center_y, width, height):
        """Create side grasp pose"""
        pose = Pose()
        pose.position.x = (center_x + width/2) * 0.001  # To the right of object
        pose.position.y = center_y * 0.001
        pose.position.z = (center_y + height/2) * 0.001  # Half height

        # Side grasp orientation (X-axis pointing toward object)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.707  # 45 degrees rotation
        pose.orientation.w = 0.707

        return pose

    def _create_pinch_grasp(self, center_x, center_y, width, height):
        """Create pinch grasp pose for small objects"""
        pose = Pose()
        pose.position.x = center_x * 0.001
        pose.position.y = center_y * 0.001
        pose.position.z = 0.5  # Standard height

        # Pinch grasp orientation
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        return pose

    def _is_graspable_object(self, class_id):
        """Determine if object class is suitable for grasp detection"""
        graspable_classes = [
            'bottle', 'cup', 'box', 'ball', 'book', 'phone',
            'laptop', 'mouse', 'keyboard', 'toy', 'food',
            'container', 'utensil', 'tool', 'cloth', 'paper',
            'can', 'bowl', 'plate', 'spoon', 'fork', 'knife'
        ]

        return class_id in graspable_classes

def main(args=None):
    rclpy.init(args=args)
    node = IsaacGraspDetectionNode()

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

## Isaac 3D Perception for Manipulation

### GPU-Accelerated 3D Scene Understanding

```python
# 3d_perception_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from geometry_msgs.msg import Point, Pose, PoseArray
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cupy as cp
from sensor_msgs_py import point_cloud2
from sklearn.cluster import DBSCAN
import open3d as o3d

class Isaac3DPerceptionNode(Node):
    def __init__(self):
        super().__init__('isaac_3d_perception')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize GPU resources
        self._initialize_gpu_resources()

        # Publishers and subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/point_cloud',
            self.pointcloud_callback,
            10
        )

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

        self.segmented_objects_pub = self.create_publisher(Detection2DArray, '/segmented_objects_3d', 10)
        self.surface_normals_pub = self.create_publisher(PoseArray, '/surface_normals', 10)

        # Camera calibration
        self.camera_info = None

        # 3D perception parameters
        self.voxel_size = 0.01  # 1cm voxels
        self.cluster_eps = 0.02  # 2cm clustering distance
        self.min_cluster_points = 10
        self.table_height_threshold = 0.8  # Table height in meters

        # Object properties database
        self.object_properties = {
            'size_thresholds': {
                'small': 0.05,      # 5cm
                'medium': 0.2,      # 20cm
                'large': 0.5        # 50cm
            },
            'material_properties': {
                'rigid': {'density': 1000, 'friction': 0.8},
                'flexible': {'density': 500, 'friction': 0.6},
                'fragile': {'density': 200, 'friction': 0.4}
            }
        }

        self.get_logger().info('Isaac 3D Perception node initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for 3D perception"""
        try:
            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for 3D perception')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU initialization failed: {e}, using CPU fallback')

    def camera_info_callback(self, msg):
        """Update camera calibration information"""
        self.camera_info = msg

    def pointcloud_callback(self, msg):
        """Process point cloud for 3D perception"""
        try:
            # Convert PointCloud2 to numpy array
            points = self._pointcloud2_to_array(msg)

            if len(points) == 0:
                return

            if self.gpu_available:
                # Use GPU-accelerated 3D processing
                objects_3d = self._process_3d_scene_gpu(points)
            else:
                # CPU fallback
                objects_3d = self._process_3d_scene_cpu(points)

            # Publish 3D object information
            if objects_3d:
                detection_array = Detection2DArray()
                detection_array.header = msg.header

                for obj in objects_3d:
                    detection_msg = self._create_detection_from_3d_object(obj, msg.header)
                    detection_array.detections.append(detection_msg)

                self.segmented_objects_pub.publish(detection_array)

                self.get_logger().info(f'Detected {len(objects_3d)} 3D objects')

        except Exception as e:
            self.get_logger().error(f'3D perception error: {e}')

    def _pointcloud2_to_array(self, pointcloud_msg):
        """Convert PointCloud2 message to numpy array"""
        try:
            # Extract XYZ points from PointCloud2
            points = []
            for point in point_cloud2.read_points(
                pointcloud_msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            ):
                points.append([point[0], point[1], point[2]])

            return np.array(points, dtype=np.float32)

        except Exception as e:
            self.get_logger().error(f'Point cloud conversion error: {e}')
            return np.array([])

    def _process_3d_scene_gpu(self, points):
        """Process 3D scene using GPU acceleration"""
        try:
            # Upload points to GPU
            gpu_points = cp.asarray(points)

            # Apply GPU-accelerated filtering (e.g., remove ground plane)
            filtered_points = self._gpu_ground_plane_filtering(gpu_points)

            # Cluster points to find objects
            clusters = self._gpu_point_clustering(filtered_points)

            # Extract object information from clusters
            objects_3d = []
            for cluster in clusters:
                if len(cluster) >= self.min_cluster_points:
                    obj_info = self._extract_object_info_gpu(cluster)
                    objects_3d.append(obj_info)

            return objects_3d

        except Exception as e:
            self.get_logger().error(f'GPU 3D scene processing error: {e}')
            return self._process_3d_scene_cpu(cp.asnumpy(gpu_points))

    def _gpu_ground_plane_filtering(self, points):
        """GPU-accelerated ground plane filtering"""
        try:
            # Simple height-based filtering (in practice, use RANSAC or other methods)
            z_values = points[:, 2]
            filtered_mask = z_values > self.table_height_threshold
            return points[filtered_mask]

        except Exception as e:
            self.get_logger().error(f'GPU ground filtering error: {e}')
            return points

    def _gpu_point_clustering(self, points):
        """GPU-accelerated point clustering"""
        try:
            # In practice, this would use GPU-parallelized clustering algorithms
            # For this example, use CPU DBSCAN as fallback but with GPU pre-processing

            # Convert back to CPU for clustering (GPU clustering libraries limited)
            cpu_points = cp.asnumpy(points)

            # Apply DBSCAN clustering
            clustering = DBSCAN(eps=self.cluster_eps, min_samples=self.min_cluster_points)
            labels = clustering.fit_predict(cpu_points)

            # Group points by cluster
            clusters = []
            unique_labels = set(labels)
            for label in unique_labels:
                if label != -1:  # -1 is noise in DBSCAN
                    cluster_points = cpu_points[labels == label]
                    clusters.append(cluster_points)

            return clusters

        except Exception as e:
            self.get_logger().error(f'GPU clustering error: {e}')
            return []

    def _extract_object_info_gpu(self, cluster_points):
        """Extract object information using GPU acceleration"""
        try:
            # Calculate bounding box using GPU
            min_point = cp.min(cluster_points, axis=0)
            max_point = cp.max(cluster_points, axis=0)
            center = cp.mean(cluster_points, axis=0)

            # Calculate object properties
            size = cp.asnumpy(max_point - min_point)
            center_pos = cp.asnumpy(center)

            # Estimate surface normals (simplified)
            # In practice, this would use GPU-accelerated normal estimation
            normal = [0, 0, 1]  # Assume upward normal for now

            # Classify object based on size
            avg_size = np.mean(size)
            if avg_size < self.object_properties['size_thresholds']['small']:
                size_class = 'small'
            elif avg_size < self.object_properties['size_thresholds']['medium']:
                size_class = 'medium'
            else:
                size_class = 'large'

            object_info = {
                'center': center_pos.tolist(),
                'size': size.tolist(),
                'bbox_min': cp.asnumpy(min_point).tolist(),
                'bbox_max': cp.asnumpy(max_point).tolist(),
                'surface_normal': normal,
                'point_count': len(cluster_points),
                'size_class': size_class
            }

            return object_info

        except Exception as e:
            self.get_logger().error(f'GPU object info extraction error: {e}')
            return None

    def _create_detection_from_3d_object(self, obj_info, header):
        """Create Detection2D from 3D object information"""
        detection = Detection2D()
        detection.header = header

        # Project 3D bounding box to 2D (simplified)
        # In practice, use camera matrix to project 3D points to 2D
        if self.camera_info:
            # Calculate projected 2D bounding box
            # This would involve projecting 3D bbox corners to 2D image
            projected_center = self._project_3d_to_2d(obj_info['center'])
            projected_size = self._project_3d_size_to_2d(obj_info['size'])

            if projected_center and projected_size:
                detection.bbox.center.x = projected_center[0]
                detection.bbox.center.y = projected_center[1]
                detection.bbox.size_x = projected_size[0]
                detection.bbox.size_y = projected_size[1]

        # Add object properties as results
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = f"object_{obj_info['size_class']}"
        result.hypothesis.score = 0.9  # High confidence for detected object
        result.pose.pose.position.x = obj_info['center'][0]
        result.pose.pose.position.y = obj_info['center'][1]
        result.pose.pose.position.z = obj_info['center'][2]

        detection.results.append(result)
        return detection

    def _project_3d_to_2d(self, point_3d):
        """Project 3D point to 2D image coordinates"""
        if not self.camera_info:
            return None

        # Simple projection using camera matrix
        # [u, v, 1]^T = K * [x, y, z, 1]^T
        x, y, z = point_3d

        if z == 0:
            return None

        u = (self.camera_info.k[0] * x + self.camera_info.k[2] * z) / z
        v = (self.camera_info.k[4] * y + self.camera_info.k[5] * z) / z

        return [u, v]

    def _project_3d_size_to_2d(self, size_3d):
        """Project 3D size to 2D pixel size"""
        if not self.camera_info:
            return [100, 100]  # Default size

        # Simple approximation
        x_size = size_3d[0] * self.camera_info.k[0]  # fx
        y_size = size_3d[1] * self.camera_info.k[4]  # fy

        # Convert to pixels (simplified)
        pixel_x_size = x_size * 1000  # Scale factor
        pixel_y_size = y_size * 1000  # Scale factor

        return [pixel_x_size, pixel_y_size]

def main(args=None):
    rclpy.init(args=args)
    node = Isaac3DPerceptionNode()

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

## Isaac Manipulation Perception Pipeline

### Complete Perception Integration

```python
# complete_manipulation_perception.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Point
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import numpy as np

class IsaacCompleteManipulationPerceptionNode(Node):
    def __init__(self):
        super().__init__('isaac_complete_manipulation_perception')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize all perception components
        self.object_detector = IsaacManipulationDetectionNode()
        self.pose_estimator = IsaacPoseEstimationNode()
        self.grasp_detector = IsaacGraspDetectionNode()
        self.three_d_perceptor = Isaac3DPerceptionNode()

        # Publishers for fused perception results
        self.fused_objects_pub = self.create_publisher(Detection2DArray, '/fused_manipulation_objects', 10)
        self.fused_grasps_pub = self.create_publisher(PoseArray, '/fused_grasp_poses', 10)
        self.scene_description_pub = self.create_publisher(String, '/scene_description', 10)

        # Perception fusion parameters
        self.fusion_window = 0.1  # 100ms fusion window
        self.confidence_threshold = 0.6
        self.spatial_tolerance = 0.05  # 5cm spatial tolerance

        # Perception state
        self.current_detections = []
        self.current_poses = []
        self.current_grasps = []
        self.current_3d_objects = []

        # Timer for perception fusion
        self.fusion_timer = self.create_timer(0.1, self.perception_fusion_loop)

        self.get_logger().info('Complete Isaac Manipulation Perception system initialized')

    def perception_fusion_loop(self):
        """Fusion loop to combine perception results"""
        try:
            # Fuse perception results from all components
            fused_objects = self._fuse_object_detections()
            fused_grasps = self._fuse_grasp_detections()

            # Publish fused results
            if fused_objects:
                self.fused_objects_pub.publish(fused_objects)

            if fused_grasps:
                self.fused_grasps_pub.publish(fused_grasps)

            # Generate scene description
            scene_description = self._generate_scene_description(fused_objects, fused_grasps)
            if scene_description:
                description_msg = String()
                description_msg.data = scene_description
                self.scene_description_pub.publish(description_msg)

        except Exception as e:
            self.get_logger().error(f'Perception fusion error: {e}')

    def _fuse_object_detections(self):
        """Fuse object detections from multiple sources"""
        # Combine 2D and 3D object detections
        all_detections = []

        # Add 2D detections
        if self.current_detections:
            all_detections.extend(self.current_detections)

        # Add 3D detections with spatial association
        if self.current_3d_objects and self.current_detections:
            associated_detections = self._associate_3d_with_2d()
            all_detections.extend(associated_detections)

        # Filter and validate detections
        validated_detections = []
        for detection in all_detections:
            if self._validate_detection(detection):
                validated_detections.append(detection)

        # Create Detection2DArray message
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera_link'
        detection_array.detections = validated_detections

        return detection_array

    def _fuse_grasp_detections(self):
        """Fuse grasp detections from multiple sources"""
        # Combine grasp detections from different methods
        all_grasps = []

        # Add 2D image-based grasps
        if self.current_grasps:
            all_grasps.extend(self.current_grasps)

        # Add 3D geometry-based grasps
        if self.current_3d_objects:
            geometry_grasps = self._generate_geometry_based_grasps()
            all_grasps.extend(geometry_grasps)

        # Filter grasps by quality and spatial distribution
        filtered_grasps = self._filter_grasps_by_quality(all_grasps)

        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'camera_link'
        pose_array.poses = filtered_grasps

        return pose_array

    def _validate_detection(self, detection):
        """Validate detection quality"""
        if not detection.results:
            return False

        # Check confidence score
        confidence = detection.results[0].hypothesis.score
        if confidence < self.confidence_threshold:
            return False

        # Check spatial validity
        if (abs(detection.bbox.center.x) > 1000 or  # Pixel coordinates sanity check
            abs(detection.bbox.center.y) > 1000 or
            detection.bbox.size_x <= 0 or
            detection.bbox.size_y <= 0):
            return False

        return True

    def _associate_3d_with_2d(self):
        """Associate 3D objects with 2D detections"""
        associated_detections = []

        for detection_2d in self.current_detections:
            for obj_3d in self.current_3d_objects:
                # Check spatial correspondence
                projected_2d = self._project_3d_to_2d(obj_3d['center'])
                if projected_2d:
                    detection_center = (detection_2d.bbox.center.x, detection_2d.bbox.center.y)
                    distance = np.sqrt(
                        (projected_2d[0] - detection_center[0])**2 +
                        (projected_2d[1] - detection_center[1])**2
                    )

                    if distance < self.spatial_tolerance:
                        # Associate 3D information with 2D detection
                        associated_detection = Detection2D()
                        associated_detection.header = detection_2d.header
                        associated_detection.bbox = detection_2d.bbox
                        associated_detection.results = detection_2d.results.copy()

                        # Add 3D pose information
                        result = ObjectHypothesisWithPose()
                        result.hypothesis.class_id = f"{detection_2d.results[0].hypothesis.class_id}_3d"
                        result.hypothesis.score = detection_2d.results[0].hypothesis.score
                        result.pose.pose.position.x = obj_3d['center'][0]
                        result.pose.pose.position.y = obj_3d['center'][1]
                        result.pose.pose.position.z = obj_3d['center'][2]

                        associated_detection.results.append(result)
                        associated_detections.append(associated_detection)

        return associated_detections

    def _filter_grasps_by_quality(self, grasps):
        """Filter grasps by quality and spatial distribution"""
        if not grasps:
            return []

        # Sort grasps by quality score
        sorted_grasps = sorted(grasps, key=lambda x: self._get_grasp_quality(x), reverse=True)

        # Keep grasps that are spatially distributed
        filtered_grasps = []
        for grasp in sorted_grasps:
            if len(filtered_grasps) == 0:
                filtered_grasps.append(grasp)
            else:
                # Check distance to existing grasps
                too_close = False
                grasp_pos = np.array([
                    grasp.position.x,
                    grasp.position.y,
                    grasp.position.z
                ])

                for existing_grasp in filtered_grasps:
                    existing_pos = np.array([
                        existing_grasp.position.x,
                        existing_grasp.position.y,
                        existing_grasp.position.z
                    ])

                    distance = np.linalg.norm(grasp_pos - existing_pos)
                    if distance < 0.1:  # 10cm minimum distance
                        too_close = True
                        break

                if not too_close:
                    filtered_grasps.append(grasp)

                # Limit total grasps
                if len(filtered_grasps) >= 10:
                    break

        return filtered_grasps

    def _get_grasp_quality(self, grasp_pose):
        """Get quality score for grasp pose (placeholder)"""
        # In a real implementation, this would evaluate grasp quality based on:
        # - Surface normals at grasp point
        # - Object geometry compatibility
        # - Robot kinematic feasibility
        # - Force closure analysis
        return 0.8  # Placeholder quality

    def _generate_scene_description(self, objects, grasps):
        """Generate textual scene description"""
        if not objects:
            return "No objects detected in the scene."

        description_parts = []

        # Describe objects
        for detection in objects.detections:
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                position = detection.results[-1].pose.pose.position if len(detection.results) > 1 else None

                if confidence > 0.7:  # High confidence objects only
                    obj_desc = f"{class_id} at"
                    if position:
                        obj_desc += f" ({position.x:.2f}, {position.y:.2f}, {position.z:.2f})"
                    else:
                        obj_desc += f" center ({detection.bbox.center.x}, {detection.bbox.center.y})"
                    description_parts.append(obj_desc)

        # Describe grasp opportunities
        if len(grasps.poses) > 0:
            description_parts.append(f"Found {len(grasps.poses)} grasp opportunities")

        return f"Scene contains: {', '.join(description_parts)}."

def main(args=None):
    rclpy.init(args=args)
    node = IsaacCompleteManipulationPerceptionNode()

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

### Perception Performance Tuning

```python
# perception_performance.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray
import time
from collections import deque
import GPUtil

class IsaacPerceptionPerformanceNode(Node):
    def __init__(self):
        super().__init__('isaac_perception_performance')

        # Performance monitoring
        self.detection_times = deque(maxlen=100)
        self.pose_estimation_times = deque(maxlen=100)
        self.grasp_detection_times = deque(maxlen=100)
        self.gpu_utilization = deque(maxlen=100)

        # Publishers for performance metrics
        self.detection_perf_pub = self.create_publisher(Float32, '/perf/detection_time', 10)
        self.pose_perf_pub = self.create_publisher(Float32, '/perf/pose_time', 10)
        self.grasp_perf_pub = self.create_publisher(Float32, '/perf/grasp_time', 10)
        self.gpu_util_pub = self.create_publisher(Float32, '/perf/gpu_utilization', 10)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/perception_diagnostics', 10)

        # Performance optimization parameters
        self.target_fps = 30.0
        self.target_detection_time = 1.0 / self.target_fps * 0.8  # 80% of frame time
        self.adaptive_resolution = True
        self.current_resolution_scale = 1.0

        # Timer for performance monitoring
        self.performance_timer = self.create_timer(1.0, self.performance_monitoring_loop)

        self.get_logger().info('Isaac Perception Performance Monitor initialized')

    def performance_monitoring_loop(self):
        """Monitor and publish performance metrics"""
        try:
            # Calculate average performance
            avg_detection_time = sum(self.detection_times) / len(self.detection_times) if self.detection_times else 0.0
            avg_pose_time = sum(self.pose_estimation_times) / len(self.pose_estimation_times) if self.pose_estimation_times else 0.0
            avg_grasp_time = sum(self.grasp_detection_times) / len(self.grasp_detection_times) if self.grasp_detection_times else 0.0
            avg_gpu_util = sum(self.gpu_utilization) / len(self.gpu_utilization) if self.gpu_utilization else 0.0

            # Publish metrics
            detection_msg = Float32()
            detection_msg.data = float(avg_detection_time)
            self.detection_perf_pub.publish(detection_msg)

            pose_msg = Float32()
            pose_msg.data = float(avg_pose_time)
            self.pose_perf_pub.publish(pose_msg)

            grasp_msg = Float32()
            grasp_msg.data = float(avg_grasp_time)
            self.grasp_perf_pub.publish(grasp_msg)

            gpu_msg = Float32()
            gpu_msg.data = float(avg_gpu_util)
            self.gpu_util_pub.publish(gpu_msg)

            # Apply adaptive optimizations based on performance
            self._apply_adaptive_optimizations(
                avg_detection_time, avg_pose_time, avg_grasp_time, avg_gpu_util
            )

            # Log performance summary
            self.get_logger().info(
                f'Perception Performance - Detection: {avg_detection_time:.3f}s, '
                f'Pose: {avg_pose_time:.3f}s, '
                f'Grasp: {avg_grasp_time:.3f}s, '
                f'GPU: {avg_gpu_util:.1f}%'
            )

        except Exception as e:
            self.get_logger().error(f'Performance monitoring error: {e}')

    def _apply_adaptive_optimizations(self, detection_time, pose_time, grasp_time, gpu_util):
        """Apply adaptive optimizations based on performance"""
        # Adjust image processing resolution based on performance
        if self.adaptive_resolution:
            target_time = self.target_detection_time

            if detection_time > target_time * 1.2:  # 20% over budget
                # Reduce resolution to improve performance
                self.current_resolution_scale = max(0.5, self.current_resolution_scale * 0.9)
                self.get_logger().info(f'Reduced processing resolution to {self.current_resolution_scale:.2f}')

            elif detection_time < target_time * 0.6:  # 40% under budget
                # Increase resolution for better quality
                self.current_resolution_scale = min(1.0, self.current_resolution_scale * 1.1)
                self.get_logger().info(f'Increased processing resolution to {self.current_resolution_scale:.2f}')

        # Adjust model complexity based on GPU utilization
        if gpu_util > 85:
            self.get_logger().warn('High GPU utilization detected, consider model optimization')

def main(args=None):
    rclpy.init(args=args)
    node = IsaacPerceptionPerformanceNode()

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

## Best Practices for Isaac Manipulation Perception

### Integration Best Practices

1. **Multi-modal Fusion**: Combine visual, depth, and tactile information
2. **Real-time Performance**: Optimize for 30Hz+ perception rates
3. **Quality Control**: Validate perception results before manipulation
4. **Safety First**: Always verify grasp feasibility and object properties
5. **Robustness**: Handle varying lighting and environmental conditions
6. **Calibration**: Maintain accurate sensor calibration for precision
7. **Filtering**: Apply appropriate filtering to reduce noise
8. **Validation**: Cross-validate results when possible

### GPU Optimization Guidelines

1. **Memory Management**: Use GPU memory pools for efficient allocation
2. **Batch Processing**: Process multiple inputs together when possible
3. **Asynchronous Operations**: Overlap computation with data transfer
4. **Model Optimization**: Use TensorRT for optimized inference
5. **Resolution Management**: Adjust processing resolution based on requirements
6. **Pipeline Parallelization**: Separate preprocessing, inference, and postprocessing

## Troubleshooting Common Issues

### Perception-Specific Issues

**Issue**: Poor grasp detection accuracy
**Solution**: Improve lighting conditions, adjust detection thresholds, verify camera calibration

**Issue**: Slow processing performance
**Solution**: Reduce image resolution, optimize GPU memory usage, use smaller models

**Issue**: Inconsistent pose estimation
**Solution**: Verify camera calibration, improve feature detection, increase point correspondences

**Issue**: Object detection misses small objects
**Solution**: Use higher resolution models, improve lighting, adjust minimum object size

**Issue**: GPU memory exhaustion
**Solution**: Reduce batch sizes, optimize memory allocation, use memory pools

## Next Steps

In the next chapter, we'll explore Isaac's manipulation planning capabilities, learning how to integrate perception results with motion planning to create robust manipulation behaviors for humanoid robots. We'll cover grasp planning, trajectory optimization, and safe manipulation execution strategies.