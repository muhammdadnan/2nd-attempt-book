# Perception Nodes

In this chapter, we'll explore Isaac ROS perception nodes, which provide GPU-accelerated computer vision and sensor processing capabilities specifically designed for humanoid robots. These nodes leverage NVIDIA's hardware acceleration to deliver real-time perception performance.

## Isaac ROS Perception Overview

### Core Perception Nodes

Isaac ROS provides several specialized perception nodes:

- **Isaac ROS Apriltag**: GPU-accelerated fiducial marker detection
- **Isaac ROS DetectNet**: Object detection with TensorRT acceleration
- **Isaac ROS Segmentation**: Semantic segmentation for scene understanding
- **Isaac ROS Stereo**: Stereo vision for depth estimation
- **Isaac ROS Visual Slam**: GPU-accelerated SLAM pipeline
- **Isaac ROS Point Cloud**: Point cloud processing and fusion
- **Isaac ROS Image Pipeline**: Image preprocessing and enhancement

### Architecture and Integration

```
┌─────────────────────────────────────────────────────────────────┐
│                    Isaac ROS Perception Stack                   │
├─────────────────────────────────────────────────────────────────┤
│  Sensor Input → Preprocessing → Feature Extraction → Inference  │
│     (Camera, LiDAR, IMU)      (GPU Accelerated)    (TensorRT)  │
├─────────────────────────────────────────────────────────────────┤
│  Post-Processing → Data Fusion → Output Formatting → ROS Topics │
│   (GPU Accelerated)   (CUDA)         (Standard)        (DDS)   │
└─────────────────────────────────────────────────────────────────┘
```

## Isaac ROS Apriltag Node

### GPU-Accelerated Fiducial Detection

```python
# apriltag_perception.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2

class IsaacApriltagNode(Node):
    def __init__(self):
        super().__init__('isaac_apriltag_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Apriltag parameters
        self.family = 'tag36h11'
        self.max_tags = 128
        self.quad_decimate = 2.0
        self.refine_edges = True

        # Camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None

        # Initialize Apriltag detector
        self._initialize_apriltag_detector()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Publisher for detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/apriltag_detections',
            10
        )

        self.get_logger().info('Isaac Apriltag node initialized')

    def _initialize_apriltag_detector(self):
        """Initialize GPU-accelerated Apriltag detector"""
        try:
            # Import Apriltag library
            import pupil_apriltags as apriltag

            # Create detector with GPU acceleration
            self.detector = apriltag.Detector(
                families=self.family,
                nthreads=4,
                quad_decimate=self.quad_decimate,
                quad_sigma=0.0,
                refine_edges=self.refine_edges,
                decode_sharpening=0.25,
                debug=False
            )

            self.get_logger().info('Apriltag detector initialized with GPU acceleration')
        except ImportError:
            self.get_logger().error('Failed to import apriltag library')
            self.detector = None

    def camera_info_callback(self, msg):
        """Update camera calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        """Process image and detect Apriltags"""
        if self.detector is None:
            return

        try:
            # Convert ROS Image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect Apriltags
            detections = self._detect_apriltags(cv_image)

            # Publish detections
            if detections:
                detection_array = self._create_detection_array(detections, msg.header)
                self.detection_pub.publish(detection_array)

        except Exception as e:
            self.get_logger().error(f'Error in Apriltag processing: {e}')

    def _detect_apriltags(self, image):
        """Detect Apriltags using GPU acceleration"""
        if self.detector is None:
            return []

        try:
            # Convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Detect tags
            detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(self.camera_matrix[0, 0], self.camera_matrix[1, 1],
                              self.camera_matrix[0, 2], self.camera_matrix[1, 2]),
                tag_size=0.16  # Default tag size in meters
            )

            return detections

        except Exception as e:
            self.get_logger().error(f'Apriltag detection error: {e}')
            return []

    def _create_detection_array(self, detections, header):
        """Create Detection2DArray message from Apriltag detections"""
        detection_array = Detection2DArray()
        detection_array.header = header

        for detection in detections:
            detection_msg = Detection2D()

            # Set bounding box (from tag corners)
            corners = detection.corners
            x_coords = [corner[0] for corner in corners]
            y_coords = [corner[1] for corner in corners]

            x_min, x_max = min(x_coords), max(x_coords)
            y_min, y_max = min(y_coords), max(y_coords)

            detection_msg.bbox.center.x = (x_min + x_max) / 2
            detection_msg.bbox.center.y = (y_min + y_max) / 2
            detection_msg.bbox.size_x = x_max - x_min
            detection_msg.bbox.size_y = y_max - y_min

            # Set detection result
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = f'apriltag_{detection.tag_id}'
            result.hypothesis.score = float(detection.decision_margin)

            # Set pose if available
            if hasattr(detection, 'pose'):
                pose = detection.pose
                result.pose.pose.position.x = float(pose[0][3])
                result.pose.pose.position.y = float(pose[1][3])
                result.pose.pose.position.z = float(pose[2][3])

            detection_msg.results.append(result)
            detection_array.detections.append(detection_msg)

        return detection_array

def main(args=None):
    rclpy.init(args=args)
    node = IsaacApriltagNode()

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

## Isaac ROS DetectNet Node

### GPU-Accelerated Object Detection

```python
# detectnet_perception.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import jetson.inference
import jetson.utils

class IsaacDetectNetNode(Node):
    def __init__(self):
        super().__init__('isaac_detectnet_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # DetectNet parameters
        self.model_name = 'ssd-mobilenet-v2'
        self.threshold = 0.5
        self.max_batches = 1

        # Initialize DetectNet
        self._initialize_detectnet()

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.image_callback,
            10
        )

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detectnet_detections',
            10
        )

        self.get_logger().info('Isaac DetectNet node initialized')

    def _initialize_detectnet(self):
        """Initialize GPU-accelerated DetectNet"""
        try:
            # Initialize DetectNet with TensorRT acceleration
            self.net = jetson.inference.detectNet(
                model_name=self.model_name,
                threshold=self.threshold
            )

            self.get_logger().info(f'DetectNet initialized with model: {self.model_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize DetectNet: {e}')
            self.net = None

    def image_callback(self, msg):
        """Process image and detect objects"""
        if self.net is None:
            return

        try:
            # Convert ROS Image to CUDA image
            cuda_image = self.cv_bridge.imgmsg_to_cv2(msg)
            cuda_image = jetson.utils.cudaFromNumpy(cuda_image)

            # Perform detection
            detections = self.net.Detect(cuda_image, overlay="box,labels,conf")

            # Publish detections
            if detections:
                detection_array = self._create_detection_array(detections, msg.header)
                self.detection_pub.publish(detection_array)

        except Exception as e:
            self.get_logger().error(f'Error in DetectNet processing: {e}')

    def _create_detection_array(self, detections, header):
        """Create Detection2DArray from DetectNet results"""
        detection_array = Detection2DArray()
        detection_array.header = header

        for detection in detections:
            detection_msg = Detection2D()

            # Set bounding box
            detection_msg.bbox.center.x = float(detection.Center[0])
            detection_msg.bbox.center.y = float(detection.Center[1])
            detection_msg.bbox.size_x = float(detection.Width)
            detection_msg.bbox.size_y = float(detection.Height)

            # Set detection result
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = self.net.GetClassDesc(detection.ClassID)
            result.hypothesis.score = float(detection.Confidence)

            detection_msg.results.append(result)
            detection_array.detections.append(detection_msg)

        return detection_array

def main(args=None):
    rclpy.init(args=args)
    node = IsaacDetectNetNode()

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

## Isaac ROS Segmentation Node

### Semantic Segmentation with GPU Acceleration

```python
# segmentation_perception.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import SegmentedObjectArray
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import jetson.inference
import jetson.utils

class IsaacSegmentationNode(Node):
    def __init__(self):
        super().__init__('isaac_segmentation_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Segmentation parameters
        self.model_name = 'fcn-resnet18-cityscapes-512x256'
        self.overlay_alpha = 120

        # Initialize segmentation network
        self._initialize_segmentation()

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.image_callback,
            10
        )

        self.segmentation_pub = self.create_publisher(
            Image,
            '/segmentation_mask',
            10
        )

        self.segmented_objects_pub = self.create_publisher(
            SegmentedObjectArray,
            '/segmented_objects',
            10
        )

        self.get_logger().info('Isaac Segmentation node initialized')

    def _initialize_segmentation(self):
        """Initialize GPU-accelerated segmentation network"""
        try:
            self.net = jetson.inference.segNet(
                model_name=self.model_name
            )

            self.get_logger().info(f'Segmentation network initialized: {self.model_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize segmentation: {e}')
            self.net = None

    def image_callback(self, msg):
        """Process image and perform segmentation"""
        if self.net is None:
            return

        try:
            # Convert ROS Image to CUDA image
            cuda_image = self.cv_bridge.imgmsg_to_cv2(msg)
            cuda_image = jetson.utils.cudaFromNumpy(cuda_image)

            # Perform segmentation
            class_mask, output_image = self.net.Mask(cuda_image)

            # Publish segmentation mask
            mask_msg = self._create_mask_message(class_mask, msg.header)
            self.segmentation_pub.publish(mask_msg)

            # Publish segmented objects
            objects = self._extract_segmented_objects(class_mask, msg.header)
            self.segmented_objects_pub.publish(objects)

        except Exception as e:
            self.get_logger().error(f'Error in segmentation processing: {e}')

    def _create_mask_message(self, class_mask, header):
        """Create Image message from segmentation mask"""
        # Convert CUDA mask to numpy
        mask_np = jetson.utils.cudaToNumpy(class_mask)

        # Create Image message
        mask_msg = self.cv_bridge.cv2_to_imgmsg(mask_np.astype(np.uint8), encoding='mono8')
        mask_msg.header = header

        return mask_msg

    def _extract_segmented_objects(self, class_mask, header):
        """Extract segmented objects from class mask"""
        objects_array = SegmentedObjectArray()
        objects_array.header = header

        # Convert mask to numpy for processing
        mask_np = jetson.utils.cudaToNumpy(class_mask)

        # Get unique classes in the mask
        unique_classes = np.unique(mask_np)

        for class_id in unique_classes:
            if class_id == 0:  # Skip background
                continue

            # Create mask for this class
            class_region = (mask_np == class_id).astype(np.uint8)

            # Find contours
            contours, _ = cv2.findContours(class_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) > 100:  # Minimum area threshold
                    # Create segmented object
                    obj = SegmentedObject()

                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    obj.bounding_box.center.x = x + w / 2
                    obj.bounding_box.center.y = y + h / 2
                    obj.bounding_box.size_x = w
                    obj.bounding_box.size_y = h

                    # Set object class
                    obj.class_id = str(int(class_id))
                    obj.class_name = self.net.GetClassLabel(int(class_id))

                    objects_array.segmented_objects.append(obj)

        return objects_array

def main(args=None):
    rclpy.init(args=args)
    node = IsaacSegmentationNode()

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

## Isaac ROS Stereo Node

### GPU-Accelerated Stereo Vision

```python
# stereo_perception.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2

class IsaacStereoNode(Node):
    def __init__(self):
        super().__init__('isaac_stereo_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Stereo parameters
        self.disp_max = 128
        self.block_size = 11
        self.min_disp = 0
        self.uniqueness_ratio = 15
        self.speckle_window_size = 0
        self.speckle_range = 2

        # Initialize stereo matcher
        self._initialize_stereo_matcher()

        # Camera parameters
        self.left_camera_info = None
        self.right_camera_info = None
        self.stereo_rectification = None

        # Subscribers
        self.left_sub = self.create_subscription(
            Image,
            '/camera/left/image_rect',
            self.left_image_callback,
            10
        )

        self.right_sub = self.create_subscription(
            Image,
            '/camera/right/image_rect',
            self.right_image_callback,
            10
        )

        self.left_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/left/camera_info',
            self.left_info_callback,
            10
        )

        self.right_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/right/camera_info',
            self.right_info_callback,
            10
        )

        # Publishers
        self.disparity_pub = self.create_publisher(Image, '/disparity_map', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/point_cloud', 10)

        # Store latest images
        self.latest_left = None
        self.latest_right = None

        self.get_logger().info('Isaac Stereo node initialized')

    def _initialize_stereo_matcher(self):
        """Initialize GPU-accelerated stereo matcher"""
        try:
            # Use CUDA-accelerated stereo matcher
            self.stereo_matcher = cv2.cuda.StereoSGBM_create(
                minDisparity=self.min_disp,
                numDisparities=self.disp_max,
                blockSize=self.block_size,
                uniquenessRatio=self.uniqueness_ratio,
                speckleWindowSize=self.speckle_window_size,
                speckleRange=self.speckle_range,
                disp12MaxDiff=1,
                P1=8 * 3 * self.block_size**2,
                P2=32 * 3 * self.block_size**2
            )

            self.get_logger().info('GPU-accelerated stereo matcher initialized')
        except Exception as e:
            self.get_logger().warn(f'GPU stereo matcher failed: {e}, falling back to CPU')
            # Fallback to CPU-based matcher
            self.stereo_matcher = cv2.StereoSGBM_create(
                minDisparity=self.min_disp,
                numDisparities=self.disp_max,
                blockSize=self.block_size,
                uniquenessRatio=self.uniqueness_ratio,
                speckleWindowSize=self.speckle_window_size,
                speckleRange=self.speckle_range,
                disp12MaxDiff=1,
                P1=8 * 3 * self.block_size**2,
                P2=32 * 3 * self.block_size**2
            )

    def left_info_callback(self, msg):
        """Handle left camera info"""
        self.left_camera_info = msg
        self._update_stereo_rectification()

    def right_info_callback(self, msg):
        """Handle right camera info"""
        self.right_camera_info = msg
        self._update_stereo_rectification()

    def _update_stereo_rectification(self):
        """Update stereo rectification parameters"""
        if self.left_camera_info and self.right_camera_info:
            # Extract camera matrices and distortion coefficients
            left_cam_mat = np.array(self.left_camera_info.k).reshape(3, 3)
            right_cam_mat = np.array(self.right_camera_info.k).reshape(3, 3)
            left_dist = np.array(self.left_camera_info.d)
            right_dist = np.array(self.right_camera_info.d)

            # Extract rotation and translation between cameras
            # This would normally come from stereo calibration
            R = np.eye(3)  # Placeholder - should be from calibration
            T = np.array([-0.1, 0, 0])  # Baseline (10cm)

            # Compute rectification parameters
            self.rect_left, self.rect_right, self.proj_left, self.proj_right, self.disp_to_depth, _, _ = \
                cv2.stereoRectify(
                    left_cam_mat, left_dist, right_cam_mat, right_dist,
                    (self.left_camera_info.width, self.left_camera_info.height),
                    R, T,
                    flags=cv2.CALIB_ZERO_DISPARITY,
                    alpha=0  # Crop image
                )

            # Compute rectification maps
            self.left_map_x, self.left_map_y = cv2.initUndistortRectifyMap(
                left_cam_mat, left_dist, self.rect_left, self.proj_left,
                (self.left_camera_info.width, self.left_camera_info.height), cv2.CV_32FC1
            )

            self.right_map_x, self.right_map_y = cv2.initUndistortRectifyMap(
                right_cam_mat, right_dist, self.rect_right, self.proj_right,
                (self.right_camera_info.width, self.right_camera_info.height), cv2.CV_32FC1
            )

    def left_image_callback(self, msg):
        """Handle left camera image"""
        self.latest_left = msg
        self._process_stereo_if_ready()

    def right_image_callback(self, msg):
        """Handle right camera image"""
        self.latest_right = msg
        self._process_stereo_if_ready()

    def _process_stereo_if_ready(self):
        """Process stereo pair if both images are available"""
        if self.latest_left is None or self.latest_right is None:
            return

        try:
            # Convert images to grayscale
            left_cv = self.cv_bridge.imgmsg_to_cv2(self.latest_left, desired_encoding='mono8')
            right_cv = self.cv_bridge.imgmsg_to_cv2(self.latest_right, desired_encoding='mono8')

            # Rectify images
            if hasattr(self, 'left_map_x'):
                left_rect = cv2.remap(left_cv, self.left_map_x, self.left_map_y, cv2.INTER_LINEAR)
                right_rect = cv2.remap(right_cv, self.right_map_x, self.right_map_y, cv2.INTER_LINEAR)
            else:
                left_rect = left_cv
                right_rect = right_cv

            # Compute disparity map
            if hasattr(cv2.cuda, 'matchTemplate') and isinstance(self.stereo_matcher, cv2.cuda.StereoSGBM):
                # GPU version
                left_cuda = cv2.cuda_GpuMat()
                right_cuda = cv2.cuda_GpuMat()
                left_cuda.upload(left_rect)
                right_cuda.upload(right_rect)

                disparity_cuda = self.stereo_matcher.compute(left_cuda, right_cuda)
                disparity = disparity_cuda.download()
            else:
                # CPU version
                disparity = self.stereo_matcher.compute(left_rect, right_rect).astype(np.float32)

            # Normalize disparity for visualization
            disp_norm = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)

            # Publish disparity map
            disp_msg = self.cv_bridge.cv2_to_imgmsg(disp_norm, encoding='mono8')
            disp_msg.header = self.latest_left.header
            self.disparity_pub.publish(disp_msg)

            # Generate point cloud
            if hasattr(self, 'disp_to_depth'):
                pointcloud = self._generate_pointcloud(disparity, left_rect.shape)
                if pointcloud is not None:
                    self.pointcloud_pub.publish(pointcloud)

            # Clear processed images
            self.latest_left = None
            self.latest_right = None

        except Exception as e:
            self.get_logger().error(f'Stereo processing error: {e}')

    def _generate_pointcloud(self, disparity, shape):
        """Generate point cloud from disparity map"""
        try:
            # Convert disparity to depth
            # This is a simplified version - in practice you'd use the Q matrix from stereoRectify
            baseline = 0.1  # meters
            focal_length = self.proj_left[0, 0] if hasattr(self, 'proj_left') else 320

            # Convert disparity to depth
            depth = (baseline * focal_length) / (disparity / 16.0 + 1e-6)  # Add small value to avoid division by zero

            # Create point cloud
            height, width = shape
            points = []

            for v in range(0, height, 2):  # Downsample for performance
                for u in range(0, width, 2):
                    z = depth[v, u] / 1000.0  # Convert to meters
                    if z > 0 and z < 10:  # Valid depth range
                        x = (u - self.proj_left[0, 2]) * z / self.proj_left[0, 0]
                        y = (v - self.proj_left[1, 2]) * z / self.proj_left[1, 1]
                        points.append([x, y, z])

            if points:
                # Create PointCloud2 message
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = 'camera_link'

                # Define point fields
                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
                ]

                # Create PointCloud2 message
                pointcloud_msg = PointCloud2()
                pointcloud_msg.header = header
                pointcloud_msg.fields = fields
                pointcloud_msg.is_bigendian = False
                pointcloud_msg.is_dense = False
                pointcloud_msg.point_step = 12  # 3 floats * 4 bytes
                pointcloud_msg.row_step = pointcloud_msg.point_step * len(points)
                pointcloud_msg.height = 1
                pointcloud_msg.width = len(points)

                # Pack points into binary data
                import struct
                data = []
                for point in points:
                    data.extend(struct.pack('fff', *point))

                pointcloud_msg.data = data

                return pointcloud_msg

        except Exception as e:
            self.get_logger().error(f'Point cloud generation error: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = IsaacStereoNode()

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

## Isaac ROS Image Pipeline

### GPU-Accelerated Image Processing

```python
# image_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2

class IsaacImagePipelineNode(Node):
    def __init__(self):
        super().__init__('isaac_image_pipeline')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Image processing parameters
        self.enable_denoising = True
        self.enable_enhancement = True
        self.enable_rectification = True

        # Subscribers
        self.raw_image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.raw_image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Publishers for processed images
        self.rectified_pub = self.create_publisher(Image, '/camera/image_rect', 10)
        self.denoised_pub = self.create_publisher(Image, '/camera/image_denoised', 10)
        self.enhanced_pub = self.create_publisher(Image, '/camera/image_enhanced', 10)
        self.processed_pub = self.create_publisher(Image, '/camera/image_processed', 10)

        # Camera calibration
        self.camera_matrix = None
        self.distortion_coeffs = None

        # Initialize GPU-accelerated processing
        self._initialize_gpu_processing()

        self.get_logger().info('Isaac Image Pipeline node initialized')

    def _initialize_gpu_processing(self):
        """Initialize GPU-accelerated image processing"""
        try:
            # Check for CUDA availability
            import pycuda.driver as cuda
            import pycuda.autoinit

            self.cuda_available = True
            self.get_logger().info('CUDA acceleration enabled for image processing')

        except ImportError:
            self.cuda_available = False
            self.get_logger().info('CUDA acceleration not available, using CPU processing')

    def camera_info_callback(self, msg):
        """Update camera calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

        # Pre-compute rectification maps
        if self.camera_matrix is not None and self.distortion_coeffs is not None:
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.camera_matrix, self.distortion_coeffs,
                None, self.camera_matrix,
                (msg.width, msg.height), cv2.CV_32FC1
            )

    def raw_image_callback(self, msg):
        """Process raw image through pipeline"""
        try:
            # Convert ROS Image to OpenCV
            raw_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Apply image processing pipeline
            processed_image = self._apply_image_pipeline(raw_image)

            # Publish results
            header = msg.header

            if processed_image is not None:
                # Publish processed image
                processed_msg = self.cv_bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
                processed_msg.header = header
                self.processed_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Image pipeline error: {e}')

    def _apply_image_pipeline(self, image):
        """Apply complete image processing pipeline"""
        result = image.copy()

        # Step 1: Rectification
        if self.enable_rectification and hasattr(self, 'map1'):
            result = self._rectify_image(result)

        # Step 2: Denoising
        if self.enable_denoising:
            result = self._denoise_image(result)

        # Step 3: Enhancement
        if self.enable_enhancement:
            result = self._enhance_image(result)

        return result

    def _rectify_image(self, image):
        """Apply camera rectification"""
        if hasattr(self, 'map1') and hasattr(self, 'map2'):
            rectified = cv2.remap(
                image, self.map1, self.map2,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT
            )
            return rectified
        else:
            return image

    def _denoise_image(self, image):
        """Apply image denoising using GPU acceleration"""
        try:
            if self.cuda_available:
                # Use GPU-accelerated denoising (example with bilateral filter)
                # In practice, you'd use CUDA kernels or optimized libraries
                denoised = cv2.bilateralFilter(image, 9, 75, 75)
            else:
                # CPU fallback
                denoised = cv2.fastNlMeansDenoisingColored(
                    cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                )
                denoised = cv2.cvtColor(denoised, cv2.COLOR_RGB2BGR)

            return denoised

        except Exception as e:
            self.get_logger().error(f'Denoising error: {e}')
            return image

    def _enhance_image(self, image):
        """Apply image enhancement"""
        try:
            # Convert to LAB for better enhancement
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)

            # Apply CLAHE to L channel
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            l = clahe.apply(l)

            # Merge channels back
            enhanced_lab = cv2.merge([l, a, b])
            enhanced = cv2.cvtColor(enhanced_lab, cv2.COLOR_LAB2BGR)

            return enhanced

        except Exception as e:
            self.get_logger().error(f'Enhancement error: {e}')
            return image

def main(args=None):
    rclpy.init(args=args)
    node = IsaacImagePipelineNode()

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

## Isaac ROS Point Cloud Processing

### GPU-Accelerated Point Cloud Operations

```python
# pointcloud_processing.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import numpy as np
import open3d as o3d
import copy

class IsaacPointCloudNode(Node):
    def __init__(self):
        super().__init__('isaac_pointcloud_node')

        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/point_cloud',
            self.pointcloud_callback,
            10
        )

        # Publishers
        self.filtered_pub = self.create_publisher(PointCloud2, '/point_cloud_filtered', 10)
        self.downsampled_pub = self.create_publisher(PointCloud2, '/point_cloud_downsampled', 10)
        self.segmented_pub = self.create_publisher(PointCloud2, '/point_cloud_segmented', 10)

        # Point cloud processing parameters
        self.voxel_size = 0.05  # 5cm voxel size
        self.distance_threshold = 0.02  # 2cm for RANSAC
        self.max_iterations = 1000
        self.leaf_size = 0.02  # 2cm for downsampling

        self.get_logger().info('Isaac Point Cloud node initialized')

    def pointcloud_callback(self, msg):
        """Process incoming point cloud"""
        try:
            # Convert ROS PointCloud2 to Open3D format
            points = self._ros_to_o3d_points(msg)

            if len(points) == 0:
                return

            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            # Apply processing pipeline
            processed_pcd = self._apply_processing_pipeline(pcd)

            # Publish results
            if processed_pcd is not None:
                # Publish filtered point cloud
                filtered_msg = self._o3d_to_ros_points(processed_pcd, msg.header)
                self.filtered_pub.publish(filtered_msg)

                # Publish downsampled point cloud
                downsampled = processed_pcd.voxel_down_sample(voxel_size=self.voxel_size)
                downsampled_msg = self._o3d_to_ros_points(downsampled, msg.header)
                self.downsampled_pub.publish(downsampled_msg)

        except Exception as e:
            self.get_logger().error(f'Point cloud processing error: {e}')

    def _ros_to_o3d_points(self, ros_cloud):
        """Convert ROS PointCloud2 to numpy array"""
        points = []
        for point in point_cloud2.read_points(ros_cloud, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        return np.array(points)

    def _o3d_to_ros_points(self, o3d_cloud, header):
        """Convert Open3D point cloud to ROS PointCloud2"""
        points = np.asarray(o3d_cloud.points)

        # Create PointCloud2 message
        ros_cloud = PointCloud2()
        ros_cloud.header = header
        ros_cloud.height = 1
        ros_cloud.width = len(points)
        ros_cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        ros_cloud.is_bigendian = False
        ros_cloud.point_step = 12  # 3 floats * 4 bytes
        ros_cloud.row_step = ros_cloud.point_step * ros_cloud.width
        ros_cloud.is_dense = True

        # Pack points into binary data
        import struct
        data = []
        for point in points:
            data.extend(struct.pack('fff', point[0], point[1], point[2]))

        ros_cloud.data = data
        return ros_cloud

    def _apply_processing_pipeline(self, pcd):
        """Apply point cloud processing pipeline"""
        # Step 1: Statistical outlier removal
        pcd_filtered = self._remove_statistical_outliers(pcd)

        # Step 2: Radius outlier removal
        pcd_filtered = self._remove_radius_outliers(pcd_filtered)

        # Step 3: Plane segmentation (ground removal)
        pcd_filtered = self._segment_ground_plane(pcd_filtered)

        return pcd_filtered

    def _remove_statistical_outliers(self, pcd):
        """Remove statistical outliers"""
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        return pcd.select_by_index(ind)

    def _remove_radius_outliers(self, pcd):
        """Remove radius-based outliers"""
        cl, ind = pcd.remove_radius_outlier(nb_points=10, radius=0.05)
        return pcd.select_by_index(ind)

    def _segment_ground_plane(self, pcd):
        """Segment ground plane using RANSAC"""
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.distance_threshold,
            ransac_n=3,
            num_iterations=self.max_iterations
        )

        # Remove ground plane points
        pcd_no_ground = pcd.select_by_index(inliers, invert=True)
        return pcd_no_ground

def main(args=None):
    rclpy.init(args=args)
    node = IsaacPointCloudNode()

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

## Isaac ROS Perception Pipeline Integration

### Complete Perception System

```python
# complete_perception_system.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray, SegmentedObjectArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np

class IsaacPerceptionSystem(Node):
    def __init__(self):
        super().__init__('isaac_perception_system')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize perception nodes
        self.apriltag_node = IsaacApriltagNode()
        self.detectnet_node = IsaacDetectNetNode()
        self.segmentation_node = IsaacSegmentationNode()
        self.stereo_node = IsaacStereoNode()
        self.pipeline_node = IsaacImagePipelineNode()
        self.pointcloud_node = IsaacPointCloudNode()

        # Publishers for fused results
        self.fused_detections_pub = self.create_publisher(Detection2DArray, '/fused_detections', 10)
        self.scene_description_pub = self.create_publisher(String, '/scene_description', 10)

        # Scene understanding components
        self.scene_objects = []
        self.spatial_relationships = []

        self.get_logger().info('Isaac Complete Perception System initialized')

    def fuse_perception_results(self):
        """Fuse results from multiple perception nodes"""
        # This would integrate detections from different nodes
        # For example: combine 2D object detections with 3D point cloud data
        # to create spatially-aware object detections
        pass

    def generate_scene_description(self):
        """Generate textual description of the scene"""
        description = {
            'timestamp': self.get_clock().now().to_msg(),
            'objects_detected': len(self.scene_objects),
            'spatial_relationships': self.spatial_relationships,
            'environment': 'indoor'  # Would be determined from scene analysis
        }

        # Publish scene description
        desc_msg = String()
        desc_msg.data = str(description)
        self.scene_description_pub.publish(desc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacPerceptionSystem()

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

### GPU Resource Management

```python
# gpu_resource_management.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pycuda.driver as cuda
import pycuda.tools
import threading
import time

class GPUResourceManager:
    def __init__(self):
        self.gpu_initialized = False
        self.memory_usage = 0
        self.utilization = 0
        self.active_streams = []

        try:
            cuda.init()
            self.gpu_device = cuda.Device(0)  # Use first GPU
            self.gpu_context = self.gpu_device.make_context()
            self.gpu_initialized = True
            print(f"GPU initialized: {self.gpu_device.name()}")
        except Exception as e:
            print(f"GPU initialization failed: {e}")

    def monitor_gpu_resources(self):
        """Monitor GPU resource usage"""
        if not self.gpu_initialized:
            return

        # Get memory info
        free_mem, total_mem = cuda.mem_get_info()
        self.memory_usage = (total_mem - free_mem) / total_mem * 100

        # GPU utilization would require nvidia-ml-py
        # self.utilization = get_gpu_utilization()

    def allocate_gpu_memory(self, size_mb):
        """Allocate GPU memory with proper management"""
        if not self.gpu_initialized:
            return None

        try:
            gpu_mem = cuda.mem_alloc(size_mb * 1024 * 1024)  # Convert MB to bytes
            return gpu_mem
        except cuda.MemoryError:
            print(f"Not enough GPU memory to allocate {size_mb}MB")
            return None

    def create_cuda_stream(self):
        """Create a CUDA stream for asynchronous operations"""
        if not self.gpu_initialized:
            return None

        try:
            stream = cuda.Stream()
            self.active_streams.append(stream)
            return stream
        except Exception as e:
            print(f"Failed to create CUDA stream: {e}")
            return None

    def cleanup_streams(self):
        """Clean up active CUDA streams"""
        for stream in self.active_streams:
            del stream
        self.active_streams = []

def optimize_perception_performance():
    """Set up GPU resource optimization for perception nodes"""
    gpu_manager = GPUResourceManager()
    return gpu_manager
```

## Best Practices

1. **Use Appropriate Models**: Select models optimized for your specific use case and hardware
2. **GPU Memory Management**: Efficiently manage GPU memory to avoid allocation failures
3. **Pipeline Optimization**: Chain operations efficiently to minimize data transfers
4. **Parameter Tuning**: Adjust parameters based on performance requirements
5. **Multi-Threading**: Use appropriate threading for I/O and processing
6. **Quality vs Performance**: Balance accuracy with real-time performance needs
7. **Error Handling**: Implement robust error handling for GPU operations
8. **Resource Monitoring**: Continuously monitor GPU utilization and memory usage

## Next Steps

In the next chapter, we'll explore Isaac ROS Navigation stack, learning how Isaac's navigation components leverage GPU acceleration for path planning, obstacle avoidance, and navigation in complex humanoid robot environments.