# Sensor Integration with ROS

In this chapter, we'll explore how to integrate simulated sensors from Gazebo with ROS 2, creating the communication bridge that allows your humanoid robot's perception and control systems to access sensor data from simulation. This integration is essential for developing and testing robot applications in a safe, controlled environment.

## ROS Sensor Message Types

### Standard Sensor Messages

ROS 2 provides standardized message types for common sensors:

- `sensor_msgs/msg/Imu`: Inertial measurement unit data
- `sensor_msgs/msg/Image`: Camera image data
- `sensor_msgs/msg/LaserScan`: 2D LiDAR data
- `sensor_msgs/msg/PointCloud2`: 3D point cloud data
- `sensor_msgs/msg/JointState`: Robot joint positions, velocities, efforts
- `geometry_msgs/msg/WrenchStamped`: Force/torque sensor data

### IMU Integration

Here's how to work with IMU data in ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        # Subscribe to IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publisher for processed data
        self.orientation_pub = self.create_publisher(
            Vector3,
            '/robot_orientation',
            10
        )

        self.get_logger().info('IMU processor initialized')

    def imu_callback(self, msg):
        # Extract orientation from quaternion
        orientation = self.quaternion_to_euler(msg.orientation)

        # Extract angular velocity and linear acceleration
        angular_vel = msg.angular_velocity
        linear_acc = msg.linear_acceleration

        # Publish processed orientation
        orientation_msg = Vector3()
        orientation_msg.x = orientation[0]  # Roll
        orientation_msg.y = orientation[1]  # Pitch
        orientation_msg.z = orientation[2]  # Yaw

        self.orientation_pub.publish(orientation_msg)

        # Log important data
        self.get_logger().info(
            f'Orientation - Roll: {orientation[0]:.3f}, '
            f'Pitch: {orientation[1]:.3f}, '
            f'Yaw: {orientation[2]:.3f}'
        )

    def quaternion_to_euler(self, quaternion):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        import math

        q = quaternion
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)

def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessor()

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

### Camera Integration

Working with camera data in ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')

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

        # Publisher for processed images
        self.processed_pub = self.create_publisher(
            Image,
            '/camera/processed',
            10
        )

        self.camera_info = None
        self.get_logger().info('Camera processor initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process the image (example: edge detection)
            processed_image = self.process_image(cv_image)

            # Convert back to ROS Image
            processed_msg = self.cv_bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            processed_msg.header = msg.header  # Preserve timestamp and frame

            # Publish processed image
            self.processed_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.get_logger().info('Camera info received')

    def process_image(self, image):
        """Example image processing function"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Canny edge detection
        edges = cv2.Canny(gray, 50, 150)

        # Convert back to BGR for output
        result = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        return result

def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()

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

### LiDAR Integration

Processing LiDAR data in ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publishers for processed data
        self.obstacle_pub = self.create_publisher(
            Float32MultiArray,
            '/obstacle_distances',
            10
        )

        self.get_logger().info('LiDAR processor initialized')

    def scan_callback(self, msg):
        # Convert scan data to numpy array
        ranges = np.array(msg.ranges)

        # Filter out invalid ranges (inf, nan)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            # Find minimum distance (potential obstacle)
            min_distance = np.min(valid_ranges)
            min_angle_idx = np.argmin(ranges)
            min_angle = msg.angle_min + min_angle_idx * msg.angle_increment

            # Check for obstacles in front (e.g., within 45 degrees of center)
            center_idx = len(ranges) // 2
            front_range = ranges[max(0, center_idx-20):center_idx+20]
            front_valid = front_range[np.isfinite(front_range)]

            if len(front_valid) > 0:
                front_distance = np.min(front_valid)

                # Publish obstacle information
                obstacle_msg = Float32MultiArray()
                obstacle_msg.data = [min_distance, min_angle, front_distance]
                self.obstacle_pub.publish(obstacle_msg)

                self.get_logger().info(
                    f'Closest obstacle: {min_distance:.2f}m at {min_angle:.2f}rad, '
                    f'Front distance: {front_distance:.2f}m'
                )

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()

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

## Multi-Sensor Fusion

### Sensor Data Synchronization

Synchronize data from multiple sensors:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import threading

class MultiSensorFusion(Node):
    def __init__(self):
        super().__init__('multi_sensor_fusion')

        # Initialize data storage with threading protection
        self.lock = threading.Lock()
        self.imu_data = None
        self.scan_data = None
        self.image_data = None

        # Create subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publisher for fused data
        self.fused_pub = self.create_publisher(
            String, '/sensor_fusion_result', 10)

        self.cv_bridge = CvBridge()
        self.get_logger().info('Multi-sensor fusion node initialized')

    def imu_callback(self, msg):
        with self.lock:
            self.imu_data = msg

    def scan_callback(self, msg):
        with self.lock:
            self.scan_data = msg

    def image_callback(self, msg):
        with self.lock:
            self.image_data = msg

    def get_fused_data(self):
        """Get synchronized sensor data"""
        with self.lock:
            return self.imu_data, self.scan_data, self.image_data

def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorFusion()

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

### Kalman Filter Integration

Implement sensor fusion with a Kalman filter:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import PoseStamped
import numpy as np

class KalmanFilterFusion(Node):
    def __init__(self):
        super().__init__('kalman_filter_fusion')

        # Subscribe to sensors
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher for fused pose estimate
        self.pose_pub = self.create_publisher(PoseStamped, '/fused_pose', 10)

        # Initialize Kalman filter
        self.initialize_kalman_filter()

        self.get_logger().info('Kalman filter fusion initialized')

    def initialize_kalman_filter(self):
        """Initialize Kalman filter parameters"""
        # State vector: [x, y, vx, vy, ax, ay]
        self.state_dim = 6
        self.obs_dim = 4  # [x, y, vx, vy] from different sensors

        # Initial state and covariance
        self.x = np.zeros((self.state_dim, 1))  # State vector
        self.P = np.eye(self.state_dim) * 1000  # Error covariance

        # Process noise
        self.Q = np.eye(self.state_dim) * 0.1

        # Measurement noise (different for IMU and LiDAR)
        self.R_imu = np.eye(2) * 0.01   # Low noise for IMU acceleration
        self.R_scan = np.eye(2) * 0.1   # Higher noise for LiDAR position

    def imu_callback(self, msg):
        """Process IMU measurement"""
        # Extract linear acceleration
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y

        # Prediction step (constant acceleration model)
        dt = 0.01  # Assuming 100Hz IMU
        self.predict(dt)

        # Update with IMU acceleration measurement
        z = np.array([[acc_x], [acc_y]])  # Measurement
        H = np.array([[0, 0, 0, 0, 1, 0],  # Mapping from state to measurement
                      [0, 0, 0, 0, 0, 1]])

        self.update(z, H, self.R_imu)

    def scan_callback(self, msg):
        """Process LiDAR measurement for position"""
        # Simplified: use closest obstacle range as position indicator
        # In practice, you'd use landmarks or SLAM
        if len(msg.ranges) > 0:
            valid_ranges = [r for r in msg.ranges if r != float('inf')]
            if valid_ranges:
                min_range = min(valid_ranges)
                # Simplified: assume obstacle at known location for demo
                # In practice, you'd have a more sophisticated position estimate
                pass

    def predict(self, dt):
        """Prediction step of Kalman filter"""
        # State transition matrix (constant acceleration model)
        F = np.array([
            [1, 0, dt, 0, 0.5*dt**2, 0],
            [0, 1, 0, dt, 0, 0.5*dt**2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        # Predict state and covariance
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z, H, R):
        """Update step of Kalman filter"""
        # Innovation
        y = z - H @ self.x
        # Innovation covariance
        S = H @ self.P @ H.T + R
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        # Update state and covariance
        self.x = self.x + K @ y
        self.P = (np.eye(self.state_dim) - K @ H) @ self.P

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterFusion()

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

## Sensor Data Processing Pipelines

### Perception Pipeline

Create a complete perception pipeline:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionPipeline(Node):
    def __init__(self):
        super().__init__('perception_pipeline')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Subscribe to sensors
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Publishers for processed data
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/object_detections', 10)
        self.obstacle_pub = self.create_publisher(
            PointStamped, '/obstacle_location', 10)

        # Processing flags
        self.imu_orientation = None

    def image_callback(self, msg):
        """Process camera image for object detection"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection (simplified - in practice, use a trained model)
            detections = self.detect_objects(cv_image)

            # Publish detections
            detection_msg = self.create_detection_message(detections, msg.header)
            self.detection_pub.publish(detection_msg)

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def scan_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        # Find closest obstacle in front
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            min_idx = np.argmin(ranges)

            # Convert polar to Cartesian coordinates
            angle = msg.angle_min + min_idx * msg.angle_increment
            x = min_distance * np.cos(angle)
            y = min_distance * np.sin(angle)

            # Create obstacle location message
            obstacle_msg = PointStamped()
            obstacle_msg.header = msg.header
            obstacle_msg.point.x = x
            obstacle_msg.point.y = y
            obstacle_msg.point.z = 0.0  # Assume ground level

            self.obstacle_pub.publish(obstacle_msg)

    def imu_callback(self, msg):
        """Process IMU data for orientation"""
        # Convert quaternion to Euler angles for use in other nodes
        self.imu_orientation = self.quaternion_to_euler(msg.orientation)

    def detect_objects(self, image):
        """Simple object detection (in practice, use a trained model)"""
        # Convert to HSV for color-based detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color range for detection (example: red objects)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                x, y, w, h = cv2.boundingRect(contour)
                detections.append((x, y, w, h))

        return detections

    def create_detection_message(self, detections, header):
        """Create vision_msgs/Detection2DArray message"""
        detection_array = Detection2DArray()
        detection_array.header = header

        for det in detections:
            x, y, w, h = det
            detection = Detection2D()
            detection.bbox.center.x = x + w/2
            detection.bbox.center.y = y + h/2
            detection.bbox.size_x = w
            detection.bbox.size_y = h
            detection_array.detections.append(detection)

        return detection_array

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionPipeline()

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

## AI Integration with Sensor Data

### Deep Learning Sensor Processing

Integrate AI models with sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms

class AISensorProcessor(Node):
    def __init__(self):
        super().__init__('ai_sensor_processor')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Subscribe to sensors
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publisher for AI results
        self.ai_result_pub = self.create_publisher(
            String, '/ai_decision', 10)

        # Load AI model (example: classification model)
        try:
            # self.model = torch.load('path/to/model.pth')
            # self.model.eval()
            self.model = None  # Placeholder
            self.get_logger().info('AI model loaded')
        except Exception as e:
            self.get_logger().error(f'Failed to load AI model: {e}')
            self.model = None

        # Image preprocessing
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                               std=[0.229, 0.224, 0.225])
        ])

    def image_callback(self, msg):
        """Process image with AI model"""
        if self.model is None:
            return

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Preprocess image for AI model
            input_tensor = self.transform(cv_image)
            input_batch = input_tensor.unsqueeze(0)  # Add batch dimension

            # Run inference
            with torch.no_grad():
                output = self.model(input_batch)
                probabilities = torch.nn.functional.softmax(output[0], dim=0)

                # Get top prediction
                top_prob, top_catid = torch.topk(probabilities, 1)

                # Publish AI decision
                result_msg = String()
                result_msg.data = f"Detected class {top_catid.item()} with {top_prob.item():.3f} confidence"
                self.ai_result_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f'AI processing error: {e}')

    def scan_callback(self, msg):
        """Process LiDAR data with AI for navigation"""
        if self.model is None:
            return

        try:
            # Convert scan to tensor for AI processing
            ranges = torch.tensor(msg.ranges, dtype=torch.float32)

            # AI model for navigation decision
            # navigation_decision = self.model.process_lidar(ranges)
            # For now, publish a simple decision

            result_msg = String()
            result_msg.data = "AI navigation processing complete"
            self.ai_result_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f'LiDAR AI processing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AISensorProcessor()

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

## Sensor Calibration and Validation

### Online Calibration Node

Perform sensor calibration in simulation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Float32
import numpy as np

class SensorCalibrator(Node):
    def __init__(self):
        super().__init__('sensor_calibrator')

        # Subscribe to sensors
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers for calibration status
        self.calibration_status_pub = self.create_publisher(Float32, '/calibration_status', 10)

        # Calibration data storage
        self.imu_samples = []
        self.calibration_samples_needed = 100
        self.calibrated = False

    def imu_callback(self, msg):
        """Collect IMU data for calibration"""
        if not self.calibrated and len(self.imu_samples) < self.calibration_samples_needed:
            self.imu_samples.append({
                'linear_acc': [msg.linear_acceleration.x,
                              msg.linear_acceleration.y,
                              msg.linear_acceleration.z],
                'angular_vel': [msg.angular_velocity.x,
                               msg.angular_velocity.y,
                               msg.angular_velocity.z]
            })

            if len(self.imu_samples) == self.calibration_samples_needed:
                self.perform_imu_calibration()

    def perform_imu_calibration(self):
        """Perform IMU bias calibration"""
        # Calculate bias from stationary measurements
        linear_acc_samples = np.array([s['linear_acc'] for s in self.imu_samples])
        angular_vel_samples = np.array([s['angular_vel'] for s in self.imu_samples])

        # Calculate biases
        acc_bias = np.mean(linear_acc_samples, axis=0)
        gyro_bias = np.mean(angular_vel_samples, axis=0)

        # The gravity vector should be ~9.81 m/s^2 in z-direction when stationary
        # Adjust z-axis bias to account for gravity
        acc_bias[2] -= 9.81

        self.get_logger().info(f'IMU calibration complete - Acc bias: {acc_bias}, Gyro bias: {gyro_bias}')
        self.calibrated = True

        # Publish calibration status
        status_msg = Float32()
        status_msg.data = 1.0  # Calibrated
        self.calibration_status_pub.publish(status_msg)

    def image_callback(self, msg):
        """Process image for camera calibration (if needed)"""
        if not self.calibrated:
            # Camera calibration would happen here
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SensorCalibrator()

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

### Sensor Data Throttling

Optimize sensor data processing:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage

class OptimizedSensorProcessor(Node):
    def __init__(self):
        super().__init__('optimized_sensor_processor')

        # Use appropriate QoS for different sensor types
        image_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Subscribe with optimized settings
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 5)  # Lower queue size

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)  # Higher for critical data

        # Use compressed images for high-bandwidth sensors
        self.compressed_image_sub = self.create_subscription(
            CompressedImage, '/camera/image_compressed',
            self.compressed_image_callback, 1)

        # Throttle processing with timers
        self.processing_timer = self.create_timer(0.1, self.process_pending_data)  # 10 Hz processing
        self.pending_image = None
        self.pending_imu = None

    def image_callback(self, msg):
        """Store image for processing at controlled rate"""
        self.pending_image = msg

    def imu_callback(self, msg):
        """Store IMU data for processing"""
        self.pending_imu = msg

    def compressed_image_callback(self, msg):
        """Handle compressed image data"""
        pass

    def process_pending_data(self):
        """Process data at controlled rate"""
        if self.pending_image is not None:
            self.process_image_optimized(self.pending_image)
            self.pending_image = None

        if self.pending_imu is not None:
            self.process_imu_optimized(self.pending_imu)
            self.pending_imu = None

    def process_image_optimized(self, msg):
        """Optimized image processing"""
        # Skip processing if computation is too heavy
        pass

    def process_imu_optimized(self, msg):
        """Optimized IMU processing"""
        pass

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedSensorProcessor()

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

## Best Practices

1. **Use appropriate QoS settings**: Match reliability and durability to sensor requirements
2. **Implement data validation**: Check sensor data ranges and plausibility
3. **Optimize processing frequency**: Balance accuracy with computational constraints
4. **Handle sensor failures**: Implement fallback strategies for sensor outages
5. **Validate sensor fusion**: Ensure combined data is more accurate than individual sensors
6. **Consider real-time constraints**: Ensure sensor processing meets timing requirements
7. **Document sensor specifications**: Keep track of simulated vs. real sensor parameters

## Next Steps

In the next chapter, we'll explore creating complex Gazebo worlds with various environments, obstacles, and scenarios for testing your humanoid robot in diverse situations. This will allow you to validate your sensor integration and robot behaviors in realistic settings.