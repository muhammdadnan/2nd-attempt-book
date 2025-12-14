# Hardware Integration

In this chapter, we'll explore the critical process of integrating real hardware sensors and actuators with your AI-powered humanoid robot system. This integration bridges the gap between simulation and reality, enabling your robot to perceive and interact with the physical world.

## Hardware Integration Overview

### Real vs. Simulation Differences

When transitioning from simulation to real hardware, several key differences emerge:

| Aspect | Simulation | Real Hardware |
|--------|------------|---------------|
| **Sensors** | Perfect, noise-free data | Noisy, limited accuracy, calibration required |
| **Actuators** | Idealized, immediate response | Physical limitations, delays, wear |
| **Physics** | Perfect physics simulation | Real-world friction, backlash, compliance |
| **Timing** | Deterministic, controllable | Real-time constraints, jitter |
| **Safety** | No physical consequences | Risk of damage to robot/environment |
| **Connectivity** | Reliable internal communication | Network delays, packet loss possible |

### Integration Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                    Hardware Integration Architecture                          │
├─────────────────────────────────────────────────────────────────────────────────┤
│  Physical Sensors → Hardware Drivers → ROS Interfaces → AI Processing        │
│  (Cameras, IMU,    (GPIO, CAN, USB)   (Standard msgs)   (Perception, Control) │
│   LiDAR, Joint Enc)                                                        │
├─────────────────────────────────────────────────────────────────────────────────┤
│  AI Decisions → ROS Commands → Hardware Controllers → Physical Actuators    │
│  (Planning,     (Standard)      (Motor controllers)    (Motors, Servos)     │
│   Reasoning)                                                             │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Sensor Integration

### Camera Integration

#### Real Camera Driver Integration

```python
# camera_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from typing import Dict, Any

class RealCameraIntegrationNode(Node):
    def __init__(self):
        super().__init__('real_camera_integration')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Camera parameters
        self.camera_id = self.declare_parameter('camera_id', 0).value
        self.image_width = self.declare_parameter('image_width', 640).value
        self.image_height = self.declare_parameter('image_height', 480).value
        self.fps = self.declare_parameter('fps', 30).value

        # Initialize camera
        self._initialize_camera()

        # Publishers for camera data
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Timer for camera capture
        self.capture_timer = self.create_timer(1.0/self.fps, self.capture_frame)

        # Camera calibration parameters
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.load_camera_calibration()

        self.get_logger().info('Real camera integration initialized')

    def _initialize_camera(self):
        """Initialize real camera hardware"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)

            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

            # Check if camera opened successfully
            if not self.cap.isOpened():
                raise RuntimeError(f'Cannot open camera {self.camera_id}')

            self.get_logger().info(f'Camera {self.camera_id} initialized successfully')

        except Exception as e:
            self.get_logger().error(f'Camera initialization error: {e}')
            raise

    def load_camera_calibration(self):
        """Load camera calibration parameters"""
        # In a real implementation, this would load from a calibration file
        # For simulation, we'll use placeholder values
        self.camera_matrix = np.array([
            [500.0, 0.0, 320.0],  # fx, 0, cx
            [0.0, 500.0, 240.0],  # 0, fy, cy
            [0.0, 0.0, 1.0]       # 0, 0, 1
        ])

        self.distortion_coeffs = np.array([0.1, 0.01, 0.0, 0.0, 0.0])  # k1, k2, p1, p2, k3

        self.get_logger().info('Camera calibration loaded')

    def capture_frame(self):
        """Capture and publish camera frame"""
        try:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error('Failed to capture frame from camera')
                return

            # Create timestamp
            timestamp = self.get_clock().now()

            # Convert to ROS Image message
            image_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = timestamp.to_msg()
            image_msg.header.frame_id = 'camera_link'

            # Publish image
            self.image_pub.publish(image_msg)

            # Create and publish camera info
            info_msg = CameraInfo()
            info_msg.header = image_msg.header
            info_msg.width = self.image_width
            info_msg.height = self.image_height
            info_msg.k = self.camera_matrix.flatten().tolist()
            info_msg.d = self.distortion_coeffs.flatten().tolist()
            info_msg.r = np.eye(3).flatten().tolist()
            info_msg.p = np.zeros(12).tolist()  # Will be filled with projection matrix
            info_msg.distortion_model = 'plumb_bob'

            self.info_pub.publish(info_msg)

        except Exception as e:
            self.get_logger().error(f'Camera capture error: {e}')

    def destroy_node(self):
        """Clean up camera resources"""
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealCameraIntegrationNode()

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

### IMU Integration

#### Real IMU Sensor Integration

```python
# imu_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
import numpy as np
import serial
import struct
import time

class RealIMUIntegrationNode(Node):
    def __init__(self):
        super().__init__('real_imu_integration')

        # IMU parameters
        self.serial_port = self.declare_parameter('serial_port', '/dev/ttyUSB0').value
        self.baud_rate = self.declare_parameter('baud_rate', 115200).value
        self.imu_frame_id = self.declare_parameter('frame_id', 'imu_link').value

        # Initialize IMU connection
        self._initialize_imu_connection()

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)

        # Timer for IMU reading
        self.imu_timer = self.create_timer(0.01, self.read_imu_data)  # 100Hz

        # IMU calibration
        self.bias_calibration = {
            'accel': np.array([0.0, 0.0, 0.0]),
            'gyro': np.array([0.0, 0.0, 0.0]),
            'mag': np.array([0.0, 0.0, 0.0])
        }

        self.get_logger().info('Real IMU integration initialized')

    def _initialize_imu_connection(self):
        """Initialize connection to real IMU sensor"""
        try:
            # Connect to IMU via serial
            self.imu_serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )

            # Wait for IMU to initialize
            time.sleep(2.0)

            # Verify connection
            self.imu_serial.write(b'INIT\r\n')  # Send initialization command
            response = self.imu_serial.readline()
            self.get_logger().info(f'IMU response: {response.decode().strip()}')

        except Exception as e:
            self.get_logger().error(f'IMU connection error: {e}')
            raise

    def read_imu_data(self):
        """Read and publish IMU data"""
        try:
            # Read IMU data from serial (simplified protocol)
            if self.imu_serial.in_waiting > 0:
                line = self.imu_serial.readline().decode().strip()

                # Parse IMU data (assuming custom protocol)
                # Format: "ACC: x,y,z | GYRO: x,y,z | MAG: x,y,z"
                data = self._parse_imu_line(line)

                if data:
                    # Create IMU message
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = self.imu_frame_id

                    # Set acceleration (apply bias calibration)
                    imu_msg.linear_acceleration.x = data['accel'][0] - self.bias_calibration['accel'][0]
                    imu_msg.linear_acceleration.y = data['accel'][1] - self.bias_calibration['accel'][1]
                    imu_msg.linear_acceleration.z = data['accel'][2] - self.bias_calibration['accel'][2]

                    # Set angular velocity (apply bias calibration)
                    imu_msg.angular_velocity.x = data['gyro'][0] - self.bias_calibration['gyro'][0]
                    imu_msg.angular_velocity.y = data['gyro'][1] - self.bias_calibration['gyro'][1]
                    imu_msg.angular_velocity.z = data['gyro'][2] - self.bias_calibration['gyro'][2]

                    # Set orientation (would be computed from gyro integration)
                    # For now, set to identity
                    imu_msg.orientation.w = 1.0
                    imu_msg.orientation.x = 0.0
                    imu_msg.orientation.y = 0.0
                    imu_msg.orientation.z = 0.0

                    # Set covariance (real values would come from sensor specs)
                    imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
                    imu_msg.angular_velocity_covariance = [0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02]
                    imu_msg.orientation_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]

                    # Publish IMU data
                    self.imu_pub.publish(imu_msg)

                    # Create magnetic field message
                    mag_msg = MagneticField()
                    mag_msg.header = imu_msg.header
                    mag_msg.magnetic_field.x = data['mag'][0] - self.bias_calibration['mag'][0]
                    mag_msg.magnetic_field.y = data['mag'][1] - self.bias_calibration['mag'][1]
                    mag_msg.magnetic_field.z = data['mag'][2] - self.bias_calibration['mag'][2]

                    # Set magnetic field covariance
                    mag_msg.magnetic_field_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]

                    self.mag_pub.publish(mag_msg)

                    self.get_logger().debug(f'IMU data published: Acc=({data["accel"]}), Gyro=({data["gyro"]})')

        except Exception as e:
            self.get_logger().error(f'IMU reading error: {e}')

    def _parse_imu_line(self, line: str) -> Dict[str, np.ndarray]:
        """Parse IMU data line"""
        try:
            # Example parsing for format: "ACC:0.1,0.2,9.8|GYRO:0.0,0.0,0.1|MAG:0.2,0.1,0.5"
            if '|' in line:
                components = line.split('|')
                data = {}

                for comp in components:
                    if ':' in comp:
                        sensor_type, values = comp.split(':', 1)
                        values_list = [float(v.strip()) for v in values.split(',')]

                        if sensor_type.startswith('ACC'):
                            data['accel'] = np.array(values_list)
                        elif sensor_type.startswith('GYRO'):
                            data['gyro'] = np.array(values_list)
                        elif sensor_type.startswith('MAG'):
                            data['mag'] = np.array(values_list)

                if 'accel' in data and 'gyro' in data and 'mag' in data:
                    return data

        except Exception as e:
            self.get_logger().error(f'IMU line parsing error: {e}')

        # Return dummy data if parsing fails
        return {
            'accel': np.array([0.0, 0.0, 9.8]),
            'gyro': np.array([0.0, 0.0, 0.0]),
            'mag': np.array([0.2, 0.0, 0.5])
        }

    def calibrate_imu(self):
        """Calibrate IMU bias"""
        self.get_logger().info('Starting IMU calibration...')

        # Collect data for calibration
        accel_readings = []
        gyro_readings = []
        mag_readings = []

        for i in range(100):  # Collect 100 samples
            line = self.imu_serial.readline().decode().strip()
            data = self._parse_imu_line(line)

            if data:
                accel_readings.append(data['accel'])
                gyro_readings.append(data['gyro'])
                mag_readings.append(data['mag'])

        # Calculate biases
        if accel_readings and gyro_readings and mag_readings:
            self.bias_calibration['accel'] = np.mean(accel_readings, axis=0)
            self.bias_calibration['gyro'] = np.mean(gyro_readings, axis=0)
            self.bias_calibration['mag'] = np.mean(mag_readings, axis=0)

            # Adjust acceleration bias to account for gravity (assuming Z-axis is up)
            self.bias_calibration['accel'][2] -= 9.81

            self.get_logger().info(f'IMU calibration completed: Acc_bias={self.bias_calibration["accel"]}, '
                                 f'Gyro_bias={self.bias_calibration["gyro"]}, '
                                 f'Mag_bias={self.bias_calibration["mag"]}')

    def destroy_node(self):
        """Clean up IMU connection"""
        if hasattr(self, 'imu_serial'):
            self.imu_serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealIMUIntegrationNode()

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

#### Real LiDAR Sensor Integration

```python
# lidar_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np
import socket
import struct

class RealLiDARIntegrationNode(Node):
    def __init__(self):
        super().__init__('real_lidar_integration')

        # LiDAR parameters
        self.lidar_ip = self.declare_parameter('lidar_ip', '192.168.1.10').value
        self.lidar_port = self.declare_parameter('lidar_port', 2368).value
        self.lidar_frame_id = self.declare_parameter('frame_id', 'lidar_link').value

        # Initialize LiDAR connection
        self._initialize_lidar_connection()

        # Publisher for LiDAR data
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Timer for LiDAR data reading
        self.lidar_timer = self.create_timer(0.02, self.read_lidar_data)  # 50Hz

        # LiDAR configuration
        self.angle_min = -np.pi  # -180 degrees
        self.angle_max = np.pi   # 180 degrees
        self.angle_increment = 2 * np.pi / 720  # 0.5 degree resolution
        self.time_increment = 0.0  # Will be calculated from LiDAR data
        self.scan_time = 0.02  # 50Hz
        self.range_min = 0.1  # 10cm minimum range
        self.range_max = 30.0  # 30m maximum range

        self.get_logger().info('Real LiDAR integration initialized')

    def _initialize_lidar_connection(self):
        """Initialize connection to real LiDAR sensor"""
        try:
            # Create UDP socket for LiDAR data
            self.lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.lidar_socket.bind(('', self.lidar_port))
            self.lidar_socket.settimeout(1.0)

            self.get_logger().info(f'LiDAR socket bound to port {self.lidar_port}')

        except Exception as e:
            self.get_logger().error(f'LiDAR connection initialization error: {e}')
            raise

    def read_lidar_data(self):
        """Read and publish LiDAR scan data"""
        try:
            # Receive LiDAR data (non-blocking with timeout)
            try:
                data, addr = self.lidar_socket.recvfrom(65536)  # Large buffer for packet

                # Parse LiDAR scan data (this is sensor-specific)
                ranges = self._parse_lidar_packet(data)

                if ranges is not None:
                    # Create LaserScan message
                    scan_msg = LaserScan()
                    scan_msg.header.stamp = self.get_clock().now().to_msg()
                    scan_msg.header.frame_id = self.lidar_frame_id

                    scan_msg.angle_min = self.angle_min
                    scan_msg.angle_max = self.angle_max
                    scan_msg.angle_increment = self.angle_increment
                    scan_msg.time_increment = self.time_increment
                    scan_msg.scan_time = self.scan_time
                    scan_msg.range_min = self.range_min
                    scan_msg.range_max = self.range_max

                    # Apply range filtering and noise reduction
                    filtered_ranges = self._filter_lidar_ranges(ranges)
                    scan_msg.ranges = filtered_ranges.tolist()

                    # Set intensities if available
                    # scan_msg.intensities = intensities  # If sensor provides intensity data

                    # Publish scan
                    self.scan_pub.publish(scan_msg)

                    self.get_logger().debug(f'LiDAR scan published: {len(ranges)} points')

            except socket.timeout:
                self.get_logger().debug('LiDAR socket timeout - no data available')

        except Exception as e:
            self.get_logger().error(f'LiDAR reading error: {e}')

    def _parse_lidar_packet(self, packet_data):
        """Parse LiDAR packet data (sensor-specific implementation)"""
        try:
            # This is a simplified example - actual parsing depends on LiDAR model
            # For example, with Velodyne VLP-16:
            # Each packet contains 12 blocks of data with 32 firings each
            # This would require the actual sensor's packet format specification

            # For demonstration, we'll create simulated data based on packet
            # In practice, this would parse the actual sensor data format
            if len(packet_data) >= 100:  # Minimum packet size check
                # Extract range measurements from packet (sensor-specific)
                # This is a placeholder implementation
                num_points = 720  # Example: 720 points per revolution
                ranges = np.random.uniform(self.range_min, self.range_max, num_points).astype(np.float32)

                return ranges

        except Exception as e:
            self.get_logger().error(f'LiDAR packet parsing error: {e}')
            return None

    def _filter_lidar_ranges(self, ranges):
        """Filter LiDAR ranges to remove noise and invalid readings"""
        # Replace invalid ranges with NaN
        filtered = ranges.copy()

        # Set out-of-range values to infinity
        filtered[filtered < self.range_min] = float('inf')
        filtered[filtered > self.range_max] = float('inf')

        # Apply simple median filtering to reduce noise
        if len(filtered) > 3:
            # Apply median filter to reduce spike noise
            for i in range(1, len(filtered) - 1):
                if np.isfinite(filtered[i]):
                    # Check if this point is significantly different from neighbors
                    if (abs(filtered[i] - filtered[i-1]) > 2.0 and
                        abs(filtered[i] - filtered[i+1]) > 2.0):
                        # This might be noise, replace with median of neighbors
                        filtered[i] = np.median([filtered[i-1], filtered[i+1]])

        return filtered

    def _compute_scan_statistics(self, ranges):
        """Compute statistics about the scan"""
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            return {
                'min_range': np.min(valid_ranges),
                'max_range': np.max(valid_ranges),
                'avg_range': np.mean(valid_ranges),
                'num_valid_points': len(valid_ranges),
                'num_invalid_points': len(ranges) - len(valid_ranges)
            }
        else:
            return {
                'min_range': float('inf'),
                'max_range': 0.0,
                'avg_range': 0.0,
                'num_valid_points': 0,
                'num_invalid_points': len(ranges)
            }

    def destroy_node(self):
        """Clean up LiDAR connection"""
        if hasattr(self, 'lidar_socket'):
            self.lidar_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealLiDARIntegrationNode()

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

## Actuator Integration

### Joint Control Integration

```python
# actuator_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from builtin_interfaces.msg import Duration
import numpy as np
import can
import struct

class RealActuatorIntegrationNode(Node):
    def __init__(self):
        super().__init__('real_actuator_integration')

        # Actuator parameters
        self.can_interface = self.declare_parameter('can_interface', 'can0').value
        self.actuator_ids = self.declare_parameter(
            'actuator_ids',
            [1, 2, 3, 4, 5, 6]  # Example IDs for 6 joints
        ).value

        # Initialize CAN bus connection
        self._initialize_can_bus()

        # Initialize actuator control
        self._initialize_actuators()

        # Publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_cmd_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.joint_trajectory_callback,
            10
        )

        self.controller_state_pub = self.create_publisher(
            JointTrajectoryControllerState,
            '/controller_state',
            10
        )

        # Timer for joint state publishing
        self.state_timer = self.create_timer(0.01, self.publish_joint_states)  # 100Hz

        # Joint state tracking
        self.current_positions = np.zeros(len(self.actuator_ids))
        self.current_velocities = np.zeros(len(self.actuator_ids))
        self.current_efforts = np.zeros(len(self.actuator_ids))

        # Joint names (should match URDF)
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

        self.get_logger().info('Real actuator integration initialized')

    def _initialize_can_bus(self):
        """Initialize CAN bus for actuator communication"""
        try:
            # Initialize CAN bus interface
            self.can_bus = can.interface.Bus(
                channel=self.can_interface,
                bustype='socketcan'
            )

            self.get_logger().info(f'CAN bus initialized on {self.can_interface}')

        except Exception as e:
            self.get_logger().error(f'CAN bus initialization error: {e}')
            raise

    def _initialize_actuators(self):
        """Initialize real actuators"""
        try:
            # Send initialization commands to all actuators
            for actuator_id in self.actuator_ids:
                # Send initialization command (sensor-specific)
                init_cmd = self._create_initialization_command(actuator_id)
                self.can_bus.send(init_cmd)

                # Wait for initialization response
                msg = self.can_bus.recv(timeout=1.0)
                if msg and msg.arbitration_id == actuator_id:
                    self.get_logger().info(f'Actuator {actuator_id} initialized successfully')
                else:
                    self.get_logger().warn(f'No response from actuator {actuator_id}')

            self.get_logger().info('All actuators initialized')

        except Exception as e:
            self.get_logger().error(f'Actuator initialization error: {e}')
            raise

    def _create_initialization_command(self, actuator_id):
        """Create initialization command for actuator"""
        # This is sensor-specific - example for a generic actuator
        cmd_data = struct.pack('<BBBH', 0xFF, 0xFE, 0x01, 0x0000)  # Example command
        return can.Message(arbitration_id=actuator_id, data=cmd_data, is_extended_id=True)

    def _create_position_command(self, actuator_id, position):
        """Create position command for actuator"""
        # Convert position to actuator-specific format
        # This would depend on the specific actuator protocol
        position_int = int(position * 1000)  # Convert to millidegrees or appropriate unit

        cmd_data = struct.pack('<BBBH', 0x01, actuator_id, 0x02, position_int)  # Example
        return can.Message(arbitration_id=0x100 + actuator_id, data=cmd_data, is_extended_id=True)

    def joint_trajectory_callback(self, msg):
        """Process joint trajectory commands"""
        try:
            # Process trajectory points
            for point in msg.points:
                # Send position commands to actuators
                for i, joint_name in enumerate(msg.joint_names):
                    if joint_name in self.joint_names:
                        joint_idx = self.joint_names.index(joint_name)
                        if i < len(point.positions):
                            # Send position command to specific actuator
                            actuator_id = self.actuator_ids[joint_idx]
                            position = point.positions[i]

                            cmd_msg = self._create_position_command(actuator_id, position)
                            self.can_bus.send(cmd_msg)

                            self.get_logger().debug(f'Sent position command to actuator {actuator_id}: {position}')

        except Exception as e:
            self.get_logger().error(f'Joint trajectory processing error: {e}')

    def publish_joint_states(self):
        """Publish current joint states from real actuators"""
        try:
            # Read current positions from actuators
            self._read_current_positions()

            # Create JointState message
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.header.frame_id = ''

            joint_state.name = self.joint_names
            joint_state.position = self.current_positions.tolist()
            joint_state.velocity = self.current_velocities.tolist()
            joint_state.effort = self.current_efforts.tolist()

            # Publish joint states
            self.joint_state_pub.publish(joint_state)

            # Also publish controller state
            controller_state = JointTrajectoryControllerState()
            controller_state.header = joint_state.header
            controller_state.joint_names = self.joint_names
            controller_state.actual.positions = self.current_positions.tolist()
            controller_state.actual.velocities = self.current_velocities.tolist()
            controller_state.actual.accelerations = [0.0] * len(self.current_positions)  # Not available from actuators

            self.controller_state_pub.publish(controller_state)

        except Exception as e:
            self.get_logger().error(f'Joint state publishing error: {e}')

    def _read_current_positions(self):
        """Read current positions from real actuators"""
        try:
            # Request position from each actuator
            for i, actuator_id in enumerate(self.actuator_ids):
                # Create position request command
                req_cmd = self._create_position_request_command(actuator_id)
                self.can_bus.send(req_cmd)

                # Wait for response
                response = self.can_bus.recv(timeout=0.1)
                if response and response.arbitration_id == actuator_id + 0x200:  # Response ID
                    # Parse position from response
                    position = self._parse_position_response(response)
                    self.current_positions[i] = position

        except Exception as e:
            self.get_logger().error(f'Position reading error: {e}')

    def _create_position_request_command(self, actuator_id):
        """Create position request command"""
        # Example command format - would be sensor-specific
        cmd_data = struct.pack('<BBBH', 0x02, actuator_id, 0x01, 0x0000)  # Request position
        return can.Message(arbitration_id=0x200 + actuator_id, data=cmd_data, is_extended_id=True)

    def _parse_position_response(self, response):
        """Parse position from actuator response"""
        try:
            # Example parsing - would be sensor-specific
            if len(response.data) >= 6:
                position_raw = struct.unpack('<h', response.data[4:6])[0]
                # Convert from raw units to radians (sensor-specific conversion)
                position_rad = position_raw * 0.001  # Example conversion
                return position_rad
        except Exception as e:
            self.get_logger().error(f'Position response parsing error: {e}')
            return 0.0

    def _safety_check_positions(self, positions):
        """Check if requested positions are safe"""
        # Check joint limits and safety constraints
        joint_limits = {
            'left_hip_joint': (-1.57, 1.57),
            'left_knee_joint': (0.0, 2.0),
            'left_ankle_joint': (-0.78, 0.78),
            'right_hip_joint': (-1.57, 1.57),
            'right_knee_joint': (0.0, 2.0),
            'right_ankle_joint': (-0.78, 0.78)
        }

        for i, (joint_name, pos) in enumerate(zip(self.joint_names, positions)):
            if joint_name in joint_limits:
                min_limit, max_limit = joint_limits[joint_name]
                if not (min_limit <= pos <= max_limit):
                    self.get_logger().warn(f'Joint {joint_name} position {pos} outside limits [{min_limit}, {max_limit}]')
                    # Clamp to safe limits
                    positions[i] = np.clip(pos, min_limit, max_limit)

        return positions

    def destroy_node(self):
        """Clean up actuator connections"""
        if hasattr(self, 'can_bus'):
            self.can_bus.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealActuatorIntegrationNode()

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

## Sensor Fusion Integration

### Multi-Sensor Data Integration

```python
# sensor_fusion_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan, JointState
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque

class RealSensorFusionNode(Node):
    def __init__(self):
        super().__init__('real_sensor_fusion')

        # Initialize TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize sensor data storage
        self.camera_data = None
        self.imu_data = None
        self.lidar_data = None
        self.joint_data = None

        # Timestamps for synchronization
        self.camera_timestamp = None
        self.imu_timestamp = None
        self.lidar_timestamp = None
        self.joint_timestamp = None

        # Synchronization window (seconds)
        self.sync_window = 0.1  # 100ms synchronization window

        # Data buffers for filtering
        self.imu_buffer = deque(maxlen=10)
        self.joint_buffer = deque(maxlen=10)

        # Publishers for fused data
        self.fused_pose_pub = self.create_publisher(PoseStamped, '/fused_pose', 10)
        self.robot_state_pub = self.create_publisher(JointState, '/robot_state', 10)
        self.environment_pub = self.create_publisher(String, '/environment_state', 10)

        # Subscribers for real sensors
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Timer for fusion processing
        self.fusion_timer = self.create_timer(0.05, self.process_sensor_fusion)  # 20Hz

        self.get_logger().info('Real sensor fusion node initialized')

    def camera_callback(self, msg):
        """Update camera data"""
        self.camera_data = msg
        self.camera_timestamp = msg.header.stamp

    def imu_callback(self, msg):
        """Update IMU data"""
        self.imu_data = msg
        self.imu_timestamp = msg.header.stamp

        # Add to buffer for filtering
        self.imu_buffer.append({
            'linear_acceleration': np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]),
            'angular_velocity': np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ]),
            'timestamp': msg.header.stamp
        })

    def lidar_callback(self, msg):
        """Update LiDAR data"""
        self.lidar_data = msg
        self.lidar_timestamp = msg.header.stamp

    def joint_callback(self, msg):
        """Update joint state data"""
        self.joint_data = msg
        self.joint_timestamp = msg.header.stamp

        # Add to buffer for filtering
        self.joint_buffer.append({
            'positions': np.array(msg.position),
            'velocities': np.array(msg.velocity),
            'efforts': np.array(msg.effort),
            'timestamp': msg.header.stamp
        })

    def process_sensor_fusion(self):
        """Process and fuse sensor data"""
        # Check if we have reasonably synchronized data
        if self._data_is_synchronized():
            try:
                # Perform sensor fusion
                fused_data = self._fuse_sensor_data()

                # Publish fused results
                if fused_data:
                    self._publish_fused_data(fused_data)

            except Exception as e:
                self.get_logger().error(f'Sensor fusion error: {e}')

    def _data_is_synchronized(self) -> bool:
        """Check if sensor data is synchronized within window"""
        if not all([self.camera_data, self.imu_data, self.lidar_data, self.joint_data]):
            return False

        # Get latest timestamps
        timestamps = [
            self.camera_timestamp,
            self.imu_timestamp,
            self.lidar_timestamp,
            self.joint_timestamp
        ]

        # Convert to seconds for comparison
        secs = [ts.sec + ts.nanosec / 1e9 for ts in timestamps]

        # Check if all timestamps are within sync window
        max_diff = max(secs) - min(secs)
        return max_diff <= self.sync_window

    def _fuse_sensor_data(self):
        """Fuse sensor data from multiple sources"""
        fused_result = {
            'timestamp': self.get_clock().now().to_msg(),
            'pose': self._estimate_pose(),
            'velocity': self._estimate_velocity(),
            'environment': self._analyze_environment(),
            'confidence': self._calculate_confidence()
        }

        return fused_result

    def _estimate_pose(self):
        """Estimate robot pose using sensor fusion"""
        # This would implement a Kalman filter or other fusion algorithm
        # For now, return a simple estimate based on joint positions

        if self.joint_data:
            # Calculate pose from forward kinematics
            # This is simplified - in reality, you'd use a proper FK solver
            pose = PoseStamped()
            pose.header.stamp = self.joint_timestamp
            pose.header.frame_id = 'base_link'

            # Simple example: estimate position based on joint angles
            # In practice, use proper forward kinematics
            pose.pose.position.x = 0.0  # Would come from FK
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.8  # Approximate standing height

            # Estimate orientation from IMU
            if self.imu_data:
                pose.pose.orientation = self.imu_data.orientation

            return pose

        return None

    def _estimate_velocity(self):
        """Estimate robot velocity using sensor fusion"""
        # Calculate velocity from IMU integration and joint velocity
        if self.imu_buffer and len(self.imu_buffer) >= 2:
            # Get recent IMU data for velocity estimation
            recent_imu = list(self.imu_buffer)[-2:]
            dt = (recent_imu[1]['timestamp'].nanoseconds - recent_imu[0]['timestamp'].nanoseconds) / 1e9

            if dt > 0:
                # Integrate acceleration to get velocity
                accel_change = recent_imu[1]['linear_acceleration'] - recent_imu[0]['linear_acceleration']
                velocity_estimate = accel_change * dt

                twist = Twist()
                twist.linear.x = velocity_estimate[0]
                twist.linear.y = velocity_estimate[1]
                twist.linear.z = velocity_estimate[2]

                return twist

        return None

    def _analyze_environment(self):
        """Analyze environment using fused sensor data"""
        environment_state = {
            'obstacle_distance': float('inf'),
            'ground_contact': True,
            'free_space_direction': 'forward',
            'object_count': 0
        }

        # Analyze LiDAR data for obstacles
        if self.lidar_data:
            ranges = np.array(self.lidar_data.ranges)
            valid_ranges = ranges[np.isfinite(ranges)]

            if len(valid_ranges) > 0:
                environment_state['obstacle_distance'] = float(np.min(valid_ranges))
                environment_state['object_count'] = len(valid_ranges)

        # Analyze joint data for ground contact
        if self.joint_data:
            # Check if feet are in contact based on force sensors
            # This would use actual force/torque sensor data in practice
            pass

        return environment_state

    def _calculate_confidence(self):
        """Calculate confidence in fused data"""
        # Calculate confidence based on sensor data quality
        confidence = 0.0

        if self.joint_data:
            confidence += 0.3  # Joint states are usually reliable

        if self.imu_data:
            confidence += 0.25  # IMU provides good orientation data

        if self.lidar_data:
            confidence += 0.25  # LiDAR provides good environment data

        if self.camera_data:
            confidence += 0.2  # Camera provides rich visual data

        return min(1.0, confidence)  # Cap at 1.0

    def _publish_fused_data(self, fused_data):
        """Publish fused sensor data"""
        # Publish fused pose
        if fused_data['pose']:
            self.fused_pose_pub.publish(fused_data['pose'])

        # Publish robot state
        if self.joint_data:
            self.robot_state_pub.publish(self.joint_data)

        # Publish environment state
        env_msg = String()
        env_msg.data = str(fused_data['environment'])
        self.environment_pub.publish(env_msg)

        self.get_logger().info(f'Sensor fusion completed with confidence: {fused_data["confidence"]:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = RealSensorFusionNode()

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

## Hardware-Specific Optimization

### Performance Optimization for Real Hardware

```python
# hardware_optimization.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Image, JointState
import numpy as np
import time
from collections import deque
import psutil
import GPUtil

class HardwareOptimizationNode(Node):
    def __init__(self):
        super().__init__('hardware_optimization')

        # Performance monitoring
        self.processing_times = deque(maxlen=100)
        self.cpu_usage_history = deque(maxlen=100)
        self.memory_usage_history = deque(maxlen=100)
        self.gpu_usage_history = deque(maxlen=100)

        # Publishers for performance metrics
        self.processing_time_pub = self.create_publisher(Float32, '/hardware/processing_time', 10)
        self.cpu_usage_pub = self.create_publisher(Float32, '/hardware/cpu_usage', 10)
        self.memory_usage_pub = self.create_publisher(Float32, '/hardware/memory_usage', 10)
        self.gpu_usage_pub = self.create_publisher(Float32, '/hardware/gpu_usage', 10)

        # Adaptive parameters
        self.current_resolution_scale = 1.0
        self.current_processing_quality = 'high'
        self.current_update_rate = 30.0

        # Performance monitoring timer
        self.performance_timer = self.create_timer(0.5, self.performance_monitoring_loop)

        # Adaptive optimization timer
        self.optimization_timer = self.create_timer(2.0, self.adaptive_optimization_loop)

        self.get_logger().info('Hardware optimization node initialized')

    def performance_monitoring_loop(self):
        """Monitor system performance"""
        try:
            # Get current performance metrics
            current_time = time.time()

            # CPU usage
            cpu_percent = psutil.cpu_percent()
            self.cpu_usage_history.append(cpu_percent)

            # Memory usage
            memory_percent = psutil.virtual_memory().percent
            self.memory_usage_history.append(memory_percent)

            # GPU usage (if available)
            gpu_percent = 0.0
            try:
                gpus = GPUtil.getGPUs()
                if gpus:
                    gpu_percent = gpus[0].load * 100
            except:
                pass  # GPU monitoring not available

            self.gpu_usage_history.append(gpu_percent)

            # Calculate averages
            avg_cpu = np.mean(list(self.cpu_usage_history)[-10:]) if self.cpu_usage_history else 0.0
            avg_memory = np.mean(list(self.memory_usage_history)[-10:]) if self.memory_usage_history else 0.0
            avg_gpu = np.mean(list(self.gpu_usage_history)[-10:]) if self.gpu_usage_history else 0.0

            # Publish metrics
            cpu_msg = Float32()
            cpu_msg.data = float(avg_cpu)
            self.cpu_usage_pub.publish(cpu_msg)

            memory_msg = Float32()
            memory_msg.data = float(avg_memory)
            self.memory_usage_pub.publish(memory_msg)

            gpu_msg = Float32()
            gpu_msg.data = float(avg_gpu)
            self.gpu_usage_pub.publish(gpu_msg)

            # Log performance
            self.get_logger().info(
                f'Performance - CPU: {avg_cpu:.1f}%, '
                f'Memory: {avg_memory:.1f}%, '
                f'GPU: {avg_gpu:.1f}%'
            )

        except Exception as e:
            self.get_logger().error(f'Performance monitoring error: {e}')

    def adaptive_optimization_loop(self):
        """Apply adaptive optimization based on performance"""
        try:
            # Get average usage over recent period
            avg_cpu = np.mean(list(self.cpu_usage_history)[-10:]) if self.cpu_usage_history else 0.0
            avg_gpu = np.mean(list(self.gpu_usage_history)[-10:]) if self.gpu_usage_history else 0.0
            avg_memory = np.mean(list(self.memory_usage_history)[-10:]) if self.memory_usage_history else 0.0

            # Apply optimizations based on load
            if avg_cpu > 80 or avg_gpu > 85 or avg_memory > 85:
                # High load - reduce quality
                self._apply_performance_optimization()
            elif avg_cpu < 50 and avg_gpu < 50 and avg_memory < 60:
                # Low load - increase quality
                self._apply_quality_enhancement()

        except Exception as e:
            self.get_logger().error(f'Adaptive optimization error: {e}')

    def _apply_performance_optimization(self):
        """Apply performance optimizations when system is under load"""
        # Reduce processing quality
        if self.current_processing_quality != 'low':
            self.current_processing_quality = 'low'
            self.get_logger().info('Applied performance optimization - reduced quality')

        # Reduce image resolution
        if self.current_resolution_scale > 0.5:
            self.current_resolution_scale *= 0.9
            self.get_logger().info(f'Reduced resolution scale to {self.current_resolution_scale:.2f}')

        # Reduce update rates
        if self.current_update_rate > 15.0:
            self.current_update_rate = max(15.0, self.current_update_rate * 0.9)
            self.get_logger().info(f'Reduced update rate to {self.current_update_rate:.1f}Hz')

    def _apply_quality_enhancement(self):
        """Apply quality enhancements when system has capacity"""
        # Increase processing quality
        if self.current_processing_quality != 'high':
            self.current_processing_quality = 'high'
            self.get_logger().info('Applied quality enhancement - increased quality')

        # Increase image resolution
        if self.current_resolution_scale < 1.0:
            self.current_resolution_scale = min(1.0, self.current_resolution_scale * 1.1)
            self.get_logger().info(f'Increased resolution scale to {self.current_resolution_scale:.2f}')

        # Increase update rates
        if self.current_update_rate < 60.0:
            self.current_update_rate = min(60.0, self.current_update_rate * 1.1)
            self.get_logger().info(f'Increased update rate to {self.current_update_rate:.1f}Hz')

def main(args=None):
    rclpy.init(args=args)
    node = HardwareOptimizationNode()

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

## Safety and Reliability

### Hardware Safety Systems

```python
# hardware_safety.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration

class HardwareSafetyNode(Node):
    def __init__(self):
        super().__init__('hardware_safety')

        # Publishers for safety
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.safety_status_pub = self.create_publisher(String, '/safety_status', 10)

        # Subscribers for monitoring
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Safety parameters
        self.emergency_stop_active = False
        self.safety_thresholds = {
            'joint_position': 2.0,  # radians
            'joint_velocity': 5.0,  # rad/s
            'joint_effort': 50.0,  # N*m
            'command_velocity': 1.0,  # m/s
            'temperature': 70.0,  # Celsius
            'current': 10.0  # Amperes
        }

        # Safety monitoring timer
        self.safety_timer = self.create_timer(0.1, self.safety_monitoring_loop)

        self.get_logger().info('Hardware safety node initialized')

    def joint_state_callback(self, msg):
        """Monitor joint states for safety violations"""
        if self.emergency_stop_active:
            return

        # Check for safety violations
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                pos = msg.position[i]
                if abs(pos) > self.safety_thresholds['joint_position']:
                    self._trigger_safety_violation(f'Joint {name} position violation: {pos:.3f}')
                    return

            if i < len(msg.velocity):
                vel = msg.velocity[i]
                if abs(vel) > self.safety_thresholds['joint_velocity']:
                    self._trigger_safety_violation(f'Joint {name} velocity violation: {vel:.3f}')
                    return

            if i < len(msg.effort):
                effort = msg.effort[i]
                if abs(effort) > self.safety_thresholds['joint_effort']:
                    self._trigger_safety_violation(f'Joint {name} effort violation: {effort:.3f}')
                    return

    def cmd_vel_callback(self, msg):
        """Monitor velocity commands for safety"""
        if self.emergency_stop_active:
            return

        # Check if velocity command is safe
        linear_speed = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
        angular_speed = (msg.angular.x**2 + msg.angular.y**2 + msg.angular.z**2)**0.5

        if linear_speed > self.safety_thresholds['command_velocity']:
            self._trigger_safety_violation(f'Linear velocity violation: {linear_speed:.3f}')
            return

        if angular_speed > self.safety_thresholds['command_velocity'] * 2:  # Higher for rotation
            self._trigger_safety_violation(f'Angular velocity violation: {angular_speed:.3f}')
            return

    def safety_monitoring_loop(self):
        """Continuous safety monitoring"""
        if not self.emergency_stop_active:
            # Additional safety checks could go here
            # - Temperature monitoring
            # - Current monitoring
            # - Position verification
            # - Balance monitoring
            pass

    def _trigger_safety_violation(self, violation_msg):
        """Trigger safety violation and emergency stop"""
        self.get_logger().error(f'SAFETY VIOLATION: {violation_msg}')

        # Activate emergency stop
        self.emergency_stop_active = True

        # Publish emergency stop signal
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)

        # Publish safety status
        status_msg = String()
        status_msg.data = f'SAFETY VIOLATION: {violation_msg}'
        self.safety_status_pub.publish(status_msg)

        self.get_logger().fatal('EMERGENCY STOP ACTIVATED - SYSTEM HALTED')

    def deactivate_emergency_stop(self):
        """Deactivate emergency stop (with safety checks)"""
        # In a real system, this would require manual confirmation
        # and verification that conditions are safe
        if self.emergency_stop_active:
            self.emergency_stop_active = False

            # Publish resume signal
            stop_msg = Bool()
            stop_msg.data = False
            self.emergency_stop_pub.publish(stop_msg)

            self.get_logger().info('Emergency stop deactivated')

def main(args=None):
    rclpy.init(args=args)
    node = HardwareSafetyNode()

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

## Best Practices for Hardware Integration

### Hardware Integration Best Practices

1. **Modular Design**: Keep sensor and actuator interfaces modular and replaceable
2. **Error Handling**: Implement robust error handling for hardware failures
3. **Calibration**: Regularly calibrate sensors for accuracy
4. **Timing**: Ensure real-time constraints are met
5. **Safety**: Implement multiple layers of safety checks
6. **Performance**: Optimize for hardware constraints
7. **Validation**: Continuously validate sensor data quality
8. **Documentation**: Document hardware-specific configurations

### Performance Optimization Guidelines

1. **Batch Processing**: Process multiple sensor readings together when possible
2. **Asynchronous Operations**: Use non-blocking I/O for sensor communication
3. **Memory Management**: Efficiently manage memory for real-time processing
4. **Threading**: Use appropriate threading for I/O and processing separation
5. **Quality of Service**: Configure appropriate QoS for real-time requirements
6. **Resource Monitoring**: Continuously monitor system resources
7. **Adaptive Processing**: Adjust processing quality based on system load

## Troubleshooting Hardware Integration

### Common Issues and Solutions

**Issue**: Sensor data not arriving
**Solution**: Check hardware connections, verify driver installation, validate communication protocols

**Issue**: Actuator commands not executing
**Solution**: Verify control authority, check safety systems, validate command formats

**Issue**: Timing constraints not met
**Solution**: Optimize processing, adjust update rates, improve hardware utilization

**Issue**: Calibration drift
**Solution**: Implement automatic recalibration, monitor calibration parameters

**Issue**: Safety system activation
**Solution**: Identify root cause, adjust thresholds, verify hardware status

## Next Steps

In the next chapter, we'll explore sensor calibration in detail, learning how to properly calibrate real-world sensors to ensure accurate perception and reliable robot operation. Calibration is crucial for bridging the gap between simulation and reality, ensuring that your AI systems can properly interpret real sensor data.