# Safety and Guardrails

In this chapter, we'll explore comprehensive safety systems and guardrails for AI-driven humanoid robots. Safety is paramount when developing robots that interact with humans and operate in human environments. We'll implement multiple layers of safety checks, validation systems, and fail-safe mechanisms.

## Safety Architecture Overview

### Multi-Layer Safety System

```
┌─────────────────────────────────────────────────────────────────┐
│                    Safety Architecture                        │
├─────────────────────────────────────────────────────────────────┤
│  Perception → Action → Execution → Monitoring → Recovery      │
│  Safety     Safety   Safety     Safety      Safety         │
│  (Input)    (Planning) (Control)  (Runtime)   (Emergency)   │
├─────────────────────────────────────────────────────────────────┤
│  Layers:                                                     │
│  • Hardware Safety: Joint limits, collision detection        │
│  • Perception Safety: Object validation, environment checks  │
│  • Planning Safety: Path validation, feasibility checking    │
│  • Execution Safety: Real-time monitoring, emergency stops   │
│  • Behavioral Safety: Ethical guidelines, social norms       │
└─────────────────────────────────────────────────────────────────┘
```

## Hardware Safety Systems

### Joint Limit Protection

```python
# hardware_safety.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Bool, String
from builtin_interfaces.msg import Duration
import numpy as np
from typing import Dict, List

class HardwareSafetyNode(Node):
    def __init__(self):
        super().__init__('hardware_safety')

        # Initialize safety parameters
        self._initialize_safety_parameters()

        # Joint state monitoring
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Command monitoring
        self.joint_command_sub = self.create_subscription(
            JointState,
            '/joint_commands',
            self.joint_command_callback,
            10
        )

        # Safety publishers
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.safety_status_pub = self.create_publisher(String, '/safety_status', 10)

        # Safety monitoring timer
        self.safety_timer = self.create_timer(0.01, self.safety_monitoring_loop)  # 100Hz

        # Initialize state
        self.current_joint_states = JointState()
        self.current_joint_commands = JointState()
        self.emergency_stop_active = False
        self.safety_violations = []

        self.get_logger().info('Hardware Safety Node initialized')

    def _initialize_safety_parameters(self):
        """Initialize safety parameters and limits"""
        # Joint limits (these should match your URDF)
        self.joint_limits = {
            'left_hip_joint': {'min': -1.57, 'max': 1.57, 'max_velocity': 2.0, 'max_effort': 20.0},
            'left_knee_joint': {'min': 0.0, 'max': 2.0, 'max_velocity': 2.0, 'max_effort': 20.0},
            'left_ankle_joint': {'min': -0.78, 'max': 0.78, 'max_velocity': 2.0, 'max_effort': 10.0},
            'right_hip_joint': {'min': -1.57, 'max': 1.57, 'max_velocity': 2.0, 'max_effort': 20.0},
            'right_knee_joint': {'min': 0.0, 'max': 2.0, 'max_velocity': 2.0, 'max_effort': 20.0},
            'right_ankle_joint': {'min': -0.78, 'max': 0.78, 'max_velocity': 2.0, 'max_effort': 10.0},
            'left_shoulder_joint': {'min': -1.57, 'max': 1.57, 'max_velocity': 3.0, 'max_effort': 15.0},
            'left_elbow_joint': {'min': -1.57, 'max': 1.57, 'max_velocity': 3.0, 'max_effort': 10.0},
            'right_shoulder_joint': {'min': -1.57, 'max': 1.57, 'max_velocity': 3.0, 'max_effort': 15.0},
            'right_elbow_joint': {'min': -1.57, 'max': 1.57, 'max_velocity': 3.0, 'max_effort': 10.0},
            # Add more joints as needed
        }

        # Safety thresholds
        self.temperature_threshold = 70.0  # Celsius
        self.current_threshold = 10.0     # Amperes
        self.voltage_threshold = 12.0     # Volts

        # Emergency stop configuration
        self.enable_emergency_stop = True
        self.emergency_stop_reason = ""

        self.get_logger().info('Safety parameters initialized')

    def joint_state_callback(self, msg):
        """Monitor current joint states for safety violations"""
        self.current_joint_states = msg

        # Check for safety violations
        violations = self._check_joint_state_safety(msg)

        if violations:
            self.safety_violations.extend(violations)
            self._handle_safety_violations(violations)

    def joint_command_callback(self, msg):
        """Monitor joint commands for safety violations"""
        self.current_joint_commands = msg

        # Check command safety
        violations = self._check_command_safety(msg)

        if violations:
            self.safety_violations.extend(violations)
            self._handle_safety_violations(violations)

    def _check_joint_state_safety(self, joint_state: JointState) -> List[str]:
        """Check joint states for safety violations"""
        violations = []

        for i, name in enumerate(joint_state.name):
            if name in self.joint_limits:
                limit = self.joint_limits[name]

                # Check position limits
                if i < len(joint_state.position):
                    pos = joint_state.position[i]
                    if pos < limit['min'] or pos > limit['max']:
                        violations.append(f"Joint {name} position violation: {pos:.3f} (limits: {limit['min']:.3f} to {limit['max']:.3f})")

                # Check velocity limits
                if i < len(joint_state.velocity):
                    vel = joint_state.velocity[i]
                    max_vel = limit['max_velocity']
                    if abs(vel) > max_vel:
                        violations.append(f"Joint {name} velocity violation: {vel:.3f} (max: {max_vel:.3f})")

                # Check effort limits
                if i < len(joint_state.effort):
                    effort = joint_state.effort[i]
                    max_effort = limit['max_effort']
                    if abs(effort) > max_effort:
                        violations.append(f"Joint {name} effort violation: {effort:.3f} (max: {max_effort:.3f})")

        return violations

    def _check_command_safety(self, joint_command: JointState) -> List[str]:
        """Check joint commands for safety violations"""
        violations = []

        for i, name in enumerate(joint_command.name):
            if name in self.joint_limits:
                limit = self.joint_limits[name]

                # Check if commanded position is within limits
                if i < len(joint_command.position):
                    pos = joint_command.position[i]
                    if pos < limit['min'] or pos > limit['max']:
                        violations.append(f"Commanded position violation for {name}: {pos:.3f}")

                # Check if commanded velocity is within limits
                if i < len(joint_command.velocity):
                    vel = joint_command.velocity[i]
                    max_vel = limit['max_velocity']
                    if abs(vel) > max_vel:
                        violations.append(f"Commanded velocity violation for {name}: {vel:.3f}")

        return violations

    def safety_monitoring_loop(self):
        """Continuous safety monitoring loop"""
        if not self.emergency_stop_active:
            # Check for various safety conditions
            self._check_temperature_safety()
            self._check_collision_safety()
            self._check_stability_safety()

    def _check_temperature_safety(self):
        """Check joint temperatures"""
        # In a real system, this would interface with temperature sensors
        # For simulation, we'll check velocity and effort for heat generation
        if self.current_joint_states:
            for i, name in enumerate(self.current_joint_states.name):
                if i < len(self.current_joint_states.velocity) and i < len(self.current_joint_states.effort):
                    velocity = abs(self.current_joint_states.velocity[i])
                    effort = abs(self.current_joint_states.effort[i])

                    # Estimate temperature based on activity
                    estimated_temp = 25.0 + (velocity * 0.1) + (effort * 0.05)

                    if estimated_temp > self.temperature_threshold:
                        violation = f"Joint {name} estimated temperature violation: {estimated_temp:.1f}°C (threshold: {self.temperature_threshold}°C)"
                        self.safety_violations.append(violation)
                        self._handle_safety_violations([violation])

    def _check_collision_safety(self):
        """Check for collision safety"""
        # This would interface with collision detection system
        # In simulation, we might check joint positions for self-collision
        if self.current_joint_states:
            # Example: Check for potential self-collisions
            self_collision_violations = self._check_self_collision(self.current_joint_states)
            if self_collision_violations:
                self.safety_violations.extend(self_collision_violations)
                self._handle_safety_violations(self_collision_violations)

    def _check_self_collision(self, joint_state: JointState) -> List[str]:
        """Check for self-collision based on joint positions"""
        violations = []

        # Example: Check if arms are crossed in a dangerous way
        # This is a simplified example - real collision checking would be much more complex
        left_elbow_pos = None
        right_elbow_pos = None

        for i, name in enumerate(joint_state.name):
            if name == 'left_elbow_joint' and i < len(joint_state.position):
                left_elbow_pos = joint_state.position[i]
            elif name == 'right_elbow_joint' and i < len(joint_state.position):
                right_elbow_pos = joint_state.position[i]

        if left_elbow_pos and right_elbow_pos:
            # Check if arms are crossing in a potentially dangerous way
            if abs(left_elbow_pos - right_elbow_pos) < 0.1:  # Too close
                violations.append("Potential self-collision: arms too close")

        return violations

    def _handle_safety_violations(self, violations: List[str]):
        """Handle safety violations"""
        for violation in violations:
            self.get_logger().error(f'Safety violation: {violation}')

        # Activate emergency stop if critical violations
        if self._has_critical_violations(violations):
            self._activate_emergency_stop(f"Critical safety violations: {', '.join(violations)}")

        # Publish safety status
        status_msg = String()
        status_msg.data = json.dumps({
            'violations': violations,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'emergency_stop_active': self.emergency_stop_active
        })
        self.safety_status_pub.publish(status_msg)

    def _has_critical_violations(self, violations: List[str]) -> bool:
        """Check if violations are critical and require emergency stop"""
        critical_keywords = ['collision', 'temperature', 'position', 'velocity', 'effort', 'stability']
        return any(any(keyword in violation.lower() for keyword in critical_keywords) for violation in violations)

    def _activate_emergency_stop(self, reason: str):
        """Activate emergency stop"""
        if self.enable_emergency_stop:
            self.emergency_stop_active = True
            self.emergency_stop_reason = reason

            # Publish emergency stop command
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_pub.publish(stop_msg)

            self.get_logger().fatal(f'EMERGENCY STOP ACTIVATED: {reason}')

    def deactivate_emergency_stop(self):
        """Deactivate emergency stop (with safety checks)"""
        # Perform safety checks before deactivation
        if self._can_safely_deactivate_emergency_stop():
            self.emergency_stop_active = False
            self.emergency_stop_reason = ""

            stop_msg = Bool()
            stop_msg.data = False
            self.emergency_stop_pub.publish(stop_msg)

            self.get_logger().info('Emergency stop deactivated')

    def _can_safely_deactivate_emergency_stop(self) -> bool:
        """Check if emergency stop can be safely deactivated"""
        # Check that no current safety violations exist
        if self.safety_violations:
            return False

        # Check joint states are within safe ranges
        if self.current_joint_states:
            violations = self._check_joint_state_safety(self.current_joint_states)
            return len(violations) == 0

        return True

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

## Perception Safety

### Safe Object Recognition and Interaction

```python
# perception_safety.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import TransformListener, Buffer
import cv2
import numpy as np
from cv_bridge import CvBridge
from typing import Dict, List, Optional

class PerceptionSafetyNode(Node):
    def __init__(self):
        super().__init__('perception_safety')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize safety components
        self._initialize_perception_safety()

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10
        )

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

        # Publishers
        self.safe_detections_pub = self.create_publisher(Detection2DArray, '/safe_detections', 10)
        self.safety_alert_pub = self.create_publisher(String, '/safety_alert', 10)
        self.perception_safety_pub = self.create_publisher(Bool, '/perception_safe', 10)

        # TF listener for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Safety state
        self.hazardous_objects = set()
        self.safe_interaction_zones = []
        self.perception_confidence_threshold = 0.7

        self.get_logger().info('Perception Safety Node initialized')

    def _initialize_perception_safety(self):
        """Initialize perception safety parameters"""
        # Define hazardous object classes
        self.hazardous_classes = {
            'knife', 'blade', 'sharp_object', 'fire', 'hot_surface',
            'chemical', 'hazardous_material', 'dangerous_object'
        }

        # Define fragile object classes
        self.fragile_classes = {
            'glass', 'ceramic', 'fragile', 'breakable', 'delicate'
        }

        # Define forbidden interaction zones
        self.forbidden_zones = [
            # Example: Kitchen stove area
            {'center': [1.0, 0.0, 0.0], 'radius': 0.5, 'type': 'forbidden'},
            # Example: Electrical panel area
            {'center': [-1.0, 2.0, 0.0], 'radius': 0.3, 'type': 'forbidden'}
        ]

        # Define safe interaction zones
        self.safe_zones = [
            # Example: Dining table area
            {'center': [0.0, -1.0, 0.0], 'radius': 1.0, 'type': 'safe'},
            # Example: Workbench area
            {'center': [2.0, 1.0, 0.0], 'radius': 0.8, 'type': 'safe'}
        ]

        self.get_logger().info('Perception safety parameters initialized')

    def detection_callback(self, msg):
        """Process object detections for safety"""
        try:
            # Filter detections based on safety criteria
            safe_detections = self._filter_safe_detections(msg)

            # Check for hazardous object detections
            hazardous_detections = self._check_hazardous_objects(msg)

            if hazardous_detections:
                self._handle_hazardous_detections(hazardous_detections)

            # Publish safe detections
            if safe_detections.detections:
                self.safe_detections_pub.publish(safe_detections)

            # Publish safety status
            safety_msg = Bool()
            safety_msg.data = len(hazardous_detections) == 0
            self.perception_safety_pub.publish(safety_msg)

        except Exception as e:
            self.get_logger().error(f'Detection safety processing error: {e}')

    def _filter_safe_detections(self, detection_array) -> Detection2DArray:
        """Filter detections to exclude unsafe objects"""
        safe_detections = Detection2DArray()
        safe_detections.header = detection_array.header

        for detection in detection_array.detections:
            if self._is_detection_safe(detection):
                safe_detections.detections.append(detection)

        return safe_detections

    def _is_detection_safe(self, detection: Detection2D) -> bool:
        """Check if a detection is safe for interaction"""
        # Check object class
        if detection.results:
            for result in detection.results:
                class_id = result.hypothesis.class_id.lower()

                # Reject hazardous objects
                if class_id in self.hazardous_classes:
                    return False

                # Check confidence
                if result.hypothesis.score < self.perception_confidence_threshold:
                    return False

        # Check spatial safety (not in forbidden zones)
        if hasattr(detection, 'bbox'):
            object_center = self._get_bbox_center(detection.bbox)
            if self._is_in_forbidden_zone(object_center):
                return False

        return True

    def _check_hazardous_objects(self, detection_array) -> List[Detection2D]:
        """Check for hazardous object detections"""
        hazardous_detections = []

        for detection in detection_array.detections:
            if detection.results:
                for result in detection.results:
                    class_id = result.hypothesis.class_id.lower()
                    confidence = result.hypothesis.score

                    if (class_id in self.hazardous_classes and
                        confidence > self.perception_confidence_threshold):
                        hazardous_detections.append(detection)

                        # Log hazardous detection
                        self.get_logger().warn(
                            f'Hazardous object detected: {class_id} '
                            f'(confidence: {confidence:.3f})'
                        )

        return hazardous_detections

    def _is_in_forbidden_zone(self, position) -> bool:
        """Check if position is in a forbidden zone"""
        for zone in self.forbidden_zones:
            center = zone['center']
            radius = zone['radius']

            distance = np.sqrt(sum((pos - center_coord)**2 for pos, center_coord in zip(position, center)))
            if distance < radius:
                return True

        return False

    def _handle_hazardous_detections(self, hazardous_detections: List[Detection2D]):
        """Handle hazardous object detections"""
        for detection in hazardous_detections:
            if detection.results:
                result = detection.results[0]  # Take first result
                hazard_msg = String()
                hazard_msg.data = json.dumps({
                    'type': 'hazardous_object_detected',
                    'object_class': result.hypothesis.class_id,
                    'confidence': result.hypothesis.score,
                    'timestamp': self.get_clock().now().nanoseconds / 1e9
                })
                self.safety_alert_pub.publish(hazard_msg)

    def image_callback(self, msg):
        """Process image for safety-related features"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform safety-related image processing
            safety_features = self._extract_safety_features(cv_image)

            # Check for safety conditions in image
            self._check_image_safety(cv_image, safety_features)

        except Exception as e:
            self.get_logger().error(f'Image safety processing error: {e}')

    def _extract_safety_features(self, image):
        """Extract safety-related features from image"""
        features = {}

        # Check for fire detection (simplified - in practice, use ML model)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        fire_mask = cv2.inRange(hsv, (0, 50, 50), (20, 255, 255))  # Red-orange for fire
        fire_pixels = cv2.countNonZero(fire_mask)

        if fire_pixels > 100:  # Threshold for fire detection
            features['fire_detected'] = True
            features['fire_pixels'] = fire_pixels

        # Check for smoke detection (simplified)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        smoke_mask = cv2.inRange(gray, 180, 255)  # Bright regions (potential smoke)
        smoke_pixels = cv2.countNonZero(smoke_mask)

        if smoke_pixels > 500:  # Threshold for smoke detection
            features['smoke_detected'] = True
            features['smoke_pixels'] = smoke_pixels

        return features

    def _check_image_safety(self, image, features):
        """Check image for safety conditions"""
        if features.get('fire_detected'):
            alert_msg = String()
            alert_msg.data = json.dumps({
                'type': 'fire_detected',
                'pixels': features['fire_pixels'],
                'timestamp': self.get_clock().now().nanoseconds / 1e9
            })
            self.safety_alert_pub.publish(alert_msg)

            self.get_logger().error('FIRE DETECTED IN CAMERA VIEW!')

        if features.get('smoke_detected'):
            alert_msg = String()
            alert_msg.data = json.dumps({
                'type': 'smoke_detected',
                'pixels': features['smoke_pixels'],
                'timestamp': self.get_clock().now().nanoseconds / 1e9
            })
            self.safety_alert_pub.publish(alert_msg)

            self.get_logger().warn('SMOKE DETECTED IN CAMERA VIEW!')

    def pointcloud_callback(self, msg):
        """Process point cloud for safety-related geometric analysis"""
        try:
            # Convert point cloud to numpy array (simplified)
            # In practice, use sensor_msgs_py.point_cloud2.read_points
            points = self._pointcloud_to_array(msg)

            if len(points) > 0:
                # Check for safety-related geometric features
                self._check_pointcloud_safety(points)

        except Exception as e:
            self.get_logger().error(f'Point cloud safety processing error: {e}')

    def _pointcloud_to_array(self, pointcloud_msg):
        """Convert PointCloud2 to numpy array"""
        # This is a simplified conversion
        # In practice, use sensor_msgs_py.point_cloud2.read_points
        return np.random.rand(1000, 3).astype(np.float32)  # Placeholder

    def _check_pointcloud_safety(self, points):
        """Check point cloud for safety-related geometric features"""
        # Check for drop-off detection (cliff detection)
        if self._detect_drop_off(points):
            alert_msg = String()
            alert_msg.data = json.dumps({
                'type': 'drop_off_detected',
                'timestamp': self.get_clock().now().nanoseconds / 1e9
            })
            self.safety_alert_pub.publish(alert_msg)

            self.get_logger().warn('DROP-OFF DETECTED - AVOIDING AREA!')

        # Check for unstable surface detection
        if self._detect_unstable_surface(points):
            alert_msg = String()
            alert_msg.data = json.dumps({
                'type': 'unstable_surface_detected',
                'timestamp': self.get_clock().now().nanoseconds / 1e9
            })
            self.safety_alert_pub.publish(alert_msg)

            self.get_logger().warn('UNSTABLE SURFACE DETECTED - PROCEED WITH CAUTION!')

    def _detect_drop_off(self, points, threshold=0.3):
        """Detect potential drop-offs/cliffs in point cloud"""
        # Simplified drop-off detection
        # In practice, use more sophisticated geometric analysis
        z_values = points[:, 2]
        z_sorted = np.sort(z_values)

        # Check for significant height differences in nearby points
        if len(z_sorted) > 100:
            # Look for large gaps in z-values
            gaps = np.diff(z_sorted)
            max_gap = np.max(gaps)

            return max_gap > threshold

        return False

    def _detect_unstable_surface(self, points, variance_threshold=0.01):
        """Detect unstable surfaces based on point distribution"""
        # Check for high variance in surface normals or point density
        # This is a simplified check - real implementation would be more complex
        if len(points) > 100:
            # Calculate local point density variations
            # This would involve more sophisticated geometric analysis
            z_variance = np.var(points[:, 2])
            return z_variance > variance_threshold

        return False

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSafetyNode()

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

## Planning Safety

### Safe Path Planning and Validation

```python
# planning_safety.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.spatial import KDTree
import json

class PlanningSafetyNode(Node):
    def __init__(self):
        super().__init__('planning_safety')

        # Initialize safety parameters
        self._initialize_planning_safety()

        # Subscribers
        self.plan_sub = self.create_subscription(
            Path,
            '/global_plan',
            self.plan_callback,
            10
        )

        self.local_plan_sub = self.create_subscription(
            Path,
            '/local_plan',
            self.local_plan_callback,
            10
        )

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap',
            self.costmap_callback,
            10
        )

        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap',
            self.local_costmap_callback,
            10
        )

        # Publishers
        self.safe_plan_pub = self.create_publisher(Path, '/safe_plan', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/safety_viz', 10)
        self.plan_safety_pub = self.create_publisher(Bool, '/plan_safe', 10)

        # Planning safety state
        self.global_costmap = None
        self.local_costmap = None
        self.safety_threshold = 50  # Cost threshold for safety (0-100 scale)

        self.get_logger().info('Planning Safety Node initialized')

    def _initialize_planning_safety(self):
        """Initialize planning safety parameters"""
        # Safety parameters
        self.collision_inflation_radius = 0.5  # 50cm safety buffer
        self.dynamic_obstacle_buffer = 0.8    # 80cm buffer for moving objects
        self.stability_threshold = 20         # Cost threshold for stable terrain
        self.narrow_passage_width = 0.6       # Minimum safe passage width (1.2m total)

        # Risk assessment weights
        self.risk_weights = {
            'collision_risk': 0.4,
            'stability_risk': 0.3,
            'visibility_risk': 0.2,
            'complexity_risk': 0.1
        }

        self.get_logger().info('Planning safety parameters initialized')

    def plan_callback(self, msg):
        """Validate global plan for safety"""
        try:
            # Validate plan safety
            is_safe, safety_report = self._validate_plan_safety(msg)

            # Publish safety status
            safety_msg = Bool()
            safety_msg.data = is_safe
            self.plan_safety_pub.publish(safety_msg)

            if not is_safe:
                self.get_logger().warn(f'Unsafe plan detected: {safety_report}')

                # Generate safe alternative if possible
                safe_plan = self._generate_safe_alternative(msg)
                if safe_plan:
                    self.safe_plan_pub.publish(safe_plan)

            else:
                # Plan is safe, publish as-is
                self.safe_plan_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Plan safety validation error: {e}')

    def local_plan_callback(self, msg):
        """Validate local plan for immediate safety"""
        try:
            # Local plan validation (more stringent)
            is_safe, safety_report = self._validate_local_plan_safety(msg)

            # Publish safety status
            safety_msg = Bool()
            safety_msg.data = is_safe
            self.plan_safety_pub.publish(safety_msg)

            if not is_safe:
                self.get_logger().error(f'Unsafe local plan: {safety_report}')

                # Emergency stop if local plan is unsafe
                emergency_msg = String()
                emergency_msg.data = json.dumps({
                    'type': 'unsafe_local_plan',
                    'plan_points': len(msg.poses),
                    'risk_factors': safety_report.get('risk_factors', []),
                    'timestamp': self.get_clock().now().nanoseconds / 1e9
                })

        except Exception as e:
            self.get_logger().error(f'Local plan safety validation error: {e}')

    def _validate_plan_safety(self, path: Path) -> tuple[bool, dict]:
        """Validate plan safety comprehensively"""
        if not path.poses:
            return True, {'status': 'empty_plan'}

        safety_report = {
            'total_points': len(path.poses),
            'collision_risk': 0.0,
            'stability_risk': 0.0,
            'visibility_risk': 0.0,
            'complexity_risk': 0.0,
            'overall_safety': 0.0,
            'risk_factors': []
        }

        collision_risks = []
        stability_risks = []
        visibility_risks = []
        complexity_risks = []

        # Check each point in the path
        for i, pose_stamped in enumerate(path.poses):
            pose = pose_stamped.pose

            # Check collision risk at this point
            collision_risk = self._assess_collision_risk(pose, i)
            collision_risks.append(collision_risk)

            # Check terrain stability
            stability_risk = self._assess_stability_risk(pose, i)
            stability_risks.append(stability_risk)

            # Check visibility/observability
            visibility_risk = self._assess_visibility_risk(pose, i)
            visibility_risks.append(visibility_risk)

            # Check path complexity
            complexity_risk = self._assess_complexity_risk(path.poses, i)
            complexity_risks.append(complexity_risk)

        # Calculate average risks
        avg_collision_risk = sum(collision_risks) / len(collision_risks) if collision_risks else 0.0
        avg_stability_risk = sum(stability_risks) / len(stability_risks) if stability_risks else 0.0
        avg_visibility_risk = sum(visibility_risks) / len(visibility_risks) if visibility_risks else 0.0
        avg_complexity_risk = sum(complexity_risks) / len(complexity_risks) if complexity_risks else 0.0

        # Weighted overall safety score
        overall_safety = (
            (1 - avg_collision_risk) * self.risk_weights['collision_risk'] +
            (1 - avg_stability_risk) * self.risk_weights['stability_risk'] +
            (1 - avg_visibility_risk) * self.risk_weights['visibility_risk'] +
            (1 - avg_complexity_risk) * self.risk_weights['complexity_risk']
        )

        safety_report.update({
            'collision_risk': avg_collision_risk,
            'stability_risk': avg_stability_risk,
            'visibility_risk': avg_visibility_risk,
            'complexity_risk': avg_complexity_risk,
            'overall_safety': overall_safety
        })

        # Check for critical risks
        if avg_collision_risk > 0.8:
            safety_report['risk_factors'].append('HIGH_COLLISION_RISK')
        if avg_stability_risk > 0.7:
            safety_report['risk_factors'].append('LOW_STABILITY_TERRAIN')
        if avg_visibility_risk > 0.6:
            safety_report['risk_factors'].append('POOR_VISIBILITY')
        if avg_complexity_risk > 0.7:
            safety_report['risk_factors'].append('COMPLEX_MANEUVERS_REQUIRED')

        is_safe = overall_safety > 0.5  # Require at least 50% safety score

        return is_safe, safety_report

    def _assess_collision_risk(self, pose, point_index) -> float:
        """Assess collision risk at a specific pose"""
        if not self.global_costmap:
            return 0.0  # Unknown risk if no costmap

        try:
            # Convert world coordinates to costmap coordinates
            world_x = pose.position.x
            world_y = pose.position.y

            map_x = int((world_x - self.global_costmap.info.origin.position.x) / self.global_costmap.info.resolution)
            map_y = int((world_y - self.global_costmap.info.origin.position.y) / self.global_costmap.info.resolution)

            # Check if coordinates are valid
            if (0 <= map_x < self.global_costmap.info.width and
                0 <= map_y < self.global_costmap.info.height):

                # Get cost from costmap
                cost_index = map_y * self.global_costmap.info.width + map_x
                cost = self.global_costmap.data[cost_index]

                # Normalize cost to [0, 1] risk scale
                # Cost of 254 = lethal obstacle, 0 = free space
                if cost >= 253:  # Lethal obstacle
                    return 1.0
                elif cost >= self.safety_threshold:
                    # Scale risk based on cost
                    return min(1.0, (cost - self.safety_threshold) / (253 - self.safety_threshold))
                else:
                    return 0.0

            else:
                return 1.0  # Outside map bounds = high risk

        except Exception as e:
            self.get_logger().error(f'Collision risk assessment error: {e}')
            return 0.5  # Default medium risk

    def _assess_stability_risk(self, pose, point_index) -> float:
        """Assess terrain stability risk"""
        # Check costmap for unstable terrain indicators
        # In practice, this would look for specific terrain types
        if not self.global_costmap:
            return 0.0

        try:
            world_x = pose.position.x
            world_y = pose.position.y

            map_x = int((world_x - self.global_costmap.info.origin.position.x) / self.global_costmap.info.resolution)
            map_y = int((world_y - self.global_costmap.info.origin.position.y) / self.global_costmap.info.resolution)

            if (0 <= map_x < self.global_costmap.info.width and
                0 <= map_y < self.global_costmap.info.height):

                cost_index = map_y * self.global_costmap.info.width + map_x
                cost = self.global_costmap.data[cost_index]

                # High costs might indicate unstable terrain (holes, obstacles, etc.)
                if cost > self.stability_threshold:
                    return min(1.0, cost / 100.0)
                else:
                    return 0.0

            else:
                return 1.0  # Outside bounds = unstable

        except Exception as e:
            self.get_logger().error(f'Stability risk assessment error: {e}')
            return 0.0

    def _assess_visibility_risk(self, pose, point_index) -> float:
        """Assess visibility/observability risk"""
        # In a real system, this would check:
        # - Lighting conditions
        # - Sensor coverage
        # - Occlusion from obstacles
        # - Visibility from robot perspective

        # For now, return based on costmap density (more obstacles = less visibility)
        if not self.global_costmap:
            return 0.5  # Unknown visibility

        try:
            # Check surrounding area for obstacles
            world_x = pose.position.x
            world_y = pose.position.y

            map_x = int((world_x - self.global_costmap.info.origin.position.x) / self.global_costmap.info.resolution)
            map_y = int((world_y - self.global_costmap.info.origin.position.y) / self.global_costmap.info.resolution)

            if (0 <= map_x < self.global_costmap.info.width and
                0 <= map_y < self.global_costmap.info.height):

                # Check 3x3 neighborhood for obstacle density
                neighborhood_size = 3
                obstacle_count = 0
                total_cells = 0

                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        nx, ny = map_x + dx, map_y + dy
                        if (0 <= nx < self.global_costmap.info.width and
                            0 <= ny < self.global_costmap.info.height):
                            cost_idx = ny * self.global_costmap.info.width + nx
                            cost = self.global_costmap.data[cost_idx]
                            if cost >= self.safety_threshold:
                                obstacle_count += 1
                            total_cells += 1

                if total_cells > 0:
                    obstacle_density = obstacle_count / total_cells
                    return min(1.0, obstacle_density * 2.0)  # Amplify for visibility risk

            return 0.0

        except Exception as e:
            self.get_logger().error(f'Visibility risk assessment error: {e}')
            return 0.0

    def _assess_complexity_risk(self, path_poses, point_index) -> float:
        """Assess path complexity risk"""
        if len(path_poses) < 2:
            return 0.0

        try:
            # Calculate path curvature and complexity
            if point_index > 0 and point_index < len(path_poses) - 1:
                prev_pose = path_poses[point_index - 1].pose
                curr_pose = path_poses[point_index].pose
                next_pose = path_poses[point_index + 1].pose

                # Calculate direction changes
                vec1 = np.array([
                    curr_pose.position.x - prev_pose.position.x,
                    curr_pose.position.y - prev_pose.position.y
                ])
                vec2 = np.array([
                    next_pose.position.x - curr_pose.position.x,
                    next_pose.position.y - curr_pose.position.y
                ])

                if np.linalg.norm(vec1) > 0 and np.linalg.norm(vec2) > 0:
                    # Calculate angle between consecutive segments
                    cos_angle = np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
                    angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))

                    # High curvature = high complexity risk
                    max_curve = np.pi  # 180 degrees
                    curve_risk = angle / max_curve

                    return min(1.0, curve_risk)

            return 0.0

        except Exception as e:
            self.get_logger().error(f'Complexity risk assessment error: {e}')
            return 0.0

    def _generate_safe_alternative(self, original_path: Path) -> Optional[Path]:
        """Generate safe alternative path if original is unsafe"""
        # This would implement path replanning with safety constraints
        # For now, return None to indicate no safe alternative found
        # In practice, this would call a replanning service

        self.get_logger().info('Generating safe alternative path...')

        # Create a service call to replanner with safety constraints
        try:
            # This would be a service call to the path planner
            # with additional safety constraints
            safe_path = self._call_safe_replanner(original_path)
            return safe_path

        except Exception as e:
            self.get_logger().error(f'Alternative path generation error: {e}')
            return None

    def _call_safe_replanner(self, original_path: Path):
        """Call safe path replanner service"""
        # In a real implementation, this would call a service
        # that generates a new path with safety constraints
        # For now, return None
        return None

def main(args=None):
    rclpy.init(args=args)
    node = PlanningSafetyNode()

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

## Execution Safety

### Real-time Monitoring and Emergency Response

```python
# execution_safety.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Bool, String, Float32
from builtin_interfaces.msg import Duration
import numpy as np
from collections import deque
import threading
import time

class ExecutionSafetyNode(Node):
    def __init__(self):
        super().__init__('execution_safety')

        # Initialize safety monitoring
        self._initialize_execution_safety()

        # Subscribers for execution monitoring
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.odom_callback,
            10
        )

        # Publishers
        self.safety_cmd_pub = self.create_publisher(Twist, '/safety_cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.safety_status_pub = self.create_publisher(String, '/execution_safety_status', 10)

        # Safety monitoring timer
        self.safety_timer = self.create_timer(0.01, self.safety_monitoring_loop)  # 100Hz

        # Execution safety state
        self.current_cmd_vel = Twist()
        self.current_joint_states = JointState()
        self.current_imu_data = Imu()
        self.current_pose = PoseStamped()

        self.joint_velocities = deque(maxlen=10)  # Last 10 velocity readings
        self.acceleration_buffer = deque(maxlen=10)
        self.balance_buffer = deque(maxlen=20)

        self.emergency_stop_active = False
        self.last_safe_cmd = Twist()

        self.get_logger().info('Execution Safety Node initialized')

    def _initialize_execution_safety(self):
        """Initialize execution safety parameters"""
        # Velocity limits
        self.max_linear_vel = 0.5  # m/s
        self.max_angular_vel = 1.0  # rad/s
        self.max_joint_vel = 3.0    # rad/s

        # Acceleration limits
        self.max_linear_accel = 1.0  # m/s²
        self.max_angular_accel = 2.0  # rad/s²

        # Balance thresholds
        self.roll_threshold = 0.3  # radians (~17 degrees)
        self.pitch_threshold = 0.4  # radians (~23 degrees)
        self.zmp_threshold = 0.15   # meters (stability margin)

        # Emergency stop conditions
        self.enable_emergency_stop = True
        self.emergency_stop_reasons = []

        self.get_logger().info('Execution safety parameters initialized')

    def cmd_vel_callback(self, msg):
        """Monitor velocity commands for safety"""
        self.current_cmd_vel = msg

        # Check if command is safe
        if not self._is_cmd_vel_safe(msg):
            self.get_logger().warn('Unsafe velocity command received')
            # Apply safe command instead
            safe_cmd = self._get_safe_cmd_vel(msg)
            self.safety_cmd_pub.publish(safe_cmd)
        else:
            # Command is safe, forward it
            self.safety_cmd_pub.publish(msg)

    def joint_state_callback(self, msg):
        """Monitor joint states during execution"""
        self.current_joint_states = msg

        # Monitor joint safety
        joint_safety_ok = self._check_joint_safety(msg)

        if not joint_safety_ok:
            self._activate_emergency_stop('Joint safety violation')

        # Update velocity tracking
        if msg.velocity:
            self.joint_velocities.append({
                'timestamp': time.time(),
                'velocities': list(msg.velocity)
            })

    def imu_callback(self, msg):
        """Monitor IMU data for balance and stability"""
        self.current_imu_data = msg

        # Check balance and stability
        balance_ok = self._check_balance_stability(msg)

        if not balance_ok:
            self._activate_emergency_stop('Balance/stability violation')

        # Update balance tracking
        self.balance_buffer.append({
            'timestamp': time.time(),
            'orientation': (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
            'angular_velocity': (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            'linear_acceleration': (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        })

    def odom_callback(self, msg):
        """Monitor robot pose for navigation safety"""
        self.current_pose = msg

        # Check for navigation safety violations
        nav_safety_ok = self._check_navigation_safety(msg)

        if not nav_safety_ok:
            self._activate_emergency_stop('Navigation safety violation')

    def safety_monitoring_loop(self):
        """Continuous execution safety monitoring"""
        if self.emergency_stop_active:
            return

        try:
            # Check velocity limits
            self._check_velocity_limits()

            # Check acceleration limits
            self._check_acceleration_limits()

            # Check balance stability
            self._check_dynamic_balance()

            # Check for joint limit violations during execution
            self._check_joint_limits_during_execution()

            # Check for obstacle proximity
            self._check_obstacle_proximity()

        except Exception as e:
            self.get_logger().error(f'Execution safety monitoring error: {e}')

    def _is_cmd_vel_safe(self, cmd_vel: Twist) -> bool:
        """Check if velocity command is safe"""
        # Check linear velocity limits
        linear_speed = np.sqrt(cmd_vel.linear.x**2 + cmd_vel.linear.y**2 + cmd_vel.linear.z**2)
        if linear_speed > self.max_linear_vel:
            return False

        # Check angular velocity limits
        angular_speed = np.sqrt(cmd_vel.angular.x**2 + cmd_vel.angular.y**2 + cmd_vel.angular.z**2)
        if angular_speed > self.max_angular_vel:
            return False

        return True

    def _get_safe_cmd_vel(self, unsafe_cmd: Twist) -> Twist:
        """Generate safe command from unsafe command"""
        safe_cmd = Twist()

        # Limit linear velocity
        linear_speed = np.sqrt(unsafe_cmd.linear.x**2 + unsafe_cmd.linear.y**2 + unsafe_cmd.linear.z**2)
        if linear_speed > self.max_linear_vel:
            scale = self.max_linear_vel / linear_speed
            safe_cmd.linear.x = unsafe_cmd.linear.x * scale
            safe_cmd.linear.y = unsafe_cmd.linear.y * scale
            safe_cmd.linear.z = unsafe_cmd.linear.z * scale
        else:
            safe_cmd.linear = unsafe_cmd.linear

        # Limit angular velocity
        angular_speed = np.sqrt(unsafe_cmd.angular.x**2 + unsafe_cmd.angular.y**2 + unsafe_cmd.angular.z**2)
        if angular_speed > self.max_angular_vel:
            scale = self.max_angular_vel / angular_speed
            safe_cmd.angular.x = unsafe_cmd.angular.x * scale
            safe_cmd.angular.y = unsafe_cmd.angular.y * scale
            safe_cmd.angular.z = unsafe_cmd.angular.z * scale
        else:
            safe_cmd.angular = unsafe_cmd.angular

        return safe_cmd

    def _check_joint_safety(self, joint_state: JointState) -> bool:
        """Check joint states for safety violations"""
        if not joint_state.position or not joint_state.velocity:
            return True

        for i, name in enumerate(joint_state.name):
            if i < len(joint_state.velocity):
                velocity = abs(joint_state.velocity[i])
                if velocity > self.max_joint_vel:
                    self.get_logger().warn(f'Joint {name} velocity violation: {velocity:.3f} (max: {self.max_joint_vel:.3f})')
                    return False

        return True

    def _check_balance_stability(self, imu_msg: Imu) -> bool:
        """Check robot balance and stability from IMU data"""
        try:
            # Convert quaternion to Euler angles
            import math
            q = imu_msg.orientation
            sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
            cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
            roll = math.atan2(sinr_cosp, cosr_cosp)

            sinp = 2 * (q.w * q.y - q.z * q.x)
            pitch = math.asin(sinp)

            # Check if robot is within balance thresholds
            if abs(roll) > self.roll_threshold:
                self.get_logger().warn(f'Roll angle violation: {roll:.3f} (threshold: {self.roll_threshold:.3f})')
                return False

            if abs(pitch) > self.pitch_threshold:
                self.get_logger().warn(f'Pitch angle violation: {pitch:.3f} (threshold: {self.pitch_threshold:.3f})')
                return False

            return True

        except Exception as e:
            self.get_logger().error(f'Balance check error: {e}')
            return False

    def _check_velocity_limits(self):
        """Check current velocity limits"""
        # This would compare current velocities with commanded velocities
        # and check for excessive speeds
        pass

    def _check_acceleration_limits(self):
        """Check acceleration limits using velocity history"""
        if len(self.joint_velocities) >= 2:
            recent_data = list(self.joint_velocities)[-2:]
            if len(recent_data) == 2:
                dt = recent_data[1]['timestamp'] - recent_data[0]['timestamp']
                if dt > 0:
                    for i in range(min(len(recent_data[0]['velocities']), len(recent_data[1]['velocities']))):
                        vel1 = recent_data[0]['velocities'][i]
                        vel2 = recent_data[1]['velocities'][i]
                        accel = abs(vel2 - vel1) / dt

                        if accel > self.max_joint_accel:
                            self.get_logger().warn(f'Joint acceleration violation for joint {i}: {accel:.3f}')
                            self._activate_emergency_stop(f'Joint acceleration violation: {accel:.3f}')

    def _check_dynamic_balance(self):
        """Check dynamic balance during movement"""
        if len(self.balance_buffer) < 2:
            return

        # Analyze recent balance data for dynamic stability
        recent_balance = list(self.balance_buffer)[-10:]  # Last 10 readings

        # Calculate balance metrics
        roll_changes = []
        pitch_changes = []

        for i in range(1, len(recent_balance)):
            prev_orient = recent_balance[i-1]['orientation']
            curr_orient = recent_balance[i]['orientation']

            # Convert quaternions to Euler angles for change calculation
            prev_roll = self._quaternion_to_roll_pitch_yaw(prev_orient)[0]
            curr_roll = self._quaternion_to_roll_pitch_yaw(curr_orient)[0]
            prev_pitch = self._quaternion_to_roll_pitch_yaw(prev_orient)[1]
            curr_pitch = self._quaternion_to_roll_pitch_yaw(curr_orient)[1]

            roll_changes.append(abs(curr_roll - prev_roll))
            pitch_changes.append(abs(curr_pitch - prev_pitch))

        # Check for excessive balance changes
        if roll_changes and max(roll_changes) > 0.1:  # 0.1 rad/s excessive change
            self.get_logger().warn(f'Rapid balance change detected: roll change = {max(roll_changes):.3f}')
            self._activate_emergency_stop('Rapid balance change')

        if pitch_changes and max(pitch_changes) > 0.1:
            self.get_logger().warn(f'Rapid balance change detected: pitch change = {max(pitch_changes):.3f}')
            self._activate_emergency_stop('Rapid balance change')

    def _quaternion_to_roll_pitch_yaw(self, quaternion):
        """Convert quaternion to roll, pitch, yaw"""
        import math
        x, y, z, w = quaternion

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _check_joint_limits_during_execution(self):
        """Check joint limits during execution"""
        # This would monitor actual joint positions during movement
        # and compare with limits from URDF
        pass

    def _check_obstacle_proximity(self):
        """Check for obstacle proximity during execution"""
        # This would interface with distance sensors or costmap
        # to detect imminent collisions
        pass

    def _activate_emergency_stop(self, reason: str):
        """Activate emergency stop for execution safety"""
        if self.enable_emergency_stop and not self.emergency_stop_active:
            self.emergency_stop_active = True
            self.emergency_stop_reasons.append(reason)

            # Stop all robot motion
            stop_cmd = Twist()
            self.safety_cmd_pub.publish(stop_cmd)

            # Publish emergency stop signal
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_pub.publish(stop_msg)

            # Publish safety status
            status_msg = String()
            status_msg.data = json.dumps({
                'type': 'emergency_stop',
                'reason': reason,
                'timestamp': self.get_clock().now().nanoseconds / 1e9,
                'active_reasons': self.emergency_stop_reasons
            })
            self.safety_status_pub.publish(status_msg)

            self.get_logger().fatal(f'EMERGENCY STOP ACTIVATED: {reason}')

    def _deactivate_emergency_stop(self):
        """Deactivate emergency stop (with safety checks)"""
        if self.emergency_stop_active:
            # Perform safety checks before deactivation
            if self._can_safely_resume():
                self.emergency_stop_active = False
                self.get_logger().info('Emergency stop deactivated, resuming operations')

                # Clear reasons
                self.emergency_stop_reasons.clear()

                # Publish resume signal
                stop_msg = Bool()
                stop_msg.data = False
                self.emergency_stop_pub.publish(stop_msg)
            else:
                self.get_logger().warn('Cannot resume - safety conditions not met')

    def _can_safely_resume(self) -> bool:
        """Check if robot can safely resume operations"""
        # Check that current state is safe
        # Check that causes of emergency stop are resolved
        # Check that environment is safe

        # For now, just return True if no current safety violations
        # In practice, this would be much more comprehensive
        return True

def main(args=None):
    rclpy.init(args=args)
    node = ExecutionSafetyNode()

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

## Behavioral Safety

### Ethical and Social Safety Guidelines

```python
# behavioral_safety.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import json
import numpy as np

class BehavioralSafetyNode(Node):
    def __init__(self):
        super().__init__('behavioral_safety')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize behavioral safety
        self._initialize_behavioral_safety()

        # Subscribers
        self.person_detection_sub = self.create_subscription(
            Detection2DArray,
            '/person_detections',
            self.person_detection_callback,
            10
        )

        self.command_sub = self.create_subscription(
            String,
            '/behavioral_command',
            self.command_callback,
            10
        )

        # Publishers
        self.safety_command_pub = self.create_publisher(String, '/safe_behavioral_command', 10)
        self.ethics_violation_pub = self.create_publisher(String, '/ethics_violation', 10)

        # Behavioral safety state
        self.personal_space_violations = []
        self.ethical_guidelines = []
        self.social_norms = []

        self.get_logger().info('Behavioral Safety Node initialized')

    def _initialize_behavioral_safety(self):
        """Initialize behavioral safety parameters"""
        # Personal space parameters
        self.personal_space_radius = {
            'intimate': 0.45,    # 45cm - very close interaction
            'personal': 1.2,     # 1.2m - personal conversations
            'social': 3.6,       # 3.6m - social interactions
            'public': 7.6        # 7.6m+ - public speaking distance
        }

        # Privacy zones
        self.privacy_zones = [
            {'type': 'bathroom', 'center': [0, 0, 0], 'radius': 2.0},
            {'type': 'bedroom', 'center': [2, 1, 0], 'radius': 2.5},
            {'type': 'office_private', 'center': [-1, 2, 0], 'radius': 1.5}
        ]

        # Ethical guidelines
        self.ethical_guidelines = [
            'Do not enter restricted areas',
            'Respect personal space',
            'Protect privacy',
            'Avoid harmful actions',
            'Follow social norms',
            'Prioritize human safety'
        ]

        # Social behavior rules
        self.social_norms = {
            'eye_contact': True,
            'respectful_distance': True,
            'non_threatening_posture': True,
            'appropriate_speed': True
        }

        self.get_logger().info('Behavioral safety parameters initialized')

    def person_detection_callback(self, msg):
        """Process person detections for behavioral safety"""
        try:
            for detection in msg.detections:
                if detection.results:
                    # Check if this is a person detection
                    for result in detection.results:
                        if result.hypothesis.class_id.lower() in ['person', 'human', 'pedestrian']:
                            # Calculate distance to person
                            person_pose = self._get_detection_pose(detection)
                            distance_to_person = self._calculate_distance_to_robot(person_pose)

                            # Check personal space violation
                            if distance_to_person < self.personal_space_radius['personal']:
                                self._handle_personal_space_violation(
                                    person_pose, distance_to_person, result.hypothesis.score
                                )

        except Exception as e:
            self.get_logger().error(f'Person detection safety processing error: {e}')

    def _get_detection_pose(self, detection):
        """Extract pose from detection message"""
        # Simplified - in practice, this would extract proper pose information
        return Point(
            x=detection.bbox.center.x,
            y=detection.bbox.center.y,
            z=0.0  # Assume ground level for now
        )

    def _calculate_distance_to_robot(self, person_pose):
        """Calculate distance from robot to detected person"""
        # In practice, this would get robot position from TF or odometry
        robot_pose = Point(x=0.0, y=0.0, z=0.0)  # Placeholder

        distance = np.sqrt(
            (person_pose.x - robot_pose.x)**2 +
            (person_pose.y - robot_pose.y)**2 +
            (person_pose.z - robot_pose.z)**2
        )

        return distance

    def _handle_personal_space_violation(self, person_pose, distance, confidence):
        """Handle personal space violation"""
        violation_msg = String()
        violation_msg.data = json.dumps({
            'type': 'personal_space_violation',
            'person_position': [person_pose.x, person_pose.y, person_pose.z],
            'distance': distance,
            'confidence': confidence,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        })

        self.ethics_violation_pub.publish(violation_msg)
        self.get_logger().warn(f'Personal space violation: {distance:.2f}m from person')

        # Log violation for behavioral analysis
        self.personal_space_violations.append({
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'distance': distance,
            'position': [person_pose.x, person_pose.y, person_pose.z]
        })

    def command_callback(self, msg):
        """Process behavioral commands for safety"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('type', '')
            command_params = command_data.get('parameters', {})

            # Check if command violates ethical guidelines
            if self._command_violates_ethics(command_type, command_params):
                self.get_logger().error(f'Ethical violation in command: {command_type}')

                # Generate safe alternative behavior
                safe_command = self._generate_safe_behavior(command_type, command_params)
                if safe_command:
                    safe_msg = String()
                    safe_msg.data = json.dumps(safe_command)
                    self.safety_command_pub.publish(safe_msg)

            else:
                # Command is ethical, forward it
                self.safety_command_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Command safety processing error: {e}')

    def _command_violates_ethics(self, command_type: str, parameters: dict) -> bool:
        """Check if command violates ethical guidelines"""
        # Check for movement commands to restricted areas
        if command_type in ['move_to', 'navigate_to', 'go_to']:
            target_location = parameters.get('location', [0, 0, 0])
            if self._is_in_restricted_area(target_location):
                return True

        # Check for inappropriate interaction commands
        if command_type in ['approach_person', 'follow_person']:
            if parameters.get('distance', 1.0) < self.personal_space_radius['personal']:
                return True

        # Check for privacy-violating commands
        if command_type in ['record_video', 'take_photo']:
            if self._is_in_privacy_zone(parameters.get('target_location', [0, 0, 0])):
                return True

        return False

    def _is_in_restricted_area(self, location) -> bool:
        """Check if location is in restricted area"""
        # In practice, this would check against defined restricted areas
        # For now, return False (no restrictions)
        return False

    def _is_in_privacy_zone(self, location) -> bool:
        """Check if location is in privacy zone"""
        x, y, z = location

        for zone in self.privacy_zones:
            center_x, center_y, center_z = zone['center']
            radius = zone['radius']

            distance = np.sqrt((x - center_x)**2 + (y - center_y)**2 + (z - center_z)**2)
            if distance < radius:
                return True

        return False

    def _generate_safe_behavior(self, original_command, parameters):
        """Generate safe alternative behavior"""
        safe_command = {
            'type': 'safe_behavior',
            'original_command': original_command,
            'safe_alternative': 'wait_for_clearance',
            'parameters': parameters,
            'reason': 'ethical_guideline_violation',
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }

        return safe_command

    def _monitor_social_behavior(self):
        """Monitor robot's social behavior for compliance"""
        # This would monitor actual robot behavior against social norms
        # For example: checking if robot maintains appropriate distance,
        # moves at appropriate speeds near people, etc.
        pass

def main(args=None):
    rclpy.init(args=args)
    node = BehavioralSafetyNode()

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

## Safety System Integration

### Comprehensive Safety Manager

```python
# safety_system_integration.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
import json
from typing import Dict, List

class SafetySystemManager(Node):
    def __init__(self):
        super().__init__('safety_system_manager')

        # Initialize safety subsystems
        self._initialize_safety_subsystems()

        # Publishers
        self.system_safety_pub = self.create_publisher(Bool, '/system_safety_status', 10)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/safety_diagnostics', 10)
        self.safety_report_pub = self.create_publisher(String, '/safety_report', 10)

        # Subscribers for safety status from various subsystems
        self.hardware_safety_sub = self.create_subscription(
            String, '/hardware_safety_status', self.hardware_safety_callback, 10)
        self.perception_safety_sub = self.create_subscription(
            Bool, '/perception_safe', self.perception_safety_callback, 10)
        self.planning_safety_sub = self.create_subscription(
            Bool, '/plan_safe', self.planning_safety_callback, 10)
        self.execution_safety_sub = self.create_subscription(
            String, '/execution_safety_status', self.execution_safety_callback, 10)

        # Timer for system-wide safety monitoring
        self.safety_monitor_timer = self.create_timer(0.1, self.system_safety_monitor)

        # Safety state tracking
        self.safety_subsystems = {
            'hardware': True,
            'perception': True,
            'planning': True,
            'execution': True,
            'behavioral': True
        }

        self.safety_violations = []
        self.emergency_stop_active = False

        self.get_logger().info('Safety System Manager initialized')

    def _initialize_safety_subsystems(self):
        """Initialize safety subsystem monitoring"""
        # Initialize connections to safety subsystems
        # Set up monitoring for each safety layer
        self.get_logger().info('Safety subsystems initialized')

    def hardware_safety_callback(self, msg):
        """Update hardware safety status"""
        try:
            status_data = json.loads(msg.data)
            is_safe = not status_data.get('emergency_stop_active', False)
            self.safety_subsystems['hardware'] = is_safe

            # Log any violations
            violations = status_data.get('violations', [])
            if violations:
                self.safety_violations.extend(violations)

        except Exception as e:
            self.get_logger().error(f'Hardware safety status parsing error: {e}')

    def perception_safety_callback(self, msg):
        """Update perception safety status"""
        self.safety_subsystems['perception'] = msg.data

    def planning_safety_callback(self, msg):
        """Update planning safety status"""
        self.safety_subsystems['planning'] = msg.data

    def execution_safety_callback(self, msg):
        """Update execution safety status"""
        try:
            status_data = json.loads(msg.data)
            is_safe = not status_data.get('emergency_stop_active', False)
            self.safety_subsystems['execution'] = is_safe

            # Log violations
            if status_data.get('violations'):
                self.safety_violations.extend(status_data['violations'])

        except Exception as e:
            self.get_logger().error(f'Execution safety status parsing error: {e}')

    def system_safety_monitor(self):
        """Monitor overall system safety status"""
        try:
            # Calculate overall system safety
            all_safe = all(self.safety_subsystems.values())

            # Check for emergency conditions
            if not all_safe:
                self._handle_system_safety_violation()

            # Publish system safety status
            safety_msg = Bool()
            safety_msg.data = all_safe
            self.system_safety_pub.publish(safety_msg)

            # Publish diagnostic information
            self._publish_diagnostics()

            # Publish safety report periodically
            if len(self.safety_violations) > 0:
                self._publish_safety_report()

        except Exception as e:
            self.get_logger().error(f'System safety monitoring error: {e}')

    def _handle_system_safety_violation(self):
        """Handle system-wide safety violation"""
        unsafe_subsystems = [name for name, safe in self.safety_subsystems.items() if not safe]

        if unsafe_subsystems:
            self.get_logger().error(f'System safety violation in: {", ".join(unsafe_subsystems)}')

            # Activate system-wide emergency stop if critical
            if self._has_critical_safety_violation(unsafe_subsystems):
                self._activate_system_emergency_stop(unsafe_subsystems)

    def _has_critical_safety_violation(self, unsafe_subsystems: List[str]) -> bool:
        """Check if there are critical safety violations"""
        critical_systems = ['hardware', 'execution']  # These are critical for safety
        return any(sys in critical_systems for sys in unsafe_subsystems)

    def _activate_system_emergency_stop(self, violating_systems: List[str]):
        """Activate system-wide emergency stop"""
        if not self.emergency_stop_active:
            self.emergency_stop_active = True

            # Stop all robot motion
            stop_cmd = Twist()
            # This would publish to the main command topic
            # stop_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            # stop_cmd_pub.publish(stop_cmd)

            self.get_logger().fatal(f'SYSTEM EMERGENCY STOP: Safety violations in {", ".join(violating_systems)}')

    def _publish_diagnostics(self):
        """Publish safety diagnostics"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        for subsystem, is_safe in self.safety_subsystems.items():
            diag_status = DiagnosticStatus()
            diag_status.name = f'Safety_{subsystem}'
            diag_status.level = DiagnosticStatus.OK if is_safe else DiagnosticStatus.ERROR
            diag_status.message = 'Safe' if is_safe else 'Safety violation detected'

            # Add additional information
            if not is_safe:
                diag_status.message += f' - Violations: {len(self.safety_violations)}'

            diag_array.status.append(diag_status)

        self.diagnostic_pub.publish(diag_array)

    def _publish_safety_report(self):
        """Publish comprehensive safety report"""
        safety_report = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'subsystem_statuses': self.safety_subsystems,
            'total_violations': len(self.safety_violations),
            'recent_violations': self.safety_violations[-10:],  # Last 10 violations
            'emergency_stop_active': self.emergency_stop_active,
            'system_overall_safety': all(self.safety_subsystems.values())
        }

        report_msg = String()
        report_msg.data = json.dumps(safety_report, indent=2)
        self.safety_report_pub.publish(report_msg)

        # Clear processed violations
        self.safety_violations.clear()

def main(args=None):
    rclpy.init(args=args)
    node = SafetySystemManager()

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

## Safety Testing and Validation

### Safety System Testing Framework

```python
# safety_testing.py
import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import time

class SafetySystemTester(Node):
    def __init__(self):
        super().__init__('safety_system_tester')

        # Test publishers to trigger safety conditions
        self.test_cmd_pub = self.create_publisher(Twist, '/test_cmd_vel', 10)
        self.test_joint_pub = self.create_publisher(JointState, '/test_joint_states', 10)
        self.test_imu_pub = self.create_publisher(Imu, '/test_imu_data', 10)

        # Subscribers for safety responses
        self.emergency_stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10)
        self.safety_status_sub = self.create_subscription(
            String, '/safety_report', self.safety_status_callback, 10)

        self.emergency_stop_received = False
        self.safety_violations = []

        self.get_logger().info('Safety System Tester initialized')

    def emergency_stop_callback(self, msg):
        """Track emergency stop activations"""
        self.emergency_stop_received = msg.data

    def safety_status_callback(self, msg):
        """Track safety violations"""
        try:
            status = json.loads(msg.data)
            self.safety_violations.append(status)
        except:
            pass

    def test_joint_limit_safety(self):
        """Test joint limit safety system"""
        self.get_logger().info('Testing joint limit safety...')

        # Create joint state message that violates limits
        joint_msg = JointState()
        joint_msg.name = ['test_joint']
        joint_msg.position = [2.0]  # Exceeds typical limit
        joint_msg.velocity = [10.0]  # Excessive velocity
        joint_msg.effort = [50.0]   # Excessive effort

        self.test_joint_pub.publish(joint_msg)

        # Wait for safety response
        time.sleep(2.0)

        # Check if emergency stop was triggered
        success = self.emergency_stop_received
        self.get_logger().info(f'Joint limit test result: {"PASS" if success else "FAIL"}')

        return success

    def test_balance_safety(self):
        """Test balance safety system"""
        self.get_logger().info('Testing balance safety...')

        # Create IMU message indicating poor balance
        imu_msg = Imu()
        imu_msg.orientation.x = 0.7  # Large roll angle
        imu_msg.orientation.y = 0.7  # Large pitch angle
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 0.0

        self.test_imu_pub.publish(imu_msg)

        # Wait for safety response
        time.sleep(2.0)

        # Check if emergency stop was triggered
        success = self.emergency_stop_received
        self.get_logger().info(f'Balance test result: {"PASS" if success else "FAIL"}')

        return success

    def test_velocity_safety(self):
        """Test velocity limit safety"""
        self.get_logger().info('Testing velocity safety...')

        # Create command that exceeds velocity limits
        cmd_msg = Twist()
        cmd_msg.linear.x = 5.0  # Excessive linear velocity
        cmd_msg.angular.z = 5.0  # Excessive angular velocity

        self.test_cmd_pub.publish(cmd_msg)

        # Wait for safety response
        time.sleep(2.0)

        # Check if emergency stop was triggered
        success = self.emergency_stop_received
        self.get_logger().info(f'Velocity test result: {"PASS" if success else "FAIL"}')

        return success

    def run_all_tests(self):
        """Run all safety tests"""
        self.get_logger().info('Starting safety system tests...')

        test_results = {
            'joint_limit_test': self.test_joint_limit_safety(),
            'balance_test': self.test_balance_safety(),
            'velocity_test': self.test_velocity_safety()
        }

        # Summary
        passed = sum(test_results.values())
        total = len(test_results)
        self.get_logger().info(f'Safety tests completed: {passed}/{total} passed')

        return test_results

def run_safety_tests():
    """Run safety system tests"""
    rclpy.init()
    tester = SafetySystemTester()

    # Run tests
    results = tester.run_all_tests()

    # Shutdown
    tester.destroy_node()
    rclpy.shutdown()

    return results

if __name__ == '__main__':
    results = run_safety_tests()
    print("Safety Test Results:", results)
```

## Best Practices for Safety Systems

### Safety-Critical Design Patterns

```python
# safety_patterns.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from builtin_interfaces.msg import Duration
import threading
import time
from enum import Enum

class SafetyLevel(Enum):
    NORMAL = 1
    WARNING = 2
    EMERGENCY = 3
    CRITICAL = 4

class SafetyPatternNode(Node):
    def __init__(self):
        super().__init__('safety_pattern_node')

        # Implement safety patterns
        self._implement_safety_patterns()

        # Publishers and subscribers
        self.safety_pub = self.create_publisher(Bool, '/safety_status', 10)

        # Safety state with thread safety
        self.safety_lock = threading.Lock()
        self.current_safety_level = SafetyLevel.NORMAL
        self.safety_timer = self.create_timer(0.01, self.safety_check_loop)

    def _implement_safety_patterns(self):
        """Implement various safety design patterns"""
        # 1. Fail-Safe Pattern
        self._initialize_fail_safe_mechanisms()

        # 2. Defense-in-Depth Pattern
        self._implement_defense_in_depth()

        # 3. Graceful Degradation Pattern
        self._setup_graceful_degradation()

        # 4. Watchdog Pattern
        self._initialize_watchdog()

    def _initialize_fail_safe_mechanisms(self):
        """Implement fail-safe mechanisms"""
        # Default safe states
        self.default_joint_positions = {}  # Set to safe positions
        self.default_velocities = [0.0] * 10  # All joints stopped
        self.default_commands = {'linear': 0.0, 'angular': 0.0}  # Stop motion

        self.get_logger().info('Fail-safe mechanisms initialized')

    def _implement_defense_in_depth(self):
        """Implement defense-in-depth safety approach"""
        # Multiple layers of safety checks
        self.safety_layers = {
            'perception': {'enabled': True, 'priority': 1},
            'planning': {'enabled': True, 'priority': 2},
            'execution': {'enabled': True, 'priority': 3},
            'hardware': {'enabled': True, 'priority': 4}
        }

        self.get_logger().info('Defense-in-depth safety layers configured')

    def _setup_graceful_degradation(self):
        """Set up graceful degradation mechanisms"""
        self.degradation_levels = {
            'level_1': {'actions': ['reduce_speed'], 'threshold': 0.7},
            'level_2': {'actions': ['reduce_speed', 'increase_caution'], 'threshold': 0.5},
            'level_3': {'actions': ['stop_movement', 'request_help'], 'threshold': 0.3}
        }

        self.get_logger().info('Graceful degradation system configured')

    def _initialize_watchdog(self):
        """Initialize watchdog safety mechanism"""
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        self.last_heartbeat = time.time()
        self.watchdog_timeout = 1.0  # 1 second timeout

        self.get_logger().info('Watchdog safety initialized')

    def watchdog_check(self):
        """Check for system heartbeat"""
        current_time = time.time()
        if current_time - self.last_heartbeat > self.watchdog_timeout:
            self._activate_emergency_stop('Watchdog timeout - system unresponsive')

    def update_heartbeat(self):
        """Update system heartbeat"""
        self.last_heartbeat = time.time()

    def safety_check_loop(self):
        """Main safety check loop"""
        with self.safety_lock:
            # Perform safety checks
            current_level = self._assess_current_safety_level()

            if current_level != self.current_safety_level:
                self._handle_safety_level_change(current_level)

    def _assess_current_safety_level(self) -> SafetyLevel:
        """Assess current safety level based on system state"""
        # This would check various system parameters
        # For now, return NORMAL as default
        return SafetyLevel.NORMAL

    def _handle_safety_level_change(self, new_level: SafetyLevel):
        """Handle safety level change"""
        old_level = self.current_safety_level
        self.current_safety_level = new_level

        if new_level == SafetyLevel.EMERGENCY:
            self._activate_emergency_procedures()
        elif new_level == SafetyLevel.WARNING:
            self._activate_warning_procedures()
        elif new_level == SafetyLevel.CRITICAL:
            self._activate_critical_procedures()

        self.get_logger().info(f'Safety level changed from {old_level} to {new_level}')

    def _activate_emergency_procedures(self):
        """Activate emergency safety procedures"""
        # Stop all motion
        # Engage brakes if available
        # Log emergency event
        pass

    def _activate_warning_procedures(self):
        """Activate warning safety procedures"""
        # Reduce speeds
        # Increase caution
        # Log warning event
        pass

    def _activate_critical_procedures(self):
        """Activate critical safety procedures"""
        # Immediate stop
        # Preserve critical data
        # Prepare for emergency shutdown
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SafetyPatternNode()

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

## Performance Monitoring and Safety Metrics

### Safety Performance Dashboard

```python
# safety_performance.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
from diagnostic_msgs.msg import DiagnosticArray
import time
from collections import deque
import statistics

class SafetyPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('safety_performance_monitor')

        # Initialize performance tracking
        self._initialize_performance_tracking()

        # Publishers for performance metrics
        self.safety_response_time_pub = self.create_publisher(Float32, '/safety_response_time', 10)
        self.safety_violation_rate_pub = self.create_publisher(Float32, '/safety_violation_rate', 10)
        self.safety_availability_pub = self.create_publisher(Float32, '/safety_availability', 10)

        # Timer for performance calculations
        self.performance_timer = self.create_timer(1.0, self.calculate_performance_metrics)

        # Performance metrics
        self.response_times = deque(maxlen=100)
        self.violation_counts = deque(maxlen=10)
        self.operational_time = 0.0
        self.safety_stop_time = 0.0
        self.start_time = time.time()

        self.get_logger().info('Safety Performance Monitor initialized')

    def _initialize_performance_tracking(self):
        """Initialize performance tracking components"""
        # Set up subscribers for safety events
        self.safety_event_sub = self.create_subscription(
            Bool, '/safety_status', self.safety_status_callback, 10)

        self.emergency_stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10)

        self.get_logger().info('Safety performance tracking initialized')

    def safety_status_callback(self, msg):
        """Track safety status changes"""
        current_time = time.time()
        # Track response times and other metrics
        pass

    def emergency_stop_callback(self, msg):
        """Track emergency stop events"""
        if msg.data:  # Emergency stop activated
            self.safety_stop_time += time.time()
        else:  # Emergency stop deactivated
            self.safety_stop_time -= time.time()

    def calculate_performance_metrics(self):
        """Calculate and publish safety performance metrics"""
        current_time = time.time()
        total_time = current_time - self.start_time

        # Calculate safety availability
        availability = (total_time - self.safety_stop_time) / total_time if total_time > 0 else 1.0

        # Calculate violation rate
        recent_violations = sum(list(self.violation_counts)[-5:]) if self.violation_counts else 0
        violation_rate = recent_violations / 5.0  # Average per 5-second window

        # Calculate average response time
        avg_response_time = statistics.mean(self.response_times) if self.response_times else 0.0

        # Publish metrics
        avail_msg = Float32()
        avail_msg.data = float(availability)
        self.safety_availability_pub.publish(avail_msg)

        rate_msg = Float32()
        rate_msg.data = float(violation_rate)
        self.safety_violation_rate_pub.publish(rate_msg)

        response_msg = Float32()
        response_msg.data = float(avg_response_time)
        self.safety_response_time_pub.publish(response_msg)

        self.get_logger().info(
            f'Safety Performance - Availability: {availability:.3f}, '
            f'Violation Rate: {violation_rate:.3f}, '
            f'Avg Response: {avg_response_time:.3f}s'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SafetyPerformanceMonitor()

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

With the safety and guardrails system fully implemented, your humanoid robot is now equipped with comprehensive safety measures that protect both the robot and its environment. In the next module, we'll explore the Vision-Language-Action (VLA) pipeline that brings together all the capabilities we've developed into a unified AI system capable of understanding natural language commands, perceiving the environment visually, and executing appropriate actions. This will complete the foundation for creating truly intelligent humanoid robots that can interact naturally with humans and environments.