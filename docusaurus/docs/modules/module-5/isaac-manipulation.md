# Isaac Manipulation

In this chapter, we'll explore NVIDIA Isaac's manipulation capabilities, which provide GPU-accelerated solutions for robotic manipulation tasks. Isaac Manipulation leverages NVIDIA's GPU computing power to deliver high-performance grasp planning, trajectory optimization, and dexterous manipulation for humanoid robots.

## Isaac Manipulation Overview

### GPU-Accelerated Manipulation Components

Isaac Manipulation includes several GPU-accelerated manipulation packages:

- **Isaac ROS Grasp Pose Detection**: GPU-accelerated grasp pose estimation
- **Isaac ROS Motion Planning**: GPU-parallelized trajectory planning
- **Isaac ROS Manipulation Controller**: GPU-optimized control algorithms
- **Isaac ROS Perception for Manipulation**: GPU-accelerated object detection and pose estimation
- **Isaac ROS Inverse Kinematics**: GPU-accelerated IK solving

### Manipulation System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           Isaac Manipulation Stack                             │
├─────────────────────────────────────────────────────────────────────────────────┤
│  Perception → Grasp Planning → Trajectory Planning → Control → Execution     │
│  (GPU)      → (GPU)         → (GPU)           → (GPU)   → (Robot)        │
├─────────────────────────────────────────────────────────────────────────────────┤
│  GPU Components:                                                              │
│  - CUDA-based grasp pose detection                                            │
│  - TensorRT-accelerated object recognition                                     │
│  - GPU-parallelized motion planning                                           │
│  - CUDA-optimized inverse kinematics                                          │
│  - Real-time trajectory optimization                                          │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Isaac Grasp Pose Detection

### GPU-Accelerated Grasp Planning

```python
# isaac_grasp_detection.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header, Bool
from cv_bridge import CvBridge
import numpy as np
import cupy as cp  # GPU acceleration
from scipy.spatial.transform import Rotation as R
import torch

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

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.grasp_poses_pub = self.create_publisher(PoseArray, '/grasp_poses', 10)
        self.grasp_quality_pub = self.create_publisher(Bool, '/grasp_feasible', 10)

        # Grasp detection parameters
        self.min_grasp_score = 0.7
        self.max_grasps = 10
        self.grasp_approach_distance = 0.1  # 10cm approach distance
        self.object_detection_threshold = 0.5

        # Camera calibration
        self.camera_matrix = None
        self.distortion_coeffs = None

        # GPU-accelerated grasp detection model
        self.grasp_model = None
        self._initialize_grasp_model()

        self.get_logger().info('Isaac Grasp Detection node initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for grasp detection"""
        try:
            # Initialize CuPy for GPU operations
            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for grasp detection')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU initialization failed: {e}, using CPU fallback')

    def _initialize_grasp_model(self):
        """Initialize GPU-accelerated grasp detection model"""
        try:
            # In a real implementation, this would load a pre-trained grasp detection model
            # such as GPD (Grasp Pose Detection) or DexNet optimized for GPU
            if self.gpu_available:
                # Initialize model on GPU
                self.grasp_model = self._create_gpu_grasp_model()
                self.get_logger().info('GPU-accelerated grasp model initialized')
            else:
                # CPU fallback model
                self.grasp_model = self._create_cpu_grasp_model()
                self.get_logger().info('CPU grasp model initialized')

        except Exception as e:
            self.get_logger().error(f'Grasp model initialization error: {e}')
            raise

    def _create_gpu_grasp_model(self):
        """Create GPU-accelerated grasp detection model"""
        # This would typically be a deep learning model optimized for GPU
        # For this example, we'll create a placeholder
        class GPUGraspModel:
            def __init__(self):
                self.model_loaded = True

            def detect_grasps(self, image, pointcloud):
                """Detect grasp poses using GPU acceleration"""
                # In practice, this would use a neural network with CUDA acceleration
                # For simulation, return dummy grasp poses
                return [
                    {'position': [0.5, 0.0, 0.2], 'orientation': [0, 0, 0, 1], 'score': 0.9},
                    {'position': [0.6, 0.1, 0.2], 'orientation': [0, 0, 0, 1], 'score': 0.85}
                ]

        return GPUGraspModel()

    def image_callback(self, msg):
        """Process image for grasp detection"""
        try:
            # Convert ROS Image to numpy array
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.gpu_available:
                # Process with GPU acceleration
                grasp_poses = self._detect_grasps_gpu(cv_image, msg.header)
            else:
                # CPU fallback
                grasp_poses = self._detect_grasps_cpu(cv_image, msg.header)

            # Filter grasps by quality
            high_quality_grasps = [
                grasp for grasp in grasp_poses
                if grasp['score'] >= self.min_grasp_score
            ]

            if high_quality_grasps:
                # Publish grasp poses
                pose_array_msg = self._create_pose_array(high_quality_grasps, msg.header)
                self.grasp_poses_pub.publish(pose_array_msg)

                # Publish feasibility
                feasible_msg = Bool()
                feasible_msg.data = True
                self.grasp_quality_pub.publish(feasible_msg)

                self.get_logger().info(f'Detected {len(high_quality_grasps)} high-quality grasps')

        except Exception as e:
            self.get_logger().error(f'Grasp detection error: {e}')

    def pointcloud_callback(self, msg):
        """Process point cloud for 3D grasp detection"""
        try:
            # Convert point cloud to numpy array (simplified)
            # In practice, use sensor_msgs_py.point_cloud2.read_points
            points = self._pointcloud_to_array(msg)

            if self.gpu_available:
                # Use GPU-accelerated 3D grasp detection
                grasp_poses = self._detect_3d_grasps_gpu(points, msg.header)
            else:
                grasp_poses = self._detect_3d_grasps_cpu(points, msg.header)

            # Publish 3D grasp poses
            if grasp_poses:
                pose_array_msg = self._create_pose_array(grasp_poses, msg.header)
                self.grasp_poses_pub.publish(pose_array_msg)

        except Exception as e:
            self.get_logger().error(f'3D grasp detection error: {e}')

    def camera_info_callback(self, msg):
        """Update camera calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def _detect_grasps_gpu(self, image, header):
        """Detect grasp poses using GPU acceleration"""
        try:
            # Upload image to GPU
            gpu_image = cp.asarray(image)

            # Apply GPU-accelerated image processing
            processed_image = self._gpu_image_preprocessing(gpu_image)

            # Run grasp detection model
            grasp_candidates = self.grasp_model.detect_grasps(processed_image, None)

            # Convert GPU results back to CPU
            cpu_grasps = []
            for grasp in grasp_candidates:
                cpu_grasp = {
                    'position': grasp['position'],
                    'orientation': grasp['orientation'],
                    'score': grasp['score']
                }
                cpu_grasps.append(cpu_grasp)

            # Create ROS Pose messages
            grasp_poses = []
            for grasp in cpu_grasps:
                pose = Pose()
                pose.position.x = grasp['position'][0]
                pose.position.y = grasp['position'][1]
                pose.position.z = grasp['position'][2]

                pose.orientation.x = grasp['orientation'][0]
                pose.orientation.y = grasp['orientation'][1]
                pose.orientation.z = grasp['orientation'][2]
                pose.orientation.w = grasp['orientation'][3]

                grasp_poses.append(pose)

            return grasp_poses

        except Exception as e:
            self.get_logger().error(f'GPU grasp detection error: {e}')
            return self._detect_grasps_cpu(image, header)

    def _gpu_image_preprocessing(self, gpu_image):
        """GPU-accelerated image preprocessing"""
        try:
            # Apply GPU-optimized preprocessing
            # This could include:
            # - Noise reduction
            # - Edge enhancement
            # - Color space conversion
            # - Normalization

            # Convert BGR to RGB if needed
            if gpu_image.ndim == 3 and gpu_image.shape[2] == 3:
                gpu_image = gpu_image[:, :, ::-1]  # BGR to RGB

            # Normalize pixel values
            gpu_image = gpu_image.astype(cp.float32) / 255.0

            return gpu_image

        except Exception as e:
            self.get_logger().error(f'GPU preprocessing error: {e}')
            return gpu_image

    def _detect_grasps_cpu(self, image, header):
        """CPU fallback for grasp detection"""
        # Implement CPU-based grasp detection
        # This would use OpenCV, scikit-image, or other CPU libraries

        # For demonstration, use simple color-based object detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        grasp_poses = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Minimum area threshold
                # Calculate grasp point as center of mass
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Convert pixel coordinates to world coordinates using camera matrix
                    if self.camera_matrix is not None:
                        # Simple conversion (would be more complex in practice)
                        world_x = (cx - self.camera_matrix[0, 2]) * 0.1 / self.camera_matrix[0, 0]  # 10cm depth assumption
                        world_y = (cy - self.camera_matrix[1, 2]) * 0.1 / self.camera_matrix[1, 1]

                        pose = Pose()
                        pose.position.x = world_x
                        pose.position.y = world_y
                        pose.position.z = 0.1  # 10cm above ground
                        pose.orientation.w = 1.0  # Identity orientation

                        grasp_poses.append(pose)

        return grasp_poses

    def _pointcloud_to_array(self, pointcloud_msg):
        """Convert PointCloud2 message to numpy array"""
        # This would use sensor_msgs_py.point_cloud2.read_points
        # For now, return a placeholder
        return np.random.rand(1000, 3).astype(np.float32)  # Placeholder

    def _create_pose_array(self, grasp_poses, header):
        """Create PoseArray message from grasp poses"""
        pose_array = PoseArray()
        pose_array.header = header
        pose_array.poses = grasp_poses
        return pose_array

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

## Isaac Motion Planning

### GPU-Accelerated Trajectory Planning

```python
# gpu_motion_planning.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
import numpy as np
import cupy as cp
from scipy.spatial import KDTree
import threading

class IsaacMotionPlanningNode(Node):
    def __init__(self):
        super().__init__('isaac_motion_planning')

        # Initialize GPU resources
        self._initialize_gpu_resources()

        # Publishers and subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.manipulation_goal_sub = self.create_subscription(
            PoseStamped,
            '/manipulation_goal',
            self.manipulation_goal_callback,
            10
        )

        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        self.trajectory_status_pub = self.create_publisher(String, '/trajectory_status', 10)

        # Robot state and parameters
        self.current_joint_state = JointState()
        self.robot_dof = 7  # 7-DOF arm example
        self.joint_limits = {
            'min': [-2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0],
            'max': [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        }

        # Motion planning parameters
        self.planning_algorithm = 'rrt_connect'  # Options: rrt, prm, chomp
        self.max_iterations = 1000
        self.smoothing_iterations = 20
        self.collision_check_resolution = 0.01

        # GPU-accelerated motion planning components
        self.motion_planner = None
        self._initialize_motion_planner()

        self.get_logger().info('Isaac Motion Planning node initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for motion planning"""
        try:
            # Initialize GPU memory pool for efficient allocation
            self.gpu_memory_pool = cp.cuda.MemoryPool()
            cp.cuda.set_allocator(self.gpu_memory_pool.malloc)

            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for motion planning')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU initialization failed: {e}, using CPU fallback')

    def _initialize_motion_planner(self):
        """Initialize GPU-accelerated motion planning components"""
        try:
            if self.gpu_available:
                # Initialize GPU-accelerated motion planning
                self.motion_planner = self._create_gpu_motion_planner()
            else:
                # CPU fallback
                self.motion_planner = self._create_cpu_motion_planner()

            self.get_logger().info('Motion planner initialized successfully')

        except Exception as e:
            self.get_logger().error(f'Motion planner initialization error: {e}')
            raise

    def _create_gpu_motion_planner(self):
        """Create GPU-accelerated motion planner"""
        class GPUMotionPlanner:
            def __init__(self, robot_dof, joint_limits):
                self.robot_dof = robot_dof
                self.joint_limits = joint_limits
                self.collision_model = None  # Would be a GPU-accelerated collision model

            def plan_trajectory(self, start_joints, goal_pose):
                """Plan trajectory using GPU acceleration"""
                try:
                    # Convert to GPU arrays
                    start_gpu = cp.array(start_joints)
                    goal_gpu = cp.array(self._pose_to_joints(goal_pose))  # Simplified

                    # GPU-parallelized path planning (RRT* example)
                    path = self._gpu_rrt_star_planning(start_gpu, goal_gpu)

                    # GPU-optimized trajectory smoothing
                    smoothed_path = self._gpu_smooth_trajectory(path)

                    return smoothed_path

                except Exception as e:
                    print(f'GPU motion planning error: {e}')
                    return None

            def _gpu_rrt_star_planning(self, start, goal):
                """GPU-accelerated RRT* planning"""
                # This would implement GPU-parallelized RRT* algorithm
                # For this example, return a simple linear interpolation
                path = [start.get()]  # Convert back to CPU for now

                # Linear interpolation as placeholder
                for i in range(1, 20):  # 20 steps
                    t = i / 20.0
                    intermediate = start * (1 - t) + goal * t
                    path.append(cp.asnumpy(intermediate))

                return path

            def _gpu_smooth_trajectory(self, path):
                """GPU-optimized trajectory smoothing"""
                # Apply GPU-parallelized smoothing
                if len(path) < 3:
                    return path

                path_gpu = cp.array(path)
                smoothed_gpu = cp.zeros_like(path_gpu)

                # Simple smoothing using GPU parallelization
                for i in range(len(path_gpu)):
                    if i == 0 or i == len(path_gpu) - 1:
                        smoothed_gpu[i] = path_gpu[i]
                    else:
                        # Average with neighbors
                        smoothed_gpu[i] = (path_gpu[i-1] + path_gpu[i] + path_gpu[i+1]) / 3.0

                return cp.asnumpy(smoothed_gpu)

            def _pose_to_joints(self, pose):
                """Convert pose to joint angles (simplified)"""
                # This would implement inverse kinematics
                # For now, return a placeholder
                return [0.0] * 7

        return GPUMotionPlanner(self.robot_dof, self.joint_limits)

    def joint_state_callback(self, msg):
        """Update current joint state"""
        self.current_joint_state = msg

    def manipulation_goal_callback(self, msg):
        """Plan trajectory to manipulation goal"""
        try:
            if not self.current_joint_state.position:
                self.get_logger().warn('No current joint state available')
                return

            # Get current joint positions
            current_joints = list(self.current_joint_state.position)

            # Plan trajectory using GPU acceleration
            if self.gpu_available:
                trajectory = self._plan_trajectory_gpu(current_joints, msg.pose)
            else:
                trajectory = self._plan_trajectory_cpu(current_joints, msg.pose)

            if trajectory:
                # Convert to ROS trajectory message
                trajectory_msg = self._trajectory_to_ros_msg(
                    trajectory,
                    self.current_joint_state.name,
                    msg.header
                )

                # Publish trajectory
                self.joint_trajectory_pub.publish(trajectory_msg)

                # Publish status
                status_msg = String()
                status_msg.data = f"Trajectory planned with {len(trajectory)} waypoints"
                self.trajectory_status_pub.publish(status_msg)

                self.get_logger().info(f'Trajectory planned with {len(trajectory)} waypoints')

            else:
                self.get_logger().error('Failed to plan trajectory')

        except Exception as e:
            self.get_logger().error(f'Trajectory planning error: {e}')

    def _plan_trajectory_gpu(self, start_joints, goal_pose):
        """Plan trajectory using GPU acceleration"""
        try:
            # Use GPU motion planner
            trajectory = self.motion_planner.plan_trajectory(start_joints, goal_pose)

            return trajectory

        except Exception as e:
            self.get_logger().error(f'GPU trajectory planning error: {e}')
            return self._plan_trajectory_cpu(start_joints, goal_pose)

    def _plan_trajectory_cpu(self, start_joints, goal_pose):
        """CPU fallback for trajectory planning"""
        # Implement CPU-based trajectory planning
        # This would use OMPL, MoveIt, or other motion planning libraries

        # For demonstration, create a simple straight-line trajectory in joint space
        trajectory = []
        steps = 20

        # This is a very simplified approach
        # In practice, you'd use proper inverse kinematics and motion planning
        for i in range(steps + 1):
            t = i / steps
            intermediate_joints = []

            # Linear interpolation in joint space
            for j in range(len(start_joints)):
                if j < 7:  # Assuming 7-DOF arm
                    start_pos = start_joints[j] if j < len(start_joints) else 0.0
                    goal_pos = 0.0  # Placeholder - would come from IK
                    pos = start_pos + t * (goal_pos - start_pos)
                    intermediate_joints.append(pos)
                else:
                    intermediate_joints.append(0.0)

            trajectory.append(intermediate_joints)

        return trajectory

    def _trajectory_to_ros_msg(self, trajectory_array, joint_names, header):
        """Convert trajectory array to ROS JointTrajectory message"""
        traj_msg = JointTrajectory()
        traj_msg.header = header
        traj_msg.joint_names = joint_names[:7]  # Use first 7 joints for 7-DOF arm

        time_step = 0.1  # 100ms between points

        for i, joint_positions in enumerate(trajectory_array):
            point = JointTrajectoryPoint()
            point.positions = joint_positions[:7]  # Use first 7 positions

            # Calculate velocities using finite differences
            if i > 0 and i < len(trajectory_array) - 1:
                prev_pos = np.array(trajectory_array[i-1][:7])
                next_pos = np.array(trajectory_array[i+1][:7])
                velocities = (next_pos - prev_pos) / (2 * time_step)
                point.velocities = velocities.tolist()
            else:
                point.velocities = [0.0] * min(7, len(joint_positions))

            # Calculate accelerations
            if i > 0 and i < len(trajectory_array) - 1:
                prev_vel = np.array(point.velocities)
                next_vel = np.array([(np.array(trajectory_array[i+2][:7]) - np.array(trajectory_array[i][:7])) / (2 * time_step)
                                   if i+2 < len(trajectory_array) else [0.0]*min(7, len(joint_positions))])
                accelerations = (next_vel - prev_vel) / (2 * time_step)
                point.accelerations = accelerations.tolist()
            else:
                point.accelerations = [0.0] * min(7, len(joint_positions))

            point.time_from_start.sec = int(i * time_step)
            point.time_from_start.nanosec = int((i * time_step % 1) * 1e9)

            traj_msg.points.append(point)

        return traj_msg

def main(args=None):
    rclpy.init(args=args)
    node = IsaacMotionPlanningNode()

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

## Isaac Inverse Kinematics

### GPU-Accelerated IK Solving

```python
# gpu_inverse_kinematics.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float32
from builtin_interfaces.msg import Duration
import numpy as np
import cupy as cp
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
import threading

class IsaacIKNode(Node):
    def __init__(self):
        super().__init__('isaac_ik_solver')

        # Initialize GPU resources
        self._initialize_gpu_resources()

        # Publishers and subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.ik_target_sub = self.create_subscription(
            PoseStamped,
            '/ik_target',
            self.ik_target_callback,
            10
        )

        self.joint_command_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.ik_performance_pub = self.create_publisher(Float32, '/ik_performance', 10)

        # Robot parameters
        self.current_joint_state = JointState()
        self.robot_config = {
            'dof': 7,
            'link_lengths': [0.1, 0.3, 0.3, 0.1, 0.1, 0.1, 0.05],  # Example link lengths
            'joint_limits': {
                'min': [-2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0],
                'max': [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
            }
        }

        # IK parameters
        self.position_tolerance = 0.01  # 1cm
        self.orientation_tolerance = 0.01  # 0.01 rad
        self.max_iterations = 100
        self.convergence_threshold = 1e-6

        # GPU-accelerated IK solver
        self.ik_solver = None
        self._initialize_ik_solver()

        self.get_logger().info('Isaac GPU Inverse Kinematics node initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for IK solving"""
        try:
            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for IK solving')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU initialization failed: {e}, using CPU fallback')

    def _initialize_ik_solver(self):
        """Initialize GPU-accelerated IK solver"""
        try:
            if self.gpu_available:
                self.ik_solver = self._create_gpu_ik_solver()
            else:
                self.ik_solver = self._create_cpu_ik_solver()

            self.get_logger().info('IK solver initialized successfully')

        except Exception as e:
            self.get_logger().error(f'IK solver initialization error: {e}')
            raise

    def _create_gpu_ik_solver(self):
        """Create GPU-accelerated IK solver"""
        class GPUIKSolver:
            def __init__(self, robot_config):
                self.config = robot_config
                self.dof = robot_config['dof']

            def solve_ik(self, target_pose, current_joints):
                """Solve inverse kinematics using GPU acceleration"""
                try:
                    # Upload data to GPU
                    target_pos_gpu = cp.array([
                        target_pose.position.x,
                        target_pose.position.y,
                        target_pose.position.z
                    ])

                    target_rot_gpu = cp.array([
                        target_pose.orientation.x,
                        target_pose.orientation.y,
                        target_pose.orientation.z,
                        target_pose.orientation.w
                    ])

                    current_joints_gpu = cp.array(current_joints)

                    # GPU-parallelized IK solving
                    # This would implement GPU-optimized Jacobian or Cyclic Coordinate Descent
                    solution = self._gpu_iterative_ik(
                        target_pos_gpu, target_rot_gpu, current_joints_gpu
                    )

                    if solution is not None:
                        return cp.asnumpy(solution)
                    else:
                        return None

                except Exception as e:
                    print(f'GPU IK solving error: {e}')
                    return None

            def _gpu_iterative_ik(self, target_pos, target_rot, current_joints):
                """GPU-parallelized iterative IK solving"""
                # Implement GPU-parallelized iterative algorithm
                # For example, parallelized Jacobian transpose or pseudoinverse
                joint_angles = current_joints.copy()

                for iteration in range(self.config['max_iterations']):
                    # Calculate current end-effector pose using GPU
                    current_pos, current_rot = self._gpu_forward_kinematics(joint_angles)

                    # Calculate position and orientation errors
                    pos_error = target_pos - current_pos
                    rot_error = self._gpu_quaternion_difference(target_rot, current_rot)

                    # Check for convergence
                    pos_error_norm = cp.linalg.norm(pos_error)
                    rot_error_norm = cp.arccos(cp.abs(rot_error[3])) * 2  # Angle between quaternions

                    if (pos_error_norm < self.config['position_tolerance'] and
                        rot_error_norm < self.config['orientation_tolerance']):
                        return joint_angles

                    # Calculate Jacobian on GPU (simplified 3D Jacobian)
                    jacobian = self._gpu_calculate_jacobian(joint_angles)

                    # Solve for joint updates using GPU
                    if jacobian.shape[0] >= jacobian.shape[1]:
                        # Use pseudoinverse for overdetermined system
                        j_pinv = cp.linalg.pinv(jacobian)
                        joint_delta = j_pinv @ cp.concatenate([pos_error, rot_error[:3]])
                    else:
                        # Use transpose for underdetermined system
                        joint_delta = jacobian.T @ cp.concatenate([pos_error, rot_error[:3]])

                    # Apply joint updates
                    joint_angles = joint_angles + 0.1 * joint_delta  # Step size

                    # Apply joint limits
                    joint_angles = cp.clip(
                        joint_angles,
                        cp.array(self.config['joint_limits']['min']),
                        cp.array(self.config['joint_limits']['max'])
                    )

                # If no convergence, return best solution found
                return joint_angles

            def _gpu_forward_kinematics(self, joint_angles):
                """GPU-accelerated forward kinematics"""
                # Simplified forward kinematics for 7-DOF arm
                # In practice, this would implement the full kinematic chain
                # using GPU-parallelized matrix operations

                # For this example, return placeholder values
                # The actual implementation would compute FK using GPU-parallelized
                # rotation matrix and transformation calculations
                pos = cp.array([0.5, 0.0, 0.8])  # Placeholder position
                rot = cp.array([0.0, 0.0, 0.0, 1.0])  # Placeholder rotation (identity)

                return pos, rot

            def _gpu_calculate_jacobian(self, joint_angles):
                """Calculate Jacobian matrix using GPU"""
                # This would compute the geometric or analytical Jacobian
                # using GPU-parallelized differentiation
                # For now, return a placeholder Jacobian matrix
                # 6x7 Jacobian (position + orientation x 7 joints)
                return cp.random.rand(6, 7)  # Placeholder

            def _gpu_quaternion_difference(self, q1, q2):
                """Calculate quaternion difference using GPU"""
                # Calculate relative rotation quaternion
                q2_inv = cp.array([q2[0], q2[1], q2[2], -q2[3]])  # Conjugate
                q_diff = self._gpu_quaternion_multiply(q1, q2_inv)
                return q_diff / cp.linalg.norm(q_diff)  # Normalize

            def _gpu_quaternion_multiply(self, q1, q2):
                """Multiply quaternions using GPU"""
                w = q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2]
                x = q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1]
                y = q1[3]*q2[1] - q1[0]*q2[2] + q1[1]*q2[3] + q1[2]*q2[0]
                z = q1[3]*q2[2] + q1[0]*q2[1] - q1[1]*q2[0] + q1[2]*q2[3]

                return cp.array([x, y, z, w])

        return GPUIKSolver(self.robot_config)

    def joint_state_callback(self, msg):
        """Update current joint state"""
        self.current_joint_state = msg

    def ik_target_callback(self, msg):
        """Solve IK for target pose"""
        try:
            if not self.current_joint_state.position:
                self.get_logger().warn('No current joint state available')
                return

            # Get current joint positions
            current_joints = list(self.current_joint_state.position)

            # Solve IK using GPU acceleration
            start_time = self.get_clock().now()

            if self.gpu_available:
                solution = self._solve_ik_gpu(msg.pose, current_joints)
            else:
                solution = self._solve_ik_cpu(msg.pose, current_joints)

            end_time = self.get_clock().now()
            processing_time = (end_time.nanoseconds - start_time.nanoseconds) / 1e9

            if solution is not None:
                # Create and publish joint command
                joint_cmd = JointState()
                joint_cmd.header = msg.header
                joint_cmd.name = self.current_joint_state.name[:7]  # 7-DOF arm
                joint_cmd.position = solution[:7]  # Use first 7 joints

                self.joint_command_pub.publish(joint_cmd)

                # Publish performance metric
                perf_msg = Float32()
                perf_msg.data = float(processing_time)
                self.ik_performance_pub.publish(perf_msg)

                self.get_logger().info(
                    f'IK solution found in {processing_time:.4f}s: {solution[:7]}'
                )
            else:
                self.get_logger().error('IK solution not found')

        except Exception as e:
            self.get_logger().error(f'IK solving error: {e}')

    def _solve_ik_gpu(self, target_pose, current_joints):
        """Solve IK using GPU acceleration"""
        try:
            solution = self.ik_solver.solve_ik(target_pose, current_joints)
            return solution
        except Exception as e:
            self.get_logger().error(f'GPU IK solving error: {e}')
            return self._solve_ik_cpu(target_pose, current_joints)

    def _solve_ik_cpu(self, target_pose, current_joints):
        """CPU fallback for IK solving"""
        # Implement CPU-based IK solving using scipy optimization
        # or other CPU-based methods

        def ik_objective(joint_angles):
            # Calculate forward kinematics error
            # This would implement the actual FK computation
            # For now, return a simple error function
            return np.sum((joint_angles - current_joints[:len(joint_angles)])**2)

        # Use scipy optimizer as CPU fallback
        result = minimize(
            ik_objective,
            current_joints[:7],  # Use first 7 joints for 7-DOF
            method='L-BFGS-B',
            bounds=list(zip(
                self.robot_config['joint_limits']['min'],
                self.robot_config['joint_limits']['max']
            ))
        )

        if result.success:
            return result.x
        else:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = IsaacIKNode()

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

## Isaac Manipulation Controller

### GPU-Optimized Manipulation Control

```python
# manipulation_controller.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Float32
from builtin_interfaces.msg import Duration
import numpy as np
import cupy as cp
from scipy.interpolate import CubicSpline
import math

class IsaacManipulationControllerNode(Node):
    def __init__(self):
        super().__init__('isaac_manipulation_controller')

        # Initialize GPU resources
        self._initialize_gpu_resources()

        # Publishers and subscribers
        self.joint_trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.joint_trajectory_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.feedback_pub = self.create_publisher(Float32, '/manipulation_feedback', 10)
        self.status_pub = self.create_publisher(Bool, '/manipulation_active', 10)

        # Controller parameters
        self.control_frequency = 100.0  # 100Hz control loop
        self.position_gain = 10.0
        self.velocity_gain = 2.0
        self.acceleration_gain = 1.0
        self.max_joint_velocity = 2.0  # rad/s
        self.max_joint_acceleration = 5.0  # rad/s^2

        # Controller state
        self.current_joint_state = JointState()
        self.current_trajectory = None
        self.trajectory_index = 0
        self.controller_active = False

        # GPU-accelerated trajectory following
        self.trajectory_interpolator = None

        # Timer for control loop
        self.control_timer = self.create_timer(1.0/self.control_frequency, self.control_loop)

        self.get_logger().info('Isaac Manipulation Controller initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for control"""
        try:
            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for manipulation control')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU initialization failed: {e}, using CPU fallback')

    def joint_state_callback(self, msg):
        """Update current joint state"""
        self.current_joint_state = msg

    def joint_trajectory_callback(self, msg):
        """Receive and process joint trajectory"""
        self.current_trajectory = msg
        self.trajectory_index = 0
        self.controller_active = True

        # Publish status
        status_msg = Bool()
        status_msg.data = True
        self.status_pub.publish(status_msg)

        self.get_logger().info(f'Received trajectory with {len(msg.points)} points')

    def control_loop(self):
        """Main control loop"""
        if not self.controller_active or not self.current_trajectory:
            # Publish zero commands when inactive
            if self.current_joint_state.position:
                zero_cmd = JointState()
                zero_cmd.header.stamp = self.get_clock().now().to_msg()
                zero_cmd.name = self.current_joint_state.name
                zero_cmd.position = [0.0] * len(self.current_joint_state.position)
                self.cmd_pub.publish(zero_cmd)
            return

        try:
            # Check if we have reached the end of trajectory
            if self.trajectory_index >= len(self.current_trajectory.points):
                self._complete_trajectory()
                return

            # Get current trajectory point
            target_point = self.current_trajectory.points[self.trajectory_index]

            # Calculate control commands
            if self.gpu_available:
                control_cmd = self._calculate_gpu_control_commands(target_point)
            else:
                control_cmd = self._calculate_cpu_control_commands(target_point)

            if control_cmd:
                self.cmd_pub.publish(control_cmd)

                # Calculate and publish feedback
                feedback = self._calculate_execution_feedback(target_point)
                feedback_msg = Float32()
                feedback_msg.data = feedback
                self.feedback_pub.publish(feedback_msg)

                # Advance trajectory index based on time
                self._advance_trajectory_index()

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')
            self._deactivate_controller()

    def _calculate_gpu_control_commands(self, target_point):
        """Calculate control commands using GPU acceleration"""
        try:
            # Get current positions
            if not self.current_joint_state.position:
                return None

            current_pos = cp.array(self.current_joint_state.position)
            target_pos = cp.array(target_point.positions)

            if target_point.velocities:
                target_vel = cp.array(target_point.velocities)
            else:
                target_vel = cp.zeros_like(target_pos)

            if target_point.accelerations:
                target_acc = cp.array(target_point.accelerations)
            else:
                target_acc = cp.zeros_like(target_pos)

            # Calculate errors
            pos_error = target_pos[:len(current_pos)] - current_pos
            vel_error = target_vel[:len(current_pos)] - cp.zeros_like(current_pos)  # Simplified velocity error

            # Apply PID control with GPU acceleration
            control_signal = (self.position_gain * pos_error +
                            self.velocity_gain * vel_error +
                            self.acceleration_gain * target_acc[:len(current_pos)])

            # Limit velocities
            control_signal = cp.clip(control_signal, -self.max_joint_velocity, self.max_joint_velocity)

            # Create command message
            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = self.current_joint_state.name
            cmd_msg.position = cp.asnumpy(control_signal).tolist()

            return cmd_msg

        except Exception as e:
            self.get_logger().error(f'GPU control calculation error: {e}')
            return self._calculate_cpu_control_commands(target_point)

    def _calculate_cpu_control_commands(self, target_point):
        """CPU fallback for control calculation"""
        if not self.current_joint_state.position:
            return None

        current_pos = np.array(self.current_joint_state.position)
        target_pos = np.array(target_point.positions[:len(current_pos)])

        if target_point.velocities:
            target_vel = np.array(target_point.velocities[:len(current_pos)])
        else:
            target_vel = np.zeros_like(current_pos)

        if target_point.accelerations:
            target_acc = np.array(target_point.accelerations[:len(current_pos)])
        else:
            target_acc = np.zeros_like(current_pos)

        # Calculate errors
        pos_error = target_pos - current_pos
        vel_error = target_vel - np.zeros_like(current_pos)  # Simplified

        # Apply PID control
        control_signal = (self.position_gain * pos_error +
                         self.velocity_gain * vel_error +
                         self.acceleration_gain * target_acc)

        # Limit velocities
        control_signal = np.clip(control_signal, -self.max_joint_velocity, self.max_joint_velocity)

        # Create command message
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = self.current_joint_state.name
        cmd_msg.position = control_signal.tolist()

        return cmd_msg

    def _calculate_execution_feedback(self, target_point):
        """Calculate trajectory execution feedback"""
        if not self.current_joint_state.position:
            return 0.0

        current_pos = np.array(self.current_joint_state.position)
        target_pos = np.array(target_point.positions[:len(current_pos)])

        # Calculate tracking error
        pos_error = np.linalg.norm(current_pos - target_pos)
        return float(pos_error)

    def _advance_trajectory_index(self):
        """Advance to next trajectory point based on timing"""
        # In a real implementation, this would be based on trajectory timing
        # For this example, advance every few control cycles
        self.trajectory_index += 1

    def _complete_trajectory(self):
        """Handle trajectory completion"""
        self.controller_active = False

        # Publish completion status
        status_msg = Bool()
        status_msg.data = False
        self.status_pub.publish(status_msg)

        self.get_logger().info('Manipulation trajectory completed')

    def _deactivate_controller(self):
        """Safely deactivate controller"""
        self.controller_active = False

        # Publish zero commands
        if self.current_joint_state.position:
            zero_cmd = JointState()
            zero_cmd.header.stamp = self.get_clock().now().to_msg()
            zero_cmd.name = self.current_joint_state.name
            zero_cmd.position = [0.0] * len(self.current_joint_state.position)
            self.cmd_pub.publish(zero_cmd)

        # Publish inactive status
        status_msg = Bool()
        status_msg.data = False
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacManipulationControllerNode()

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

## Isaac Manipulation Integration

### Complete Manipulation System

```python
# complete_manipulation_system.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from vision_msgs.msg import Detection2DArray
import numpy as np

class IsaacManipulationSystemNode(Node):
    def __init__(self):
        super().__init__('isaac_manipulation_system')

        # Initialize all manipulation components
        self.grasp_detector = IsaacGraspDetectionNode()
        self.motion_planner = IsaacMotionPlanningNode()
        self.ik_solver = IsaacIKNode()
        self.manipulation_controller = IsaacManipulationControllerNode()

        # Publishers for integrated system
        self.manipulation_command_pub = self.create_publisher(String, '/manipulation_command', 10)
        self.system_status_pub = self.create_publisher(String, '/manipulation_system_status', 10)

        # Manipulation state
        self.current_task = None
        self.manipulation_active = False
        self.system_ready = False

        # Timer for system coordination
        self.coordination_timer = self.create_timer(0.1, self.system_coordination_loop)

        self.get_logger().info('Isaac Complete Manipulation System initialized')

    def system_coordination_loop(self):
        """Coordinate manipulation system components"""
        try:
            # Check system readiness
            self.system_ready = all([
                self.grasp_detector is not None,
                self.motion_planner is not None,
                self.ik_solver is not None,
                self.manipulation_controller is not None
            ])

            if self.system_ready:
                # Process manipulation tasks
                self._process_manipulation_tasks()

                # Monitor system status
                self._monitor_system_status()

        except Exception as e:
            self.get_logger().error(f'System coordination error: {e}')

    def _process_manipulation_tasks(self):
        """Process manipulation tasks through integrated pipeline"""
        if self.current_task:
            task_type = self.current_task.get('type', 'unknown')

            if task_type == 'grasp_object':
                self._execute_grasp_task(self.current_task)
            elif task_type == 'place_object':
                self._execute_place_task(self.current_task)
            elif task_type == 'manipulate_object':
                self._execute_manipulate_task(self.current_task)

    def _execute_grasp_task(self, task):
        """Execute grasp task using complete pipeline"""
        try:
            # 1. Detect object and potential grasp poses
            # This would call the grasp detection system
            object_pose = task.get('object_pose')
            if not object_pose:
                self.get_logger().warn('No object pose specified for grasp task')
                return

            # 2. Plan motion to approach object
            approach_pose = self._calculate_approach_pose(object_pose)
            approach_trajectory = self.motion_planner.plan_trajectory_to_pose(approach_pose)

            if approach_trajectory:
                # 3. Execute approach trajectory
                self.manipulation_controller.follow_trajectory(approach_trajectory)

                # 4. Plan grasp trajectory
                grasp_trajectory = self._plan_grasp_trajectory(object_pose)
                if grasp_trajectory:
                    # 5. Execute grasp
                    self.manipulation_controller.follow_trajectory(grasp_trajectory)

                    # 6. Verify grasp success
                    grasp_successful = self._verify_grasp_success()

                    result_msg = String()
                    result_msg.data = f'Grasp task completed: {grasp_successful}'
                    self.manipulation_command_pub.publish(result_msg)

                    self.get_logger().info(f'Grasp task completed: {grasp_successful}')

        except Exception as e:
            self.get_logger().error(f'Grasp task execution error: {e}')

    def _calculate_approach_pose(self, object_pose):
        """Calculate approach pose for grasping"""
        # Calculate approach position 10cm away from object
        approach_pose = PoseStamped()
        approach_pose.header = object_pose.header

        # Move 10cm away along approach direction
        approach_distance = 0.1  # 10cm
        approach_pose.pose.position.x = object_pose.pose.position.x - approach_distance
        approach_pose.pose.position.y = object_pose.pose.position.y
        approach_pose.pose.position.z = object_pose.pose.position.z + 0.05  # Slightly higher

        # Maintain same orientation for now
        approach_pose.pose.orientation = object_pose.pose.orientation

        return approach_pose

    def _plan_grasp_trajectory(self, target_pose):
        """Plan trajectory for grasping"""
        # This would integrate with motion planner and IK solver
        # For now, return a placeholder trajectory
        return JointTrajectory()  # Placeholder

    def _verify_grasp_success(self):
        """Verify if grasp was successful"""
        # This would check force/torque sensors, vision, etc.
        # For simulation, return True
        return True

    def _monitor_system_status(self):
        """Monitor manipulation system status"""
        status_msg = String()
        status_msg.data = f"Ready: {self.system_ready}, Active: {self.manipulation_active}"
        self.system_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacManipulationSystemNode()

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

### Manipulation Performance Tuning

```python
# manipulation_performance.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from builtin_interfaces.msg import Time
import time
from collections import deque
import GPUtil

class IsaacManipulationPerformanceNode(Node):
    def __init__(self):
        super().__init__('isaac_manipulation_performance')

        # Performance metrics
        self.grasp_detection_times = deque(maxlen=100)
        self.motion_planning_times = deque(maxlen=100)
        self.ik_solving_times = deque(maxlen=100)
        self.control_loop_times = deque(maxlen=100)

        # Publishers for performance metrics
        self.grasp_perf_pub = self.create_publisher(Float32, '/perf/grasp_detection', 10)
        self.planning_perf_pub = self.create_publisher(Float32, '/perf/motion_planning', 10)
        self.ik_perf_pub = self.create_publisher(Float32, '/perf/ik_solving', 10)
        self.control_perf_pub = self.create_publisher(Float32, '/perf/control_loop', 10)

        # GPU monitoring
        self.gpu_util_pub = self.create_publisher(Float32, '/perf/gpu_utilization', 10)
        self.gpu_memory_pub = self.create_publisher(Float32, '/perf/gpu_memory', 10)

        # Performance monitoring timer
        self.performance_timer = self.create_timer(1.0, self.performance_monitoring_loop)

        # Adaptive parameters
        self.current_model_size = 'small'  # Start with smaller model for performance
        self.adaptive_resolution = True
        self.current_resolution = 1.0

        self.get_logger().info('Isaac Manipulation Performance Monitor initialized')

    def performance_monitoring_loop(self):
        """Monitor and publish performance metrics"""
        try:
            # Calculate average performance metrics
            avg_grasp_time = np.mean(self.grasp_detection_times) if self.grasp_detection_times else 0.0
            avg_planning_time = np.mean(self.motion_planning_times) if self.motion_planning_times else 0.0
            avg_ik_time = np.mean(self.ik_solving_times) if self.ik_solving_times else 0.0
            avg_control_time = np.mean(self.control_loop_times) if self.control_loop_times else 0.0

            # Publish performance metrics
            metrics = {
                'grasp_detection': avg_grasp_time,
                'motion_planning': avg_planning_time,
                'ik_solving': avg_ik_time,
                'control_loop': avg_control_time
            }

            for metric_name, value in metrics.items():
                msg = Float32()
                msg.data = float(value)

                if metric_name == 'grasp_detection':
                    self.grasp_perf_pub.publish(msg)
                elif metric_name == 'motion_planning':
                    self.planning_perf_pub.publish(msg)
                elif metric_name == 'ik_solving':
                    self.ik_perf_pub.publish(msg)
                elif metric_name == 'control_loop':
                    self.control_perf_pub.publish(msg)

            # Monitor GPU utilization
            self._monitor_gpu_performance()

            # Log performance summary
            self.get_logger().info(
                f'Manipulation Performance - Grasp: {avg_grasp_time:.3f}s, '
                f'Planning: {avg_planning_time:.3f}s, '
                f'IK: {avg_ik_time:.3f}s, '
                f'Control: {avg_control_time:.3f}s'
            )

            # Apply adaptive optimizations
            self._apply_adaptive_optimizations(
                avg_grasp_time, avg_planning_time, avg_ik_time, avg_control_time
            )

        except Exception as e:
            self.get_logger().error(f'Performance monitoring error: {e}')

    def _monitor_gpu_performance(self):
        """Monitor GPU performance"""
        try:
            gpus = GPUtil.getGPUs()
            if gpus:
                gpu = gpus[0]  # Use first GPU

                # Publish GPU utilization
                util_msg = Float32()
                util_msg.data = float(gpu.load * 100)
                self.gpu_util_pub.publish(util_msg)

                # Publish GPU memory usage
                mem_msg = Float32()
                mem_msg.data = float(gpu.memoryUtil * 100)
                self.gpu_memory_pub.publish(mem_msg)

        except Exception as e:
            self.get_logger().warn(f'GPU monitoring error: {e}')

    def _apply_adaptive_optimizations(self, avg_grasp_time, avg_planning_time, avg_ik_time, avg_control_time):
        """Apply adaptive optimizations based on performance"""
        target_rate = 30.0  # Target 30Hz for manipulation tasks

        # Adjust model size based on performance
        if avg_grasp_time > 1.0/target_rate * 0.8:  # 80% of budget
            # Performance is degrading, use smaller model
            if self.current_model_size != 'tiny':
                self.current_model_size = 'tiny'
                self.get_logger().info('Switched to tiny model for better performance')

        elif avg_grasp_time < 1.0/target_rate * 0.3:  # 30% of budget
            # Performance is good, can use larger model
            if self.current_model_size == 'tiny':
                self.current_model_size = 'small'
                self.get_logger().info('Switched to small model for better accuracy')

        # Adjust processing resolution
        if self.adaptive_resolution:
            if avg_planning_time > 0.05:  # 50ms threshold
                self.current_resolution = max(0.5, self.current_resolution * 0.9)
                self.get_logger().info(f'Reduced resolution scale to {self.current_resolution:.2f}')
            elif avg_planning_time < 0.02:  # 20ms threshold
                self.current_resolution = min(1.0, self.current_resolution * 1.1)
                self.get_logger().info(f'Increased resolution scale to {self.current_resolution:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = IsaacManipulationPerformanceNode()

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

## Safety and Guardrails

### Manipulation Safety Systems

```python
# manipulation_safety.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import WrenchStamped
from builtin_interfaces.msg import Duration
import numpy as np

class IsaacManipulationSafetyNode(Node):
    def __init__(self):
        super().__init__('isaac_manipulation_safety')

        # Safety parameters
        self.joint_limit_safety = True
        self.collision_avoidance = True
        self.force_limit_safety = True
        self.velocity_limit_safety = True

        # Joint safety limits
        self.joint_limits = {
            'min': [-2.5, -1.5, -2.5, -2.0, -2.5, -1.5, -3.0],
            'max': [2.5, 1.5, 2.5, 2.0, 2.5, 1.5, 3.0]
        }

        self.max_joint_velocity = 3.0  # rad/s
        self.max_force_threshold = 50.0  # N
        self.max_torque_threshold = 10.0  # N*m

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            '/wrench/ee_force',
            self.wrench_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publishers
        self.safety_status_pub = self.create_publisher(Bool, '/manipulation_safe', 10)
        self.safety_alert_pub = self.create_publisher(String, '/safety_alert', 10)

        # Safety monitoring timer
        self.safety_timer = self.create_timer(0.01, self.safety_monitoring_loop)  # 100Hz

        # Safety state
        self.current_joint_state = None
        self.current_wrench = None
        self.current_imu = None
        self.safety_violation = False
        self.violation_reasons = []

        self.get_logger().info('Isaac Manipulation Safety system initialized')

    def joint_state_callback(self, msg):
        """Monitor joint states for safety violations"""
        self.current_joint_state = msg

        # Check joint limits
        if self.joint_limit_safety:
            self._check_joint_limits()

        # Check velocity limits
        if self.velocity_limit_safety:
            self._check_velocity_limits()

    def wrench_callback(self, msg):
        """Monitor force/torque for safety"""
        self.current_wrench = msg

        if self.force_limit_safety:
            self._check_force_limits()

    def imu_callback(self, msg):
        """Monitor IMU for balance and safety"""
        self.current_imu = msg

        # Check for dangerous orientations that could affect manipulation
        self._check_balance_safety()

    def safety_monitoring_loop(self):
        """Continuous safety monitoring"""
        if not self.current_joint_state:
            return

        try:
            # Check for safety violations
            safe = self._evaluate_safety_conditions()

            # Publish safety status
            safety_msg = Bool()
            safety_msg.data = safe
            self.safety_status_pub.publish(safety_msg)

            # If unsafe, publish alerts
            if not safe and self.violation_reasons:
                alert_msg = String()
                alert_msg.data = f"Safety violation: {', '.join(self.violation_reasons)}"
                self.safety_alert_pub.publish(alert_msg)

                self.get_logger().error(f'Safety violation: {alert_msg.data}')

        except Exception as e:
            self.get_logger().error(f'Safety monitoring error: {e}')

    def _evaluate_safety_conditions(self):
        """Evaluate all safety conditions"""
        safe = True
        self.violation_reasons = []

        # Check each safety condition
        if not self._check_joint_safety():
            safe = False
            self.violation_reasons.append('Joint safety violation')

        if not self._check_force_safety():
            safe = False
            self.violation_reasons.append('Force safety violation')

        if not self._check_balance_safety():
            safe = False
            self.violation_reasons.append('Balance safety violation')

        return safe

    def _check_joint_limits(self):
        """Check if joints are within limits"""
        if not self.current_joint_state or not self.current_joint_state.position:
            return True

        for i, (joint_name, position) in enumerate(zip(
            self.current_joint_state.name,
            self.current_joint_state.position
        )):
            if i < len(self.joint_limits['min']):
                min_limit = self.joint_limits['min'][i]
                max_limit = self.joint_limits['max'][i]

                if position < min_limit or position > max_limit:
                    self.get_logger().error(f'Joint limit violation: {joint_name} = {position:.3f}, limits: [{min_limit:.3f}, {max_limit:.3f}]')
                    return False

        return True

    def _check_velocity_limits(self):
        """Check if joint velocities are within limits"""
        if not self.current_joint_state or not self.current_joint_state.velocity:
            return True

        for velocity in self.current_joint_state.velocity:
            if abs(velocity) > self.max_joint_velocity:
                self.get_logger().error(f'Velocity limit violation: {velocity:.3f} > {self.max_joint_velocity:.3f}')
                return False

        return True

    def _check_force_limits(self):
        """Check if forces are within safe limits"""
        if not self.current_wrench:
            return True

        force_magnitude = np.sqrt(
            self.current_wrench.wrench.force.x**2 +
            self.current_wrench.wrench.force.y**2 +
            self.current_wrench.wrench.force.z**2
        )

        torque_magnitude = np.sqrt(
            self.current_wrench.wrench.torque.x**2 +
            self.current_wrench.wrench.torque.y**2 +
            self.current_wrench.wrench.torque.z**2
        )

        if force_magnitude > self.max_force_threshold:
            self.get_logger().error(f'Force limit violation: {force_magnitude:.3f}N > {self.max_force_threshold}N')
            return False

        if torque_magnitude > self.max_torque_threshold:
            self.get_logger().error(f'Torque limit violation: {torque_magnitude:.3f}Nm > {self.max_torque_threshold}Nm')
            return False

        return True

    def _check_balance_safety(self):
        """Check robot balance safety during manipulation"""
        if not self.current_imu:
            return True

        # Check if robot is tilting too much during manipulation
        # This would check roll/pitch angles from IMU
        orientation = self.current_imu.orientation
        # Convert quaternion to roll/pitch/yaw for balance check
        # (implementation would use tf_transformations)

        # For now, assume safe if no extreme angles
        return True

def main(args=None):
    rclpy.init(args=args)
    node = IsaacManipulationSafetyNode()

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

### Isaac Manipulation Best Practices

1. **GPU Memory Management**: Pre-allocate memory pools for consistent performance
2. **Real-time Constraints**: Ensure manipulation planning meets timing requirements
3. **Safety First**: Always validate joint limits and forces before execution
4. **Modular Design**: Keep grasp detection, planning, and control separate
5. **Performance Monitoring**: Continuously monitor and optimize performance
6. **Error Recovery**: Implement graceful degradation and recovery behaviors
7. **Calibration**: Ensure proper robot and sensor calibration
8. **Validation**: Test manipulation tasks in simulation before real deployment

### Performance Optimization Guidelines

1. **Batch Processing**: Process multiple grasp candidates together
2. **Asynchronous Operations**: Use non-blocking computation where possible
3. **Model Optimization**: Use TensorRT for optimized inference
4. **Resolution Management**: Adjust processing resolution based on requirements
5. **Caching**: Cache expensive computations when possible
6. **Threading**: Use appropriate threading for I/O and processing
7. **Resource Monitoring**: Continuously monitor GPU and CPU utilization

## Troubleshooting

### Common Manipulation Issues and Solutions

**Issue**: Grasp detection is too slow
**Solution**: Use smaller model, reduce image resolution, optimize GPU memory usage

**Issue**: IK solutions are unstable
**Solution**: Add joint limit constraints, improve initial guess, adjust convergence parameters

**Issue**: Trajectory execution is jerky
**Solution**: Increase smoothing, adjust control gains, verify timing

**Issue**: Robot collides with environment
**Solution**: Improve collision checking, adjust trajectory planning parameters

**Issue**: GPU memory exhaustion
**Solution**: Reduce batch sizes, optimize memory allocation, use memory pools

## Next Steps

In the next chapter, we'll explore Isaac's perception capabilities for manipulation, learning how to integrate vision systems with manipulation planning to enable robots to perceive objects, understand their properties, and execute precise manipulation tasks with visual feedback and guidance.