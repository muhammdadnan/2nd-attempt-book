# Isaac Manipulation

In this chapter, we'll explore NVIDIA Isaac's manipulation capabilities, which provide GPU-accelerated solutions for robotic manipulation tasks. Isaac Manipulation leverages NVIDIA's GPU computing power to deliver high-performance grasp planning, trajectory optimization, and dexterous manipulation for humanoid robots.

## Isaac Manipulation Overview

### Core Manipulation Components

Isaac Manipulation includes several GPU-accelerated manipulation packages:

- **Isaac ROS Grasp Pose Detection**: GPU-accelerated grasp pose estimation
- **Isaac ROS Motion Planning**: GPU-parallelized trajectory planning
- **Isaac ROS Manipulation Controller**: GPU-optimized control algorithms
- **Isaac ROS Perception for Manipulation**: GPU-accelerated object detection and pose estimation
- **Isaac ROS Inverse Kinematics**: GPU-accelerated IK solving

### Manipulation Pipeline Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                            Isaac Manipulation Stack                            │
├─────────────────────────────────────────────────────────────────────────────────┤
│  Perception → Grasp Planning → Trajectory Planning → Control → Execution      │
│  (GPU Accel)   (GPU Accel)      (GPU Accel)        (GPU)    (Robot)         │
│      │              │                  │               │          │          │
│      ▼              ▼                  ▼               ▼          ▼          │
│  Object    →   Grasp Poses   →   Joint Trajectories →  Commands →  Actions   │
│  Detection      Estimation       Optimization         Generation   Execution │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Isaac Grasp Pose Detection

### GPU-Accelerated Grasp Planning

```python
# isaac_grasp_detection.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose, PoseArray, Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cupy as cp  # GPU acceleration
from scipy.spatial.transform import Rotation as R

class IsaacGraspDetectionNode(Node):
    def __init__(self):
        super().__init__('isaac_grasp_detection')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize GPU resources for grasp detection
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

        self.grasp_poses_pub = self.create_publisher(PoseArray, '/grasp_poses', 10)

        # Grasp detection parameters
        self.min_grasp_score = 0.7
        self.max_grasps = 10
        self.grasp_approach_distance = 0.1  # 10cm approach distance

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

    def image_callback(self, msg):
        """Process image for grasp detection"""
        try:
            # Convert ROS Image to numpy array
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.gpu_available:
                # Process with GPU acceleration
                grasp_poses = self._detect_grasps_gpu(cv_image)
            else:
                # CPU fallback
                grasp_poses = self._detect_grasps_cpu(cv_image)

            # Publish grasp poses
            if grasp_poses:
                pose_array_msg = PoseArray()
                pose_array_msg.header = msg.header
                pose_array_msg.poses = grasp_poses[:self.max_grasps]  # Limit number of grasps

                self.grasp_poses_pub.publish(pose_array_msg)

                self.get_logger().info(f'Detected {len(grasp_poses)} grasp poses')

        except Exception as e:
            self.get_logger().error(f'Grasp detection error: {e}')

    def _detect_grasps_gpu(self, image):
        """Detect grasp poses using GPU acceleration"""
        try:
            # Upload image to GPU
            gpu_image = cp.asarray(image)

            # Apply GPU-accelerated edge detection
            edges = self._gpu_edge_detection(gpu_image)

            # Apply GPU-accelerated grasp detection
            grasp_candidates = self._gpu_grasp_detection(edges)

            # Filter candidates based on quality
            high_quality_grasps = [g for g in grasp_candidates if g['score'] > self.min_grasp_score]

            # Convert to ROS Pose format
            grasp_poses = []
            for grasp in high_quality_grasps:
                pose = Pose()
                pose.position.x = grasp['position'][0]
                pose.position.y = grasp['position'][1]
                pose.position.z = grasp['position'][2]

                # Set orientation based on grasp direction
                rotation = R.from_euler('xyz', grasp['orientation'], degrees=True)
                quat = rotation.as_quat()
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                grasp_poses.append(pose)

            return grasp_poses

        except Exception as e:
            self.get_logger().error(f'GPU grasp detection error: {e}')
            # Fallback to CPU
            return self._detect_grasps_cpu(cp.asnumpy(gpu_image))

    def _gpu_edge_detection(self, image):
        """Apply GPU-accelerated edge detection"""
        # Convert to grayscale if needed
        if image.ndim == 3:
            gray = cp.dot(image[...,:3], [0.299, 0.587, 0.114])
        else:
            gray = image

        # Apply Gaussian blur
        blurred = cp.zeros_like(gray)
        # This would use a GPU convolution kernel in practice
        # For now, we'll use a simplified approach

        # Apply Canny edge detection (simulated)
        # In practice, this would use a GPU-optimized edge detection algorithm
        edges = cp.zeros_like(gray)
        # Simulate edge detection
        edges[gray > 128] = 255

        return edges

    def _gpu_grasp_detection(self, edges):
        """Detect grasp candidates using GPU acceleration"""
        # This would implement GPU-accelerated grasp detection algorithms
        # Such as: GPD (Grasp Pose Detection), DexNet, or other GPU-optimized methods

        # For demonstration, return some simulated grasp poses
        grasp_candidates = []

        # Find edge pixels
        edge_coords = cp.where(edges > 0)
        if len(edge_coords[0]) > 0:
            # Sample some edge points as grasp candidates
            sample_indices = cp.random.choice(len(edge_coords[0]), size=min(20, len(edge_coords[0])), replace=False)

            for idx in sample_indices:
                y, x = edge_coords[0][idx], edge_coords[1][idx]

                # Create grasp candidate
                grasp = {
                    'position': [float(x) * 0.01, float(y) * 0.01, 0.5],  # Convert pixel to meters
                    'orientation': [0, 0, 0],  # Placeholder orientation
                    'score': float(cp.random.uniform(0.5, 1.0))  # Random score for demo
                }
                grasp_candidates.append(grasp)

        return grasp_candidates

    def _detect_grasps_cpu(self, image):
        """CPU fallback for grasp detection"""
        # Implement CPU-based grasp detection
        # This would use OpenCV, scikit-image, or other CPU-based algorithms
        grasps = []

        # Example: Simple grasp detection using image moments
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Minimum area threshold
                # Calculate moment-based grasp point
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    pose = Pose()
                    pose.position.x = cx * 0.01  # Convert to meters
                    pose.position.y = cy * 0.01
                    pose.position.z = 0.5
                    pose.orientation.w = 1.0

                    grasps.append(pose)

        return grasps

    def pointcloud_callback(self, msg):
        """Process point cloud for 3D grasp detection"""
        try:
            # Convert point cloud to numpy array (simplified)
            # In practice, use sensor_msgs_py.point_cloud2.read_points
            points = self._pointcloud_to_array(msg)

            if self.gpu_available:
                grasp_poses = self._detect_3d_grasps_gpu(points)
            else:
                grasp_poses = self._detect_3d_grasps_cpu(points)

            # Publish 3D grasp poses
            if grasp_poses:
                pose_array_msg = PoseArray()
                pose_array_msg.header = msg.header
                pose_array_msg.poses = grasp_poses[:self.max_grasps]

                self.grasp_poses_pub.publish(pose_array_msg)

        except Exception as e:
            self.get_logger().error(f'3D grasp detection error: {e}')

    def _pointcloud_to_array(self, pointcloud_msg):
        """Convert PointCloud2 message to numpy array"""
        # This would use sensor_msgs_py.point_cloud2.read_points
        # For now, return a placeholder
        return np.random.rand(1000, 3).astype(np.float32)  # Placeholder

    def _detect_3d_grasps_gpu(self, points):
        """Detect 3D grasp poses using GPU acceleration"""
        try:
            # Upload points to GPU
            gpu_points = cp.asarray(points)

            # Apply GPU-accelerated 3D grasp detection
            grasp_candidates = self._gpu_3d_grasp_detection(gpu_points)

            # Filter and convert to ROS format
            high_quality_grasps = [g for g in grasp_candidates if g['score'] > self.min_grasp_score]

            grasp_poses = []
            for grasp in high_quality_grasps:
                pose = Pose()
                pose.position.x = grasp['position'][0]
                pose.position.y = grasp['position'][1]
                pose.position.z = grasp['position'][2]

                rotation = R.from_euler('xyz', grasp['orientation'], degrees=True)
                quat = rotation.as_quat()
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                grasp_poses.append(pose)

            return grasp_poses

        except Exception as e:
            self.get_logger().error(f'GPU 3D grasp detection error: {e}')
            return self._detect_3d_grasps_cpu(cp.asnumpy(gpu_points))

    def _gpu_3d_grasp_detection(self, points):
        """Apply GPU-accelerated 3D grasp detection"""
        # This would implement algorithms like:
        # - Antipodal grasp detection
        # - Geometric grasp analysis
        # - Learning-based grasp pose estimation

        # For demonstration, return simulated 3D grasps
        grasp_candidates = []

        # Find surface points
        if len(points) > 10:
            # Sample random points as potential grasp centers
            sample_indices = cp.random.choice(len(points), size=min(10, len(points)), replace=False)
            sampled_points = points[sample_indices]

            for point in sampled_points:
                grasp = {
                    'position': [float(point[0]), float(point[1]), float(point[2])],
                    'orientation': [0, 0, 0],  # Placeholder
                    'score': float(cp.random.uniform(0.6, 1.0))
                }
                grasp_candidates.append(grasp)

        return grasp_candidates

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
from std_msgs.msg import Float32
import numpy as np
import cupy as cp
from scipy.interpolate import CubicSpline

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

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/manipulation_goal',
            self.goal_callback,
            10
        )

        self.trajectory_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        # Motion planning parameters
        self.robot_state = JointState()
        self.planning_algorithm = 'rrt'  # Options: 'rrt', 'prm', 'ik'
        self.max_iterations = 1000
        self.smoothing_iterations = 50
        self.collision_check_resolution = 0.01  # 1cm resolution

        self.get_logger().info('Isaac Motion Planning node initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for motion planning"""
        try:
            # Check GPU availability
            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for motion planning')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU initialization failed: {e}, using CPU fallback')

    def joint_state_callback(self, msg):
        """Update current joint state"""
        self.current_joint_state = msg

    def goal_callback(self, msg):
        """Plan trajectory to reach goal pose"""
        if not self.current_joint_state:
            self.get_logger().warn('No current joint state available')
            return

        try:
            # Plan trajectory using GPU acceleration
            if self.gpu_available:
                trajectory = self._plan_trajectory_gpu(msg.pose, self.current_joint_state)
            else:
                trajectory = self._plan_trajectory_cpu(msg.pose, self.current_joint_state)

            if trajectory:
                self.trajectory_pub.publish(trajectory)
                self.get_logger().info(f'Trajectory planned with {len(trajectory.points)} points')
            else:
                self.get_logger().warn('Failed to plan trajectory')

        except Exception as e:
            self.get_logger().error(f'Trajectory planning error: {e}')

    def _plan_trajectory_gpu(self, goal_pose, current_state):
        """Plan trajectory using GPU acceleration"""
        try:
            # Convert joint state to GPU array
            current_positions = cp.array(current_state.position)

            # Define goal in task space (position and orientation)
            goal_pos = cp.array([goal_pose.position.x, goal_pose.position.y, goal_pose.position.z])
            goal_rot = cp.array([
                goal_pose.orientation.x, goal_pose.orientation.y,
                goal_pose.orientation.z, goal_pose.orientation.w
            ])

            # Use GPU-accelerated motion planning algorithm
            if self.planning_algorithm == 'rrt':
                joint_trajectory = self._gpu_rrt_planning(current_positions, goal_pos, goal_rot)
            elif self.planning_algorithm == 'ik':
                joint_trajectory = self._gpu_ik_planning(current_positions, goal_pos, goal_rot)
            else:
                joint_trajectory = self._gpu_rrt_planning(current_positions, goal_pos, goal_rot)

            if joint_trajectory:
                # Smooth trajectory using GPU
                smoothed_trajectory = self._smooth_trajectory_gpu(joint_trajectory)

                # Convert to ROS message format
                return self._trajectory_to_ros_msg(smoothed_trajectory, current_state.name)

        except Exception as e:
            self.get_logger().error(f'GPU trajectory planning error: {e}')
            # Fallback to CPU planning
            return self._plan_trajectory_cpu(goal_pose, current_state)

    def _gpu_rrt_planning(self, start_joints, goal_pos, goal_rot):
        """GPU-accelerated RRT planning"""
        try:
            # Initialize RRT tree on GPU (conceptual - real implementation would be more complex)
            # This would involve:
            # - GPU-parallelized nearest neighbor search
            # - GPU-parallelized collision checking
            # - GPU-parallelized path optimization

            # For demonstration, create a simple trajectory
            num_points = 20
            trajectory = []

            # Linear interpolation in joint space (simplified)
            for i in range(num_points + 1):
                t = i / num_points
                # This would be replaced with actual RRT sampling and connection
                interpolated_joints = start_joints + t * (goal_joints - start_joints)
                trajectory.append(interpolated_joints)

            return trajectory

        except Exception as e:
            self.get_logger().error(f'GPU RRT planning error: {e}')
            return None

    def _smooth_trajectory_gpu(self, trajectory):
        """Smooth trajectory using GPU acceleration"""
        try:
            # Convert trajectory to GPU array
            traj_array = cp.array(trajectory)

            # Apply GPU-based smoothing (e.g., cubic spline interpolation)
            smoothed_traj = cp.zeros_like(traj_array)

            # For each joint dimension, apply smoothing
            for joint_idx in range(traj_array.shape[1]):
                joint_trajectory = traj_array[:, joint_idx]

                # Create time vector
                time_vector = cp.linspace(0, 1, len(joint_trajectory))

                # Apply smoothing using GPU-accelerated spline
                spline = CubicSpline(cp.asnumpy(time_vector), cp.asnumpy(joint_trajectory))
                smoothed_values = spline(cp.asnumpy(time_vector))
                smoothed_traj[:, joint_idx] = cp.array(smoothed_values)

            return cp.asnumpy(smoothed_traj)

        except Exception as e:
            self.get_logger().error(f'GPU trajectory smoothing error: {e}')
            return trajectory

    def _trajectory_to_ros_msg(self, trajectory_array, joint_names):
        """Convert trajectory array to ROS JointTrajectory message"""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

        time_step = 0.1  # 100ms between points

        for i, joint_positions in enumerate(trajectory_array):
            point = JointTrajectoryPoint()
            point.positions = joint_positions.tolist()
            point.time_from_start.sec = int(i * time_step)
            point.time_from_start.nanosec = int((i * time_step % 1) * 1e9)

            # Add velocities and accelerations if available
            if i > 0 and i < len(trajectory_array) - 1:
                # Calculate velocities using finite differences
                prev_pos = trajectory_array[i-1]
                next_pos = trajectory_array[i+1]
                velocities = (next_pos - prev_pos) / (2 * time_step)
                point.velocities = velocities.tolist()

            traj_msg.points.append(point)

        return traj_msg

    def _plan_trajectory_cpu(self, goal_pose, current_state):
        """CPU fallback for trajectory planning"""
        # Implement CPU-based trajectory planning
        # This would use OMPL, MoveIt, or other CPU-based planners
        pass

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
import numpy as np
import cupy as cp
from scipy.optimize import minimize
import math

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

        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            '/ik_target',
            self.target_pose_callback,
            10
        )

        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # IK parameters
        self.current_joint_state = JointState()
        self.arm_chain_joints = ['shoulder_joint', 'elbow_joint', 'wrist_joint']  # Example joints
        self.position_tolerance = 0.01  # 1cm tolerance
        self.orientation_tolerance = 0.1  # 0.1 rad tolerance

        # Robot kinematic parameters (simplified 3-DOF arm)
        self.link_lengths = [0.3, 0.3, 0.1]  # Example link lengths

        self.get_logger().info('Isaac Inverse Kinematics solver initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for IK solving"""
        try:
            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for IK solving')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU initialization failed: {e}, using CPU fallback')

    def joint_state_callback(self, msg):
        """Update current joint state"""
        self.current_joint_state = msg

    def target_pose_callback(self, msg):
        """Solve IK for target pose"""
        try:
            # Solve IK using GPU acceleration
            if self.gpu_available:
                joint_solution = self._solve_ik_gpu(msg.pose, self.current_joint_state)
            else:
                joint_solution = self._solve_ik_cpu(msg.pose, self.current_joint_state)

            if joint_solution is not None:
                # Create and publish joint command
                joint_cmd = JointState()
                joint_cmd.header = msg.header
                joint_cmd.name = self.current_joint_state.name
                joint_cmd.position = joint_solution

                self.joint_cmd_pub.publish(joint_cmd)

                self.get_logger().info(f'IK solution published: {joint_solution}')

            else:
                self.get_logger().warn('IK solution not found')

        except Exception as e:
            self.get_logger().error(f'IK solving error: {e}')

    def _solve_ik_gpu(self, target_pose, current_state):
        """Solve inverse kinematics using GPU acceleration"""
        try:
            # Extract target position and orientation
            target_pos = cp.array([
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z
            ])

            target_quat = cp.array([
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w
            ])

            # Get current joint positions
            current_joints = cp.array(current_state.position)

            # Solve IK using GPU-optimized algorithm
            # This would typically use GPU-parallelized optimization
            solution = self._gpu_ik_optimization(target_pos, target_quat, current_joints)

            if solution is not None:
                return cp.asnumpy(solution)
            else:
                return None

        except Exception as e:
            self.get_logger().error(f'GPU IK solving error: {e}')
            return self._solve_ik_cpu(target_pose, current_state)

    def _gpu_ik_optimization(self, target_pos, target_quat, current_joints):
        """GPU-optimized IK optimization"""
        try:
            # Define objective function for GPU optimization
            def objective_function_gpu(joint_angles):
                # Calculate forward kinematics on GPU
                fk_pos, fk_quat = self._gpu_forward_kinematics(joint_angles)

                # Calculate position error
                pos_error = cp.linalg.norm(target_pos - fk_pos)

                # Calculate orientation error
                # Using quaternion difference
                quat_diff = self._gpu_quaternion_difference(target_quat, fk_quat)
                orient_error = cp.arccos(cp.abs(quat_diff[3])) * 2  # Angle between quaternions

                # Combined error
                total_error = pos_error + orient_error

                return total_error

            # Use GPU-parallelized optimization
            # In practice, this would use custom CUDA kernels
            # For now, we'll use a simplified approach

            # Initialize optimization parameters
            initial_guess = current_joints

            # This would be replaced with GPU-parallelized optimization
            # For demonstration, return a simple solution
            return self._analytical_3dof_ik(target_pos, target_quat)

        except Exception as e:
            self.get_logger().error(f'GPU IK optimization error: {e}')
            return None

    def _gpu_forward_kinematics(self, joint_angles):
        """GPU-accelerated forward kinematics"""
        try:
            # Simplified 3-DOF arm forward kinematics
            theta1, theta2, theta3 = joint_angles[:3]

            # Calculate end-effector position
            x = self.link_lengths[0] * cp.cos(theta1) + self.link_lengths[1] * cp.cos(theta1 + theta2) + self.link_lengths[2] * cp.cos(theta1 + theta2 + theta3)
            y = self.link_lengths[0] * cp.sin(theta1) + self.link_lengths[1] * cp.sin(theta1 + theta2) + self.link_lengths[2] * cp.sin(theta1 + theta2 + theta3)
            z = 0  # Simplified 2D case

            ee_pos = cp.array([x, y, z])

            # Calculate end-effector orientation
            total_angle = theta1 + theta2 + theta3
            ee_quat = cp.array([0, 0, cp.sin(total_angle/2), cp.cos(total_angle/2)])

            return ee_pos, ee_quat

        except Exception as e:
            self.get_logger().error(f'GPU FK error: {e}')
            return cp.zeros(3), cp.array([0, 0, 0, 1])

    def _gpu_quaternion_difference(self, q1, q2):
        """Calculate quaternion difference on GPU"""
        # Calculate quaternion difference: q1 * inverse(q2)
        q2_inv = cp.array([q2[0], q2[1], q2[2], -q2[3]])  # Conjugate
        q_diff = self._gpu_quaternion_multiply(q1, q2_inv)
        return q_diff / cp.linalg.norm(q_diff)  # Normalize

    def _gpu_quaternion_multiply(self, q1, q2):
        """Multiply quaternions on GPU"""
        w = q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2]
        x = q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1]
        y = q1[3]*q2[1] - q1[0]*q2[2] + q1[1]*q2[3] + q1[2]*q2[0]
        z = q1[3]*q2[2] + q1[0]*q2[1] - q1[1]*q2[0] + q1[2]*q2[3]

        return cp.array([x, y, z, w])

    def _analytical_3dof_ik(self, target_pos, target_quat):
        """Analytical IK solution for 3-DOF arm (simplified)"""
        try:
            # Simplified analytical solution for 3-DOF planar arm
            x, y, z = target_pos

            # Calculate distance to target
            r = cp.sqrt(x*x + y*y)

            # Check if target is reachable
            l1, l2, l3 = self.link_lengths
            l_total = l1 + l2 + l3

            if r > l_total:
                # Target too far
                scale = l_total / r
                x *= scale
                y *= scale
                r = l_total

            if r < abs(l1 - l2):
                # Target too close
                return None

            # Calculate joint angles
            cos_theta2 = (l1*l1 + l2*l2 - r*r) / (2*l1*l2)
            cos_theta2 = cp.clip(cos_theta2, -1, 1)  # Clamp to valid range
            theta2 = cp.arccos(cos_theta2)

            k1 = l1 + l2 * cp.cos(theta2)
            k2 = l2 * cp.sin(theta2)

            theta1 = cp.arctan2(y, x) - cp.arctan2(k2, k1)
            theta3 = 0  # Simplified - no wrist rotation

            return cp.array([theta1, theta2, theta3])

        except Exception as e:
            self.get_logger().error(f'Analytical IK error: {e}')
            return None

    def _solve_ik_cpu(self, target_pose, current_state):
        """CPU fallback for IK solving"""
        # Implement CPU-based IK solver
        # This would use analytical methods, numerical optimization, or libraries like ikpy
        pass

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
from geometry_msgs.msg import WrenchStamped, Twist
from std_msgs.msg import Bool, Float32
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np
import cupy as cp
from scipy.spatial.transform import Rotation as R

class IsaacManipulationController(Node):
    def __init__(self):
        super().__init__('isaac_manipulation_controller')

        # Initialize GPU resources
        self._initialize_gpu_resources()

        # Publishers and subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.joint_trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.trajectory_callback,
            10
        )

        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            '/wrench',
            self.wrench_callback,
            10
        )

        self.cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.gripper_cmd_pub = self.create_publisher(Bool, '/gripper_command', 10)

        # Control parameters
        self.current_joint_state = JointState()
        self.desired_joint_state = JointState()
        self.control_frequency = 100  # 100 Hz control loop
        self.position_gains = [10.0] * 6  # Example gains for 6 joints
        self.velocity_gains = [1.0] * 6

        # Gripper control
        self.gripper_open = True

        # Initialize control timer
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

    def trajectory_callback(self, msg):
        """Update desired trajectory"""
        self.desired_trajectory = msg

    def wrench_callback(self, msg):
        """Process force/torque feedback"""
        self.current_wrench = msg

    def control_loop(self):
        """Main control loop with GPU acceleration"""
        if not self.current_joint_state or not self.desired_trajectory:
            return

        try:
            # Get desired joint state from trajectory
            desired_state = self._get_desired_state_from_trajectory()

            # Calculate control commands using GPU acceleration
            if self.gpu_available:
                control_commands = self._calculate_gpu_control(
                    self.current_joint_state,
                    desired_state
                )
            else:
                control_commands = self._calculate_cpu_control(
                    self.current_joint_state,
                    desired_state
                )

            # Apply force control if force feedback is available
            if hasattr(self, 'current_wrench'):
                control_commands = self._apply_force_control(control_commands)

            # Publish control commands
            self.cmd_pub.publish(control_commands)

        except Exception as e:
            self.get_logger().error(f'Manipulation control error: {e}')

    def _get_desired_state_from_trajectory(self):
        """Get desired joint state from trajectory at current time"""
        # This would interpolate the trajectory based on current time
        # For now, return the first point as an example
        if self.desired_trajectory.points:
            point = self.desired_trajectory.points[0]
            desired_state = JointState()
            desired_state.name = self.desired_trajectory.joint_names
            desired_state.position = point.positions
            if point.velocities:
                desired_state.velocity = point.velocities
            if point.effort:
                desired_state.effort = point.effort
            return desired_state
        else:
            return self.current_joint_state

    def _calculate_gpu_control(self, current_state, desired_state):
        """Calculate control commands using GPU acceleration"""
        try:
            # Convert to GPU arrays
            current_pos = cp.array(current_state.position)
            desired_pos = cp.array(desired_state.position)

            # Calculate position error
            pos_error = desired_pos - current_pos

            # Calculate control effort using PD control (GPU accelerated)
            if hasattr(desired_state, 'velocity') and desired_state.velocity:
                current_vel = cp.array(current_state.velocity)
                desired_vel = cp.array(desired_state.velocity)
                vel_error = desired_vel - current_vel

                # PD control with GPU acceleration
                position_term = cp.multiply(cp.array(self.position_gains), pos_error)
                velocity_term = cp.multiply(cp.array(self.velocity_gains), vel_error)
                control_effort = position_term + velocity_term
            else:
                # P control only
                control_effort = cp.multiply(cp.array(self.position_gains), pos_error)

            # Create control command message
            control_msg = JointState()
            control_msg.header.stamp = self.get_clock().now().to_msg()
            control_msg.name = desired_state.name
            control_msg.position = cp.asnumpy(control_effort).tolist()

            return control_msg

        except Exception as e:
            self.get_logger().error(f'GPU control calculation error: {e}')
            return self._calculate_cpu_control(current_state, desired_state)

    def _apply_force_control(self, control_commands):
        """Apply force control based on wrench feedback"""
        if not hasattr(self, 'current_wrench'):
            return control_commands

        # Check if forces exceed thresholds (indicating contact)
        force_threshold = 10.0  # 10N threshold
        torque_threshold = 1.0  # 1Nm threshold

        current_force = self.current_wrench.wrench.force
        current_torque = self.current_wrench.wrench.torque

        force_magnitude = math.sqrt(
            current_force.x**2 + current_force.y**2 + current_force.z**2
        )

        torque_magnitude = math.sqrt(
            current_torque.x**2 + current_torque.y**2 + current_torque.z**2
        )

        if force_magnitude > force_threshold or torque_magnitude > torque_threshold:
            # Apply impedance control or admittance control
            # This would modify control commands based on contact forces
            pass

        return control_commands

    def _calculate_cpu_control(self, current_state, desired_state):
        """CPU fallback for control calculation"""
        # Implement CPU-based control algorithm
        control_msg = JointState()
        control_msg.header.stamp = self.get_clock().now().to_msg()
        control_msg.name = desired_state.name

        # Simple PD control on CPU
        positions = []
        for i, (curr_pos, des_pos) in enumerate(zip(current_state.position, desired_state.position)):
            pos_error = des_pos - curr_pos
            control_effort = self.position_gains[i] * pos_error
            positions.append(curr_pos + control_effort * 0.01)  # Simple integration

        control_msg.position = positions
        return control_msg

    def grasp_object(self):
        """Execute grasping motion"""
        self.gripper_open = False
        cmd_msg = Bool()
        cmd_msg.data = False  # Close gripper
        self.gripper_cmd_pub.publish(cmd_msg)

    def release_object(self):
        """Execute releasing motion"""
        self.gripper_open = True
        cmd_msg = Bool()
        cmd_msg.data = True  # Open gripper
        self.gripper_cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacManipulationController()

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

## Isaac Manipulation Launch Configuration

### Complete Manipulation System Launch

```python
# manipulation_launch.py
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
    enable_gpu_manipulation = LaunchConfiguration('enable_gpu_manipulation', default='true')
    enable_perception = LaunchConfiguration('enable_perception', default='true')
    enable_control = LaunchConfiguration('enable_control', default='true')

    # Isaac Manipulation Container
    manipulation_container = ComposableNodeContainer(
        name='isaac_manipulation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_gpu_manipulation),
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_grasp_pose_detection',
                plugin='nvidia::isaac_ros::grasp_pose_detection::GraspPoseDetectionNode',
                name='grasp_pose_detection',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'enable_gpu_acceleration': True,
                    'min_grasp_score': 0.7,
                    'max_grasps': 10,
                    'approach_distance': 0.1
                }],
                remappings=[
                    ('/grasp_pose_detection/image', '/camera/image_rect_color'),
                    ('/grasp_pose_detection/pointcloud', '/point_cloud'),
                    ('/grasp_pose_detection/grasps', '/potential_grasps')
                ]
            ),
            ComposableNode(
                package='isaac_ros_motion_planning',
                plugin='nvidia::isaac_ros::motion_planning::MotionPlanningNode',
                name='motion_planning',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'enable_gpu_planning': True,
                    'planning_algorithm': 'rrt',
                    'max_iterations': 1000,
                    'smoothing_iterations': 50
                }],
                remappings=[
                    ('/motion_planning/joint_states', '/joint_states'),
                    ('/motion_planning/goal', '/manipulation_goal'),
                    ('/motion_planning/trajectory', '/joint_trajectory')
                ]
            ),
            ComposableNode(
                package='isaac_ros_inverse_kinematics',
                plugin='nvidia::isaac_ros::inverse_kinematics::InverseKinematicsNode',
                name='inverse_kinematics',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'enable_gpu_ik': True,
                    'position_tolerance': 0.01,
                    'orientation_tolerance': 0.1
                }],
                remappings=[
                    ('/inverse_kinematics/target_pose', '/ik_target'),
                    ('/inverse_kinematics/current_joints', '/joint_states'),
                    ('/inverse_kinematics/solution', '/ik_solution')
                ]
            ),
            ComposableNode(
                package='isaac_ros_manipulation_controller',
                plugin='nvidia::isaac_ros::manipulation_controller::ManipulationControllerNode',
                name='manipulation_controller',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'enable_gpu_control': True,
                    'control_frequency': 100,
                    'position_gains': [10.0, 10.0, 10.0, 5.0, 5.0, 5.0],
                    'velocity_gains': [1.0, 1.0, 1.0, 0.5, 0.5, 0.5]
                }],
                remappings=[
                    ('/manipulation_controller/joint_states', '/joint_states'),
                    ('/manipulation_controller/trajectory', '/joint_trajectory'),
                    ('/manipulation_controller/commands', '/joint_commands'),
                    ('/manipulation_controller/wrench', '/wrench')
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
            'enable_gpu_manipulation',
            default_value='true',
            description='Enable GPU-accelerated manipulation components'
        ),
        DeclareLaunchArgument(
            'enable_perception',
            default_value='true',
            description='Enable Isaac perception components'
        ),
        DeclareLaunchArgument(
            'enable_control',
            default_value='true',
            description='Enable Isaac control components'
        ),
        manipulation_container
    ])
```

## Isaac Manipulation Best Practices

### Performance Optimization

```python
# manipulation_best_practices.py
class IsaacManipulationBestPractices:
    def __init__(self):
        self.practices = {
            'gpu_optimization': [
                "Use appropriate batch sizes for GPU operations",
                "Minimize CPU-GPU memory transfers",
                "Pre-allocate GPU memory pools",
                "Use CUDA streams for overlapping computation",
                "Optimize kernel launch parameters for your GPU",
                "Leverage TensorRT for deep learning inference"
            ],
            'control_optimization': [
                "Implement cascaded control loops for stability",
                "Use appropriate control gains for your robot",
                "Implement anti-windup mechanisms",
                "Apply feedforward control where appropriate",
                "Use trajectory smoothing for smooth motions",
                "Implement safety limits and constraints"
            ],
            'planning_optimization': [
                "Use bidirectional planning algorithms",
                "Implement any-angle path planning",
                "Apply dynamic replanning when needed",
                "Use kinodynamic planning for dynamic systems",
                "Implement multi-modal planning for complex tasks",
                "Validate plans before execution"
            ],
            'integration_practices': [
                "Validate GPU results with CPU fallbacks",
                "Monitor GPU utilization continuously",
                "Implement proper error handling",
                "Use appropriate QoS settings for real-time requirements",
                "Log performance metrics for optimization",
                "Implement graceful degradation on failures"
            ]
        }

    def get_practices_by_category(self, category):
        """Get best practices for specific category"""
        if category in self.practices:
            return self.practices[category]
        else:
            return self.practices['gpu_optimization']  # Default

    def print_all_practices(self):
        """Print all Isaac manipulation best practices"""
        print("Isaac Manipulation Best Practices")
        print("=" * 50)

        for category, practices in self.practices.items():
            print(f"\n{category.replace('_', ' ').title()}:")
            for practice in practices:
                print(f"  • {practice}")

def validate_manipulation_setup(robot_description):
    """Validate Isaac manipulation setup"""
    validation_results = {
        'gpu_available': check_gpu_availability(),
        'cuda_compatible': check_cuda_compatibility(),
        'tensorrt_available': check_tensorrt_availability(),
        'robot_description_valid': validate_urdf(robot_description),
        'kinematic_chain_valid': validate_kinematic_chains(robot_description),
        'joint_limits_valid': validate_joint_limits(robot_description)
    }

    print("Isaac Manipulation Setup Validation:")
    for component, valid in validation_results.items():
        status = "✓" if valid else "✗"
        print(f"  {status} {component}: {'Valid' if valid else 'Invalid'}")

    return all(validation_results.values())

def check_gpu_availability():
    """Check if GPU is available for Isaac manipulation"""
    try:
        import GPUtil
        gpus = GPUtil.getGPUs()
        return len(gpus) > 0
    except ImportError:
        return False

def check_cuda_compatibility():
    """Check CUDA compatibility"""
    try:
        import pycuda.driver as cuda
        cuda.init()
        return cuda.Device.count() > 0
    except ImportError:
        return False

def check_tensorrt_availability():
    """Check TensorRT availability"""
    try:
        import tensorrt as trt
        return trt.Builder(trt.Logger(trt.Logger.WARNING)) is not None
    except ImportError:
        return False

def validate_urdf(urdf_path):
    """Validate URDF for manipulation"""
    # This would validate the URDF for proper joint definitions, limits, etc.
    return True  # Simplified validation

def validate_kinematic_chains(urdf_path):
    """Validate kinematic chains for manipulation"""
    # This would validate that kinematic chains are properly defined
    return True  # Simplified validation

def validate_joint_limits(urdf_path):
    """Validate joint limits for safe operation"""
    # This would validate that joint limits are appropriate for manipulation
    return True  # Simplified validation

def main():
    practices = IsaacManipulationBestPractices()
    practices.print_all_practices()

    print("\n" + "="*50)
    print("Setup Validation:")
    is_valid = validate_manipulation_setup("/path/to/robot.urdf")
    print(f"Overall validation: {'PASSED' if is_valid else 'FAILED'}")

if __name__ == '__main__':
    main()
```

## Troubleshooting Isaac Manipulation

### Common Issues and Solutions

```python
# manipulation_troubleshooting.py
class IsaacManipulationTroubleshooting:
    def __init__(self):
        self.common_issues = {
            'gpu_memory_error': {
                'symptoms': ['CUDA_ERROR_OUT_OF_MEMORY', 'Memory allocation failed', 'GPU operations failing'],
                'causes': ['Insufficient GPU memory', 'Memory leaks', 'Too large batch sizes'],
                'solutions': [
                    'Reduce batch sizes for processing',
                    'Implement proper GPU memory cleanup',
                    'Use memory pools for efficient allocation',
                    'Monitor GPU memory usage continuously'
                ]
            },
            'ik_failure': {
                'symptoms': ['No IK solution found', 'Joint limits exceeded', 'Singularity errors'],
                'causes': ['Target unreachable', 'Poor initial guess', 'Joint limit violations'],
                'solutions': [
                    'Verify target pose is within robot workspace',
                    'Provide better initial joint configuration',
                    'Check and validate joint limits in URDF',
                    'Use multiple IK attempts with different initial states'
                ]
            },
            'control_instability': {
                'symptoms': ['Oscillatory behavior', 'Vibration', 'Unstable tracking'],
                'causes': ['Improper control gains', 'Sampling rate issues', 'Mechanical compliance'],
                'solutions': [
                    'Tune control gains appropriately',
                    'Ensure adequate sampling rate',
                    'Consider mechanical compliance in control design',
                    'Implement notch filters for resonant frequencies'
                ]
            },
            'perception_noise': {
                'symptoms': ['Inconsistent detections', 'Noisy sensor data', 'False positives'],
                'causes': ['Poor lighting conditions', 'Sensor calibration', 'Environmental factors'],
                'solutions': [
                    'Improve lighting conditions',
                    'Recalibrate sensors',
                    'Apply appropriate filtering',
                    'Increase sensor integration time'
                ]
            }
        }

    def diagnose_issue(self, error_message):
        """Diagnose issue based on error message"""
        for issue_type, info in self.common_issues.items():
            for symptom in info['symptoms']:
                if symptom.lower() in error_message.lower():
                    return {
                        'issue_type': issue_type,
                        'symptoms': info['symptoms'],
                        'causes': info['causes'],
                        'solutions': info['solutions']
                    }

        return None

    def get_solutions(self, issue_type):
        """Get solutions for specific issue type"""
        if issue_type in self.common_issues:
            return self.common_issues[issue_type]['solutions']
        else:
            return ["Check Isaac documentation and community forums"]

def main():
    troubleshooter = IsaacManipulationTroubleshooting()

    print("Isaac Manipulation Troubleshooting Guide")
    print("=" * 50)

    for issue_type, info in troubleshooter.common_issues.items():
        print(f"\nIssue: {issue_type.replace('_', ' ').title()}")
        print(f"Symptoms: {', '.join(info['symptoms'])}")
        print(f"Causes: {', '.join(info['causes'])}")
        print(f"Solutions: {', '.join(info['solutions'])}")

if __name__ == '__main__':
    main()
```

## Advanced Manipulation Techniques

### Learning-Based Manipulation

```python
# learning_based_manipulation.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose, WrenchStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import torch
import torch.nn as nn
import cupy as cp

class IsaacLearningBasedManipulationNode(Node):
    def __init__(self):
        super().__init__('isaac_learning_based_manipulation')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize deep learning model
        self._initialize_learning_model()

        # Publishers and subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.wrench_sub = self.create_subscription(WrenchStamped, '/wrench', self.wrench_callback, 10)

        self.action_pub = self.create_publisher(JointState, '/learned_actions', 10)

        # Learning state
        self.current_image = None
        self.current_joints = None
        self.current_wrench = None

        self.get_logger().info('Isaac Learning-Based Manipulation node initialized')

    def _initialize_learning_model(self):
        """Initialize learning-based manipulation model"""
        try:
            # Check if GPU is available for model inference
            self.gpu_available = torch.cuda.is_available()

            if self.gpu_available:
                self.device = torch.device('cuda')
                self.get_logger().info('Learning model will use GPU')
            else:
                self.device = torch.device('cpu')
                self.get_logger().info('Learning model will use CPU')

            # Initialize neural network model
            # This would typically be a pre-trained model for manipulation
            self.manipulation_model = self._create_manipulation_network()
            self.manipulation_model.to(self.device)
            self.manipulation_model.eval()

            self.get_logger().info('Learning-based manipulation model initialized')

        except Exception as e:
            self.get_logger().error(f'Learning model initialization error: {e}')
            self.learning_enabled = False

    def _create_manipulation_network(self):
        """Create neural network for learning-based manipulation"""
        class ManipulationNet(nn.Module):
            def __init__(self, input_dim=224*224*3, joint_dim=6, output_dim=6):
                super().__init__()

                # Vision processing branch
                self.vision_branch = nn.Sequential(
                    nn.Conv2d(3, 32, 3, padding=1),
                    nn.ReLU(),
                    nn.MaxPool2d(2),
                    nn.Conv2d(32, 64, 3, padding=1),
                    nn.ReLU(),
                    nn.MaxPool2d(2),
                    nn.Flatten(),
                    nn.Linear(64 * 56 * 56, 512),  # Adjust based on input size
                    nn.ReLU()
                )

                # Joint state processing branch
                self.joint_branch = nn.Sequential(
                    nn.Linear(joint_dim, 128),
                    nn.ReLU(),
                    nn.Linear(128, 256),
                    nn.ReLU()
                )

                # Fusion layer
                self.fusion = nn.Sequential(
                    nn.Linear(512 + 256, 512),
                    nn.ReLU(),
                    nn.Dropout(0.2),
                    nn.Linear(512, 256),
                    nn.ReLU(),
                    nn.Linear(256, output_dim)
                )

            def forward(self, image, joints):
                vision_features = self.vision_branch(image)
                joint_features = self.joint_branch(joints)

                combined = torch.cat([vision_features, joint_features], dim=1)
                output = self.fusion(combined)

                return output

        # For demonstration, return a simple model
        # In practice, this would load a pre-trained model
        return ManipulationNet()

    def image_callback(self, msg):
        """Process image for learning-based manipulation"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Preprocess image for neural network
            input_tensor = self._preprocess_image(cv_image)
            input_tensor = input_tensor.to(self.device)

            self.current_image = input_tensor

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def joint_state_callback(self, msg):
        """Update joint state for learning model"""
        try:
            joint_tensor = torch.tensor(msg.position, dtype=torch.float32).unsqueeze(0).to(self.device)
            self.current_joints = joint_tensor

            # If we have both image and joint data, perform inference
            if self.current_image is not None:
                self._perform_learning_inference()

        except Exception as e:
            self.get_logger().error(f'Joint state processing error: {e}')

    def _preprocess_image(self, image):
        """Preprocess image for neural network"""
        # Resize image to model input size
        resized = cv2.resize(image, (224, 224))

        # Convert to tensor and normalize
        tensor_image = torch.from_numpy(resized).permute(2, 0, 1).float() / 255.0
        tensor_image = tensor_image.unsqueeze(0)  # Add batch dimension

        return tensor_image

    def _perform_learning_inference(self):
        """Perform learning-based manipulation inference"""
        try:
            with torch.no_grad():
                # Get action prediction from neural network
                predicted_action = self.manipulation_model(
                    self.current_image,
                    self.current_joints
                )

                # Convert to ROS message
                action_msg = JointState()
                action_msg.header.stamp = self.get_clock().now().to_msg()
                action_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # Example names
                action_msg.position = predicted_action.cpu().numpy().tolist()[0]

                self.action_pub.publish(action_msg)

                self.get_logger().info('Learning-based manipulation action published')

        except Exception as e:
            self.get_logger().error(f'Learning inference error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = IsaacLearningBasedManipulationNode()

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

## Summary

Isaac Manipulation provides powerful GPU-accelerated capabilities for robotic manipulation tasks. By leveraging NVIDIA's hardware acceleration, you can achieve:

- **Real-time grasp pose detection** using GPU-accelerated computer vision
- **High-performance motion planning** with GPU-parallelized algorithms
- **Efficient inverse kinematics** solving with GPU optimization
- **Responsive manipulation control** using GPU-accelerated control algorithms
- **Learning-based manipulation** with GPU-accelerated deep learning

The integration of these capabilities creates a comprehensive manipulation system that can handle complex tasks with the performance required for humanoid robot applications.

## Next Steps

In the next module, we'll explore the Vision-Language-Action (VLA) pipeline that brings together all the perception, planning, and control capabilities we've developed into a unified AI system for humanoid robots. This will enable robots to understand natural language commands, perceive their environment visually, and execute complex actions with human-like understanding.