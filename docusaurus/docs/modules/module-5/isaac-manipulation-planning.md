# Isaac Manipulation Planning

In this chapter, we'll explore NVIDIA Isaac's manipulation planning capabilities, which provide GPU-accelerated solutions for generating complex manipulation trajectories and behaviors. Isaac's manipulation planning leverages GPU computing to solve complex kinematic and dynamic problems in real-time, enabling humanoid robots to perform dexterous manipulation tasks with precision and efficiency.

## Isaac Manipulation Planning Overview

### Core Manipulation Planning Components

Isaac Manipulation Planning includes several GPU-accelerated planning packages:

- **Isaac ROS Grasp Planning**: GPU-accelerated grasp pose generation and evaluation
- **Isaac ROS Trajectory Optimization**: GPU-parallelized trajectory optimization
- **Isaac ROS Inverse Kinematics**: GPU-accelerated IK solving with multiple solutions
- **Isaac ROS Collision Checking**: GPU-parallelized collision detection
- **Isaac ROS Task Planning**: GPU-accelerated high-level task planning
- **Isaac ROS Motion Primitives**: Pre-computed motion patterns for efficient execution

### Manipulation Planning Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                    Isaac Manipulation Planning Stack                           │
├─────────────────────────────────────────────────────────────────────────────────┤
│  Task Specification → Grasp Planning → IK Solution → Trajectory → Execution   │
│  (Language, Vision)   (GPU Accel)    (GPU Accel)   (GPU Opt)  (Robot)      │
├─────────────────────────────────────────────────────────────────────────────────┤
│  GPU-Accelerated Components:                                                 │
│  - Parallel grasp evaluation across multiple poses                           │
│  - GPU-parallelized inverse kinematics                                       │
│  - CUDA-optimized trajectory optimization                                    │
│  - Real-time collision checking                                              │
│  - Multi-armed coordination                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Isaac Grasp Planning

### GPU-Accelerated Grasp Generation

```python
# isaac_grasp_planning.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float32
from builtin_interfaces.msg import Time
import numpy as np
import cupy as cp  # GPU acceleration
from scipy.spatial.transform import Rotation as R
import math

class IsaacGraspPlanningNode(Node):
    def __init__(self):
        super().__init__('isaac_grasp_planning')

        # Initialize GPU resources
        self._initialize_gpu_resources()

        # Publishers and subscribers
        self.object_pose_sub = self.create_subscription(
            Pose,
            '/target_object_pose',
            self.object_pose_callback,
            10
        )

        self.grasp_poses_pub = self.create_publisher(PoseArray, '/candidate_grasps', 10)
        self.best_grasp_pub = self.create_publisher(Pose, '/best_grasp', 10)
        self.grasp_quality_pub = self.create_publisher(Float32, '/grasp_quality', 10)

        # Grasp planning parameters
        self.grasp_approach_distance = 0.15  # 15cm approach distance
        self.grasp_retreat_distance = 0.10   # 10cm retreat distance
        self.max_grasp_attempts = 20
        self.min_grasp_quality = 0.6
        self.gripper_width = 0.08  # 8cm gripper width
        self.gripper_height = 0.02  # 2cm gripper height

        # Robot-specific parameters
        self.robot_config = {
            'arm_dof': 7,
            'gripper_joint_names': ['left_gripper_joint', 'right_gripper_joint'],
            'arm_joint_names': [
                'left_shoulder_pan_joint', 'left_shoulder_lift_joint',
                'left_elbow_joint', 'left_wrist_1_joint', 'left_wrist_2_joint', 'left_wrist_3_joint'
            ]
        }

        # Grasp types to generate
        self.grasp_types = [
            'top_grasp',     # Grasp from above
            'side_grasp',    # Grasp from the side
            'pinch_grasp',   # Pinch grasp for small objects
            'power_grasp',   # Power grasp for large objects
            'wrap_grasp'     # Wrap grasp for irregular objects
        ]

        self.get_logger().info('Isaac Grasp Planning node initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for grasp planning"""
        try:
            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for grasp planning')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU initialization failed: {e}, using CPU fallback')

    def object_pose_callback(self, msg):
        """Generate grasp poses for target object"""
        try:
            if self.gpu_available:
                grasp_poses = self._generate_grasp_poses_gpu(msg)
            else:
                grasp_poses = self._generate_grasp_poses_cpu(msg)

            if grasp_poses:
                # Evaluate grasp quality using GPU acceleration
                evaluated_grasps = self._evaluate_grasp_quality_gpu(grasp_poses, msg)

                # Filter high-quality grasps
                high_quality_grasps = [
                    grasp for grasp in evaluated_grasps
                    if grasp['quality'] > self.min_grasp_quality
                ]

                if high_quality_grasps:
                    # Sort by quality and select best grasps
                    sorted_grasps = sorted(high_quality_grasps, key=lambda x: x['quality'], reverse=True)

                    # Publish candidate grasps
                    candidate_array = PoseArray()
                    candidate_array.header.frame_id = 'map'
                    candidate_array.header.stamp = self.get_clock().now().to_msg()

                    for grasp in sorted_grasps[:5]:  # Top 5 grasps
                        candidate_array.poses.append(grasp['pose'])

                    self.grasp_poses_pub.publish(candidate_array)

                    # Publish best grasp
                    best_grasp = sorted_grasps[0]
                    best_grasp_msg = Pose()
                    best_grasp_msg = best_grasp['pose']
                    self.best_grasp_pub.publish(best_grasp_msg)

                    # Publish quality metric
                    quality_msg = Float32()
                    quality_msg.data = float(best_grasp['quality'])
                    self.grasp_quality_pub.publish(quality_msg)

                    self.get_logger().info(
                        f'Generated {len(candidate_array.poses)} candidate grasps, '
                        f'best quality: {best_grasp["quality"]:.3f}'
                    )

        except Exception as e:
            self.get_logger().error(f'Grasp planning error: {e}')

    def _generate_grasp_poses_gpu(self, object_pose):
        """Generate grasp poses using GPU acceleration"""
        try:
            # Convert object pose to GPU array
            obj_pos = cp.array([object_pose.position.x, object_pose.position.y, object_pose.position.z])
            obj_rot = cp.array([
                object_pose.orientation.x,
                object_pose.orientation.y,
                object_pose.orientation.z,
                object_pose.orientation.w
            ])

            # Generate grasp candidates in parallel on GPU
            grasp_candidates = []

            # Generate different grasp types
            for grasp_type in self.grasp_types:
                if grasp_type == 'top_grasp':
                    top_grasps = self._generate_top_grasps_gpu(obj_pos, obj_rot)
                    grasp_candidates.extend(top_grasps)
                elif grasp_type == 'side_grasp':
                    side_grasps = self._generate_side_grasps_gpu(obj_pos, obj_rot)
                    grasp_candidates.extend(side_grasps)
                # Add other grasp types as needed

            return grasp_candidates

        except Exception as e:
            self.get_logger().error(f'GPU grasp generation error: {e}')
            return self._generate_grasp_poses_cpu(object_pose)

    def _generate_top_grasps_gpu(self, obj_pos, obj_rot):
        """Generate top-down grasp poses using GPU"""
        try:
            grasp_poses = []

            # Generate multiple top-down grasp poses around the object
            for angle in cp.linspace(0, 2*math.pi, 8):  # 8 positions around object
                for height_offset in cp.array([0.05, 0.1, 0.15]):  # Different approach heights
                    # Calculate grasp position above object
                    offset_x = 0.1 * cp.cos(angle)  # 10cm offset
                    offset_y = 0.1 * cp.sin(angle)  # 10cm offset
                    offset_z = height_offset

                    grasp_pos = obj_pos + cp.array([offset_x, offset_y, offset_z])

                    # Calculate grasp orientation (pointing down)
                    # For top grasp, gripper should point down (negative Z)
                    grasp_rot = cp.array([0.0, 0.0, 0.0, 1.0])  # Identity quaternion

                    # Create grasp pose
                    grasp = {
                        'position': cp.asnumpy(grasp_pos),
                        'orientation': cp.asnumpy(grasp_rot),
                        'type': 'top',
                        'quality': 0.0  # Will be evaluated later
                    }

                    grasp_poses.append(grasp)

            return grasp_poses

        except Exception as e:
            self.get_logger().error(f'GPU top grasp generation error: {e}')
            return []

    def _generate_side_grasps_gpu(self, obj_pos, obj_rot):
        """Generate side grasp poses using GPU"""
        try:
            grasp_poses = []

            # Generate side grasp poses at different orientations
            for angle in cp.linspace(0, 2*math.pi, 8):  # 8 orientations
                # Approach from side
                offset_x = 0.1 * cp.cos(angle)  # 10cm offset
                offset_y = 0.1 * cp.sin(angle)  # 10cm offset
                offset_z = 0.0  # Same height as object center

                grasp_pos = obj_pos + cp.array([offset_x, offset_y, offset_z])

                # Calculate orientation to face object
                # This is simplified - in practice would use proper quaternion math
                grasp_rot = cp.array([0.0, 0.0, 0.0, 1.0])

                grasp = {
                    'position': cp.asnumpy(grasp_pos),
                    'orientation': cp.asnumpy(grasp_rot),
                    'type': 'side',
                    'quality': 0.0
                }

                grasp_poses.append(grasp)

            return grasp_poses

        except Exception as e:
            self.get_logger().error(f'GPU side grasp generation error: {e}')
            return []

    def _evaluate_grasp_quality_gpu(self, grasp_candidates, object_pose):
        """Evaluate grasp quality using GPU acceleration"""
        try:
            evaluated_grasps = []

            for grasp in grasp_candidates:
                # Convert to GPU arrays for processing
                grasp_pos_gpu = cp.array(grasp['position'])
                obj_pos_gpu = cp.array([
                    object_pose.position.x,
                    object_pose.position.y,
                    object_pose.position.z
                ])

                # Calculate quality metrics using GPU operations
                quality = self._calculate_grasp_quality_gpu(
                    grasp_pos_gpu, grasp['orientation'], obj_pos_gpu, object_pose
                )

                grasp['quality'] = float(quality)
                evaluated_grasps.append(grasp)

            return evaluated_grasps

        except Exception as e:
            self.get_logger().error(f'GPU grasp quality evaluation error: {e}')
            return grasp_candidates  # Return without quality evaluation

    def _calculate_grasp_quality_gpu(self, grasp_pos, grasp_orient, obj_pos, obj_pose):
        """Calculate grasp quality using GPU operations"""
        try:
            # Distance to object (closer is better)
            distance = cp.linalg.norm(grasp_pos - obj_pos)
            distance_quality = cp.exp(-distance * 5)  # Exponential decay

            # Approach angle quality (for top grasps)
            if grasp['type'] == 'top':
                # Check if grasp is roughly above object
                vertical_alignment = cp.abs(grasp_pos[2] - obj_pos[2])
                vertical_quality = cp.exp(-vertical_alignment * 10)
            else:
                vertical_quality = 1.0

            # Orientation quality (how well gripper aligns with object)
            # This would involve more complex geometric calculations
            orientation_quality = 0.8  # Placeholder

            # Size compatibility (object size vs gripper width)
            size_quality = 0.9  # Placeholder

            # Combine quality metrics
            total_quality = (0.3 * distance_quality +
                           0.2 * vertical_quality +
                           0.3 * orientation_quality +
                           0.2 * size_quality)

            return cp.minimum(total_quality, 1.0)  # Clamp to [0, 1]

        except Exception as e:
            self.get_logger().error(f'GPU grasp quality calculation error: {e}')
            return 0.5  # Default quality

def main(args=None):
    rclpy.init(args=args)
    node = IsaacGraspPlanningNode()

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

## Isaac Inverse Kinematics Planning

### GPU-Accelerated IK Solutions

```python
# gpu_inverse_kinematics_planning.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
import numpy as np
import cupy as cp
from scipy.optimize import minimize
import threading

class IsaacGPUInverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('isaac_gpu_inverse_kinematics')

        # Initialize GPU resources
        self._initialize_gpu_resources()

        # Publishers and subscribers
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            '/ik_target_pose',
            self.target_pose_callback,
            10
        )

        self.joint_command_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.solution_status_pub = self.create_publisher(Bool, '/ik_solution_found', 10)

        # Robot configuration
        self.robot_config = {
            'joint_limits': {
                'min': [-2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0],
                'max': [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
            },
            'link_lengths': [0.1, 0.3, 0.3, 0.1, 0.1, 0.1, 0.05],  # Example lengths
            'dof': 7,
            'base_position': [0.0, 0.0, 1.0]  # Base height for humanoid
        }

        # IK parameters
        self.position_tolerance = 0.01  # 1cm
        self.orientation_tolerance = 0.01  # 0.01 rad
        self.max_iterations = 1000
        self.convergence_threshold = 1e-6

        # GPU-optimized IK components
        self.jacobian_cache = {}
        self.forward_kinematics_kernel = None
        self.inverse_kinematics_kernel = None

        self.get_logger().info('Isaac GPU Inverse Kinematics node initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for IK computation"""
        try:
            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for IK computation')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU IK initialization failed: {e}, using CPU fallback')

    def target_pose_callback(self, msg):
        """Solve IK for target pose"""
        try:
            # Get current joint state for initial guess
            current_joints = self._get_current_joints()

            if not current_joints:
                self.get_logger().warn('No current joint state available')
                return

            # Solve IK using GPU acceleration
            if self.gpu_available:
                solution = self._solve_ik_gpu(msg.pose, current_joints)
            else:
                solution = self._solve_ik_cpu(msg.pose, current_joints)

            if solution is not None:
                # Publish joint commands
                joint_cmd = JointState()
                joint_cmd.header = msg.header
                joint_cmd.name = self.robot_config['joint_names']
                joint_cmd.position = solution

                self.joint_command_pub.publish(joint_cmd)

                # Publish success status
                status_msg = Bool()
                status_msg.data = True
                self.solution_status_pub.publish(status_msg)

                self.get_logger().info(f'IK solution found: {solution}')

            else:
                self.get_logger().error('IK solution not found')
                status_msg = Bool()
                status_msg.data = False
                self.solution_status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'IK solving error: {e}')

    def _solve_ik_gpu(self, target_pose, current_joints):
        """Solve inverse kinematics using GPU acceleration"""
        try:
            # Convert to GPU arrays
            target_pos_gpu = cp.array([
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z
            ])

            target_quat_gpu = cp.array([
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w
            ])

            current_joints_gpu = cp.array(current_joints)

            # Use GPU-parallelized IK solving
            # This would implement GPU-parallelized optimization algorithms
            solution = self._gpu_ik_optimization(target_pos_gpu, target_quat_gpu, current_joints_gpu)

            if solution is not None:
                return cp.asnumpy(solution)
            else:
                return None

        except Exception as e:
            self.get_logger().error(f'GPU IK solving error: {e}')
            return self._solve_ik_cpu(target_pose, current_joints)

    def _gpu_ik_optimization(self, target_pos, target_quat, initial_joints):
        """GPU-parallelized IK optimization"""
        try:
            # Define objective function for GPU optimization
            def objective_function_gpu(joint_angles):
                # Calculate forward kinematics on GPU
                fk_pos, fk_quat = self._gpu_forward_kinematics(joint_angles)

                # Calculate position error
                pos_error = cp.linalg.norm(target_pos - fk_pos)

                # Calculate orientation error
                quat_diff = self._gpu_quaternion_difference(target_quat, fk_quat)
                orient_error = 2 * cp.arccos(cp.abs(quat_diff[3]))

                # Combined error
                total_error = pos_error + orient_error

                return total_error

            # Use GPU-optimized optimization algorithm
            # In practice, this would use custom CUDA kernels for IK
            # For this example, we'll use a hybrid approach

            # Sample multiple initial guesses in parallel on GPU
            num_samples = 10
            joint_ranges = cp.array(self.robot_config['joint_limits']['max']) - cp.array(self.robot_config['joint_limits']['min'])
            joint_mins = cp.array(self.robot_config['joint_limits']['min'])

            # Generate random initial guesses
            random_offsets = cp.random.uniform(-0.1, 0.1, (num_samples, len(initial_joints)))
            sample_joints = cp.tile(initial_joints, (num_samples, 1)) + random_offsets

            # Clamp to joint limits
            sample_joints = cp.clip(
                sample_joints,
                cp.array(self.robot_config['joint_limits']['min']),
                cp.array(self.robot_config['joint_limits']['max'])
            )

            # Evaluate each sample
            errors = cp.zeros(num_samples)
            for i in range(num_samples):
                errors[i] = objective_function_gpu(sample_joints[i])

            # Select best initial guess
            best_idx = cp.argmin(errors)
            best_initial = sample_joints[best_idx]

            # For final optimization, use CPU with GPU-accelerated objective function
            # (GPU optimization libraries for general optimization are limited)
            def cpu_objective(joint_angles):
                gpu_angles = cp.array(joint_angles)
                return float(objective_function_gpu(gpu_angles).get())

            from scipy.optimize import minimize

            result = minimize(
                cpu_objective,
                cp.asnumpy(best_initial),
                method='L-BFGS-B',
                bounds=list(zip(
                    self.robot_config['joint_limits']['min'],
                    self.robot_config['joint_limits']['max']
                )),
                options={'maxiter': self.max_iterations, 'ftol': self.convergence_threshold}
            )

            if result.success and result.fun < (self.position_tolerance + self.orientation_tolerance):
                return cp.array(result.x)
            else:
                return None

        except Exception as e:
            self.get_logger().error(f'GPU IK optimization error: {e}')
            return None

    def _gpu_forward_kinematics(self, joint_angles):
        """GPU-accelerated forward kinematics"""
        try:
            # Simplified forward kinematics for 7-DOF arm
            # In practice, this would implement the full kinematic chain
            # using GPU-parallelized matrix operations

            # For demonstration, use a simplified model
            # This would be replaced with actual DH parameters or other kinematic model
            base_pos = cp.array(self.robot_config['base_position'])

            # Calculate end-effector position based on joint angles
            # This is a placeholder - real implementation would use proper FK
            ee_pos = base_pos + cp.array([0.5, 0.0, 0.0])  # Placeholder
            ee_quat = cp.array([0.0, 0.0, 0.0, 1.0])  # Identity quaternion

            return ee_pos, ee_quat

        except Exception as e:
            self.get_logger().error(f'GPU forward kinematics error: {e}')
            return cp.zeros(3), cp.array([0.0, 0.0, 0.0, 1.0])

    def _gpu_quaternion_difference(self, q1, q2):
        """Calculate quaternion difference using GPU"""
        try:
            # Calculate relative rotation quaternion
            q2_conj = cp.array([q2[0], q2[1], q2[2], -q2[3]])  # Conjugate
            q_diff = self._gpu_quaternion_multiply(q1, q2_conj)
            return q_diff / cp.linalg.norm(q_diff)  # Normalize

        except Exception as e:
            self.get_logger().error(f'GPU quaternion difference error: {e}')
            return cp.array([0.0, 0.0, 0.0, 1.0])

    def _gpu_quaternion_multiply(self, q1, q2):
        """Multiply quaternions using GPU"""
        try:
            w = q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2]
            x = q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1]
            y = q1[3]*q2[1] - q1[0]*q2[2] + q1[1]*q2[3] + q1[2]*q2[0]
            z = q1[3]*q2[2] + q1[0]*q2[1] - q1[1]*q2[0] + q1[2]*q2[3]

            return cp.array([x, y, z, w])

        except Exception as e:
            self.get_logger().error(f'GPU quaternion multiplication error: {e}')
            return cp.array([0.0, 0.0, 0.0, 1.0])

    def _solve_ik_cpu(self, target_pose, current_joints):
        """CPU fallback for IK solving"""
        # Implement CPU-based IK solving using standard libraries
        # like KDL, PyKDL, or other inverse kinematics solvers
        pass

    def _get_current_joints(self):
        """Get current joint state (would subscribe to joint states in real implementation)"""
        # Placeholder - in real system, subscribe to /joint_states
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def main(args=None):
    rclpy.init(args=args)
    node = IsaacGPUInverseKinematicsNode()

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

## Isaac Trajectory Optimization

### GPU-Parallelized Trajectory Planning

```python
# gpu_trajectory_optimization.py
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import numpy as np
import cupy as cp
from scipy.interpolate import CubicSpline
import threading

class IsaacTrajectoryOptimizationNode(Node):
    def __init__(self):
        super().__init__('isaac_trajectory_optimization')

        # Initialize GPU resources
        self._initialize_gpu_resources()

        # Publishers and subscribers
        self.joint_trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/desired_trajectory',
            self.trajectory_callback,
            10
        )

        self.optimized_trajectory_pub = self.create_publisher(JointTrajectory, '/optimized_trajectory', 10)
        self.execution_command_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        # Trajectory optimization parameters
        self.smoothing_factor = 0.1
        self.velocity_limit = 2.0  # rad/s
        self.acceleration_limit = 5.0  # rad/s^2
        self.jerk_limit = 10.0  # rad/s^3
        self.collision_check_resolution = 0.05  # 5cm resolution for collision checking

        # Optimization algorithms
        self.optimization_method = 'cubic_spline'  # Options: 'cubic_spline', 'quintic_polynomial', 'minimum_jerk'

        self.get_logger().info('Isaac Trajectory Optimization node initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for trajectory optimization"""
        try:
            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for trajectory optimization')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU trajectory optimization initialization failed: {e}')

    def trajectory_callback(self, msg):
        """Process and optimize incoming trajectory"""
        try:
            if self.gpu_available:
                optimized_trajectory = self._optimize_trajectory_gpu(msg)
            else:
                optimized_trajectory = self._optimize_trajectory_cpu(msg)

            if optimized_trajectory:
                # Publish optimized trajectory
                self.optimized_trajectory_pub.publish(optimized_trajectory)

                # Optionally publish for immediate execution
                self.execution_command_pub.publish(optimized_trajectory)

                self.get_logger().info(f'Trajectory optimized: {len(optimized_trajectory.points)} points')

        except Exception as e:
            self.get_logger().error(f'Trajectory optimization error: {e}')

    def _optimize_trajectory_gpu(self, trajectory_msg):
        """Optimize trajectory using GPU acceleration"""
        try:
            # Extract trajectory data
            positions = []
            velocities = []
            accelerations = []
            time_stamps = []

            for point in trajectory_msg.points:
                positions.append(point.positions)
                velocities.append(point.velocities)
                accelerations.append(point.accelerations)
                time_stamps.append(
                    point.time_from_start.sec + point.time_from_start.nanosec / 1e9
                )

            if len(positions) < 2:
                return trajectory_msg  # Cannot optimize single point

            # Convert to GPU arrays
            pos_gpu = cp.array(positions)
            vel_gpu = cp.array(velocities) if velocities else cp.zeros_like(pos_gpu)
            acc_gpu = cp.array(accelerations) if accelerations else cp.zeros_like(pos_gpu)
            time_gpu = cp.array(time_stamps)

            # Apply GPU-parallelized trajectory optimization
            if self.optimization_method == 'cubic_spline':
                optimized_pos, optimized_vel, optimized_acc = self._gpu_cubic_spline_optimization(
                    pos_gpu, vel_gpu, acc_gpu, time_gpu
                )
            elif self.optimization_method == 'minimum_jerk':
                optimized_pos, optimized_vel, optimized_acc = self._gpu_minimum_jerk_optimization(
                    pos_gpu, vel_gpu, acc_gpu, time_gpu
                )
            else:
                optimized_pos, optimized_vel, optimized_acc = pos_gpu, vel_gpu, acc_gpu

            # Apply constraints and limits
            constrained_pos, constrained_vel, constrained_acc = self._apply_gpu_constraints(
                optimized_pos, optimized_vel, optimized_acc
            )

            # Create optimized trajectory message
            optimized_msg = self._create_trajectory_message(
                cp.asnumpy(constrained_pos),
                cp.asnumpy(constrained_vel),
                cp.asnumpy(constrained_acc),
                time_stamps,
                trajectory_msg.joint_names
            )

            return optimized_msg

        except Exception as e:
            self.get_logger().error(f'GPU trajectory optimization error: {e}')
            return self._optimize_trajectory_cpu(trajectory_msg)

    def _gpu_cubic_spline_optimization(self, positions, velocities, accelerations, times):
        """GPU-parallelized cubic spline trajectory optimization"""
        try:
            # For each joint dimension, create a cubic spline
            num_joints = positions.shape[1]
            optimized_positions = cp.zeros_like(positions)
            optimized_velocities = cp.zeros_like(velocities)
            optimized_accelerations = cp.zeros_like(accelerations)

            # Apply spline optimization in parallel for each joint
            for joint_idx in range(num_joints):
                joint_positions = positions[:, joint_idx]

                # Create spline using GPU-accelerated interpolation
                # In practice, this would use custom CUDA kernels
                # For now, we'll use scipy on CPU but with GPU-optimized data handling

                # Calculate derivatives for smooth trajectory
                if len(times) >= 2:
                    # Compute velocity and acceleration profiles
                    joint_velocities = cp.gradient(joint_positions, times)
                    joint_accelerations = cp.gradient(joint_velocities, times)

                    optimized_positions[:, joint_idx] = joint_positions
                    optimized_velocities[:, joint_idx] = joint_velocities
                    optimized_accelerations[:, joint_idx] = joint_accelerations

            return optimized_positions, optimized_velocities, optimized_accelerations

        except Exception as e:
            self.get_logger().error(f'GPU cubic spline optimization error: {e}')
            return positions, velocities, accelerations

    def _gpu_minimum_jerk_optimization(self, positions, velocities, accelerations, times):
        """GPU-parallelized minimum jerk trajectory optimization"""
        try:
            # Minimum jerk trajectories minimize jerk (third derivative of position)
            # This creates smooth, human-like movements
            num_points, num_joints = positions.shape
            optimized_positions = cp.zeros_like(positions)
            optimized_velocities = cp.zeros_like(velocities)
            optimized_accelerations = cp.zeros_like(accelerations)

            # For each joint, optimize for minimum jerk
            for joint_idx in range(num_joints):
                joint_positions = positions[:, joint_idx]

                # Apply minimum jerk optimization
                # This would use GPU-parallelized optimization
                # For this example, apply smoothing to reduce jerk

                # Simple smoothing to reduce jerk
                smoothed_positions = cp.zeros_like(joint_positions)

                # Apply smoothing kernel in parallel
                for i in range(len(joint_positions)):
                    start_idx = max(0, i - 2)
                    end_idx = min(len(joint_positions), i + 3)
                    smoothed_positions[i] = cp.mean(joint_positions[start_idx:end_idx])

                optimized_positions[:, joint_idx] = smoothed_positions

            # Calculate derivatives for velocities and accelerations
            for joint_idx in range(num_joints):
                if len(times) > 1:
                    optimized_velocities[:, joint_idx] = cp.gradient(optimized_positions[:, joint_idx], times)
                    optimized_accelerations[:, joint_idx] = cp.gradient(optimized_velocities[:, joint_idx], times)

            return optimized_positions, optimized_velocities, optimized_accelerations

        except Exception as e:
            self.get_logger().error(f'GPU minimum jerk optimization error: {e}')
            return positions, velocities, accelerations

    def _apply_gpu_constraints(self, positions, velocities, accelerations):
        """Apply velocity, acceleration, and joint limit constraints using GPU"""
        try:
            # Apply velocity limits
            velocities = cp.clip(velocities, -self.velocity_limit, self.velocity_limit)

            # Apply acceleration limits
            accelerations = cp.clip(accelerations, -self.acceleration_limit, self.acceleration_limit)

            # Apply joint position limits
            joint_limits_min = cp.array(self.robot_config['joint_limits']['min'])
            joint_limits_max = cp.array(self.robot_config['joint_limits']['max'])

            # Expand limits to match trajectory shape
            limits_min_expanded = cp.tile(joint_limits_min, (len(positions), 1))
            limits_max_expanded = cp.tile(joint_limits_max, (len(positions), 1))

            positions = cp.clip(positions, limits_min_expanded, limits_max_expanded)

            # Apply jerk limits by smoothing high jerk regions
            if len(positions) > 2:
                # Calculate jerk (derivative of acceleration)
                jerks = cp.gradient(accelerations, axis=0)

                # Identify high jerk regions and smooth them
                high_jerk_mask = cp.abs(jerks) > self.jerk_limit
                smoothed_positions = positions.copy()

                for joint_idx in range(positions.shape[1]):
                    for point_idx in range(1, len(positions) - 1):
                        if cp.any(high_jerk_mask[point_idx, joint_idx]):
                            # Smooth the trajectory around high jerk point
                            smoothed_positions[point_idx, joint_idx] = (
                                positions[point_idx-1, joint_idx] +
                                positions[point_idx+1, joint_idx]
                            ) / 2.0

                positions = smoothed_positions

            return positions, velocities, accelerations

        except Exception as e:
            self.get_logger().error(f'GPU constraint application error: {e}')
            return positions, velocities, accelerations

    def _create_trajectory_message(self, positions, velocities, accelerations, times, joint_names):
        """Create JointTrajectory message from optimized data"""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

        for i in range(len(positions)):
            point = JointTrajectoryPoint()

            point.positions = positions[i].tolist()

            if i < len(velocities):
                point.velocities = velocities[i].tolist()

            if i < len(accelerations):
                point.accelerations = accelerations[i].tolist()

            # Calculate time from start
            if i == 0:
                time_from_start = Duration()
                time_from_start.sec = 0
                time_from_start.nanosec = 0
            else:
                dt = times[i] - times[i-1]
                time_from_start = Duration()
                time_from_start.sec = int(dt)
                time_from_start.nanosec = int((dt % 1) * 1e9)

            point.time_from_start = time_from_start
            traj_msg.points.append(point)

        return traj_msg

    def _optimize_trajectory_cpu(self, trajectory_msg):
        """CPU fallback for trajectory optimization"""
        # Implement CPU-based trajectory optimization
        # using scipy, numpy, or other CPU-based libraries
        pass

def main(args=None):
    rclpy.init(args=args)
    node = IsaacTrajectoryOptimizationNode()

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

## Isaac Collision Avoidance

### GPU-Accelerated Collision Checking

```python
# gpu_collision_avoidance.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import PoseArray, Twist
from std_msgs.msg import Bool, Float32
from builtin_interfaces.msg import Time
import numpy as np
import cupy as cp
from scipy.spatial.distance import cdist
import threading

class IsaacCollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__('isaac_collision_avoidance')

        # Initialize GPU resources
        self._initialize_gpu_resources()

        # Publishers and subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/point_cloud',
            self.pointcloud_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/desired_trajectory',
            self.trajectory_callback,
            10
        )

        self.avoidance_cmd_pub = self.create_publisher(Twist, '/collision_avoidance_cmd', 10)
        self.collision_status_pub = self.create_publisher(Bool, '/collision_status', 10)
        self.safety_distance_pub = self.create_publisher(Float32, '/safety_distance', 10)

        # Collision detection parameters
        self.collision_distance = 0.3  # 30cm collision threshold
        self.safety_distance = 0.5    # 50cm safety distance
        self.robot_radius = 0.3      # 30cm robot radius
        self.check_resolution = 0.1  # 10cm resolution for collision checking

        # Environment representation
        self.obstacle_points = cp.array([]).reshape(0, 3)
        self.robot_trajectory = None

        # Collision checking timer
        self.collision_timer = self.create_timer(0.05, self.collision_checking_loop)  # 20Hz

        self.get_logger().info('Isaac Collision Avoidance node initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for collision detection"""
        try:
            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for collision avoidance')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU collision avoidance initialization failed: {e}')

    def pointcloud_callback(self, msg):
        """Process point cloud for obstacle detection"""
        try:
            # Convert point cloud to numpy array (simplified)
            points = self._pointcloud_to_array(msg)

            if self.gpu_available:
                self._update_obstacle_points_gpu(points)
            else:
                self._update_obstacle_points_cpu(points)

        except Exception as e:
            self.get_logger().error(f'Point cloud processing error: {e}')

    def scan_callback(self, msg):
        """Process laser scan for collision avoidance"""
        try:
            # Convert scan ranges to obstacle points
            obstacle_points = []
            angle = msg.angle_min

            for range_val in msg.ranges:
                if msg.range_min <= range_val <= msg.range_max:
                    x = range_val * np.cos(angle)
                    y = range_val * np.sin(angle)
                    obstacle_points.append([x, y, 0.0])  # Assume z=0 for 2D obstacles

                angle += msg.angle_increment

            if obstacle_points:
                if self.gpu_available:
                    scan_points = cp.array(obstacle_points, dtype=cp.float32)
                    self._update_obstacle_points_gpu(scan_points)
                else:
                    self._update_obstacle_points_cpu(np.array(obstacle_points))

        except Exception as e:
            self.get_logger().error(f'Laser scan processing error: {e}')

    def trajectory_callback(self, msg):
        """Process trajectory for collision checking"""
        self.robot_trajectory = msg

    def _update_obstacle_points_gpu(self, new_points):
        """Update obstacle points using GPU acceleration"""
        try:
            if len(new_points) > 0:
                new_points_gpu = cp.asarray(new_points, dtype=cp.float32)

                # Filter points within relevant range
                distances = cp.linalg.norm(new_points_gpu[:, :2], axis=1)  # Only x,y for distance
                relevant_mask = distances < 5.0  # Only within 5m

                filtered_points = new_points_gpu[relevant_mask]

                # Update obstacle points
                if len(self.obstacle_points) == 0:
                    self.obstacle_points = filtered_points
                else:
                    # Combine with existing points
                    self.obstacle_points = cp.vstack([self.obstacle_points, filtered_points])

                # Limit number of points for performance
                if len(self.obstacle_points) > 10000:
                    # Keep most recent points
                    self.obstacle_points = self.obstacle_points[-5000:]

        except Exception as e:
            self.get_logger().error(f'GPU obstacle point update error: {e}')

    def collision_checking_loop(self):
        """Main collision checking loop"""
        if not self.robot_trajectory or len(self.obstacle_points) == 0:
            return

        try:
            # Check collision along robot trajectory
            if self.gpu_available:
                collision_info = self._check_trajectory_collision_gpu(self.robot_trajectory)
            else:
                collision_info = self._check_trajectory_collision_cpu(self.robot_trajectory)

            # Publish collision status
            collision_msg = Bool()
            collision_msg.data = collision_info['collision_detected']
            self.collision_status_pub.publish(collision_msg)

            # Publish safety distance
            safety_msg = Float32()
            safety_msg.data = float(collision_info['min_safety_distance'])
            self.safety_distance_pub.publish(safety_msg)

            if collision_info['collision_detected']:
                # Generate avoidance command
                avoidance_cmd = self._generate_avoidance_command(collision_info)
                self.avoidance_cmd_pub.publish(avoidance_cmd)

                self.get_logger().warn(
                    f'Collision detected! Min distance: {collision_info["min_distance"]:.3f}m, '
                    f'Closest obstacle: ({collision_info["closest_point"][0]:.3f}, {collision_info["closest_point"][1]:.3f})'
                )

        except Exception as e:
            self.get_logger().error(f'Collision checking error: {e}')

    def _check_trajectory_collision_gpu(self, trajectory_msg):
        """Check trajectory for collisions using GPU acceleration"""
        try:
            # Extract trajectory points
            trajectory_points = []
            for point in trajectory_msg.points:
                if point.positions:
                    # Calculate forward kinematics to get end-effector positions
                    # This would be more complex in practice
                    pos = self._calculate_ee_position(point.positions)  # Simplified
                    trajectory_points.append(pos)

            if not trajectory_points:
                return {'collision_detected': False, 'min_distance': float('inf'), 'closest_point': [0, 0, 0]}

            # Upload trajectory to GPU
            traj_gpu = cp.array(trajectory_points, dtype=cp.float32)
            obstacles_gpu = self.obstacle_points

            # Calculate distances between trajectory and obstacles using GPU
            if len(obstacles_gpu) > 0:
                # Use GPU-parallelized distance calculation
                min_distances = []
                closest_points = []

                for traj_point in traj_gpu:
                    # Calculate distances to all obstacles in parallel
                    distances = cp.linalg.norm(obstacles_gpu - traj_point, axis=1)
                    min_dist = cp.min(distances)
                    min_idx = cp.argmin(distances)

                    min_distances.append(min_dist)
                    closest_points.append(obstacles_gpu[min_idx])

                # Find minimum distance across entire trajectory
                min_trajectory_distance = cp.min(cp.array(min_distances))
                closest_obstacle_idx = cp.argmin(cp.array(min_distances))

                collision_detected = min_trajectory_distance < self.collision_distance

                result = {
                    'collision_detected': bool(collision_detected.get()),
                    'min_distance': float(min_trajectory_distance.get()),
                    'closest_point': cp.asnumpy(obstacles_gpu[closest_obstacle_idx]).tolist(),
                    'min_safety_distance': float(cp.maximum(0.0, min_trajectory_distance - self.robot_radius).get())
                }

                return result
            else:
                return {
                    'collision_detected': False,
                    'min_distance': float('inf'),
                    'closest_point': [0, 0, 0],
                    'min_safety_distance': 5.0  # Default safe distance
                }

        except Exception as e:
            self.get_logger().error(f'GPU collision checking error: {e}')
            return {'collision_detected': False, 'min_distance': float('inf'), 'closest_point': [0, 0, 0], 'min_safety_distance': 5.0}

    def _calculate_ee_position(self, joint_positions):
        """Calculate end-effector position from joint positions (simplified)"""
        # This would implement forward kinematics
        # For this example, return a simple approximation
        return [sum(joint_positions[:3])/len(joint_positions[:3]), 0, 0.5]  # Placeholder

    def _generate_avoidance_command(self, collision_info):
        """Generate collision avoidance command"""
        cmd = Twist()

        # Calculate avoidance direction based on closest obstacle
        closest_obstacle = collision_info['closest_point']

        # Simple avoidance: move away from closest obstacle
        robot_pos = [0, 0, 0]  # Would come from robot state

        avoidance_vector = np.array(robot_pos) - np.array(closest_obstacle)
        avoidance_direction = avoidance_vector / (np.linalg.norm(avoidance_vector) + 1e-8)

        # Set avoidance velocity
        avoidance_speed = min(0.5, 1.0 / (collision_info['min_distance'] + 0.1))  # Faster when closer
        cmd.linear.x = avoidance_direction[0] * avoidance_speed
        cmd.linear.y = avoidance_direction[1] * avoidance_speed
        cmd.linear.z = 0.0
        cmd.angular.z = np.arctan2(avoidance_direction[1], avoidance_direction[0]) * 0.5  # Turn toward safety

        return cmd

def main(args=None):
    rclpy.init(args=args)
    node = IsaacCollisionAvoidanceNode()

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

## Isaac Manipulation Planning Integration

### Complete Manipulation Planning Pipeline

```python
# complete_manipulation_planning.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from trajectory_msgs.msg import JointTrajectory
from builtin_interfaces.msg import Duration
import numpy as np

class IsaacCompleteManipulationPlanningNode(Node):
    def __init__(self):
        super().__init__('isaac_complete_manipulation_planning')

        # Initialize all manipulation planning components
        self.grasp_planner = IsaacGraspPlanningNode()
        self.ik_solver = IsaacGPUInverseKinematicsNode()
        self.trajectory_optimizer = IsaacTrajectoryOptimizationNode()
        self.collision_avoider = IsaacCollisionAvoidanceNode()

        # Publishers for integrated system
        self.combined_plan_pub = self.create_publisher(JointTrajectory, '/complete_manipulation_plan', 10)
        self.manipulation_status_pub = self.create_publisher(String, '/manipulation_status', 10)

        # Manipulation state
        self.current_object_pose = None
        self.current_robot_state = None
        self.manipulation_active = False
        self.current_task = None

        # Timer for system integration
        self.integration_timer = self.create_timer(0.1, self.integration_loop)

        self.get_logger().info('Complete Isaac Manipulation Planning system initialized')

    def integration_loop(self):
        """Integrate all manipulation planning components"""
        if not self.manipulation_active or not self.current_object_pose:
            return

        try:
            # 1. Generate grasp poses
            grasp_poses = self.grasp_planner.generate_grasps(self.current_object_pose)
            if not grasp_poses:
                self.get_logger().warn('No grasp poses generated')
                return

            # 2. Solve IK for best grasp pose
            best_grasp = self._select_best_grasp(grasp_poses)
            ik_solution = self.ik_solver.solve_ik(best_grasp.pose, self.current_robot_state)
            if not ik_solution:
                self.get_logger().warn('No IK solution found for grasp pose')
                return

            # 3. Plan approach trajectory
            approach_trajectory = self._plan_approach_trajectory(ik_solution)
            if not approach_trajectory:
                self.get_logger().warn('Could not plan approach trajectory')
                return

            # 4. Optimize trajectory
            optimized_trajectory = self.trajectory_optimizer.optimize_trajectory(approach_trajectory)
            if not optimized_trajectory:
                self.get_logger().warn('Trajectory optimization failed')
                return

            # 5. Check for collisions
            collision_free = self.collision_avoider.check_trajectory_collision(optimized_trajectory)
            if not collision_free:
                self.get_logger().warn('Collision detected in planned trajectory')
                # Try to replan with collision avoidance
                optimized_trajectory = self._replan_with_collision_avoidance(optimized_trajectory)

            # 6. Publish complete manipulation plan
            self.combined_plan_pub.publish(optimized_trajectory)

            # 7. Publish status
            status_msg = String()
            status_msg.data = f'Manipulation plan generated with {len(optimized_trajectory.points)} waypoints'
            self.manipulation_status_pub.publish(status_msg)

            self.get_logger().info(f'Manipulation plan published: {len(optimized_trajectory.points)} waypoints')

        except Exception as e:
            self.get_logger().error(f'Complete manipulation planning error: {e}')

    def _select_best_grasp(self, grasp_poses):
        """Select best grasp from candidates based on quality metrics"""
        if not grasp_poses:
            return None

        # Sort grasps by quality score (assuming they have quality information)
        sorted_grasps = sorted(grasp_poses, key=lambda x: x.quality, reverse=True)
        return sorted_grasps[0]  # Return highest quality grasp

    def _plan_approach_trajectory(self, grasp_solution):
        """Plan approach trajectory to grasp pose"""
        try:
            # Create trajectory with multiple waypoints:
            # 1. Pre-grasp pose (above object)
            # 2. Approach pose (at grasp distance)
            # 3. Grasp pose (actual grasp)
            # 4. Lift pose (lift object slightly)

            trajectory = JointTrajectory()
            trajectory.joint_names = self.current_robot_state.name

            # Define trajectory points
            waypoints = [
                self._create_pre_grasp_pose(grasp_solution),  # Pre-grasp (above object)
                self._create_approach_pose(grasp_solution),  # Approach (just before grasp)
                grasp_solution,                              # Actual grasp
                self._create_lift_pose(grasp_solution)       # Lift after grasp
            ]

            time_step = 2.0  # 2 seconds per segment

            for i, waypoint in enumerate(waypoints):
                point = JointTrajectoryPoint()
                point.positions = waypoint  # This would be joint positions from IK
                point.velocities = [0.0] * len(waypoint)  # Start/stop with zero velocity
                point.accelerations = [0.0] * len(waypoint)  # Start/stop with zero acceleration

                # Set timing
                total_time = (i + 1) * time_step
                point.time_from_start.sec = int(total_time)
                point.time_from_start.nanosec = int((total_time % 1) * 1e9)

                trajectory.points.append(point)

            return trajectory

        except Exception as e:
            self.get_logger().error(f'Approach trajectory planning error: {e}')
            return None

    def _create_pre_grasp_pose(self, grasp_solution):
        """Create pre-grasp pose (slightly above grasp point)"""
        # This would modify the grasp pose to be above the object
        # For now, return the grasp solution
        return grasp_solution

    def _create_approach_pose(self, grasp_solution):
        """Create approach pose (slightly before grasp)"""
        # This would modify the grasp pose to be a bit further
        # For now, return the grasp solution
        return grasp_solution

    def _create_lift_pose(self, grasp_solution):
        """Create lift pose (raise object after grasp)"""
        # This would modify the grasp pose to lift the object
        # For now, return the grasp solution
        return grasp_solution

    def _replan_with_collision_avoidance(self, original_trajectory):
        """Replan trajectory considering collision avoidance"""
        # This would implement collision-aware trajectory replanning
        # Could use RRT*, PRM, or other collision-aware planning algorithms
        return original_trajectory  # Placeholder

def main(args=None):
    rclpy.init(args=args)
    node = IsaacCompleteManipulationPlanningNode()

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

### GPU Resource Management for Manipulation

```python
# manipulation_performance.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import time
from collections import deque
import GPUtil

class IsaacManipulationPerformanceNode(Node):
    def __init__(self):
        super().__init__('isaac_manipulation_performance')

        # Performance monitoring
        self.grasp_times = deque(maxlen=100)
        self.ik_times = deque(maxlen=100)
        self.trajectory_times = deque(maxlen=100)
        self.collision_times = deque(maxlen=100)

        # Publishers for performance metrics
        self.grasp_perf_pub = self.create_publisher(Float32, '/perf/grasp_generation', 10)
        self.ik_perf_pub = self.create_publisher(Float32, '/perf/ik_solution', 10)
        self.trajectory_perf_pub = self.create_publisher(Float32, '/perf/trajectory_optimization', 10)
        self.collision_perf_pub = self.create_publisher(Float32, '/perf/collision_checking', 10)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/manipulation_diagnostics', 10)

        # Performance optimization parameters
        self.target_rate = 30.0  # Target 30Hz for manipulation
        self.adaptive_complexity = True
        self.current_model_complexity = 'high'  # Options: 'low', 'medium', 'high'

        # Timer for performance monitoring
        self.performance_timer = self.create_timer(1.0, self.performance_monitoring_loop)

        self.get_logger().info('Isaac Manipulation Performance Monitor initialized')

    def performance_monitoring_loop(self):
        """Monitor and optimize manipulation performance"""
        try:
            # Calculate performance metrics
            avg_grasp_time = np.mean(self.grasp_times) if self.grasp_times else 0.0
            avg_ik_time = np.mean(self.ik_times) if self.ik_times else 0.0
            avg_trajectory_time = np.mean(self.trajectory_times) if self.trajectory_times else 0.0
            avg_collision_time = np.mean(self.collision_times) if self.collision_times else 0.0

            # Publish performance metrics
            metrics = {
                'grasp_generation': avg_grasp_time,
                'ik_solution': avg_ik_time,
                'trajectory_optimization': avg_trajectory_time,
                'collision_checking': avg_collision_time
            }

            for metric_name, value in metrics.items():
                msg = Float32()
                msg.data = float(value)

                if metric_name == 'grasp_generation':
                    self.grasp_perf_pub.publish(msg)
                elif metric_name == 'ik_solution':
                    self.ik_perf_pub.publish(msg)
                elif metric_name == 'trajectory_optimization':
                    self.trajectory_perf_pub.publish(msg)
                elif metric_name == 'collision_checking':
                    self.collision_perf_pub.publish(msg)

            # Calculate total manipulation cycle time
            total_time = avg_grasp_time + avg_ik_time + avg_trajectory_time + avg_collision_time
            current_rate = 1.0 / total_time if total_time > 0 else 0.0

            # Apply adaptive optimizations
            self._apply_adaptive_optimizations(current_rate, total_time)

            # Monitor GPU utilization
            self._monitor_gpu_utilization()

            # Log performance summary
            self.get_logger().info(
                f'Manipulation Performance - Grasp: {avg_grasp_time:.3f}s, '
                f'IK: {avg_ik_time:.3f}s, '
                f'Traj: {avg_trajectory_time:.3f}s, '
                f'Collision: {avg_collision_time:.3f}s, '
                f'Rate: {current_rate:.2f}Hz'
            )

        except Exception as e:
            self.get_logger().error(f'Performance monitoring error: {e}')

    def _apply_adaptive_optimizations(self, current_rate, total_time):
        """Apply adaptive optimizations based on performance"""
        target_rate = self.target_rate

        if current_rate < target_rate * 0.8:  # Performance is 20% below target
            if self.current_model_complexity == 'high':
                self.current_model_complexity = 'medium'
                self.get_logger().info('Reduced model complexity to medium for performance')
            elif self.current_model_complexity == 'medium':
                self.current_model_complexity = 'low'
                self.get_logger().info('Reduced model complexity to low for performance')

        elif current_rate > target_rate * 1.2:  # Performance is 20% above target
            if self.current_model_complexity == 'low':
                self.current_model_complexity = 'medium'
                self.get_logger().info('Increased model complexity to medium')
            elif self.current_model_complexity == 'medium':
                self.current_model_complexity = 'high'
                self.get_logger().info('Increased model complexity to high')

    def _monitor_gpu_utilization(self):
        """Monitor GPU utilization for manipulation components"""
        try:
            gpus = GPUtil.getGPUs()
            if gpus:
                gpu = gpus[0]  # Use first GPU
                gpu_load = gpu.load * 100
                gpu_memory = gpu.memoryUtil * 100

                self.get_logger().debug(f'GPU Utilization: Load={gpu_load:.1f}%, Memory={gpu_memory:.1f}%')

                # Log warnings for high utilization
                if gpu_load > 90:
                    self.get_logger().warn(f'High GPU load: {gpu_load:.1f}%')
                elif gpu_load > 75:
                    self.get_logger().info(f'Moderate GPU load: {gpu_load:.1f}%')

        except Exception as e:
            self.get_logger().debug(f'GPU monitoring error: {e}')

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

## Best Practices for Isaac Manipulation Planning

### Manipulation Planning Best Practices

1. **Multi-Grasp Strategy**: Generate multiple grasp candidates and select best one
2. **Collision Awareness**: Always check for collisions during planning
3. **Real-time Performance**: Optimize for real-time manipulation requirements
4. **Safety Margins**: Include safety margins in all planning calculations
5. **Recovery Behaviors**: Implement robust recovery from failed grasps
6. **Sensor Integration**: Combine multiple sensor inputs for robust planning
7. **Adaptive Planning**: Adjust planning parameters based on task requirements
8. **Quality Assessment**: Evaluate grasp quality before execution

### GPU Optimization Guidelines

1. **Memory Management**: Use GPU memory pools for efficient allocation
2. **Batch Processing**: Process multiple planning queries together
3. **Asynchronous Operations**: Overlap computation with data transfer
4. **Kernel Optimization**: Optimize CUDA kernels for specific tasks
5. **Data Formats**: Use appropriate data types (FP16 when possible)
6. **Resource Monitoring**: Continuously monitor GPU utilization
7. **Fallback Systems**: Implement CPU-based fallbacks for critical functions
8. **Load Balancing**: Distribute computation across available GPUs

## Troubleshooting Common Issues

### Manipulation Planning Issues and Solutions

**Issue**: IK solver fails to find solution
**Solution**: Check joint limits, verify target pose reachability, adjust tolerances

**Issue**: Trajectory optimization is slow
**Solution**: Reduce trajectory complexity, optimize GPU memory usage, use smaller models

**Issue**: Collision detection misses obstacles
**Solution**: Increase point cloud resolution, improve sensor coverage, adjust thresholds

**Issue**: Grasp planning generates poor grasps
**Solution**: Improve object detection, adjust grasp quality thresholds, use better models

**Issue**: GPU memory exhaustion
**Solution**: Reduce batch sizes, optimize memory allocation, use memory pools

## Next Steps

In the next chapter, we'll explore Isaac's simulation capabilities specifically for manipulation tasks, learning how to create realistic manipulation environments and validate manipulation behaviors in simulation before real-world deployment.