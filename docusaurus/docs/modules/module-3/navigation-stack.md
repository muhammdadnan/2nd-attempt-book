# Isaac ROS Navigation Stack

In this chapter, we'll explore the Isaac ROS Navigation stack, which provides GPU-accelerated navigation capabilities specifically designed for humanoid robots. Isaac ROS Navigation leverages NVIDIA's hardware acceleration to deliver high-performance path planning, obstacle avoidance, and navigation in complex environments.

## Isaac ROS Navigation Overview

### Architecture Components

Isaac ROS Navigation includes several key components:

- **Isaac ROS Visual SLAM**: GPU-accelerated simultaneous localization and mapping
- **Isaac ROS Path Planning**: GPU-accelerated path planning algorithms
- **Isaac ROS Costmap**: GPU-accelerated costmap generation and updates
- **Isaac ROS Controller**: GPU-accelerated trajectory control
- **Isaac ROS Obstacle Avoidance**: Real-time GPU-accelerated obstacle detection and avoidance

### Navigation System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Isaac ROS Navigation Stack                   │
├─────────────────────────────────────────────────────────────────┤
│  Sensor Input → Mapping → Path Planning → Control → Execution  │
│   (Cameras,     (GPU)       (GPU)       (GPU)    (Robot)     │
│    LiDAR, IMU)                                                   │
├─────────────────────────────────────────────────────────────────┤
│  GPU-Accelerated Components:                                   │
│  - CUDA-based costmap generation                               │
│  - TensorRT-accelerated obstacle detection                     │
│  - GPU-parallelized path planning algorithms                   │
│  - CUDA-optimized trajectory optimization                      │
│  - Real-time collision checking                                │
└─────────────────────────────────────────────────────────────────┘
```

## Isaac ROS Navigation Core

### GPU-Accelerated Navigation Core

```python
# navigation_core.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Bool
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
from builtin_interfaces.msg import Duration
import numpy as np
import cv2
from scipy.spatial import KDTree
import cupy as cp  # For GPU operations

class IsaacNavigationCore(Node):
    def __init__(self):
        super().__init__('isaac_navigation_core')

        # Initialize TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize navigation components
        self.costmap = None
        self.planner = None
        self.controller = None
        self.obstacle_detector = None

        # Initialize GPU resources
        self._initialize_gpu_resources()

        # Navigation state
        self.current_pose = None
        self.current_goal = None
        self.current_path = None
        self.is_active = False

        # Publishers
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.global_costmap_pub = self.create_publisher(OccupancyGrid, '/global_costmap', 10)
        self.local_costmap_pub = self.create_publisher(OccupancyGrid, '/local_costmap', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/navigation_viz', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
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

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/point_cloud',
            self.pointcloud_callback,
            10
        )

        # Services
        self.cancel_nav_service = self.create_service(Bool, '/cancel_navigation', self.cancel_navigation)

        # Timers
        self.navigation_timer = self.create_timer(0.1, self.navigation_loop)

        # Initialize navigation components
        self._initialize_navigation_components()

        self.get_logger().info('Isaac Navigation Core initialized')

    def _initialize_gpu_resources(self):
        """Initialize GPU resources for navigation"""
        try:
            # Initialize CuPy for GPU operations
            self.gpu_available = True
            self.get_logger().info('GPU resources initialized for navigation')
        except Exception as e:
            self.gpu_available = False
            self.get_logger().warn(f'GPU initialization failed: {e}, using CPU fallback')

    def _initialize_navigation_components(self):
        """Initialize navigation components"""
        try:
            # Initialize GPU-accelerated costmap
            self.costmap = IsaacCostmap(self.get_logger(), self.gpu_available)

            # Initialize GPU-accelerated path planner
            self.planner = IsaacPathPlanner(self.get_logger(), self.gpu_available)

            # Initialize trajectory controller
            self.controller = IsaacTrajectoryController(self.get_logger())

            # Initialize obstacle detector
            self.obstacle_detector = IsaacObstacleDetector(self.get_logger(), self.gpu_available)

            self.get_logger().info('Navigation components initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize navigation components: {e}')

    def odom_callback(self, msg):
        """Update current robot pose"""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Process laser scan data"""
        if self.costmap:
            self.costmap.update_from_scan(msg)

    def pointcloud_callback(self, msg):
        """Process point cloud data"""
        if self.costmap:
            self.costmap.update_from_pointcloud(msg)

    def goal_callback(self, msg):
        """Process navigation goal"""
        self.current_goal = msg.pose
        self.is_active = True

        if self.current_pose:
            self._plan_path()

    def _plan_path(self):
        """Plan path from current pose to goal"""
        if not self.current_pose or not self.current_goal:
            return

        try:
            # Convert poses to numpy arrays
            start = np.array([self.current_pose.position.x, self.current_pose.position.y])
            goal = np.array([self.current_goal.position.x, self.current_goal.position.y])

            # Plan path using GPU acceleration
            path = self.planner.plan_path(start, goal, self.costmap)

            if path is not None:
                self.current_path = path
                self._publish_path(path)
                self.get_logger().info(f'Path planned with {len(path)} waypoints')
            else:
                self.get_logger().warn('Failed to plan path')

        except Exception as e:
            self.get_logger().error(f'Path planning error: {e}')

    def navigation_loop(self):
        """Main navigation loop"""
        if not self.is_active or not self.current_path:
            return

        try:
            # Update costmap with latest sensor data
            if self.costmap:
                self.costmap.update()

            # Check for obstacles along path
            if self.obstacle_detector.has_obstacles_ahead():
                self.get_logger().info('Obstacle detected, replanning')
                self._plan_path()

            # Follow current path
            if self.current_path:
                cmd_vel = self.controller.follow_path(
                    self.current_pose,
                    self.current_path,
                    self.costmap
                )

                if cmd_vel:
                    self.cmd_vel_pub.publish(cmd_vel)

            # Check if goal reached
            if self._goal_reached():
                self._arrive_at_goal()

        except Exception as e:
            self.get_logger().error(f'Navigation loop error: {e}')

    def _goal_reached(self):
        """Check if goal has been reached"""
        if not self.current_pose or not self.current_goal:
            return False

        current_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        goal_pos = np.array([self.current_goal.position.x, self.current_goal.position.y])

        distance = np.linalg.norm(current_pos - goal_pos)
        return distance < 0.5  # 50cm tolerance

    def _arrive_at_goal(self):
        """Handle arrival at goal"""
        self.is_active = False
        self.current_path = None

        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        self.get_logger().info('Goal reached successfully')

    def _publish_path(self, path):
        """Publish planned path"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in path:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = float(point[0])
            pose_stamped.pose.position.y = float(point[1])
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0

            path_msg.poses.append(pose_stamped)

        self.path_pub.publish(path_msg)

    def cancel_navigation(self, request, response):
        """Cancel current navigation"""
        self.is_active = False
        self.current_path = None

        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        response.data = True
        self.get_logger().info('Navigation cancelled')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = IsaacNavigationCore()

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

## Isaac ROS Costmap

### GPU-Accelerated Costmap Generation

```python
# costmap_generation.py
import numpy as np
import cv2
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import cupy as cp  # For GPU operations

class IsaacCostmap:
    def __init__(self, logger, gpu_available=True):
        self.logger = logger
        self.gpu_available = gpu_available

        # Costmap parameters
        self.resolution = 0.05  # 5cm resolution
        self.width = 400  # 20m x 20m map
        self.height = 400
        self.origin_x = -10.0
        self.origin_y = -10.0

        # Initialize costmap
        self.costmap = np.zeros((self.height, self.width), dtype=np.uint8)
        self.updated_cells = set()

        # GPU resources
        self.gpu_costmap = None
        if self.gpu_available:
            self._initialize_gpu_costmap()

        # Obstacle inflation parameters
        self.inflation_radius = 0.5  # 50cm inflation
        self.cost_scaling_factor = 10.0

        self.logger.info(f'Costmap initialized: {self.width}x{self.height}, resolution: {self.resolution}m')

    def _initialize_gpu_costmap(self):
        """Initialize GPU costmap"""
        try:
            self.gpu_costmap = cp.zeros((self.height, self.width), dtype=cp.uint8)
            self.logger.info('GPU costmap initialized')
        except Exception as e:
            self.logger.warn(f'GPU costmap initialization failed: {e}')
            self.gpu_available = False

    def update_from_scan(self, scan_msg):
        """Update costmap from laser scan data"""
        try:
            # Convert scan ranges to points
            angles = np.linspace(
                scan_msg.angle_min,
                scan_msg.angle_max,
                len(scan_msg.ranges)
            )

            valid_ranges = []
            valid_angles = []

            for i, range_val in enumerate(scan_msg.ranges):
                if scan_msg.range_min <= range_val <= scan_msg.range_max:
                    valid_ranges.append(range_val)
                    valid_angles.append(angles[i])

            if not valid_ranges:
                return

            # Convert to Cartesian coordinates
            x_points = np.array(valid_ranges) * np.cos(valid_angles)
            y_points = np.array(valid_ranges) * np.sin(valid_angles)

            # Update costmap with obstacles
            self._add_obstacles_to_costmap(x_points, y_points)

            # Update occupancy grid
            self._inflate_obstacles()

        except Exception as e:
            self.logger.error(f'Error updating costmap from scan: {e}')

    def update_from_pointcloud(self, pc_msg):
        """Update costmap from point cloud data"""
        try:
            # Convert point cloud to numpy array
            # This would use point_cloud2.read_points() in practice
            # For now, we'll simulate the conversion

            # Simulate point cloud processing
            # In practice, this would extract x, y coordinates from 3D points
            x_points = np.random.uniform(-5, 5, 100)  # Placeholder
            y_points = np.random.uniform(-5, 5, 100)  # Placeholder

            # Update costmap with point cloud obstacles
            self._add_obstacles_to_costmap(x_points, y_points, height_threshold=0.5)

        except Exception as e:
            self.logger.error(f'Error updating costmap from point cloud: {e}')

    def _add_obstacles_to_costmap(self, x_points, y_points, height_threshold=0.0):
        """Add obstacles to costmap using GPU acceleration"""
        try:
            # Convert world coordinates to costmap indices
            x_indices = ((x_points - self.origin_x) / self.resolution).astype(int)
            y_indices = ((y_points - self.origin_y) / self.resolution).astype(int)

            # Filter valid indices
            valid_mask = (
                (x_indices >= 0) & (x_indices < self.width) &
                (y_indices >= 0) & (y_indices < self.height)
            )

            valid_x = x_indices[valid_mask]
            valid_y = y_indices[valid_mask]

            if len(valid_x) == 0:
                return

            # Update costmap
            if self.gpu_available:
                # Use GPU for updates
                gpu_x = cp.array(valid_x)
                gpu_y = cp.array(valid_y)

                # Set obstacle costs (254 = lethal obstacle)
                self.gpu_costmap[gpu_y, gpu_x] = 254

                # Copy back to CPU if needed
                self.costmap = cp.asnumpy(self.gpu_costmap)
            else:
                # CPU fallback
                self.costmap[valid_y, valid_x] = 254

            # Track updated cells
            for x, y in zip(valid_x, valid_y):
                self.updated_cells.add((x, y))

        except Exception as e:
            self.logger.error(f'Error adding obstacles to costmap: {e}')

    def _inflate_obstacles(self):
        """Inflate obstacles using GPU acceleration"""
        try:
            if self.gpu_available:
                # GPU-based inflation using distance transform
                gpu_costmap = cp.asarray(self.costmap)

                # Create binary obstacle map
                gpu_obstacles = (gpu_costmap == 254).astype(cp.uint8)

                # Compute distance transform on GPU
                gpu_distance = cp.zeros_like(gpu_costmap, dtype=cp.float32)

                # For now, use CPU-based approach for distance transform
                # In practice, this would use GPU-accelerated distance transform
                cpu_obstacles = cp.asnumpy(gpu_obstacles)
                distance_map = distance_transform_edt(1 - cpu_obstacles)

                # Apply inflation based on distance
                inflation_mask = (distance_map <= self.inflation_radius) & (self.costmap == 0)
                inflation_values = np.clip(
                    (self.inflation_radius - distance_map[inflation_mask]) * self.cost_scaling_factor,
                    1, 253
                )
                self.costmap[inflation_mask] = inflation_values.astype(np.uint8)

            else:
                # CPU-based inflation
                obstacles = (self.costmap == 254).astype(np.uint8)
                distance_map = distance_transform_edt(1 - obstacles)

                inflation_mask = (distance_map <= self.inflation_radius) & (self.costmap == 0)
                inflation_values = np.clip(
                    (self.inflation_radius - distance_map[inflation_mask]) * self.cost_scaling_factor,
                    1, 253
                )
                self.costmap[inflation_mask] = inflation_values.astype(np.uint8)

        except Exception as e:
            self.logger.error(f'Error inflating obstacles: {e}')

    def get_costmap(self):
        """Get current costmap"""
        return self.costmap

    def get_cost_at_point(self, x, y):
        """Get cost at specific point"""
        try:
            x_idx = int((x - self.origin_x) / self.resolution)
            y_idx = int((y - self.origin_y) / self.resolution)

            if 0 <= x_idx < self.width and 0 <= y_idx < self.height:
                return self.costmap[y_idx, x_idx]
            else:
                return 255  # Unknown/invalid
        except:
            return 255

    def update(self):
        """Update costmap with latest sensor data"""
        # This would be called periodically to refresh the costmap
        # with the latest sensor observations
        pass

def create_costmap(logger):
    """Create Isaac ROS costmap instance"""
    return IsaacCostmap(logger)
```

## Isaac ROS Path Planner

### GPU-Accelerated Path Planning

```python
# path_planning.py
import numpy as np
from scipy.spatial import KDTree
import heapq
import cupy as cp

class IsaacPathPlanner:
    def __init__(self, logger, gpu_available=True):
        self.logger = logger
        self.gpu_available = gpu_available
        self.costmap = None

        # Planning parameters
        self.step_size = 0.5  # 50cm steps
        self.max_iterations = 5000
        self.smoothing_iterations = 10

        self.logger.info('Isaac Path Planner initialized')

    def plan_path(self, start, goal, costmap):
        """Plan path using GPU-accelerated A* algorithm"""
        self.costmap = costmap

        try:
            # Use GPU-accelerated path planning if available
            if self.gpu_available:
                path = self._gpu_astar_planning(start, goal)
            else:
                path = self._cpu_astar_planning(start, goal)

            if path is not None:
                # Smooth the path
                smoothed_path = self._smooth_path(path)
                return smoothed_path
            else:
                return None

        except Exception as e:
            self.logger.error(f'Path planning error: {e}')
            return None

    def _gpu_astar_planning(self, start, goal):
        """GPU-accelerated A* path planning"""
        try:
            # Convert to costmap coordinates
            start_idx = self._world_to_costmap_index(start)
            goal_idx = self._world_to_costmap_index(goal)

            if start_idx is None or goal_idx is None:
                return None

            # Create GPU-based priority queue and visited array
            height, width = self.costmap.get_costmap().shape
            visited = cp.zeros((height, width), dtype=cp.bool_)
            costs = cp.full((height, width), cp.inf, dtype=cp.float32)
            parents = cp.full((height, width, 2), -1, dtype=cp.int32)

            # Initialize start position
            costs[start_idx[1], start_idx[0]] = 0

            # Use CPU-based priority queue for now (GPU priority queue implementation is complex)
            # In practice, this would use GPU-optimized data structures
            return self._cpu_astar_planning(start, goal)

        except Exception as e:
            self.logger.warn(f'GPU path planning failed: {e}, falling back to CPU')
            return self._cpu_astar_planning(start, goal)

    def _cpu_astar_planning(self, start, goal):
        """CPU-based A* path planning"""
        try:
            # Convert to costmap coordinates
            start_idx = self._world_to_costmap_index(start)
            goal_idx = self._world_to_costmap_index(goal)

            if start_idx is None or goal_idx is None:
                return None

            # Check if start and goal are traversable
            start_cost = self.costmap.get_cost_at_point(start[0], start[1])
            goal_cost = self.costmap.get_cost_at_point(goal[0], goal[1])

            if start_cost >= 253 or goal_cost >= 253:  # Lethal or inflated obstacle
                self.logger.warn('Start or goal position is blocked')
                return None

            # A* algorithm implementation
            open_set = [(0, start_idx)]  # (f_score, position)
            came_from = {}
            g_score = {start_idx: 0}
            f_score = {start_idx: self._heuristic(start_idx, goal_idx)}

            visited = set()

            while open_set:
                current = heapq.heappop(open_set)[1]

                if current == goal_idx:
                    # Reconstruct path
                    path = self._reconstruct_path(came_from, current)
                    return path

                visited.add(current)

                # Check neighbors (8-connectivity)
                for neighbor in self._get_neighbors(current):
                    if neighbor in visited:
                        continue

                    # Check if neighbor is traversable
                    neighbor_world = self._costmap_index_to_world(neighbor)
                    neighbor_cost = self.costmap.get_cost_at_point(neighbor_world[0], neighbor_world[1])

                    if neighbor_cost >= 253:  # Blocked
                        continue

                    tentative_g_score = g_score[current] + self._distance(current, neighbor)

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self._heuristic(neighbor, goal_idx)

                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

                # Prevent infinite loops
                if len(visited) > self.max_iterations:
                    break

            # Path not found
            return None

        except Exception as e:
            self.logger.error(f'CPU A* planning error: {e}')
            return None

    def _heuristic(self, pos1, pos2):
        """Heuristic function for A* (Euclidean distance)"""
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def _distance(self, pos1, pos2):
        """Distance between two positions"""
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def _get_neighbors(self, pos):
        """Get 8-connected neighbors"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                new_x = pos[0] + dx
                new_y = pos[1] + dy

                # Check bounds
                if 0 <= new_x < self.costmap.width and 0 <= new_y < self.costmap.height:
                    neighbors.append((new_x, new_y))

        return neighbors

    def _reconstruct_path(self, came_from, current):
        """Reconstruct path from A* result"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()

        # Convert back to world coordinates
        world_path = []
        for idx in path:
            world_pos = self._costmap_index_to_world(idx)
            world_path.append(world_pos)

        return world_path

    def _world_to_costmap_index(self, world_pos):
        """Convert world coordinates to costmap index"""
        x, y = world_pos
        x_idx = int((x - self.costmap.origin_x) / self.costmap.resolution)
        y_idx = int((y - self.costmap.origin_y) / self.costmap.resolution)

        if 0 <= x_idx < self.costmap.width and 0 <= y_idx < self.costmap.height:
            return (x_idx, y_idx)
        else:
            return None

    def _costmap_index_to_world(self, index):
        """Convert costmap index to world coordinates"""
        x_idx, y_idx = index
        x = x_idx * self.costmap.resolution + self.costmap.origin_x
        y = y_idx * self.costmap.resolution + self.costmap.origin_y
        return (x, y)

    def _smooth_path(self, path):
        """Smooth the path using GPU acceleration if available"""
        if len(path) < 3:
            return path

        try:
            path_array = np.array(path)

            # Apply smoothing using moving average
            smoothed_path = []
            for i in range(len(path_array)):
                start_idx = max(0, i - 2)
                end_idx = min(len(path_array), i + 3)
                segment = path_array[start_idx:end_idx]
                smoothed_point = np.mean(segment, axis=0)
                smoothed_path.append(smoothed_point)

            return smoothed_path

        except Exception as e:
            self.logger.error(f'Path smoothing error: {e}')
            return path

def create_path_planner(logger):
    """Create Isaac ROS path planner instance"""
    return IsaacPathPlanner(logger)
```

## Isaac ROS Trajectory Controller

### GPU-Accelerated Trajectory Control

```python
# trajectory_controller.py
import numpy as np
from geometry_msgs.msg import Twist, Pose, Point
from tf_transformations import euler_from_quaternion
import math

class IsaacTrajectoryController:
    def __init__(self, logger):
        self.logger = logger

        # Control parameters
        self.linear_gain = 1.0
        self.angular_gain = 2.0
        self.lookahead_distance = 1.0
        self.max_linear_vel = 0.5
        self.max_angular_vel = 1.0
        self.arrival_threshold = 0.2

        # Path following state
        self.current_waypoint_idx = 0
        self.path_following_tolerance = 0.3

        self.logger.info('Isaac Trajectory Controller initialized')

    def follow_path(self, current_pose, path, costmap):
        """Follow the given path using control laws"""
        if not path or len(path) == 0:
            return None

        try:
            # Get current position and orientation
            current_pos = np.array([current_pose.position.x, current_pose.position.y])
            current_yaw = self._get_yaw_from_pose(current_pose)

            # Find closest waypoint
            closest_idx = self._find_closest_waypoint(current_pos, path)
            if closest_idx is None:
                return None

            # Determine target point ahead
            target_point = self._get_lookahead_point(current_pos, path, closest_idx)

            if target_point is None:
                return None

            # Calculate control commands
            cmd_vel = self._calculate_control_commands(current_pos, current_yaw, target_point)

            # Check for obstacles
            if self._check_for_obstacles(current_pos, target_point, costmap):
                # Reduce speed or stop if obstacles detected
                cmd_vel.linear.x *= 0.5  # Slow down
                cmd_vel.angular.z *= 0.5

            return cmd_vel

        except Exception as e:
            self.logger.error(f'Trajectory following error: {e}')
            return None

    def _find_closest_waypoint(self, current_pos, path):
        """Find the closest waypoint to current position"""
        if len(path) == 0:
            return None

        min_dist = float('inf')
        closest_idx = 0

        for i, waypoint in enumerate(path):
            dist = np.linalg.norm(current_pos - np.array(waypoint))
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def _get_lookahead_point(self, current_pos, path, closest_idx):
        """Get lookahead point for pure pursuit"""
        # Look ahead along the path
        for i in range(closest_idx, len(path)):
            waypoint = np.array(path[i])
            dist = np.linalg.norm(current_pos - waypoint)

            if dist >= self.lookahead_distance:
                return waypoint

        # If no point is far enough, return the last point
        if len(path) > 0:
            return np.array(path[-1])
        else:
            return None

    def _calculate_control_commands(self, current_pos, current_yaw, target_point):
        """Calculate linear and angular velocities"""
        cmd_vel = Twist()

        # Calculate vector to target
        dx = target_point[0] - current_pos[0]
        dy = target_point[1] - current_pos[1]

        # Calculate distance to target
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate angle to target
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - current_yaw

        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Proportional control
        linear_vel = min(self.linear_gain * distance, self.max_linear_vel)
        angular_vel = self.angular_gain * angle_diff

        # Clamp velocities
        linear_vel = max(0, min(linear_vel, self.max_linear_vel))
        angular_vel = max(-self.max_angular_vel, min(angular_vel, self.max_angular_vel))

        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel

        return cmd_vel

    def _get_yaw_from_pose(self, pose):
        """Extract yaw angle from pose quaternion"""
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        return euler[2]  # Yaw

    def _check_for_obstacles(self, current_pos, target_pos, costmap):
        """Check for obstacles along the path to target"""
        # Simple check: sample points along the path
        steps = 10
        for i in range(1, steps + 1):
            t = i / steps
            check_pos = current_pos + t * (target_pos - current_pos)

            cost = costmap.get_cost_at_point(check_pos[0], check_pos[1])
            if cost >= 250:  # High cost indicates obstacle
                return True

        return False

def create_trajectory_controller(logger):
    """Create Isaac ROS trajectory controller instance"""
    return IsaacTrajectoryController(logger)
```

## Isaac ROS Obstacle Detector

### Real-time Obstacle Detection

```python
# obstacle_detector.py
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Point
import math

class IsaacObstacleDetector:
    def __init__(self, logger, gpu_available=True):
        self.logger = logger
        self.gpu_available = gpu_available

        # Obstacle detection parameters
        self.detection_range = 3.0  # 3m detection range
        self.obstacle_threshold = 200  # Cost threshold for obstacles
        self.clearance_distance = 0.8  # Minimum clearance

        # Sensor data
        self.latest_scan = None
        self.scan_timestamp = None

        self.logger.info('Isaac Obstacle Detector initialized')

    def update_scan_data(self, scan_msg):
        """Update with latest laser scan data"""
        self.latest_scan = scan_msg
        self.scan_timestamp = scan_msg.header.stamp

    def has_obstacles_ahead(self):
        """Check if there are obstacles directly ahead"""
        if self.latest_scan is None:
            return False

        try:
            # Check forward-looking scan rays
            forward_rays = self._get_forward_rays()

            for distance in forward_rays:
                if 0.1 < distance < self.clearance_distance:  # Valid range, too close
                    return True

            return False

        except Exception as e:
            self.logger.error(f'Obstacle detection error: {e}')
            return False

    def _get_forward_rays(self):
        """Get forward-facing laser rays"""
        if self.latest_scan is None:
            return []

        # Define forward sector (e.g., +/- 30 degrees from heading)
        angle_increment = self.latest_scan.angle_increment
        angle_min = self.latest_scan.angle_min

        forward_angles = []
        forward_distances = []

        # Sample rays in forward direction
        center_idx = len(self.latest_scan.ranges) // 2
        sample_width = int(30 * math.pi / 180 / angle_increment)  # 30-degree sector

        start_idx = max(0, center_idx - sample_width // 2)
        end_idx = min(len(self.latest_scan.ranges), center_idx + sample_width // 2)

        for i in range(start_idx, end_idx):
            if (self.latest_scan.range_min <= self.latest_scan.ranges[i] <= self.latest_scan.range_max):
                forward_distances.append(self.latest_scan.ranges[i])

        return forward_distances

    def get_obstacle_direction(self):
        """Get direction of nearest obstacle"""
        if self.latest_scan is None:
            return None

        try:
            min_distance = float('inf')
            min_angle = 0

            for i, distance in enumerate(self.latest_scan.ranges):
                if (self.latest_scan.range_min <= distance <= self.latest_scan.range_max and
                    distance < min_distance):
                    min_distance = distance
                    min_angle = self.latest_scan.angle_min + i * self.latest_scan.angle_increment

            if min_distance < float('inf'):
                return {'angle': min_angle, 'distance': min_distance}
            else:
                return None

        except Exception as e:
            self.logger.error(f'Obstacle direction detection error: {e}')
            return None

    def get_free_space_direction(self):
        """Get direction with maximum free space"""
        if self.latest_scan is None:
            return None

        try:
            # Divide scan into sectors and find sector with maximum distance
            sector_size = 30  # degrees
            num_sectors = 360 // sector_size
            sector_distances = [0] * num_sectors

            angle_increment = self.latest_scan.angle_increment

            for i, distance in enumerate(self.latest_scan.ranges):
                if self.latest_scan.range_min <= distance <= self.latest_scan.range_max:
                    angle = self.latest_scan.angle_min + i * angle_increment
                    sector_idx = int((angle + math.pi) / (2 * math.pi) * num_sectors) % num_sectors
                    sector_distances[sector_idx] = max(sector_distances[sector_idx], distance)

            max_sector = max(range(len(sector_distances)), key=lambda i: sector_distances[i])
            if sector_distances[max_sector] > 0:
                angle = (max_sector * sector_size - 180) * math.pi / 180
                return {'angle': angle, 'distance': sector_distances[max_sector]}
            else:
                return None

        except Exception as e:
            self.logger.error(f'Free space detection error: {e}')
            return None

def create_obstacle_detector(logger):
    """Create Isaac ROS obstacle detector instance"""
    return IsaacObstacleDetector(logger)
```

## Isaac ROS Navigation Configuration

### Complete Navigation Launch Configuration

```python
# navigation_launch.py
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
    enable_visual_slam = LaunchConfiguration('enable_visual_slam', default='true')
    enable_path_planning = LaunchConfiguration('enable_path_planning', default='true')
    enable_controller = LaunchConfiguration('enable_controller', default='true')

    # Isaac ROS Navigation Container
    navigation_container = ComposableNodeContainer(
        name='isaac_navigation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[{'use_sim_time': use_sim_time}],
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                condition=IfCondition(enable_visual_slam),
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
            ),
            ComposableNode(
                package='isaac_ros_path_planner',
                plugin='nvidia::isaac_ros::path_planner::PathPlannerNode',
                name='path_planner',
                condition=IfCondition(enable_path_planning),
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'planning_algorithm': 'astar',
                    'max_iterations': 5000,
                    'smoothing_iterations': 10,
                    'step_size': 0.5,
                    'inflation_radius': 0.5,
                    'cost_scaling_factor': 10.0
                }],
                remappings=[
                    ('/path_planner/costmap', '/costmap'),
                    ('/path_planner/goal', '/move_base_simple/goal'),
                    ('/path_planner/path', '/plan')
                ]
            ),
            ComposableNode(
                package='isaac_ros_trajectory_controller',
                plugin='nvidia::isaac_ros::trajectory_controller::TrajectoryControllerNode',
                name='trajectory_controller',
                condition=IfCondition(enable_controller),
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'linear_gain': 1.0,
                    'angular_gain': 2.0,
                    'lookahead_distance': 1.0,
                    'max_linear_vel': 0.5,
                    'max_angular_vel': 1.0,
                    'arrival_threshold': 0.2
                }],
                remappings=[
                    ('/trajectory_controller/current_pose', '/amcl_pose'),
                    ('/trajectory_controller/path', '/plan'),
                    ('/trajectory_controller/cmd_vel', '/cmd_vel')
                ]
            )
        ]
    )

    # Additional standalone nodes
    costmap_node = Node(
        package='costmap_2d',
        executable='costmap_2d_node',
        name='costmap_node',
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory('isaac_ros_navigation'),
                'config',
                'costmap.yaml'
            ])
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/camera/depth/image', '/camera/depth/image_rect'),
            ('/costmap/costmap', '/costmap'),
            ('/costmap/costmap_updates', '/costmap_updates')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'enable_visual_slam',
            default_value='true',
            description='Enable Isaac ROS Visual SLAM'
        ),
        DeclareLaunchArgument(
            'enable_path_planning',
            default_value='true',
            description='Enable Isaac ROS Path Planning'
        ),
        DeclareLaunchArgument(
            'enable_controller',
            default_value='true',
            description='Enable Isaac ROS Trajectory Controller'
        ),
        navigation_container,
        costmap_node
    ])
```

## Performance Optimization

### Navigation Performance Tuning

```python
# navigation_performance.py
class NavigationPerformanceTuner:
    def __init__(self, navigation_node):
        self.navigation_node = navigation_node
        self.performance_metrics = {
            'path_planning_time': [],
            'control_loop_rate': [],
            'costmap_update_rate': [],
            'memory_usage': [],
            'cpu_usage': []
        }

    def monitor_performance(self):
        """Monitor navigation performance metrics"""
        # This would collect real-time performance data
        metrics = {
            'path_planning_time': 0.0,  # Placeholder
            'control_loop_rate': 0.0,
            'costmap_update_rate': 0.0,
            'memory_usage': 0.0,
            'cpu_usage': 0.0
        }

        # Update performance history
        for key, value in metrics.items():
            self.performance_metrics[key].append(value)
            if len(self.performance_metrics[key]) > 100:  # Keep last 100 samples
                self.performance_metrics[key].pop(0)

        return metrics

    def auto_tune_parameters(self):
        """Auto-tune navigation parameters based on performance"""
        if len(self.performance_metrics['path_planning_time']) < 10:
            return

        # Calculate averages
        avg_planning_time = sum(self.performance_metrics['path_planning_time'][-10:]) / 10
        avg_control_rate = sum(self.performance_metrics['control_loop_rate'][-10:]) / 10

        # Adjust parameters based on performance
        adjustments = {}

        # If planning is too slow, reduce complexity
        if avg_planning_time > 0.1:  # 100ms threshold
            adjustments['max_iterations'] = max(1000, self.navigation_node.planner.max_iterations * 0.8)
            adjustments['smoothing_iterations'] = max(1, self.navigation_node.planner.smoothing_iterations * 0.5)

        # If control rate is too low, adjust controller parameters
        if avg_control_rate < 10:  # Below 10Hz
            adjustments['lookahead_distance'] = max(0.5, self.navigation_node.controller.lookahead_distance * 0.8)

        # Apply adjustments
        for param, value in adjustments.items():
            if hasattr(self.navigation_node, param):
                setattr(self.navigation_node, param, value)

def optimize_navigation_performance(navigation_node):
    """Set up performance monitoring and optimization for navigation"""
    tuner = NavigationPerformanceTuner(navigation_node)

    # Set up periodic performance monitoring
    navigation_node.performance_timer = navigation_node.create_timer(
        1.0,  # Every second
        lambda: tuner.monitor_performance()
    )

    # Set up periodic auto-tuning
    navigation_node.tuning_timer = navigation_node.create_timer(
        10.0,  # Every 10 seconds
        lambda: tuner.auto_tune_parameters()
    )

    return tuner
```

## Best Practices

1. **GPU Resource Management**: Efficiently manage GPU memory and compute resources
2. **Real-time Performance**: Ensure navigation algorithms run at sufficient frequency
3. **Safety First**: Always implement obstacle detection and emergency stopping
4. **Path Smoothing**: Smooth planned paths for comfortable robot motion
5. **Costmap Inflation**: Properly inflate obstacles for safe navigation
6. **Sensor Fusion**: Integrate multiple sensors for robust navigation
7. **Parameter Tuning**: Adjust parameters based on robot dynamics and environment
8. **Error Recovery**: Implement recovery behaviors for navigation failures

## Next Steps

In the next chapter, we'll explore Isaac ROS-specific integration techniques, learning how to leverage Isaac's unique features and capabilities to enhance your humanoid robot's navigation and autonomy in complex environments.