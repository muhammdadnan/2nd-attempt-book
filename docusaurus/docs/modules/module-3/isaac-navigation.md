# Isaac Navigation

In this chapter, we'll explore NVIDIA Isaac's navigation capabilities, which provide GPU-accelerated path planning, obstacle avoidance, and navigation specifically optimized for humanoid robots. Isaac Navigation leverages NVIDIA's hardware acceleration to deliver high-performance navigation systems that can handle complex environments and real-time requirements.

## Isaac Navigation Overview

### GPU-Accelerated Navigation Components

Isaac Navigation includes several GPU-accelerated navigation packages:

- **Isaac ROS Visual SLAM**: GPU-accelerated simultaneous localization and mapping
- **Isaac ROS Path Planning**: GPU-parallelized path planning algorithms
- **Isaac ROS Costmap**: GPU-accelerated costmap generation and updates
- **Isaac ROS Controller**: GPU-optimized trajectory control
- **Isaac ROS Obstacle Avoidance**: Real-time GPU-accelerated collision avoidance

### Navigation System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          Isaac Navigation Architecture                         │
├─────────────────────────────────────────────────────────────────────────────────┤
│  Perception → Localization → Mapping → Path Planning → Control → Execution     │
│  (GPU)      → (GPU)      → (GPU)   → (GPU)        → (GPU)   → (Robot)       │
├─────────────────────────────────────────────────────────────────────────────────┤
│  GPU Components:                                                              │
│  - CUDA-based path planning algorithms                                         │
│  - TensorRT-accelerated obstacle detection                                     │
│  - GPU-parallelized map updates                                                │
│  - CUDA-optimized trajectory generation                                        │
│  - Real-time collision checking                                                │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Isaac ROS Visual SLAM

### GPU-Accelerated SLAM Implementation

```python
# visual_slam_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import jetson.inference
import jetson.utils

class IsaacVisualSLAMNode(Node):
    def __init__(self):
        super().__init__('isaac_visual_slam')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize Visual SLAM components
        self._initialize_visual_slam()

        # Publishers and subscribers
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

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/visual_slam/odometry', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/visual_slam/pose', 10)
        self.path_pub = self.create_publisher(Path, '/visual_slam/path', 10)

        # SLAM state
        self.current_pose = np.eye(4)
        self.pose_history = []
        self.map_points = []
        self.keyframes = []

        self.get_logger().info('Isaac Visual SLAM node initialized')

    def _initialize_visual_slam(self):
        """Initialize GPU-accelerated Visual SLAM"""
        try:
            # Initialize Isaac Visual SLAM components
            # This would typically use Isaac's GPU-accelerated SLAM libraries
            self.vslam_initialized = True

            # Initialize tracking parameters
            self.min_features = 100
            self.max_features = 1000
            self.feature_threshold = 0.01
            self.relocalization_threshold = 0.7

            # Initialize map management
            self.map_update_rate = 5.0  # Hz
            self.keyframe_threshold = 0.5  # meters

            self.get_logger().info('Isaac Visual SLAM initialized successfully')

        except Exception as e:
            self.get_logger().error(f'Visual SLAM initialization error: {e}')
            self.vslam_initialized = False

    def left_image_callback(self, msg):
        """Process left camera image for SLAM"""
        if not self.vslam_initialized:
            return

        try:
            # Convert ROS Image to CUDA image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cuda_image = jetson.utils.cudaFromNumpy(cv_image)

            # Perform GPU-accelerated feature tracking
            features = self._track_features_gpu(cuda_image)

            # Update pose estimate
            if len(features) >= self.min_features:
                self._update_pose_estimate(features, msg.header.stamp)

        except Exception as e:
            self.get_logger().error(f'Left image SLAM processing error: {e}')

    def right_image_callback(self, msg):
        """Process right camera image for stereo depth"""
        if not self.vslam_initialized:
            return

        try:
            # Convert ROS Image to CUDA image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cuda_image = jetson.utils.cudaFromNumpy(cv_image)

            # Perform GPU-accelerated stereo processing
            depth_map = self._compute_stereo_depth_gpu(cuda_image)

            # Update map with depth information
            self._update_map_with_depth(depth_map)

        except Exception as e:
            self.get_logger().error(f'Right image SLAM processing error: {e}')

    def imu_callback(self, msg):
        """Process IMU data for sensor fusion"""
        # Use IMU data to improve pose estimation
        self._fuse_imu_data(msg)

    def _track_features_gpu(self, cuda_image):
        """Track features using GPU acceleration"""
        try:
            # This would use Isaac's GPU-accelerated feature tracking
            # For this example, we'll simulate the process
            features = []

            # In a real implementation, this would use CUDA kernels
            # to perform feature detection and tracking
            # Example: GPU-accelerated ORB, SIFT, or other feature detectors

            # Simulate feature extraction
            for i in range(200):  # Simulate 200 features
                feature = {
                    'x': np.random.randint(0, 640),
                    'y': np.random.randint(0, 480),
                    'response': np.random.uniform(0.1, 1.0),
                    'descriptor': np.random.rand(32).astype(np.float32)  # Descriptor vector
                }
                features.append(feature)

            return features

        except Exception as e:
            self.get_logger().error(f'GPU feature tracking error: {e}')
            return []

    def _compute_stereo_depth_gpu(self, right_cuda_image):
        """Compute depth using GPU-accelerated stereo vision"""
        try:
            # This would use Isaac's GPU-accelerated stereo algorithms
            # In practice, this would implement CUDA-based stereo matching

            # Simulate depth computation
            height, width = right_cuda_image.shape[:2]
            depth_map = np.random.uniform(0.1, 10.0, (height, width)).astype(np.float32)

            return depth_map

        except Exception as e:
            self.get_logger().error(f'GPU stereo depth computation error: {e}')
            return None

    def _update_pose_estimate(self, features, timestamp):
        """Update robot pose estimate using tracked features"""
        try:
            # Perform GPU-accelerated pose estimation
            # This would use GPU-parallelized PnP (Perspective-n-Point) solver

            # In a real implementation, this would:
            # 1. Match features with previous frame
            # 2. Compute motion using GPU-parallelized algorithms
            # 3. Update pose estimate

            # For simulation, update pose based on time
            dt = 0.033  # Assuming 30Hz frame rate
            self.current_pose[0, 3] += 0.01  # Move forward slowly
            self.current_pose[1, 3] += 0.005 * np.sin(timestamp.sec)  # Oscillate sideways

            # Publish odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = timestamp
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id = 'base_link'

            # Extract position and orientation from transformation matrix
            position = self.current_pose[:3, 3]
            odom_msg.pose.pose.position.x = position[0]
            odom_msg.pose.pose.position.y = position[1]
            odom_msg.pose.pose.position.z = position[2]

            # Convert rotation matrix to quaternion
            rotation_matrix = self.current_pose[:3, :3]
            qw = np.sqrt(1 + rotation_matrix[0,0] + rotation_matrix[1,1] + rotation_matrix[2,2]) / 2
            qx = (rotation_matrix[2,1] - rotation_matrix[1,2]) / (4 * qw)
            qy = (rotation_matrix[0,2] - rotation_matrix[2,0]) / (4 * qw)
            qz = (rotation_matrix[1,0] - rotation_matrix[0,1]) / (4 * qw)

            odom_msg.pose.pose.orientation.x = qx
            odom_msg.pose.pose.orientation.y = qy
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw

            self.odom_pub.publish(odom_msg)

            # Publish pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = timestamp
            pose_msg.header.frame_id = 'map'
            pose_msg.pose = odom_msg.pose.pose
            self.pose_pub.publish(pose_msg)

            # Update pose history
            self.pose_history.append((timestamp, self.current_pose.copy()))

            # Check if we should add a keyframe
            if self._should_add_keyframe():
                self._add_keyframe(features, self.current_pose.copy())

            self.get_logger().info(f'SLAM pose updated: x={position[0]:.3f}, y={position[1]:.3f}')

        except Exception as e:
            self.get_logger().error(f'Pose estimation error: {e}')

    def _should_add_keyframe(self) -> bool:
        """Determine if current frame should be a keyframe"""
        if not self.keyframes:
            return True

        # Check translation distance
        last_pose = self.keyframes[-1]['pose']
        current_pos = self.current_pose[:3, 3]
        last_pos = last_pose[:3, 3]

        distance = np.linalg.norm(current_pos - last_pos)
        return distance > self.keyframe_threshold

    def _add_keyframe(self, features, pose):
        """Add current frame as a keyframe"""
        keyframe = {
            'timestamp': self.get_clock().now().to_msg(),
            'features': features,
            'pose': pose,
            'id': len(self.keyframes)
        }

        self.keyframes.append(keyframe)

        # Limit keyframe history
        if len(self.keyframes) > 100:  # Keep last 100 keyframes
            self.keyframes.pop(0)

        self.get_logger().info(f'Added keyframe {keyframe["id"]} at position {pose[:3, 3]}')

    def _fuse_imu_data(self, imu_msg):
        """Fuse IMU data with visual SLAM"""
        # Use IMU data to improve pose estimation
        # This would implement sensor fusion algorithms like EKF or UKF
        pass

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

## Isaac ROS Path Planning

### GPU-Accelerated Path Planning

```python
# path_planning_integration.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from builtin_interfaces.msg import Duration
import numpy as np
import cupy as cp  # For GPU operations
from scipy.spatial import KDTree
import heapq

class IsaacPathPlanningNode(Node):
    def __init__(self):
        super().__init__('isaac_path_planning')

        # Publishers and subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap',
            self.costmap_callback,
            10
        )

        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.visualization_pub = self.create_publisher(MarkerArray, '/path_visualization', 10)

        # Initialize GPU-accelerated path planning
        self._initialize_gpu_path_planning()

        # Planning parameters
        self.planning_algorithm = 'astar_gpu'  # Options: astar_gpu, dijkstra_gpu, rrt_gpu
        self.planning_resolution = 0.1  # 10cm resolution
        self.inflation_radius = 0.5  # 50cm obstacle inflation
        self.max_iterations = 5000
        self.smoothing_iterations = 10

        # Current state
        self.current_costmap = None
        self.current_start = None
        self.current_goal = None

        self.get_logger().info('Isaac Path Planning node initialized')

    def _initialize_gpu_path_planning(self):
        """Initialize GPU-accelerated path planning components"""
        try:
            # Check if GPU is available
            self.gpu_available = True
            self.get_logger().info('GPU path planning initialized')

            # Initialize GPU memory pools for efficient allocation
            try:
                import cupy as cp
                self.gpu_memory_pool = cp.cuda.MemoryPool()
                cp.cuda.set_allocator(self.gpu_memory_pool.malloc)
            except ImportError:
                self.gpu_available = False
                self.get_logger().warn('CuPy not available, using CPU path planning')

        except Exception as e:
            self.get_logger().error(f'GPU path planning initialization error: {e}')
            self.gpu_available = False

    def costmap_callback(self, msg):
        """Update costmap for path planning"""
        try:
            # Convert OccupancyGrid to numpy array
            costmap_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)

            # Store costmap with metadata
            self.current_costmap = {
                'data': costmap_data,
                'info': msg.info,
                'header': msg.header
            }

            self.get_logger().info(f'Updated costmap: {msg.info.width}x{msg.info.height}')

        except Exception as e:
            self.get_logger().error(f'Costmap processing error: {e}')

    def goal_callback(self, msg):
        """Process navigation goal and plan path"""
        if self.current_costmap is None:
            self.get_logger().warn('No costmap available, cannot plan path')
            return

        try:
            self.current_goal = msg.pose
            self.current_start = self._get_current_robot_pose()

            if self.current_start:
                # Plan path using GPU acceleration
                path = self._plan_path_gpu(self.current_start, self.current_goal)

                if path:
                    # Smooth path using GPU acceleration
                    smoothed_path = self._smooth_path_gpu(path)

                    # Publish path
                    path_msg = self._create_path_message(smoothed_path, msg.header)
                    self.path_pub.publish(path_msg)

                    # Visualize path
                    self._visualize_path(smoothed_path)

                    self.get_logger().info(f'Path planned with {len(smoothed_path)} waypoints')

                else:
                    self.get_logger().error('Failed to plan path')

        except Exception as e:
            self.get_logger().error(f'Path planning error: {e}')

    def _plan_path_gpu(self, start_pose, goal_pose):
        """Plan path using GPU acceleration"""
        if not self.gpu_available or self.current_costmap is None:
            return self._plan_path_cpu(start_pose, goal_pose)

        try:
            # Convert poses to grid coordinates
            start_grid = self._world_to_grid(start_pose.position.x, start_pose.position.y)
            goal_grid = self._world_to_grid(goal_pose.position.x, goal_pose.position.y)

            if start_grid is None or goal_grid is None:
                self.get_logger().error('Start or goal pose outside costmap bounds')
                return None

            # Upload costmap to GPU
            gpu_costmap = cp.asarray(self.current_costmap['data'])

            # Perform GPU-accelerated path planning
            path = self._gpu_astar_search(gpu_costmap, start_grid, goal_grid)

            # Convert path back to world coordinates
            world_path = []
            for grid_point in path:
                world_point = self._grid_to_world(grid_point[0], grid_point[1])
                if world_point:
                    world_path.append(world_point)

            return world_path

        except Exception as e:
            self.get_logger().error(f'GPU path planning error: {e}')
            return self._plan_path_cpu(start_pose, goal_pose)

    def _gpu_astar_search(self, gpu_costmap, start, goal):
        """GPU-accelerated A* path search"""
        try:
            # Convert to cupy arrays
            height, width = gpu_costmap.shape
            start_pos = cp.array(start)
            goal_pos = cp.array(goal)

            # Initialize A* data structures on GPU
            open_set = cp.zeros((height, width), dtype=cp.bool_)
            closed_set = cp.zeros((height, width), dtype=cp.bool_)
            g_scores = cp.full((height, width), cp.inf, dtype=cp.float32)
            f_scores = cp.full((height, width), cp.inf, dtype=cp.float32)
            came_from = cp.full((height, width, 2), -1, dtype=cp.int32)

            # Initialize start position
            start_x, start_y = start
            g_scores[start_y, start_x] = 0
            f_scores[start_y, start_x] = self._gpu_heuristic(start, goal)

            # For practical implementation, we'll use a hybrid approach
            # where the actual A* search is done on CPU with GPU-optimized helpers
            return self._hybrid_astar_search(gpu_costmap.get(), start, goal)

        except Exception as e:
            self.get_logger().error(f'GPU A* search error: {e}')
            return []

    def _hybrid_astar_search(self, cpu_costmap, start, goal):
        """Hybrid CPU-GPU A* search using GPU-optimized helpers"""
        height, width = cpu_costmap.shape

        # Use CPU-based A* with GPU-optimized heuristics
        open_set = [(0, start)]  # (f_score, position)
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._cpu_heuristic(start, goal)}

        visited = set()

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            visited.add(current)

            # Check neighbors (8-connectivity)
            for neighbor in self._get_neighbors(current, width, height):
                if neighbor in visited:
                    continue

                # Check if neighbor is traversable
                cost = cpu_costmap[neighbor[1], neighbor[0]]
                if cost >= 253:  # Lethal obstacle
                    continue

                tentative_g_score = g_score[current] + self._gpu_distance(current, neighbor, cpu_costmap)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self._cpu_heuristic(neighbor, goal)

                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

            if len(visited) > self.max_iterations:
                break  # Prevent infinite loops

        return []  # No path found

    def _gpu_distance(self, pos1, pos2, costmap):
        """Calculate distance with GPU optimization for costmap lookup"""
        try:
            if self.gpu_available:
                # Use GPU-optimized distance calculation with costmap weighting
                base_distance = np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

                # Get cost at position (higher cost = higher distance penalty)
                cost = costmap[pos2[1], pos2[0]]
                cost_penalty = cost / 255.0  # Normalize cost

                return base_distance * (1 + cost_penalty)
            else:
                return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        except:
            return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def _cpu_heuristic(self, pos1, pos2):
        """CPU-based heuristic function"""
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def _smooth_path_gpu(self, path):
        """Smooth path using GPU acceleration"""
        if not path or len(path) < 3:
            return path

        try:
            if self.gpu_available:
                # Convert path to GPU array
                path_array = cp.array(path, dtype=cp.float32)

                # Apply GPU-parallelized path smoothing
                smoothed_path = self._gpu_path_smoothing(path_array)

                # Convert back to CPU
                return smoothed_path.get().tolist()
            else:
                # CPU-based smoothing fallback
                return self._cpu_path_smoothing(path)

        except Exception as e:
            self.get_logger().error(f'Path smoothing error: {e}')
            return path

    def _gpu_path_smoothing(self, gpu_path):
        """Apply GPU-parallelized path smoothing"""
        try:
            # Implement GPU-parallelized path smoothing
            # This could use CUDA kernels for:
            # - Moving average smoothing
            # - Spline fitting
            # - Collision checking along smoothed path

            # For this example, implement a simple GPU-based smoothing
            if len(gpu_path) > 2:
                # Use GPU to apply smoothing filter
                smoothed = cp.zeros_like(gpu_path)

                for i in range(len(gpu_path)):
                    start_idx = max(0, i - 1)
                    end_idx = min(len(gpu_path), i + 2)

                    # Compute moving average
                    smoothed[i] = cp.mean(gpu_path[start_idx:end_idx], axis=0)

                return smoothed
            else:
                return gpu_path

        except Exception as e:
            self.get_logger().error(f'GPU path smoothing error: {e}')
            return gpu_path

    def _get_neighbors(self, pos, width, height):
        """Get 8-connected neighbors"""
        neighbors = []
        x, y = pos

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    neighbors.append((nx, ny))

        return neighbors

    def _world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        if self.current_costmap is None:
            return None

        info = self.current_costmap['info']
        grid_x = int((x - info.origin.position.x) / info.resolution)
        grid_y = int((y - info.origin.position.y) / info.resolution)

        if 0 <= grid_x < info.width and 0 <= grid_y < info.height:
            return (grid_x, grid_y)
        else:
            return None

    def _grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        if self.current_costmap is None:
            return None

        info = self.current_costmap['info']
        world_x = grid_x * info.resolution + info.origin.position.x
        world_y = grid_y * info.resolution + info.origin.position.y

        return (world_x, world_y)

    def _create_path_message(self, path_points, header):
        """Create Path message from path points"""
        path_msg = Path()
        path_msg.header = header

        for point in path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header = header
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0

            path_msg.poses.append(pose_stamped)

        return path_msg

    def _visualize_path(self, path_points):
        """Visualize path in RViz"""
        marker_array = MarkerArray()

        # Path line marker
        line_marker = Marker()
        line_marker.header.frame_id = 'map'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'path'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.02  # Line width
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        for point in path_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.05  # Slightly above ground
            line_marker.points.append(p)

        marker_array.markers.append(line_marker)

        # Waypoint markers
        for i, point in enumerate(path_points):
            wp_marker = Marker()
            wp_marker.header.frame_id = 'map'
            wp_marker.header.stamp = self.get_clock().now().to_msg()
            wp_marker.ns = 'waypoints'
            wp_marker.id = i + 1
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD

            wp_marker.pose.position.x = point[0]
            wp_marker.pose.position.y = point[1]
            wp_marker.pose.position.z = 0.1
            wp_marker.pose.orientation.w = 1.0

            wp_marker.scale.x = 0.1
            wp_marker.scale.y = 0.1
            wp_marker.scale.z = 0.1

            wp_marker.color.r = 1.0
            wp_marker.color.g = 0.0
            wp_marker.color.b = 0.0
            wp_marker.color.a = 1.0

            marker_array.markers.append(wp_marker)

        self.visualization_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacPathPlanningNode()

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

## Isaac ROS Navigation Controller

### GPU-Accelerated Trajectory Control

```python
# navigation_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from builtin_interfaces.msg import Duration
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.distance import euclidean
import math

class IsaacNavigationControllerNode(Node):
    def __init__(self):
        super().__init__('isaac_navigation_controller')

        # Publishers and subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

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

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.controller_status_pub = self.create_publisher(Bool, '/controller_active', 10)
        self.progress_pub = self.create_publisher(Float32, '/navigation_progress', 10)

        # Controller parameters
        self.linear_gain = 1.0
        self.angular_gain = 2.0
        self.lookahead_distance = 0.5  # 50cm lookahead
        self.arrival_threshold = 0.2   # 20cm arrival threshold
        self.max_linear_vel = 0.5      # 0.5 m/s max
        self.max_angular_vel = 1.0     # 1.0 rad/s max
        self.min_approach_vel = 0.1    # 0.1 m/s minimum approach velocity

        # Controller state
        self.current_path = None
        self.current_pose = None
        self.path_index = 0
        self.controller_active = False
        self.obstacle_detected = False

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.get_logger().info('Isaac Navigation Controller initialized')

    def path_callback(self, msg):
        """Receive and follow navigation path"""
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return

        # Store path and reset controller
        self.current_path = msg.poses
        self.path_index = 0
        self.controller_active = True

        # Publish controller status
        status_msg = Bool()
        status_msg.data = True
        self.controller_status_pub.publish(status_msg)

        self.get_logger().info(f'Starting to follow path with {len(msg.poses)} waypoints')

    def odom_callback(self, msg):
        """Update current robot pose"""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        try:
            # Check for obstacles in front of robot
            front_scan_start = int(len(msg.ranges) / 2 - len(msg.ranges) / 8)  # Front quarter
            front_scan_end = int(len(msg.ranges) / 2 + len(msg.ranges) / 8)

            front_ranges = msg.ranges[front_scan_start:front_scan_end]
            valid_ranges = [r for r in front_ranges if msg.range_min <= r <= msg.range_max]

            if valid_ranges:
                min_range = min(valid_ranges)
                self.obstacle_detected = min_range < self.lookahead_distance + 0.3  # 30cm safety margin

                if self.obstacle_detected:
                    self.get_logger().warn(f'Obstacle detected at {min_range:.2f}m')
            else:
                self.obstacle_detected = False

        except Exception as e:
            self.get_logger().error(f'Scan processing error: {e}')

    def control_loop(self):
        """Main control loop"""
        if not self.controller_active or not self.current_path or not self.current_pose:
            if self.controller_active:
                # Publish stop command
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
            return

        try:
            # Check for obstacles
            if self.obstacle_detected:
                self._handle_obstacle_avoidance()
                return

            # Get current path point
            if self.path_index >= len(self.current_path):
                # Reached end of path
                self._reach_goal()
                return

            # Calculate control commands
            cmd_vel = self._calculate_control_commands()
            if cmd_vel:
                self.cmd_vel_pub.publish(cmd_vel)

                # Calculate and publish progress
                progress = self._calculate_navigation_progress()
                progress_msg = Float32()
                progress_msg.data = progress
                self.progress_pub.publish(progress_msg)

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')
            self._deactivate_controller()

    def _calculate_control_commands(self):
        """Calculate velocity commands using pure pursuit algorithm"""
        if self.path_index >= len(self.current_path):
            return None

        # Get current position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        # Find lookahead point
        lookahead_point = self._find_lookahead_point(current_x, current_y)

        if lookahead_point is None:
            return None

        # Calculate distance to lookahead point
        dx = lookahead_point[0] - current_x
        dy = lookahead_point[1] - current_y
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate angle to target
        current_yaw = self._get_yaw_from_pose(self.current_pose)
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - current_yaw

        # Normalize angle to [-π, π]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Pure pursuit control
        cmd_vel = Twist()

        # Linear velocity proportional to distance (with approach velocity control)
        approach_factor = max(self.min_approach_vel, min(1.0, distance / self.lookahead_distance))
        cmd_vel.linear.x = min(self.max_linear_vel * approach_factor, self.max_linear_vel)

        # Angular velocity proportional to angle error
        cmd_vel.angular.z = self.angular_gain * angle_diff
        cmd_vel.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, cmd_vel.angular.z))

        # Log control commands
        self.get_logger().debug(
            f'Control - Distance: {distance:.3f}, Angle diff: {angle_diff:.3f}, '
            f'Linear: {cmd_vel.linear.x:.3f}, Angular: {cmd_vel.angular.z:.3f}'
        )

        return cmd_vel

    def _find_lookahead_point(self, current_x, current_y):
        """Find lookahead point on path"""
        if not self.current_path or self.path_index >= len(self.current_path):
            return None

        # Start searching from current path index
        for i in range(self.path_index, len(self.current_path)):
            path_point = self.current_path[i]
            path_x = path_point.pose.position.x
            path_y = path_point.pose.position.y

            distance = math.sqrt((path_x - current_x)**2 + (path_y - current_y)**2)

            if distance >= self.lookahead_distance:
                # Update path index to the point we're tracking
                self.path_index = i
                return (path_x, path_y)

        # If no point is far enough, return the last point
        last_point = self.current_path[-1]
        return (last_point.pose.position.x, last_point.pose.position.y)

    def _handle_obstacle_avoidance(self):
        """Handle obstacle avoidance"""
        # Stop robot when obstacle is detected
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        self.get_logger().info('Obstacle avoidance activated - robot stopped')

    def _reach_goal(self):
        """Handle reaching the goal"""
        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        # Deactivate controller
        self.controller_active = False

        # Publish status
        status_msg = Bool()
        status_msg.data = False
        self.controller_status_pub.publish(status_msg)

        self.get_logger().info('Navigation goal reached')

    def _calculate_navigation_progress(self):
        """Calculate navigation progress percentage"""
        if not self.current_path or self.path_index >= len(self.current_path):
            return 100.0

        # Simple progress calculation based on path index
        progress = (self.path_index / len(self.current_path)) * 100.0
        return min(100.0, progress)

    def _get_yaw_from_pose(self, pose):
        """Extract yaw angle from pose quaternion"""
        import tf_transformations
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        return euler[2]  # Yaw angle

    def _deactivate_controller(self):
        """Deactivate navigation controller"""
        self.controller_active = False

        # Publish stop command
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        # Publish inactive status
        status_msg = Bool()
        status_msg.data = False
        self.controller_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacNavigationControllerNode()

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

## Isaac Navigation Performance Optimization

### GPU-Accelerated Navigation Optimization

```python
# navigation_optimization.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import cupy as cp
from collections import deque
import time

class IsaacNavigationOptimizerNode(Node):
    def __init__(self):
        super().__init__('isaac_navigation_optimizer')

        # Publishers for optimization metrics
        self.path_time_pub = self.create_publisher(Float32, '/path_planning_time', 10)
        self.control_time_pub = self.create_publisher(Float32, '/control_loop_time', 10)
        self.gpu_util_pub = self.create_publisher(Float32, '/gpu_utilization', 10)

        # Performance monitoring
        self.path_times = deque(maxlen=50)
        self.control_times = deque(maxlen=50)
        self.gpu_utilization = deque(maxlen=50)

        # Optimization parameters
        self.adaptive_resolution = True
        self.dynamic_lookahead = True
        self.current_resolution = 0.1  # meters
        self.current_lookahead = 0.5  # meters

        # Timer for optimization monitoring
        self.optimization_timer = self.create_timer(1.0, self.optimization_loop)

        self.get_logger().info('Isaac Navigation Optimizer initialized')

    def optimization_loop(self):
        """Monitor performance and apply optimizations"""
        try:
            # Calculate average performance metrics
            avg_path_time = np.mean(self.path_times) if self.path_times else 0.0
            avg_control_time = np.mean(self.control_times) if self.control_times else 0.0
            avg_gpu_util = np.mean(self.gpu_utilization) if self.gpu_utilization else 0.0

            # Apply adaptive optimizations based on performance
            self._apply_adaptive_optimizations(avg_path_time, avg_control_time, avg_gpu_util)

            # Publish metrics
            path_time_msg = Float32()
            path_time_msg.data = float(avg_path_time)
            self.path_time_pub.publish(path_time_msg)

            control_time_msg = Float32()
            control_time_msg.data = float(avg_control_time)
            self.control_time_pub.publish(control_time_msg)

            gpu_util_msg = Float32()
            gpu_util_msg.data = float(avg_gpu_util)
            self.gpu_util_pub.publish(gpu_util_msg)

            self.get_logger().info(
                f'Navigation Performance - Path: {avg_path_time:.3f}s, '
                f'Control: {avg_control_time:.3f}s, '
                f'GPU: {avg_gpu_util:.1f}%'
            )

        except Exception as e:
            self.get_logger().error(f'Optimization loop error: {e}')

    def _apply_adaptive_optimizations(self, avg_path_time, avg_control_time, avg_gpu_util):
        """Apply adaptive optimizations based on performance metrics"""
        target_rate = 20.0  # 20Hz navigation update rate
        target_path_time = 1.0 / target_rate  # 50ms target for path planning

        # Adjust resolution based on performance
        if avg_path_time > target_path_time * 0.8:  # 80% of budget
            # Performance is degrading, reduce resolution
            self.current_resolution = min(0.3, self.current_resolution * 1.1)  # Lower resolution
            self.get_logger().info(f'Reduced path planning resolution to {self.current_resolution:.2f}m')
        elif avg_path_time < target_path_time * 0.5:  # 50% of budget
            # Performance is good, can increase resolution
            self.current_resolution = max(0.05, self.current_resolution * 0.9)  # Higher resolution
            self.get_logger().info(f'Increased path planning resolution to {self.current_resolution:.2f}m')

        # Adjust lookahead distance based on speed and performance
        if avg_control_time > 0.02:  # 20ms control budget exceeded
            # Reduce lookahead for faster response
            self.current_lookahead = max(0.2, self.current_lookahead * 0.9)
            self.get_logger().info(f'Reduced lookahead distance to {self.current_lookahead:.2f}m')
        elif avg_control_time < 0.01:  # Control is fast
            # Increase lookahead for smoother navigation
            self.current_lookahead = min(1.0, self.current_lookahead * 1.1)
            self.get_logger().info(f'Increased lookahead distance to {self.current_lookahead:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = IsaacNavigationOptimizerNode()

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

## Isaac Navigation Best Practices

### Navigation System Best Practices

1. **Multi-layer Navigation**: Implement global and local planners
2. **Sensor Fusion**: Combine multiple sensor inputs for robust navigation
3. **Dynamic Replanning**: Continuously update plans based on new information
4. **Safety First**: Always prioritize obstacle avoidance and safety
5. **Performance Monitoring**: Continuously monitor and optimize performance
6. **Adaptive Parameters**: Adjust parameters based on environment and performance
7. **Failure Recovery**: Implement robust recovery behaviors
8. **Human Interaction**: Design for safe human-robot interaction

### GPU Optimization Guidelines

1. **Memory Management**: Use GPU memory pools and efficient allocation
2. **Batch Processing**: Process multiple queries together when possible
3. **Asynchronous Operations**: Overlap computation with data transfer
4. **Kernel Optimization**: Optimize CUDA kernels for your specific GPU
5. **Data Format**: Use appropriate data types (FP16 when precision allows)
6. **Resource Monitoring**: Continuously monitor GPU utilization and memory
7. **Fallback Systems**: Implement CPU-based fallbacks for critical functions
8. **Load Balancing**: Distribute computation across available GPU resources

## Troubleshooting Navigation Issues

### Common Navigation Problems and Solutions

**Issue**: Robot oscillates when following path
**Solution**: Adjust lookahead distance and controller gains

**Issue**: Path planning takes too long
**Solution**: Reduce map resolution, optimize costmap parameters

**Issue**: Robot gets stuck in local minima
**Solution**: Implement recovery behaviors, adjust obstacle inflation

**Issue**: Poor obstacle avoidance
**Solution**: Tune local planner parameters, improve sensor coverage

**Issue**: GPU memory exhaustion
**Solution**: Reduce batch sizes, optimize memory usage, use memory pools

## Next Steps

In the next chapter, we'll explore Isaac's manipulation capabilities, learning how to leverage GPU acceleration for robotic manipulation tasks including grasp planning, trajectory optimization, and dexterous manipulation for humanoid robots.