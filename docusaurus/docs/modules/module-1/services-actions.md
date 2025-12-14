# Services and Actions

In this chapter, we'll explore two more important communication patterns in ROS 2: services for synchronous request-response communication and actions for long-running tasks with feedback and goal management.

## Services

Services provide synchronous, one-to-one communication where a client sends a request and waits for a response from a server.

### Creating a Service Server

Here's how to create a service server:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState

class JointCalibrationService(Node):
    def __init__(self):
        super().__init__('joint_calibration_service')

        # Create service server
        self.srv = self.create_service(
            SetBool,
            'calibrate_joints',
            self.calibrate_joints_callback
        )

        # Publisher for joint commands during calibration
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)

        self.get_logger().info('Joint calibration service ready')

    def calibrate_joints_callback(self, request, response):
        self.get_logger().info(f'Calibration request: {request.data}')

        if request.data:  # If request is True, start calibration
            # Perform calibration sequence
            self.perform_calibration()
            response.success = True
            response.message = 'Joint calibration completed successfully'
        else:
            response.success = False
            response.message = 'Calibration request rejected'

        return response

    def perform_calibration(self):
        # Simulate calibration process
        self.get_logger().info('Starting joint calibration sequence...')

        # In a real implementation, this would send commands to move joints
        # to calibration positions and read encoders
        for i in range(10):
            self.get_logger().info(f'Calibrating... {i*10}% complete')
            # Simulate work
            from time import sleep
            sleep(0.1)

        self.get_logger().info('Calibration sequence completed')

def main(args=None):
    rclpy.init(args=args)
    node = JointCalibrationService()

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

### Creating a Service Client

Here's how to create a service client:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class CalibrationClient(Node):
    def __init__(self):
        super().__init__('calibration_client')
        self.cli = self.create_client(SetBool, 'calibrate_joints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = SetBool.Request()

    def send_request(self, calibrate=True):
        self.req.data = calibrate
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = CalibrationClient()

    response = client.send_request(True)
    client.get_logger().info(f'Response: {response.success}, {response.message}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Custom Service Types

For humanoid-specific services, you can create custom service definitions:

```python
# In your package's srv directory, create JointCalibration.srv:
# float64[] target_positions
# ---
# bool success
# string message
# float64[] actual_positions
```

## Actions

Actions are designed for long-running tasks that require feedback and the ability to cancel goals. They're perfect for humanoid robot tasks like walking, manipulation, or navigation.

### Creating an Action Server

Here's how to create an action server for humanoid walking:

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class WalkingActionServer(Node):
    def __init__(self):
        super().__init__('walking_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'walking_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        self.get_logger().info('Walking action server ready')

    def goal_callback(self, goal_request):
        """Accept or reject a goal."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.joint_names = [
            'left_hip_joint', 'right_hip_joint',
            'left_knee_joint', 'right_knee_joint',
            'left_ankle_joint', 'right_ankle_joint'
        ]

        trajectory = goal_handle.request.trajectory
        total_points = len(trajectory.points)

        for i, point in enumerate(trajectory.points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return FollowJointTrajectory.Result()

            # Publish trajectory point
            joint_trajectory = JointTrajectory()
            joint_trajectory.joint_names = trajectory.joint_names
            joint_trajectory.points = [point]

            self.joint_pub.publish(joint_trajectory)

            # Publish feedback
            feedback_msg.actual.positions = point.positions
            feedback_msg.actual.velocities = point.velocities
            goal_handle.publish_feedback(feedback_msg)

            # Calculate progress
            progress = float(i + 1) / total_points
            self.get_logger().info(f'Progress: {progress * 100:.1f}%')

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = 'Walking trajectory completed successfully'

        self.get_logger().info('Goal succeeded')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = WalkingActionServer()

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating an Action Client

Here's how to create an action client:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class WalkingActionClient(Node):
    def __init__(self):
        super().__init__('walking_action_client')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'walking_controller/follow_joint_trajectory'
        )

    def send_goal(self, joint_positions_list, joint_velocities_list=None):
        if joint_velocities_list is None:
            joint_velocities_list = [[0.0] * len(pos) for pos in joint_positions_list]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'left_hip_joint', 'right_hip_joint',
            'left_knee_joint', 'right_knee_joint',
            'left_ankle_joint', 'right_ankle_joint'
        ]

        # Create trajectory points
        for i, (pos, vel) in enumerate(zip(joint_positions_list, joint_velocities_list)):
            point = JointTrajectoryPoint()
            point.positions = pos
            point.velocities = vel
            point.time_from_start = Duration(sec=i, nanosec=0).to_msg()
            goal_msg.trajectory.points.append(point)

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(
            f'Received feedback: {feedback_msg.feedback.actual.positions}'
        )

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.error_string}')

def main(args=None):
    rclpy.init(args=args)
    action_client = WalkingActionClient()

    # Define a simple walking gait
    joint_positions = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Standing position
        [0.1, -0.1, 0.2, -0.2, 0.0, 0.0],  # Step forward left
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Return to center
        [-0.1, 0.1, -0.2, 0.2, 0.0, 0.0],  # Step forward right
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Return to center
    ]

    action_client.send_goal(joint_positions)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service vs Action: When to Use Which

### Use Services for:
- Simple, quick operations
- Configuration changes
- Calibration requests
- State queries
- Operations that should complete quickly

### Use Actions for:
- Long-running tasks (walking, manipulation, navigation)
- Tasks that provide feedback during execution
- Operations that might need to be canceled
- Trajectory execution
- Complex multi-step processes

## Humanoid-Specific Service Examples

Here are some common services for humanoid robots:

```python
# Balance control service
balance_srv = self.create_service(
    SetBool,
    'enable_balance_control',
    self.enable_balance_control_callback
)

# Emergency stop service
estop_srv = self.create_service(
    Trigger,
    'emergency_stop',
    self.emergency_stop_callback
)

# Robot pose service
pose_srv = self.create_service(
    GetRobotPose,
    'get_robot_pose',
    self.get_robot_pose_callback
)
```

## Humanoid-Specific Action Examples

Common actions for humanoid robots include:

```python
# Walking action
walking_action = ActionServer(
    self,
    WalkTrajectory,
    'walking_controller/walk_trajectory',
    execute_callback=self.execute_walk_callback
)

# Manipulation action
manipulation_action = ActionServer(
    self,
    ManipulationGoal,
    'manipulation_controller/grasp_object',
    execute_callback=self.execute_grasp_callback
)

# Whole-body motion action
motion_action = ActionServer(
    self,
    WholeBodyMotion,
    'motion_controller/execute_motion',
    execute_callback=self.execute_motion_callback
)
```

## Best Practices

1. **Use services for quick, discrete operations** and actions for long-running tasks
2. **Provide meaningful feedback** in action feedback messages
3. **Handle cancellation requests** properly in action servers
4. **Use appropriate timeouts** when calling services
5. **Implement proper error handling** for both services and actions
6. **Follow naming conventions** for service and action names
7. **Document expected behavior** for services and actions

## Testing Services and Actions

ROS 2 provides command-line tools for testing:

```bash
# Call a service from command line
ros2 service call /calibrate_joints std_srvs/srv/SetBool '{data: true}'

# Send an action goal from command line (if action interface is available)
ros2 action send_goal /walking_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {joint_names: ["joint1"], points: [{positions: [1.0], time_from_start: {sec: 1, nanosec: 0}}]}}'
```

## Next Steps

In the next chapter, we'll explore parameters and launch systems, which are essential for configuring and starting ROS 2 systems in a reproducible way. These tools are crucial for managing complex humanoid robot systems with many configurable components.