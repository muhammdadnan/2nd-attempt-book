# rclpy and AI Integration

In this chapter, we'll explore the Python client library for ROS 2 (rclpy) in depth and learn how to integrate AI capabilities with ROS 2 for humanoid robot applications. This combination enables robots to perceive, reason, and act intelligently in complex environments.

## Deep Dive into rclpy

rclpy is the Python client library for ROS 2 that provides Python bindings for the ROS 2 client library (rcl). It allows you to create ROS 2 nodes, publishers, subscribers, services, and actions in Python.

### Advanced Node Patterns

Here are some advanced patterns for creating robust ROS 2 nodes:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import threading
import time

class AdvancedHumanoidNode(Node):
    def __init__(self):
        super().__init__('advanced_humanoid_node')

        # Use custom QoS profiles
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Create multiple publishers with different QoS
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', sensor_qos)

        # Create subscribers with custom callbacks
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_commands',
            self.joint_command_callback,
            10
        )

        # Create timers with different frequencies
        self.high_freq_timer = self.create_timer(0.01, self.high_frequency_callback)  # 100 Hz
        self.low_freq_timer = self.create_timer(1.0, self.low_frequency_callback)     # 1 Hz

        # Threading example for blocking operations
        self.ai_thread = None
        self.ai_result = None

        self.get_logger().info('Advanced humanoid node initialized')

    def joint_command_callback(self, msg):
        # Process joint commands with validation
        if len(msg.position) != len(msg.name):
            self.get_logger().error('Joint position and name arrays have different lengths')
            return

        # Validate joint limits (simplified)
        for pos in msg.position:
            if abs(pos) > 3.14:  # Check for extreme values
                self.get_logger().warn(f'Joint position {pos} may be out of range')

        self.get_logger().info(f'Received command for {len(msg.name)} joints')

    def high_frequency_callback(self):
        # High-frequency control loop
        current_time = self.get_clock().now()
        self.get_logger().debug(f'High freq loop at {current_time}')

    def low_frequency_callback(self):
        # Low-frequency status updates
        status_msg = String()
        status_msg.data = f'Node running at {self.get_clock().now().nanoseconds}'
        self.status_pub.publish(status_msg)

    def start_ai_processing(self):
        """Start AI processing in a separate thread"""
        if self.ai_thread is None or not self.ai_thread.is_alive():
            self.ai_thread = threading.Thread(target=self.ai_processing_worker)
            self.ai_thread.start()

    def ai_processing_worker(self):
        """AI processing work in a separate thread"""
        # Simulate AI processing
        time.sleep(2)  # Simulate heavy computation
        self.ai_result = "AI processing completed"

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedHumanoidNode()

    try:
        # Start AI processing in background
        node.start_ai_processing()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Custom Message Handling

Working with custom message types and complex data structures:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
from cv_bridge import CvBridge

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Initialize CV bridge for image processing
        self.cv_bridge = CvBridge()

        # Create subscribers for camera data
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

        # Create publisher for processed data
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_object_pose', 10)

        self.camera_info = None
        self.get_logger().info('Perception node initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform AI-based object detection (simulated)
            detected_objects = self.detect_objects(cv_image)

            # Process results and publish poses
            for obj in detected_objects:
                pose_msg = self.create_pose_message(obj)
                self.pose_pub.publish(pose_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def detect_objects(self, image):
        # Simulate object detection
        # In a real implementation, this would use a neural network
        import random
        objects = []
        for i in range(random.randint(0, 3)):
            obj = {
                'class': 'object',
                'confidence': random.uniform(0.7, 0.99),
                'bbox': [random.randint(0, 300), random.randint(0, 300),
                        random.randint(50, 100), random.randint(50, 100)]
            }
            objects.append(obj)
        return objects

    def create_pose_message(self, obj):
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'camera_link'

        # Convert bounding box center to 3D pose (simplified)
        pose_msg.pose.position.x = obj['bbox'][0] + obj['bbox'][2] / 2
        pose_msg.pose.position.y = obj['bbox'][1] + obj['bbox'][3] / 2
        pose_msg.pose.position.z = 1.0  # Fixed depth for simulation

        return pose_msg
```

## AI Integration Patterns

### TensorFlow/Keras Integration

Integrating deep learning models with ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
import tensorflow as tf
import numpy as np
from cv_bridge import CvBridge

class AIPerceptionNode(Node):
    def __init__(self):
        super().__init__('ai_perception_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Load pre-trained model (in a real implementation)
        # self.model = tf.keras.models.load_model('path/to/model')
        # For simulation, create a dummy model
        self.model = self.create_dummy_model()

        # Create subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.result_pub = self.create_publisher(String, '/ai_detection_result', 10)

        self.get_logger().info('AI perception node initialized')

    def create_dummy_model(self):
        """Create a dummy model for simulation"""
        # In a real implementation, load your actual model
        return "dummy_model"

    def image_callback(self, msg):
        try:
            # Convert ROS Image to numpy array
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Preprocess image for the model
            input_image = self.preprocess_image(cv_image)

            # Run inference (simulated)
            results = self.run_inference(input_image)

            # Publish results
            result_msg = String()
            result_msg.data = f'Detection: {results["class"]} with confidence {results["confidence"]:.2f}'
            self.result_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f'Error in AI processing: {e}')

    def preprocess_image(self, image):
        """Preprocess image for model input"""
        # Resize and normalize image
        resized = tf.image.resize(image, [224, 224])
        normalized = tf.cast(resized, tf.float32) / 255.0
        return tf.expand_dims(normalized, 0)  # Add batch dimension

    def run_inference(self, input_image):
        """Run model inference (simulated)"""
        # In a real implementation, call self.model.predict(input_image)
        # For simulation, return dummy results
        import random
        classes = ['person', 'obstacle', 'object', 'floor']
        return {
            'class': random.choice(classes),
            'confidence': random.uniform(0.7, 0.99)
        }

def main(args=None):
    rclpy.init(args=args)
    node = AIPerceptionNode()

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

### OpenAI Integration

Integrating with external AI services like OpenAI:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import openai
import json
from cv_bridge import CvBridge

class AIBrainNode(Node):
    def __init__(self):
        super().__init__('ai_brain_node')

        # Initialize OpenAI client (requires API key)
        # openai.api_key = os.getenv('OPENAI_API_KEY')

        # For simulation, we'll mock the API calls
        self.simulation_mode = True

        # Create subscribers for different inputs
        self.vision_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.vision_callback,
            10
        )

        self.command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10
        )

        self.perception_sub = self.create_subscription(
            PoseStamped,
            '/detected_object_pose',
            self.perception_callback,
            10
        )

        # Create publishers for AI decisions
        self.action_pub = self.create_publisher(String, '/ai_action_plan', 10)
        self.response_pub = self.create_publisher(String, '/ai_response', 10)

        self.cv_bridge = CvBridge()
        self.detected_objects = []

        self.get_logger().info('AI brain node initialized')

    def vision_callback(self, msg):
        """Process visual input and make AI decisions"""
        try:
            # Convert image to format suitable for AI processing
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Analyze scene (simulated)
            scene_description = self.analyze_scene(cv_image)

            # Update internal state
            self.update_scene_knowledge(scene_description)

        except Exception as e:
            self.get_logger().error(f'Error processing vision: {e}')

    def command_callback(self, msg):
        """Process voice/command input and generate response"""
        try:
            command = msg.data
            self.get_logger().info(f'Received command: {command}')

            # Generate AI response
            response = self.process_command(command)

            # Publish response
            response_msg = String()
            response_msg.data = response
            self.response_pub.publish(response_msg)

            # If command requires action, publish action plan
            if self.should_take_action(command):
                action_plan = self.generate_action_plan(command)
                action_msg = String()
                action_msg.data = action_plan
                self.action_pub.publish(action_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')

    def perception_callback(self, msg):
        """Process perception data"""
        self.detected_objects.append(msg)
        # Limit to last 10 detections
        if len(self.detected_objects) > 10:
            self.detected_objects = self.detected_objects[-10:]

    def analyze_scene(self, image):
        """Analyze the scene using AI (simulated)"""
        # In a real implementation, this would use computer vision and/or OpenAI
        # For simulation, return dummy analysis
        return {
            "objects": ["person", "table", "chair"],
            "layout": "indoor environment with furniture",
            "navigation_clear": True
        }

    def process_command(self, command):
        """Process command using AI (simulated)"""
        if self.simulation_mode:
            # Simulate AI processing
            responses = {
                "hello": "Hello! How can I assist you today?",
                "how are you": "I'm functioning optimally, thank you for asking!",
                "what do you see": f"I see {', '.join(self.get_recent_objects())}",
                "walk forward": "Moving forward as requested",
                "stop": "Stopping movement"
            }
            return responses.get(command.lower(), "I understand the command and am processing it.")
        else:
            # Real OpenAI API call would go here
            try:
                response = openai.ChatCompletion.create(
                    model="gpt-3.5-turbo",
                    messages=[
                        {"role": "system", "content": "You are a helpful humanoid robot assistant."},
                        {"role": "user", "content": command}
                    ]
                )
                return response.choices[0].message.content
            except Exception as e:
                self.get_logger().error(f'OpenAI API error: {e}')
                return "I encountered an error processing your request."

    def should_take_action(self, command):
        """Determine if command requires physical action"""
        action_keywords = ["move", "walk", "go", "turn", "pick", "grasp", "navigate"]
        return any(keyword in command.lower() for keyword in action_keywords)

    def generate_action_plan(self, command):
        """Generate action plan based on command and current state"""
        # This would integrate with planning systems in a real implementation
        return f"EXECUTE: {command} - Using current navigation and manipulation capabilities"

    def get_recent_objects(self):
        """Get list of recently detected objects"""
        if not self.detected_objects:
            return ["no objects detected"]
        # Return simplified object information
        return ["object", "object", "person"]  # Simulated

def main(args=None):
    rclpy.init(args=args)
    node = AIBrainNode()

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

## Real-time AI Processing Considerations

When integrating AI with humanoid robots, real-time performance is critical:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float32
import time
import threading
from collections import deque

class RealTimeAINode(Node):
    def __init__(self):
        super().__init__('realtime_ai_node')

        # Create publishers for performance monitoring
        self.processing_time_pub = self.create_publisher(Float32, '/ai_processing_time', 10)

        # Buffer for sensor data to handle timing
        self.joint_buffer = deque(maxlen=10)
        self.image_buffer = deque(maxlen=5)  # Images are heavier, smaller buffer

        # Timer for AI processing at fixed rate
        self.ai_timer = self.create_timer(0.1, self.process_ai)  # 10 Hz AI processing

        # Track processing performance
        self.last_processing_time = 0.0

        self.get_logger().info('Real-time AI node initialized')

    def process_ai(self):
        """Process AI tasks with timing constraints"""
        start_time = time.time()

        try:
            # Process the most recent sensor data
            if self.joint_buffer:
                latest_joints = self.joint_buffer[-1]
                self.process_joint_data(latest_joints)

            if self.image_buffer:
                latest_image = self.image_buffer[-1]
                self.process_image_data(latest_image)

        except Exception as e:
            self.get_logger().error(f'Error in AI processing: {e}')

        # Calculate and publish processing time
        processing_time = time.time() - start_time
        self.last_processing_time = processing_time

        time_msg = Float32()
        time_msg.data = processing_time
        self.processing_time_pub.publish(time_msg)

        # Log warning if processing takes too long
        if processing_time > 0.05:  # 50ms threshold
            self.get_logger().warn(f'AI processing took {processing_time:.3f}s, exceeding real-time constraints')

    def process_joint_data(self, joint_state):
        """Process joint state data with AI"""
        # In a real implementation, this would run control algorithms
        pass

    def process_image_data(self, image_msg):
        """Process image data with AI"""
        # In a real implementation, this would run computer vision algorithms
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RealTimeAINode()

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

## Best Practices for AI Integration

1. **Separate AI processing from real-time control** to avoid timing issues
2. **Use appropriate QoS settings** for AI-related topics (may tolerate some message loss)
3. **Implement proper error handling** for AI model failures
4. **Monitor AI processing time** to ensure real-time constraints are met
5. **Cache AI model results** when appropriate to avoid redundant computation
6. **Use threading carefully** to avoid blocking the ROS main thread
7. **Validate AI outputs** before using them for robot control
8. **Implement graceful degradation** when AI components fail

## Performance Monitoring

Monitor AI performance in ROS 2:

```python
from rclpy.qos import QoSProfile, HistoryPolicy
from std_msgs.msg import Float32, Int32

class AIPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('ai_performance_monitor')

        # Create monitoring publishers
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
        self.inference_time_pub = self.create_publisher(Float32, '/ai/inference_time', qos)
        self.accuracy_pub = self.create_publisher(Float32, '/ai/accuracy', qos)
        self.confidence_pub = self.create_publisher(Float32, '/ai/confidence', qos)

        self.get_logger().info('AI performance monitor initialized')
```

## Next Steps

In the next chapter, we'll explore URDF (Unified Robot Description Format) fundamentals, which is essential for describing the physical structure of your humanoid robot. URDF works hand-in-hand with the ROS 2 nodes we've been creating to provide the robot's physical representation.