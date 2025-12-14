# Sensor Simulation

Sensor simulation is critical for developing perception and control systems for humanoid robots. In this chapter, we'll explore how to model various sensors in Gazebo, configure realistic sensor parameters, and validate sensor outputs for humanoid robot applications.

## Sensor Types Overview

### Inertial Measurement Units (IMU)
- **Purpose**: Measure angular velocity and linear acceleration
- **Use cases**: Balance control, orientation estimation, motion tracking
- **Key parameters**: Noise characteristics, update rate, bias

### Cameras
- **Purpose**: Visual perception, object recognition, navigation
- **Use cases**: Object detection, SLAM, visual servoing
- **Key parameters**: Resolution, field of view, noise, distortion

### LiDAR
- **Purpose**: 3D mapping, obstacle detection, navigation
- **Use cases**: Path planning, environment mapping, collision avoidance
- **Key parameters**: Range, resolution, field of view, noise

### Force/Torque Sensors
- **Purpose**: Measure contact forces and torques
- **Use cases**: Grasping, walking control, contact detection
- **Key parameters**: Range, sensitivity, noise

## IMU Sensor Configuration

### Basic IMU Setup

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>  <!-- 100 Hz - typical for humanoid balance -->
    <visualize>true</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>  <!-- 0.2 mrad/s (typical for high-grade IMU) -->
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>  <!-- 17 mg noise -->
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.05</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.05</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.05</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>

  <!-- ROS plugin to publish IMU data -->
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
    <topicName>imu/data</topicName>
    <bodyName>imu_link</bodyName>
    <updateRate>100</updateRate>
    <gaussianNoise>0.0004</gaussianNoise>
    <frameName>imu_link</frameName>
  </plugin>
</gazebo>
```

### IMU Placement for Humanoid Robots

For humanoid balance control, IMUs are typically placed:
- In the torso for body orientation
- In the feet for contact detection
- In the head for visual stabilization

```xml
<!-- Torso IMU for balance control -->
<gazebo reference="torso_imu">
  <sensor name="torso_imu" type="imu">
    <always_on>true</always_on>
    <update_rate>200</update_rate>  <!-- Higher rate for balance control -->
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><stddev>1e-4</stddev></x>
        <y><noise type="gaussian"><stddev>1e-4</stddev></y>
        <z><noise type="gaussian"><stddev>1e-4</stddev></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><stddev>1e-2</stddev></x>
        <y><noise type="gaussian"><stddev>1e-2</stddev></y>
        <z><noise type="gaussian"><stddev>1e-2</stddev></z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

## Camera Sensor Configuration

### RGB Camera Setup

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>  <!-- 30 FPS - standard for vision -->
    <camera name="head_camera">
      <horizontal_fov>1.0472</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
      <cameraName>head_camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
      <hackBaseline>0.07</hackBaseline>
      <distortionK1>0.0</distortionK1>
      <distortionK2>0.0</distortionK2>
      <distortionK3>0.0</distortionK3>
      <distortionT1>0.0</distortionT1>
      <distortionT2>0.0</distortionT2>
    </plugin>
  </sensor>
</gazebo>
```

### Stereo Camera Setup

For depth perception:

```xml
<!-- Left camera -->
<gazebo reference="left_camera_link">
  <sensor name="left_camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="left_cam">
      <horizontal_fov>1.0472</horizontal_fov>
      <image><width>640</width><height>480</height><format>R8G8B8</format></image>
      <clip><near>0.1</near><far>10.0</far></clip>
    </camera>
  </sensor>
</gazebo>

<!-- Right camera -->
<gazebo reference="right_camera_link">
  <sensor name="right_camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="right_cam">
      <horizontal_fov>1.0472</horizontal_fov>
      <image><width>640</width><height>480</height><format>R8G8B8</format></image>
      <clip><near>0.1</near><far>10.0</far></clip>
    </camera>
  </sensor>
</gazebo>
```

### Depth Camera Setup

For 3D perception:

```xml
<gazebo reference="depth_camera_link">
  <sensor name="depth_camera" type="depth">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="depth_cam">
      <horizontal_fov>1.0472</horizontal_fov>
      <image><width>640</width><height>480</height></image>
      <clip><near>0.1</near><far>10.0</far></clip>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
      <cameraName>depth_camera</cameraName>
      <imageTopicName>rgb/image_raw</imageTopicName>
      <depthImageTopicName>depth/image_raw</depthImageTopicName>
      <pointCloudTopicName>depth/points</pointCloudTopicName>
      <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
      <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
      <frameName>depth_camera_link</frameName>
      <baseline>0.1</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <pointCloudCutoff>0.1</pointCloudCutoff>
      <pointCloudCutoffMax>10.0</pointCloudCutoffMax>
    </plugin>
  </sensor>
</gazebo>
```

## LiDAR Sensor Configuration

### 2D LiDAR (Hokuyo-style)

```xml
<gazebo reference="laser_link">
  <sensor name="laser" type="ray">
    <always_on>true</always_on>
    <update_rate>40</update_rate>  <!-- 40 Hz - typical for navigation -->
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>  <!-- 0.5 degree resolution -->
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>  <!-- -90 degrees -->
          <max_angle>1.570796</max_angle>   <!-- 90 degrees -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>  <!-- 10m max range -->
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="laser_controller" filename="libgazebo_ros_laser.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
      <topicName>scan</topicName>
      <frameName>laser_link</frameName>
      <minAngle>-1.570796</minAngle>
      <maxAngle>1.570796</maxAngle>
      <gaussianNoise>0.005</gaussianNoise>  <!-- 5mm noise -->
    </plugin>
  </sensor>
</gazebo>
```

### 3D LiDAR (Velodyne-style)

```xml
<gazebo reference="velodyne_link">
  <sensor name="velodyne" type="ray">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>32</samples>
          <resolution>1</resolution>
          <min_angle>-0.261799</min_angle>  <!-- -15 degrees -->
          <max_angle>0.261799</max_angle>   <!-- 15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.2</min>
        <max>100.0</max>
        <resolution>0.001</resolution>
      </range>
    </ray>
    <plugin name="velodyne_controller" filename="libgazebo_ros_velodyne_laser.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
      <topicName>velodyne_points</topicName>
      <frameName>velodyne_link</frameName>
      <min_range>0.2</min_range>
      <max_range>100.0</max_range>
      <gaussianNoise>0.01</gaussianNoise>
    </plugin>
  </sensor>
</gazebo>
```

## Force/Torque Sensor Configuration

### Joint Force/Torque Sensors

```xml
<gazebo>
  <plugin name="ft_sensors" filename="libgazebo_ros_ft_sensor.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
    <jointName>left_ankle_joint</jointName>
    <topicName>left_ankle_ft</topicName>
    <updateRate>100</updateRate>
    <gaussianNoise>0.1</gaussianNoise>
  </plugin>
</gazebo>
```

### Custom Force/Torque Sensor Plugin

```xml
<gazebo>
  <plugin name="custom_ft_sensor" filename="libcustom_ft_sensor.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
    <parentLink>left_foot</parentLink>
    <childLink>left_foot_contact</childLink>
    <topicName>left_foot_contact_wrench</topicName>
    <updateRate>1000</updateRate>
    <forceNoise>1.0</forceNoise>
    <torqueNoise>0.1</torqueNoise>
  </plugin>
</gazebo>
```

## Sensor Fusion and Coordination

### Multi-Sensor Integration

Coordinate multiple sensors for comprehensive perception:

```xml
<!-- Head-mounted sensors for perception -->
<gazebo reference="head">
  <sensor name="head_imu" type="imu">
    <!-- IMU configuration -->
  </sensor>
  <sensor name="head_camera" type="camera">
    <!-- Camera configuration -->
  </sensor>
  <sensor name="head_laser" type="ray">
    <!-- 2D laser configuration -->
  </sensor>
</gazebo>
```

### Sensor Calibration

Simulate calibration procedures:

```xml
<!-- Simulate extrinsic calibration between sensors -->
<gazebo reference="camera_link">
  <sensor name="calibrated_camera" type="camera">
    <camera name="calib_cam">
      <distortion>
        <k1>0.1</k1>  <!-- Radial distortion -->
        <k2>-0.2</k2>
        <k3>0.05</k3>
        <p1>0.001</p1>  <!-- Tangential distortion -->
        <p2>-0.002</p2>
      </distortion>
    </camera>
  </sensor>
</gazebo>
```

## Realistic Sensor Modeling

### Noise Modeling

Model realistic sensor noise characteristics:

```xml
<!-- Camera with realistic noise -->
<camera name="realistic_camera">
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.01</stddev>  <!-- 1% of signal -->
  </noise>
</camera>

<!-- IMU with bias and drift -->
<imu>
  <angular_velocity>
    <noise type="gaussian">
      <mean>0.0</mean>
      <stddev>2e-4</stddev>
      <bias_mean>0.0</bias_mean>
      <bias_stddev>0.01</bias_stddev>
      <dynamic_bias_stddev>1e-6</dynamic_bias_stddev>  <!-- Random walk -->
      <dynamic_bias_correlation_time>100</dynamic_bias_correlation_time>
    </noise>
  </angular_velocity>
</imu>
```

### Environmental Effects

Simulate environmental conditions:

```xml
<!-- Weather effects on sensors -->
<gazebo>
  <sensor name="weather_affected_camera" type="camera">
    <camera name="weather_cam">
      <distortion>
        <k1>0.01</k1>  <!-- Fog/humidity effects -->
      </distortion>
    </camera>
  </sensor>
</gazebo>
```

## Sensor Validation

### Sensor Output Validation

Validate sensor outputs against expected behavior:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image, LaserScan
from std_msgs.msg import Float32

class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        # Subscribe to sensor data
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers for validation metrics
        self.imu_valid_pub = self.create_publisher(Float32, '/validation/imu_validity', 10)
        self.scan_valid_pub = self.create_publisher(Float32, '/validation/scan_validity', 10)

    def imu_callback(self, msg):
        # Validate IMU data ranges and plausibility
        validity_score = self.validate_imu_data(msg)
        score_msg = Float32()
        score_msg.data = validity_score
        self.imu_valid_pub.publish(score_msg)

    def validate_imu_data(self, imu_msg):
        # Check if values are within expected ranges
        linear_acc_valid = all(abs(val) < 20.0 for val in [imu_msg.linear_acceleration.x,
                                                           imu_msg.linear_acceleration.y,
                                                           imu_msg.linear_acceleration.z])
        angular_vel_valid = all(abs(val) < 10.0 for val in [imu_msg.angular_velocity.x,
                                                            imu_msg.angular_velocity.y,
                                                            imu_msg.angular_velocity.z])

        return float(linear_acc_valid and angular_vel_valid)

def main(args=None):
    rclpy.init(args=args)
    validator = SensorValidator()
    rclpy.spin(validator)
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Ground Truth Comparison

Compare simulated sensors with ground truth:

```python
class GroundTruthComparator(Node):
    def __init__(self):
        super().__init__('ground_truth_comparator')

        # Subscribe to sensor and ground truth
        self.sensor_sub = self.create_subscription(Imu, '/imu/data', self.sensor_callback, 10)
        self.truth_sub = self.create_subscription(Imu, '/ground_truth/imu', self.truth_callback, 10)

        self.error_pub = self.create_publisher(Float32, '/sensor_error', 10)

    def sensor_callback(self, sensor_msg):
        # Compare with stored ground truth
        if hasattr(self, 'truth_msg'):
            error = self.calculate_error(sensor_msg, self.truth_msg)
            error_msg = Float32()
            error_msg.data = error
            self.error_pub.publish(error_msg)

def calculate_error(self, sensor, truth):
    # Calculate RMSE or other error metrics
    acc_error = ((sensor.linear_acceleration.x - truth.linear_acceleration.x)**2 +
                 (sensor.linear_acceleration.y - truth.linear_acceleration.y)**2 +
                 (sensor.linear_acceleration.z - truth.linear_acceleration.z)**2)**0.5

    gyro_error = ((sensor.angular_velocity.x - truth.angular_velocity.x)**2 +
                  (sensor.angular_velocity.y - truth.angular_velocity.y)**2 +
                  (sensor.angular_velocity.z - truth.angular_velocity.z)**2)**0.5

    return (acc_error + gyro_error) / 2.0
```

## Performance Optimization

### Sensor Update Rate Management

Balance sensor accuracy with performance:

```xml
<!-- High-rate sensors for critical functions -->
<sensor name="balance_imu" type="imu">
  <update_rate>200</update_rate>  <!-- Critical for balance -->
</sensor>

<!-- Standard-rate sensors for perception -->
<sensor name="navigation_camera" type="camera">
  <update_rate>30</update_rate>   <!-- Standard for vision -->
</sensor>

<!-- Low-rate sensors for mapping -->
<sensor name="mapping_lidar" type="ray">
  <update_rate>10</update_rate>   <!-- Lower for performance -->
</sensor>
```

### Selective Sensor Activation

Activate sensors only when needed:

```python
class SensorManager(Node):
    def __init__(self):
        super().__init__('sensor_manager')

        # Service to enable/disable sensors
        self.enable_service = self.create_service(
            SetBool, 'enable_navigation_sensors', self.enable_sensors_callback)

    def enable_sensors_callback(self, request, response):
        if request.data:
            # Enable navigation sensors
            self.enable_navigation_sensors()
        else:
            # Disable to save resources
            self.disable_navigation_sensors()

        response.success = True
        response.message = f"Sensors {'enabled' if request.data else 'disabled'}"
        return response
```

## Advanced Sensor Features

### Custom Sensor Plugins

Create custom sensors for specific applications:

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/physics.hh>

class CustomHumanoidSensor : public gazebo::SensorPlugin
{
public:
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    // Cast to RaySensor
    this->parentSensor =
      std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(_sensor);

    if (!this->parentSensor)
    {
      gzerr << "CustomHumanoidSensor requires a RaySensor.\n";
      return;
    }

    // Connect to sensor update event
    this->updateConnection = this->parentSensor->ConnectUpdated(
        std::bind(&CustomHumanoidSensor::OnUpdate, this));

    // Initialize ROS publisher
    // ... ROS setup code ...
  }

  void OnUpdate()
  {
    // Custom processing of sensor data
    auto ranges = this->parentSensor->Ranges();
    // Process ranges for humanoid-specific use case
  }

private:
  gazebo::sensors::RaySensorPtr parentSensor;
  gazebo::event::ConnectionPtr updateConnection;
};

GZ_REGISTER_SENSOR_PLUGIN(CustomHumanoidSensor)
```

## Sensor Integration with AI Systems

Connect sensors to AI perception systems:

```python
class AISensorInterface(Node):
    def __init__(self):
        super().__init__('ai_sensor_interface')

        # Subscribe to multiple sensors
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.process_vision, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.process_imu, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.process_lidar, 10)

        # AI model integration
        # self.ai_model = load_model('perception_model.pt')

    def process_vision(self, image_msg):
        # Process image with AI model
        # results = self.ai_model.process_image(image_msg)
        pass

    def process_fused_data(self):
        # Combine sensor data for AI processing
        pass
```

## Best Practices

1. **Match real sensor specifications**: Use actual sensor parameters when available
2. **Validate sensor outputs**: Compare with expected ranges and behaviors
3. **Consider computational cost**: Balance sensor fidelity with performance
4. **Model sensor limitations**: Include noise, bias, and failure modes
5. **Coordinate sensor placement**: Ensure sensors work together effectively
6. **Test in various conditions**: Validate performance under different scenarios

## Next Steps

In the next chapter, we'll explore how to integrate these simulated sensors with ROS 2, creating the communication bridge that allows your humanoid robot's perception and control systems to access sensor data from simulation.