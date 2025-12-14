# Unity Robotics

Unity provides high-fidelity rendering and simulation capabilities that are ideal for humanoid robot development, particularly for perception training and photorealistic visualization. In this chapter, we'll explore how to integrate Unity with ROS 2 for advanced humanoid robot simulation.

## Unity Robotics Overview

Unity's robotics capabilities include:

- **Photorealistic rendering**: High-quality visual simulation for perception training
- **Physics simulation**: Realistic physics with configurable parameters
- **Flexible environment creation**: Easy creation of complex 3D environments
- **Cross-platform support**: Deploy to various platforms and devices
- **Asset ecosystem**: Rich library of models, materials, and environments

## Unity Robotics Setup

### Installing Unity Robotics Hub

1. **Install Unity Hub**: Download from unity.com
2. **Install Unity Editor**: Version 2022.3 LTS or later
3. **Install Robotics Hub**: Through Unity Package Manager
4. **Install ROS TCP Connector**: For ROS 2 communication

### Unity Project Structure for Robotics

```
UnityRoboticsProject/
├── Assets/
│   ├── Scenes/                 # Unity scenes
│   ├── Scripts/                # C# scripts for robot control
│   ├── Models/                 # 3D models
│   ├── Materials/              # Material definitions
│   ├── Prefabs/                # Reusable robot components
│   └── Plugins/                # ROS communication plugins
├── Packages/
│   ├── manifest.json          # Package dependencies
│   └── packages-lock.json
└── ProjectSettings/
```

## Creating a Humanoid Robot in Unity

### Basic Robot Setup

```csharp
// RobotController.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class RobotController : MonoBehaviour
{
    [Header("Robot Configuration")]
    public string robotName = "humanoid_robot";
    public float controlFrequency = 100f; // Hz

    [Header("Joint Configuration")]
    public Transform headJoint;
    public Transform leftShoulderJoint;
    public Transform rightShoulderJoint;
    public Transform leftElbowJoint;
    public Transform rightElbowJoint;
    public Transform leftHipJoint;
    public Transform rightHipJoint;
    public Transform leftKneeJoint;
    public Transform rightKneeJoint;

    private ROSConnection ros;
    private float controlTimer;

    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.instance;

        // Subscribe to joint commands
        ros.Subscribe<sensor_msgs.JointState>("/joint_commands", JointCommandCallback);

        // Initialize control timer
        controlTimer = 0f;
    }

    void Update()
    {
        // Control frequency management
        controlTimer += Time.deltaTime;
        if (controlTimer >= 1f / controlFrequency)
        {
            controlTimer = 0f;
            UpdateRobotState();
        }
    }

    void JointCommandCallback(sensor_msgs.JointState jointState)
    {
        // Update joint positions based on ROS message
        if (jointState.name.Length != jointState.position.Length)
        {
            Debug.LogError("Joint names and positions array length mismatch");
            return;
        }

        for (int i = 0; i < jointState.name.Length; i++)
        {
            string jointName = jointState.name[i];
            float position = jointState.position[i];

            switch (jointName)
            {
                case "head_joint":
                    if (headJoint != null)
                        headJoint.localRotation = Quaternion.Euler(0, position * Mathf.Rad2Deg, 0);
                    break;
                case "left_shoulder_joint":
                    if (leftShoulderJoint != null)
                        leftShoulderJoint.localRotation = Quaternion.Euler(0, 0, position * Mathf.Rad2Deg);
                    break;
                case "right_shoulder_joint":
                    if (rightShoulderJoint != null)
                        rightShoulderJoint.localRotation = Quaternion.Euler(0, 0, position * Mathf.Rad2Deg);
                    break;
                // Add other joints as needed
            }
        }
    }

    void UpdateRobotState()
    {
        // Publish current joint states to ROS
        var jointState = new sensor_msgs.JointState();
        jointState.header = new std_msgs.Header();
        jointState.header.stamp = new builtin_interfaces.Time();
        jointState.name = new string[] {
            "head_joint", "left_shoulder_joint", "right_shoulder_joint"
            // Add all joint names
        };
        jointState.position = new double[] {
            headJoint.localEulerAngles.y * Mathf.Deg2Rad,
            leftShoulderJoint.localEulerAngles.z * Mathf.Deg2Rad,
            rightShoulderJoint.localEulerAngles.z * Mathf.Deg2Rad
            // Add all joint positions
        };

        ros.Publish("/joint_states", jointState);
    }
}
```

### Robot Prefab Structure

Create a humanoid robot prefab with proper hierarchy:

```
HumanoidRobot (RobotController)
├── BaseLink
│   ├── Torso
│   │   ├── Head
│   │   │   └── Camera
│   │   ├── LeftShoulder
│   │   │   └── LeftElbow
│   │   │       └── LeftHand
│   │   ├── RightShoulder
│   │   │   └── RightElbow
│   │   │       └── RightHand
│   │   ├── LeftHip
│   │   │   └── LeftKnee
│   │   │       └── LeftFoot
│   │   └── RightHip
│   │       └── RightKnee
│   │           └── RightFoot
└── Sensors
    ├── IMU
    ├── HeadCamera
    └── OtherSensors
```

## ROS-Unity Bridge Implementation

### Custom ROS Message Handling

```csharp
// SensorPublisher.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class SensorPublisher : MonoBehaviour
{
    [Header("Sensor Configuration")]
    public Camera headCamera;
    public Transform imuTransform;
    public float publishFrequency = 30f; // Hz

    private ROSConnection ros;
    private float publishTimer;
    private int sequenceNumber = 0;

    void Start()
    {
        ros = ROSConnection.instance;
        publishTimer = 0f;
    }

    void Update()
    {
        publishTimer += Time.deltaTime;
        if (publishTimer >= 1f / publishFrequency)
        {
            publishTimer = 0f;
            PublishSensorData();
        }
    }

    void PublishSensorData()
    {
        // Publish camera data
        if (headCamera != null)
        {
            PublishCameraData();
        }

        // Publish IMU data
        PublishImuData();

        // Publish other sensor data as needed
    }

    void PublishCameraData()
    {
        // In a real implementation, you would capture and send camera data
        // This is a simplified example
        var cameraInfo = new sensor_msgs.CameraInfo();
        cameraInfo.header = new std_msgs.Header();
        cameraInfo.header.stamp = new builtin_interfaces.Time();
        cameraInfo.header.frame_id = "head_camera";
        cameraInfo.width = (uint)headCamera.pixelWidth;
        cameraInfo.height = (uint)headCamera.pixelHeight;

        // Publish camera info
        ros.Publish("/camera_info", cameraInfo);
    }

    void PublishImuData()
    {
        var imuMsg = new sensor_msgs.Imu();
        imuMsg.header = new std_msgs.Header();
        imuMsg.header.stamp = new builtin_interfaces.Time();
        imuMsg.header.frame_id = "imu_link";

        // Set orientation (simplified)
        imuMsg.orientation.x = imuTransform.rotation.x;
        imuMsg.orientation.y = imuTransform.rotation.y;
        imuMsg.orientation.z = imuTransform.rotation.z;
        imuMsg.orientation.w = imuTransform.rotation.w;

        // Set angular velocity (simplified)
        imuMsg.angular_velocity.x = 0.0f;
        imuMsg.angular_velocity.y = 0.0f;
        imuMsg.angular_velocity.z = 0.0f;

        // Set linear acceleration (simplified)
        imuMsg.linear_acceleration.x = Physics.gravity.x;
        imuMsg.linear_acceleration.y = Physics.gravity.y;
        imuMsg.linear_acceleration.z = Physics.gravity.z;

        ros.Publish("/imu/data", imuMsg);
    }
}
```

### Custom ROS Service Implementation

```csharp
// RobotService.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RobotService : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.instance;

        // Register service callback
        ros.RegisterServiceCallback("robot_control_service",
            (SetBoolRequest request) => HandleRobotControlService(request));
    }

    SetBoolResponse HandleRobotControlService(SetBoolRequest request)
    {
        SetBoolResponse response = new SetBoolResponse();

        if (request.data)
        {
            // Enable robot control
            EnableRobotControl();
            response.success = true;
            response.message = "Robot control enabled";
        }
        else
        {
            // Disable robot control
            DisableRobotControl();
            response.success = true;
            response.message = "Robot control disabled";
        }

        return response;
    }

    void EnableRobotControl()
    {
        Debug.Log("Robot control enabled");
        // Enable robot control logic
    }

    void DisableRobotControl()
    {
        Debug.Log("Robot control disabled");
        // Disable robot control logic
    }
}
```

## Unity Scene Setup for Humanoid Robotics

### Environment Creation

```csharp
// EnvironmentManager.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class EnvironmentManager : MonoBehaviour
{
    [Header("Environment Configuration")]
    public float gravity = -9.81f;
    public PhysicMaterial defaultMaterial;

    [Header("Spawn Points")]
    public Transform[] spawnPoints;

    void Start()
    {
        // Configure physics
        Physics.gravity = new Vector3(0, gravity, 0);

        // Set default physics material if provided
        if (defaultMaterial != null)
        {
            Physics.defaultMaterial = defaultMaterial;
        }

        // Initialize ROS connection
        ROSConnection.instance = GetComponent<ROSConnection>();
    }

    public Vector3 GetRandomSpawnPoint()
    {
        if (spawnPoints.Length > 0)
        {
            int randomIndex = Random.Range(0, spawnPoints.Length);
            return spawnPoints[randomIndex].position;
        }
        return Vector3.zero;
    }
}
```

### Lighting and Rendering Configuration

```csharp
// RenderingManager.cs
using UnityEngine;
using UnityEngine.Rendering;

public class RenderingManager : MonoBehaviour
{
    [Header("Rendering Configuration")]
    public bool useHDRP = false;
    public bool enableRealtimeGI = true;
    public bool enableReflectionProbes = true;

    [Header("Performance Settings")]
    public int targetFrameRate = 60;

    void Start()
    {
        // Set target frame rate
        Application.targetFrameRate = targetFrameRate;

        // Configure rendering settings based on requirements
        ConfigureRendering();
    }

    void ConfigureRendering()
    {
        if (useHDRP)
        {
            // HDRP-specific configurations
            ConfigureHDRP();
        }
        else
        {
            // Built-in render pipeline configurations
            ConfigureBuiltInPipeline();
        }

        // Configure global illumination
        if (enableRealtimeGI)
        {
            Lightmapping.giWorkflowMode = Lightmapping.GIWorkflowMode.Iterative;
        }

        // Configure reflection probes
        if (enableReflectionProbes)
        {
            ConfigureReflectionProbes();
        }
    }

    void ConfigureHDRP()
    {
        // HDRP specific settings
        // This would include volume configuration, lighting settings, etc.
    }

    void ConfigureBuiltInPipeline()
    {
        // Built-in pipeline specific settings
    }

    void ConfigureReflectionProbes()
    {
        // Find and configure reflection probes in the scene
        ReflectionProbe[] probes = FindObjectsOfType<ReflectionProbe>();
        foreach (var probe in probes)
        {
            probe.mode = UnityEngine.Rendering.ReflectionProbeMode.Realtime;
            probe.refreshMode = UnityEngine.Rendering.ReflectionProbeRefreshMode.ViaScripting;
        }
    }
}
```

## Perception Training in Unity

### Synthetic Data Generation

```csharp
// SyntheticDataGenerator.cs
using UnityEngine;
using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class SyntheticDataGenerator : MonoBehaviour
{
    [Header("Data Generation")]
    public Camera dataCamera;
    public bool generateDepth = true;
    public bool generateSegmentation = true;
    public int datasetSize = 1000;
    public float collectionInterval = 0.1f; // seconds

    [Header("Domain Randomization")]
    public Material[] randomMaterials;
    public Color[] randomColors;
    public Light[] lightsToRandomize;

    private int dataCounter = 0;
    private bool isCollecting = false;

    void Start()
    {
        if (dataCamera == null)
        {
            dataCamera = Camera.main;
        }
    }

    public void StartDataCollection()
    {
        if (!isCollecting)
        {
            isCollecting = true;
            StartCoroutine(CollectData());
        }
    }

    public void StopDataCollection()
    {
        isCollecting = false;
    }

    IEnumerator CollectData()
    {
        while (isCollecting && dataCounter < datasetSize)
        {
            // Apply domain randomization
            ApplyDomainRandomization();

            // Capture data
            CaptureAndSaveData();

            dataCounter++;
            yield return new WaitForSeconds(collectionInterval);
        }

        Debug.Log($"Data collection completed. Collected {dataCounter} samples.");
    }

    void ApplyDomainRandomization()
    {
        // Randomize materials
        if (randomMaterials.Length > 0)
        {
            Renderer[] renderers = FindObjectsOfType<Renderer>();
            foreach (var renderer in renderers)
            {
                Material randomMat = randomMaterials[Random.Range(0, randomMaterials.Length)];
                renderer.material = randomMat;
            }
        }

        // Randomize lighting
        if (lightsToRandomize.Length > 0 && randomColors.Length > 0)
        {
            foreach (var light in lightsToRandomize)
            {
                Color randomColor = randomColors[Random.Range(0, randomColors.Length)];
                light.color = randomColor;
                light.intensity = Random.Range(0.5f, 2.0f);
            }
        }
    }

    void CaptureAndSaveData()
    {
        // Capture RGB image
        Texture2D rgbImage = CaptureCameraImage(dataCamera);

        // Capture depth if enabled
        if (generateDepth)
        {
            Texture2D depthImage = CaptureDepthImage(dataCamera);
            // Save depth image
        }

        // Capture segmentation if enabled
        if (generateSegmentation)
        {
            Texture2D segImage = CaptureSegmentationImage();
            // Save segmentation image
        }

        // Publish to ROS for further processing
        PublishToROS(rgbImage);
    }

    Texture2D CaptureCameraImage(Camera cam)
    {
        // Capture camera image logic
        return null; // Placeholder
    }

    Texture2D CaptureDepthImage(Camera cam)
    {
        // Capture depth image logic
        return null; // Placeholder
    }

    Texture2D CaptureSegmentationImage()
    {
        // Capture segmentation image logic
        return null; // Placeholder
    }

    void PublishToROS(Texture2D image)
    {
        // Convert and publish image to ROS
        // This would involve converting Unity texture to ROS Image message
    }
}
```

## Integration with ROS 2

### ROS 2 Message Types in Unity

```csharp
// UnityROS2Bridge.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Actionlib;

public class UnityROS2Bridge : MonoBehaviour
{
    [Header("ROS 2 Integration")]
    public string robotNamespace = "/humanoid_robot";
    public float rosUpdateRate = 50f;

    private ROSConnection ros;
    private float updateTimer;

    void Start()
    {
        ros = ROSConnection.instance;
        updateTimer = 0f;

        // Subscribe to ROS 2 topics
        SubscribeToTopics();

        // Advertise services
        AdvertiseServices();
    }

    void Update()
    {
        updateTimer += Time.deltaTime;
        if (updateTimer >= 1f / rosUpdateRate)
        {
            updateTimer = 0f;
            UpdateROSConnection();
        }
    }

    void SubscribeToTopics()
    {
        // Subscribe to various ROS topics
        ros.Subscribe<geometry_msgs.Twist>(robotNamespace + "/cmd_vel", CmdVelCallback);
        ros.Subscribe<nav_msgs.Odometry>(robotNamespace + "/odom", OdometryCallback);
        ros.Subscribe<sensor_msgs.JointState>(robotNamespace + "/joint_states", JointStateCallback);
    }

    void AdvertiseServices()
    {
        // Register services if needed
    }

    void CmdVelCallback(geometry_msgs.Twist cmdVel)
    {
        // Handle velocity commands from ROS
        Vector3 linear = new Vector3((float)cmdVel.linear.x, (float)cmdVel.linear.y, (float)cmdVel.linear.z);
        Vector3 angular = new Vector3((float)cmdVel.angular.x, (float)cmdVel.angular.y, (float)cmdVel.angular.z);

        // Apply movement to robot
        ApplyVelocityCommand(linear, angular);
    }

    void OdometryCallback(nav_msgs.Odometry odom)
    {
        // Update robot position based on odometry
        Vector3 position = new Vector3((float)odom.pose.pose.position.x,
                                      (float)odom.pose.pose.position.y,
                                      (float)odom.pose.pose.position.z);
        Quaternion rotation = new Quaternion((float)odom.pose.pose.orientation.x,
                                           (float)odom.pose.pose.orientation.y,
                                           (float)odom.pose.pose.orientation.z,
                                           (float)odom.pose.pose.orientation.w);

        // Update robot transform
        transform.position = position;
        transform.rotation = rotation;
    }

    void JointStateCallback(sensor_msgs.JointState jointState)
    {
        // Update joint positions from ROS
        // Implementation similar to RobotController
    }

    void ApplyVelocityCommand(Vector3 linear, Vector3 angular)
    {
        // Apply the velocity command to the Unity robot
        // This would involve physics simulation or direct transform manipulation
    }

    void UpdateROSConnection()
    {
        // Publish current robot state to ROS
        PublishRobotState();
    }

    void PublishRobotState()
    {
        // Publish current state including position, orientation, joint states, etc.
    }
}
```

## Performance Optimization

### Unity-Specific Optimizations

```csharp
// PerformanceOptimizer.cs
using UnityEngine;
using System.Collections.Generic;

public class PerformanceOptimizer : MonoBehaviour
{
    [Header("Performance Settings")]
    public int targetFrameRate = 60;
    public float physicsUpdateRate = 50f; // Hz
    public bool enableLOD = true;
    public bool enableOcclusionCulling = true;

    [Header("Robot Complexity")]
    public int maxPolygonCount = 10000;
    public int maxJointCount = 30;

    private List<Renderer> robotRenderers = new List<Renderer>();
    private List<Collider> robotColliders = new List<Collider>();

    void Start()
    {
        // Configure performance settings
        ConfigurePerformanceSettings();

        // Initialize robot components
        InitializeRobotComponents();

        // Validate robot complexity
        ValidateRobotComplexity();
    }

    void ConfigurePerformanceSettings()
    {
        Application.targetFrameRate = targetFrameRate;
        Time.fixedDeltaTime = 1f / physicsUpdateRate;

        if (enableLOD)
        {
            EnableLOD();
        }

        if (enableOcclusionCulling)
        {
            EnableOcclusionCulling();
        }
    }

    void InitializeRobotComponents()
    {
        // Find all robot components
        robotRenderers.AddRange(GetComponentsInChildren<Renderer>());
        robotColliders.AddRange(GetComponentsInChildren<Collider>());
    }

    void ValidateRobotComplexity()
    {
        int polygonCount = 0;
        foreach (var renderer in robotRenderers)
        {
            if (renderer.GetComponent<MeshFilter>() != null)
            {
                Mesh mesh = renderer.GetComponent<MeshFilter>().sharedMesh;
                if (mesh != null)
                {
                    polygonCount += mesh.triangles.Length / 3;
                }
            }
        }

        if (polygonCount > maxPolygonCount)
        {
            Debug.LogWarning($"Robot polygon count ({polygonCount}) exceeds recommended limit ({maxPolygonCount})");
        }

        // Validate joint count
        var joints = GetComponentsInChildren<ConfigurableJoint>();
        if (joints.Length > maxJointCount)
        {
            Debug.LogWarning($"Robot joint count ({joints.Length}) exceeds recommended limit ({maxJointCount})");
        }
    }

    void EnableLOD()
    {
        // Configure Level of Detail system
        LODGroup lodGroup = GetComponent<LODGroup>();
        if (lodGroup == null)
        {
            lodGroup = gameObject.AddComponent<LODGroup>();
        }
    }

    void EnableOcclusionCulling()
    {
        // Enable occlusion culling in the scene
        StaticOcclusionCulling.GenerateInBackground();
    }

    void Update()
    {
        // Dynamic optimization based on performance
        OptimizeBasedOnPerformance();
    }

    void OptimizeBasedOnPerformance()
    {
        // Adjust quality settings based on frame rate
        if (Time.unscaledDeltaTime > 1f / (targetFrameRate * 0.8f))
        {
            // Performance is degrading, reduce quality
            ReduceRenderQuality();
        }
        else if (Time.unscaledDeltaTime < 1f / (targetFrameRate * 1.2f))
        {
            // Performance is good, can increase quality
            IncreaseRenderQuality();
        }
    }

    void ReduceRenderQuality()
    {
        // Reduce render quality for better performance
        // This could involve reducing shadow quality, disabling effects, etc.
    }

    void IncreaseRenderQuality()
    {
        // Increase render quality if performance allows
    }
}
```

## Best Practices

1. **Use appropriate Unity version**: LTS versions for stability
2. **Optimize for real-time performance**: Balance visual quality with frame rate
3. **Validate physics parameters**: Match Unity physics with real robot behavior
4. **Implement proper error handling**: Handle ROS connection failures gracefully
5. **Use domain randomization**: Improve AI model generalization
6. **Profile performance**: Monitor Unity and ROS communication performance
7. **Modular design**: Create reusable components for different robots
8. **Documentation**: Comment Unity scripts for clarity

## Troubleshooting Common Issues

### ROS Connection Issues
- Verify ROS TCP Connector settings
- Check firewall settings
- Ensure correct IP addresses and ports

### Performance Issues
- Reduce polygon count of robot models
- Use Level of Detail (LOD) systems
- Optimize physics simulation parameters

### Synchronization Issues
- Ensure proper time synchronization between Unity and ROS
- Use appropriate update rates for different components

## Next Steps

In the next chapter, we'll explore Unity HDRP (High Definition Render Pipeline) for creating photorealistic environments and materials that are essential for perception training and high-fidelity visualization of humanoid robots.