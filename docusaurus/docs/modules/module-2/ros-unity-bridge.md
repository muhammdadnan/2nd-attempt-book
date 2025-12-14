# ROS-Unity Bridge

The ROS-Unity bridge is a critical component that enables real-time communication between ROS 2 and Unity, allowing your humanoid robot to be controlled from ROS nodes while running high-fidelity simulation in Unity. In this chapter, we'll explore how to establish robust communication between these two powerful platforms.

## ROS-Unity Bridge Architecture

### Communication Overview

The ROS-Unity bridge typically uses TCP/IP communication to exchange messages between ROS 2 nodes and Unity:

```
ROS 2 Nodes ←→ TCP/IP Network ←→ Unity ROS TCP Connector ←→ Unity Game Engine
```

### Key Components

1. **ROS TCP Connector**: Unity package that handles TCP communication
2. **Message Serialization**: Conversion between ROS messages and Unity data structures
3. **Threading Model**: Proper handling of ROS communication in Unity's main thread
4. **Transform Synchronization**: Coordination between ROS TF and Unity coordinate systems

## Setting Up the ROS-Unity Bridge

### Unity Setup

First, install the Unity Robotics Hub and ROS TCP Connector:

1. Open Unity Package Manager (Window → Package Manager)
2. Install "ROS TCP Connector" package
3. Add ROS TCP Connector to your scene

### Basic Bridge Configuration

```csharp
// ROSConnectionManager.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class ROSConnectionManager : MonoBehaviour
{
    [Header("ROS Connection Settings")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;
    public bool autoConnect = true;

    [Header("Robot Configuration")]
    public string robotName = "humanoid_robot";
    public float rosUpdateRate = 100f; // Hz

    private ROSConnection ros;
    private float updateTimer;
    private bool isConnected = false;

    void Start()
    {
        InitializeROSConnection();
        SetupROSTopics();
    }

    void InitializeROSConnection()
    {
        // Get or create ROS connection instance
        ros = ROSConnection.instance;

        if (ros == null)
        {
            GameObject rosObject = new GameObject("ROSConnection");
            ros = rosObject.AddComponent<ROSConnection>();
        }

        // Configure connection settings
        ros.host = rosIPAddress;
        ros.port = rosPort;

        // Connect to ROS
        if (autoConnect)
        {
            ConnectToROS();
        }
    }

    public void ConnectToROS()
    {
        if (ros != null)
        {
            ros.Connect(ros.host, ros.port);
            isConnected = true;
            Debug.Log($"Connected to ROS at {ros.host}:{ros.port}");
        }
    }

    void SetupROSTopics()
    {
        // Subscribe to ROS topics
        ros.Subscribe<sensor_msgs.JointState>($"{robotName}/joint_commands", JointCommandCallback);
        ros.Subscribe<geometry_msgs.Twist>($"{robotName}/cmd_vel", CmdVelCallback);
        ros.Subscribe<std_msgs.Bool>($"{robotName}/enable_control", ControlEnableCallback);

        // Publishers will be set up as needed
    }

    void JointCommandCallback(sensor_msgs.JointState jointState)
    {
        // Process joint commands from ROS
        Debug.Log($"Received joint command for {jointState.name.Length} joints");
        // Forward to robot controller
    }

    void CmdVelCallback(geometry_msgs.Twist cmdVel)
    {
        // Process velocity commands from ROS
        Debug.Log($"Received velocity command: linear=({cmdVel.linear.x}, {cmdVel.linear.y}, {cmdVel.linear.z}), " +
                  $"angular=({cmdVel.angular.x}, {cmdVel.angular.y}, {cmdVel.angular.z})");
        // Forward to robot movement system
    }

    void ControlEnableCallback(std_msgs.Bool enableMsg)
    {
        Debug.Log($"Control enable message received: {enableMsg.data}");
        // Enable/disable robot control based on message
    }

    void Update()
    {
        updateTimer += Time.deltaTime;
        if (updateTimer >= 1f / rosUpdateRate)
        {
            updateTimer = 0f;
            UpdateROSCommunication();
        }
    }

    void UpdateROSCommunication()
    {
        // Publish current robot state to ROS
        PublishRobotState();
    }

    void PublishRobotState()
    {
        // Create and publish joint state message
        var jointState = new sensor_msgs.JointState();
        jointState.header = new std_msgs.Header();
        jointState.header.stamp = new builtin_interfaces.Time();
        jointState.header.frame_id = robotName;

        // Fill in joint names and positions (example)
        jointState.name = new string[] { "joint1", "joint2", "joint3" };
        jointState.position = new double[] { 0.0, 0.0, 0.0 };

        ros.Publish($"{robotName}/joint_states", jointState);

        // Publish other state information as needed
        PublishTransforms();
    }

    void PublishTransforms()
    {
        // Publish transforms for ROS TF system
        // This would involve publishing geometry_msgs.TransformStamped messages
    }
}
```

## Advanced Message Handling

### Custom Message Types

Creating and handling custom message types:

```csharp
// CustomHumanoidMessage.cs
using System;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

// Custom message structure for humanoid-specific data
[Serializable]
public class HumanoidStateMsg
{
    public std_msgs.Header header;
    public string robot_name;
    public double[] joint_positions;
    public double[] joint_velocities;
    public double[] joint_efforts;
    public geometry_msgs.Pose robot_pose;
    public double balance_score;
    public string[] contact_points;
    public bool[] joint_limits_exceeded;

    public HumanoidStateMsg()
    {
        header = new std_msgs.Header();
        robot_name = "";
        joint_positions = new double[0];
        joint_velocities = new double[0];
        joint_efforts = new double[0];
        robot_pose = new geometry_msgs.Pose();
        balance_score = 0.0;
        contact_points = new string[0];
        joint_limits_exceeded = new bool[0];
    }
}

// Custom message handler
public class HumanoidMessageHandler : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.instance;

        // Subscribe to custom message
        ros.Subscribe<HumanoidStateMsg>("/humanoid/state", HumanoidStateCallback);
    }

    void HumanoidStateCallback(HumanoidStateMsg msg)
    {
        Debug.Log($"Received humanoid state: Robot={msg.robot_name}, Balance={msg.balance_score}");

        // Process the custom message data
        ProcessHumanoidState(msg);
    }

    void ProcessHumanoidState(HumanoidStateMsg state)
    {
        // Handle humanoid-specific state information
        // Update Unity robot model, check balance, etc.
    }
}
```

### Service Implementation

Implementing ROS services in Unity:

```csharp
// UnityRobotServices.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Nav;

public class UnityRobotServices : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.instance;

        // Register services
        ros.RegisterServiceCallback<SetBoolRequest, SetBoolResponse>(
            "enable_robot_control",
            EnableRobotControlService);

        ros.RegisterServiceCallback<EmptyRequest, EmptyResponse>(
            "reset_robot_position",
            ResetRobotPositionService);

        ros.RegisterServiceCallback<GetPlanRequest, GetPlanResponse>(
            "get_navigation_plan",
            GetNavigationPlanService);
    }

    SetBoolResponse EnableRobotControlService(SetBoolRequest request)
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

        Debug.Log($"Robot control service called: {request.data}, Success: {response.success}");
        return response;
    }

    EmptyResponse ResetRobotPositionService(EmptyRequest request)
    {
        EmptyResponse response = new EmptyResponse();

        // Reset robot to default position
        ResetRobotToDefaultPosition();

        Debug.Log("Robot position reset service called");
        return response;
    }

    GetPlanResponse GetNavigationPlanService(GetPlanRequest request)
    {
        GetPlanResponse response = new GetPlanResponse();

        // Generate navigation plan (simplified)
        // In a real implementation, this would use Unity's NavMesh or custom pathfinding
        response.plan = new nav_msgs.Path();
        response.plan.header = new std_msgs.Header();
        response.plan.header.stamp = new builtin_interfaces.Time();
        response.plan.header.frame_id = "map";

        // Add example poses to the plan
        geometry_msgs.PoseStamped[] poses = new geometry_msgs.PoseStamped[3];
        for (int i = 0; i < poses.Length; i++)
        {
            poses[i] = new geometry_msgs.PoseStamped();
            poses[i].header = new std_msgs.Header();
            poses[i].header.stamp = new builtin_interfaces.Time();
            poses[i].header.frame_id = "map";
            poses[i].pose = new geometry_msgs.Pose();
            poses[i].pose.position.x = i * 1.0; // Example positions
            poses[i].pose.position.y = 0.0;
            poses[i].pose.position.z = 0.0;
            poses[i].pose.orientation.w = 1.0;
        }

        response.plan.poses = poses;

        Debug.Log($"Navigation plan service called, returning {poses.Length} poses");
        return response;
    }

    void EnableRobotControl()
    {
        // Enable robot control systems
        Debug.Log("Robot control enabled");
    }

    void DisableRobotControl()
    {
        // Disable robot control systems
        Debug.Log("Robot control disabled");
    }

    void ResetRobotToDefaultPosition()
    {
        // Reset robot to default position in Unity
        Transform robotTransform = transform; // Assuming this is attached to robot
        robotTransform.position = Vector3.zero;
        robotTransform.rotation = Quaternion.identity;

        Debug.Log("Robot reset to default position");
    }
}
```

## Transform and Coordinate System Management

### TF System Integration

Managing coordinate system synchronization between ROS and Unity:

```csharp
// TransformSynchronizer.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class TransformSynchronizer : MonoBehaviour
{
    [Header("Transform Configuration")]
    public string robotNamespace = "humanoid_robot";
    public string baseFrame = "base_link";
    public string[] jointFrames; // Names of joint frames to publish

    [Header("Coordinate System")]
    public bool convertCoordinateSystem = true;
    public Vector3 rosToUnityScale = new Vector3(1, 1, 1);

    private ROSConnection ros;
    private Transform[] jointTransforms;

    void Start()
    {
        ros = ROSConnection.instance;

        // Initialize joint transforms
        InitializeJointTransforms();
    }

    void InitializeJointTransforms()
    {
        jointTransforms = new Transform[jointFrames.Length];
        for (int i = 0; i < jointFrames.Length; i++)
        {
            Transform joint = transform.FindJointByName(jointFrames[i]);
            if (joint != null)
            {
                jointTransforms[i] = joint;
            }
            else
            {
                Debug.LogWarning($"Joint transform not found: {jointFrames[i]}");
            }
        }
    }

    void Update()
    {
        // Publish transforms at regular intervals
        PublishTransforms();
    }

    void PublishTransforms()
    {
        // Publish base link transform
        PublishTransform(baseFrame, transform);

        // Publish joint transforms
        for (int i = 0; i < jointTransforms.Length; i++)
        {
            if (jointTransforms[i] != null)
            {
                PublishTransform(jointFrames[i], jointTransforms[i]);
            }
        }
    }

    void PublishTransform(string frameId, Transform unityTransform)
    {
        geometry_msgs.TransformStamped tfMsg = new geometry_msgs.TransformStamped();

        tfMsg.header = new std_msgs.Header();
        tfMsg.header.stamp = new builtin_interfaces.Time();
        tfMsg.header.frame_id = baseFrame; // Parent frame
        tfMsg.child_frame_id = frameId;

        // Convert Unity transform to ROS transform
        tfMsg.transform = UnityTransformToROSTransform(unityTransform, transform);

        // Publish the transform
        ros.Publish($"{robotNamespace}/tf", tfMsg);
    }

    geometry_msgs.Transform UnityTransformToROSTransform(Transform child, Transform parent)
    {
        geometry_msgs.Transform rosTransform = new geometry_msgs.Transform();

        // Calculate relative position
        Vector3 relativePosition = parent.InverseTransformPoint(child.position);

        // Apply coordinate system conversion if needed
        if (convertCoordinateSystem)
        {
            relativePosition = ConvertCoordinateSystem(relativePosition);
        }

        rosTransform.translation = new geometry_msgs.Vector3(relativePosition.x, relativePosition.y, relativePosition.z);

        // Calculate relative rotation
        Quaternion relativeRotation = Quaternion.Inverse(parent.rotation) * child.rotation;

        if (convertCoordinateSystem)
        {
            relativeRotation = ConvertRotationCoordinateSystem(relativeRotation);
        }

        rosTransform.rotation = new geometry_msgs.Quaternion(relativeRotation.x, relativeRotation.y, relativeRotation.z, relativeRotation.w);

        return rosTransform;
    }

    Vector3 ConvertCoordinateSystem(Vector3 unityPos)
    {
        // ROS uses X-forward, Y-left, Z-up
        // Unity uses X-right, Y-up, Z-forward
        // Convert: ROS_X = Unity_Z, ROS_Y = -Unity_X, ROS_Z = Unity_Y
        return new Vector3(unityPos.z, -unityPos.x, unityPos.y);
    }

    Quaternion ConvertRotationCoordinateSystem(Quaternion unityRot)
    {
        // Convert quaternion from Unity to ROS coordinate system
        // This is a simplified conversion - more complex in practice
        return new Quaternion(unityRot.z, -unityRot.x, unityRot.y, unityRot.w);
    }

    // Helper method to find joint by name in hierarchy
    Transform FindJointByName(this Transform parent, string jointName)
    {
        Transform[] children = parent.GetComponentsInChildren<Transform>();
        foreach (Transform child in children)
        {
            if (child.name == jointName)
                return child;
        }
        return null;
    }
}
```

## Sensor Data Integration

### Publishing Sensor Data to ROS

Integrating Unity sensors with ROS:

```csharp
// SensorPublisher.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using System.Collections;

public class SensorPublisher : MonoBehaviour
{
    [Header("Camera Configuration")]
    public Camera sensorCamera;
    public int cameraWidth = 640;
    public int cameraHeight = 480;
    public float cameraFramerate = 30f;

    [Header("IMU Configuration")]
    public Transform imuTransform;
    public float imuUpdateRate = 100f;

    [Header("LIDAR Configuration")]
    public Transform lidarTransform;
    public int lidarRays = 720;
    public float lidarRange = 10f;

    [Header("Publishing Settings")]
    public string robotNamespace = "humanoid_robot";
    public float publishFrequency = 30f;

    private ROSConnection ros;
    private float publishTimer;
    private int sequenceNumber = 0;

    // Sensor data buffers
    private float imuUpdateTimer;
    private float cameraUpdateTimer;

    void Start()
    {
        ros = ROSConnection.instance;
        publishTimer = 0f;
        imuUpdateTimer = 0f;
        cameraUpdateTimer = 0f;

        // Initialize camera if not set
        if (sensorCamera == null)
        {
            sensorCamera = GetComponent<Camera>();
            if (sensorCamera == null)
            {
                sensorCamera = Camera.main;
            }
        }

        // Set camera resolution
        if (sensorCamera != null)
        {
            sensorCamera.targetTexture = new RenderTexture(cameraWidth, cameraHeight, 24);
        }
    }

    void Update()
    {
        publishTimer += Time.deltaTime;
        if (publishTimer >= 1f / publishFrequency)
        {
            publishTimer = 0f;
            PublishAllSensorData();
        }

        // Update individual sensors at their rates
        UpdateIMUSensor();
        UpdateCameraSensor();
    }

    void PublishAllSensorData()
    {
        sequenceNumber++;

        // Publish IMU data
        PublishIMUData();

        // Publish camera data
        PublishCameraData();

        // Publish LIDAR data
        PublishLIDARData();

        // Publish other sensor data as needed
    }

    void PublishIMUData()
    {
        if (imuTransform == null) return;

        var imuMsg = new sensor_msgs.Imu();
        imuMsg.header = new std_msgs.Header();
        imuMsg.header.stamp = new builtin_interfaces.Time();
        imuMsg.header.frame_id = "imu_link";
        imuMsg.header.seq = (uint)sequenceNumber;

        // Set orientation (simplified - in practice, integrate gyroscope data)
        imuMsg.orientation.x = imuTransform.rotation.x;
        imuMsg.orientation.y = imuTransform.rotation.y;
        imuMsg.orientation.z = imuTransform.rotation.z;
        imuMsg.orientation.w = imuTransform.rotation.w;

        // Set angular velocity (simplified)
        imuMsg.angular_velocity.x = 0.0f; // Would come from gyroscope simulation
        imuMsg.angular_velocity.y = 0.0f;
        imuMsg.angular_velocity.z = 0.0f;

        // Set linear acceleration (includes gravity)
        Vector3 linearAcc = imuTransform.InverseTransformDirection(Physics.gravity);
        imuMsg.linear_acceleration.x = linearAcc.x;
        imuMsg.linear_acceleration.y = linearAcc.y;
        imuMsg.linear_acceleration.z = linearAcc.z;

        ros.Publish($"{robotNamespace}/imu/data", imuMsg);
    }

    void PublishCameraData()
    {
        if (sensorCamera == null) return;

        // In a real implementation, you would capture the camera image
        // For now, we'll just publish camera info

        var cameraInfo = new sensor_msgs.CameraInfo();
        cameraInfo.header = new std_msgs.Header();
        cameraInfo.header.stamp = new builtin_interfaces.Time();
        cameraInfo.header.frame_id = "camera_link";
        cameraInfo.header.seq = (uint)sequenceNumber;

        cameraInfo.width = (uint)cameraWidth;
        cameraInfo.height = (uint)cameraHeight;

        // Standard camera intrinsic parameters (would be calibrated)
        cameraInfo.K = new double[] {
            320.0, 0.0, 320.0,   // fx, 0, cx
            0.0, 320.0, 240.0,   // 0, fy, cy
            0.0, 0.0, 1.0        // 0, 0, 1
        };

        cameraInfo.P = new double[] {
            320.0, 0.0, 320.0, 0.0,  // fx, 0, cx, 0
            0.0, 320.0, 240.0, 0.0,  // 0, fy, cy, 0
            0.0, 0.0, 1.0, 0.0      // 0, 0, 1, 0
        };

        ros.Publish($"{robotNamespace}/camera/camera_info", cameraInfo);

        // Image data publishing would happen separately with actual image capture
    }

    void PublishLIDARData()
    {
        if (lidarTransform == null) return;

        var laserMsg = new sensor_msgs.LaserScan();
        laserMsg.header = new std_msgs.Header();
        laserMsg.header.stamp = new builtin_interfaces.Time();
        laserMsg.header.frame_id = "lidar_link";
        laserMsg.header.seq = (uint)sequenceNumber;

        laserMsg.angle_min = -Mathf.PI;
        laserMsg.angle_max = Mathf.PI;
        laserMsg.angle_increment = (2 * Mathf.PI) / lidarRays;
        laserMsg.time_increment = 0.0f; // Not applicable for simulated data
        laserMsg.scan_time = 1.0f / 10.0f; // 10Hz
        laserMsg.range_min = 0.1f;
        laserMsg.range_max = lidarRange;

        // Simulate LIDAR ranges
        laserMsg.ranges = new float[lidarRays];
        for (int i = 0; i < lidarRays; i++)
        {
            float angle = laserMsg.angle_min + i * laserMsg.angle_increment;

            // Simulate raycasting in the direction of the laser beam
            Vector3 direction = new Vector3(
                Mathf.Cos(angle),
                0,
                Mathf.Sin(angle)
            ).normalized;

            // In a real implementation, this would involve raycasting
            // For simulation, return a fixed range or simulate based on environment
            laserMsg.ranges[i] = lidarRange * 0.8f; // Simulated distance
        }

        ros.Publish($"{robotNamespace}/scan", laserMsg);
    }

    void UpdateIMUSensor()
    {
        imuUpdateTimer += Time.deltaTime;
        if (imuUpdateTimer >= 1f / imuUpdateRate)
        {
            imuUpdateTimer = 0f;
            // IMU data is published in the main loop
        }
    }

    void UpdateCameraSensor()
    {
        cameraUpdateTimer += Time.deltaTime;
        if (cameraUpdateTimer >= 1f / cameraFramerate)
        {
            cameraUpdateTimer = 0f;
            // Camera data is published in the main loop
        }
    }
}
```

## Performance Optimization

### Threading and Communication Optimization

Optimizing the ROS-Unity bridge for performance:

```csharp
// OptimizedROSCommunicator.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Concurrent;
using System.Threading.Tasks;

public class OptimizedROSCommunicator : MonoBehaviour
{
    [Header("Performance Settings")]
    public float rosUpdateRate = 100f;
    public int messageQueueSize = 100;
    public bool enableMessageBundling = true;

    [Header("Threading")]
    public bool useBackgroundThreads = true;
    public int backgroundThreadCount = 2;

    private ROSConnection ros;
    private float updateTimer;

    // Message queues for optimization
    private ConcurrentQueue<System.Action> mainThreadActions = new ConcurrentQueue<System.Action>();
    private ConcurrentQueue<object> outgoingMessages = new ConcurrentQueue<object>();

    void Start()
    {
        ros = ROSConnection.instance;
        updateTimer = 0f;

        // Initialize background threads if enabled
        if (useBackgroundThreads)
        {
            InitializeBackgroundThreads();
        }
    }

    void InitializeBackgroundThreads()
    {
        // Create background threads for message processing
        for (int i = 0; i < backgroundThreadCount; i++)
        {
            Task.Run(() => BackgroundMessageProcessor());
        }
    }

    void BackgroundMessageProcessor()
    {
        // Background thread for processing outgoing messages
        while (true)
        {
            if (outgoingMessages.TryDequeue(out object message))
            {
                // Process message in background thread
                ProcessOutgoingMessage(message);
            }
            else
            {
                // No messages to process, sleep briefly
                System.Threading.Thread.Sleep(1);
            }
        }
    }

    void ProcessOutgoingMessage(object message)
    {
        // Process the message (serialization, validation, etc.)
        // This runs in a background thread to avoid blocking Unity's main thread
    }

    void Update()
    {
        // Process any actions that need to run on the main thread
        ProcessMainThreadActions();

        updateTimer += Time.deltaTime;
        if (updateTimer >= 1f / rosUpdateRate)
        {
            updateTimer = 0f;
            OptimizedROSUpdate();
        }
    }

    void ProcessMainThreadActions()
    {
        while (mainThreadActions.TryDequeue(out System.Action action))
        {
            action?.Invoke();
        }
    }

    void OptimizedROSUpdate()
    {
        // Optimized update method that bundles messages when possible
        if (enableMessageBundling)
        {
            BundleAndSendMessages();
        }
        else
        {
            SendMessagesIndividually();
        }
    }

    void BundleAndSendMessages()
    {
        // Bundle multiple messages together to reduce network overhead
        // This is particularly useful for high-frequency sensor data
    }

    void SendMessagesIndividually()
    {
        // Send messages individually (default behavior)
        PublishRobotState();
        PublishSensorData();
        PublishTransforms();
    }

    void PublishRobotState()
    {
        // Publish robot state with optimization
        // Use object pooling to reduce garbage collection
    }

    void PublishSensorData()
    {
        // Publish sensor data with optimization
        // Consider message frequency and importance
    }

    void PublishTransforms()
    {
        // Publish transforms with optimization
        // Only publish when transforms actually change
    }

    // Method to safely queue actions to main thread
    public void QueueActionOnMainThread(System.Action action)
    {
        mainThreadActions.Enqueue(action);
    }

    // Method to safely queue messages for sending
    public void QueueMessageForSending(object message, string topic)
    {
        if (useBackgroundThreads)
        {
            outgoingMessages.Enqueue(new { Message = message, Topic = topic });
        }
        else
        {
            // Send directly if not using background threads
            ros.Publish(topic, message);
        }
    }
}
```

## Error Handling and Diagnostics

### Robust Error Handling

Implementing error handling for the ROS-Unity bridge:

```csharp
// ROSBridgeDiagnostics.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;
using System.Diagnostics;

public class ROSBridgeDiagnostics : MonoBehaviour
{
    [Header("Connection Diagnostics")]
    public bool enableConnectionDiagnostics = true;
    public float diagnosticUpdateRate = 1f;
    public float connectionTimeout = 5f;

    [Header("Performance Diagnostics")]
    public bool enablePerformanceDiagnostics = true;
    public float performanceUpdateRate = 2f;

    private ROSConnection ros;
    private float diagnosticTimer;
    private float performanceTimer;

    // Connection diagnostics
    private float lastMessageTime;
    private int messagesReceived = 0;
    private int messagesSent = 0;

    // Performance diagnostics
    private List<float> messageProcessingTimes = new List<float>();
    private float avgProcessingTime = 0f;

    void Start()
    {
        ros = ROSConnection.instance;
        diagnosticTimer = 0f;
        performanceTimer = 0f;
        lastMessageTime = Time.time;
    }

    void Update()
    {
        UpdateDiagnostics();
    }

    void UpdateDiagnostics()
    {
        // Update connection diagnostics
        if (enableConnectionDiagnostics)
        {
            diagnosticTimer += Time.deltaTime;
            if (diagnosticTimer >= 1f / diagnosticUpdateRate)
            {
                diagnosticTimer = 0f;
                UpdateConnectionDiagnostics();
            }
        }

        // Update performance diagnostics
        if (enablePerformanceDiagnostics)
        {
            performanceTimer += Time.deltaTime;
            if (performanceTimer >= 1f / performanceUpdateRate)
            {
                performanceTimer = 0f;
                UpdatePerformanceDiagnostics();
            }
        }
    }

    void UpdateConnectionDiagnostics()
    {
        // Check connection status
        bool isConnected = ros != null && ros.IsConnected();

        if (!isConnected)
        {
            Debug.LogWarning("ROS connection lost!");
            AttemptReconnection();
        }
        else
        {
            // Check if we're receiving messages
            float timeSinceLastMessage = Time.time - lastMessageTime;
            if (timeSinceLastMessage > connectionTimeout)
            {
                Debug.LogWarning($"No messages received for {timeSinceLastMessage:F2} seconds");
            }
        }

        // Log diagnostic information
        Debug.Log($"ROS Connection Status: {(isConnected ? "Connected" : "Disconnected")}, " +
                 $"Messages Rcvd: {messagesReceived}, Sent: {messagesSent}");
    }

    void UpdatePerformanceDiagnostics()
    {
        // Calculate average processing time
        if (messageProcessingTimes.Count > 0)
        {
            avgProcessingTime = 0f;
            foreach (float time in messageProcessingTimes)
            {
                avgProcessingTime += time;
            }
            avgProcessingTime /= messageProcessingTimes.Count;

            // Keep only recent measurements
            if (messageProcessingTimes.Count > 100)
            {
                messageProcessingTimes.RemoveRange(0, messageProcessingTimes.Count - 100);
            }
        }

        // Log performance information
        Debug.Log($"Avg Message Processing Time: {avgProcessingTime * 1000:F2}ms, " +
                 $"Messages/sec: {messagesReceived / performanceUpdateRate:F2}");
    }

    void AttemptReconnection()
    {
        if (ros != null)
        {
            Debug.Log("Attempting to reconnect to ROS...");
            ros.Connect(ros.host, ros.port);
        }
    }

    // Callback for when messages are received
    public void OnMessageReceived()
    {
        lastMessageTime = Time.time;
        messagesReceived++;
    }

    // Callback for when messages are sent
    public void OnMessageSent()
    {
        messagesSent++;
    }

    // Method to measure message processing time
    public void MeasureProcessingTime(System.Action processAction)
    {
        Stopwatch stopwatch = Stopwatch.StartNew();
        processAction?.Invoke();
        stopwatch.Stop();

        float processingTime = stopwatch.ElapsedMilliseconds / 1000f;
        messageProcessingTimes.Add(processingTime);
    }

    // Error handling wrapper
    public T SafeROSCall<T>(System.Func<T> rosCall, T defaultValue = default(T))
    {
        try
        {
            return rosCall();
        }
        catch (System.Exception e)
        {
            Debug.LogError($"ROS call failed: {e.Message}");
            return defaultValue;
        }
    }

    public void SafeROSCall(System.Action rosCall)
    {
        try
        {
            rosCall?.Invoke();
        }
        catch (System.Exception e)
        {
            Debug.LogError($"ROS call failed: {e.Message}");
        }
    }
}
```

## Best Practices

1. **Connection Management**: Implement proper connection handling and reconnection logic
2. **Message Frequency**: Balance message frequency with performance requirements
3. **Threading**: Use background threads for heavy processing to avoid blocking Unity
4. **Error Handling**: Implement comprehensive error handling and diagnostics
5. **Coordinate Systems**: Properly handle coordinate system conversions
6. **Performance Monitoring**: Monitor communication performance and optimize as needed
7. **Message Validation**: Validate messages before processing to prevent errors
8. **Resource Management**: Properly manage memory and resources to prevent leaks

## Troubleshooting Common Issues

### Connection Issues
- Verify IP addresses and ports are correct
- Check firewall settings
- Ensure ROS network is properly configured
- Use ROS network tools to diagnose connectivity

### Performance Issues
- Reduce message frequency for non-critical data
- Use message bundling for high-frequency updates
- Optimize Unity scene complexity
- Monitor Unity and ROS performance metrics

### Synchronization Issues
- Ensure proper time synchronization
- Use appropriate coordinate system conversions
- Verify transform frame relationships

## Next Steps

In the next chapter, we'll conclude Module 2 by summarizing the digital twin concepts we've covered and preparing for Module 3, where we'll explore NVIDIA Isaac as an alternative simulation and AI development platform for humanoid robots.