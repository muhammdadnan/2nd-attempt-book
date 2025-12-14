# Unity HDRP

The High Definition Render Pipeline (HDRP) in Unity provides advanced rendering capabilities that are essential for creating photorealistic environments for humanoid robot simulation and perception training. In this chapter, we'll explore how to configure and use HDRP to achieve high-fidelity visual simulation.

## HDRP Overview

HDRP is a Scriptable Render Pipeline that provides advanced rendering features:

- **Physically Based Rendering (PBR)**: Realistic material response to lighting
- **Global Illumination**: Advanced light bouncing and color bleeding
- **Real-time Ray Tracing**: Accurate reflections, shadows, and lighting
- **High Dynamic Range**: Accurate light intensity representation
- **Advanced Post-Processing**: Sophisticated image effects

## Setting Up HDRP for Robotics

### Creating an HDRP Project

To create a new HDRP project:

1. **Open Unity Hub** and create a new project
2. **Select the HDRP template** or manually configure HDRP
3. **Install HDRP package** through Package Manager (com.unity.render-pipelines.high-definition)

### HDRP Asset Configuration

```json
// HDRP Asset Configuration
{
  "m_Name": "HumanoidRobotHDRP",
  "m_Version": 18,
  "m_ObsoleteRenderingPath": 0,
  "m_RenderingPath": 0,
  "m_RenderPipelineResources": {
    "m_RendererTypes": [
      "Unity.RenderPipelines.HighDefinition.Runtime.ForwardOnlyRendererData",
      "Unity.RenderPipelines.HighDefinition.Runtime.ForwardRendererData",
      "Unity.RenderPipelines.HighDefinition.Runtime.TileAndClusterRendererData"
    ]
  },
  "m_DefaultVolumeProfile": "Assets/Settings/DefaultVolumeProfile.asset",
  "m_RenderingPathDefaultCameraFrameSettings": {
    "bitDatas": {
      "mask": 17668280385832192
    }
  }
}
```

### Volume Configuration for Robotics

HDRP volumes control post-processing effects and rendering settings:

```csharp
// RobotEnvironmentVolume.cs
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

[ExecuteAlways]
public class RobotEnvironmentVolume : MonoBehaviour
{
    [Header("Lighting Settings")]
    public float exposure = 0.0f;
    public float ambientIntensity = 1.0f;
    public Color ambientColor = Color.white;

    [Header("Post-Processing")]
    public bool enableBloom = true;
    public float bloomIntensity = 0.5f;
    public bool enableDepthOfField = false;
    public float dofFocusDistance = 10.0f;

    [Header("Camera Settings")]
    public float cameraFocalLength = 50.0f;
    public float cameraSensorSize = 36.0f;

    private Volume volume;
    private Exposure exposureComponent;
    private Bloom bloomComponent;
    private DepthOfField dofComponent;

    void Start()
    {
        InitializeVolume();
        ConfigureHDRPRoboticsSettings();
    }

    void InitializeVolume()
    {
        volume = GetComponent<Volume>();
        if (volume == null)
        {
            volume = gameObject.AddComponent<Volume>();
        }

        // Get or create rendering components
        if (!volume.profile.TryGet<Exposure>(out exposureComponent))
        {
            exposureComponent = volume.profile.Add<Exposure>();
        }

        if (!volume.profile.TryGet<Bloom>(out bloomComponent))
        {
            bloomComponent = volume.profile.Add<Bloom>();
        }

        if (!volume.profile.TryGet<DepthOfField>(out dofComponent))
        {
            dofComponent = volume.profile.Add<DepthOfField>();
        }
    }

    void ConfigureHDRPRoboticsSettings()
    {
        // Configure exposure settings
        exposureComponent.m_Enabled.value = true;
        exposureComponent.m_PostExposure.value = exposure;

        // Configure bloom settings for realistic lighting
        if (enableBloom)
        {
            bloomComponent.m_Enabled.value = true;
            bloomComponent.threshold.value = 1.0f;
            bloomComponent.intensity.value = bloomIntensity;
            bloomComponent.scatter.value = 0.7f;
        }
        else
        {
            bloomComponent.m_Enabled.value = false;
        }

        // Configure depth of field for camera realism
        if (enableDepthOfField)
        {
            dofComponent.m_Enabled.value = true;
            dofComponent.focusDistance.value = dofFocusDistance;
            dofComponent.aperture.value = 5.6f;
            dofComponent.focalLength.value = cameraFocalLength;
            dofComponent.sensorSize.value = cameraSensorSize;
        }
        else
        {
            dofComponent.m_Enabled.value = false;
        }
    }

    void Update()
    {
        // Dynamically adjust settings if needed
        UpdateDynamicSettings();
    }

    void UpdateDynamicSettings()
    {
        // Example: Adjust settings based on robot state or environment
        if (exposureComponent != null)
        {
            exposureComponent.m_PostExposure.value = exposure;
        }
    }
}
```

## Advanced HDRP Features for Robotics

### Real-time Ray Tracing

For photorealistic reflections and lighting:

```csharp
// RayTracingManager.cs
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

public class RayTracingManager : MonoBehaviour
{
    [Header("Ray Tracing Settings")]
    public bool enableRayTracing = true;
    public bool enableRayTracedReflections = true;
    public bool enableRayTracedShadows = true;
    public bool enableRayTracedAO = true;

    [Header("Performance Settings")]
    public int rayTracingMaxRecursion = 3;
    public int rayTracingResolutionScale = 100; // Percentage

    private HDAdditionalLightData lightData;
    private HDAdditionalCameraData cameraData;

    void Start()
    {
        ConfigureRayTracingSettings();
    }

    void ConfigureRayTracingSettings()
    {
        if (!enableRayTracing) return;

        // Configure camera for ray tracing
        cameraData = GetComponent<Camera>().GetComponent<HDAdditionalCameraData>();
        if (cameraData != null)
        {
            cameraData.volumeLayerMask = 1; // Default layer
            cameraData.volumeAnchorOverride = null;
        }

        // Configure lights for ray tracing
        Light[] lights = FindObjectsOfType<Light>();
        foreach (Light light in lights)
        {
            lightData = light.GetComponent<HDAdditionalLightData>();
            if (lightData != null)
            {
                // Enable ray traced shadows if applicable
                if (enableRayTracedShadows)
                {
                    lightData.enableRayTracedShadows = true;
                }

                // Configure light for ray tracing
                lightData.SetLightLayers(1); // Enable on layer 1
            }
        }

        // Configure materials for ray tracing
        ConfigureRayTracedMaterials();
    }

    void ConfigureRayTracedMaterials()
    {
        // Find all materials that should support ray tracing
        Renderer[] renderers = FindObjectsOfType<Renderer>();
        foreach (Renderer renderer in renderers)
        {
            Material[] materials = renderer.sharedMaterials;
            foreach (Material material in materials)
            {
                // Configure material for ray tracing
                if (material.HasProperty("_RayTracing"))
                {
                    material.SetFloat("_RayTracing", enableRayTracing ? 1.0f : 0.0f);
                }
            }
        }
    }

    void Update()
    {
        // Update ray tracing settings dynamically if needed
        UpdateRayTracingSettings();
    }

    void UpdateRayTracingSettings()
    {
        // Dynamic ray tracing configuration
        if (cameraData != null)
        {
            // Adjust settings based on performance or quality requirements
        }
    }
}
```

### Global Illumination Configuration

For realistic indirect lighting:

```csharp
// GlobalIlluminationManager.cs
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

public class GlobalIlluminationManager : MonoBehaviour
{
    [Header("Global Illumination Settings")]
    public bool enableRealtimeGI = true;
    public bool enableBakedGI = true;
    public float realtimeGIBounceScale = 1.0f;
    public float realtimeGIIntensity = 1.0f;

    [Header("Lightmap Settings")]
    public LightmapBakeType lightmapBakeType = LightmapBakeType.Baked;
    public TextureCompression textureCompression = TextureCompression.HighQuality;

    private HDAdditionalSceneData sceneData;

    void Start()
    {
        ConfigureGlobalIllumination();
    }

    void ConfigureGlobalIllumination()
    {
        // Get or create HD scene data
        sceneData = FindObjectOfType<HDAdditionalSceneData>();
        if (sceneData == null)
        {
            GameObject sceneObject = new GameObject("HD Scene Data");
            sceneObject.transform.SetParent(transform);
            sceneData = sceneObject.AddComponent<HDAdditionalSceneData>();
        }

        // Configure scene for GI
        sceneData.isRealtimeLightmapsEnabled = enableRealtimeGI;
        sceneData.isBakedLightmapsEnabled = enableBakedGI;

        // Configure global illumination settings
        ConfigureLightingSettings();
    }

    void ConfigureLightingSettings()
    {
        // Configure light bounce settings
        if (enableRealtimeGI)
        {
            // In HDRP, this is typically configured through the Lighting window
            // or through custom volume settings
            ConfigureRealtimeGIVolume();
        }

        if (enableBakedGI)
        {
            ConfigureBakedGIVolume();
        }
    }

    void ConfigureRealtimeGIVolume()
    {
        // Add or configure a volume with realtime GI settings
        Volume volume = GetComponent<Volume>();
        if (volume == null)
        {
            volume = gameObject.AddComponent<Volume>();
        }

        // Configure any specific GI-related volume components
        // This might include LightCluster or other HDRP-specific settings
    }

    void ConfigureBakedGIVolume()
    {
        // Configure baked GI settings
        // This is typically done through Unity's Lighting window
        // but can be automated through scripting if needed
    }

    public void TriggerLightmapBake()
    {
        if (enableBakedGI)
        {
            // Trigger lightmap baking
            Lightmapping.Bake();
        }
    }
}
```

## HDRP Materials for Robotics

### PBR Material Configuration

Creating realistic robot materials:

```csharp
// RobotMaterialManager.cs
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

public class RobotMaterialManager : MonoBehaviour
{
    [Header("Robot Material Properties")]
    public Material baseMaterial;
    public Material metalMaterial;
    public Material rubberMaterial;
    public Material glassMaterial;

    [Header("Material Variations")]
    public float metalSmoothness = 0.8f;
    public float rubberSmoothness = 0.2f;
    public Color metalColor = Color.gray;
    public Color rubberColor = Color.black;

    [Header("Wear and Tear")]
    public bool enableWearAndTear = false;
    public float wearAmount = 0.0f;

    void Start()
    {
        ConfigureRobotMaterials();
    }

    void ConfigureRobotMaterials()
    {
        // Configure base material
        if (baseMaterial != null)
        {
            ConfigureBaseMaterial(baseMaterial);
        }

        // Configure metal material
        if (metalMaterial != null)
        {
            ConfigureMetalMaterial(metalMaterial);
        }

        // Configure rubber material (for feet, joints)
        if (rubberMaterial != null)
        {
            ConfigureRubberMaterial(rubberMaterial);
        }

        // Configure glass material (for cameras, sensors)
        if (glassMaterial != null)
        {
            ConfigureGlassMaterial(glassMaterial);
        }

        // Apply wear and tear if enabled
        if (enableWearAndTear)
        {
            ApplyWearAndTear();
        }
    }

    void ConfigureBaseMaterial(Material material)
    {
        // Set base PBR properties
        material.SetColor("_BaseColor", metalColor);
        material.SetFloat("_Smoothness", metalSmoothness);
        material.SetFloat("_Metallic", 0.9f); // Highly metallic
        material.SetFloat("_Specular", 1.0f);
    }

    void ConfigureMetalMaterial(Material material)
    {
        // Configure for metallic surfaces
        material.SetColor("_BaseColor", metalColor);
        material.SetFloat("_Smoothness", metalSmoothness);
        material.SetFloat("_Metallic", 0.95f);
        material.SetFloat("_Specular", 1.0f);

        // Add normal map for surface detail
        if (material.HasProperty("_BumpMap"))
        {
            // Assign appropriate normal map
        }
    }

    void ConfigureRubberMaterial(Material material)
    {
        // Configure for rubber/silicone surfaces
        material.SetColor("_BaseColor", rubberColor);
        material.SetFloat("_Smoothness", rubberSmoothness);
        material.SetFloat("_Metallic", 0.0f); // Non-metallic
        material.SetFloat("_Specular", 0.5f);

        // Add slight subsurface scattering for rubber look
        if (material.HasProperty("_SubsurfaceMask"))
        {
            material.SetFloat("_SubsurfaceMask", 0.3f);
        }
    }

    void ConfigureGlassMaterial(Material material)
    {
        // Configure for transparent surfaces (camera domes, sensors)
        material.SetColor("_BaseColor", new Color(0.9f, 0.9f, 0.95f, 0.8f));
        material.SetFloat("_Smoothness", 0.98f);
        material.SetFloat("_Metallic", 0.0f);
        material.SetFloat("_Specular", 1.0f);
        material.SetFloat("_Transmission", 0.9f);

        // Enable transparency
        material.SetFloat("_SurfaceType", 1.0f); // Transparent
        material.SetFloat("_BlendMode", 0.0f); // Alpha
    }

    void ApplyWearAndTear()
    {
        // Apply wear patterns to materials
        Renderer[] renderers = GetComponentsInChildren<Renderer>();
        foreach (Renderer renderer in renderers)
        {
            Material[] materials = renderer.sharedMaterials;
            foreach (Material material in materials)
            {
                // Add scratches, wear patterns, etc.
                ApplyWearPattern(material, wearAmount);
            }
        }
    }

    void ApplyWearPattern(Material material, float wearAmount)
    {
        // Apply wear and tear using texture overlays or property modifications
        if (material.HasProperty("_NormalMap"))
        {
            // Add scratch normal map based on wear amount
        }

        if (material.HasProperty("_Smoothness"))
        {
            // Reduce smoothness in worn areas
            float currentSmoothness = material.GetFloat("_Smoothness");
            material.SetFloat("_Smoothness", currentSmoothness * (1 - wearAmount * 0.3f));
        }
    }
}
```

## Camera Configuration for Robotics

### HDRP Camera Setup

Configuring cameras for robotic perception:

```csharp
// RobotCameraController.cs
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

public class RobotCameraController : MonoBehaviour
{
    [Header("Camera Specifications")]
    public float horizontalFOV = 60.0f; // Degrees
    public int resolutionWidth = 640;
    public int resolutionHeight = 480;
    public float nearClip = 0.1f;
    public float farClip = 100.0f;

    [Header("HDRP Specific Settings")]
    public bool enableMotionBlur = false;
    public bool enableLensDistortion = false;
    public float aperture = 2.8f;
    public float shutterSpeed = 1.0f / 60.0f;
    public float iso = 200.0f;

    [Header("Depth of Field")]
    public bool enableDOF = false;
    public float focusDistance = 5.0f;
    public float focalLength = 50.0f;

    private Camera unityCamera;
    private HDAdditionalCameraData hdCameraData;
    private MotionBlur motionBlurEffect;
    private DepthOfField dofEffect;

    void Start()
    {
        InitializeCamera();
        ConfigureHDRPCamera();
    }

    void InitializeCamera()
    {
        unityCamera = GetComponent<Camera>();
        if (unityCamera == null)
        {
            unityCamera = gameObject.AddComponent<Camera>();
        }

        hdCameraData = GetComponent<HDAdditionalCameraData>();
        if (hdCameraData == null)
        {
            hdCameraData = gameObject.AddComponent<HDAdditionalCameraData>();
        }
    }

    void ConfigureHDRPCamera()
    {
        // Set basic camera properties
        unityCamera.fieldOfView = CalculateVerticalFOV();
        unityCamera.nearClipPlane = nearClip;
        unityCamera.farClipPlane = farClip;
        unityCamera.aspect = (float)resolutionWidth / resolutionHeight;

        // Configure HDRP-specific camera data
        hdCameraData.backgroundColorHDR = Color.black;
        hdCameraData.clearColorMode = HDAdditionalCameraData.ClearColorMode.Color;
        hdCameraData.volumeLayerMask = 1; // Default layer
        hdCameraData.antialiasing = HDAdditionalCameraData.AntialiasingMode.SubpixelMorphologicalAntiAliasing;
        hdCameraData.SMAAQuality = HDAdditionalCameraData.SMAAQualityLevel.High;

        // Configure exposure settings
        ConfigureExposureSettings();

        // Configure advanced effects
        ConfigureAdvancedEffects();
    }

    float CalculateVerticalFOV()
    {
        // Calculate vertical FOV from horizontal FOV
        float aspectRatio = (float)resolutionWidth / resolutionHeight;
        float verticalFOV = 2.0f * Mathf.Atan(Mathf.Tan(horizontalFOV * Mathf.Deg2Rad / 2.0f) / aspectRatio) * Mathf.Rad2Deg;
        return verticalFOV;
    }

    void ConfigureExposureSettings()
    {
        // Configure camera exposure for robotics applications
        hdCameraData.SetGenericFramingMode(CameraFramingMode.Fit, 1.0f);
    }

    void ConfigureAdvancedEffects()
    {
        // Configure motion blur
        if (enableMotionBlur)
        {
            hdCameraData.volumeLayerMask |= 1 << 8; // Enable motion blur layer
        }

        // Configure depth of field
        if (enableDOF)
        {
            ConfigureDepthOfField();
        }

        // Configure lens distortion
        if (enableLensDistortion)
        {
            ConfigureLensDistortion();
        }
    }

    void ConfigureDepthOfField()
    {
        // This would typically be configured through a volume
        // Create or find a volume with DOF settings
        Volume volume = GetComponent<Volume>();
        if (volume == null)
        {
            volume = gameObject.AddComponent<Volume>();
        }

        DepthOfField dof;
        if (!volume.profile.TryGet<DepthOfField>(out dof))
        {
            dof = volume.profile.Add<DepthOfField>();
        }

        dof.focusDistance.value = focusDistance;
        dof.focalLength.value = focalLength;
        dof.aperture.value = aperture;
    }

    void ConfigureLensDistortion()
    {
        // Configure lens distortion through volume
        Volume volume = GetComponent<Volume>();
        if (volume == null)
        {
            volume = gameObject.AddComponent<Volume>();
        }

        Distortion distortion;
        if (!volume.profile.TryGet<Distortion>(out distortion))
        {
            distortion = volume.profile.Add<Distortion>();
        }

        // Configure distortion parameters
        // These would be based on real camera specifications
        distortion.intensity.value = 0.1f;
        distortion.xCoeff.value = 0.0f;
        distortion.yCoeff.value = 0.0f;
    }

    public void SetCameraParameters(float newFov, int newWidth, int newHeight)
    {
        horizontalFOV = newFov;
        resolutionWidth = newWidth;
        resolutionHeight = newHeight;

        // Update camera settings
        unityCamera.fieldOfView = CalculateVerticalFOV();
        unityCamera.aspect = (float)resolutionWidth / resolutionHeight;
    }

    void Update()
    {
        // Update camera dynamically if needed
        UpdateDynamicSettings();
    }

    void UpdateDynamicSettings()
    {
        // Update any settings that change dynamically
        // For example, adjusting focus based on detected objects
    }
}
```

## Performance Optimization for HDRP

### HDRP Performance Settings

Optimizing HDRP for real-time robotics simulation:

```csharp
// HDRPPerformanceOptimizer.cs
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

public class HDRPPerformanceOptimizer : MonoBehaviour
{
    [Header("Performance Quality Settings")]
    public QualityLevel qualityLevel = QualityLevel.High;
    public bool enableAdaptiveQuality = false;
    public float targetFrameRate = 30.0f;

    [Header("Rendering Optimizations")]
    public bool enableDynamicBatching = true;
    public bool enableGPUInstancing = true;
    public int maxLODLevel = 2;
    public float lodBias = 1.0f;

    [Header("HDRP Specific Optimizations")]
    public bool enableTileAndCluster = true;
    public bool enableComputeSkinning = true;
    public bool enableComputeDeformation = true;

    [Header("Ray Tracing Optimizations")]
    public bool enableRayTracingOptimizations = true;
    public int rayTracingMaxRecursionDepth = 2;
    public float rayTracingResolutionScale = 75.0f; // Percentage

    private HDRenderPipelineAsset hdAsset;
    private float lastFrameTime;

    void Start()
    {
        ConfigurePerformanceSettings();
        InitializePerformanceMonitoring();
    }

    void ConfigurePerformanceSettings()
    {
        // Configure Unity engine settings
        QualitySettings.vSyncCount = 0; // Disable VSync for consistent frame timing
        Application.targetFrameRate = Mathf.RoundToInt(targetFrameRate);

        // Configure batching
        GraphicsSettings.useScriptableRenderPipelineBatching = true;

        // Configure HDRP asset settings
        ConfigureHDRPAsset();

        // Configure renderer settings
        ConfigureRendererSettings();
    }

    void ConfigureHDRPAsset()
    {
        hdAsset = GraphicsSettings.renderPipelineAsset as HDRenderPipelineAsset;
        if (hdAsset != null)
        {
            // Configure asset-specific performance settings
            ConfigureQualitySettings();
        }
    }

    void ConfigureQualitySettings()
    {
        // Configure quality settings based on selected level
        switch (qualityLevel)
        {
            case QualityLevel.Low:
                ConfigureLowQualitySettings();
                break;
            case QualityLevel.Medium:
                ConfigureMediumQualitySettings();
                break;
            case QualityLevel.High:
                ConfigureHighQualitySettings();
                break;
            case QualityLevel.Ultra:
                ConfigureUltraQualitySettings();
                break;
        }

        // Apply adaptive quality settings
        if (enableAdaptiveQuality)
        {
            ConfigureAdaptiveQualitySettings();
        }
    }

    void ConfigureLowQualitySettings()
    {
        // Low quality settings for maximum performance
        QualitySettings.SetQualityLevel(0, true);
        lodBias = 0.5f;
        maxLODLevel = 1;
    }

    void ConfigureMediumQualitySettings()
    {
        // Medium quality settings balancing performance and quality
        QualitySettings.SetQualityLevel(2, true);
        lodBias = 0.75f;
        maxLODLevel = 2;
    }

    void ConfigureHighQualitySettings()
    {
        // High quality settings for good balance
        QualitySettings.SetQualityLevel(4, true);
        lodBias = 1.0f;
        maxLODLevel = 3;
    }

    void ConfigureUltraQualitySettings()
    {
        // Ultra quality settings for maximum visual fidelity
        QualitySettings.SetQualityLevel(5, true);
        lodBias = 1.5f;
        maxLODLevel = 4;
    }

    void ConfigureAdaptiveQualitySettings()
    {
        // Configure adaptive quality based on performance
        // This would typically involve monitoring frame rate and adjusting settings
    }

    void ConfigureRendererSettings()
    {
        // Configure renderer-specific optimizations
        ConfigureLODSettings();
        ConfigureOcclusionCulling();
    }

    void ConfigureLODSettings()
    {
        // Configure LOD bias and maximum levels
        QualitySettings.lodBias = lodBias;
        QualitySettings.maximumLODLevel = maxLODLevel;

        // Configure LOD groups in the scene
        LODGroup[] lodGroups = FindObjectsOfType<LODGroup>();
        foreach (LODGroup lodGroup in lodGroups)
        {
            lodGroup.animateCrossFading = true;
        }
    }

    void ConfigureOcclusionCulling()
    {
        // Enable occlusion culling for performance
        StaticOcclusionCulling.Compute();
    }

    void InitializePerformanceMonitoring()
    {
        lastFrameTime = Time.realtimeSinceStartup;
    }

    void Update()
    {
        MonitorPerformance();
        AdjustSettingsBasedOnPerformance();
    }

    void MonitorPerformance()
    {
        float currentFrameTime = Time.realtimeSinceStartup;
        float frameTime = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        float currentFrameRate = 1.0f / frameTime;

        // Log performance if needed
        if (Time.frameCount % 60 == 0) // Every 60 frames
        {
            Debug.Log($"Current Frame Rate: {currentFrameRate:F2} FPS");
        }
    }

    void AdjustSettingsBasedOnPerformance()
    {
        if (!enableAdaptiveQuality) return;

        float currentFrameRate = 1.0f / Time.unscaledDeltaTime;

        if (currentFrameRate < targetFrameRate * 0.8f)
        {
            // Performance is degrading, reduce quality
            ReduceQualitySettings();
        }
        else if (currentFrameRate > targetFrameRate * 1.2f)
        {
            // Performance is good, can increase quality
            IncreaseQualitySettings();
        }
    }

    void ReduceQualitySettings()
    {
        // Reduce rendering quality to maintain frame rate
        // This might involve reducing shadow resolution, disabling effects, etc.
        QualitySettings.SetQualityLevel(Mathf.Max(0, QualitySettings.GetQualityLevel() - 1));
    }

    void IncreaseQualitySettings()
    {
        // Increase rendering quality if performance allows
        QualitySettings.SetQualityLevel(Mathf.Min(QualitySettings.names.Length - 1, QualitySettings.GetQualityLevel() + 1));
    }
}

public enum QualityLevel
{
    Low,
    Medium,
    High,
    Ultra
}
```

## Synthetic Data Generation with HDRP

### Photorealistic Dataset Creation

```csharp
// HDRPSyntheticDataManager.cs
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using System.Collections;
using System.IO;

public class HDRPSyntheticDataManager : MonoBehaviour
{
    [Header("Synthetic Data Configuration")]
    public Camera dataCaptureCamera;
    public int datasetSize = 1000;
    public float captureInterval = 0.1f;
    public string datasetSavePath = "Assets/SyntheticData/";

    [Header("Domain Randomization")]
    public Light[] lightsToRandomize;
    public Material[] materialOptions;
    public Color[] environmentColors;
    public float[] exposureVariations;

    [Header("Annotation Settings")]
    public bool generateDepthMaps = true;
    public bool generateSegmentation = true;
    public bool generateOpticalFlow = false;

    private bool isCollectingData = false;
    private int currentDataIndex = 0;
    private Material originalMaterial;

    void Start()
    {
        if (dataCaptureCamera == null)
        {
            dataCaptureCamera = Camera.main;
        }

        // Ensure dataset directory exists
        if (!Directory.Exists(datasetSavePath))
        {
            Directory.CreateDirectory(datasetSavePath);
        }
    }

    public void StartDataCollection()
    {
        if (!isCollectingData)
        {
            isCollectingData = true;
            StartCoroutine(CollectSyntheticDataset());
        }
    }

    public void StopDataCollection()
    {
        isCollectingData = false;
    }

    IEnumerator CollectSyntheticDataset()
    {
        Debug.Log($"Starting synthetic dataset collection: {datasetSize} samples");

        while (isCollectingData && currentDataIndex < datasetSize)
        {
            // Apply domain randomization
            ApplyDomainRandomization();

            // Capture data with all required annotations
            CaptureAndSaveData();

            currentDataIndex++;

            // Wait for next capture
            yield return new WaitForSeconds(captureInterval);

            // Log progress
            if (currentDataIndex % 100 == 0)
            {
                Debug.Log($"Captured {currentDataIndex}/{datasetSize} samples");
            }
        }

        Debug.Log($"Dataset collection completed. Captured {currentDataIndex} samples.");
        isCollectingData = false;
    }

    void ApplyDomainRandomization()
    {
        // Randomize lighting conditions
        if (lightsToRandomize.Length > 0)
        {
            foreach (Light light in lightsToRandomize)
            {
                // Randomize light color
                if (environmentColors.Length > 0)
                {
                    light.color = environmentColors[Random.Range(0, environmentColors.Length)];
                }

                // Randomize light intensity
                light.intensity = Random.Range(0.5f, 2.0f);

                // Randomize light rotation for directional variety
                light.transform.rotation = Random.rotation;
            }
        }

        // Randomize materials
        if (materialOptions.Length > 0)
        {
            Renderer[] renderers = FindObjectsOfType<Renderer>();
            foreach (Renderer renderer in renderers)
            {
                if (renderer.CompareTag("Randomizable")) // Only randomize tagged objects
                {
                    Material randomMaterial = materialOptions[Random.Range(0, materialOptions.Length)];
                    renderer.material = randomMaterial;
                }
            }
        }

        // Randomize camera exposure
        if (exposureVariations.Length > 0)
        {
            float randomExposure = exposureVariations[Random.Range(0, exposureVariations.Length)];
            ApplyCameraExposure(randomExposure);
        }
    }

    void ApplyCameraExposure(float exposureValue)
    {
        Volume volume = dataCaptureCamera.GetComponent<Volume>();
        if (volume != null)
        {
            Exposure exposure;
            if (volume.profile.TryGet<Exposure>(out exposure))
            {
                exposure.m_PostExposure.value = exposureValue;
            }
        }
    }

    void CaptureAndSaveData()
    {
        // Capture RGB image
        Texture2D rgbImage = CaptureCameraImage(dataCaptureCamera);
        SaveImage(rgbImage, $"rgb_{currentDataIndex:D6}.png");

        // Capture depth map if enabled
        if (generateDepthMaps)
        {
            Texture2D depthImage = CaptureDepthMap();
            SaveImage(depthImage, $"depth_{currentDataIndex:D6}.png");
        }

        // Capture segmentation if enabled
        if (generateSegmentation)
        {
            Texture2D segImage = CaptureSegmentationMap();
            SaveImage(segImage, $"seg_{currentDataIndex:D6}.png");
        }

        // Capture optical flow if enabled
        if (generateOpticalFlow)
        {
            Texture2D flowImage = CaptureOpticalFlow();
            SaveImage(flowImage, $"flow_{currentDataIndex:D6}.png");
        }

        // Save metadata
        SaveMetadata();
    }

    Texture2D CaptureCameraImage(Camera cam)
    {
        // Render texture approach for HDRP
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture renderTexture = new RenderTexture(cam.pixelWidth, cam.pixelHeight, 24);

        cam.targetTexture = renderTexture;
        cam.Render();

        RenderTexture.active = renderTexture;
        Texture2D image = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        image.Apply();

        // Cleanup
        RenderTexture.active = currentRT;
        cam.targetTexture = null;
        Destroy(renderTexture);

        return image;
    }

    Texture2D CaptureDepthMap()
    {
        // In HDRP, depth can be captured using custom passes or post-processing
        // This is a simplified approach - real implementation would use HDRP's depth buffer
        Texture2D depthTexture = new Texture2D(dataCaptureCamera.pixelWidth, dataCaptureCamera.pixelHeight);
        // Fill with placeholder depth data
        Color[] colors = new Color[dataCaptureCamera.pixelWidth * dataCaptureCamera.pixelHeight];
        for (int i = 0; i < colors.Length; i++)
        {
            colors[i] = Color.gray; // Placeholder
        }
        depthTexture.SetPixels(colors);
        depthTexture.Apply();
        return depthTexture;
    }

    Texture2D CaptureSegmentationMap()
    {
        // For segmentation, you'd typically render objects with unique colors
        // or use semantic segmentation passes in HDRP
        Texture2D segTexture = new Texture2D(dataCaptureCamera.pixelWidth, dataCaptureCamera.pixelHeight);
        // Fill with placeholder segmentation data
        Color[] colors = new Color[dataCaptureCamera.pixelWidth * dataCaptureCamera.pixelHeight];
        for (int i = 0; i < colors.Length; i++)
        {
            colors[i] = Color.red; // Placeholder
        }
        segTexture.SetPixels(colors);
        segTexture.Apply();
        return segTexture;
    }

    Texture2D CaptureOpticalFlow()
    {
        // Optical flow capture requires previous frame comparison
        // This is complex in static captures but can be simulated
        Texture2D flowTexture = new Texture2D(dataCaptureCamera.pixelWidth, dataCaptureCamera.pixelHeight);
        Color[] colors = new Color[dataCaptureCamera.pixelWidth * dataCaptureCamera.pixelHeight];
        for (int i = 0; i < colors.Length; i++)
        {
            colors[i] = Color.black; // Placeholder
        }
        flowTexture.SetPixels(colors);
        flowTexture.Apply();
        return flowTexture;
    }

    void SaveImage(Texture2D image, string filename)
    {
        byte[] pngData = ImageConversion.EncodeToPNG(image);
        string fullPath = Path.Combine(datasetSavePath, filename);
        File.WriteAllBytes(fullPath, pngData);
        Destroy(image);
    }

    void SaveMetadata()
    {
        // Create metadata file for current sample
        string metadata = $"Sample Index: {currentDataIndex}\n" +
                         $"Timestamp: {System.DateTime.Now}\n" +
                         $"Camera Position: {dataCaptureCamera.transform.position}\n" +
                         $"Camera Rotation: {dataCaptureCamera.transform.rotation}\n" +
                         $"Lighting Conditions: [Randomized]\n";

        string metadataPath = Path.Combine(datasetSavePath, $"metadata_{currentDataIndex:D6}.txt");
        File.WriteAllText(metadataPath, metadata);
    }
}
```

## Best Practices for HDRP in Robotics

1. **Balance Quality and Performance**: Adjust settings based on required frame rate
2. **Use Appropriate Materials**: Configure PBR materials to match real robot surfaces
3. **Validate Physics Parameters**: Ensure HDRP physics match real robot behavior
4. **Optimize for Real-time**: Prioritize performance for interactive simulation
5. **Domain Randomization**: Use HDRP features for synthetic data diversity
6. **Proper Lighting Setup**: Configure lighting to match real-world conditions
7. **Camera Calibration**: Match Unity camera parameters to real sensors
8. **Test on Target Hardware**: Validate performance on deployment platform

## Next Steps

In the next chapter, we'll explore the ROS-Unity bridge in detail, learning how to establish robust communication between ROS 2 and Unity for real-time humanoid robot control and perception. This integration is crucial for creating effective digital twins that bridge simulation and reality.