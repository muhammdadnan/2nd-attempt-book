# Photorealistic Rendering

In this chapter, we'll explore NVIDIA Isaac's photorealistic rendering capabilities, leveraging RTX ray tracing and advanced PBR materials to create highly realistic simulation environments. This is crucial for generating synthetic datasets that can train AI models to work effectively in real-world conditions.

## Understanding RTX Ray Tracing in Isaac

### RTX Rendering Pipeline

NVIDIA Isaac Sim leverages RTX ray tracing technology to achieve photorealistic rendering:

- **Real-time Ray Tracing**: Accurate lighting, shadows, and reflections
- **Physically Based Rendering (PBR)**: Materials that respond realistically to light
- **Global Illumination**: Light bouncing and color bleeding for realistic lighting
- **Advanced Shading**: Complex surface interactions and subsurface scattering

### Ray Tracing vs Traditional Rendering

| Feature | Traditional Rasterization | RTX Ray Tracing |
|---------|---------------------------|-----------------|
| Reflections | Approximate (Screen Space) | Accurate (Ray-based) |
| Refractions | Limited | Physically accurate |
| Global Illumination | Baked/Precomputed | Real-time |
| Shadows | Shadow mapping | Ray-traced shadows |
| Performance | High FPS | Moderate FPS, High quality |

## Isaac Sim Rendering Configuration

### High-Definition Render Setup

```python
# rt_rendering_setup.py
import omni
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
import carb

class RTXRenderingSetup:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()

    def configure_rtx_settings(self):
        """
        Configure RTX rendering settings for photorealistic output
        """
        # Set rendering settings for RTX
        set_carb_setting("rtx-defaults", "renderMode", "RaytracedLightmap")
        set_carb_setting("rtx-defaults", "enableAccumulation", True)
        set_carb_setting("rtx-defaults", "maxSurfaceBounces", 8)
        set_carb_setting("rtx-defaults", "enableDenoising", True)

        # Configure path tracing settings
        set_carb_setting("rtx-defaults", "pathTracing.enable", True)
        set_carb_setting("rtx-defaults", "pathTracing.samplesPerPixel", 16)
        set_carb_setting("rtx-defaults", "pathTracing.maxBounces", 10)

        carb.log_info("RTX rendering settings configured")

    def create_hdri_environment(self, hdri_path=None):
        """
        Create photorealistic environment with HDRI lighting
        """
        # Create dome light with HDRI
        dome_light_path = "/World/DomeLight"
        dome_light = UsdGeom.DomeLight.Define(self.stage, dome_light_path)

        if hdri_path:
            # Use custom HDRI
            dome_light.CreateTextureFileAttr(hdri_path)
        else:
            # Use procedural environment
            dome_light.CreateTextureFileAttr("builtin://Assets/Isaac/Textures/sky.hdr")

        # Configure dome light properties
        dome_light.GetColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
        dome_light.GetIntensityAttr().Set(3.0)  # Adjust for realistic lighting
        dome_light.GetEnableColorTemperatureAttr().Set(True)
        dome_light.GetColorTemperatureAttr().Set(6500)  # Daylight

        carb.log_info(f"HDRI environment created: {dome_light_path}")

    def configure_advanced_materials(self):
        """
        Configure advanced PBR materials for realistic surfaces
        """
        # Create various material types for realistic rendering
        self._create_metal_materials()
        self._create_plastic_materials()
        self._create_glass_materials()
        self._create_organic_materials()

    def _create_metal_materials(self):
        """
        Create realistic metal materials
        """
        materials = [
            ("brushed_steel", 1.0, 0.1, 0.9),      # Metallic, Roughness, Specular
            ("aluminum", 0.95, 0.2, 1.0),
            ("copper", 0.98, 0.3, 0.8),
            ("gold", 1.0, 0.2, 0.9)
        ]

        for name, metallic, roughness, specular in materials:
            self._create_pbr_material(
                f"/World/Looks/{name}_material",
                base_color=(0.7, 0.7, 0.7),
                metallic=metallic,
                roughness=roughness,
                specular=specular
            )

    def _create_plastic_materials(self):
        """
        Create realistic plastic materials
        """
        materials = [
            ("matte_plastic", 0.0, 0.8, 0.5),
            ("glossy_plastic", 0.0, 0.2, 0.8),
            ("translucent_plastic", 0.0, 0.4, 0.6)
        ]

        for name, metallic, roughness, specular in materials:
            self._create_pbr_material(
                f"/World/Looks/{name}_material",
                base_color=(0.8, 0.8, 0.8),
                metallic=metallic,
                roughness=roughness,
                specular=specular
            )

    def _create_glass_materials(self):
        """
        Create realistic glass and transparent materials
        """
        materials = [
            ("clear_glass", 0.0, 0.05, 1.0, 0.95, True),
            ("tinted_glass", 0.0, 0.1, 1.0, 0.8, True),
            ("frosted_glass", 0.0, 0.3, 0.8, 0.7, True)
        ]

        for name, metallic, roughness, specular, opacity, transparent in materials:
            self._create_pbr_material(
                f"/World/Looks/{name}_material",
                base_color=(0.9, 0.95, 1.0),
                metallic=metallic,
                roughness=roughness,
                specular=specular,
                opacity=opacity,
                transparent=transparent
            )

    def _create_organic_materials(self):
        """
        Create organic materials like skin, fabric, etc.
        """
        materials = [
            ("human_skin", (0.9, 0.6, 0.5), 0.0, 0.4, 0.3, True),  # Subsurface
            ("fabric", (0.8, 0.7, 0.6), 0.0, 0.7, 0.2, False),
            ("wood", (0.6, 0.4, 0.2), 0.0, 0.5, 0.3, False)
        ]

        for name, color, metallic, roughness, specular, subsurface in materials:
            self._create_pbr_material(
                f"/World/Looks/{name}_material",
                base_color=color,
                metallic=metallic,
                roughness=roughness,
                specular=specular,
                subsurface=subsurface
            )

    def _create_pbr_material(self, material_path, base_color=(0.8, 0.8, 0.8),
                           metallic=0.0, roughness=0.4, specular=0.5,
                           opacity=1.0, transparent=False, subsurface=False):
        """
        Create a PBR material with realistic properties
        """
        # Create material
        material = UsdShade.Material.Define(self.stage, material_path)

        # Create and configure shader
        shader = UsdShade.Shader.Define(self.stage, f"{material_path}/Shader")
        shader.SetId("OmniPBR")

        # Set PBR properties
        shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(base_color)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(specular)
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)

        # Configure advanced properties
        if subsurface:
            shader.CreateInput("subsurface", Sdf.ValueTypeNames.Float).Set(0.1)
            shader.CreateInput("subsurface_color", Sdf.ValueTypeNames.Color3f).Set((0.9, 0.6, 0.6))

        if transparent:
            shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
            shader.CreateInput("enable_transparency", Sdf.ValueTypeNames.Bool).Set(True)

        # Connect shader to material
        material.GetSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "out")

        carb.log_info(f"Created PBR material: {material_path}")

    def setup_advanced_lights(self):
        """
        Set up advanced lighting for photorealistic rendering
        """
        # Create key light (main light source)
        key_light_path = "/World/KeyLight"
        key_light = UsdGeom.DistantLight.Define(self.stage, key_light_path)
        key_light.GetColorAttr().Set(Gf.Vec3f(1.0, 0.98, 0.9))
        key_light.GetIntensityAttr().Set(500.0)
        key_light.GetAngleAttr().Set(0.5)

        # Position the key light
        key_light_xform = UsdGeom.Xformable(key_light.GetPrim())
        key_light_xform.AddRotateXYZOp().Set((45, 45, 0))

        # Create fill light
        fill_light_path = "/World/FillLight"
        fill_light = UsdGeom.DistantLight.Define(self.stage, fill_light_path)
        fill_light.GetColorAttr().Set(Gf.Vec3f(0.8, 0.85, 1.0))
        fill_light.GetIntensityAttr().Set(150.0)

        fill_light_xform = UsdGeom.Xformable(fill_light.GetPrim())
        fill_light_xform.AddRotateXYZOp().Set((135, -30, 0))

        # Create rim light
        rim_light_path = "/World/RimLight"
        rim_light = UsdGeom.DistantLight.Define(self.stage, rim_light_path)
        rim_light.GetColorAttr().Set(Gf.Vec3f(1.0, 1.0, 0.9))
        rim_light.GetIntensityAttr().Set(200.0)

        rim_light_xform = UsdGeom.Xformable(rim_light.GetPrim())
        rim_light_xform.AddRotateXYZOp().Set((-45, 150, 0))

        carb.log_info("Advanced lighting setup completed")

def configure_photorealistic_rendering():
    """
    Complete setup for photorealistic rendering in Isaac Sim
    """
    renderer = RTXRenderingSetup()
    renderer.configure_rtx_settings()
    renderer.create_hdri_environment()
    renderer.configure_advanced_materials()
    renderer.setup_advanced_lights()

    carb.log_info("Photorealistic rendering configured successfully")
```

## Synthetic Data Generation Pipeline

### Photorealistic Dataset Creation

```python
# synthetic_data_generator.py
import omni
from pxr import Usd, UsdGeom, Gf
import numpy as np
import cv2
import os
from PIL import Image
import carb

class SyntheticDataGenerator:
    def __init__(self, robot_prim_path, dataset_path="synthetic_data"):
        self.robot_prim_path = robot_prim_path
        self.dataset_path = dataset_path
        self.stage = omni.usd.get_context().get_stage()

        # Create dataset directories
        os.makedirs(f"{dataset_path}/rgb", exist_ok=True)
        os.makedirs(f"{dataset_path}/depth", exist_ok=True)
        os.makedirs(f"{dataset_path}/seg", exist_ok=True)
        os.makedirs(f"{dataset_path}/meta", exist_ok=True)

        # Initialize domain randomization parameters
        self.lighting_conditions = []
        self.material_variations = []
        self.camera_positions = []

    def setup_domain_randomization(self):
        """
        Set up domain randomization for synthetic dataset generation
        """
        # Randomize lighting conditions
        self.lighting_conditions = [
            {"intensity": 300, "color": (1.0, 0.9, 0.8), "position": (45, 45, 0)},
            {"intensity": 800, "color": (0.9, 1.0, 1.0), "position": (-30, 60, 0)},
            {"intensity": 200, "color": (1.0, 1.0, 0.9), "position": (120, -20, 0)},
        ]

        # Randomize material properties
        self.material_variations = [
            {"roughness_range": (0.1, 0.3), "metallic_range": (0.8, 1.0)},
            {"roughness_range": (0.4, 0.7), "metallic_range": (0.0, 0.2)},
            {"roughness_range": (0.2, 0.5), "metallic_range": (0.3, 0.6)},
        ]

        # Randomize camera positions
        self.camera_positions = [
            {"position": (2, 0, 1), "rotation": (0, 30, 0)},
            {"position": (-1, 2, 1.5), "rotation": (-15, -45, 0)},
            {"position": (0, -3, 2), "rotation": (10, 90, 0)},
        ]

        carb.log_info("Domain randomization setup completed")

    def randomize_environment(self, sample_idx):
        """
        Randomize environment for current sample
        """
        # Randomize lighting
        light_idx = sample_idx % len(self.lighting_conditions)
        light_config = self.lighting_conditions[light_idx]

        # Update key light
        key_light_prim = self.stage.GetPrimAtPath("/World/KeyLight")
        if key_light_prim.IsValid():
            key_light = UsdGeom.DistantLight(key_light_prim)
            key_light.GetIntensityAttr().Set(light_config["intensity"])
            key_light.GetColorAttr().Set(Gf.Vec3f(*light_config["color"]))

        # Randomize materials
        material_idx = sample_idx % len(self.material_variations)
        material_config = self.material_variations[material_idx]

        # Apply material randomization to robot
        self._randomize_robot_materials(material_config)

        # Randomize camera
        camera_idx = sample_idx % len(self.camera_positions)
        camera_config = self.camera_positions[camera_idx]

        # Update camera position (assuming camera exists)
        camera_prim = self.stage.GetPrimAtPath(f"{self.robot_prim_path}/head/head_camera")
        if camera_prim.IsValid():
            camera_xform = UsdGeom.Xformable(camera_prim)
            camera_xform.AddTranslateOp().Set(Gf.Vec3f(*camera_config["position"]))

        carb.log_info(f"Environment randomized for sample {sample_idx}")

    def _randomize_robot_materials(self, material_config):
        """
        Randomize robot materials within specified ranges
        """
        # Get all material prims
        for prim in self.stage.TraverseAll():
            if prim.GetTypeName() == "Material":
                shader_prim = prim.GetChild("Shader")
                if shader_prim.IsValid():
                    shader = UsdShade.Shader(shader_prim)

                    # Randomize roughness
                    roughness_range = material_config["roughness_range"]
                    random_roughness = np.random.uniform(*roughness_range)
                    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(random_roughness)

                    # Randomize metallic
                    metallic_range = material_config["metallic_range"]
                    random_metallic = np.random.uniform(*metallic_range)
                    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(random_metallic)

    def capture_synthetic_data(self, sample_idx):
        """
        Capture RGB, depth, and segmentation data for current sample
        """
        # Ensure rendering is complete
        omni.kit.app.get_app().update()

        # Capture RGB image
        rgb_image = self._capture_rgb_image()

        # Capture depth image
        depth_image = self._capture_depth_image()

        # Capture segmentation
        seg_image = self._capture_segmentation_image()

        # Save images
        self._save_images(sample_idx, rgb_image, depth_image, seg_image)

        # Save metadata
        self._save_metadata(sample_idx)

        carb.log_info(f"Synthetic data captured for sample {sample_idx}")

    def _capture_rgb_image(self):
        """
        Capture RGB image from robot camera
        """
        # In Isaac Sim, this would involve using the Isaac Sim capture API
        # For demonstration, we'll return a placeholder
        import numpy as np
        # This would be replaced with actual Isaac Sim capture
        return np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    def _capture_depth_image(self):
        """
        Capture depth image from robot camera
        """
        # In Isaac Sim, this would use depth sensor
        import numpy as np
        # This would be replaced with actual Isaac Sim depth capture
        return np.random.uniform(0.1, 10.0, (480, 640)).astype(np.float32)

    def _capture_segmentation_image(self):
        """
        Capture segmentation image
        """
        import numpy as np
        # This would be replaced with actual Isaac Sim segmentation
        return np.random.randint(0, 10, (480, 640), dtype=np.uint8)

    def _save_images(self, sample_idx, rgb, depth, seg):
        """
        Save captured images to dataset directory
        """
        # Save RGB image
        rgb_pil = Image.fromarray(rgb)
        rgb_pil.save(f"{self.dataset_path}/rgb/{sample_idx:06d}.png")

        # Save depth image (as 16-bit PNG for precision)
        depth_normalized = ((depth - depth.min()) / (depth.max() - depth.min()) * 65535).astype(np.uint16)
        depth_pil = Image.fromarray(depth_normalized)
        depth_pil.save(f"{self.dataset_path}/depth/{sample_idx:06d}.png")

        # Save segmentation image
        seg_pil = Image.fromarray(seg)
        seg_pil.save(f"{self.dataset_path}/seg/{sample_idx:06d}.png")

    def _save_metadata(self, sample_idx):
        """
        Save metadata for current sample
        """
        import json

        metadata = {
            "sample_index": sample_idx,
            "timestamp": carb.events.get_time(),
            "lighting_condition": self.lighting_conditions[sample_idx % len(self.lighting_conditions)],
            "material_variation": self.material_variations[sample_idx % len(self.material_variations)],
            "camera_position": self.camera_positions[sample_idx % len(self.camera_positions)]
        }

        with open(f"{self.dataset_path}/meta/{sample_idx:06d}.json", 'w') as f:
            json.dump(metadata, f, indent=2)

    def generate_dataset(self, num_samples=1000):
        """
        Generate complete synthetic dataset
        """
        carb.log_info(f"Starting synthetic dataset generation: {num_samples} samples")

        for i in range(num_samples):
            # Randomize environment
            self.randomize_environment(i)

            # Capture data
            self.capture_synthetic_data(i)

            # Log progress
            if (i + 1) % 100 == 0:
                carb.log_info(f"Generated {i + 1}/{num_samples} samples")

        carb.log_info(f"Dataset generation completed: {num_samples} samples saved to {self.dataset_path}")

def generate_photorealistic_dataset(robot_prim_path, num_samples=1000):
    """
    Generate photorealistic synthetic dataset for robot perception training
    """
    generator = SyntheticDataGenerator(robot_prim_path)
    generator.setup_domain_randomization()
    generator.generate_dataset(num_samples)

    carb.log_info(f"Photorealistic dataset generated: {num_samples} samples")
```

## Advanced Material Systems

### PBR Material Configuration for Robotics

```python
# advanced_materials.py
from pxr import UsdShade, Sdf, Gf
import carb

class AdvancedRobotMaterials:
    def __init__(self, stage):
        self.stage = stage
        self.materials = {}

    def create_robot_surface_materials(self):
        """
        Create specialized materials for different robot surface types
        """
        # Metal surfaces (joints, structural components)
        self.create_metal_surface_material("structural_metal",
                                         base_color=(0.5, 0.5, 0.5),
                                         finish="brushed")

        # Plastic surfaces (body panels, covers)
        self.create_plastic_surface_material("body_plastic",
                                           base_color=(0.8, 0.8, 0.8),
                                           finish="textured")

        # Rubber surfaces (feet, grippers)
        self.create_rubber_material("gripper_rubber",
                                  base_color=(0.1, 0.1, 0.1),
                                  finish="matte")

        # Glass/sensor surfaces
        self.create_glass_material("sensor_cover",
                                 base_color=(0.9, 0.95, 1.0),
                                 transparency=0.8)

    def create_metal_surface_material(self, name, base_color, finish="polished"):
        """
        Create realistic metal surface material
        """
        material_path = f"/World/Looks/{name}_material"
        material = UsdShade.Material.Define(self.stage, material_path)

        # Create metallic shader
        shader = UsdShade.Shader.Define(self.stage, f"{material_path}/MetalShader")
        shader.SetId("OmniPBR")

        # Configure based on finish type
        if finish == "polished":
            shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(base_color)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.95)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.05)
            shader.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(1.0)
        elif finish == "brushed":
            shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(base_color)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.9)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.2)
            shader.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.9)
            # Add normal map for brushed effect
            shader.CreateInput("normalmap", Sdf.ValueTypeNames.Normal3f).Set(Gf.Vec3f(0, 0, 1))
        elif finish == "matte":
            shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(base_color)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.8)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
            shader.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.7)

        # Connect to material
        material.GetSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "out")

        self.materials[name] = material
        carb.log_info(f"Created metal material: {name} ({finish} finish)")

    def create_plastic_surface_material(self, name, base_color, finish="smooth"):
        """
        Create realistic plastic surface material
        """
        material_path = f"/World/Looks/{name}_material"
        material = UsdShade.Material.Define(self.stage, material_path)

        shader = UsdShade.Shader.Define(self.stage, f"{material_path}/PlasticShader")
        shader.SetId("OmniPBR")

        # Configure based on finish
        if finish == "smooth":
            shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(base_color)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.2)
            shader.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.8)
        elif finish == "textured":
            shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(base_color)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
            shader.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.6)
            # Add subtle bump map for texture
        elif finish == "matte":
            shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(base_color)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
            shader.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.3)

        # Add subsurface scattering for realistic plastic appearance
        shader.CreateInput("subsurface", Sdf.ValueTypeNames.Float).Set(0.1)
        shader.CreateInput("subsurface_color", Sdf.ValueTypeNames.Color3f).Set(base_color)

        # Connect to material
        material.GetSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "out")

        self.materials[name] = material
        carb.log_info(f"Created plastic material: {name} ({finish} finish)")

    def create_rubber_material(self, name, base_color, finish="grippy"):
        """
        Create realistic rubber material
        """
        material_path = f"/World/Looks/{name}_material"
        material = UsdShade.Material.Define(self.stage, material_path)

        shader = UsdShade.Shader.Define(self.stage, f"{material_path}/RubberShader")
        shader.SetId("OmniPBR")

        # Configure rubber properties
        shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(base_color)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
        shader.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.2)

        # Add subsurface scattering for realistic rubber appearance
        shader.CreateInput("subsurface", Sdf.ValueTypeNames.Float).Set(0.3)
        shader.CreateInput("subsurface_color", Sdf.ValueTypeNames.Color3f).Set((0.8, 0.2, 0.1))
        shader.CreateInput("subsurface_radius", Sdf.ValueTypeNames.Color3f).Set((1.0, 1.0, 1.0))

        # Connect to material
        material.GetSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "out")

        self.materials[name] = material
        carb.log_info(f"Created rubber material: {name} ({finish} finish)")

    def create_glass_material(self, name, base_color, transparency=0.9):
        """
        Create realistic glass/sensor cover material
        """
        material_path = f"/World/Looks/{name}_material"
        material = UsdShade.Material.Define(self.stage, material_path)

        shader = UsdShade.Shader.Define(self.stage, f"{material_path}/GlassShader")
        shader.SetId("OmniPBR")

        # Configure glass properties
        shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(base_color)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.05)
        shader.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(1.0)
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(transparency)
        shader.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)

        # Add refraction properties
        shader.CreateInput("refraction", Sdf.ValueTypeNames.Float).Set(1.0)
        shader.CreateInput("refraction_ior", Sdf.ValueTypeNames.Float).Set(1.52)  # Glass IOR

        # Connect to material
        material.GetSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "out")

        self.materials[name] = material
        carb.log_info(f"Created glass material: {name} (transparency: {transparency})")

    def apply_material_to_robot_parts(self, robot_prim_path):
        """
        Apply appropriate materials to different robot parts
        """
        material_mapping = {
            "base_link": "structural_metal",
            "torso": "body_plastic",
            "head": "body_plastic",
            "left_upper_arm": "structural_metal",
            "right_upper_arm": "structural_metal",
            "left_lower_arm": "body_plastic",
            "right_lower_arm": "body_plastic",
            "left_upper_leg": "structural_metal",
            "right_upper_leg": "structural_metal",
            "left_lower_leg": "body_plastic",
            "right_lower_leg": "body_plastic",
            "left_foot": "gripper_rubber",
            "right_foot": "gripper_rubber",
            "head_camera": "sensor_cover",
        }

        for link_name, material_name in material_mapping.items():
            if material_name in self.materials:
                link_path = f"{robot_prim_path}/{link_name}"
                self._apply_material_to_link(link_path, self.materials[material_name])

    def _apply_material_to_link(self, link_path, material):
        """
        Apply material to a specific robot link
        """
        link_prim = self.stage.GetPrimAtPath(link_path)
        if not link_prim.IsValid():
            carb.log_warn(f"Link prim not found: {link_path}")
            return

        # Find visual mesh in the link
        for child in link_prim.GetAllChildren():
            if any(keyword in child.GetName().lower() for keyword in ["visual", "mesh", "geometry"]):
                UsdShade.MaterialBindingAPI(child).Bind(material)
                carb.log_info(f"Applied material to: {child.GetPath()}")
                return

        # If no specific visual child found, apply to the link itself
        UsdShade.MaterialBindingAPI(link_prim).Bind(material)
        carb.log_info(f"Applied material to: {link_path}")

def setup_advanced_robot_materials(robot_prim_path):
    """
    Set up advanced materials for photorealistic robot rendering
    """
    stage = omni.usd.get_context().get_stage()
    material_manager = AdvancedRobotMaterials(stage)

    # Create all material types
    material_manager.create_robot_surface_materials()

    # Apply materials to robot parts
    material_manager.apply_material_to_robot_parts(robot_prim_path)

    carb.log_info("Advanced robot materials configured for photorealistic rendering")
```

## Camera Systems for Perception

### High-Quality Camera Configuration

```python
# camera_systems.py
import omni
from omni.isaac.sensor import Camera
from pxr import UsdGeom, Gf
import carb

class PerceptionCameraSystem:
    def __init__(self, robot_prim_path):
        self.robot_prim_path = robot_prim_path
        self.cameras = {}

    def setup_perception_cameras(self):
        """
        Set up multiple cameras for comprehensive perception
        """
        # Head-mounted RGB camera
        self.create_camera(
            name="head_rgb",
            link_name="head",
            position=(0.05, 0, 0.05),
            resolution=(640, 480),
            fov=60.0,
            sensor_type="rgb"
        )

        # Head-mounted depth camera
        self.create_camera(
            name="head_depth",
            link_name="head",
            position=(0.06, 0, 0.05),
            resolution=(640, 480),
            fov=60.0,
            sensor_type="depth"
        )

        # Chest-mounted wide-angle camera
        self.create_camera(
            name="chest_wide",
            link_name="torso",
            position=(0.1, 0, 0.1),
            resolution=(1280, 720),
            fov=90.0,
            sensor_type="rgb"
        )

        # Hand-mounted manipulation camera
        self.create_camera(
            name="left_hand_camera",
            link_name="left_lower_arm",
            position=(0, 0, -0.15),
            resolution=(320, 240),
            fov=45.0,
            sensor_type="rgb"
        )

    def create_camera(self, name, link_name, position, resolution, fov, sensor_type):
        """
        Create a perception camera with realistic properties
        """
        camera_path = f"{self.robot_prim_path}/{link_name}/{name}_camera"

        # Create camera using Isaac API
        camera = Camera(
            prim_path=camera_path,
            frequency=30 if sensor_type == "rgb" else 15,  # Depth can run slower
            resolution=resolution
        )

        # Set camera position
        camera.set_translation(position)

        # Configure camera properties for photorealistic rendering
        camera.set_fov(fov)

        # Store camera reference
        self.cameras[name] = camera

        carb.log_info(f"Created {sensor_type} camera: {name} at {position}")

    def configure_camera_for_photorealism(self, camera_name):
        """
        Configure camera for photorealistic rendering
        """
        if camera_name not in self.cameras:
            carb.log_error(f"Camera {camera_name} not found")
            return

        camera = self.cameras[camera_name]

        # Enable photorealistic effects
        # This would involve configuring Isaac Sim's camera settings
        # for realistic lens effects, chromatic aberration, etc.

        carb.log_info(f"Configured photorealistic settings for camera: {camera_name}")

    def setup_stereo_camera_pair(self, name_prefix, link_name, baseline=0.1):
        """
        Set up stereo camera pair for depth perception
        """
        # Left camera
        self.create_camera(
            name=f"{name_prefix}_left",
            link_name=link_name,
            position=(-baseline/2, 0, 0),
            resolution=(640, 480),
            fov=60.0,
            sensor_type="rgb"
        )

        # Right camera
        self.create_camera(
            name=f"{name_prefix}_right",
            link_name=link_name,
            position=(baseline/2, 0, 0),
            resolution=(640, 480),
            fov=60.0,
            sensor_type="rgb"
        )

        carb.log_info(f"Created stereo camera pair: {name_prefix}")

def setup_perception_camera_system(robot_prim_path):
    """
    Set up comprehensive camera system for robot perception
    """
    camera_system = PerceptionCameraSystem(robot_prim_path)
    camera_system.setup_perception_cameras()
    camera_system.setup_stereo_camera_pair("head", "head", 0.07)  # Human-like baseline

    carb.log_info("Perception camera system configured")
```

## Rendering Optimization

### Performance vs Quality Balance

```python
# rendering_optimization.py
import carb

class RenderingOptimizer:
    def __init__(self):
        self.performance_mode = "balanced"  # 'quality', 'performance', 'balanced'

    def set_rendering_quality_mode(self, mode):
        """
        Set rendering quality mode based on requirements
        """
        self.performance_mode = mode

        if mode == "quality":
            self._set_quality_settings()
        elif mode == "performance":
            self._set_performance_settings()
        else:  # balanced
            self._set_balanced_settings()

        carb.log_info(f"Rendering quality mode set to: {mode}")

    def _set_quality_settings(self):
        """
        Configure for maximum rendering quality
        """
        set_carb_setting("rtx-defaults", "renderMode", "RaytracedLightmap")
        set_carb_setting("rtx-defaults", "enableAccumulation", True)
        set_carb_setting("rtx-defaults", "pathTracing.samplesPerPixel", 64)
        set_carb_setting("rtx-defaults", "pathTracing.maxBounces", 10)
        set_carb_setting("rtx-defaults", "enableDenoising", True)
        set_carb_setting("rtx-defaults", "denoising.mode", "Optix")

    def _set_performance_settings(self):
        """
        Configure for maximum performance
        """
        set_carb_setting("rtx-defaults", "renderMode", "Hybrid")
        set_carb_setting("rtx-defaults", "enableAccumulation", False)
        set_carb_setting("rtx-defaults", "pathTracing.samplesPerPixel", 1)
        set_carb_setting("rtx-defaults", "pathTracing.maxBounces", 2)
        set_carb_setting("rtx-defaults", "enableDenoising", False)

    def _set_balanced_settings(self):
        """
        Configure for balanced quality and performance
        """
        set_carb_setting("rtx-defaults", "renderMode", "RaytracedLightmap")
        set_carb_setting("rtx-defaults", "enableAccumulation", True)
        set_carb_setting("rtx-defaults", "pathTracing.samplesPerPixel", 16)
        set_carb_setting("rtx-defaults", "pathTracing.maxBounces", 6)
        set_carb_setting("rtx-defaults", "enableDenoising", True)

    def optimize_for_training_data(self):
        """
        Optimize rendering specifically for synthetic training data generation
        """
        # For training data, we want good quality but reasonable performance
        self._set_balanced_settings()

        # Enable features important for training data
        set_carb_setting("rtx-defaults", "enableDenoising", True)
        set_carb_setting("rtx-defaults", "denoising.mode", "Optix")

        # Configure for consistent lighting across samples
        set_carb_setting("rtx-defaults", "lightLoop.enable", True)

    def optimize_for_real_time_control(self):
        """
        Optimize rendering for real-time robot control scenarios
        """
        # For real-time control, prioritize performance
        self._set_performance_settings()

        # Reduce quality settings for higher frame rates
        set_carb_setting("rtx-defaults", "maxSurfaceBounces", 4)
        set_carb_setting("rtx-defaults", "enableDenoising", False)

def optimize_rendering_for_use_case(use_case="training"):
    """
    Optimize rendering settings for specific use case
    """
    optimizer = RenderingOptimizer()

    if use_case == "training":
        optimizer.optimize_for_training_data()
        carb.log_info("Rendering optimized for training data generation")
    elif use_case == "control":
        optimizer.optimize_for_real_time_control()
        carb.log_info("Rendering optimized for real-time control")
    else:
        optimizer.set_rendering_quality_mode("balanced")
        carb.log_info("Rendering set to balanced mode")
```

## Best Practices

1. **Use appropriate materials**: Match real robot surface properties
2. **Configure lighting realistically**: Use HDRI and multiple light sources
3. **Balance quality and performance**: Adjust settings based on requirements
4. **Implement domain randomization**: Vary lighting, materials, and camera angles
5. **Validate synthetic data**: Compare with real sensor data when possible
6. **Optimize for your use case**: Training vs. real-time control have different needs
7. **Use proper camera configurations**: Match real robot sensor specifications

## Next Steps

With photorealistic rendering configured, you're ready to generate synthetic datasets for AI training. In the next chapter, we'll explore how to create comprehensive synthetic datasets with domain randomization and use them to train perception models for your humanoid robot.