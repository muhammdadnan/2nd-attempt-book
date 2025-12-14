# Synthetic Datasets

In this chapter, we'll explore how to generate comprehensive synthetic datasets using NVIDIA Isaac's photorealistic rendering capabilities. These datasets are crucial for training AI models that can work effectively with real-world robot perception systems, especially for humanoid robots operating in diverse environments.

## Understanding Synthetic Dataset Generation

### The Importance of Synthetic Data

Synthetic datasets provide several key advantages for robotics AI:

- **Safety**: Train models without risking physical robots
- **Diversity**: Generate scenarios that are difficult or dangerous to create in reality
- **Annotations**: Automatically generate perfect ground truth labels
- **Cost**: Reduce the need for expensive real-world data collection
- **Scalability**: Generate large datasets quickly and consistently

### Isaac's Synthetic Data Capabilities

NVIDIA Isaac provides advanced tools for synthetic dataset generation:

- **Photorealistic rendering**: RTX ray tracing for realistic images
- **Domain randomization**: Automatic variation of scene parameters
- **Multi-modal data**: RGB, depth, segmentation, and sensor data
- **Real-time generation**: High-throughput data creation
- **Annotation tools**: Automatic ground truth generation

## Dataset Structure and Organization

### Standard Dataset Format

```python
# dataset_structure.py
import os
import json
from datetime import datetime

class SyntheticDatasetStructure:
    def __init__(self, dataset_name, base_path="datasets"):
        self.dataset_name = dataset_name
        self.base_path = base_path
        self.dataset_path = os.path.join(base_path, dataset_name)

        # Create standard directory structure
        self.directories = {
            'rgb': os.path.join(self.dataset_path, 'rgb'),
            'depth': os.path.join(self.dataset_path, 'depth'),
            'segmentation': os.path.join(self.dataset_path, 'segmentation'),
            'normal': os.path.join(self.dataset_path, 'normal'),
            'optical_flow': os.path.join(self.dataset_path, 'optical_flow'),
            'annotations': os.path.join(self.dataset_path, 'annotations'),
            'metadata': os.path.join(self.dataset_path, 'metadata'),
            'calibration': os.path.join(self.dataset_path, 'calibration')
        }

        self._create_directories()
        self._create_dataset_config()

    def _create_directories(self):
        """Create the standard dataset directory structure"""
        for dir_path in self.directories.values():
            os.makedirs(dir_path, exist_ok=True)

    def _create_dataset_config(self):
        """Create dataset configuration file"""
        config = {
            'dataset_name': self.dataset_name,
            'creation_date': datetime.now().isoformat(),
            'total_samples': 0,
            'modalities': ['rgb', 'depth', 'segmentation', 'normal'],
            'image_resolution': [640, 480],
            'camera_parameters': {
                'fov': 60.0,
                'focal_length': 320.0,
                'principal_point': [320.0, 240.0]
            },
            'domain_randomization': {
                'lighting_conditions': True,
                'material_variations': True,
                'camera_poses': True,
                'backgrounds': True
            }
        }

        config_path = os.path.join(self.dataset_path, 'dataset_config.json')
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)

    def add_sample_metadata(self, sample_id, metadata):
        """Add metadata for a specific sample"""
        metadata_path = os.path.join(self.directories['metadata'], f'{sample_id:06d}.json')
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)

    def update_dataset_stats(self, total_samples):
        """Update dataset statistics"""
        config_path = os.path.join(self.dataset_path, 'dataset_config.json')
        with open(config_path, 'r') as f:
            config = json.load(f)

        config['total_samples'] = total_samples

        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)

def create_dataset_structure(dataset_name):
    """Create standard synthetic dataset structure"""
    return SyntheticDatasetStructure(dataset_name)
```

## Domain Randomization Implementation

### Advanced Domain Randomization

```python
# domain_randomization.py
import numpy as np
import random
from pxr import Gf, Usd, UsdGeom
import carb

class DomainRandomizer:
    def __init__(self, stage):
        self.stage = stage
        self.randomization_config = self._get_default_config()

    def _get_default_config(self):
        """Get default domain randomization configuration"""
        return {
            'lighting': {
                'intensity_range': (100, 1000),
                'color_temperature_range': (3000, 8000),
                'direction_variation': 45.0,  # degrees
                'number_of_lights': (1, 4)
            },
            'materials': {
                'roughness_range': (0.0, 1.0),
                'metallic_range': (0.0, 1.0),
                'albedo_range': (0.1, 1.0),
                'normal_map_strength_range': (0.0, 1.0)
            },
            'geometry': {
                'scale_variation': 0.1,  # 10% scale variation
                'position_variation': 0.1,  # 0.1m position variation
                'rotation_variation': 15.0  # 15 degree rotation variation
            },
            'camera': {
                'position_variation': 0.2,  # 0.2m camera position variation
                'rotation_variation': 10.0,  # 10 degree camera rotation variation
                'fov_variation': 5.0  # 5 degree FOV variation
            },
            'backgrounds': {
                'environment_types': ['indoor', 'outdoor', 'warehouse', 'office'],
                'object_placement': True,
                'texture_randomization': True
            }
        }

    def randomize_lighting(self, sample_idx):
        """Randomize lighting conditions"""
        # Get all lights in the scene
        lights = []
        for prim in self.stage.TraverseAll():
            if prim.GetTypeName() in ['DistantLight', 'SphereLight', 'DiskLight']:
                lights.append(prim)

        if not lights:
            # Create default lights if none exist
            lights = self._create_default_lights()

        # Randomize each light
        for i, light_prim in enumerate(lights):
            if light_prim.GetTypeName() == 'DistantLight':
                light = UsdGeom.DistantLight(light_prim)

                # Randomize intensity
                intensity_range = self.randomization_config['lighting']['intensity_range']
                random_intensity = np.random.uniform(*intensity_range)
                light.GetIntensityAttr().Set(random_intensity)

                # Randomize color temperature
                temp_range = self.randomization_config['lighting']['color_temperature_range']
                random_temp = np.random.uniform(*temp_range)
                light.GetColorTemperatureAttr().Set(random_temp)

                # Randomize direction (for sample-specific variation)
                variation = self.randomization_config['lighting']['direction_variation']
                random_rotation = (
                    np.random.uniform(-variation, variation),
                    np.random.uniform(-variation, variation),
                    np.random.uniform(-variation, variation)
                )

                light_xform = UsdGeom.Xformable(light_prim)
                light_xform.AddRotateXYZOp().Set(random_rotation)

    def _create_default_lights(self):
        """Create default lighting setup"""
        # Create key light
        key_light = UsdGeom.DistantLight.Define(self.stage, "/World/KeyLight")
        key_light.GetIntensityAttr().Set(500.0)
        key_light.GetColorTemperatureAttr().Set(6500)

        # Create fill light
        fill_light = UsdGeom.DistantLight.Define(self.stage, "/World/FillLight")
        fill_light.GetIntensityAttr().Set(200.0)
        fill_light.GetColorTemperatureAttr().Set(5000)

        return [key_light.GetPrim(), fill_light.GetPrim()]

    def randomize_materials(self, robot_prim_path, sample_idx):
        """Randomize materials for the robot"""
        from pxr import UsdShade

        # Traverse robot links and randomize materials
        robot_prim = self.stage.GetPrimAtPath(robot_prim_path)
        if not robot_prim.IsValid():
            carb.log_error(f"Robot prim not found: {robot_prim_path}")
            return

        for link_prim in robot_prim.GetAllChildren():
            if link_prim.GetName().endswith("_link"):
                # Find material on this link
                for child in link_prim.GetAllChildren():
                    if child.GetTypeName() == "Material":
                        self._randomize_material(child, sample_idx)

    def _randomize_material(self, material_prim, sample_idx):
        """Randomize a specific material"""
        from pxr import UsdShade, Sdf

        material = UsdShade.Material(material_prim)
        shader = material.ComputeSurfaceSource()[0]  # Get the surface shader

        if shader and shader.GetId() == "OmniPBR":
            # Randomize PBR properties
            config = self.randomization_config['materials']

            # Randomize roughness
            roughness = np.random.uniform(*config['roughness_range'])
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)

            # Randomize metallic
            metallic = np.random.uniform(*config['metallic_range'])
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)

            # Randomize albedo
            albedo_min, albedo_max = config['albedo_range']
            albedo = (
                np.random.uniform(albedo_min, albedo_max),
                np.random.uniform(albedo_min, albedo_max),
                np.random.uniform(albedo_min, albedo_max)
            )
            shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*albedo))

    def randomize_background(self, sample_idx):
        """Randomize background environment"""
        # This would involve changing environment maps, adding objects, etc.
        # For Isaac Sim, this might involve changing the stage's background

        # Example: Randomize environment map
        dome_light_prim = self.stage.GetPrimAtPath("/World/DomeLight")
        if dome_light_prim.IsValid():
            dome_light = UsdGeom.DomeLight(dome_light_prim)

            # Cycle through different environment types
            env_types = self.randomization_config['backgrounds']['environment_types']
            env_idx = sample_idx % len(env_types)
            env_type = env_types[env_idx]

            # In a real implementation, you would load different HDRI maps
            # based on the environment type
            carb.log_info(f"Background environment set to: {env_type}")

    def randomize_camera(self, camera_prim_path, sample_idx):
        """Randomize camera parameters"""
        camera_prim = self.stage.GetPrimAtPath(camera_prim_path)
        if not camera_prim.IsValid():
            carb.log_error(f"Camera prim not found: {camera_prim_path}")
            return

        # Randomize camera position and orientation
        config = self.randomization_config['camera']

        # Get current transform
        camera_xform = UsdGeom.Xformable(camera_prim)

        # Apply random position offset
        pos_offset = (
            np.random.uniform(-config['position_variation'], config['position_variation']),
            np.random.uniform(-config['position_variation'], config['position_variation']),
            np.random.uniform(-config['position_variation']/2, config['position_variation']/2)  # Less Z variation
        )

        # Apply random rotation offset
        rot_offset = (
            np.random.uniform(-config['rotation_variation'], config['rotation_variation']),
            np.random.uniform(-config['rotation_variation'], config['rotation_variation']),
            np.random.uniform(-config['rotation_variation'], config['rotation_variation'])
        )

        # In a real implementation, you would apply these transformations
        # camera_xform.AddTranslateOp().Set(Gf.Vec3f(*pos_offset))
        # camera_xform.AddRotateXYZOp().Set(Gf.Vec3f(*rot_offset))

def setup_domain_randomization():
    """Set up domain randomization for synthetic dataset generation"""
    stage = omni.usd.get_context().get_stage()
    return DomainRandomizer(stage)
```

## Data Capture and Annotation Pipeline

### Multi-Modal Data Capture

```python
# data_capture_pipeline.py
import omni
from omni.isaac.sensor import Camera
from pxr import Usd, UsdGeom
import numpy as np
import cv2
from PIL import Image
import json
import os
import carb

class DataCapturePipeline:
    def __init__(self, dataset_structure):
        self.dataset = dataset_structure
        self.sample_counter = 0
        self.stage = omni.usd.get_context().get_stage()

    def capture_multimodal_data(self, sample_id, camera_configs):
        """
        Capture multi-modal data for a single sample
        """
        sample_data = {}

        for cam_name, cam_config in camera_configs.items():
            # Capture different modalities
            sample_data[cam_name] = {
                'rgb': self._capture_rgb(cam_config),
                'depth': self._capture_depth(cam_config),
                'segmentation': self._capture_segmentation(cam_config),
                'normal': self._capture_normal(cam_config)
            }

        # Save captured data
        self._save_multimodal_data(sample_id, sample_data)

        return sample_data

    def _capture_rgb(self, camera_config):
        """Capture RGB image"""
        # In Isaac Sim, this would use the actual camera capture API
        # For demonstration, we'll create a placeholder
        width, height = camera_config['resolution']
        rgb_image = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
        return rgb_image

    def _capture_depth(self, camera_config):
        """Capture depth image"""
        width, height = camera_config['resolution']
        # Generate realistic depth data (0.1m to 10m range)
        depth_image = np.random.uniform(0.1, 10.0, (height, width)).astype(np.float32)
        return depth_image

    def _capture_segmentation(self, camera_config):
        """Capture segmentation mask"""
        width, height = camera_config['resolution']
        # Generate segmentation with different object classes
        segmentation = np.random.randint(0, 10, (height, width), dtype=np.uint8)
        return segmentation

    def _capture_normal(self, camera_config):
        """Capture surface normal map"""
        width, height = camera_config['resolution']
        # Generate normal map (X, Y, Z components in range [-1, 1])
        normal_map = np.random.uniform(-1, 1, (height, width, 3)).astype(np.float32)
        # Normalize vectors
        norms = np.linalg.norm(normal_map, axis=2, keepdims=True)
        normal_map = normal_map / (norms + 1e-8)  # Avoid division by zero
        return normal_map

    def _save_multimodal_data(self, sample_id, sample_data):
        """Save multi-modal data to dataset structure"""
        for cam_name, modalities in sample_data.items():
            for modality, data in modalities.items():
                if modality == 'rgb':
                    # Save RGB as PNG
                    img = Image.fromarray(data)
                    img.save(f"{self.dataset.directories[modality]}/{sample_id:06d}_{cam_name}.png")
                elif modality == 'depth':
                    # Save depth as 16-bit PNG for precision
                    depth_normalized = ((data - data.min()) / (data.max() - data.min()) * 65535).astype(np.uint16)
                    img = Image.fromarray(depth_normalized)
                    img.save(f"{self.dataset.directories[modality]}/{sample_id:06d}_{cam_name}.png")
                elif modality == 'segmentation':
                    # Save segmentation as PNG
                    img = Image.fromarray(data)
                    img.save(f"{self.dataset.directories['segmentation']}/{sample_id:06d}_{cam_name}.png")
                elif modality == 'normal':
                    # Save normal map as PNG (convert from [-1,1] to [0,255])
                    normal_normalized = ((data + 1) / 2 * 255).astype(np.uint8)
                    img = Image.fromarray(normal_normalized)
                    img.save(f"{self.dataset.directories['normal']}/{sample_id:06d}_{cam_name}.png")

    def generate_annotations(self, sample_id, segmentation_data):
        """Generate annotations from segmentation data"""
        annotations = {}

        for cam_name, seg_map in segmentation_data.items():
            # Analyze segmentation to create object annotations
            unique_labels = np.unique(seg_map)

            objects = []
            for label in unique_labels:
                if label == 0:  # Skip background
                    continue

                # Find object mask
                mask = (seg_map == label).astype(np.uint8)

                # Calculate bounding box
                coords = np.where(mask)
                if len(coords[0]) > 0:  # Check if object exists in frame
                    y_min, y_max = coords[0].min(), coords[0].max()
                    x_min, x_max = coords[1].min(), coords[1].max()

                    # Calculate center and area
                    center_x = (x_min + x_max) / 2
                    center_y = (y_min + y_max) / 2
                    area = np.sum(mask)

                    objects.append({
                        'label': int(label),
                        'bbox': [int(x_min), int(y_min), int(x_max), int(y_max)],
                        'center': [float(center_x), float(center_y)],
                        'area': int(area),
                        'pixel_count': int(area)
                    })

            annotations[cam_name] = objects

        # Save annotations
        annotations_path = os.path.join(self.dataset.directories['annotations'], f'{sample_id:06d}.json')
        with open(annotations_path, 'w') as f:
            json.dump(annotations, f, indent=2)

        return annotations

    def capture_sample_metadata(self, sample_id, domain_randomizer, robot_state):
        """Capture metadata for current sample"""
        metadata = {
            'sample_id': sample_id,
            'timestamp': carb.events.get_time(),
            'domain_randomization': {
                'lighting': domain_randomizer.randomization_config['lighting'],
                'materials': domain_randomizer.randomization_config['materials'],
                'geometry': domain_randomizer.randomization_config['geometry'],
                'camera': domain_randomizer.randomization_config['camera'],
                'backgrounds': domain_randomizer.randomization_config['backgrounds']
            },
            'robot_state': robot_state,
            'camera_configurations': self.get_camera_configs(),
            'environment_state': self.get_environment_state()
        }

        self.dataset.add_sample_metadata(sample_id, metadata)
        return metadata

    def get_camera_configs(self):
        """Get current camera configurations"""
        # This would return actual camera parameters
        return {
            'head_rgb': {
                'resolution': [640, 480],
                'fov': 60.0,
                'position': [0.05, 0, 0.05],
                'type': 'rgb'
            }
        }

    def get_environment_state(self):
        """Get current environment state"""
        # This would capture actual environment parameters
        return {
            'lighting_conditions': 'randomized',
            'background': 'randomized',
            'objects_present': 'varied'
        }

def create_data_capture_pipeline(dataset_structure):
    """Create data capture pipeline for synthetic dataset generation"""
    return DataCapturePipeline(dataset_structure)
```

## Advanced Dataset Generation Techniques

### Procedural Scene Generation

```python
# procedural_scene_generation.py
import numpy as np
import random
from pxr import Usd, UsdGeom, Gf
import carb

class ProceduralSceneGenerator:
    def __init__(self, stage):
        self.stage = stage
        self.scene_objects = []
        self.scene_types = [
            'indoor_office',
            'indoor_warehouse',
            'outdoor_urban',
            'outdoor_park',
            'indoor_home'
        ]

    def generate_scene(self, scene_type, sample_idx):
        """Generate a procedural scene based on type"""
        if scene_type not in self.scene_types:
            scene_type = random.choice(self.scene_types)

        # Clear previous scene objects
        self._clear_scene_objects()

        # Generate scene based on type
        if scene_type == 'indoor_office':
            self._generate_office_scene()
        elif scene_type == 'indoor_warehouse':
            self._generate_warehouse_scene()
        elif scene_type == 'outdoor_urban':
            self._generate_urban_scene()
        elif scene_type == 'outdoor_park':
            self._generate_park_scene()
        elif scene_type == 'indoor_home':
            self._generate_home_scene()

        carb.log_info(f"Generated {scene_type} scene for sample {sample_idx}")

    def _clear_scene_objects(self):
        """Clear previously generated scene objects"""
        # Remove objects in the scene objects collection
        for obj_path in self.scene_objects:
            obj_prim = self.stage.GetPrimAtPath(obj_path)
            if obj_prim.IsValid():
                self.stage.RemovePrim(obj_path)

        self.scene_objects = []

    def _generate_office_scene(self):
        """Generate an office environment"""
        # Create floor
        floor = UsdGeom.Cube.Define(self.stage, "/World/OfficeFloor")
        floor.GetSizeAttr().Set(20.0)
        floor.GetXformOp().Set(Gf.Vec3d(0, 0, -0.5))

        # Add office furniture
        self._add_office_furniture()

        # Add office objects
        self._add_office_objects()

    def _add_office_furniture(self):
        """Add office furniture to the scene"""
        furniture_types = ['desk', 'chair', 'shelf', 'plant']

        for i in range(10):  # Add 10 furniture items
            furn_type = random.choice(furniture_types)
            x = np.random.uniform(-8, 8)
            y = np.random.uniform(-8, 8)
            z = 0

            if furn_type == 'desk':
                furn = UsdGeom.Cube.Define(self.stage, f"/World/Desk_{i}")
                furn.GetSizeAttr().Set(1.5)
                furn.GetXformOp().Set(Gf.Vec3d(x, y, 0.3))
            elif furn_type == 'chair':
                furn = UsdGeom.Cylinder.Define(self.stage, f"/World/Chair_{i}")
                furn.GetRadiusAttr().Set(0.3)
                furn.GetHeightAttr().Set(0.8)
                furn.GetXformOp().Set(Gf.Vec3d(x, y, 0.4))
            elif furn_type == 'shelf':
                furn = UsdGeom.Cube.Define(self.stage, f"/World/Shelf_{i}")
                furn.GetSizeAttr().Set(1.0)
                furn.GetXformOp().Set(Gf.Vec3d(x, y, 0.75))
            elif furn_type == 'plant':
                furn = UsdGeom.Cylinder.Define(self.stage, f"/World/Plant_{i}")
                furn.GetRadiusAttr().Set(0.15)
                furn.GetHeightAttr().Set(0.6)
                furn.GetXformOp().Set(Gf.Vec3d(x, y, 0.3))

            self.scene_objects.append(f"/World/{furn_type.title()}_{i}")

    def _add_office_objects(self):
        """Add smaller office objects"""
        objects = ['laptop', 'cup', 'book', 'phone']

        for i in range(20):  # Add 20 small objects
            obj_type = random.choice(objects)
            x = np.random.uniform(-9, 9)
            y = np.random.uniform(-9, 9)
            z = np.random.uniform(0.3, 1.5)  # Various heights

            if obj_type == 'laptop':
                obj = UsdGeom.Cube.Define(self.stage, f"/World/Laptop_{i}")
                obj.GetSizeAttr().Set(0.3)
            elif obj_type == 'cup':
                obj = UsdGeom.Cylinder.Define(self.stage, f"/World/Cup_{i}")
                obj.GetRadiusAttr().Set(0.05)
                obj.GetHeightAttr().Set(0.1)
            elif obj_type == 'book':
                obj = UsdGeom.Cube.Define(self.stage, f"/World/Book_{i}")
                obj.GetSizeAttr().Set(0.15)
            elif obj_type == 'phone':
                obj = UsdGeom.Cube.Define(self.stage, f"/World/Phone_{i}")
                obj.GetSizeAttr().Set(0.08)

            obj.GetXformOp().Set(Gf.Vec3d(x, y, z))
            self.scene_objects.append(f"/World/{obj_type.title()}_{i}")

    def _generate_warehouse_scene(self):
        """Generate a warehouse environment"""
        # Create large floor
        floor = UsdGeom.Cube.Define(self.stage, "/World/WarehouseFloor")
        floor.GetSizeAttr().Set(50.0)
        floor.GetXformOp().Set(Gf.Vec3d(0, 0, -1.0))

        # Add warehouse elements
        self._add_warehouse_elements()

    def _add_warehouse_elements(self):
        """Add warehouse-specific elements"""
        # Create aisles with pallets
        for aisle_x in range(-20, 21, 4):
            for aisle_z in range(-20, 21, 4):
                # Add pallet
                pallet = UsdGeom.Cube.Define(self.stage, f"/World/Pallet_{aisle_x}_{aisle_z}")
                pallet.GetSizeAttr().Set(1.2)
                pallet.GetXformOp().Set(Gf.Vec3d(aisle_x, aisle_z, 0.2))
                self.scene_objects.append(f"/World/Pallet_{aisle_x}_{aisle_z}")

        # Add warehouse lighting
        for i in range(10):
            light_x = np.random.uniform(-20, 20)
            light_y = np.random.uniform(-20, 20)
            light_z = 8.0

            light = UsdGeom.DistantLight.Define(self.stage, f"/World/WarehouseLight_{i}")
            light.GetIntensityAttr().Set(800.0)
            light.GetXformOp().Set(Gf.Vec3d(light_x, light_y, light_z))
            self.scene_objects.append(f"/World/WarehouseLight_{i}")

    def _generate_urban_scene(self):
        """Generate an urban environment"""
        # Create ground plane
        ground = UsdGeom.Cube.Define(self.stage, "/World/UrbanGround")
        ground.GetSizeAttr().Set(100.0)
        ground.GetXformOp().Set(Gf.Vec3d(0, 0, -2.0))

        # Add urban elements
        self._add_urban_elements()

    def _add_urban_elements(self):
        """Add urban-specific elements"""
        # Create buildings
        for i in range(20):
            x = np.random.uniform(-40, 40)
            y = np.random.uniform(-40, 40)
            width = np.random.uniform(5, 15)
            height = np.random.uniform(10, 30)

            building = UsdGeom.Cube.Define(self.stage, f"/World/Building_{i}")
            building.GetSizeAttr().Set(10.0)  # Will be scaled
            # Apply custom scale
            building.GetXformOp().Set(Gf.Vec3d(x, y, height/2))
            self.scene_objects.append(f"/World/Building_{i}")

        # Add vehicles
        for i in range(5):
            x = np.random.uniform(-30, 30)
            y = np.random.uniform(-30, 30)

            car = UsdGeom.Cube.Define(self.stage, f"/World/Car_{i}")
            car.GetSizeAttr().Set(4.0)
            car.GetXformOp().Set(Gf.Vec3d(x, y, 0.7))
            self.scene_objects.append(f"/World/Car_{i}")

    def _generate_park_scene(self):
        """Generate a park environment"""
        # Create grass ground
        grass = UsdGeom.Cube.Define(self.stage, "/World/ParkGround")
        grass.GetSizeAttr().Set(80.0)
        grass.GetXformOp().Set(Gf.Vec3d(0, 0, -0.5))

        # Add park elements
        self._add_park_elements()

    def _add_park_elements(self):
        """Add park-specific elements"""
        # Create trees
        for i in range(15):
            x = np.random.uniform(-35, 35)
            y = np.random.uniform(-35, 35)

            # Tree trunk
            trunk = UsdGeom.Cylinder.Define(self.stage, f"/World/TreeTrunk_{i}")
            trunk.GetRadiusAttr().Set(0.3)
            trunk.GetHeightAttr().Set(4.0)
            trunk.GetXformOp().Set(Gf.Vec3d(x, y, 2.0))

            # Tree top
            top = UsdGeom.Sphere.Define(self.stage, f"/World/TreeTop_{i}")
            top.GetRadiusAttr().Set(2.0)
            top.GetXformOp().Set(Gf.Vec3d(x, y, 5.0))

            self.scene_objects.extend([f"/World/TreeTrunk_{i}", f"/World/TreeTop_{i}"])

        # Add park benches
        for i in range(8):
            x = np.random.uniform(-30, 30)
            y = np.random.uniform(-30, 30)

            bench = UsdGeom.Cube.Define(self.stage, f"/World/Bench_{i}")
            bench.GetSizeAttr().Set(2.0)
            bench.GetXformOp().Set(Gf.Vec3d(x, y, 0.3))
            self.scene_objects.append(f"/World/Bench_{i}")

    def _generate_home_scene(self):
        """Generate a home environment"""
        # Create room structure
        floor = UsdGeom.Cube.Define(self.stage, "/World/HomeFloor")
        floor.GetSizeAttr().Set(20.0)
        floor.GetXformOp().Set(Gf.Vec3d(0, 0, -0.5))

        # Add home furniture
        self._add_home_furniture()

    def _add_home_furniture(self):
        """Add home-specific furniture"""
        furniture = [
            ('sofa', (1.8, 0.8, 0.7)),
            ('table', (1.2, 0.6, 0.7)),
            ('tv', (1.5, 0.05, 1.0)),
            ('lamp', (0.2, 0.2, 1.5)),
            ('plant', (0.3, 0.3, 0.8))
        ]

        for i in range(12):
            furn_type, dimensions = random.choice(furniture)
            x = np.random.uniform(-8, 8)
            y = np.random.uniform(-8, 8)
            z = dimensions[2] / 2

            furn = UsdGeom.Cube.Define(self.stage, f"/World/Home_{furn_type.title()}_{i}")
            furn.GetSizeAttr().Set(2.0)  # Will be scaled
            furn.GetXformOp().Set(Gf.Vec3d(x, y, z))
            self.scene_objects.append(f"/World/Home_{furn_type.title()}_{i}")

def setup_procedural_scene_generator():
    """Set up procedural scene generation for synthetic datasets"""
    stage = omni.usd.get_context().get_stage()
    return ProceduralSceneGenerator(stage)
```

## Dataset Quality Assurance

### Data Validation and Quality Checks

```python
# quality_assurance.py
import numpy as np
from PIL import Image
import json
import os
import cv2
import carb

class DatasetQualityAssurance:
    def __init__(self, dataset_path):
        self.dataset_path = dataset_path
        self.quality_metrics = {}

    def validate_dataset_integrity(self):
        """Validate that all dataset files exist and are accessible"""
        integrity_issues = []

        # Check directory structure
        required_dirs = ['rgb', 'depth', 'segmentation', 'metadata', 'annotations']
        for dir_name in required_dirs:
            dir_path = os.path.join(self.dataset_path, dir_name)
            if not os.path.exists(dir_path):
                integrity_issues.append(f"Missing directory: {dir_path}")

        # Check file counts across modalities
        rgb_files = self._get_file_list('rgb', 'png')
        depth_files = self._get_file_list('depth', 'png')
        seg_files = self._get_file_list('segmentation', 'png')

        if not (len(rgb_files) == len(depth_files) == len(seg_files)):
            integrity_issues.append(f"File count mismatch: RGB={len(rgb_files)}, Depth={len(depth_files)}, Seg={len(seg_files)}")

        return integrity_issues

    def _get_file_list(self, modality, extension):
        """Get list of files for a specific modality"""
        modality_path = os.path.join(self.dataset_path, modality)
        if not os.path.exists(modality_path):
            return []

        return [f for f in os.listdir(modality_path) if f.endswith(f'.{extension}')]

    def validate_data_quality(self, sample_range=None):
        """Validate the quality of dataset samples"""
        quality_issues = []

        # Get all RGB files to determine sample count
        rgb_files = self._get_file_list('rgb', 'png')

        if sample_range is None:
            sample_range = range(len(rgb_files))

        for i in sample_range:
            sample_id = rgb_files[i].split('_')[0]  # Extract sample ID

            # Validate RGB image
            rgb_path = os.path.join(self.dataset_path, 'rgb', rgb_files[i])
            rgb_issues = self._validate_rgb_image(rgb_path, sample_id)
            quality_issues.extend(rgb_issues)

            # Validate depth image
            depth_files = [f for f in self._get_file_list('depth', 'png') if f.startswith(sample_id)]
            for depth_file in depth_files:
                depth_path = os.path.join(self.dataset_path, 'depth', depth_file)
                depth_issues = self._validate_depth_image(depth_path, sample_id)
                quality_issues.extend(depth_issues)

            # Validate segmentation
            seg_files = [f for f in self._get_file_list('segmentation', 'png') if f.startswith(sample_id)]
            for seg_file in seg_files:
                seg_path = os.path.join(self.dataset_path, 'segmentation', seg_file)
                seg_issues = self._validate_segmentation_image(seg_path, sample_id)
                quality_issues.extend(seg_issues)

        return quality_issues

    def _validate_rgb_image(self, image_path, sample_id):
        """Validate RGB image quality"""
        issues = []

        try:
            img = Image.open(image_path)
            img_array = np.array(img)

            # Check image dimensions
            height, width = img_array.shape[:2]
            if height < 100 or width < 100:
                issues.append(f"Sample {sample_id}: Image too small ({width}x{height})")

            # Check for completely black or white images
            if np.mean(img_array) < 10:  # Nearly black
                issues.append(f"Sample {sample_id}: Image too dark")
            elif np.mean(img_array) > 245:  # Nearly white
                issues.append(f"Sample {sample_id}: Image too bright")

            # Check for uniform color (potential rendering issue)
            if np.std(img_array) < 5:
                issues.append(f"Sample {sample_id}: Image has very low variance (uniform color)")

        except Exception as e:
            issues.append(f"Sample {sample_id}: Error reading RGB image - {str(e)}")

        return issues

    def _validate_depth_image(self, image_path, sample_id):
        """Validate depth image quality"""
        issues = []

        try:
            img = Image.open(image_path)
            depth_array = np.array(img).astype(np.float32)

            # Normalize if needed (assuming 16-bit)
            if depth_array.max() > 1000:  # Likely 16-bit
                depth_array = depth_array / 65535.0 * 10.0  # Normalize to 0-10m range

            # Check for invalid depth values
            if np.isnan(depth_array).any():
                issues.append(f"Sample {sample_id}: Depth image contains NaN values")

            if np.isinf(depth_array).any():
                issues.append(f"Sample {sample_id}: Depth image contains infinite values")

            # Check depth range
            min_depth, max_depth = np.min(depth_array), np.max(depth_array)
            if min_depth < 0.01 or max_depth > 50.0:  # Unreasonable depth range
                issues.append(f"Sample {sample_id}: Depth range {min_depth}-{max_depth}m seems unreasonable")

        except Exception as e:
            issues.append(f"Sample {sample_id}: Error reading depth image - {str(e)}")

        return issues

    def _validate_segmentation_image(self, image_path, sample_id):
        """Validate segmentation image quality"""
        issues = []

        try:
            img = Image.open(image_path)
            seg_array = np.array(img)

            # Check for reasonable number of classes
            unique_labels = np.unique(seg_array)
            if len(unique_labels) > 100:  # Too many classes might indicate an issue
                issues.append(f"Sample {sample_id}: Segmentation has too many classes ({len(unique_labels)})")

            # Check for completely empty segmentation (all background)
            if len(unique_labels) == 1 and unique_labels[0] == 0:
                issues.append(f"Sample {sample_id}: Segmentation is completely empty")

        except Exception as e:
            issues.append(f"Sample {sample_id}: Error reading segmentation image - {str(e)}")

        return issues

    def validate_annotations(self):
        """Validate annotation quality and consistency"""
        annotation_issues = []

        annotation_files = self._get_file_list('annotations', 'json')

        for annotation_file in annotation_files:
            sample_id = annotation_file.replace('.json', '')
            annotation_path = os.path.join(self.dataset_path, 'annotations', annotation_file)

            try:
                with open(annotation_path, 'r') as f:
                    annotations = json.load(f)

                # Validate annotation structure
                for cam_name, objects in annotations.items():
                    for obj in objects:
                        # Check bounding box validity
                        if 'bbox' in obj:
                            bbox = obj['bbox']
                            if len(bbox) != 4:
                                annotation_issues.append(f"Sample {sample_id}: Invalid bbox format")
                            elif bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:  # x_max <= x_min or y_max <= y_min
                                annotation_issues.append(f"Sample {sample_id}: Invalid bbox coordinates")

                        # Check center point validity
                        if 'center' in obj:
                            center = obj['center']
                            if len(center) != 2:
                                annotation_issues.append(f"Sample {sample_id}: Invalid center format")

            except Exception as e:
                annotation_issues.append(f"Sample {sample_id}: Error reading annotations - {str(e)}")

        return annotation_issues

    def generate_quality_report(self):
        """Generate comprehensive quality report"""
        report = {
            'dataset_path': self.dataset_path,
            'timestamp': carb.events.get_time(),
            'integrity_issues': self.validate_dataset_integrity(),
            'quality_issues': self.validate_data_quality(),
            'annotation_issues': self.validate_annotations(),
            'summary': {}
        }

        # Calculate summary statistics
        rgb_files = self._get_file_list('rgb', 'png')
        report['summary']['total_samples'] = len(rgb_files)
        report['summary']['integrity_issues_count'] = len(report['integrity_issues'])
        report['summary']['quality_issues_count'] = len(report['quality_issues'])
        report['summary']['annotation_issues_count'] = len(report['annotation_issues'])

        # Calculate pass rate
        total_issues = (report['summary']['integrity_issues_count'] +
                       report['summary']['quality_issues_count'] +
                       report['summary']['annotation_issues_count'])

        report['summary']['pass_rate'] = max(0, (len(rgb_files) - total_issues) / len(rgb_files)) if len(rgb_files) > 0 else 0

        return report

    def save_quality_report(self, report_path=None):
        """Save quality report to file"""
        if report_path is None:
            report_path = os.path.join(self.dataset_path, 'quality_report.json')

        report = self.generate_quality_report()

        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2)

        carb.log_info(f"Quality report saved to: {report_path}")
        return report_path

def validate_synthetic_dataset(dataset_path):
    """Perform comprehensive quality validation on synthetic dataset"""
    qa = DatasetQualityAssurance(dataset_path)
    report = qa.generate_quality_report()

    # Print summary
    summary = report['summary']
    carb.log_info(f"Dataset Quality Report:")
    carb.log_info(f"  Total Samples: {summary['total_samples']}")
    carb.log_info(f"  Pass Rate: {summary['pass_rate']:.2%}")
    carb.log_info(f"  Integrity Issues: {summary['integrity_issues_count']}")
    carb.log_info(f"  Quality Issues: {summary['quality_issues_count']}")
    carb.log_info(f"  Annotation Issues: {summary['annotation_issues_count']}")

    return report
```

## Dataset Usage and Training Integration

### Dataset Loading for Training

```python
# dataset_integration.py
import torch
import torch.utils.data as data
import numpy as np
from PIL import Image
import os
import json

class IsaacSyntheticDataset(data.Dataset):
    def __init__(self, dataset_path, modality='rgb', transform=None, target_transform=None):
        """
        Initialize Isaac synthetic dataset

        Args:
            dataset_path: Path to the dataset directory
            modality: 'rgb', 'depth', 'segmentation', or 'multimodal'
            transform: Transformations to apply to input data
            target_transform: Transformations to apply to targets
        """
        self.dataset_path = dataset_path
        self.modality = modality
        self.transform = transform
        self.target_transform = target_transform

        # Get all sample IDs
        self.sample_ids = self._get_sample_ids()

        # Load dataset configuration
        self.config = self._load_config()

        print(f"Loaded Isaac synthetic dataset with {len(self.sample_ids)} samples")

    def _get_sample_ids(self):
        """Extract sample IDs from RGB directory"""
        rgb_dir = os.path.join(self.dataset_path, 'rgb')
        if not os.path.exists(rgb_dir):
            raise ValueError(f"RGB directory not found: {rgb_dir}")

        # Get unique sample IDs (remove camera suffixes)
        files = os.listdir(rgb_dir)
        sample_ids = set()

        for file in files:
            if file.endswith('.png'):
                # Extract sample ID (first 6 digits)
                sample_id = file[:6]
                sample_ids.add(sample_id)

        return sorted(list(sample_ids))

    def _load_config(self):
        """Load dataset configuration"""
        config_path = os.path.join(self.dataset_path, 'dataset_config.json')
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                return json.load(f)
        return {}

    def __len__(self):
        """Return the number of samples in the dataset"""
        return len(self.sample_ids)

    def __getitem__(self, idx):
        """Get a sample from the dataset"""
        sample_id = self.sample_ids[idx]

        if self.modality == 'rgb':
            return self._get_rgb_sample(sample_id)
        elif self.modality == 'depth':
            return self._get_depth_sample(sample_id)
        elif self.modality == 'segmentation':
            return self._get_segmentation_sample(sample_id)
        elif self.modality == 'multimodal':
            return self._get_multimodal_sample(sample_id)
        else:
            raise ValueError(f"Unknown modality: {self.modality}")

    def _get_rgb_sample(self, sample_id):
        """Get RGB image sample"""
        # Find first RGB file for this sample ID
        rgb_dir = os.path.join(self.dataset_path, 'rgb')
        rgb_files = [f for f in os.listdir(rgb_dir) if f.startswith(sample_id)]

        if not rgb_files:
            raise ValueError(f"No RGB file found for sample {sample_id}")

        rgb_path = os.path.join(rgb_dir, rgb_files[0])
        image = Image.open(rgb_path).convert('RGB')

        if self.transform:
            image = self.transform(image)

        return image

    def _get_depth_sample(self, sample_id):
        """Get depth image sample"""
        depth_dir = os.path.join(self.dataset_path, 'depth')
        depth_files = [f for f in os.listdir(depth_dir) if f.startswith(sample_id)]

        if not depth_files:
            raise ValueError(f"No depth file found for sample {sample_id}")

        depth_path = os.path.join(depth_dir, depth_files[0])
        depth_image = Image.open(depth_path)
        depth_array = np.array(depth_image).astype(np.float32)

        # Normalize depth if needed
        if depth_array.max() > 1000:  # 16-bit depth
            depth_array = depth_array / 65535.0 * 10.0  # Normalize to 0-10m

        depth_tensor = torch.from_numpy(depth_array).unsqueeze(0)  # Add channel dimension

        if self.transform:
            depth_tensor = self.transform(depth_tensor)

        return depth_tensor

    def _get_segmentation_sample(self, sample_id):
        """Get segmentation sample with annotations"""
        # Load segmentation image
        seg_dir = os.path.join(self.dataset_path, 'segmentation')
        seg_files = [f for f in os.listdir(seg_dir) if f.startswith(sample_id)]

        if not seg_files:
            raise ValueError(f"No segmentation file found for sample {sample_id}")

        seg_path = os.path.join(seg_dir, seg_files[0])
        segmentation = Image.open(seg_path)
        seg_array = np.array(segmentation).astype(np.long)

        seg_tensor = torch.from_numpy(seg_array)

        # Load annotations if available
        annotations = self._load_annotations(sample_id)

        if self.transform:
            seg_tensor = self.transform(seg_tensor)

        return seg_tensor, annotations

    def _get_multimodal_sample(self, sample_id):
        """Get multimodal sample with RGB, depth, and segmentation"""
        sample = {}

        # Load RGB
        sample['rgb'] = self._get_rgb_sample(sample_id)

        # Load depth
        try:
            sample['depth'] = self._get_depth_sample(sample_id)
        except:
            sample['depth'] = torch.zeros(1, 480, 640)  # Default depth

        # Load segmentation
        try:
            seg_data, annotations = self._get_segmentation_sample(sample_id)
            sample['segmentation'] = seg_data
            sample['annotations'] = annotations
        except:
            sample['segmentation'] = torch.zeros(480, 640, dtype=torch.long)
            sample['annotations'] = []

        # Load metadata
        metadata = self._load_metadata(sample_id)
        sample['metadata'] = metadata

        return sample

    def _load_annotations(self, sample_id):
        """Load annotations for a sample"""
        annotation_path = os.path.join(self.dataset_path, 'annotations', f'{sample_id}.json')
        if os.path.exists(annotation_path):
            with open(annotation_path, 'r') as f:
                return json.load(f)
        return {}

    def _load_metadata(self, sample_id):
        """Load metadata for a sample"""
        metadata_path = os.path.join(self.dataset_path, 'metadata', f'{sample_id}.json')
        if os.path.exists(metadata_path):
            with open(metadata_path, 'r') as f:
                return json.load(f)
        return {}

def create_dataset_loader(dataset_path, batch_size=32, modality='rgb', shuffle=True):
    """Create data loader for Isaac synthetic dataset"""
    dataset = IsaacSyntheticDataset(dataset_path, modality=modality)

    loader = data.DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=4,
        pin_memory=True
    )

    return loader

def prepare_dataset_for_training(dataset_path, modality='multimodal'):
    """Prepare synthetic dataset for training"""
    print(f"Preparing dataset from: {dataset_path}")

    # Create data loader
    train_loader = create_dataset_loader(
        dataset_path,
        batch_size=16,
        modality=modality,
        shuffle=True
    )

    # Validate dataset
    qa = DatasetQualityAssurance(dataset_path)
    report = qa.generate_quality_report()

    if report['summary']['pass_rate'] < 0.95:
        print(f"Warning: Dataset quality pass rate is low: {report['summary']['pass_rate']:.2%}")

    print(f"Dataset ready for training with {len(train_loader.dataset)} samples")

    return train_loader
```

## Best Practices

1. **Diverse Scene Generation**: Create varied environments to improve model generalization
2. **Proper Annotation**: Ensure high-quality ground truth annotations for all modalities
3. **Quality Validation**: Implement comprehensive validation to catch rendering issues
4. **Domain Randomization**: Apply systematic variations to improve real-world transfer
5. **Data Balance**: Ensure balanced representation of different classes and scenarios
6. **Performance Monitoring**: Track dataset generation performance and efficiency
7. **Version Control**: Keep track of dataset versions and generation parameters

## Next Steps

With synthetic datasets properly configured, you're ready to explore Isaac ROS architecture in the next chapter. We'll learn how Isaac's ROS packages provide optimized perception, navigation, and manipulation capabilities specifically designed for NVIDIA's GPU-accelerated platform.