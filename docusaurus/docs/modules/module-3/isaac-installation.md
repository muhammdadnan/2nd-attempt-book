# Isaac Installation and Setup

In this chapter, we'll walk through the complete installation and setup process for NVIDIA Isaac, including hardware verification, software installation, and initial configuration. Proper setup is crucial for leveraging Isaac's GPU-accelerated simulation and AI capabilities for humanoid robot development.

## System Requirements Verification

### Hardware Requirements Check

Before installing Isaac, verify your system meets the requirements:

```bash
# Check GPU and CUDA capability
nvidia-smi

# Check CUDA version
nvcc --version

# Verify GPU compute capability (minimum 6.0)
# For RTX cards:
# - RTX 20xx series: Compute Capability 7.5
# - RTX 30xx series: Compute Capability 8.6
# - RTX 40xx series: Compute Capability 8.9
```

### System Information Verification

```bash
# Check system specifications
echo "CPU: $(lscpu | grep 'Model name' | head -n1)"
echo "RAM: $(free -h | grep 'Mem:' | awk '{print $2}')"
echo "OS: $(lsb_release -d | cut -f2)"

# Check available disk space
df -h | grep -E "Filesystem|/$"
```

## Prerequisites Installation

### CUDA Installation

Isaac requires CUDA for GPU acceleration. Install CUDA if not already present:

```bash
# For Ubuntu 22.04
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda

# Add CUDA to PATH (add to ~/.bashrc)
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### Docker Installation

Isaac often runs in containerized environments:

```bash
# Install Docker
sudo apt update
sudo apt install ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up Docker repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Add user to docker group
sudo usermod -aG docker $USER
```

### ROS 2 Installation

Isaac integrates with ROS 2, so ensure ROS 2 Humble is installed:

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool

# Source ROS 2 environment
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## Isaac Sim Installation

### Download Isaac Sim

Isaac Sim is available from the NVIDIA Developer Zone:

1. Visit [developer.nvidia.com](https://developer.nvidia.com)
2. Navigate to the Isaac Sim download page
3. Download the appropriate version for your OS

### Installation Methods

#### Method 1: Local Installation (Recommended for Development)

```bash
# Extract the downloaded Isaac Sim package
tar -xzf isaac_sim-2023.1.1-linux.tar.gz -C ~/
cd ~/isaac_sim-2023.1.1

# Run the installation script
./install.sh

# The installer will guide you through the process
# It will download additional components and set up the environment
```

#### Method 2: Docker Installation

```bash
# Pull Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Run Isaac Sim container with GPU support
docker run --gpus all -it --rm \
  --network=host \
  --env "DISPLAY" \
  --env "QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/$USER:/home/$USER" \
  --volume="/dev/shm:/dev/shm" \
  --privileged \
  --pid=host \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

## Environment Configuration

### Environment Variables Setup

Add Isaac Sim environment variables to your shell configuration:

```bash
# Add to ~/.bashrc
echo 'export ISAAC_SIM_PATH="$HOME/isaac_sim-2023.1.1"' >> ~/.bashrc
echo 'export PYTHONPATH="${ISAAC_SIM_PATH}/python:${ISAAC_SIM_PATH}/apps:${PYTHONPATH}"' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH="${ISAAC_SIM_PATH}/lib:${ISAAC_SIM_PATH}/exts/omni.isaac.sim.python.gym/lib:${LD_LIBRARY_PATH}"' >> ~/.bashrc

source ~/.bashrc
```

### Verification Script

Create a verification script to ensure Isaac is properly installed:

```python
#!/usr/bin/env python3
# verify_isaac_installation.py

import sys
import subprocess
import os

def check_gpu():
    """Check if GPU is available and CUDA is working"""
    try:
        import torch
        if torch.cuda.is_available():
            print(f"✓ CUDA available: {torch.cuda.get_device_name(0)}")
            return True
        else:
            print("✗ CUDA not available")
            return False
    except ImportError:
        print("✗ PyTorch not installed (install with: pip install torch)")
        return False

def check_isaac_imports():
    """Check if Isaac Python modules can be imported"""
    modules_to_check = [
        "omni",
        "pxr",
        "carb",
    ]

    success = True
    for module in modules_to_check:
        try:
            __import__(module)
            print(f"✓ {module} import successful")
        except ImportError as e:
            print(f"✗ {module} import failed: {e}")
            success = False

    return success

def check_isaac_executable():
    """Check if Isaac Sim executable exists"""
    isaac_path = os.environ.get("ISAAC_SIM_PATH")
    if not isaac_path:
        print("✗ ISAAC_SIM_PATH not set")
        return False

    executable_path = os.path.join(isaac_path, "isaac-sim.sh")
    if os.path.exists(executable_path):
        print(f"✓ Isaac Sim executable found: {executable_path}")
        return True
    else:
        print(f"✗ Isaac Sim executable not found: {executable_path}")
        return False

def main():
    print("Isaac Installation Verification")
    print("=" * 40)

    gpu_ok = check_gpu()
    imports_ok = check_isaac_imports()
    exec_ok = check_isaac_executable()

    print("=" * 40)
    if gpu_ok and imports_ok and exec_ok:
        print("✓ All checks passed! Isaac is ready for use.")
        return 0
    else:
        print("✗ Some checks failed. Please review the installation.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
```

## Isaac ROS Installation

### Isaac ROS Dependencies

Install Isaac ROS dependencies and packages:

```bash
# Create a ROS 2 workspace for Isaac packages
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws

# Install Isaac ROS dependencies
sudo apt update
sudo apt install python3-pip python3-colcon-common-extensions python3-rosdep

# Install Isaac ROS packages via apt (if available)
sudo apt install ros-humble-isaac-ros-* ros-humble-isaac-ros-gems-*

# Or clone from source
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git src/isaac_ros_common
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git src/isaac_ros_visual_slam
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation.git src/isaac_ros_pose_estimation
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git src/isaac_ros_apriltag
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_gems.git src/isaac_ros_gems
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_tensor_rt.git src/isaac_ros_tensor_rt
```

### Build Isaac ROS Packages

```bash
cd ~/isaac_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install --packages-select \
  isaac_ros_common \
  isaac_ros_visual_slam \
  isaac_ros_pose_estimation \
  isaac_ros_apriltag \
  isaac_ros_gems \
  isaac_ros_tensor_rt

# Source the workspace
source install/setup.bash
echo 'source ~/isaac_ws/install/setup.bash' >> ~/.bashrc
```

## Isaac Lab Installation

### Isaac Lab Setup

Isaac Lab provides a framework for robot learning:

```bash
# Create Isaac Lab workspace
mkdir -p ~/isaac_lab_ws
cd ~/isaac_lab_ws

# Clone Isaac Lab repository
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# Install Isaac Lab dependencies
pip3 install -e .
```

### Isaac Lab Verification

Create a verification script for Isaac Lab:

```python
#!/usr/bin/env python3
# verify_isaac_lab.py

def check_isaac_lab():
    """Check if Isaac Lab is properly installed"""
    try:
        from omni.isaac.kit import SimulationApp
        print("✓ Isaac Lab SimulationApp import successful")

        # Initialize simulation app (minimal)
        simulation_app = SimulationApp({"headless": True})

        # Import core components
        import omni.isaac.core.utils.prims as prims
        import omni.isaac.core.utils.stage as stage
        print("✓ Isaac Lab core components import successful")

        # Cleanup
        simulation_app.close()
        print("✓ Isaac Lab verification completed successfully")
        return True

    except ImportError as e:
        print(f"✗ Isaac Lab import failed: {e}")
        return False
    except Exception as e:
        print(f"✗ Isaac Lab verification failed: {e}")
        return False

if __name__ == "__main__":
    success = check_isaac_lab()
    exit(0 if success else 1)
```

## Configuration and Optimization

### Isaac Sim Configuration

Create a configuration file for Isaac Sim:

```bash
# Create Isaac Sim configuration directory
mkdir -p ~/.nvidia-isaac-sim

# Create configuration file
cat > ~/.nvidia-isaac-sim/config.yaml << EOF
physics:
  gpu: true
  solver_type: 0  # 0 for NVSC, 1 for TGS
  solver_position_iteration_count: 4
  solver_velocity_iteration_count: 1
  enable_ccd: true

rendering:
  gpu: true
  msaa_level: 4
  max_render_time: 0.033  # 30 FPS equivalent

simulation:
  time_step: 0.008333  # 120 Hz
  sub_steps: 2
  stage_units_per_meter: 1.0

camera:
  resolution: [640, 480]
  fov: 60.0
  enable_denoising: true

gpu:
  memory_budget: 8192  # MB
  texture_budget: 4096  # MB
EOF
```

### Performance Optimization

Configure system for optimal Isaac performance:

```bash
# Increase shared memory size (important for Isaac Sim)
echo 'tmpfs /dev/shm tmpfs defaults,size=8G 0 0' | sudo tee -a /etc/fstab
sudo mount -o remount,size=8G /dev/shm

# Optimize GPU settings
nvidia-smi -ac 5000,1200  # Set memory and graphics clock (adjust based on GPU)

# Set power management to performance mode
sudo nvidia-smi -pm 1
```

## Testing the Installation

### Basic Isaac Sim Test

Test that Isaac Sim launches correctly:

```bash
# Launch Isaac Sim
cd ~/isaac_sim-2023.1.1
./isaac-sim.sh

# Or run with specific scene
./isaac-sim.sh --exec "omni.kit.scripting.create_stage()" --/app/window/state="maximized"
```

### ROS Integration Test

Test ROS integration with Isaac:

```bash
# Terminal 1: Launch Isaac Sim with ROS bridge
cd ~/isaac_sim-2023.1.1
./isaac-sim.sh --exec "from omni.isaac.ros2_bridge import get_ros2_node; get_ros2_node()" --enable-omni-kit-profiler

# Terminal 2: Test ROS communication
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash

# Check available topics
ros2 topic list

# Echo a test topic
ros2 topic echo /isaac_sim/clock builtin_interfaces/Time
```

## Troubleshooting Common Issues

### GPU and CUDA Issues

```bash
# Check CUDA installation
nvidia-smi
nvcc --version

# Verify CUDA samples work
cd /usr/local/cuda/samples/1_Utilities/deviceQuery
sudo make
./deviceQuery
```

### Isaac Sim Launch Issues

```bash
# Check Isaac Sim logs
ls -la ~/isaac_sim-2023.1.1/logs/

# Clear Isaac Sim cache if needed
rm -rf ~/.nvidia-omniverse/cache/
rm -rf ~/isaac_sim-2023.1.1/cache/
```

### ROS Integration Issues

```bash
# Check ROS environment
printenv | grep -i ros

# Verify ROS 2 installation
ros2 doctor

# Check Isaac ROS packages
colcon list --packages-select | grep isaac
```

## Performance Verification

Run a performance benchmark to ensure Isaac is configured correctly:

```bash
# Create performance test script
cat > ~/isaac_performance_test.py << EOF
import time
import numpy as np

def performance_test():
    print("Isaac Performance Test")
    print("=" * 30)

    # Test CPU performance
    start_time = time.time()
    arr = np.random.rand(1000, 1000)
    result = np.dot(arr, arr.T)
    cpu_time = time.time() - start_time
    print(f"CPU Matrix multiplication: {cpu_time:.4f}s")

    # Test GPU performance if available
    try:
        import torch
        if torch.cuda.is_available():
            start_time = time.time()
            gpu_arr = torch.randn(1000, 1000).cuda()
            gpu_result = torch.mm(gpu_arr, gpu_arr.T)
            torch.cuda.synchronize()  # Ensure operation completed
            gpu_time = time.time() - start_time
            print(f"GPU Matrix multiplication: {gpu_time:.4f}s")
            print(f"GPU acceleration factor: {cpu_time/gpu_time:.2f}x")
    except ImportError:
        print("PyTorch not available for GPU test")

    print("Performance test completed.")

if __name__ == "__main__":
    performance_test()
EOF

python3 ~/isaac_performance_test.py
```

## Next Steps

With Isaac properly installed and configured, you're ready to adapt your humanoid robot model for Isaac simulation. In the next chapter, we'll explore how to integrate your URDF robot model with Isaac's simulation environment, including adding Isaac-specific extensions and optimizing the model for high-fidelity simulation.