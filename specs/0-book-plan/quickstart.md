# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Prerequisites

- Ubuntu 22.04 LTS (recommended) or Docker
- 16GB+ RAM
- NVIDIA GPU with 8GB+ VRAM (for Isaac Sim)
- ROS 2 Humble Hawksbill installed
- Python 3.10+
- Node.js 18+
- Git

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up ROS 2 Environment
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 3. Install Python Dependencies
```bash
pip3 install -r requirements.txt
# For the RAG chatbot
pip3 install fastapi uvicorn pydantic qdrant-client openai python-dotenv
```

### 4. Set up Docusaurus Documentation Site
```bash
cd docusaurus
npm install
```

### 5. Environment Variables
Create a `.env` file in the backend directory:
```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_url
NEON_DB_URL=your_neon_db_url
```

## Running the Book Components

### 1. Run the Documentation Site
```bash
cd docusaurus
npm run start
```
The site will be available at http://localhost:3000

### 2. Run the RAG Chatbot
```bash
cd backend/rag-chatbot
uvicorn main:app --reload --port 8000
```
The API will be available at http://localhost:8000

### 3. Run the VLA Pipeline Simulation
```bash
cd backend/vla-pipeline
python3 -m vla_pipeline.main
```

## Running Simulations

### 1. Gazebo Simulation
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch your_robot_gazebo your_robot_world.launch.py
```

### 2. Isaac Sim
1. Launch Isaac Sim from the Omniverse launcher
2. Import your robot URDF following the book's Module 3 instructions
3. Run the perception and navigation examples

## Developing Book Content

### 1. Adding a New Chapter
1. Create a new `.mdx` file in the appropriate module directory
2. Follow the template structure with learning objectives, code examples, and simulation steps
3. Ensure consistent terminology with other modules

### 2. Testing Code Examples
1. Each code example should be tested in the target environment
2. Validate ROS 2 communications
3. Ensure simulation steps are reproducible

### 3. Building the Documentation
```bash
cd docusaurus
npm run build
```

## Testing the Complete Pipeline

### 1. End-to-End Test
1. Start all components (documentation, chatbot, simulation)
2. Use the RAG chatbot to query book content
3. Run a VLA command simulation:
   - Voice command: "Move the red block to the table"
   - Expect: Robot navigates, perceives, and manipulates as described in Module 4

### 2. Module Integration Test
1. Verify that concepts from Module 1 (ROS 2) work with Module 2 (Simulation)
2. Test that Module 3 (Isaac Sim) integrates with Module 4 (VLA)
3. Ensure consistent terminology across all modules

## Troubleshooting

### Common Issues
- **ROS 2 packages not found**: Ensure you've sourced the setup.bash file
- **Isaac Sim GPU errors**: Check NVIDIA drivers and Isaac Sim compatibility
- **API rate limits**: Verify your OpenAI API key and rate limits
- **Simulation performance**: Close other applications to free up resources

### Resetting the Environment
```bash
# Reset ROS workspace
cd ~/ros2_ws
rm -rf build install log
colcon build

# Clear simulation cache
rm -rf ~/.gazebo/
```

## Next Steps

1. Follow Module 1: The Robotic Nervous System (ROS 2) to learn the basics
2. Progress through Modules 2-4 in sequence for the complete learning path
3. Complete the capstone project integrating all components
4. Contribute to the book by following the contribution guidelines