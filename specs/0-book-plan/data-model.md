# Data Model: Physical AI & Humanoid Robotics — Full Book Plan

## Entities

### Book Module
- **id**: string (unique identifier for the module)
- **title**: string (name of the module)
- **description**: string (purpose and focus of the module)
- **chapters**: array of Chapter objects
- **learning_outcomes**: array of string (skills students will acquire)
- **validation_rules**:
  - Must have 8-12 chapters as per constitution
  - Title must follow consistent terminology standards

### Chapter
- **id**: string (unique identifier for the chapter)
- **title**: string (name of the chapter)
- **module_id**: string (reference to parent module)
- **content_type**: enum (text, mdx, simulation, code_example)
- **learning_objectives**: array of string (what students will learn)
- **code_examples**: array of CodeExample objects
- **simulation_steps**: array of SimulationStep objects
- **diagram_descriptions**: array of string (text descriptions of diagrams)
- **validation_rules**:
  - Must have 800-1500 words as per constitution
  - Must include learning objectives
  - Must include at least one code example or simulation step

### CodeExample
- **id**: string (unique identifier)
- **title**: string (brief description)
- **language**: string (programming language)
- **code**: string (the actual code)
- **explanation**: string (what the code does)
- **chapter_id**: string (reference to parent chapter)
- **validation_rules**:
  - Code must be validated in actual environment per constitution
  - Must follow consistent terminology standards

### SimulationStep
- **id**: string (unique identifier)
- **title**: string (brief description)
- **environment**: enum (gazebo, isaac_sim, unity, ros2)
- **instructions**: string (step-by-step guide)
- **expected_outcome**: string (what should happen)
- **chapter_id**: string (reference to parent chapter)
- **validation_rules**:
  - Must be reproducible in specified environment
  - Must follow learn → simulate → deploy pedagogy

### VLACommand
- **id**: string (unique identifier)
- **voice_input**: string (spoken command)
- **structured_task**: string (parsed command)
- **planning_sequence**: array of ROSAction objects
- **validation_rules**:
  - Must be generated from natural language processing
  - Must result in valid ROS 2 action sequence

### ROSAction
- **id**: string (unique identifier)
- **action_type**: enum (navigation, perception, manipulation, other)
- **parameters**: object (specific parameters for the action)
- **target_module**: string (which module this action relates to)
- **validation_rules**:
  - Must be executable in simulation environment
  - Must follow ROS 2 standards

### HumanoidRobot
- **id**: string (unique identifier)
- **urdf_model**: string (URDF file path)
- **joints**: array of Joint objects
- **sensors**: array of Sensor objects
- **capabilities**: array of string (navigation, manipulation, perception)
- **validation_rules**:
  - Must be importable into all simulation environments
  - Must follow consistent URDF standards

### Joint
- **id**: string (unique identifier)
- **name**: string (joint name)
- **type**: enum (revolute, prismatic, fixed)
- **limits**: object (min/max values)
- **robot_id**: string (reference to parent robot)
- **validation_rules**:
  - Must follow ROS 2 joint standards
  - Limits must be physically realistic

### Sensor
- **id**: string (unique identifier)
- **type**: enum (lidar, camera, imu, depth_camera)
- **topic_name**: string (ROS 2 topic)
- **parameters**: object (sensor-specific settings)
- **robot_id**: string (reference to parent robot)
- **validation_rules**:
  - Must be simulatable in Gazebo and Isaac Sim
  - Must output valid ROS 2 messages

### ChatbotQuery
- **id**: string (unique identifier)
- **question**: string (user's question)
- **context**: string (relevant book content)
- **response**: string (generated answer)
- **confidence**: number (0-1 confidence score)
- **validation_rules**:
  - Response must have 90%+ grounding accuracy as per constitution
  - Must reference specific book content

## Relationships

- Book Module **contains** many Chapters
- Chapter **contains** many CodeExamples
- Chapter **contains** many SimulationSteps
- VLACommand **contains** many ROSActions
- HumanoidRobot **contains** many Joints
- HumanoidRobot **contains** many Sensors
- ChatbotQuery **references** Book content

## State Transitions

### Chapter State Transitions
- Draft → In Review → Approved → Published
- Validation: Each state transition must meet specific criteria per constitution

### VLACommand State Transitions
- Voice Input → Parsed → Planned → Executed → Completed/Failed
- Validation: Safety checks at each transition point