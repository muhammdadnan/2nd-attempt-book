---
description: "Task list for Physical AI & Humanoid Robotics book project"
---

# Tasks: Physical AI & Humanoid Robotics — Full Book Tasks

**Input**: Design documents from `/specs/0-book-plan/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

<!--
  ============================================================================
  IMPORTANT: The tasks below are SAMPLE TASKS for illustration purposes only.

  The /sp.tasks command MUST replace these with actual tasks based on:
  - User stories from spec.md (with their priorities P1, P2, P3...)
  - Feature requirements from plan.md
  - Entities from data-model.md
  - Endpoints from contracts/

  Tasks MUST be organized by user story so each story can be:
  - Implemented independently
  - Tested independently
  - Delivered as an MVP increment

  DO NOT keep these sample tasks in the generated tasks.md file.
  ============================================================================
-->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create Docusaurus project structure using classic template
- [ ] T002 Initialize GitHub Pages deployment workflow in `.github/workflows/deploy.yml`
- [ ] T003 [P] Create `/docusaurus/docs/modules/module-1`, `/docusaurus/docs/modules/module-2`, `/docusaurus/docs/modules/module-3`, `/docusaurus/docs/modules/module-4` directories
- [ ] T004 Create Docusaurus sidebar configuration in `docusaurus/sidebars.js`
- [ ] T005 [P] Initialize backend directory with FastAPI structure: `backend/rag-chatbot/main.py`, `backend/rag-chatbot/models/`, `backend/rag-chatbot/routes/`, `backend/rag-chatbot/vector_db/`
- [ ] T006 Create requirements.txt for backend dependencies: FastAPI, Neon Postgres, Qdrant, OpenAI SDK
- [ ] T007 [P] Set up ROS 2 workspace structure: `ros2_ws/src/robot_description/`, `ros2_ws/src/robot_control/`, `ros2_ws/src/simulation/`

---
## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [ ] T008 Configure Docusaurus site configuration in `docusaurus/docusaurus.config.js` with proper navigation and theme
- [ ] T009 [P] Set up MDX formatting rules and global styles in `docusaurus/src/css/custom.css`
- [ ] T010 Create global glossary file in `docusaurus/docs/glossary.mdx` with robotics, AI, and simulation terms
- [ ] T011 [P] Initialize Qdrant vector database connection in `backend/rag-chatbot/vector_db/qdrant_client.py`
- [ ] T012 Set up Neon Postgres connection in `backend/rag-chatbot/models/database.py`
- [ ] T013 Create base humanoid URDF model in `ros2_ws/src/robot_description/urdf/humanoid.urdf.xacro`
- [ ] T014 Create simulation environment templates in `ros2_ws/src/simulation/worlds/`
- [ ] T015 [P] Create Unity HDRP template project structure in `unity-project/`
- [ ] T016 [P] Create Isaac Sim sample scene structure in `isaac-sim-scenes/`
- [ ] T017 Set up common constants and configuration files for all modules

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---
## Phase 3: Module 1 - The Robotic Nervous System (ROS 2) (Priority: P1) 🎯 MVP

**Goal**: Students able to operate a ROS 2 robot skeleton + control joints

**Independent Test**: Student can create ROS 2 nodes, control humanoid joints, and visualize the robot in RViz

### Tests for Module 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T018 [P] [M1] Unit test for ROS 2 node communication in `ros2_ws/src/robot_control/test/test_nodes.py`
- [ ] T019 [P] [M1] Integration test for topic messaging in `ros2_ws/src/robot_control/test/test_topics.py`

### Implementation for Module 1

- [ ] T020 [P] [M1] Write Module 1 introduction chapter in `docusaurus/docs/modules/module-1/introduction.mdx`
- [ ] T021 [P] [M1] Write ROS 2 architecture and DDS chapter in `docusaurus/docs/modules/module-1/ros2-architecture.mdx`
- [ ] T022 [P] [M1] Create ROS 2 Node examples with Python/rclpy in `docusaurus/docs/modules/module-1/nodes.mdx`
- [ ] T023 [P] [M1] Create Topic pub-sub exercises chapter in `docusaurus/docs/modules/module-1/topics.mdx`
- [ ] T024 [P] [M1] Write Services & Actions chapter with sample robot task in `docusaurus/docs/modules/module-1/services-actions.mdx`
- [ ] T025 [P] [M1] Add Parameters, Launch files, configs chapter in `docusaurus/docs/modules/module-1/parameters-launch.mdx`
- [ ] T026 [P] [M1] Document rclpy integration with AI agents in `docusaurus/docs/modules/module-1/rclpy-ai.mdx`
- [ ] T027 [P] [M1] Teach URDF fundamentals + links/joints in `docusaurus/docs/modules/module-1/urdf-fundamentals.mdx`
- [ ] T028 [P] [M1] Build humanoid URDF skeleton chapter in `docusaurus/docs/modules/module-1/humanoid-urdf.mdx`
- [ ] T029 [P] [M1] Create RViz visualization tutorial in `docusaurus/docs/modules/module-1/rviz-visualization.mdx`
- [ ] T030 [M1] Create ROS 2 node for controlling humanoid joints in `ros2_ws/src/robot_control/src/joint_controller.py`
- [ ] T031 [M1] Implement joint control service in `ros2_ws/src/robot_control/src/joint_control_service.py`
- [ ] T032 [M1] Create launch file for joint control demo in `ros2_ws/src/robot_control/launch/joint_control.launch.py`
- [ ] T033 [M1] Add MCQs, Summary, Review Exercises for Module 1 in `docusaurus/docs/modules/module-1/conclusion.mdx`

**Checkpoint**: At this point, Module 1 should be fully functional and testable independently

---
## Phase 4: Module 2 - The Digital Twin (Gazebo & Unity) (Priority: P2)

**Goal**: Students simulate sensors + build interactive environments

**Independent Test**: Student can import URDF into Gazebo, simulate sensors, and create Unity interaction scenes

### Tests for Module 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T034 [P] [M2] Unit test for Gazebo sensor simulation in `ros2_ws/src/simulation/test/test_sensors.py`
- [ ] T035 [P] [M2] Integration test for ROS-Gazebo bridge in `ros2_ws/src/simulation/test/test_bridge.py`

### Implementation for Module 2

- [ ] T036 [P] [M2] Write Module 2 introduction on Digital Twins in `docusaurus/docs/modules/module-2/introduction.mdx`
- [ ] T037 [P] [M2] Explain Gazebo physics engine in `docusaurus/docs/modules/module-2/gazebo-physics.mdx`
- [ ] T038 [P] [M2] Create URDF import to Gazebo tutorial in `docusaurus/docs/modules/module-2/urdf-gazebo.mdx`
- [ ] T039 [P] [M2] Write physics tuning chapter (collisions, mass, inertia) in `docusaurus/docs/modules/module-2/physics-tuning.mdx`
- [ ] T040 [P] [M2] Add LiDAR, Depth Camera, IMU simulations chapter in `docusaurus/docs/modules/module-2/sensor-simulation.mdx`
- [ ] T041 [P] [M2] Connect sensors to ROS 2 subscriber nodes chapter in `docusaurus/docs/modules/module-2/sensor-ros.mdx`
- [ ] T042 [P] [M2] Create Gazebo world + lighting + materials chapter in `docusaurus/docs/modules/module-2/gazebo-worlds.mdx`
- [ ] T043 [P] [M2] Introduce Unity for robotics in `docusaurus/docs/modules/module-2/unity-robotics.mdx`
- [ ] T044 [P] [M2] Build HDRP high-fidelity interaction environment chapter in `docusaurus/docs/modules/module-2/unity-hdrp.mdx`
- [ ] T045 [P] [M2] Teach ROS-Unity bridge communication in `docusaurus/docs/modules/module-2/ros-unity-bridge.mdx`
- [ ] T046 [M2] Create Gazebo world with humanoid robot in `ros2_ws/src/simulation/worlds/humanoid_world.sdf`
- [ ] T047 [M2] Configure Gazebo sensors for humanoid robot in `ros2_ws/src/robot_description/urdf/humanoid.gazebo.xacro`
- [ ] T048 [M2] Implement Unity HDRP scene for humanoid interaction in `unity-project/Assets/Scenes/HumanoidInteraction.unity`
- [ ] T049 [M2] Create ROS-Unity bridge configuration in `ros2_ws/src/robot_control/config/ros_unity_bridge.yaml`
- [ ] T050 [M2] Add MCQs, Summary, Simulation Tasks for Module 2 in `docusaurus/docs/modules/module-2/conclusion.mdx`

**Checkpoint**: At this point, Module 2 should be fully functional and testable independently

---
## Phase 5: Module 3 - The AI-Robot Brain (NVIDIA Isaac™) (Priority: P3)

**Goal**: Students build a GPU-accelerated AI brain using Isaac + VSLAM + Nav2

**Independent Test**: Student can generate synthetic datasets, use Isaac ROS for VSLAM, and run Nav2 for navigation

### Tests for Module 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T051 [P] [M3] Unit test for Isaac Sim VSLAM pipeline in `ros2_ws/src/isaac_control/test/test_vslam.py`
- [ ] T052 [P] [M3] Integration test for Isaac ROS + Nav2 in `ros2_ws/src/isaac_control/test/test_nav_integration.py`

### Implementation for Module 3

- [ ] T053 [P] [M3] Write Module 3 introduction on AI-driven robotics in `docusaurus/docs/modules/module-3/introduction.mdx`
- [ ] T054 [P] [M3] Write Isaac Sim installation and configuration guide in `docusaurus/docs/modules/module-3/isaac-installation.mdx`
- [ ] T055 [P] [M3] Import humanoid URDF into Isaac with ROS Bridge chapter in `docusaurus/docs/modules/module-3/urdf-isaac.mdx`
- [ ] T056 [P] [M3] Configure photorealistic rendering & materials chapter in `docusaurus/docs/modules/module-3/photorealistic-rendering.mdx`
- [ ] T057 [P] [M3] Generate synthetic vision datasets chapter in `docusaurus/docs/modules/module-3/synthetic-datasets.mdx`
- [ ] T058 [P] [M3] Explain Isaac ROS architecture in `docusaurus/docs/modules/module-3/isaac-ros-architecture.mdx`
- [ ] T059 [P] [M3] Implement VSLAM pipeline tutorial in `docusaurus/docs/modules/module-3/vslam-pipeline.mdx`
- [ ] T060 [P] [M3] Integrate perception nodes (AprilTags, stereo, depth) chapter in `docusaurus/docs/modules/module-3/perception-nodes.mdx`
- [ ] T061 [P] [M3] Explain Nav2 stack: mapping → planning → control in `docusaurus/docs/modules/module-3/nav2-stack.mdx`
- [ ] T062 [P] [M3] Connect Nav2 + Isaac ROS for humanoid navigation in `docusaurus/docs/modules/module-3/nav2-isaac.mdx`
- [ ] T063 [M3] Create Isaac Sim scene with humanoid robot in `isaac-sim-scenes/humanoid_navigation.usd`
- [ ] T064 [M3] Implement Isaac ROS VSLAM pipeline in `ros2_ws/src/isaac_control/src/vslam_node.py`
- [ ] T065 [M3] Configure Nav2 for humanoid navigation in `ros2_ws/src/robot_control/config/nav2_params_humanoid.yaml`
- [ ] T066 [M3] Create Isaac Sim perception pipeline in `ros2_ws/src/isaac_control/src/perception_pipeline.py`
- [ ] T067 [M3] Add MCQs, Summary, Navigation Tasks for Module 3 in `docusaurus/docs/modules/module-3/conclusion.mdx`

**Checkpoint**: At this point, Module 3 should be fully functional and testable independently

---
## Phase 6: Module 4 - Vision-Language-Action (VLA) (Priority: P4)

**Goal**: Students create a fully autonomous voice-driven humanoid agent

**Independent Test**: Student can create a full VLA pipeline that responds to voice commands with navigation, perception, and manipulation

### Tests for Module 4 (OPTIONAL - only if tests requested) ⚠️

- [ ] T068 [P] [M4] Unit test for Whisper voice processing in `backend/vla_pipeline/test/test_whisper.py`
- [ ] T069 [P] [M4] Integration test for VLA pipeline in `backend/vla_pipeline/test/test_vla_integration.py`

### Implementation for Module 4

- [ ] T070 [P] [M4] Write Module 4 introduction on VLA paradigm in `docusaurus/docs/modules/module-4/introduction.mdx`
- [ ] T071 [P] [M4] Describe Voice → Language → Action pipeline in `docusaurus/docs/modules/module-4/vla-pipeline.mdx`
- [ ] T072 [P] [M4] Integrate Whisper for speech-to-text robot commands in `docusaurus/docs/modules/module-4/whisper-integration.mdx`
- [ ] T073 [P] [M4] Build natural language → structured task parser in `docusaurus/docs/modules/module-4/nlp-parser.mdx`
- [ ] T074 [P] [M4] Implement LLM cognitive planner in `docusaurus/docs/modules/module-4/llm-planner.mdx`
- [ ] T075 [P] [M4] Add safety guardrails for LLM-controlled robots in `docusaurus/docs/modules/module-4/safety-guardrails.mdx`
- [ ] T076 [P] [M4] Connect perception models for object selection in `docusaurus/docs/modules/module-4/perception-selection.mdx`
- [ ] T077 [P] [M4] Integrate Nav2 for LLM-generated navigation routes in `docusaurus/docs/modules/module-4/nav2-integration.mdx`
- [ ] T078 [P] [M4] Implement manipulation tasks (pick, place, align) in `docusaurus/docs/modules/module-4/manipulation-tasks.mdx`
- [ ] T079 [P] [M4] Build full VLA agent loop in `docusaurus/docs/modules/module-4/vla-agent.mdx`
- [ ] T080 [M4] Create VLA pipeline API endpoints in `backend/vla_pipeline/routes/vla.py`
- [ ] T081 [M4] Implement Whisper voice processing service in `backend/vla_pipeline/services/voice_processor.py`
- [ ] T082 [M4] Create NLP command parser in `backend/vla_pipeline/services/command_parser.py`
- [ ] T083 [M4] Implement LLM cognitive planner in `backend/vla_pipeline/services/llm_planner.py`
- [ ] T084 [M4] Add safety constraint validation in `backend/vla_pipeline/services/safety_validator.py`
- [ ] T085 [M4] Create full VLA agent orchestrator in `backend/vla_pipeline/main.py`
- [ ] T086 [M4] Add MCQs, Summary, Capstone Review for Module 4 in `docusaurus/docs/modules/module-4/conclusion.mdx`

**Checkpoint**: At this point, Module 4 should be fully functional and testable independently

---
## Phase 7: Final Capstone & Integration (Priority: P5)

**Goal**: Complete the full autonomous humanoid system and deploy the book

**Independent Test**: Capstone system completes full Voice → Plan → Navigate → Perceive → Manipulate cycle

### Tests for Capstone (OPTIONAL - only if tests requested) ⚠️

- [ ] T087 [P] [CAP] Unit test for unified simulation in `ros2_ws/src/integration_tests/test_unified_sim.py`
- [ ] T088 [P] [CAP] Integration test for full VLA pipeline in `backend/integration_tests/test_full_pipeline.py`

### Implementation for Capstone

- [ ] T089 [P] [CAP] Build unified simulation combining ROS 2 + Gazebo/Isaac in `ros2_ws/src/integration/launch/unified_simulation.launch.py`
- [ ] T090 [P] [CAP] Implement full VLA pipeline integration in `backend/vla_pipeline/integration.py`
- [ ] T091 [P] [CAP] Create capstone evaluation framework in `evaluation/capstone_eval.py`
- [ ] T092 [P] [CAP] Prepare final documentation in Docusaurus in `docusaurus/docs/capstone.mdx`
- [ ] T093 [CAP] Create RAG chatbot indexing pipeline for MDX chapters in `backend/rag-chatbot/ingestion/index_chapters.py`
- [ ] T094 [CAP] Configure OpenAI Agent to answer from book content in `backend/rag-chatbot/agents/book_agent.py`
- [ ] T095 [CAP] Deploy GitHub Pages site with documentation in `docusaurus/deploy.sh`
- [ ] T096 [CAP] Deploy RAG backend with proper configuration in `backend/deploy.sh`

**Checkpoint**: At this point, the full system should be functional with all modules integrated

---
## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T097 [P] Documentation updates in `docusaurus/docs/`
- [ ] T098 Code cleanup and refactoring across all modules
- [ ] T099 Performance optimization across all components
- [ ] T100 [P] Additional unit tests (if requested) in all test directories
- [ ] T101 Security hardening for web APIs
- [ ] T102 Run quickstart validation in `specs/0-book-plan/quickstart.md`

---
## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - Module 1 (M1) can start after Foundational (Phase 2) - No dependencies on other modules
  - Module 2 (M2) can start after Foundational (Phase 2) - May integrate with M1 but should be independently testable
  - Module 3 (M3) can start after Foundational (Phase 2) - May integrate with M1/M2 but should be independently testable
  - Module 4 (M4) can start after Foundational (Phase 2) - May integrate with M1/M2/M3 but should be independently testable
- **Capstone (Final Phase)**: Depends on all desired modules being complete
- **Polish (Final Phase)**: Depends on all desired modules being complete

### Within Each Module

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Module complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all modules can start in parallel (if team capacity allows)
- All tests for a module marked [P] can run in parallel
- Models within a module marked [P] can run in parallel
- Different modules can be worked on in parallel by different team members

---
## Parallel Example: Module 1

```bash
# Launch all tests for Module 1 together (if tests requested):
Task: "Unit test for ROS 2 node communication in ros2_ws/src/robot_control/test/test_nodes.py"
Task: "Integration test for topic messaging in ros2_ws/src/robot_control/test/test_topics.py"

# Launch all chapters for Module 1 together:
Task: "Write Module 1 introduction chapter in docusaurus/docs/modules/module-1/introduction.mdx"
Task: "Write ROS 2 architecture and DDS chapter in docusaurus/docs/modules/module-1/ros2-architecture.mdx"
Task: "Create ROS 2 Node examples with Python/rclpy in docusaurus/docs/modules/module-1/nodes.mdx"
```

---
## Implementation Strategy

### MVP First (Module 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all modules)
3. Complete Phase 3: Module 1
4. **STOP and VALIDATE**: Test Module 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add Module 1 → Test independently → Deploy/Demo (MVP!)
3. Add Module 2 → Test independently → Deploy/Demo
4. Add Module 3 → Test independently → Deploy/Demo
5. Add Module 4 → Test independently → Deploy/Demo
6. Each module adds value without breaking previous modules

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: Module 1
   - Developer B: Module 2
   - Developer C: Module 3
   - Developer D: Module 4
3. Modules complete and integrate independently

---
## Notes

- [P] tasks = different files, no dependencies
- [M1], [M2], [M3], [M4], [CAP] labels map task to specific module for traceability
- Each module should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate module independently
- Avoid: vague tasks, same file conflicts, cross-module dependencies that break independence