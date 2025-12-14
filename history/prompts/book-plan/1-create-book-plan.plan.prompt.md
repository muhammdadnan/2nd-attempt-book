---
id: 1
title: "Create book plan"
stage: plan
date: "2025-12-12"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "book-plan"
branch: "0-book-plan"
user: "user"
command: "/sp.plan"
labels: ["plan", "robotics", "book", "vla", "isaac", "ros2"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "specs/0-book-plan/plan.md"
  - "specs/0-book-plan/research.md"
  - "specs/0-book-plan/data-model.md"
  - "specs/0-book-plan/quickstart.md"
  - "specs/0-book-plan/contracts/rag-chatbot-api.yaml"
  - "specs/0-book-plan/contracts/vla-pipeline-api.yaml"
tests: []
---

# Prompt History Record: Create book plan

## Original Prompt

```
/sp.plan Physical AI & Humanoid Robotics — Full Book Plan

Objective:
Create a complete, multi-module textbook (Docusaurus MDX) covering the entire embodied-AI pipeline:
ROS 2 → Digital Twin Simulation → NVIDIA Isaac AI Brain → Vision-Language-Action Robotics.
All content will be structured for Spec-Kit Plus + Claude Code workflows and deployed to GitHub Pages.

Scope (4 Modules):
- Module 1: The Robotic Nervous System (ROS 2)
- Module 2: The Digital Twin (Gazebo & Unity)
- Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- Module 4: Vision-Language-Action (VLA)

Deliverables:
- 50+ Docusaurus chapters across 4 modules
- Capstone: Autonomous Humanoid Agent receiving a voice command and performing a full action cycle
- GitHub Pages deployment
- RAG chatbot over the book (OpenAI Agents/ChatKit + FastAPI + Neon + Qdrant)

-------------------------------------------------------------------
Module 1 — The Robotic Nervous System (ROS 2)
-------------------------------------------------------------------
Purpose:
Teach ROS 2 as the "nervous system" of humanoid robots.

Chapters:
1. What is ROS 2 and Why It Matters
2. DDS & ROS Architecture
3. ROS 2 Nodes
4. Topics (Pub/Sub Messaging)
5. Services & Actions
6. Parameters & Configuration
7. Launch Files
8. rclpy Basics
9. Python Agents → ROS Control Bridge
10. URDF Introduction
11. Building a Humanoid URDF
12. Visualizing in RViz
13. Mini Project: Humanoid Joint Control
14. Summary + MCQs + Exercises

Outcomes:
- Students publish/subscribe topics
- Build URDF
- Control humanoid joints with Python

-------------------------------------------------------------------
Module 2 — The Digital Twin (Gazebo & Unity)
-------------------------------------------------------------------
Purpose:
Teach physics simulation, world-building, and perception simulation.

Chapters:
1. What is a Digital Twin?
2. Gazebo Overview
3. Importing URDF to Gazebo
4. Physics Simulation (gravity, collisions)
5. Sensor Simulation (LiDAR, Depth, IMU)
6. Subscribing to Sensor Data from ROS 2
7. Environment Building in Gazebo
8. Unity for Robotics
9. High-Fidelity Rendering in Unity HDRP
10. Human–Robot Interaction Simulation
11. ROS–Unity Communication
12. Mini Project: Full Digital Twin of Humanoid
13. Summary + MCQs + Exercises

Outcomes:
- Build full digital twin
- Simulate perception sensors
- Create Unity interaction scenes

-------------------------------------------------------------------
Module 3 — The AI-Robot Brain (NVIDIA Isaac™)
-------------------------------------------------------------------
Purpose:
Teach advanced perception, VSLAM, navigation, and GPU-accelerated robotics.

Chapters:
1. Overview of Isaac Sim
2. Installing Isaac Sim for Robotics
3. Importing Robots into Isaac
4. Photorealism & Materials
5. Synthetic Data Generation
6. Isaac ROS Overview
7. Isaac ROS VSLAM
8. Isaac ROS Perception Nodes
9. Overview of Nav2
10. Nav2 Architecture (planner, controller, recovery)
11. Integrating Isaac ROS + Nav2
12. Project: Humanoid Walks an Obstacle Course
13. Summary + MCQs + Exercises

Outcomes:
- Generate synthetic vision datasets
- Use Isaac ROS for VSLAM
- Run Nav2 for humanoid navigation

-------------------------------------------------------------------
Module 4 — Vision-Language-Action (VLA)
-------------------------------------------------------------------
Purpose:
Teach LLM-powered robotics: Voice → Understanding → Planning → Acting.

Chapters:
1. What is VLA?
2. End-to-End VLA Pipeline
3. Whisper Voice Commands
4. Natural Language → Structured Commands
5. LLM-Based Cognitive Planning
6. Safety Guardrails for LLM-Controlled Robots
7. Visual Perception Integration
8. Object Selection & Manipulation
9. Navigation via LLM Plans → Nav2
10. Manipulator Control
11. Full VLA Agent Loop
12. Capstone: Autonomous Humanoid
13. Testing & Failure Modes
14. Summary + MCQs + Tasks

Outcomes:
- Voice-to-Action pipeline working in simulation
- LLM translates "Clean the room" → ROS 2 actions
- Full autonomous humanoid

-------------------------------------------------------------------
Cross-Module Infrastructure
-------------------------------------------------------------------
- Docusaurus project setup
- GitHub Pages deployment
- Unified glossary and terminology style


-------------------------------------------------------------------
Success Criteria:
-------------------------------------------------------------------
- 4 modules completed as independent but connected learning paths
- All MDX builds successfully in Docusaurus
- Capstone humanoid completes full Voice→Plan→Navigate→Perceive→Act cycle
- Clean coding standards and reproducible simulation instructions
```

## Response Summary

Created a comprehensive implementation plan for the Physical AI & Humanoid Robotics book project. The plan includes:

- Detailed technical context with resolved technology stack decisions
- Constitution check aligned with project principles
- Complete project structure for documentation, simulation, and AI components
- Research document resolving all technical clarifications
- Data model with entities for book modules, chapters, code examples, simulation steps, and VLA components
- API contracts for the RAG chatbot and VLA pipeline
- Quickstart guide for setting up and running the complete system

The plan addresses all four modules (ROS 2, Digital Twin, AI-Robot Brain, VLA) with proper integration between them, following the learn → simulate → deploy pedagogy.

## Outcome

Successfully created a complete implementation plan for the Physical AI & Humanoid Robotics book project with all required artifacts. The plan provides a roadmap for developing 50+ chapters across 4 modules, implementing the RAG chatbot, and creating the capstone autonomous humanoid system.

## Evaluation

The implementation plan meets all requirements with comprehensive technical context, proper constitution alignment, and detailed specifications for all system components. It provides clear guidance for development while maintaining consistency with the project's core principles.