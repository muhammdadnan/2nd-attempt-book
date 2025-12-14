<!-- SYNC IMPACT REPORT:
Version change: N/A -> 1.0.0
List of modified principles: N/A (initial constitution)
Added sections: All principles and sections (initial constitution)
Removed sections: None
Templates requiring updates: N/A (initial constitution)
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics — Full Book + Integrated RAG Chatbot Constitution

## Core Principles

### Technical Accuracy
All robotics and AI claims must reference official documentation (ROS 2, NVIDIA Isaac, Gazebo, Unity, OpenAI, etc.); Code samples must be validated in ROS 2 Humble + Gazebo Garden + Isaac Sim environments

### Clarity and Accessibility
Explanations must follow 'learn → simulate → deploy' pedagogy; Content must be accessible to both intermediate and advanced learners in robotics and embodied AI

### Hands-On Reproducibility
All code samples must be validated in actual environments; All examples must be reproducible through executable code, simulations, and project pipelines

### Modular Architecture
Book structure must follow Docusaurus + Spec-Kit Plus conventions; Architecture must enable integration with Claude Code and structured specs

### Consistent Terminology
Terminology must stay consistent across modules (ROS graph, namespaces, URDF, digital twin, VLA models); All MDX must be exportable to GitHub Pages without breaking builds

### Validated Code and Testing
All code must be tested or validated via Spec-Kit Plus or Claude Code execution sandbox; Each module forms a coherent learning path: ROS → Simulation → Isaac → VLA

## Structure and Format Standards

Four modules, each 8–12 chapters (minimum 40 total chapters); Each chapter must include: Learning objectives, Code examples, Simulation steps, At least one robotics diagram or flow explanation; RAG Chatbot must follow OpenAI Agents/ChatKit SDK + FastAPI + Neon Postgres + Qdrant stack

## Success Criteria and Validation

Entire book compiles successfully in Docusaurus and deploys on GitHub Pages; RAG Chatbot answers questions using book content with 90%+ grounding accuracy; Final capstone robot completes a full Voice-to-Action task in simulation; Capstone must detail the end-to-end pipeline: Voice → LLM Planning → ROS 2 Actions → Navigation → Perception → Manipulation

## Governance
Constitution supersedes all other practices; All implementations must follow the learn → simulate → deploy pedagogy; Amendments require documentation, approval, and migration plan; All PRs/reviews must verify compliance with robotics/AI standards and validation requirements

**Version**: 1.0.0 | **Ratified**: 2025-12-12 | **Last Amended**: 2025-12-12
