<!--
Sync Impact Report:
Version change: 0.0.0 (initial) -> 1.0.0
List of modified principles:
- Beginner-first teaching: New principle
- Hands-on over theory: New principle
- Real-world relevance: New principle
- Honesty about difficulty and cost: New principle
- Sim-to-real mindset: New principle
Added sections:
- Key Standards
- Project Requirements & Goals
Removed sections:
- None
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated (Constitution Check alignment)
- .specify/templates/spec-template.md: ✅ updated (scope/requirements alignment)
- .specify/templates/tasks-template.md: ✅ updated (task categorization alignment)
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics – An AI-Native Textbook Constitution

## Core Principles

### Beginner-first teaching
Every concept must be explainable to someone who knows Python + basic AI but has never touched a robot. The content must be accessible and progressively introduce complex topics.

### Hands-on over theory
Every major topic must include at least one runnable code example or lab. Practical application and experimentation are prioritized to reinforce theoretical understanding.

### Real-world relevance
Focus on technologies actually used in 2025 humanoid robots (ROS 2 Humble/Iron, NVIDIA Isaac Sim, Jetson Orin, Unitree G1/Go2, Vision-Language-Action models). This ensures the content is current and valuable to aspiring roboticists.

### Honesty about difficulty and cost
Never hide hardware requirements, costs, or the fact that this is computationally heavy. Transparency about the challenges and investments required in physical AI is crucial.

### Sim-to-real mindset
Always show the path from simulation → edge device → real robot. Emphasize the practical transition of algorithms and models from simulated environments to physical hardware.

## Key Standards

All code examples MUST be tested and runnable on Ubuntu 22.04 + ROS 2 Humble or Iron.
Mermaid diagrams MUST be used for every architecture (ROS 2 graph, VLA pipeline, URDF structure, etc.) to enhance visual understanding.
Every module and week MUST contain:
- Learning objectives
- Key concepts with analogies
- At least one hands-on lab or mini-project
- Quiz/questions (multiple choice + open-ended)
- Further reading / video links
The tone of the textbook MUST be excited, encouraging, and practical (like a senior teammate teaching a junior).
The language MUST be English with clear, simple sentences (Flesch-Kincaid grade 8–10 so beginners can follow).
For tools and research papers, citation style MUST use inline links or (Author, Year) – no strict APA needed.

## Project Requirements & Goals

### Technical Requirements
- Built with Spec-Kit Plus + Docusaurus.
- Fully deployed and live on GitHub Pages.
- Mobile-responsive (Spec-Kit Plus already handles this).
- All images/diagrams generated with Mermaid or publicly usable sources.
- Include complete hardware guide (Digital Twin Workstation + Economy Jetson Kit + Robot options) exactly as in the original course spec.

### Constraints
- Zero plagiarism – everything MUST be originally written or properly attributed.
- MUST strictly follow the official 4 modules and 13-week breakdown provided by Panaversity:
  - Module 1: The Robotic Nervous System (ROS 2) - Weeks 3-5: ROS 2 Nodes, Topics, and Services; Bridging Python Agents to ROS controllers using rclpy; Understanding URDF (Unified Robot Description Format) for humanoids
  - Module 2: The Digital Twin (Gazebo & Unity) - Weeks 6-7: Physics simulation and environment building; Simulating physics, gravity, and collisions in Gazebo; High-fidelity rendering and human-robot interaction in Unity; Simulating sensors: LiDAR, Depth Cameras, and IMUs
  - Module 3: The AI-Robot Brain (NVIDIA Isaac™) - Weeks 8-10: NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation; Isaac ROS: Hardware-accelerated VSLAM (Visual SLAM) and navigation; Nav2: Path planning for bipedal humanoid movement
  - Module 4: Vision-Language-Action (VLA) - Week 13: The convergence of LLMs and Robotics; Voice-to-Action: Using OpenAI Whisper for voice commands; Cognitive Planning: Using LLMs to translate natural language ("Clean the room") into a sequence of ROS 2 actions
  - Weeks 1-2: Introduction to Physical AI: Foundations of Physical AI and embodied intelligence; From digital AI to robots that understand physical laws; Overview of humanoid robotics landscape; Sensor systems: LIDAR, cameras, IMUs, force/torque sensors
  - Weeks 11-12: Humanoid Robot Development: Humanoid robot kinematics and dynamics; Bipedal locomotion and balance control; Manipulation and grasping with humanoid hands; Natural human-robot interaction design
- Capstone project MUST be fully specified with step-by-step instructions: The Autonomous Humanoid - A simulated robot that receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it
- MUST include "Why Physical AI Matters" preface and future-of-work context: Humanoid robots are poised to excel in our human-centered world because they share our physical form and can be trained with abundant data from interacting in human environments. This represents a significant transition from AI models confined to digital environments to embodied intelligence that operates in physical space.
- No fictional hardware or fake benchmarks.

### Success Criteria (What the judges will check)
- Book is live on GitHub Pages with a clean, professional URL.
- All 4 modules + 13 weeks + capstone are complete and navigable.
- At least 50+ pages of original, high-quality content.
- Contains working code snippets, Mermaid diagrams, quizzes, and labs.
- Hardware section is honest and matches the original course requirements.
- Built 100% using Spec-Kit Plus (visible in repo structure and commit history).
- Uses AI (Claude/Gemini) transparently in the making (mention in README is a bonus).
- Feels like a real Panaversity textbook – beautiful, interactive, and inspiring.

### Bonus Points (not required but will make you win)
- Add embedded YouTube videos of real humanoid robots (Unitree G1, Figure 01, Boston Dynamics Atlas, Tesla Optimus).
- Include a “Student Gallery” page (empty but ready for future submissions).
- Add a contributor section thanking Zia, Rehan, Junaid, Wania and the Panaversity team.
- Deploy within 7 days of hackathon start.

## Governance

The Constitution supersedes all other practices and serves as the ultimate source of truth for project principles and standards.
Amendments to this constitution require thorough documentation of the changes, explicit approval from project stakeholders, and a clear migration plan for any affected systems or processes.
All Pull Requests and code reviews MUST explicitly verify compliance with the principles and standards outlined in this constitution.
Any proposed increase in complexity within the project MUST be thoroughly justified against the core principles and overall project goals.

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
