---
id: 1
title: Physical AI Robotics Textbook Specification
stage: spec
date: 2025-12-05
surface: agent
model: claude-3-5-sonnet-20241022
feature: physical-ai-textbook
branch: 1-physical-ai-textbook
user: unknown
command: "/sp.specify Physical AI & Humanoid Robotics – An AI-Native Textbook\n\nTarget audience:\n- Students who already know Python, basic deep learning, and AI agents\n- Aspiring roboticists and AI engineers aged 18–35\n- Complete beginners in robotics (never touched ROS or Gazebo before)\n\nFocus:\nBridging digital AI (LLMs, agents) with physical bodies. Teaching embodied intelligence through simulation → edge device → real humanoid robots using ROS 2, Gazebo, NVIDIA Isaac Sim, and Vision-Language-Action (VLA) models.\n\nSuccess criteria – After reading this textbook, the reader will be able to:\n- Install and run ROS 2 Humble/Iron on Ubuntu 22.04\n- Build and simulate a humanoid robot in Gazebo and NVIDIA Isaac Sim\n- Write Python ROS 2 nodes that control a real or simulated robot\n- Deploy perception (VSLAM) and navigation (Nav2) on a Jetson Orin\n- Create a Vision-Language-Action pipeline where a robot understands voice commands like “pick up the red cup” and executes them\n- Complete the full capstone: an autonomous simulated humanoid that navigates, recognizes objects, and manipulates them using only natural language instructions\n- Choose and budget the exact hardware needed (from $700 Jetson kit to full Unitree G1 lab)\n\nConstraints:\n- Must strictly follow the official 4 modules and 13-week structure given by Panaversity\n- Must be built 100 % with Spec-Kit Plus + Docusaurus and deployed on GitHub Pages\n- All code examples must run on standard student hardware (Ubuntu 22.04 + RTX 4070 Ti or better)\n- Must include the complete, honest hardware guide with prices and alternatives\n- Zero fictional content – every tool, robot model, price, and command must be real as of December 2025\n- Timeline: Complete and live before hackathon deadline\n\nNot building:\n- A complete ROS 2 from-scratch reference (assume reader can Google basic syntax)\n- In-depth mechanical engineering or control theory (this is a software + AI course)\n- Real-time kernel or low-level firmware programming\n- Training of foundation models from scratch (we fine-tune or use APIs)\n\nThis textbook WILL deliver:\n- A beautiful, interactive, mobile-friendly book at https://yourname.github.io/physical-ai-humanoid-robotics-textbook/\n- 4 complete modules + 13 weeks + full capstone project\n- Mermaid diagrams, quizzes, labs, code sandbox links\n- Honest hardware buyers’ guide with budget and premium options\n- Inspiration to join the embodied intelligence revolution\n\nThis is the textbook that turns AI agent builders into Physical AI engineers.\nLet’s ship it."
labels: ["specification","textbook","robotics","ai"]
links:
  spec: specs/1-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-physical-ai-textbook/spec.md
  - specs/1-physical-ai-textbook/checklists/requirements.md
tests:
null
---

## Prompt

Physical AI & Humanoid Robotics – An AI-Native Textbook

Target audience:
- Students who already know Python, basic deep learning, and AI agents
- Aspiring roboticists and AI engineers aged 18–35
- Complete beginners in robotics (never touched ROS or Gazebo before)

Focus:
Bridging digital AI (LLMs, agents) with physical bodies. Teaching embodied intelligence through simulation → edge device → real humanoid robots using ROS 2, Gazebo, NVIDIA Isaac Sim, and Vision-Language-Action (VLA) models.

Success criteria – After reading this textbook, the reader will be able to:
- Install and run ROS 2 Humble/Iron on Ubuntu 22.04
- Build and simulate a humanoid robot in Gazebo and NVIDIA Isaac Sim
- Write Python ROS 2 nodes that control a real or simulated robot
- Deploy perception (VSLAM) and navigation (Nav2) on a Jetson Orin
- Create a Vision-Language-Action pipeline where a robot understands voice commands like “pick up the red cup” and executes them
- Complete the full capstone: an autonomous simulated humanoid that navigates, recognizes objects, and manipulates them using only natural language instructions
- Choose and budget the exact hardware needed (from $700 Jetson kit to full Unitree G1 lab)

Constraints:
- Must strictly follow the official 4 modules and 13-week structure given by Panaversity
- Must be built 100 % with Spec-Kit Plus + Docusaurus and deployed on GitHub Pages
- All code examples must run on standard student hardware (Ubuntu 22.04 + RTX 4070 Ti or better)
- Must include the complete, honest hardware guide with prices and alternatives
- Zero fictional content – every tool, robot model, price, and command must be real as of December 2025
- Timeline: Complete and live before hackathon deadline

Not building:
- A complete ROS 2 from-scratch reference (assume reader can Google basic syntax)
- In-depth mechanical engineering or control theory (this is a software + AI course)
- Real-time kernel or low-level firmware programming
- Training of foundation models from scratch (we fine-tune or use APIs)

This textbook WILL deliver:
- A beautiful, interactive, mobile-friendly book at https://yourname.github.io/physical-ai-humanoid-robotics-textbook/
- 4 complete modules + 13 weeks + full capstone project
- Mermaid diagrams, quizzes, labs, code sandbox links
- Honest hardware buyers’ guide with budget and premium options
- Inspiration to join the embodied intelligence revolution

This is the textbook that turns AI agent builders into Physical AI engineers.
Let’s ship it.

## Response snapshot

The feature specification for "Physical AI & Humanoid Robotics Textbook" has been successfully created and validated.

Branch: `1-physical-ai-textbook`
Spec File: `specs/1-physical-ai-textbook/spec.md`
Checklist Results: All items passed.
The spec is ready for the next phase: `/sp.plan`.

## Outcome

- ✅ Impact: Specification for Physical AI & Humanoid Robotics Textbook created and validated.
- 🧪 Tests: No automated tests were run, manual validation of spec against checklist performed.
- 📁 Files: Created specs/1-physical-ai-textbook/spec.md and specs/1-physical-ai-textbook/checklists/requirements.md.
- 🔁 Next prompts: User can now run /sp.clarify or /sp.plan.
- 🧠 Reflection: PowerShell script for feature creation failed, performed steps manually.

## Evaluation notes (flywheel)

- Failure modes observed: PowerShell script 'create-new-feature.ps1' failed to execute.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Investigate PowerShell execution environment or provide alternative Bash scripts for feature creation.
