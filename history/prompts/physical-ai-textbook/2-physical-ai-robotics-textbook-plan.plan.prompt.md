---
id: 2
title: Physical AI Robotics Textbook Plan
stage: plan
date: 2025-12-05
surface: agent
model: claude-3-5-sonnet-20241022
feature: physical-ai-textbook
branch: 1-physical-ai-textbook
user: unknown
command: "/sp.plan Physical AI & Humanoid Robotics - AI-Native Textbook\n\nArchitecture sketch (Spec-Kit Plus + Docusaurus)\n- Repository structure: 100 % Spec-Kit Plus standard\n  ├─ src/\n  │   ├─ modules/01-ros2/, 02-digital-twin/, 03-isaac/, 04-vla/\n  │   ├─ weeks/week-01/ → week-13/\n  │   ├─ capstone/\n  │   └─ assets/ (images, videos, mermaid diagrams)\n  ├─ spec.yaml (master blueprint - will be generated next)\n  └─ docusaurus.config.js (already perfect from template)\n- Deployment: GitHub Pages (automatic on push to main)\n- Interactivity: Mermaid live diagrams, collapsible code blocks, built-in search\n\nSection structure (exactly matches Panaversity official outline)\n1. Frontmatter & Preface\n   - Why Physical AI Matters\n   - From AI Agents to Embodied Intelligence\n   - Learning Outcomes\n2. Hardware Guide (complete & honest)\n   - Digital Twin Workstation spec\n   - Economy Jetson Kit (~$700)\n   - Robot options (Go2 → G1)\n   - Cloud alternative (AWS/NVIDIA Omniverse)\n3. Module 1 → Weeks 1-5  : ROS 2 - The Robotic Nervous System\n4. Module 2 → Weeks 6-7  : The Digital Twin (Gazebo + Unity)\n5. Module 3 → Weeks 8-10 : The AI-Robot Brain (NVIDIA Isaac Platform)\n6. Module 4 → Week 13     : Vision-Language-Action (VLA) + Conversational Robotics\n7. Weeks 11-12            : Humanoid-specific Development\n8. Capstone Project      : Full Autonomous Simulated Humanoid\n9. Appendices             : Setup guides, troubleshooting, resources\n\nExecution approach (AI-accelerated concurrent writing)\n- Primary tool: Claude 3.5 Sonnet OR Gemini 1.5 Pro via Claude Code router\n- One module at a time → generate all markdown + diagrams + code in a single prompt\n- Immediate commit & preview after every module\n- No upfront research phase - research happens inside Claude prompts (2025 knowledge is fresh)\n\nKey decisions & trade-offs (already locked for hackathon success)\n1. Simulation focus: Isaac Sim + Gazebo (not MuJoCo or Webots) → matches Panaversity stack\n2. ROS 2 version: Humble (LTS) → maximum stability for students\n3. LLM integration: OpenAI Whisper + any open-source VLA (e.g., OpenVLA, RT-2) → no paid API keys required for capstone\n4. Real robot: Unitree Go2 as recommended proxy → cheapest real hardware that runs full stack\n5. Depth vs speed: Prioritize runnable code + clear explanations over theoretical depth\n\nQuality validation & testing strategy\n- Every generated file will be instantly viewable at http://localhost:3000 after `npm run start`\n- Success checklist per module:\n  □ All Mermaid diagrams render correctly\n  □ Every code block has ```python tag and is < 100 lines\n  □ At least one hands-on lab with step-by-step commands\n  □ Quiz section with 5-8 questions (multiple choice + open)\n  □ Hardware notes clearly mention cost and alternatives\n  □ Tone is beginner-friendly and exciting\n- Final acceptance criteria (must be TRUE before submission):\n  □ Book is live on GitHub Pages\n  □ All 4 modules + 13 weeks + capstone are complete and linked\n  □ Hardware guide matches the original spec 100 %\n  □ Zero broken links or syntax errors\n  □ Mobile-friendly and beautiful\n\nTimeline (hackathon-optimized - 5 days total)\nDay 1 - Done: .constitution + /sp.specify + /sp.plan\nDay 2 - spec.yaml + Module 1 (ROS 2) complete\nDay 3 - Module 2 + Module 3\nDay 4 - Module 4 + Capstone + Hardware Guide\nDay 5 - Polish, videos, final deploy, submission\n\nWe are now 100 % aligned with Spec-Kit Plus methodology.\nNext command: generate the master spec.yaml"
labels: ["plan","textbook","robotics","ai"]
links:
  spec: specs/1-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-physical-ai-textbook/plan.md
  - spec.yaml
tests:
null
---

## Prompt

Physical AI & Humanoid Robotics - AI-Native Textbook

Architecture sketch (Spec-Kit Plus + Docusaurus)
- Repository structure: 100 % Spec-Kit Plus standard
  ├─ src/
  │   ├─ modules/01-ros2/, 02-digital-twin/, 03-isaac/, 04-vla/
  │   ├─ weeks/week-01/ → week-13/
  │   ├─ capstone/
  │   └─ assets/ (images, videos, mermaid diagrams)
  ├─ spec.yaml (master blueprint - will be generated next)
  └─ docusaurus.config.js (already perfect from template)
- Deployment: GitHub Pages (automatic on push to main)
- Interactivity: Mermaid live diagrams, collapsible code blocks, built-in search

Section structure (exactly matches Panaversity official outline)
1. Frontmatter & Preface
   - Why Physical AI Matters
   - From AI Agents to Embodied Intelligence
   - Learning Outcomes
2. Hardware Guide (complete & honest)
   - Digital Twin Workstation spec
   - Economy Jetson Kit (~$700)
   - Robot options (Go2 → G1)
   - Cloud alternative (AWS/NVIDIA Omniverse)
3. Module 1 → Weeks 1-5  : ROS 2 - The Robotic Nervous System
4. Module 2 → Weeks 6-7  : The Digital Twin (Gazebo + Unity)
5. Module 3 → Weeks 8-10 : The AI-Robot Brain (NVIDIA Isaac Platform)
6. Module 4 → Week 13     : Vision-Language-Action (VLA) + Conversational Robotics
7. Weeks 11-12            : Humanoid-specific Development
8. Capstone Project      : Full Autonomous Simulated Humanoid
9. Appendices             : Setup guides, troubleshooting, resources

Execution approach (AI-accelerated concurrent writing)
- Primary tool: Claude 3.5 Sonnet OR Gemini 1.5 Pro via Claude Code router
- One module at a time → generate all markdown + diagrams + code in a single prompt
- Immediate commit & preview after every module
- No upfront research phase - research happens inside Claude prompts (2025 knowledge is fresh)

Key decisions & trade-offs (already locked for hackathon success)
1. Simulation focus: Isaac Sim + Gazebo (not MuJoCo or Webots) → matches Panaversity stack
2. ROS 2 version: Humble (LTS) → maximum stability for students
3. LLM integration: OpenAI Whisper + any open-source VLA (e.g., OpenVLA, RT-2) → no paid API keys required for capstone
4. Real robot: Unitree Go2 as recommended proxy → cheapest real hardware that runs full stack
5. Depth vs speed: Prioritize runnable code + clear explanations over theoretical depth

Quality validation & testing strategy\n- Every generated file will be instantly viewable at http://localhost:3000 after `npm run start`\n- Success checklist per module:\n  □ All Mermaid diagrams render correctly\n  □ Every code block has ```python tag and is < 100 lines\n  □ At least one hands-on lab with step-by-step commands\n  □ Quiz section with 5-8 questions (multiple choice + open)\n  □ Hardware notes clearly mention cost and alternatives\n  □ Tone is beginner-friendly and exciting\n- Final acceptance criteria (must be TRUE before submission):\n  □ Book is live on GitHub Pages\n  □ All 4 modules + 13 weeks + capstone are complete and linked\n  □ Hardware guide matches the original spec 100 %\n  □ Zero broken links or syntax errors\n  □ Mobile-friendly and beautiful\n\nTimeline (hackathon-optimized - 5 days total)\nDay 1 - Done: .constitution + /sp.specify + /sp.plan\nDay 2 - spec.yaml + Module 1 (ROS 2) complete\nDay 3 - Module 2 + Module 3\nDay 4 - Module 4 + Capstone + Hardware Guide\nDay 5 - Polish, videos, final deploy, submission\n\nWe are now 100 % aligned with Spec-Kit Plus methodology.\nNext command: generate the master spec.yaml

## Response snapshot

The implementation plan for "Physical AI & Humanoid Robotics Textbook" has been successfully created.\n\n**Branch**: `1-physical-ai-textbook`\n**Plan File**: `specs/1-physical-ai-textbook/plan.md`\n**Generated Artifacts**: `spec.yaml`\nThe plan is ready. The next command is to proceed with the implementation, which will involve using `/sp.tasks` or directly creating modules based on the plan.\n
## Outcome

- ✅ Impact: Implementation plan for Physical AI & Humanoid Robotics Textbook created, and spec.yaml generated.
- 🧪 Tests: Manual validation of the plan against the input description performed.
- 📁 Files: Created specs/1-physical-ai-textbook/plan.md and spec.yaml.
- 🔁 Next prompts: User can now proceed with /sp.tasks or direct module generation.
- 🧠 Reflection: Successfully manually created plan.md and spec.yaml after script failure.

## Evaluation notes (flywheel)

- Failure modes observed: PowerShell script for plan setup failed.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Investigate PowerShell execution or use Bash alternatives for all setup scripts.
