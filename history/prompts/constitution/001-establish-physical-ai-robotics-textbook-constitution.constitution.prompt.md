---
id: 001
title: Establish Physical AI Robotics Textbook Constitution
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: Physical AI & Humanoid Robotics Textbook
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "governance", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests: null
---

## Prompt

```text
Project: Physical AI & Humanoid Robotics – An AI-Native Textbook
Repository: https://github.com/Muhammed-Riaz/physical-ai-humanoid-robotics-textbook
Course URL: (will be filled after GitHub Pages deployment)

Core Principles:
- Beginner-first teaching: Every concept must be explainable to someone who knows Python + basic AI but has never touched a robot
- Hands-on over theory: Every major topic must include at least one runnable code example or lab
- Real-world relevance: Focus on technologies actually used in 2025 humanoid robots (ROS 2 Humble/Iron, NVIDIA Isaac Sim, Jetson Orin, Unitree G1/Go2, Vision-Language-Action models)
- Honesty about difficulty and cost: Never hide hardware requirements, costs, or the fact that this is computationally heavy
- Sim-to-real mindset: Always show the path from simulation → edge device → real robot

Key Standards:
- All code examples must be tested and runnable on Ubuntu 22.04 + ROS 2 Humble or Iron
- Use Mermaid diagrams for every architecture (ROS 2 graph, VLA pipeline, URDF structure, etc.)
- Every module and week must contain:
   • Learning objectives
   • Key concepts with analogies
   • At least one hands-on lab or mini-project
   • Quiz/questions (multiple choice + open-ended)
   • Further reading / video links
- Tone: Excited, encouraging, and practical (like a senior teammate teaching a junior)
- Language: English with clear, simple sentences (Flesch-Kincaid grade 8–10 so beginners can follow)
- Citation style: For tools and research papers use inline links or (Author, Year) – no strict APA needed

Technical Requirements:
- Built with Spec-Kit Plus + Docusaurus
- Fully deployed and live on GitHub Pages
- Mobile-responsive (Spec-Kit Plus already handles this)
- All images/diagrams generated with Mermaid or publicly usable sources
- Include complete hardware guide (Digital Twin Workstation + Economy Jetson Kit + Robot options) exactly as in the original course spec

Constraints:
- Zero plagiarism – everything must be originally written or properly attributed
- Must strictly follow the official 4 modules and 13-week breakdown provided by Panaversity
- Capstone project must be fully specified with step-by-step instructions
- Must include “Why Physical AI Matters” preface and future-of-work context
- No fictional hardware or fake benchmarks

Success Criteria (What the judges will check):
- Book is live on GitHub Pages with a clean, professional URL
- All 4 modules + 13 weeks + capstone are complete and navigable
- At least 50+ pages of original, high-quality content
- Contains working code snippets, Mermaid diagrams, quizzes, and labs
- Hardware section is honest and matches the original course requirements
- Built 100% using Spec-Kit Plus (visible in repo structure and commit history)
- Uses AI (Claude/Gemini) transparently in the making (mention in README is a bonus)
- Feels like a real Panaversity textbook – beautiful, interactive, and inspiring

Bonus Points (not required but will make you win):
- Add embedded YouTube videos of real humanoid robots (Unitree G1, Figure 01, Boston Dynamics Atlas, Tesla Optimus)
- Include a “Student Gallery” page (empty but ready for future submissions)
- Add a contributor section thanking Zia, Rehan, Junaid, Wania and the Panaversity team
- Deploy within 7 days of hackathon start

This textbook will bridge digital AI agents to physical bodies and inspire the next generation of roboticists.

Let’s build the future together. 🚀
```

## Response snapshot

```text
The project constitution at .specify/memory/constitution.md has been established with version 1.0.0. This is the initial creation, incorporating all core principles, standards, and requirements from the user's prompt.
```

## Outcome

- ✅ Impact: Constitution established successfully.
- 🧪 Tests: N/A
- 📁 Files: Updated .specify/memory/constitution.md
- 🔁 Next prompts: Suggested commit message provided to user.
- 🧠 Reflection: Successfully parsed user input and updated project constitution based on the provided template and instructions. Ensured versioning and date updates. Confirmed template alignment.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
