# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/1-physical-ai-textbook/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Paths shown below assume single project - adjust based on plan.md structure

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create root Docusaurus project structure
- [x] T002 Initialize npm dependencies in package.json
- [x] T003 Create `src/modules`, `src/weeks`, `src/capstone`, `src/assets`, `src/components` directories
- [x] T004 Create `src/assets/images`, `src/assets/videos`, `src/assets/mermaid-diagrams` subdirectories

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Configure `docusaurus.config.js` with project metadata and sidebar options
- [x] T006 Implement `sidebars.js` to reflect the 4 modules and 13-week structure
- [x] T007 Configure GitHub Pages deployment in `docusaurus.config.js`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Textbook Content Consumption (Priority: P1) 🎯 MVP

**Goal**: Readers can navigate the interactive, mobile-friendly textbook, including reading modules, quizzes, labs, and code sandbox links.

**Independent Test**: Access the deployed GitHub Pages site on various devices (desktop, mobile) and verify that all content renders correctly and interactive elements function as expected, delivering a seamless learning experience.

### Implementation for User Story 1

- [x] T008 [US1] Create Frontmatter & "Why Physical AI Matters" preface content in `src/frontmatter/index.md`
- [x] T009 [P] [US1] Create Module 1 (The Robotic Nervous System - ROS 2) content for Weeks 3-5 in `src/modules/01-ros2/index.md`
- [x] T010 [P] [US1] Create Module 2 (The Digital Twin - Gazebo & Unity) content for Weeks 6-7 in `src/modules/02-digital-twin/index.md`
- [x] T011 [P] [US1] Create Module 3 (The AI-Robot Brain - NVIDIA Isaac™) content for Weeks 8-10 in `src/modules/03-isaac/index.md`
- [x] T012 [P] [US1] Create Module 4 (Vision-Language-Action - VLA) content for Week 13 in `src/modules/04-vla/index.md`
- [x] T013 [P] [US1] Create Introduction to Physical AI content for Weeks 1-2 in `src/weeks/week-01_02/index.md`
- [x] T014 [P] [US1] Create Humanoid Robot Development content for Weeks 11-12 in `src/weeks/week-11_12/index.md`
- [x] T015 [US1] Add example Mermaid diagram to `src/modules/01-ros2/index.md` and save diagram source in `src/assets/mermaid-diagrams/ros2_arch.mmd`
- [x] T016 [US1] Add example interactive quiz to `src/modules/01-ros2/index.md` and corresponding quiz data
- [x] T017 [US1] Add example lab with code sandbox link to `src/modules/01-ros2/index.md`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Practical Skill Acquisition (Priority: P1)

**Goal**: Readers can follow code examples and practical exercises to gain hands-on experience in ROS 2, robot simulation, and VLA pipeline development.

**Independent Test**: A target audience member follows the installation and code example instructions from start to finish on the specified hardware, verifying that each practical outcome (e.g., ROS 2 running, robot simulated, VSLAM deployed) is achieved.

### Implementation for User Story 2

- [x] T018 [US2] Develop ROS 2 installation guide in `src/modules/01-ros2/installation.md`
- [x] T019 [P] [US2] Develop Gazebo simulation guide in `src/modules/02-digital-twin/gazebo_sim.md`
- [x] T020 [P] [US2] Develop NVIDIA Isaac Sim guide in `src/modules/03-isaac/isaac_sim.md`
- [x] T021 [US2] Add Python ROS 2 publisher example in `src/modules/01-ros2/code_examples/publisher.py` and link from `src/modules/01-ros2/index.md`
- [x] T022 [US2] Add Python ROS 2 subscriber example in `src/modules/01-ros2/code_examples/subscriber.py` and link from `src/modules/01-ros2/index.md`
- [x] T023 [P] [US2] Develop VSLAM deployment guide for Jetson Orin in `src/modules/03-isaac/vslam_jetson.md`
- [x] T024 [P] [US2] Develop Nav2 deployment guide for Jetson Orin in `src/modules/03-isaac/nav2_jetson.md`
- [x] T025 [US2] Develop VLA pipeline code example in `src/modules/04-vla/code_examples/vla_pipeline.py` and link from `src/modules/04-vla/index.md`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Capstone Project Completion (Priority: P1)

**Goal**: Readers can successfully complete the capstone project, demonstrating autonomous navigation, object recognition, and manipulation using natural language instructions.

**Independent Test**: A reader, having completed previous modules, successfully sets up and runs the capstone project in simulation, and demonstrates all specified functionalities (navigation, object recognition, manipulation via natural language).

### Implementation for User Story 3

- [x] T026 [US3] Create Capstone Project introduction and overview in `src/capstone/index.md`
- [x] T027 [US3] Create detailed Capstone Project step-by-step guide in `src/capstone/tasks.md`
- [x] T028 [P] [US3] Develop autonomous simulated humanoid control code in `src/capstone/robot_control.py`
- [x] T029 [P] [US3] Develop object recognition component for capstone in `src/capstone/object_recognition.py`
- [x] T030 [P] [US3] Develop natural language instruction interface for capstone in `src/capstone/nlp_interface.py`

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - Hardware Selection and Budgeting (Priority: P2)

**Goal**: Readers can use the provided hardware guide to choose and budget for appropriate robotics hardware based on their needs and financial constraints.

**Independent Test**: A reader consults the hardware guide, compares options, and accurately identifies the components and estimated budget required for various robotics setups (e.g., a Jetson kit vs. a full Unitree G1 lab).

### Implementation for User Story 4

- [x] T031 [US4] Create Hardware Guide content in `src/hardware-guide/index.md`
- [x] T032 [US4] Detail Digital Twin Workstation specifications in `src/hardware-guide/index.md`
- [x] T033 [US4] Detail Economy Jetson Kit (~$700) options and pricing in `src/hardware-guide/index.md`
- [x] T034 [US4] Detail Robot options (e.g., Unitree Go2 → G1) with pricing in `src/hardware-guide/index.md`
- [x] T035 [US4] Detail Cloud alternative (AWS/NVIDIA Omniverse) options and costs in `src/hardware-guide/index.md`

---

## Phase 7: Bonus Features (Priority: P3)

**Goal**: Implement bonus features that will enhance the textbook and potentially earn bonus points in the hackathon.

### Implementation for Bonus Features

- [x] T036 [BF] Create Student Gallery page in `src/gallery/index.md` (empty but ready for future submissions)
- [x] T037 [BF] Create contributor acknowledgment section in `src/contributors/index.md` thanking Zia, Rehan, Junaid, Wania and the Panaversity team

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T038 Review all textbook content for clarity, accuracy, and tone across all modules/weeks
- [x] T039 Verify all internal and external links throughout the textbook
- [x] T040 Perform mobile-friendliness and responsiveness testing for the Docusaurus site
- [x] T041 Final review against all original success criteria in `specs/1-physical-ai-textbook/spec.md`
- [x] T042 Prepare and finalize GitHub Pages deployment steps

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Integrates with US1 examples.
- **User Story 3 (P1)**: Can start after Foundational (Phase 2) - Builds upon US1 and US2 knowledge.
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - Independent from other content stories.

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Task ID Reference

- T001-T004: Phase 1 (Setup)
- T005-T007: Phase 2 (Foundational)
- T008-T017: Phase 3 (User Story 1 - Textbook Content)
- T018-T025: Phase 4 (User Story 2 - Practical Skills)
- T026-T030: Phase 5 (User Story 3 - Capstone)
- T031-T035: Phase 6 (User Story 4 - Hardware Guide)
- T036-T037: Phase 7 (Bonus Features)
- T038-T042: Phase N (Polish & Cross-Cutting)

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, User Stories 1, 2, 3 and 4 can conceptually start in parallel (if team capacity allows, though US2/3 have conceptual dependencies on earlier modules from US1)
- Tasks within a story marked [P] can run in parallel (e.g., creating multiple content files, or different code examples)
- Different user stories can be worked on in parallel by different team members for distinct content sections (e.g., one person on hardware guide, another on a module content).

---

## Parallel Example: User Story 1

```bash
# Example parallel creation of module content
Task: "Create Module 1 (ROS 2) index content for Weeks 1-5 in src/modules/01-ros2/index.md"
Task: "Create Module 2 (Digital Twin) index content for Weeks 6-7 in src/modules/02-digital-twin/index.md"
Task: "Create Module 3 (NVIDIA Isaac) index content for Weeks 8-10 in src/modules/03-isaac/index.md"
Task: "Create Module 4 (VLA) index content for Week 13 in src/modules/04-vla/index.md"
Task: "Create Humanoid-specific Development content for Weeks 11-12 in src/weeks/week-11_12/index.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently (textbook content rendering, navigation, basic interactivity)
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (content creation)
   - Developer B: User Story 2 (practical skill acquisition, code examples)
   - Developer C: User Story 3 (capstone project development)
   - Developer D: User Story 4 (hardware guide)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
