---
sidebar_position: 1
---

# Week 14: Capstone Project

## The Autonomous Humanoid

The capstone project demonstrates mastery of all course content. You will build a **simulated humanoid robot** that can:

1. **Receive** a voice command via speech recognition
2. **Understand** the command using natural language processing
3. **Plan** a sequence of actions using LLM-based reasoning
4. **Navigate** the environment while avoiding obstacles
5. **Identify** target objects using computer vision
6. **Manipulate** objects to complete the task

## Project Requirements

### Core Requirements (Must Complete All)

| Requirement | Module | Points |
|-------------|--------|--------|
| ROS 2 package with proper structure | Module 1 | 15 |
| Working Gazebo simulation | Module 2 | 15 |
| Isaac-based perception (or equivalent) | Module 3 | 20 |
| Voice command recognition | Module 4 | 15 |
| LLM task planning | Module 4 | 20 |
| End-to-end demo | Integration | 15 |
| **Total** | | **100** |

### Deliverables

1. **GitHub Repository** containing:
   - Complete ROS 2 package
   - Simulation world files
   - Launch files
   - Documentation (README)

2. **Deployed Book** on GitHub Pages or Vercel

3. **Demo Video** (3-5 minutes) showing:
   - Voice command input
   - Task planning output
   - Robot execution
   - Successful task completion

4. **ChatKit Integration** (optional bonus)

## Technical Specifications

### System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Capstone System Architecture                      │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────┐                                                    │
│  │   Microphone    │                                                    │
│  │   (ReSpeaker)   │                                                    │
│  └────────┬────────┘                                                    │
│           │ Audio                                                        │
│           ▼                                                              │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐    │
│  │    Whisper      │───►│    GPT-4        │───►│  Action Parser  │    │
│  │  (Speech-to-    │    │  (Task Planner) │    │                 │    │
│  │    Text)        │    │                 │    │                 │    │
│  └─────────────────┘    └─────────────────┘    └────────┬────────┘    │
│                                                         │              │
│                                                         ▼              │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐    │
│  │   RealSense     │───►│   Perception    │───►│   ROS 2 Node    │    │
│  │   Camera        │    │   Pipeline      │    │   Graph         │    │
│  └─────────────────┘    └─────────────────┘    └────────┬────────┘    │
│                                                         │              │
│           ┌─────────────────────────────────────────────┘              │
│           │                                                             │
│           ▼                                                             │
│  ┌─────────────────────────────────────────────────────────────┐      │
│  │                    Gazebo / Isaac Sim                        │      │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │      │
│  │  │  Humanoid   │  │ Environment │  │   Objects   │         │      │
│  │  │   Robot     │  │   (Room)    │  │  (Target)   │         │      │
│  │  └─────────────┘  └─────────────┘  └─────────────┘         │      │
│  └─────────────────────────────────────────────────────────────┘      │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Required ROS 2 Nodes

| Node | Function | Topics |
|------|----------|--------|
| `speech_node` | Whisper transcription | `/speech/text` |
| `planner_node` | LLM task planning | `/plan/actions` |
| `perception_node` | Object detection | `/perception/objects` |
| `navigation_node` | Nav2 wrapper | `/cmd_vel`, `/goal_pose` |
| `manipulation_node` | Grasp control | `/gripper/command` |
| `coordinator_node` | State machine | All above |

### Example Task Flow

**Voice Command**: "Get me the red ball from the table"

```yaml
1. Speech Recognition:
   Input: Audio waveform
   Output: "Get me the red ball from the table"

2. Task Planning (LLM):
   Input: Text command + scene context
   Output:
     - navigate_to: "table"
     - detect_object: "red ball"
     - approach_object: true
     - grasp_object: "red ball"
     - navigate_to: "user"
     - release_object: true

3. Execution:
   - Nav2 plans path to table
   - Perception detects red ball at (x, y, z)
   - Arm IK solves for grasp pose
   - Gripper closes
   - Nav2 plans path to user
   - Gripper opens
```

## Assessment Rubric

### Assessment 1: ROS 2 Package (15 points)

| Criteria | Points |
|----------|--------|
| Proper package structure | 3 |
| Working nodes (publisher/subscriber) | 4 |
| Services or actions implemented | 4 |
| Launch files functional | 2 |
| Code quality and documentation | 2 |

### Assessment 2: Gazebo Simulation (15 points)

| Criteria | Points |
|----------|--------|
| Robot model loads correctly | 3 |
| Physics properties configured | 3 |
| Sensors functional (camera, IMU) | 4 |
| Environment with objects | 3 |
| ROS 2 bridge working | 2 |

### Assessment 3: Perception Pipeline (20 points)

| Criteria | Points |
|----------|--------|
| Camera integration | 4 |
| Object detection working | 6 |
| Localization functional | 6 |
| Performance (>10 FPS) | 4 |

### Assessment 4: Voice Interface (15 points)

| Criteria | Points |
|----------|--------|
| Whisper integration | 5 |
| Real-time transcription | 5 |
| Error handling | 5 |

### Assessment 5: Task Planning (20 points)

| Criteria | Points |
|----------|--------|
| LLM integration | 5 |
| Action generation | 5 |
| Context awareness | 5 |
| Execution coordination | 5 |

### Integration Demo (15 points)

| Criteria | Points |
|----------|--------|
| End-to-end task completion | 8 |
| Robustness (handles variations) | 4 |
| Video quality and explanation | 3 |

## Submission Checklist

- [ ] GitHub repository with all code
- [ ] README with setup instructions
- [ ] Book deployed on GitHub Pages/Vercel
- [ ] Demo video uploaded
- [ ] All required nodes functional
- [ ] Launch file starts complete system

## Example Commands to Support

Your system should handle commands like:

1. "Go to the kitchen"
2. "Pick up the red cup"
3. "Bring me the book from the desk"
4. "Clean the table"
5. "Find the remote control"

## Getting Help

- Review module content for specific implementations
- Check course repository for starter code
- Use the ChatKit for questions
- Office hours for debugging support

## Grading Timeline

| Date | Milestone |
|------|-----------|
| Week 12 | Project proposal approved |
| Week 13 | Progress check (core systems working) |
| Week 14 | Final submission |
| Week 14+1 | Grades released |

---

**Good luck! This project represents the culmination of your Physical AI journey.**
