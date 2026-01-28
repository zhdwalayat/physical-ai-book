---
sidebar_position: 1
---

# Week 14: Capstone Project

## Project Overview

The capstone project demonstrates mastery of all course content by building a complete **Vision-Language-Action (VLA) humanoid robot system**. You will create a simulated humanoid robot that can:

1. **Receive** voice commands via speech recognition (Whisper)
2. **Understand** commands using Large Language Model reasoning
3. **Plan** action sequences with context awareness
4. **Navigate** environments while avoiding obstacles
5. **Perceive** and identify target objects using computer vision
6. **Manipulate** objects to complete requested tasks

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        Capstone System Architecture                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │                         HUMAN INTERFACE LAYER                          │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                │ │
│  │  │  Microphone  │  │   Display    │  │   Speaker    │                │ │
│  │  │  (ReSpeaker) │  │   (RViz2)    │  │   (TTS)      │                │ │
│  │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘                │ │
│  └─────────┼─────────────────┼─────────────────┼────────────────────────┘ │
│            │                 │                 │                           │
│            ▼                 │                 ▲                           │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │                         COGNITION LAYER                                │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                │ │
│  │  │   Whisper    │  │    GPT-4     │  │   Action     │                │ │
│  │  │    ASR       │──│  Task Plan   │──│   Parser     │                │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘                │ │
│  │                           │                                           │ │
│  │  ┌──────────────┐        ▼         ┌──────────────┐                 │ │
│  │  │   Scene      │  ┌──────────┐   │   Dialog     │                 │ │
│  │  │   Graph      │◄─│ Reasoner │──►│   Manager    │                 │ │
│  │  └──────────────┘  └──────────┘   └──────────────┘                 │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
│                           │                                                 │
│                           ▼                                                 │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │                         PERCEPTION LAYER                               │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                │ │
│  │  │   Camera     │  │    VSLAM     │  │   Object     │                │ │
│  │  │   Driver     │──│   (cuVSLAM)  │──│   Detector   │                │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘                │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
│                           │                                                 │
│                           ▼                                                 │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │                          ACTION LAYER                                  │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                │ │
│  │  │    Nav2      │  │    MoveIt    │  │   Gripper    │                │ │
│  │  │  Navigation  │  │     IK       │  │   Control    │                │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘                │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
│                           │                                                 │
│                           ▼                                                 │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │                       SIMULATION LAYER                                 │ │
│  │  ┌─────────────────────────────────────────────────────────────────┐ │ │
│  │  │                    Gazebo / Isaac Sim                            │ │ │
│  │  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │ │ │
│  │  │  │ Humanoid │  │  Room    │  │ Objects  │  │ Physics  │       │ │ │
│  │  │  │  Robot   │  │  World   │  │ (Target) │  │  Engine  │       │ │ │
│  │  │  └──────────┘  └──────────┘  └──────────┘  └──────────┘       │ │ │
│  │  └─────────────────────────────────────────────────────────────────┘ │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Project Requirements

### Core Requirements (All Required)

| Requirement | Module | Description | Points |
|-------------|--------|-------------|--------|
| **ROS 2 Package** | Module 1 | Proper package structure with nodes | 15 |
| **Simulation World** | Module 2 | Gazebo world with robot and objects | 15 |
| **Perception System** | Module 3 | Camera-based object detection | 20 |
| **Voice Interface** | Module 4 | Whisper speech recognition | 15 |
| **Task Planning** | Module 4 | LLM-based action planning | 20 |
| **Integration Demo** | All | End-to-end task completion | 15 |
| **Total** | | | **100** |

### Minimum Viable Product (MVP)

Your system must demonstrate at minimum:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Minimum Requirements                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ✓ Robot spawns in Gazebo world with functional sensors                     │
│  ✓ Voice command "go to the kitchen" → robot navigates to kitchen           │
│  ✓ Voice command "pick up the cup" → robot attempts grasp (simulation OK)   │
│  ✓ LLM generates valid action plan from natural language                    │
│  ✓ Object detection identifies at least one object class                    │
│  ✓ Complete demo video showing end-to-end task                              │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Extension Options (Bonus Points)

| Extension | Points | Description |
|-----------|--------|-------------|
| Isaac Sim integration | +10 | Use Isaac Sim instead of Gazebo |
| Real hardware demo | +15 | Deploy to actual robot/Jetson |
| Multi-turn dialogue | +5 | Handle clarification questions |
| Multiple objects | +5 | Track and manipulate multiple objects |
| Recovery behaviors | +5 | Graceful handling of failures |
| ChatKit integration | +5 | Interactive documentation |

---

## Deliverables

### 1. GitHub Repository

```
capstone_humanoid/
├── README.md                      # Setup and usage instructions
├── LICENSE
├── .github/
│   └── workflows/
│       └── ci.yml                 # CI/CD pipeline
│
├── humanoid_bringup/              # Main launch package
│   ├── package.xml
│   ├── setup.py
│   ├── launch/
│   │   ├── simulation_launch.py   # Start Gazebo + robot
│   │   ├── perception_launch.py   # Start perception nodes
│   │   ├── navigation_launch.py   # Start Nav2
│   │   └── full_system_launch.py  # Complete system
│   ├── config/
│   │   ├── robot_params.yaml
│   │   ├── nav2_params.yaml
│   │   └── perception_params.yaml
│   └── rviz/
│       └── humanoid.rviz
│
├── humanoid_description/          # Robot model
│   ├── package.xml
│   ├── urdf/
│   │   └── humanoid.urdf.xacro
│   ├── meshes/
│   └── config/
│
├── humanoid_gazebo/               # Simulation world
│   ├── package.xml
│   ├── worlds/
│   │   └── apartment.sdf
│   └── models/
│       ├── table/
│       ├── cup/
│       └── ...
│
├── humanoid_perception/           # Perception nodes
│   ├── package.xml
│   ├── setup.py
│   ├── humanoid_perception/
│   │   ├── __init__.py
│   │   ├── object_detector.py
│   │   └── scene_graph.py
│   └── models/                    # ML models
│
├── humanoid_speech/               # Speech processing
│   ├── package.xml
│   ├── setup.py
│   └── humanoid_speech/
│       ├── __init__.py
│       ├── speech_recognition.py
│       └── text_to_speech.py
│
├── humanoid_planner/              # Task planning
│   ├── package.xml
│   ├── setup.py
│   └── humanoid_planner/
│       ├── __init__.py
│       ├── task_planner.py
│       └── action_executor.py
│
├── humanoid_msgs/                 # Custom messages
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── msg/
│   │   ├── DetectedObject.msg
│   │   └── TaskPlan.msg
│   └── srv/
│       └── PlanTask.srv
│
└── docs/                          # Documentation
    ├── architecture.md
    ├── setup.md
    └── demo.md
```

### 2. Required ROS 2 Nodes

| Node | Package | Function | Key Topics |
|------|---------|----------|------------|
| `speech_node` | humanoid_speech | Whisper transcription | Pub: `/speech/text` |
| `tts_node` | humanoid_speech | Text-to-speech | Sub: `/robot/speak` |
| `planner_node` | humanoid_planner | LLM task planning | Srv: `/plan_task` |
| `executor_node` | humanoid_planner | Action execution | Sub: `/task_plan` |
| `detector_node` | humanoid_perception | Object detection | Pub: `/detected_objects` |
| `scene_node` | humanoid_perception | Scene understanding | Pub: `/scene_graph` |
| `coordinator_node` | humanoid_bringup | State machine | All above |

### 3. Demo Video Requirements

Your 3-5 minute demo video must show:

1. **System startup** (30s)
   - Launch file execution
   - RViz visualization
   - Gazebo simulation

2. **Voice command** (30s)
   - Wake word activation
   - Speech transcription
   - LLM planning output

3. **Navigation** (60s)
   - Path planning visualization
   - Obstacle avoidance
   - Goal reaching

4. **Object interaction** (60s)
   - Object detection
   - Approach motion
   - Grasp attempt

5. **Task completion** (30s)
   - Success confirmation
   - Robot response

6. **Code walkthrough** (60s)
   - Key components explanation
   - Design decisions

---

## Technical Specifications

### ROS 2 Node Graph

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ROS 2 Node Graph                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌──────────────┐                                                          │
│   │  /microphone │                                                          │
│   │   (driver)   │                                                          │
│   └──────┬───────┘                                                          │
│          │ /audio                                                           │
│          ▼                                                                   │
│   ┌──────────────┐    /speech/text     ┌──────────────┐                    │
│   │ /speech_node │────────────────────►│ /planner_node│                    │
│   │  (Whisper)   │                     │   (GPT-4)    │                    │
│   └──────────────┘                     └──────┬───────┘                    │
│                                               │                             │
│                                               │ /task_plan                  │
│                                               ▼                             │
│   ┌──────────────┐    /scene_graph     ┌──────────────┐                    │
│   │ /scene_node  │────────────────────►│ /coordinator │                    │
│   │              │                     │    _node     │                    │
│   └──────────────┘                     └──────┬───────┘                    │
│          ▲                                    │                             │
│          │ /detected_objects                  │                             │
│   ┌──────┴───────┐                           │                             │
│   │ /detector    │                           │                             │
│   │    _node     │                           │                             │
│   └──────┬───────┘                           │                             │
│          │                                    │                             │
│          │ /camera/image                      │                             │
│          │                                    │                             │
│   ┌──────┴───────┐    /joint_states          │ /cmd_vel                    │
│   │              │◄──────────────────────────┼──────────┐                  │
│   │   Gazebo     │                           │          │                  │
│   │              │    /odom                   │          │                  │
│   │              │───────────────────────────┤          │                  │
│   └──────────────┘                           │          │                  │
│                                               ▼          │                  │
│                                        ┌──────────────┐ │                  │
│                                        │ /nav2_...    │─┘                  │
│                                        │ (Navigation) │                    │
│                                        └──────────────┘                    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Custom Messages

```python
# msg/DetectedObject.msg
std_msgs/Header header
string class_name
float32 confidence
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions
bool graspable

# msg/TaskPlan.msg
std_msgs/Header header
string original_command
bool feasible
string response
humanoid_msgs/Action[] actions
float32 estimated_duration

# msg/Action.msg
string type  # navigate, pick, place, speak, etc.
string[] param_names
string[] param_values

# srv/PlanTask.srv
# Request
string command
humanoid_msgs/DetectedObject[] scene_objects
geometry_msgs/Pose robot_pose
string holding_object
---
# Response
bool success
humanoid_msgs/TaskPlan plan
string error_message
```

### Example Task Flow

```yaml
Voice Command: "Get me the red ball from the table"

1. Speech Recognition (speech_node):
   Input: Audio waveform from microphone
   Output: Text "Get me the red ball from the table"
   Topic: /speech/text

2. Scene Understanding (scene_node):
   Input: /detected_objects, /robot_pose
   Output: Scene graph with object locations
   Topic: /scene_graph

3. Task Planning (planner_node):
   Input: Command + Scene graph
   LLM Prompt: "Plan actions for: Get me the red ball from the table"
   Output:
     feasible: true
     actions:
       - type: navigate, target: table
       - type: look, target: red ball
       - type: approach, target: red ball
       - type: pick, object: red ball
       - type: navigate, target: user
       - type: handover
     response: "I'll get the red ball from the table for you."

4. Execution (coordinator_node):
   For each action in plan:
     - navigate → Send goal to Nav2
     - look → Publish to /head/command
     - approach → Fine positioning with vision
     - pick → Arm IK + gripper control
     - handover → Extend arm, wait, release

5. Completion:
   Robot: "Here's the red ball!"
   Status: Task completed successfully
```

---

## Assessment Rubric

### Assessment 1: ROS 2 Package Structure (15 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Package organization | 3 | Follows ROS 2 conventions |
| Working nodes | 4 | Publisher/subscriber functional |
| Services/Actions | 4 | At least one service or action |
| Launch files | 2 | Single command starts system |
| Documentation | 2 | README with clear instructions |

**Evaluation:**
```bash
# Check package structure
ros2 pkg list | grep humanoid
colcon build --packages-select humanoid_bringup

# Test nodes
ros2 launch humanoid_bringup full_system_launch.py
ros2 node list
ros2 topic list
ros2 service list
```

### Assessment 2: Gazebo Simulation (15 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Robot model | 3 | URDF loads, joints move |
| Physics config | 3 | Appropriate friction, mass |
| Working sensors | 4 | Camera publishes, IMU functional |
| World design | 3 | Room with furniture, objects |
| ROS 2 bridge | 2 | Sensors bridged to ROS topics |

**Evaluation:**
```bash
# Launch simulation
ros2 launch humanoid_gazebo simulation_launch.py

# Verify sensors
ros2 topic echo /camera/image_raw --once
ros2 topic echo /imu --once
ros2 topic hz /camera/image_raw
```

### Assessment 3: Perception Pipeline (20 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Camera integration | 4 | Image subscription working |
| Object detection | 6 | Detects target objects |
| Localization | 6 | Pose estimation accurate |
| Performance | 4 | Runs at >10 FPS |

**Evaluation:**
```bash
# Test detection
ros2 launch humanoid_perception perception_launch.py
ros2 topic echo /detected_objects

# Check performance
ros2 topic hz /detected_objects
```

### Assessment 4: Voice Interface (15 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Whisper integration | 5 | Model loads, transcribes |
| Real-time processing | 5 | Latency < 2 seconds |
| Error handling | 5 | Graceful on no speech/noise |

**Evaluation:**
```bash
# Test speech recognition
ros2 launch humanoid_speech speech_launch.py
ros2 topic echo /speech/text

# Speak test command
# Expected output: Transcription appears within 2s
```

### Assessment 5: Task Planning (20 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| LLM integration | 5 | API calls work |
| Action generation | 5 | Valid action sequences |
| Context awareness | 5 | Uses scene information |
| Execution coordination | 5 | Actions execute in order |

**Evaluation:**
```bash
# Test planning
ros2 service call /plan_task humanoid_msgs/srv/PlanTask \
  "{command: 'get me the cup from the table'}"

# Verify action sequence is valid and executable
```

### Integration Demo (15 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| End-to-end completion | 8 | Task succeeds start to finish |
| Robustness | 4 | Handles command variations |
| Video quality | 3 | Clear, well-narrated |

---

## Sample Commands to Support

Your system should handle these commands (or similar):

### Navigation Commands
```
"Go to the kitchen"
"Move to the living room"
"Come here"
"Go near the table"
```

### Object Interaction Commands
```
"Pick up the red cup"
"Grab the book from the desk"
"Put the cup on the table"
"Bring me the remote"
```

### Compound Commands
```
"Get me the red ball from the table"
"Take the cup to the kitchen"
"Find the keys and bring them here"
"Clean up the toys on the floor"
```

### Conversational Commands
```
"What do you see?"
"Where is the cup?"
"Can you help me?"
"What are you holding?"
```

---

## Project Timeline

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Project Timeline                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   Week 12: Foundation                                                        │
│   ─────────────────────                                                     │
│   □ Project proposal submitted                                              │
│   □ Repository created with basic structure                                 │
│   □ Robot model loading in Gazebo                                           │
│   □ Basic ROS 2 nodes communicating                                         │
│                                                                              │
│   Week 13: Core Systems                                                      │
│   ────────────────────                                                      │
│   □ Speech recognition working                                              │
│   □ LLM integration complete                                                │
│   □ Navigation functional                                                    │
│   □ Object detection running                                                │
│   □ Progress check meeting                                                  │
│                                                                              │
│   Week 14: Integration & Demo                                                │
│   ────────────────────────────                                              │
│   □ Full system integration                                                 │
│   □ End-to-end testing                                                      │
│   □ Demo video recording                                                    │
│   □ Documentation complete                                                  │
│   □ Final submission                                                        │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Milestones

| Date | Milestone | Deliverable |
|------|-----------|-------------|
| Week 12 | Proposal | 1-page project plan |
| Week 13 | Progress Check | Working subsystems demo |
| Week 14 | Final Submission | Complete repository + video |
| Week 14+1 | Grades | Feedback released |

---

## Common Issues and Solutions

### Issue: Gazebo crashes on launch
```bash
# Solution: Check GPU drivers and reset Gazebo
killall gzserver gzclient
export LIBGL_ALWAYS_SOFTWARE=1  # If GPU issues
ros2 launch humanoid_gazebo simulation_launch.py
```

### Issue: Whisper too slow
```bash
# Solution: Use smaller model or GPU acceleration
# In speech_recognition.py:
model = whisper.load_model("tiny")  # Instead of "base"

# Or enable CUDA:
model = whisper.load_model("base", device="cuda")
```

### Issue: LLM returns invalid actions
```python
# Solution: Add validation and retry logic
def validate_plan(plan):
    valid_actions = ['navigate', 'pick', 'place', 'speak', 'look']
    for action in plan.actions:
        if action.type not in valid_actions:
            return False, f"Invalid action: {action.type}"
    return True, ""

# Retry with feedback if invalid
```

### Issue: Nav2 fails to plan path
```bash
# Solution: Check costmap and robot footprint
ros2 topic echo /local_costmap/costmap
ros2 param get /controller_server robot_radius

# Ensure robot footprint doesn't intersect obstacles
```

---

## Submission Checklist

### Required
- [ ] GitHub repository URL submitted
- [ ] README with setup instructions
- [ ] All packages build with `colcon build`
- [ ] `ros2 launch humanoid_bringup full_system_launch.py` starts system
- [ ] Demo video uploaded (YouTube/Vimeo link)
- [ ] Video shows end-to-end task completion

### Documentation
- [ ] Architecture diagram in docs/
- [ ] Setup instructions tested on clean machine
- [ ] Known limitations documented
- [ ] Future work section

### Code Quality
- [ ] Consistent code style (use `ament_flake8`)
- [ ] Docstrings on public functions
- [ ] No hardcoded paths (use parameters)
- [ ] Error handling present

---

## Getting Help

### Resources
- **Course Materials**: Review module content for implementations
- **ROS 2 Documentation**: [docs.ros.org](https://docs.ros.org)
- **Gazebo Tutorials**: [gazebosim.org](https://gazebosim.org)
- **OpenAI Whisper**: [github.com/openai/whisper](https://github.com/openai/whisper)

### Support Channels
- **Office Hours**: Schedule via course portal
- **Discussion Forum**: Post questions with code snippets
- **ChatKit**: Interactive Q&A (if enabled)

### Debugging Tips
1. Always check `ros2 topic list` and `ros2 node list` first
2. Use `ros2 topic echo` to verify message flow
3. Check Gazebo physics with `gz physics` command
4. Enable debug logging: `ros2 run <pkg> <node> --ros-args --log-level debug`

---

## Evaluation Criteria Summary

| Category | Points | Key Requirements |
|----------|--------|------------------|
| **ROS 2 Package** | 15 | Structure, nodes, launch |
| **Simulation** | 15 | Robot, world, sensors |
| **Perception** | 20 | Detection, localization |
| **Voice** | 15 | Whisper, real-time |
| **Planning** | 20 | LLM, actions, context |
| **Integration** | 15 | E2E demo, video |
| **Total** | **100** | |
| **Bonus** | +45 | Extensions (optional) |

---

**Good luck! This capstone represents the culmination of your Physical AI journey. Build something you're proud of!**

---

## Next Steps After Course

Having completed this course, you are prepared to:

1. **Industry Roles**
   - Robotics Software Engineer
   - AI/ML Engineer (Physical AI)
   - Simulation Engineer
   - Research Scientist

2. **Advanced Topics**
   - Reinforcement Learning for Control
   - Multi-Robot Systems
   - Human-Robot Interaction Research
   - Foundation Models for Robotics

3. **Hardware Projects**
   - Deploy to real humanoid platforms
   - Build custom robot with ROS 2
   - Create edge AI applications

4. **Research Directions**
   - Sim-to-real transfer improvements
   - Foundation models for robotics
   - Long-horizon task planning
   - Social robot interaction
