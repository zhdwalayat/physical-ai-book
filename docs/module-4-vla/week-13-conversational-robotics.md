---
sidebar_position: 3
---

# Week 13: Conversational Robotics

## Learning Objectives

By the end of this week, you will be able to:

- **Integrate** speech recognition using OpenAI Whisper
- **Design** LLM-based task planners for robot control
- **Build** complete voice-to-action pipelines
- **Implement** multi-modal human-robot interaction
- **Create** safe and robust conversational interfaces

---

## 1. Vision-Language-Action (VLA) Architecture

### 1.1 The VLA Pipeline

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                   Vision-Language-Action (VLA) Pipeline                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌───────────────────────────────────────────────────────────────────────┐ │
│   │                        PERCEPTION LAYER                                │ │
│   │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                │ │
│   │  │    Vision    │  │    Audio     │  │    Touch     │                │ │
│   │  │   (Camera)   │  │   (Microphone)│  │   (Force)   │                │ │
│   │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘                │ │
│   │         │                 │                 │                         │ │
│   │         ▼                 ▼                 ▼                         │ │
│   │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                │ │
│   │  │ Object Det.  │  │    ASR       │  │   Contact    │                │ │
│   │  │ Scene Graph  │  │  (Whisper)   │  │  Detection   │                │ │
│   │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘                │ │
│   └─────────┼─────────────────┼─────────────────┼─────────────────────────┘ │
│             │                 │                 │                           │
│             └─────────────────┼─────────────────┘                           │
│                               ▼                                             │
│   ┌───────────────────────────────────────────────────────────────────────┐ │
│   │                        REASONING LAYER                                 │ │
│   │  ┌────────────────────────────────────────────────────────────────┐  │ │
│   │  │                    Large Language Model                         │  │ │
│   │  │                    (GPT-4 / Claude / Llama)                     │  │ │
│   │  │                                                                  │  │ │
│   │  │    Input: Scene description + Speech transcription + History    │  │ │
│   │  │    Output: Task plan + Natural language response                │  │ │
│   │  └────────────────────────────────────────────────────────────────┘  │ │
│   └───────────────────────────────────────────────────────────────────────┘ │
│                               │                                             │
│                               ▼                                             │
│   ┌───────────────────────────────────────────────────────────────────────┐ │
│   │                        ACTION LAYER                                    │ │
│   │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                │ │
│   │  │    Motion    │  │    Speech    │  │    Grasp     │                │ │
│   │  │   Planning   │  │    Output    │  │   Planning   │                │ │
│   │  │  (Nav2/MPC)  │  │    (TTS)     │  │  (MoveIt)    │                │ │
│   │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘                │ │
│   │         │                 │                 │                         │ │
│   │         ▼                 ▼                 ▼                         │ │
│   │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                │ │
│   │  │  Locomotion  │  │   Speaker    │  │   Gripper    │                │ │
│   │  └──────────────┘  └──────────────┘  └──────────────┘                │ │
│   └───────────────────────────────────────────────────────────────────────┘ │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Key Components

| Component | Technology | Function |
|-----------|------------|----------|
| **ASR** | Whisper | Speech to text |
| **Scene Understanding** | YOLO + CLIP | Visual grounding |
| **Reasoning** | GPT-4 / Claude | Task planning |
| **Motion Planning** | Nav2 / MoveIt | Path generation |
| **TTS** | Coqui / ElevenLabs | Text to speech |

---

## 2. Speech Recognition with Whisper

### 2.1 Installation and Setup

```bash
# Install OpenAI Whisper
pip install openai-whisper

# Install audio dependencies
pip install sounddevice numpy scipy

# For faster inference (optional)
pip install faster-whisper
```

### 2.2 Real-Time Speech Recognition

```python
#!/usr/bin/env python3
"""
speech_recognition.py - Real-time speech recognition using Whisper
"""

import whisper
import numpy as np
import sounddevice as sd
import queue
import threading
from typing import Optional, Callable
from dataclasses import dataclass
import time


@dataclass
class SpeechConfig:
    """Configuration for speech recognition."""
    model_size: str = "base"      # tiny, base, small, medium, large
    language: str = "en"
    sample_rate: int = 16000
    chunk_duration: float = 0.5   # seconds
    silence_threshold: float = 0.01
    silence_duration: float = 1.0  # seconds of silence to end utterance
    max_duration: float = 30.0     # maximum utterance duration


class SpeechRecognizer:
    """Real-time speech recognition using Whisper."""

    def __init__(self, config: SpeechConfig = None):
        self.config = config or SpeechConfig()

        # Load Whisper model
        print(f"Loading Whisper model: {self.config.model_size}")
        self.model = whisper.load_model(self.config.model_size)

        # Audio state
        self.audio_queue = queue.Queue()
        self.is_recording = False
        self.stream = None

    def _audio_callback(self, indata, frames, time_info, status):
        """Callback for audio stream."""
        if status:
            print(f"Audio status: {status}")
        self.audio_queue.put(indata.copy())

    def start_stream(self):
        """Start audio input stream."""
        self.stream = sd.InputStream(
            samplerate=self.config.sample_rate,
            channels=1,
            dtype=np.float32,
            blocksize=int(self.config.sample_rate * self.config.chunk_duration),
            callback=self._audio_callback
        )
        self.stream.start()
        self.is_recording = True

    def stop_stream(self):
        """Stop audio input stream."""
        if self.stream:
            self.stream.stop()
            self.stream.close()
        self.is_recording = False

    def record_utterance(self, timeout: float = None) -> np.ndarray:
        """Record audio until silence is detected.

        Args:
            timeout: Maximum recording time (uses config.max_duration if None)

        Returns:
            Audio data as numpy array
        """
        timeout = timeout or self.config.max_duration
        chunks = []
        silence_start = None

        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                chunk = self.audio_queue.get(timeout=0.1)
                chunks.append(chunk)

                # Check for silence
                volume = np.abs(chunk).mean()
                if volume < self.config.silence_threshold:
                    if silence_start is None:
                        silence_start = time.time()
                    elif time.time() - silence_start > self.config.silence_duration:
                        # End of utterance
                        break
                else:
                    silence_start = None

            except queue.Empty:
                continue

        if len(chunks) == 0:
            return np.array([])

        return np.concatenate(chunks).flatten()

    def transcribe(self, audio: np.ndarray) -> str:
        """Transcribe audio to text.

        Args:
            audio: Audio data (float32, 16kHz)

        Returns:
            Transcribed text
        """
        if len(audio) == 0:
            return ""

        # Whisper expects float32 audio
        audio = audio.astype(np.float32)

        # Transcribe
        result = self.model.transcribe(
            audio,
            language=self.config.language,
            fp16=False,  # Use FP32 for CPU
            task="transcribe"
        )

        return result["text"].strip()

    def listen_once(self) -> str:
        """Listen for one utterance and return transcription."""
        self.start_stream()
        try:
            audio = self.record_utterance()
            text = self.transcribe(audio)
            return text
        finally:
            self.stop_stream()

    def listen_continuous(
        self,
        callback: Callable[[str], None],
        stop_event: threading.Event = None
    ):
        """Continuously listen and transcribe.

        Args:
            callback: Function to call with each transcription
            stop_event: Threading event to stop listening
        """
        if stop_event is None:
            stop_event = threading.Event()

        self.start_stream()
        try:
            while not stop_event.is_set():
                audio = self.record_utterance(timeout=5.0)
                if len(audio) > self.config.sample_rate * 0.5:  # At least 0.5s
                    text = self.transcribe(audio)
                    if text:
                        callback(text)
        finally:
            self.stop_stream()


class WakeWordDetector:
    """Detect wake word before processing commands."""

    def __init__(
        self,
        wake_words: list = None,
        speech_recognizer: SpeechRecognizer = None
    ):
        self.wake_words = wake_words or ["hey robot", "ok robot", "robot"]
        self.recognizer = speech_recognizer or SpeechRecognizer(
            SpeechConfig(model_size="tiny")  # Fast model for wake word
        )

    def detected(self, text: str) -> bool:
        """Check if wake word is in text."""
        text_lower = text.lower()
        return any(wake in text_lower for wake in self.wake_words)

    def wait_for_wake_word(self, timeout: float = None) -> bool:
        """Wait for wake word detection.

        Returns:
            True if wake word detected, False if timeout
        """
        start_time = time.time()

        self.recognizer.start_stream()
        try:
            while True:
                if timeout and time.time() - start_time > timeout:
                    return False

                audio = self.recognizer.record_utterance(timeout=3.0)
                if len(audio) > 0:
                    text = self.recognizer.transcribe(audio)
                    if self.detected(text):
                        return True
        finally:
            self.recognizer.stop_stream()


# Example usage
def speech_recognition_example():
    """Demonstrate speech recognition."""

    recognizer = SpeechRecognizer(SpeechConfig(model_size="base"))

    print("Say something...")
    text = recognizer.listen_once()
    print(f"You said: {text}")


if __name__ == "__main__":
    speech_recognition_example()
```

---

## 3. LLM-Based Task Planning

### 3.1 Task Planner Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        LLM Task Planner Architecture                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   Input Processing                                                          │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  User Command: "Get me the red cup from the kitchen"                 │  │
│   │  Scene Context: {objects: ["red cup", "blue mug", ...], ...}        │  │
│   │  Robot State: {position: [2.0, 1.0], holding: null, ...}            │  │
│   │  History: [previous commands and results]                            │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                    │                                        │
│                                    ▼                                        │
│   Prompt Construction                                                       │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  System Prompt: Robot capabilities, action definitions              │  │
│   │  + Context: Scene description, robot state                          │  │
│   │  + Command: User's natural language instruction                     │  │
│   │  + Format: JSON output specification                                │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                    │                                        │
│                                    ▼                                        │
│   LLM Reasoning                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │                    GPT-4 / Claude / Llama                            │  │
│   │                                                                      │  │
│   │  1. Parse user intent                                                │  │
│   │  2. Check feasibility against scene/robot state                     │  │
│   │  3. Decompose into primitive actions                                │  │
│   │  4. Order actions with dependencies                                  │  │
│   │  5. Generate confirmation message                                    │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                    │                                        │
│                                    ▼                                        │
│   Output                                                                    │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  {                                                                   │  │
│   │    "feasible": true,                                                │  │
│   │    "actions": [                                                     │  │
│   │      {"type": "navigate", "target": "kitchen"},                     │  │
│   │      {"type": "look", "target": "red cup"},                         │  │
│   │      {"type": "pick", "object": "red cup"},                         │  │
│   │      {"type": "navigate", "target": "user"},                        │  │
│   │      {"type": "handover"}                                           │  │
│   │    ],                                                               │  │
│   │    "response": "I'll go to the kitchen and bring you the red cup." │  │
│   │  }                                                                   │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Task Planner Implementation

```python
#!/usr/bin/env python3
"""
task_planner.py - LLM-based task planning for robot control
"""

import json
from typing import List, Dict, Optional, Any
from dataclasses import dataclass, asdict
from enum import Enum
import os


class ActionType(Enum):
    """Available robot action types."""
    NAVIGATE = "navigate"
    PICK = "pick"
    PLACE = "place"
    HANDOVER = "handover"
    LOOK = "look"
    SPEAK = "speak"
    WAVE = "wave"
    WAIT = "wait"


@dataclass
class RobotAction:
    """A single robot action."""
    type: str
    params: Dict[str, Any]
    preconditions: List[str] = None
    expected_effects: List[str] = None


@dataclass
class TaskPlan:
    """A complete task plan."""
    feasible: bool
    actions: List[RobotAction]
    response: str
    reasoning: Optional[str] = None
    estimated_duration: Optional[float] = None


@dataclass
class SceneContext:
    """Current scene information."""
    objects: List[Dict[str, Any]]
    locations: Dict[str, List[float]]
    obstacles: List[Dict[str, Any]]


@dataclass
class RobotState:
    """Current robot state."""
    position: List[float]
    orientation: float
    holding: Optional[str]
    battery_level: float
    available_actions: List[str]


class TaskPlanner:
    """LLM-based task planner for humanoid robot."""

    def __init__(
        self,
        model: str = "gpt-4",
        api_key: Optional[str] = None
    ):
        self.model = model
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")

        # Action definitions for the robot
        self.action_definitions = """
Available actions:
- navigate(target): Move to a location or near an object
  - target: location name (e.g., "kitchen", "living room") or object name
  - Precondition: Path must be clear
  - Effect: Robot at target location

- pick(object): Pick up an object
  - object: Name of object to pick
  - Precondition: Object reachable, hand empty
  - Effect: Robot holding object

- place(location): Place held object at location
  - location: Where to place (e.g., "table", "shelf", "user_hand")
  - Precondition: Robot holding something, location reachable
  - Effect: Object at location, hand empty

- handover(): Hand object to user
  - Precondition: Robot holding something, user nearby
  - Effect: User has object, hand empty

- look(target): Turn head to look at target
  - target: Object, location, or direction
  - Effect: Camera pointed at target

- speak(message): Say something to the user
  - message: What to say
  - Effect: User hears message

- wave(): Wave hand in greeting
  - Effect: Social acknowledgment

- wait(duration): Wait for specified time
  - duration: Seconds to wait
  - Effect: Time passes
"""

        self.system_prompt = f"""You are a task planner for a humanoid robot. Given a natural language command, scene context, and robot state, output a plan to accomplish the task.

{self.action_definitions}

Rules:
1. Only use actions from the available list
2. Check preconditions before each action
3. Consider the scene context and robot state
4. If a task is impossible, explain why
5. Keep plans simple and safe
6. Always respond naturally to the user

Output format (JSON):
{{
  "feasible": true/false,
  "actions": [
    {{"type": "action_name", "params": {{"param": "value"}}}},
    ...
  ],
  "response": "Natural language response to user",
  "reasoning": "Brief explanation of plan"
}}
"""

    def _build_context_prompt(
        self,
        scene: SceneContext,
        robot_state: RobotState
    ) -> str:
        """Build context portion of prompt."""
        context = f"""
Current Scene:
- Objects visible: {json.dumps(scene.objects)}
- Known locations: {json.dumps(scene.locations)}
- Obstacles: {json.dumps(scene.obstacles)}

Robot State:
- Position: {robot_state.position}
- Currently holding: {robot_state.holding or "nothing"}
- Battery: {robot_state.battery_level}%
"""
        return context

    def plan(
        self,
        command: str,
        scene: SceneContext,
        robot_state: RobotState,
        history: List[Dict] = None
    ) -> TaskPlan:
        """Generate a task plan from natural language command.

        Args:
            command: User's natural language command
            scene: Current scene context
            robot_state: Current robot state
            history: Previous interactions

        Returns:
            TaskPlan with actions and response
        """
        # Build messages
        messages = [
            {"role": "system", "content": self.system_prompt}
        ]

        # Add history if provided
        if history:
            for item in history[-5:]:  # Last 5 interactions
                messages.append({"role": "user", "content": item["command"]})
                messages.append({"role": "assistant", "content": json.dumps(item["plan"])})

        # Add current request
        context = self._build_context_prompt(scene, robot_state)
        user_message = f"{context}\n\nUser command: {command}"
        messages.append({"role": "user", "content": user_message})

        # Call LLM
        try:
            response = self._call_llm(messages)
            plan_dict = json.loads(response)

            # Parse actions
            actions = []
            for action_dict in plan_dict.get("actions", []):
                action = RobotAction(
                    type=action_dict["type"],
                    params=action_dict.get("params", {})
                )
                actions.append(action)

            return TaskPlan(
                feasible=plan_dict.get("feasible", True),
                actions=actions,
                response=plan_dict.get("response", ""),
                reasoning=plan_dict.get("reasoning", "")
            )

        except Exception as e:
            return TaskPlan(
                feasible=False,
                actions=[],
                response=f"I'm sorry, I had trouble understanding that. Could you try again?",
                reasoning=str(e)
            )

    def _call_llm(self, messages: List[Dict]) -> str:
        """Call the LLM API."""
        # OpenAI API
        if "gpt" in self.model.lower():
            from openai import OpenAI
            client = OpenAI(api_key=self.api_key)

            response = client.chat.completions.create(
                model=self.model,
                messages=messages,
                response_format={"type": "json_object"},
                temperature=0.7,
                max_tokens=1000
            )
            return response.choices[0].message.content

        # Anthropic API
        elif "claude" in self.model.lower():
            import anthropic
            client = anthropic.Anthropic()

            # Convert to Anthropic format
            system = messages[0]["content"]
            anthropic_messages = messages[1:]

            response = client.messages.create(
                model=self.model,
                max_tokens=1000,
                system=system,
                messages=anthropic_messages
            )
            return response.content[0].text

        else:
            raise ValueError(f"Unsupported model: {self.model}")

    def validate_plan(
        self,
        plan: TaskPlan,
        scene: SceneContext,
        robot_state: RobotState
    ) -> Tuple[bool, List[str]]:
        """Validate a plan against current state.

        Returns:
            (is_valid, list of issues)
        """
        issues = []
        current_state = {
            "position": robot_state.position,
            "holding": robot_state.holding
        }

        for i, action in enumerate(plan.actions):
            # Check action-specific preconditions
            if action.type == "pick":
                if current_state["holding"] is not None:
                    issues.append(f"Action {i}: Cannot pick while holding {current_state['holding']}")
                obj_name = action.params.get("object")
                if not any(obj["name"] == obj_name for obj in scene.objects):
                    issues.append(f"Action {i}: Object '{obj_name}' not in scene")
                else:
                    current_state["holding"] = obj_name

            elif action.type == "place" or action.type == "handover":
                if current_state["holding"] is None:
                    issues.append(f"Action {i}: Cannot place/handover - not holding anything")
                else:
                    current_state["holding"] = None

            elif action.type == "navigate":
                target = action.params.get("target")
                if target not in scene.locations and not any(
                    obj["name"] == target for obj in scene.objects
                ):
                    issues.append(f"Action {i}: Unknown navigation target '{target}'")

        return len(issues) == 0, issues


# Example usage
def task_planning_example():
    """Demonstrate task planning."""

    planner = TaskPlanner(model="gpt-4")

    # Create scene context
    scene = SceneContext(
        objects=[
            {"name": "red cup", "location": [5.0, 2.0, 0.8], "graspable": True},
            {"name": "blue mug", "location": [5.2, 2.1, 0.8], "graspable": True},
            {"name": "laptop", "location": [3.0, 1.0, 0.75], "graspable": False},
        ],
        locations={
            "kitchen": [5.0, 2.0],
            "living_room": [0.0, 0.0],
            "desk": [3.0, 1.0],
        },
        obstacles=[]
    )

    robot_state = RobotState(
        position=[0.0, 0.0],
        orientation=0.0,
        holding=None,
        battery_level=85.0,
        available_actions=["navigate", "pick", "place", "look", "speak"]
    )

    # Plan a task
    command = "Can you get me the red cup from the kitchen?"
    plan = planner.plan(command, scene, robot_state)

    print(f"Command: {command}")
    print(f"Feasible: {plan.feasible}")
    print(f"Response: {plan.response}")
    print(f"Actions:")
    for action in plan.actions:
        print(f"  - {action.type}: {action.params}")


if __name__ == "__main__":
    task_planning_example()
```

---

## 4. Action Execution with ROS 2

### 4.1 Action Executor

```python
#!/usr/bin/env python3
"""
action_executor.py - Execute planned actions via ROS 2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState

import numpy as np
from typing import Dict, Any, Callable, Optional
from dataclasses import dataclass
import asyncio


@dataclass
class ActionResult:
    """Result of action execution."""
    success: bool
    message: str
    data: Optional[Dict] = None


class ActionExecutor(Node):
    """Execute robot actions via ROS 2 interfaces."""

    def __init__(self):
        super().__init__('action_executor')

        # Callback group for async operations
        self.cb_group = ReentrantCallbackGroup()

        # Navigation action client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group
        )

        # Publishers
        self.speech_pub = self.create_publisher(String, '/robot/speech', 10)
        self.head_pub = self.create_publisher(JointState, '/head/joint_commands', 10)
        self.gripper_pub = self.create_publisher(JointState, '/gripper/joint_commands', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # State
        self.current_joint_states = {}
        self.is_holding = False

        # Action handlers
        self.handlers: Dict[str, Callable] = {
            'navigate': self.execute_navigate,
            'pick': self.execute_pick,
            'place': self.execute_place,
            'handover': self.execute_handover,
            'look': self.execute_look,
            'speak': self.execute_speak,
            'wave': self.execute_wave,
            'wait': self.execute_wait,
        }

        # Known locations (would be loaded from map)
        self.locations = {
            'kitchen': (5.0, 2.0, 0.0),
            'living_room': (0.0, 0.0, 0.0),
            'desk': (3.0, 1.0, 1.57),
        }

        self.get_logger().info('Action executor initialized')

    def joint_state_callback(self, msg: JointState):
        """Update current joint states."""
        for i, name in enumerate(msg.name):
            self.current_joint_states[name] = msg.position[i]

    async def execute_action(self, action_type: str, params: Dict) -> ActionResult:
        """Execute a single action.

        Args:
            action_type: Type of action to execute
            params: Action parameters

        Returns:
            ActionResult indicating success/failure
        """
        handler = self.handlers.get(action_type)
        if handler is None:
            return ActionResult(
                success=False,
                message=f"Unknown action type: {action_type}"
            )

        try:
            result = await handler(params)
            return result
        except Exception as e:
            self.get_logger().error(f"Action failed: {e}")
            return ActionResult(success=False, message=str(e))

    async def execute_plan(self, actions: list) -> ActionResult:
        """Execute a sequence of actions.

        Args:
            actions: List of RobotAction objects

        Returns:
            ActionResult for the overall plan
        """
        for i, action in enumerate(actions):
            self.get_logger().info(f"Executing action {i+1}/{len(actions)}: {action.type}")

            result = await self.execute_action(action.type, action.params)

            if not result.success:
                return ActionResult(
                    success=False,
                    message=f"Plan failed at action {i+1}: {result.message}"
                )

        return ActionResult(success=True, message="Plan executed successfully")

    # Action implementations

    async def execute_navigate(self, params: Dict) -> ActionResult:
        """Navigate to a target location."""
        target = params.get('target')

        # Resolve target to coordinates
        if target in self.locations:
            x, y, theta = self.locations[target]
        else:
            # Could be an object - would need perception
            return ActionResult(success=False, message=f"Unknown location: {target}")

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return ActionResult(success=False, message="Navigation server unavailable")

        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = np.sin(theta / 2)
        goal.pose.pose.orientation.w = np.cos(theta / 2)

        # Send goal
        goal_handle = await self.nav_client.send_goal_async(goal)
        if not goal_handle.accepted:
            return ActionResult(success=False, message="Navigation goal rejected")

        # Wait for result
        result = await goal_handle.get_result_async()

        if result.result.result == 0:
            return ActionResult(success=True, message=f"Arrived at {target}")
        else:
            return ActionResult(success=False, message="Navigation failed")

    async def execute_pick(self, params: Dict) -> ActionResult:
        """Pick up an object."""
        object_name = params.get('object')

        if self.is_holding:
            return ActionResult(success=False, message="Already holding an object")

        # In real implementation:
        # 1. Locate object using perception
        # 2. Plan grasp
        # 3. Execute grasp motion
        # 4. Verify grasp success

        self.get_logger().info(f"Picking up: {object_name}")

        # Simulate pick action
        await asyncio.sleep(2.0)  # Grasp execution time

        # Open gripper
        self._send_gripper_command(0.08)  # Open
        await asyncio.sleep(0.5)

        # Close gripper
        self._send_gripper_command(0.02)  # Close
        await asyncio.sleep(0.5)

        self.is_holding = True

        return ActionResult(
            success=True,
            message=f"Picked up {object_name}",
            data={"object": object_name}
        )

    async def execute_place(self, params: Dict) -> ActionResult:
        """Place held object at location."""
        location = params.get('location')

        if not self.is_holding:
            return ActionResult(success=False, message="Not holding anything")

        # In real implementation:
        # 1. Move to place position
        # 2. Lower arm
        # 3. Open gripper
        # 4. Retract arm

        self.get_logger().info(f"Placing object at: {location}")

        await asyncio.sleep(1.5)

        # Open gripper
        self._send_gripper_command(0.08)
        await asyncio.sleep(0.5)

        self.is_holding = False

        return ActionResult(success=True, message=f"Placed object at {location}")

    async def execute_handover(self, params: Dict) -> ActionResult:
        """Hand object to user."""
        if not self.is_holding:
            return ActionResult(success=False, message="Not holding anything")

        # In real implementation:
        # 1. Extend arm toward user
        # 2. Wait for user to grasp
        # 3. Detect grasp (force feedback)
        # 4. Release gripper

        self.get_logger().info("Handing over to user")

        # Extend arm (simplified)
        await asyncio.sleep(1.0)

        # Wait for user (simplified - would use force sensing)
        await asyncio.sleep(2.0)

        # Release
        self._send_gripper_command(0.08)
        await asyncio.sleep(0.5)

        self.is_holding = False

        return ActionResult(success=True, message="Object handed to user")

    async def execute_look(self, params: Dict) -> ActionResult:
        """Turn head to look at target."""
        target = params.get('target')

        # Convert target to head angles
        # In real implementation, would use perception to find target
        pan, tilt = 0.0, 0.0

        if target == 'left':
            pan = 0.5
        elif target == 'right':
            pan = -0.5
        elif target == 'up':
            tilt = -0.3
        elif target == 'down':
            tilt = 0.3
        elif target == 'forward':
            pan, tilt = 0.0, 0.0

        # Send head command
        head_cmd = JointState()
        head_cmd.name = ['head_pan', 'head_tilt']
        head_cmd.position = [pan, tilt]
        self.head_pub.publish(head_cmd)

        await asyncio.sleep(1.0)

        return ActionResult(success=True, message=f"Looking at {target}")

    async def execute_speak(self, params: Dict) -> ActionResult:
        """Speak a message."""
        message = params.get('message', '')

        msg = String()
        msg.data = message
        self.speech_pub.publish(msg)

        # Estimate speaking time (rough)
        duration = len(message.split()) * 0.3
        await asyncio.sleep(duration)

        return ActionResult(success=True, message=f"Said: {message}")

    async def execute_wave(self, params: Dict) -> ActionResult:
        """Wave hand in greeting."""
        self.get_logger().info("Waving")

        # In real implementation:
        # Execute pre-defined wave motion

        await asyncio.sleep(2.0)

        return ActionResult(success=True, message="Waved")

    async def execute_wait(self, params: Dict) -> ActionResult:
        """Wait for specified duration."""
        duration = params.get('duration', 1.0)

        await asyncio.sleep(duration)

        return ActionResult(success=True, message=f"Waited {duration} seconds")

    def _send_gripper_command(self, width: float):
        """Send gripper command."""
        cmd = JointState()
        cmd.name = ['gripper_joint']
        cmd.position = [width]
        self.gripper_pub.publish(cmd)
```

---

## 5. Complete Conversational Robot

### 5.1 Integration

```python
#!/usr/bin/env python3
"""
conversational_robot.py - Complete conversational humanoid robot
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import asyncio
import threading
from typing import Optional, Dict, List
from dataclasses import dataclass

from speech_recognition import SpeechRecognizer, SpeechConfig, WakeWordDetector
from task_planner import TaskPlanner, TaskPlan, SceneContext, RobotState
from action_executor import ActionExecutor, ActionResult


@dataclass
class ConversationState:
    """State of the conversation."""
    history: List[Dict]
    current_task: Optional[TaskPlan]
    user_name: Optional[str]
    context: Dict


class ConversationalRobot(Node):
    """Complete conversational humanoid robot system."""

    def __init__(self):
        super().__init__('conversational_robot')

        # Components
        self.speech = SpeechRecognizer(SpeechConfig(model_size="base"))
        self.wake_detector = WakeWordDetector()
        self.planner = TaskPlanner(model="gpt-4")
        self.executor = ActionExecutor()

        # State
        self.conversation = ConversationState(
            history=[],
            current_task=None,
            user_name=None,
            context={}
        )

        # Scene (would be updated by perception)
        self.scene = SceneContext(
            objects=[],
            locations={
                'kitchen': [5.0, 2.0],
                'living_room': [0.0, 0.0],
            },
            obstacles=[]
        )

        # Robot state (would be updated by sensors)
        self.robot_state = RobotState(
            position=[0.0, 0.0],
            orientation=0.0,
            holding=None,
            battery_level=85.0,
            available_actions=["navigate", "pick", "place", "look", "speak"]
        )

        # Control
        self.running = False
        self.interaction_thread = None

        self.get_logger().info('Conversational robot initialized')

    async def speak(self, message: str):
        """Speak a message using TTS."""
        self.get_logger().info(f"Robot: {message}")
        await self.executor.execute_action('speak', {'message': message})

    async def process_command(self, command: str) -> bool:
        """Process a user command.

        Args:
            command: User's command text

        Returns:
            True if command was executed successfully
        """
        # Check for special commands
        if self._is_stop_command(command):
            await self.speak("Okay, stopping.")
            return True

        if self._is_status_command(command):
            await self._report_status()
            return True

        # Plan the task
        self.get_logger().info(f"Planning: {command}")
        plan = self.planner.plan(
            command,
            self.scene,
            self.robot_state,
            self.conversation.history
        )

        # Check feasibility
        if not plan.feasible:
            await self.speak(plan.response)
            return False

        # Confirm with user
        await self.speak(plan.response)

        # Execute plan
        self.conversation.current_task = plan
        result = await self.executor.execute_plan(plan.actions)

        # Report result
        if result.success:
            await self.speak("Done!")
        else:
            await self.speak(f"I'm sorry, I couldn't complete the task. {result.message}")

        # Update history
        self.conversation.history.append({
            'command': command,
            'plan': {
                'feasible': plan.feasible,
                'actions': [{'type': a.type, 'params': a.params} for a in plan.actions],
                'response': plan.response
            },
            'result': result.success
        })

        self.conversation.current_task = None
        return result.success

    def _is_stop_command(self, command: str) -> bool:
        """Check if command is a stop command."""
        stop_phrases = ['stop', 'cancel', 'abort', 'nevermind', 'forget it']
        return any(phrase in command.lower() for phrase in stop_phrases)

    def _is_status_command(self, command: str) -> bool:
        """Check if command is asking for status."""
        status_phrases = ['status', 'how are you', 'what are you doing', 'battery']
        return any(phrase in command.lower() for phrase in status_phrases)

    async def _report_status(self):
        """Report current robot status."""
        status = f"I'm at position {self.robot_state.position}. "
        if self.robot_state.holding:
            status += f"I'm holding {self.robot_state.holding}. "
        else:
            status += "My hands are empty. "
        status += f"Battery is at {self.robot_state.battery_level}%."

        await self.speak(status)

    async def interaction_loop(self):
        """Main interaction loop."""
        await self.speak("Hello! I'm ready to help. Say 'Hey Robot' to get my attention.")

        while self.running:
            try:
                # Wait for wake word
                self.get_logger().info("Waiting for wake word...")
                if not self.wake_detector.wait_for_wake_word(timeout=None):
                    continue

                # Acknowledge
                await self.speak("Yes?")

                # Listen for command
                self.get_logger().info("Listening for command...")
                command = self.speech.listen_once()

                if command:
                    self.get_logger().info(f"User: {command}")
                    await self.process_command(command)
                else:
                    await self.speak("I didn't catch that. Could you repeat?")

            except Exception as e:
                self.get_logger().error(f"Interaction error: {e}")
                await self.speak("I encountered an error. Please try again.")

    def start(self):
        """Start the conversational robot."""
        self.running = True

        # Run interaction loop in thread
        def run_async():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.interaction_loop())

        self.interaction_thread = threading.Thread(target=run_async)
        self.interaction_thread.start()

    def stop(self):
        """Stop the conversational robot."""
        self.running = False
        if self.interaction_thread:
            self.interaction_thread.join(timeout=5.0)


def main(args=None):
    rclpy.init(args=args)

    robot = ConversationalRobot()

    # Use multi-threaded executor for concurrent callbacks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(robot)
    executor.add_node(robot.executor)

    try:
        robot.start()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        robot.destroy_node()
        robot.executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 6. Safety and Error Handling

### 6.1 Safety Monitor

```python
#!/usr/bin/env python3
"""
safety_monitor.py - Safety monitoring for conversational robot
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

from typing import Set, Callable
from dataclasses import dataclass
from enum import Enum


class SafetyLevel(Enum):
    """Safety alert levels."""
    NORMAL = 0
    CAUTION = 1
    WARNING = 2
    EMERGENCY = 3


@dataclass
class SafetyConstraint:
    """A safety constraint."""
    name: str
    check: Callable[[], bool]
    level: SafetyLevel
    message: str


class SafetyMonitor(Node):
    """Monitor safety during robot operation."""

    def __init__(self):
        super().__init__('safety_monitor')

        # Emergency stop publisher
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.status_pub = self.create_publisher(String, '/safety_status', 10)

        # Safety state
        self.constraints: Set[SafetyConstraint] = set()
        self.current_level = SafetyLevel.NORMAL
        self.violations = []

        # Add default constraints
        self._add_default_constraints()

        # Monitoring timer
        self.timer = self.create_timer(0.1, self.check_safety)

        self.get_logger().info('Safety monitor initialized')

    def _add_default_constraints(self):
        """Add default safety constraints."""
        # These would connect to actual sensors
        self.add_constraint(SafetyConstraint(
            name="emergency_button",
            check=lambda: True,  # Placeholder
            level=SafetyLevel.EMERGENCY,
            message="Emergency stop button pressed"
        ))

        self.add_constraint(SafetyConstraint(
            name="obstacle_proximity",
            check=lambda: True,  # Would check LIDAR
            level=SafetyLevel.CAUTION,
            message="Obstacle detected nearby"
        ))

    def add_constraint(self, constraint: SafetyConstraint):
        """Add a safety constraint."""
        self.constraints.add(constraint)

    def check_safety(self):
        """Periodic safety check."""
        self.violations = []
        max_level = SafetyLevel.NORMAL

        for constraint in self.constraints:
            if not constraint.check():
                self.violations.append(constraint)
                if constraint.level.value > max_level.value:
                    max_level = constraint.level

        self.current_level = max_level

        # Publish status
        status = String()
        status.data = f"Level: {max_level.name}, Violations: {len(self.violations)}"
        self.status_pub.publish(status)

        # Handle based on level
        if max_level == SafetyLevel.EMERGENCY:
            self._trigger_emergency_stop()
        elif max_level == SafetyLevel.WARNING:
            self._slow_down()

    def _trigger_emergency_stop(self):
        """Trigger emergency stop."""
        self.get_logger().error("EMERGENCY STOP TRIGGERED")
        msg = Bool()
        msg.data = True
        self.estop_pub.publish(msg)

    def _slow_down(self):
        """Reduce robot speed."""
        self.get_logger().warn("Safety warning - reducing speed")
```

---

## 7. Summary

### VLA Pipeline Components

| Component | Technology | Purpose |
|-----------|------------|---------|
| **Speech Recognition** | Whisper | Convert speech to text |
| **Task Planning** | GPT-4/Claude | Convert text to action plan |
| **Action Execution** | ROS 2 Actions | Execute robot actions |
| **Scene Understanding** | YOLO + CLIP | Understand environment |
| **TTS** | Coqui/ElevenLabs | Robot speech output |

### Key Design Patterns

| Pattern | Description |
|---------|-------------|
| **Wake Word** | Listen continuously, activate on trigger |
| **Structured Output** | JSON action plans from LLM |
| **Async Execution** | Non-blocking action execution |
| **Safety Monitoring** | Continuous constraint checking |

---

## Exercises

### Exercise 13.1: Speech Recognition (⭐⭐)

1. Set up Whisper speech recognition
2. Implement continuous listening with wake word
3. Test recognition accuracy with different speakers
4. Handle noisy environments

### Exercise 13.2: Task Planner (⭐⭐⭐)

1. Create a task planner with custom action set
2. Implement plan validation
3. Handle ambiguous commands
4. Test with various natural language commands

### Exercise 13.3: Action Executor (⭐⭐⭐)

1. Implement action execution with ROS 2
2. Add error handling and recovery
3. Implement action cancellation
4. Test with simulated actions

### Exercise 13.4: Complete System (⭐⭐⭐⭐)

1. Integrate all components into conversational robot
2. Add scene understanding using perception
3. Implement multi-turn conversation handling
4. Add safety monitoring
5. Deploy and test in simulation

---

## Quiz

<details>
<summary>Q1: What are the main components of a VLA (Vision-Language-Action) system?</summary>

A VLA system typically includes:
1. **Vision**: Camera-based scene understanding (object detection, segmentation)
2. **Language**: Natural language processing (speech recognition, LLM reasoning)
3. **Action**: Motion planning and execution (navigation, manipulation)

Plus supporting components:
- Scene representation (object locations, affordances)
- Task planning (LLM-based decomposition)
- Safety monitoring (constraint checking)

</details>

<details>
<summary>Q2: Why use an LLM for task planning instead of rule-based planning?</summary>

LLM advantages:
1. **Natural language understanding**: Handles varied phrasings naturally
2. **Common sense reasoning**: Knows typical task structures
3. **Flexible**: No need to pre-program all possible commands
4. **Context-aware**: Can use conversation history
5. **Graceful degradation**: Provides helpful responses for impossible tasks

Challenges to address:
- Latency (API calls)
- Consistency (may vary between calls)
- Safety (must validate outputs)
- Cost (API usage fees)

</details>

<details>
<summary>Q3: How do you handle safety in a conversational robot?</summary>

Safety approaches:
1. **Constraint checking**: Continuous monitoring of safety constraints
2. **Action validation**: Verify plans before execution
3. **Emergency stop**: Hardware and software E-stop
4. **Speed limiting**: Reduce speed near humans/obstacles
5. **Workspace monitoring**: Track human positions
6. **Command filtering**: Block dangerous commands
7. **Timeout handling**: Abort long-running operations

</details>

<details>
<summary>Q4: What is the advantage of structured JSON output from LLMs?</summary>

Structured output advantages:
1. **Parseable**: Directly convertible to action objects
2. **Validatable**: Can check required fields
3. **Consistent**: Predictable format
4. **Type-safe**: Known parameter types
5. **Debuggable**: Easy to log and inspect

Modern LLM APIs support JSON mode:
- OpenAI: `response_format={"type": "json_object"}`
- Anthropic: JSON in system prompt

</details>

---

## Next Steps

In [Week 14: Capstone Project](/assessments/week-14-capstone), we bring everything together:
- Complete humanoid robot system integration
- Real-world deployment considerations
- Project presentations and evaluation
