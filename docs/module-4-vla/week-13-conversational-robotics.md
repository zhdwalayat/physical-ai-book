---
sidebar_position: 3
---

# Week 13: Conversational Robotics

## Learning Objectives

By the end of this week, you will be able to:

- Integrate speech recognition using OpenAI Whisper
- Connect Large Language Models to robot control
- Implement voice-to-action pipelines
- Design multi-modal human-robot interaction

## The Vision-Language-Action (VLA) Pipeline

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Vision-Language-Action Pipeline                   │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐    │
│   │  Voice   │───►│  Speech  │───►│   LLM    │───►│  Action  │    │
│   │  Input   │    │  (Whisper)│    │ (GPT-4)  │    │ Planner  │    │
│   └──────────┘    └──────────┘    └──────────┘    └────┬─────┘    │
│                                                        │           │
│   ┌──────────┐                                         │           │
│   │  Vision  │─────────────────────────────────────────┤           │
│   │ (Camera) │                                         │           │
│   └──────────┘                                         ▼           │
│                                                   ┌──────────┐     │
│                                                   │  ROS 2   │     │
│                                                   │ Actions  │     │
│                                                   └──────────┘     │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

## Speech Recognition with Whisper

### Installation

```bash
pip install openai-whisper
pip install sounddevice numpy
```

### Real-Time Speech Recognition

```python
import whisper
import sounddevice as sd
import numpy as np
import queue

class SpeechRecognizer:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.audio_queue = queue.Queue()
        self.sample_rate = 16000

    def audio_callback(self, indata, frames, time, status):
        """Callback for audio stream."""
        self.audio_queue.put(indata.copy())

    def listen(self, duration=5.0):
        """Record audio for specified duration."""
        audio_data = []

        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            callback=self.audio_callback
        ):
            sd.sleep(int(duration * 1000))

        while not self.audio_queue.empty():
            audio_data.append(self.audio_queue.get())

        audio = np.concatenate(audio_data).flatten()
        return audio

    def transcribe(self, audio):
        """Transcribe audio to text."""
        result = self.model.transcribe(
            audio,
            language="en",
            fp16=False
        )
        return result["text"]

    def listen_and_transcribe(self):
        """Complete pipeline: listen then transcribe."""
        print("Listening...")
        audio = self.listen()
        print("Transcribing...")
        text = self.transcribe(audio)
        print(f"Heard: {text}")
        return text
```

## LLM-Based Task Planning

### Connecting to GPT for Action Planning

```python
from openai import OpenAI

class RobotTaskPlanner:
    def __init__(self):
        self.client = OpenAI()
        self.system_prompt = """
You are a robot task planner. Given a natural language command,
output a sequence of robot actions in JSON format.

Available actions:
- navigate(x, y): Move to coordinates
- pick(object_name): Pick up an object
- place(location): Place held object
- speak(message): Say something
- look(direction): Turn head to look

Example:
Input: "Get me the red cup from the kitchen"
Output: {
    "actions": [
        {"type": "navigate", "params": {"location": "kitchen"}},
        {"type": "look", "params": {"direction": "around"}},
        {"type": "pick", "params": {"object": "red cup"}},
        {"type": "navigate", "params": {"location": "user"}},
        {"type": "place", "params": {"location": "user_hand"}}
    ]
}
"""

    def plan(self, command, context=None):
        """Generate action plan from natural language command."""
        messages = [
            {"role": "system", "content": self.system_prompt},
        ]

        if context:
            messages.append({
                "role": "user",
                "content": f"Current context: {context}"
            })

        messages.append({
            "role": "user",
            "content": f"Command: {command}"
        })

        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=messages,
            response_format={"type": "json_object"}
        )

        plan = json.loads(response.choices[0].message.content)
        return plan["actions"]
```

## Action Execution with ROS 2

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

class RobotActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')

        # Navigation client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Speech publisher
        self.speech_pub = self.create_publisher(
            String,
            '/robot/speech',
            10
        )

        # Action handlers
        self.action_handlers = {
            'navigate': self.execute_navigate,
            'pick': self.execute_pick,
            'place': self.execute_place,
            'speak': self.execute_speak,
            'look': self.execute_look,
        }

    async def execute_plan(self, actions):
        """Execute a sequence of planned actions."""
        for action in actions:
            action_type = action['type']
            params = action['params']

            handler = self.action_handlers.get(action_type)
            if handler:
                success = await handler(params)
                if not success:
                    self.get_logger().error(f"Action failed: {action}")
                    return False
            else:
                self.get_logger().warn(f"Unknown action: {action_type}")

        return True

    async def execute_navigate(self, params):
        """Navigate to a location."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'

        if 'x' in params and 'y' in params:
            goal.pose.pose.position.x = params['x']
            goal.pose.pose.position.y = params['y']
        elif 'location' in params:
            # Look up named location
            coords = self.get_location_coords(params['location'])
            goal.pose.pose.position.x = coords[0]
            goal.pose.pose.position.y = coords[1]

        result = await self.nav_client.send_goal_async(goal)
        return result.result().result

    async def execute_speak(self, params):
        """Speak a message."""
        msg = String()
        msg.data = params['message']
        self.speech_pub.publish(msg)
        return True
```

## Complete Voice-to-Action Pipeline

```python
class ConversationalRobot:
    def __init__(self):
        self.speech = SpeechRecognizer()
        self.planner = RobotTaskPlanner()
        self.executor = RobotActionExecutor()
        self.vision = VisionSystem()

    async def run(self):
        """Main interaction loop."""
        print("Robot ready. Say 'Hey Robot' to start.")

        while True:
            # Listen for wake word
            text = self.speech.listen_and_transcribe()

            if "hey robot" in text.lower():
                await self.executor.execute_speak({
                    "message": "Yes, how can I help you?"
                })

                # Listen for command
                command = self.speech.listen_and_transcribe()

                # Get visual context
                context = self.vision.describe_scene()

                # Plan actions
                actions = self.planner.plan(command, context)

                # Confirm plan
                await self.executor.execute_speak({
                    "message": f"I'll {self.summarize_plan(actions)}"
                })

                # Execute
                success = await self.executor.execute_plan(actions)

                if success:
                    await self.executor.execute_speak({
                        "message": "Done!"
                    })
                else:
                    await self.executor.execute_speak({
                        "message": "I encountered a problem."
                    })

    def summarize_plan(self, actions):
        """Generate human-readable summary of plan."""
        descriptions = []
        for action in actions:
            if action['type'] == 'navigate':
                descriptions.append(f"go to {action['params'].get('location', 'target')}")
            elif action['type'] == 'pick':
                descriptions.append(f"pick up the {action['params']['object']}")
            elif action['type'] == 'place':
                descriptions.append(f"place it at {action['params']['location']}")

        return ", then ".join(descriptions)
```

## Multi-Modal Interaction

```python
class MultiModalInteraction:
    def __init__(self):
        self.speech = SpeechRecognizer()
        self.gesture = GestureRecognizer()
        self.gaze = GazeTracker()

    def get_reference(self, text, vision_data):
        """
        Resolve references like "that", "there", "this one"
        using multi-modal cues.
        """
        # Check for pointing gesture
        if self.gesture.is_pointing():
            point_target = self.gesture.get_point_target()
            return self.vision.object_at(point_target)

        # Check for gaze direction
        if "that" in text or "there" in text:
            gaze_target = self.gaze.get_gaze_target()
            return self.vision.object_at(gaze_target)

        # Fall back to text-based resolution
        return self.vision.find_object_by_description(text)
```

## Exercises

1. Set up Whisper for speech recognition
2. Create a task planner that converts commands to actions
3. Implement action execution with ROS 2
4. Build a complete voice-controlled robot demo

## Next Steps

In [Week 14](/assessments/week-14-capstone), we bring everything together in the Capstone Project.
