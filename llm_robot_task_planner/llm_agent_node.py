"""LLM Agent node: natural language commands -> ROS 2 tool calls.

Receives user commands on /user_command, uses an LLM to decompose them
into tool calls (navigate, detect, pick, place, status), and executes
each step via ROS 2 topics/actions.

Supports three LLM providers (auto-detected in this order):
  1. Ollama (local) — no API key needed, runs on GPU
  2. Groq (free tier) — set GROQ_API_KEY
  3. OpenAI — set OPENAI_API_KEY
"""

import json
import math
import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from groq import Groq
from openai import OpenAI


# Tool definitions for OpenAI function calling
TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "navigate_to",
            "description": (
                "Navigate the robot to a named location. "
                "Available locations: room_a, room_b, doorway."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "location": {
                        "type": "string",
                        "description": "Target location name (room_a, room_b, doorway)",
                    },
                },
                "required": ["location"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "detect_object",
            "description": (
                "Look for a specific colored cube using the camera. "
                "Returns detection results including 3D position if found. "
                "The robot must be in the same room as the object."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "color": {
                        "type": "string",
                        "description": "Color of the cube to detect (red, green, blue, yellow)",
                    },
                },
                "required": ["color"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "pick_up",
            "description": (
                "Pick up a cube that has been detected. The robot arm "
                "will move to the grasp position and pick up the object. "
                "The object must have been detected first."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "object_name": {
                        "type": "string",
                        "description": "Name of the object to pick up (e.g., red_cube)",
                    },
                },
                "required": ["object_name"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "place_object",
            "description": (
                "Place the currently held object down at the current location."
            ),
            "parameters": {
                "type": "object",
                "properties": {},
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_robot_status",
            "description": (
                "Get the robot's current status including which room it's in, "
                "whether it's holding an object, and known object locations."
            ),
            "parameters": {
                "type": "object",
                "properties": {},
                "required": [],
            },
        },
    },
]

SYSTEM_PROMPT = """You are a robot task planner controlling a mobile robot with a 5-DOF arm in a two-room environment.

Environment:
- Two rooms: room_a (west) and room_b (east), connected by a doorway
- 4 colored cubes: red and blue in room_a, green and yellow in room_b
- The robot can navigate between rooms, detect cubes by color, and pick/place them

Rules:
1. Always navigate to a room before trying to detect objects there.
2. You must detect an object before picking it up.
3. You can only hold one object at a time.
4. If an object is not found in the expected room, try the other room.
5. Report what you are doing and why at each step.
6. If a step fails, explain why and try an alternative approach.

Think step by step. For each action, explain your reasoning briefly before calling the tool."""


class LLMAgentNode(Node):
    def __init__(self):
        super().__init__('llm_agent')

        self.cb_group = ReentrantCallbackGroup()

        # LLM client — prefer Ollama (local), then Groq (free), then OpenAI
        self.declare_parameter('llm_provider', 'auto')
        provider = self.get_parameter('llm_provider').value

        if provider == 'auto':
            # Check if Ollama is running locally
            import urllib.request
            try:
                urllib.request.urlopen('http://localhost:11434/api/tags', timeout=2)
                provider = 'ollama'
            except Exception:
                if os.environ.get('GROQ_API_KEY'):
                    provider = 'groq'
                elif os.environ.get('OPENAI_API_KEY'):
                    provider = 'openai'
                else:
                    self.get_logger().error(
                        'No LLM available. Start Ollama or set GROQ/OPENAI API key.')
                    raise RuntimeError('No LLM provider available')

        if provider == 'ollama':
            self.client = OpenAI(
                base_url='http://localhost:11434/v1',
                api_key='ollama',  # Ollama doesn't need a real key
            )
            self.model = 'qwen2.5:7b'
        elif provider == 'groq':
            self.client = Groq()
            self.model = 'llama-3.3-70b-versatile'
        else:
            self.client = OpenAI()
            self.model = 'gpt-4o-mini'

        self.declare_parameter('llm_model', self.model)
        self.model = self.get_parameter('llm_model').value

        # Publishers
        self.response_pub = self.create_publisher(
            String, '/agent_response', 10)
        self.arm_cmd_pub = self.create_publisher(
            String, '/arm/command', 10)

        # Subscribers
        self.create_subscription(
            String, '/user_command', self._user_command_cb, 10,
            callback_group=self.cb_group)

        # World model status (updated via subscription)
        self.world_status = {}
        self.create_subscription(
            String, '/world_model/status', self._world_status_cb, 10)

        # World model query/response
        self.world_query_pub = self.create_publisher(
            String, '/world_model/query', 10)
        self.world_response = None
        self.world_response_event = threading.Event()
        self.create_subscription(
            String, '/world_model/response', self._world_response_cb, 10)

        # Arm status tracking
        self.arm_status = None
        self.arm_status_event = threading.Event()
        self.create_subscription(
            String, '/arm/status', self._arm_status_cb, 10)

        # Perception (latest detections)
        self.latest_detections = []
        self.create_subscription(
            String, '/detections', self._detections_cb, 10)

        # Nav2 action client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group)

        # Location coordinates (from world_map.yaml)
        self.locations = {
            'room_a': {'x': 2.0, 'y': 2.0, 'yaw': 0.0},
            'room_b': {'x': 6.0, 'y': 2.0, 'yaw': 3.14},
            'doorway': {'x': 4.0, 'y': 2.0, 'yaw': 0.0},
        }

        self.busy = False
        self.get_logger().info(f'LLM Agent started (model: {self.model})')

    def _world_status_cb(self, msg: String):
        try:
            self.world_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _world_response_cb(self, msg: String):
        try:
            self.world_response = json.loads(msg.data)
        except json.JSONDecodeError:
            self.world_response = {'error': 'Invalid JSON'}
        self.world_response_event.set()

    def _arm_status_cb(self, msg: String):
        try:
            self.arm_status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        state = self.arm_status.get('state', '')
        if state in ('done', 'error'):
            self.arm_status_event.set()

    def _detections_cb(self, msg: String):
        try:
            dets = json.loads(msg.data)
            if isinstance(dets, list):
                self.latest_detections = dets
        except json.JSONDecodeError:
            pass

    def _user_command_cb(self, msg: String):
        if self.busy:
            self._publish_response(
                "I'm currently busy with another task. Please wait.")
            return

        command = msg.data.strip()
        if not command:
            return

        self.get_logger().info(f'User command: {command}')
        self._publish_response(f'Received: "{command}". Planning...')

        # Run in thread to avoid blocking executor
        thread = threading.Thread(
            target=self._process_command, args=(command,), daemon=True)
        thread.start()

    def _process_command(self, command: str):
        """Process a user command through the LLM agent loop."""
        self.busy = True
        try:
            self._run_agent_loop(command)
        except Exception as e:
            self.get_logger().error(f'Agent error: {e}')
            self._publish_response(f'Error: {e}')
        finally:
            self.busy = False

    def _run_agent_loop(self, command: str):
        """Run the LLM agent loop with tool calling."""
        # Build initial status context
        status_str = json.dumps(self.world_status, indent=2) if self.world_status else "No status available yet"

        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": (
                f"Current robot status:\n{status_str}\n\n"
                f"User command: {command}"
            )},
        ]

        max_iterations = 15
        for iteration in range(max_iterations):
            self.get_logger().info(f'Agent iteration {iteration + 1}')

            try:
                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    tools=TOOLS,
                    tool_choice="auto",
                )
            except Exception as e:
                self._publish_response(f'LLM API error: {e}')
                return

            choice = response.choices[0]
            message = choice.message

            # Add assistant message to history
            messages.append(message)

            # If the LLM wants to call tools
            if message.tool_calls:
                for tool_call in message.tool_calls:
                    fn_name = tool_call.function.name
                    try:
                        fn_args = json.loads(tool_call.function.arguments)
                    except json.JSONDecodeError:
                        fn_args = {}

                    # Log the reasoning if any
                    if message.content:
                        self._publish_response(f'Agent: {message.content}')

                    self.get_logger().info(
                        f'Tool call: {fn_name}({fn_args})')
                    self._publish_response(
                        f'Executing: {fn_name}({json.dumps(fn_args)})')

                    # Execute the tool
                    result = self._execute_tool(fn_name, fn_args)

                    self.get_logger().info(
                        f'Tool result: {json.dumps(result)[:200]}')

                    # Add tool result to messages
                    messages.append({
                        "role": "tool",
                        "tool_call_id": tool_call.id,
                        "content": json.dumps(result),
                    })

            # If the LLM is done (no tool calls, just a text response)
            elif message.content:
                self._publish_response(f'Agent: {message.content}')
                return

            # Stop reason is 'stop' — the agent is done
            if choice.finish_reason == 'stop':
                if message.content:
                    self._publish_response(f'Agent: {message.content}')
                return

        self._publish_response('Agent reached maximum iterations. Stopping.')

    def _execute_tool(self, name: str, args: dict) -> dict:
        """Execute a tool call and return the result."""
        if name == 'navigate_to':
            return self._tool_navigate(args.get('location', ''))
        elif name == 'detect_object':
            return self._tool_detect(args.get('color', ''))
        elif name == 'pick_up':
            return self._tool_pick(args.get('object_name', ''))
        elif name == 'place_object':
            return self._tool_place()
        elif name == 'get_robot_status':
            return self._tool_status()
        else:
            return {'error': f'Unknown tool: {name}'}

    def _tool_navigate(self, location: str) -> dict:
        """Navigate to a named location using Nav2."""
        if location not in self.locations:
            return {
                'success': False,
                'error': f'Unknown location: {location}. '
                         f'Available: {list(self.locations.keys())}',
            }

        coords = self.locations[location]
        self.get_logger().info(
            f'Navigating to {location}: ({coords["x"]}, {coords["y"]})')

        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            return {'success': False, 'error': 'Nav2 action server not available'}

        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = coords['x']
        goal.pose.pose.position.y = coords['y']
        # Convert yaw to quaternion
        yaw = coords.get('yaw', 0.0)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Send goal and wait
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return {'success': False, 'error': 'Navigation goal rejected'}

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)

        result = result_future.result()
        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            return {
                'success': True,
                'message': f'Arrived at {location}',
                'location': location,
            }
        else:
            return {
                'success': False,
                'error': f'Navigation to {location} failed',
            }

    def _tool_detect(self, color: str) -> dict:
        """Detect a colored cube using perception."""
        valid_colors = ['red', 'green', 'blue', 'yellow']
        if color not in valid_colors:
            return {
                'success': False,
                'error': f'Invalid color: {color}. Valid: {valid_colors}',
            }

        # Wait a moment for fresh detections
        time.sleep(2.0)

        # Check latest detections for the requested color
        for det in self.latest_detections:
            if det.get('color') == color:
                result = {
                    'success': True,
                    'object': f'{color}_cube',
                    'color': color,
                }
                if det.get('position_map'):
                    result['position_map'] = det['position_map']
                if det.get('depth_m'):
                    result['depth_m'] = det['depth_m']
                return result

        return {
            'success': False,
            'message': f'{color} cube not detected in current view',
        }

    def _tool_pick(self, object_name: str) -> dict:
        """Pick up an object using the arm controller."""
        # Check if we're already holding something
        held = self.world_status.get('held_object')
        if held:
            return {
                'success': False,
                'error': f'Already holding {held}. Place it first.',
            }

        # Send pick command
        self.arm_status_event.clear()
        cmd = String()
        cmd.data = json.dumps({'action': 'pick', 'object': object_name})
        self.arm_cmd_pub.publish(cmd)

        # Wait for arm to complete
        if self.arm_status_event.wait(timeout=90):
            if self.arm_status and self.arm_status.get('state') == 'done':
                return {
                    'success': True,
                    'message': f'Picked up {object_name}',
                }
        return {
            'success': False,
            'error': f'Pick of {object_name} timed out or failed',
        }

    def _tool_place(self) -> dict:
        """Place the currently held object."""
        held = self.world_status.get('held_object')
        if not held:
            return {
                'success': False,
                'error': 'Not holding any object',
            }

        # Send place command
        self.arm_status_event.clear()
        cmd = String()
        cmd.data = json.dumps({'action': 'place'})
        self.arm_cmd_pub.publish(cmd)

        # Wait for arm to complete
        if self.arm_status_event.wait(timeout=90):
            if self.arm_status and self.arm_status.get('state') == 'done':
                return {
                    'success': True,
                    'message': f'Placed {held}',
                }
        return {
            'success': False,
            'error': 'Place timed out or failed',
        }

    def _tool_status(self) -> dict:
        """Get current robot status from world model."""
        if self.world_status:
            return self.world_status
        return {
            'current_room': 'unknown',
            'held_object': None,
            'known_objects': {},
        }

    def _publish_response(self, text: str):
        """Publish a response to the user."""
        msg = String()
        msg.data = text
        self.response_pub.publish(msg)
        self.get_logger().info(f'Response: {text}')


def main(args=None):
    rclpy.init(args=args)
    node = LLMAgentNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
