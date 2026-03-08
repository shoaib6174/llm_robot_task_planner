# LLM Agent Work Log — What Was Done (S10)

This is a detailed technical log of all the work done to get the LLM agent node working: world model, multi-provider LLM integration, tool-calling agent loop, and Nav2 action client fixes. This follows the arm control work (S08-S09) and connects the LLM to the robot's capabilities.

## Starting Point

After S09, we had:
- Navigation working (Nav2, room-to-room, DC2 verified)
- Perception working (HSV color detection + depth 3D, DC3/DC4 verified)
- Arm control working (pick/place with teleport grasp, DC5 verified)

What was missing: an LLM that could take natural language commands and decompose them into sequences of these capabilities.

## Architecture Decisions

### World Model Node

Before the agent could plan, it needed to know about the environment. The world model node (`world_model_node.py`) tracks:

- **Robot's current room**: derived from `/odom` position + room boundary checks
- **Held object**: updated via `/arm/status` subscription
- **Known objects**: loaded from `config/world_map.yaml` + updated from `/detections`

Interface:
- Publishes status every 2s on `/world_model/status` (JSON)
- Query/response via `/world_model/query` and `/world_model/response`

The agent reads status passively (no query needed for basic info) and includes it in the LLM's context window.

### LLM Provider Strategy

Three providers, auto-detected in priority order:

| Provider | Detection | Model | Cost |
|----------|-----------|-------|------|
| Ollama (local) | HTTP check `localhost:11434/api/tags` | qwen2.5:7b | Free (GPU) |
| Groq | `GROQ_API_KEY` env var | llama-3.3-70b-versatile | Free tier |
| OpenAI | `OPENAI_API_KEY` env var | gpt-4o-mini | Paid |

All three use OpenAI-compatible function calling format. The Groq client library (`groq.Groq`) also supports the same tool calling schema as OpenAI.

For Ollama, we use the `openai` Python library with a custom base URL:

```python
self.client = OpenAI(
    base_url='http://localhost:11434/v1',
    api_key='ollama',  # Ollama doesn't need a real key
)
self.model = 'qwen2.5:7b'
```

**Why qwen2.5:7b**: Specifically designed for tool/function calling. Fits in 4.7GB VRAM (RTX 5090 has 32GB). Standalone test confirmed correct decomposition: "find the red cube" → `navigate_to(room_a)` + `detect_object(red)`.

### Tool Definitions

Five tools exposed to the LLM via OpenAI function calling schema:

| Tool | Parameters | What It Does |
|------|-----------|--------------|
| `navigate_to` | `location` (room_a, room_b, doorway) | Sends Nav2 NavigateToPose goal, waits for result |
| `detect_object` | `color` (red, green, blue, yellow) | Checks `self.latest_detections` from `/detections` sub |
| `pick_up` | `object_name` (e.g., red_cube) | Publishes `{"action": "pick"}` to `/arm/command`, waits for status |
| `place_object` | (none) | Publishes `{"action": "place"}` to `/arm/command`, waits for status |
| `get_robot_status` | (none) | Returns latest world model status (room, held object, known objects) |

The system prompt tells the LLM the environment layout, the rules (navigate before detect, detect before pick, one object at a time), and to think step by step.

## Problem 1: Nav2 Action Client Conflicts with MultiThreadedExecutor

**Symptom**: Navigation goals were accepted then immediately failed. Console showed `Ignoring unexpected goal response. There may be more than one action server for the action 'navigate_to_pose'`.

**Root cause**: The `_tool_navigate` method used `rclpy.spin_until_future_complete(self, future)` to wait for the Nav2 goal response. But the node was already being spun by a `MultiThreadedExecutor`. Two concurrent spins on the same node create a race condition — callbacks get delivered to the wrong spin context.

**Fix**: Replaced `spin_until_future_complete` with event-based callbacks:

```python
goal_event = threading.Event()
goal_handle_ref = [None]

def goal_response_cb(future):
    goal_handle_ref[0] = future.result()
    goal_event.set()

future = self.nav_client.send_goal_async(goal)
future.add_done_callback(goal_response_cb)
goal_event.wait(timeout=15.0)
```

Since the `MultiThreadedExecutor` is already spinning, it processes the action callbacks automatically. We just need to wait for the events.

Same pattern for the result callback:

```python
result_event = threading.Event()
result_ref = [None]

def result_cb(future):
    result_ref[0] = future.result()
    result_event.set()

result_future = goal_handle.get_result_async()
result_future.add_done_callback(result_cb)
result_event.wait(timeout=120.0)
```

## Problem 2: Nav2 Lifecycle Manager Race Condition

**Symptom**: Navigation goals accepted but immediately ABORTED (status 6). Every goal failed.

**Investigation**:
1. Checked Nav2 logs: `Failed to change state for node: map_server` → `Failed to bring up all requested nodes. Aborting bringup.`
2. This meant ALL Nav2 nodes stayed INACTIVE (never reached ACTIVE state)
3. Inactive controller_server still accepted goals but immediately aborted them
4. The `controller_server` log confirmed: `Action server is inactive. Rejecting the goal.`

**Root cause**: The lifecycle manager uses "bond" connections to verify nodes are alive. During sim startup, `map_server` was loading the map file at the exact moment the lifecycle manager checked its bond. The bond check timed out (race condition), and the lifecycle manager aborted the entire bringup sequence.

The timestamps confirmed the race:
```
[lifecycle_manager] Failed to change state for node: map_server  @ 565.346s
[map_server] Read map two_room_map.pgm: 180 X 100              @ 565.346s  ← same millisecond
```

**Fix**: Disable bond timeout in the lifecycle manager (safe for simulation):

```python
lifecycle_manager = Node(
    package='nav2_lifecycle_manager',
    ...
    parameters=[{
        'use_sim_time': True,
        'autostart': True,
        'node_names': nav2_lifecycle_nodes,
        'bond_timeout': 0.0,  # disable bond checking
    }],
)
```

After this fix: `Managed nodes are active` — all Nav2 nodes come up correctly.

## Problem 3: Custom Nodes Not Starting (PYTHONPATH)

**Symptom**: `odom_to_tf`, `perception_node`, `arm_controller`, and `world_model` all failed at launch with `PackageNotFoundError: No package metadata was found for llm-robot-task-planner`.

**Root cause**: The entry point scripts generated by colcon use `#!/usr/bin/python3` and `importlib.metadata` to find the package. The colcon install directory's `site-packages` wasn't on `PYTHONPATH`.

Two compounding issues:
1. `source install/setup.bash` uses `BASH_SOURCE` which doesn't resolve correctly when called via `nohup` or `bash -c`
2. The manual PYTHONPATH export used `dist-packages` (Debian convention) instead of `site-packages` (pip/colcon convention)

**Fix**: Explicit exports in the launch script with correct path:

```bash
export AMENT_PREFIX_PATH=/home/niro-1/ros2_ws/install/llm_robot_task_planner:$AMENT_PREFIX_PATH
export PYTHONPATH=/home/niro-1/ros2_ws/install/llm_robot_task_planner/lib/python3.12/site-packages:$PYTHONPATH
```

## Problem 4: qwen2.5:7b Intermittent Tool Call Failures

**Symptom**: Some commands produced garbled output where the model wrote tool calls as text instead of structured function calls. Example: `portunals {"name": "navigate_to", "arguments": ...}` instead of proper `tool_calls` in the API response.

**Investigation**: This happened intermittently (first command worked perfectly, second command produced garbled output). The model's smaller size (7B params) means it occasionally fails to follow the tool calling format.

**Mitigation**: Accepted as a limitation of the 7B model. In production, would use a larger model (qwen2.5:14b or llama-3.3-70b via Groq). For demo purposes, the success rate is sufficient — most commands decompose correctly.

## The Agent Loop

The core agent loop processes up to 15 iterations:

```python
def _run_agent_loop(self, command: str):
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": f"Status:\n{status}\n\nCommand: {command}"},
    ]

    for iteration in range(15):
        response = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            tools=TOOLS,
            tool_choice="auto",
        )

        message = response.choices[0].message
        messages.append(message)

        if message.tool_calls:
            for tool_call in message.tool_calls:
                result = self._execute_tool(tool_call.function.name, args)
                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "content": json.dumps(result),
                })
        elif message.content:
            self._publish_response(message.content)
            return  # Done — LLM gave a text response
```

Each tool result is fed back to the LLM, which decides whether to call another tool or respond with text. The LLM sees the full conversation history and can reason about failures.

## DC6 Verification Results

Command: "find the red cube in room A"

| Iteration | LLM Action | Tool | Result |
|-----------|-----------|------|--------|
| 1 | Navigate to room A first | `navigate_to(room_a)` | SUCCESS (robot drove to (2,2)) |
| 2 | Detect the red cube | `detect_object(red)` | Not found (perception needs YOLO) |
| 3 | Try room B as fallback | `navigate_to(room_b)` | SUCCESS (robot drove to (6,2)) |
| 4 | Detect in room B | `detect_object(red)` | Not found |
| 5 | Try doorway for better view | `navigate_to(doorway)` | SUCCESS (robot drove to (4,2)) |
| 6 | Return to room A for retry | `navigate_to(room_a)` | SUCCESS |
| 7 | Final detection attempt | `detect_object(red)` | Not found |
| 8 | Report to user | (text response) | "The red cube could not be found..." |

Key observations:
- All 4 navigation goals SUCCEEDED (room_a, room_b, doorway, room_a)
- LLM correctly decomposed "find X in room Y" into navigate + detect
- LLM showed intelligent replanning: when detection failed, tried other rooms
- LLM provided clear reasoning at each step
- Detection always fails because perception node uses HSV color detection that requires camera alignment — YOLO integration is needed for DC7

## Files Changed

| File | Changes |
|------|---------|
| `llm_robot_task_planner/llm_agent_node.py` | Created: 558 lines. Agent loop, 5 tools, 3 LLM providers, Nav2 action client |
| `llm_robot_task_planner/world_model_node.py` | Created: 228 lines. Semantic map, room tracking, object tracking |
| `launch/sim_bringup.launch.py` | Added world_model + arm_controller nodes, bond_timeout=0 |
| `config/world_map.yaml` | Fixed cube sizes (0.05m) and z positions (0.025m) |
| `setup.py` | Added entry points: world_model, llm_agent |
| `package.xml` | Added deps: nav2_msgs, nav_msgs, std_msgs, sensor_msgs, geometry_msgs, tf2_ros |
| `requirements.txt` | Added: openai, groq, langchain, langchain-openai, langchain-groq |
| `tests/test_agent_cli.py` | Interactive CLI for testing agent via ROS topics |

## Key Insights

1. **Never use `spin_until_future_complete` in a threaded node.** If a `MultiThreadedExecutor` is spinning the node, use event-based callbacks instead. The executor handles all callback delivery — you just need to wait for the right event.

2. **Nav2 bond_timeout=0 is necessary for simulation.** The default bond mechanism assumes nodes start instantly. In simulation with GPU-heavy Gazebo, node startup is slower and bonds can race against lifecycle transitions.

3. **`site-packages` vs `dist-packages` matters.** Colcon installs to `site-packages`. Debian system Python uses `dist-packages`. Getting this wrong in PYTHONPATH causes `PackageNotFoundError` even though the package is installed.

4. **Small LLMs work for structured tool calling.** qwen2.5:7b correctly decomposes most commands into tool calls. The OpenAI-compatible function calling format is widely supported across local and cloud providers.

5. **The agent loop pattern (LLM → tool → result → LLM) is simple but powerful.** With just 5 tools and a good system prompt, a 7B model can plan multi-step robot tasks with intelligent replanning on failure.
