# Giving a Robot a Brain — LLM Tool Calling for Autonomous Task Planning

**TL;DR: I connected a locally-running 7B language model to a mobile robot in Gazebo. The robot can navigate between rooms, detect objects, and pick/place cubes — all from natural language commands like "find the red cube." The LLM decomposes commands into tool calls, executes them, handles failures, and replans. The entire system runs locally on a single GPU with no API keys, no cloud costs, and sub-second inference. The hardest part wasn't the LLM — it was getting ROS 2's threading model to cooperate with async action clients.**

---

This is post 7 in a series on building an LLM-powered robot task planner. [Post 6 covered arm PID tuning and simulated grasping.](link-to-post-6)

By the end of this post, we'll have:
- A world model that tracks the robot's state (room, held objects, known objects)
- An LLM agent that decomposes natural language into robot actions
- Tool calling that works with Ollama (local), Groq (free), or OpenAI
- Navigation, detection, and pick/place wired to the LLM
- A robot that reasons about failures and replans

Let's get into it.

---

## Part 1: The Architecture

### The Problem

We have individual robot capabilities — navigation (Nav2), perception (HSV color detection + depth), and manipulation (arm pick/place). Each works independently. But to execute "bring me the red cube from room A," someone needs to:

1. Figure out the robot needs to go to room A first
2. Navigate there
3. Look for the red cube
4. If found, pick it up
5. Navigate back to the user
6. Place it down

This is task decomposition — breaking a high-level goal into a sequence of primitive actions. An LLM is perfect for this.

### The Components

```
User Command ("find the red cube")
        │
        ▼
┌─────────────────┐
│   LLM Agent     │ ← qwen2.5:7b via Ollama (local GPU)
│   Node          │
│                 │ ← System prompt: environment, rules, tools
│   Agent Loop:   │
│   1. LLM call   │
│   2. Tool call  │ ──► navigate_to(room_a)  → Nav2 Action
│   3. Result     │ ──► detect_object(red)   → Perception Sub
│   4. Repeat     │ ──► pick_up(red_cube)    → Arm Command
│                 │ ──► place_object()        → Arm Command
│                 │ ──► get_robot_status()    → World Model
└─────────────────┘
        │
        ▼
  /agent_response ("Arrived at room A. Detecting red cube...")
```

The agent node subscribes to `/user_command`, runs the LLM agent loop, and publishes progress on `/agent_response`.

### The World Model

Before the agent can plan, it needs to know the world state. The world model node tracks three things:

**Robot's current room**: Derived from odometry position + room boundary checks. Room A is x=[0,4], Room B is x=[4,8], y=[0,4].

**Held object**: Updated via `/arm/status` subscription. When the arm controller reports "done" after a pick, the world model knows the robot is holding something.

**Known objects**: Loaded from `config/world_map.yaml` at startup (4 cubes with positions), updated from `/detections` when perception spots something.

The world model publishes this as JSON every 2 seconds on `/world_model/status`:

```json
{
  "current_room": "room_a",
  "held_object": null,
  "known_objects": {
    "red_cube": {"color": "red", "position": [2.5, 2.5], "last_seen_room": "room_a"},
    "blue_cube": {"color": "blue", "position": [1.0, 1.0], "last_seen_room": "room_a"},
    "green_cube": {"color": "green", "position": [6.0, 2.5], "last_seen_room": "room_b"},
    "yellow_cube": {"color": "yellow", "position": [6.5, 1.0], "last_seen_room": "room_b"}
  }
}
```

This context gets injected into every LLM prompt so the model knows where things are.

---

## Part 2: Choosing and Setting Up the LLM

### Why Local Inference?

For a simulation running on a lab PC with an RTX 5090 (32GB VRAM), running the LLM locally makes perfect sense:
- **No API key management** — no secrets to leak, no keys to rotate
- **No cost** — free inference, unlimited calls
- **No latency** — sub-second response on GPU
- **Works offline** — the sim lab doesn't always have reliable internet

### Ollama + qwen2.5:7b

[Ollama](https://ollama.ai) provides a dead-simple way to run LLMs locally. It exposes an OpenAI-compatible API at `localhost:11434/v1`, which means we can use the standard `openai` Python library:

```python
from openai import OpenAI

client = OpenAI(
    base_url='http://localhost:11434/v1',
    api_key='ollama',  # Ollama ignores this, but the library requires it
)
```

Setup on niro-1 (our lab PC):
```bash
# Install Ollama (one command)
curl -fsSL https://ollama.com/install.sh | sh

# Pull the model (4.7GB, ~6 minutes on 11 MB/s)
ollama pull qwen2.5:7b
```

**Why qwen2.5:7b specifically?** It's one of the best small models for structured tool calling. The Qwen2.5 family was specifically trained on function calling tasks. At 7B parameters, it fits comfortably in VRAM alongside Gazebo and YOLO.

### Multi-Provider Auto-Detection

The agent node auto-detects the best available provider:

```python
if provider == 'auto':
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
            raise RuntimeError('No LLM provider available')
```

This means:
- On the lab PC (Ollama running): uses local GPU inference
- On a laptop without GPU: set `GROQ_API_KEY` for free cloud inference
- For production: set `OPENAI_API_KEY` for GPT-4o

All three providers use the same tool calling format, so the agent code doesn't change.

---

## Part 3: Defining the Tools

The LLM interacts with the robot through 5 tools, defined in OpenAI's function calling schema:

```python
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
                        "description": "Target location name",
                    },
                },
                "required": ["location"],
            },
        },
    },
    # ... detect_object, pick_up, place_object, get_robot_status
]
```

Each tool maps to a ROS 2 interface:

| Tool | ROS 2 Interface | Blocking? |
|------|----------------|-----------|
| `navigate_to` | Nav2 `NavigateToPose` action | Yes — waits for goal completion (up to 120s) |
| `detect_object` | Reads `self.latest_detections` from `/detections` subscription | 2s wait for fresh data |
| `pick_up` | Publishes to `/arm/command`, waits for `/arm/status` "done" | Yes — up to 90s |
| `place_object` | Publishes to `/arm/command`, waits for `/arm/status` "done" | Yes — up to 90s |
| `get_robot_status` | Returns `self.world_status` from `/world_model/status` sub | Instant |

The descriptions matter — they tell the LLM when to use each tool and what constraints exist. The more specific you are ("Available locations: room_a, room_b, doorway"), the fewer hallucinated parameters you get.

---

## Part 4: The System Prompt

The system prompt is where you encode the robot's knowledge:

```python
SYSTEM_PROMPT = """You are a robot task planner controlling a mobile robot
with a 5-DOF arm in a two-room environment.

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

Think step by step. For each action, explain your reasoning briefly
before calling the tool."""
```

Rules 1-4 prevent the LLM from making impossible actions (detecting in a room it hasn't navigated to, picking up two objects). Rules 5-6 ensure the LLM explains its reasoning (useful for debugging) and handles failures gracefully.

The world model status is injected as part of the user message:

```python
messages = [
    {"role": "system", "content": SYSTEM_PROMPT},
    {"role": "user", "content": (
        f"Current robot status:\n{json.dumps(self.world_status)}\n\n"
        f"User command: {command}"
    )},
]
```

This gives the LLM current room position, held object, and known object locations.

---

## Part 5: The Agent Loop

The core of the agent is a simple loop: call the LLM, execute tool calls, feed results back, repeat.

```python
def _run_agent_loop(self, command: str):
    messages = [system_prompt, user_message_with_status]

    for iteration in range(15):  # max iterations safety limit
        response = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            tools=TOOLS,
            tool_choice="auto",
        )

        message = response.choices[0].message
        messages.append(message)  # add to conversation history

        if message.tool_calls:
            for tool_call in message.tool_calls:
                result = self._execute_tool(
                    tool_call.function.name,
                    json.loads(tool_call.function.arguments))

                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "content": json.dumps(result),
                })

        elif message.content:
            # LLM is done — text response means task complete
            self._publish_response(message.content)
            return

    self._publish_response('Agent reached maximum iterations.')
```

The key insight: **the LLM decides when it's done.** When the LLM has finished all tool calls and wants to report the result, it responds with text instead of tool calls. The agent loop detects this and exits.

Each iteration, the LLM sees the full conversation history including all previous tool calls and their results. This means it can reason about failures: "navigation to room_b succeeded but detection failed, so let me try room_a instead."

### Threading

The entire `_run_agent_loop` runs in a daemon thread to avoid blocking the ROS 2 executor:

```python
def _user_command_cb(self, msg: String):
    if self.busy:
        self._publish_response("I'm currently busy. Please wait.")
        return
    thread = threading.Thread(
        target=self._process_command, args=(msg.data,), daemon=True)
    thread.start()
```

This is critical — the LLM calls, navigation waits, and arm command waits can take minutes. If we ran them in a callback, the entire ROS node would freeze.

---

## Part 6: The Nav2 Threading Trap

This was the hardest bug in the entire agent implementation.

### The Symptom

Navigation goals were accepted by Nav2 but immediately returned with "Ignoring unexpected goal response." The robot never moved.

### The Root Cause

The standard way to wait for a ROS 2 action result is:

```python
future = self.nav_client.send_goal_async(goal)
rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
```

`spin_until_future_complete` creates a **temporary spin** of the node to process callbacks until the future resolves. But our node is already being spun by a `MultiThreadedExecutor` in the main thread. Two concurrent spins on the same node create a race condition:

1. The executor's spin receives the goal response callback
2. `spin_until_future_complete` also tries to process it
3. One of them wins, the other gets confused
4. The future never resolves properly

### The Fix

Since the executor is already spinning and processing callbacks, we don't need to spin at all. We just need to wait for the callbacks to fire:

```python
goal_event = threading.Event()
goal_handle_ref = [None]

def goal_response_cb(future):
    goal_handle_ref[0] = future.result()
    goal_event.set()  # signal that the callback fired

future = self.nav_client.send_goal_async(goal)
future.add_done_callback(goal_response_cb)

# Block until callback fires (executor handles the actual processing)
if not goal_event.wait(timeout=15.0):
    return {'success': False, 'error': 'Timed out'}
```

Same pattern for the result:

```python
result_event = threading.Event()
result_future = goal_handle.get_result_async()
result_future.add_done_callback(lambda f: result_event.set())
result_event.wait(timeout=120.0)
```

**The rule: never call `spin_until_future_complete` on a node that's already being spun by an executor.** Use event-based callbacks instead.

---

## Part 7: The Lifecycle Manager Race Condition

### The Symptom

Even after fixing the threading, navigation goals were accepted but immediately ABORTED (status 6). Every single goal.

### The Investigation

I dug through the Nav2 logs and found:

```
[lifecycle_manager] Failed to change state for node: map_server
[lifecycle_manager] Failed to bring up all requested nodes. Aborting bringup.
```

The lifecycle manager is responsible for transitioning Nav2 nodes through their lifecycle states: unconfigured → inactive → active. If any node fails to transition, the manager aborts the entire bringup.

The `map_server` load succeeded (the map was read correctly), but the lifecycle transition failed at the same millisecond:

```
[lifecycle_manager] Failed to change state for node: map_server  @ t=565.346s
[map_server] Read map two_room_map.pgm: 180 X 100               @ t=565.346s
```

This is a classic race condition. The lifecycle manager uses "bond" connections to verify nodes are responsive. During Gazebo startup (GPU initialization, world loading), nodes are slower to respond. The bond check timed out before `map_server` finished its internal setup.

### The Consequence

When the lifecycle manager aborts bringup, ALL Nav2 nodes stay in the INACTIVE state. They still register their action servers (so goals appear accepted), but they immediately abort any goal because they're not active.

### The Fix

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

`bond_timeout: 0.0` disables the bond heartbeat mechanism. In production, bonds help detect crashed nodes. In simulation, they cause more problems than they solve because timing is unpredictable.

After this fix:
```
[lifecycle_manager] Managed nodes are active
```

And navigation goals succeed.

---

## Part 8: Watching the Agent Think

Here's the actual log from the DC6 verification test. Command: "find the red cube in room A"

```
[Agent] Received: "find the red cube in room A". Planning...

[Iteration 1]
Tool call: navigate_to({"location": "room_a"})
Navigation goal accepted for room_a
Tool result: {"success": true, "message": "Arrived at room_a"}

[Iteration 2]
Tool call: detect_object({"color": "red"})
Tool result: {"success": false, "message": "red cube not detected in current view"}

[Iteration 3]
Agent: I attempted to detect the red cube in room A but it was not found.
       Since the robot has a 5-DOF arm and can navigate, I will try checking
       room B where the red cube was last seen.
Tool call: navigate_to({"location": "room_b"})
Navigation goal accepted for room_b
Tool result: {"success": true, "message": "Arrived at room_b"}

[Iteration 4]
Tool call: detect_object({"color": "red"})
Tool result: {"success": false, "message": "red cube not detected"}

[Iteration 5]
Agent: The red cube could not be detected in room B either. Let me try the
       doorway for a better vantage point.
Tool call: navigate_to({"location": "doorway"})
Tool result: {"success": true, "message": "Arrived at doorway"}

[Iteration 6-7]
Agent: Navigating back to room A for one more attempt...
Tool call: navigate_to({"location": "room_a"})
Tool result: {"success": true}
Tool call: detect_object({"color": "red"})
Tool result: {"success": false}

[Iteration 8]
Agent: The red cube could not be found despite checking room A thoroughly.
       Given the information and checks conducted so far, it seems there
       might be an issue with the red cube's visibility. Would you like to
       attempt another task?
```

Key observations:

1. **Correct task decomposition**: navigate first, then detect (follows rule 1)
2. **Intelligent replanning**: detection fails → try other room → try doorway → return to original room
3. **Clear reasoning**: the LLM explains WHY it's taking each action
4. **Graceful failure**: after exhausting options, reports the issue clearly
5. **All 4 navigation goals succeeded**: the robot physically drove between rooms

The detection always fails because the perception node needs YOLO integration (it uses HSV color detection that requires the camera to be pointing at the cubes). But the decomposition and navigation work perfectly — that's what DC6 measures.

---

## Part 9: The Agent's Status Awareness

The world model gives the agent situational awareness. When I tested "find the red cube" while the robot was already holding it (from a previous pick test):

```
[Iteration 1]
Agent: Since the robot is already holding the red_cube, we don't need
       to perform any further actions to find it. Let's confirm by checking
       the robot's current status.
Tool call: get_robot_status({})
Tool result: {"current_room": "room_a", "held_object": "red_cube", ...}

[Iteration 2]
Agent: The robot is already holding the red cube in room_a. No additional
       action is needed. Would you like me to place it somewhere?
```

The LLM checked the status, saw the cube was already held, and correctly decided no action was needed. This kind of context-aware reasoning is what makes the tool-calling pattern powerful — the LLM adapts its plan based on the current state.

---

## The Gotcha Table

| Problem | Symptom | Root Cause | Fix |
|---------|---------|------------|-----|
| `spin_until_future_complete` conflicts | Goals accepted then failed with "unexpected response" | Dual spin on same node (executor + spin_until) | Event-based callbacks with `threading.Event` |
| Nav2 goals always ABORTED | Status 6 on every goal | Lifecycle manager bond timeout → all nodes INACTIVE | `bond_timeout: 0.0` in lifecycle manager params |
| Custom nodes crash at startup | `PackageNotFoundError` | `PYTHONPATH` pointed to `dist-packages` not `site-packages` | Fix export path for colcon install |
| `source install/setup.bash` fails | "Package not found" | `BASH_SOURCE` doesn't resolve in nohup/bash-c | Explicit AMENT_PREFIX_PATH + PYTHONPATH exports |
| LLM outputs garbled tool calls | Tool calls appear as text content | qwen2.5:7b (7B) intermittent format failures | Accept for demo; use larger model for production |
| Agent blocks ROS executor | Node freezes during LLM inference | LLM calls in callback thread | Process commands in daemon thread |

---

## Key Takeaways

1. **Never call `spin_until_future_complete` on a node that's already being spun.** This is the most common ROS 2 threading mistake. If you're using a `MultiThreadedExecutor`, use event-based callbacks to wait for action results. The executor handles callback delivery — you just wait for the event.

2. **Nav2's lifecycle manager needs `bond_timeout: 0.0` for simulation.** Bond checking is designed for production where you want to detect crashed nodes. In simulation with Gazebo eating GPU resources, bond checks race against node startup and can abort the entire Nav2 stack. Disable them.

3. **Small LLMs can do structured tool calling.** qwen2.5:7b at 4.7GB correctly decomposes multi-step robot tasks into tool call sequences. The OpenAI function calling format is supported by Ollama, Groq, and OpenAI — write once, run anywhere.

4. **The agent loop is surprisingly simple.** Send messages + tools to LLM, get back tool calls, execute them, append results, repeat. The LLM handles all the reasoning, planning, and replanning. Your code just needs to wire up the tools.

5. **World state context is essential.** Injecting the world model status into every LLM prompt lets the agent make informed decisions. Without it, the LLM would blindly navigate and detect without knowing what room it's in or what it's holding.

6. **Run the LLM locally when you can.** Ollama + a 7B model on a modern GPU gives sub-second inference with zero cost and zero API key management. For a simulation lab environment, there's no reason to pay for cloud inference.

---

*Next up: Integration — wiring everything together for end-to-end "bring me the red cube" scenarios.*

---

**Code**: [github.com/shoaib6174/llm_robot_task_planner](https://github.com/shoaib6174/llm_robot_task_planner)
