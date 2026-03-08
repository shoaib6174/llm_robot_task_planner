# From Tool Calls to Task Completion — Wiring an LLM Robot for End-to-End Autonomy

**TL;DR: Previous post got the LLM decomposing commands into tool calls. This post wires perception, navigation, and manipulation together so the robot actually executes them end-to-end. "Bring me the red cube from room A to room B" -> 7 autonomous steps -> 76 seconds -> done. The hardest part wasn't the wiring — it was making the camera actually see tiny cubes and recovering from an unreliable 7B model.**

---

This is post 8 in a series on building an LLM-powered robot task planner. [Post 7 covered LLM tool calling and the agent loop.](blog_llm_agent_tool_calling.md)

By the end of this post, we'll have:
- A detection system that actually finds 5cm cubes in a 360-degree environment
- A garbled-output recovery parser for when the 7B model goes off-script
- A concurrency fix that prevents the robot from processing two commands at once
- An AMCL workaround that keeps navigation reliable after the robot spins
- A complete end-to-end run of Scenario 1: "Bring me the red cube from room A to room B"

Let's get into it.

---

## Part 1: The Integration Gap

At the end of post 7, the LLM agent could decompose "find the red cube in room A" into a correct sequence of tool calls. Navigation worked (DC2). Detection worked in isolation (DC3-DC4). The arm could pick and place (DC5). The agent loop ran and called tools (DC6).

But none of these had ever been tested together in a single command.

```
What we had:              What we needed:

  navigate ✓                navigate ──┐
  detect   ✓                detect   ──┤── one seamless chain
  pick     ✓                pick     ──┤   triggered by a single
  place    ✓                place    ──┤   natural language command
  agent    ✓                agent    ──┘

  Each tested alone         All wired together
```

Integration is where assumptions die. Every component had been developed with implicit assumptions about the environment — the camera's field of view, the robot's heading after navigation, AMCL's behavior during rotation. None of those assumptions had been tested against reality.

The work broke down into four problems: the camera can't see the cubes, the concurrency model has a race condition, the LLM garbles its output, and AMCL can't handle spinning. Let's take them one at a time.

---

## Part 2: The Camera Can't See

### The Setup

The robot's camera has these specs:
- 90-degree horizontal field of view
- 640x480 resolution
- Tilted 20 degrees downward
- 15Hz publish rate

The cubes are 5cm. That's small. In a 4x4 meter room, a 5cm cube at 1 meter distance occupies roughly 30 pixels in a 640-pixel-wide frame. At 2 meters, it's maybe 15 pixels. Detectable, but only if the camera is pointing at it.

### The Problem

When the robot navigates to room A's center (2, 2), it arrives facing east (yaw = 0). The red cube sits at (2.5, 2.5) — that's 45 degrees off the robot's forward axis:

```
                    N
                    │
            red cube (2.5, 2.5)
                  ╲ │
                   ╲│  45°
    W ─────────── robot (2, 2) ──────── E  (camera facing this way)
                    │                         ◄── 90° HFOV ──►
                    │                   only sees ±45° from center
                    │
                    S
```

The camera's 90-degree HFOV means it sees 45 degrees to either side of its forward direction. The red cube is right at the edge of that cone — and in practice, 45 degrees off-axis with a 5cm object means zero reliable detections.

### The Debug

I captured camera frames and counted HSV pixels in the red range. Facing east at room center: **0 red pixels**. The cube simply wasn't in view.

But after manually rotating the robot 45 degrees toward the cube: **2269 red pixels**. The cube was there all along — the camera just wasn't looking at it.

This is the kind of thing that works perfectly in unit tests (where you place the camera directly in front of the cube) and fails completely in integration (where the robot arrives at an arbitrary heading).

### The Fix: Rotation Scan

The solution is straightforward: if you can't see it, look around. The `_tool_detect` method now does a 360-degree rotation scan before reporting failure:

```python
def _tool_detect(self, color: str) -> dict:
    """Detect a colored cube using perception with rotation scan."""
    valid_colors = ['red', 'green', 'blue', 'yellow']
    if color not in valid_colors:
        return {
            'success': False,
            'error': f'Invalid color: {color}. Valid: {valid_colors}',
        }

    # Check current detections first (quick check)
    time.sleep(1.0)
    det = self._find_detection(color)
    if det:
        return self._detection_result(color, det)

    # Not found — do a 360° rotation scan
    self.get_logger().info(
        f'Detect: {color} not in view, scanning room...')
    twist = Twist()
    twist.angular.z = 0.5  # rad/s rotation

    scan_duration = 14.0  # ~360° at 0.5 rad/s
    elapsed = 0.0
    check_interval = 1.0

    while elapsed < scan_duration:
        self.cmd_vel_pub.publish(twist)
        time.sleep(check_interval)
        elapsed += check_interval

        det = self._find_detection(color)
        if det:
            # Stop rotation
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.5)
            # Re-check to get stable detection
            det = self._find_detection(color)
            if det:
                return self._detection_result(color, det)

    # Stop rotation and let AMCL settle
    self.cmd_vel_pub.publish(Twist())
    time.sleep(2.0)
    return {
        'success': False,
        'message': f'{color} cube not detected after scanning room',
    }
```

The logic:

1. **Quick check first.** Maybe the cube is already in view. Wait 1 second for a fresh frame, check `/detections`. If found, return immediately — no need to scan.

2. **Rotation scan.** Publish `cmd_vel` with 0.5 rad/s angular velocity. At that rate, a full 360-degree rotation takes about 12.6 seconds. We scan for 14 seconds to account for timing slop.

3. **Check every second.** During rotation, read `/detections` every 1 second. The perception node publishes at 15Hz, so 1 second guarantees multiple frames have been processed.

4. **Stop and re-verify.** When a detection is found, stop rotating, wait 0.5 seconds for the robot to settle, then check again. This filters out spurious single-frame detections that disappear once the camera stops moving.

5. **AMCL settle.** If the full scan completes without finding anything, stop and wait 2 seconds before returning. This is for the AMCL problem described in Part 5.

The scan adds up to 14 seconds in the worst case (object directly behind the camera). In the best case (already in view), it adds 1 second. For Scenario 1, the red cube was found after about 3 seconds of scanning.

---

## Part 3: The Concurrency Bug

### The Problem

The agent node uses a `busy` flag to prevent processing two commands simultaneously. The original code looked like this:

```python
def _user_command_cb(self, msg: String):
    if self.busy:
        self._publish_response("I'm currently busy. Please wait.")
        return
    thread = threading.Thread(
        target=self._process_command, args=(msg.data,), daemon=True)
    thread.start()

def _process_command(self, command: str):
    self.busy = True  # <-- set busy here, inside the thread
    try:
        self._run_agent_loop(command)
    finally:
        self.busy = False
```

See the race? `self.busy = True` is set inside `_process_command`, which runs in a new thread. But `_user_command_cb` checks `self.busy` in the callback thread. Between `thread.start()` and the thread actually reaching `self.busy = True`, there's a window where a second command can slip through.

```
Timeline:

Callback thread        Worker thread 1       Worker thread 2
     │
cmd1 arrives
     │
check busy → False
     │
thread.start() ─────► (starting up...)
     │                      │
cmd2 arrives               │
     │                      │
check busy → False         │  ◄── still False! Thread hasn't set it yet
     │                      │
thread.start() ──────────────────────► (starting up...)
                            │                    │
                       busy = True          busy = True
                            │                    │
                       processing...        processing...  ◄── TWO agents running
```

Two agent loops running concurrently on the same node means two LLM calls, two navigation goals, two sets of arm commands — total chaos.

### The Fix

One line, moved to the right place:

```python
def _user_command_cb(self, msg: String):
    if self.busy:
        self._publish_response(
            "I'm currently busy with another task. Please wait.")
        return

    command = msg.data.strip()
    if not command:
        return

    # Set busy BEFORE spawning thread to prevent race condition
    self.busy = True
    self.get_logger().info(f'User command: {command}')
    self._publish_response(f'Received: "{command}". Planning...')

    # Run in thread to avoid blocking executor
    thread = threading.Thread(
        target=self._process_command, args=(command,), daemon=True)
    thread.start()
```

Now `self.busy = True` is set in the callback thread, before `thread.start()`. The callback thread is single-threaded for this subscription (it's in a `ReentrantCallbackGroup` on a `MultiThreadedExecutor`, but any second callback will see `busy = True` immediately). No window, no race.

The `_process_command` method clears the flag in `finally`:

```python
def _process_command(self, command: str):
    try:
        self._run_agent_loop(command)
    except Exception as e:
        self.get_logger().error(f'Agent error: {e}')
        self._publish_response(f'Error: {e}')
    finally:
        self.busy = False
```

One-line change, but it prevents a class of bugs that would be extremely hard to diagnose in production — imagine the robot getting two conflicting navigation goals simultaneously.

---

## Part 4: When the LLM Goes Off-Script

### The Problem

qwen2.5:7b is a 7-billion parameter model. It's remarkably good at structured tool calling for its size. But "remarkably good" isn't "perfect." Sometimes, instead of producing a proper function call in the API response, it outputs the tool call as plain text:

```
portun {"name": "pick_up", "arguments": {"object_name": "red_cube"}}
```

That `portun` isn't a typo — it's what the model actually outputs. The model knows it wants to call `pick_up` with `object_name: "red_cube"`, but something goes wrong in the structured output generation and it spills into the text content instead of the `tool_calls` field.

When this happens, the agent loop sees `message.content` with no `message.tool_calls`, interprets it as the LLM's final text response, and exits the loop. The task dies midway through.

### The Recovery Parser

The fix is a regex-based parser that scans text responses for embedded tool calls:

```python
def _parse_text_tool_call(self, text: str):
    """Try to extract a tool call from garbled text output.

    qwen2.5:7b sometimes outputs tool calls as text like:
      portun {"name": "pick_up", "arguments": {"object_name": "red_cube"}}
    Returns (fn_name, fn_args) or None.
    """
    valid_tools = {
        'navigate_to', 'detect_object', 'pick_up',
        'place_object', 'get_robot_status',
    }
    # Try to find JSON with "name" and "arguments" keys
    match = re.search(
        r'\{[^{}]*"name"\s*:\s*"(\w+)"[^{}]*"arguments"\s*:\s*(\{[^{}]*\})',
        text)
    if match:
        fn_name = match.group(1)
        if fn_name in valid_tools:
            try:
                fn_args = json.loads(match.group(2))
                return (fn_name, fn_args)
            except json.JSONDecodeError:
                pass
    return None
```

The regex looks for JSON containing `"name"` and `"arguments"` keys — the OpenAI tool call schema. It extracts the function name, validates it against the known tool set (so random text can't trigger arbitrary function calls), and parses the arguments.

### Feeding the Result Back

There's a subtlety here. Normal tool call results go back as `role: "tool"` messages with a `tool_call_id` that references the original tool call:

```python
messages.append({
    "role": "tool",
    "tool_call_id": tool_call.id,
    "content": json.dumps(result),
})
```

But garbled tool calls don't have a `tool_call_id` — they were never proper tool calls in the first place. If you try to use `role: "tool"` without a valid ID, the API rejects it.

The solution: feed the result back as a `role: "user"` message:

```python
# Add as user message with result (no tool_call_id)
messages.append({
    "role": "user",
    "content": f"Tool result for {fn_name}: {json.dumps(result)}",
})
continue  # continue the agent loop, don't exit
```

The LLM sees the result as user-provided context and continues planning normally. It's not elegant, but it keeps the agent loop alive instead of crashing out mid-task.

This recovery parser fires maybe once in every 10-15 tool calls with qwen2.5:7b. For a 7-step task like Scenario 1, that's about a 50% chance of encountering at least one garbled call. Without the parser, that's a 50% failure rate. With it, the task continues seamlessly.

---

## Part 5: AMCL Doesn't Like Spinning

### The Problem

After a 360-degree rotation scan (Part 2), navigation sometimes fails with status 6 (ABORTED). The scan finds the cube, the pick succeeds, but then `navigate_to(doorway)` fails.

### The Root Cause

AMCL (Adaptive Monte Carlo Localization) maintains a particle filter that tracks the robot's position and orientation. Each particle is a hypothesis about where the robot might be. When the robot moves, AMCL updates particles based on laser scan matching.

During a 360-degree rotation at 0.5 rad/s, the laser scan changes rapidly. AMCL's particle filter struggles to keep up — particles spread out, confidence drops, and the estimated pose becomes uncertain. When the scan stops and we immediately send a navigation goal, Nav2 sees a high-uncertainty pose and aborts.

```
Before rotation scan:

  Particles:  ****    (tight cluster around true pose)
  Confidence: HIGH
  Nav2:       "I know where I am, I'll navigate."

After 360° rotation:

  Particles:  *  *  *  *  *  *  *  (spread across the room)
  Confidence: LOW
  Nav2:       "I have no idea where I am. ABORT."
```

### The Fix

Wait 2 seconds after stopping the rotation:

```python
# Stop rotation and let AMCL settle
self.cmd_vel_pub.publish(Twist())
time.sleep(2.0)
```

That's it. Two seconds of the robot sitting still with the laser scanner providing consistent data is enough for AMCL to reconverge its particle filter. The particles collapse back to a tight cluster, confidence rises, and Nav2 accepts the next goal.

This fix lives at the end of the rotation scan loop (already shown in the `_tool_detect` code in Part 2). It applies whether the scan found something or not — in either case, the robot has been spinning and AMCL needs a moment.

Is a 2-second delay a hack? Maybe. The principled approach would be to monitor AMCL's covariance estimate and wait until it drops below a threshold. But for a demo where reliability matters more than shaving seconds, a fixed delay that works every time beats a complex solution that might have its own edge cases.

---

## Part 6: The Moment of Truth — Scenario 1

Command: **"Bring me the red cube from room A to room B"**

The robot starts in room A at position (2, 2). Here's what the agent did:

```
┌─────┬────────────────────────────┬──────────────────────────────────────────┬──────────┐
│ Iter│ Action                     │ Result                                   │ Duration │
├─────┼────────────────────────────┼──────────────────────────────────────────┼──────────┤
│  1  │ navigate_to(room_a)        │ Already there                            │   <1s    │
│  2  │ detect_object(red)         │ Found at (2.431, 2.51), depth 0.503m     │   ~3s    │
│  3  │ pick_up(red_cube)          │ Teleport grasp succeeded                 │  ~22s    │
│  4  │ navigate_to(doorway)       │ Drove to (4, 2)                          │  ~14s    │
│  5  │ navigate_to(room_b)        │ Drove to (6, 2)                          │  ~12s    │
│  6  │ place_object()             │ Placed in room B                         │  ~24s    │
│  7  │ Text summary               │ Task complete                            │    —     │
└─────┴────────────────────────────┴──────────────────────────────────────────┴──────────┘

Total: ~76 seconds
```

Let's walk through the LLM's reasoning at each step:

**Iteration 1** — The LLM checks the world model, sees the robot is already in room A, and calls `navigate_to(room_a)` anyway. The navigation completes instantly because the robot is already at the goal. Slight inefficiency, but correct behavior — better to confirm you're in the right room than to skip the step and be wrong.

**Iteration 2** — The LLM calls `detect_object(red)`. The initial quick check doesn't find the cube (camera facing east, cube is northeast). The rotation scan kicks in, rotates about 45 degrees, and detects the red cube at map position (2.431, 2.51) with depth 0.503m. The 3D position is close to the ground truth of (2.5, 2.5) — good enough for a pick.

**Iteration 3** — With the cube detected, the LLM calls `pick_up(red_cube)`. The arm controller executes the teleport grasp sequence (approach, close gripper, lift). 22 seconds for the full arm motion sequence.

**Iteration 4** — This is the interesting one. The LLM doesn't go directly to room B. It navigates to the doorway first. Why? The system prompt says the rooms are "connected by a doorway," and the LLM correctly reasons that the doorway is an intermediate waypoint. Nav2 could probably plan a path directly from room A to room B, but the LLM's conservative approach of going through the doorway is actually more robust — it avoids potential path planning failures through narrow passages.

**Iteration 5** — From the doorway, navigate to room B center. Smooth 12-second drive.

**Iteration 6** — Place the cube. The arm controller runs the place sequence (lower, open gripper, retract). 24 seconds.

**Iteration 7** — The LLM outputs a text summary: task complete, red cube has been moved from room A to room B. The agent loop detects a text response with no tool calls and exits.

Seven iterations. No errors. No retries. No replanning needed. Seventy-six seconds from command to completion.

### What the LLM Got Right

The interesting thing isn't what the LLM did — it's what it figured out on its own:

1. **Doorway waypoint.** Nobody told it to go through the doorway. The system prompt mentions the doorway exists and lists it as an available location. The LLM inferred that traveling from room A to room B requires passing through the doorway and planned accordingly.

2. **Progress reporting.** At each step, the LLM explained what it was doing and why. "I'm in room A, I need to detect the red cube before I can pick it up." This comes from rule 5 in the system prompt ("Report what you are doing and why at each step"), but the LLM applies it naturally.

3. **Task summary.** After placing the cube, the LLM didn't just stop. It generated a summary confirming the task was complete and what was accomplished. This is the kind of behavior that makes the system feel like it's actually reasoning rather than following a script.

---

## Part 7: What I Learned

### Integration is where assumptions die

Every component worked perfectly in isolation. Navigation was tested with manual goal poses. Detection was tested with the camera pointing directly at cubes. The arm was tested with objects placed at known positions. None of those test conditions matched what happens when the components run together.

The camera FOV assumption was the most insidious. During perception development, I positioned the robot facing the cubes — because that's the obvious way to test a camera. It never occurred to me that after navigating to a room center, the robot might be facing the wrong direction entirely. The fix (rotation scan) is simple, but diagnosing it required capturing frames, counting HSV pixels, and systematically ruling out other hypotheses.

### Small models need safety nets

qwen2.5:7b is impressive for its size. It correctly decomposes multi-step tasks, handles failures, and generates coherent reasoning. But at 7 billion parameters, it's working at the edge of its capabilities for structured output. The garbled tool call problem happens maybe 7-10% of the time — infrequent enough that you might miss it in testing, frequent enough to kill half your demo runs.

The regex recovery parser is a safety net, not a fix. The real fix would be a larger model (70B) or a fine-tuned model. But for a demo, the safety net works: it catches the garbled output, extracts the intended tool call, and keeps the agent loop alive. The task completes as if nothing went wrong.

### Systematic debugging saves time

When detection failed at room center, I could have guessed: maybe the HSV thresholds are wrong, maybe the lighting is bad, maybe the cube is too small. Instead, I captured frames, ran them through the HSV pipeline manually, and counted pixels. Zero red pixels in the frame. That immediately rules out threshold issues and lighting — the cube isn't in the frame at all.

Then I rotated the robot manually: 2269 red pixels. The cube is detectable. The problem is purely geometric. That diagnosis took 10 minutes and pointed directly at the fix. Guessing could have taken hours.

### The simplest fix is often correct

The AMCL settling delay is 2 seconds of `time.sleep`. The busy flag fix is moving one line of code. The garbled output recovery is a single regex. None of these are sophisticated. All of them work.

In integration, you're debugging the interactions between components, not the components themselves. The bugs are in the seams — timing windows, geometric assumptions, format expectations. The fixes tend to be small and targeted. The hard part is finding them.

---

*Next up: Scenario 2 — search with replanning. What happens when the robot can't find the cube where it expects?*

---

**Code**: [github.com/shoaib6174/llm_robot_task_planner](https://github.com/shoaib6174/llm_robot_task_planner)
