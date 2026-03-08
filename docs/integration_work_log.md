# Integration Work Log — What Was Done (S11)

This is a detailed technical log of all the work done to get DC7 (end-to-end Scenario 1) working: wiring navigation, perception, arm control, and the LLM agent into a single pipeline that can execute "Bring me the red cube from room A to room B" from start to finish. This follows the LLM agent work (S10) and is the first full integration test.

## Starting Point

After S10, we had every component working independently:
- Navigation working (Nav2, room-to-room, DC2 verified)
- Perception working (HSV color detection + depth 3D, DC3/DC4 verified)
- Arm control working (pick/place with teleport grasp, DC5 verified)
- LLM agent working (tool calling with qwen2.5:7b, DC6 verified)

What was missing: wiring them together so a single natural language command could drive the full navigate-detect-pick-navigate-place sequence. Each component had been tested in isolation, but running them in sequence exposed four integration problems that did not appear during individual testing.

## Problem 1: Detection Fails at Room Center

**Symptom**: The LLM agent navigates to room A (center at (2,2)), calls `detect_object(red)`, and gets back an empty result. The red cube at (2.5, 2.5) is physically present in the world, but the perception node publishes `[]` on `/detections`.

**Investigation**: The robot's camera has a 90-degree horizontal field of view and is tilted 20 degrees downward. When the robot is at (2, 2) facing east (yaw=0), the red cube at (2.5, 2.5) is 45 degrees off the camera's central axis — outside the visible frustum. The camera can only see objects within +/-45 degrees of the heading, and the cube's angular offset is right at the edge or beyond.

A debug script confirmed the problem: with the robot facing east, 0 red pixels appeared in the camera image. After manually rotating the robot to face the cube, the detection triggered with area=2269.5 red pixels.

**Root cause**: `_tool_detect` in S10 only checked the current `/detections` — a single-frame passive read. If the cube happened to be in the camera's FOV, detection succeeded. If not, it returned "not found" immediately. There was no mechanism to look around.

**Fix**: Added a 360-degree rotation scan to `_tool_detect`. The robot rotates at 0.5 rad/s and checks `/detections` every 1 second. At that rotation speed, a full 360 degrees takes approximately 12.6 seconds (the scan budget is set to 14 seconds for margin). When a detection is found mid-scan, the robot stops, waits 0.5 seconds for the image to stabilize, then re-reads `/detections` to confirm the detection is real (not a motion-blurred artifact).

The implementation from `llm_agent_node.py`:

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

The logic has three phases: (1) quick check of the current frame, (2) if not found, rotate and poll, (3) if found mid-rotation, stop, settle, and confirm. The 0.5-second settling wait after stopping is important — the camera needs a still frame for HSV thresholding to produce a clean detection. Motion blur during rotation can cause false negatives even when the cube is in view.

## Problem 2: Duplicate Command Processing (Race Condition)

**Symptom**: Sending two rapid commands on `/user_command` caused both to be processed simultaneously, leading to two threads issuing Nav2 goals and arm commands in parallel. The robot received conflicting velocity commands and navigation goals.

**Root cause**: The original `_user_command_cb` checked `self.busy` and then spawned a thread that called `_process_command`, where `self.busy = True` was set as the first line. The problem is a classic TOCTOU (time-of-check/time-of-use) race: two messages can both read `self.busy == False` and pass the check before either thread has a chance to set it to `True`.

```
Thread 1: _user_command_cb → checks self.busy (False) → spawns thread → ...
Thread 2: _user_command_cb → checks self.busy (False) → spawns thread → ...
Thread 1's thread:          → self.busy = True (too late — Thread 2 already passed)
Thread 2's thread:          → self.busy = True
```

**Fix**: Move `self.busy = True` into `_user_command_cb` itself, before `thread.start()`. This makes the check-and-set atomic from the callback's perspective (ROS 2 serializes callbacks within the same callback group by default, and even with the ReentrantCallbackGroup, the GIL ensures the flag is visible immediately after assignment).

The corrected code:

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

The `finally` block in `_process_command` resets `self.busy = False` when the command finishes (whether it succeeds or fails):

```python
def _process_command(self, command: str):
    """Process a user command through the LLM agent loop."""
    try:
        self._run_agent_loop(command)
    except Exception as e:
        self.get_logger().error(f'Agent error: {e}')
        self._publish_response(f'Error: {e}')
    finally:
        self.busy = False
```

## Problem 3: Navigation Fails After Rotation Scan

**Symptom**: After the 360-degree rotation scan in `_tool_detect` completed (either finding the cube or exhausting the scan), the next `navigate_to` call failed with status 6 (ABORTED). This was 100% reproducible — every navigation attempt immediately after a rotation scan was rejected.

**Investigation**: Status 6 is `GoalStatus.STATUS_ABORTED`, meaning the controller server accepted the goal but then aborted it during execution. Nav2 logs showed the controller was unable to compute a valid trajectory because the robot's estimated position was wildly uncertain.

**Root cause**: The AMCL particle filter maintains a probability distribution of the robot's position using laser scan matching. During continuous rotation at 0.5 rad/s for up to 14 seconds, the particle filter's estimate becomes dispersed — the laser scans are changing rapidly, and the motion model accumulates uncertainty faster than the sensor model can correct it. By the time the rotation stops, AMCL's position estimate has a large covariance, and Nav2's planner/controller treats the position as unreliable.

AMCL needs a few seconds of the robot being stationary to reconverge — the laser scans stop changing, and the particle filter can collapse back onto a tight estimate.

**Fix**: Added a 2-second settling delay after stopping rotation, specifically in the "not found" path (the "found" path already had a 0.5-second delay, but needed to be extended):

```python
# Stop rotation and let AMCL settle
self.cmd_vel_pub.publish(Twist())
time.sleep(2.0)
```

The 2-second value was determined empirically. 1 second was not consistently enough (AMCL sometimes still had high covariance). 2 seconds worked in every test case. This delay only occurs on the failure path — when the cube is found mid-scan, the 0.5-second settling wait is sufficient because less rotation has occurred (the scan exits early).

## Problem 4: qwen2.5:7b Garbled Tool Calls

**Symptom**: The LLM sometimes produced text output that contained a tool call but was not formatted as a proper OpenAI function call. The API response had `message.tool_calls = None` and `message.content` containing something like:

```
portun {"name": "pick_up", "arguments": {"object_name": "red_cube"}}
```

The agent loop treated this as a text response (the `elif message.content` branch) and exited the loop, leaving the task incomplete. The robot had navigated and detected the cube but never picked it up.

**Investigation**: This is a known limitation of smaller language models. qwen2.5:7b (4.7GB) has been fine-tuned for tool calling, but it occasionally fails to produce the structured output format that the OpenAI API expects. Instead, it writes something that looks like a tool call but is embedded in regular text. The "portun" prefix appears to be a garbled version of "function" — the model is trying to emit a function call but the tokenizer output is slightly wrong.

This happened intermittently — perhaps 1 in 5 iterations for certain tool names (especially `pick_up` and `place_object`). The first few iterations (usually `navigate_to` and `detect_object`) worked correctly as structured tool calls.

**Fix**: Added `_parse_text_tool_call()` — a regex-based recovery parser that scans the text content for JSON structures containing `"name"` and `"arguments"` keys. If it finds one and the name matches a known tool, it extracts the arguments and executes the tool call as if it were structured.

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
    match = re.search(r'\{[^{}]*"name"\s*:\s*"(\w+)"[^{}]*"arguments"\s*:\s*(\{[^{}]*\})', text)
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

The key design decisions in this parser:

1. **Validation against `valid_tools`**: Prevents executing arbitrary function names that might appear in garbled output.
2. **Regex instead of full JSON parse**: The surrounding text is not valid JSON, so `json.loads` on the full string would fail. The regex extracts only the structured parts.
3. **Nested `{[^{}]*}` for arguments**: This handles simple argument objects (one level deep) without trying to match arbitrarily nested JSON, which avoids catastrophic backtracking.

The recovered tool call result is fed back as a `user` message (not a `tool` message) because there is no `tool_call_id` to reference:

```python
# Check for garbled tool calls (qwen2.5:7b sometimes outputs
# tool calls as text instead of structured function calls)
parsed_tool = self._parse_text_tool_call(message.content)
if parsed_tool:
    fn_name, fn_args = parsed_tool
    self.get_logger().info(
        f'Recovered garbled tool call: {fn_name}({fn_args})')
    self._publish_response(
        f'Executing: {fn_name}({json.dumps(fn_args)})')
    result = self._execute_tool(fn_name, fn_args)
    self.get_logger().info(
        f'Tool result: {json.dumps(result)[:200]}')
    # Add as user message with result (no tool_call_id)
    messages.append({
        "role": "user",
        "content": f"Tool result for {fn_name}: {json.dumps(result)}",
    })
    continue
```

Using `role: "user"` with the tool result works because the LLM sees it as context — "here's what happened when we executed that tool" — and can reason about the next step. The `continue` ensures the agent loop does not exit; it sends the result back to the LLM for the next iteration.

## DC7 Verification

Full end-to-end Scenario 1: "Bring me the red cube from room A to room B"

The agent completed the task in 7 iterations with 0 errors:

| Iter | LLM Action | Tool Call | Result | Duration |
|------|-----------|-----------|--------|----------|
| 1 | Navigate to room A | `navigate_to(room_a)` | Already there (robot started in room A) | <1s |
| 2 | Detect the red cube | `detect_object(red)` | Rotation scan found cube at (2.431, 2.51), depth 0.503m | ~3s |
| 3 | Pick up the cube | `pick_up(red_cube)` | Teleport grasp succeeded | ~22s |
| 4 | Navigate to doorway | `navigate_to(doorway)` | Drove to (4, 2) | ~14s |
| 5 | Navigate to room B | `navigate_to(room_b)` | Drove to (6, 2) | ~12s |
| 6 | Place the cube | `place_object()` | Placed in room B | ~24s |
| 7 | Report completion | (text response) | "Task complete!" | — |

**Total time**: ~76 seconds

Notable behaviors:
- The LLM correctly decomposed "bring X from A to B" into the full six-step tool call sequence.
- It navigated through the doorway as an intermediate waypoint (room A -> doorway -> room B) rather than trying a direct room-to-room path. This matches the system prompt guidance and the room layout (the doorway is a narrow 1m opening between the two rooms).
- The detection found the cube after a partial rotation scan (~3s), not the full 14s budget, because the cube was only partially outside the initial FOV.
- The 22s pick and 24s place durations include the arm's pre-grasp -> grasp -> lift sequence and the settling waits.
- The LLM provided reasoning at each step ("I'll navigate to room A first to locate the red cube") before calling each tool.

## Files Changed

| File | Changes |
|------|---------|
| `llm_robot_task_planner/llm_agent_node.py` | Busy flag moved to callback (race fix), rotation scan in `_tool_detect`, 2s AMCL settle, `_parse_text_tool_call()` garbled output recovery |

## Key Insights

1. **Camera FOV matters for detection — 5cm cubes need active scanning, not passive waiting.** A 90-degree HFOV means the robot can only see 25% of the surrounding area at any given heading. Objects placed at arbitrary positions in a room will often be outside the camera's view when the robot arrives at the room center. Active scanning (rotation) is essential for reliable detection.

2. **AMCL needs settling time after rotation.** The particle filter's localization estimate degrades during continuous rotation because the laser scan data changes rapidly. After stopping, AMCL needs 1-2 seconds of stationary laser scans to reconverge. Without this delay, Nav2 receives a goal with high position uncertainty and aborts.

3. **Small LLMs produce garbled output — regex recovery makes the system robust.** Rather than upgrading to a larger model (which requires more VRAM and increases latency), a simple regex parser that extracts tool calls from malformed text output is sufficient. The key is validating the extracted tool name against a whitelist before execution.

4. **Set busy flags BEFORE spawning threads, not inside them.** This is a standard concurrency pattern but easy to get wrong. The invariant is: by the time the callback returns, any state that guards against re-entry must already be set. If the flag is set inside the thread, there is a window between thread creation and flag assignment where another callback can slip through.

5. **The end-to-end pipeline is simple once components work — most time was spent on perception integration.** Of the four problems in this session, three were related to the interaction between perception (camera FOV, rotation scan) and other subsystems (AMCL localization, navigation). The actual pipeline wiring (navigate -> detect -> pick -> navigate -> place) required zero additional code — the LLM's tool-calling loop handled the sequencing entirely. The lesson: integration bugs live at component boundaries, not in the orchestration layer.
