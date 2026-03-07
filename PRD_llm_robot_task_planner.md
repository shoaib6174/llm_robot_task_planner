# PRD: LLM-Powered Robot Task Planner — Phase 1 (Simulation)

## Overview

An agentic robotics system where a user gives natural language commands and a robot autonomously plans and executes multi-step tasks in Gazebo — navigating between rooms, detecting colored cubes, and performing pick-and-place with a simulated arm.

**Example interaction**:
```
User: "Find the red cube in room A and bring it to room B"

LLM Agent:
  Step 1: navigate_to("room_a")         → Nav2 goal sent → Success
  Step 2: detect_object("red cube")     → YOLO detection → Found at (x, y, z)
  Step 3: pick_up("red cube")           → Arm moves to pose → Gripper closes → Lifted
  Step 4: navigate_to("room_b")         → Nav2 goal sent → Success
  Step 5: place_object("room_b")        → Arm lowers → Gripper opens → Done

Agent: "Done. I found the red cube in room A and placed it in room B."
```

---

## Goals

1. Working demo of LLM + ROS 2 agentic control in Gazebo
2. Multi-step task decomposition with error recovery
3. Compelling demo video for job applications
4. Clean, modular codebase ready for real hardware transfer later

## Non-Goals (Phase 1)

- Real hardware deployment (Phase 2)
- Voice control (text commands only)
- Training custom LLMs (use existing APIs)
- Research-grade manipulation
- SLAM (use static map in sim)

---

## System Architecture

```
┌─────────────────────────────────────────────────┐
│                   User Interface                 │
│                  (Terminal CLI)                   │
└──────────────────────┬──────────────────────────┘
                       │ natural language command
                       ▼
┌─────────────────────────────────────────────────┐
│              LLM Agent (LangChain)               │
│                                                  │
│  - Task decomposition (command → subtask list)   │
│  - Tool selection (which ROS 2 action to call)   │
│  - Error handling (replan on failure)             │
│  - World state tracking (semantic map + memory)  │
│  - Reasoning trace (chain of thought logging)    │
└──────┬───────┬───────┬───────┬──────────────────┘
       │       │       │       │
       ▼       ▼       ▼       ▼
┌────────┐┌────────┐┌────────┐┌────────────┐
│ Nav2   ││ YOLO   ││  Arm   ││  Status    │
│ Action ││ Percep ││ Control││  Query     │
│ Client ││ Node   ││ Node   ││  Node      │
└───┬────┘└───┬────┘└───┬────┘└─────┬──────┘
    │         │         │           │
    ▼         ▼         ▼           ▼
┌─────────────────────────────────────────────────┐
│              ROS 2 Middleware                     │
│    Topics / Services / Actions / TF2              │
└──────────────────────┬──────────────────────────┘
                       │
          ┌────────────┼────────────┐
          ▼            ▼            ▼
    ┌──────────┐ ┌──────────┐ ┌──────────┐
    │  Motors  │ │  Camera  │ │  LiDAR   │
    │ (chassis │ │  (depth  │ │(simulated│
    │  + arm)  │ │  + RGB)  │ │  in sim) │
    └──────────┘ └──────────┘ └──────────┘
         Gazebo Simulation
```

---

## Gazebo Simulation Environment

### World design

Modeled to roughly match the real lab: **two rooms, ~150 sq ft total**, connected by a doorway.

```
┌──────────────────┬──────────────────┐
│                  │                  │
│     Room A       │     Room B       │
│                  │                  │
│  [blue cube]     │                  │
│           [red   │   [green cube]   │
│            cube] │                  │
│                  │                  │
│        ┌────┐    │                  │
│        │door│    │  [yellow cube]   │
│        └────┘    │                  │
│                  │                  │
└──────────────────┴──────────────────┘
```

**Requirements**:
- 2 rooms (~75 sq ft each) with walls and a doorway
- Flat floor, adequate lighting
- 4-6 colored cubes (5cm-8cm) placed on floor or on low tables
- Cube colors: red, blue, green, yellow (distinct, easy for YOLO)

### Robot model

**Option A (preferred)**: JetRover URDF — check Hiwonder's ROS 2 Docker on the real robot for URDF/Xacro files. If available, use directly in Gazebo.

**Option B (fallback)**: TurtleBot3 Waffle (mecanum variant or standard differential) + simulated 6-DOF arm attached via `ros2_control` + Gazebo plugin. Add depth camera plugin.

**Key**: Use **holonomic controller** for mecanum drive (matches real JetRover chassis). Configure `cmd_vel` for omnidirectional movement.

### Simulated sensors

| Sensor | Gazebo Plugin | Topic | Notes |
|--------|--------------|-------|-------|
| RGB-D camera | `libgazebo_ros_camera` | `/camera/color/image_raw`, `/camera/depth/image_raw` | Mount on robot body or arm end-effector |
| LiDAR | `libgazebo_ros_ray_sensor` | `/scan` | 360-degree, matches A1 LiDAR specs |

---

## ROS 2 Nodes

### Nodes to build (new code)

| Node | Language | Purpose |
|------|----------|---------|
| `llm_agent_node` | Python | Main agent — receives commands, calls LLM, dispatches ROS 2 actions |
| `perception_node` | Python | Runs YOLO on camera feed, returns detections with 3D positions |
| `arm_controller_node` | Python | Receives pick/place commands, controls simulated arm joints |
| `world_model_node` | Python | Maintains semantic map, answers location/status queries |

### Topics

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/user_command` | `std_msgs/String` | CLI | `llm_agent_node` |
| `/agent_response` | `std_msgs/String` | `llm_agent_node` | CLI |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Gazebo | `perception_node` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Gazebo | `perception_node` |
| `/detections` | `vision_msgs/Detection2DArray` | `perception_node` | `llm_agent_node` |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo | Nav2 |

### Actions

| Action | Server | Client | Purpose |
|--------|--------|--------|---------|
| `NavigateToPose` | Nav2 | `llm_agent_node` | Navigate to (x, y, theta) |
| `DetectObject` | `perception_node` | `llm_agent_node` | Run YOLO, return object pose |
| `PickObject` | `arm_controller_node` | `llm_agent_node` | Move arm to pose, grasp |
| `PlaceObject` | `arm_controller_node` | `llm_agent_node` | Lower arm, release |

### Services

| Service | Server | Purpose |
|---------|--------|---------|
| `GetRobotStatus` | `world_model_node` | Return current pose, held object |
| `QueryLocation` | `world_model_node` | Return coordinates for a named location |

---

## LLM Agent Design

### LangChain Tools

```python
tools = [
    Tool(
        name="navigate_to",
        description="Navigate the robot to a named location (e.g., 'room_a', 'room_b'). Returns success/failure and current position.",
        func=navigate_to_location,
    ),
    Tool(
        name="detect_object",
        description="Look for a specific object using the camera (e.g., 'red cube', 'blue cube'). Returns object name, confidence, and 3D position if found, or 'not found'.",
        func=detect_object,
    ),
    Tool(
        name="pick_up",
        description="Pick up a detected object. The robot arm moves to the object and grasps it. Returns success/failure.",
        func=pick_up_object,
    ),
    Tool(
        name="place_object",
        description="Place the currently held object down at the current location. Returns success/failure.",
        func=place_object,
    ),
    Tool(
        name="get_robot_status",
        description="Get the robot's current status: which room it's in, whether it's holding an object, and what it has seen.",
        func=get_robot_status,
    ),
    Tool(
        name="look_around",
        description="Rotate 360 degrees to scan surroundings and detect all visible objects. Returns list of detected objects with positions.",
        func=look_around,
    ),
]
```

### System Prompt

```
You are a robot task planner controlling a mobile robot with a 6-DOF arm in a two-room environment.

Available locations:
- room_a: First room (coordinates from semantic map)
- room_b: Second room (coordinates from semantic map)

You can navigate between rooms, detect colored cubes with your camera, and pick up / place them.

Rules:
1. Always navigate to a location before trying to detect objects there.
2. If an object is not found at the expected location, use look_around() first, then try the other room.
3. You must detect an object before picking it up.
4. Report what you are doing and why at each step.
5. If a step fails, explain why and try an alternative approach.

Current status: {current_status}
Known object locations: {known_objects}
```

### Error Recovery

| Failure | Agent Response |
|---------|---------------|
| Navigation fails | Retry once. If still fails, report obstacle to user. |
| Object not detected | `look_around()` → try other room → report "not found" |
| Arm grasp fails | Retry with adjusted pose → report failure after 2 attempts |
| Unknown location name | Ask user for clarification |
| Ambiguous command | Ask user to be more specific ("which cube?") |

---

## Semantic World Model

### `world_map.yaml`

```yaml
locations:
  room_a:
    coordinates: [1.5, 1.0, 0.0]   # x, y, theta (center of room)
    description: "Room A — first room"
  room_b:
    coordinates: [5.0, 1.0, 0.0]
    description: "Room B — second room"

objects:
  red_cube:
    color: "red"
    size: 0.06           # 6cm cube
    expected_location: "room_a"
    graspable: true
  blue_cube:
    color: "blue"
    size: 0.06
    expected_location: "room_a"
    graspable: true
  green_cube:
    color: "green"
    size: 0.06
    expected_location: "room_b"
    graspable: true
  yellow_cube:
    color: "yellow"
    size: 0.06
    expected_location: "room_b"
    graspable: true
```

Loaded into the LLM system prompt and updated dynamically when objects are detected in new locations.

---

## Perception Pipeline

### YOLO Object Detection

- **Model**: YOLOv8n (fast, lightweight)
- **Input**: RGB image from `/camera/color/image_raw`
- **Detection strategy**: Use COCO-pretrained model + color filtering
  - YOLO detects generic objects → filter by color in HSV space to identify "red cube", "blue cube", etc.
  - Alternative: fine-tune YOLOv8n on colored cube images (small dataset, quick training)
- **Output**: Class label, confidence, bounding box

### 2D Detection → 3D Position

```
YOLO bbox center pixel (u, v)
  → Sample depth at (u, v) from /camera/depth/image_raw
  → Deproject to 3D point (x, y, z) in camera frame using camera intrinsics
  → TF2 transform: camera_frame → base_link
  → Object position in robot frame
```

---

## Arm Control (Simulation)

### Approach: Direct joint control with predefined poses

For Phase 1, use simple predefined poses rather than MoveIt. This is faster to implement and sufficient for top-down grasping of cubes on flat surfaces.

### Poses

| Pose | Description |
|------|-------------|
| `HOME` | Arm tucked against body, gripper closed |
| `PRE_GRASP` | Arm extended to (x, y) of target, positioned above object |
| `GRASP` | Arm lowered to object height, gripper open |
| `CLOSE_GRIPPER` | Gripper closes around object |
| `LIFT` | Arm raised with object, safe for navigation |
| `PRE_PLACE` | Arm extended to place position, above surface |
| `RELEASE` | Gripper opens, object placed |

### Pick-and-place sequence

```
HOME → navigate to object location
  → PRE_GRASP (above object xy, safe height)
  → GRASP (lower to object z)
  → CLOSE_GRIPPER
  → LIFT (raise arm)
  → navigate to destination
  → PRE_PLACE (above place location)
  → RELEASE (open gripper)
  → HOME
```

### Gazebo arm simulation

- Use `ros2_control` with `GazeboSystem` hardware interface
- Joint position controller for each of the 6 joints
- Gripper: prismatic joint with position control (open = 0.04, closed = 0.0)
- Attach Gazebo gripper plugin or use contact sensor for grasp detection

---

## Repository Structure

```
llm-robot-task-planner/
├── README.md
├── config/
│   ├── world_map.yaml              # Semantic map (rooms, objects, coordinates)
│   ├── nav2_params.yaml            # Nav2 configuration (holonomic controller)
│   └── yolo_config.yaml            # YOLO model path, confidence threshold, color ranges
├── launch/
│   ├── sim_bringup.launch.py       # Gazebo + robot + Nav2 + all nodes
│   └── agent_only.launch.py        # LLM agent only (for testing without sim)
├── models/
│   └── yolov8n.pt                  # YOLO weights
├── src/
│   ├── llm_agent_node.py           # LangChain agent + ROS 2 bridge
│   ├── perception_node.py          # YOLO + depth → 3D position
│   ├── arm_controller_node.py      # Joint control + pick/place sequences
│   ├── world_model_node.py         # Semantic map + status service
│   └── tools/
│       ├── navigate_tool.py        # LangChain tool → Nav2 action client
│       ├── detect_tool.py          # LangChain tool → perception action
│       ├── pick_tool.py            # LangChain tool → arm pick action
│       ├── place_tool.py           # LangChain tool → arm place action
│       └── status_tool.py          # LangChain tool → status service
├── worlds/
│   └── two_room_world.sdf          # Gazebo world (2 rooms, cubes)
├── urdf/
│   └── robot.urdf.xacro            # Robot description (JetRover or TurtleBot3+arm)
├── package.xml
├── setup.py
└── requirements.txt
```

---

## Dependencies

### Python
```
langchain>=0.2
langchain-openai>=0.1        # or langchain-groq
openai>=1.0                  # or groq
ultralytics>=8.0             # YOLOv8
numpy
opencv-python
pyyaml
```

### ROS 2
```
ros2 humble
nav2_bringup
gazebo_ros_pkgs
ros2_control
gazebo_ros2_control
robot_state_publisher
joint_state_publisher
tf2_ros
sensor_msgs
geometry_msgs
vision_msgs
```

### Environment Variables
```bash
export OPENAI_API_KEY="sk-..."       # or GROQ_API_KEY for free tier
export LLM_MODEL="gpt-4o-mini"      # cheap, fast, good at tool calling
export YOLO_MODEL_PATH="models/yolov8n.pt"
export YOLO_CONFIDENCE=0.5
export WORLD_MAP_PATH="config/world_map.yaml"
```

---

## Build Plan

### Week 1: Foundation

| Day | Task | Done when |
|-----|------|-----------|
| 1 | Build Gazebo world — 2 rooms, walls, doorway, lighting | World loads, robot can be spawned |
| 1-2 | Place 4 colored cubes in world as Gazebo models | Cubes visible in Gazebo |
| 2 | Set up robot URDF in Gazebo (JetRover or TurtleBot3+arm) | Robot spawns, drives with `cmd_vel` |
| 2-3 | Configure Nav2 — static map, holonomic controller for mecanum | Robot navigates to goal poses |
| 3 | Create `world_map.yaml` with room coords + cube info | YAML loads correctly |
| 3 | Test: manually send Nav2 goals to both rooms | Robot reaches both rooms reliably |

### Week 2: Perception + Arm + Agent

| Day | Task | Done when |
|-----|------|-----------|
| 4 | Build `perception_node` — YOLO on sim camera | Colored cubes detected, bboxes published |
| 4-5 | Add depth → 3D position pipeline + TF2 transform | 3D object positions correct in base_link frame |
| 5-6 | Build `arm_controller_node` — predefined poses, pick/place sequence | Arm picks up cube in Gazebo |
| 6-7 | Build `world_model_node` — semantic map + status service | Queries return correct room/object data |
| 7-8 | Build `llm_agent_node` — LangChain agent with all tools | Agent receives text, calls navigate_to successfully |

### Week 3: Integration + Demo

| Day | Task | Done when |
|-----|------|-----------|
| 9 | Wire all nodes together — end-to-end pipeline | "find red cube in room A" → robot does it |
| 10 | Add error recovery — LLM replans on detection failure | "cube not in room A" → agent checks room B |
| 10-11 | Add `look_around` tool — 360 scan | Robot rotates and reports all visible objects |
| 11 | Add chain-of-thought logging — display reasoning in terminal | Terminal shows step-by-step LLM reasoning |
| 12 | Test all 5 demo scenarios (see below) | All scenarios work or fail gracefully |
| 12-13 | Record demo video, write README | Video + README done |

---

## Demo Scenarios

### Scenario 1: Simple fetch (MVP baseline)
```
"Bring me the red cube from room A"
→ navigate(room_a) → detect(red cube) → pick_up → navigate(room_b) → place
```

### Scenario 2: Search with replanning
```
"Find the green cube and put it in room A"
→ navigate(room_a) → detect(green cube) → NOT FOUND
→ LLM: "Green cube not in room A. Checking room B."
→ navigate(room_b) → detect(green cube) → FOUND → pick_up → navigate(room_a) → place
```

### Scenario 3: Multi-step cleanup
```
"Move all cubes to room B"
→ navigate(room_a) → look_around → detect blue + red cubes
→ pick_up(blue) → navigate(room_b) → place
→ navigate(room_a) → pick_up(red) → navigate(room_b) → place
→ "Done. Moved 2 cubes from room A to room B."
```

### Scenario 4: Status query
```
"What do you see?"
→ look_around → "I see: red cube (1.2m ahead), blue cube (0.8m to the left)"
```

### Scenario 5: Ambiguity handling
```
"Get the cube"
→ LLM: "I see multiple cubes. Which one — red, blue, green, or yellow?"
```

---

## MVP Checklist

- [ ] Gazebo world with 2 rooms and 4 colored cubes
- [ ] Robot with mecanum drive navigates between rooms via Nav2
- [ ] YOLO detects colored cubes from camera feed
- [ ] Depth camera provides 3D position for detected objects
- [ ] Arm picks up and places at least one cube
- [ ] LLM agent decomposes multi-step commands into tool calls
- [ ] LLM replans when object not found in expected room
- [ ] Chain-of-thought reasoning displayed in terminal
- [ ] Demo video recorded (split screen: Gazebo + RViz + terminal)
- [ ] README with architecture diagram, setup instructions, demo GIF

---

## Success Metrics

| Metric | Target |
|--------|--------|
| Navigation success (room to room) | >95% |
| Cube detection (visible in frame) | >85% |
| Pick-and-place success | >70% |
| End-to-end task completion (Scenarios 1-2) | >60% |
| LLM replanning triggers correctly | Works in demo |
| Task time (navigate + detect + pick + deliver) | <90 seconds |

---

## Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| JetRover URDF not available | Use TurtleBot3 Waffle + simulated arm. Swap URDF later for Phase 2. |
| LangChain → ROS 2 bridge is complex | Start simple: LLM returns JSON `{"tool": "navigate_to", "args": {"location": "room_a"}}`, Python parses and calls ROS 2. Add full LangChain agent framework after basic flow works. |
| YOLO doesn't detect colored cubes well | Use color filtering in HSV on top of basic object detection. Cubes are solid colors — color alone may suffice. |
| Arm grasp fails in simulation | Cubes are uniform size on flat floor — top-down grasp is reliable. Use Gazebo attach plugin to "cheat" grasp if physics is unreliable. |
| LLM API latency >2s | Use Groq (free, fast) or gpt-4o-mini (~500ms). Cache system prompt. |
| Nav2 holonomic config is tricky | Start with differential drive (TurtleBot3 default). Switch to holonomic later — navigation still works, just less agile. |

---

## Cost

| Item | Cost |
|------|------|
| OpenAI API (gpt-4o-mini, ~500 calls during dev) | ~$1-3 |
| Groq API (free tier, 30 req/min) | $0 |
| Everything else | Free |

---

## Phase 2 Notes (for reference only — not in scope)

These decisions were made with Phase 2 (real JetRover) in mind:

- **Mecanum holonomic drive** in sim matches real chassis
- **Two-room world** matches real lab layout (~150 sq ft)
- **Colored cubes** match real demo objects
- **TF2 transforms** used throughout so camera-to-arm math transfers directly
- **Node interfaces** (topics, actions, services) are hardware-agnostic — same code runs on real robot by swapping Gazebo drivers for real drivers
- **Real hardware**: Hiwonder JetRover, Jetson Orin NX, 6-DOF arm, LiDAR A1, 3D depth camera, ROS 2 Humble
