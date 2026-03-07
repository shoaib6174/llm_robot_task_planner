# Project: LLM-Powered Robot Task Planner (Gazebo Simulation)

## Repository
- **Local**: `/Users/mohammadshoaib/Codes/robotics/llm_robot_task_planner`

## Overview
An agentic robotics system: natural language commands -> LLM task decomposition -> ROS 2 + Gazebo execution. Mobile robot with 6-DOF arm navigates two rooms, detects colored cubes (YOLO), performs pick-and-place.

See `PRD_llm_robot_task_planner.md` for full specification.

## Structure
```
llm_robot_task_planner/
├── CLAUDE.md                          # This file
├── PRD_llm_robot_task_planner.md      # Product requirements document
├── docs/
│   ├── workflow_tracker.md            # Session index, blockers, demo checkpoints
│   ├── decisions.md                   # Searchable decision log (aggregated from worklogs)
│   ├── demo_results.md               # Demo scenario pass/fail tracker
│   └── worklogs/                      # Per-session detailed worklogs
│       └── TEMPLATE.md
├── config/
│   ├── world_map.yaml                 # Semantic map (rooms, objects, coordinates)
│   ├── nav2_params.yaml               # Nav2 configuration
│   └── yolo_config.yaml               # YOLO model path, thresholds, color ranges
├── launch/
│   ├── sim_bringup.launch.py          # Gazebo + robot + Nav2 + all nodes
│   └── agent_only.launch.py           # LLM agent only (testing without sim)
├── models/                            # YOLO weights
├── src/
│   ├── llm_agent_node.py              # LangChain agent + ROS 2 bridge
│   ├── perception_node.py             # YOLO + depth -> 3D position
│   ├── arm_controller_node.py         # Joint control + pick/place sequences
│   ├── world_model_node.py            # Semantic map + status service
│   └── tools/                         # LangChain tool wrappers for ROS 2 actions
├── worlds/
│   └── two_room_world.sdf             # Gazebo world (2 rooms, cubes)
├── urdf/
│   └── robot.urdf.xacro               # Robot description
├── tests/                             # Unit and integration tests
├── package.xml
├── setup.py
└── requirements.txt
```

## Session Protocol

Every Claude Code session MUST maintain a worklog. Follow these steps:

1. **Session start**:
   - Read the **previous session's worklog** (linked in the template) — specifically its "Next Steps" section.
   - Check **Current Blockers** in `docs/workflow_tracker.md` for unresolved issues.
   - Create a new worklog file in `docs/worklogs/` using `YYYY-MM-DD_S##.md` (see `docs/worklogs/TEMPLATE.md`).
   - Fill in session number, date, previous session link, context, and objectives.
2. **During session**:
   - Update "Work Done" and "Files Changed" as tasks are completed.
   - For any non-obvious decision, add it to **both** the worklog's "Decisions Made" table **and** `docs/decisions.md`.
3. **Session end**:
   - Complete all remaining sections (Decisions, Issues & Blockers, Next Steps).
   - Add a one-liner entry to the Session Index in `docs/workflow_tracker.md`.
   - If any blocker is unresolved, add/update it in the **Current Blockers** section of `docs/workflow_tracker.md`.
   - If a blocker was resolved this session, remove it from Current Blockers.

To determine the next session number, check the Session Index in `docs/workflow_tracker.md`.

## Git Branching Strategy

Use feature branches. Keep `main` always demo-able.

| Branch | Purpose | Merge to main when |
|--------|---------|-------------------|
| `gazebo-world` | Gazebo world, cubes, lighting | World loads, robot spawns |
| `robot-urdf` | Robot URDF/Xacro, ros2_control | Robot drives with cmd_vel |
| `navigation` | Nav2 config, static map, holonomic | Robot navigates to goal poses |
| `perception` | YOLO node, depth pipeline, TF2 | Cubes detected with 3D positions |
| `arm-control` | Arm controller, pick/place poses | Arm picks up cube in Gazebo |
| `world-model` | Semantic map node, status service | Queries return correct data |
| `llm-agent` | LangChain agent, tools, CLI | Agent decomposes commands into tool calls |
| `integration` | End-to-end wiring, error recovery | Scenario 1 works end-to-end |

Rules:
- Create branch from latest `main`: `git checkout -b <branch> main`
- Merge to `main` only when the component passes its tests and demo checkpoint
- Delete branch after merge
- Never force-push to `main`
- **Do NOT add `Co-Authored-By: Claude` or any AI attribution in commit messages**

## Engineering Standards — MANDATORY

**The goal of this project is to produce a clean, working demo. Every decision must reflect that.**

- **NO shortcuts, NO hack fixes, NO "make it pass" workarounds.** If something doesn't work, diagnose the root cause properly. Understand WHY it fails before changing anything.
- **Debug systematically.** Read error messages carefully, form hypotheses, test them one at a time. Do not blindly tweak parameters, comment out checks, or add try/except blocks just to suppress errors.
- **Never fake a passing test.** If a test or validation criterion fails, the code is wrong — not the test. If the test itself is genuinely flawed, explain why and fix the test properly with justification.
- **Rework when necessary.** If an approach is fundamentally broken, say so and propose a proper redesign. Wasting time patching a bad foundation is worse than starting that component over.
- **Correctness over speed.** A correct implementation that takes longer is always preferred over a fast but fragile one. This is demo infrastructure that later phases (real hardware) depend on — cutting corners now compounds into major problems later.
- **Ask when uncertain.** If something is ambiguous or you're unsure of the right approach, ask rather than guess. Wrong assumptions waste more time than a quick clarification.

This applies to ALL sessions, ALL phases, ALL code. No exceptions.

## Development Environment — MANDATORY

**Always use the project virtual environment.** Never install packages into or run code with the system Python.

- **Venv path**: `./venv/` (add to `.gitignore`)
- **Venv type**: `--system-site-packages` (required for rclpy access)
- **Python**: `./venv/bin/python`
- **Pip**: `./venv/bin/pip`
- **Pytest**: `./venv/bin/python -m pytest tests/ -v`
- **Run scripts**: `./venv/bin/python src/<script>.py`

**Source order matters** — ROS setup FIRST, then venv:
```bash
source /opt/ros/jazzy/setup.zsh    # FIRST — sets PYTHONPATH for rclpy (use setup.bash if in bash)
source ./venv/bin/activate          # SECOND — prepends venv site-packages
```
Reversing this order breaks PYTHONPATH. Both must be sourced in every terminal.

If the venv doesn't exist, create it first:
```bash
python3 -m venv --system-site-packages venv
./venv/bin/pip install -r requirements.txt
```

**Do NOT pip install `pyyaml`** — ROS 2 provides `python3-yaml` via apt. Pip-installing it causes version conflicts.

## ROS 2 / Gazebo Environment

- **ROS 2 distro**: Jazzy
- **Gazebo version**: Gazebo Harmonic (Gz Sim 8)
- **Source ROS 2 before running**: `source /opt/ros/jazzy/setup.zsh` (added to `~/.zshrc` on niro-1)
- **Build the package**: `colcon build --packages-select llm_robot_task_planner`
- **Source the overlay**: `source install/setup.bash`

## Training / Heavy Compute

**NEVER run Gazebo simulation or YOLO inference on the local MacBook Air (8 GB RAM).** Use niro-1 for anything compute-heavy.

To run on niro-1:
1. Push latest code to GitHub
2. SSH into niro-1 and pull
3. Run simulation there

## Remote Servers

### niro-1 (Lab PC)
- **Host**: 100.91.14.70
- **User**: niro-1
- **Password**: 123456
- **Access**: `sshpass -p '123456' ssh niro-1@100.91.14.70`
- **Use for**: Gazebo simulation, YOLO inference, Nav2 testing, demo recording

## Key Dependencies
- **LLM**: LangChain + OpenAI (gpt-4o-mini) or Groq (free tier)
- **Perception**: YOLOv8n (ultralytics)
- **Navigation**: Nav2 (ros2 jazzy)
- **Simulation**: Gazebo Harmonic (Gz Sim 8) + ros2_control
- **Robot**: JetRover URDF (preferred) or TurtleBot3 Waffle + arm (fallback)

## Demo Checkpoints

Each checkpoint produces a concrete artifact (video/screenshot). A checkpoint is "done" only when the artifact exists.

| # | Checkpoint | Artifact | Branch |
|---|-----------|----------|--------|
| DC1 | Robot spawns and drives with cmd_vel in Gazebo | Video clip | `robot-urdf` |
| DC2 | Robot navigates room-to-room via Nav2 | Video clip | `navigation` |
| DC3 | YOLO detects colored cubes from camera feed | Screenshot with bboxes | `perception` |
| DC4 | Depth pipeline gives correct 3D positions | Terminal log screenshot | `perception` |
| DC5 | Arm picks up and places a cube | Video clip | `arm-control` |
| DC6 | LLM decomposes "find red cube" into tool calls | Terminal screenshot | `llm-agent` |
| DC7 | End-to-end Scenario 1 (simple fetch) | Full demo video | `integration` |
| DC8 | Error recovery — Scenario 2 (search + replan) | Full demo video | `integration` |
| DC9 | Final demo video (split screen: Gazebo + RViz + terminal) | Polished video | `main` |

Track status in `docs/workflow_tracker.md`.

## Build Plan Reference
- **Week 1**: Foundation — Gazebo world, robot URDF, Nav2, semantic map (DC1-DC2)
- **Week 2**: Perception + Arm + Agent — YOLO, arm control, world model, LLM agent (DC3-DC6)
- **Week 3**: Integration + Demo — end-to-end pipeline, error recovery, demo video (DC7-DC9)

## Demo Scenarios (Success Criteria)
1. Simple fetch: "Bring me the red cube from room A"
2. Search with replanning: "Find the green cube and put it in room A"
3. Multi-step cleanup: "Move all cubes to room B"
4. Status query: "What do you see?"
5. Ambiguity handling: "Get the cube" -> asks for clarification

Track pass/fail results in `docs/demo_results.md`.

## Auto Memory Guidelines

Memory files live in the Claude auto-memory directory. Organize by topic from day one:

| File | Contents |
|------|----------|
| `MEMORY.md` | Short index (<50 lines) — current state + links to topic files |
| `architecture.md` | Architecture decisions, node interfaces, topic/service names |
| `ros2-gotchas.md` | ROS 2 / Gazebo specific issues and fixes |
| `lessons.md` | Proven lessons (only add after verified across 2+ sessions) |

Do NOT dump everything into MEMORY.md. Keep it a concise index.

## Lessons from Prior Projects
1. **Session worklogs are essential** — they preserve context across sessions and prevent rework.
2. **Never train/simulate locally** — MacBook has 8GB RAM, will OOM. Always use niro-1.
3. **Use `PYTHONUNBUFFERED=1`** for nohup logs on remote.
4. **Systematic debugging only** — never blindly tweak. Form hypothesis, test it, iterate.
5. **Test early and continuously** — write tests alongside implementation, not after.
6. **niro-1 PYTHONPATH**: May need `PYTHONPATH=.` when running scripts on niro-1.
7. **Start simple, iterate** — e.g., start with differential drive before holonomic; start with JSON tool calls before full LangChain agent framework.
8. **Document decisions** — every non-obvious choice gets recorded in the worklog AND `docs/decisions.md`.
