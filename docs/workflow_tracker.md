# LLM Robot Task Planner — Workflow Tracker
## Last Updated: 2026-03-08

## Project
**LLM-Powered Robot Task Planner** — Natural language commands -> LLM task decomposition -> ROS 2 + Gazebo pick-and-place demo.

## Current Blockers
<!-- Unresolved blockers from any session. Remove when resolved. -->

| Blocker | Since Session | Details |
|---------|--------------|---------|
| (none) | — | — |

## Demo Checkpoints

| # | Checkpoint | Status | Artifact | Completed In |
|---|-----------|--------|----------|-------------|
| DC1 | Robot spawns and drives with cmd_vel | Verified (no video) | Video clip | S04 |
| DC2 | Robot navigates room-to-room via Nav2 | Verified (no video) | Video clip | S05 |
| DC3 | Color-based cube detection with bboxes | ✅ Verified | Screenshots in docs/artifacts/ | S06 |
| DC4 | Depth pipeline gives correct 3D positions | ✅ Verified | docs/artifacts/dc4_3d_position_accuracy.md | S07 |
| DC5 | Arm picks up and places a cube | Not started | Video clip | — |
| DC6 | LLM decomposes commands into tool calls | Not started | Terminal screenshot | — |
| DC7 | End-to-end Scenario 1 (simple fetch) | Not started | Demo video | — |
| DC8 | Error recovery — Scenario 2 (search + replan) | Not started | Demo video | — |
| DC9 | Final polished demo video | Not started | Split-screen video | — |

## Session Index

### Week 1: Foundation
| # | Date | Branch | Summary | Worklog |
|---|------|--------|---------|---------|
| S01 | 2026-03-07 | main | niro-1 env setup: ROS 2 Jazzy, Gazebo Harmonic, Nav2, PyTorch cu128, venv | [S01](worklogs/2026-03-07_S01.md) |
| S02 | 2026-03-07 | main | Blog formatting fixes for Medium, cover image with real logos, LinkedIn caption | [S02](worklogs/2026-03-07_S02.md) |
| S03 | 2026-03-07 | gazebo-world | Two-room Gazebo world with 4 colored cubes, semantic map config, validated on niro-1 | [S03](worklogs/2026-03-07_S03.md) |
| S04 | 2026-03-07 | robot-urdf | JetRover URDF integration, sim launch file, robot spawns in Gazebo, cubes resized to 3cm | [S04](worklogs/2026-03-07_S04.md) |
| S05 | 2026-03-07 | navigation | Nav2 stack, static map, odom_to_tf TF fix, A→B navigation succeeds, B→A needs costmap tuning | [S05](worklogs/2026-03-07_S05.md) |

### Week 2: Perception + Arm + Agent
| # | Date | Branch | Summary | Worklog |
|---|------|--------|---------|---------|
| S06 | 2026-03-08 | perception | Body RGBD camera, HSV color detection, arm damping, DC3 verified (all 4 cube colors) | [S06](worklogs/2026-03-08_S06.md) |
| S07 | 2026-03-08 | perception | 3D depth pipeline (camera intrinsics + depth + TF2), AMCL tuning, DC4 verified (0.038m close-range) | [S07](worklogs/2026-03-08_S07.md) |

### Week 3: Integration + Demo
| # | Date | Branch | Summary | Worklog |
|---|------|--------|---------|---------|
