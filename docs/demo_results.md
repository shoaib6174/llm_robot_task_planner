# Demo Results Tracker

Track each attempt at the 5 demo scenarios. Prevents "I thought this was working" regressions.

## Scenario 1: Simple Fetch
> "Bring me the red cube from room A to room B"

| Date | Session | Config / Notes | Result | Failure Reason |
|------|---------|---------------|--------|----------------|
| 2026-03-08 | S11 | qwen2.5:7b, teleport grasp (0.5s interval) | ✅ PASS | — |
| 2026-03-08 | S12 | qwen2.5:7b, teleport grasp (background thread) | ✅ PASS | — |

**S11 run**: navigate(room_a) → detect(red) → pick_up → navigate(doorway) → navigate(room_b) → place. 7 iterations, ~76s.

**S12 run**: navigate(room_a) → detect(red, scan) → pick_up → navigate(doorway, 11s) → navigate(room_b, 34s) → place at (6.183, 2.046). 8 iterations, ~100s. Teleport thread ran 46s during navigation.

## Scenario 2: Search with Replanning
> "Find the green cube and put it in room A"

| Date | Session | Config / Notes | Result | Failure Reason |
|------|---------|---------------|--------|----------------|
| 2026-03-08 | S11 | qwen2.5:7b, green cube initially in room B | ✅ PASS | — |
| 2026-03-08 | S12 | qwen2.5:7b, green cube in room B | ✅ PASS | — |

**S11 run**: navigate(room_b) → detect(green) NOT FOUND → replan → navigate(room_a) → detect(green) found → pick_up → place. 9 iterations. LLM autonomously searched room_a after room_b failed.

**S12 run**: navigate(room_b) → detect(green, scan) found at (6.0, 2.506) → pick_up → navigate(room_a, 24s) → place. 5 iterations. Green cube was in expected location this time.

## Scenario 3: Multi-step Cleanup
> "Move all cubes to room B"

| Date | Session | Config / Notes | Result | Failure Reason |
|------|---------|---------------|--------|----------------|
| 2026-03-08 | S12 | qwen2.5:7b, 4 cubes across 2 rooms | ⚠️ PARTIAL | AMCL drift after multiple rotation scans |

**S12 run**: LLM correctly planned to move all cubes. Successfully picked up red_cube from room_a. Navigation to room_b failed repeatedly (status 6) after multiple rotation scans disrupted AMCL localization. LLM attempted workarounds (doorway routing, re-detection) but navigation remained broken. Root cause: AMCL particle filter doesn't recover from continuous rotation in symmetric rooms.

## Scenario 4: Status Query
> "What do you see right now?"

| Date | Session | Config / Notes | Result | Failure Reason |
|------|---------|---------------|--------|----------------|
| 2026-03-08 | S12 | qwen2.5:7b, system prompt rules 7-8 | ✅ PASS | — |

**S12 run**: Agent called get_robot_status, reported: "Currently in room_a, not holding any object. Known objects: red_cube at [2.5, 2.5], blue_cube at [1.0, 1.0], green_cube last seen in room_b, yellow_cube last seen in room_b." No physical actions taken.

## Scenario 5: Ambiguity Handling
> "Get the cube"

| Date | Session | Config / Notes | Result | Failure Reason |
|------|---------|---------------|--------|----------------|
| 2026-03-08 | S12 | qwen2.5:7b, system prompt rule 8 | ✅ PASS | — |

**S12 run**: Agent responded: "Could you please specify a color or room where the cube is located? For example, say 'get the red cube' or 'go to room_b and get any cube.'" No tool calls made, no physical actions.
