# Arm Control Work Log — What Was Done (S08–S09)

This is a detailed technical log of all the work done to get the 5-DOF arm + gripper working in Gazebo Harmonic, after the perception blog posts (Posts 4-5, sessions S06-S07).

## Starting Point

After S07, the arm joints had initial damping/friction values (`damping=10.0, friction=5.0` on all joints) that prevented gravity collapse at startup. PID controllers were configured as `P=200, I=5, D=20, cmd_max=20` for all joints. The arm was untested — no commands had been sent to verify joint motion.

## Problem 1: Oscillation with D Gain

**Symptom**: When sending target positions, the arm oscillated violently instead of settling.

**Root cause**: On low-inertia links (~1e-5 kg*m^2 for arm links), velocity spikes are instantaneous. The D term (proportional to velocity) saturated at `cmd_max`, opposing the P term. The net PID output was approximately zero because P and D were canceling each other at the saturation limits.

**Fix**: Set `D_gain=0` for all joints. Physical damping (the `<dynamics damping="X"/>` element in URDF) provides derivative stabilization instead. This is a fundamental pattern: for Gazebo's DART physics with low-inertia links, never use PID D gain — use physical damping.

## Problem 2: DART Numerical Lockup (Damping/Inertia Ratio)

**Symptom**: Joints 3, 4, 5 were completely immovable despite the PID outputting maximum torque. The joints received commands but didn't move at all.

**Root cause**: The DART physics engine has numerical issues when the damping-to-inertia ratio exceeds ~25,000. With `damping=10.0` and link inertia ~3e-5 kg*m^2, the ratio was ~330,000. The solver effectively treated the joints as frozen.

**Investigation process**:
1. Verified PID was outputting commands (checked Gazebo topics)
2. Verified joint limits weren't hit (well within +-2.09 rad)
3. Tried increasing PID gains — no effect
4. Tried unique plugin names — discovered `name` must be the class name `gz::sim::systems::JointPositionController`, not an instance label
5. Reduced damping progressively: 20 -> 10 -> 5 -> 1.0
6. Damping=1.0 worked for joints 3/4/5 (ratio ~25k, borderline)

**Fix**: Different damping per joint based on link inertia:
- Joints 1-2 (heavy links, inertia ~3e-5): `damping=10.0, friction=2.0`
- Joints 3-4-5 (light links, inertia ~1e-5): `damping=1.0, friction=0.5`
- Gripper r_joint (tiny links, inertia ~4e-7): `damping=0.001, friction=0.001`

## Problem 3: Joint4 Stuck During Multi-Joint Motion

**Symptom**: Joint4 worked correctly when tested in isolation, but failed to move when joints 2 and 3 were moving simultaneously. All joints were sent commands at the same time.

**Root cause**: Coupling torques. When joint2 (shoulder) and joint3 (elbow) both rotate large angles simultaneously, they create reaction torques on downstream joints. Joint4 (wrist), with `cmd_max=5`, couldn't generate enough force to overcome the coupling.

**Investigation process**:
1. Tested joint4 alone — worked fine
2. Tested joint4 with joint3 moving — failed
3. Tested joint2 alone — reached -0.923 (target -1.0)
4. Sent repeated commands to joint4 after arm settled — still didn't move (I term not yet added)
5. Increased cmd_max from 5 to 20 — joint4 started responding
6. Added staged commands — joint4 reached target accurately

**Fix**: Two-part solution:
1. Increased `cmd_max` to 20 for joints 3/4/5 (was 5)
2. Staged joint commands: send joints 1-2 first, wait 1.5s for them to settle, then send joints 3-5

## Problem 4: P-Only Steady-State Error

**Symptom**: Joints reached ~90-95% of their target position and stopped. Joint2 at -0.923 instead of -1.0 (8% error).

**Root cause**: Proportional-only control has inherent steady-state error. At equilibrium: `P * error = gravity_torque + friction`. The error never reaches zero because the P term approaches zero as the joint approaches the target.

**Fix**: Added I gain with anti-windup clamping:
- Joints 1-2: `I=50, i_max=10, i_min=-10`
- Joints 3-5 + gripper: `I=30, i_max=5, i_min=-5`

Result: Joint tracking improved from ~90% to ~97-99% accuracy.

## Problem 5: Gripper Completely Frozen

**Symptom**: Gripper r_joint didn't move at all when commanded. Value stayed at 0.0 regardless of commands sent.

**Root cause**: Same DART numerical lockup as Problem 2, but much worse. r_link inertia is ~4e-7 kg*m^2. Even with `damping=0.5`, the ratio was 1.25 million. The gripper also has 5 mimic joints (4-bar linkage mechanism) that all needed dynamics elements.

**Fix**:
1. Lowered r_joint `damping` from 10.0 to 0.001
2. Added `<dynamics damping="0.001" friction="0.001"/>` to all 5 mimic joints
3. Increased gripper PID: `P=100, I=30, cmd_max=10`

Result: Gripper opens (0.5 rad) and closes (-0.2 rad) reliably. Tracks within 2.5% of target.

## Problem 6: Joint5 Coupling from Gripper

**Symptom**: When the gripper opens/closes, joint5 (wrist rotation) drifts by ~0.25-1.0 radians, even though its target is 0.

**Root cause**: The gripper r_joint and joint5 share the Z rotation axis. Gripper motion creates coupling torque on joint5 through the mechanical linkage.

**Mitigation**: Increased joint5 `cmd_max` from 10 to 20. Reduced drift from ~1.4 rad to ~0.25 rad. Accepted as cosmetic — doesn't affect grasping.

## Problem 7: DDS Discovery Delay for Command Scripts

**Symptom**: Standalone Python scripts that published commands to `/arm/command` didn't work — the arm controller never received the messages.

**Root cause**: DDS middleware needs time for subscriber discovery. A new publisher node must wait for the subscriber to be discovered before publishing.

**Fix**: Added discovery loop in helper scripts:
```python
for i in range(20):
    time.sleep(0.2)
    if pub.get_subscription_count() > 0:
        break
```

## Final PID Configuration

| Joint | P | I | D | cmd_max | damping | friction |
|-------|---|---|---|---------|---------|----------|
| joint1 | 200 | 50 | 0 | 40 | 10.0 | 2.0 |
| joint2 | 200 | 50 | 0 | 40 | 10.0 | 2.0 |
| joint3 | 100 | 30 | 0 | 20 | 1.0 | 0.5 |
| joint4 | 100 | 30 | 0 | 20 | 1.0 | 0.5 |
| joint5 | 100 | 30 | 0 | 20 | 1.0 | 0.5 |
| gripper | 100 | 30 | 0 | 10 | 0.001 | 0.001 |

## Key Insight: Damping/Inertia Ratio Rule

The single most important lesson from this work: **DART's physics solver breaks when damping/inertia ratio exceeds ~25,000.** The joint becomes numerically frozen — no amount of PID torque can move it.

Scale damping proportional to link inertia:
- Heavy links (inertia ~3e-5): damping up to 10
- Medium links (inertia ~1e-5): damping ~1.0
- Tiny links (inertia ~4e-7): damping ~0.001

## Iterations Summary

The PID tuning went through approximately 8 iterations across 2 sessions:
1. P=200, I=5, D=20, damping=10 → oscillation (D term issue)
2. P=200, I=0, D=0, damping=10 → joints 3/4/5 frozen (damping ratio)
3. P=200, I=0, D=0, damping=1 (joints 3-5) → joint4 stuck during multi-joint motion
4. P=200/50, I=0, D=0, cmd_max=5/20 → joint4 intermittent
5. Staged commands added → joint4 works but steady-state error
6. I gain added → tracking within 3-5%
7. Gripper damping lowered → gripper works
8. Joint5 cmd_max increased → coupling reduced

## Files Changed

| File | Changes |
|------|---------|
| `urdf/robot.urdf.xacro` | PID gains for all 6 controllers (5 arm + gripper), cmd_max values |
| `urdf/jetrover_description/urdf/arm.urdf.xacro` | Joint dynamics (damping/friction) for joints 2-5, removed link4/servo_link2 collision meshes |
| `urdf/jetrover_description/urdf/connect.urdf.xacro` | Joint1 dynamics |
| `urdf/jetrover_description/urdf/gripper.urdf.xacro` | r_joint dynamics, added dynamics to all 5 mimic joints |
| `urdf/jetrover_description/urdf/depth_camera.urdf.xacro` | Removed camera collision meshes |
| `llm_robot_task_planner/arm_controller_node.py` | Staged joint commands, settle time, pick/place sequences |

## Problem 8: 4-Bar Linkage Gripper Broken in DART (S09)

**Symptom**: The gripper has 6 joints forming a 4-bar linkage mechanism. After S08 got r_joint working, the full gripper still couldn't physically grasp a cube — DART doesn't support SDF mimic joint constraints for the 4-bar closed kinematic chain.

**Investigation**:
1. Tried controlling all 6 gripper joints independently → ODE collision crash: `ODE INTERNAL ERROR 1: assertion "aabbBound >= dMinIntExact"`. The independently-controlled closed chain creates impossible geometric states.
2. Reduced to 2-joint control (r_joint + l_joint with mimic multiplier=-1) → fingers close visually but DART physics can't enforce the contact forces needed for grasping.
3. Researched alternatives: Gazebo DART has no reliable physics-based grasping for complex grippers.

**Fix**: Simulated grasp via teleport — standard approach in Gazebo demos:
- After gripper closes, set `self.grasped_object = obj_name` to track the held object
- During arm motions, teleport the object to the end-effector position using `gz service set_pose`
- On place: set the cube to ground level and clear `grasped_object`

## Problem 9: SingleThreadedExecutor Blocks TF During Pick/Place (S09)

**Symptom**: During pick/place sequences, the arm controller uses `time.sleep()` to wait for joints to settle. With the default `SingleThreadedExecutor`, ALL callbacks (including TF2 listener and joint_state) are blocked during sleep. TF lookups fail with stale transforms.

**Fix**: Switched to `MultiThreadedExecutor(num_threads=4)` + `ReentrantCallbackGroup` for the command subscription. TF and joint_state callbacks now run on separate threads during blocking sequences.

## Problem 10: Odom Frame Reports (0,0,0) at Startup (S09)

**Symptom**: The EE world position was wrong — computed relative to odom origin (0,0,0) instead of the robot's actual spawn position (2.0, 2.0, 0.05).

**Root cause**: DiffDrive plugin starts odometry at (0,0,0) regardless of where the robot spawns in Gazebo.

**Fix**: Query `gz model --pose` for the robot's actual Gazebo world pose, then combine with TF lookup (base_footprint→end_effector_link):
```python
# TF gives EE offset in robot frame
dx, dy, dz = tf_lookup(base_footprint → end_effector_link)
# gz model gives robot world pose
rx, ry, rz, yaw = gz_get_model_pose('jetrover')
# Rotate EE offset by robot yaw to get world coords
world_x = rx + dx * cos(yaw) - dy * sin(yaw)
world_y = ry + dx * sin(yaw) + dy * cos(yaw)
world_z = rz + dz
```

## Problem 11: Slow Subprocess Calls Drop Cube During Teleport (S09)

**Symptom**: Each `gz service set_pose` call took ~5s (subprocess spawn + `source /opt/ros/jazzy/setup.bash`). During the 3s settle period, only 1 teleport happened. The cube fell under gravity between teleports.

**Fix**: Two optimizations:
1. **Fast gz wrapper** (`/tmp/gz_fast.sh`): Pre-exports `GZ_CONFIG_PATH` and `LD_LIBRARY_PATH` instead of sourcing full ROS setup. Reduced gz calls from ~0.8s to ~0.3s.
2. **Cached robot pose**: Call `_gz_get_model_pose('jetrover')` once at start of pick/place, store in `self._cached_robot_pose`. Eliminates one subprocess call per teleport. During pick/place the robot is stationary, so the cached pose is valid.

Result: teleports happen every ~0.8s (fast enough to keep the cube tracked). Cube smoothly follows EE from z=0.025 → 0.331 → 0.470 → 0.472 during lift.

## Files Changed (S09)

| File | Changes |
|------|---------|
| `llm_robot_task_planner/arm_controller_node.py` | Teleport grasp, 2-finger gripper, cached robot pose, fast gz wrapper, MultiThreadedExecutor |
| `launch/sim_bringup.launch.py` | Bridge topics for gripper r_cmd/l_cmd |
| `urdf/robot.urdf.xacro` | 2-joint gripper controllers (r_joint + l_joint only) |
| `tests/test_pick_place.py` | Pick/place integration test |

## Result

Full pick and place sequence executes reliably with simulated grasp:
1. Gripper open -> pre_grasp -> grasp -> gripper close -> attach (simulated) -> lift with teleport (pick)
2. Place position with teleport -> grasp height -> gripper open -> place on ground (z=0.025) -> home (place)

**DC5 verified**: Cube lifted from z=0.025 to z=0.472 — delta of 0.447m.
Cube placed back at z=0.025 at (2.370, 1.999).

Joint tracking accuracy: ~95-99% for most joints. Gripper open/close within 2.5% of target. Total pick sequence time: ~20s. Total place sequence time: ~18s.
