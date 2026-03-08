# Teaching a Simulated Arm to Pick and Place — PID Tuning and Simulated Grasping in Gazebo Harmonic

**TL;DR: Getting a 5-DOF arm to pick up a cube in Gazebo sounds straightforward — just send joint angles, close the gripper, lift. In reality, I spent 8 iterations fighting DART physics engine numerical lockups, multi-joint coupling torques, and a gripper that refused to close. Then I discovered that physics-based grasping doesn't work at all for complex grippers in DART, so I had to implement a teleport-based simulated grasp. The fix wasn't better PID gains — it was understanding the relationship between joint damping and link inertia, and accepting that simulated grasping requires a different approach than real hardware.**

---

This is post 6 in a series on building an LLM-powered robot task planner. [Post 5 covered 3D object localization with depth + TF2.](link-to-post-5)

By the end of this post, we'll have:
- A 5-DOF arm that reliably reaches target poses in Gazebo
- A gripper that opens and closes on command
- A simulated grasp system that teleports cubes to follow the end effector
- Full pick and place sequences that actually lift and place cubes
- A deep understanding of why simulated PID control is harder than it looks

Let's get into it.

---

## Part 1: The Setup

### The Arm

The JetRover has a 5-DOF serial arm:
- **Joint 1**: Base rotation (Z axis)
- **Joint 2**: Shoulder pitch (Y axis) — the heavy lifter
- **Joint 3**: Elbow pitch (Y axis)
- **Joint 4**: Wrist pitch (Y axis)
- **Joint 5**: Wrist rotation (Z axis)

Plus a parallel-jaw gripper with one actuated joint (`r_joint`) and five mimic joints that form a 4-bar linkage.

### Gazebo JointPositionController

Gazebo Harmonic provides `JointPositionController` — a PID-based position controller. You configure it in URDF as a Gazebo plugin:

```xml
<gazebo>
    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
        <joint_name>joint2</joint_name>
        <topic>arm/joint2/cmd_pos</topic>
        <p_gain>200</p_gain>
        <i_gain>50</i_gain>
        <d_gain>0</d_gain>
        <cmd_max>40</cmd_max>
        <cmd_min>-40</cmd_min>
    </plugin>
</gazebo>
```

Publish a `Float64` to the topic, the PID controller applies torque to drive the joint to that position. Simple, right?

**Important**: The `name` attribute must be `gz::sim::systems::JointPositionController` — the fully qualified class name. It's NOT an instance label. Changing it to something like `joint2_controller` causes Gazebo to silently fail to load the system. I lost 20 minutes to this.

### The Arm Controller Node

On the ROS 2 side, a Python node subscribes to `/arm/command` (JSON messages) and publishes `Float64` position commands to each joint's topic:

```python
JOINT_TOPICS = [
    'arm/joint1/cmd_pos',
    'arm/joint2/cmd_pos',
    'arm/joint3/cmd_pos',
    'arm/joint4/cmd_pos',
    'arm/joint5/cmd_pos',
    'arm/gripper/cmd_pos',
]
```

The node accepts commands like `{"action": "pick"}` or `{"action": "pose", "pose": "pre_grasp"}` and executes predefined motion sequences.

---

## Part 2: The First Attempt — Everything Oscillates

The initial PID configuration was `P=200, I=5, D=20` with `cmd_max=20` and URDF joint damping of 10.0 on all joints. I sent a target position and the arm vibrated like it was possessed.

### Why D Gain Doesn't Work Here

On a real servo with significant inertia, D gain provides useful velocity damping. In simulation, the arm links are tiny (masses in grams, inertias around 1e-5 kg*m^2). When you command a step change in position:

1. The P term creates a large torque proportional to the position error
2. The link accelerates almost instantly (because it weighs nothing)
3. Velocity spikes to a huge value in one simulation step
4. The D term (proportional to velocity) generates a torque that saturates at `cmd_max`
5. The D term now **opposes** the P term with equal magnitude
6. Net PID output ≈ 0

The P and D terms are fighting each other at the saturation limits. The joint oscillates between "P wins" and "D wins" every few simulation steps.

### The Fix: D = 0, Use Physical Damping

Setting `D_gain=0` and relying on the URDF `<dynamics damping="X"/>` element for derivative stabilization works much better. Physical damping is applied by the physics engine as a velocity-dependent resistance force, integrated properly within the solver step. It doesn't saturate at `cmd_max`.

```xml
<joint name="joint2" type="revolute">
    ...
    <dynamics damping="10.0" friction="2.0"/>
</joint>
```

This is a general rule for Gazebo's DART physics with low-inertia links: **never use PID D gain — use physical damping instead.**

---

## Part 3: The DART Numerical Lockup

After setting D=0, joints 1 and 2 worked. Joints 3, 4, and 5 didn't move at all. The PID controller was outputting maximum torque (verified by echoing the Gazebo command topic), but the joints stayed frozen at zero.

### The Damping/Inertia Ratio Problem

This was the hardest bug to diagnose. The issue wasn't the PID controller — it was the physics engine.

DART (the physics solver Gazebo Harmonic uses) has numerical stability issues when the damping-to-inertia ratio is too high. Here's the math:

- Joint 2 link: mass = 0.033 kg, inertia ≈ 3e-5 kg*m^2
- Joint 3 link: mass = 0.033 kg, inertia ≈ 3e-5 kg*m^2
- Joint 4 link: mass = 0.010 kg, inertia ≈ 8e-6 kg*m^2

With `damping=10.0` on all joints:
- Joint 2 ratio: 10.0 / 3e-5 = **333,000**
- Joint 4 ratio: 10.0 / 8e-6 = **1,250,000**

When this ratio exceeds approximately **25,000**, DART's implicit integrator treats the damping term as dominant and effectively makes the joint immovable. No amount of applied torque can overcome it — the solver's numerical precision breaks down.

### The Fix: Scale Damping to Inertia

The solution is to scale damping proportional to each link's inertia:

| Joint | Link Inertia | Damping | Ratio |
|-------|-------------|---------|-------|
| 1-2 | ~3e-5 | 10.0 | 333k (borderline, but works because cmd_max=40 is high) |
| 3-4 | ~1e-5 | 1.0 | 100k |
| 5 | ~2e-8 | 1.0 | 50k |
| Gripper | ~4e-7 | 0.001 | 2,500 |

The gripper was the worst case — its link inertia is 4e-7 kg*m^2. Even `damping=0.5` gives a ratio of 1.25 million. I had to drop it to 0.001 before the gripper would move.

**This is the single most important lesson from this work**: if a Gazebo joint won't move despite the PID controller outputting torque, check the damping/inertia ratio. If it's above ~25k, lower the damping.

---

## Part 4: Multi-Joint Coupling — Why Joint 4 Worked Alone but Failed Together

After fixing the damping ratios, I could move each joint individually. But when I sent the `pre_grasp` pose (all joints moving simultaneously), joint 4 stayed frozen.

The test that revealed the pattern:
1. Send joint 2 to -1.0 alone → reached -0.923 ✓
2. Send joint 4 to -0.6 alone → reached -0.59 ✓
3. Send joints 2, 3, 4 all at once → joint 4 stuck at 0 ✗

### Coupling Torques

When joint 2 (shoulder) rapidly swings the entire arm forward, it creates reaction torques on all downstream joints. Joint 4 (wrist), with `cmd_max=5`, couldn't generate enough torque to overcome the coupling forces while joint 2 was still in motion.

### The Fix: Staged Commands + Higher Authority

Two-part solution:

**1. Increase cmd_max for downstream joints**:
```xml
<!-- Joints 3-4: increased from 5 to 20 N*m -->
<cmd_max>20</cmd_max>
```

**2. Send commands in stages** — upstream joints first:

```python
def send_pose(self, joint_values, gripper_value):
    # Stage 1: base + shoulder (high inertia, high authority)
    for i in range(2):
        msg = Float64()
        msg.data = float(joint_values[i])
        self.joint_pubs[i].publish(msg)
        time.sleep(0.02)

    time.sleep(1.5)  # let upstream joints mostly settle

    # Stage 2: elbow + wrist joints (low inertia)
    for i in range(2, 5):
        msg = Float64()
        msg.data = float(joint_values[i])
        self.joint_pubs[i].publish(msg)
        time.sleep(0.02)
```

By waiting 1.5 seconds between stages, joint 2 is mostly at its target before joints 3-5 start moving. The coupling torques are dramatically reduced.

Result with staged commands + higher cmd_max:

```
pre_grasp targets:  [0.0, -1.0, -0.4, -0.6, 0.0]
actual positions:   [0.0, -1.003, -0.401, -0.597, 0.0]
```

Joint 4 went from stuck at 0 to within 0.5% of target.

---

## Part 5: Eliminating Steady-State Error with I Gain

Even with the damping and staging fixes, joints settled at ~90-95% of their targets. Joint 2 would reach -0.923 instead of -1.0 — an 8% error.

This is the classic proportional-only steady-state error. At equilibrium:

```
P * error = gravity_torque + friction
```

Since P is finite, error can never reach zero when there's any load. The I (integral) term fixes this by accumulating error over time:

```xml
<p_gain>200</p_gain>
<i_gain>50</i_gain>
<d_gain>0</d_gain>
<i_max>10</i_max>   <!-- anti-windup clamp -->
<i_min>-10</i_min>
```

The `i_max/i_min` parameters clamp the integrator to prevent windup — without them, the integrator can grow to enormous values during large transient errors, causing overshoot when the joint finally reaches the target.

Result with PI control:
```
joint2: -1.003 (target -1.0)  — 0.3% error
joint3: -0.401 (target -0.4)  — 0.4% error
joint4: -0.597 (target -0.6)  — 0.5% error
```

---

## Part 6: The Gripper — A Whole Separate Debugging Session

The gripper had the worst case of the damping/inertia problem. The r_joint link has inertia of ~4e-7 kg*m^2. Even after lowering damping to 0.5, the ratio was 1.25 million. The gripper was completely frozen.

### The Mimic Joints

The gripper is a parallel-jaw mechanism with 6 joints:
- `r_joint` — actuated (controlled by PID)
- `l_joint`, `l_in_joint`, `l_out_joint`, `r_in_joint`, `r_out_joint` — mimic joints

URDF mimic joints tell Gazebo to keep these joints synchronized with `r_joint` (some with multiplier=-1 for opposite fingers). But these mimic joints also needed `<dynamics>` elements — without them, the default behavior in the physics engine could resist motion.

### The Fix

```xml
<!-- Actuated joint: extremely low damping for tiny inertia -->
<joint name="r_joint" type="revolute">
    ...
    <dynamics damping="0.001" friction="0.001"/>
</joint>

<!-- Each mimic joint also gets dynamics -->
<joint name="l_joint" type="revolute">
    ...
    <dynamics damping="0.001" friction="0.001"/>
    <mimic joint="r_joint" multiplier="-1" offset="0"/>
</joint>
```

Result:
```
gripper close: r_joint = -0.205 (target -0.2)  — 2.5% error ✓
gripper open:  r_joint = 0.506  (target 0.5)   — 1.2% error ✓
```

### Joint 5 Coupling

One side effect: when the gripper opens/closes, it creates a coupling torque on joint 5 (wrist rotation — same Z axis). Joint 5 drifts by ~0.25 radians during gripper operation. Increasing joint 5's `cmd_max` from 10 to 20 reduced this but didn't eliminate it.

For the demo, this is cosmetic — the wrist rotating slightly during a grasp doesn't affect whether the cube gets picked up. A proper fix would require decoupling the gripper mechanism from joint 5, which isn't worth the engineering effort for a simulation demo.

---

## Part 7: Why Physics-Based Grasping Doesn't Work Here

At this point, all joints work, the gripper opens and closes, and the arm can execute motion sequences. Time to pick up a cube, right?

Not so fast. The gripper is a 4-bar linkage mechanism with 6 joints — one actuated (`r_joint`) and five mimic joints. The mimic joints enforce a closed kinematic chain where all fingers move together. In SDF format, this works via `<mimic>` tags that constrain joints to follow the master.

### DART Doesn't Support Mimic Constraints

DART — the physics engine Gazebo Harmonic uses — **does not support SDF mimic joint constraints**. The `<mimic>` tags are parsed but silently ignored. Each joint moves independently.

I tried three approaches:

**Approach 1: Control all 6 joints independently.** Send synchronized position commands to all gripper joints, each with its own PID controller. Result: `ODE INTERNAL ERROR 1: assertion "aabbBound >= dMinIntExact"` — Gazebo crashes. The independently-controlled closed kinematic chain creates impossible geometric states that crash the collision solver.

**Approach 2: Control only 2 joints (r_joint + l_joint).** The outer fingers (r_joint and l_joint) are the only ones that touch the object. Send opposite commands to close them. Result: fingers close visually, but DART's contact physics can't generate enough friction/normal force through these tiny links (~4e-7 kg*m^2 inertia) to hold a 5cm cube against gravity. The cube slips through.

**Approach 3: Simulated grasp via teleport.** This is the standard approach in Gazebo demos and most published robotics simulation work. Instead of relying on contact physics to hold the object, you programmatically teleport the object to follow the end effector.

### How Simulated Grasp Works

The concept is simple:
1. Close the gripper (visual only — the fingers close around the cube for the camera)
2. Mark the object as "grasped" in software: `self.grasped_object = "red_cube"`
3. During any arm motion, teleport the grasped object to the end-effector's world position
4. On release, open the gripper and place the object at its final position

The teleport uses Gazebo's `set_pose` service:

```python
def gz_set_model_pose(self, model_name, x, y, z):
    req = f'name: "{model_name}", position: {{x: {x}, y: {y}, z: {z}}}'
    self._gz_cmd([
        'service', '-s', f'/world/{GZ_WORLD}/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000',
        '--req', req,
    ])
```

This is used in Gazebo tutorials, university robotics courses, and most simulation-based manipulation demos. The alternative — implementing a custom Gazebo plugin for constraint-based attachment — would be significantly more complex for the same visual result.

---

## Part 8: Getting the End-Effector World Position

To teleport the cube to the right place, we need to know where the end effector is in the Gazebo world frame. This requires combining two pieces of information:

### Step 1: TF Lookup (Robot Frame)

ROS 2's TF2 gives us the end-effector position relative to the robot's base:

```python
t = self.tf_buffer.lookup_transform(
    'base_footprint', 'end_effector_link', rclpy.time.Time(),
    timeout=rclpy.duration.Duration(seconds=1.0))
dx = t.transform.translation.x  # EE offset in robot frame
dy = t.transform.translation.y
dz = t.transform.translation.z
```

### Step 2: Robot's World Pose (Gazebo)

The odometry frame starts at (0,0,0) regardless of where the robot spawns in Gazebo. So we can't use odom for world coordinates. Instead, we query Gazebo directly:

```python
robot_pose = self._gz_get_model_pose('jetrover')  # (x, y, z, yaw)
rx, ry, rz, robot_yaw = robot_pose
```

### Step 3: Combine with Rotation

The EE offset is in the robot's local frame. To convert to world coordinates, rotate by the robot's yaw:

```python
cos_yaw = math.cos(robot_yaw)
sin_yaw = math.sin(robot_yaw)
world_x = rx + dx * cos_yaw - dy * sin_yaw
world_y = ry + dx * sin_yaw + dy * cos_yaw
world_z = rz + dz
```

This gives us the end effector's position in the same frame Gazebo uses for `set_pose`.

### The Threading Problem

There's a catch: the TF lookup requires TF callbacks to be running. But the pick/place sequence uses `time.sleep()` to wait for joints to settle, which blocks the entire event loop in a single-threaded executor. No callbacks run → TF data goes stale → lookups fail.

The fix: `MultiThreadedExecutor` with a `ReentrantCallbackGroup`:

```python
# In __init__:
self.cmd_cb_group = ReentrantCallbackGroup()
self.create_subscription(
    String, '/arm/command', self.command_cb, 10,
    callback_group=self.cmd_cb_group)

# In main():
executor = MultiThreadedExecutor(num_threads=4)
```

Now TF and joint_state callbacks run on separate threads while the command callback sleeps.

---

## Part 9: Making Teleport Fast Enough

Each teleport requires a subprocess call to `gz service`. The first version sourced the full ROS setup on every call:

```python
subprocess.run(['bash', '-c',
    'source /opt/ros/jazzy/setup.bash && gz service ...'])
```

This took ~0.8 seconds per call. During the 3-second settle time after each joint command, we could only teleport once. The cube fell under gravity between teleports, bouncing off the floor and never staying "held."

### Optimization 1: Fast Gazebo Wrapper

Instead of sourcing `setup.bash` (which parses hundreds of package configs), I created a wrapper script that pre-exports only the env vars `gz` needs:

```bash
#!/bin/bash
export GZ_CONFIG_PATH=/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:...
export LD_LIBRARY_PATH=/opt/ros/jazzy/opt/gz_sim_vendor/lib:...
export DISPLAY=:1
exec /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz "$@"
```

This cuts `gz` call time from ~0.8s to ~0.3s.

### Optimization 2: Cache the Robot Pose

During a pick or place, the robot doesn't move — only the arm moves. So we query the robot's Gazebo pose once at the start and reuse it:

```python
def execute_pick(self, obj_name):
    self._cached_robot_pose = self._gz_get_model_pose('jetrover')
    # ... all teleports use cached pose, no subprocess per lookup
    self._cached_robot_pose = None  # clear after pick
```

This eliminates one subprocess call per teleport iteration.

### The Teleport Loop

With both optimizations, teleports happen every ~0.8s — fast enough to keep the cube tracked:

```python
def move_to(self, target_joints, gripper_value, settle=3.0):
    self.send_pose(target_joints, gripper_value)
    if self.grasped_object:
        elapsed = 0.0
        while elapsed < settle:
            time.sleep(0.5)
            elapsed += 0.5
            self.teleport_grasped_to_ee()
    else:
        time.sleep(settle)
```

The cube's trajectory during a lift shows it smoothly tracking the EE:
```
Teleported red_cube to (2.336, 1.999, 0.013)   # near ground
Teleported red_cube to (2.393, 1.999, 0.331)   # rising
Teleported red_cube to (2.320, 1.999, 0.470)   # near top
Teleported red_cube to (2.318, 1.999, 0.472)   # settled at lift height
```

---

## Part 10: The Complete Pick and Place

With simulated grasping working, the full sequence is:

### Pick
```python
def execute_pick(self, obj_name):
    self._cached_robot_pose = self._gz_get_model_pose('jetrover')
    self.send_gripper(GRIPPER_OPEN)          # 1. Open gripper
    self.move_to(POSES['pre_grasp'], ...)    # 2. Hover above target
    self.move_to(POSES['grasp'], ...)        # 3. Lower to cube height
    self.send_gripper(GRIPPER_CLOSE)         # 4. Close gripper (visual)
    self.grasped_object = obj_name           # 5. Attach (simulated)
    self.move_to(POSES['lift'], ...)         # 6. Lift — cube teleports to EE
```

### Place
```python
def execute_place(self):
    self._cached_robot_pose = self._gz_get_model_pose('jetrover')
    self.move_to(POSES['place'], ...)        # 1. Move to place (cube follows)
    self.move_to(POSES['grasp'], ...)        # 2. Lower to drop height
    self.send_gripper(GRIPPER_OPEN)          # 3. Open gripper
    # Place cube on ground at EE's x,y position
    pos = self.get_ee_world_position()
    self.gz_set_model_pose(obj, pos[0], pos[1], 0.025)
    self.grasped_object = None               # 4. Detach
    self.move_to(POSES['home'], ...)         # 5. Return home
```

### The Result

```
Cube before: [2.34, 2.0, 0.025]     # on the ground
Cube after pick: [2.319, 1.997, 0.472]  # lifted 0.447m
Cube after place: [2.370, 1.999, 0.025] # back on ground
```

**The cube lifts 44.7 centimeters off the ground** — definitively picked up, carried, and placed back down. DC5 verified.

### The Command Interface

The arm controller accepts JSON commands on `/arm/command`:

```json
{"action": "pick", "object": "red_cube"}
{"action": "place"}
{"action": "pose", "pose": "pre_grasp"}
{"action": "gripper", "state": "close"}
{"action": "joints", "values": [0.0, -0.5, -0.3, -0.4, 0.0]}
```

And publishes status on `/arm/status`:
```json
{"state": "picking", "message": "Starting pick of red_cube"}
{"state": "done", "message": "Pick complete — holding red_cube"}
```

This gives the LLM agent (which we'll build later) a clean interface to control the arm without knowing anything about joint angles or simulated grasping.

---

## Part 11: The Final PID Configuration

After 8 iterations of tuning:

| Joint | P | I | D | cmd_max | URDF damping | URDF friction |
|-------|---|---|---|---------|-------------|---------------|
| joint1 (base) | 200 | 50 | 0 | 40 | 10.0 | 2.0 |
| joint2 (shoulder) | 200 | 50 | 0 | 40 | 10.0 | 2.0 |
| joint3 (elbow) | 100 | 30 | 0 | 20 | 1.0 | 0.5 |
| joint4 (wrist pitch) | 100 | 30 | 0 | 20 | 1.0 | 0.5 |
| joint5 (wrist rot) | 100 | 30 | 0 | 20 | 1.0 | 0.5 |
| gripper | 100 | 30 | 0 | 10 | 0.001 | 0.001 |

The pattern: heavier links get more damping and more PID authority. Lighter links get proportionally less damping. The gripper, with the smallest links, gets three orders of magnitude less damping than the shoulder.

---

## The Gotcha Table

| Problem | Symptom | Root Cause | Fix |
|---------|---------|------------|-----|
| Arm oscillates violently | Joints vibrate instead of settling | D gain saturates at cmd_max, opposes P gain | Set D_gain=0, use physical damping |
| Joints completely frozen | PID outputs max torque, joint doesn't move | Damping/inertia ratio > 25k breaks DART solver | Lower damping proportional to link inertia |
| Joint works alone, fails in group | Downstream joints stuck during multi-joint motion | Coupling torques from upstream motion exceed cmd_max | Stage commands (upstream first) + increase cmd_max |
| Steady-state position error | Joints reach ~90% of target | P-only control has inherent error under load | Add I gain with anti-windup clamping |
| Gripper won't close | r_joint frozen at 0 | Damping=0.5 with inertia=4e-7 → ratio 1.25M | Damping=0.001, add dynamics to mimic joints |
| Plugin name "joint2_ctrl" fails | Controller not loaded, no error message | `name` must be class name, not instance name | Use `gz::sim::systems::JointPositionController` |
| Wrist rotates during gripper operation | Joint 5 drifts ~0.25 rad | Coupling torque from shared Z axis | Increase cmd_max (partial fix); accept for demo |
| Standalone command scripts ignored | Messages published but not received | DDS discovery delay | Wait for `get_subscription_count() > 0` |
| 4-bar linkage crashes ODE | `ODE INTERNAL ERROR 1: assertion failed` | 6-joint independent control creates impossible kinematic states | Use 2-joint gripper + simulated grasp |
| DART doesn't support mimic joints | Fingers close but can't hold object | SDF mimic constraints silently ignored by DART | Teleport-based simulated grasp |
| Odom starts at (0,0,0) not spawn pose | EE world position is wrong | DiffDrive odometry starts at origin | Query `gz model --pose` for actual world pose |
| TF lookup fails during sleep | Stale transforms, timeout errors | SingleThreadedExecutor blocks all callbacks during `time.sleep()` | MultiThreadedExecutor + ReentrantCallbackGroup |
| Cube falls between teleports | Object drops during arm motion | Subprocess call too slow (~0.8s) for tight teleport loop | Fast gz wrapper + cached robot pose |

---

## Key Takeaways

1. **In Gazebo DART, the damping/inertia ratio is everything.** Above ~25,000, joints freeze. This isn't a PID problem — it's a physics solver numerical stability issue. Scale damping proportional to link inertia.

2. **D gain is counterproductive for low-inertia simulated links.** Velocity spikes instantly, D term saturates at cmd_max opposing P term. Use physical damping (`<dynamics damping="X"/>`) instead of PID D gain.

3. **Multi-joint coupling requires staged commands.** Sending all joint targets simultaneously causes upstream motion to overwhelm downstream controllers. Send heavy upstream joints first, wait for them to settle, then send downstream joints.

4. **PI control eliminates steady-state error.** P-only control always leaves a residual error proportional to the load. Adding I gain with anti-windup clamping drives the error to near zero.

5. **Mimic joints need dynamics elements too.** In Gazebo Harmonic, URDF mimic joints without `<dynamics>` tags can resist motion in unexpected ways. Add low damping/friction to keep them cooperative.

6. **Physics-based grasping rarely works in Gazebo with complex grippers.** DART doesn't support mimic constraints, and low-inertia finger links can't generate reliable contact forces. Teleport-based simulated grasping is the standard approach — it's what most Gazebo tutorials and published demos use. Don't fight the physics engine; work around it.

7. **MultiThreadedExecutor is essential for blocking sequences.** Any ROS 2 node that uses `time.sleep()` in callbacks needs multiple threads, otherwise all other callbacks (TF, joint states) are blocked. Use `ReentrantCallbackGroup` for the callbacks that do the blocking.

8. **Simulation PID tuning is iterative.** The "textbook" approach of calculating gains from a plant model doesn't account for physics engine quirks. Expect 5-10 iterations of test-observe-adjust. Each iteration teaches you something about the system.

---

*Next up: The LLM agent that ties it all together — natural language commands to robot actions.*

---

**Code**: [github.com/shoaib6174/llm_robot_task_planner](https://github.com/shoaib6174/llm_robot_task_planner)
