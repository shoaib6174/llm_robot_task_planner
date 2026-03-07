# From Zero to Room-to-Room Navigation — Nav2 in Gazebo Harmonic

**TL;DR: Setting up Nav2 for a custom robot in Gazebo Harmonic is a minefield of subtle integration issues. This post covers building a static map, wiring AMCL localization, configuring costmaps, choosing the right controller — and the five separate issues I had to fix before the robot could reliably navigate between rooms through a doorway.**

---

This is post 3 in a series on building an LLM-powered robot task planner. [Post 2 covered building the Gazebo world and spawning the robot.](link-to-post-2)

By the end of this post, we'll have:
- A robot that autonomously navigates between two rooms through a doorway
- Bidirectional navigation (A→B and B→A) working reliably
- A proper understanding of how Nav2, Gazebo, and TF interact

Let's get into it.

---

## Part 1: The Static Map

### Why Not SLAM?

For a known simulation environment, SLAM is overkill. We know exactly where every wall is. A static map generated from the world geometry is faster, more accurate, and eliminates a whole class of mapping errors.

### Generating a PGM from World Geometry

Our world has two 4m × 4m rooms connected by a 1m doorway at x=4.0. The map format Nav2 expects is a PGM image (grayscale) plus a YAML metadata file.

Key parameters:
- **Resolution**: 0.05m/pixel (5cm) — standard for indoor navigation
- **Origin**: (-0.5, -0.5) — slight offset to include wall thickness
- **Image size**: 180 × 100 pixels (covers 9m × 5m)

```python
# Map pixel values: 254 = free, 0 = occupied
# Walls are drawn as filled rectangles matching the SDF geometry
# The doorway at x=4.0, y=1.5-2.5 is left as free space
```

The YAML file tells Nav2 how to interpret the image:

```yaml
image: two_room_map.pgm
mode: trinary
resolution: 0.05
origin: [-0.5, -0.5, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

**Gotcha**: If your wall thickness in the SDF doesn't match the wall pixels in the PGM, the planner and the lidar will disagree about where walls are. Double-check your wall dimensions.

---

## Part 2: Sensor Setup — LiDAR and IMU

### The Frame ID Problem

In Gazebo Harmonic, sensor frames are automatically scoped as `model_name/link_name/sensor_name`. So our lidar publishes with frame_id `jetrover/base_footprint/lidar` — but AMCL expects `lidar_frame` (matching the URDF link name).

The fix is the `gz_frame_id` element inside the sensor definition:

```xml
<gazebo reference="lidar_frame">
    <sensor name="lidar" type="gpu_lidar">
        <topic>scan</topic>
        <gz_frame_id>lidar_frame</gz_frame_id>
        <!-- ... ray config ... -->
    </sensor>
</gazebo>
```

This produces an SDF warning ("XML Element[gz_frame_id] not defined in SDF") but works correctly. It's a Gazebo extension that overrides the auto-generated frame name.

Same pattern for the IMU:

```xml
<gazebo reference="imu_link">
    <sensor name="imu" type="imu">
        <topic>imu</topic>
        <gz_frame_id>imu_link</gz_frame_id>
        <!-- ... noise config ... -->
    </sensor>
</gazebo>
```

### Bridging Sensor Topics

Each Gazebo topic needs a bridge entry to reach ROS 2:

```python
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ],
)
```

Note the direction arrows: `]` means ROS→Gazebo, `[` means Gazebo→ROS.

---

## Part 3: The TF Problem — Never Bridge Gazebo's /tf

This was the single most impactful issue. Let me explain why.

### What Goes Wrong

The DiffDrive Gazebo plugin publishes odometry AND a TF transform (odom→base_footprint) on Gazebo's `/tf` topic. The natural instinct is to bridge this:

```python
# DON'T DO THIS
'/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
```

The problem: Gazebo's `/tf` topic uses `gz.msgs.Pose_V` (a vector of poses) which contains **ALL model poses** — every link in your robot. When bridged, these flood into the ROS TF tree and conflict with `robot_state_publisher`, which is already publishing the same joint transforms from the URDF.

The result: hundreds of "moved backwards in time" warnings per second. The robot navigates for a few meters and then freezes because TF becomes unreliable.

### The Fix: odom_to_tf Node

Instead of bridging `/tf`, we bridge only `/odom` (which gives us the `nav_msgs/Odometry` message) and write a tiny node to extract the TF from it:

```python
class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
```

This publishes exactly ONE transform (odom→base_footprint) — the only TF from Gazebo that Nav2 needs. Everything else (base_link→wheels, base_link→sensors, etc.) comes from `robot_state_publisher`.

**After this fix: zero time sync errors. Clean navigation.**

---

## Part 4: Why Not nav2_bringup?

The standard approach is to include `nav2_bringup`'s `bringup_launch.py`:

```python
# Don't do this either
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(nav2_bringup_launch),
    launch_arguments={'params_file': params_file}.items(),
)
```

The problem: `nav2_bringup` hardcodes a list of nodes that includes `docking_server` and `route_server`. These require configuration we don't have (dock plugins, route graphs). They fail to configure, and the lifecycle manager hangs waiting for them.

**The fix**: Launch Nav2 nodes directly. This gives you full control over exactly which nodes run:

```python
nav2_lifecycle_nodes = [
    'map_server', 'amcl', 'controller_server',
    'smoother_server', 'planner_server',
    'behavior_server', 'bt_navigator', 'velocity_smoother',
]

# Each node launched individually with configured_params
map_server = Node(package='nav2_map_server', executable='map_server', ...)
amcl = Node(package='nav2_amcl', executable='amcl', ...)
# ... etc
```

More verbose, but you know exactly what's running and why.

---

## Part 5: The Doorway Problem — Five Fixes for One Route

Getting A→B navigation working was straightforward. Getting B→A (the return trip) took five separate fixes. Here's the diagnostic journey.

### Fix 1: Reduce Inflation Radius

**Symptom**: Planner can't find path from Room B to Room A.

**Diagnosis**: The doorway is 1.0m wide. With `robot_radius=0.15m` and `inflation_radius=0.35m`, the inflated zone from each wall extends 0.50m — leaving zero free space in the doorway.

**Fix**: Reduce `inflation_radius` from 0.35 to 0.25, leaving 0.50m of clearance.

### Fix 2: Remove Obstacle Layer from Global Costmap

**Symptom**: Even with reduced inflation, the planner can't find a path through the doorway.

**Diagnosis**: I wrote a costmap visualization script that showed the actual cell costs:

```
Doorway costmap at y=2.0 (center of doorway):
x=3.5..4.5: [.....79XXXX9..8XXX8..]
```

The `X` marks are LETHAL costs — right through the doorway opening! The static map shows free space here, so the obstacle_layer (from lidar) was the culprit. The lidar sees the wall edges at the doorway and marks nearby cells. Combined with inflation, the entire doorway gets blocked.

**Fix**: Remove `obstacle_layer` from the global costmap. The static map already has the walls. The obstacle_layer is only needed in the local costmap for real-time avoidance.

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "inflation_layer"]  # NO obstacle_layer
```

### Fix 3: Switch from DWB to Regulated Pure Pursuit

**Symptom**: Planner finds a path, but the robot goes off-course during the U-turn and ends up against the south wall at (4.3, 0.5).

**Diagnosis**: After arriving in Room B facing east, the robot needs a 180° turn to go back west. DWB (Dynamic Window Approach) evaluates sampled trajectories, and in a 4m×4m room with walls, none of its samples produce a clean U-turn. The robot arcs southward and hits a wall.

**Fix**: Switch to the Regulated Pure Pursuit (RPP) controller, which follows the planned path directly instead of searching for locally optimal trajectories:

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  desired_linear_vel: 0.3
  lookahead_dist: 0.6
  allow_reversing: true
  rotate_to_heading_min_angle: 1.6  # prefer reversing over full rotation
```

### Fix 4: Cylinder Wheel Collisions

**Symptom**: Even with RPP, the robot drifts during rotation.

**Diagnosis**: Our robot uses mecanum wheels (angled rollers for omnidirectional motion). But we're driving it with a DiffDrive plugin (differential drive, not omnidirectional). The mecanum wheel collision meshes have angled roller geometry — when DiffDrive sends rotation commands, the rollers cause unexpected lateral sliding.

I verified this with a pure rotation test: sending only angular velocity for 3 seconds caused 0.073m of lateral drift.

**Fix**: Replace the mecanum wheel collision STL meshes with simple cylinders. The visual meshes still show mecanum wheels for appearance, but the collision geometry is smooth:

```xml
<collision>
  <origin xyz="0 0 0" rpy="1.5708 0 0" />
  <geometry>
    <cylinder radius="0.048" length="0.035" />
  </geometry>
</collision>
```

### Fix 5: Disable RPP Collision Detection

**Symptom**: Controller immediately rejects the path — "RegulatedPurePursuitController detected collision ahead!"

**Diagnosis**: RPP has a built-in collision detection feature that checks the local costmap for obstacles along the planned path. The LOCAL costmap still has `obstacle_layer` (we only removed it from the global). When the robot is near the doorway, the lidar marks the wall edges, and RPP's collision check sees obstacles in the path.

This is a conflict between two systems:
- **Global costmap** (no obstacle_layer) → planner says "path exists"
- **Local costmap** (has obstacle_layer) → controller says "collision ahead"

**Fix**: Disable RPP's collision detection. The local costmap still provides obstacle avoidance through cost-regulated velocity scaling.

```yaml
use_collision_detection: false
```

### The Result

After all five fixes, bidirectional A↔B navigation works reliably:
- 100% success rate over multiple round trips
- Zero recovery behaviors needed
- ~14 seconds per leg
- Zero time sync errors

---

## Part 6: The Complete Nav2 Configuration

Here's a summary of the key configuration choices and why:

| Setting | Value | Why |
|---------|-------|-----|
| Controller | RPP (not DWB) | Reliable path following, handles U-turns |
| Planner | NavFn with A* | Fast, reliable for indoor environments |
| Global costmap plugins | static_layer + inflation | No obstacle_layer — blocks doorways |
| Local costmap plugins | obstacle_layer + inflation | Real-time avoidance from lidar |
| inflation_radius | 0.25m | Leaves clearance in 1.0m doorway |
| robot_radius | 0.15m | Matches actual robot footprint |
| allow_reversing | true | Avoids problematic 180° U-turns |
| RPP collision detection | false | Prevents conflict with local costmap |
| Wheel collision | Cylinders | Predictable with DiffDrive plugin |
| TF approach | odom_to_tf node | Never bridge Gazebo /tf |

---

## The Gotcha Table

| Problem | Symptom | Root Cause | Fix |
|---------|---------|------------|-----|
| "moved backwards in time" flood | Robot freezes after ~2m | Bridging Gazebo `/tf` brings ALL model poses, conflicts with RSP | Bridge only `/odom`, use odom_to_tf node |
| nav2_bringup fails | docking_server "Charging dock plugins not given!" | nav2_bringup hardcodes unused nodes | Launch Nav2 nodes directly |
| Sensor frame mismatch | AMCL can't localize | Gazebo scopes frames as model/link/sensor | Add `gz_frame_id` in sensor SDF |
| Planner can't find cross-room path | "Failed to create plan" | Lidar + obstacle_layer blocks doorway in costmap | Remove obstacle_layer from global costmap |
| Robot drifts during U-turn | Ends up against south wall | DWB trajectory sampling fails in confined space | Use RPP controller with allow_reversing |
| Lateral drift during rotation | Robot slides sideways while turning | Mecanum STL collision has angled rollers | Use cylinder collision geometry |
| Controller rejects valid path | "collision ahead!" immediately | RPP collision check uses local costmap (has obstacles) | Disable RPP collision detection |
| Gazebo won't start via nohup | "could not connect to display" | No DISPLAY env var in nohup context | Export DISPLAY=:1 in launch script |
| colcon setup.bash fails via SSH | Package not found | BASH_SOURCE doesn't work in bash -c | Explicitly export AMENT_PREFIX_PATH |

---

## Key Takeaways

1. **Never bridge Gazebo's `/tf` topic.** It contains every link pose and will destroy your TF tree. Bridge `/odom` and convert to TF yourself.

2. **Don't use nav2_bringup for custom setups.** It assumes a specific node set that may not match your needs. Launch nodes directly.

3. **Global and local costmaps serve different purposes.** Global = path planning with the static map. Local = real-time obstacle avoidance. Don't put obstacle_layer in the global costmap if you have narrow passages.

4. **Mecanum wheels + DiffDrive don't mix.** If your URDF has mecanum wheel meshes but you're using DiffDrive, replace the collision geometry with cylinders.

5. **Debug costmaps visually.** Write a small subscriber that prints cell costs at specific coordinates. It's the fastest way to understand why the planner is failing.

6. **The diagnostic journey matters.** Each fix revealed the next problem. Systematic debugging — not guessing — is the only way through a multi-layer integration like Nav2.

---

*Next up: Adding YOLO-based perception to detect colored cubes in the simulation. The robot can navigate — now it needs eyes.*

---

**Code**: [github.com/shoaib6174/llm_robot_task_planner](https://github.com/shoaib6174/llm_robot_task_planner) (branch: `main`)
