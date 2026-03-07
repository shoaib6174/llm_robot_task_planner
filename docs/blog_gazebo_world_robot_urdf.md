# From Vendor URDF to Gazebo Simulation — Building the World and Spawning the Robot

**TL;DR: Getting a vendor-provided robot description into your own ROS 2 Gazebo simulation involves more than dropping files in a folder. This post covers building a custom Gazebo world, integrating a third-party URDF with proper mesh resolution, wiring up Gazebo plugins, and bridging everything to ROS 2 — with every gotcha I hit along the way.**

---

This is post 2 in a series on building an LLM-powered robot task planner. [Post 1 covered setting up ROS 2, Gazebo, and PyTorch.](link-to-post-1)

By the end of this post, we'll have:
- A two-room Gazebo world with colored cubes sized for our robot's gripper
- A real robot (JetRover with mecanum chassis + 6-DOF arm) spawned and rendering in Gazebo
- ROS 2 topics bridged so we can command the robot from the terminal

Let's get into it.

---

## Part 1: Building the Gazebo World

### Why Build from Scratch?

I looked for existing two-room worlds in Gazebo's model repository and the ROS 2 ecosystem. Nothing matched what I needed: two connected rooms with a doorway, colored objects to manipulate, and proper lighting for camera-based perception. Simple box geometry is fast to build and gives you full control over dimensions.

### The Layout

```
y=4  +==================+=================+
     |                  |                 |
     |     Room A       |     Room B      |
     |   (0-4, 0-4)     |   (4-8, 0-4)   |
     |                  |                 |
     |              door (1m)             |
     |            y=1.5 to 2.5            |
     |                  |                 |
y=0  +==================+=================+
     x=0               x=4               x=8
```

Two 4m x 4m rooms connected by a 1m doorway. Walls are 0.15m thick, 2.5m tall. Large enough for a small mobile robot to navigate without Nav2 getting claustrophobic in tight spaces.

### SDF, Not URDF

Gazebo worlds use the SDF (Simulation Description Format), not URDF. The key structural difference: SDF uses `<model>` elements where each model has `<link>` children with `<visual>` and `<collision>` geometry. URDF uses `<link>` and `<joint>` for robot descriptions — different purpose, different format.

For Gazebo Harmonic (Gz Sim 8), use **SDF version 1.8**.

### World Structure

A Gazebo world SDF needs these sections:

```xml
<sdf version="1.8">
  <world name="two_room_world">
    <!-- Physics engine config -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Required system plugins -->
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Scene, lighting, models... -->
  </world>
</sdf>
```

The `Physics`, `UserCommands`, and `SceneBroadcaster` plugins are mandatory — without them, nothing simulates. `Sensors` with `ogre2` is needed for camera rendering later.

### Walls as Box Models

Each wall is a static model with a box geometry:

```xml
<model name="wall_south">
  <static>true</static>
  <pose>4 -0.075 1.25 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>8.15 0.15 2.5</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>8.15 0.15 2.5</size></box></geometry>
      <material>
        <ambient>0.9 0.9 0.85 1</ambient>
        <diffuse>0.9 0.9 0.85 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

The `<pose>` is the center of the box. A wall at y=0 with thickness 0.15 has its center at y=-0.075. The dividing wall between rooms has a gap (the doorway) — I split it into two segments, one below the door opening and one above.

**Tip**: Use slightly different ambient colors for the divider walls vs perimeter walls. It's a small thing, but it helps visually distinguish room boundaries in the Gazebo viewport.

### Room Floor Panels

I added thin colored panels (0.002m thick) just above the ground plane to visually distinguish the rooms — light beige for Room A, light gray-blue for Room B. These are visual-only (no collision needed) and make it immediately obvious which room the robot is in.

### Colored Cubes — Sized for the Gripper

This is where the robot's hardware constrains the world design.

My robot (JetRover) has a parallel-jaw gripper. Looking at the gripper URDF:
- Right finger joint origin: `xyz="-0.007 -0.014 0.027"` from gripper_link
- Left finger joint origin: `xyz="-0.007 0.014 0.027"` from gripper_link
- Finger joint separation: ~28mm

The gripper uses a 4-bar linkage mechanism with mimic joints, so the actual opening is wider than the joint separation when open. But a good rule: **make objects about 75% of the maximum gripper opening** for reliable grasping.

I sized the cubes at **3cm (0.03m)** — small enough to grip, large enough for YOLO to detect from the depth camera.

```xml
<model name="red_cube">
  <pose>2.5 2.5 0.015 0 0 0</pose>  <!-- z = half-height -->
  <link name="link">
    <inertial>
      <mass>0.05</mass>
      <inertia>
        <!-- I = (1/6) * m * s^2 = (1/6) * 0.05 * 0.0009 -->
        <ixx>7.5e-06</ixx><iyy>7.5e-06</iyy><izz>7.5e-06</izz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry><box><size>0.03 0.03 0.03</size></box></geometry>
      <surface>
        <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry><box><size>0.03 0.03 0.03</size></box></geometry>
      <material>
        <ambient>0.8 0.0 0.0 1</ambient>
        <diffuse>0.9 0.1 0.1 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

Key details:
- **Z position = half the cube height** (0.015) so it sits on the ground, not embedded in it
- **Friction mu=1.0** prevents the cube from sliding when the robot approaches
- **Inertia calculated correctly**: `I = (1/6) * mass * side^2` for a uniform cube. Getting this wrong causes physics instability — the cube will either vibrate or fly off

I placed 4 cubes: red and blue in Room A, green and yellow in Room B. These map to the demo scenarios (e.g., "bring me the red cube from Room A").

### Lighting

Three lights: one directional sun for overall illumination, plus a point light in each room's ceiling. The room lights matter — they'll affect camera perception quality later. I kept them moderate (not too bright, not too dim) with soft attenuation.

### Semantic Map Config

Alongside the world SDF, I created a YAML config mapping object names to positions and rooms:

```yaml
objects:
  red_cube:
    color: "red"
    size: 0.03
    initial_position: [2.5, 2.5, 0.015]
    expected_location: "room_a"
    graspable: true
```

This isn't used by Gazebo — it's for the LLM agent's world model. The agent needs to know "red_cube is expected in room_a" without parsing SDF files. Keeping the source of truth in a simple YAML makes it easy to update and query.

---

## Part 2: Integrating a Vendor Robot URDF

This is where it gets interesting. Most tutorials show TurtleBot3 — a robot designed for ROS 2 with clean, ready-to-use description packages. Real vendor robots are messier.

### The Starting Point

I'm using a JetRover from Hiwonder — a mecanum-wheeled chassis with a 6-DOF arm and parallel-jaw gripper. Hiwonder provides a `jetrover_description` ROS 2 package with xacro files and STL meshes. The challenge: integrating this into my own package without maintaining two separate packages.

### The Nested Package Problem

The obvious approach — drop `jetrover_description/` into your package — creates a problem. It has its own `package.xml`, so colcon tries to discover and build it as a separate package. But it's inside your package's source tree, so you get weird build conflicts.

**Solution: COLCON_IGNORE + data_files**

```
urdf/
├── robot.urdf.xacro              # Our wrapper
└── jetrover_description/         # Vendor package (nested)
    ├── COLCON_IGNORE             # Tells colcon to skip this directory
    ├── package.xml               # Ignored by colcon
    ├── meshes/                   # STL files
    └── urdf/                     # Xacro files
```

The `COLCON_IGNORE` file (empty) tells colcon "pretend this directory doesn't exist." Then in `setup.py`, we install its files as data files of *our* package:

```python
# Recursively collect all jetrover_description files
jetrover_data_files = []
jetrover_base = 'urdf/jetrover_description'
for dirpath, dirnames, filenames in os.walk(jetrover_base):
    if filenames:
        install_dir = os.path.join('share', package_name, dirpath)
        file_paths = [os.path.join(dirpath, f) for f in filenames
                      if not f.endswith('.py') and f != 'COLCON_IGNORE']
        if file_paths:
            jetrover_data_files.append((install_dir, file_paths))
```

This installs the meshes and xacro files under our package's share directory when you run `colcon build`.

### Mesh Path Resolution — The GZ_SIM_RESOURCE_PATH Trick

The vendor xacro files reference meshes like:
```xml
<mesh filename="package://jetrover_description/meshes/chassis/base_link.STL"/>
```

Since `jetrover_description` isn't a real ROS 2 package anymore (we COLCON_IGNORE'd it), Gazebo can't resolve `package://jetrover_description/`.

You could find-and-replace all ~40 mesh references, but there's a cleaner solution. In the launch file:

```python
gz_resource_path = os.path.join(pkg_dir, 'urdf')
set_gz_resource_path = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=gz_resource_path,
)
```

This tells Gazebo "also look in `<pkg_share>/urdf/` when resolving package paths." Since the directory structure is `urdf/jetrover_description/meshes/...`, Gazebo resolves `package://jetrover_description/meshes/foo.STL` to `<pkg_share>/urdf/jetrover_description/meshes/foo.STL`. Zero changes to vendor files.

### The Xacro Include Path Gotcha

When you process xacro files in a launch file, `$(dirname)` doesn't work:

```
Cannot substitute $(dirname), no file/directory information available
```

This happens because the xacro is processed programmatically (`xacro.process_file()`), not from a file context where `dirname` would resolve. The fix: use `$(find package_name)` instead:

```xml
<!-- DON'T: $(dirname)/jetrover_description/urdf/car_mecanum.urdf.xacro -->
<!-- DO: -->
<xacro:include filename="$(find llm_robot_task_planner)/urdf/jetrover_description/urdf/car_mecanum.urdf.xacro"/>
```

This resolves at xacro processing time using the installed package share directory.

### Making Wheels Drivable

The vendor URDF had all four wheel joints as `type="fixed"`. This makes sense for their real robot (motor control handles wheel actuation directly), but in Gazebo, the DiffDrive plugin needs revolute or continuous joints to actually spin the wheels.

```xml
<!-- Changed from type="fixed" to type="continuous" -->
<joint name="wheel_left_front_joint" type="continuous">
  <axis xyz="0 1 0"/>  <!-- Added rotation axis -->
  <!-- ... -->
</joint>
```

Changed all 4 wheel joints. Without this, the DiffDrive plugin loads but the robot doesn't move.

---

## Part 3: Gazebo Plugins and ROS 2 Bridging

### Choosing DiffDrive Over MecanumDrive

The JetRover has mecanum wheels, and Gazebo Harmonic does have a `gz-sim-mecanum-drive-system` plugin. But mecanum simulation requires configuring angled friction directions (`fdir1 gz:expressed_in`) on each wheel — tricky to get right and easy to debug wrong.

For initial testing, DiffDrive is simpler and gets the robot moving. Mecanum is an upgrade for later.

```xml
<gazebo>
  <plugin filename="gz-sim-diff-drive-system"
          name="gz::sim::systems::DiffDrive">
    <left_joint>wheel_left_front_joint</left_joint>
    <left_joint>wheel_left_back_joint</left_joint>
    <right_joint>wheel_right_front_joint</right_joint>
    <right_joint>wheel_right_back_joint</right_joint>
    <wheel_separation>0.2242</wheel_separation>
    <wheel_radius>0.048</wheel_radius>
    <odom_publish_frequency>50</odom_publish_frequency>
    <topic>cmd_vel</topic>
  </plugin>
</gazebo>
```

**Where do you get wheel_separation and wheel_radius?** From the URDF. The wheel joint origins give you the Y-offsets: left wheels at Y=+0.1121, right wheels at Y=-0.1121. Separation = 2 * 0.1121 = 0.2242m. Wheel radius comes from the mesh dimensions or the collision cylinder radius.

### JointStatePublisher for the Arm

The arm has joints (joint1 through joint5 + gripper r_joint) that need their states published for RViz visualization and later for arm control:

```xml
<gazebo>
  <plugin filename="gz-sim-joint-state-publisher-system"
          name="gz::sim::systems::JointStatePublisher">
    <joint_name>wheel_left_front_joint</joint_name>
    <!-- ... all wheel + arm joints ... -->
    <joint_name>r_joint</joint_name>
  </plugin>
</gazebo>
```

### The ros_gz_bridge

Gazebo and ROS 2 speak different message formats. The bridge translates between them:

```python
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        # ROS topic@ROS_msg_type]gz_msg_type  (] = ROS->Gz, [ = Gz->ROS)
        '/model/jetrover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/model/jetrover/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    ],
    remappings=[
        ('/model/jetrover/cmd_vel', '/cmd_vel'),
        ('/model/jetrover/odometry', '/odom'),
    ],
)
```

The bridge syntax is dense:
- `]` means ROS-to-Gazebo (we publish `/cmd_vel` in ROS, bridge sends it to Gazebo)
- `[` means Gazebo-to-ROS (Gazebo publishes odometry, bridge sends it to ROS)
- `remappings` clean up the topic names (Gazebo prefixes everything with `/model/<name>/`)

---

## Part 4: The Launch File

Everything comes together in a single launch file:

```python
def generate_launch_description():
    pkg_dir = get_package_share_directory('llm_robot_task_planner')

    # 1. Set mesh resolution path
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_dir, 'urdf'),
    )

    # 2. Process xacro to URDF string
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # 3. Robot state publisher (publishes TF transforms)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                      'use_sim_time': True}],
    )

    # 4. Launch Gazebo with our world
    gazebo = IncludeLaunchDescription(...)

    # 5. Spawn robot from the /robot_description topic
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'jetrover', '-topic', 'robot_description',
                   '-x', '2.0', '-y', '2.0', '-z', '0.05'],
    )

    # 6. Bridge Gazebo <-> ROS 2 topics
    bridge = Node(...)

    return LaunchDescription([
        set_gz_resource_path, gazebo,
        robot_state_publisher, spawn_robot, bridge,
    ])
```

The spawn position (`-x 2.0 -y 2.0 -z 0.05`) puts the robot in the center of Room A, slightly above ground to let it settle with physics.

---

## Part 5: Building and Running

### Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select llm_robot_task_planner
source install/setup.bash
```

### Launch

```bash
ros2 launch llm_robot_task_planner sim_bringup.launch.py
```

If everything is wired correctly, you'll see:
1. Gazebo opens with the two-room world
2. The robot appears in Room A with full mesh rendering (chassis, wheels, arm, gripper, LiDAR, camera)
3. Four small colored cubes on the floor

### Quick Test

In another terminal:

```bash
# Check topics are bridged
ros2 topic list
# Should see /cmd_vel, /odom, /joint_states, /robot_description, /tf

# Drive the robot forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

---

## Gotchas Summary

| Problem | Cause | Fix |
|---------|-------|-----|
| Nested package discovered by colcon | Vendor package.xml inside your source tree | Add empty `COLCON_IGNORE` file |
| `package://vendor_pkg/meshes/` not found | Vendor package isn't a real ROS 2 package | Set `GZ_SIM_RESOURCE_PATH` in launch file |
| `$(dirname)` fails in xacro | No file context when processed via `xacro.process_file()` | Use `$(find your_package)/path/...` |
| Robot spawns but doesn't move | Wheel joints are `type="fixed"` | Change to `type="continuous"` with axis |
| `source install/setup.bash` fails over SSH | `BASH_SOURCE[0]` doesn't resolve in non-interactive SSH | Export `AMENT_PREFIX_PATH` explicitly |
| Cubes too big for gripper | Didn't check gripper URDF dimensions | Measure finger joint separation, size objects to ~75% |

---

## What's Next

The robot spawns and the world is ready. Next steps:
- **DC1**: Verify the robot drives with `cmd_vel` and record a demo video
- **Navigation**: Set up Nav2 for autonomous room-to-room navigation
- **Perception**: Add depth camera and LiDAR sensor plugins for YOLO-based cube detection

---

**This is post 2 in a series on building an LLM-powered robot task planner.** The robot takes natural language commands ("bring me the red cube from Room A"), decomposes them into subtasks using an LLM, and executes them with ROS 2 navigation and manipulation. Follow along for the next post on getting the robot driving and navigating.
