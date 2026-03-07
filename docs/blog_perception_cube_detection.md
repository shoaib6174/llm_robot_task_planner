# Adding Eyes to the Robot — RGBD Camera and Color-Based Cube Detection in Gazebo

**TL;DR: YOLO doesn't know what a "cube" is. Instead of training a custom model, I used HSV color segmentation for reliable object detection in simulation. But getting a working camera feed required understanding how Gazebo renders cameras (hint: it's not the optical frame convention), preventing the robot arm from knocking itself across the room at startup, and making 5cm cubes visible at room-scale distances.**

---

This is post 4 in a series on building an LLM-powered robot task planner. [Post 3 covered Nav2 room-to-room navigation.](link-to-post-3)

By the end of this post, we'll have:
- An RGBD camera sensor on the robot publishing color + depth images
- A perception node that detects red, green, blue, and yellow cubes with bounding boxes
- Depth estimation for each detected cube
- All four cube colors reliably detected through a 360° scan

Let's get into it.

---

## Part 1: Why Not YOLO?

This was the first decision that saved me hours. The original plan was to use YOLOv8 for cube detection — it's fast, it's well-supported, and it runs great on the RTX 5090 we have on the lab PC.

The problem: YOLO is trained on the COCO dataset, which has 80 object classes. None of them is "cube." The closest is "suitcase" or "box," which won't reliably detect a 5cm colored cube sitting on a simulation floor.

The alternatives:
1. **Train a custom YOLO model** — requires labeled data from the simulation, training time, and iteration. Worth it for real hardware, overkill for simulation.
2. **Use HSV color segmentation** — the cubes have known, distinct colors (red, green, blue, yellow). In simulation, these colors render consistently. No training, no GPU, deterministic.

For a simulation demo where the goal is to prove the full pipeline works end-to-end, HSV segmentation is the right call. We can swap in YOLO later for real hardware where lighting varies and colors aren't perfect.

---

## Part 2: Adding the Camera — The Rendering Direction Trap

### The Approach: Fixed Body Camera

Our robot has a 6-DOF arm with a depth camera on the wrist. The natural thought is to use that camera for perception. But the arm camera's view depends entirely on joint positions — at the default all-zeros pose, it points at the ceiling.

Better approach: add a **fixed body camera** on the robot's chassis that always faces forward. Use the arm camera later for close-up pick/place guidance.

### The URDF Setup

The camera needs three things:
1. A physical link (for TF)
2. An optical frame (for ROS image convention: z-forward, x-right, y-down)
3. A Gazebo sensor plugin (for actual rendering)

```xml
<!-- Camera link: tilted 20° down to see the floor -->
<link name="body_cam_link">
    <inertial>
        <mass value="0.01"/>
        <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
</link>
<joint name="body_cam_joint" type="fixed">
    <origin xyz="0.12 0.0 0.06" rpy="0 0.35 0"/>
    <parent link="base_link"/>
    <child link="body_cam_link"/>
</joint>

<!-- Optical frame (standard ROS convention) -->
<link name="body_cam_optical_frame"/>
<joint name="body_cam_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
    <parent link="body_cam_link"/>
    <child link="body_cam_optical_frame"/>
</joint>
```

The camera link sits at the front-top of the chassis with a 20° downward tilt (0.35 rad pitch). This angle captures both the floor nearby and objects at room distance.

### The Trap: Where You Attach the Sensor Matters

Here's where I lost an hour. The Gazebo sensor goes in a `<gazebo reference="...">` block. My first attempt:

```xml
<!-- WRONG — attaches sensor to the optical frame -->
<gazebo reference="body_cam_optical_frame">
    <sensor name="body_rgbd_camera" type="rgbd_camera">
        ...
    </sensor>
</gazebo>
```

The result: a sideways, rotated camera view that showed walls at 45° angles. Nothing was where it should be.

**The root cause**: Gazebo cameras render along the **+x axis** of whatever frame they're attached to. The optical frame has a rotation of `rpy=(-π/2, 0, -π/2)` to convert from Gazebo convention to ROS optical convention. When you attach the sensor to this rotated frame, Gazebo's renderer dutifully points the camera along the rotated +x — which is now sideways.

**The fix**: Attach the sensor to `body_cam_link` (the unrotated frame), and use `gz_frame_id` to set the frame_id in published messages to the optical frame:

```xml
<!-- CORRECT — sensor on cam_link, frame_id overridden to optical -->
<gazebo reference="body_cam_link">
    <sensor name="body_rgbd_camera" type="rgbd_camera">
        <topic>camera</topic>
        <gz_frame_id>body_cam_optical_frame</gz_frame_id>
        <update_rate>15</update_rate>
        <always_on>true</always_on>
        <camera>
            <horizontal_fov>1.57</horizontal_fov>
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
            </image>
            <clip><near>0.05</near><far>10.0</far></clip>
            <depth_camera>
                <clip><near>0.05</near><far>10.0</far></clip>
            </depth_camera>
        </camera>
    </sensor>
</gazebo>
```

This is the same pattern from the lidar and IMU in Post 3: `gz_frame_id` controls the frame name in published messages, but does NOT affect rendering direction. The distinction matters here because cameras are directional.

**Key insight**: In Gazebo Harmonic, `<gazebo reference="X">` means "attach to link X." The sensor renders along +x of that link. If you need a different frame_id for ROS (e.g., the optical frame), use `gz_frame_id` — don't attach to a rotated frame.

### Camera Parameters

A few things I tuned:
- **Horizontal FOV: 90°** (1.57 rad). Started at 60°, but a cube 0.5m ahead and 0.5m to the side (45° angle) was outside the frame. Wider FOV catches more of the room.
- **Tilt: 20° down** (0.35 rad pitch). Enough to see cubes on the floor at 0.5–2m distance without pointing entirely at the ground.
- **Resolution: 640×480**. Plenty for color segmentation. No need for 1080p.
- **Update rate: 15 Hz**. Fast enough for real-time detection without drowning the system.

---

## Part 3: The Arm Gravity Problem

Before I could test the camera, I had to solve a physics problem I didn't see coming.

### The Symptom

The robot spawns at (2.0, 2.0) in Room A. Within 2 seconds, it's at (2.5, 0.5) with a yaw of -90°. The wheels have no velocity commands. The robot just... slid across the room on its own.

### The Diagnosis

The 6-DOF arm joints have `JointPositionController` plugins that use PID control to hold positions. But here's the critical behavior: **JointPositionController applies zero force until it receives its first command.**

At simulation startup, no commands have been published yet. The arm's five revolute joints have zero resistance. Gravity pulls the arm down. The arm's momentum transfers to the base. The robot slides.

It's a 3-second chain reaction:
1. Sim starts, arm is at default (all zeros — upright)
2. No position commands yet → controllers apply zero torque
3. Gravity pulls the arm forward and down
4. The falling arm's weight pushes the robot sideways
5. By the time the sim settles, the robot is 1.5m from where it spawned

### The Fix: Joint Damping

URDF revolute joints support a `<dynamics>` element with damping and friction coefficients. These simulate the resistance of a real servo motor even when no control signal is applied:

```xml
<joint name="joint2" type="revolute">
    <origin xyz="0 0 0.0338648012164686" rpy="0 0 0" />
    <parent link="servo_link1" />
    <child link="link2" />
    <axis xyz="0 1 0" />
    <limit lower="-2.09" upper="2.09" effort="1000" velocity="10" />
    <dynamics damping="10.0" friction="5.0"/>
</joint>
```

I added `damping="10.0" friction="5.0"` to all five arm joints (joint1 through joint5). These values are high enough to resist gravity (the arm links are very light — ~10–30g each) but low enough that the PID controllers can still move the arm when commanded.

**After the fix**: robot stays exactly at (2.0, 2.0) at startup. The arm holds its position until commanded to move.

---

## Part 4: Making Cubes Detectable

### The Size Problem

Our cubes were originally 3cm — sized for the gripper's 28mm finger gap. At 2m distance, a 3cm cube is ~10 pixels tall in a 640×480 image. That's barely visible, let alone reliably detectable.

The PRD spec says 5–8cm. I bumped the cubes to 5cm, which gives ~25 pixels at 2m distance — enough for color segmentation to work.

In the world SDF, this meant updating every cube:

```xml
<!-- Before -->
<box><size>0.03 0.03 0.03</size></box>

<!-- After -->
<box><size>0.05 0.05 0.05</size></box>
```

Don't forget to update the z-position too — cubes sit on the floor, so the center should be at half-height (0.025m instead of 0.015m). And the inertia tensor: `I = (1/6) × m × s²`.

### Bridging the Camera Topics

RGBD cameras in Gazebo Harmonic publish three topics:
- `{topic}/image` — RGB image
- `{topic}/depth_image` — depth map (32-bit float, meters)
- `{topic}/camera_info` — camera intrinsics

Each needs a bridge entry:

```python
'/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
'/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
'/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
```

All three use `[` (Gazebo→ROS direction).

---

## Part 5: The Perception Node

### HSV Color Segmentation

The detection pipeline is straightforward:

1. Convert BGR image to HSV color space
2. For each target color, create a binary mask using `cv2.inRange()`
3. Clean the mask with morphological operations (open + close)
4. Find contours, filter by minimum area
5. Extract bounding box, center pixel, and depth from the depth image

```python
COLOR_RANGES = {
    'red': [
        ((0, 120, 70), (10, 255, 255)),      # Red wraps around hue 0/180
        ((170, 120, 70), (180, 255, 255)),
    ],
    'green': [((35, 120, 70), (85, 255, 255))],
    'blue':  [((100, 120, 70), (130, 255, 255))],
    'yellow': [((20, 120, 70), (35, 255, 255))],
}
```

Notice that red needs two ranges because it wraps around the hue circle (0 and 180 are both red in HSV). The saturation and value minimums (120, 70) filter out washed-out or dark pixels — wall reflections, shadows, etc.

### Depth Estimation

For each detected cube, I sample the depth image at the contour's center pixel using a 5×5 median:

```python
def _get_depth_at(self, cx, cy):
    patch = self.latest_depth[cy-2:cy+3, cx-2:cx+3]
    valid = patch[np.isfinite(patch) & (patch > 0)]
    if len(valid) == 0:
        return None
    return float(np.median(valid))
```

Median over a small window handles noisy depth values and avoids edge artifacts where depth transitions between the cube and the floor.

### The cv_bridge NumPy Trap

Our venv has NumPy 2.x (installed with ultralytics). The system `cv_bridge` package was compiled against NumPy 1.x. When you activate the venv and import cv_bridge:

```
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.4.2
```

**Fix**: Don't activate the venv when running ROS nodes that use cv_bridge. The colcon-installed entry points use system Python by default, which has the compatible NumPy 1.x. This is fine because the perception node only needs OpenCV and NumPy, both available system-wide.

---

## Part 6: Verification — The 360° Scan

To verify all four colors are detectable, I wrote a test script that slowly rotates the robot 360° while recording detections:

```python
# Rotate at 0.4 rad/s for ~16 seconds (full circle)
while time.time() - start < 16:
    node.rotate(0.4)
    rclpy.spin_once(node, timeout_sec=0.1)
```

### Results

From the robot's spawn point at (2.0, 2.0) in Room A:

| Color | Distance | Pixel Area | Notes |
|-------|----------|-----------|-------|
| Red | 0.45m | 2241 px | Nearby in Room A, large and clear |
| Blue | 0.86m | 615 px | Across Room A, medium |
| Green | 0.95m | ~350 px | Visible through doorway in Room B |
| Yellow | detected | ~280 px | Adjacent to green in Room B |

Driving into Room B and scanning again detected the green and yellow cubes up close with both bounding boxes and depth labels visible in the annotated image.

---

## Part 7: Detection Output Format

Each detection is published as a JSON message on `/detections`:

```json
{
    "color": "red",
    "bbox": [185, 210, 55, 42],
    "center_px": [212, 231],
    "area": 2241,
    "confidence": 1.0,
    "depth_m": 0.447
}
```

The annotated image (published on `/detections/image`) draws bounding boxes and labels directly on the camera frame. This makes it easy to verify detections visually — just subscribe to the annotated topic in rqt or save frames to disk.

---

## The Gotcha Table

| Problem | Symptom | Root Cause | Fix |
|---------|---------|------------|-----|
| Camera view rotated/sideways | Image shows walls at 45° angles | Sensor attached to optical frame; Gazebo renders along +x of attached frame | Attach to cam_link, use gz_frame_id for TF |
| Robot slides from spawn position | Ends up 1.5m away with no cmd_vel | Arm falls under gravity (JointPositionController applies zero force before first command) | Add `<dynamics damping="10.0" friction="5.0"/>` to all arm joints |
| 3cm cubes invisible at distance | No detections, cubes too small | 3cm cube = ~10 pixels at 2m distance | Increase to 5cm (PRD spec: 5–8cm) |
| Camera FOV too narrow | Nearby cubes outside frame | 60° FOV misses objects at 45° angle | Widen to 90° (1.57 rad) |
| cv_bridge import crash | "NumPy 1.x compiled module can't run in NumPy 2.x" | Venv NumPy 2.x conflicts with system cv_bridge | Don't activate venv for ROS nodes using cv_bridge |
| YOLO can't detect cubes | No "cube" class in COCO dataset | YOLO trained on COCO (80 classes, no cubes) | Use HSV color segmentation for simulation |

---

## Key Takeaways

1. **Gazebo cameras render along +x of their attached frame.** This is different from the ROS optical frame convention (z-forward). Never attach a camera sensor to a rotated optical frame — attach to the camera link and override the frame_id with `gz_frame_id`.

2. **JointPositionController applies zero force at startup.** Until you publish the first position command, the arm is in free-fall. Add damping and friction to joint dynamics to prevent this.

3. **HSV beats YOLO for simulation perception.** When you have known, consistent colors and don't need real-world robustness, color segmentation is faster to implement, deterministic, and requires no training.

4. **Object size matters more than you think.** A 3cm object at 2m distance is basically invisible in a 640×480 image. Size your simulation objects for the camera resolution you're using.

5. **Test with a rotation scan.** A 360° spin from the robot's starting position is the fastest way to verify your detection pipeline sees everything it should. Save the best frame as your artifact.

---

*Next up: Projecting pixel detections into 3D world coordinates using depth + camera intrinsics + TF2. The robot can see cubes — now it needs to know where they are in the world.*

---

**Code**: [github.com/shoaib6174/llm_robot_task_planner](https://github.com/shoaib6174/llm_robot_task_planner) (branch: `perception`)
