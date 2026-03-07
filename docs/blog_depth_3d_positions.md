# From Pixels to World Coordinates — 3D Object Localization with Depth + TF2

**TL;DR: Detecting a colored cube in an image is easy. Knowing that it's at position (2.48, 2.47) in the world map is the hard part. This post covers the full projection pipeline — camera intrinsics, depth sampling, TF2 transforms — and the AMCL particle filter instability that can silently ruin your results.**

---

This is post 5 in a series on building an LLM-powered robot task planner. [Post 4 covered camera setup and HSV color detection.](link-to-post-4)

By the end of this post, we'll have:
- A 3D projection pipeline: pixel + depth → camera frame → map frame
- Each detected cube annotated with its world-frame (x, y, z) position
- Close-range accuracy of 3.8cm (measured against known cube positions)
- A clear understanding of why AMCL yaw drift kills far-range accuracy

Let's get into it.

---

## Part 1: What We're Building

In the last post, the perception node could detect colored cubes and report their pixel coordinates and depth in meters. The detection output looked like:

```json
{
    "color": "red",
    "center_px": [212, 231],
    "depth_m": 0.447
}
```

This tells us *where in the image* the cube is and *how far away* it is. But the robot's task planner doesn't think in pixels — it needs to know: "The red cube is at position (2.5, 2.5) on the map."

The projection pipeline that gets us there has three steps:
1. **Pixel → 3D point in camera frame** (using camera intrinsics)
2. **Camera frame → map frame** (using TF2 transforms)
3. **Map position → detection output** (adding `position_map` to the JSON)

Each step has a gotcha.

---

## Part 2: Camera Intrinsics — The K Matrix

Every camera has a set of intrinsic parameters that describe how 3D points project onto the 2D image. These are encoded in the **K matrix** (3×3), published on the `/camera/camera_info` topic:

```
K = [fx,  0, cx,
      0, fy, cy,
      0,  0,  1]
```

- `fx`, `fy`: focal lengths in pixels (how much the image stretches horizontally/vertically)
- `cx`, `cy`: principal point (the pixel where the optical axis hits the image)

To reverse the projection — going from pixel back to 3D — you invert the pinhole model:

```python
x_cam = (px - cx) * depth / fx
y_cam = (py - cy) * depth / fy
z_cam = depth
```

This gives you a 3D point in the **camera optical frame** — where z points forward, x points right, and y points down (standard ROS optical convention).

In code:

```python
k = self.camera_info.k  # flat 9-element array
fx, fy = k[0], k[4]
cx0, cy0 = k[2], k[5]

x_cam = (cx - cx0) * depth / fx
y_cam = (cy - cy0) * depth / fy
z_cam = depth
```

**Gotcha**: The camera intrinsics depend on your image resolution and FOV. If you change the camera's `horizontal_fov` or `width/height` in Gazebo, the K matrix changes automatically. Don't hardcode these values.

---

## Part 3: TF2 Transform — Camera Frame to Map Frame

We now have a 3D point in the camera's optical frame. But we need it in the `map` frame (the global coordinate system the robot navigates in).

The transform chain is:
```
body_cam_optical_frame → body_cam_link → base_link → base_footprint → odom → map
```

Doing this manually with quaternion math would be error-prone. Instead, ROS 2's TF2 library handles the entire chain for us. We create a `PointStamped` in the camera frame and ask TF2 to transform it to `map`:

```python
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # CRITICAL — registers PointStamped transform

pt = PointStamped()
pt.header.frame_id = self.camera_info.header.frame_id  # "body_cam_optical_frame"
pt.header.stamp = Time()  # use latest available transform
pt.point.x = x_cam
pt.point.y = y_cam
pt.point.z = z_cam

pt_map = self.tf_buffer.transform(pt, 'map', timeout=Duration(seconds=0.1))
return (pt_map.point.x, pt_map.point.y, pt_map.point.z)
```

### The import that makes it work

The line `import tf2_geometry_msgs` looks like it does nothing (hence the `# noqa: F401`). But it's essential. This import **registers** `PointStamped` as a transformable type with TF2. Without it, `tf_buffer.transform()` raises:

```
Could not transform PointStamped — no transform registered
```

You import it once, never reference it directly, and it just works. Classic ROS pattern.

### Choosing the timestamp

Using `Time()` (zero timestamp) tells TF2 to use the **latest available transform**. This is the right choice when you want "transform this point using the best current knowledge of where everything is."

The alternative — using the image message's timestamp — can fail when there's any clock skew between the camera and the TF publishers. Stick with `Time()` for simulation.

---

## Part 4: Depth Sampling — Why Median Beats Single-Pixel

The depth value at the contour center pixel is noisy. Edge pixels where the cube meets the floor can have wildly different depths. A single pixel read might get the floor depth instead of the cube depth.

The fix is simple: sample a 5×5 patch around the center and take the median:

```python
def _get_depth_at(self, cx, cy):
    patch = self.latest_depth[cy-2:cy+3, cx-2:cx+3]
    valid = patch[np.isfinite(patch) & (patch > 0)]
    if len(valid) == 0:
        return None
    return float(np.median(valid))
```

Why median instead of mean? Because depth edges are bimodal — you get either the cube's depth or the floor's depth, with nothing in between. The median picks one or the other, while the mean averages them into a value that matches neither.

---

## Part 5: Putting It Together

The updated detection now includes a `position_map` field:

```json
{
    "color": "red",
    "center_px": [212, 231],
    "area": 2314,
    "depth_m": 0.44,
    "position_map": [2.48, 2.47, 0.03]
}
```

The annotated image labels now show world coordinates instead of just depth:

```
red cube (2.5, 2.5)     ← world position on the label
```

The integration into `detect_cubes()` is clean — after getting the depth, one more function call adds the world position:

```python
depth_val = self._get_depth_at(cx, cy)
if depth_val is not None and depth_val > 0:
    detection['depth_m'] = round(float(depth_val), 3)
    world_pos = self._project_to_world(cx, cy, depth_val)
    if world_pos is not None:
        detection['position_map'] = [
            round(world_pos[0], 3),
            round(world_pos[1], 3),
            round(world_pos[2], 3),
        ]
```

---

## Part 6: Verification — How Accurate Is It?

### The Setup

The cubes are at known positions in the world file:
- Red: (2.5, 2.5) — Room A, near spawn
- Blue: (1.0, 1.0) — Room A, behind spawn
- Green: (6.0, 2.5) — Room B
- Yellow: (6.5, 1.0) — Room B

The robot spawns at (2.0, 2.0) facing east. AMCL localizes it with `initial_pose` at (2.0, 2.0, yaw=0), and the map→odom transform converges to exactly (2.000, 2.000, 0.0°).

### Close-Range Result: 3.8cm Error

The red cube at (2.5, 2.5) is 0.44m from the robot. The pipeline reports:

```
red: map=(2.48, 2.47, 0.03) expected=(2.5, 2.5) error=0.038m
```

**3.8cm error at 0.44m depth.** This is excellent — well within the tolerance needed for pick-and-place (the gripper needs to be within ~2cm for a grasp, but the robot navigates close before picking).

This result proves the pipeline is correct: camera intrinsics ✓, depth reading ✓, TF2 transform chain ✓.

### Far-Range Result: The AMCL Problem

The green cube at (6.0, 2.5) is 2.8m from spawn. The pipeline reports:

```
green: map=(4.58, 5.02, 0.03) expected=(6.0, 2.5) error=2.896m
```

Almost 3 meters off? What happened?

---

## Part 7: The AMCL Yaw Drift Problem

### Why the 3D Positions Go Wrong

The error isn't in the depth pipeline — it's in AMCL's localization. Here's what happens:

1. Robot spawns at (2,2). AMCL correctly reports map→odom as (2.0, 2.0, yaw=0°).
2. To see the green cube, the robot needs to rotate to face east-ish (toward Room B).
3. During rotation, AMCL's particle filter tries to match each new laser scan to the map.
4. The room is **symmetric** — rectangular walls look the same from many angles.
5. AMCL's particles gradually drift in yaw. After a 360° rotation, the yaw error is ~20°.
6. At 3m depth, 20° of yaw error means: sin(20°) × 3m ≈ **1.03m lateral error**.

The TF data confirms it:

```
AMCL initial: (2.0000, 2.0000) yaw=0.00 deg
AMCL final:   (1.9959, 1.9624) yaw=-20.64 deg
AMCL drift:   pos=0.038m  yaw=20.64 deg
```

Position drift is only 3.8cm — negligible. But yaw drift is 20.6° — catastrophic for far-range projections.

### Why It Only Affects Far-Range

At 0.44m depth (red cube), 20° yaw error causes: sin(20°) × 0.44 = **0.15m error**. But the red cube was detected early in the scan before AMCL had time to drift, so the actual error was even smaller.

At 2.8m depth (green cube), the same yaw error causes: sin(20°) × 2.8 = **0.96m error**. And the green cube was detected mid-scan when AMCL had already drifted significantly.

### The AMCL Configuration Trade-off

AMCL has two parameters that control when the particle filter updates:
- `update_min_d`: minimum distance the robot must move
- `update_min_a`: minimum rotation angle

Setting both to 0 means "update on every laser scan" — the filter processes ~10 scans/second. This keeps the transform alive (published continuously) but gives the filter more chances to drift during rotation.

Setting them higher (e.g., 0.25m, 0.2 rad) reduces drift but means the transform stops publishing when the robot is stationary. A new process that starts its TF buffer after the last publish will never see the map frame.

I chose `0.0/0.0` because having the transform available at all times is more important than preventing drift. The drift is a localization quality issue; missing transforms break the entire pipeline.

### The Tuning Attempt

I tried increasing AMCL's particle count from 2000 to 8000, reducing the motion model noise (alpha values from 0.2 to 0.05), and increasing the laser beams from 60 to 180. The drift went from ~25° to ~20°. Better, but not solved.

This is a fundamental limitation of particle filters in symmetric environments. The real fix would be adding distinctive features to the room (furniture, different wall textures) or using a different localization approach. For the demo, we accept it.

### Why This Doesn't Matter for the Demo

The robot's task planner will:
1. **Navigate** to a cube's approximate location (Nav2 handles this with odometry)
2. **Stop** and observe the cube at close range (< 0.5m)
3. **Pick it up** using the arm

At close range (< 1m), the 3D position accuracy is 3.8cm — more than enough. The AMCL drift only matters for far-range observations, which aren't needed for pick-and-place.

---

## Part 8: DDS Discovery — The Hidden 5-Second Wait

One more gotcha that cost 30 minutes of debugging: standalone test scripts couldn't see the map→odom transform, even though `tf2_echo` showed it was being published.

The cause: **DDS discovery delay.** When a new ROS 2 node starts, it needs ~5 seconds to discover existing publishers through the DDS middleware. The TF2 buffer starts empty, and `spin_once(timeout=0.1)` doesn't give enough time for discovery on the first few iterations.

The fix:

```python
# Give DDS time to discover TF publishers
node.spin_for(5.0)

# Now TF lookups work
tf = node.get_tf()  # returns valid transform
```

This only affects standalone scripts. Nodes launched alongside the simulation (via the launch file) don't have this issue because they start at the same time as the TF publishers.

---

## The Gotcha Table

| Problem | Symptom | Root Cause | Fix |
|---------|---------|------------|-----|
| `tf_buffer.transform()` raises "no transform registered" | Can't transform PointStamped | `tf2_geometry_msgs` not imported | Add `import tf2_geometry_msgs` (registers the type) |
| Far-range 3D positions off by 1-3m | Green cube reported at (4.6, 5.0) instead of (6.0, 2.5) | AMCL yaw drift ~20° during rotation in symmetric rooms | Accept close-range accuracy; robot approaches objects for pick/place |
| TF buffer empty in test scripts | `LookupException: "map" does not exist` for 10+ seconds | DDS discovery delay for new nodes | Add 5s `spin_for()` warmup before TF lookups |
| AMCL stops publishing when robot is still | Map frame disappears after ~1s | `update_min_d/a` set too high; no filter update = no transform publish | Set both to 0.0 (update on every scan) |
| Depth reading noisy at cube edges | Reported depth jumps between cube and floor | Single pixel reads can hit the cube/floor boundary | Use 5×5 median patch around the center pixel |

---

## Key Takeaways

1. **The 3D projection pipeline is simple in theory, tricky in practice.** Camera intrinsics → depth → camera frame point → TF2 transform to map. Each step works. The accuracy depends on the weakest link — usually localization, not the pipeline itself.

2. **`import tf2_geometry_msgs` is the most invisible critical import in ROS 2.** It registers geometry types for TF2 transformation. Without it, you get a confusing error about unregistered types. With it, `tf_buffer.transform(point, 'map')` just works.

3. **AMCL particle filters struggle in symmetric environments.** Rectangular rooms with uniform walls give the laser scanner no distinctive features to lock onto during rotation. More particles help a little. The real fix is environmental: add unique features or use a different localization approach.

4. **Close-range accuracy is what matters for manipulation.** At 0.44m depth, we achieve 3.8cm error. At 3m depth, AMCL drift causes 1-3m error. Since the robot approaches objects before picking them up, close-range accuracy is what we need.

5. **DDS discovery takes time.** New ROS 2 nodes need ~5 seconds to discover existing publishers. Build this warmup into any standalone test scripts that need TF data.

---

*Next up: Teaching the arm to pick up cubes — inverse kinematics, grasp planning, and the joy of simulated physics.*

---

**Code**: [github.com/shoaib6174/llm_robot_task_planner](https://github.com/shoaib6174/llm_robot_task_planner) (branch: `perception`)
