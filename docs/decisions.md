# Decision Log

Searchable index of all non-obvious architectural and design decisions. Each entry links back to the session worklog where it was made.

| # | Date | Session | Decision | Rationale |
|---|------|---------|----------|-----------|
| D001 | 2026-03-07 | S01 | ROS 2 Jazzy (not Humble) | Ubuntu 24.04 doesn't support Humble; Jazzy is Tier 1 for Noble |
| D002 | 2026-03-07 | S01 | Gazebo Harmonic (not Fortress) | Harmonic is Jazzy-paired Gazebo version |
| D003 | 2026-03-07 | S01 | Native install (not Docker/OS downgrade) | GUI-heavy project; Docker adds GPU+X11 complexity; OS downgrade risks GPU drivers |
| D004 | 2026-03-07 | S01 | venv with --system-site-packages | Required for rclpy access from venv; pip packages still isolated |
| D005 | 2026-03-07 | S01 | Install PyTorch cu124 before ultralytics | Prevents ultralytics from pulling CPU-only torch; cu124 backward-compat with CUDA 13.0 |
| D006 | 2026-03-07 | S01 | Skip pip pyyaml, use ROS python3-yaml | Avoids version conflict between pip and apt PyYAML |
| D007 | 2026-03-07 | S01 | Use niro-1 (100.91.14.70) instead of niro-2 | User preference; RTX 5090 + 125GB RAM |
| D008 | 2026-03-07 | S03 | Build world from scratch instead of adapting existing | No suitable two-room world found; simple box geometry is faster and more controllable |
| D009 | 2026-03-07 | S03 | 4m x 4m rooms (larger than PRD's ~75 sq ft) | More navigation space for robot; prevents tight-space issues with Nav2 |
| D010 | 2026-03-07 | S03 | SDF 1.8 format | Matches ros_gz_sim_demos reference; compatible with Gz Sim 8 |
| D011 | 2026-03-07 | S04 | Embed jetrover_description inside our package | Avoids managing two packages; COLCON_IGNORE + data_files + GZ_SIM_RESOURCE_PATH solves mesh resolution |
| D012 | 2026-03-07 | S04 | Use $(find pkg) not $(dirname) in xacro includes | $(dirname) fails when xacro is processed in launch context (no file info available) |
| D013 | 2026-03-07 | S04 | DiffDrive instead of MecanumDrive plugin | MecanumDrive requires angled friction setup; DiffDrive simpler for initial testing |
| D014 | 2026-03-07 | S04 | 3cm cubes (down from 6cm) | Gripper finger separation ~28mm; 3cm is ~75% of max opening for reliable grasping |
| D015 | 2026-03-07 | S04 | Helper launch.sh script on niro-1 | colcon source install/setup.bash doesn't work via SSH bash -c; explicit env exports as workaround |
| D016 | 2026-03-07 | S05 | Direct Nav2 node launch instead of nav2_bringup | nav2_bringup hardcodes docking_server + route_server which fail without dock plugins; direct launch gives full control |
| D017 | 2026-03-07 | S05 | Remove /tf bridge, use odom_to_tf node | Gazebo /tf (Pose_V) contains ALL model poses, conflicts with robot_state_publisher; odom_to_tf extracts only odom→base_footprint |
| D018 | 2026-03-07 | S05 | gz_frame_id for sensor frame overrides | Gazebo scopes sensor frames as model/link/sensor; gz_frame_id overrides to match URDF frame names for AMCL |
