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
