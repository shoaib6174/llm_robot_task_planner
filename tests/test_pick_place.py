"""Test pick/place sequence with simulated grasp — teleport approach.

Verifies DC5: arm picks up red_cube, lifts it (z delta > 0.1m), places it back.
Run on niro-1 with simulation running.
"""
import time
import json
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def gz_get_pose(model_name):
    """Get model XYZ pose from Gazebo."""
    result = subprocess.run(
        ["bash", "-c",
         f"source /opt/ros/jazzy/setup.bash && DISPLAY=:1 "
         f"gz model -m {model_name} --pose 2>/dev/null"],
        capture_output=True, text=True, timeout=10
    )
    lines = result.stdout.strip().split('\n')
    for i, line in enumerate(lines):
        if 'XYZ' in line and i + 1 < len(lines):
            vals = lines[i + 1].strip().strip('[]').split()
            if len(vals) == 3:
                try:
                    return [float(v) for v in vals]
                except ValueError:
                    pass
    return None


def gz_set_pose(model_name, x, y, z, qx=0, qy=0, qz=0, qw=1):
    """Set model pose in Gazebo."""
    req = (
        f'name: "{model_name}", '
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
    )
    result = subprocess.run(
        ["bash", "-c",
         f"source /opt/ros/jazzy/setup.bash && DISPLAY=:1 "
         f"gz service -s /world/two_room_world/set_pose "
         f"--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 "
         f"--req '{req}'"],
        capture_output=True, text=True, timeout=10
    )
    return 'true' in result.stdout.lower()


class PickTester(Node):
    def __init__(self):
        super().__init__("pick_tester")
        self.cmd_pub = self.create_publisher(String, "/arm/command", 10)
        self.all_statuses = []
        self.create_subscription(String, "/arm/status", self._status_cb, 10)

        for i in range(60):
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.cmd_pub.get_subscription_count() > 0:
                break

    def _status_cb(self, msg):
        status = json.loads(msg.data)
        self.all_statuses.append(status)

    def send_cmd(self, cmd):
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Sent: {cmd}")

    def wait_for_status(self, state, message_contains=None, timeout=60):
        """Wait for a specific status message."""
        self.all_statuses.clear()
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            for s in self.all_statuses:
                if s.get('state') == state:
                    if message_contains is None or message_contains in s.get('message', ''):
                        return True
        return False


def main():
    rclpy.init()
    node = PickTester()

    print("\n=== SETUP ===")
    gz_set_pose("red_cube", 2.34, 2.0, 0.025)
    time.sleep(2)

    robot_pose = gz_get_pose("jetrover")
    cube_before = gz_get_pose("red_cube")
    print(f"  Robot: {robot_pose}")
    print(f"  Cube:  {cube_before}")

    # Home
    print("\n=== HOME ===")
    node.send_cmd({"action": "pose", "pose": "home"})
    done = node.wait_for_status('done', 'home', timeout=15)
    print(f"  Home done: {done}")
    time.sleep(1)

    # Pick
    print("\n=== PICK red_cube ===")
    node.send_cmd({"action": "pick", "object": "red_cube"})
    done = node.wait_for_status('done', 'Pick complete', timeout=60)
    print(f"  Pick done: {done}")

    time.sleep(1)
    cube_after_pick = gz_get_pose("red_cube")
    print(f"  Cube after pick: {cube_after_pick}")

    if cube_before and cube_after_pick:
        dz = cube_after_pick[2] - cube_before[2]
        print(f"  Z delta: {dz:.4f}m")
        if dz > 0.1:
            print("  *** SUCCESS: Cube was lifted significantly! ***")
        elif dz > 0.03:
            print("  ** PARTIAL: Cube lifted slightly **")
        else:
            print("  FAIL: Cube was not lifted")

    # Place
    print("\n=== PLACE ===")
    node.send_cmd({"action": "place"})
    done = node.wait_for_status('done', 'Place complete', timeout=60)
    print(f"  Place done: {done}")

    time.sleep(1)
    cube_final = gz_get_pose("red_cube")
    print(f"  Cube after place: {cube_final}")

    # Print all statuses for debugging
    print("\n=== Status log ===")
    for s in node.all_statuses:
        print(f"  {s}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
