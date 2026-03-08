"""Arm controller node: manages 5-DOF arm + gripper for pick/place operations.

Publishes joint position commands to Gazebo JointPositionController plugins.
Provides action-like services for pick and place sequences.

Grasping uses a simulated attach/detach approach: DART physics doesn't
support the 4-bar linkage mimic constraints, so after the gripper closes
the arm controller teleports the grasped object to follow the end effector
via Gazebo's set_pose service. This is a standard approach in Gazebo demos.
"""

import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
import tf2_ros
import json


# Joint indices in our command array: [j1, j2, j3, j4, j5]
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']
JOINT_TOPICS = [
    'arm/joint1/cmd_pos',
    'arm/joint2/cmd_pos',
    'arm/joint3/cmd_pos',
    'arm/joint4/cmd_pos',
    'arm/joint5/cmd_pos',
]

# Two-finger gripper: r_joint and l_joint (mimic with multiplier=-1).
# DART doesn't support mimic constraints so we actuate both directly.
GRIPPER_R_TOPIC = 'arm/gripper/r_cmd'
GRIPPER_L_TOPIC = 'arm/gripper/l_cmd'

# Gripper values
GRIPPER_OPEN = 1.0     # radians — fingers spread apart
GRIPPER_CLOSE = -0.2   # radians — fingers closed (visual only)

# Predefined poses: [joint1, joint2, joint3, joint4, joint5]
POSES = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0],
    'tuck': [0.0, 0.3, 1.0, 0.8, 0.0],
    'pre_grasp': [0.0, 1.5, 0.7, 0.3, 0.0],
    'grasp': [0.0, 1.7, 0.6, 0.1, 0.0],
    'lift': [0.0, 0.5, 0.3, 0.3, 0.0],
    'place': [0.0, 1.5, 0.7, 0.3, 0.0],
}

SETTLE_TIME = 3.0

# Gazebo world name (for set_pose service)
GZ_WORLD = 'two_room_world'


class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Publishers for arm joints
        self.joint_pubs = []
        for topic in JOINT_TOPICS:
            pub = self.create_publisher(Float64, topic, 10)
            self.joint_pubs.append(pub)

        # Publishers for gripper (two-finger: r_joint + l_joint)
        self.gripper_r_pub = self.create_publisher(Float64, GRIPPER_R_TOPIC, 10)
        self.gripper_l_pub = self.create_publisher(Float64, GRIPPER_L_TOPIC, 10)

        # Subscribe to joint states for feedback
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.current_joints = {name: 0.0 for name in JOINT_NAMES}

        # TF2 for looking up end-effector position in odom frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Command subscriber — uses reentrant callback group so TF and
        # joint_state callbacks can run during blocking pick/place sequences
        self.cmd_cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            String, '/arm/command', self.command_cb, 10,
            callback_group=self.cmd_cb_group)

        # Status publisher
        self.status_pub = self.create_publisher(String, '/arm/status', 10)

        # State
        self.target_pose = POSES['home'][:]
        self.gripper_target = GRIPPER_OPEN
        self.busy = False

        # Simulated grasp state
        self.grasped_object = None  # Name of Gazebo model being held

        # Send initial home pose after a short delay
        self.create_timer(0.5, self.init_pose, callback_group=None)
        self.init_done = False

        self.get_logger().info('Arm controller started')

    def init_pose(self):
        if not self.init_done:
            self.init_done = True
            self.get_logger().info('Sending initial home pose')
            self.send_pose(POSES['home'], GRIPPER_OPEN)

    def joint_state_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.current_joints:
                self.current_joints[name] = msg.position[i]

    def command_cb(self, msg: String):
        """Handle arm commands.

        JSON format:
          {"action": "pose", "pose": "home"}
          {"action": "gripper", "state": "open"}
          {"action": "gripper", "state": "close"}
          {"action": "pick", "object": "red_cube"}
          {"action": "place"}
          {"action": "joints", "values": [j1, j2, j3, j4, j5]}
        """
        if self.busy:
            self.publish_status('busy', 'Arm is busy executing a command')
            return

        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid JSON command: {msg.data}')
            return

        action = cmd.get('action', '')
        self.get_logger().info(f'Command received: {action}')

        if action == 'pose':
            pose_name = cmd.get('pose', '')
            if pose_name in POSES:
                self.execute_pose(pose_name)
            else:
                self.get_logger().warn(f'Unknown pose: {pose_name}')

        elif action == 'gripper':
            state = cmd.get('state', 'open')
            if state == 'open':
                self.send_gripper(GRIPPER_OPEN)
                if self.grasped_object:
                    self.grasped_object = None
                self.publish_status('done', 'Gripper opened')
            elif state == 'close':
                self.send_gripper(GRIPPER_CLOSE)
                self.publish_status('done', 'Gripper closed')

        elif action == 'joints':
            values = cmd.get('values', [])
            if len(values) == 5:
                self.send_pose(values, self.gripper_target)
                self.publish_status('done', f'Joints set to {values}')
            else:
                self.get_logger().warn('joints action requires 5 values')

        elif action == 'pick':
            obj_name = cmd.get('object', 'red_cube')
            self.execute_pick(obj_name)

        elif action == 'place':
            self.execute_place()

        else:
            self.get_logger().warn(f'Unknown action: {action}')

    def send_pose(self, joint_values, gripper_value):
        """Send position commands to all joints."""
        for i in range(2):
            msg = Float64()
            msg.data = float(joint_values[i])
            self.joint_pubs[i].publish(msg)
            time.sleep(0.02)

        time.sleep(1.5)

        for i in range(2, 5):
            msg = Float64()
            msg.data = float(joint_values[i])
            self.joint_pubs[i].publish(msg)
            time.sleep(0.02)

        self.target_pose = joint_values[:]
        self.send_gripper(gripper_value)

    def send_gripper(self, value):
        """Send gripper position command to both finger joints."""
        msg_r = Float64()
        msg_r.data = float(value)
        self.gripper_r_pub.publish(msg_r)

        msg_l = Float64()
        msg_l.data = float(-value)  # mimic: multiplier=-1
        self.gripper_l_pub.publish(msg_l)

        self.gripper_target = value

    def move_to(self, target_joints, gripper_value, settle=SETTLE_TIME):
        """Send target pose and wait for PID to settle.

        If holding an object, teleports it to the EE position every 0.5s
        during the settle wait to prevent it from falling under gravity.
        """
        self.send_pose(target_joints, gripper_value)
        if self.grasped_object:
            # Teleport repeatedly during settle to keep object at EE
            elapsed = 0.0
            while elapsed < settle:
                time.sleep(0.5)
                elapsed += 0.5
                self.teleport_grasped_to_ee()
        else:
            time.sleep(settle)

    def execute_pose(self, pose_name):
        self.busy = True
        self.publish_status('moving', f'Moving to {pose_name}')
        self.move_to(POSES[pose_name], self.gripper_target)
        self.publish_status('done', f'Reached {pose_name}')
        self.busy = False

    def execute_pick(self, obj_name):
        """Execute a full pick sequence with simulated grasp."""
        self.busy = True
        self.publish_status('picking', f'Starting pick of {obj_name}')

        # 1. Open gripper
        self.get_logger().info('Pick: opening gripper')
        self.send_gripper(GRIPPER_OPEN)
        time.sleep(0.5)

        # 2. Move to pre-grasp
        self.get_logger().info('Pick: moving to pre_grasp')
        self.move_to(POSES['pre_grasp'], GRIPPER_OPEN)

        # 3. Lower to grasp position
        self.get_logger().info('Pick: lowering to grasp')
        self.move_to(POSES['grasp'], GRIPPER_OPEN)

        # 4. Close gripper (visual)
        self.get_logger().info('Pick: closing gripper')
        self.send_gripper(GRIPPER_CLOSE)
        time.sleep(2.0)

        # 5. Attach object (simulated grasp)
        self.grasped_object = obj_name
        self.get_logger().info(f'Pick: attached {obj_name} (simulated grasp)')

        # 6. Lift — teleport object to follow EE
        self.get_logger().info('Pick: lifting')
        self.move_to(POSES['lift'], GRIPPER_CLOSE)

        self.publish_status('done', f'Pick complete — holding {obj_name}')
        self.busy = False

    def execute_place(self):
        """Execute a full place sequence."""
        self.busy = True
        obj_name = self.grasped_object or 'unknown'
        self.publish_status('placing', f'Starting place of {obj_name}')

        # 1. Move to place position (object follows via teleport)
        self.get_logger().info('Place: moving to place position')
        self.move_to(POSES['place'], GRIPPER_CLOSE)

        # 2. Lower to grasp height for drop
        self.get_logger().info('Place: lowering to drop height')
        self.move_to(POSES['grasp'], GRIPPER_CLOSE)

        # 3. Open gripper and release object
        self.get_logger().info('Place: opening gripper')
        self.send_gripper(GRIPPER_OPEN)
        if self.grasped_object:
            # Place cube on ground (z=0.025 for 5cm cube center)
            pos = self.get_ee_world_position()
            if pos:
                self.gz_set_model_pose(self.grasped_object, pos[0], pos[1], 0.025)
                self.get_logger().info(
                    f'Placed {self.grasped_object} at ({pos[0]:.3f}, {pos[1]:.3f}, 0.025)')
            self.grasped_object = None
        time.sleep(1.5)

        # 4. Return to home
        self.get_logger().info('Place: returning home')
        self.move_to(POSES['home'], GRIPPER_OPEN)

        self.publish_status('done', 'Place complete')
        self.busy = False

    def get_ee_world_position(self, z_offset=0.0):
        """Get end-effector position in Gazebo world frame.

        Combines the TF lookup (base_footprint→EE) with the robot's
        actual Gazebo world pose (from gz model command). The /odom
        topic reports (0,0,0) at startup regardless of spawn position,
        so we query Gazebo directly.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                'base_footprint', 'end_effector_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            dx = t.transform.translation.x
            dy = t.transform.translation.y
            dz = t.transform.translation.z
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

        # Get robot's actual world pose from Gazebo
        robot_pose = self._gz_get_model_pose('jetrover')
        if robot_pose is None:
            self.get_logger().warn('Failed to get robot pose from Gazebo')
            return None

        rx, ry, rz, robot_yaw = robot_pose

        # Transform EE offset from base_footprint to world using robot yaw
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        world_x = rx + dx * cos_yaw - dy * sin_yaw
        world_y = ry + dx * sin_yaw + dy * cos_yaw
        world_z = rz + dz + z_offset

        return (world_x, world_y, world_z)

    def _gz_get_model_pose(self, model_name):
        """Get model pose from Gazebo. Returns (x, y, z, yaw) or None."""
        try:
            result = subprocess.run(
                ['bash', '-c',
                 f'source /opt/ros/jazzy/setup.bash && DISPLAY=:1 '
                 f'gz model -m {model_name} --pose 2>/dev/null'],
                capture_output=True, text=True, timeout=10)
        except Exception:
            return None

        lines = result.stdout.strip().split('\n')
        xyz = None
        rpy = None
        for i, line in enumerate(lines):
            if 'XYZ' in line and i + 1 < len(lines):
                vals = lines[i + 1].strip().strip('[]').split()
                if len(vals) == 3:
                    try:
                        xyz = [float(v) for v in vals]
                    except ValueError:
                        pass
            if 'RPY' in line and i + 1 < len(lines):
                vals = lines[i + 1].strip().strip('[]').split()
                if len(vals) == 3:
                    try:
                        rpy = [float(v) for v in vals]
                    except ValueError:
                        pass

        if xyz is None:
            return None
        yaw = rpy[2] if rpy else 0.0
        return (xyz[0], xyz[1], xyz[2], yaw)

    def teleport_grasped_to_ee(self, z_offset=0.0):
        """Teleport the grasped object to the current end-effector position."""
        if not self.grasped_object:
            return

        pos = self.get_ee_world_position(z_offset)
        if pos is None:
            return

        x, y, z = pos
        self.gz_set_model_pose(self.grasped_object, x, y, z)
        self.get_logger().info(
            f'Teleported {self.grasped_object} to ({x:.3f}, {y:.3f}, {z:.3f})')

    def gz_set_model_pose(self, model_name, x, y, z, qx=0, qy=0, qz=0, qw=1):
        """Set a Gazebo model's pose via gz service."""
        req = (
            f'name: "{model_name}", '
            f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
            f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        )
        cmd = (
            f'gz service -s /world/{GZ_WORLD}/set_pose '
            f'--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 '
            f"--req '{req}'"
        )
        try:
            subprocess.run(
                ['bash', '-c', f'source /opt/ros/jazzy/setup.bash && DISPLAY=:1 {cmd}'],
                capture_output=True, timeout=5)
        except Exception as e:
            self.get_logger().warn(f'gz set_pose failed for {model_name}: {e}')

    def publish_status(self, state, message):
        msg = String()
        msg.data = json.dumps({'state': state, 'message': message})
        self.status_pub.publish(msg)
        self.get_logger().info(f'[{state}] {message}')


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
