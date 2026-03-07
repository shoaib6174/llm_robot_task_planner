"""Arm controller node: manages 5-DOF arm + gripper for pick/place operations.

Publishes joint position commands to Gazebo JointPositionController plugins.
Provides action-like services for pick and place sequences.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
import json


# Joint indices in our command array: [j1, j2, j3, j4, j5, gripper]
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']
JOINT_TOPICS = [
    'arm/joint1/cmd_pos',
    'arm/joint2/cmd_pos',
    'arm/joint3/cmd_pos',
    'arm/joint4/cmd_pos',
    'arm/joint5/cmd_pos',
    'arm/gripper/cmd_pos',
]

# Gripper values
GRIPPER_OPEN = 0.5     # radians — fingers spread apart
GRIPPER_CLOSE = -0.2   # radians — fingers closed (gripping)

# Predefined poses: [joint1, joint2, joint3, joint4, joint5]
# joint1: base rotation (Z axis), negative = left, positive = right
# joint2: shoulder pitch (Y axis), negative = forward tilt
# joint3: elbow pitch (Y axis), negative = forward bend
# joint4: wrist pitch (Y axis), adjusts gripper angle
# joint5: wrist rotation (Z axis)
POSES = {
    # Arm straight up, out of the way
    'home': [0.0, 0.0, 0.0, 0.0, 0.0],

    # Arm tucked back, low profile for navigation
    'tuck': [0.0, 0.3, 1.0, 0.8, 0.0],

    # Reach forward and slightly down — hover above a cube in front
    'pre_grasp': [0.0, -1.0, -0.4, -0.6, 0.0],

    # Lower to grasp position — gripper at cube height on floor
    'grasp': [0.0, -1.2, -0.3, -0.5, 0.0],

    # Lift cube up after grasping
    'lift': [0.0, -0.3, -0.2, -0.5, 0.0],

    # Place position — arm forward and down to release
    'place': [0.0, -1.0, -0.4, -0.6, 0.0],
}

# Time to wait after sending a pose command for the PID to settle (seconds)
SETTLE_TIME = 2.0


class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Publishers for each joint
        self.joint_pubs = []
        for topic in JOINT_TOPICS:
            pub = self.create_publisher(Float64, topic, 10)
            self.joint_pubs.append(pub)

        # Subscribe to joint states for feedback
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.current_joints = {name: 0.0 for name in JOINT_NAMES}

        # Command subscriber — accepts JSON commands
        self.create_subscription(String, '/arm/command', self.command_cb, 10)

        # Status publisher
        self.status_pub = self.create_publisher(String, '/arm/status', 10)

        # State
        self.target_pose = POSES['home'][:]
        self.gripper_target = GRIPPER_OPEN
        self.busy = False

        # Send initial home pose after a short delay (must be fast to prevent arm falling)
        self.create_timer(0.5, self.init_pose, callback_group=None)
        self.init_done = False

        self.get_logger().info('Arm controller started')

    def init_pose(self):
        """Send home pose once at startup."""
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
          {"action": "pose", "pose": "pre_grasp"}
          {"action": "gripper", "state": "open"}
          {"action": "gripper", "state": "close"}
          {"action": "pick"}
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
            self.execute_pick()

        elif action == 'place':
            self.execute_place()

        else:
            self.get_logger().warn(f'Unknown action: {action}')

    def send_pose(self, joint_values, gripper_value):
        """Send position commands to all joints immediately."""
        for i, val in enumerate(joint_values):
            msg = Float64()
            msg.data = float(val)
            self.joint_pubs[i].publish(msg)
        self.target_pose = joint_values[:]
        self.send_gripper(gripper_value)

    def send_gripper(self, value):
        """Send gripper position command."""
        msg = Float64()
        msg.data = float(value)
        self.joint_pubs[5].publish(msg)
        self.gripper_target = value

    def move_to(self, target_joints, gripper_value, settle=SETTLE_TIME):
        """Send target pose and wait for PID to settle."""
        self.send_pose(target_joints, gripper_value)
        time.sleep(settle)

    def execute_pose(self, pose_name):
        """Move to a named pose."""
        self.busy = True
        self.publish_status('moving', f'Moving to {pose_name}')
        self.move_to(POSES[pose_name], self.gripper_target)
        self.publish_status('done', f'Reached {pose_name}')
        self.busy = False

    def execute_pick(self):
        """Execute a full pick sequence."""
        self.busy = True
        self.publish_status('picking', 'Starting pick sequence')

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

        # 4. Close gripper
        self.get_logger().info('Pick: closing gripper')
        self.send_gripper(GRIPPER_CLOSE)
        time.sleep(1.5)

        # 5. Lift
        self.get_logger().info('Pick: lifting')
        self.move_to(POSES['lift'], GRIPPER_CLOSE)

        self.publish_status('done', 'Pick complete')
        self.busy = False

    def execute_place(self):
        """Execute a full place sequence."""
        self.busy = True
        self.publish_status('placing', 'Starting place sequence')

        # 1. Move to place position
        self.get_logger().info('Place: moving to place position')
        self.move_to(POSES['place'], GRIPPER_CLOSE)

        # 2. Open gripper
        self.get_logger().info('Place: opening gripper')
        self.send_gripper(GRIPPER_OPEN)
        time.sleep(1.5)

        # 3. Return to home
        self.get_logger().info('Place: returning home')
        self.move_to(POSES['home'], GRIPPER_OPEN)

        self.publish_status('done', 'Place complete')
        self.busy = False

    def publish_status(self, state, message):
        msg = String()
        msg.data = json.dumps({'state': state, 'message': message})
        self.status_pub.publish(msg)
        self.get_logger().info(f'[{state}] {message}')


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
