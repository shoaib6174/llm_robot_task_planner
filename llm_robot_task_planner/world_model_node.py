"""World model node: semantic map + robot status tracking.

Loads world_map.yaml, tracks which room the robot is in, what object
it's holding, and known object locations. Provides services for the
LLM agent to query world state.
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
import yaml


class WorldModelNode(Node):
    def __init__(self):
        super().__init__('world_model')

        # Load semantic map
        pkg_share = get_package_share_directory('llm_robot_task_planner')
        map_path = f'{pkg_share}/config/world_map.yaml'
        self.declare_parameter('world_map_path', map_path)
        map_file = self.get_parameter('world_map_path').value

        with open(map_file, 'r') as f:
            self.world_map = yaml.safe_load(f)

        self.locations = self.world_map.get('locations', {})
        self.objects = self.world_map.get('objects', {})

        # Robot state
        self.robot_x = 2.0
        self.robot_y = 2.0
        self.robot_yaw = 0.0
        self.held_object = None

        # Known object positions (updated by perception)
        self.known_objects = {}
        for name, info in self.objects.items():
            pos = info.get('initial_position', [0, 0, 0])
            self.known_objects[name] = {
                'color': info.get('color', ''),
                'position': pos[:2],
                'last_seen_room': info.get('expected_location', ''),
                'confirmed': False,
            }

        # Subscribe to odometry for robot position
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Subscribe to arm status to track held object
        self.create_subscription(String, '/arm/status', self._arm_status_cb, 10)

        # Subscribe to detections to update known positions
        self.create_subscription(String, '/detections', self._detections_cb, 10)

        # Service-like interface via pub/sub (simpler than ROS 2 services)
        self.create_subscription(
            String, '/world_model/query', self._query_cb, 10)
        self.response_pub = self.create_publisher(
            String, '/world_model/response', 10)

        # Publish status periodically
        self.status_pub = self.create_publisher(String, '/world_model/status', 10)
        self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            f'World model started: {len(self.locations)} locations, '
            f'{len(self.objects)} objects')

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def _arm_status_cb(self, msg: String):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        message = status.get('message', '')
        if 'Pick complete' in message:
            # Extract object name: "Pick complete — holding red_cube"
            parts = message.split('holding ')
            if len(parts) > 1:
                self.held_object = parts[1].strip()
                self.get_logger().info(f'Holding: {self.held_object}')
        elif 'Place complete' in message:
            self.held_object = None
            self.get_logger().info('No longer holding any object')

    def _detections_cb(self, msg: String):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(detections, list):
            return
        for det in detections:
            color = det.get('color', '')
            pos_map = det.get('position_map')
            if color and pos_map:
                obj_name = f'{color}_cube'
                room = self._position_to_room(pos_map[0], pos_map[1])
                self.known_objects[obj_name] = {
                    'color': color,
                    'position': [pos_map[0], pos_map[1]],
                    'last_seen_room': room or 'unknown',
                    'confirmed': True,
                }

    def _position_to_room(self, x, y):
        """Determine which room a position is in."""
        for name, loc in self.locations.items():
            bounds = loc.get('bounds')
            if bounds:
                if (bounds['x_min'] <= x <= bounds['x_max'] and
                        bounds['y_min'] <= y <= bounds['y_max']):
                    return name
        return None

    def get_current_room(self):
        """Determine which room the robot is currently in."""
        # Odom starts at (0,0), real world position is offset by spawn
        # For now use odom + spawn offset (robot spawns at 2,2)
        world_x = self.robot_x + 2.0  # approximate — odom starts at 0
        world_y = self.robot_y + 2.0
        room = self._position_to_room(world_x, world_y)
        return room or 'unknown'

    def get_status(self):
        """Get full robot status as dict."""
        current_room = self.get_current_room()
        return {
            'current_room': current_room,
            'held_object': self.held_object,
            'known_objects': self.known_objects,
            'robot_position': {
                'odom_x': round(self.robot_x, 2),
                'odom_y': round(self.robot_y, 2),
                'yaw_rad': round(self.robot_yaw, 2),
            },
        }

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps(self.get_status())
        self.status_pub.publish(msg)

    def _query_cb(self, msg: String):
        """Handle queries from the LLM agent."""
        try:
            query = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        query_type = query.get('type', '')
        response = {}

        if query_type == 'status':
            response = self.get_status()

        elif query_type == 'location':
            location_name = query.get('name', '')
            if location_name in self.locations:
                loc = self.locations[location_name]
                response = {
                    'name': location_name,
                    'coordinates': loc.get('coordinates', [0, 0, 0]),
                    'description': loc.get('description', ''),
                }
            else:
                response = {'error': f'Unknown location: {location_name}'}

        elif query_type == 'object':
            obj_name = query.get('name', '')
            if obj_name in self.known_objects:
                response = {
                    'name': obj_name,
                    **self.known_objects[obj_name],
                }
            else:
                response = {'error': f'Unknown object: {obj_name}'}

        elif query_type == 'all_locations':
            response = {
                name: {
                    'coordinates': loc.get('coordinates', [0, 0, 0]),
                    'description': loc.get('description', ''),
                }
                for name, loc in self.locations.items()
            }

        else:
            response = {'error': f'Unknown query type: {query_type}'}

        reply = String()
        reply.data = json.dumps(response)
        self.response_pub.publish(reply)

    def update_object_position(self, obj_name, x, y, room=None):
        """Programmatic update from LLM agent tools."""
        if room is None:
            room = self._position_to_room(x, y) or 'unknown'
        if obj_name in self.known_objects:
            self.known_objects[obj_name]['position'] = [x, y]
            self.known_objects[obj_name]['last_seen_room'] = room
            self.known_objects[obj_name]['confirmed'] = True


def main(args=None):
    rclpy.init(args=args)
    node = WorldModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
