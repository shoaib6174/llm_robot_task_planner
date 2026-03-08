"""Simple CLI to test the LLM agent interactively.

Run alongside the simulation:
  OPENAI_API_KEY=sk-... ros2 run llm_robot_task_planner llm_agent

Then in another terminal:
  python3 tests/test_agent_cli.py

Type commands and see agent responses.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import threading


class AgentCLI(Node):
    def __init__(self):
        super().__init__('agent_cli')
        self.cmd_pub = self.create_publisher(String, '/user_command', 10)
        self.create_subscription(
            String, '/agent_response', self._response_cb, 10)

        # Wait for publisher discovery
        for i in range(30):
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.cmd_pub.get_subscription_count() > 0:
                break

    def _response_cb(self, msg: String):
        print(f'\n  [Agent] {msg.data}')

    def send(self, text: str):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)
        print(f'  [You] {text}')


def main():
    rclpy.init()
    node = AgentCLI()

    # Spin in background
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('\n=== LLM Robot Task Planner — Agent CLI ===')
    print('Type commands (or "quit" to exit):\n')

    try:
        while True:
            cmd = input('> ').strip()
            if cmd.lower() in ('quit', 'exit', 'q'):
                break
            if cmd:
                node.send(cmd)
    except (KeyboardInterrupt, EOFError):
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
