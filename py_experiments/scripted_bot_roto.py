import rclpy
from rclpy.node import Node

from scripted_bot_driver.single_move_client import SingleMoveClient

'''
This script defines a ScriptedBotRoto class that uses the SingleMoveClient to execute a rotation move.
It initializes the node and executes a rotation action when run.
'''

class ScriptedBotRoto:
    def __init__(self, node):
        self.node = node
        self.move_client = SingleMoveClient(node)

    def execute(self):
        self.move_client.execute_move('rotate_odom', ['1.57'])  # Example angle in radians

def main(argv=None):
    if argv is None:
        import sys
        argv = sys.argv

    rclpy.init(args=argv)
    node = Node('scripted_bot_roto')

    scripted_bot_roto = ScriptedBotRoto(node)
    scripted_bot_roto.execute()

    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    main()