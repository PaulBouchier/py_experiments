import rclpy
from rclpy.node import Node

from scripted_bot_driver.single_move_client import SingleMoveClient

'''
This script uses the SingleMoveClient to execute a rotation move.
It initializes the node, creates the client, and executes a rotation action when run.
'''

def main(argv=None):
    rclpy.init()
    node = Node('scripted_bot_roto')

    move_client = SingleMoveClient(node)
    move_client.execute_move('rotate_odom', ['1.57'])  # Example angle in radians
    move_client.execute_move('drive_straight_odom', ['0.2'])  # Drive forward, example distance in meters
    move_client.execute_move('drive_straight_odom', ['-0.1'])  # Drive backward, example distance in meters
    move_client.execute_move('seek2can', [])  # Seek to can by tracking with lidar
    rclpy.spin_once(node, timeout_sec=1.0)  # Allow time for the action to complete

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    main()