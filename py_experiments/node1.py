import sys
from time import sleep
from math import pow, sqrt
from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from nav_msgs.msg import Odometry

'''
Test how to start a subscriber and wait for it to receive messages.
Requires a /odom publisher to be running, e.g. generic_turtlesim
'''
class Node1(Node):
    def __init__(self):
        super().__init__('node1')
        self.create_timer(5, self.doit, callback_group=MutuallyExclusiveCallbackGroup())
        self.doit_count = 0
        self.odom_msg_count = 0
        self.rate = self.create_rate(1)

    def doit(self):
        self.doit_count += 1
        self.get_logger().info('Iteration {} of doit()'.format(self.doit_count))
        if (self.doit_count == 2):
            self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())
            while (rclpy.ok() and self.odom_msg_count < 130):
                self.get_logger().info('odom_msg_count in rate-wait: {}'.format(self.odom_msg_count))
                self.rate.sleep()
                
        if (self.doit_count == 3):
            self.get_logger().info('slept waiting for odom_started, odom_msg_count: {}'.format(self.odom_msg_count))
        if (self.doit_count == 4):
            self.destroy_subscription(self.odom_sub)
            self.get_logger().info('destroyed odom subscription, odom_msg_count: {}'.format(self.odom_msg_count))


    def odom_callback(self, odom_msg):
        self.odom = odom_msg
        self.odom_msg_count += 1
        self.get_logger().info('in odom_callback, msg_count: {}'.format(self.odom_msg_count))

    def is_odom_started(self):
        return self.odom_msg_count > 2

def main():
    print('Starting py_experiments, node1')
    rclpy.init()

    nh = Node1()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(nh, executor=executor)

    nh.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
