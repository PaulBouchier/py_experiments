import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("my_node")

        # Create a timer that fires every quarter second
        timer_period = 0.25
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        self.get_logger().info("timer has fired")

def main():
    rclpy.init()
    my_node = MyNode()
    rclpy.spin(my_node)
    my_node.destroy_node()  # cleans up pub-subs, etc
    rclpy.shutdown()

if __name__ == '__main__':
    main()

