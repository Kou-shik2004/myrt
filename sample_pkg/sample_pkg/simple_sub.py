import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__("simple_rpi_subscriber")
        self.sub_ = self.create_subscription(String, "talk", self.msgCallback, 10)

    def msgCallback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)


def main():
    rclpy.init()

    simple_publisher = SimpleSubscriber()
    
    rclpy.spin(simple_publisher)
    
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()