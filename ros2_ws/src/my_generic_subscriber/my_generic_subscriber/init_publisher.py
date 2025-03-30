import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class IntPublisher(Node):
    def __init__(self):
        super().__init__('int_publisher')
        self.publisher = self.create_publisher(Int32, 'numbers', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.counter = 0

    def publish_message(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.get_logger().info(f"\U0001f4e4 Sent: {msg.data}")
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = IntPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()