import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
import sys

class GenericSubscriber(Node):
    def __init__(self, topic_name):
        super().__init__('generic_subscriber')
        self.topic_name = topic_name

        # Discover available topics
        topic_list = self.get_topic_names_and_types()
        topic_type = None

        # Find the message type for the topic
        for name, types in topic_list:
            if name == topic_name:
                topic_type = types[0]  # Select the first available type
                break

        if topic_type is None:
            self.get_logger().error(f"\u274c Topic '{topic_name}' not found! Ensure a publisher is running.")
            return

        # Get the actual message type
        msg_type = get_message(topic_type)
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            msg_type, topic_name, self.callback, qos_profile
        )
        self.get_logger().info(f"\u2705 Subscribed to '{topic_name}' of type [{topic_type}]")

    def callback(self, msg):
        self.get_logger().info(f"\U0001f4e9 Received [{type(msg).__name__}]: {msg}")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run my_generic_subscriber generic_subscriber <topic_name>")
        return

    topic_name = sys.argv[1]
    node = GenericSubscriber(topic_name)
    if node.subscription is None:
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
