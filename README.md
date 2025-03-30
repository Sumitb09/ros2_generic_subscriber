# ROS 2 Generic Subscriber

A ROS 2 package that implements a generic subscriber capable of dynamically detecting and subscribing to any topic type. This package also includes example publishers for testing.

## 📌 Features
- Dynamically discovers topics and subscribes without predefined message types.
- Prints the received message along with its type.
- Includes example publishers for `std_msgs/String` and `std_msgs/Int32` topics.

## 📦 Installation
```sh
cd ~/ros2_ws/src
# Clone the repository
git clone https://github.com/Sumitb09/ros2_generic_subscriber.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select my_generic_subscriber
source install/setup.bash
```

## 🚀 Running the Subscriber
To subscribe to any topic, run:
```sh
ros2 run my_generic_subscriber generic_subscriber /chatter
```

## 📡 Example Publishers
### **String Publisher**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StringPublisher(Node):
    def __init__(self):
        super().__init__('string_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.count = 0
    
    def publish_message(self):
        msg = String()
        msg.data = f'Hello from String Publisher! {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'📤 Published: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = StringPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **Integer Publisher**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class IntPublisher(Node):
    def __init__(self):
        super().__init__('int_publisher')
        self.publisher_ = self.create_publisher(Int32, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.count = 0
    
    def publish_message(self):
        msg = Int32()
        msg.data = self.count
        self.publisher_.publish(msg)
        self.get_logger().info(f'📤 Published: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = IntPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **Generic Subscriber**
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
import sys

class GenericSubscriber(Node):
    def __init__(self, topic_name):
        super().__init__('generic_subscriber')
        self.topic_name = topic_name

        topic_list = self.get_topic_names_and_types()
        topic_type = None

        for name, types in topic_list:
            if name == topic_name:
                topic_type = types[0]
                break

        if topic_type is None:
            self.get_logger().error(f"❌ Topic '{topic_name}' not found! Ensure a publisher is running.")
            return

        msg_type = get_message(topic_type)
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            msg_type, topic_name, self.callback, qos_profile
        )
        self.get_logger().info(f"✅ Subscribed to '{topic_name}' of type [{topic_type}]")

    def callback(self, msg):
        self.get_logger().info(f"📩 Received [{type(msg).__name__}]: {msg}")

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
```

## 📸 Example Output
Below is a sample output from the `string_publisher`:

![String Publisher Output](/images/output.jpeg)

## 🛠 Debugging
If the package is not found, try:
```sh
rm -rf build/ install/ log/
colcon build --packages-select my_generic_subscriber
source install/setup.bash
ros2 run my_generic_subscriber --list
```

## 📜 License
This project is licensed under the **Apache License 2.0**.

## 👨‍💻 Author
[Your Name](https://github.com/Sumitb09)

