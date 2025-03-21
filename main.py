import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SnekBotPublisher(Node):
    def __init__(self):
        super().__init__('snekbot_publisher')
        self.publisher_ = self.create_publisher(String, 'snekbot_chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # Publish every second
        self.get_logger().info("SnekBot Publisher has started!")
        self.counter = 0  # Initialize a counter for the incrementing number

    def publish_message(self):
        msg = String()
        msg.data = f"Message {self.counter}"  # Incrementing message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1  # Increment the counter each time a message is published

def main(args=None):
    rclpy.init(args=args)
    node = SnekBotPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
