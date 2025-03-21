import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SnekBotPublisher(Node):
    def __init__(self):
        super().__init__('snekbot_publisher')
        self.publisher_ = self.create_publisher(String, 'snekbot_chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # Publish every second
        self.get_logger().info("SnekBot Publisher has started!")

    def publish_message(self):
        msg = String()
        msg.data = "Hello from SnekBot!"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

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
