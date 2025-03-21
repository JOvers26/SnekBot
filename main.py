import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SnekBotPublisher(Node):
    def __init__(self):
        super().__init__('snekbot_publisher')
        
        # Create a publisher for position
        self.position_publisher_ = self.create_publisher(Float32MultiArray, 'snekbot_position', 10)
        
        # Timer to publish every second
        self.timer = self.create_timer(1.0, self.publish_message)
        self.get_logger().info("SnekBot Publisher has started!")
        
        # Initialize joint position data
        self.joint_positions = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]

    def publish_message(self):
        # Create a message for positions
        position_msg = Float32MultiArray()
        
        # Assign joint positions to the message
        position_msg.data = self.joint_positions
        
        # Publish the position data
        self.position_publisher_.publish(position_msg)
        
        self.get_logger().info(f'Publishing Position: {position_msg.data}')

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
