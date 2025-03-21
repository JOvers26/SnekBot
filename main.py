import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SnekBotPublisher(Node):
    def __init__(self):
        super().__init__('snekbot_publisher')
        
        # Create publishers for speed and position
        self.speed_publisher_ = self.create_publisher(Float32MultiArray, 'snekbot_speed', 10)
        self.position_publisher_ = self.create_publisher(Float32MultiArray, 'snekbot_position', 10)
        
        # Timer to publish every second
        self.timer = self.create_timer(1.0, self.publish_message)  
        self.get_logger().info("SnekBot Publisher has started!")
        
        # Initialize joint data
        self.counter = 0  # This can be used for some kind of data change or increment
        self.joint_speeds = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1]  # Example speeds for each joint
        self.joint_positions = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]  # Example positions for each joint

    def publish_message(self):
        # Create message for speeds
        speed_msg = Float32MultiArray()
        speed_msg.data = self.joint_speeds  # Assign speed data to the message
        
        # Create message for positions
        position_msg = Float32MultiArray()
        position_msg.data = self.joint_positions  # Assign position data to the message
        
        # Publish both messages
        self.speed_publisher_.publish(speed_msg)
        self.position_publisher_.publish(position_msg)
        
        self.get_logger().info(f'Publishing Speed: {speed_msg.data}')
        self.get_logger().info(f'Publishing Position: {position_msg.data}')

        self.counter += 1  # Increment the counter (optional, for future use)

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
