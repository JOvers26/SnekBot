import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SnekBotPublisher(Node):
    def __init__(self):
        super().__init__('snekbot_publisher')
        self.publisher = self.create_publisher(String, 'position', 10)  # Topic name: 'position'
        self.get_logger().info("SnekBotPublisher initialized.")
    
    def send_position(self, position_str):
        # Create a message of type String
        msg = String()
        msg.data = position_str
        
        # Publish the message to the 'position' topic
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent position: {position_str}")
    
    def spin(self):
        # Keep the node alive and processing events
        rclpy.spin(self)
    
    def shutdown(self):
        # Shutdown the node
        rclpy.shutdown()
