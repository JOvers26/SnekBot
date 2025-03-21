import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SnekBotPublisher(Node):
    def __init__(self):
        super().__init__('snekbot_publisher')
        
        # Create a publisher for position
        self.position_publisher_ = self.create_publisher(Float32MultiArray, 'snekbot_position', 10)

        # Create a subscriber for receiving position updates
        self.position_subscription = self.create_subscription(
            Float32MultiArray, 
            'position_updates',  # Topic to subscribe to
            self.position_callback, 
            10
        )
        
        self.get_logger().info("SnekBot Publisher is running and waiting for position updates.")
        
    def position_callback(self, msg):
        """Callback to handle incoming position updates."""
        # The received position data is in msg.data
        self.get_logger().info(f'Received Position Update: {msg.data}')
        
        # Publish the received position data to the snekbot_position topic
        self.position_publisher_.publish(msg)
        self.get_logger().info(f'Publishing Position: {msg.data}')

def run_publisher():
    """Starts the publisher and keeps it running."""
    rclpy.init()
    node = SnekBotPublisher()

    # Keep the program running, subscribing to the position updates topic
    rclpy.spin(node)

    # Shutdown ROS 2 when done
    rclpy.shutdown()
