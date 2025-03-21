import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SnekBotPublisher(Node):
    def __init__(self):
        super().__init__('snekbot_publisher')
        
        # Create a publisher for position
        self.position_publisher_ = self.create_publisher(Float32MultiArray, 'snekbot_position', 10)
        
        # Initialize joint position data (default)
        self.joint_positions = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
        
        # Timer to periodically publish position
        self.timer = self.create_timer(1.0, self.publish_positions)  # Publish every second
        self.get_logger().info("SnekBot Publisher is running!")

    def update_positions(self, new_positions):
        """Allows the positions to be updated."""
        self.joint_positions = new_positions

    def publish_positions(self):
        """Publishes the current positions repeatedly."""
        position_msg = Float32MultiArray()
        position_msg.data = self.joint_positions
        self.position_publisher_.publish(position_msg)
        self.get_logger().info(f'Publishing Position: {position_msg.data}')

def run_publisher(positions=None):
    """Starts the publisher and keeps it running."""
    rclpy.init()
    node = SnekBotPublisher()

    # Update positions if new ones are provided
    if positions:
        node.update_positions(positions)

    # Keep the program running, publishing positions on the timer
    rclpy.spin(node)  # This keeps the node alive until manually terminated

    # Shutdown ROS 2 when done
    rclpy.shutdown()
