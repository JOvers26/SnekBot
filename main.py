import random
import rclpy
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
import time

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'position_updates', 10)

    def publish_random_positions(self):
        """Generate random joint positions and publish them."""
        joint_positions = [random.uniform(0.0, 10.0) for _ in range(7)]  # Random positions for 7 joints
        position_msg = Float32MultiArray()
        position_msg.data = joint_positions
        self.publisher_.publish(position_msg)
        self.get_logger().info(f'Publishing random positions: {joint_positions}')


def main():
    rclpy.init()
    
    # Create a PositionPublisher to send random positions
    node = PositionPublisher()

    # Continuously publish random positions every second
    while rclpy.ok():
        node.publish_random_positions()
        rclpy.spin_once(node)  # Allow ROS to process callbacks
        time.sleep(1)  # Wait for 1 second before sending the next update

    # Shutdown ROS 2 when done
    rclpy.shutdown()

if __name__ == '__main__':
    main()
