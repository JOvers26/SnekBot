from snekbot_publisher import SnekBotPublisher
import rclpy

def main():
    rclpy.init()  # Initialize ROS 2

    # Create an instance of the publisher class, which will initialize the node
    publisher_node = SnekBotPublisher()

    # Send a position message
    publisher_node.send_position("Position: (1, 2, 3)")

    # Keep the node running and processing messages
    publisher_node.spin()

    # Shutdown ROS 2 once done
    rclpy.shutdown()

if __name__ == '__main__':
    main()
