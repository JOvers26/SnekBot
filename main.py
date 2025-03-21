from snekbot_publisher import SnekBotPublisher
import rclpy
import random

def main():
    rclpy.init()  # Initialize ROS 2

    # Create an instance of the publisher class, which will initialize the node
    publisher_node = SnekBotPublisher()

    # Create a timer to send messages as fast as possible
    def send_continuous_position():
        publisher_node.send_position(str(random.randint(0, 100)))

    # Set a high-frequency timer, such as 10 Hz or higher if needed
    publisher_node.create_timer(0.01, send_continuous_position)  # 0.01 sec = 100 Hz

    # Keep the node running and processing messages
    rclpy.spin(publisher_node)

    # Shutdown ROS 2 once done
    rclpy.shutdown()

if __name__ == '__main__':
    main()
