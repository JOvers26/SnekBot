from snekbot_publisher import SnekBotPublisher
import rclpy
import random
import time

def main():
    rclpy.init()  # Initialize ROS 2

    # Create an instance of the publisher class, which will initialize the node
    publisher_node = SnekBotPublisher()

    # Set the rate of publishing (e.g., 100 Hz)
    rate = 100  # 100 times per second
    period = 1.0 / rate  # Period for time.sleep in seconds

    # Run the loop to send messages as fast as possible
    while rclpy.ok():
        # Send a random position message
        publisher_node.send_position(str(random.randint(0, 100)))
        print("tick`")

        # Sleep for the specified period to control the rate
        time.sleep(period)

        # Allow ROS 2 to process callbacks (important for maintaining the node)
        rclpy.spin_once(publisher_node)

    # Shutdown ROS 2 once done
    rclpy.shutdown()

if __name__ == '__main__':
    main()
