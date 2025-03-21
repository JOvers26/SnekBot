from snekbot_publisher import SnekBotPublisher

def main():
    # Create an instance of the publisher class, which will initialize ROS 2
    publisher_node = SnekBotPublisher()

    # Send a position message
    publisher_node.send_position("Position: (1, 2, 3)")

    # Keep the node running and processing messages
    publisher_node.spin()

    # Shutdown ROS 2 once done
    publisher_node.shutdown()

if __name__ == '__main__':
    main()
