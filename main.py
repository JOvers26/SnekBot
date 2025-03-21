import random
import time
from snekbot_publisher import run_publisher  # Import the function from the file above

def generate_random_positions():
    """Generate random joint positions every second."""
    return [random.uniform(0.0, 10.0) for _ in range(7)]  # Random values for 7 joints

def run_publisher_with_random_positions():
    """Run the publisher, updating positions randomly every second."""
    while True:
        # Generate new random positions for the joints
        new_positions = generate_random_positions()

        # Start the publisher with the new random positions
        run_publisher(new_positions)

        # Wait for 1 second before generating new random positions
        time.sleep(1)  # Import time at the top if it's not already

if __name__ == '__main__':
    run_publisher_with_random_positions()
