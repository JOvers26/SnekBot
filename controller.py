import pygame
import time

# Initialize Pygame
pygame.init()

# Initialize the joystick module
pygame.joystick.init()

# Check if any joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    pygame.quit()
    exit()

# Get the first joystick (Xbox controller should be the first if connected)
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Store the previous state of the joystick axes
previous_state = {
    "left_x": 0.0,
    "left_y": 0.0,
    "right_x": 0.0,
    "right_y": 0.0,
    "left_trigger": 0.0,
    "right_trigger": 0.0
}

# Define a function to print joystick axis movements in an array
def print_joystick_status():
    # Left stick (axis 0 and 1)
    left_x = joystick.get_axis(0)
    left_y = joystick.get_axis(1)
    
    # Right stick (axis 3 and 4)
    right_x = joystick.get_axis(3)
    right_y = joystick.get_axis(4)
    
    # Triggers (axis 2 and 5)
    left_trigger = joystick.get_axis(2)
    right_trigger = joystick.get_axis(5)
    
    # Create the status array (dictionary)
    status = {
        "Left Stick X": round(left_x, 2),
        "Left Stick Y": round(left_y, 2),
        "Right Stick X": round(right_x, 2),
        "Right Stick Y": round(right_y, 2),
        "Left Trigger": round(left_trigger, 2),
        "Right Trigger": round(right_trigger, 2)
    }
    
    # Print the status as one big array (or dictionary)
    print(status)

try:
    while True:
        # Process events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        # Print joystick movements continuously
        print_joystick_status()
        
        # Wait a short time before updating the status again
        time.sleep(0.1)

except KeyboardInterrupt:
    # Exit on Ctrl+C
    print("\nExiting...")
    pygame.quit()
