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
    "right_y": 0.0,  # Right stick Y (up/down)
    "left_trigger": 0.0,  # Left trigger
    "right_trigger": 0.0  # Right trigger
}

# Define a function to check and print joystick changes
def print_joystick_status():
    global previous_state
    
    # Right stick Y (axis 4, up/down)
    right_y = joystick.get_axis(4)
    
    # Triggers (axis 2 and 5)
    left_trigger = joystick.get_axis(2)
    right_trigger = joystick.get_axis(5)
    
    # Track changes
    changed = False
    
    # Check for right joystick Y movement (up/down)
    if right_y != previous_state["right_y"]:
        if right_y > 0:
            print("Right Stick Moved Down")
        elif right_y < 0:
            print("Right Stick Moved Up")
        previous_state["right_y"] = right_y
        changed = True
    
    # Check for left trigger press
    if left_trigger != previous_state["left_trigger"]:
        if left_trigger > 0:
            print("Left Trigger Pressed")
        previous_state["left_trigger"] = left_trigger
        changed = True
    
    # Check for right trigger press
    if right_trigger != previous_state["right_trigger"]:
        if right_trigger > 0:
            print("Right Trigger Pressed")
        previous_state["right_trigger"] = right_trigger
        changed = True
    
    return changed

try:
    while True:
        # Process events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        # Check and print if there are any changes in joystick movements
        print_joystick_status()

        # Wait a short time before checking again
        time.sleep(0.1)

except KeyboardInterrupt:
    # Exit on Ctrl+C
    print("\nExiting...")
    pygame.quit()
