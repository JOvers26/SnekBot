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

# Define a function to print joystick axis movements
def print_joystick_status():
    global previous_state
    
    # Left stick (axis 0 and 1)
    left_x = joystick.get_axis(0)
    left_y = joystick.get_axis(1)
    
    # Right stick (axis 3 and 4)
    right_x = joystick.get_axis(3)
    right_y = joystick.get_axis(4)
    
    # Triggers (axis 2 and 5)
    left_trigger = joystick.get_axis(2)
    right_trigger = joystick.get_axis(5)
    
    # Only print if there's a change in any axis
    changed = False

    if left_x != previous_state["left_x"]:
        print(f"Left Stick - X: {left_x:.2f}")
        previous_state["left_x"] = left_x
        changed = True
    if left_y != previous_state["left_y"]:
        print(f"Left Stick - Y: {left_y:.2f}")
        previous_state["left_y"] = left_y
        changed = True
    if right_x != previous_state["right_x"]:
        print(f"Right Stick - X: {right_x:.2f}")
        previous_state["right_x"] = right_x
        changed = True
    if right_y != previous_state["right_y"]:
        print(f"Right Stick - Y: {right_y:.2f}")
        previous_state["right_y"] = right_y
        changed = True
    if left_trigger != previous_state["left_trigger"]:
        print(f"Left Trigger: {left_trigger:.2f}")
        previous_state["left_trigger"] = left_trigger
        changed = True
    if right_trigger != previous_state["right_trigger"]:
        print(f"Right Trigger: {right_trigger:.2f}")
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

        # Print joystick movements if there's a change
        if print_joystick_status():
            print("Movement detected!")
        
        # Wait a short time before updating the status again
        time.sleep(0.1)

except KeyboardInterrupt:
    # Exit on Ctrl+C
    print("\nExiting...")
    pygame.quit()
