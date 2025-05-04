import numpy as np
import os
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame
import pygame
print(pygame.__version__)
import time
from SnekBot import SnekBot

pygame.init()

joystick = None
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick detected: {joystick.get_name()}")
else:
    print("No joystick detected. Please connect a joystick.")
    exit()

def apply_deadzone(value, threshold=0.3):
    return value if abs(value) > threshold else 0

snekbot = SnekBot()
print(snekbot)
time.sleep(1)
print("Moving from init to stance")
snekbot.move_to_joint_position(snekbot.configs["init"], snekbot.configs["stance"], 200)

speed_factor = 0.02
running = True

theta = 0
trigger_sensitivity = 0.1

while running:
    pygame.event.pump()

    if joystick.get_button(0):
        snekbot.move_to_joint_position(snekbot.q, snekbot.configs["init"], 200)

    if joystick.get_button(1):
        snekbot.move_to_joint_position(snekbot.q, snekbot.configs["stance"], 200)

    if joystick:
        x_axis = apply_deadzone(joystick.get_axis(0))
        y_axis = apply_deadzone(joystick.get_axis(1))
        R = apply_deadzone(joystick.get_axis(3))
        P = apply_deadzone(joystick.get_axis(4))

        z_axis = 0
        if joystick.get_button(4):
            z_axis = -0.6
        elif joystick.get_button(5):
            z_axis = 0.6

        Y = 0
        hat = joystick.get_hat(0)
        if hat[0] == 1:
            Y = -0.1
        elif hat[0] == -1:
            Y = 0.1

        pivot = 0
        hat = joystick.get_hat(0)
        if hat[1] == 1:
            pivot = -0.02
        elif hat[1] == -1:
            pivot = 0.02

        if pivot != 0:
            snekbot.increment_joint1(pivot)

        left_trigger = joystick.get_axis(2)
        right_trigger = joystick.get_axis(5)

        new_theta = theta + right_trigger * trigger_sensitivity - left_trigger * trigger_sensitivity
        new_theta = np.clip(new_theta, 0.5, np.pi)

        if new_theta != theta:
            theta = new_theta
            snekbot.move_grippers(theta)

        step = np.array([z_axis * speed_factor, x_axis * speed_factor, y_axis * speed_factor, 
                         P * speed_factor * 200, Y * speed_factor * 35, R * speed_factor * 200])

        if np.any(step):
            snekbot.set_target_position(step)

    time.sleep(0.05)

snekbot.stop_movement()
pygame.quit()
