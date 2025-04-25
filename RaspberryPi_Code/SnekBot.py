import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.robot.ERobot import ERobot
from roboticstoolbox.backends.swift import Swift
from pathlib import Path
from spatialmath import *
import spatialgeometry as sg
import swift
import time
import threading

class SnekBot(ERobot):
    def __init__(self, urdf_filename="SnekBot_URDF/SnekBot.urdf"):
        urdf_path = self.get_urdf_path(urdf_filename)
        print(urdf_path)
        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        args = super().URDF_read(str(urdf_path))
        super().__init__(args[0], name=args[1])

        self.addconfiguration("init", np.deg2rad([0, -135, -135, 0, -135, 0]))
        self.addconfiguration("stance", np.deg2rad([0, 20, -50, 0, -40, 0]))
        self.q = self.configs["init"]

        self.env = swift.Swift()
        self.env.launch(realtime=True, headless=True)
        self.env.add(self, robot_alpha=0, collision_alpha=1)
        self.current_position = self.set_position()

        self.target_position = None
        self.running = False
        self.control_thread = None
        self.gripper_position = 0.0  # Initialize gripper position

        # ROS 2 Node Initialization
        rclpy.init()
        self.node = Node('snekbot_node')
        
        # Create joint state publisher
        self.joint_state_pub = self.node.create_publisher(JointState, 'snekbot/test_joint_states', 10)

    @staticmethod
    def get_urdf_path(urdf_filename):
        script_dir = Path(__file__).resolve().parent
        return script_dir / urdf_filename
    
    def set_position(self):
        self.current_position = self.fkine(self.q)

    def move_to_joint_position(self, start, end, steps):
        qt = rtb.jtraj(start, end, steps)
        for q in qt.q:
            self.q = q
            time.sleep(0.05)
            self.publish_joint_state()  # Publish joint states
        self.set_position()

    def set_target_position(self, new_position):
        self.target_position = new_position
        if not self.running:
            self.running = True
            self.control_thread = threading.Thread(target=self._move_loop, daemon=True)
            self.control_thread.start()

    def _move_loop(self):     
        while self.running:
            if self.target_position is None:
                time.sleep(0.01)
                continue

            x, y, z, R, P, Y = self.target_position
            
            end_effector_position = self.fkine(self.q) * SE3(x, y, z) * SE3.RPY(
                [np.deg2rad(P*60), np.deg2rad(R), np.deg2rad(Y)], order='xyz'
            )

            arrived = False
            while not arrived:
                print(self.q)  # Send joint data
                self.publish_joint_state()  # Publish joint states
                if not np.array_equal(self.target_position, np.array([x, y, z, R, P, Y])):
                    break

                v, arrived = rtb.p_servo(self.fkine(self.q), end_effector_position, gain=5, threshold=0.01)
                J = self.jacobe(self.q)
                self.qd = np.clip(np.linalg.pinv(J) @ v, -10, 10)
                self.env.step(0.01)

            if arrived:
                self.target_position = None

    def move_grippers(self, theta):
        self.gripper_position = theta  # Store gripper position
        print(f"Gripper position: {theta}")  
        self.publish_joint_state()  # Publish updated joint state with gripper

    def stop_movement(self):
        self.running = False
        if self.control_thread:
            self.control_thread.join()

    def publish_joint_state(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rclpy.time.Time().to_msg()
        joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'gripper']  # Added 'gripper'
        joint_state_msg.position = self.q.tolist() + [self.gripper_position]  # Append gripper position
        self.joint_state_pub.publish(joint_state_msg)
        print("Published: " + str(joint_state_msg))

def main():
    snekbot = SnekBot()
    snekbot.move_to_joint_position(snekbot.configs["init"], snekbot.configs["stance"], 50)
    snekbot.move_grippers(0.5)  # Example gripper position
    snekbot.stop_movement()

if __name__ == '__main__':
    main()
