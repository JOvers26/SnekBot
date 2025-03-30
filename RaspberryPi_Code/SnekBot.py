import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from snekbot_interfaces.msg import SnekBotData  # Custom message
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

        self.addconfiguration("init", np.deg2rad([0, -135, -45, 0, 0, 0]))
        self.addconfiguration("stance", np.deg2rad([0, 20, -20, 0, -50, 0]))
        self.q = self.configs["init"]

        self.env = swift.Swift()
        self.env.launch(realtime=True, headless=True)
        self.env.add(self, robot_alpha=0, collision_alpha=1)
        self.current_position = self.set_position()

        self.target_position = None
        self.running = False
        self.control_thread = None

        # ROS 2 Node Initialization
        rclpy.init()
        self.node = Node('snekbot_node')
        
        # Create a single publisher for snekbot/data
        self.data_pub = self.node.create_publisher(SnekBotData, 'snekbot/data', 10)

    @staticmethod
    def get_urdf_path(urdf_filename):
        # Get the absolute path of the Python script
        script_dir = Path(__file__).resolve().parent
        # Return the full path of the URDF file
        return script_dir / urdf_filename
    
    def set_position(self):
        self.current_position = self.fkine(self.q)
        # origin_axes = sg.Axes(length=0.1, pose=self.current_position)
        # self.env.add(origin_axes)

    def move_to_joint_position(self, start, end, steps):
        qt = rtb.jtraj(start, end, steps)
        for q in qt.q:
            self.q = q
            self.publish_data()  # Publish joint states and gripper data here
            # self.env.step(0.01)
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
            previous_q = self.q.copy()
            
            end_effector_position = self.fkine(self.q) * SE3(x, y, z) * SE3.RPY(
            [np.deg2rad(P*60), np.deg2rad(R), np.deg2rad(Y)], order='xyz'
        )

            arrived = False
            while not arrived:
                print(self.q)  # Send joint data here
                self.publish_data()  # Publish joint states and gripper data
                if not np.array_equal(self.target_position, np.array([x, y, z, R, P, Y])):  
                    break

                v, arrived = rtb.p_servo(self.fkine(self.q), end_effector_position, gain=5, threshold=0.01)
                J = self.jacobe(self.q)
                self.qd = np.clip(np.linalg.pinv(J) @ v, -10, 10)
                self.env.step(0.01)

            if arrived:
                self.target_position = None

    def move_grippers(self, theta):
        print(theta)  # Send gripper data here
        self.publish_data(theta)  # Publish gripper position

    def stop_movement(self):
        self.running = False
        if self.control_thread:
            self.control_thread.join()

    def publish_data(self, gripper_theta=None):
        # Publish joint states and gripper data in a single message
        data_msg = SnekBotData()
        data_msg.joint_1 = self.q[0]
        data_msg.joint_2 = self.q[1]
        data_msg.joint_3 = self.q[2]
        data_msg.joint_4 = self.q[3]
        data_msg.joint_5 = self.q[4]
        data_msg.joint_6 = self.q[5]
        
        # If gripper theta is provided, update the gripper position
        if gripper_theta is not None:
            data_msg.gripper = gripper_theta
        else:
            data_msg.gripper = 0.0  # Default gripper position

        self.data_pub.publish(data_msg)

def main():
    # Example of creating a robot and controlling it
    snekbot = SnekBot()
    snekbot.move_to_joint_position(snekbot.configs["init"], snekbot.configs["stance"], 50)
    snekbot.move_grippers(0.5)  # Example gripper position
    snekbot.stop_movement()

if __name__ == '__main__':
    main()
