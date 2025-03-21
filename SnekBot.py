import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.robot.ERobot import ERobot
from roboticstoolbox.backends.swift import Swift
from pathlib import Path
from spatialmath import SE3
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class SnekBot(ERobot, Node):
    def __init__(self, urdf_filename="SnekBot/SnekBot.urdf"):
        ERobot.__init__(self, urdf_filename)
        Node.__init__(self, "snekbot")

        urdf_path = self.get_urdf_path(urdf_filename)
        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        args = super().URDF_read(str(urdf_path))
        super().__init__(args[0], name=args[1])

        self.addconfiguration("init", np.deg2rad([0, -135, -45, 0, 0, 0]))
        self.addconfiguration("stance", np.deg2rad([0, 20, -20, 0, -50, 0]))
        self.q = self.configs["init"]

        self.env = Swift()
        self.env.launch(realtime=True, headless=True)
        self.env.add(self, robot_alpha=0, collision_alpha=1)
        self.current_position = self.set_position()

        self.target_position = None
        self.running = False
        self.control_thread = None

        # ROS 2 Publishers
        self.joint_publisher = self.create_publisher(JointState, "snekbot/joint_states", 10)
        self.gripper_publisher = self.create_publisher(Float64, "snekbot/gripper_angle", 10)

    @staticmethod
    def get_urdf_path(urdf_filename):
        return Path(__file__).resolve().parent / urdf_filename

    def set_position(self):
        self.current_position = self.fkine(self.q)

    def move_to_joint_position(self, start, end, steps):
        qt = rtb.jtraj(start, end, steps)
        for q in qt.q:
            self.q = q
            self.publish_joint_states()
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
                [np.deg2rad(P * 60), np.deg2rad(R), np.deg2rad(Y)], order="xyz"
            )

            arrived = False
            while not arrived:
                if not np.array_equal(self.target_position, np.array([x, y, z, R, P, Y])):
                    break

                v, arrived = rtb.p_servo(self.fkine(self.q), end_effector_position, gain=5, threshold=0.01)
                J = self.jacobe(self.q)
                self.qd = np.clip(np.linalg.pinv(J) @ v, -10, 10)
                self.publish_joint_states()
                self.env.step(0.01)

            if arrived:
                self.target_position = None

    def move_grippers(self, theta):
        msg = Float64()
        msg.data = theta
        self.gripper_publisher.publish(msg)
        print(f"Published gripper angle: {theta}")

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint_{i}" for i in range(len(self.q))]
        msg.position = self.q.tolist()
        self.joint_publisher.publish(msg)
        print(f"Published joint states: {msg.position}")

    def stop_movement(self):
        self.running = False
        if self.control_thread:
            self.control_thread.join()
