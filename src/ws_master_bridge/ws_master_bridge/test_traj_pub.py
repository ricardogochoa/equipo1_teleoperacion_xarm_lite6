#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TestTrajPub(Node):
    def __init__(self):
        super().__init__("test_traj_pub")
        self.pub = self.create_publisher(JointTrajectory, "/master/cmd_traj", 10)
        self.t = 0.0
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.create_timer(0.2, self.tick)  # 5 Hz
        self.get_logger().info("Publicando /master/cmd_traj (fake)")

    def tick(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        p = JointTrajectoryPoint()
        p.positions = [0.2 * math.sin(self.t + 0.3*i) for i in range(6)]
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = int(200e6)

        msg.points = [p]
        self.pub.publish(msg)
        self.t += 0.2


def main():
    rclpy.init()
    node = TestTrajPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
