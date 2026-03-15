#!/usr/bin/env python3
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from xarm.wrapper import XArmAPI


class SlaveNode(Node):
    XARM_IP = "192.168.1.179"   # xArm esclavo

    HOME_CART = [300.0, 0.0, 300.0, 180.0, 0.0, 0.0]  # mm / deg

    CMD_HZ = 30.0
    FILTER_ALPHA = 0.35

    CONNECT_RETRIES = 5
    CONNECT_DELAY_S = 1.0

    def __init__(self):
        super().__init__("slave_node")
        self.get_logger().info("=== SLAVE NODE (JOINT MIRROR) ===")

        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._lock = threading.Lock()
        self._stopped = False
        self._have_joints = False
        self._last_stop_info = None
        self._arm = None

        self._target_joints = np.zeros(6, dtype=float)
        self._cmd_joints = np.zeros(6, dtype=float)

        self.create_subscription(
            JointState,
            "/teleop/master_joints",
            self._cb_joints,
            pose_qos,
        )
        self.create_subscription(String, "/esp32/cmd", self._cb_cmd, cmd_qos)
        self.create_subscription(String, "/esp32/stop_info", self._cb_stop_info, cmd_qos)

        self._status_pub = self.create_publisher(String, "/slave/status", status_qos)

        self._connect_and_prepare_arm()

        self.get_logger().info("xArm esclavo listo ✓")
        self.get_logger().info("Ready. Waiting for /teleop/master_joints ...")

        self.create_timer(1.0 / self.CMD_HZ, self._tick)

    def _connect_and_prepare_arm(self):
        last_error = None

        for attempt in range(1, self.CONNECT_RETRIES + 1):
            arm = None
            try:
                self.get_logger().info(
                    f"Connecting to xArm @ {self.XARM_IP} "
                    f"(attempt {attempt}/{self.CONNECT_RETRIES})"
                )

                arm = XArmAPI(self.XARM_IP, is_radian=False, do_not_open=True)
                arm.connect()

                arm.clean_error()
                arm.clean_warn()
                arm.motion_enable(True)
                arm.set_mode(0)
                arm.set_state(0)

                self.get_logger().info("Moving to HOME ...")
                arm.set_position(*self.HOME_CART, speed=60, mvacc=300, wait=True)

                ret, ang = arm.get_servo_angle(is_radian=False)
                if ret == 0 and ang is not None and len(ang) >= 6:
                    self._target_joints = np.array(ang[:6], dtype=float)
                    self._cmd_joints = np.array(ang[:6], dtype=float)
                else:
                    self._target_joints = np.zeros(6, dtype=float)
                    self._cmd_joints = np.zeros(6, dtype=float)

                arm.set_mode(1)
                arm.set_state(0)

                self._arm = arm
                return

            except Exception as e:
                last_error = e
                self.get_logger().warn(f"Connect failed: {e}")
                try:
                    if arm is not None:
                        arm.disconnect()
                except Exception:
                    pass
                time.sleep(self.CONNECT_DELAY_S)

        raise RuntimeError(f"Could not connect to xArm {self.XARM_IP}: {last_error}")

    def _cb_joints(self, msg: JointState):
        if len(msg.position) < 6:
            return

        with self._lock:
            if self._stopped:
                return
            self._target_joints = np.array(msg.position[:6], dtype=float)
            self._have_joints = True

    def _cb_cmd(self, msg: String):
        if self._arm is None:
            return

        cmd = msg.data.strip().upper()

        if cmd == "STOP":
            with self._lock:
                self._stopped = True
            self.get_logger().warn("STOP received — holding slave")
            try:
                self._arm.set_state(4)
            except Exception as e:
                self.get_logger().error(f"STOP failed: {e}")

        elif cmd == "START":
            with self._lock:
                self._stopped = False
            self.get_logger().info("START received — resuming slave")
            try:
                self._arm.set_mode(1)
                self._arm.set_state(0)
            except Exception as e:
                self.get_logger().error(f"START failed: {e}")

        elif cmd == "RESUME":
            with self._lock:
                self._stopped = False
            self.get_logger().info("RESUME received — continuing slave")
            try:
                self._arm.set_mode(1)
                self._arm.set_state(0)
            except Exception as e:
                self.get_logger().error(f"RESUME failed: {e}")

        elif cmd == "RESET":
            with self._lock:
                self._stopped = True
                self._have_joints = False

            self.get_logger().info("RESET received — going HOME")
            try:
                self._arm.set_mode(0)
                self._arm.set_state(0)
                self._arm.set_position(*self.HOME_CART, speed=60, mvacc=300, wait=True)

                ret, ang = self._arm.get_servo_angle(is_radian=False)
                if ret == 0 and ang is not None and len(ang) >= 6:
                    self._target_joints = np.array(ang[:6], dtype=float)
                    self._cmd_joints = np.array(ang[:6], dtype=float)

                self._arm.set_mode(1)
                self._arm.set_state(0)
            except Exception as e:
                self.get_logger().error(f"RESET failed: {e}")

    def _cb_stop_info(self, msg: String):
        self._last_stop_info = msg.data
        self.get_logger().warn(f"[SLAVE STOP_INFO] {msg.data}")

        status_msg = String()
        status_msg.data = f"STOP_INFO_RECEIVED,{msg.data}"
        self._status_pub.publish(status_msg)

    def _recover_arm(self):
        if self._arm is None:
            return

        try:
            self._arm.clean_error()
            self._arm.clean_warn()
            self._arm.motion_enable(True)
            self._arm.set_mode(1)
            self._arm.set_state(0)
            self.get_logger().warn("Recovered slave xArm")
        except Exception as e:
            self.get_logger().error(f"Recover failed: {e}")

    def _tick(self):
        if self._arm is None:
            return

        with self._lock:
            if self._stopped or not self._have_joints:
                return
            target = self._target_joints.copy()

        self._cmd_joints = (
            (1.0 - self.FILTER_ALPHA) * self._cmd_joints
            + self.FILTER_ALPHA * target
        )

        try:
            ret = self._arm.set_servo_angle_j(
                angles=self._cmd_joints.tolist(),
                is_radian=False
            )
        except Exception as e:
            self.get_logger().error(f"set_servo_angle_j exception: {e}")
            self._recover_arm()
            return

        if ret != 0:
            self.get_logger().warn(f"set_servo_angle_j ret={ret} — recovering")
            self._recover_arm()
            return

        s = String()
        s.data = "RUN," + ",".join(f"{a:.2f}" for a in self._cmd_joints.tolist())
        self._status_pub.publish(s)

    def destroy_node(self):
        try:
            if self._arm is not None:
                self._arm.set_mode(0)
                self._arm.set_state(0)
                self._arm.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SlaveNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()