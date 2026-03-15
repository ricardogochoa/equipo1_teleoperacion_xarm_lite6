#!/usr/bin/env python3
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import JointState

try:
    from xarm.wrapper import XArmAPI
    XARM_AVAILABLE = True
except ImportError:
    XARM_AVAILABLE = False
    print("[WARN] xarm-python-sdk not found — SIMULATION MODE")


class MasterNode(Node):
    XARM_IP = "192.168.1.154"
    READ_HZ = 30.0
    CONNECT_RETRIES = 5
    CONNECT_DELAY_S = 1.0

    HOME = [300.0, 0.0, 300.0, 180.0, 0.0, 0.0]  # mm / deg

    def __init__(self):
        super().__init__("master_node")
        self.get_logger().info("=== MASTER NODE (FREE-DRIVE / TELEOP SOURCE) ===")

        self._lock = threading.Lock()
        self._state = "IDLE"   # IDLE | RUNNING | STOPPED
        self._arm = None
        self._last_stop_info = None
        self._manual_mode = False

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

        self._joints_pub = self.create_publisher(
            JointState,
            "/teleop/master_joints",
            pose_qos,
        )
        self._status_pub = self.create_publisher(String, "/master/status", status_qos)

        self.create_subscription(String, "/esp32/cmd", self._cb_cmd, cmd_qos)
        self.create_subscription(String, "/esp32/stop_info", self._cb_stop_info, cmd_qos)

        self._init_arm()
        self.create_timer(1.0 / self.READ_HZ, self._loop)

        self.get_logger().info("Ready. Press START to enter manual mode and stream joints.")

    def _safe_arm_ready(self):
        if not self._arm:
            return False
        try:
            self._arm.clean_error()
            self._arm.clean_warn()
            self._arm.motion_enable(True)
            self._arm.set_state(0)
            return True
        except Exception as e:
            self.get_logger().error(f"_safe_arm_ready failed: {e}")
            return False

    def _init_arm(self):
        if not XARM_AVAILABLE:
            self.get_logger().warn("No xArm SDK — simulation only")
            return

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

                self._arm = arm
                self.get_logger().info("xArm maestro listo ✓")
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

    def _enter_manual_mode(self):
        if not self._arm or self._manual_mode:
            return

        if not self._safe_arm_ready():
            return

        try:
            self._arm.set_mode(2)   # free-drive / teaching
            time.sleep(0.1)
            self._arm.set_state(0)
            self._manual_mode = True
            self.get_logger().info("Maestro en modo manual: ya puedes moverlo con la mano")
        except Exception as e:
            self.get_logger().error(f"_enter_manual_mode failed: {e}")

    def _exit_manual_mode(self):
        if not self._arm:
            return
        try:
            self._arm.set_mode(0)
            self._arm.set_state(0)
            self._manual_mode = False
        except Exception as e:
            self.get_logger().error(f"_exit_manual_mode failed: {e}")

    def _go_home(self):
        if not self._arm:
            return
        try:
            self._exit_manual_mode()
            if not self._safe_arm_ready():
                return
            ret = self._arm.set_position(*self.HOME, speed=80, mvacc=500, wait=False)
            self.get_logger().info(f"HOME command ret={ret}")
        except Exception as e:
            self.get_logger().error(f"_go_home failed: {e}")

    def _cb_cmd(self, msg: String):
        cmd = msg.data.strip().upper()

        with self._lock:
            if cmd == "START":
                if self._state != "RUNNING":
                    self.get_logger().info("▶ START — streaming joints from master")
                self._state = "RUNNING"
                self._enter_manual_mode()

            elif cmd == "STOP":
                if self._state != "STOPPED":
                    self.get_logger().info("■ STOP — stop streaming, keep manual mode")
                self._state = "STOPPED"
                self._enter_manual_mode()

            elif cmd == "RESUME":
                if self._state != "RUNNING":
                    self.get_logger().info("▶ RESUME — continue streaming")
                self._state = "RUNNING"
                self._enter_manual_mode()

            elif cmd == "RESET":
                self._state = "IDLE"
                self.get_logger().info("↺ RESET — back to HOME")
                self._go_home()

    def _cb_stop_info(self, msg: String):
        self._last_stop_info = msg.data
        self.get_logger().warn(f"[MASTER STOP_INFO] {msg.data}")

        status_msg = String()
        status_msg.data = f"STOP_INFO_RECEIVED,{msg.data}"
        self._status_pub.publish(status_msg)

    def _loop(self):
        with self._lock:
            state = self._state

        if state != "RUNNING" or not self._arm:
            return

        try:
            ret_a, angles = self._arm.get_servo_angle(is_radian=False)
            if ret_a != 0 or angles is None:
                self.get_logger().warn(f"No pude leer joints del maestro ret={ret_a}")
                return

            angles = list(angles[:6])

            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = [f"joint{i+1}" for i in range(6)]
            joint_msg.position = [float(a) for a in angles]
            self._joints_pub.publish(joint_msg)

            s = String()
            s.data = "RUNNING," + ",".join(f"{a:.2f}" for a in angles)
            self._status_pub.publish(s)

        except Exception as e:
            self.get_logger().error(f"_loop exception: {e}")

    def destroy_node(self):
        try:
            if self._arm:
                self._exit_manual_mode()
                self._arm.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MasterNode()
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