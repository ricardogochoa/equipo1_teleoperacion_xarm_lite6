#!/usr/bin/env python3
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import JointState


class UDPBridgeA(Node):
    def __init__(self):
        super().__init__("udp_bridge_A")

        self.declare_parameter("listen_port", 9001)
        self.declare_parameter("esp32_ip", "192.168.137.198")   # ESP32 maestro
        self.declare_parameter("esp32_port", 9100)

        listen_port = int(self.get_parameter("listen_port").value)
        self._esp32_ip = str(self.get_parameter("esp32_ip").value)
        self._esp32_port = int(self.get_parameter("esp32_port").value)

        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind(("", listen_port))
        self._rx.setblocking(False)

        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Poses = tiempo real => no acumular backlog
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Comandos = más importantes que una pose individual
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._cmd_pub = self.create_publisher(String, "/esp32/cmd", cmd_qos)
        self._stop_info_pub = self.create_publisher(String, "/esp32/stop_info", cmd_qos)

        self.create_subscription(
            JointState,
            "/teleop/master_joints",
            self._cb_joints,
            pose_qos,
        )

        self.create_timer(0.002, self._poll)

        self._last_cmd = ""
        self._last_cmd_ts = 0.0
        self._last_stop_info = ""
        self._last_stop_info_ts = 0.0

        self.get_logger().info(
            f"UDP bridge A: :{listen_port} ↔ {self._esp32_ip}:{self._esp32_port}"
        )

    def _publish_cmd_if_needed(self, msg: str):
        now = time.monotonic()

        # Evita floods de STOP/RESUME repetidos muy juntos
        if msg == self._last_cmd and (now - self._last_cmd_ts) < 0.15:
            return

        self._last_cmd = msg
        self._last_cmd_ts = now

        out = String()
        out.data = msg
        self._cmd_pub.publish(out)
        self.get_logger().info(f"[A CMD] {msg}")

    def _publish_stop_info_if_needed(self, msg: str):
        now = time.monotonic()

        if msg == self._last_stop_info and (now - self._last_stop_info_ts) < 0.25:
            return

        self._last_stop_info = msg
        self._last_stop_info_ts = now

        out = String()
        out.data = msg
        self._stop_info_pub.publish(out)
        self.get_logger().warn(f"[A STOP_INFO] {msg}")

    def _poll(self):
        try:
            while True:
                data, _addr = self._rx.recvfrom(256)
                msg = data.decode("utf-8", errors="ignore").strip()

                if not msg:
                    continue

                if msg in ("START", "STOP", "RESUME", "RESET"):
                    self._publish_cmd_if_needed(msg)

                elif msg.startswith("STOP_INFO,"):
                    self._publish_stop_info_if_needed(msg)

        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().error(f"_poll error: {e}")

    def _cb_joints(self, msg: JointState):
        vals = list(msg.position[:6])
        if len(vals) < 6:
            return

        payload = "pose," + ",".join(f"{v:.3f}" for v in vals)

        try:
            self._tx.sendto(
                payload.encode("utf-8"),
                (self._esp32_ip, self._esp32_port),
            )
        except Exception as e:
            self.get_logger().error(f"_cb_joints send error: {e}")

    def destroy_node(self):
        try:
            self._rx.close()
        except Exception:
            pass

        try:
            self._tx.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = UDPBridgeA()
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