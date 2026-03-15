#!/usr/bin/env python3
import json
import time
import queue
import threading
import asyncio

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory

import websockets


def traj_to_dict(msg: JointTrajectory) -> dict:
    return {
        "joint_names": list(msg.joint_names),
        "points": [
            {
                "positions": list(p.positions),
                "velocities": list(p.velocities),
                "accelerations": list(p.accelerations),
                "effort": list(p.effort),
                "time_from_start_sec": int(p.time_from_start.sec),
                "time_from_start_nanosec": int(p.time_from_start.nanosec),
            }
            for p in msg.points
        ],
    }


class MasterWsBridge(Node):
    """
    MAESTRO:
      ROS2 SUB  /master/cmd_traj   (JointTrajectory) -> WS send CMD_TRAJ
      WS recv PRESSURE -> ROS2 PUB /slave/pressure   (Float32)
    """

    def __init__(self):
        super().__init__("master_ws_bridge")
        self.declare_parameter("ws_uri", "ws://127.0.0.1:9002")
        self.ws_uri = self.get_parameter("ws_uri").get_parameter_value().string_value

        self.sub_traj = self.create_subscription(
            JointTrajectory, "/master/cmd_traj", self.on_traj, 10
        )
        self.pub_pressure = self.create_publisher(Float32, "/slave/pressure", 10)

        self.tx_q: "queue.Queue[dict]" = queue.Queue()
        self.rx_q: "queue.Queue[dict]" = queue.Queue()

        self._stop = threading.Event()
        threading.Thread(target=self.ws_worker, daemon=True).start()

        self.create_timer(0.01, self.drain_rx)
        self.get_logger().info(f"Bridge listo. Conectando a {self.ws_uri}")

    def on_traj(self, msg: JointTrajectory):
        out = {
            "type": "CMD_TRAJ",
            "seq": int(time.time() * 1000),
            "t": time.time(),
            "payload": traj_to_dict(msg),
        }
        self.tx_q.put(out)

    def drain_rx(self):
        while True:
            try:
                m = self.rx_q.get_nowait()
            except queue.Empty:
                break

            if m.get("type") == "PRESSURE":
                val = float(m.get("payload", {}).get("value", 0.0))
                msg = Float32()
                msg.data = val
                self.pub_pressure.publish(msg)

    def ws_worker(self):
        asyncio.run(self.ws_loop())

    async def ws_loop(self):
        while not self._stop.is_set():
            try:
                async with websockets.connect(self.ws_uri, ping_interval=10, ping_timeout=10) as ws:
                    self.get_logger().info("✅ WS conectado")
                    rx_task = asyncio.create_task(self.ws_rx(ws))

                    while not self._stop.is_set():
                        try:
                            msg = self.tx_q.get_nowait()
                            await ws.send(json.dumps(msg))
                        except queue.Empty:
                            pass

                        await asyncio.sleep(0.002)

                    rx_task.cancel()

            except Exception as e:
                self.get_logger().warn(f"WS desconectado, reintentando: {e}")
                await asyncio.sleep(1.0)

    async def ws_rx(self, ws):
        async for raw in ws:
            try:
                self.rx_q.put_nowait(json.loads(raw))
            except Exception:
                pass


def main():
    rclpy.init()
    node = MasterWsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
