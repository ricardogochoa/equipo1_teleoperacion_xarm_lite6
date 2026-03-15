#!/usr/bin/env python3
import asyncio
import json
import time
import random
import websockets

# Fake esclavo:
# - recibe CMD_TRAJ
# - envía PRESSURE a 10 Hz

async def handler(ws):
    print("✅ Maestro conectado al fake esclavo")
    last_send = 0.0

    async for raw in ws:
        try:
            msg = json.loads(raw)
            if msg.get("type") == "CMD_TRAJ":
                print(f"CMD_TRAJ recibido seq={msg.get('seq')}")
        except Exception:
            pass

        now = time.time()
        if now - last_send > 0.1:  # 10 Hz
            pressure = 20.0 + 5.0 * random.random()
            out = {
                "type": "PRESSURE",
                "seq": int(now * 1000),
                "t": now,
                "payload": {"value": pressure, "unit": "kPa"},
            }
            await ws.send(json.dumps(out))
            last_send = now

async def main():
    async with websockets.serve(handler, "0.0.0.0", 9002):
        print("🟢 Fake slave server en ws://0.0.0.0:9002")
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())
