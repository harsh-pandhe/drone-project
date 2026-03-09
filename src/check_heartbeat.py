import asyncio
from mavsdk import System

async def run():
    drone = System()
    # Using the serial port verified in your stack check
    await drone.connect(system_address="serial:///dev/ttyAMA0:57600")

    print("Waiting for Pixhawk heartbeat...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[+] HEARTBEAT DETECTED: Pixhawk 6C is communicating!")
            break

if __name__ == "__main__":
    asyncio.run(run())