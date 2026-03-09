import asyncio
from mavsdk import System

# Correct Baud Rate from your MAVProxy success
MAVSDK_CONN = "serial:///dev/ttyAMA0:115200"

async def run():
    drone = System(mavsdk_server_address="localhost", port=50051)
    print(f"Connecting to {MAVSDK_CONN}...")
    await drone.connect(system_address=MAVSDK_CONN)

    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("SUCCESS: Connected to Pixhawk!")
            break

if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    loop.run_until_complete(run())
