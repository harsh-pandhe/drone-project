import asyncio
from mavsdk import System

# Configuration
MAVSDK_CONN = "serial:///dev/serial0:921600" # Update if using a different port

async def check_connection():
    drone = System()
    print(f"Checking connection to: {MAVSDK_CONN}...")
    
    try:
        # Attempt to connect
        await drone.connect(system_address=MAVSDK_CONN)

        print("Waiting for heartbeat (Timeout: 5s)...")
        # Wait for the drone to connect with a timeout
        async for state in asyncio.wait_for(drone.core.connection_state(), timeout=5.0):
            if state.is_connected:
                print("SUCCESS: Drone is connected!")
                return
            
    except asyncio.TimeoutError:
        print("ERROR: Connection timed out. Check baud rate (try 57600 or 115200) or wiring.")
    except Exception as e:
        print(f"ERROR: Connection failed: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(check_connection())
    except KeyboardInterrupt:
        print("\nCheck cancelled.")
