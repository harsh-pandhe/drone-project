import asyncio
from mavsdk import System

# Configuration confirmed by serial_dump.py success
MAVSDK_CONN = "serial:///dev/ttyAMA0:115200"

async def run():
    print("------------------------------------------------")
    print("      ASCEND FINAL CONNECTION TEST              ")
    print("------------------------------------------------")
    
    # CHANGE: Use default System() to auto-manage the backend server
    print("1. Initializing MAVSDK System (Auto-Backend)...")
    drone = System()
    
    print(f"2. Attempting connection to: {MAVSDK_CONN}")
    try:
        # This starts the mavsdk_server process in the background
        await drone.connect(system_address=MAVSDK_CONN)
        print("   -> Connection command sent. Waiting for backend...")
    except Exception as e:
        print(f"   !! ERROR: Failed to connect call: {e}")
        return

    print("3. Waiting for Heartbeat from Pixhawk...")
    
    # Define a separate function to check connection state
    async def wait_for_connection():
        async for state in drone.core.connection_state():
            if state.is_connected:
                return True
                
    try:
        await asyncio.wait_for(wait_for_connection(), timeout=20.0)
        
        print("   -> HEARTBEAT RECEIVED!")
        print("------------------------------------------------")
        print("   RESULT: SUCCESS ✅")
        print("   The link is stable. You are ready for drone_server.py")
        print("------------------------------------------------")
        return

    except asyncio.TimeoutError:
        print("------------------------------------------------")
        print("   RESULT: FAILED ❌")
        print("   Timed out waiting for heartbeat.")
        print("   Action: Reboot Pi to clear serial ports.")
        print("------------------------------------------------")
    except Exception as e:
        print(f"   !! ERROR: {e}")

if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    try:
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("\nTest Cancelled.")
