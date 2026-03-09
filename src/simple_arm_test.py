import asyncio
import signal
import sys
from mavsdk import System

# Configuration
# Ensure this matches the successful baud rate from MAVProxy
MAVSDK_CONN = "serial:///dev/serial0:115200"

async def run():
    # 1. Initialize MAVSDK
    # We specify the mavsdk_server_address to avoid conflicts if one is already running
    drone = System(mavsdk_server_address="localhost", port=50051)
    
    print(f"-- Connecting to {MAVSDK_CONN}...")
    try:
        await drone.connect(system_address=MAVSDK_CONN)
    except Exception as e:
        print(f"!! Connection Error: {e}")
        return

    print("-- Waiting for drone to connect...")
    # Wait for the drone to send a heartbeat
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to Pixhawk!")
            break

    # 2. Check Health
    print("-- Checking Arming Health...")
    async for health in drone.telemetry.health():
        if not health.is_armable:
            print("!! Drone is NOT armable. Check GPS, Safety Switch, or Calibration.")
            # We continue anyway for this test, but it might fail to arm
        else:
            print("-- Drone is armable.")
        break # Only check once

    # 3. Arm the Drone
    print("-- Arming...")
    try:
        await drone.action.arm()
        print("-- ARMED! Motors should be spinning.")
    except Exception as e:
        print(f"!! Arming failed: {e}")
        return

    # 4. Wait 5 seconds
    print("-- Waiting 5 seconds...")
    await asyncio.sleep(5)

    # 5. Disarm
    print("-- Disarming...")
    try:
        await drone.action.disarm()
        print("-- DISARMED.")
    except Exception as e:
        print(f"!! Disarm failed: {e}")

async def shutdown(loop):
    print("\n-- Shutting down...")
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    [task.cancel() for task in tasks]
    await asyncio.gather(*tasks, return_exceptions=True)
    loop.stop()

if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    # Register signal handlers for clean exit
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: asyncio.create_task(shutdown(loop)))

    try:
        loop.run_until_complete(run())
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        print("-- Test Stopped.")
