import asyncio
import socket
import json
import logging
import signal
import sys
from mavsdk import System

# --- Configuration ---
HOST_IP = "0.0.0.0"  # Listen on all interfaces
PORT = 5000
# Connection string verified by final_test.py
MAVSDK_CONN = "serial:///dev/ttyAMA0:115200" 

# --- Logging Setup ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s | %(levelname)s | %(message)s')
logger = logging.getLogger("DroneServer")

class DroneServer:
    def __init__(self):
        # FIX: Initialize System() empty so it starts the backend automatically!
        # This matches the success of final_test.py
        self.drone = System()
        self.connected = False
        self.server = None
        self.running = True
        
        # Shared state dictionary
        self.state = {
            "bat_v": 0.0, "bat_p": 0.0, "gps_sats": 0, "gps_fix": "None",
            "alt": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "mode": "UNKNOWN", "armed": False, "status": "Initializing..."
        }

    async def start(self):
        logger.info(f"Starting Drone Server on Port {PORT}...")
        
        # 1. Start Connection Loop (Background)
        asyncio.create_task(self.drone_connection_loop())

        # 2. Start TCP Server (Listen for Laptop)
        self.server = await asyncio.start_server(self.handle_client, HOST_IP, PORT)
        logger.info(f"Listening for Mission Control on {HOST_IP}:{PORT}")

        async with self.server:
            while self.running:
                await asyncio.sleep(1)

    async def stop(self):
        logger.info("Stopping Drone Server...")
        self.running = False
        if self.server:
            self.server.close()
            await self.server.wait_closed()

    async def drone_connection_loop(self):
        """Monitors Pixhawk connection"""
        while self.running:
            try:
                if not self.connected:
                    logger.info(f"Attempting MAVSDK connection to {MAVSDK_CONN}...")
                    # connect() starts the background mavsdk_server
                    await self.drone.connect(system_address=MAVSDK_CONN)
                    
                    logger.info("Waiting for heartbeat (Scanning)...")
                    # This loop waits until the flight controller actually responds
                    async for state in self.drone.core.connection_state():
                        if state.is_connected:
                            if not self.connected:
                                logger.info(">>> PIXHAWK CONNECTED! <<<")
                                self.connected = True
                                self.state["status"] = "Connected"
                                asyncio.create_task(self.start_telemetry_streams())
                        else:
                            if self.connected:
                                logger.warning("Pixhawk Link Lost!")
                                self.connected = False
                                self.state["status"] = "Disconnected"
            except Exception as e:
                logger.error(f"Connection Error: {e}")
                self.connected = False
            
            await asyncio.sleep(5)

    async def start_telemetry_streams(self):
        logger.info("Starting Telemetry Streams...")
        try:
            # Set update rates to ensure data flow
            await self.drone.telemetry.set_rate_battery(1.0)
            await self.drone.telemetry.set_rate_position(5.0)
            await self.drone.telemetry.set_rate_attitude(10.0)
        except: pass

        asyncio.create_task(self.stream_battery())
        asyncio.create_task(self.stream_gps())
        asyncio.create_task(self.stream_position())
        asyncio.create_task(self.stream_attitude())
        asyncio.create_task(self.stream_flight_mode())
        asyncio.create_task(self.stream_armed())

    # --- Telemetry Collectors ---
    async def stream_battery(self):
        try:
            async for bat in self.drone.telemetry.battery():
                self.state["bat_v"] = round(bat.voltage_v, 2)
                self.state["bat_p"] = int(bat.remaining_percent * 100)
        except: pass

    async def stream_gps(self):
        try:
            async for gps in self.drone.telemetry.gps_info():
                self.state["gps_sats"] = gps.num_satellites
                self.state["gps_fix"] = str(gps.fix_type)
        except: pass

    async def stream_position(self):
        try:
            async for pos in self.drone.telemetry.position():
                self.state["alt"] = round(pos.relative_altitude_m, 2)
        except: pass

    async def stream_attitude(self):
        try:
            async for att in self.drone.telemetry.attitude_euler():
                self.state["roll"] = round(att.roll_deg, 1)
                self.state["pitch"] = round(att.pitch_deg, 1)
                self.state["yaw"] = round(att.yaw_deg, 1)
        except: pass

    async def stream_flight_mode(self):
        try:
            async for mode in self.drone.telemetry.flight_mode():
                self.state["mode"] = str(mode)
        except: pass

    async def stream_armed(self):
        try:
            async for armed in self.drone.telemetry.armed():
                self.state["armed"] = armed
        except: pass

    # --- Client Handling ---
    async def handle_client(self, reader, writer):
        addr = writer.get_extra_info('peername')
        logger.info(f"Client Connected: {addr}")

        while self.running:
            try:
                data = await reader.read(1024)
                if not data: break
                
                command = data.decode().strip().upper()
                
                if command == "GET_DATA":
                    response = json.dumps(self.state)
                else:
                    logger.info(f"Received Command: {command}")
                    response = await self.execute_action(command)
                
                writer.write(response.encode())
                await writer.drain()
            except: break
        
        writer.close()

    async def execute_action(self, cmd):
        if not self.connected:
            return "ERR: DRONE NOT CONNECTED"

        try:
            if cmd == "ARM":
                await self.drone.action.arm()
                return "ACK: ARMED"
            elif cmd == "DISARM":
                await self.drone.action.disarm()
                return "ACK: DISARMED"
            elif cmd == "TAKEOFF":
                await self.drone.action.set_takeoff_altitude(1.0)
                await self.drone.action.takeoff()
                return "ACK: TAKEOFF SENT"
            elif cmd == "LAND":
                await self.drone.action.land()
                return "ACK: LANDING"
            else:
                return f"ERR: UNKNOWN {cmd}"
        except Exception as e:
            logger.error(f"Action Failed: {e}")
            return f"ERR: {str(e)}"

# --- Shutdown Handler ---
async def shutdown(server_instance, loop):
    print("\nShutting down...")
    await server_instance.stop()
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    [task.cancel() for task in tasks]
    await asyncio.gather(*tasks, return_exceptions=True)
    loop.stop()

if __name__ == "__main__":
    server = DroneServer()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: asyncio.create_task(shutdown(server, loop)))

    try:
        loop.run_until_complete(server.start())
    except (KeyboardInterrupt, asyncio.CancelledError): pass
    finally: print("Server Stopped.")
