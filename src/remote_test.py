import socket
import subprocess
import os

def check_hardware():
    print("--- Project ASCEND: Remote Hardware Diagnostic ---")
    
    # 1. Check Coral TPU
    print("\n[1/3] Checking Google Coral TPU...")
    lsusb = subprocess.run(['lsusb'], capture_output=True, text=True).stdout
    if "1a6e:089a" in lsusb:
        print("SUCCESS: Coral detected (Global Unichip mode). Needs driver initialization.")
    elif "18d1:9302" in lsusb:
        print("SUCCESS: Coral detected and initialized (Google Inc. mode).")
    else:
        print("FAILED: Coral TPU not found on USB bus.")

    # 2. Check LiDAR Network Connection (Ping)
    lidar_ip = "192.168.1.199"
    print(f"\n[2/3] Pinging Livox Mid-360 at {lidar_ip}...")
    response = os.system(f"ping -c 1 -W 1 {lidar_ip} > /dev/null 2>&1")
    if response == 0:
        print(f"SUCCESS: Mid-360 is reachable at {lidar_ip}")
    else:
        print(f"FAILED: Cannot reach Mid-360. Check Ethernet/Subnet (Pi must be 192.168.1.50).")

    # 3. Check UDP Data Stream (Port 56101)
    print("\n[3/3] Listening for Point Cloud packets (Port 56101)...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3.0) # 3 second wait
    try:
        # Bind to the Pi's static IP to listen
        sock.bind(('192.168.1.50', 56101))
        data, addr = sock.recvfrom(1024)
        print(f"SUCCESS: Receiving data from {addr}! LiDAR is streaming.")
    except socket.timeout:
        print("FAILED: No data packets received on Port 56101. Is the SDK running?")
    except Exception as e:
        print(f"ERROR: {e}")
    finally:
        sock.close()

if __name__ == "__main__":
    check_hardware()
