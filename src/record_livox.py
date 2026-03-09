import socket
import struct
import csv

UDP_PORT = 56101 

# Setup Socket to listen to anything hitting this port
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', UDP_PORT))

print("Eavesdropping on Port 56101... Waiting for Quick Start to begin...")

with open('livox_scan.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y", "z", "reflectivity"])
    count = 0
    try:
        while True:
            data, addr = sock.recvfrom(1500)
            if len(data) >= 1300: # Standard point cloud packet size
                # Extract first point from each packet to verify connection
                x, y, z, ref = struct.unpack_from('<iiiB', data, 24)
                writer.writerow([x/1000.0, y/1000.0, z/1000.0, ref])
                count += 1
                if count % 100 == 0:
                    print(f"Captured {count} packets...", end='\r')
    except KeyboardInterrupt:
        print(f"\nScan saved! Total lines in CSV: {count}")