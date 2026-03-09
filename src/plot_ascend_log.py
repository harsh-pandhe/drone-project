import csv
import matplotlib.pyplot as plt

timestamps, vx, vy, alt = [], [], [], []

with open('ascend_flight_log.csv', 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        timestamps.append(float(row['timestamp']))
        vx.append(float(row['vx']))
        vy.append(float(row['vy']))
        alt.append(float(row['alt']))

# Normalize time
t0 = timestamps[0]
time_rel = [t - t0 for t in timestamps]

# Plotting
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

ax1.plot(time_rel, alt, label='Altitude', color='green')
ax1.set_title('ISRO ASCEND: Altitude Profile')
ax1.set_ylabel('Height (m)')
ax1.grid(True)

ax2.plot(vx, vy, label='Drift Path', color='orange')
ax2.set_title('Optical Flow Trajectory (X vs Y)')
ax2.set_xlabel('Velocity X')
ax2.set_ylabel('Velocity Y')
ax2.grid(True)

plt.tight_layout()
plt.show()