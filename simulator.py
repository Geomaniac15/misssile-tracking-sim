import matplotlib.pyplot as plt
import numpy as np
from missile import Missile
from radar import Radar

missile = Missile(
    x=0,
    y=0,
    speed=300,
    angle_deg=85,
    thrust=450000,
    mass_initial=30000,
    mass_flow_rate=120,
    burn_time=120,
    pitch_rate_deg=0.6
)

radar = Radar(
    x=0,
    y=0,
    max_range=800_000,
    range_noise=300,
    bearing_noise=0.003,
    scan_interval=2.0
)

dt = 0.1

positions = []
measurement_points = []

for i in range(5000):

    missile.update(dt)
    radar_measurement = radar.scan(missile, dt)

    if radar_measurement is not None:
        r, b = radar_measurement

        x_meas = radar.x + r * np.cos(b)
        y_meas = radar.y + r * np.sin(b)

        measurement_points.append((x_meas, y_meas))



    print(f'Time: {missile.time:.1f} s')
    print(f'Position: ({missile.x:.1f}, {missile.y:.1f}) m')
    print(f'Mass: {missile.mass:.1f} kg')

    positions.append((missile.x, missile.y))

    if missile.y < 0:
        break

x_vals, y_vals = zip(*positions)
x_meas_vals, y_meas_vals = zip(*measurement_points)

plt.plot(x_vals, y_vals)
plt.scatter(
    x_meas_vals,
    y_meas_vals,
    s=15,
    alpha=0.6
)

plt.title("Missile Trajectory")
plt.xlabel("Distance (m)")
plt.ylabel("Height (m)")
plt.grid()
plt.show()
