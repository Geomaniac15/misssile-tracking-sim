import matplotlib.pyplot as plt
import numpy as np
from missile import Missile
from radar import Radar
from kalman import KalmanCV

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
    max_range=800_000,      # 800 km
    range_noise=300,        # 300 meters
    bearing_noise=0.004,    # 0.004 rad = ~0.23 degrees
    scan_interval=2.0       # 2 seconds
)

dt = 0.1

positions = []
measurement_points = []
kf_points = []

kf = None # not initialised until first measurement

for i in range(20_000): # max 20000 iterations = 2000 seconds = ~33 minutes

    missile.update(dt)
    positions.append((missile.x, missile.y))

    measurement = radar.scan(missile, dt)

    # predict step of Kalman Filter
    if kf is not None:
        kf.predict(dt)
    
    if measurement is not None:
        r, b = measurement
        x_meas = radar.x + r * np.cos(b)
        y_meas = radar.y + r * np.sin(b)
        measurement_points.append((x_meas, y_meas))

        z = np.array([[x_meas], [y_meas]], dtype=float)

        if kf is None:
            x0 = np.array([[x_meas], [y_meas], [0], [0]], dtype=float)
            P0 = np.diag([1e6, 1e6, 1e4, 1e4])  # huge initial uncertainty
            kf = KalmanCV(x0, P0, sigma_a=15.0, sigma_pos=800.0)
        else:
            kf.update(z)
    
    # log kf points for plotting
    if kf is not None:
        kf_points.append((float(kf.x[0, 0]), float(kf.x[1, 0])))
    
    if missile.y < 0:
        break

x_vals, y_vals = zip(*positions)
plt.plot(x_vals, y_vals, label='True Trajectory', color='blue')

if measurement_points:
    xm, ym = zip(*measurement_points)
    plt.scatter(
        xm,
        ym,
        s=15,
        alpha=0.6,
        label='Radar Measurements',
    )

if kf_points:
    xk, yk = zip(*kf_points)
    plt.plot(xk, 
             yk, 
             label='Kalman Estimate', 
             color='red')

plt.title("Trajectory + Radar + Kalman Track")
plt.xlabel("Distance (m)")
plt.ylabel("Height (m)")
plt.grid(True)
plt.legend()
plt.show()