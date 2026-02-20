import matplotlib.pyplot as plt
from missile import Missile

missile = Missile(
    x=0,
    y=0,
    speed=300,
    angle_deg=60,
    boost_accel=30,
    boost_time=60
)

dt = 0.1

positions = []

for i in range(2000):

    missile.update(dt)
    positions.append((missile.x, missile.y))

    if missile.y < 0:
        break

x_vals, y_vals = zip(*positions)
plt.plot(x_vals, y_vals)
plt.title("Missile Trajectory")
plt.xlabel("Distance (m)")
plt.ylabel("Height (m)")
plt.grid()
plt.show()
