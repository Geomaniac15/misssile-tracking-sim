import matplotlib.pyplot as plt
from missile import Missile

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

dt = 0.1

positions = []

for i in range(5000):

    missile.update(dt)

    print(f'Time: {missile.time:.1f} s')
    print(f'Position: ({missile.x:.1f}, {missile.y:.1f}) m')
    print(f'Mass: {missile.mass:.1f} kg')

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
