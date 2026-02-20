import numpy as np

GRAVITY = 9.81

class Missile:
    def __init__(self, x, y, speed, angle_deg, boost_accel, boost_time):
        self.x = x
        self.y = y
        
        angle = np.radians(angle_deg)

        self.vx = speed * np.cos(angle)
        self.vy = speed * np.sin(angle)

        self.boost_accel = boost_accel
        self.boost_time = boost_time

        self.time = 0
    
    def update(self, dt):

        # find acceleration
        if self.time < self.boost_time:
            angle = np.arctan2(self.vy, self.vx)

            ax = self.boost_accel * np.cos(angle)
            ay = self.boost_accel * np.sin(angle) - GRAVITY
        else:
            ax = 0
            ay = -GRAVITY
        
        # update velocity
        self.vx += ax * dt
        self.vy += ay * dt

        # update position
        self.x += self.vx * dt
        self.y += self.vy * dt

        self.time += dt