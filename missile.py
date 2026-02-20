import numpy as np

GRAVITY = 9.81

class Missile:
    def __init__(self, 
                 x, 
                 y, 
                 speed, 
                 angle_deg,
                 thrust,          # Newtons
                 mass_initial,    # kg
                 mass_flow_rate,  # kg/s
                 burn_time,       # seconds
                 pitch_rate_deg   # degrees per second
                 ):
        
        self.x = x
        self.y = y
        
        angle = np.radians(angle_deg)

        self.vx = speed * np.cos(angle)
        self.vy = speed * np.sin(angle)

        self.thrust = thrust
        self.mass_initial = mass_initial
        self.mass = mass_initial
        self.mass_flow_rate = mass_flow_rate
        self.burn_time = burn_time

        self.theta0 = angle
        self.pitch_rate = np.radians(pitch_rate_deg)

        self.time = 0
    
    def update(self, dt):

        # find acceleration
        if self.time < self.burn_time:

            theta = self.theta0 - self.pitch_rate * self.time

            ax = (self.thrust * np.cos(theta)) / self.mass
            ay = (self.thrust * np.sin(theta)) / self.mass - GRAVITY

            # update mass
            self.mass -= self.mass_flow_rate * dt
            
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