import numpy as np

class Radar:
    def __init__(self,
                 x,
                 y,
                 max_range,
                 range_noise,
                 bearing_noise,
                 scan_interval
                 ):
        
        self.x = x
        self.y = y
        self.max_range = max_range
        self.range_noise = range_noise
        self.bearing_noise = bearing_noise
        self.scan_interval = scan_interval
        self.time_since_last_scan = 0
    
    def scan(self, missile, dt):
        self.time_since_last_scan += dt

        if self.time_since_last_scan < self.scan_interval:
            return None
        
        self.time_since_last_scan = 0

        dx = missile.x - self.x
        dy = missile.y - self.y

        true_range = np.sqrt(dx**2 + dy**2)

        if true_range > self.max_range:
            return None
        
        true_bearing = np.arctan2(dy, dx)

        measured_range = true_range + np.random.normal(0, self.range_noise)
        measured_bearing = true_bearing + np.random.normal(0, self.bearing_noise)

        return measured_range, measured_bearing

