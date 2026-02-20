import numpy as np

class KalmanCV:
    """
    constant-velocity Kalman Filter for state [x, y, vx, vy]^T
    measurements are [x, y]^T
    """

    def __init__(self,
                 x0,
                 P0,
                 sigma_a=10.0,      # bigger = filter trusts measurements more
                 sigma_pos=800.0
                 ):
        
        self.x = x0.astype(float)
        self.P = P0.astype(float)
        self.sigma_a = float(sigma_a)
        self.sigma_pos = float(sigma_pos)

        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=float)

        self.R = (self.sigma_pos ** 2) * np.eye(2)
    
    def _F(self, dt):
        return np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=float)
    
    def _Q(self, dt):
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt2
        q = (self.sigma_a ** 2)

        return q * np.array([
            [dt4/4, 0, dt3/2, 0],
            [0, dt4/4, 0, dt3/2],
            [dt3/2, 0, dt2, 0],
            [0, dt3/2, 0, dt2]
        ], dtype=float)
    
    def predict(self, dt):
        F = self._F(dt)
        Q = self._Q(dt)

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
    
    def update(self, z):
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + (K @ y)
        identity_matrix = np.eye(4)
        self.P = (identity_matrix - K @ self.H) @ self.P