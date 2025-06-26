import numpy as np
class KalmanFilter:
    def __init__(self, pos, vel):
        self.x = np.array([pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]])
        self.P = np.eye(6)
        self.F = np.eye(6)
        self.F[0,3] = self.F[1,4] = self.F[2,5] = 1
        self.H = np.zeros((3,6))
        self.H[0,0] = self.H[1,1] = self.H[2,2] = 1
        self.R = np.eye(3) * 0.5
        self.Q = np.eye(6) * 0.01

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:3]

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
