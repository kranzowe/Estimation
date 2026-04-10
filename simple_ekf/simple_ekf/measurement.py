import numpy as np

class ImuIntegrator:
    def __init__(self):
        self.last_write = None
        self.v = 0.0
        self.theta = 0.0

    def update(self, linear_acceleration, angular_velocity, current_time):
        if self.last_write is not None:
            dt = current_time - self.last_write
            self.v += linear_acceleration * dt
            self.theta += angular_velocity * dt
        self.last_write = current_time

    def get_vel_displacement(self, cmd):
        vint = self.v
        self.v = cmd[0]
        return vint

    def get_angle_displacement(self):
        thetaint = self.theta
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi
        return thetaint

def h_imu(state):
    return np.array([
        state[3],
        state[2]
    ])

def H_imu(_):
    return np.array([
        [0, 0, 0, 1],
        [0, 0, 1, 0]
    ])

R_imu = np.eye(2) * 1.0