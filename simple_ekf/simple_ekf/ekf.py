import numpy as np

class EKF:
    def __init__(
            self, initial_state, initial_covariance, process_noise_cov,
            dynamics_func, dynamics_jacobian_func
        ):
        self.x = initial_state
        self.P = initial_covariance
        self.Q = process_noise_cov
        self.f = dynamics_func
        self.F = dynamics_jacobian_func

    def predict(self, control_input, dt):
        # State transition model (simple kinematic model)
        F = self.F(self.x, control_input, dt)
        self.x = self.f(self.x, control_input, dt)
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement, measurement_func, measurement_jacobian_func, sensor_noise_cov):
        # Measurement model (identity for simplicity)
        R = sensor_noise_cov
        H = measurement_jacobian_func(self.x)  # Measurement matrix
        y = measurement - measurement_func(self.x)  # Measurement residual
        S = H @ self.P @ H.T + R  # Residual covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain
        self.x += K @ y  # Update state estimate
        I = np.eye(len(self.x))
        self.P = (I - K @ H) @ self.P  # Update covariance estimate
    
    def get_state(self):
        return self.x