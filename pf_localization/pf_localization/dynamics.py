import numpy as np
from scipy.integrate import solve_ivp
from dataclasses import dataclass

def inertial_dyn(_, state, u):
    x, y, theta, v = state
    accel, turn_rate = u

    xdot = v * np.sin(theta)
    ydot = v * np.cos(theta)
    vdot = accel
    thetadot = turn_rate

    return [xdot, ydot, thetadot, vdot]

def solve_dyn(x, u, dt):
    sol = solve_ivp(lambda t, x: inertial_dyn(t, x, u), (0, dt), x)
    xk1 = sol.y[:,-1].T
    xk1[2] = (xk1[2] + np.pi) % (2 * np.pi) - np.pi
    return xk1

@dataclass
class NoiseParams:
    lateral_std: float
    forward_std: float
    theta_std: float
    v_std: float

def process_noise(x, dt, params):
    _, _, theta, _ = x
    rng = np.random.default_rng()
    lateral_noise = rng.normal(0, params.lateral_std) * dt
    forward_noise = rng.normal(0, params.forward_std) * dt
    x_noise = lateral_noise * np.sin(theta) + forward_noise * np.cos(theta)
    y_noise = lateral_noise * np.cos(theta) + forward_noise * np.sin(theta)
    theta_noise = rng.normal(0, params.theta_std) * dt
    v_noise = rng.normal(0, params.v_std) * dt
    return np.array([x_noise, y_noise, theta_noise, v_noise])

