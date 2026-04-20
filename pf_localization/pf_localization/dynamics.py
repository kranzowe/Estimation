import numpy as np
from scipy.integrate import solve_ivp

WHEELBASE = 0.171 # m

def steering_model(_, state, u):
    x, y, v, theta = state
    accel, turn_rate = u

    xdot = v * np.sin(theta)
    ydot = v * np.cos(theta)
    vdot = accel
    thetadot = turn_rate

    return [xdot, ydot, vdot, thetadot]

def solve_dyn(x, u, dt):
    sol = solve_ivp(steering_model, dt, x, args=(u))
    return sol.y[:, -1].T

def process_noise(x, dt):
    _, _, theta, _, _ = x
    rng = np.random.default_rng()
    lateral_noise = rng.normal(0, 0.05) * dt
    forward_noise = rng.normal(0, 0.2) * dt
    x_noise = lateral_noise * np.sin(theta) + forward_noise * np.cos(theta)
    y_noise = lateral_noise * np.cos(theta) + forward_noise * np.sin(theta)
    theta_noise = rng.normal(0, np.pi/4) * dt
    v_noise = rng.normal(0, 0.3) * dt
    return np.array([x_noise, y_noise, theta_noise, v_noise])

