import numpy as np
from scipy import integrate

x0 = np.array([0, 0, 0, 0])
P0 = np.zeros((4, 4))
Q = np.eye(4) * 1.0
R = np.eye(4) * 1.0

def f(x, u, dt):
    def dx_dt(t, x, u):
        x_pos, y_pos, theta = x
        v, turn_radius = u

        w = v / turn_radius if turn_radius != 0 else 0

        dx_pos = v * np.cos(theta)
        dy_pos = v * np.sin(theta)
        dtheta = w

        return [dx_pos, dy_pos, dtheta]

    sol = integrate.solve_ivp(dx_dt, [0, dt], x[0:3], args=(u,), method='RK45')
    next_x = sol.y[:, -1]
    next_x[2] = (next_x[2] + np.pi) % (2 * np.pi) - np.pi
    next_x = np.append(next_x, u[0])
    return next_x

def F(x, u, dt):
    x_pos, y_pos, theta, _ = x
    v, turn_radius = u

    curvature = 1/turn_radius if turn_radius != 0 else 0

    F = np.eye(4)
    A = np.array([
        [0, 0, -v * np.sin(theta), np.cos(theta)],
        [0, 0, v * np.cos(theta), np.sin(theta)],
        [0, 0, 0, curvature],
        [0, 0, 0, 0]
    ]) * dt
    F += A
    return F
