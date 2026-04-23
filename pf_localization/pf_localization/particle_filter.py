import numpy as np
from dataclasses import dataclass

@dataclass
class ParticleFilterParams:
    num_particles = 500
    num_states = 4 # x, y, theta, v, battery
    x0_min = [-30, 5, -np.pi, 0]
    x0_max = [-25, 10, np.pi, 0]

class ParticleFilter:
    def __init__(self, params):
        self.params = params
        self.rng = np.random.default_rng()
        self.n = params.num_states
        self.N = params.num_particles

        self.init_particles()

    def init_particles(self):
        self.particles = list(self.rng.uniform(low=self.params.x0_min, high=self.params.x0_max, size=(self.N,self.n)))
        self.weights = np.ones(self.N) / self.N

    def resample(self, collision_func):
        """
        validity_func: Only resample from particles where this is True
        """
        valid_idxs = []
        for i in range(len(self.particles)):
            if not collision_func(self.particles[i]):
                valid_idxs.append(i)
        valid_weights = self.weights[valid_idxs]
        valid_weights /= np.sum(valid_weights)

        resample_idxs = self.rng.choice(valid_idxs, size=self.N, p=valid_weights)
        self.particles = [self.particles[i] for i in resample_idxs]
        self.weights = np.ones(self.N) / self.N

    def predict(self, f, u, q, dt):
        """
        f: Dynamics function
        u: Control input
        q: Process noise
        """
        pps = []
        for p in self.particles:
            pp = f(p, u, dt)
            pp += q(p, dt)
            pps.append(pp)
        self.particles = pps

    def update(self, y, h, lf):
        """
        y: Observed measurement
        h: Measurement function
        lf: Sensor error likelihood function
        """
        weights = self.weights
        for i in range(len(self.particles)):
            p = self.particles[i]
            expected_measurements, uncertainties = h(p)
            for j in range(len(y)):
                err = y[j] - expected_measurements[j]
                uncertainty = uncertainties[j]
                weights[i] *= lf(err, uncertainty)
        if np.sum(self.weights) == 0:
            weights = np.ones(self.N)
        self.weights = weights / np.sum(weights)

    def map_estimate(self):
        """Maximum A Priori (Mode) Estimate"""
        likeliest_particle_idx = np.argmax(self.weights)
        return self.particles[likeliest_particle_idx]
    
    def mmse_estimate(self):
        """Minimum Mean Squared Error (Mean) Estimate"""
        return np.average(self.particles, weights=self.weights, axis=0)