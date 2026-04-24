import numpy as np

class UKF:
    """Unscented Kalman Filter"""
    def __init__(self, x0, P0, Q):
        # State
        self.x = x0
        # Dimension
        self.n = x0.shape[0]
        # Information Matrix
        self.P = P0
        # Process Noise
        self.Q = Q

        self.a = 0.5
        self.k = 0
        self.b = 2
        self.l = self.a**2 * (self.n + self.k) - self.n

        self.wm0 = self.l/(self.n+self.l)
        self.wc0 = self.l/(self.n+self.l)+1-self.a**2+self.b
        self.wmi = 1/(2*(self.n+self.l))
        self.wci = self.wmi

        self.wm = np.array([self.wm0] + (2*self.n * [self.wmi]))
        self.wc = np.array([self.wc0] + (2*self.n * [self.wci]))

    def prediction(self, dt, f, _):
        S = np.linalg.cholesky(self.P)
        sp = [self.x]
        for i in range(self.n):
            offset = np.sqrt(self.n + self.l) * S[:,i]
            sp.append(self.x + offset)
            sp.append(self.x - offset)
        spp = [f(x,dt) for x in sp]
        self.x = np.sum([wm * p for wm, p in zip(self.wm, spp)], axis=0)
        P = self.Q.copy()
        for i in range(len(spp)):
            P += self.wc[i] * (spp[i] - self.x)[:,None]@(spp[i] - self.x)[None,:]
        self.P = P

        return self.x, self.P

    def measurement(self, y, h, logger):
        S = np.linalg.cholesky(self.P)
        sp = [self.x]
        for i in range(self.n):
            offset = np.sqrt(self.n + self.l) * S[:,i]
            sp.append(self.x + offset)
            sp.append(self.x - offset)
        innov_cov = np.diag(h(self.x)[1].flatten()).copy()
        spp = [h(p)[0] for p in sp]
        yhat = np.sum([wm * p for wm, p in zip(self.wm, spp)], axis=0)
        cross_cov = np.zeros((self.n, y.shape[0]))
        for i in range(len(spp)):
            innov = spp[i] - yhat
            innov_cov +=  self.wc[i] * innov @ innov.T
            cross_cov += self.wc[i] * (sp[i] - self.x)[:,None] @ innov.T
        K = cross_cov @ np.linalg.inv(innov_cov)

        logger.warn(f"{(K @ (y-yhat)).shape}")
        self.x = self.x + K @ (y - yhat)
        self.P = self.P - (K @ innov_cov @ K.T)

        return self.x, self.P

    def NEES(self, x):
        ex = x - self.x
        NEES = ex.T @ np.linalg.inv(self.P) @ ex
        return NEES

    def pm2sigma(self):
        sig = np.sqrt(np.diag(self.P))
        return [self.x - 2*sig, self.x + 2*sig]
