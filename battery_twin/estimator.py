"""Extended Kalman filter: keeps the digital twin synchronised with the sensors
and acts as a virtual sensor for the unmeasured cell-core temperature."""
import casadi as ca
import numpy as np

from .model import ND, NU, NX, PackModel

# measured: cell surface temperature, coolant temperature, SOC (BMS coulomb counting)
C = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)


class EKF:
    def __init__(self, model: PackModel, dt, x0, P0=(25.0, 1.0, 1.0, 1e-4),
                 Q=(2e-3, 2e-3, 2e-3, 1e-8), R=(0.1 ** 2, 0.1 ** 2, 0.003 ** 2), substeps=2):
        self.model, self.dt = model, dt
        x, u, d = ca.SX.sym("x", NX), ca.SX.sym("u", NU), ca.SX.sym("d", ND)
        xn = model.step(x, u, d, dt, substeps)
        self.F = ca.Function("F", [x, u, d], [xn, ca.jacobian(xn, x)])
        self.x = np.asarray(x0, float)
        self.P = np.diag(P0)
        self.Q, self.R = np.diag(Q) * dt, np.diag(R)

    def predict(self, u, d):
        xn, A = self.F(self.x, u, d)
        self.x = xn.full().ravel()
        A = A.full()
        self.P = A @ self.P @ A.T + self.Q

    def update(self, y):
        S = C @ self.P @ C.T + self.R
        K = self.P @ C.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.asarray(y) - C @ self.x)
        self.P = (np.eye(NX) - K @ C) @ self.P
        return self.x
