import numpy as np
from numba import jit
import time

@jit(nopython=True, cache=True)
def solve_coefficients(t1, x0, v0, a0, x1, v1, a1):
    A = np.zeros((3, 3), dtype=np.float64)
    A[0, 0] = t1 ** 3
    A[0, 1] = t1 ** 4
    A[0, 2] = t1 ** 5
    A[1, 0] = 3 * t1 ** 2
    A[1, 1] = 4 * t1 ** 3
    A[1, 2] = 5 * t1 ** 4
    A[2, 0] = 6 * t1
    A[2, 1] = 12 * t1 ** 2
    A[2, 2] = 20 * t1 ** 3

    b = np.zeros(3, dtype=np.float64)
    b[0] = x1 - x0 - v0 * t1 - a0 * t1 ** 2 / 2
    b[1] = v1 - v0 - a0 * t1
    b[2] = a1 - a0

    return np.linalg.solve(A, b)

@jit(nopython=True, cache=True)
def precompute_powers(t):
    t2 = t ** 2
    t3 = t ** 3
    t4 = t ** 4
    t5 = t ** 5
    return t2, t3, t4, t5

@jit(nopython=True, cache=True)
def calc_xt(a0, a1, a2, a3, a4, a5, t, t2, t3, t4, t5):
    xt = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5
    return xt

@jit(nopython=True, cache=True)
def calc_dxt(a1, a2, a3, a4, a5, t, t2, t3, t4):
    dxt = a1 + 2 * a2 * t + 3 * a3 * t2 + 4 * a4 * t3 + 5 * a5 * t4
    return dxt

@jit(nopython=True, cache=True)
def calc_ddxt(a2, a3, a4, a5, t, t2, t3):
    ddxt = 2 * a2 + 6 * a3 * t + 12 * a4 * t2 + 20 * a5 * t3
    return ddxt

@jit(nopython=True, cache=True)
def calc_dddxt(a3, a4, a5, t, t2):
    dddxt = 6 * a3 + 24 * a4 * t + 60 * a5 * t2
    return dddxt

class QuinticPolynomial(object):
    def __init__(self, x0, v0, a0, x1, v1, a1, t0, t1, DT):
        self.update(x0, v0, a0, x1, v1, a1, t0, t1, DT)

    def update(self, x0, v0, a0, x1, v1, a1, t0, t1, DT):
        X = solve_coefficients(t1, x0, v0, a0, x1, v1, a1)

        self.a0 = x0
        self.a1 = v0
        self.a2 = a0 / 2.0
        self.a3 = X[0]
        self.a4 = X[1]
        self.a5 = X[2]

        # Create time span from t0 to t1 with sampling time DT
        self.t = np.arange(t0, t1, DT)
        # Precompute powers of t
        self.t2, self.t3, self.t4, self.t5 = precompute_powers(self.t)

    def calc_xt(self):
        return calc_xt(self.a0, self.a1, self.a2, self.a3, self.a4, self.a5, self.t, self.t2, self.t3, self.t4, self.t5)

    def calc_dxt(self):
        return calc_dxt(self.a1, self.a2, self.a3, self.a4, self.a5, self.t, self.t2, self.t3, self.t4)

    def calc_ddxt(self):
        return calc_ddxt(self.a2, self.a3, self.a4, self.a5, self.t, self.t2, self.t3)

    def calc_dddxt(self):
        return calc_dddxt(self.a3, self.a4, self.a5, self.t, self.t2)

if __name__ == '__main__':
    # Example usage
    t0 = 0.0
    t1 = 1.0
    DT = 0.01  # Example time step

    # Measure time for solving coefficients and initial calculations
    start_time = time.time()
    qp = QuinticPolynomial(x0=0, v0=0, a0=0, x1=1, v1=0, a1=0, t0=t0, t1=t1, DT=DT)
    xt = qp.calc_xt()
    dxt = qp.calc_dxt()
    ddxt = qp.calc_ddxt()
    dddxt = qp.calc_dddxt()
    initial_time = time.time() - start_time

    # Measure time for subsequent calculations
    start_time = time.time()
    xt = qp.calc_xt()
    dxt = qp.calc_dxt()
    ddxt = qp.calc_ddxt()
    dddxt = qp.calc_dddxt()
    subsequent_time = time.time() - start_time

    print("Initial computation time (including JIT compilation): {:.6f} seconds".format(initial_time))
    print("Subsequent computation time: {:.6f} seconds".format(subsequent_time))

    # print(xt)
    # print(dxt)
    # print(ddxt)
    # print(dddxt)

    # Update the polynomial with new parameters
    qp.update(x0=0, v0=0, a0=0, x1=2, v1=0, a1=0, t0=t0, t1=t1, DT=DT)

    # Measure time for calculations after update
    start_time = time.time()
    xt = qp.calc_xt()
    dxt = qp.calc_dxt()
    ddxt = qp.calc_ddxt()
    dddxt = qp.calc_dddxt()
    update_time = time.time() - start_time

    print("Computation time after update: {:.6f} seconds".format(update_time))

    # print(xt)
    # print(dxt)
    # print(ddxt)
    # print(dddxt)