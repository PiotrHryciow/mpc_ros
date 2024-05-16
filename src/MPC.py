import casadi
import numpy as np


class ModelPredictiveControl(object):
    """
    Special class of MPC for reference tracking (only position [x,y]'), arguments: \n
        N - control horizon \n
        dt - time interval \n
        params - [Lf], where Lf is half on wheelbase in [m] \n
        constraints - list of constaints: [acc_max[m/s^2], acc_min[m/s^2], delta[rad], v_max[m/s], v_min[m/s]], \n
        where delta is max steering angle (in radians) \n
    To solve actual problem use: solve()

    """

    def __init__(self, N: int, dt: float, params: list, constraints: list):

        Lf = params[0]
        self.Q = np.diag([1, 1])
        self.R = np.diag([10])

        # The history states and controls
        self.states_list = np.zeros((4, N + 1))
        self.controls_list = np.zeros((2, N))

        self.opti = casadi.Opti()

        # --------- states ---------
        self.X = self.opti.variable(4, N + 1)
        self.pos_x = self.X[0, :]
        self.pos_y = self.X[1, :]
        self.heading = self.X[2, :]
        self.vel = self.X[3, :]

        # --------- controls ---------
        self.U = self.opti.variable(2, N)
        self.acc = self.U[0, :]
        self.steering_angle = self.U[1, :]

        # --------- reference ---------
        self.pos_ref = self.opti.parameter(2, N + 1)

        # --------- model ---------
        # x[x,y,psi,v], u[a, delta]
        f = lambda x, u: casadi.vertcat(
            *[
                x[0] + x[3] * casadi.cos(x[2]) * dt,
                x[1] + x[3] * casadi.sin(x[2]) * dt,
                x[2] + (x[3] / Lf) * u[1] * dt,
                x[3] + u[0] * dt,
            ]
        )
        # --------- set model equations ---------
        for i in range(N):
            X_next = f(self.X[:, i], self.U[:, i])
            self.opti.subject_to(self.X[:, i + 1] == X_next)

        # --------- objective ---------
        obj = 0
        # for tracking error
        for i in range(1, N + 1):
            state_error = self.pos_ref[:, i] - self.X[0:2, i]
            obj += casadi.mtimes([state_error.T, self.Q, state_error])
        # to minimaze rate of change in steering angle
        for i in range(0, N - 1):
            rate_of_change_error = self.U[1, i + 1] - self.U[1, i]
            obj += casadi.mtimes([rate_of_change_error.T, self.R, rate_of_change_error])
        self.opti.minimize(obj)

        # --------- constraints ---------
        acc_max, acc_min, delta, v_max, v_min = constraints
        self.opti.subject_to(self.opti.bounded(acc_min, self.acc, acc_max))
        self.opti.subject_to(self.opti.bounded(-delta, self.steering_angle, delta))
        self.opti.subject_to(self.opti.bounded(v_min, self.vel, v_max))
        # self.opti.subject_to(self.opti.bounded(-2, self.U, 2))
        # self.opti.subject_to(self.opti.bounded(-10, self.vel, 10))

        # --------- solver ---------
        opts = {"ipopt.print_level": 0, "print_time": 0}
        self.opti.solver("ipopt", opts)

    def solve(self, trajectory, X0):
        """
        solve mpc problem for reference tracking: \n
            trajectory - list of points[x,y]' to follow, list length must be the same as control horizon!!! \n
            X0 - initial states list [pos_x, pos_y, yaw_z, velocity] \n
        returns full states list[pos_x, pos_y, yaw_z, velocity], and controls list[acc, steering_angle]
        over the whole control horizon
        """
        # need to copy whole solver at each iteration, because casadi is not suited for real time,
        # can't dynamicly change "subject_to"
        opti_sol = self.opti.copy()

        # --------- set initial states values ---------
        opti_sol.subject_to(self.X[:, 0] == X0)

        # --------- set value for tracking trajectory ---------
        opti_sol.set_value(self.pos_ref, trajectory)

        # --------- set initial guess for states and controls ---------
        opti_sol.set_initial(self.X, self.states_list)
        opti_sol.set_initial(self.U, self.controls_list)

        # ---------solve ---------
        sol = opti_sol.solve()

        ## obtain the control input
        U_sol = sol.value(self.U)
        X_sol = sol.value(self.X)
        self.states_list = np.hstack((X_sol[:, 1:], X_sol[:, -1:]))
        self.controls_list = np.hstack((U_sol[:, 1:], U_sol[:, -1:]))
        return U_sol, X_sol
