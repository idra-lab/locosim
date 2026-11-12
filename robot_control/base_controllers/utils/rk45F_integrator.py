import numpy as np
from termcolor import colored
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt #need to import as last to avoid issues with qt
np.set_printoptions(threshold=np.inf, precision = 9, linewidth = 10000, suppress = True)

#https://stoccodavide.github.io/indigo/api-matlab/class_a00231.html#exhale-class-a00231
class RK45F_step:
    def __init__(self,m_ode,  adaptive_step=False):
        # Weights for the 4th-order accurate solution
        self.m_b = np.array([47./450., 0., 12./25., 32./225., 1./30., 6./25.])
        # Weights for the 3rd-order embedded solution
        self.m_b_e = np.array([1./9., 0., 9./20., 16./45., 1./12., 0.])
        self.m_c = np.array([0., 2. / 9., 1. / 3., 3. / 4., 1., 5. / 6.]) # Coefficients for intermediate stages
        self.m_A = np.array([[0.,      0.,        0.,      0.,     0.,     0.],
                             [2./9.,    0.,        0.,      0.,     0.,     0.],
                             [1./12.,   1./4.,      0.,      0.,     0.,     0.],
                             [69./128., -243./128., 135./64., 0.,     0.,     0.],
                             [-17./12., 27./4.,     -27./5.,  16./15., 0.,     0.],
                             [65./432., -5./16.,    13./16.,  4./27.,  5./144., 0. ]]) #Matrix \f$ \mathbf{A} \f$ (lower triangular matrix).
        self.m_adaptive_step = adaptive_step
        self.m_A_tol =  1e-6  # Absolute tolerance
        self.m_R_tol = 1e-4  # Relative tolerance
        self.m_order = 4  # Order of the method
        self.m_fac =0.9 #Safety factor for adaptive step.
        self.m_fac_max = 3.5  # Maximum safety factor for adaptive step.
        self.m_fac_min = 0.2  #Minimum safety factor for adaptive step.
        self.m_ode = m_ode  # ODE system object with methods f (explicit evaluation)
        self.tol = 1e-10

    def step(self, x_k,  t_k=None, d_t=0.001, *args, **kwargs):
        """
        Perform a single integration step.

        Parameters:
            x_k: Current state vector.
            x_dot_k: Current derivative vector.
            t_k: Current time.
            d_t: Current time step.

        Returns:
            x_out: State vector at the next step.
            x_dot_out: Derivative vector at the next step.
            d_t_star: Suggested time step for the next step.
            ierr: Error code (0 if successful, >0 if an error occurs).
        """
        # Solve the system to obtain K
        K = self.solve_step(x_k, t_k, d_t, *args, **kwargs)

        # Suggested time step for the next advancing step
        d_t_star = d_t

        # Perform the step and obtain x_k+1
        x_out = x_k + d_t * K @ self.m_b

        # Adapt next time step
        if self.m_adaptive_step and self.m_b_e is not None:
            x_e = x_k + d_t * K @ self.m_b_e
            d_t_star = self.adapt_step(x_out, x_e, d_t_star)

        return x_out,  d_t_star


    def solve_step(self, x_k, t_k=None, d_t=0.001, *args, **kwargs):
        """
        Solve the system to compute the stage values K.

        Parameters:
            x_k: Current state vector.
            t_k: Current time.
            d_t: Time step.

        Returns:
            out: Computed K matrix (each column corresponds to a stage).
            ierr: Error code (0 if successful, >0 if an error occurs).
        """
        n_states = len(x_k)
        nc = len(self.m_c)  # Number of stages nodi
        K = np.zeros((n_states, nc))  # Initialize K matrix by replicating K_0

        for i in range(nc):
            # Compute the node state
            x_i = self.step_node(i, x_k, K, d_t)
            if t_k is not None:
                K[:, i] = self.m_ode(x_i, t_k + self.m_c[i] * d_t, *args, **kwargs)
            else:
                K[:, i] = self.m_ode(x_i, *args, **kwargs)
        return K


    def step_node(self, i, x_k, K, d_t):
        """
        Compute the intermediate node for the Runge-Kutta method.

        Parameters:
            i: Current stage index (1-based index)
            x_k: State at the current step
            K: Matrix of stage derivatives
            d_t: Time step

        Returns:
            out: Intermediate node value
        """
        # Initialize the node computation
        out = np.zeros_like(x_k)

        # Accumulate contributions from previous stages
        for j in range(i - 1):  # Python uses 0-based indexing
            out += self.m_A[i - 1, j] * K[:, j]  # Adjust indices for 0-based indexing

        # Compute the node
        out = x_k + out * d_t
        return out

    def adapt_step(self, x_h, x_l, d_t):
        """
        Adapt the time step based on the error between high and low-order approximations.

        Parameters:
            x_h: High-order approximation
            x_l: Low-order approximation
            d_t: Current time step

        Returns:
            out: Suggested time step for the next iteration
        """
        # Compute the error with 2-norm
        r = (x_h - x_l) / (self.m_A_tol + 0.5 * self.m_R_tol * (np.abs(x_h) + np.abs(x_l)))
        e = np.linalg.norm(r, 2) / len(x_h)

        # Compute the suggested time step
        q = self.m_order + 1
        out = d_t * min(self.m_fac_max, max(self.m_fac_min, self.m_fac * (1 / e) ** (1 / q)))

        return out


def f(x, t):
    a = 4
    c = 1
    x_next = np.zeros(2)
    x_next[0] = x[0] * (a - x[1])
    x_next[1] = x[1] * (c * x[0] - x[1])
    return x_next


if __name__ == '__main__':

    # Example instantiation
    rk = RK45F_step( m_ode=f,   adaptive_step=True)

    dt = 0.01
    d_t_star = dt
    t_end = 10.
    pose_init = np.array([2., 1.])
    number_of_steps = np.int32(t_end / dt)

    time = 0.
    sim_counter = 0
    pose_log = np.full((2, 20000), np.nan)
    time_log = np.full((20000), np.nan)
    x = pose_init
    while time < t_end:
        if np.mod(time, 1) == 0:
            print(colored(f"TIME: {time}", "red"))
        x, d_t_star = rk.step(x, time, d_t_star)
        time = np.round(time + d_t_star, 4)
        time_log[sim_counter] = time
        pose_log[:,sim_counter] = x
        sim_counter += 1

    # #TODO fix this to get integration on the mesh
    # dt_grid = 0.01
    # dt_s = dt_grid
    # t_end = 10.
    # pose_init = np.array([2., 1.])
    # t_s = 0.
    # time_grid = 0.
    # sim_counter = 0
    # pose_log = np.full((2, 20000), np.nan)
    # time_log = np.full((20000), np.nan)
    # x_s = pose_init
    # while True:
    #     if np.mod(t_s, 1) == 0:
    #         print(colored(f"TIME: {t_s}", "red"))
    #     x_s, d_t_star = rk.stepRKF45(x_s, t_s, dt_s)
    #
    #     # updte the current ts
    #     t_s = np.round(t_s + d_t_star, 4)
    #     time_grid_next = time_grid + dt_grid
    #
    #     # Saturate the suggested timestep
    #     is_on_mesh_point = abs(t_s - time_grid_next) < rk.tol
    #     is_saturation = (t_s) > (time_grid_next + rk.tol)
    #     if (rk.m_adaptive_step and not is_on_mesh_point and is_saturation):
    #         dt_tmp = d_t_star
    #         dt_s = time_grid_next - t_s
    #     else:
    #         dt_s = d_t_star
    #
    #     # Store solution if the step is a mesh point
    #     if (not rk.m_adaptive_step or is_on_mesh_point):
    #         # Update temporaries
    #         sim_counter = sim_counter + 1
    #         dt_s = dt_tmp
    #         # Update outputs
    #         x_out = x_s
    #         # Check if the current step is the last one
    #         if (abs(t_s - t_end) < rk.tol):
    #             break
    #
    # time_log[sim_counter] = t_s
    # pose_log[:, sim_counter] = x_out



    fig = plt.figure()
    fig.suptitle("lin states", fontsize=20)
    plt.subplot(2, 1, 1)
    plt.ylabel("X")
    plt.plot(time_log[:-1], pose_log[0, :-1], "o", lw=3, color='blue')
    plt.grid()
    plt.subplot(2, 1, 2)
    plt.ylabel("Y")
    plt.plot(time_log[:-1], pose_log[1, :-1], "o", lw=3, color='blue')
    plt.grid()
    plt.ylim([-10, 10])