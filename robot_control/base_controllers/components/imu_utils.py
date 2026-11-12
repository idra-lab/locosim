import numpy as np
import pinocchio as pin

class IMU_utils:
    def __init__(self, timeout = 2000, dt = 0.002):
        self.counter = 0
        self.timeout = timeout
        self.dt = dt
        self.IMU_accelerometer_bias = np.zeros(3)
        self.IMU_accelerometer_bias_log = np.full((3, timeout), np.nan)
        self.g0 = np.array([0., 0., 9.806])

        # filters data
        self.alpha_accelerometer = 0.98
        self.alpha_velocity = np.array([0.21, 0.21, 0.21]) # X Y should be equal
        self.baseLinTwistImuW = np.zeros(3)


    def IMU_bias_estimation(self, b_R_w, IMU_accelerometer):
        if any(np.isnan(IMU_accelerometer)):
            return
        self.IMU_accelerometer_bias = self.alpha_accelerometer * self.IMU_accelerometer_bias + \
                (1 - self.alpha_accelerometer) * (IMU_accelerometer - b_R_w @ self.g0)
        self.IMU_accelerometer_bias_log[:, self.counter] = self.IMU_accelerometer_bias
        self.counter += 1


    def compute_lin_vel(self, W_lin_acc, loop_dt):
        if self.counter < self.timeout:
            self.baseLinTwistImuW[:] = 0.
        # this is a dampled leaky integrator for velocity vdot = The damping term gradually pulls the velocity estimate toward zero;
        # loop_dt * W_lin_acc : adds the velocity change caused by acceleration during this short time
        # the velocity is slightly shrinked by a factor self.alpha_velocity[0] * loop_dt. That shrinkage is the damping, which can help prevent velocity from drifting indefinitely because of sensor bias or noise.
        self.baseLinTwistImuW[0] = (1 - self.alpha_velocity[0] * loop_dt) * self.baseLinTwistImuW[0] + loop_dt * W_lin_acc[0]
        self.baseLinTwistImuW[1] = (1 - self.alpha_velocity[1] * loop_dt) * self.baseLinTwistImuW[1] + loop_dt * W_lin_acc[1]
        self.baseLinTwistImuW[2] = (1 - self.alpha_velocity[2] * loop_dt) * self.baseLinTwistImuW[2] + loop_dt * W_lin_acc[2]


