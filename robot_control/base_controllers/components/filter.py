import numpy as np

class SecondOrderFilter():
    def __init__(self, size):
        self.filter_1 = np.zeros(size)
        self.filter_2 = np.zeros(size)
        self.dt = 0

    def initFilter(self, q, dt):
        self.filter_1 = np.copy(q)
        self.filter_2 = np.copy(q)
        self.dt = dt

    def filter(self, input, settling_time):
        gain = self.dt / (0.1 * settling_time + self.dt)
        self.filter_1 = (1 - gain) * self.filter_1 + gain * input
        self.filter_2 = (1 - gain) * self.filter_2 + gain * self.filter_1
        return self.filter_2


if __name__ == '__main__':

    filt = SecondOrderFilter(3)
    dt = 0.01
    q0 = np.array([1.,1.,1.])
    filt.initFilter(q0, dt)
    time = 0
    q1 = q0 + np.array([2., 2., 2.])
    for i in range(1000):
        print(time)
        print(filt.filter(q1, 0.5))
        time += dt

