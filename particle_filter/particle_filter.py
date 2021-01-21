import numpy as np
from numpy.random import randn, random, uniform
import scipy.stats
import matplotlib.pyplot as plt


class ParticleFilter:
    def __init__(self, N, measure_std_error):
        self.num_states = 6  # x, y, z, vx, vy, vz
        self.particles = np.empty((N, self.num_states))
        self.N = N
        self.R = measure_std_error

        # distribute particles randomly with uniform weight
        self.weights = np.empty(N)
        # self.weights.fill(1./N)
        '''self.particles[:, 0] = uniform(0, x_dim, size=N)
        self.particles[:, 1] = uniform(0, y_dim, size=N)
        self.particles[:, 2] = uniform(0, 2*np.pi, size=N)'''

    def create_uniform_particles(self, initial_state, state_range):
        assert len(initial_state) == self.num_states
        assert len(state_range) == self.num_states

        for state_num in range(self.num_states):
            min_val = initial_state[state_num] - state_range[state_num]
            max_val = initial_state[state_num] + state_range[state_num]
            self.particles[:, state_num] = uniform(min_val, max_val, size=self.N)

    def create_gaussian_particles(self, mean, var):
        for state_num in range(self.num_states):
            self.particles[:, state_num] = mean[state_num] + randn(self.N) * var[state_num]

    def predict(self, u, std, dt):
        """ move according to control input u (velocity of robot and velocity of object)
        with noise std"""

        for state_num in range(self.num_states):
            self.particles[:, state_num] += u[state_num] * dt + randn(self.N) * std[state_num]

    def update(self, z):
        """Update particle filter according to measurement z (object position)"""
        self.weights.fill(1.0)

        # weight according to how far away the particle is from the measurement in x, y, z
        for axis in range(2):
            axis_val = self.particles[:, axis]
            self.weights *= scipy.stats.norm(axis_val, self.R).pdf(z[axis])

        self.weights += 1.e-300  # avoid divide by zero error
        self.weights /= sum(self.weights)  # normalize

    def neff(self):
        return 1.0 / np.sum(np.square(self.weights))

    def resample(self):
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.  # avoid round-off error
        indexes = np.searchsorted(cumulative_sum, random(self.N))

        # resample according to indexes
        self.particles = self.particles[indexes]
        self.weights = self.weights[indexes]
        self.weights /= np.sum(self.weights)  # normalize

    def resample_from_index(self, indexes):
        assert len(indexes) == self.N

        self.particles = self.particles[indexes]
        self.weights = self.weights[indexes]
        self.weights /= np.sum(self.weights)

    def estimate(self):
        """ returns mean and variance """
        pos = self.particles[:, 0:3]
        mu = np.average(pos, weights=self.weights, axis=0)
        var = np.average((pos - mu) ** 2, weights=self.weights, axis=0)

        return mu, var

    def mean(self):
        """ returns weighted mean position"""
        return np.average(self.particles[:, 0:3], weights=self.weights, axis=0)


def neff(weights):
    return 1. / np.sum(np.square(weights))


def systemic_resample(w):
    N = len(w)
    Q = np.cumsum(w)
    indexes = np.zeros(N, 'int')
    t = np.linspace(0, 1 - 1 / N, N) + random() / N

    i, j = 0, 0
    while i < N and j < N:
        while Q[j] < t[i]:
            j += 1
        indexes[i] = j
        i += 1

    return indexes


def plot_pf(pf, wlim=50, hlim=50, weights=True):
    if weights:
        a = plt.subplot(221)
        a.cla()

        plt.xlim(-wlim / 2, wlim / 2)
        # plt.ylim(0, 1)
        a.set_yticklabels('')
        plt.scatter(pf.particles[:, 0], pf.weights, marker='.', s=1, color='k')
        a.set_ylim(bottom=0)

        a = plt.subplot(224)
        a.cla()
        a.set_xticklabels('')
        plt.scatter(pf.weights, pf.particles[:, 1], marker='.', s=1, color='k')
        plt.ylim(-hlim / 2, hlim / 2)
        a.set_xlim(left=0)
        # plt.xlim(0, 1)

        a = plt.subplot(223)
        a.cla()

    else:
        plt.cla()
    plt.scatter(pf.particles[:, 0], pf.particles[:, 1], marker='.', s=1, color='k')
    plt.xlim(-wlim / 2, wlim / 2)
    plt.ylim(-hlim / 2, hlim / 2)
