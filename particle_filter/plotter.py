import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation

from state import State


class ParticleFilterPlotter:
    def __init__(self, pf, x_window, y_window, z_window, plot_delay=0.01):
        self.pf = pf
        self.x_window = x_window
        self.y_window = y_window
        self.z_window = z_window
        self.plot_delay = plot_delay
        self.fig = None
        self.ax = None
        self.odom_state = State()
        self.meas_state = State()
    
    def init(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.tight_layout()
        plt.ion()
        self.fig.show()
    
    def update_odom(self, state):
        self.odom_state = state
    
    def update_measure(self, state):
        self.meas_state = state
    
    def draw(self):
        try:
            mu, var = self.pf.estimate()
        except ZeroDivisionError:
            return
        
        particles = self.base_link_to_odom(self.odom_state, self.pf.particles[:, 0], self.pf.particles[:, 1], self.pf.particles[:, 2])
        plt.cla()
        self.ax.scatter(particles[0], particles[1], particles[2], marker='.', s=1, color='k', alpha=0.5)
        self.ax.set_xlim(-self.x_window / 2, self.x_window / 2)
        self.ax.set_ylim(-self.y_window / 2, self.y_window / 2)
        self.ax.set_zlim(0.0, self.z_window)

        odom_mu = self.base_link_to_odom(self.odom_state, mu[0], mu[1], mu[2])
        self.ax.scatter(odom_mu[0], odom_mu[1], odom_mu[2], color='g', s=25)

        self.ax.scatter(self.odom_state.x, self.odom_state.y, self.odom_state.z, marker='*', color='b', s=25)

        if self.odom_state.stamp - self.meas_state.stamp < 0.25:
            odom_meas = self.base_link_to_odom(self.odom_state, self.meas_state.x, self.meas_state.y, self.meas_state.z)
            self.ax.scatter(odom_meas[0], odom_meas[1], odom_meas[2], marker='*', color='r', s=25)
        # plt.pause(self.plot_delay)
        self.fig.canvas.flush_events()

    def base_link_to_odom(self, state, x, y, z):
        angle = state.t
        # tf_mat = np.array([
        #     [np.cos(angle), -np.sin(angle), state.x],
        #     [np.sin(angle), np.cos(angle), state.y],
        #     [0.0, 0.0,  1.0]
        # ])
        # point = np.array([x, y, 1.0])
        # tf_point = np.dot(tf_mat, point)
        rot_mat = Rotation.from_euler("z", angle)
        
        point = np.array([x, y, z])
        tf_point = np.dot(rot_mat.as_matrix(), point)
        tf_point[0] += state.x
        tf_point[1] += state.y
        tf_point[2] += state.z
        return tf_point

    def stop(self):
        plt.ioff()
        plt.show()

