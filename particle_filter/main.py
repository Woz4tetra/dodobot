import math
import time
import numpy as np
import matplotlib.pyplot as plt
from state_loader import read_pkl
from particle_filter import ParticleFilter, plot_pf, systemic_resample, neff


class InputVector:
    def __init__(self):
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_t = 0.0
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.friction = 0.0
        self.u = [0.0, 0.0, 0.0]  # input state vector

        self.prev_stamp = None

        # self.u[4] = self.friction
        # self.u[5] = self.friction

    def update_odom(self, state):
        self.odom_vx = state.vx
        self.odom_vy = state.vy
        self.odom_t = state.t
        self.update_input()

    def update_cmd_vel(self, state):
        self.cmd_vx = state.vx * math.cos(self.odom_t)
        self.cmd_vy = state.vy * math.cos(self.odom_t)
        self.update_input()

    def update_input(self):
        # self.u[0] = -self.odom_vx + self.cmd_vx
        # self.u[1] = -self.odom_vy + self.cmd_vy
        # self.u[0] = self.cmd_vx
        # self.u[1] = self.cmd_vy
        self.u[0] = -self.odom_vx
        self.u[1] = -self.odom_vy
        # self.u[2] = 0.0  # no Z velocity
        print(self.u)

    def dt(self, timestamp):
        if self.prev_stamp is None:
            self.prev_stamp = timestamp
        dt = timestamp - self.prev_stamp
        self.prev_stamp = timestamp
        return dt


def base_link_to_odom(odom_state, x, y):
    # angle = -odom_state.t
    angle = odom_state.t
    rot_mat = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])
    # rot_mat = np.linalg.inv(rot_mat)
    point = np.array([x, y])
    tf_point = np.dot(rot_mat, point)
    tf_point[0] += odom_state.x
    tf_point[1] += odom_state.y
    return tf_point


def plot_pf_odom(pf, wlim, hlim, odom_state):
    particles = base_link_to_odom(odom_state, pf.particles[:, 0], pf.particles[:, 1])
    plt.cla()
    plt.scatter(particles[0], particles[1], marker='.', s=1, color='k')
    plt.xlim(-wlim / 2, wlim / 2)
    plt.ylim(-hlim / 2, hlim / 2)


def draw_pf_odom(pf, plot_w, plot_h, z, odom_state, dt):
    if odom_state is None:
        return
    try:
        mu, var = pf.estimate()
    except ZeroDivisionError:
        return
    # plot_pf(pf, plot_w, plot_h, weights=False)
    plot_pf_odom(pf, plot_w, plot_h, odom_state)

    odom_mu = base_link_to_odom(odom_state, mu[0], mu[1])
    plt.scatter(odom_mu[0], odom_mu[1], color='g', s=25)

    if z is not None:
        odom_z = base_link_to_odom(odom_state, z[0], z[1])
        plt.plot(odom_z[0], odom_z[1], marker='*', color='r', ms=10)

    plt.plot(odom_state.x, odom_state.y, marker='*', color='b', ms=10)
    # if odom_state is not None:
    #     plt.plot(odom_state.x, odom_state.y, marker='*', color='b', ms=10)

    plt.tight_layout()

    plt.pause(dt)


def draw_pf_base_link(pf, plot_w, plot_h, z, dt):
    try:
        mu, var = pf.estimate()
    except ZeroDivisionError:
        return
    plot_pf(pf, plot_w, plot_h, weights=False)

    plt.scatter(mu[0], mu[1], color='g', s=25)

    if z is not None:
        plt.plot(z[0], z[1], marker='*', color='r', ms=10)

    plt.tight_layout()
    plt.pause(dt)

def main():
    # path = "data/objects_2021-01-06-23-36-19.json"
    path = "data/objects_2021-01-06-23-37-06.json"

    # repickle = True
    repickle = False
    states = read_pkl(path, repickle)

    # t = []
    # vx = []
    # vy = []
    # for state in states:
    #     if state.type == "odom":
    #         vx.append(state.vx)
    #         vy.append(state.vy)
    #         t.append(state.stamp)
    #
    # plt.plot(t, vx, label="vx")
    # plt.plot(t, vy, label="vy")
    # plt.legend()
    # plt.show()
    run_pf(states)


def run_pf(states):
    plt.figure(1)
    draw_measurements = [[], []]
    initial_state = None
    initial_range = [1.0, 1.0, 1.0]
    for state in states:
        if state.type == "blue_cut_sphere":
            draw_measurements[0].append(state.x)
            draw_measurements[1].append(state.y)
            if initial_state is None:
                initial_state = [state.x, state.y, state.z]
    plt.plot(draw_measurements[0], draw_measurements[1], marker='*', color='r', ms=10)

    meas_std_val = 0.01
    pf = ParticleFilter(500, meas_std_val)

    pf.create_uniform_particles(initial_state, initial_range)

    input_vector = InputVector()
    u_std = [0.005, 0.005, 0.005]

    plot_w = 3.0
    plot_h = 3.0

    dt = 0.01  # plot pause timer
    plt.figure(2)

    plt.pause(dt)
    plt.ion()
    input()

    z = None  # measurement state vector
    prev_z_update_t = 0.0
    odom_state = None
    # cmd_vel_state = None

    sim_start_t = states[0].stamp
    real_start_t = time.time()

    for state in states:
        sim_time = state.stamp
        real_time = time.time()
        sim_duration = sim_time - sim_start_t
        real_duration = real_time - real_start_t

        if state.type == "odom":
            input_vector.update_odom(state)
            pf.predict(input_vector.u, u_std, input_vector.dt(state.stamp))
            odom_state = state
        # elif state.type == "cmd_vel":
        #     input_vector.update_cmd_vel(state)
        #     pf.predict(input_vector.u, u_std, input_vector.dt(state.stamp))
        #     # cmd_vel_state = state
        elif state.type == "blue_cut_sphere":
            z = [state.x, state.y, state.z]
            pf.update(z)
            prev_z_update_t = sim_duration

        if neff(pf.weights) < pf.N / 2:
            pf.resample()

        if sim_duration - prev_z_update_t > 0.5:
            z = None

        if sim_duration >= real_duration:
            draw_pf_odom(pf, plot_w, plot_h, z, odom_state, dt)
            # draw_pf_base_link(pf, plot_w, plot_h, z, dt)

    plt.ioff()
    plt.show()


if __name__ == '__main__':
    main()
