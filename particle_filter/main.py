import math
import time
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
        self.u = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # input state vector

        self.prev_stamp = None

        self.u[4] = self.friction
        self.u[5] = self.friction

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
        self.u[0] = -self.odom_vx + self.cmd_vx
        self.u[1] = -self.odom_vy + self.cmd_vy
        # self.u[0] = self.cmd_vx
        # self.u[1] = self.cmd_vy
        # self.u[0] = -self.odom_vx
        # self.u[1] = -self.odom_vy

    def dt(self, timestamp):
        if self.prev_stamp is None:
            self.prev_stamp = timestamp
        dt = timestamp - self.prev_stamp
        self.prev_stamp = timestamp
        return dt


def draw_pf(pf, plot_w, plot_h, z, odom_state, dt):
    try:
        mu, var = pf.estimate()
    except ZeroDivisionError:
        return
    plot_pf(pf, plot_w, plot_h, weights=False)
    plt.scatter(mu[0], mu[1], color='g', s=100)
    plt.plot(z[0], z[1], marker='*', color='r', ms=10)
    if odom_state is not None:
        plt.plot(odom_state.x, odom_state.y, marker='*', color='b', ms=10)

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
    for state in states:
        if state.type == "blue_cut_sphere":
            draw_measurements[0].append(state.x)
            draw_measurements[1].append(state.y)
    plt.plot(draw_measurements[0], draw_measurements[1], marker='*', color='r', ms=10)

    meas_std_val = 0.01
    pf = ParticleFilter(500, meas_std_val)

    input_vector = InputVector()
    u_std = [0.005, 0.005, 0.005, 0.0, 0.0, 0.0]

    plot_w = 3.0
    plot_h = 3.0

    dt = 0.01  # plot pause timer
    plt.figure(2)
    plot_pf(pf, plot_w, plot_h, weights=False)
    plt.pause(dt)
    plt.ion()
    input()

    z = [0.0, 0.0, 0.0]  # measurement state vector
    odom_state = None
    # cmd_vel_state = None

    sim_start_t = states[0].stamp
    real_start_t = time.time()

    resample_timer = 0.0
    resample_timeout = 0.25

    for state in states:
        if state.type == "odom":
            input_vector.update_odom(state)
            pf.predict(input_vector.u, u_std, input_vector.dt(state.stamp))
            odom_state = state
        elif state.type == "cmd_vel":
            input_vector.update_cmd_vel(state)
            print(input_vector.u)
            pf.predict(input_vector.u, u_std, input_vector.dt(state.stamp))
            # cmd_vel_state = state
        elif state.type == "blue_cut_sphere":
            z[0] = state.x
            z[1] = state.y
            z[2] = state.z
            pf.update(z)

        if neff(pf.weights) < pf.N / 2:
            pf.resample()

        sim_time = state.stamp
        real_time = time.time()
        sim_duration = sim_time - sim_start_t
        real_duration = real_time - real_start_t

        if sim_duration >= real_duration:
            draw_pf(pf, plot_w, plot_h, z, odom_state, dt)

    plt.ioff()
    plt.show()


if __name__ == '__main__':
    main()
