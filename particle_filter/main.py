import math
import matplotlib.pyplot as plt
from state_loader import read_pkl
from particle_filter import ParticleFilter, plot_pf


class InputVector:
    def __init__(self):
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_t = 0.0
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.friction = -0.25
        self.u = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # input state vector

        self.u[4] = self.friction
        self.u[5] = self.friction

    def update_odom(self, state):
        self.odom_vx = state.vx
        self.odom_vy = state.vy
        self.odom_t = state.t

    def update_cmd_vel(self, state):
        self.cmd_vx = state.vx * math.cos(self.odom_t)
        self.cmd_vy = state.vy * math.cos(self.odom_t)

    def update_input(self):
        self.u[0] = -self.odom_vx + self.cmd_vx
        self.u[1] = -self.odom_vy + self.cmd_vy


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

    plt.figure(1)
    draw_measurements = [[], []]
    for state in states:
        if state.type == "blue_cut_sphere":
            draw_measurements[0].append(state.x)
            draw_measurements[1].append(state.y)
    plt.plot(draw_measurements[0], draw_measurements[1], marker='*', color='r', ms=10)

    u_std_val = 0.1
    meas_std_val = 1.0
    pf = ParticleFilter(1000, meas_std_val)

    input_vector = InputVector()
    u_std = [u_std_val] * pf.num_states

    plot_w = 10.0
    plot_h = 10.0

    dt = 0.01  # plot pause timer
    plt.figure(2)
    plot_pf(pf, plot_w, plot_h, weights=False)
    plt.pause(dt)
    plt.ion()

    z = [0.0, 0.0, 0.0]  # measurement state vector
    odom_state = None
    # cmd_vel_state = None

    for state in states[100:]:
        if state.type == "odom":
            input_vector.update_odom(state)
            pf.predict(input_vector.u, u_std)
            odom_state = state
        elif state.type == "cmd_vel":
            input_vector.update_cmd_vel(state)
            pf.predict(input_vector.u, u_std)
            # cmd_vel_state = state
        elif state.type == "blue_cut_sphere":
            z[0] = state.x
            z[1] = state.y
            z[2] = state.z
            pf.update(z)
            pf.resample()

        draw_pf(pf, plot_w, plot_h, z, odom_state, dt)

    plt.ioff()
    plt.show()


if __name__ == '__main__':
    main()
