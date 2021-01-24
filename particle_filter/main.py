import math
import time
import numpy as np
from plotter import ParticleFilterPlotter

from state_loader import read_pkl
from particle_filter import ParticleFilter


class InputVector:
    def __init__(self):
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_vt = 0.0
        self.odom_t = 0.0
        self.dist = 0.0
        self.friction = 0.0
        self.u = [0.0, 0.0, 0.0]  # input state vector: dx, dy, dz

        self.prev_stamp = None

        # self.u[4] = self.friction
        # self.u[5] = self.friction

    def update(self, state):
        dt = self.dt(state.stamp)
        angle = -state.t
        rot_mat = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])
        velocity = np.array([state.vx, state.vy])
        rot_vel = np.dot(rot_mat, velocity)
        self.odom_vx = -rot_vel[0]
        self.odom_vy = -rot_vel[1]  # ~0.0
        self.odom_vt = -state.vt

        self.u[0] = self.odom_vx
        self.u[1] = self.odom_vt

        return dt

    def dt(self, timestamp):
        if self.prev_stamp is None:
            self.prev_stamp = timestamp
        dt = timestamp - self.prev_stamp
        self.prev_stamp = timestamp
        return dt


def main():
    # path = "data/objects_2021-01-06-23-36-19.json"
    path = "data/objects_2021-01-06-23-37-06.json"

    # repickle = True
    repickle = False
    states = read_pkl(path, repickle)
    run_pf(states)


def run_pf(states):
    initial_state = None
    initial_range = [1.0, 1.0, 1.0]
    for state in states:
        if state.type == "blue_cut_sphere":
            if initial_state is None:
                initial_state = [state.x, state.y, state.z]
                break
    meas_std_val = 0.01
    pf = ParticleFilter(250, meas_std_val)
    plotter = ParticleFilterPlotter(pf, 3.0, 3.0, 3.0)

    pf.create_uniform_particles(initial_state, initial_range)

    input_vector = InputVector()
    u_std = [0.005, 0.005, 0.005]

    z = None  # measurement state vector
    prev_z_update_t = 0.0

    sim_start_t = states[0].stamp
    real_start_t = time.time()

    plotter.init()
    input()

    for state in states:
        sim_time = state.stamp
        real_time = time.time()
        sim_duration = sim_time - sim_start_t
        real_duration = real_time - real_start_t

        if state.type == "odom":
            dt = input_vector.update(state)
            pf.predict(input_vector.u, u_std, dt)
            plotter.update_odom(state)
        elif state.type == "blue_cut_sphere":
            z = [state.x, state.y, state.z]
            pf.update(z)
            prev_z_update_t = sim_duration
            plotter.update_measure(state)

        pf.check_resample()

        if sim_duration - prev_z_update_t > 0.5:
            z = None  # hide measurement if one hasn't appeared in a while

        if sim_duration >= real_duration:  # if simulation time has caught up to real time, spend some time drawing
            plotter.draw()
    plotter.stop()

if __name__ == '__main__':
    main()
