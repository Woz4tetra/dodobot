import math
import numpy as np
import matplotlib.pyplot as plt

# physical_parameters = dict(
#     la=0.029,
#     lb=0.01775,
#     lc=0.024,
#     ld=0.001,
# )

physical_parameters = dict(
    la=0.042,
    lb=0.01775,
    lc=0.024,
    ld=0.001,
)

def get_spring_len(theta, params):
    # meters:
    la = params["la"]
    lb = params["lb"]
    lc = params["lc"]
    ld = params["ld"]

    x0 = -ld
    y0 = -lc
    x1 = la * math.cos(theta) - lb * math.sin(theta)
    y1 = -la * math.sin(theta) - lb * math.cos(theta)

    theta_spring = math.atan2(y1 - y0, x1 - x0)
    l = (x1 - x0) / math.cos(theta_spring)

    return l, theta_spring, ((x0, y0), (x1, y1))


def motor_normal_force(l, y1):
    k = 1353  # N/m
    x0 = 0.0119
    lm = 0.065

    return k * (l - x0) * (abs(y1) / lm)


def print_results(theta):
    l, theta_spring, ((x0, y0), (x1, y1)) = get_spring_len(math.radians(theta), physical_parameters)
    motor_F = motor_normal_force(l, y1)
    dual_motor_F = motor_F * 2

    print("%0.1f\t%0.3f\t%0.3f" % (theta, l, dual_motor_F))



def main():
    print_results(9.0)
    print_results(12.1)
    print_results(22.0)

    plt.figure(1)
    plt.plot([0.0], [0.0], 'x')
    plt.plot([0.065], [0.0], 'x')

    thetas = np.linspace(0.0, math.radians(22.0), 50)
    ls = []
    forces = []
    for theta in thetas:
        l, theta_spring, ((x0, y0), (x1, y1)) = get_spring_len(theta, physical_parameters)
        motor_F = motor_normal_force(l, y1)
        ls.append(l)
        forces.append(motor_F)

        plt.plot([x0, x1], [y0, y1])

    plt.figure(2)
    # plt.plot(thetas, ls)
    plt.plot(np.degrees(thetas), forces)


    plt.show()

main()
