import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from data_loader import iter_bag, get_key


def yaw_from_quat(quat):
    r_mat = Rotation.from_quat([quat["x"], quat["y"], quat["z"], quat["w"]])
    return r_mat.as_euler("xyz")[2]


def header_to_stamp(header):
    return header["secs"] + header["nsecs"] * 1E-9


OBJECT_NAMES = [
    "BACKGROUND",
    "cozmo_cube",
    "blue_cut_sphere",
    "red_cut_sphere",
    "blue_low_bin",
    "red_low_bin",
    "blue_cube",
    "red_cube",
]


class State:
    def __init__(self):
        self.stamp = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.t = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vt = 0.0

    def __str__(self):
        return f"(x={self.x}, y={self.y}, z={self.z}, t={self.t}, vx={self.vx}, vy={self.vy}, vz={self.vz}, vt={self.vt}) @ {self.stamp}"


odom_xs = []
odom_ys = []

detect_xs = []
detect_ys = []

path = "data/objects_2021-01-06-23-36-19.json"
# path = "data/objects_2021-01-06-23-37-06.json"

for timestamp, topic, msg in iter_bag(path):
    if topic == "/dodobot/cmd_vel":
        state = State()
        state.stamp = timestamp
        state.vx = get_key(msg, "linear.x")
        state.vy = get_key(msg, "linear.y")
        state.vt = get_key(msg, "angular.z")
        # print("cmd:", state)

    elif topic == "/dodobot/odom":
        state = State()
        state.stamp = header_to_stamp(get_key(msg, "header.stamp"))
        state.x = get_key(msg, "pose.pose.position.x")
        state.y = get_key(msg, "pose.pose.position.y")
        state.t = yaw_from_quat(get_key(msg, "pose.pose.orientation"))

        state.vx = get_key(msg, "twist.twist.linear.x")
        state.vy = get_key(msg, "twist.twist.linear.y")
        state.vt = get_key(msg, "twist.twist.angular.z")

        odom_xs.append(state.x)
        odom_ys.append(state.y)

        # print("odom:", state)

    elif topic == "/dodobot/detections":
        stamp = header_to_stamp(get_key(msg, "header.stamp"))
        detections = get_key(msg, "detections")
        for detection in detections.values():
            state = State()
            state.stamp = stamp
            object_id = get_key(detection, "results.0.id")
            object_label = OBJECT_NAMES[object_id]
            state.x = get_key(detection, "results.0.pose.pose.position.x")
            state.y = get_key(detection, "results.0.pose.pose.position.y")
            state.z = get_key(detection, "results.0.pose.pose.position.z")

            # print(get_key(detection, "header.frame_id"))
            # print("detect %s: %s" % (object_label, state))

            if object_label == "blue_cut_sphere":
                detect_xs.append(state.x)
                detect_ys.append(state.y)

plt.plot(detect_xs, detect_ys)
plt.plot(odom_xs, odom_ys)
plt.show()
