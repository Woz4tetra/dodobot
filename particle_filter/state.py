
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
        self.type = ""
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
        return f"<{self.type}>(x={self.x}, y={self.y}, z={self.z}, t={self.t}, vx={self.vx}, vy={self.vy}, vz={self.vz}, vt={self.vt}) @ {self.stamp}"

