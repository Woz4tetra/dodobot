from .config import Config


class JoystickConfig(Config):
    def __init__(self):
        self.path = "/dev/input/js0"

        super(JoystickConfig, self).__init__("joystick.yaml")

    def to_dict(self):
        return {
            "path": self.path,
        }
