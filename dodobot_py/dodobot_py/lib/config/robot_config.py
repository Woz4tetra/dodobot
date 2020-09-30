from .config import Config


class RobotConfig(Config):
    def __init__(self):
        self.pid_ks = {}
        self.check_ready_timeout = 5.0
        self.write_timeout = 1.0
        self.packet_start_timeout = 0.15

        self.drive_command_timeout = 30.0
        self.drive_command_update_delay = 1.0 / 30.0

        self.joystick_deadzone = 0.1
        self.max_joy_val = 1.0

        self.shutdown_time_limit = 3.0

        self.write_date_delay = 0.5

        self.stepper_max_speed = 31250000 * 8
        self.drive_max_speed = 6800.0
        self.drive_min_speed = 2500.0

        super(RobotConfig, self).__init__("robot.yaml")

    def to_dict(self):
        return {
            "pid_ks": self.pid_ks,
            "check_ready_timeout": self.check_ready_timeout,
            "write_timeout": self.write_timeout,
            "packet_start_timeout": self.packet_start_timeout,
            "drive_command_timeout": self.drive_command_timeout,
            "drive_command_update_delay": self.drive_command_update_delay,
            "joystick_deadzone": self.joystick_deadzone,
            "max_joy_val": self.max_joy_val,
            "shutdown_time_limit": self.shutdown_time_limit,
            "write_date_delay": self.write_date_delay,
            "stepper_max_speed": self.stepper_max_speed,
            "drive_max_speed": self.drive_max_speed,
            "drive_min_speed": self.drive_min_speed,
        }
