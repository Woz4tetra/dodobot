from .config import Config


class RobotConfig(Config):
    def __init__(self):
        self.pid_ks = {}
        self.check_ready_timeout = 5.0
        self.write_timeout = 1.0
        self.packet_read_timeout = 0.15

        self.drive_command_timeout = 30.0
        self.drive_command_update_delay = 1.0 / 30.0

        self.joystick_deadzone = 0.1
        self.max_joy_val = 1.0

        self.shutdown_time_limit = 3.0

        self.write_date_delay = 0.5

        self.stepper_max_speed = 31250000 * 8
        self.stepper_max_accel = 1250000 * 8
        self.drive_max_speed = 6800.0
        self.drive_min_speed = 2500.0

        self.gripper_open = 0
        self.gripper_closed = 180

        self.breakout_level_config = "########\n##oooo##\n#oo##oo#\n########"

        super(RobotConfig, self).__init__("robot.yaml")

    def to_dict(self):
        return {
            "pid_ks": self.pid_ks,
            "check_ready_timeout": self.check_ready_timeout,
            "write_timeout": self.write_timeout,
            "packet_read_timeout": self.packet_read_timeout,
            "drive_command_timeout": self.drive_command_timeout,
            "drive_command_update_delay": self.drive_command_update_delay,
            "joystick_deadzone": self.joystick_deadzone,
            "max_joy_val": self.max_joy_val,
            "shutdown_time_limit": self.shutdown_time_limit,
            "write_date_delay": self.write_date_delay,
            "stepper_max_speed": self.stepper_max_speed,
            "drive_max_speed": self.drive_max_speed,
            "drive_min_speed": self.drive_min_speed,
            "gripper_open": self.gripper_open,
            "gripper_closed": self.gripper_closed,
            "breakout_level_config": self.breakout_level_config,
        }
