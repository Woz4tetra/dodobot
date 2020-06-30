import math

from .config import Config


class RobotConfig(Config):
    def __init__(self):
        self.wheel_radius_cm = 32.5
        self.cm_per_tick = 2.0 * math.pi * self.wheel_radius_cm / 1920.0
        self.max_linear_speed_cps = 915.0
        self.cps_to_cmd = 255.0 / self.max_linear_speed_cps

        self.tof_off_axis_mm = 15.0  # how far the sensor is from the axis of rotation
        self.tof_ground_dist_mm = 28.7  # how far the axis of rotation is off the ground
        self.tof_front_wall_dist_mm = 30.8  # how far the front wall is from the axis of rotation
        self.tof_back_wall_dist_mm = 15.0  # how far the back wall is from the axis of rotation

        self.front_ledge_y_mm = 100
        self.front_obstacle_x_mm = 100
        self.back_ledge_y_mm = 100
        self.back_obstacle_x_mm = 100

        self.front_tilter_servo_num = 0
        self.back_tilter_servo_num = 1

        self.tof_servo_upper_command = 90
        self.tof_servo_lower_command = 180
        self.tof_servo_upper_angle_deg = 360.0
        self.tof_servo_lower_angle_deg = 275.0

        self.num_servos = 16

        self.pan_servo_num = 2
        self.tilt_servo_num = 3

        self.pan_right_command = 90
        self.pan_center_command = 43
        self.pan_left_command = 0
        self.tilt_up_command = 0
        self.tilt_center_command = 105
        self.tilt_down_command = 150

        self.kp = 40.0
        self.ki = 0.0
        self.kd = 0.01
        self.obstacle_threshold_x_mm = 110.0
        self.ledge_threshold_y_mm = 35.0
        self.buffer_x_mm = 20.0

        self.packet_codes = {}
        self.packet_names = {}
        self.write_commands = {}
        self.write_codes = {}

        self.update_rpi_state_delay = 0.5

        super(RobotConfig, self).__init__("rover.yaml")

        for identifier, names in self.packet_names.items():
            new_names = []
            for name in names:
                if name == "$NUM_SERVOS":
                    new_names.extend([str(x) for x in range(self.num_servos)])
                else:
                    new_names.append(name)
            self.packet_names[identifier] = new_names
