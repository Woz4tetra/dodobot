import math
import time
import pprint

from .robot_client import Robot
from .task import Task
from .network_info import NetworkInfo
from lib.config import ConfigManager
from lib.logger_manager import LoggerManager

device_port_config = ConfigManager.get_device_port_config()
robot_config = ConfigManager.get_robot_config()
logger = LoggerManager.get_logger()


class Dodobot(Robot):
    def __init__(self, session):
        super(Dodobot, self).__init__(session)

        self.gripper_state = {
            "recv_time": 0.0,
            "pos"      : 0
        }

        self.brake_pedal_gripper_threshold = 0

        self.prev_drive_command_time = 0.0
        self.drive_command_timeout = robot_config.drive_command_timeout
        self.drive_command_update_delay = robot_config.drive_command_update_delay
        self.prev_sent_command_time = 0.0

        self.joystick_deadzone = robot_config.joystick_deadzone
        self.max_joy_val = robot_config.max_joy_val
        self.drive_cmd_forward = 0.0
        self.drive_cmd_rotate = 0.0
        self.prev_cmd_A = 0.0
        self.prev_cmd_B = 0.0

        self.stepper_max_speed = robot_config.stepper_max_speed
        self.drive_max_speed = robot_config.drive_max_speed
        self.drive_min_speed = robot_config.drive_min_speed
        self.linear_vel_command = 0
        self.prev_linear_vel_command = 0

        self.pid_ks = []
        self.constant_names = "kp_A ki_A kd_A kp_B ki_B kd_B speed_kA speed_kB".split(" ")
        self.reload_pid_ks()

        self.stepper_events = {
            1 : "ACTIVE_TRUE",
            2 : "ACTIVE_FALSE",
            3 : "HOMING_STARTED",
            4 : "HOMING_FINISHED",
            5 : "MOVE_STARTED",
            6 : "MOVE_FINISHED",
            7 : "POSITION_ERROR",
            8 : "NOT_HOMED",
            9 : "NOT_ACTIVE",
            10: "HOMING_FAILED"
        }

        self.tilt_position = 0
        self.sent_tilt_position = 0
        self.tilt_speed = 0

        self.thumbl_pressed = False
        self.thumbr_pressed = False
        self.set_pid_event = True

        self.joystick = None

        self.network_str_task = Task(self.write_network_task)
        self.network_str_delay = 5.0 * 60.0
        self.network_info = NetworkInfo(["wlan0"])

    def start(self):
        super(Dodobot, self).start()

        self.joystick = self.session.joystick

        self.set_pid_ks()
        self.set_gripper_config()

        self.network_str_task.start()

    def process_packet(self, category):
        super(Dodobot, self).process_packet(category)

        if category == "ir" and self.parse_segments("udd"):
            logger.info("remote type: %s\t\tvalue: %s" % (self.parsed_data[1], self.parsed_data[2]))

        elif category == "tilt" and self.parse_segments("ud"):
            # recv_time = self.parsed_data[0]
            self.tilt_position = self.parsed_data[1]

        elif category == "grip" and self.parse_segments("ud"):
            self.gripper_state["recv_time"] = self.get_device_time(self.parsed_data[0])
            self.gripper_state["pos"] = self.parsed_data[1]

        elif category == "le" and self.parse_segments("ud"):
            # recv_time = self.get_device_time(self.parsed_data[0])
            event_code = self.parsed_data[1]
            if event_code in self.stepper_events:
                event_str = self.stepper_events[event_code]
                logger.info("Received stepper event: %s" % event_str)
                if event_str == "HOMING_STARTED":
                    self.pause_serial_device()
                elif event_str == "HOMING_FINISHED" or event_str == "HOMING_FAILED":
                    self.resume_serial_device()
            else:
                logger.warn("Received unknown stepper event code: %s" % event_code)

    def open_gripper(self, position=None):
        if position is None:
            self.write("grip", 0)
        else:
            self.write("grip", 0, position)

    def close_gripper(self, force_threshold=-1, position=None):
        if position is None:
            self.write("grip", 1, force_threshold)
        else:
            self.write("grip", 1, force_threshold, position)

    def toggle_gripper(self, force_threshold=-1, position=None):
        if position is None:
            self.write("grip", 2, force_threshold)
        else:
            self.write("grip", 2, force_threshold, position)

    def tilter_up(self):
        self.write("tilt", 0)

    def tilter_down(self):
        self.write("tilt", 1)

    def tilter_toggle(self):
        self.write("tilt", 2)

    def set_tilter(self, pos):
        self.sent_tilt_position = pos
        self.write("tilt", 3, pos)

    def set_linear_pos(self, pos):
        self.write("linear", 0, pos)

    def set_linear_vel(self, vel):
        self.write("linear", 1, vel)

    def stop_linear(self):
        self.write("linear", 2)

    def reset_linear(self):
        self.write("linear", 3)

    def home_linear(self):
        self.write("linear", 4)

    def set_drive_motors(self, speed_A, speed_B):
        self.write("drive", float(speed_A), float(speed_B))

    def set_pid_ks(self):
        logger.info("Writing PID Ks: %s" % self.pid_ks)
        self.write("ks", *self.pid_ks)

    def set_gripper_config(self):
        logger.info(
            "Setting gripper limits open: %s, closed: %s" % (robot_config.gripper_open, robot_config.gripper_closed))
        self.write("gripcfg", int(robot_config.gripper_open), int(robot_config.gripper_closed))

    def reload_pid_ks(self):
        robot_config.load()
        self.pid_ks = []
        for name in self.constant_names:
            assert name in robot_config.pid_ks, robot_config.pid_ks
            self.pid_ks.append(robot_config.pid_ks[name])
        logger.info("Set PID Ks to:\n%s" % pprint.pformat(robot_config.pid_ks))

    def write_network_info(self, kwargs):
        logger.info("Writing networking info:\n%s" % kwargs)
        self.write("network", kwargs["ip"], kwargs["nmask"], kwargs["bcast"], kwargs["name"], kwargs["error"])

    def write_network_task(self, should_stop):
        while True:
            if should_stop():
                return
            self.network_info.update()
            self.write_network_info(self.network_info.info["wlan0"])
            time.sleep(self.network_str_delay)

    def update(self):
        super(Dodobot, self).update()

        if not self.network_str_task.is_errored():
            logger.error("Error detected in network str task. Raising exception")
            raise self.network_str_task.thread_exception

        for name, value in self.joystick.get_axis_events():
            if abs(value) < self.joystick_deadzone:
                value = 0.0
            else:
                joy_val = abs(value) - self.joystick_deadzone
                joy_val = math.copysign(joy_val, value)
                max_joy_val_adj = self.max_joy_val - self.joystick_deadzone
                value = 1.0 / max_joy_val_adj * joy_val

            # logger.info("%s: %.3f" % (name, value))

            if name == "ry":
                self.linear_vel_command = int(-self.stepper_max_speed * value)
                # self.prev_drive_command_time = time.time()
            elif name == "x":
                self.drive_cmd_rotate = self.drive_max_speed * value
                self.prev_drive_command_time = time.time()
                # logger.info("rotate cmd: %s" % self.drive_cmd_rotate)
            elif name == "y":
                self.drive_cmd_forward = -self.drive_max_speed * value
                self.prev_drive_command_time = time.time()
                # logger.info("forward cmd: %s" % self.drive_cmd_forward)
            elif name == "hat0x":
                self.drive_cmd_rotate = self.drive_min_speed * value
                self.prev_drive_command_time = time.time()
            elif name == "hat0y":
                self.drive_cmd_forward = -self.drive_min_speed * value
                self.prev_drive_command_time = time.time()
            elif name == "rz":
                self.set_brake_pedal_gripper(value)

        for name, value in self.joystick.get_button_events():
            logger.info("%s: %.3f" % (name, value))
            if value == 1:
                if name == "a":
                    logger.info("Homing linear")
                    self.home_linear()
                elif name == "b":
                    logger.info("Toggling gripper")
                    self.toggle_gripper(750)
                    self.reset_brake_pedal_gripper()
                elif name == "x":
                    logger.info("Toggling tilter")
                    self.tilter_toggle()
                elif name == "y":
                    self.set_active(not self.is_active)
                elif name == "tl":
                    self.tilt_speed = -1
                elif name == "tr":
                    self.tilt_speed = 1
                elif name == "thumbl":
                    self.thumbl_pressed = True
                elif name == "thumbr":
                    self.thumbr_pressed = True
            else:
                if name == "tl":
                    self.tilt_speed = 0
                elif name == "tr":
                    self.tilt_speed = 0
                elif name == "thumbl":
                    self.set_pid_event = True
                    self.thumbl_pressed = False
                elif name == "thumbr":
                    self.set_pid_event = True
                    self.thumbr_pressed = False

        if self.thumbl_pressed and self.thumbr_pressed and self.set_pid_event:
            self.reload_pid_ks()
            self.set_pid_ks()
            self.set_pid_event = False

        if self.tilt_speed != 0:
            logger.debug("Sending tilt command: %s" % (self.sent_tilt_position + self.tilt_speed))
            self.set_tilter(self.sent_tilt_position + self.tilt_speed)

        self.update_drive_command()

    def joy_to_gripper(self, joy_value):
        return int((robot_config.gripper_closed - robot_config.gripper_open) / 2.0 * (
                joy_value + 1.0) + robot_config.gripper_open)

    def set_brake_pedal_gripper(self, joy_value):
        gripper_pos = self.joy_to_gripper(joy_value)
        logger.info("gripper_pos: %s" % gripper_pos)
        logger.info("brake_pedal_gripper_threshold: %s" % self.brake_pedal_gripper_threshold)

        if gripper_pos > self.brake_pedal_gripper_threshold:
            self.close_gripper(100, gripper_pos)
            self.brake_pedal_gripper_threshold = gripper_pos

    def reset_brake_pedal_gripper(self):
        self.brake_pedal_gripper_threshold = robot_config.gripper_open

    def update_drive_command(self):
        current_time = time.time()
        if self.drive_cmd_forward != 0.0 or self.drive_cmd_rotate != 0.0:
            if current_time - self.prev_sent_command_time < self.drive_command_update_delay:
                return
            self.prev_sent_command_time = current_time

        if current_time - self.prev_drive_command_time > self.drive_command_timeout:
            cmd_A = 0.0
            cmd_B = 0.0
        else:
            cmd_A = self.drive_cmd_forward + self.drive_cmd_rotate
            cmd_B = self.drive_cmd_forward - self.drive_cmd_rotate
        linear_vel = self.linear_vel_command

        if self.prev_linear_vel_command != linear_vel:
            logger.debug("linear_vel: %s" % linear_vel)
            self.set_linear_vel(linear_vel)
            self.prev_linear_vel_command = linear_vel

        if self.prev_cmd_A != cmd_A or self.prev_cmd_B != cmd_B:
            # logger.info("A cmd: %s" % cmd_A)
            # logger.info("B cmd: %s" % cmd_B)
            self.set_drive_motors(cmd_A, cmd_B)

            self.prev_cmd_A = cmd_A
            self.prev_cmd_B = cmd_B

    def pre_serial_stop_callback(self):
        self.network_str_task.stop()
