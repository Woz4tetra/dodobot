import math
import time
import pprint

from .robot_client import Robot
from .task import Task
from .network_info import NetworkInfo
from . import image
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
        self.prev_drive_repeat_command_time = 0.0
        self.prev_drive_repeat_stop_time = 0.0
        self.prev_sent_command_time = 0.0
        self.drive_command_timeout = robot_config.drive_command_timeout
        self.drive_command_repeat_timeout = robot_config.drive_command_repeat_timeout
        self.drive_stop_repeat_timeout = self.drive_command_repeat_timeout + 0.5
        self.drive_command_update_delay = 1.0 / robot_config.drive_command_update_rate

        self.joystick_deadzone = robot_config.joystick_deadzone
        self.max_joy_val = robot_config.max_joy_val
        self.drive_cmd_forward = 0.0
        self.drive_cmd_rotate = 0.0
        self.prev_cmd_A = 0.0
        self.prev_cmd_B = 0.0

        self.stepper_max_speed = robot_config.stepper_max_speed
        self.stepper_max_accel = robot_config.stepper_max_accel

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

        self.breakout_events = {
            1 : "BRICK_COLLIDE",
            2 : "WIN_CONDITION",
            3 : "BALL_OUT_OF_BOUNDS",
            4 : "PADDLE_COLLIDE",
        }

        self.tilt_position = 0
        self.sent_tilt_position = 0
        self.tilt_speed = 0
        self.max_tilt_speed = robot_config.max_tilt_speed

        self.grab_speed = 0
        self.grab_joy_val_threshold = 0.25
        self.max_grab_speed = robot_config.max_grab_speed

        self.enable_tilt_axis = False

        self.thumbl_pressed = False
        self.thumbr_pressed = False
        self.set_pid_event = True

        self.joystick = None
        self.sounds = None

        self.network_str_task = Task(self.write_network_task)
        self.network_str_task.thread.daemon = True
        self.network_str_delay = 5.0 * 60.0
        self.network_str_delay_no_connection = 1.0
        self.interface_name = "wlan0"
        self.network_info = NetworkInfo([self.interface_name])

    def start(self):
        super(Dodobot, self).start()

        self.joystick = self.session.joystick
        self.sounds = self.session.sounds

        self.set_pid_ks()
        self.set_gripper_config()
        self.set_breakout_level()
        self.set_linear_max_speed(self.stepper_max_speed)
        self.set_linear_max_accel(self.stepper_max_accel)

        self.network_str_task.start()

        self.sounds["startup"].play()

        time.sleep(0.35)
        try:
            self.write_image(
                robot_config.startup_image_path,
                robot_config.startup_image_size,
                robot_config.startup_image_quality,
            )
        except BaseException as e:
            logger.error(str(e), exc_info=True)

    def process_packet(self, category):
        super(Dodobot, self).process_packet(category)

        if category == "ir" and self.parse_segments("udd"):
            logger.info("remote type: %s\t\tvalue: %s" % (self.parsed_data[1], self.parsed_data[2]))
            ir_code = self.parsed_data[2]
            if ir_code == 0x00ff:  # VOL-
                self.sounds.controller.set_volume(self.sounds.controller.volume - 0.05)
                self.sounds["volume_change"].play()
            elif ir_code == 0x40bf:  # VOL+
                self.sounds.controller.set_volume(self.sounds.controller.volume + 0.05)
                self.sounds["volume_change"].play()

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

        elif category == "breakout" and self.parse_segments("ud"):
            event_code = self.parsed_data[1]
            if event_code in self.breakout_events:
                event_str = self.breakout_events[event_code]
                logger.info("Received breakout event: %s" % event_str)
                if event_str == "BRICK_COLLIDE":
                    self.sounds["breakout_brick"].play()
                elif event_str == "WIN_CONDITION":
                    self.sounds["breakout_win"].play()
                elif event_str == "BALL_OUT_OF_BOUNDS":
                    self.sounds["breakout_out_of_bounds"].play()
                elif event_str == "PADDLE_COLLIDE":
                    self.sounds["breakout_paddle"].play()
            else:
                logger.warn("Received unknown breakout event code: %s" % event_code)

        elif category == "bump" and self.parse_segments("udd"):
            bump1_state = self.parsed_data[1]
            bump2_state = self.parsed_data[2]
            logger.info("bump 1: %s, 2: %s" % (bump1_state, bump2_state))
            if bump1_state or bump2_state:
                self.sounds["bumper_sound"].play()

    def open_gripper(self, position=None):
        logger.debug("Sending gripper open command: %s" % str(position))
        if position is None:
            self.write("grip", 0)
        else:
            self.write("grip", 0, position)

    def close_gripper(self, force_threshold=-1, position=None):
        logger.debug("Sending gripper close command: %s" % str(position))
        if position is None:
            self.write("grip", 1, force_threshold)
        else:
            self.write("grip", 1, force_threshold, position)

    def toggle_gripper(self, force_threshold=-1, position=None):
        logger.debug("Sending gripper toggle command: %s" % str(position))
        if position is None:
            self.write("grip", 2, force_threshold)
        else:
            self.write("grip", 2, force_threshold, position)

    def set_gripper(self, position, force_threshold=-1):
        position = max(robot_config.gripper_open, position)
        position = min(robot_config.gripper_closed, position)
        position = int(position)
        if position < self.gripper_state["pos"]:
            self.open_gripper(position)
        else:
            self.close_gripper(force_threshold, position)

    def tilter_up(self):
        self.write("tilt", 0)

    def tilter_down(self):
        self.write("tilt", 1)

    def tilter_toggle(self):
        self.write("tilt", 2)

    def set_tilter(self, pos):
        self.sent_tilt_position = pos
        logger.debug("Sending tilt command: %s" % (self.sent_tilt_position))
        self.write("tilt", 3, pos)

    def set_linear_max_speed(self, max_speed):
        self.write("lincfg", 0, max_speed)

    def set_linear_max_accel(self, max_accel):
        self.write("lincfg", 1, max_accel)

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
        for attempt in range(5):
            self.write("ks", *self.pid_ks)
            if self.wait_for_ok():
                break
            logger.warn("Failed to receive ok signal for PID. Trying again.")

    def set_gripper_config(self):
        logger.info(
            "Setting gripper limits open: %s, closed: %s" % (robot_config.gripper_open, robot_config.gripper_closed))
        self.write("gripcfg", int(robot_config.gripper_open), int(robot_config.gripper_closed))

    def set_breakout_level(self):
        level_config = robot_config.breakout_level_config
        level_config = level_config.replace("\n", "-")
        self.write("breakout", level_config)

    def reload_pid_ks(self):
        robot_config.load()
        self.pid_ks = []
        for name in self.constant_names:
            assert name in robot_config.pid_ks, robot_config.pid_ks
            self.pid_ks.append(robot_config.pid_ks[name])
        logger.info("Set PID Ks to:\n%s" % pprint.pformat(robot_config.pid_ks))

    def write_network_info(self, kwargs):
        logger.debug("Writing networking info:\n%s" % kwargs)
        self.write("network", kwargs["ip"], kwargs["nmask"], kwargs["bcast"], kwargs["name"], kwargs["error"])

    def write_network_task(self, should_stop):
        while True:
            if should_stop():
                return
            updated = self.network_info.update()
            if self.interface_name in updated:
                self.write_network_info(self.network_info.info[self.interface_name])

            if self.network_info.info[self.interface_name]["error"] == " ":
                time.sleep(self.network_str_delay)
            else:
                time.sleep(self.network_str_delay_no_connection)

    def update(self):
        super(Dodobot, self).update()

        if not self.network_str_task.is_errored():
            logger.error("Error detected in network str task. Raising exception")
            raise self.network_str_task.thread_exception

        self.update_joystick()

    def update_joystick(self):
        if self.joystick.is_open():
            self.update_axis_events()
            self.update_button_events()
        else:
            self.linear_vel_command = 0.0
            self.drive_cmd_rotate = 0.0
            self.drive_cmd_forward = 0.0
            self.grab_speed = 0
            self.tilt_speed = 0

        if self.tilt_speed != 0:
            self.set_tilter(self.tilt_position + self.tilt_speed)
        if self.grab_speed != 0:
            self.set_gripper(self.gripper_state["pos"] + self.grab_speed, robot_config.force_threshold)

        self.update_drive_command()
        self.update_linear_command()

    def update_axis_events(self):
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
                if self.enable_tilt_axis:
                    self.tilt_speed = int(-self.max_tilt_speed * value)
                else:
                    self.linear_vel_command = int(-self.stepper_max_speed * value)
                # self.prev_drive_command_time = time.time()
            elif name == "rx":
                if abs(value) > self.grab_joy_val_threshold:
                    value -= math.copysign(self.grab_joy_val_threshold, value)
                    value *= 1.0 / (1.0 - self.grab_joy_val_threshold)
                    self.grab_speed = int(-self.max_grab_speed * value)
                else:
                    self.grab_speed = 0
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
            # elif name == "rz":
            #     self.set_brake_pedal_gripper(value)

    def update_button_events(self):
        for name, value in self.joystick.get_button_events():
            logger.info("%s: %.3f" % (name, value))
            if value == 1:
                if name == "a":
                    logger.info("Homing linear")
                    self.home_linear()
                elif name == "b":
                    logger.info("Toggling gripper")
                    self.toggle_gripper(robot_config.force_threshold)
                    # self.reset_brake_pedal_gripper()
                elif name == "x":
                    logger.info("Toggling tilter")
                    self.tilter_toggle()
                elif name == "y":
                    self.set_active(not self.is_active)
                elif name == "tl":
                    self.enable_tilt_axis = True
                    self.linear_vel_command = 0.0
                # elif name == "tr":
                elif name == "thumbl":
                    self.thumbl_pressed = True
                elif name == "thumbr":
                    self.thumbr_pressed = True
            else:
                if name == "tl":
                    self.enable_tilt_axis = False
                    self.tilt_speed = 0
                # elif name == "tr":
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

    def joy_to_gripper(self, joy_value):
        return int((robot_config.gripper_closed - robot_config.gripper_open) / 2.0 * (
                joy_value + 1.0) + robot_config.gripper_open)

    def set_brake_pedal_gripper(self, joy_value):
        gripper_pos = self.joy_to_gripper(joy_value)
        logger.info("gripper_pos: %s" % gripper_pos)
        logger.info("brake_pedal_gripper_threshold: %s" % self.brake_pedal_gripper_threshold)

        if gripper_pos > self.brake_pedal_gripper_threshold:
            self.close_gripper(robot_config.force_threshold, gripper_pos)
            self.brake_pedal_gripper_threshold = gripper_pos

    def reset_brake_pedal_gripper(self):
        self.brake_pedal_gripper_threshold = robot_config.gripper_open

    def update_linear_command(self):
        linear_vel = self.linear_vel_command
        if self.prev_linear_vel_command != linear_vel:
            self.set_linear_vel(linear_vel)
            self.prev_linear_vel_command = linear_vel

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

        if (self.prev_cmd_A != cmd_A or self.prev_cmd_B != cmd_B or
                current_time - self.prev_drive_repeat_command_time > self.drive_command_repeat_timeout):
            self.prev_drive_repeat_command_time = current_time

            if (cmd_A == 0.0 and cmd_B == 0.0):
                if current_time - self.prev_drive_repeat_stop_time > self.drive_stop_repeat_timeout:
                    return
            else:
                self.prev_drive_repeat_stop_time = current_time

            self.prev_cmd_A = cmd_A
            self.prev_cmd_B = cmd_B

            self.set_drive_motors(cmd_A, cmd_B)
            logger.info("A: %s, B: %s" % (cmd_A, cmd_B))

    def pre_serial_stop_callback(self):
        self.network_str_task.stop()

    def write_image(self, path, size, quality=15):
        img_bytes = image.bytes_from_file(path, size, quality)
        if len(img_bytes) == 0:
            logger.warn("Image is empty!")
            return

        len_img_bytes = len(img_bytes)
        if len_img_bytes > 0x20000:
            logger.warn("Image is too large: %s!" % len_img_bytes)
            return

        logger.info("Writing image: %s" % len_img_bytes)
        self.write_large("img", img_bytes)
