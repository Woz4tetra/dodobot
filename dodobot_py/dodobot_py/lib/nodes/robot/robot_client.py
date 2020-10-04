import math
import time
import pprint
import serial
import datetime
import threading

from .device_port import DevicePort
from lib.config import ConfigManager
from lib.logger_manager import LoggerManager
from lib.exceptions import ShutdownException, LowBatteryException, DeviceNotReadyException
from ..node import Node
from .battery_state import BatteryState

device_port_config = ConfigManager.get_device_port_config()
robot_config = ConfigManager.get_robot_config()
logger = LoggerManager.get_logger()


class Robot(Node):
    def __init__(self, session):
        super(Robot, self).__init__(session)

        self.device = DevicePort(
            device_port_config.address,
            device_port_config.baud_rate,
            device_port_config.timeout,
            device_port_config.write_timeout
        )

        self.prev_time_command_update = time.time()
        self.should_stop = False
        self.should_stop_fn = (lambda: self.should_stop,)
        self.thread = threading.Thread(target=self.read_task, args=self.should_stop_fn)
        self.read_thread_exception = None
        self.write_date_thread_exception = None
        self.serial_device_paused = False

        self.read_update_rate_hz = device_port_config.update_rate_hz
        self.update_delay = 1.0 / self.read_update_rate_hz

        self.prev_packet_time = 0.0

        self.write_packet_num = 0
        self.read_packet_num = -1
        self.recv_packet_num = 0

        self.prev_packet_num_report_time = 0.0

        self.write_lock = threading.Lock()
        self.read_lock = threading.Lock()

        self.PACKET_START_0 = b'\x12'
        self.PACKET_START_1 = b'\x34'
        self.PACKET_STOP = b'\n'
        self.PACKET_SEP = b'\t'
        self.PACKET_SEP_STR = '\t'

        self.ready_state = {
            "name"    : "",
            "is_ready": False,
            "time_ms" : 0
        }

        self.power_state = {
            "recv_time"     : 0.0,
            "current_mA"    : 0.0,
            "power_mW"      : 0.0,
            "load_voltage_V": 0.0
        }

        self.robot_state = {
            "recv_time"    : 0.0,
            "is_active"    : False,
            "battery_ok"   : True,
            "motors_active": False,
        }

        self.gripper_state = {
            "recv_time"    : 0.0,
            "pos"          : 0
        }
        self.gripper_open_pos = 0
        self.gripper_closed_pos = 180

        self.brake_pedal_gripper_threshold = 0

        self.read_buffer = ""
        self.buffer_index = 0
        self.current_segment = ""
        self.current_segment_num = 0
        self.parsed_data = []
        self.recv_time = 0.0

        self.check_ready_timeout = robot_config.check_ready_timeout
        self.write_timeout = robot_config.write_timeout
        self.packet_start_timeout = robot_config.packet_start_timeout

        self.packet_error_codes = {
            0: "no error",
            1: "c1 != \\x12",
            2: "c2 != \\x34",
            3: "packet is too short",
            4: "checksums don't match",
            5: "packet count segment not found",
            6: "packet counts not synchronized",
            7: "failed to find category segment",
            8: "invalid format",
        }

        self.device_start_time = 0.0
        self.offset_time_ms = 0

        self.is_active = False

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

        self.shutdown_timer = 0.0
        self.shutdown_starting = False
        self.prev_display_countdown = None
        self.shutdown_time_limit = robot_config.shutdown_time_limit

        self.write_date_thread = threading.Thread(target=self.write_date_task, args=self.should_stop_fn)
        self.write_date_delay = robot_config.write_date_delay

        self.stepper_max_speed = robot_config.stepper_max_speed
        self.drive_max_speed = robot_config.drive_max_speed
        self.drive_min_speed = robot_config.drive_min_speed
        self.linear_vel_command = 0
        self.prev_linear_vel_command = 0

        self.pid_ks = []
        self.constant_names = "kp_A ki_A kd_A kp_B ki_B kd_B speed_kA speed_kB".split(" ")
        self.reload_pid_ks()

        self.stepper_events = {
            1: "ACTIVE_TRUE",
            2: "ACTIVE_FALSE",
            3: "HOMING_STARTED",
            4: "HOMING_FINISHED",
            5: "MOVE_STARTED",
            6: "MOVE_FINISHED",
            7: "POSITION_ERROR",
            8: "NOT_HOMED",
            9: "NOT_ACTIVE",
            10: "HOMING_FAILED"
        }

        self.tilt_position = 0
        self.sent_tilt_position = 0
        self.tilt_speed = 0

        self.thumbl_pressed = False
        self.thumbr_pressed = False
        self.set_pid_event = True

        self.battery_state = BatteryState()

    def start(self):
        self.joystick = self.session.joystick

        logger.info("Starting rover client")

        self.device.configure()
        logger.info("Device configured")

        self.thread.start()
        logger.info("Read thread started")
        time.sleep(1.0)

        self.check_ready()
        self.prev_command_time = time.time()

        self.set_reporting(True)

        self.set_pid_ks()

        self.write_date_thread.start()

    def process_packet(self, category):
        if category == "txrx" and self.parse_segments("dd"):
            packet_num = self.parsed_data[0]
            error_code = self.parsed_data[1]

            if error_code != 0:
                self.log_packet_error_code(error_code, packet_num)
            else:
                logger.debug("No error in transmitted packet #%s" % packet_num)

        elif category == "ready" and self.parse_segments("ds"):
            self.ready_state["time_ms"] = self.parsed_data[0]
            self.ready_state["name"] = self.parsed_data[1]
            self.ready_state["is_ready"] = True

            logger.info("Ready signal received! %s" % self.ready_state)

        elif category == "ir" and self.parse_segments("udd"):
            logger.info("remote type: %s\t\tvalue: %s" % (self.parsed_data[1], self.parsed_data[2]))

        elif category == "batt" and self.parse_segments("ufff"):
            self.power_state["recv_time"] = self.get_device_time(self.parsed_data[0])
            self.power_state["current_mA"] = self.parsed_data[1]
            self.power_state["power_mW"] = self.parsed_data[2]
            self.power_state["load_voltage_V"] = self.parsed_data[3]
            state_changed = self.battery_state.set(self.power_state)
            if state_changed:
                self.battery_state.log_state()
            if self.battery_state.should_shutdown():
                raise LowBatteryException("Battery is critically low: {load_voltage_V:0.2f}!! Shutting down.".format(**self.power_state))

        elif category == "state" and self.parse_segments("uddd"):
            self.robot_state["recv_time"] = self.get_device_time(self.parsed_data[0])
            self.robot_state["is_active"] = self.parsed_data[1]
            self.robot_state["battery_ok"] = self.parsed_data[2]
            self.robot_state["motors_active"] = self.parsed_data[3]

        elif category == "tilt" and self.parse_segments("ud"):
            # recv_time = self.parsed_data[0]
            self.tilt_position = self.parsed_data[1]

        elif category == "grip" and self.parse_segments("ud"):
            self.gripper_state["recv_time"] = self.get_device_time(self.parsed_data[0])
            self.gripper_state["pos"] = self.parsed_data[1]

        elif category == "le" and self.parse_segments("ud"):
            recv_time = self.get_device_time(self.parsed_data[0])
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

        elif category == "latch_btn" and self.parse_segments("ud"):
            button_state = self.parsed_data[1]
            if button_state == 1:
                self.start_shutdown()
            else:
                self.cancel_shutdown()

        elif category == "unlatch" and self.parse_segments("u"):
            logger.warning("Unlatch signal received! System unlatching soon.")

        elif category == "shutdown" and self.parse_segments("s"):
            name = self.parsed_data[0]
            if name == "dodobot":
                logger.info("'Shutdown now' signal received")
                raise ShutdownException
            else:
                logger.warning("Received name doesn't match expected: %s" % name)

    def start_shutdown(self):
        logger.info("Starting shutdown timer")
        self.shutdown_timer = time.time()
        self.shutdown_starting = True

    def cancel_shutdown(self):
        logger.info("Canceling shutdown")
        self.shutdown_starting = False
        self.prev_display_countdown = None

    def check_shutdown_timer(self):
        if not self.shutdown_starting:
            return

        current_time = time.time()
        countdown_time = self.shutdown_time_limit - (current_time - self.shutdown_timer)
        countdown_time_int = int(countdown_time) + 1
        if countdown_time_int != self.prev_display_countdown:
            logger.info("%s..." % countdown_time_int)
            self.prev_display_countdown = countdown_time_int
        if countdown_time <= 0.0:
            logger.info("Shutting down")
            self.write_shutdown_signal()
            time.sleep(0.15)
            raise ShutdownException

    def write_shutdown_signal(self):
        self.write("shutdown", "dodobot")

    def set_reporting(self, state):
        self.write("[]", 1 if state else 0)

    def set_active(self, state):
        self.is_active = state
        self.write("<>", 1 if state else 0)

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

    def reload_pid_ks(self):
        robot_config.load()
        self.pid_ks = []
        for name in self.constant_names:
            assert name in robot_config.pid_ks, robot_config.pid_ks
            self.pid_ks.append(robot_config.pid_ks[name])
        logger.info("Set PID Ks to:\n%s" % pprint.pformat(robot_config.pid_ks))

    def write_date(self):
        date_str = datetime.datetime.now().strftime("%I:%M:%S%p")
        self.write("date", date_str)

    def write_date_task(self, should_stop):
        failed_write_attempts = 0
        try:
            while True:
                if should_stop():
                    return
                time.sleep(self.write_date_delay)
                if self.serial_device_paused:
                    continue
                try:
                    self.write_date()
                    failed_write_attempts = 0
                except serial.SerialTimeoutException as e:
                    failed_write_attempts += 1
                    if failed_write_attempts >= 10:
                        self.write_date_thread_exception = e
                        return
        except BaseException as e:
            logger.error(str(e), exc_info=True)
            self.write_date_thread_exception = e

    def is_write_date_task_running(self):
        return self.write_date_thread_exception is None

    def check_ready(self):
        self.write("?", "dodobot")

        begin_time = time.time()
        write_time = time.time()

        while not self.ready_state["is_ready"]:
            if time.time() - begin_time > self.check_ready_timeout:
                break
            if time.time() - write_time > self.write_timeout:
                logger.info("Writing ready signal again")
                self.write("?", "dodobot")
                write_time = time.time()

            if self.device.in_waiting() > 2:
                self.read()

        if self.ready_state["is_ready"]:
            self.set_start_time(self.ready_state["time_ms"])
            logger.info("Serial device is ready. Robot name is %s" % self.ready_state["name"])
        else:
            raise DeviceNotReadyException("Failed to receive ready signal within %ss" % self.check_ready_timeout)

    def update(self):
        if not self.is_read_task_running():
            logger.error("Error detected in read task. Raising exception")
            raise self.read_thread_exception

        if not self.is_write_date_task_running():
            logger.error("Error detected in write date task. Raising exception")
            raise self.write_date_thread_exception

        for name, value in self.joystick.get_axis_events():
            if abs(value) < self.joystick_deadzone:
                value = 0.0
            else:
                joy_val = abs(value) - self.joystick_deadzone
                joy_val = math.copysign(joy_val, value)
                max_joy_val_adj = self.max_joy_val - self.joystick_deadzone
                command = 1.0 / max_joy_val_adj * joy_val

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
        self.check_shutdown_timer()

        current_time = time.time()
        if current_time - self.prev_packet_num_report_time > 60.0:
            logger.info("Packet numbers. Read: %s; Write: %s" % (self.read_packet_num, self.write_packet_num))
            self.prev_packet_num_report_time = current_time

    def joy_to_gripper(self, joy_value):
        return int((self.gripper_closed_pos - self.gripper_open_pos) / 2.0 * (joy_value + 1.0) + self.gripper_open_pos)

    def set_brake_pedal_gripper(self, joy_value):
        gripper_pos = self.joy_to_gripper(joy_value)
        logger.info("gripper_pos: %s" % gripper_pos)
        logger.info("brake_pedal_gripper_threshold: %s" % self.brake_pedal_gripper_threshold)

        if gripper_pos > self.brake_pedal_gripper_threshold:
            self.close_gripper(100, gripper_pos)
            self.brake_pedal_gripper_threshold = gripper_pos

    def reset_brake_pedal_gripper(self):
        self.brake_pedal_gripper_threshold = self.gripper_open_pos

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


    def stop(self):
        if self.should_stop:
            logger.info("Stop flag already set")
            return

        logger.info("Stopping robot client")

        self.should_stop = True
        logger.info("Set read thread stop flag")

        self.set_reporting(False)
        self.set_active(False)

        time.sleep(0.1)

        with self.read_lock:
            self.device.stop()
        logger.info("Device connection closed")

    def write(self, name, *args):
        if self.serial_device_paused:
            logger.debug("Serial device is paused. Skipping write: %s, %s" % (str(name), str(args)))
            return

        packet = b""
        packet += self.PACKET_START_0
        packet += self.PACKET_START_1
        packet += str(self.write_packet_num).encode()
        packet += self.PACKET_SEP + str(name).encode()
        for arg in args:
            packet += self.PACKET_SEP + str(arg).encode()

        calc_checksum = 0
        for val in packet[2:]:
            calc_checksum += val
        calc_checksum &= 0xff

        packet += b"%02x" % calc_checksum
        packet += self.PACKET_STOP

        with self.write_lock:
            logger.debug("Writing %s" % str(packet))
            try:
                self.device.write(packet)
            except BaseException as e:
                logger.error("Exception while writing packet %s: %s" % (packet, str(e)), exc_info=True)

            self.write_packet_num += 1
            time.sleep(0.0005)  # give the microcontroller a chance to not drop the next packet

    def wait_for_packet_start(self):
        begin_time = time.time()
        msg_buffer = b""
        c1 = ""
        c2 = ""

        while True:
            if time.time() - begin_time > self.packet_start_timeout:
                return False

            if self.device.in_waiting() < 2:
                continue

            c1 = self.device.read(1)
            # logger.info("buffer: %s" % msg_buffer)
            if c1 == self.PACKET_START_0:
                c2 = self.device.read(1)
                if c2 == self.PACKET_START_1:
                    return True
            elif c1 == self.PACKET_STOP:
                time.sleep(self.update_delay)
                logger.info("Device message: %s" % (msg_buffer.decode()))
                msg_buffer = b""
            else:
                msg_buffer += c1

    def get_next_segment(self):
        if self.buffer_index >= len(self.read_buffer):
            return False

        sep_index = self.read_buffer.find(self.PACKET_SEP_STR, self.buffer_index)
        if sep_index == -1:
            self.current_segment = self.read_buffer[self.buffer_index:]
            self.buffer_index = len(self.read_buffer)
        else:
            self.current_segment = self.read_buffer[self.buffer_index: sep_index]
            self.buffer_index = sep_index + 1
        return True

    def read(self):
        with self.read_lock:
            return self._read()

    def readline(self):
        buffer = b""
        while True:
            if self.device.in_waiting():
                c = self.device.read(1)
                if c == self.PACKET_STOP:
                    break
                buffer += c
        return buffer

    def _read(self):
        # if self.serial_device_paused:
        #     logger.debug("Serial device is paused. Skipping read")
        #     return

        if not self.wait_for_packet_start():
            return False

        buffer = self.readline()
        logger.debug("buffer: %s" % buffer.decode())

        if len(buffer) < 5:
            logger.error("Received packet has an invalid number of characters! %s" % repr(buffer.decode()))
            self.read_packet_num += 1
            return False

        calc_checksum = 0
        for val in buffer[:-2]:
            calc_checksum += val
        calc_checksum &= 255

        self.read_buffer = buffer.decode()
        try:
            recv_checksum = int(self.read_buffer[-2:], 16)
        except ValueError as e:
            logger.error("Failed to parsed checksum as hex int: %s" % repr(self.read_buffer))
            self.read_packet_num += 1
            return False
        self.read_buffer = self.read_buffer[:-2]
        if calc_checksum != recv_checksum:
            logger.error(
                "Checksum failed! recv %s != calc %s. %s" % (recv_checksum, calc_checksum, repr(self.read_buffer)))
            self.read_packet_num += 1
            return False

        self.buffer_index = 0

        # get packet num segment
        if not self.get_next_segment():
            logger.error(
                "Failed to find packet number segment %s! %s" % (repr(self.current_segment), repr(self.read_buffer)))
            self.read_packet_num += 1
            return False
        self.recv_packet_num = int(self.current_segment)
        if self.read_packet_num == -1:
            self.read_packet_num = self.recv_packet_num
        if self.recv_packet_num != self.read_packet_num:
            logger.warning("Received packet num doesn't match local count. "
                           "recv %s != local %s", self.recv_packet_num, self.read_packet_num)
            logger.debug("Buffer: %s" % self.read_buffer)
            self.read_packet_num = self.recv_packet_num

        # find category segment
        if not self.get_next_segment():
            logger.error(
                "Failed to find category segment %s! %s" % (repr(self.current_segment), repr(self.read_buffer)))
            self.read_packet_num += 1
            return False
        category = self.current_segment

        try:
            self.process_packet(category)
        except ShutdownException:
            raise
        except LowBatteryException:
            raise
        except BaseException as e:
            logger.error("Exception while processing packet %s: %s" % (self.read_buffer, str(e)), exc_info=True)
            return False

        self.read_packet_num += 1
        return True

    def parse_segments(self, formats):
        self.parsed_data = []
        self.recv_time = time.time()
        for index, f in enumerate(formats):
            if not self.get_next_segment():
                logger.error("Failed to parse segment #%s. Buffer: %s" % (index, self.read_buffer))
                return False
            if f == 'd' or f == 'u':
                self.parsed_data.append(int(self.current_segment))
            elif f == 's':
                self.parsed_data.append(self.current_segment)
            elif f == 'f':
                self.parsed_data.append(float(self.current_segment))
        return True

    def is_read_task_running(self):
        return self.read_thread_exception is None

    def read_task(self, should_stop):
        self.prev_packet_time = time.time()

        try:
            while True:
                time.sleep(self.update_delay)
                if should_stop():
                    logger.info("Exiting read thread\n\n")
                    return

                while self.device.in_waiting() > 2:
                    self.read()

        except BaseException as e:
            logger.error("An exception occurred in the read thread", exc_info=True)
            self.read_thread_exception = e

    def log_packet_error_code(self, error_code, packet_num):
        logger.warning("Packet %s returned an error:" % packet_num)
        logger.warning("\t%s" % self.packet_error_codes[error_code])

    def set_start_time(self, time_ms):
        self.device_start_time = time.time()
        self.offset_time_ms = time_ms

    def get_device_time(self, time_ms):
        return self.device_start_time + (time_ms - self.offset_time_ms) / 1000.0

    def pause_serial_device(self):
        logger.info("Pausing serial device")
        self.serial_device_paused = True

    def resume_serial_device(self):
        logger.info("Resuming serial device")
        self.serial_device_paused = False
