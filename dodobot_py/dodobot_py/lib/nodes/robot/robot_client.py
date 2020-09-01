import math
import time
import datetime
import threading

from .device_port import DevicePort
from lib.config import ConfigManager
from lib.logger_manager import LoggerManager
from lib.exceptions import ShutdownException, LowBatteryException, DeviceNotReadyException
from ..node import Node
from .battery_state import BatteryState

device_port_config = ConfigManager.get_device_port_config()
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
        self.thread_exception = None

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

        self.read_buffer = ""
        self.buffer_index = 0
        self.current_segment = ""
        self.current_segment_num = 0
        self.parsed_data = []
        self.recv_time = 0.0

        self.check_ready_timeout = 5.0
        self.write_timeout = 1.0
        self.packet_start_timeout = 0.15

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
        self.drive_command_timeout = 2.0
        self.drive_command_update_delay = 1.0 / 30.0
        self.prev_sent_command_time = 0.0

        self.joystick_deadzone = 0.1
        self.drive_cmd_forward = 0.0
        self.drive_cmd_rotate = 0.0
        self.prev_cmd_A = 0.0
        self.prev_cmd_B = 0.0

        self.shutdown_timer = 0.0
        self.shutdown_starting = False
        self.prev_display_countdown = None
        self.shutdown_time_limit = 3.0

        self.write_date_thread = threading.Thread(target=self.write_date_task, args=self.should_stop_fn)
        self.write_date_delay = 0.5

        self.stepper_max_speed = 300000000
        self.drive_max_speed = 6800
        self.linear_vel_command = 0
        self.prev_linear_vel_command = 0

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
                raise LowBatteryException("Battery is critically low: %{load_voltage_V:0.2f}!! Shutting down.".format(**self.power_state))

        elif category == "state" and self.parse_segments("uddd"):
            self.robot_state["recv_time"] = self.get_device_time(self.parsed_data[0])
            self.robot_state["is_active"] = self.parsed_data[1]
            self.robot_state["battery_ok"] = self.parsed_data[2]
            self.robot_state["motors_active"] = self.parsed_data[3]

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

    def open_gripper(self):
        self.write("grip", 0)

    def close_gripper(self, force_threshold=-1):
        self.write("grip", 1, force_threshold)

    def toggle_gripper(self, force_threshold=-1):
        self.write("grip", 2, force_threshold)

    def tilter_up(self):
        self.write("tilt", 0)

    def tilter_down(self):
        self.write("tilt", 1)

    def tilter_toggle(self):
        self.write("tilt", 2)

    def set_tilter(self, pos):
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

    def write_date(self):
        date_str = datetime.datetime.now().strftime("%I:%M:%S%p")
        self.write("date", date_str)

    def write_date_task(self, should_stop):
        try:
            while True:
                if should_stop():
                    return
                self.write_date()
                time.sleep(self.write_date_delay)
        except BaseException as e:
            logger.error(str(e), exc_info=True)

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
        if not self.read_task_running():
            logger.error("Error detected in read task. Raising exception")
            raise self.thread_exception

        for name, value in self.joystick.get_axis_events():
            if abs(value) < self.joystick_deadzone:
                value = 0.0
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
        self.update_drive_command()
        for name, value in self.joystick.get_button_events():
            if name == "a" and value == 1:
                logger.info("Homing linear")
                self.home_linear()
            elif name == "b" and value == 1:
                logger.info("Toggling gripper")
                self.toggle_gripper(750)
            elif name == "x" and value == 1:
                logger.info("Toggling tilter")
                self.tilter_toggle()
            elif name == "y" and value == 1:
                self.set_active(not self.is_active)

        self.check_shutdown_timer()

        current_time = time.time()
        if current_time - self.prev_packet_num_report_time > 60.0:
            logger.info("Packet numbers. Read: %s; Write: %s" % (self.read_packet_num, self.write_packet_num))
            self.prev_packet_num_report_time = current_time

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
            logger.info("linear_vel: %s" % linear_vel)
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
            self.device.write(packet)
            self.write_packet_num += 1
            time.sleep(0.0005)  # give the microcontroller a chance to not drop the next packet

    def wait_for_packet_start(self):
        begin_time = time.time()
        msg_buffer = b""
        c1 = ""
        c2 = ""

        while True:
            time.sleep(self.update_delay)
            if time.time() - begin_time > self.packet_start_timeout:
                return False

            if self.device.in_waiting() < 2:
                continue

            c1 = self.device.read(1)
            if c1 == self.PACKET_START_0:
                c2 = self.device.read(1)
                if c2 == self.PACKET_START_1:
                    return True
            elif (c1 == self.PACKET_STOP or c2 == self.PACKET_STOP):
                logger.info("Device message: %s" % (msg_buffer.decode()))
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

    def read_task_running(self):
        return self.thread_exception is None

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
            self.thread_exception = e

    def log_packet_error_code(self, error_code, packet_num):
        logger.warning("Packet %s returned an error:" % packet_num)
        logger.warning("\t%s" % self.packet_error_codes[error_code])

    def set_start_time(self, time_ms):
        self.device_start_time = time.time()
        self.offset_time_ms = time_ms

    def get_device_time(self, time_ms):
        return self.device_start_time + (time_ms - self.offset_time_ms) / 1000.0
