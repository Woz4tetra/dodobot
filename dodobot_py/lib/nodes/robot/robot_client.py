import time
import datetime
import threading
import numpy as np

from .device_port import DevicePort, DevicePortReadException, DevicePortWriteException
from lib.config import ConfigManager
from lib.logger_manager import LoggerManager
from lib.exceptions import ShutdownException, LowBatteryException
from ..node import Node

rover_config = ConfigManager.get_robot_config()
device_port_config = ConfigManager.get_device_port_config()
logger = LoggerManager.get_logger()


class Robot(Node):
    def __init__(self, session):
        self.device = DevicePort(
            device_port_config.address,
            device_port_config.baud_rate,
            device_port_config.timeout,
            device_port_config.write_timeout
        )

        self.prev_time_command_update = time.time()
        self.should_stop = False
        self.thread = threading.Thread(target=self.read_task, args=(lambda: self.should_stop,))
        self.thread_exception = None

        self.read_update_rate_hz = device_port_config.update_rate_hz
        self.prev_packet_time = 0.0

        self.write_lock = threading.Lock()
        self.read_lock = threading.Lock()

        self.joystick = None

        super(Robot, self).__init__(session)

    def start(self):
        self.joystick = self.session.joystick

        logger.info("Starting rover client")

        self.device.configure()
        logger.info("Device configured")

        self.thread.start()
        logger.info("Read thread started")
        time.sleep(1.0)

    def update(self):
        if not self.read_task_running():
            logger.error("Error detected in read task. Raising exception")
            raise self.thread_exception

        self.update_joystick()

    def stop(self):
        if self.should_stop:
            logger.info("Stop flag already set")
            return

        logger.info("Stopping rover client")

        self.should_stop = True
        logger.info("Set read thread stop flag")

        with self.read_lock:
            self.device.stop()
        logger.info("Device connection closed")

    def write(self, packet: str):
        with self.write_lock:
            packet += '\n'
            self.device.write(packet.encode())
            time.sleep(0.0005)

    def read_task_running(self):
        return self.thread_exception is None

    def read_task(self, should_stop):
        update_delay = 1.0 / self.read_update_rate_hz
        self.prev_packet_time = time.time()

        try:
            while True:
                time.sleep(update_delay)
                if should_stop():
                    logger.info("Exiting read thread\n\n")
                    return

                while self.device.in_waiting() > 0 or self.device.newline_available():
                    with self.read_lock:
                        packet_buffer = self.device.readline()
                        if packet_buffer[-1:] == b"\n":
                            packet_buffer = packet_buffer[:-1]  # remove newline

                        logger.info(packet_buffer.decode())

        except BaseException as e:
            logger.error("An exception occurred in the read thread", exc_info=True)
            self.thread_exception = e

    def set_speed(self, speed_A, speed_B):
        speed_A = str(int(speed_A))
        speed_B = str(int(speed_B))
        self.write("ma%s" % speed_A)
        self.write("mb%s" % speed_B)

    def set_gripper(self, state):
        assert state in ("o", "c", "t")
        self.write(state)

    def toggle_tilter(self):
        self.write("b")

    def update_joystick(self):
        for codename, event in self.joystick.get_events():
            print(codename, event)
