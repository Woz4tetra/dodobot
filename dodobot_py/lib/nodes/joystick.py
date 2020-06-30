import time
import select
from evdev import ecodes, InputDevice

from lib.logger_manager import LoggerManager
from lib.config import ConfigManager

from .node import Node

logger = LoggerManager.get_logger()

joystick_config = ConfigManager.get_joystick_config()


class Joystick(Node):
    def __init__(self, session):
        super(Joystick, self).__init__(session)

        self.address = joystick_config.path
        self.prev_open_attempt_time = time.time()

    def start(self):
        self.open_joystick()
        self.prev_open_attempt_time = time.time()

    def is_open(self):
        return self.device is not None

    def open_joystick(self):
        try:
            return InputDevice(self.address)
        except FileNotFoundError:
            return None

    def get_events(self):
        if self.is_open():
            select.select([self.device.fd], [], [], 0.01)
            try:
                for event in self.device.read():
                    codename = self.process_event(event)
                    yield codename, event
            except BlockingIOError:
                pass
            except OSError:
                pass
        else:
            if time.time() - self.prev_open_attempt_time > 1.0:
                self.device = self.open_joystick()
                self.prev_open_attempt_time = time.time()
                if self.device:
                    logger.info("Joystick opened with address {}".format(self.address))
        raise StopIteration

    @staticmethod
    def process_event(e):
        if e.type == ecodes.EV_SYN:
            if e.code == ecodes.SYN_MT_REPORT:
                msg = 'time {:<16} +++++++++ {} ++++++++'
            else:
                msg = 'time {:<16} --------- {} --------'
            # print(msg.format(e.timestamp(), ecodes.SYN[e.code]))
            return None
        else:
            if e.type in ecodes.bytype:
                codename = ecodes.bytype[e.type][e.code]
            else:
                codename = '?'

            # evfmt = 'time {:<16} type {} ({}), code {:<4} ({}), value {}'
            # print(evfmt.format(e.timestamp(), e.type, ecodes.EV[e.type], e.code, codename, e.value))

            return codename
