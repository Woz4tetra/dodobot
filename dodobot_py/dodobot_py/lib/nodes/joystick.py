# adapted from https://gist.github.com/rdb/8864666

import os
import time
import array
import struct
import select
from fcntl import ioctl


from lib.logger_manager import LoggerManager
from lib.config import ConfigManager

from .node import Node

logger = LoggerManager.get_logger()

joystick_config = ConfigManager.get_joystick_config()

axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'trottle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}


button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

class Joystick(Node):
    def __init__(self, session):
        super(Joystick, self).__init__(session)

        self.jsdev = None
        self.address = joystick_config.path
        self.prev_open_attempt_time = time.time()

        self.axis_states = {}
        self.button_states = {}

        self.axis_map = []
        self.button_map = []

        self.num_axes = 0
        self.num_buttons = 0

        self.axis_events = []
        self.button_events = []

    def start(self):
        if not joystick_config.enabled:
            return
        self.open_joystick()
        self.prev_open_attempt_time = time.time()

    def is_open(self):
        return self.jsdev is not None

    def open_joystick(self):
        try:
            self.jsdev = open(self.address, 'rb')

            logger.info("Joystick: %s" % self.get_device_name())
            self.get_axes_buttons()
            self.get_axis_map()
            self.get_button_map()

            logger.info("%d axes found: %s" % (self.num_axes, ", ".join(self.axis_map)))
            logger.info("%d buttons found: %s" % (self.num_buttons, ", ".join(self.button_map)))
        except FileNotFoundError:
            pass

    def get_device_name(self):
        buf = array.array('B', [0] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')

    def get_axes_buttons(self):
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf) # JSIOCGAXES
        self.num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        self.num_buttons = buf[0]

    def get_axis_map(self):
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf) # JSIOCGAXMAP

        for axis in buf[:self.num_axes]:
            axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0

    def get_button_map(self):
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

        for btn in buf[:self.num_buttons]:
            btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0

    def get_axis_events(self):
        while len(self.axis_events) > 0:
            yield self.axis_events.pop()

        raise StopIteration

    def get_button_events(self):
        while len(self.button_events) > 0:
            yield self.button_events.pop()

        raise StopIteration

    def update(self):
        if not joystick_config.enabled:
            return
        if not self.is_open():
            if time.time() - self.prev_open_attempt_time > 1.0:
                self.open_joystick()
                self.prev_open_attempt_time = time.time()
                if self.jsdev:
                    logger.info("Joystick opened with address {}".format(self.address))
            return
        r, w, e = select.select([self.jsdev], [], [], 0)

        if self.jsdev in r:
            evbuf = self.jsdev.read(8)
        else:
            return

        if evbuf:
            evtime, value, type, number = struct.unpack('IhBB', evbuf)

            # if type & 0x80:
            #      logger.info("(initial)")

            if type & 0x01:
                button = self.button_map[number]
                if button:
                    self.button_states[button] = value
                    self.button_events.append((button, value))
                    # if value:
                    #     logger.info("%s pressed" % (button))
                    # else:
                    #     logger.info("%s released" % (button))

            if type & 0x02:
                axis = self.axis_map[number]
                if axis:
                    fvalue = value / 32767.0
                    self.axis_states[axis] = fvalue
                    self.axis_events.append((axis, fvalue))
                    # logger.info("%s: %.3f" % (axis, fvalue))
