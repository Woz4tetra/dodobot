from lib.nodes.robot import Robot
from lib.nodes.joystick import Joystick
from lib.nodes.data_logger import DataLogger


class Session:
    def __init__(self):
        self.robot = Robot(self)
        self.joystick = Joystick(self)
        self.data_logger = DataLogger(self)

    def start(self):
        self.robot.start()
        self.joystick.start()
        self.data_logger.start()

    def update(self):
        self.robot.update()
        self.joystick.update()
        self.data_logger.update()

    def stop(self):
        self.robot.stop()
        self.joystick.stop()
        self.data_logger.stop()
