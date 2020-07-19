from lib.nodes.robot import Robot
from lib.nodes.joystick import Joystick


class Session:
    def __init__(self):
        self.robot = Robot(self)
        self.joystick = Joystick(self)

    def start(self):
        self.joystick.start()
        self.robot.start()

    def update(self):
        self.robot.update()
        self.joystick.update()

    def stop(self):
        self.robot.stop()
