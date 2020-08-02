from .log_config import LogConfig
from .device_port_config import DevicePortConfig
from .general_config import GeneralConfig
from .joystick_config import JoystickConfig
from .data_log_config import DataLogConfig


class ConfigManager:
    log_config = None
    device_port_config = None
    general_config = None
    joystick_config = None
    data_log_config = None

    def __init__(self):
        raise Exception("{} is class only".format(self.__class__.__name__))

    @classmethod
    def get_log_config(cls):
        if cls.log_config is None:
            cls.log_config = LogConfig()
        return cls.log_config

    @classmethod
    def get_device_port_config(cls):
        if cls.device_port_config is None:
            cls.device_port_config = DevicePortConfig()
        return cls.device_port_config

    @classmethod
    def get_general_config(cls):
        if cls.general_config is None:
            cls.general_config = GeneralConfig()
        return cls.general_config

    @classmethod
    def get_joystick_config(cls):
        if cls.joystick_config is None:
            cls.joystick_config = JoystickConfig()
        return cls.joystick_config

    @classmethod
    def get_data_log_config(cls):
        if cls.data_log_config is None:
            cls.data_log_config = DataLogConfig()
        return cls.data_log_config
