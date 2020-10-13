from .log_config import LogConfig
from .device_port_config import DevicePortConfig
from .general_config import GeneralConfig
from .joystick_config import JoystickConfig
from .data_log_config import DataLogConfig
from .battery_config import BatteryConfig
from .robot_config import RobotConfig
from .sounds_config import SoundsConfig


def get_config_base(cls, name):
    if name not in cls.instances:
        cls.instances[name] = cls.configs[name]()
    return cls.instances[name]

class ConfigManager:
    configs = dict(
        log_config=LogConfig,
        device_port_config=DevicePortConfig,
        general_config=GeneralConfig,
        joystick_config=JoystickConfig,
        data_log_config=DataLogConfig,
        battery_config=BatteryConfig,
        robot_config=RobotConfig,
        sounds_config=SoundsConfig,
    )
    instances = {}


    def __init__(self):
        raise Exception("{} is class only".format(self.__class__.__name__))

    @classmethod
    def init_configs(cls):
        for name, config_cls in cls.configs.items():
            cls.instances[name] = config_cls()
            instance = cls.instances[name]
            setattr(cls, "get_" + name, classmethod(lambda c, instance=instance: instance))


ConfigManager.init_configs()
