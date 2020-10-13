from .config import Config


class SoundsConfig(Config):
    def __init__(self):
        self.audio_sink = "0"
        self.volume = 0.5
        self.sounds = {}
        super(SoundsConfig, self).__init__("sounds.yaml")

    def to_dict(self):
        return {
            "audio_sink": self.audio_sink,
            "volume": self.volume,
            "sounds": self.sounds,
        }
