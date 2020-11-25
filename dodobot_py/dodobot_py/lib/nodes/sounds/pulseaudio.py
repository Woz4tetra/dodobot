import re
import time
from subprocess import Popen, PIPE

from pydub import AudioSegment
from pydub.playback import _play_with_simpleaudio

from lib.logger_manager import LoggerManager

logger = LoggerManager.get_logger()

# helpful forum posts:
# how to get audio to play:
# https://askubuntu.com/questions/14077/how-can-i-change-the-default-audio-device-from-command-line
#   pacmd list-sinks
#   pacmd set-default-sink 0
# fix volume issue:
# https://chrisjean.com/fix-for-usb-audio-is-too-loud-and-mutes-at-low-volume-in-ubuntu/


class Audio:
    def __init__(self):
        self.audio = None
        self.playback = None

    @classmethod
    def load_from_path(cls, path):
        return cls.load(AudioSegment.from_file(path))

    @classmethod
    def load(cls, audio_segment):
        self = cls()
        self.unload()
        self.audio = audio_segment
        return self

    def unload(self):
        self.stop()
        if self.audio is not None:
            del self.audio
            self.audio = None

    def play(self):
        self.stop()
        if self.audio is not None:
            self.playback = _play_with_simpleaudio(self.audio)

    def is_playing(self):
        return self.playback is not None and self.playback.is_playing()

    def wait(self):
        if self.is_playing():
            self.playback.wait_done()

    def stop(self):
        if self.is_playing():
            self.playback.stop()

class Pacmd:
    def __init__(self, sink: str, volume_raw_min: int, volume_raw_max: int):
        self.sink = str(sink)
        self.volume_raw_min = volume_raw_min
        self.volume_raw_max = volume_raw_max
        self.volume = self.volume_raw_min
        self.sink_timeout_s = 30.0

        self.wait_for_sinks()
        self.set_sink()

    def list_sinks(self):
        return self._run_cmd(["list-sinks"])

    def set_volume(self, ratio):
        # ratio is 0...1
        self.volume = (self.volume_raw_max - self.volume_raw_min) * ratio + self.volume_raw_min
        self.volume = max(min(self.volume, self.volume_raw_max), self.volume_raw_min)
        self.volume = int(self.volume)
        logger.info("Setting volume to %s (%s%%)" % (self.volume, ratio * 100.0))
        self._run_cmd(["set-sink-volume", self.sink, str(self.volume)])

    def wait_for_sinks(self):
        start_time = time.time()
        regex = r"(\d*) sink\(s\) available"
        while time.time() - start_time < self.sink_timeout_s:
            output = self.list_sinks()
            match = re.search(regex, output)
            if match:
                sinks = int(match.group(1))
                if sinks > 0:
                    logger.info("Found %s available sinks" % sinks)
                    return sinks
        logger.error("Failed to find sinks!")
        return 0

    def set_sink(self):
        logger.info("Setting audio sink to %s" % self.sink)
        self._run_cmd(["set-default-sink", self.sink])

    def _run_cmd(self, command: list):
        p = Popen(["pacmd"] + command, stdout=PIPE)
        output = p.communicate()[0]
        output = output.decode()
        logger.debug("pacmd output: '%s'" % output)
        return output
