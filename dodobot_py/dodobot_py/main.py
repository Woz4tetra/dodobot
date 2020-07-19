
import sys
import time
import subprocess

from lib.config import ConfigManager
from lib.logger_manager import LoggerManager
from lib.exceptions import ShutdownException, LowBatteryException
from lib.session import Session

logger = LoggerManager.get_logger()

general_config = ConfigManager.get_general_config()


def shutdown(session):
    logger.warn("Shutdown function called. Shutting down everything.")
    session.stop()
    subprocess.call("sudo shutdown -h now", shell=True)
    sys.exit()


def close(master):
    logger.info("Close function called. Exiting\n\n")
    master.stop()


def main():
    logger.info("Starting robot")
    session = Session()
    update_delay = 1.0 / general_config.update_rate_hz
    try:
        session.start()
        while True:
            session.update()
            time.sleep(update_delay)
    except LowBatteryException as e:
        logger.error(str(e), exc_info=True)
        shutdown(session)
    except ShutdownException as e:
        logger.error(str(e), exc_info=True)
        shutdown(session)
    except BaseException as e:
        logger.error(str(e), exc_info=True)
        close(session)


if __name__ == "__main__":
    try:
        main()
    except BaseException as e:
        logger.error(str(e), exc_info=True)
