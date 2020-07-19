#!/usr/bin/env bash

echo "Running rover6 systemd service uninstall script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/rover6
fi

rm -r ~/.config/systemd/user/${SERVICE_NAME}

echo "Disabling systemd services"
systemctl --user daemon-reload
systemctl --user stop ${SERVICE_NAME}
loginctl enable-linger $USER
systemctl --user disable ${SERVICE_NAME}
echo "rover6 systemd service uninstallation complete"
