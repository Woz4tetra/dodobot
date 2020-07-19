#!/usr/bin/env bash

echo "Running dodobot_py install script"

BASE_DIR=$(realpath "$(dirname $0)")
SRC_DIR=${BASE_DIR}/dodobot_py

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=~/.local/dodobot/dodobot_py
fi

chmod +x ${BASE_DIR}/copy.sh
bash ${BASE_DIR}/copy.sh

echo "Installing python dependencies"
pip3 install -U -r ${SRC_DIR}/requirements.txt
sudo apt-get install libatlas-base-dev  # for numpy

bash ${BASE_DIR}/systemd/install_systemd.sh

echo "dodobot_py installation complete"
