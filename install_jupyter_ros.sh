#!/bin/bash

# install basic python tools
sudo apt install -y python2.7 python-virtualenv python-dev git

# create and activate virtualenv
cd /home/zoef
python -m virtualenv jupyter
source /home/zoef/jupyter/bin/activate

# install jupyros
pip install jupyter bqplot pyyaml ipywidgets
pip install jupyros
jupyter nbextension enable --py --sys-prefix jupyros
deactivate
sudo chown -R zoef:zoef /home/zoef/jupyter

# TEMP: download examples
git clone https://github.com/RoboStack/jupyter-ros.git
sudo chown -R zoef:zoef /home/zoef/jupyter-ros

# Add systemd service to start jupyter
sudo rm /lib/systemd/system/zoef_jupyter.service
sudo cp /home/zoef/zoef_install_scripts/zoef_jupyter.service /lib/systemd/system/

sudo systemctl daemon-reload
sudo systemctl stop zoef_jupyter || /bin/true
sudo systemctl start zoef_jupyter
sudo systemctl enable zoef_jupyter

