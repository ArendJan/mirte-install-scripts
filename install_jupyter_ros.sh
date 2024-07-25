#!/bin/bash
set -xe
MIRTE_SRC_DIR=/usr/local/src/mirte

# install basic python tools
sudo apt install -y python3 python3-venv python3-dev git libffi-dev

# create and activate virtualenv
# Due to a build error on numpy we need to install numpy and
# pandas globally and us it in the virtual environment
# TODO: check if (micro/mino)mamba/conda could fix this
cd /home/mirte || exit 1
sudo apt install -y python3-numpy python3-pandas
python3 -m venv jupyter --system-site-packages
source /home/mirte/jupyter/bin/activate

# install jupyros
sudo apt install -y libjpeg-dev zlib1g-dev
pip3 install wheel
pip3 install markupsafe==2.0.1 pyzmq==24 zipp==3.1.0 \
	ipython==8.7.0 ipykernel==6.17.1 ipywidgets==7.7.2 \
	jupyter-client==7.4.8 jupyter-core==5.1.0 \
	nbclient==0.7.2 nbconvert==7.2.6 nbformat==5.7.0 \
	qtconsole==5.4.0 traitlets==5.6.0 \
	notebook==6.5.2 bqplot==0.12.18 pyyaml \
	jupyros==0.7.0
jupyter nbextension enable --py --sys-prefix jupyros
deactivate
sudo chown -R mirte:mirte /home/mirte/jupyter

# TEMP: download examples
if [ "$UPDATE" ]; then
	cd /home/mirte/jupyter-ros || exit
	git pull
else
	git clone https://github.com/RoboStack/jupyter-ros.git
fi

sudo chown -R mirte:mirte /home/mirte/jupyter-ros

# Add systemd service to start jupyter
sudo rm /lib/systemd/system/mirte-jupyter.service || true
sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-jupyter.service /lib/systemd/system/
