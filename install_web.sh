#!/bin/bash

MIRTE_SRC_DIR=/usr/local/src/mirte

# Update
sudo apt update

# Install nodeenv
$UPDATE || sudo apt install -y python3-pip python3-setuptools python3-wheel
sudo -H pip install nodeenv

# Install nodeenv
sudo nodeenv --node=16.2.0 $MIRTE_SRC_DIR/mirte-web-interface/node_env

# Install web interface
. $MIRTE_SRC_DIR/mirte-web-interface/node_env/bin/activate

# Install frontend
cd $MIRTE_SRC_DIR/mirte-web-interface/vue-frontend || exit
npm install .
npm run build

# Install backend
cd $MIRTE_SRC_DIR/mirte-web-interface/nodejs-backend || exit
npm install .

# Install wetty
#cd $MIRTE_SRC_DIR/mirte-web-interface
#npm -g install wetty
deactivate_node

# Install strace for linetrace functionality
sudo apt install -y strace

# Install nginx (as reverse proxy to all services)
sudo cp $MIRTE_SRC_DIR/mirte-install-scripts/nginx.conf /etc/nginx/
$UPDATE || sudo apt install -y nginx
sudo cp $MIRTE_SRC_DIR/mirte-install-scripts/nginx.conf /etc/nginx/sites-available/mirte.conf
sudo ln /etc/nginx/sites-available/mirte.conf /etc/nginx/sites-enabled/
sudo rm /etc/nginx/sites-enabled/default #this will catch :80 by default

# give nginx access to that folder to put a new password in:
sudo chown www-data:www-data -R /usr/local/src/mirte/mirte-install-scripts/config/web

# Add systemd service
sudo rm /lib/systemd/system/mirte-web-interface.service
sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-web-interface.service /lib/systemd/system/
sudo systemctl daemon-reload
sudo systemctl stop mirte-web-interface || /bin/true
sudo systemctl start mirte-web-interface
sudo systemctl enable mirte-web-interface
