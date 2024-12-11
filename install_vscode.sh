#!/bin/bash
set -xe
MIRTE_SRC_DIR=/usr/local/src/mirte

# VScode in 2 parts:
# - vscode remote server, when using the ssh plugin from another computer
# - 'vscode' website

# First part:
cd $MIRTE_SRC_DIR || exit
mkdir vscode
cd vscode || exit
wget https://gist.githubusercontent.com/b01/0a16b6645ab7921b0910603dfb85e4fb/raw/ea48d972a176b90b3956de59eb7a43da9be86ec5/download-vs-code-server.sh
chmod +x download-vs-code-server.sh
sudo -u mirte $MIRTE_SRC_DIR/vscode/download-vs-code-server.sh

# Second part:
cd $MIRTE_SRC_DIR/vscode || exit
sudo -u mirte bash -c "curl -fsSL https://code-server.dev/install.sh | sh"
sudo -u mirte bash -c "mkdir -p ~/.config/code-server && cp $MIRTE_SRC_DIR/mirte-install-scripts/config/code_server_config.yaml ~/.config/code-server/config.yaml" || true
sudo systemctl enable code-server@mirte.service || true # Added by the code-server install script
