#!/bin/bash
set -xe
MIRTE_SRC_DIR=/usr/local/src/mirte

cd $MIRTE_SRC_DIR || exit
mkdir vscode
cd vscode || exit
wget https://gist.githubusercontent.com/b01/0a16b6645ab7921b0910603dfb85e4fb/raw/ea48d972a176b90b3956de59eb7a43da9be86ec5/download-vs-code-server.sh
chmod +x download-vs-code-server.sh
sudo -u mirte $MIRTE_SRC_DIR/vscode/download-vs-code-server.sh


cd /home/mirte/
# For the website:
sudo -u mirte wget -O vscode_cli.tar.gz https://az764295.vo.msecnd.net/stable/f1b07bd25dfad64b0167beb15359ae573aecd2cc/vscode_cli_alpine_arm64_cli.tar.gz
sudo -u mirte tar -xvf vscode_cli.tar.gz 
sudo -u mirte rm vscode_cli.tar.gz
sudo -u mirte ./code update # update the server

# first load it will trigger a download of the actual server. The Mirtes don't have networking, so download it during sd generation
sudo -u mirte ./code serve-web --port 9000 --host 0.0.0.0 --without-connection-token --accept-server-license-terms &
code_pid=$!
until [ "$(wget -qO- http://localhost:9000/ | wc --bytes)" -gt "1000" ]; do
	echo "wait for vscode"
	sleep 5
done

# Add the license terms after <!--TERMS--> in the vscode/index.html file that the users must accept before using it.
# rerun vscode with the same port to let it stop immediately, but it will show the license terms
sudo sed -i '/^<\!--TERMS-->$/r'<(./code serve-web --port 9000 --host 0.0.0.0 | grep -v error) $MIRTE_SRC_DIR/mirte-install-scripts/sites/vscode/index.html

# Stop the server started earlier
sudo kill $code_pid

sudo ln -s $MIRTE_SRC_DIR/mirte-install-scripts/services/mirte-vscode.service /lib/systemd/system/
sudo systemctl enable mirte-vscode
