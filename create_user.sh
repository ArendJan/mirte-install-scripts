#!/bin/bash
set -ex

MIRTE_SRC_DIR=/usr/local/src/mirte
# Add mirte user with sudo rights
#TODO: user without homedir (create homedir for user)
sudo useradd -m -G sudo,audio -s /bin/bash mirte
sudo mkdir /home/mirte/workdir
sudo chown mirte:mirte /home/mirte/workdir

sudo ./install_pam.sh # setup pam before changing the password for the mirte user.

echo "mirte:mirte_mirte" | sudo chpasswd
echo "root:mirte_mirte" | sudo chpasswd

sudo mkdir -p $MIRTE_SRC_DIR
sudo chown mirte:mirte $MIRTE_SRC_DIR

# For Raspberry create ssh keys
sudo mkdir /home/mirte/.ssh
sudo ssh-keygen -q -t rsa -N '' -f /home/mirte/.ssh/id_rsa <<<y >/dev/null 2>&1
sudo chown mirte:mirte -R /home/mirte/.ssh
