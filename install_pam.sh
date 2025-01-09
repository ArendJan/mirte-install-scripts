#!/bin/bash

cd pam || exit
wget https://apt.kitware.com/kitware-archive.sh --no-check-certificate
chmod +x kitware-archive.sh
sudo ./kitware-archive.sh
rm kitware-archive.sh
sudo apt update
sudo apt-get install cmake libpam0g-dev -y
mkdir build
cd build || exit 1
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
sudo make install -j # requires sudo privileges
