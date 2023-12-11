#!/bin/bash

cd pam || exit
sudo apt-get install cmake libpam0g-dev -y
mkdir build
cd build || exit 1
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
sudo make install -j # requires sudo privileges
