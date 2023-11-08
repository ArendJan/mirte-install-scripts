#!/bin/bash

cd pam || exit
sudo apt-get install libpam0g-dev
mkdir build
cd build || exit 1
cmake -DCMAKE_BUILD_TYPE=Release .. # will install json-c when not yet installed
make
make install # requires sudo privileges
