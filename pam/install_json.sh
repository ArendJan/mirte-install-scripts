#!/bin/bash
git clone https://github.com/json-c/json-c.git -b json-c-0.17
cd json-c || exit 1
mkdir build
cd build || exit 1
cmake -DCMAKE_BUILD_TYPE=Release ..
make
make install # requires sudo privileges
cd ../../
rm -rf json-c
ldconfig
