#! /bin/sh

rm -r ../build/

cd ..
mkdir build
cd build

cmake ../src
make -j4