#! /bin/bash

# get build options
# options are:
## - build OR clean OR rebuild

build=1
clean=0

if [[ $1 == '' ]]; then
    echo "No build mode selected"
    echo "Build mode set to: build"
    build=1
    clean=0
else
    if [[ $1 == 'build' ]]; then
        echo "Build mode set to: build"
        build=1
        clean=0
    else
        if [[ $1 == 'clean' ]]; then
            echo "Build mode set to: clean"
            build=0
            clean=1
        else
            if [[ $1 == 'rebuild' ]]; then
                echo "Build mode set to: rebuild"
                build=1
                clean=1
            else
                echo "Unknown argument given"
                echo "Build mode set to: build"
                build=1
            fi
        fi
    fi
fi

if [[ $clean == 1 ]]; then
    if [ -d "../build" ]; then
        echo "Removing build directory ../build/"
        rm -r ../build/
    else
        echo "Build directory does not exist"
    fi
fi

if [[ $build == 1 ]]; then
    echo "Checking build directory"
    if [ ! -d "../build" ]; then
        echo "Creating directory"
        cd ..
        mkdir build
        cd scripts
    fi
    echo "Build is starting"
    cd ../build
    echo "Executing CMake"
    cmake ../src
    echo "Executing Make"
    make -j4
fi