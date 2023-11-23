#!/usr/bin/env bash

mkdir -p r2r
docker run \
    -v `pwd`/ferret_msgs:/home/eku/ros_ws/src/ \
    -v `pwd`/ferret:/home/eku/ \
    -v `pwd`/r2r:/r2r \
    --rm -it eku-humble /bin/bash