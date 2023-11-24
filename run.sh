#!/usr/bin/env bash
xhost +

docker run \
    --gpus all \
    --env DISPLAY \
    --env SSH_AUTH_SOCK=/ssh-agent \
    -v ${SSH_AUTH_SOCK}:/ssh-agent \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v `pwd`/ferret_msgs:/home/eku/ros_ws/src/ \
    -v `pwd`/ferret:/home/eku/ferret/ \
    -v `pwd`/r2r:/r2r \
    --rm -it eku-humble /bin/bash