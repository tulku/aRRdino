#!/usr/bin/env bash

docker build --build-arg USERID=$(id -u) -t eku-humble .
