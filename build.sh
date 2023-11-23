#!/usr/bin/env bash

docker build --build-arg UID=$(id -u) -t eku-humble .
