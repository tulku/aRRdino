#!/usr/bin/env bash

mkdir workspace
docker run -v ./workspace:/workspace --rm -it eku /bin/bash