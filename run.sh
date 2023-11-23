#!/usr/bin/env bash

mkdir -p workspace/src
docker run -v `pwd`/r2r:/r2r --rm -it eku-humble /bin/bash