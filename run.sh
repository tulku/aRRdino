#!/usr/bin/env bash

mkdir -p r2r
mkdir -p cache/.cargo
docker run -v `pwd`/r2r:/r2r -v `pwd`/cache:/home/eku/ --rm -it eku-humble /bin/bash