#!/usr/bin/env bash

mkdir -p workspace/src
docker run -v `pwd`/workspace:/workspace --rm -it eku /bin/bash