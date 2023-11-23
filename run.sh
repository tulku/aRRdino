#!/usr/bin/env bash

mkdir -p workspace
docker run -v `pwd`/workspace:/workspace --rm -it eku /bin/bash