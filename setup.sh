#!/usr/bin/env bash
cd r2r
git clone https://github.com/sequenceplanner/r2r.git
cd r2r
. /opt/ros/humble/setup.sh
cargo build
