#!/bin/bash
# Fetches the headers of the shared kinematics core at the commit in shared_kinematics.ref into ./shared-kinematics.
set -euo pipefail
ref=$(head -n 1 shared_kinematics.ref)
git clone --quiet --filter=blob:none --no-checkout https://github.com/Positronic-Robotics/shared-kinematics.git \
  shared-kinematics
git -C shared-kinematics sparse-checkout set include
git -C shared-kinematics checkout --quiet "$ref"
