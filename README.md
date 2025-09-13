# Interface to Franka robots for [Positronic library](https://github.com/Positronic-Robotics/positronic)

This repository is supposed to be used as a part of `positronic` package, rather than a standalone project.

## Dependencies
This project requires:

- libfranka installed system-wide (CMake package `Franka`)
- Eigen3 available to CMake (e.g. `/usr/share/eigen3/cmake`)

## Developer installation
Build and install into your active virtual environment:

- Ensure your venv is active (e.g., `. .venv/bin/activate` from repo root)
- Initialize submodules, then install:

```
git submodule update --init --recursive
pip install .
```

## Installing libfranka on Ubuntu 24.04 (recommended)
As said, you must have [`libfranka`](https://github.com/frankarobotics/libfranka) installed locally before installing `positronic-franka` (wheels are not provided).

1) System dependencies

```
sudo apt-get update && sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  libpoco-dev \
  libpcre3-dev \
  libfmt-dev
```

2) Install [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)

```
sudo mkdir -p /etc/apt/keyrings
curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | \
  sudo tee /etc/apt/sources.list.d/robotpkg.list

sudo apt-get update && sudo apt-get install -y robotpkg-pinocchio

# Make CMake and the loader find robotpkg installs (Pinocchio)
export ROBOTPKG_PREFIX=/opt/openrobots
export CMAKE_PREFIX_PATH=${ROBOTPKG_PREFIX}:${CMAKE_PREFIX_PATH}
export LD_LIBRARY_PATH=${ROBOTPKG_PREFIX}/lib:${LD_LIBRARY_PATH}
export PKG_CONFIG_PATH=${ROBOTPKG_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH}
```

3) Build and install libfranka from source (0.15.3 recommended)

```
git clone --single-branch --recurse-submodules --branch 0.15.3 https://github.com/frankaemika/libfranka
mkdir -p libfranka/build && cd libfranka/build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build . -j
sudo cmake --install .

# Ensure the loader can find the installed library
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
```
