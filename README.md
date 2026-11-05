# Interface to Franka robots for [Positronic library](https://github.com/Positronic-Robotics/positronic)

This repository is supposed to be used as a part of `positronic` package, rather than a standalone project.

## Dependencies
This project requires:

- libfranka installed system-wide (CMake package `Franka`), in the version that the robot's system version needs
  (see [Which libfranka](#which-libfranka))
- Eigen3 available to CMake (e.g. `/usr/share/eigen3/cmake`)

## Which libfranka
The robot's system version (shown in Desk) sets the libfranka version. This package builds against both
versions below, and CI builds it against each of them.

| Robot system version | libfranka |
|---|---|
| 5.7.2 up to, not including, 5.9.0 | 0.15.3 |
| 5.9.0 and later | 0.21.3 |

The extension links the libfranka that was on the machine when it was built. libfranka puts its major and minor
version in the library name (`libfranka.so.0.15`, `libfranka.so.0.21`), so an extension built against one version
does not load the other. When you change the libfranka of a machine, build the package again without the cache,
for example `pip install --no-cache-dir --force-reinstall --no-deps positronic-franka`. A tool that caches built
wheels (uv, pip) otherwise installs the old build again.

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
  libfmt-dev \
  libconsole-bridge-dev \
  libtinyxml2-dev
```

libfranka 0.15.3 does not use `libconsole-bridge-dev` and `libtinyxml2-dev`. libfranka 0.21.3 needs them.

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

3) Build and install libfranka from source. Set `LIBFRANKA_VERSION` from the table in
[Which libfranka](#which-libfranka).

```
LIBFRANKA_VERSION=0.15.3  # 0.21.3 for robot system version 5.9.0 and later
git clone --single-branch --recurse-submodules --branch ${LIBFRANKA_VERSION} https://github.com/frankarobotics/libfranka
mkdir -p libfranka/build && cd libfranka/build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build . -j
sudo cmake --install .

# Ensure the loader can find the installed library
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
```

Franka publishes a Debian package for libfranka 0.21.3 (Ubuntu 20.04, 22.04 and 24.04) on its
[releases page](https://github.com/frankarobotics/libfranka/releases/tag/0.21.3). It does not publish one for
0.15.3, so build 0.15.3 from source.
