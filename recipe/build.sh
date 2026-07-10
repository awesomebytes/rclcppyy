#!/bin/bash
set -euxo pipefail

# Standard ament_cmake out-of-source build. rclcppyy has no compiled targets of
# its own (ament_python_install_package + install(DIRECTORY) only), so this just
# configures ament, installs the Python package into $PREFIX/lib/pythonX.Y/
# site-packages, the scripts into $PREFIX/lib/rclcppyy, and the ament index
# markers into $PREFIX/share.
mkdir -p build
cd build

cmake "${SRC_DIR}" \
  ${CMAKE_ARGS} \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DCMAKE_PREFIX_PATH="${PREFIX}" \
  -DPython3_EXECUTABLE="${PYTHON}" \
  -DPYTHON_EXECUTABLE="${PYTHON}" \
  -DBUILD_TESTING=OFF

cmake --build . --parallel "${CPU_COUNT:-1}"
cmake --install .
