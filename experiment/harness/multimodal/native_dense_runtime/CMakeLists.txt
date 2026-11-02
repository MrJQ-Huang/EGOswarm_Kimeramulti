cmake_minimum_required(VERSION 3.0.2)
project(native_dense_runtime)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/native_dense_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
