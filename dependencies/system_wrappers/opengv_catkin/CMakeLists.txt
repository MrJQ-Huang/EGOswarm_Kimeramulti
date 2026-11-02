cmake_minimum_required(VERSION 3.5)
project(opengv_catkin)
find_package(catkin REQUIRED)
include(ExternalProject)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
ExternalProject_Add(opengv_src
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../opengv
  DOWNLOAD_COMMAND "" UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
             -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_PYTHON=OFF
             -DBUILD_SHARED_LIBS=ON)
catkin_package(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include LIBRARIES opengv)
