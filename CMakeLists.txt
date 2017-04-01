# Copyright 2016 The carto_release Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 2.8)
project(carto_release)

#Ｃ++11 required
add_compile_options(-std=c++11)

# set version number
set(CARTO_VERSION_MAJOR 1)
set(CARTO_VERSION_MINOR 0)

#TODO: set the cross-compiling envirment
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER "/home/hyb/jzarm/gcc-linaro-arm-linux-gnueabihf-4.7-2013.03-20130313_linux/bin/arm-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "/home/hyb/jzarm/gcc-linaro-arm-linux-gnueabihf-4.7-2013.03-20130313_linux/bin/arm-linux-gnueabihf-g++")

# set(CMAKE_FIND_ROOT_PATH "/usr/include/eigen3/Eigen", "/usr/include/boost", "/usr/include/opencv2" )
# set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# some general setting!
set(CARTO_SOURCE_PATH "src")
set(CARTO_INCLUDE_PATH "include")

# this program needs these libraries!
# find_package(OpenCV REQUIRED)

# set include dir for this project, maybe it is not necessary
include_directories(${CARTO_INLCUDE_PATH} ${CARTO_SOURCE_PATH})

# link_directories(${Boost_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS} ${EIGEN3_LIBRARY_DIRS})
aux_source_directory(${CARTO_SOURCE_PATH} ${CARTO_INLCUDE_PATH} SRC_LIST)

# add_library(${PROJECT_NAME} SHARED ${SRC_LIST})
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries( ${PROJECT_NAME} -pthread)




