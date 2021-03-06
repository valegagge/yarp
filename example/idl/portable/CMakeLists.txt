# Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

cmake_minimum_required(VERSION 3.5)
project(portable_example)
set(CMAKE_CXX_FLAGS "-Weverything -Wno-c++98-compat -Wno-padded -Wno-shadow")

find_package(YARP COMPONENTS OS idl_tools REQUIRED)

yarp_add_idl(IDL_GEN_FILES SharedData.thrift)

add_executable(sender sender.cpp
                      ${IDL_GEN_FILES})
target_link_libraries(sender PRIVATE YARP::YARP_OS
                                     YARP::YARP_init)

add_executable(receiver receiver.cpp
                        ${IDL_GEN_FILES})
target_link_libraries(receiver PRIVATE YARP::YARP_OS
                                       YARP::YARP_init)
