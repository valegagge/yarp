# Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(HDRS module.h)

yarp_prepare_plugin(myModuleBar
                    CATEGORY rfmodule
                    TYPE moduleBar
                    INCLUDE module.h)

if(ENABLE_myModules_myModuleBar)
  yarp_add_plugin(module_Bar ${HDRS})
  target_link_libraries(module_Bar YARP::YARP_OS)
endif()

#create the executable target
add_executable(myModuleBarExe main.cpp ${SRC} ${HDRS})
target_link_libraries(myModuleBarExe YARP::YARP_OS YARP::YARP_init)
