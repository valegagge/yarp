# Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

set(SRC main.cpp)
add_executable(moduleLoader ${SRC})
target_link_libraries(moduleLoader YARP::YARP_OS
                                   YARP::YARP_init)
