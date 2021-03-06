# Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
# Copyright (C) 2006-2010 RobotCub Consortium
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

project(YARP_name)

set(YARP_name_HDRS include/yarp/name/all.h
                   include/yarp/name/api.h
                   include/yarp/name/BootstrapServer.h
                   include/yarp/name/NameServerConnectionHandler.h
                   include/yarp/name/NameServerManager.h
                   include/yarp/name/NameService.h)

set(YARP_name_IMPL_HDRS )

set(YARP_name_SRCS src/BootstrapServer.cpp
                   src/NameService.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}"
             PREFIX "Source Files"
             FILES ${YARP_name_SRCS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}"
             PREFIX "Header Files"
             FILES ${YARP_name_HDRS}
                   ${YARP_name_IMPL_HDRS})

add_library(YARP_name ${YARP_name_SRCS}
                      ${YARP_name_HDRS}
                      ${YARP_name_IMPL_HDRS})
add_library(YARP::YARP_name ALIAS YARP_name)

target_include_directories(YARP_name PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(YARP_name PUBLIC YARP::YARP_conf
                                       YARP::YARP_OS)
list(APPEND YARP_name_PUBLIC_DEPS YARP_conf
                                  YARP_OS)

if(YARP_HAS_ACE)
  target_compile_definitions(YARP_name PRIVATE YARP_HAS_ACE)
  target_link_libraries(YARP_name PRIVATE ACE::ACE)
  list(APPEND YARP_name_PRIVATE_DEPS ACE)
endif()

target_compile_features(YARP_name PUBLIC cxx_override)

set_property(TARGET YARP_name PROPERTY PUBLIC_HEADER ${YARP_name_HDRS})
set_property(TARGET YARP_name PROPERTY PRIVATE_HEADER ${YARP_name_IMPL_HDRS})
set_property(TARGET YARP_name PROPERTY VERSION ${YARP_VERSION_SHORT})
set_property(TARGET YARP_name PROPERTY SOVERSION ${YARP_SOVERSION})

install(TARGETS YARP_name
        EXPORT YARP_name
        COMPONENT YARP_name
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/yarp/name"
        PRIVATE_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/yarp/name/impl")

include(YarpInstallBasicPackageFiles)
yarp_install_basic_package_files(YARP_name
                                 DEPENDENCIES ${YARP_name_PUBLIC_DEPS}
                                 PRIVATE_DEPENDENCIES ${YARP_name_PRIVATE_DEPS})

set_property(TARGET YARP_name PROPERTY FOLDER "Libraries/Private")
