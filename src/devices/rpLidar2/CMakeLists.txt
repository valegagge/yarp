# Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(rpLidar2
                    CATEGORY device
                    TYPE RpLidar2
                    INCLUDE rpLidar2.h
                    DEPENDS "NOT APPLE;TARGET YARP::YARP_math")

if(NOT SKIP_rpLidar2)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  if(WIN32)
    set(RPLIDAR_ARCH_DIR win32)
  elseif(APPLE)
    set(RPLIDAR_ARCH_DIR macOS)
  elseif(UNIX)
    set(RPLIDAR_ARCH_DIR linux)
  else()
    message(SEND_ERROR "rpLidar2: Unknown architecture")
  endif()

  set(RPLIDAR_SDK_SRCS ${RPLIDAR_SDK_ROOT}/src/rplidar_driver.cpp
                       ${RPLIDAR_SDK_ROOT}/src/arch/${RPLIDAR_ARCH_DIR}/net_serial.cpp
                       ${RPLIDAR_SDK_ROOT}/src/arch/${RPLIDAR_ARCH_DIR}/net_socket.cpp
                       ${RPLIDAR_SDK_ROOT}/src/arch/${RPLIDAR_ARCH_DIR}/timer.cpp
                       ${RPLIDAR_SDK_ROOT}/src/hal/thread.cpp)

  add_library(rpLidar2_sdk OBJECT ${RPLIDAR_SDK_SRCS})
  target_compile_options(rpLidar2_sdk PRIVATE -w)
  target_include_directories(rpLidar2_sdk SYSTEM PUBLIC ${RPLIDAR_SDK_ROOT}/include
                                                        ${RPLIDAR_SDK_ROOT}/src)

  yarp_add_plugin(yarp_rpLidar2 rpLidar2.h
                                rpLidar2.cpp
                                $<TARGET_OBJECTS:rpLidar2_sdk>)
  target_compile_definitions(yarp_rpLidar2 PRIVATE _USE_MATH_DEFINES)
  target_include_directories(yarp_rpLidar2 SYSTEM PRIVATE $<TARGET_PROPERTY:rpLidar2_sdk,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries(yarp_rpLidar2 PRIVATE YARP::YARP_OS
                                              YARP::YARP_sig
                                              YARP::YARP_dev
                                              YARP::YARP_math)
  list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS YARP_OS
                                                      YARP_sig
                                                      YARP_dev)

  yarp_install(TARGETS yarp_rpLidar2
               EXPORT YARP_${YARP_PLUGIN_MASTER}
               COMPONENT ${YARP_PLUGIN_MASTER}
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET yarp_rpLidar2 PROPERTY FOLDER "Plugins/Device")
endif()
