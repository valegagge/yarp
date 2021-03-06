# Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
# Copyright (C) 2006-2010 RobotCub Consortium
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

project(YARP_sig)

set(YARP_sig_HDRS include/yarp/sig/all.h
                  include/yarp/sig/api.h
                  include/yarp/sig/Image.h
                  include/yarp/sig/ImageDraw.h
                  include/yarp/sig/ImageFile.h
                  include/yarp/sig/ImageNetworkHeader.h
                  include/yarp/sig/ImageUtils.h
                  include/yarp/sig/IntrinsicParams.h
                  include/yarp/sig/Matrix.h
                  include/yarp/sig/PointCloud.h
                  include/yarp/sig/PointCloudBase.h
                  include/yarp/sig/PointCloudNetworkHeader.h
                  include/yarp/sig/PointCloudTypes.h
                  include/yarp/sig/PointCloudUtils.h
                  include/yarp/sig/PointCloudUtils-inl.h
                  include/yarp/sig/SoundFile.h
                  include/yarp/sig/Sound.h
                  include/yarp/sig/Vector.h)

if(NOT YARP_NO_DEPRECATED) # Since YARP 3.0.0
  list(APPEND YARP_sig_HDRS include/yarp/sig/IplImage.h)
endif()

set(YARP_sig_IMPL_HDRS include/yarp/sig/impl/DeBayer.h
                       include/yarp/sig/impl/IplImage.h)

set(YARP_sig_SRCS src/ImageCopy.cpp
                  src/Image.cpp
                  src/ImageFile.cpp
                  src/ImageUtils.cpp
                  src/IntrinsicParams.cpp
                  src/IplImage.cpp
                  src/Matrix.cpp
                  src/PointCloudBase.cpp
                  src/PointCloudUtils.cpp
                  src/Sound.cpp
                  src/SoundFile.cpp
                  src/Vector.cpp
                  src/DeBayer.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}"
             PREFIX "Source Files"
             FILES ${YARP_sig_SRCS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}"
             PREFIX "Header Files"
             FILES ${YARP_sig_HDRS}
                   ${YARP_sig_IMPL_HDRS})

add_library(YARP_sig ${YARP_sig_SRCS}
                     ${YARP_sig_HDRS}
                     ${YARP_sig_IMPL_HDRS})
add_library(YARP::YARP_sig ALIAS YARP_sig)

target_include_directories(YARP_sig PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(YARP_sig PUBLIC YARP::YARP_conf
                                      YARP::YARP_OS)
list(APPEND YARP_sig_PUBLIC_DEPS YARP_conf
                                 YARP_OS)

# YARP_sig library uses headers from YARP_OS impl, and therefore ACE headers
# are also required.
if(YARP_HAS_ACE)
  target_compile_definitions(YARP_sig PRIVATE YARP_HAS_ACE)
  target_link_libraries(YARP_sig PRIVATE ACE::ACE_INLINE)
  list(APPEND YARP_sig_PRIVATE_DEPS ACE)
endif()

if(YARP_HAS_JPEG)
  target_include_directories(YARP_sig SYSTEM PRIVATE ${JPEG_INCLUDE_DIR})
  target_compile_definitions(YARP_sig PRIVATE YARP_HAS_JPEG_C=1)
  target_link_libraries(YARP_sig PRIVATE ${JPEG_LIBRARY})
  list(APPEND YARP_sig_PRIVATE_DEPS JPEG)
endif()

if(YARP_HAS_PNG)
  target_include_directories(YARP_sig SYSTEM PRIVATE ${PNG_INCLUDE_DIR})
  target_compile_definitions(YARP_sig PRIVATE YARP_HAS_PNG)
  target_link_libraries(YARP_sig PRIVATE ${PNG_LIBRARY})
  list(APPEND YARP_sig_PRIVATE_DEPS PNG)
endif()

target_compile_features(YARP_sig PUBLIC cxx_override)

set_property(TARGET YARP_sig PROPERTY PUBLIC_HEADER ${YARP_sig_HDRS})
set_property(TARGET YARP_sig PROPERTY PRIVATE_HEADER ${YARP_sig_IMPL_HDRS})
set_property(TARGET YARP_sig PROPERTY VERSION ${YARP_VERSION_SHORT})
set_property(TARGET YARP_sig PROPERTY SOVERSION ${YARP_SOVERSION})

install(TARGETS YARP_sig
        EXPORT YARP_sig
        COMPONENT YARP_sig
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/yarp/sig
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/yarp/sig/impl)

include(YarpInstallBasicPackageFiles)
yarp_install_basic_package_files(YARP_sig
                                 DEPENDENCIES ${YARP_sig_PUBLIC_DEPS}
                                 PRIVATE_DEPENDENCIES ${YARP_sig_PRIVATE_DEPS})

set_property(TARGET YARP_sig PROPERTY FOLDER "Libraries")
