#
# MultiSense - Makefile
#

#
# Platform checks
#

include(CheckFunctionExists)

check_function_exists(vasprintf HAVE_VASPRINTF)

option(MULTISENSE_INSTALL_WIRE_PROTOCOL "Install low level wire headers, for integration with external event systems" OFF)

#
# Include all of our child directories.
#

include_directories ( .
                      ${BASE_DIRECTORY}/external/include
                    )

set(MULTISENSE_HEADERS MultiSenseChannel.hh
                       MultiSenseTypes.hh)

set(WIRE_HEADERS
  details/wire/PollMotorInfoMessage.h
  details/wire/CamControlMessage.h
  details/wire/VersionResponseMessage.h
  details/wire/SysGetSensorCalibrationMessage.h
  details/wire/LedStatusMessage.h
  details/wire/SysMtuMessage.h
  details/wire/SysLidarCalibrationMessage.h
  details/wire/ExposureConfigMessage.h
  details/wire/CamHistoryMessage.h
  details/wire/LidarSetMotorMessage.h
  details/wire/CamGetHistoryMessage.h
  details/wire/ImageMessage.h
  details/wire/SysFlashResponseMessage.h
  details/wire/LidarPollMotorMessage.h
  details/wire/SysGetDirectedStreamsMessage.h
  details/wire/CamConfigMessage.h
  details/wire/SysSetPtpMessage.h
  details/wire/SysFlashOpMessage.h
  details/wire/Protocol.h
  details/wire/StreamControlMessage.h
  details/wire/CamSetResolutionMessage.h
  details/wire/ImageMetaMessage.h
  details/wire/LedSensorStatusMessage.h
  details/wire/SysGetMtuMessage.h
  details/wire/SysDirectedStreamsMessage.h
  details/wire/ImuInfoMessage.h
  details/wire/SysGetDeviceInfoMessage.h
  details/wire/SysExternalCalibrationMessage.h
  details/wire/SysDeviceModesMessage.h
  details/wire/SysGetLidarCalibrationMessage.h
  details/wire/JpegMessage.h
  details/wire/SysGetExternalCalibrationMessage.h
  details/wire/SysTransmitDelayMessage.h
  details/wire/LidarDataMessage.h
  details/wire/SysGetNetworkMessage.h
  details/wire/SysTestMtuMessage.h
  details/wire/LedGetSensorStatusMessage.h
  details/wire/LedSetMessage.h
  details/wire/ImuGetConfigMessage.h
  details/wire/CamSetTriggerSourceMessage.h
  details/wire/SysCameraCalibrationMessage.h
  details/wire/StatusResponseMessage.h
  details/wire/SysGetCameraCalibrationMessage.h
  details/wire/SysDeviceInfoMessage.h
  details/wire/DisparityMessage.h
  details/wire/VersionRequestMessage.h
  details/wire/ImuDataMessage.h
  details/wire/SysGetDeviceModesMessage.h
  details/wire/ImuConfigMessage.h
  details/wire/SysSensorCalibrationMessage.h
  details/wire/LedGetStatusMessage.h
  details/wire/SysTestMtuResponseMessage.h
  details/wire/CamGetConfigMessage.h
  details/wire/SysGetTransmitDelayMessage.h
  details/wire/SysNetworkMessage.h
  details/wire/StatusRequestMessage.h
  details/wire/ImuGetInfoMessage.h
  details/wire/AckMessage.h
)
set(UTILITY_HEADERS
  details/utility/Portability.hh
  details/utility/BufferStream.hh
  details/utility/Constants.hh
  details/utility/Exception.hh
  details/utility/TimeStamp.hh
  details/utility/Thread.hh
  details/utility/ReferenceCount.hh
  details/utility/Units.hh
  )

set(DETAILS_HEADERS details/channel.hh
                    details/query.hh
                    details/listeners.hh
                    details/signal.hh
                    details/storage.hh)

set(LINUX_HEADERS details/utility/linux/Thread.hh)
set(WIN32_HEADERS details/utility/win32/Thread.hh)
set(MACOS_HEADERS details/utility/macos/Thread.hh)

set(DETAILS_SRC details/channel.cc
                details/public.cc
                details/constants.cc
                details/flash.cc
                details/dispatch.cc
                details/utility/Constants.cc
                details/utility/TimeStamp.cc
                details/utility/Exception.cc)

if(NOT HAVE_VASPRINTF)
    add_definitions(-DNEED_VASPRINTF=1)
endif()

#
# Add in all of the source files in this directory.
#

add_library(MultiSense SHARED ${MULTISENSE_HEADERS}
                              ${DETAILS_HEADERS}
                              ${DETAILS_SRC})
#
# Versioning...someday lets automate this somehow
#
set(version "3.9")
set_target_properties(MultiSense PROPERTIES VERSION "${version}")

target_include_directories(MultiSense INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:include>)

set_target_properties(MultiSense PROPERTIES PUBLIC_HEADER "${MULTISENSE_HEADERS}")

#
# We want to link against our child libraries.
#

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_link_libraries(MultiSense ws2_32)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(MultiSense pthread)
else()
    target_link_libraries(MultiSense pthread rt)
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/MultiSenseConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfig.cmake
  INSTALL_DESTINATION lib/cmake/MultiSense)

# create install targets
install(TARGETS MultiSense
  EXPORT MultiSenseTargets
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/MultiSense
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

if(MULTISENSE_INSTALL_WIRE_PROTOCOL)
  install(FILES
    ${WIRE_HEADERS}
    DESTINATION
    include/MultiSense/details/wire)
  install(FILES
    ${UTILITY_HEADERS}
    DESTINATION
    include/MultiSense/details/utility)
  install(FILES
    ${DETAILS_HEADERS}
    DESTINATION
    include/MultiSense/details)
  if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    install(FILES
      ${LINUX_HEADERS}
      DESTINATION
      include/MultiSense/details/utility/linux)
  endif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
  if (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    install(FILES
      ${MACOS_HEADERS}
      DESTINATION
      include/MultiSense/details/utility/macos)
  endif (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    install(FILES
      ${WIN32_HEADERS}
      DESTINATION
      include/MultiSense/details/utility/win32)
  endif (CMAKE_SYSTEM_NAME STREQUAL "Windows")
endif(MULTISENSE_INSTALL_WIRE_PROTOCOL)

install(EXPORT MultiSenseTargets
  DESTINATION lib/cmake/MultiSense)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfigVersion.cmake"
  VERSION "${version}"
  COMPATIBILITY AnyNewerVersion
  )

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfigVersion.cmake
  DESTINATION lib/cmake/MultiSense)


install(FILES ${MULTISENSE_HEADERS} DESTINATION include/MultiSense)
