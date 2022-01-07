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

set(MULTISENSE_HEADERS include/MultiSense/MultiSenseChannel.hh
                       include/MultiSense/MultiSenseTypes.hh)

set(WIRE_HEADERS
  include/MultiSense/details/wire/PollMotorInfoMessage.h
  include/MultiSense/details/wire/CamControlMessage.h
  include/MultiSense/details/wire/VersionResponseMessage.h
  include/MultiSense/details/wire/SysGetSensorCalibrationMessage.h
  include/MultiSense/details/wire/LedStatusMessage.h
  include/MultiSense/details/wire/SysMtuMessage.h
  include/MultiSense/details/wire/SysLidarCalibrationMessage.h
  include/MultiSense/details/wire/ExposureConfigMessage.h
  include/MultiSense/details/wire/CamHistoryMessage.h
  include/MultiSense/details/wire/LidarSetMotorMessage.h
  include/MultiSense/details/wire/CamGetHistoryMessage.h
  include/MultiSense/details/wire/ImageMessage.h
  include/MultiSense/details/wire/SysFlashResponseMessage.h
  include/MultiSense/details/wire/LidarPollMotorMessage.h
  include/MultiSense/details/wire/SysGetDirectedStreamsMessage.h
  include/MultiSense/details/wire/CamConfigMessage.h
  include/MultiSense/details/wire/SysSetPtpMessage.h
  include/MultiSense/details/wire/SysFlashOpMessage.h
  include/MultiSense/details/wire/Protocol.h
  include/MultiSense/details/wire/StreamControlMessage.h
  include/MultiSense/details/wire/CamSetResolutionMessage.h
  include/MultiSense/details/wire/ImageMetaMessage.h
  include/MultiSense/details/wire/LedSensorStatusMessage.h
  include/MultiSense/details/wire/SysGetMtuMessage.h
  include/MultiSense/details/wire/SysDirectedStreamsMessage.h
  include/MultiSense/details/wire/ImuInfoMessage.h
  include/MultiSense/details/wire/SysGetDeviceInfoMessage.h
  include/MultiSense/details/wire/SysExternalCalibrationMessage.h
  include/MultiSense/details/wire/SysDeviceModesMessage.h
  include/MultiSense/details/wire/SysGetLidarCalibrationMessage.h
  include/MultiSense/details/wire/JpegMessage.h
  include/MultiSense/details/wire/SysGetExternalCalibrationMessage.h
  include/MultiSense/details/wire/SysTransmitDelayMessage.h
  include/MultiSense/details/wire/LidarDataMessage.h
  include/MultiSense/details/wire/SysGetNetworkMessage.h
  include/MultiSense/details/wire/SysTestMtuMessage.h
  include/MultiSense/details/wire/LedGetSensorStatusMessage.h
  include/MultiSense/details/wire/LedSetMessage.h
  include/MultiSense/details/wire/ImuGetConfigMessage.h
  include/MultiSense/details/wire/CamSetTriggerSourceMessage.h
  include/MultiSense/details/wire/SysCameraCalibrationMessage.h
  include/MultiSense/details/wire/StatusResponseMessage.h
  include/MultiSense/details/wire/SysGetCameraCalibrationMessage.h
  include/MultiSense/details/wire/SysDeviceInfoMessage.h
  include/MultiSense/details/wire/DisparityMessage.h
  include/MultiSense/details/wire/VersionRequestMessage.h
  include/MultiSense/details/wire/ImuDataMessage.h
  include/MultiSense/details/wire/SysGetDeviceModesMessage.h
  include/MultiSense/details/wire/ImuConfigMessage.h
  include/MultiSense/details/wire/SysSensorCalibrationMessage.h
  include/MultiSense/details/wire/LedGetStatusMessage.h
  include/MultiSense/details/wire/SysTestMtuResponseMessage.h
  include/MultiSense/details/wire/CamGetConfigMessage.h
  include/MultiSense/details/wire/SysGetTransmitDelayMessage.h
  include/MultiSense/details/wire/SysNetworkMessage.h
  include/MultiSense/details/wire/StatusRequestMessage.h
  include/MultiSense/details/wire/ImuGetInfoMessage.h
  include/MultiSense/details/wire/AckMessage.h
)
set(UTILITY_HEADERS
  include/MultiSense/details/utility/Portability.hh
  include/MultiSense/details/utility/BufferStream.hh
  include/MultiSense/details/utility/Constants.hh
  include/MultiSense/details/utility/Exception.hh
  include/MultiSense/details/utility/TimeStamp.hh
  include/MultiSense/details/utility/Thread.hh
  include/MultiSense/details/utility/ReferenceCount.hh
  include/MultiSense/details/utility/Units.hh
  )

set(DETAILS_HEADERS include/MultiSense/details/channel.hh
                    include/MultiSense/details/query.hh
                    include/MultiSense/details/listeners.hh
                    include/MultiSense/details/signal.hh
                    include/MultiSense/details/storage.hh)

set(LINUX_HEADERS include/MultiSense/details/utility/linux/Thread.hh)
set(WIN32_HEADERS include/MultiSense/details/utility/win32/Thread.hh)
set(MACOS_HEADERS include/MultiSense/details/utility/macos/Thread.hh)

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

add_library(MultiSense ${MULTISENSE_HEADERS}
                       ${DETAILS_HEADERS}
                       ${DETAILS_SRC})
#
# Versioning...someday lets automate this somehow
#

set_target_properties(MultiSense PROPERTIES VERSION "3.9")

target_include_directories(MultiSense PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

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

# create install targets
install(TARGETS MultiSense
  EXPORT MultiSenseConfig
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/MultiSense
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT MultiSenseConfig
    DESTINATION lib/cmake/MultiSense)

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
