include(CheckFunctionExists)

option(MULTISENSE_USE_MONOTONIC_CLOCK "Build LibMultiSense to use CLOCK_MONOTONIC for network time sync. Defaults to OFF." OFF)

check_function_exists(vasprintf HAVE_VASPRINTF)

include_directories(include)

set(WIRE_HEADERS
  include/wire/PollMotorInfoMessage.hh
  include/wire/CamControlMessage.hh
  include/wire/VersionResponseMessage.hh
  include/wire/SysGetSensorCalibrationMessage.hh
  include/wire/LedStatusMessage.hh
  include/wire/SysMtuMessage.hh
  include/wire/SysLidarCalibrationMessage.hh
  include/wire/ExposureConfigMessage.hh
  include/wire/CamHistoryMessage.hh
  include/wire/LidarSetMotorMessage.hh
  include/wire/CamGetHistoryMessage.hh
  include/wire/ImageMessage.hh
  include/wire/SysFlashResponseMessage.hh
  include/wire/LidarPollMotorMessage.hh
  include/wire/SysGetDirectedStreamsMessage.hh
  include/wire/CamConfigMessage.hh
  include/wire/SysSetPtpMessage.hh
  include/wire/SysFlashOpMessage.hh
  include/wire/Protocol.hh
  include/wire/StreamControlMessage.hh
  include/wire/CamSetResolutionMessage.hh
  include/wire/ImageMetaMessage.hh
  include/wire/LedSensorStatusMessage.hh
  include/wire/SysGetMtuMessage.hh
  include/wire/SysDirectedStreamsMessage.hh
  include/wire/ImuInfoMessage.hh
  include/wire/SysGetDeviceInfoMessage.hh
  include/wire/SysExternalCalibrationMessage.hh
  include/wire/SysDeviceModesMessage.hh
  include/wire/SysGetLidarCalibrationMessage.hh
  include/wire/JpegMessage.hh
  include/wire/SysGetExternalCalibrationMessage.hh
  include/wire/SysTransmitDelayMessage.hh
  include/wire/LidarDataMessage.hh
  include/wire/SysGetNetworkMessage.hh
  include/wire/SysTestMtuMessage.hh
  include/wire/LedGetSensorStatusMessage.hh
  include/wire/LedSetMessage.hh
  include/wire/ImuGetConfigMessage.hh
  include/wire/CamSetTriggerSourceMessage.hh
  include/wire/SysCameraCalibrationMessage.hh
  include/wire/StatusResponseMessage.hh
  include/wire/SysGetCameraCalibrationMessage.hh
  include/wire/SysDeviceInfoMessage.hh
  include/wire/DisparityMessage.hh
  include/wire/VersionRequestMessage.hh
  include/wire/ImuDataMessage.hh
  include/wire/SysGetDeviceModesMessage.hh
  include/wire/ImuConfigMessage.hh
  include/wire/SysSensorCalibrationMessage.hh
  include/wire/LedGetStatusMessage.hh
  include/wire/SysTestMtuResponseMessage.hh
  include/wire/CamGetConfigMessage.hh
  include/wire/SysGetTransmitDelayMessage.hh
  include/wire/SysNetworkMessage.hh
  include/wire/StatusRequestMessage.hh
  include/wire/ImuGetInfoMessage.hh
  include/wire/AckMessage.hh
  include/wire/ApriltagDetections.hh
  include/wire/RemoteHeadControlMessage.hh
  include/wire/RemoteHeadConfigMessage.hh
  include/wire/RemoteHeadGetConfigMessage.hh
  include/wire/PtpStatusRequestMessage.hh
  include/wire/PtpStatusResponseMessage.hh
  include/wire/SysPacketDelayMessage.hh
  include/wire/SysGetPacketDelayMessage.hh
)
set(UTILITY_HEADERS
  include/utility/Portability.hh
  include/utility/BufferStream.hh
  include/utility/Exception.hh
  include/utility/TimeStamp.hh
  )

set(DETAILS_SRC source/utility/TimeStamp.cc
                source/utility/Exception.cc)

if(NOT HAVE_VASPRINTF)
    add_definitions(-DNEED_VASPRINTF=1)
endif()

add_library(MultiSenseWire ${WIRE_HEADERS}
                           ${UTILITY_HEADERS}
                           ${DETAILS_SRC})

target_compile_definitions(MultiSenseWire PRIVATE MultiSense_EXPORTS=True)
if (MULTISENSE_USE_MONOTONIC_CLOCK)
    target_compile_definitions(MultiSenseWire PRIVATE USE_MONOTONIC_CLOCK=${MULTISENSE_USE_MONOTONIC_CLOCK})
endif()

target_include_directories(MultiSenseWire
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PUBLIC $<INSTALL_INTERFACE:include>)

if (BUILD_TESTS)
    add_subdirectory(test)
endif()

# create install targets
install(TARGETS MultiSenseWire
  EXPORT MultiSenseWireTargets
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/MultiSense
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT MultiSenseWireTargets
  DESTINATION lib/cmake/MultiSense)

install(FILES ${WIRE_HEADERS} DESTINATION include/MultiSense/wire)
install(FILES ${UTILITY_HEADERS} DESTINATION include/MultiSense/utility)
