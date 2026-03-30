if (NOT BUILD_LEGACY_API)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

include(CheckFunctionExists)

option(MULTISENSE_USE_MONOTONIC_CLOCK "Build LibMultiSense to use CLOCK_MONOTONIC for network time sync. Defaults to OFF." OFF)

check_function_exists(vasprintf HAVE_VASPRINTF)

include_directories(include)

set(WIRE_HEADERS
  include/MultiSense/wire/PollMotorInfoMessage.hh
  include/MultiSense/wire/CamControlMessage.hh
  include/MultiSense/wire/VersionResponseMessage.hh
  include/MultiSense/wire/SysGetSensorCalibrationMessage.hh
  include/MultiSense/wire/LedStatusMessage.hh
  include/MultiSense/wire/SysMtuMessage.hh
  include/MultiSense/wire/SysLidarCalibrationMessage.hh
  include/MultiSense/wire/ExposureConfigMessage.hh
  include/MultiSense/wire/CamHistoryMessage.hh
  include/MultiSense/wire/LidarSetMotorMessage.hh
  include/MultiSense/wire/CamGetHistoryMessage.hh
  include/MultiSense/wire/ImageMessage.hh
  include/MultiSense/wire/SysFlashResponseMessage.hh
  include/MultiSense/wire/LidarPollMotorMessage.hh
  include/MultiSense/wire/SysGetDirectedStreamsMessage.hh
  include/MultiSense/wire/CamConfigMessage.hh
  include/MultiSense/wire/SysSetPtpMessage.hh
  include/MultiSense/wire/SysFlashOpMessage.hh
  include/MultiSense/wire/Protocol.hh
  include/MultiSense/wire/StreamControlMessage.hh
  include/MultiSense/wire/CamSetResolutionMessage.hh
  include/MultiSense/wire/ImageMetaMessage.hh
  include/MultiSense/wire/LedSensorStatusMessage.hh
  include/MultiSense/wire/SysGetMtuMessage.hh
  include/MultiSense/wire/SysDirectedStreamsMessage.hh
  include/MultiSense/wire/ImuInfoMessage.hh
  include/MultiSense/wire/SysGetDeviceInfoMessage.hh
  include/MultiSense/wire/SysExternalCalibrationMessage.hh
  include/MultiSense/wire/SysDeviceModesMessage.hh
  include/MultiSense/wire/SysGetLidarCalibrationMessage.hh
  include/MultiSense/wire/JpegMessage.hh
  include/MultiSense/wire/SysGetExternalCalibrationMessage.hh
  include/MultiSense/wire/SysTransmitDelayMessage.hh
  include/MultiSense/wire/LidarDataMessage.hh
  include/MultiSense/wire/SysGetNetworkMessage.hh
  include/MultiSense/wire/SysTestMtuMessage.hh
  include/MultiSense/wire/LedGetSensorStatusMessage.hh
  include/MultiSense/wire/LedSetMessage.hh
  include/MultiSense/wire/ImuGetConfigMessage.hh
  include/MultiSense/wire/CamSetTriggerSourceMessage.hh
  include/MultiSense/wire/SysCameraCalibrationMessage.hh
  include/MultiSense/wire/StatusResponseMessage.hh
  include/MultiSense/wire/SysGetCameraCalibrationMessage.hh
  include/MultiSense/wire/SysDeviceInfoMessage.hh
  include/MultiSense/wire/DisparityMessage.hh
  include/MultiSense/wire/VersionRequestMessage.hh
  include/MultiSense/wire/ImuDataMessage.hh
  include/MultiSense/wire/SysGetDeviceModesMessage.hh
  include/MultiSense/wire/ImuConfigMessage.hh
  include/MultiSense/wire/SysSensorCalibrationMessage.hh
  include/MultiSense/wire/LedGetStatusMessage.hh
  include/MultiSense/wire/SysTestMtuResponseMessage.hh
  include/MultiSense/wire/CamGetConfigMessage.hh
  include/MultiSense/wire/SysGetTransmitDelayMessage.hh
  include/MultiSense/wire/SysNetworkMessage.hh
  include/MultiSense/wire/StatusRequestMessage.hh
  include/MultiSense/wire/ImuGetInfoMessage.hh
  include/MultiSense/wire/AckMessage.hh
  include/MultiSense/wire/ApriltagDetections.hh
  include/MultiSense/wire/RemoteHeadControlMessage.hh
  include/MultiSense/wire/RemoteHeadConfigMessage.hh
  include/MultiSense/wire/RemoteHeadGetConfigMessage.hh
  include/MultiSense/wire/PtpStatusRequestMessage.hh
  include/MultiSense/wire/PtpStatusResponseMessage.hh
  include/MultiSense/wire/SysPacketDelayMessage.hh
  include/MultiSense/wire/SysGetPacketDelayMessage.hh
)
set(UTILITY_HEADERS
  include/MultiSense/utility/Portability.hh
  include/MultiSense/utility/BufferStream.hh
  include/MultiSense/utility/Exception.hh
  include/MultiSense/utility/TimeStamp.hh
  )

set(DETAILS_SRC source/utility/TimeStamp.cc
                source/utility/Exception.cc)

if(NOT HAVE_VASPRINTF)
    add_definitions(-DNEED_VASPRINTF=1)
endif()

if (${BUILD_SHARED_LIBS})
    add_library(MultiSenseWire SHARED ${WIRE_HEADERS}
                                       ${UTILITY_HEADERS}
                                       ${DETAILS_SRC})
else()
    add_library(MultiSenseWire STATIC  ${WIRE_HEADERS}
                                       ${UTILITY_HEADERS}
                                       ${DETAILS_SRC})
    set_target_properties(MultiSenseWire PROPERTIES POSITION_INDEPENDENT_CODE ON)

endif()

target_compile_definitions(MultiSenseWire PRIVATE MultiSense_EXPORTS=True)
if (MULTISENSE_USE_MONOTONIC_CLOCK)
    target_compile_definitions(MultiSenseWire PRIVATE USE_MONOTONIC_CLOCK=${MULTISENSE_USE_MONOTONIC_CLOCK})
endif()

target_include_directories(MultiSenseWire
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PUBLIC $<INSTALL_INTERFACE:include>)

set_target_properties(MultiSenseWire PROPERTIES VERSION "${version}")

if (BUILD_TESTS)
    add_subdirectory(test)
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/MultiSenseWireConfig.cmake.in
                              ${CMAKE_CURRENT_BINARY_DIR}/MultiSenseWireConfig.cmake
                              INSTALL_DESTINATION lib/cmake/MultiSenseWire)

write_basic_package_version_file(
"${CMAKE_CURRENT_BINARY_DIR}/MultiSenseWireConfigVersion.cmake"
  VERSION "${version}"
  COMPATIBILITY AnyNewerVersion
  )

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/MultiSenseWireConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/MultiSenseWireConfigVersion.cmake
    DESTINATION lib/cmake/MultiSenseWire)

# create install targets
install(TARGETS MultiSenseWire
  EXPORT MultiSenseWireTargets
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/MultiSense
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT MultiSenseWireTargets
DESTINATION lib/cmake/MultiSenseWire)

install(FILES ${WIRE_HEADERS} DESTINATION include/MultiSense/wire)
install(FILES ${UTILITY_HEADERS} DESTINATION include/MultiSense/utility)
