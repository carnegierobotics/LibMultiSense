#
# MultiSense - Makefile
#

#
# Search the source directory for header files.
#

include_directories (
    shared
    ${BASE_DIRECTORY}${SOURCE_DIRECTORY}/source
)

#
# Common libraries that each utility needs to link with.
#

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")

set (MULTISENSE_UTILITY_LIBS
    MultiSense
    ws2_32
)

elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

set (MULTISENSE_UTILITY_LIBS
    MultiSense
    pthread
)

else ()

set (MULTISENSE_UTILITY_LIBS
    MultiSense
    pthread
    rt
)

endif ()

#
# Recurse into each of these directories.
#

add_subdirectory(AprilTagTestUtility)
add_subdirectory(ChangeFps)
add_subdirectory(ChangeIpUtility)
add_subdirectory(ChangeResolution)
add_subdirectory(ChangeTransmitDelay)
add_subdirectory(ColorImageUtility)
add_subdirectory(DeviceInfoUtility)
add_subdirectory(DepthImageUtility)
add_subdirectory(ExternalCalUtility)
add_subdirectory(FeatureDetectorUtility)
add_subdirectory(FirmwareUpdateUtility)
add_subdirectory(ImageCalUtility)
add_subdirectory(ImuConfigUtility)
add_subdirectory(ImuTestUtility)
add_subdirectory(LidarCalUtility)
add_subdirectory(PointCloudUtility)
add_subdirectory(RectifiedFocalLengthUtility)
add_subdirectory(SaveImageUtility)
add_subdirectory(VersionInfoUtility)
