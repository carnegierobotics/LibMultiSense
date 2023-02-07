#
# MultiSense - Makefile
#

#
# Search the source directory for header files.
#

include_directories (
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
# Check for a system-defined implementation of getopt
#

include (CheckFunctionExists)
check_function_exists (getopt HAVE_GETOPT)

if (HAVE_GETOPT)
    add_definitions (-DCRL_HAVE_GETOPT=1)
else ()
    add_subdirectory (portability/getopt)
    set (MULTISENSE_UTILITY_LIBS ${MULTISENSE_UTILITY_LIBS} getopt)
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
add_subdirectory(ExternalCalUtility)
add_subdirectory(FlashUtility)
add_subdirectory(ImageCalUtility)
add_subdirectory(ImuConfigUtility)
add_subdirectory(ImuTestUtility)
add_subdirectory(LidarCalUtility)
add_subdirectory(PointCloudUtility)
add_subdirectory(RectifiedFocalLengthUtility)
add_subdirectory(SaveImageUtility)
add_subdirectory(VersionInfoUtility)
