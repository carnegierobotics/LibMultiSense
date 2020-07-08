# TODO(ajakhotia): Evaluate the need and remove unnecessary parts.
# #
# # MultiSense - Makefile
# #
#
# #
# # Search the source directory for header files.
# #
#
# include_directories (
#     ${BASE_DIRECTORY}${SOURCE_DIRECTORY}/source
# )
#
# #
# # Common libraries that each utility needs to link with.
# #
#
# if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
#
# set (MULTISENSE_UTILITY_LIBS
#     MultiSense
#     ws2_32
# )
#
# elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#
# set (MULTISENSE_UTILITY_LIBS
#     MultiSense
#     pthread
# )
#
# else ()
#
# set (MULTISENSE_UTILITY_LIBS
#     MultiSense
#     pthread
#     rt
# )
#
# endif ()

#
# Check for a system-defined implementation of getopt
#



#
# Recurse into each of these directories.
#
add_subdirectory(portability)
add_subdirectory(shared)
add_subdirectory(FlashUtility)
add_subdirectory(ChangeIpUtility)
add_subdirectory(DeviceInfoUtility)
add_subdirectory(SaveImageUtility)
add_subdirectory(DirectedStreamsUtility)
add_subdirectory(ImuTestUtility)
add_subdirectory(ImuConfigUtility)
add_subdirectory(ImageCalUtility)
add_subdirectory(SensorCalUtility)
add_subdirectory(LidarCalUtility)
add_subdirectory(ExternalCalUtility)
add_subdirectory(ChangeResolution)
add_subdirectory(ChangeTransmitDelay)
add_subdirectory(ChangeFps)
