
if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")

set (MULTISENSE_UTILITY_LIBS
    ${MULTISENSE_UTILITY_LIBS}
    MultiSense
    ws2_32
)

elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

set (MULTISENSE_UTILITY_LIBS
    ${MULTISENSE_UTILITY_LIBS}
    MultiSense
    pthread
)

else ()

set (MULTISENSE_UTILITY_LIBS
    ${MULTISENSE_UTILITY_LIBS}
    MultiSense
    pthread
    rt
)

endif ()

add_subdirectory(ColorImageUtility)
add_subdirectory(DepthImageUtility)
add_subdirectory(DeviceInfoUtility)
add_subdirectory(PointCloudUtility)
add_subdirectory(SaveImageUtility)
add_subdirectory(RectifiedFocalLengthUtility)
add_subdirectory(VersionInfoUtility)
