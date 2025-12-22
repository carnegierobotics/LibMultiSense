
find_package(Threads REQUIRED)
list(APPEND MULTISENSE_UTILITY_LIBS
    MultiSense
    Threads::Threads
)

if(CMAKE_SYSTEM_NAME STREQUAL "Windows" OR CMAKE_SYSTEM_NAME STREQUAL "WindowsStore")
    list(APPEND MULTISENSE_UTILITY_LIBS ws2_32)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    list(APPEND MULTISENSE_UTILITY_LIBS rt)
endif()

add_subdirectory(ChangeIpUtility)
add_subdirectory(DeviceInfoUtility)
add_subdirectory(ImageCalUtility)
add_subdirectory(MultiChannelUtility)
add_subdirectory(PointCloudUtility)
add_subdirectory(SaveImageUtility)
add_subdirectory(RectifiedFocalLengthUtility)
add_subdirectory(VersionInfoUtility)
