#
# MultiSense - Makefile
#

set(MULTISENSE_SHARED_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/Shared")

add_subdirectory(Wire)

if (${BUILD_LEGACY_API})
    add_subdirectory(Legacy)
else ()
    add_subdirectory(LibMultiSense)
endif()

if (${MULTISENSE_BUILD_UTILITIES})
    add_subdirectory(Utilities)
endif ()

