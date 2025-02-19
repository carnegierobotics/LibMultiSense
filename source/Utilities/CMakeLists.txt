#
# Check for a system-defined implementation of getopt
#

include (CheckFunctionExists)
check_function_exists (getopt HAVE_GETOPT)

include_directories(portability)

if (HAVE_GETOPT)
    add_definitions (-DCRL_HAVE_GETOPT=1)
else ()
    add_subdirectory (portability/getopt)
    set (MULTISENSE_UTILITY_LIBS getopt)
endif ()

if (${BUILD_LEGACY_API})
    add_subdirectory(Legacy)
else()
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    add_subdirectory(LibMultiSense)
endif()
