# @file CMakeLists.txt 
# 
# Top level build instructions.
#

#
# Make sure there is a reasonable version of CMAKE installed.
#

cmake_minimum_required(VERSION 2.8)

project(LibMultiSense)

# For Backwards compatablity with other ROS builds
if (BASE_DIRECTORY)
else()
    set(BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    set(SOURCE_DIRECTORY )
endif()

# Define where the result of the build should go.

if (CMAKE_RUNTIME_OUTPUT_DIRECTORY)
else()
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIRECTORY}/bin)
endif()

if (CMAKE_LIBRARY_OUTPUT_DIRECTORY)
else()
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIRECTORY}/bin)
endif()

#
# We want to build "Release" by default
#

if (CMAKE_BUILD_TYPE)
else()
  set(CMAKE_BUILD_TYPE "Release")
endif()

#
# Turn on optimizations and maximum warnings.
#

# Turn on debugging symbols / optimizations and warnings.  To choose
# which build type, use "cmake -DCMAKE_BUILD_TYPE=Debug <directory>" or
# "cmake -DCMAKE_BUILD_TYPE=Release <directory>".

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")

    set(CMAKE_C_FLAGS_DEBUG "-Zi -W3")
    set(CMAKE_CXX_FLAGS_DEBUG "-Zi -W3")

    set(CMAKE_C_FLAGS_RELEASE "-Zi -O2 -W3 -Gy -Oy")
    set(CMAKE_CXX_FLAGS_RELEASE "-Zi -O2 -W3 -Gy -Oy")

	#TODO: MSVC warns about deprecated functions. These 
    add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_WINSOCK_DEPRECATED_NO_WARNINGS)

else ()

    set(CMAKE_C_FLAGS_DEBUG "-g -Wall -fstrict-aliasing")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -fstrict-aliasing")

    set(CMAKE_C_FLAGS_RELEASE "-g -O2 -Wall -ffunction-sections -fomit-frame-pointer -Wall -fstrict-aliasing")
    set(CMAKE_CXX_FLAGS_RELEASE "-g -O2 -Wall -ffunction-sections -fomit-frame-pointer -Wall -fstrict-aliasing")

endif ()

# Dispatch to subordinate CMakeList.txt files.

add_subdirectory(source)

