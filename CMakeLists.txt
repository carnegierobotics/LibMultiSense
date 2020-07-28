# @file CMakeLists.txt
#
# Top level build instructions.
#

#
# Make sure there is a reasonable version of CMAKE installed.
#

cmake_minimum_required(VERSION 2.8)

project(LibMultiSense)

add_compile_options(-Wall -Wextra -Werror -Wpedantic)

option (MULTISENSE_BUILD_UTILITIES "Build MultiSense utility applications. Defaults to ON for backwards compatibility." ON)

#
# We want to build "Release" by default
#

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Selecting default build type: Release")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default: Release)" FORCE)
endif ()

#
# For Backwards compatablity with other ROS builds
#
if (BASE_DIRECTORY)
else()
    set(BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    set(SOURCE_DIRECTORY )
endif()

#
# Use full RPATH
#
set (CMAKE_SKIP_BUILD_RPATH FALSE)
set (CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set (CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#
# Dispatch to subordinate CMakeList.txt files.
#

add_subdirectory(source)
