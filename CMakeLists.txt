# @file CMakeLists.txt 
# 
# Top level build instructions.
#
# Significant history (date, user, job code, action):
#   2012-05-07, dtascione@carnegierobotics.com, RD1034, Created file.

#
# Make sure there is a reasonable version of CMAKE installed.
#

cmake_minimum_required(VERSION 2.8)

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

set(CMAKE_C_FLAGS_DEBUG "-g -Wall -fstrict-aliasing")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -fstrict-aliasing")

set(CMAKE_C_FLAGS_RELEASE "-g -O2 -Wall -ffunction-sections -fomit-frame-pointer -Wall -fstrict-aliasing")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O2 -Wall -ffunction-sections -fomit-frame-pointer -Wall -fstrict-aliasing")

# Dispatch to subordinate CMakeList.txt files.

add_subdirectory(source)

