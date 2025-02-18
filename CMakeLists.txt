# @file CMakeLists.txt
#
# Top level build instructions.
#

#
# Make sure there is a reasonable version of CMAKE installed.
#

cmake_minimum_required(VERSION 3.12)

set(CPACK_PACKAGE_VERSION "7.0.0")

project(LibMultiSense
  LANGUAGES C CXX
  HOMEPAGE_URL https://carnegierobotics.com
  DESCRIPTION "The CRL Multisense Interface Library"
  VERSION ${CPACK_PACKAGE_VERSION})

include (CheckCXXSourceCompiles)

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    string (REGEX REPLACE "/W3" "/W4" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    string (REGEX REPLACE "/W3" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    add_compile_options(/WX)

    add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_WINSOCK_DEPRECATED_NO_WARNINGS)
else()
    add_compile_options(-Wall -Wextra -Werror -Wpedantic -Wno-variadic-macros)
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    #
    # Fix warnings with __VA__OPT__ when we port to c++2a
    # https://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html
    #
    add_compile_options(-Wno-gnu-zero-variadic-macro-arguments)
endif()

option(BUILD_LEGACY_API "Build the legacy MultiSense API. Defaults to On for backwards compatibility." ON)
option(MULTISENSE_BUILD_UTILITIES "Build MultiSense utility applications. Defaults to ON for backwards compatibility." ON)
option(BUILD_SHARED_LIBS "Build LibMultiSense as a shared library. Defaults to ON." ON)
option(BUILD_TESTS "Build LibMultiSense GTests. Defaults to ON." ON)
option(BUILD_OPENCV "Use OpenCV if found on the host system" OFF)
option(BUILD_PYTHON_BINDINGS "Build Python bindings with pybind11." OFF)
option(BUILD_JSON_SERIALIZATION "Build json type serialization with nlohmann json." OFF)

if (BUILD_TESTS)
    enable_testing()
endif()

#
# We want to build "Release" by default
#

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Selecting default build type: Release")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default: Release)" FORCE)
endif ()

#
# The new API requires C++17
#
if (NOT BUILD_LEGACY_API)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

#
# Check if the compiler supports constexpr
#
check_cxx_source_compiles (
    "int main() { constexpr int x = 0; }"
    HAVE_CONSTEXPR
)
if (HAVE_CONSTEXPR)
    add_definitions(-DCRL_HAVE_CONSTEXPR)
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
# Setup optional dependencies
#
if (BUILD_OPENCV)
   find_package(OpenCV QUIET)

  if(OpenCV_FOUND)
    message(STATUS "OpenCV found. Enabling OpenCV-specific code.")
  else()
    message(STATUS "OpenCV not found. Building without OpenCV support.")
  endif()
endif()

if (BUILD_JSON_SERIALIZATION)
    find_package(nlohmann_json 3.9)
    if(nlohmann_json_FOUND)
        message(STATUS "Building JSON serialization helper functions")
    else()
        message(STATUS "Could not find nlohmann json. Skipping JSON serialization helpers")
    endif()
endif()

#
# Dispatch to subordinate CMakeList.txt files.
#

add_subdirectory(source)

if (NOT BUILD_LEGACY_API AND BUILD_PYTHON_BINDINGS)
    add_subdirectory(python)
endif()

#
# Generate pc file, for pkgconfig compatibility
#
if (NOT ${CMAKE_VERSION} VERSION_LESS "3.12")
    file(GENERATE OUTPUT ${CMAKE_BINARY_DIR}/LibMultiSense.pc CONTENT
    "prefix=\"${CMAKE_INSTALL_PREFIX}\"
    exec_prefix=\"\${prefix}\"
    libdir=\"\${prefix}/lib\"
    includedir=\"\${prefix}/include\"
    Name: ${PROJECT_NAME}
    Description: ${CMAKE_PROJECT_DESCRIPTION}
    URL: ${CMAKE_PROJECT_HOMEPAGE_URL}
    Version: ${PROJECT_VERSION}
    Cflags: -I$<JOIN:\"\${includedir}\";$<TARGET_PROPERTY:MultiSense,INTERFACE_SYSTEM_INCLUDE_DIRECTORIES>, -I>
    Libs: -L\"\${libdir}\" -lMultiSense
    ")

    install(FILES ${CMAKE_BINARY_DIR}/LibMultiSense.pc DESTINATION lib/pkgconfig COMPONENT development)
endif ()

#
# Add cpack
#

if(CMAKE_ENABLE_CPACK)
    include(${CMAKE_CURRENT_LIST_DIR}/cmake/cpack.cmake)
endif()
