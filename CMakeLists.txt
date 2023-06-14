# @file CMakeLists.txt
#
# Top level build instructions.
#

#
# Make sure there is a reasonable version of CMAKE installed.
#

cmake_minimum_required(VERSION 3.1)

set(CPACK_PACKAGE_VERSION "6.1.0")

if (${CMAKE_VERSION} VERSION_LESS "3.12")
    project(LibMultiSense
      LANGUAGES C CXX
      VERSION ${CPACK_PACKAGE_VERSION})
else()
    project(LibMultiSense
      LANGUAGES C CXX
      HOMEPAGE_URL https://carnegierobotics.com
      DESCRIPTION "The CRL Multisense Interface Library"
      VERSION ${CPACK_PACKAGE_VERSION})
endif ()

include (CheckCXXSourceCompiles)

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    string (REGEX REPLACE "/W3" "/W4" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    string (REGEX REPLACE "/W3" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    add_compile_options(/WX)

    add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_WINSOCK_DEPRECATED_NO_WARNINGS)
else()
    add_compile_options(-Wall -Wextra -Werror -Wpedantic -Wno-variadic-macros)
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    #
    # Fix warnings with __VA__OPT__ when we port to c++2a
    # https://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html
    #
    add_compile_options(-Wno-gnu-zero-variadic-macro-arguments)
endif()

option (MULTISENSE_BUILD_UTILITIES "Build MultiSense utility applications. Defaults to ON for backwards compatibility." ON)
option (BUILD_SHARED_LIBS "Build LibMultiSense as a shared library. Defaults to ON." ON)

#
# We want to build "Release" by default
#

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Selecting default build type: Release")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default: Release)" FORCE)
endif ()

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
# Write target outputs to bin and lib directories
#
if (NOT CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif ()
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif ()
if (NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif ()

#
# Dispatch to subordinate CMakeList.txt files.
#

add_subdirectory(source)

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
