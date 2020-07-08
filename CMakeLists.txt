cmake_minimum_required(VERSION 3.5)

project(LibMultiSense VERSION 3.9.0.0 LANGUAGES CXX C)

add_compile_options(-Wall -Wextra -Werror -Wpedantic)

option(MULTISENSE_BUILD_UTILITIES "Build MultiSense utility applications. Defaults to ON for backwards compatibility." ON)


# We want to build "Release" by default.
# NOTE: Cmake cache variable are not over written without the FORCE keyword. If the user specifies
#       -DCMAKE_BUILD_TYPE:STRING=... during configure step, cmake will appropriate honor the
#       user provided value.
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type: Debug, Release(default), RelWithDebInfo, MinSizeRel")

add_subdirectory(LibMultiSense)

if(MULTISENSE_BUILD_UTILITIES)
    add_subdirectory(Utilities)
endif()

# TODO(ajakhotia): Resolve the statements below after evaluating the need.
#
# For Backwards compatibility with other ROS builds
#
#if (BASE_DIRECTORY)
#else()
#    set(BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
#    set(SOURCE_DIRECTORY )
#endif()

#
# Use full RPATH
#
#set (CMAKE_SKIP_BUILD_RPATH FALSE)
#set (CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
#set (CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
#set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


#[[ Include cmake-default tools to help create export description files. ]]
include(CMakePackageConfigHelpers)

#[[ Install ${CMAKE_INSTALL_PREFIX}/lib/cmake/MultiSense/MultiSenseTargets.cmake to enable other projects
    to include the installation tree produced by this project. ]]
install(EXPORT MultiSenseTargets NAMESPACE MultiSense::
        FILE MultiSenseTargets.cmake
        DESTINATION lib/cmake/MultiSense)


#[[ Generate configuration version file and install it. ]]
write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/MultiSenseConfigVersion.cmake
        COMPATIBILITY SameMajorVersion)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/MultiSenseConfigVersion.cmake
        DESTINATION lib/cmake/MultiSense)


#[[ Configure and install MultiSenseConfig.cmake.in to enable other projects find MultiSense from
    the install tree. Even though the function configure_package_config_file takes the INSTALL_DESTINATION, it
    doesn't actually create an installation step for "make install". So, to install the generated config file, we
    are required to manually create an install step. ]]
configure_package_config_file(
        cmake/MultiSenseConfig.cmake.in cmake/MultiSenseConfig.cmake
        INSTALL_DESTINATION lib/cmake/MultiSense)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/MultiSenseConfig.cmake
        DESTINATION lib/cmake/MultiSense)
