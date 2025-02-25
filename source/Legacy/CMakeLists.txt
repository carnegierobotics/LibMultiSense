#
# MultiSense - Makefile
#

#
# Platform checks
#

include(CheckFunctionExists)

check_function_exists(vasprintf HAVE_VASPRINTF)

option(MULTISENSE_UDP_ASSEMBLER_DEBUG "Build LibMultiSense to print any failures to assemble a message. Defaults to OFF." OFF)
option(MULTISENSE_BUILD_API_DATE "Build in the API date into LibMultiSense. Defaults to ON." ON)

#
# Include all of our child directories.
#

set(MULTISENSE_HEADERS include/MultiSense/MultiSenseChannel.hh
                       include/MultiSense/MultiSenseTypes.hh)

set(UTILITY_HEADERS
  include/MultiSense/details/utility/Portability.hh
  include/MultiSense/details/utility/BufferStream.hh
  include/MultiSense/details/utility/Constants.hh
  include/MultiSense/details/utility/Exception.hh
  include/MultiSense/details/utility/TimeStamp.hh
  include/MultiSense/details/utility/Thread.hh
  include/MultiSense/details/utility/ReferenceCount.hh
  include/MultiSense/details/utility/Units.hh
  )

set(DETAILS_HEADERS include/MultiSense/details/channel.hh
                    include/MultiSense/details/query.hh
                    include/MultiSense/details/listeners.hh
                    include/MultiSense/details/signal.hh
                    include/MultiSense/details/storage.hh)

set(LINUX_HEADERS include/MultiSense/details/utility/linux/Thread.hh)
set(WIN32_HEADERS include/MultiSense/details/utility/win32/Thread.hh)
set(MACOS_HEADERS include/MultiSense/details/utility/macos/Thread.hh)

set(DETAILS_SRC details/channel.cc
                details/public.cc
                details/constants.cc
                details/dispatch.cc
                details/utility/Constants.cc)

if(NOT HAVE_VASPRINTF)
    add_definitions(-DNEED_VASPRINTF=1)
endif()

#
# Add in all of the source files in this directory.
#

add_library(MultiSense ${MULTISENSE_HEADERS}
                       ${DETAILS_HEADERS}
                       ${DETAILS_SRC})

target_link_libraries(MultiSense MultiSenseWire)
target_include_directories(MultiSense
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PUBLIC $<INSTALL_INTERFACE:include>)

if (MULTISENSE_UDP_ASSEMBLER_DEBUG)
    target_compile_definitions(MultiSense PRIVATE UDP_ASSEMBLER_DEBUG=${MULTISENSE_UDP_ASSEMBLER_DEBUG})
endif()

#
# Versioning...someday lets automate this somehow
#

set_target_properties(MultiSense PROPERTIES VERSION "${CMAKE_PROJECT_VERSION}")

if (MULTISENSE_BUILD_API_DATE)
    target_compile_definitions(MultiSense PRIVATE BUILD_API_DATE="True")
endif()

target_include_directories(MultiSense PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

set_target_properties(MultiSense PROPERTIES PUBLIC_HEADER "${MULTISENSE_HEADERS}")

#
# We want to link against our child libraries.
#

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_link_libraries(MultiSense ws2_32)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(MultiSense pthread)
else()
    target_link_libraries(MultiSense pthread rt)
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/MultiSenseConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfig.cmake
  INSTALL_DESTINATION lib/cmake/MultiSense)


# create install targets
install(TARGETS MultiSense
  EXPORT MultiSenseTargets
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/MultiSense
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT MultiSenseTargets
  DESTINATION lib/cmake/MultiSense)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfigVersion.cmake"
  VERSION "${CMAKE_PROJECT_VERSION}"
  COMPATIBILITY AnyNewerVersion
  )

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfigVersion.cmake
  DESTINATION lib/cmake/MultiSense)


install(FILES ${MULTISENSE_HEADERS} DESTINATION include/MultiSense)
