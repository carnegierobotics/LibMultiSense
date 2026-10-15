set(MULTISENSE_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/MultiSense/MultiSenseChannel.hh
                       ${CMAKE_CURRENT_LIST_DIR}/include/MultiSense/MultiSenseSecondaryApplication.hh
                       ${CMAKE_CURRENT_LIST_DIR}/include/MultiSense/MultiSenseTypes.hh
                       ${CMAKE_CURRENT_LIST_DIR}/include/MultiSense/MultiSenseUtilities.hh)

if (BUILD_JSON_SERIALIZATION)
    list("APPEND" MULTISENSE_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/MultiSense/MultiSenseSerialization.hh)
endif()

# Keep CPU detection and the scalar fallback at the baseline instruction set.
# Only the AVX2 kernel receives architecture-specific compiler options.
add_library(MultiSenseColorConversion OBJECT details/color_conversion.cc)
set_target_properties(MultiSenseColorConversion PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(MultiSenseColorConversion PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
if (MSVC)
    target_compile_options(MultiSenseColorConversion PRIVATE /fp:strict)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang|AppleClang)$")
    target_compile_options(MultiSenseColorConversion PRIVATE -ffp-contract=off)
endif()

set(MULTISENSE_HAVE_AVX2 OFF)
if (MULTISENSE_ENABLE_SIMD)
    include(MultiSenseSimd)
    multisense_check_avx2(MULTISENSE_HAVE_AVX2 MULTISENSE_AVX2_OPTIONS)
    if (MULTISENSE_HAVE_AVX2)
        target_sources(MultiSenseColorConversion PRIVATE details/color_conversion_avx2.cc)
        set_source_files_properties(details/color_conversion_avx2.cc PROPERTIES
            COMPILE_OPTIONS "${MULTISENSE_AVX2_OPTIONS}")
        target_compile_definitions(MultiSenseColorConversion PRIVATE MULTISENSE_HAVE_AVX2)
        message(STATUS "Enabling AVX2 color conversion with runtime CPU detection")
    else()
        message(STATUS "AVX2 color conversion is unavailable for this target/toolchain; using scalar conversion")
    endif()
endif()

set(DETAILS_SRC details/factory.cc
                details/multi_channel.cc
                details/secondary_application.cc
                details/utilities.cc
                details/legacy/calibration.cc
                details/legacy/channel.cc
                details/legacy/configuration.cc
                details/legacy/info.cc
                details/legacy/ip.cc
                details/legacy/message.cc
                details/legacy/status.cc
                details/legacy/storage.cc
                details/legacy/udp.cc
                details/legacy/utilities.cc)

if (${BUILD_SHARED_LIBS})
    add_library(MultiSense SHARED ${MULTISENSE_HEADERS}
                                      ${DETAILS_SRC})
else()
    add_library(MultiSense STATIC ${MULTISENSE_HEADERS}
                                   ${DETAILS_SRC})
    set_target_properties(MultiSense PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_compile_definitions(MultiSense PUBLIC MultiSense_STATIC)

endif()

# Provide a namespaced alias so that in-tree consumers can link against the same
# target name that the installed package exports.
add_library(MultiSense::MultiSense ALIAS MultiSense)
target_sources(MultiSense PRIVATE $<TARGET_OBJECTS:MultiSenseColorConversion>)

target_link_libraries(MultiSense PUBLIC MultiSenseWire::MultiSenseWire)
if (FILESYSTEM_LIBRARY)
    target_link_libraries(MultiSense PUBLIC ${FILESYSTEM_LIBRARY})
endif()
target_include_directories(MultiSense
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PUBLIC $<INSTALL_INTERFACE:include>
        PRIVATE ${MULTISENSE_SHARED_INCLUDE_DIR})

if (BUILD_OPENCV)
    target_compile_definitions(MultiSense PUBLIC MULTISENSE_HAVE_OPENCV)

    target_link_libraries(MultiSense
        PUBLIC
            opencv_core
            opencv_${MULTISENSE_OPENCV_FEATURES_COMPONENT}
        PRIVATE
            opencv_imgcodecs)
endif()

if (BUILD_JSON_SERIALIZATION AND nlohmann_json_FOUND)
    target_link_libraries(MultiSense PUBLIC nlohmann_json::nlohmann_json)
endif()

set_target_properties(MultiSense PROPERTIES VERSION "${version}")

if (MULTISENSE_BUILD_API_DATE)
    target_compile_definitions(MultiSense PRIVATE BUILD_API_DATE="True")
endif()

target_include_directories(MultiSense PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

set_target_properties(MultiSense PROPERTIES PUBLIC_HEADER "${MULTISENSE_HEADERS}")

#
# We want to link against our child libraries.
#

find_package(Threads REQUIRED)
target_link_libraries(MultiSense PUBLIC Threads::Threads)
if(CMAKE_SYSTEM_NAME STREQUAL "Windows" OR CMAKE_SYSTEM_NAME STREQUAL "WindowsStore")
    target_link_libraries(MultiSense PRIVATE ws2_32)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(MultiSense PRIVATE rt)
endif()

if (BUILD_TESTS)
    add_subdirectory(test)
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/MultiSenseConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfig.cmake"
  INSTALL_DESTINATION "lib/cmake/MultiSense"
)

# create install targets
install(TARGETS MultiSense
  EXPORT MultiSenseTargets
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/MultiSense
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT MultiSenseTargets
        NAMESPACE MultiSense::
        DESTINATION lib/cmake/MultiSense)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfigVersion.cmake"
  VERSION "${version}"
  COMPATIBILITY AnyNewerVersion
  )

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfigVersion.cmake
  ${MULTISENSE_CMAKE_MODULE_CXX_CHECK_CONSTEXPR}
  ${MULTISENSE_CMAKE_MODULE_CXX_CHECK_FILESYSTEM}
  DESTINATION lib/cmake/MultiSense)

install(FILES ${MULTISENSE_HEADERS} DESTINATION include/MultiSense)
