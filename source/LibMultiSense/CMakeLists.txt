set(MULTISENSE_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/MultiSense/MultiSenseChannel.hh
                       ${CMAKE_CURRENT_LIST_DIR}/include/MultiSense/MultiSenseTypes.hh
                       ${CMAKE_CURRENT_LIST_DIR}/include/MultiSense/MultiSenseUtilities.hh)

if (BUILD_JSON_SERIALIZATION)
    list("APPEND" MULTISENSE_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/MultiSense/MultiSenseSerialization.hh)
endif()

set(DETAILS_SRC details/factory.cc
                details/multi_channel.cc
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

endif()

target_link_libraries(MultiSense MultiSenseWire)
target_include_directories(MultiSense
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PUBLIC $<INSTALL_INTERFACE:include>)

if (BUILD_OPENCV)
  if(OpenCV_FOUND)
    target_compile_definitions(MultiSense PRIVATE HAVE_OPENCV)

    target_include_directories(MultiSense PUBLIC ${OpenCV_INCLUDE_DIRS})

    target_link_libraries(MultiSense ${OpenCV_LIBS})
  endif()
endif()

if (BUILD_JSON_SERIALIZATION)
    target_link_libraries(MultiSense nlohmann_json::nlohmann_json)
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

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_link_libraries(MultiSense ws2_32)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(MultiSense pthread)
else()
    target_link_libraries(MultiSense pthread rt)
endif()

if (BUILD_TESTS)
    add_subdirectory(test)
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
  VERSION "${version}"
  COMPATIBILITY AnyNewerVersion
  )

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/MultiSenseConfigVersion.cmake
  DESTINATION lib/cmake/MultiSense)

install(FILES ${MULTISENSE_HEADERS} DESTINATION include/MultiSense)
