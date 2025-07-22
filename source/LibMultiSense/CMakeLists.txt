include(ExternalProject)

set(MULTISENSE_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/MultiSense/MultiSenseChannel.hh
                       ${CMAKE_CURRENT_LIST_DIR}/include/MultiSense/MultiSenseTypes.hh
                       ${CMAKE_CURRENT_LIST_DIR}/include/MultiSense/MultiSenseUtilities.hh)

if (BUILD_JSON_SERIALIZATION)
    if(nlohmann_json_FOUND)
        list("APPEND" MULTISENSE_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/MultiSense/MultiSenseSerialization.hh)
    endif()
endif()

set(DETAILS_SRC details/factory.cc
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

find_program(CARGO "cargo")
if (CARGO)
  set(BUILD_MSWEBRTC ON)
endif (CARGO)

if (BUILD_MSWEBRTC)
  set(DETAILS_SRC ${DETAILS_SRC} details/mswebrtc/channel.cc)
  set(MSWEBRTC_SRC_DIR ${CMAKE_SOURCE_DIR}/rust/libmultisense-ambarella)
  ExternalProject_Add(libmultisense-ambarella
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/libmultisense-ambarella-prefix
    SOURCE_DIR ${MSWEBRTC_SRC_DIR}
    BUILD_IN_SOURCE TRUE
    CONFIGURE_COMMAND ""
    INSTALL_COMMAND ""
    BUILD_ALWAYS TRUE
    BUILD_COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CMAKE_CURRENT_BINARY_DIR}/libmultisense-ambarella-prefix/src/libmultisense-ambarella-build ${CARGO} build --profile release)
else (BUILD_MSWEBRTC)
  message("Cargo not found, not building mswebrtc channel implementation")
endif (BUILD_MSWEBRTC)

if (${BUILD_SHARED_LIBS})
    add_library(MultiSense SHARED ${MULTISENSE_HEADERS}
                                      ${DETAILS_SRC})
else()
    add_library(MultiSense STATIC ${MULTISENSE_HEADERS}
                                   ${DETAILS_SRC})
    set_target_properties(MultiSense PROPERTIES POSITION_INDEPENDENT_CODE ON)

endif()


if (BUILD_MSWEBRTC)
  ExternalProject_Get_property(libmultisense-ambarella PREFIX)
  target_include_directories(MultiSense
    PRIVATE ${PREFIX}/src/libmultisense-ambarella-build)
  add_dependencies(MultiSense libmultisense-ambarella)

  #this option avoids a linker warning, because libmswebrtc.a is missing a section
  target_link_options(MultiSense INTERFACE -z noexecstack)

  #This rather long list of libraries was generated with the following invocation
  # `cargo rustc -- --print native-static-libs`
  # we may need to run that again in the mysterious future if our list of dependencies changes
  # maybe there's a way to have cmake do this?
  target_link_libraries(MultiSense INTERFACE ${PREFIX}/src/libmultisense-ambarella-build/release/libmswebrtc.a -lssl -lcrypto -lgcc_s -lutil -lrt -lpthread -lm -ldl -lc)
endif (BUILD_MSWEBRTC)


target_link_libraries(MultiSense PRIVATE MultiSenseWire)
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
    if(nlohmann_json_FOUND)
        target_link_libraries(MultiSense PUBLIC nlohmann_json::nlohmann_json)
    endif()
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
    target_link_libraries(MultiSense PUBLIC ws2_32)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(MultiSense PUBLIC pthread)
else()
    target_link_libraries(MultiSense PUBLIC pthread rt)
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
