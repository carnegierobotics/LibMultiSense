find_package(GTest REQUIRED)

set(TEST_NAMES
    calibration_test
    configuration_test
    feature_test
    info_test
    message_test
    multi_channel_test
    multisense_utilities_test
    request_test
    status_test
    storage_test
    utilities_test)

foreach(TEST_NAME ${TEST_NAMES})
    add_executable(${TEST_NAME} ${TEST_NAME}.cc)
    target_link_libraries(${TEST_NAME} PRIVATE MultiSense::MultiSense
                                               MultiSenseWire::MultiSenseWire
                                               GTest::GTest GTest::Main)

    add_test(${TEST_NAME} ${TEST_NAME})
endforeach()

# Exercise the private dispatcher and both kernels without exporting them from
# the shared library (including Windows DLL builds).
add_executable(color_conversion_test color_conversion_test.cc
    $<TARGET_OBJECTS:MultiSenseColorConversion>)
target_include_directories(color_conversion_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_link_libraries(color_conversion_test PRIVATE GTest::GTest GTest::Main)
if (MULTISENSE_HAVE_AVX2)
    target_compile_definitions(color_conversion_test PRIVATE MULTISENSE_HAVE_AVX2)
endif()
add_test(color_conversion_test color_conversion_test)

if (BUILD_JSON_SERIALIZATION)
    target_compile_definitions(request_test PRIVATE MULTISENSE_TEST_JSON)
endif()
