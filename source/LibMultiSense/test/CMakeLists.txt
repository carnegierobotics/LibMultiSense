find_package(GTest REQUIRED)

set(TEST_NAMES
    calibration_test
    configuration_test
    info_test
    message_test
    multi_channel_test
    multisense_utilities_test
    status_test
    storage_test
    utilities_test)

foreach(TEST_NAME ${TEST_NAMES})
    add_executable(${TEST_NAME} ${TEST_NAME}.cc)
    target_link_libraries(${TEST_NAME} PRIVATE MultiSense
                                               GTest::GTest GTest::Main)

    add_test(${TEST_NAME} ${TEST_NAME})
endforeach()

