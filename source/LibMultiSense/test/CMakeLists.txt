find_package(GTest REQUIRED)

set(TEST_NAMES
    TestTimestamp)

foreach(TEST_NAME ${TEST_NAMES})
    add_executable(${TEST_NAME} ${TEST_NAME}.cc)
    target_link_libraries(${TEST_NAME} PRIVATE MultiSense
                                               GTest::GTest GTest::Main)

    add_test(${TEST_NAME} ${TEST_NAME})
endforeach()
