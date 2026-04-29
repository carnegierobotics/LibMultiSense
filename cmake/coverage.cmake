#
# Coverage reporting
#

find_program(LCOV_PATH lcov)
find_program(GENHTML_PATH genhtml)

if(NOT LCOV_PATH)
    message(FATAL_ERROR "lcov not found! Coverage target will not be available.")
endif()

if(NOT GENHTML_PATH)
    message(FATAL_ERROR "genhtml not found! Coverage target will not be available.")
endif()

# Detect lcov version to use compatible flags
execute_process(COMMAND ${LCOV_PATH} --version OUTPUT_VARIABLE LCOV_VERSION_OUTPUT)
if(LCOV_VERSION_OUTPUT MATCHES "version ([2-9][0-9]*\\.[0-9]+)")
    message(STATUS "Detected lcov version 2.0+ (${CMAKE_MATCH_1})")
    # For lcov 2.0+, we need to ignore certain errors that were previously warnings
    set(LCOV_FLAGS --ignore-errors mismatch,unused)
else()
    message(STATUS "Detected lcov version 1.x")
    # For lcov 1.x, we use older ignore-errors categories
    set(LCOV_FLAGS --ignore-errors gcov,source)
endif()

add_custom_target(coverage
    # Cleanup old data
    COMMAND ${LCOV_PATH} --directory ${CMAKE_BINARY_DIR} --zerocounters
    # Run tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    # Capture coverage data
    COMMAND ${LCOV_PATH} --directory ${CMAKE_BINARY_DIR} --capture --output-file ${CMAKE_BINARY_DIR}/coverage.info ${LCOV_FLAGS}
    # Remove system headers and third-party code
    COMMAND ${LCOV_PATH} --remove ${CMAKE_BINARY_DIR}/coverage.info '/usr/*' '*/test/*' '*/vcpkg_installed/*' '*/details/legacy/*' --output-file ${CMAKE_BINARY_DIR}/coverage_filtered.info ${LCOV_FLAGS}
    # Generate HTML report
    COMMAND ${GENHTML_PATH} ${CMAKE_BINARY_DIR}/coverage_filtered.info --output-directory ${CMAKE_BINARY_DIR}/coverage_report
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating unit test coverage report"
)

message(STATUS "Coverage target 'coverage' added. Run 'make coverage' to generate report.")
