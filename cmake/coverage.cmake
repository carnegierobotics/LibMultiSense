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

add_custom_target(coverage
    # Cleanup old data
    COMMAND ${LCOV_PATH} --directory ${CMAKE_BINARY_DIR} --zerocounters
    # Run tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    # Capture coverage data
    COMMAND ${LCOV_PATH} --directory ${CMAKE_BINARY_DIR} --capture --output-file ${CMAKE_BINARY_DIR}/coverage.info --ignore-errors mismatch
    # Remove system headers and third-party code
    COMMAND ${LCOV_PATH} --remove ${CMAKE_BINARY_DIR}/coverage.info '/usr/*' '*/test/*' '*/vcpkg_installed/*' '*/details/legacy/*' --output-file ${CMAKE_BINARY_DIR}/coverage_filtered.info --ignore-errors unused
    # Generate HTML report
    COMMAND ${GENHTML_PATH} ${CMAKE_BINARY_DIR}/coverage_filtered.info --output-directory ${CMAKE_BINARY_DIR}/coverage_report
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating unit test coverage report"
)

message(STATUS "Coverage target 'coverage' added. Run 'make coverage' to generate report.")
