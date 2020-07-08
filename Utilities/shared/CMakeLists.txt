add_library(CalibrationYaml INTERFACE)
add_library(MultiSense::CalibrationYaml ALIAS CalibrationYaml)

target_include_directories(CalibrationYaml
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>)
