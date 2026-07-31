add_executable(SecondaryAppUtility SecondaryAppUtility.cc)

target_link_libraries(SecondaryAppUtility
    ${MULTISENSE_UTILITY_LIBS}
    MultiSenseWire::MultiSenseWire)

install(TARGETS SecondaryAppUtility RUNTIME DESTINATION "bin")
