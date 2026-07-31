add_executable(ThermalUtility ThermalUtility.cc)

target_link_libraries(ThermalUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS ThermalUtility RUNTIME DESTINATION "bin")
