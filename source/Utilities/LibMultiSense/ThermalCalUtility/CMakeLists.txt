add_executable(ThermalCalUtility ThermalCalUtility.cc)

target_link_libraries(ThermalCalUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS ThermalCalUtility RUNTIME DESTINATION "bin")
