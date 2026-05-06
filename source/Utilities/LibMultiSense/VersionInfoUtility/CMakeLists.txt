
add_executable(VersionInfoUtility VersionInfoUtility.cc)

target_link_libraries (VersionInfoUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS VersionInfoUtility RUNTIME DESTINATION "bin")
