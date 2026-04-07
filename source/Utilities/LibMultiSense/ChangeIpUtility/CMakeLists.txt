
add_executable(ChangeIpUtility ChangeIpUtility.cc)

target_link_libraries (ChangeIpUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS ChangeIpUtility RUNTIME DESTINATION "bin")
