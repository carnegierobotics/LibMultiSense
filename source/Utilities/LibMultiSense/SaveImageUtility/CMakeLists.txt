
add_executable(SaveImageUtility SaveImageUtility.cc)

target_link_libraries (SaveImageUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS SaveImageUtility RUNTIME DESTINATION "bin")
