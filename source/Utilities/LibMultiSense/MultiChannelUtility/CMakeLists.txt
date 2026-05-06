
add_executable(MultiChannelUtility MultiChannelUtility.cc)

target_link_libraries (MultiChannelUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS MultiChannelUtility RUNTIME DESTINATION "bin")
