
add_executable(MultiChannelUtility MultiChannelUtility.cc)

target_link_libraries (MultiChannelUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS MultiChannelUtility RUNTIME DESTINATION "bin")
install(FILES multi_channel_utility.py PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION "bin")
