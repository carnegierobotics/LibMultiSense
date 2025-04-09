
add_executable(ChangeIpUtility ChangeIpUtility.cc)

target_link_libraries (ChangeIpUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS ChangeIpUtility RUNTIME DESTINATION "bin")
install(FILES change_ip_utility.py PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION "bin")
