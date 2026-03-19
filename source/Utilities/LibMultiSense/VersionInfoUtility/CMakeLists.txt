
add_executable(VersionInfoUtility VersionInfoUtility.cc)

target_link_libraries (VersionInfoUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS VersionInfoUtility RUNTIME DESTINATION "bin")
install(FILES version_info_utility.py PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION "bin")
