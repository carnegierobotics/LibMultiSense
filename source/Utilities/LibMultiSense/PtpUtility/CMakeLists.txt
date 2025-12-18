
add_executable(PtpUtility PtpUtility.cc)

target_link_libraries (PtpUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS PtpUtility RUNTIME DESTINATION "bin")
install(FILES ptp_utility.py PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION "bin")
