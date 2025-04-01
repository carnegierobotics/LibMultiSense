
add_executable(ImageCalUtility ImageCalUtility.cc)

target_link_libraries (ImageCalUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS ImageCalUtility RUNTIME DESTINATION "bin")
install(FILES image_cal_utility.py PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION "bin")
