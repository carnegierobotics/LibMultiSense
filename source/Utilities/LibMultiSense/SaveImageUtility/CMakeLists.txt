
add_executable(SaveImageUtility SaveImageUtility.cc)

target_link_libraries (SaveImageUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS SaveImageUtility RUNTIME DESTINATION "bin")
install(FILES save_image_utility.py PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION "bin")
