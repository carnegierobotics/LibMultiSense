
add_executable(ColorImageUtility ColorImageUtility.cc)

target_link_libraries (ColorImageUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS ColorImageUtility RUNTIME DESTINATION "bin")
install(FILES color_image_utility.py PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION "bin")
