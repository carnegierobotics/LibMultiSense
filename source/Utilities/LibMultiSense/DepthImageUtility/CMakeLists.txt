
add_executable(DepthImageUtility DepthImageUtility.cc)

target_link_libraries (DepthImageUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS DepthImageUtility RUNTIME DESTINATION "bin")
install(FILES depth_image_utility.py PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION "bin")
