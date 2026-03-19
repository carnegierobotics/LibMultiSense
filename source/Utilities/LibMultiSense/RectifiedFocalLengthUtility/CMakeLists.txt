
add_executable(RectifiedFocalLengthUtility RectifiedFocalLengthUtility.cc)

target_link_libraries (RectifiedFocalLengthUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS RectifiedFocalLengthUtility RUNTIME DESTINATION "bin")
install(FILES rectified_focal_length_utility.py PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION "bin")
