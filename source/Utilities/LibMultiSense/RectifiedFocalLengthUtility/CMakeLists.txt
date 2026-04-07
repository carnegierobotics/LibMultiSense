
add_executable(RectifiedFocalLengthUtility RectifiedFocalLengthUtility.cc)

target_link_libraries (RectifiedFocalLengthUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS RectifiedFocalLengthUtility RUNTIME DESTINATION "bin")
