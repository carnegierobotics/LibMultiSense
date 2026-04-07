
add_executable(ImageCalUtility ImageCalUtility.cc)

target_link_libraries (ImageCalUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS ImageCalUtility RUNTIME DESTINATION "bin")
