
add_executable(PointCloudUtility PointCloudUtility.cc)

target_link_libraries (PointCloudUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS PointCloudUtility RUNTIME DESTINATION "bin")
