
add_executable(PointCloudUtility PointCloudUtility.cc)

target_link_libraries (PointCloudUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS PointCloudUtility RUNTIME DESTINATION "bin")
install(FILES point_cloud_utility.py PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION "bin")
