if(OpenCV_FOUND)
    add_executable(FeatureDetectorUtility FeatureDetectorUtility.cc)

    target_link_libraries (FeatureDetectorUtility ${MULTISENSE_UTILITY_LIBS})

    install(TARGETS FeatureDetectorUtility RUNTIME DESTINATION "bin")
else()
    message("Missing OpenCV. Not building the c++ FeatureDetectorUtility")
endif()
install(FILES feature_detector_utility.py PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION "bin")

