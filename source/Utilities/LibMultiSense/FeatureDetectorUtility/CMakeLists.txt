if(BUILD_OPENCV)
    add_executable(FeatureDetectorUtility FeatureDetectorUtility.cc)

    target_link_libraries (FeatureDetectorUtility
        ${MULTISENSE_UTILITY_LIBS}
        opencv_highgui
        opencv_imgproc)

    install(TARGETS FeatureDetectorUtility RUNTIME DESTINATION "bin")
else()
    message(STATUS "BUILD_OPENCV is OFF. Not building the C++ FeatureDetectorUtility")
endif()
