if(BUILD_OPENCV AND OpenCV_FOUND)
    add_executable(FeatureDetectorUtility FeatureDetectorUtility.cc)

    target_link_libraries (FeatureDetectorUtility
        ${MULTISENSE_UTILITY_LIBS}
        opencv_highgui
        opencv_imgproc)

    install(TARGETS FeatureDetectorUtility RUNTIME DESTINATION "bin")
else()
    message(STATUS "Missing OpenCV or BUILD_OPENCV is OFF. Not building the C++ FeatureDetectorUtility")
endif()
