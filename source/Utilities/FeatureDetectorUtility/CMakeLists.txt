#
# FeatureDetectorUtility - Makefile
#

#
# This utility requires c++14
#

if ((CMAKE_CXX_COMPILER_ID MATCHES MSVC AND
    CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 14.0) OR
    (CMAKE_CXX_COMPILER_ID MATCHES GNU AND
    CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9) OR
    (CMAKE_CXX_COMPILER_ID MATCHES Clang AND
    CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 3.4))

    set(CMAKE_CXX_STANDARD 14)
    #
    # Setup the executable that we will use.
    #

    add_executable(FeatureDetectorUtility FeatureDetectorUtility.cc)

    #
    # Specify libraries against which to link.
    #

    target_link_libraries (FeatureDetectorUtility ${MULTISENSE_UTILITY_LIBS})
    target_include_directories (FeatureDetectorUtility PRIVATE include)

    install(TARGETS FeatureDetectorUtility RUNTIME DESTINATION "bin")
endif()
