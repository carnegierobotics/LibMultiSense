#
# PointCloudUtility - Makefile
#

#
# This utility requires c++11
#

if ((CMAKE_CXX_COMPILER_ID MATCHES MSVC AND
    CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 11.0) OR
    (CMAKE_CXX_COMPILER_ID MATCHES GNU AND
    CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 4.7) OR
    (CMAKE_CXX_COMPILER_ID MATCHES CLANG AND
    CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 3.3))

    #
    # Setup the executable that we will use.
    #

    add_executable(PointCloudUtility PointCloudUtility.cc)

    #
    # Specify libraries against which to link.
    #

    target_link_libraries (PointCloudUtility ${MULTISENSE_UTILITY_LIBS})

    #
    # Specify the install location
    #

    install(TARGETS PointCloudUtility RUNTIME DESTINATION "bin")

endif()
