#
# SecondaryAppTestUtility - Makefile
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

    #
    # Setup the executable that we will use.
    #

    add_executable(SecondaryAppTestUtility SecondaryAppTestUtility.cc)

    #
    # Specify libraries against which to link.
    #

    target_link_libraries (SecondaryAppTestUtility ${MULTISENSE_UTILITY_LIBS})

    #
    # Specify the install location
    #

    install(TARGETS SecondaryAppTestUtility RUNTIME DESTINATION "bin")

endif()
