#
# AprilTagTestUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(AprilTagTestUtility AprilTagTestUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (AprilTagTestUtility ${MULTISENSE_UTILITY_LIBS})

#
# Specify the install location
#

install(TARGETS AprilTagTestUtility RUNTIME DESTINATION "bin")
