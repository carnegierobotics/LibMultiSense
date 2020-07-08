#
# ImuTestUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(ImuTestUtility ImuTestUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (ImuTestUtility MultiSense::MultiSense)

#
# Specify the install location
#

install(TARGETS ImuTestUtility RUNTIME DESTINATION "bin")
