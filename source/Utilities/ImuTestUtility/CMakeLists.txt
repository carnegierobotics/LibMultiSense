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

target_link_libraries (ImuTestUtility ${MULTISENSE_UTILITY_LIBS})
