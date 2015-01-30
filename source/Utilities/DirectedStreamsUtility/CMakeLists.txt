#
# SaveImageUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(DirectedStreamsUtility DirectedStreamsUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (DirectedStreamsUtility ${MULTISENSE_UTILITY_LIBS})
