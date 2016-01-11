#
# ImageCalUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(ImageCalUtility ImageCalUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (ImageCalUtility ${MULTISENSE_UTILITY_LIBS})
