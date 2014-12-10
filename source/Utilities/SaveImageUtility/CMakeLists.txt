#
# SaveImageUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(SaveImageUtility SaveImageUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (SaveImageUtility ${MULTISENSE_UTILITY_LIBS})
