#
# ExternalCalUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(ExternalCalUtility ExternalCalUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (ExternalCalUtility ${MULTISENSE_UTILITY_LIBS})
