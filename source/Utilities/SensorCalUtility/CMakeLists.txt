#
# SensorCalUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(SensorCalUtility SensorCalUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (SensorCalUtility ${MULTISENSE_UTILITY_LIBS})
