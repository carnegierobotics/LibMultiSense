#
# DeviceInfoUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(DeviceInfoUtility DeviceInfoUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (DeviceInfoUtility ${MULTISENSE_UTILITY_LIBS})
