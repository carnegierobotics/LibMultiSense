#
# LidarCalUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(LidarCalUtility LidarCalUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (LidarCalUtility ${MULTISENSE_UTILITY_LIBS})
