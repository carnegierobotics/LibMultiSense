#
# ChangeIpUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(ChangeIpUtility ChangeIpUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (ChangeIpUtility ${MULTISENSE_UTILITY_LIBS})
