#
# ImuConfigUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(ImuConfigUtility ImuConfigUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (ImuConfigUtility ${MULTISENSE_UTILITY_LIBS})

#
# Specify the install location
#

install(TARGETS ImuConfigUtility RUNTIME DESTINATION "bin")
