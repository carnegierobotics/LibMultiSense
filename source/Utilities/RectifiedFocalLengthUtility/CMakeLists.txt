#
# RectifiedFocalLengthUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(RectifiedFocalLengthUtility RectifiedFocalLengthUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (RectifiedFocalLengthUtility ${MULTISENSE_UTILITY_LIBS})

#
# Specify the install location
#

install(TARGETS RectifiedFocalLengthUtility RUNTIME DESTINATION "bin")
