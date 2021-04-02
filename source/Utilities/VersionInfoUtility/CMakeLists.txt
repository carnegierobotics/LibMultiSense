#
# VersionInfoUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(VersionInfoUtility VersionInfoUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (VersionInfoUtility ${MULTISENSE_UTILITY_LIBS})

#
# Specify the install location
#

install(TARGETS VersionInfoUtility RUNTIME DESTINATION "bin")
