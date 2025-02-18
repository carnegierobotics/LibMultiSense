#
# ChangeFps - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(ChangeFps ChangeFps.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (ChangeFps ${MULTISENSE_UTILITY_LIBS})

#
# Specify the install location
#

install(TARGETS ChangeFps RUNTIME DESTINATION "bin")
