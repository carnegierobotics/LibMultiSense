#
# ChangeResolution - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(ChangeResolution ChangeResolution.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (ChangeResolution ${MULTISENSE_UTILITY_LIBS})

#
# Specify the install location
#

install(TARGETS ChangeResolution RUNTIME DESTINATION "bin")
