#
# ChangeTransmitDelay - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(ChangeTransmitDelay ChangeTransmitDelay.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (ChangeTransmitDelay ${MULTISENSE_UTILITY_LIBS})

#
# Specify the install location
#

install(TARGETS ChangeTransmitDelay RUNTIME DESTINATION "bin")
