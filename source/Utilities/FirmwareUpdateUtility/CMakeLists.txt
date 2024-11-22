#
# FirmwareUpdateUtility - Makefile
#

#
# Setup the executable that we will use.
#
project (FirmwareUpdateUtility C CXX)
add_executable(FirmwareUpdateUtility
  FirmwareUpdateUtility.cc
  Ip.cc
  Updater.cc
  crc.c
)

#
# Specify libraries against which to link.
#

target_link_libraries (FirmwareUpdateUtility ${MULTISENSE_UTILITY_LIBS})

#
# Specify the install location
#

install(TARGETS FirmwareUpdateUtility RUNTIME DESTINATION "bin")
