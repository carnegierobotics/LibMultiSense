#
# FlashUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(FlashUtility FlashUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (FlashUtility ${MULTISENSE_UTILITY_LIBS})

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
else()
    install(PROGRAMS _MultiSenseUpdater DESTINATION bin RENAME MultiSenseUpdater)
endif()

#
# Specify the install location
#

install(TARGETS FlashUtility RUNTIME DESTINATION "bin")
