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
    add_custom_target(MultiSenseUpdater ALL
                      DEPENDS FlashUtility
                      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/_MultiSenseUpdater
                      ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/MultiSenseUpdater)

endif()
