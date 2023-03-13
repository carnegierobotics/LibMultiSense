#
# DpuTestUtility - Makefile
#

# 
# Setup the executable to use.
#

add_executable(DpuTestUtility DpuTestUtility.cc)

#
# Specify libraries to link against.
#

target_link_libraries(DpuTestUtility ${MULTISENSE_UTILITY_LIBS})

#
# Specify install location
#

install(TARGETS DpuTestUtility RUNTIME DESTINATION "bin")
