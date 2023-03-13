#
# DpuBboxTestUtility - Makefile
#

# 
# Setup the executable to use.
#

add_executable(DpuBboxTestUtility DpuBboxTestUtility.cc)

#
# Specify libraries to link against.
#

target_link_libraries(DpuBboxTestUtility ${MULTISENSE_UTILITY_LIBS})

#
# Specify install location
#

install(TARGETS DpuBboxTestUtility RUNTIME DESTINATION "bin")

