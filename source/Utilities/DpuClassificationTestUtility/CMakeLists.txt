#
# DpuClassificationTestUtility - Makefile
#

# 
# Setup the executable to use.
#

add_executable(DpuClassificationTestUtility DpuClassificationTestUtility.cc)

#
# Specify libraries to link against.
#

target_link_libraries(DpuClassificationTestUtility ${MULTISENSE_UTILITY_LIBS})

#
# Specify install location
#

install(TARGETS DpuClassificationTestUtility RUNTIME DESTINATION "bin")
