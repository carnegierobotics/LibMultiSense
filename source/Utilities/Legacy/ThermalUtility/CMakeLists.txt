#
# ThermalUtility - Makefile
#
# Exercises the "crl_thermal" in-process secondary application on T6:
# enumerate / activate / config / control round-trips, then streams and
# validates thermal frame groups.
#

set(CMAKE_CXX_STANDARD 14)

add_executable(ThermalUtility ThermalUtility.cc)

target_link_libraries (ThermalUtility ${MULTISENSE_UTILITY_LIBS})

install(TARGETS ThermalUtility RUNTIME DESTINATION "bin")
