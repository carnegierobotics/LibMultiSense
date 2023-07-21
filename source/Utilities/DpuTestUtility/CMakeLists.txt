#
# DpuTestUtility - Makefile
#

list(APPEND CMAKE_PREFIX_PATH "/home/bblakeslee/build_libtorch_x86/pytorch/install_lp64")

# find_package(Torch REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)

# 
# Setup the executable to use.
#

add_executable(DpuTestUtility DpuTestUtility.cc)

# TODO: This shouldn't be hardcoded.
target_include_directories(DpuTestUtility PUBLIC "/home/bblakeslee/build_libtorch_x86/pytorch/install_lp64/include/")

#
# Specify libraries to link against.
#

target_link_libraries(DpuTestUtility ${MULTISENSE_UTILITY_LIBS} ${TORCH_LIBRARIES} ${OpenCV_LIBS})

#
# Specify install location
#

install(TARGETS DpuTestUtility RUNTIME DESTINATION "bin")
