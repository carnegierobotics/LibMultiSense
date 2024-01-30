#
# FeatureDetectorUtility - Makefile
#

#
# Setup the executable that we will use.
#

add_executable(FeatureDetectorUtility FeatureDetectorUtility.cc)

#
# Specify libraries against which to link.
#

target_link_libraries (FeatureDetectorUtility ${MULTISENSE_UTILITY_LIBS})

#
# Specify the install location
#

install(TARGETS FeatureDetectorUtility RUNTIME DESTINATION "bin")
