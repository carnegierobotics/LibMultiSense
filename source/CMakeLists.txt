#
# MultiSense - Makefile
#

#
# Recurse into each of these directories.
#

add_subdirectory(Wire)

add_subdirectory(LibMultiSense)

if (${MULTISENSE_BUILD_UTILITIES})
    add_subdirectory(Utilities)
endif ()
