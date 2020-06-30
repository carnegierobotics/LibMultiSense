#
# portability/getopt - Makefile
#

add_library (getopt STATIC
    getopt.c
)

#
# Specify the install location
#
install(TARGETS getopt
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
