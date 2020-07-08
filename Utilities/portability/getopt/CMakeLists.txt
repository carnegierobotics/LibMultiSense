include(CheckFunctionExists)
check_function_exists(getopt HAVE_GETOPT)


if(HAVE_GETOPT)
    add_library(getopt INTERFACE)
else()
    add_library(getopt STATIC getopt.c)

    target_include_directories(getopt
            PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
            $<INSTALL_INTERFACE:include>)

    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            DESTINATION include
            FILES_MATCHING PATTERN "*.h")
endif()

add_library(MultiSense::getopt ALIAS getopt)


install(TARGETS getopt EXPORT MultiSenseTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include)
