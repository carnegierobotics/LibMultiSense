find_package(Threads REQUIRED)

#[[ Add MultiSense as a shared library. Also add an alias target with the export namespace so that dependee
    libraries my link against MultiSense::MultiSense regardless of whether MultiSense was included in
    the dependee's source tree or imported from an independent install tree. ]]
add_library(MultiSense SHARED
        src/details/channel.cc
        src/details/dispatch.cc
        src/details/flash.cc
        src/details/public.cc
        src/details/utility/Constants.cc
        src/details/utility/Exception.cc
        src/details/utility/TimeStamp.cc)

add_library(MultiSense::MultiSense ALIAS MultiSense)


target_include_directories(MultiSense
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/LibMultiSense
        ${CMAKE_CURRENT_SOURCE_DIR}/include/LibMultiSense/details
        ${CMAKE_CURRENT_SOURCE_DIR}/include/LibMultiSense/details/utility)


include(CheckFunctionExists)
check_function_exists(vasprintf HAVE_VASPRINTF)

if(NOT HAVE_VASPRINTF)
    target_compile_definitions(MultiSense PRIVATE -DNEED_VASPRINTF=1)
endif()


# TODO(ajakhotia): Evaluate if this will automatically resolve to correct library on windows.
# TODO(ajakhotia): Evaluate if the dependency is public or private.
target_link_libraries(MultiSense PRIVATE Threads::Threads)

# TODO(ajakhotia): Evaluate if this is still necessary.
#if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
#    target_link_libraries(MultiSense ws2_32)
#elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#    target_link_libraries(MultiSense pthread)
#else()
#    target_link_libraries(MultiSense pthread rt)
#endif()


# TODO(ajakhotia): Explore possibilities of version-ing using project version.
# Versioning...someday lets automate this somehow
#
set_target_properties(MultiSense PROPERTIES VERSION "3.8")


install(DIRECTORY include/LibMultiSense
        DESTINATION include
        FILES_MATCHING PATTERN "*.hh")

install(TARGETS MultiSense EXPORT MultiSenseTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include)
