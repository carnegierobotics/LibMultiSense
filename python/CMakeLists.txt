set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_libmultisense MODULE bindings.cc)

target_link_libraries(
    _libmultisense
    PRIVATE
    MultiSense)

if (BUILD_JSON_SERIALIZATION)
    if(nlohmann_json_FOUND)
        message("Building python bindings with json print support")
        target_compile_definitions(_libmultisense PRIVATE BUILD_JSON="True")
    endif()
endif()

install(TARGETS _libmultisense DESTINATION libmultisense)
