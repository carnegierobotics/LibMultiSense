if(BUILD_JSON_SERIALIZATION AND nlohmann_json_FOUND)
    add_executable(DeviceInfoUtility DeviceInfoUtility.cc)

    target_link_libraries(DeviceInfoUtility ${MULTISENSE_UTILITY_LIBS} nlohmann_json::nlohmann_json)

    install(TARGETS DeviceInfoUtility RUNTIME DESTINATION "bin")
else()
    message(
        STATUS
        "BUILD_JSON_SERIALIZATION is OFF or nlohmann_json is not found. Not building the c++ DeviceInfoUtility"
    )
endif()
