find_package(nlohmann_json)

if(nlohmann_json_FOUND)
    add_executable(DeviceInfoUtility DeviceInfoUtility.cc)

    target_link_libraries(DeviceInfoUtility ${MULTISENSE_UTILITY_LIBS} nlohmann_json::nlohmann_json)

    install(TARGETS DeviceInfoUtility RUNTIME DESTINATION "bin")
else()
    message("Missing nlohmann_json. Not building the c++ DeviceInfoUtility")
endif()

install(FILES device_info_utility.py PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION "bin")
