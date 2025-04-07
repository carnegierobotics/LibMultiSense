add_executable(FirmwareUpdateUtility
  FirmwareUpdateUtility.cc
  Ip.cc
  Updater.cc
  crc.c
)
#target_link_libraries(FirmwareUpdateUtility ${MULTISENSE_UTILITY_LIBS} nlohmann_json::nlohmann_json)

install(TARGETS FirmwareUpdateUtility RUNTIME DESTINATION "bin")

#install(FILES firmware_update_utility.py PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
#        DESTINATION "bin")
