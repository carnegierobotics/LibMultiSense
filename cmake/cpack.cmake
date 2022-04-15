option(CRL_ENABLE_CI "Determines if the build is occuring within CI." FALSE)
set(CPACK_PACKAGE_NAME libmultisense)
set(CPACK_PACKAGE_VENDOR "Carnegie Robotics, LLC")
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Carnegie Robotics") #required
set(CPACK_PACKAGE_CONTACT "info@carnegierobotics.com") #required
set(CPACK_INSTALLED_DIRECTORIES "${CMAKE_INSTALL_PREFIX};/usr/local/${PROJECT_NAME}/${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "")
set(CPACK_INSTALL_CMAKE_PROJECTS "")
execute_process(
    COMMAND /bin/bash -c "source /etc/os-release && echo $ID$VERSION_ID | sed 's/\\./_/g'"
    OUTPUT_VARIABLE CPACK_SYSTEM_NAME
    OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CPACK_STRIP_FILES TRUE)
include(CPack)
