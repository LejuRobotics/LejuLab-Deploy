cmake_minimum_required(VERSION 3.0.2)
project(leju_launch)

include(GNUInstallDirs)

# ── Catkin support (required for roslaunch to find this package) ──
find_package(catkin QUIET)
if(catkin_FOUND)
  catkin_package()
endif()

# Install
if(catkin_FOUND)
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN "*.launch"
  )
  install(DIRECTORY scripts/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
    USE_SOURCE_PERMISSIONS
    PATTERN "*.sh"
    PATTERN "*.py"
  )
  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )
else()
  install(DIRECTORY launch/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/launch
    PATTERN "*.launch"
  )
  install(DIRECTORY scripts/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/scripts
    USE_SOURCE_PERMISSIONS
    PATTERN "*.sh"
    PATTERN "*.py"
  )
  install(DIRECTORY config/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config
  )
endif()
