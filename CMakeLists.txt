cmake_minimum_required(VERSION 3.5)
project(rqt_nav_view)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/rqt_nav_view
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
