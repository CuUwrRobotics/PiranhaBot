# Add at least these lines to your CMakeLists.txt file to use the watchdog

find_package(catkin REQUIRED COMPONENTS
  watchdog
)

catkin_package(
 INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
 CATKIN_DEPENDS watchdog
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
