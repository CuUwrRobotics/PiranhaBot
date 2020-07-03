cmake_minimum_required(VERSION 3.16)
project(main)

# Add the actual files to be built here, using format:
# add_executable(package_name path/to/main.cpp)
# install(TARGETS package_name)
add_executable(board_interface board_interface/src/HwMain.cpp)
install(TARGETS board_interface)
