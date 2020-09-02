echo Copying files...
# All variables used here are set by the dockerfile when it moves files into $temporary_package_directory
cd $temporary_package_directory
mv board_interface $final_package_directory/board_interface
mv camera_controller $final_package_directory/camera_controller
mv diagnostic_display $final_package_directory/diagnostic_display
mv mpu6050_reader $final_package_directory/mpu6050_reader
mv watchdog $final_package_directory/watchdog
mv web-gui $final_package_directory/web-gui

# Move the makefile in which makes catkin_make commands more bearable
mv catkin/Makefile $final_package_directory/../Makefile
