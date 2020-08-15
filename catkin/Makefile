# This file exists only becase ROS's catkin is a bad packge manager.

# I mean, Make was last updated in 2006 and even it can do more than catkin in a
# convinient way. Why, ROS? Why?

CATKIN_MAKE=catkin_make

# Compiler setup
CXX=g++-8
CMAKE_CXX_ARG=-DCMAKE_CXX_COMPILER=$(CXX)

CC=gcc-8
CMAKE_CC_ARG=-DCMAKE_C_COMPILER=$(CC)

# Debug symbols for debugging with GDB
CMAKE_DEBUG_ARG=-DCMAKE_BUILD_TYPE=DEBUG

# This command, which literally just does the most basic build, is 63 characters long!
# Normally, it's just 'make'! AND this doesen't even run in their OWN docker image
# without extremely convoluted setup. Even then I have to run this, then clean,
# then this again! Only after all that will ROS accept that the node exists!
# wtf, ROS?
all:
	$(CATKIN_MAKE) $(CMAKE_CC_ARG) $(CMAKE_CXX_ARG)

first:
	-$(CATKIN_MAKE) $(CMAKE_CC_ARG) $(CMAKE_CXX_ARG)
	$(CATKIN_MAKE) clean
	$(CATKIN_MAKE) $(CMAKE_CC_ARG) $(CMAKE_CXX_ARG)

debug:
	$(CATKIN_MAKE) $(CMAKE_CC_ARG) $(CMAKE_CXX_ARG) $(CMAKE_DEBUG_ARG)

clean:
	$(CATKIN_MAKE) clean
