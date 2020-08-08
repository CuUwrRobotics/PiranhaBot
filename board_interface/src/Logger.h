/** Use these logger functions to print the source of the message.
 * int main() {
 *  char file_name = "file.txt";
 *  log_error("File not found: %s.", file_name);
 * }
 * Produces:
 * "Internal Error: File not found: file.txt [from int main() (in main.c:3)]"
 */

#ifndef LOGGER_H
#define LOGGER_H

#include "ros/ros.h"

#define log_info(a, ...) \
	ROS_INFO("Internal Message: " a " [from %s (in %s:%d)]\n" \
	         __VA_OPT__(, ) __VA_ARGS__,  \
	         __FUNCTION__, __FILE__, __LINE__)

#define log_warn(a, ...) \
	ROS_WARN("Internal Warning: " a " [from %s (in %s:%d)]\n" \
	         __VA_OPT__(, ) __VA_ARGS__,  \
	         __FUNCTION__, __FILE__, __LINE__)

#define log_error(a, ...) \
	ROS_ERROR("Internal Error: " a " [from %s (in %s:%d)]\n" \
	          __VA_OPT__(, ) __VA_ARGS__,  \
	          __FUNCTION__, __FILE__, __LINE__)

#endif /* end of include guard: LOGGER_H */
