// This is a template code for the baseline code needed to run code that will
// speak to our watchdog

// These should be moved to a headers file!
#include <ros/ros.h>
#include "watchdog/pet_dog_msg.h"

int main(int argc, char const *argv[]) {
	// Connect ROS
	printf("Starting up ROS.\n");
	// Start ROS and get the node instance
	ros::init(argc, argv, "board_interface");
	ros::NodeHandle nd;
	// Get the node name ot send to the watchdog
	std::string nodeName = ros::this_node::getName();
	printf("Node name: %s\n", nodeName.c_str());
	// Set up the message publisher
	ros::Publisher wd_petter =
		nd.advertise <watchdog::pet_dog_msg> ("pet_dog_msg", 1000);
	// Allows for a 1 second delay between messages
	ros::Duration loop_wait(1000);
	// For storing pets
	watchdog::pet_dog_msg msg;
	while (ros::ok()) {
		// Pet the dog
		msg.petterName = nodeName; // Pack data
		wd_petter.publish(msg); // Sends the pet over
		// Wait 1 second
		loop_wait.sleep();
	}
	// Fin
	exit(EXIT_SUCCESS);
} // main
