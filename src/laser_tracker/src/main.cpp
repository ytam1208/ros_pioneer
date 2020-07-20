#include <ros/ros.h>
#include "laser_tracker.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_tracker");

	laser_tracker lk(ros::this_node::getName());

	lk.runLoop();

	return 0;
}
