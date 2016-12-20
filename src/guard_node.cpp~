#include "ros/ros.h"
#include "guard.hpp"
bool status=1;
int main(int argc, char **argv)
{
  	ros::init(argc, argv, "guard");
	ros::NodeHandle node;
	//Create object.
	guard guard_object(node, status);
	ros::spin();
 	return 0;
}
