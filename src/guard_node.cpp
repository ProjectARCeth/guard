#include "../include/guard/guard.hpp"
bool status=1;
int main(int argc, char **argv)
{
  	ros::init(argc, argv, "high_level_guard");
	ros::NodeHandle node;
	//Create object.
	guard guard_object(node);
	ros::spin();
 	return 0;
}
