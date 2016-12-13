#include "ros/ros.h"
#include "gridAnalyser.hpp"
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gridAnalyser");
	ros::Nodehandle node;
	gridAnalyser gridAnalyser_object(node);
	ros::spin();
	return 0;
}
