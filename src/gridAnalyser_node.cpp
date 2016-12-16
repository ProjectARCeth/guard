#include "../include/gridAnalyser/gridAnalyser.hpp"
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gridAnalyser");
	ros::NodeHandle node;
	gridAnalyser gridAnalyser_object(node);
	gridAnalyser_object.readPathFromTxt("pathRov_openLoop2.txt");
	ros::spin();
	return 0;
}
