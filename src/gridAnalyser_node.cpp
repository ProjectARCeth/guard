#include "../include/gridAnalyser/gridAnalyser.hpp"
int main(int argc, char **argv)
{	
	ros::init(argc, argv, "gridAnalyser");
	ros::NodeHandle node;
	gridAnalyser gridAnalyser_object(node);
	gridAnalyser_object.readPathFromTxt("pathRov_openLoop2.txt"); //Only for testing. 
	gridAnalyser_object.calculateIndex(8, 4); //Only for testing. 
	ros::spin();
	return 0;
}
