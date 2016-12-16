#include "arc_msgs/State.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/OccupancyGrid.h"
#include  "nav_msgs/Path.h"
#include <iostream>
#include <fstream>
#include <sstream>
class gridAnalyser
{
public:
	//Constructor.
	gridAnalyser(const ros::NodeHandle &nh);
	//Destructor.
	~gridAnalyser();
	//Function which saves the incoming state.
	void getState (const arc_msgs::State::ConstPtr& arc_state);
	//Function which saves the incoming occupancy map.
	void getGridMap (const nav_msgs::OccupancyGrid::ConstPtr& grip_map);
	//Function to save from TXT-File to a Path copied from Moritz.
	void readPathFromTxt(std::string inFileName);

private:
	//NodeHandle.
	ros::NodeHandle nh_;
	//Publisher for the boolean.
	ros::Publisher status_pub;
	//Subscriber to the state.
	ros::Subscriber state_sub;
	//Subscriber to the incoming map.
	ros::Subscriber grid_map_sub;
	bool status_;
	//Variable to store the path.
	nav_msgs::Path path_;
};