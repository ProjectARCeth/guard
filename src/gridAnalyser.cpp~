#include "guard/inlcude/gridAnalyser/gridAnalyser.hpp"
//Constructor
gridAnalyser::gridAnalyser(const ros::NodeHandle &nh): nh_(nh)
{
	//First value of status_.
	status_=0;
	//Publsiher. 
	//status_pub=nh_.advertise 
	state_sub=nh_.subscribe("/state", 100, &gridAnalyser::getState, this);
	//GridMap_sub=nh_.subscribe(...)
}
//Standart-Destructor
gridAnalyser::~gridAnalyser(){}
//Callback Function which processes incoming state
void gridAnalyser::getState (const arc_msgs::State::ConstPtr& arc_state)
{
	//something is done
}
//Callback Function which processes incoming Gridmap
//void gridAnalyser::getGridMap(const ...)
{
	//something ist done
}
