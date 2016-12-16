#include "arc_msgs/State.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
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
	// void getGridMap (const ....);
private:
	//NodeHandle.
	ros::NodeHandle nh_;
	//Publisher for the boolean.
	ros::Publisher status_pub;
	//Subscriber to the state.
	ros::Subscriber state_sub;
	//Subscriber to the incoming map.
	//ros::Subscriber GridMap_sub
	bool status_;
}
