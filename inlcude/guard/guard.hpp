#include "arc_msgs/State.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
class guard
{
public:
	//Konstruktor.
	guard(const ros::NodeHandle &nh, bool status);
	//Destruktor.
	~guard();
	//Save the incoming current state.
	void getStopBool (const arc_msgs::State::ConstPtr& arc_state);
	//Save the incoming incoming steering angle and the velocity.
	void getCommand (const std_msgs::Float64MultiArray::ConstPtr& save_com);

private:
	//NodeHandle.
	ros::NodeHandle nh_;
	//Publishers::.
	//Pullisher for the save steering angle and the velocity.
	ros::Publisher safeState_pub;
	//Subscriber:.
	//Subscriber to the state.
	ros::Subscriber state_sub;
	//Subscriber to the stellgroessen.
	ros::Subscriber stellgroessen_sub;
	//Transfer varible.
	std_msgs::Float64MultiArray trans_stellgroessen;
	//Status variable. 
	bool status_;
};

	


	
