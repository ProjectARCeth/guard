#include "guard.hpp"
//Konstruktor.
guard::guard(const ros::NodeHandle &nh, bool status) : nh_(nh)
{
	//Transfer status. 
	status_=status;
	//Publishers.
	safeState_pub=nh_.advertise<std_msgs::Float64MultiArray>("/stellgroessen_safe", 1000);
	//Subscriber.
	state_sub=nh_.subscribe("/state", 100, &guard::getStopBool, this);
	stellgroessen_sub=nh_.subscribe("/stellgroessen", 100, &guard::getCommand, this);
}
//Standart Destruktor.
guard::~guard() {}
//Member methods.
//Callback function which processes the incoming stellgroessen.
void guard::getCommand (const std_msgs::Float64MultiArray::ConstPtr& save_com)
{	if(status_)
	{
		trans_stellgroessen.data[0]=save_com->data[0];
		trans_stellgroessen.data[1]=save_com->data[1];
	}
	else
	{
		trans_stellgroessen.data[0]=0;
		trans_stellgroessen.data[1]=0;
	}
	safeState_pub.publish(trans_stellgroessen);
}
void guard::getStopBool(const arc_msgs::State::ConstPtr& arc_state)
{
	status_=arc_state->stop;
}

