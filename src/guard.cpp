#include "guard.hpp"
//Konstruktor.
guard::guard(const ros::NodeHandle &nh, bool status) : nh_(nh)
{
	//Transfer status. 
	status_=status;
	//Publishers.
	safeState_pub_=nh_.advertise<ackermann_msgs::AckermannDrive>("/stellgroessen_safe", 1000);
	//Subscriber.
	state_sub_=nh_.subscribe("/state", 100, &guard::getStopBool, this);
	stellgroessen_sub_=nh_.subscribe("/stellgroessen", 100, &guard::getCommand, this);
}
//Standart Destruktor.
guard::~guard() {}
//Member methods.
//Callback function which processes the incoming stellgroessen.
void guard::getCommand (const ackermann_msgs::AckermannDrive::ConstPtr& save_com)
{	if(status_)
	{
		trans_stellgroessen_.steering_angle=save_com->steering_angle;
		//Keep the steering angle if systems fails.
		keep_steering_angle_=save_com->steering_angle;
		trans_stellgroessen_.speed=save_com->speed;
	}
	else
	{
		//If Systems fails brake and keep steering angle. 
		trans_stellgroessen_.steering_angle=keep_steering_angle_;
		trans_stellgroessen_.speed=0;
	}
	safeState_pub_.publish(trans_stellgroessen_);
}
void guard::getStopBool(const arc_msgs::State::ConstPtr& arc_state)
{
	status_=arc_state->stop;
}

