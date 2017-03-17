#include "../include/guard/guard.hpp"
//Konstruktor.
guard::guard(const ros::NodeHandle &nh) : nh_(nh)
{	//Inizialisierung
	not_stop_=0;
	state_stop_=0;
	vcu_stop_=0;
	laser_stop_=0;
	//YAML Parameter.
	nh_.getParam("/general/QUEUE_LENGTH", QUEUE_LENGTH);
	nh_.getParam("/topic/STELLGROESSEN_SAFE", STELLGROESSEN_SAFE_TOPICNAME);
	nh_.getParam("/topic/NOTSTOP", NOTSTOP_TOPICNAME);
	nh_.getParam("/topic/STATE", STATE_TOPICNAME);
	nh_.getParam("/topic/STELLGROESSEN", STELLGROESSEN_TOPICNAME);
	nh_.getParam("/topic/LASER_STOP", LASER_STOP_TOPICNAME);
	nh_.getParam("/topic/STOP_NI", STOP_NI_TOPICNAME);	
	//Publishers.
	safe_command_pub_=nh_.advertise<ackermann_msgs::AckermannDrive>(STELLGROESSEN_SAFE_TOPICNAME, QUEUE_LENGTH);
	not_stop_pub_=nh_.advertise<std_msgs::Bool>(NOTSTOP_TOPICNAME, QUEUE_LENGTH);
	//Subscriber.
	state_sub_=nh_.subscribe(STATE_TOPICNAME, QUEUE_LENGTH, &guard::getState, this);
	stellgroessen_sub_=nh_.subscribe(STELLGROESSEN_TOPICNAME, QUEUE_LENGTH, &guard::getCommand, this);
	grid_stop_sub_=nh_.subscribe(LASER_STOP_TOPICNAME, QUEUE_LENGTH, &guard::getLaserStop, this);
	vcu_stop_sub_=nh_.subscribe(STOP_NI_TOPICNAME, QUEUE_LENGTH, &guard::getVcuStop, this);
}

void guard::getState(const arc_msgs::State::ConstPtr& arc_state)
{	
	std::cout<<"State_CallBack"<<std::endl;
	state_stop_=arc_state->stop;
	chooseAndPublish();
}
void guard::getLaserStop(const std_msgs::Bool::ConstPtr& msg)
{
	std::cout<<"Laser_CallBack"<<std::endl;
	laser_stop_=msg->data;
	chooseAndPublish();
}
void guard::getVcuStop(const std_msgs::Bool::ConstPtr& msg)
{
	std::cout<<"Vcu_CallBack"<<std::endl;
	vcu_stop_=msg->data;
	chooseAndPublish();
}
void guard::getCommand (const ackermann_msgs::AckermannDrive::ConstPtr& msg)
{
	std::cout<<"Command_CallBack"<<std::endl;
	guard_stellgroessen_=*msg;
	chooseAndPublish();
}

void guard::chooseAndPublish()
{
	//Choose
	if( (state_stop_==0) && (vcu_stop_==0) && (laser_stop_==0) )
	{
		not_stop_=0;
	}
	else
	{
		not_stop_=1;
		guard_stellgroessen_.steering_angle=0;
		std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!NOTSTOP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
	}
	//Publish
	if(not_stop_=1)
	{
		for(int i=0;i<10;i++)
		{	
			not_stop_msg_.data=not_stop_;
			not_stop_pub_.publish(not_stop_msg_);
		}
	}
	else
	{
		not_stop_msg_.data=not_stop_;
		not_stop_pub_.publish(not_stop_msg_);
	}
}
/*
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
*/

