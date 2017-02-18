#include "arc_msgs/State.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include"ackermann_msgs/AckermannDrive.h"
class guard
{
public:
	//Konstruktor.
	guard(const ros::NodeHandle &nh);
	//Save the incoming current state.
	void getState (const arc_msgs::State::ConstPtr& arc_state);
	//Save the incoming incoming steering angle and the velocity.
	void getCommand (const ackermann_msgs::AckermannDrive::ConstPtr& save_com);
	void getLaserStop (const std_msgs::Bool::ConstPtr& msg);
	void getVcuStop (const std_msgs::Bool::ConstPtr& msg);
	void chooseAndPublish();
private:
	ros::NodeHandle nh_;
	//Publishers::.
	ros::Publisher safe_command_pub_;
	ros::Publisher not_stop_pub_;
	//Subscriber
	ros::Subscriber state_sub_;
	ros::Subscriber stellgroessen_sub_;
	ros::Subscriber grid_stop_sub_;
	//Subscriber to aVcu stop bool.
	ros::Subscriber vcu_stop_sub_;
	//Transfer varible.
	ackermann_msgs::AckermannDrive guard_stellgroessen_;
	//Status variable. 
	bool state_stop_;
	bool laser_stop_;
	bool vcu_stop_;
	bool not_stop_;
	std_msgs::Bool not_stop_msg_;
};

	


	
