#include "arc_msgs/State.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include"ackermann_msgs/AckermannDrive.h"
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
	void getCommand (const ackermann_msgs::AckermannDrive::ConstPtr& save_com);
private:
	//NodeHandle.
	ros::NodeHandle nh_;
	//Publishers::.
	//Pullisher for the save steering angle and the velocity.
	ros::Publisher safeState_pub_;
	//Subscriber:.
	//Subscriber to the state.
	ros::Subscriber state_sub_;
	//Subscriber to the stellgroessen.
	ros::Subscriber stellgroessen_sub_;
	//Transfer varible.
	ackermann_msgs::AckermannDrive trans_stellgroessen_;
	//Status variable. 
	bool status_;
	//Safe steering_angle. 
	float keep_steering_angle_;
};

	


	
