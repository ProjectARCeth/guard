#include "../include/gridAnalyser/gridAnalyser.hpp"

float FOS_TUBE=1.5;
float FOS_BRAKING_DISTANCE=2;
float TOTAL_WIDTH=1.6;
float D_LASER_REARAXIS=1;
//Constructor
gridAnalyser::gridAnalyser(const ros::NodeHandle &nh): nh_(nh)
{
	std::cout<<"Constructor"<<std::endl;
	//Publsiher. 
	stop_pub_=nh_.advertise<std_msgs::Bool>("/laser_stop",10);
	distance_to_obstacle_pub_=nh_.advertise<std_msgs::Float64>("/distance_to_obstacle",10);
	danger_pub_=nh_.advertise<nav_msgs::OccupancyGrid>("/danger_grid",10);			//to visualize danger zone with rviz
	path_pub_=nh_.advertise<nav_msgs::Path>("/path",10);
	//Subscriber.
	
	grid_map_sub_=nh_.subscribe("/gridmap", 1, &gridAnalyser::getGridMap, this);
		
	//Read path.
	readPathFromTxt("/home/moritz/.ros/Paths/current_path_HG.txt");
	std::cout<<"GRID ANALYSER: Constructor"<<std::endl;
}

//Callback Function which processes incoming state
void gridAnalyser::getState (const arc_msgs::State::ConstPtr& arc_state)
{
/*
std::cout<<"Position: "<<arc_state->pose.pose.position.x<<" "<<arc_state->pose.pose.position.y<<" "<<arc_state->pose.pose.position.z<<std::endl;
std::cout<<"Quatrnions: "<<arc_state->pose.pose.orientation.x<<" "<<arc_state->pose.pose.orientation.y<<" "<<arc_state->pose.pose.orientation.z<<" "<<arc_state->pose.pose.orientation.w<<std::endl;
std::cout<<"Velocities: "<<arc_state->pose_diff.twist.linear.x<<" "<<arc_state->pose_diff.twist.linear.y<<" "<<arc_state->pose_diff.twist.linear.z<<std::endl;
std::cout<<"Angular velocities: "<<arc_state->pose_diff.twist.angular.x<<" "<<arc_state->pose_diff.twist.angular.y<<" "<<arc_state->pose_diff.twist.angular.z<<std::endl;
*/


	state_=*arc_state;
/*
std::cout<<"Position: "<<state_.pose.pose.position.x<<" "<<state_.pose.pose.position.y<<" "<<state_.pose.pose.position.z<<std::endl;
std::cout<<"Quatrnions: "<<state_.pose.pose.orientation.x<<" "<<state_.pose.pose.orientation.y<<" "<<state_.pose.pose.orientation.z<<" "<<state_.pose.pose.orientation.w<<std::endl;
std::cout<<"Velocities: "<<state_.pose_diff.twist.linear.x<<" "<<state_.pose_diff.twist.linear.y<<" "<<state_.pose_diff.twist.linear.z<<std::endl;
std::cout<<"Angular velocities: "<<state_.pose_diff.twist.angular.x<<" "<<state_.pose_diff.twist.angular.y<<" "<<state_.pose_diff.twist.angular.z<<std::endl;
*/
	//LOOP
	std::cout<<"GRID ANALYSER: Loop NewState"<<std::endl;
	float x_now=state_.pose.pose.position.x;
	float y_now=state_.pose.pose.position.y;
	float x_path=path_.poses[state_.current_arrayposition].pose.position.x;
	float y_path=path_.poses[state_.current_arrayposition].pose.position.y;
	tracking_error_=sqrt(pow((x_now-x_path),2)+pow((y_now-y_path),2));
	createDangerZone (nico_map_);
	compareGrids();
	publish_all();
	
	//END LOOP
}
//SAFE NICOMAP
void gridAnalyser::getGridMap(const nav_msgs::OccupancyGrid::ConstPtr& grid_map)
{	
	nico_map_=*grid_map;
	state_sub_=nh_.subscribe("/state", 1, &gridAnalyser::getState, this);	//Definierung state subscriber
	//LOOP

	//Calc tracking error.
	std::cout<<"GRID ANALYSER: Loop NewGrid"<<std::endl;
	float x_now=state_.pose.pose.position.x;
	float y_now=state_.pose.pose.position.y;
	float x_path=path_.poses[state_.current_arrayposition].pose.position.x;
	float y_path=path_.poses[state_.current_arrayposition].pose.position.y;
	tracking_error_=sqrt(pow((x_now-x_path),2)+pow((y_now-y_path),2));
	//Jetzt lateral error selber berechnet und ist noch Fehler HINTERACHSE

	n_cells_=nico_map_.info.height*nico_map_.info.width;	//Easy access to this parameter
	width_=grid_map->info.width;					//Easy access to this parameter
	height_=grid_map->info.height;				//Easy access to this parameter
	resolution_=grid_map->info.resolution; 			//Easy access to this parameter

	//Really neccessary steps	
	createDangerZone (nico_map_);
	compareGrids();
	publish_all();
	//END LOOP
	//path_pub_.publish(path_);
		
}

//DANGERZONE (Schlauch um Pfad)
void gridAnalyser::createDangerZone (const nav_msgs::OccupancyGrid grid_map)
{	
	tube_map_=grid_map;
	for (int i=0;i<n_cells_;i++) tube_map_.data[i]=0;
	//Inflate path
	for(int i=state_.current_arrayposition; i<n_poses_path_; i++) inflate(gridIndexOfGlobalPoint(path_.poses[i].pose.position));
}

//INFLATE XY
void gridAnalyser::inflate(int x, int y)
{
	if((x>0) && x<height_ && y<0 && (y>-width_))
	{
		float Radius_float=((TOTAL_WIDTH/2+tracking_error_)*FOS_TUBE)/resolution_;
		int R=5;//round(Radius_float);
		for(int i=(x-R); i<=(x+R); i++)
		{
			for(int j=(y-R); j<=(y+R); j++)
			{	
				//calculates dinstance squared between (i, j) and (x, y).
				float d=(((x-i)*(x-i))+((y-j)*(y-j)));
				if((i>0) && (i<height_) && (j<0) && j>-width_ && (d<(R*R)))				//Wieso Klammern??
				{
					//If (i, j) is on the map and the distance to (x,y) is smaller than the defined.
					tube_map_.data[convertIndex(i, j)]=100;
				}
			}
		}
	}
}
//INFLATEINDEX
void gridAnalyser::inflate(int n)
{	
	int* p=convertIndex(n);
	inflate(p[0],p[1]);
}

//GRIDCOMPARE
void gridAnalyser::compareGrids()
{	int counter=0;
	int j=0;
	for(int i=0; i<n_cells_; i++)
	{	
		int a=tube_map_.data[i];
		int b=nico_map_.data[i];
		int distance_old=100;
		if ((a!=0)&&(b!=0))
		{	counter++;				//Counts matching cells.
			int* p=convertIndex(i);
			float distance_new=sqrt(pow(p[0]-width_/2,2)+pow(p[1]-height_/2,2))*resolution_;		//Objektdistanz bezüglich gridMittelpunkt	
			if (distance_new<distance_old)
			{
				distance_old=distance_new;
				j=i;
			}
		}
	}

	if (counter!=0)		//Falls es minestens eine grid Zelle gibt, die obstacle enthält UND im gefahrbereich ist
	{
		whattodo(j);
		std::cout<<"GRID ANALYSER: Etwas auf dem Weg!"<<std::endl;
	}
	else 
	{	
		stop_=0;
		obstacle_distance_=100;
	}	
}	
//CONVERT
int gridAnalyser::convertIndex(int x, int y)
{	
	int n=(-y-1+width_*(x-1));	
	return n;
}
int* gridAnalyser::convertIndex(const int i)
{	
	int x[2];
	x[1]=-(i%width_)-1;	//y-richtung zelle
	x[0]=int(i/width_)+1;	//x-richtung zelle
	int* p=x;
	return p;
}

int gridAnalyser::gridIndexOfGlobalPoint(geometry_msgs::Point P)	//Global Point given with x up y right
{	
	int n=-1;		//Damit wenn es geändert wird gut, ansonsten wird in späteren schritten n=-1 durch die if eliminiert
	geometry_msgs::Point local_msg=arc_tools::globalToLocal(P,state_);
	float x_local=local_msg.x;	//Translation von Nico gemacht von Laser zu rearaxis 
	float y_local=local_msg.y;
	if(	(y_local/resolution_<width_/2) && (y_local/resolution_>(-width_/2)) && 
		(x_local/resolution_>-height_/2) && (x_local/resolution_<height_/2)  )
	{
      	n = round(round(-y_local/resolution_)+round(x_local/resolution_)*width_+width_/2+(height_/2)*width_);
	} 
	return n;
}
//""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

//WHATTODO
void gridAnalyser::whattodo(const int i)
{	
	int* p=convertIndex(i);
	float d=sqrt(pow(p[0]-height_/2,2)+pow(-p[1]-width_/2,2))*resolution_;	//Objektdistanz bezüglich gridMittelpunkt	
	float v_abs=sqrt(pow(state_.pose_diff.twist.linear.x,2)+pow(state_.pose_diff.twist.linear.y,2));
	obstacle_distance_=d;

	float crit_obs_dist=pow(v_abs*3.6,2)/2*FOS_BRAKING_DISTANCE;	//Bremsweg mit Sicherheitsfaktor (FOS_BRAKING_DISTANCE) Gleichung aus internet
		if(d<crit_obs_dist)
		{	stop_=1;
			std::cout<<"GRID ANALYSER: NOTSTOPP! Hindernis in Bremsweg"<<std::endl;
		}
		else
		{	
			stop_=0;
			std::cout<<"GRID ANALYSER: LANGSAMER! Doch noch nicht Not"<<std::endl;
		}
}


void gridAnalyser::publish_all()
{
stop_msg_.data=stop_;
stop_pub_.publish(stop_msg_);

obstacle_distance_msg_.data=obstacle_distance_;
distance_to_obstacle_pub_.publish(obstacle_distance_msg_);

danger_pub_.publish(tube_map_);
}

//READPATH
void gridAnalyser::readPathFromTxt(std::string inFileName)
{
	std::fstream fin; 	
	fin.open (inFileName.c_str());
	if(!fin.is_open())
		{
		std::cout << "GRID ANALYSER: Fehler beim Oeffnen von " <<inFileName << std::endl;
		}
	//Length of File.
	fin.seekg(-2,fin.end); //-2 to cut off the last |.
	int length = fin.tellg();
	fin.seekg (0, fin.beg);
	char * file = new char [length];
	fin.read (file,length);
	std::istringstream stream(file,std::ios::in);
	delete[] file;	
	fin.close () ; //Close.
	int i=0;
	int j;	
	geometry_msgs::PoseStamped temp_pose;
	while(!stream.eof()&& i<length)
		{
		geometry_msgs::PoseStamped temp_pose;	
		path_.poses.push_back(temp_pose);
		stream>>j;
		stream>>path_.poses[j-1].pose.position.x;
		stream>>path_.poses[j-1].pose.position.y;
		stream>>path_.poses[j-1].pose.position.z;	
		stream.ignore (300, '|');
		i++;	
		}
	n_poses_path_=i;

}

geometry_msgs::Point gridAnalyser::GlobalToLocal(geometry_msgs::Point global)
{
	//Translatation
	Eigen::Vector3d glob=arc_tools::transformPointMessageToEigen(global);
	Eigen::Vector3d stat= arc_tools::transformPointMessageToEigen(state_.pose.pose.position);
	Eigen::Vector3d temp=glob-stat;
	//Rotation
	Eigen::Vector4d quat=arc_tools::transformQuatMessageToEigen(state_.pose.pose.orientation);
	Eigen::Vector3d euler=arc_tools::transformEulerQuaternionVector(quat);
	Eigen::Matrix3d R=arc_tools::getRotationMatrix(euler);
	Eigen::Matrix3d T=R.transpose();
	Eigen::Vector3d local=T*temp;
	geometry_msgs::Point local_msg=arc_tools::transformEigenToPointMessage(local);
	return local_msg;
}
/*
arc_msgs::State gridAnalyser::arc_tools::generate2DState(const float x, const float y, const float alpha )
{
	arc_msgs::State state;
	state.pose.pose.position.x=x;
	state.pose.pose.position.y=y;
	geometry_msgs::Vector3 eu;
	eu.x=0;
	eu.y=0;
	eu.z=alpha;
	geometry_msgs::Quaternion quat;
	quat=arc_tools::transformQuaternionEulerMsg(eu);
	state.pose.pose.orientation=quat;
	return state;

}

*/






