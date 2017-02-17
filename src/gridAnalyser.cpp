#include "../include/gridAnalyser/gridAnalyser.hpp"

float FoS_tube=1.5;
float FoS_break_distance=2;
float erod_width=1.6;
//Constructor
gridAnalyser::gridAnalyser(const ros::NodeHandle &nh): nh_(nh)
{
	std::cout<<"Constructor"<<std::endl;
//TEST	
/*width_=10;
height_=30;
resolution_=0.1;*/

/*geometry_msgs::Point P1,P2,P3;
P1.x=1;
P1.y=1;
P1.z=0;
P2.x=2;
P2.y=2;
P2.z=0;
P3.x=3;
P3.y=3;
P3.z=0;
int n1 =gridIndexOfGlobalPoint(P1);
int n2 =gridIndexOfGlobalPoint(P2);
int n3 =gridIndexOfGlobalPoint(P3);
inflate(0);
inflate(n1);
inflate(n2);
inflate(n3);
*/
	stop_=0;
	obstacle_distance_=100;
	tracking_error_=0.5;
	state_.pose.pose.position.x=0;
	state_.pose.pose.position.y=0;

	//Publsiher. 
	stop_pub_=nh_.advertise<std_msgs::Bool>("/laser_stop",10);
	distance_to_obstacle_pub_=nh_.advertise<std_msgs::Float64>("/distance_to_obstacle",10);		//mit welchem ?? bool stop oder distance??
	danger_pub_=nh_.advertise<nav_msgs::OccupancyGrid>("/danger_grid",10);				//to visualize danger zone with rviz
	//Subscriber.
	state_sub_=nh_.subscribe("/state", 100, &gridAnalyser::getState, this);
	grid_map_sub_=nh_.subscribe("/gridmap", 100, &gridAnalyser::getGridMap, this);
						//Brauchen wir? tracking_error_sub_=nh_.subscribe("/tracking_error", 100, &gridAnalyser::getTrackingError, this);
	//Read path.
	readPathFromTxt("/home/moritz/.ros/Paths/pathRov.txt");
	std::cout<<"Constructor finish"<<std::endl;
}

//Callback Function which processes incoming state
void gridAnalyser::getState (const arc_msgs::State::ConstPtr& arc_state)
{

	state_=*arc_state;
	//LOOP
	std::cout<<"Loop NewState"<<std::endl;
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
	//LOOP

	//Calc tracking error.
	std::cout<<"Loop NewGrid"<<std::endl;
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
		
}
/*//TRACKING ERROR	?????
void gridAnalyser::getTrackingError(const std_msgs::Float64::ConstPtr& t_err)
{
	tracking_error_=t_err->data;
}
*/
//DANGERZONE (Schlauch um Pfad)
void gridAnalyser::createDangerZone (const nav_msgs::OccupancyGrid grid_map)
{	
	
	tube_map_=grid_map;	//Damit Metadaten übereinstimmen( Zellenbreite ecc).
	//Set every cell to 0 in the tube_grid.
	for (int i=0;i<n_cells_;i++) 
	{
		tube_map_.data[i]=0;
	}

							//START TEST
							inflate(100,50);
							inflate(50,300);
							geometry_msgs::Point P1,P2;
							P1.x=1;
							P1.y=1;
							P1.z=0;
							P2.x=-2;
							P2.y=2;
							P2.z=0;							
							int n1 =gridIndexOfGlobalPoint(P1);
							int n2 =gridIndexOfGlobalPoint(P2);
							inflate(n1);
							inflate(n2);
							//for(int i=0;i<400;i++) inflate(n1+i*width_);
							//END TEST
	//Inflate path
	for(int i=state_.current_arrayposition; i<state_.current_arrayposition+100; i++)
	{	
		//int n =gridIndexOfGlobalPoint(path_.poses[i].pose.position);
		//inflate(n);
	}

}

//INFLATEXY
void gridAnalyser::inflate(int x, int y)
{
	float Radius_float=((erod_width/2+tracking_error_)*FoS_tube)/resolution_;
	int R=round(Radius_float);
	for(int i=(x-R); i<(x+R+1); i++)
	{
		for(int j=(y-R); j<(y+R+1); j++)
		{	
			//calculates dinstance squared between (i, j) and (x, y).
			float d=(((x-i)*(x-i))+((y-j)*(y-j)));
			if((i>0) && (i<(width_+1)) && (j>0) && (j<height_+1) && (d<(R*R)))				//Wieso Klammern??
			{
				//If (i, j) is on the map and the distance to (x,y) is smaller than the defined.
				tube_map_.data[convertIndex(i, j)]=100;
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
		std::cout<<"Etwas auf dem Weg!"<<std::endl;
	}
	else 
	{	
		stop_=0;
		obstacle_distance_=100;
		std::cout<<"Weg Frei!"<<std::endl;
	}	
}	
//CONVERT
int gridAnalyser::convertIndex(int x, int y)
{	
	int n=(x-1+width_*(y-1));	
	return n;
}
int* gridAnalyser::convertIndex(const int i)
{	
	int x[2];
	int r=i/width_;
	x[0]=i-r*width_+1;
	x[1]=r+1;
	int* p=x;
//	std::cout<<width_<<" "<<i<<" "<<x<<" "<<y<<std::endl;
	return p;
}
//POINT TO INDEX"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
int gridAnalyser::gridIndexOfGlobalPoint(geometry_msgs::Point P)	//Global Point given with x up y right
{
	int n=0;
	geometry_msgs::Point S=state_.pose.pose.position;
//S.x=2;
//S.y=1;
	geometry_msgs::Point V;
	V.x=P.x-S.x;
	V.y=P.y-S.y;
	V.z=P.z-S.z;
		float ox=state_.pose.pose.orientation.x;	
		float oy=state_.pose.pose.orientation.y;
		float oz=state_.pose.pose.orientation.z;
		float ow=state_.pose.pose.orientation.w;
		const Eigen::Vector4d quat(ox, oy, oz, ow);
		geometry_msgs::Vector3 eul;
		eul=arc_tools::transformEulerQuaternionMsg(quat);
		float theta2=-eul.z;					//theta2 soll positiv sein falls Auto im gegenuhrzeigersinn gedreht ist
//theta2=0;

		float theta1=atan2(V.y,V.x);
//		std::cout<<theta1+theta2<<std::endl;
	float d=sqrt(V.x*V.x+V.y*V.y);
	float x_local=d*cos(theta1+theta2);
	float y_local=d*sin(theta1+theta2);
	std::cout<<"X_local: "<<x_local<<std::endl<<"Y_local: "<<y_local<<std::endl;
      n = round(round(x_local/resolution_)*width_+round(y_local/resolution_)+width_/2+(height_/2)*width_);
	return n;
}
//""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

//WHATTODO
void gridAnalyser::whattodo(const int i)
{	
	int* p=convertIndex(i);
	float d=sqrt(pow(p[0]-width_/2,2)+pow(p[1]-height_/2,2))*resolution_;	//Objektdistanz bezüglich gridMittelpunkt	
	float v_abs=sqrt(pow(state_.pose_diff.twist.linear.x,2)+pow(state_.pose_diff.twist.linear.x,2));
	obstacle_distance_=d;

	float crit_obs_dist=pow(v_abs*3.6,2)/2*FoS_break_distance;	//Bremsweg mit Sicherheitsfaktor (FoS_break_distance) Gleichung aus internet
		if(d<crit_obs_dist)
		{	stop_=1;
			std::cout<<"NOTSTOPP! Hindernis in Bremsweg"<<std::endl;
		}
		else
		{	stop_=0;
			std::cout<<"LANGSAMER! Doch noch nicht Not"<<std::endl;

				
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
		std::cout << " Fehler beim Oeffnen von " <<inFileName << std::endl;
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
		//std::cout<<j<<" "<<path_.poses[j-1].pose.position.x<<" "<<path_.poses[j-1].pose.position.y<<" "<<path_.poses[j-1].pose.position.z<<std::endl;		
		stream.ignore (300, '|');
		i++;	
		}
	n_poses_path_=i;
}











