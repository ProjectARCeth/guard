#include "../include/gridAnalyser/gridAnalyser.hpp"
//Constructor
gridAnalyser::gridAnalyser(const ros::NodeHandle &nh): nh_(nh)
{
	//First value of status_.
	status_=0;
	//Publsiher. 
	//status_pub=nh_.advertise 
	state_sub=nh_.subscribe("/state", 100, &gridAnalyser::getState, this);
	grid_map_sub=nh_.subscribe("/NicoMap", 100, &gridAnalyser::getGridMap, this);
}
//Standart-Destructor
gridAnalyser::~gridAnalyser(){}
//Callback Function which processes incoming state
void gridAnalyser::getState (const arc_msgs::State::ConstPtr& arc_state)
{
	//something is done
}
//Callback Function which processes incoming Gridmap
void gridAnalyser::getGridMap(const nav_msgs::OccupancyGrid::ConstPtr& grid_map)
{
	int width=grid_map->info.width;
	int height=grid_map->info.height;
	float resolution=grid_map->info.resolution; //resolution is equal to length and width of cell.
	float x_curr;
	float y_curr;
	float z_curr;
	int ccells=width*height; //Number of cells.
	
	for(int i=0; i<ccells; i++)
	{
		int nx=i%width+1;
		int ny=(i+1-nx)/width+1;
		
	}
		
}
//Function to save from TXT-File to a Path copied from Mortiz
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
		std::cout<<j<<" "<<path_.poses[j-1].pose.position.x<<" "<<path_.poses[j-1].pose.position.y<<" "<<path_.poses[j-1].pose.position.z<<std::endl;		
		stream.ignore (300, '|');
		i++;	
		}

}
