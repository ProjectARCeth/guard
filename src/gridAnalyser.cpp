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
	erod_width_=0.8;
	FoS_=2;
	//Nachfolgendes ist nur zum Testen:
	height_=10;
	width_=10;
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
	width_=grid_map->info.width;
	height_=grid_map->info.height;
	resolution_=grid_map->info.resolution; //resolution is equal to length and width of cell.
	
	createSafeMap (grid_map);

/*
	float x_curr;
	float y_curr;
	float z_curr;
	int ccells=width*height; //Number of cells.
	
	for(int i=0; i<ccells; i++)
	{
		int nx=i%width+1;
		int ny=(i+1-nx)/width+1;
		
	}*/
		
}
//Function to save from TXT-File to a Path copied from Mortiz.
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
//Function to create the inflating map
void gridAnalyser::createSafeMap (const nav_msgs::OccupancyGrid::ConstPtr& grid_map)
{	
	//Set every cell to 0 in the safe grid.
	//X-direction. 
	for(int i=1; i<(width_+1); i++)
	{
		//Y-direction.
		for(int j=1; j<(height_+1); i++)
		{
			int n=calculateIndex(i, j);
			grid_safe_.data[n]=0;
		}
	}
	//Iterate over the whole incoming map.
	//X-direction. 
	for(int i=1; i<(width_+1); i++)
	{
		//Y-direction.
		for(int j=1; j<(height_+1); i++)
		{
			if(grid_map->data[calculateIndex(i,j)]!=0)
			{
				//Inflate if the the cell isn't empty.
				inflate(i,j);
			}
		}
	}
}
//Function which calculate the index n from (x, y) to row-major.
int gridAnalyser::calculateIndex(int x, int y)
{	
	int n=(x+(width_*(y-1)-1));	
	std::cout<<n<<std::endl; //Only for testing.
	return n;
}
//Function which inflates around (x, y).
void gridAnalyser::inflate(int x, int y)
{
	float Radius_float=(erod_width_*FoS_)/resolution_;
	float R=Radius_float;
	if((Radius_float-R)>0.5)
	{	
		//if-condition for the correct rounding. 
		R=R+1;
	}
	for(int i=(x-R); i<(x+R+1); i++)
	{
		for(int j=(y-R); j<(y+R+1); j++)
		{	
			//calculates dinstance squared between (i, j) and (x, y).
			float d=(((x-i)*(x-i))+((y-j)*(y-j)));
			if((i>0) && (i<(width_+1)) && (j>0) && (j<height_+1) && (d<(R*R)))
			{
				//If (i, j) is on the map and the distance to (x,y) is smaller than the defined.
				grid_safe_.data[calculateIndex(i, j)]=100;
			}
		}
	}
}
	

	























