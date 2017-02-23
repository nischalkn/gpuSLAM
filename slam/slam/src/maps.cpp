#include "maps.h"
#include "timer.h"

/**
*	Initialize local map, global map and occupancy grid
*
*	@param 	lmap_w 		Maximum width of local map.
*	@param 	lmap_h 		Maximum height of local map.
*	@param 	lmap_res 	Resolution of local map.
*	@param 	gmap_w 		Maximum width of global map.
*	@param 	gmap_h 		Maximum height of global map.
*	@param 	gmap_res 	Resolution of local map.
*	@param 	sat 		Maximum saturation for occupancy grid.
*	@param 	logodd_o 	LogOdds for occupied cell.
*	@param 	logodd_f 	LogOdds for free cell.
*	@param 	angle_min 	Minimum angle of LiDAR.
*	@param 	angle_max 	Maximum angle of LiDAR.
*	@param 	angle_inc 	Angle between scans.
*/
void Maps::init(int lmap_w, int lmap_h, double lmap_res, int gmap_w, int gmap_h,
		double gmap_res, int sat, int logodd_o, int logodd_f, double angle_min,
		double angle_max, double angle_inc)
{
	lmap.info.width = lmap_w/lmap_res + 1;
	lmap.info.height = lmap_h/lmap_res + 1;
	lmap.info.resolution = lmap_res;
	lmap.info.origin.position.x = -lmap_w/2;
	lmap.info.origin.position.y = -lmap_h/2;
	lmap.info.origin.position.z = 0;
	lmap.info.origin.orientation.x = 0;
	lmap.info.origin.orientation.y = 0;
	lmap.info.origin.orientation.z = 0;
	lmap.info.origin.orientation.w = 1;
	gmap.info.width = gmap_w/gmap_res + 1;
	gmap.info.height = gmap_h/gmap_res + 1;
	gmap.info.resolution = gmap_res;
	gmap.info.origin.position.x = -gmap_w/2;
	gmap.info.origin.position.y = -gmap_h/2;
	gmap.info.origin.position.z = 0;
	gmap.info.origin.orientation.x = 0;
	gmap.info.origin.orientation.y = 0;
	gmap.info.origin.orientation.z = 0;
	gmap.info.origin.orientation.w = 1;
	map.info.width = gmap_w/gmap_res + 1;
	map.info.height = gmap_h/gmap_res + 1;
	map.info.resolution = gmap_res;
	map.info.origin.position.x = -gmap_w/2;
	map.info.origin.position.y = -gmap_h/2;
	map.info.origin.position.z = 0;
	map.info.origin.orientation.x = 0;
	map.info.origin.orientation.y = 0;
	map.info.origin.orientation.z = 0;
	map.info.origin.orientation.w = 1;
	scan_angle_min = angle_min;
	scan_angle_max = angle_max;
	scan_angle_inc = angle_inc;
	saturation_thresh = sat;
	logodd_occ = logodd_o;
	logodd_free = logodd_f;
	lmap.data.resize(lmap.info.width * lmap.info.height);
	gmap.data.resize(gmap.info.width * gmap.info.height);
	gmap.data.assign(gmap.info.width * gmap.info.height, 0);
	map.data.resize(map.info.width * map.info.height);
	map.data.assign(map.info.width * map.info.height, -1);
}

/**
*	Draw a line between two coordinates using Breshnam's algorithm
*
*	@param 	x0 		X coordinate of point 1.
*	@param 	y0 		Y coordinate of point 1.
*	@param 	x1 		X coordinate of point 2.
*	@param 	y1 		X coordinate of point 2.
*	@param 	xis 	All the X coordinates through which the line passes through
*	@param 	yis 	All the Y coordinates through which the line passes through
*	@return 		Number of points in the line
*/
int Maps::line(int x0, int y0, int x1, int y1, int *xis, int *yis) {
 
	int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
	int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1; 
	int err = (dx>dy ? dx : -dy)/2, e2;
	int i = 0;

	for(;;)
	{
		xis[i] = x0; yis[i] = y0;
		if (x0==x1 && y0==y1) break;
		e2 = err;
		if (e2 >-dx) { err -= dy; x0 += sx; }
		if (e2 < dy) { err += dx; y0 += sy; }
		i++;
	}
	return i;
}

/**
*	Publish a local map and occupancy grid
*
*	@param 	pub1	Topic to which local map has to be published.
*	@param 	y0 		Topic to which occupancy grid must be published.
*/
void Maps::publish(const ros::Publisher& pub1, const ros::Publisher& pub2)
{
		pub1.publish(lmap);
		pub2.publish(gmap);
}

/**
*	Copy the local map generated from the current scan to the global map displaced
*	by (x, y) from the center of global map and roated by theta anticlockwise.
*
*	@param 	x 		translation of local map from center of global map along X axis
*	@param 	y 		translation of local map from center of global map along Y axis
*	@param 	theta 	rotation of local map in anti clockwise direction.
*/
void Maps::copyLocalToGlobal(double x, double y, double theta)
{
	int maxCellsWidth_lmap = lmap.info.width;
	int maxCellsHeight_lmap = lmap.info.height;
	int maxCellsWidth_gmap = gmap.info.width;
	int maxCellsHeight_gmap = gmap.info.height;
	// CpuTimer timer;
	// timer.Start();
	for (int i = 0; i < maxCellsHeight_lmap; ++i)
	{
		for (int j = 0; j < maxCellsWidth_lmap; ++j)
		{
			int lmap_index1D = i * maxCellsWidth_lmap + j;
			int offset_w = (maxCellsWidth_gmap - maxCellsWidth_lmap)/2;
			int offset_h = (maxCellsHeight_gmap - maxCellsHeight_lmap)/2;
			int gmap_index1D = (y+i+offset_h) * maxCellsWidth_gmap + (x+j+offset_w);
			gmap.data[gmap_index1D] += lmap.data[lmap_index1D];
			short int val = gmap.data[gmap_index1D];
			if(val > saturation_thresh)
				gmap.data[gmap_index1D] = saturation_thresh;
			else if(val < -saturation_thresh)
				gmap.data[gmap_index1D] = -saturation_thresh;
			if(val > 50)
				map.data[gmap_index1D] = 100;
			else if(val < -30)
				map.data[gmap_index1D] = 0;
		}
	}
	// timer.Stop();
	// printf("%f\n", timer.Elapsed());
	gmap.header.frame_id = "map";
  	gmap.header.stamp = ros::Time::now();
  	map.header.frame_id = "map";
  	map.header.stamp = ros::Time::now();
}

/**
*	Return the Global map
*
*	@return 		Global Map
*/
nav_msgs::OccupancyGrid Maps::returnGmap()
{
	return gmap;
}

/**
*	Clean the local map by setting all pixels/cells to 0
*
*/
void Maps::clearLmap() 
{
	lmap.data.assign(lmap.info.width * lmap.info.height, 0);
}

/**
*	Convert laser scan from polar coordinate to cartesian coordinate
*
*	@param 	ranges 			radius in polar coordinates in meters
*	@param 	valid 			index of scans which are valid between 0.1m and 30m
*	@param 	num 			Total number of scans
*	@param 	scan_cart_x 	Cartesian X coordinate of valid scans only in meters
*	@param 	scan_cart_y 	Cartesian Y coordinate of valid scans only in meters
*	@return 				Total number of valid scans
*/
int Maps::convertToCart(float *ranges, int *valid, int num, float *scan_cart_x, float *scan_cart_y)
{
	int j = 0;
	//CpuTimer timer;
	//timer.Start();
	for(int i=0;i<num;i++) 
	{
		if(valid[i]==1) 
		{
			scan_cart_x[j] = ranges[i]*std::cos(scan_angle_min+scan_angle_inc*i);
			scan_cart_y[j] = ranges[i]*std::sin(scan_angle_min+scan_angle_inc*i);
			j++;
		}
	}
	//timer.Stop();
	//printf("%f\n", timer.Elapsed());
	return j;
}

/**
*	Create a local map from the current scan based on the current pose
*
*	@param 	pose 			Current pose of the robot obtained from sensor.
*	@param 	scan_cart_x 	X coordinates of scans in cartesian system
*	@param 	scan_cart_y 	Y coordinates of scans in cartesian system
*	@param 	valid_scans 	total number of valid scans.
*/
void Maps::createLocalMap(double *pose, float *scan_cart_x, float *scan_cart_y, int valid_scans)
{
	// CpuTimer timer;
	// timer.Start();
	double ang = pose[2];
	double R[2][2] = {std::cos(ang), -std::sin(ang), std::sin(ang), std::cos(ang)};
	double *roatated_scan_x = (double*)malloc(sizeof(double)*valid_scans);
	double *roatated_scan_y = (double*)malloc(sizeof(double)*valid_scans);
	int cells[valid_scans][2];
	int maxCellsWidth = lmap.info.width;
	int maxCellsHeight = lmap.info.height;
	for (int j = 0; j < valid_scans; ++j)
	{
		roatated_scan_x[j] = R[0][0]*scan_cart_x[j] + R[0][1]*scan_cart_y[j];
		roatated_scan_y[j] = R[1][0]*scan_cart_x[j] + R[1][1]*scan_cart_y[j];
		cells[j][0] = ceil(roatated_scan_x[j]/lmap.info.resolution);
		cells[j][1] = ceil(roatated_scan_y[j]/lmap.info.resolution);
	}

	// timer.Stop();
	// printf("%f\n", timer.Elapsed());

	// CpuTimer timer;
	// timer.Start();
	int *xis; int *yis;
	////////////////<---------------TODO: move these to initialization ----------/
	xis = (int *)malloc(sizeof(int)*maxCellsWidth*2);
	yis = (int *)malloc(sizeof(int)*maxCellsHeight*2);
	for(int i=0;i<valid_scans;i++) 
	{
		if(cells[i][0] > -maxCellsWidth/2 && cells[i][0]<maxCellsWidth/2 && cells[i][1] > -maxCellsHeight/2 && cells[i][1]<maxCellsHeight/2)
		{
			//std::cout << "scan num: " << i << ", " << cells[i][0] << ", " << cells[i][1] << std::endl;
			int len = Maps::line(0,0,cells[i][0],cells[i][1],xis,yis);
			//std::cout << "Out of line fn" << std::endl;
			for (int k = 0; k < len; ++k)
			{
				int temp_x = xis[k]+maxCellsWidth/2;
				int temp_y = yis[k]+maxCellsHeight/2;
				int index_1d = temp_y * maxCellsWidth + temp_x;
				//std::cout << "x: " << xis[k] << ", y: " << yis[k] << ", index_1d: " << index_1d << std::endl;
				lmap.data[index_1d] = -logodd_free;
			}
			int temp_x = xis[len]+maxCellsWidth/2;
			int temp_y = yis[len]+maxCellsHeight/2;
			int index_1d = temp_y * maxCellsWidth + temp_x;
			lmap.data[index_1d] = logodd_occ;
		}

	}
	//timer.Stop();
	//printf("%f\n", timer.Elapsed());
	lmap.header.frame_id = "map";
  	lmap.header.stamp = ros::Time::now();
}