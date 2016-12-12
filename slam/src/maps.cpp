#include "maps.h"
#include "timer.h"


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

int Maps::buildLocalMap(float *ranges, int *valid, int num)
{
	//std::cout << "building new map" << std::endl;
	lmap.data.assign(lmap.info.width * lmap.info.height, 0);
	double ranges_cart[num][2];
	int cells[num][2];
	int j = 0;
	int maxCellsWidth = lmap.info.width;
	int maxCellsHeight = lmap.info.height;
	for(int i=0;i<num;i++) 
	{
		if(valid[i]==1) 
		{
			ranges_cart[j][0] = ranges[i]*std::cos(scan_angle_min+scan_angle_inc*i);
			ranges_cart[j][1] = ranges[i]*std::sin(scan_angle_min+scan_angle_inc*i);
			cells[j][0] = ceil(ranges_cart[j][0]/lmap.info.resolution);
			cells[j][1] = ceil(ranges_cart[j][1]/lmap.info.resolution);
			j++;
//			std::cout << cells[j][0] << ", " << cells[j][1] << ", " << j << std::endl;
		}
	}

	//std::cout << "Valid scans: " << j << ", Map Size: " << lmap.data.size() << std::endl;

	int *xis; int *yis;
	xis = (int *)malloc(sizeof(int)*maxCellsWidth*2);
	yis = (int *)malloc(sizeof(int)*maxCellsHeight*2);
	for(int i=0;i<j;i++) 
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
	// for (int i = 0; i < maxCellsHeight; ++i)
	// {
	// 	for (int k = 0; k < maxCellsWidth; ++k)
	// 	{
	// 		std::cout << lmap.data[i*maxCellsWidth + k] << " ";
	// 	}
	// 	std::cout << std::endl;
	// 	/* code */
	// }
	lmap.header.frame_id = "map";
  	lmap.header.stamp = ros::Time::now();
	return j;
}

void Maps::publish(const ros::Publisher& pub1, const ros::Publisher& pub2)
{
		pub1.publish(lmap);
		pub2.publish(map);
}

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

nav_msgs::OccupancyGrid Maps::returnGmap()
{
	return gmap;
}

void Maps::clearLmap() 
{
	lmap.data.assign(lmap.info.width * lmap.info.height, 0);
}

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

void Maps::createLocalMap(double *pose, float *scan_cart_x, float *scan_cart_y, int valid_scans)
{
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

	// CpuTimer timer;
	// timer.Start();
	int *xis; int *yis;
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