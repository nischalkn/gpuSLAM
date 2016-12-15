#include "../include/maps.h"
//#include "timer.h"

#define blockSize 128

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

	int lmap_size = sizeof(char)*lmap.info.width * lmap.info.height;
	int gmap_size = sizeof(char)*gmap.info.width * gmap.info.height;
	int map_size = sizeof(char)*map.info.width * map.info.height;
	cudaMalloc((void**)&dev_lmap, lmap_size);
	cudaMalloc((void**)&dev_gmap, gmap_size);
	cudaMalloc((void**)&dev_map, map_size);
	cudaMemcpy(dev_lmap, &lmap.data[0], lmap_size, cudaMemcpyHostToDevice);
	cudaMemcpy(dev_gmap, &gmap.data[0], gmap_size, cudaMemcpyHostToDevice);
	cudaMemcpy(dev_map, &map.data[0], map_size, cudaMemcpyHostToDevice);

	int scan_count = (scan_angle_max - scan_angle_min)/scan_angle_inc+1;
	cudaMalloc((void**)&dev_rotated_scan_x, sizeof(double)*scan_count);
	cudaMalloc((void**)&dev_rotated_scan_y, sizeof(double)*scan_count);
	cudaMalloc((void**)&dev_cells_x, sizeof(int)*scan_count);
	cudaMalloc((void**)&dev_cells_y, sizeof(int)*scan_count);
}

void Maps::publish(const ros::Publisher& pub1, const ros::Publisher& pub2)
{
	int lmap_size = sizeof(char)*lmap.info.width * lmap.info.height;
	//int gmap_size = sizeof(char)*gmap.info.width * gmap.info.height;
	int map_size = sizeof(char)*map.info.width * map.info.height;
	cudaMemcpy(&lmap.data[0], dev_lmap, lmap_size, cudaMemcpyDeviceToHost);
	//cudaMemcpy(&gmap.data[0], dev_gmap, gmap_size, cudaMemcpyDeviceToHost);
	cudaMemcpy(&map.data[0], dev_map, map_size, cudaMemcpyDeviceToHost);
	pub1.publish(lmap);
	pub2.publish(map);
}

__global__ void kernelCopyLtoG(int num, double x, double y, double theta, int width_lmap, int height_lmap,
	int width_gmap, int height_gmap, char* lmap, char* gmap, char* map, int saturation_thresh)
{
	int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
	if(idx >= num)
		return;

	int offset_w = (width_gmap - width_lmap)/2;
	int offset_h = (height_gmap - height_lmap)/2;
	int i = idx/width_lmap;
	int j = idx - i*width_lmap;
	int gmap_index1D = (y+i+offset_h) * width_gmap + (x+j+offset_w);
	gmap[gmap_index1D] += lmap[idx];
	
	short int val = gmap[gmap_index1D];
	if(val > saturation_thresh)
		gmap[gmap_index1D] = saturation_thresh;
	else if(val < -saturation_thresh)
		gmap[gmap_index1D] = -saturation_thresh;
	if(val > 50)
		map[gmap_index1D] = 100;
	else if(val < -30)
		map[gmap_index1D] = 0;
}

void Maps::copyLocalToGlobal(double x, double y, double theta)
{
	int maxCellsWidth_lmap = lmap.info.width;
	int maxCellsHeight_lmap = lmap.info.height;
	int maxCellsWidth_gmap = gmap.info.width;
	int maxCellsHeight_gmap = gmap.info.height;
	
	dim3 gridSize((lmap.info.width * lmap.info.height + blockSize - 1) / blockSize);
	kernelCopyLtoG<<<gridSize, blockSize>>>(lmap.info.width * lmap.info.height, x, y, theta, maxCellsWidth_lmap, 
		maxCellsHeight_lmap, maxCellsWidth_gmap, maxCellsHeight_gmap, dev_lmap, dev_gmap, dev_map, saturation_thresh);
	
	gmap.header.frame_id = "map";
  	gmap.header.stamp = ros::Time::now();
  	map.header.frame_id = "map";
  	map.header.stamp = ros::Time::now();
}

__global__ void kernelCleanMap(char* data, int size)
{
	int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
	if(idx >= size)
		return;
	data[idx] = 120;
}

void Maps::clearLmap() 
{
	//lmap.data.assign(lmap.info.width * lmap.info.height, 0);
	dim3 gridSize((lmap.info.width * lmap.info.height + blockSize - 1) / blockSize);
	kernelCleanMap<<<gridSize, blockSize>>>(dev_lmap,lmap.info.width * lmap.info.height);
}

__global__ void kernelConvertToCart(float* ranges, int *valid, float *scan_cart_x, float *scan_cart_y, int num,
	double angle_min, double angle_inc)
{
	int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
	if(idx >= num)
		return;
	scan_cart_x[idx] = ranges[valid[idx]]*std::cos(angle_min+angle_inc*valid[idx]);
	scan_cart_y[idx] = ranges[valid[idx]]*std::sin(angle_min+angle_inc*valid[idx]);
}

void Maps::convertToCart(float *ranges, int *valid, int num, float *scan_cart_x, float *scan_cart_y)
{
	dim3 gridSize((num + blockSize - 1) / blockSize);
	kernelConvertToCart<<<gridSize,blockSize>>>(ranges,valid,scan_cart_x,scan_cart_y,num, scan_angle_min, scan_angle_inc);
}

__global__ void kernelConvertToCells(int num, double *rotated_scan_x, double *rotated_scan_y,
	float* scan_cart_x, float* scan_cart_y, double R0, double R1, double R2, double R3, int *cells_x,
	int *cells_y, double res)
{
	int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
	if(idx >= num)
		return;
	rotated_scan_x[idx] = R0*scan_cart_x[idx] + R1*scan_cart_y[idx];
	rotated_scan_y[idx] = R2*scan_cart_x[idx] + R3*scan_cart_y[idx];
	cells_x[idx] = ceil(rotated_scan_x[idx]/res);
	cells_y[idx] = ceil(rotated_scan_y[idx]/res);
}

__global__ void kernelCreateLmap(int num, int *cells_x, int *cells_y, int maxCellsWidth, int maxCellsHeight,
	char* data, int logodd_free, int logodd_occ)
{
	int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
	if(idx >= num)
		return;

	if(cells_x[idx] > -maxCellsWidth/2 && cells_x[idx]<maxCellsWidth/2 && cells_y[idx] > -maxCellsHeight/2 && cells_y[idx]<maxCellsHeight/2)
	{
		int x0 = 0, y0 = 0;
		int dx = abs(cells_x[idx]-x0), sx = x0<cells_x[idx] ? 1 : -1;
		int dy = abs(cells_y[idx]-y0), sy = y0<cells_y[idx] ? 1 : -1; 
		int err = (dx>dy ? dx : -dy)/2, e2;
		int i = 0;
		int temp_x, temp_y;

		for(;;)
		{
			if (x0==cells_x[idx] && y0==cells_y[idx]) break;
			temp_x = x0+maxCellsWidth/2;
			temp_y = y0+maxCellsHeight/2;
			data[temp_y * maxCellsWidth + temp_x] = -logodd_free;
			e2 = err;
			if (e2 >-dx) { err -= dy; x0 += sx; }
			if (e2 < dy) { err += dx; y0 += sy; }
			i++;
		}

		temp_x = x0+maxCellsWidth/2;
		temp_y = y0+maxCellsHeight/2;
		data[temp_y * maxCellsWidth + temp_x] = logodd_occ;
	}
}

void Maps::createLocalMap(double *pose, float *scan_cart_x, float *scan_cart_y, int valid_scans)
{
	double ang = pose[2];
	double R[2][2] = {std::cos(ang), -std::sin(ang), std::sin(ang), std::cos(ang)};

	dim3 gridSize((valid_scans + blockSize - 1) / blockSize);
	kernelConvertToCells<<<gridSize,blockSize>>>(valid_scans, dev_rotated_scan_x, dev_rotated_scan_y, 
		scan_cart_x, scan_cart_y, R[0][0], R[0][1], R[1][0], R[1][1], dev_cells_x, dev_cells_y, lmap.info.resolution);
	
	kernelCreateLmap<<<gridSize,blockSize>>>(valid_scans, dev_cells_x, dev_cells_y, lmap.info.width, lmap.info.height,
		dev_lmap, logodd_free, logodd_occ);

	lmap.header.frame_id = "map";
  	lmap.header.stamp = ros::Time::now();
}