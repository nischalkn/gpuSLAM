//#include "../include/main.h"
#include "../include/particleFilter.h"
#include "../include/maps.h"
#include <cuda.h>
#include <cuda_runtime.h>

int firstscan = 1;
float *dev_scan_cart_x;
float *dev_scan_cart_y;
long *dev_correlationScores;
float *dev_ranges;
int *dev_valid;

void initSLAM(int scan_count, double scan_angle_max, double scan_angle_min, double scan_angle_inc, int particles)
{
	scan_count = (scan_angle_max - scan_angle_min)/scan_angle_inc+1;
	// scan_cart_x = (float *)malloc(sizeof(float)*scan_count);
	// scan_cart_y = (float *)malloc(sizeof(float)*scan_count);
	// correlationScores = (long*)malloc(sizeof(long)*particles);
	cudaMalloc((void**)&dev_scan_cart_x, sizeof(float)*scan_count);
	cudaMalloc((void**)&dev_scan_cart_y, sizeof(float)*scan_count);
	cudaMalloc((void**)&dev_correlationScores, sizeof(long)*particles);
	cudaMalloc((void**)&dev_ranges, sizeof(float)*scan_count);
	cudaMalloc((void**)&dev_valid, sizeof(int)*scan_count);
}

void runSLAM(ParticleFilter *pf, Maps *og, float* ranges, int* valid, int scan_count, double *newPos, int valid_scans)
{
	double local_odom[3];
	double best_pose[3] = {0,0,0};
	cudaMemcpy(dev_ranges, ranges, sizeof(float)*scan_count, cudaMemcpyHostToDevice);
	cudaMemcpy(dev_valid, valid, sizeof(int)*scan_count, cudaMemcpyHostToDevice);
	cudaMemset(dev_scan_cart_x, 0, sizeof(float)*scan_count);
	cudaMemset(dev_scan_cart_y, 0, sizeof(float)*scan_count);
	if(firstscan)
	{
		og->clearLmap();
		og->convertToCart(dev_ranges, dev_valid, valid_scans, dev_scan_cart_x, dev_scan_cart_y);
		og->createLocalMap(&best_pose[0], dev_scan_cart_x, dev_scan_cart_y, valid_scans);
		og->copyLocalToGlobal(0,0,0);
		firstscan = 0;
	}
	else
	{
		og->clearLmap();
		pf->newPos(newPos[0], newPos[1], newPos[2]);
		og->convertToCart(dev_ranges, dev_valid, valid_scans, dev_scan_cart_x, dev_scan_cart_y);
		pf->motionUpdate(&local_odom[0]);
		// //nav_msgs::OccupancyGrid gmap = og->returnGmap();
		pf->calculateScores(&local_odom[0], og->dev_gmap, dev_scan_cart_x, dev_scan_cart_y, valid_scans, dev_correlationScores);
	// 	pf->getBestPose(&best_pose[0], correlationScores);
	// 	//std::cout << best_pose[0] << ", " << best_pose[1] << ", " << best_pose[2] << std::endl;
	// 	og->createLocalMap(&best_pose[0], scan_cart_x, scan_cart_y, valid_scans);
	// 	double resolution = og->gmap.info.resolution;
	// 	og->copyLocalToGlobal(ceil(best_pose[0]/resolution),ceil(best_pose[1]/resolution),0);
	// 	pf->resampleParticles();
	// 	// for (int i = 0; i < particles; ++i)
	// 	// {
	// 	// 	std::cout << i << ", " << correlationScores[i] << std::endl;
	// 	// }
	// 	// while(1);
	}
}