#include "ros/ros.h"
#include "particleFilter.h"
#include "maps.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "math.h"

// Laser Params
double scan_angle_min;
double scan_angle_max;
double scan_angle_inc;
double scan_range_max;
double scan_range_min;

// Map parameters
int lmap_width;
int lmap_height;
double lmap_resolution;
int gmap_width;
int gmap_height;
double gmap_resolution;
int sat_thresh;
int neff_thresh;
int logodd_occ;
int logodd_free;

// Particle Filter param
int particles;
double motion_noise_x;
double motion_noise_y;
double motion_noise_theta;

void initSLAM(int, double, double, double, int);
void runSLAM(ParticleFilter*, Maps*, float*, int*, int, double*, int);