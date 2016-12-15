#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/OccupancyGrid.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>

class ParticleFilter {
	private:
		int particles;
		double motion_noise[3];
		double prev_pose[3];
		double curr_pose[3];
		double *weights;
		double *particles_pose;
		double *cumulative_sum;
		int neff_thresh;
		double *dev_Weights;
		double *dev_particles_pose;
		double *dev_Cumulative_sum;
		double *dev_rotated_scan_x;
        double *dev_rotated_scan_y;
        double *dev_R;

		float resolution;
		float min_w;
		float min_h;
		int height;
		int width;
		double xmin;
		double xmax;
		double xresolution;
		double ymin;
		double ymax;
		double yresolution;

		long *correlationScores;
		double *d_sums;
	public:
		ParticleFilter()
		{
			prev_pose[0] = 0; prev_pose[1] = 0; prev_pose[2] = 0;
			curr_pose[0] = 0; curr_pose[1] = 0; curr_pose[2] = 0;
		}
		void initPF(int parts, double mn_x, double mn_y, double mn_theta, int neff_t, int scan_count, nav_msgs::OccupancyGrid gmap);
		void newPos(double x, double y, double theta);
		void motionUpdate(double *local_odom);
		void calculateScores(double *local_odom, char *dev_map, float *scan_cart_x, 
			float *scan_cart_y, int valid_scans, long *correlationScores);
		long map_correlation(nav_msgs::OccupancyGrid* gmap, double* y_im, double* x_im, double* vp_y, double* vp_x, int np);
		void getBestPose(double *best_pose, long *correlationScores);
		void resampleParticles();
};