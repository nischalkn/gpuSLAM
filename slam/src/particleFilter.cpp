#include "particleFilter.h"
#include "timer.h"
void ParticleFilter::initPF(int parts, double mn_x, double mn_y, double mn_theta, int neff_t)
{
	particles = parts;
	motion_noise[0] = mn_x;
	motion_noise[1] = mn_y;
	motion_noise[2] = mn_theta;
	neff_thresh = neff_t;
	weights = (double *)malloc(sizeof(double)*particles);
	//memset(weights,(double)1/particles,sizeof(double)*particles);
	for (int i = 0; i < particles; ++i)
		weights[i] = (double)1/particles;
	particles_pose = (double *)malloc(sizeof(double)*particles*3);
	memset(particles_pose,0,particles*3);
	cumulative_sum = (double *)malloc(sizeof(double)*particles);

	//pose_pub = n.advertise<geometry_msgs::Pose2D>("pose", 1000);
}

void ParticleFilter::newPos(double x, double y, double theta)
{
	prev_pose[0] = curr_pose[0];
	prev_pose[1] = curr_pose[1];
	prev_pose[2] = curr_pose[2];
	curr_pose[0] = x;
	curr_pose[1] = y;
	curr_pose[2] = theta;
	//std::cout << "Prev Pose: " <<curr_pose[0] << ", " << curr_pose[1] << ", " << curr_pose[2] << std::endl;
}

void ParticleFilter::motionUpdate(double *local_odom)
{
	double delta_pose[3];
	delta_pose[0] = curr_pose[0] - prev_pose[0];
	delta_pose[1] = curr_pose[1] - prev_pose[1];
	delta_pose[2] = curr_pose[2] - prev_pose[2];
	
	double R[2][2] = {std::cos(prev_pose[2]), std::sin(prev_pose[2]), -std::sin(prev_pose[2]), std::cos(prev_pose[2])};
	local_odom[0] = R[0][0]*delta_pose[0]+R[0][1]*delta_pose[1];
	local_odom[1] = R[1][0]*delta_pose[0]+R[1][1]*delta_pose[1];
	local_odom[2] = delta_pose[2];
	//std::cout << "Curr Pose: " <<curr_pose[0] << ", " << curr_pose[1] << ", " << curr_pose[2] << std::endl;
	//std::cout << "Prev Pose: " <<prev_pose[0] << ", " << prev_pose[1] << ", " << prev_pose[2] << std::endl;
	//std::cout << local_odom[0] << ", " << local_odom[1] << ", " << local_odom[2] << std::endl << std::endl;;
}

void ParticleFilter::calculateScores(double *local_odom, nav_msgs::OccupancyGrid gmap, float *scan_cart_x, 
	float *scan_cart_y, int valid_scans, long *correlationScores) 
{
	double *roatated_scan_x = (double*)malloc(sizeof(double)*valid_scans);
	double *roatated_scan_y = (double*)malloc(sizeof(double)*valid_scans);
	double *x_im = (double*)malloc(sizeof(double)*gmap.info.width);
	double *y_im = (double*)malloc(sizeof(double)*gmap.info.height);
	float resolution = gmap.info.resolution;
	float min_w = -((gmap.info.width-1)*(resolution))/2;
	float min_h = -((gmap.info.height-1)*(resolution))/2;

	// CpuTimer timer;
	// timer.Start();

	for (int i = 0; i < gmap.info.width; ++i)
	{
		x_im[i] = min_w + resolution*i;
	}
	for (int i = 0; i < gmap.info.height; ++i)
	{
		y_im[i] = min_h + resolution*i;
	}

	for (int i = 0; i < particles; ++i)
	{
		double ang = particles_pose[i*3+2];
		double R[2][2] = {std::cos(ang), -std::sin(ang), std::sin(ang), std::cos(ang)};
		double global_odom[3];
		global_odom[0] = R[0][0]*local_odom[0]+R[0][1]*local_odom[1];
		global_odom[1] = R[1][0]*local_odom[0]+R[1][1]*local_odom[1];
		global_odom[2] = local_odom[2];
		double m_noise[3];
		m_noise[0] = (((double) rand() / (RAND_MAX))-0.5)*motion_noise[0];
		m_noise[1] = (((double) rand() / (RAND_MAX))-0.5)*motion_noise[1];
		m_noise[2] = (((double) rand() / (RAND_MAX))-0.5)*motion_noise[2];
		particles_pose[i*3+0] += global_odom[0] + m_noise[0];
		particles_pose[i*3+1] += global_odom[1] + m_noise[1];
		particles_pose[i*3+2] += global_odom[2] + m_noise[2];
		ang = particles_pose[i*3+2];
		R[0][0] = cos(ang); R[0][1] = -sin(ang); R[1][0] = sin(ang); R[1][1] = cos(ang);

		for (int j = 0; j < valid_scans; ++j)
		{
			roatated_scan_x[j] = R[0][0]*scan_cart_x[j] + R[0][1]*scan_cart_y[j] + particles_pose[i*3+0];
			roatated_scan_y[j] = R[1][0]*scan_cart_x[j] + R[1][1]*scan_cart_y[j] + particles_pose[i*3+1];
		}

		correlationScores[i] = map_correlation(gmap,x_im,y_im,roatated_scan_y,roatated_scan_x, valid_scans);
		//std::cout << i << ": " << correlationScores[i] << std::endl;
	}
	// timer.Stop();
	// printf("%f\n", timer.Elapsed());
}

long ParticleFilter::map_correlation(nav_msgs::OccupancyGrid gmap, double* y_im, double* x_im, double* vp_y, double* vp_x, int np)
{
	signed char* im = &gmap.data[0];
	int nx = gmap.info.height;
	int ny = gmap.info.width;

	double xmin = x_im[0];
	double xmax = x_im[nx-1];
	double xresolution = (xmax-xmin)/(nx-1);
	//printf("x: %.3f %.3f %.3f\n", xmin, xmax, xresolution);

	double ymin = y_im[0];
	double ymax = y_im[ny-1];
	double yresolution = (ymax-ymin)/(ny-1);
	//printf("y: %.3f %.3f %.3f\n", ymin, ymax, yresolution);

	long cpr = 0;

	// CpuTimer timer;
	// timer.Start();
	for (int k = 0; k < np; k++) 
	{
		double x0 = vp_y[k];
		double y0 = vp_x[k];

		int iy = (int) round((y0-ymin)/yresolution);
		if ((iy < 0) || (iy >= ny)) 
			continue;

		int ix = (int) round((x0-xmin)/xresolution);
		if ((ix < 0) || (ix >= nx)) 
			continue;

		int index = iy + nx*ix;
		cpr += im[index];
	}
	// timer.Stop();
	// printf("%f\n", timer.Elapsed());
	return cpr;
}

void ParticleFilter::getBestPose(double *best_pose, long *correlationScores)
{
	//CpuTimer timer;
	//timer.Start();
	long min_c = correlationScores[0];
	double sum_weights = 0;
	for (int i = 1; i < particles; ++i)
		min_c = std::min(correlationScores[i], min_c);
	//std::cout << "min_c: " << min_c << std::endl;
	for (int i = 0; i < particles; ++i)
	{
		correlationScores[i] -= min_c;
		//std::cout << i << ": " << weights[i];
		weights[i] *= correlationScores[i];
		//std::cout << ", " << weights[i] << std::endl;
		sum_weights +=weights[i];
	}
	for (int i = 0; i < particles; ++i)
		weights[i] /= sum_weights;
	int bestParticle = std::distance(weights, std::max_element(weights, weights + particles));
	//std::cout << bestParticle << ", " << weights[bestParticle] << std::endl << std::endl;
	best_pose[0] = particles_pose[bestParticle*3+0];
	best_pose[1] = particles_pose[bestParticle*3+1];
	best_pose[2] = particles_pose[bestParticle*3+2];
	//std::cout << best_pose[0] << ", " << best_pose[1] << ", " << best_pose[2] << ", " << std::endl;
	// timer.Stop();
	// printf("%f\n", timer.Elapsed());
}

void ParticleFilter::resampleParticles()
{
	double sum_weights = 0;
	double sum_weight_squares = 0;
	for (int i = 0; i < particles; ++i)
	{
		sum_weights += weights[i];
		sum_weight_squares += weights[i]*weights[i];
	}
	double neff = (sum_weights*sum_weights)/sum_weight_squares;
	//std::cout << "neff: " << neff << std::endl;
	if(neff<neff_thresh)
	{
		CpuTimer timer;
		timer.Start();
		// /std::cout << "Resampled: " << neff << std::endl;
		cumulative_sum[0] = weights[0];
		for (int i = 1; i < particles; i++)
		{
			cumulative_sum[i] = cumulative_sum[i - 1] + weights[i];
		}
		
		for (int i = 0; i < particles; ++i)
		{
			//particles_pose(k,:) = pose_particle(find(rand <= cumulative_sum,1),:);
			double rand_num = (double) rand() / (RAND_MAX);
			int j = 0;
			while(rand_num>cumulative_sum[j])
			{
				j++;
			}
			particles_pose[i*3+0] = particles_pose[j*3+0];
			particles_pose[i*3+1] = particles_pose[j*3+1];
			particles_pose[i*3+2] = particles_pose[j*3+2];
		}

		for (int i = 0; i < particles; ++i)
			weights[i] = (double)1/particles;

		timer.Stop();
		printf("%f\n", timer.Elapsed());
	}
	// else
	// {
	// 	std::cout << "Dint Resample: "  << neff << std::endl;
	// }
}