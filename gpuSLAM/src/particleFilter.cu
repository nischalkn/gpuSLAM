#include "../include/particleFilter.h"
//#include "timer.h"

#define blockSize 128

__global__ void kernelUpdateMapWidth(int num, double *x_im, float min_w, float resolution)
{
	int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
	if(idx >= num)
		return;	
	x_im[idx] = min_w + resolution*idx;
}

__global__ void kernelUpdateMapHeight(int num, double *y_im, float min_h, float resolution)
{
	int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
	if(idx >= num)
		return;	
	y_im[idx] = min_h + resolution*idx;
}

void ParticleFilter::initPF(int parts, double mn_x, double mn_y, double mn_theta, int neff_t, int scan_count, nav_msgs::OccupancyGrid gmap)
{
	particles = parts;
	motion_noise[0] = mn_x;
	motion_noise[1] = mn_y;
	motion_noise[2] = mn_theta;
	neff_thresh = neff_t;
	weights = (double *)malloc(sizeof(double)*particles);
	for (int i = 0; i < particles; ++i)
		weights[i] = (double)1/particles;
	particles_pose = (double *)malloc(sizeof(double)*particles*3);
	memset(particles_pose,0,particles*3);
	cumulative_sum = (double *)malloc(sizeof(double)*particles);

	cudaMalloc((void**)&dev_Weights, sizeof(double)*particles);
	cudaMalloc((void**)&dev_particles_pose, sizeof(double)*particles*3);
	cudaMalloc((void**)&dev_Cumulative_sum, sizeof(double)*particles);
	cudaMalloc((void**)&dev_R, sizeof(double)*particles*4);
	cudaMemcpy(weights, dev_Weights, sizeof(double)*particles, cudaMemcpyHostToDevice);
	cudaMemcpy(particles_pose, dev_particles_pose, sizeof(double)*particles*3, cudaMemcpyHostToDevice);
	cudaMemcpy(cumulative_sum, dev_Cumulative_sum, sizeof(double)*particles, cudaMemcpyHostToDevice);

	resolution = gmap.info.resolution;
	min_w = -((gmap.info.width-1)*(resolution))/2;
	min_h = -((gmap.info.height-1)*(resolution))/2;
	height = gmap.info.height;
	width = gmap.info.width;

	xmin = min_w;
	xmax = min_w + resolution*(height-1);
	xresolution = (xmax-xmin)/(height-1);

	ymin = min_h;
	ymax = min_h + resolution*(width-1);
	yresolution = (ymax-ymin)/(width-1);

	// dim3 gridSize((gmap.info.width + blockSize - 1) / blockSize);
	// kernelUpdateMapWidth<<<gridSize, blockSize>>>(gmap.info.width, dev_x_im, min_w, resolution);

	// dim3 gridSize2((gmap.info.height + blockSize - 1) / blockSize);
	// kernelUpdateMapHeight<<<gridSize2, blockSize>>>(gmap.info.height, dev_y_im, min_h, resolution);

	correlationScores = (long*)malloc(sizeof(long)*particles);
	int gridSize = ((particles + blockSize - 1) / blockSize);
 	cudaMalloc((void**)&d_sums, sizeof(double) * (gridSize + 1)); 
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

__global__ void kernelGenerateParticlePose(int num, double *R, double *particles_pose, double local_odom_x,
	double local_odom_y, double local_odom_theta, double motion_noise_x, double motion_noise_y, double motion_noise_theta)
{
	int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
	if(idx >= num)
		return;
	curandState state;
    curand_init(clock64(), idx, 0, &state);
	double ang = particles_pose[idx*3+2];
	R[idx*4+0] = cos(ang); R[idx*4+1] = -sin(ang); R[idx*4+2] = sin(ang); R[idx*4+3] = cos(ang);
	double global_odom[3];
	global_odom[0] = R[idx*4+0]*local_odom_x+R[idx*4+1]*local_odom_y;
	global_odom[1] = R[idx*4+2]*local_odom_x+R[idx*4+3]*local_odom_y;
	global_odom[2] = local_odom_theta;
	double m_noise[3];
	m_noise[0] = (curand_uniform(&state)-0.5)*motion_noise_x;
	m_noise[1] = (curand_uniform(&state)-0.5)*motion_noise_y;
	m_noise[2] = (curand_uniform(&state)-0.5)*motion_noise_theta;
	particles_pose[idx*3+0] += global_odom[0] + m_noise[0];
	particles_pose[idx*3+1] += global_odom[1] + m_noise[1];
	particles_pose[idx*3+2] += global_odom[2] + m_noise[2];
	ang = particles_pose[idx*3+2];
	R[idx*4+0] = cos(ang); R[idx*4+1] = -sin(ang); R[idx*4+2] = sin(ang); R[idx*4+3] = cos(ang);
}

__global__ void kernelCalculateScores(int num, float *scan_cart_x, float *scan_cart_y, double *R, double *particles_pose, 
	char *map, int valid_scans,	long *correlationScores, double ymin, double yresolution, int width, double xmin, 
	double xresolution, int height)
{
	int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
	if(idx >= num)
		return;

	long cpr = 0;
	for (int k = 0; k < valid_scans; k++) 
	{
		double x0 = R[idx*4+0]*scan_cart_x[k] + R[idx*4+1]*scan_cart_y[k] + particles_pose[idx*3+0];
		double y0 = R[idx*4+2]*scan_cart_x[k] + R[idx*4+3]*scan_cart_y[k] + particles_pose[idx*3+1];

		int iy = (int) round((y0-ymin)/yresolution);
		if ((iy < 0) || (iy >= width)) 
			continue;

		int ix = (int) round((x0-xmin)/xresolution);
		if ((ix < 0) || (ix >= height)) 
			continue;

		int index = iy + height*ix;
		cpr += map[index];
	}
	correlationScores[idx] = cpr;

}

void ParticleFilter::calculateScores(double *local_odom, char* dev_map, float *dev_scan_cart_x, 
	float *dev_scan_cart_y, int valid_scans, long *dev_correlationScores) 
{
	dim3 gridSize((particles + blockSize - 1) / blockSize);
	kernelGenerateParticlePose<<<gridSize, blockSize>>>(particles, dev_R, dev_particles_pose, local_odom[0],
		local_odom[1], local_odom[2], motion_noise[0], motion_noise[1], motion_noise[2]);

	kernelCalculateScores<<<gridSize, blockSize>>>(particles, dev_scan_cart_x, dev_scan_cart_y, dev_R, dev_particles_pose, 
		dev_map, valid_scans, dev_correlationScores, ymin, yresolution, width, xmin, xresolution, height);

}

void ParticleFilter::getBestPose(double *best_pose, long *dev_correlationScores)
{
	//CpuTimer timer;
	//timer.Start();
	cudaMemcpy(correlationScores, dev_correlationScores, sizeof(long)* particles, cudaMemcpyDeviceToHost);

	long min_c = correlationScores[0];
	double sum_weights = 0;
	for (int i = 1; i < particles; ++i)
		min_c = std::min(correlationScores[i], min_c);

	for (int i = 0; i < particles; ++i)
	{
		correlationScores[i] -= min_c;
		weights[i] *= correlationScores[i];
		sum_weights +=weights[i];
	}

	for (int i = 0; i < particles; ++i)
		weights[i] /= sum_weights;
	cudaMemcpy(dev_Weights, weights, sizeof(double)*particles, cudaMemcpyHostToDevice);

	int bestParticle = std::distance(weights, std::max_element(weights, weights + particles));
	cudaMemcpy(best_pose, particles_pose+bestParticle*3+0, sizeof(double), cudaMemcpyDeviceToHost);
	cudaMemcpy(best_pose+1, particles_pose+bestParticle*3+1, sizeof(double), cudaMemcpyDeviceToHost);
	cudaMemcpy(best_pose+2, particles_pose+bestParticle*3+2, sizeof(double), cudaMemcpyDeviceToHost);
	// best_pose[0] = particles_pose[bestParticle*3+0];
	// best_pose[1] = particles_pose[bestParticle*3+1];
	// best_pose[2] = particles_pose[bestParticle*3+2];
	// timer.Stop();
	// printf("%f\n", timer.Elapsed());
}

__global__ void block_sum(double *input, double *results, size_t n)
{
	extern __shared__ double sdata[];
	int i = threadIdx.x + blockDim.x * blockIdx.x;
	if(i>n)
		return;
	int tx = threadIdx.x;
	// load input into __shared__ memory
	double x = 0;
	if(i < n)
		x = input[i];
	sdata[tx] = x;
	__syncthreads(); 
	// block-wide reduction in __shared__ mem
	for(int offset = blockDim.x / 2; offset > 0; offset >>= 1)
	{
		if(tx < offset)
		{
			// add a partial sum upstream to our own
			sdata[tx] += sdata[tx + offset];
		}
		__syncthreads();
	} 
	// finally, thread 0 writes the result
	if(threadIdx.x == 0)
	{
	// note that the result is per-block
	// not per-thread
		results[blockIdx.x] = sdata[0];
	}
} 

__global__ void block_sum2(double *input, double *results, size_t n)
{
	extern __shared__ double sdata[];
	int i = threadIdx.x + blockDim.x * blockIdx.x;
	if(i>n)
		return;
	int tx = threadIdx.x;
	// load input into __shared__ memory
	double x = 0;
	if(i < n)
		x = input[i];
	sdata[tx] = x*x;
	__syncthreads(); 
	// block-wide reduction in __shared__ mem
	for(int offset = blockDim.x / 2; offset > 0; offset >>= 1)
	{
		if(tx < offset)
		{
			// add a partial sum upstream to our own
			sdata[tx] += sdata[tx + offset];
		}
		__syncthreads();
	} 
	// finally, thread 0 writes the result
	if(threadIdx.x == 0)
	{
	// note that the result is per-block
	// not per-thread
		results[blockIdx.x] = sdata[0];
	}
} 

__global__ void scan_bel(double* inputarray,int loop,double* outputarray,int number)
{
	unsigned int thIdx = blockIdx.x * blockDim.x + threadIdx.x;

	int divisor = 2;
	int adder = 1;
	int temp;

	for(int i=0;i<loop;i++)
	{
		if(thIdx%(divisor) == divisor-1)
		{
			outputarray[thIdx] = outputarray[thIdx-adder]+outputarray[thIdx];
		}
		__syncthreads();
		divisor*=2;
		adder*=2;
	}

	divisor = number;
	adder = divisor/2;

	outputarray[number-1] = 0;
	for(int i=0;i<loop;i++)
	{
		if(thIdx%(divisor) == divisor-1)
		{
			temp = outputarray[thIdx];
			outputarray[thIdx] = outputarray[thIdx-adder]+outputarray[thIdx];
			outputarray[thIdx-adder] = temp;
		}
		__syncthreads();
		divisor/=2;
		adder/=2;
	}
}

__global__ void kernelResample(int num, double *cumulative_sum, double *particles_pose)
{
	int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
	if(idx >= num)
		return;

	curandState state;
	double rand_num = curand_uniform(&state);

	int j = 0;
	while(rand_num>cumulative_sum[j])
	{
		j++;
	}
	particles_pose[idx*3+0] = particles_pose[j*3+0];
	particles_pose[idx*3+1] = particles_pose[j*3+1];
	particles_pose[idx*3+2] = particles_pose[j*3+2];
}

void ParticleFilter::resampleParticles()
{

	// reduce per-block partial sums
	int smem_sz = blockSize*sizeof (double);
	int gridSize = ((particles + blockSize - 1) / blockSize);
	block_sum<<<gridSize,blockSize,smem_sz>>>(dev_Weights, d_sums, particles);
	// reduce partial sums to a total sum
	block_sum<<<1,blockSize,smem_sz>>>(d_sums, d_sums + gridSize, gridSize); 

	double sum_weights;
	double sum_weight_squares;
	cudaMemcpy(&sum_weights, d_sums+gridSize, sizeof(double), cudaMemcpyDeviceToHost);

	block_sum2<<<gridSize,blockSize,smem_sz>>>(dev_Weights, d_sums, particles);
	// reduce partial sums to a total sum
	block_sum<<<1,blockSize,smem_sz>>>(d_sums, d_sums + gridSize, gridSize); 
	cudaMemcpy(&sum_weight_squares, d_sums+gridSize, sizeof(double), cudaMemcpyDeviceToHost);
	
	double neff = (sum_weights*sum_weights)/sum_weight_squares;
	
	if(neff<neff_thresh)
	{
		int loop;
		loop = (int)log2f(particles);
		dim3 gridSize((particles + blockSize - 1) / blockSize);
		scan_bel<<<gridSize,blockSize>>>(dev_Weights,loop,dev_Cumulative_sum,particles);

		kernelResample<<<gridSize, blockSize>>>(particles, dev_Cumulative_sum, dev_particles_pose);

		for (int i = 0; i < particles; ++i)
			weights[i] = (double)1/particles;
	}
}