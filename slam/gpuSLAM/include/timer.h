#ifndef GPU_TIMER_H__
#define GPU_TIMER_H__

// #include <cuda_runtime.h>
#include <time.h>

// struct GpuTimer
// {
//   cudaEvent_t start;
//   cudaEvent_t stop;

//   GpuTimer()
//   {
//     cudaEventCreate(&start);
//     cudaEventCreate(&stop);
//   }

//   ~GpuTimer()
//   {
//     cudaEventDestroy(start);
//     cudaEventDestroy(stop);
//   }

//   void Start()
//   {
//     cudaEventRecord(start, 0);
//   }

//   void Stop()
//   {
//     cudaEventRecord(stop, 0);
//   }

//   float Elapsed()
//   {
//     float elapsed;
//     cudaEventSynchronize(stop);
//     cudaEventElapsedTime(&elapsed, start, stop);
//     return elapsed;
//   }
// };

struct CpuTimer
{
	clock_t begin;
	clock_t end;

	CpuTimer()
	{
		begin = 0;
		end = 0;
	}

	~CpuTimer()
	{
	}

	void Start()
	{
		begin = clock();
	}

	void Stop()
	{
		end = clock();
	}

	double Elapsed()
	{
		double elapsed = (double)(end - begin) / CLOCKS_PER_SEC;
		return elapsed;
	}
};

#endif  /* GPU_TIMER_H__ */
