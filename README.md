**University of Pennsylvania, CIS 565: GPU Programming and Architecture**
Final Project - GPU accelerated SLAM
====================

* [Nischal K N](https://www.linkedin.com/in/nischalkn)
* Tested on: Linux Ubuntu 14.04, Nvidia Tegra K1 @ 2.32GHz 2GB, NVIDIA Kepler "GK20a" GPU

### SUMMARY
Mapping of the environment using sensor data is an important problem in robotics and even more in autonomous cars. Fusion of this data helps us to build a spatial model of the physical environment around the vehicle and also to know its position in the world if the spatial model is already available. This information is very crucial for making decisions while planning trajectories to the destination. In case of autonomous cars which typically travel at 60mph(88ft/s), it is crucial to update its position and map the surroundings at a very high rate. This project aims at speeding up one such mapping algorithm called Simultaneous Localization and Mapping(SLAM)[1] using Monte Carlo Localization utilizing the parallelism of the GPU.

### OUTPUT
The following image was built using LIDAR data real-time on the vehicle.

![](images/output.png)

### PARAMETERS
#### Subscribed Topics
* `scan` (sensor_msgs/LaserScan) - Laser scans from Lidar
* `scanmatch_odom`(nav_msgs/Odometry) - Odometry from dead reckoning with respect to global map

#### Published Topics
* `lmap` (nav_msgs/OccupancyGrid) - Local map orientied based on best pose
* `map` (nav_msgs/OccupancyGrid) - Global map
* `pose` (nav_msgs/Odometry) - Pose with respect to initial starting position

#### Parameters

* `particles` - Number of particles for Particle filter (default:500)
* `motion_noise_x`- Initial pose covariance (x*x), used to initialize filter with uniform distribution. (default:0.05)
* `motion_noise_y`- Initial pose covariance (y*y), used to initialize filter with uniform distribution. (default:0.05)
* `motion_noise_theta`- Initial pose covariance (yaw*yaw), used to initialize filter with uniform distribution.(default:0.0534)
* `lmap_width`- Local map width(default:60)
* `lmap_height`- Local map height(default:60)
* `lmap_resolution`- Local map resolution(default:0.1)
* `gmap_width`- Global map width(default:100)
* `gmap_height`- Global map height(default:100)
* `gmap_resolution`- Global map resolution(default:0.1)
* `sat_thresh`- saturation threshold of the occupancy grid cells(default:120)
* `neff_thresh`- Resample threshold(default:40)
* `logodd_occ`- log-odd free value(default:3)
* `logodd_free`- log-odd occupied value(default:1)

### Video
Video of map being built real time

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/xe6MGNJKmvg/0.jpg)](http://www.youtube.com/watch?v=xe6MGNJKmvg)

### Algorithm Overview
* Convert scan from polar to Cartesian coordinates (parallelized)
* Update the motion of the particle based on Odometry
* Generate uniformly distributed particles around odometry position and orient scans based on the new position (parallelized)
* Calculate correlation scores by comparing with the map (parallelized)
* Update weights and obtain pose of best particle (parallelized)
* Create local map with the best pose (parallelized)
* Copy the map from local buffer to global buffer (parallelized)
* Check and Resample particles (parallelized)

### Performance Analysis
The algorithm was implemented on CPU and GPU. Eight kernels were used to parallelize most parts of the algorithm. The execution time of these sections improved dramatically on the GPU. The different execution times on GPU and CPU for each kernel is shown in the figure below. The number of particles used is 500.

![](images/kernels.png)

It is seen that all the functionality had a speedup on the GPU except for calculation of CDF. This was because the CDF calculation involves an operation of atomic add that defeats the purpose of parallelism.

The total execution time for CPU and GPU for various particle sizes is shown in the figure below. The execution time in the CPU implementation increases exponentially with particle sizes. This is evident because of the nature of series computation. On the other hand the parallel computation on the GPU has a very linear increase with particle size. Additionally, another implementation with zero copy was performed as the operations were performed on streaming data. This eliminated the need for copying the data from one CPU to GPU. This implementation on an average executed 1-2ms faster.

![](images/time.png)

The following figure shows the update rates of GPU implementation with and without zero copy. These are the rates at which the output topics are published to the ROS system.

![](images/rate.png)

Another Performance done was on the power consumption of the execution. Three kernels were made to switch dynamically from CPU to GPU (enabled by zero copy)
1. Particle generation and scan orientation
2. Calculation of Correlation Scores
3. Copy of Local Map to Global Map
The CPU and the GPU were executed at different clock frequencies with different combinations of the above three kernels on CPU and GPU. The update rates and power consumption were recorded

![](images/power.png)

A combination of `C C G` denotes the first and second operation (mentioned above) run on CPU and the third on GPU. It is seen from the first graph that the update rate is maximum when all the three operations are run on the GPU but at the cost of high power as seen in the second graph. The information from this graph can be used to train a controller to switch to a lower update rate when the map is already known by varying the operations on CPU and GPU and also throttling the frequencies thus saving power.

### Install and Build instructions
1. Install Robot Operating System from [here](http://wiki.ros.org/ROS/Installation)
2. Create a catkin workspace using [this](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
3. Clone this repository into the src folder of the created Catkin workspace
4. `catkin_make` from the home of the workspace
5. Source the workspace using
> source devel/setup.bash
6. run `roscore`
6. Run Lidar node and odometry node
7. Run the GPU slam using
> rosrun slam gpuSLAM
8. To visualize data, run rviz. An rviz vizualization configuration file has been provided.
