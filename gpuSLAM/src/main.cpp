#include "main.h"

ParticleFilter pf;
Maps og;
int scan_count=0;
int valid_scan_count=0;
float *ranges;
int *valid;
double newPos[3] = {0, 0, 0};

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{	
	scan_angle_min = msg->angle_min;
	scan_angle_max = msg->angle_max;
	scan_angle_inc = msg->angle_increment;
	scan_count = (scan_angle_max - scan_angle_min)/scan_angle_inc+1;
	memset(valid,0,scan_count);
	valid_scan_count = 0;
	//std::cout << "received scan of size: " << scan_count << std::endl;
	for(int i=0;i<scan_count;i++) 
	{
		ranges[i] = msg->ranges[i];
		if((ranges[i] < 30) & (ranges[i] > 0.1))
		{
			valid[valid_scan_count] = i;
			valid_scan_count++;
		}
	}
}

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	double w = msg->pose.pose.orientation.w;
	double x = msg->pose.pose.orientation.x;
	double y = msg->pose.pose.orientation.y;
	double z = msg->pose.pose.orientation.z;
	double ysqr = y * y;
    double t0 = -2.0f * (ysqr + z * z) + 1.0f;
    double t1 = +2.0f * (x * y - w * z);
    newPos[0] = msg->pose.pose.position.x;
    newPos[1] = msg->pose.pose.position.y;
    newPos[2] = -std::atan2(t1,t0);
	//pf.newPos(msg->pose.pose.position.x, msg->pose.pose.position.y, -std::atan2(t1,t0));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "slam");
    ros::NodeHandle n;
			
	ranges = (float *)malloc(sizeof(float)*2000);
	valid = (int *)malloc(sizeof(int)*2000);

	n.param("particles",particles,500);
	n.param("motion_noise_x",motion_noise_x,0.05);
	n.param("motion_noise_y",motion_noise_y,0.05);
	n.param("motion_noise_theta",motion_noise_theta,0.0524);
	n.param("lmap_width",lmap_width,60);
	n.param("lmap_height",lmap_height,60);
	n.param("lmap_resolution",lmap_resolution,0.1);
	n.param("gmap_width",gmap_width,100);
	n.param("gmap_height",gmap_height,100);
	n.param("gmap_resolution",gmap_resolution,0.1);
	n.param("sat_thresh",sat_thresh,120);
	n.param("neff_thresh",neff_thresh,40);
	n.param("logodd_occ",logodd_occ,3);
	n.param("logodd_free",logodd_free,1);
	n.param("scan_angle_min",scan_angle_min,(double)-1.57079);
	n.param("scan_angle_max",scan_angle_max,(double)1.57079);
	n.param("scan_angle_inc",scan_angle_inc,(double)0.0044);
	n.param("scan_range_max",scan_range_max,(double)30);
	n.param("scan_range_min",scan_range_min,(double)0.1);

	
	og.init(lmap_width, lmap_height, lmap_resolution, gmap_width, gmap_height,
        gmap_resolution, sat_thresh, logodd_occ, logodd_free,
        scan_angle_min, scan_angle_max, scan_angle_inc);
	pf.initPF(particles, motion_noise_x, motion_noise_y, motion_noise_theta, neff_thresh, (scan_angle_max - scan_angle_min)/scan_angle_inc+1, og.gmap);

	initSLAM(scan_count, scan_angle_max, scan_angle_min, scan_angle_inc, particles);

	ros::Publisher lmap_pub = n.advertise<nav_msgs::OccupancyGrid>("lmap", 1000);
	ros::Publisher gmap_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1000);
	ros::Subscriber scan = n.subscribe("scan", 1, scanCallback);
	ros::Subscriber pose = n.subscribe("scanmatch_odom", 1, poseCallback);
	std::cout << "slam node started" << std::endl;
	ros::Rate loop_rate(50);

	while(ros::ok()) 
	{
		if(scan_count>0) {
			runSLAM(&pf, &og, ranges, valid, scan_count,&newPos[0],valid_scan_count);
			og.publish(lmap_pub,gmap_pub);
			scan_count = 0;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}	
	return 1;
}