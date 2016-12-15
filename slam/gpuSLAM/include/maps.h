#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8.h"
#include "stdio.h"
#include <cuda.h>
#include <cuda_runtime.h>

class Maps {
    private:
        int logodd_occ;
        int logodd_free;
        double scan_angle_min;
        double scan_angle_max;
        double scan_angle_inc;
        nav_msgs::OccupancyGrid lmap;
        nav_msgs::OccupancyGrid map;
        char* dev_lmap;
        char* dev_map;
        int saturation_thresh;
        double *dev_rotated_scan_x;
        double *dev_rotated_scan_y;
        int *dev_cells_x;
        int *dev_cells_y;

    public:
        nav_msgs::OccupancyGrid gmap;
        char* dev_gmap;
        void init(int lmap_w, int lmap_h, double lmap_res, int gmap_w, int gmap_h,
            double gmap_res, int sat, int logodd_o, int logodd_f,
            double, double, double);
        int buildLocalMap(float *ranges, int *valid, int num);
        void publish(const ros::Publisher& pub1, const ros::Publisher& pub2);
        void copyLocalToGlobal(double x, double y, double theta);
        //nav_msgs::OccupancyGrid returnGmap();
        void clearLmap(); 
        void convertToCart(float *ranges, int *valid, int num, float *scan_cart_x, float *scan_cart_y); 
        void createLocalMap(double *pose, float *scan_cart_x, float *scan_cart_y, int valid_scans);
};