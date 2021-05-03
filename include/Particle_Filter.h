//
// Created by jeffqjn on 12.03.21.
//
#include <rosbag/bag.h>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/time.h>
#include <pcl/PCLHeader.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <ibex.h>
#include <codac/codac.h>
#include <iostream>
#include "std_msgs/String.h"
#include "Parameters.h"
#include "KdTree.h"
#include "IMU.h"
#include "LiDAR_PointCloud.h"
#include "omp.h"
#include <chrono>
#include <thread>
#include <algorithm>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include "matplotlibcpp.h"
using namespace std;
using namespace ibex;
#ifndef IMUDATA_PARTICLE_FILTER_H
#define IMUDATA_PARTICLE_FILTER_H
long curTime();
class Particle_Filter
{
public:
    struct particle_weighted
    {
        particle_weighted(int index, double w, double w_n=0)
        {
            particle_index=index;
            weight=w;
            weight_normal=w_n;
        }
        int particle_index;
        double weight;
        double weight_normal;

    };
    struct particle_sorted
    {
        particle_sorted(double distance, Eigen::Vector3d value)
        {
            d=distance;
            v=value;
        }
        double d;
        Eigen::Vector3d v;
    };
public:

    pcl::PointCloud<pcl::PointXYZ> after_transform;
    Eigen::Matrix4d transformationsmatrix= Eigen::Matrix4d::Identity();
    map<int,int> summe_index;
    vector<pair<double,vector<IntervalVector>>> pointclouds_Interval;
    pcl::PointCloud<pcl::PointXYZ> transform_last_use_particle;
    pcl::PointCloud<pcl::PointXYZRGB> need_show_truth;
    pcl::PointCloud<pcl::PointXYZRGB> need_show_transformed;
    vector<IntervalVector> LiDARpoints;
    pcl::KdTreeFLANN<pcl::PointXYZ> tree_after_transform;
    double summe=0;
    vector<bool> if_matched;
    vector<particle_weighted> weights;
    visualization_msgs::MarkerArray unmatched_marker_array;
    visualization_msgs::MarkerArray matched_marker_array;
    visualization_msgs::MarkerArray truth_marker_array;
    visualization_msgs::MarkerArray boden_truth_marker_array;
    visualization_msgs::MarkerArray boden_transformed_marker_array;
    vector<int> label_transformed;
    vector<int> label_matched;
    int c=1;
    int d=1000;
    bool first=true;
    vector<int> match;
    pcl::PointCloud<pcl::PointXYZRGB> matched;
    pcl::PointCloud<pcl::PointXYZRGB> unmatched;
    pcl::PointCloud<pcl::PointXYZRGB> boden_truth;
    pcl::PointCloud<pcl::PointXYZRGB> boden_transformed;
    vector<int> point_index;
    vector<int> unmatch;
    vector<double> debug_sums;

    vector<geometry_msgs::PoseStamped> path_temp;
    int flag1;
    int flag2;

    double gesamte_sums=0;
    int particle_number=0;
    double max_value=0;
    double min_value=99999;
    int max_index;
    int min_index;
    int start_index=-1, end_index=-1;
    double start_time, end_time;
    bool particle_filter_first=true;
    int current_index_first, current_index_second;
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> particle;
    vector<particle_weighted> sums;
    vector<particle_weighted> resample_weight;
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> result;
    IntervalVector box_6D;
    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> ground_truth;
    double non_ground_weight;
    double ground_weight;
    int calculate_interval;
    vector<Eigen::Matrix4d> result_matrix;
public:


    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> particle_filter_set_up(Parameters &parameters,IMU &imu, KdTree & kd, LiDAR_PointCloud &pointcloud , Measurement &measurement,int argc, char ** argv);
    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> generate_particle(IntervalVector box_6d, int num0,int num1, int num2, int num3, int num4, int num5);
    void transform_use_particle(pcl::PointCloud<pcl::PointXYZ> pointcloud, Eigen::Vector3d &angle, Eigen::Vector3d &translation);
    void transform_use_particle_Interval(vector<IntervalVector> pointcloud_interval, Eigen::Vector3d &angle, Eigen::Vector3d &translation,vector<IntervalVector> &after_transform_interval);
    Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta);
    IntervalVector create_6D_box(IMU imu, LiDAR_PointCloud pointcloud,Measurement measurement);
    double calculate_weight(LiDAR_PointCloud &pointcloud, vector<int> &indices, vector<bool> &if_matched, IntervalVector & point_after_transform,int k,std::mutex* mutex);
    Eigen::Vector3d estimation_position(vector<pair<Eigen::Vector3d,Eigen::Vector3d>>est_pos,Eigen::Vector3d &initial_pos);
    IntervalVector IntervalrotationMatrixtoEulerAngle(IntervalMatrix & matrix);
    void pointcloud_show( int argc,char **argv);
    void show_boxes(int argc, char** argv);
    void build_LiDAR_Interval(Parameters &parameters,LiDAR_PointCloud &pointcloud);
    void intervalCalcuateThread(pcl::PointCloud<pcl::PointXYZ> *pc, int start_index, int end_index,double rho_uncertinty,double phi_uncertinty,double theta_uncertinty,double horizontal_angle_uncertainty,double vertical_angle_uncertainty,std::mutex* mutex);
    void particleCalcuateThread(LiDAR_PointCloud *pointCloud,int start_index, int end_index,std::mutex* mutex);
    void get_label(pcl::PointCloud<pcl::PointXYZI> &temp, vector<int> &label);
    void pointxyz2pointxyzi(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc2,pcl::PointCloud<pcl::PointXYZI> &temp);
    void add_marker_to_array(IntervalVector point,visualization_msgs::MarkerArray &marker_array, double r, double g,
                                              double b, double a);
    void show_pointcloud_original(int argc, char** argv,LiDAR_PointCloud & pointcloud);
    void add_point2pointcloud(LiDAR_PointCloud pointCloud);
    void update_max(double s, int index);
    void update_min(double s, int index);
    void get_ground_truth(Parameters &parameters, Measurement &measurement, IMU &imu);
    void pointcloud_show_match( int argc,char **argv);
    void get_start_end_cloud_index(LiDAR_PointCloud &pointcloud ,Parameters & parameters,int &start_index, int &end_index);
    void show_all(int argc, char ** argv, LiDAR_PointCloud &pointcloud);
    void validate_result(Parameters &parameters, Measurement &measurement, IMU &imu);
    void particle_filter_do(Parameters &parameters,IMU &imu, KdTree & kd, LiDAR_PointCloud &pointcloud , Measurement &measurement,int argc, char ** argv);
    void particle_filter_parallelism(Parameters &parameters, LiDAR_PointCloud &pointcloud, int particle_index);
    double calcualte_gesamte_sums();
    void calcualte_normalized_sums(double gesamt_sum);
    void resampling();
    void calculate_average();
    //DEBUG Version
    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> generate_particle_translation(IntervalVector box_6d, Eigen::Vector3d rotation,  int num3, int num4, int num5);
    vector<Eigen::Vector3d> generate_particle_rotation(IntervalVector box_6d,int num0, int num1, int num2);
    Eigen::Vector3d rotationMatrixtoEulerAngle(Eigen::Matrix3d & matrix);
    void plot_debug();
    void create_pic(Parameters &parameters,IMU &imu, KdTree & kd,  LiDAR_PointCloud &pointcloud ,Measurement &measurement,int argc, char ** argv);
    vector<double> calculate_distance();
    vector<Eigen::Vector3d> particle_sort(vector<Eigen::Vector3d> temp);
    double calculate_angular_distance(Eigen::Vector3d item);
    vector<double> calculate_rotation_distance();
    void copy_pointcloud(pcl::PointCloud<pcl::PointXYZ> & from, pcl::PointCloud<pcl::PointXYZ> & to);
    void show();
    void show_error();
    IntervalVector speed_prediction();
    void show_path(int argc, char** argv);
};

#endif //IMUDATA_PARTICLE_FILTER_H
