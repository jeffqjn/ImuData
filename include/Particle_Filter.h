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
private:

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
    int flag1;
    int flag2;

    double max_value=0;
    double min_value=99999;
    int max_index;
    int min_index;
    int start_index=-1, end_index=-1;

public:


    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> particle_filter_set_up(Parameters &parameters,IMU &imu, KdTree & kd, LiDAR_PointCloud &pointcloud , Measurement &measurement,int argc, char ** argv);
    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> generate_particle(IntervalVector box_6d, int num0,int num1, int num2, int num3, int num4, int num5);
    void transform_use_particle(pcl::PointCloud<pcl::PointXYZ> pointcloud, Eigen::Vector3d &angle, Eigen::Vector3d &translation);
    void transform_use_particle_Interval(vector<IntervalVector> pointcloud_interval, Eigen::Vector3d &angle, Eigen::Vector3d &translation,vector<IntervalVector> &after_transform_interval);
    Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta);
    IntervalVector create_6D_box(IMU imu, LiDAR_PointCloud pointcloud);
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
    vector<Eigen::Vector3d> get_ground_truth(Parameters &parameters, Measurement &measurement, IMU &imu);
    void pointcloud_show_match( int argc,char **argv);
    void get_start_end_cloud_index(LiDAR_PointCloud &pointcloud ,Parameters & parameters,int &start_index, int &end_index);
    void show_all(int argc, char ** argv, LiDAR_PointCloud &pointcloud);
};

#endif //IMUDATA_PARTICLE_FILTER_H
