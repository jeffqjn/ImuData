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
#include "matplotlibcpp.h"
using namespace std;
using namespace ibex;
#ifndef IMUDATA_PARTICLE_FILTER_H
#define IMUDATA_PARTICLE_FILTER_H
long curTime();
class Particle_Filter
{
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

public:
    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> particle_filter_set_up(Parameters &parameters,IMU &imu, KdTree & kd, LiDAR_PointCloud &pointcloud ,int argc, char ** argv);
    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> generate_particle(IntervalVector box_6d, int num0,int num1, int num2, int num3, int num4, int num5);
    void transform_use_particle(pcl::PointCloud<pcl::PointXYZ> pointcloud, Eigen::Vector3d &angle, Eigen::Vector3d &translation);
    void transform_use_particle_Interval(vector<IntervalVector> pointcloud_interval, Eigen::Vector3d &angle, Eigen::Vector3d &translation,vector<IntervalVector> &after_transform_interval);
    Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta);
    IntervalVector create_6D_box(IMU imu, LiDAR_PointCloud pointcloud);
    double calculate_weight(LiDAR_PointCloud &pointcloud, vector<int> &indices, vector<bool> &if_matched, IntervalVector & point_after_transform,int k);
    Eigen::Vector3d estimation_position(vector<pair<Eigen::Vector3d,Eigen::Vector3d>>est_pos,Eigen::Vector3d &initial_pos);
    IntervalVector IntervalrotationMatrixtoEulerAngle(IntervalMatrix & matrix);
    void pointcloud_show( int argc,char **argv,pcl::PointCloud<pcl::PointXYZ> transformed, pcl::PointCloud<pcl::PointXYZ> match);
    void show_pointcloud(int argc, char** argv);
    void build_LiDAR_Interval(Parameters &parameters,LiDAR_PointCloud &pointcloud);
    void intervalCalcuateThread(pcl::PointCloud<pcl::PointXYZ> *pc, int start_index, int end_index,double rho_uncertinty,double phi_uncertinty,double theta_uncertinty,double horizontal_angle_uncertainty,double vertical_angle_uncertainty,std::mutex* mutex);
    void particleCalcuateThread(LiDAR_PointCloud *pointCloud,int start_index, int end_index);

};

#endif //IMUDATA_PARTICLE_FILTER_H
