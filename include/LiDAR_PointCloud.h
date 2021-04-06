//
// Created by noah on 27.02.21.
//
#include <iostream>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/console/time.h>
#include <pcl/PCLHeader.h>
#include <pcl/registration/icp.h>
#include "Parameters.h"
#include "KdTree.h"
#include <ibex.h>
#include <ros/ros.h>
#include <thread>
#include "FloorSegment.h"

using namespace std;
using namespace ibex;
#ifndef IMUDATA_LIDAR_POINTCLOUD_H
#define IMUDATA_LIDAR_POINTCLOUD_H
class LiDAR_PointCloud
{
public:
    vector<pair<double,pcl::PointCloud<pcl::PointXYZ>>> pointclouds;
    vector<pair<double,vector<int>>> labels;
    vector<pair<double,vector<IntervalVector>>> pointclouds_Interval;
    vector<ros::Time> velocity_0;
    Eigen::Matrix4d threshold_matrix= Eigen::Matrix4d::Identity ();
    Eigen::Matrix4d Template;
    pcl::PointCloud<pcl::PointXYZRGB> ground;
    bool pointcloud_ready=false;


public:
    LiDAR_PointCloud(double leading_diagonal_threshold, double sub_diagonal_threshold, double tranlation_threshold);
    void add_pointcloud(sensor_msgs::PointCloud2ConstPtr m, Parameters parameters,KdTree &kdTree,int argc, char** argv);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_convert(const boost::shared_ptr<const sensor_msgs::PointCloud2> & input);
    //bool compare_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc1, pcl::PointCloud<pcl::PointXYZ>::Ptr pc2, int Iteration);
    //Eigen::Matrix4d transformation_matrix_round(Eigen::Matrix4d transformation_matrix);
    bool data_exist();
    void show_pointcloud(int argc, char ** argv);
    void pointxyz2pointxyzi(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc2,pcl::PointCloud<pcl::PointXYZI> &temp);
    void get_label(sensor_msgs::PointCloud2ConstPtr m,pcl::PointCloud<pcl::PointXYZI> &temp);
    bool pointcloud_if_ready();
};
#endif //IMUDATA_LIDAR_POINTCLOUD_H
