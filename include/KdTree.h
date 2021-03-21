//
// Created by noah on 27.02.21.
//
#include <iostream>
#include <ibex.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/time.h>
#include <pcl/PCLHeader.h>
#include <pcl/registration/icp.h>
using namespace std;
using namespace ibex;
#ifndef IMUDATA_KDTREE_H
#define IMUDATA_KDTREE_H


class KdTree {
public:



    struct KdNode
    {
        KdNode * parent;
        KdNode * leftchild;
        KdNode * rightchild;
        IntervalVector point;
        int axis;
        int splitAttribute;
        KdNode(IntervalVector data, int ax=0)
        {
            point=data;
            axis=ax;
            parent=NULL;
            leftchild=NULL;
            rightchild=NULL;
        }
    };

    vector<pair<ros::Time,pcl::KdTreeFLANN<pcl::PointXYZ>::ConstPtr>> Forest;
    KdNode * root;
    double rho_uc;
    double phi_uc;
    double theta_uc;
    double horizontal_angle_uncertainty_uc;
    double vertical_angle_uncertainty_uc;
    int splitAttribute;
    static int splitAttribute_static;

    KdNode* create_kd_tree(pcl::PointCloud<pcl::PointXYZ> pc);
    void add_Tree2Forest(pcl::KdTreeFLANN<pcl::PointXYZ>::ConstPtr tree, ros::Time time);
    double getVariance(pcl::PointCloud<pcl::PointXYZ> pc, int index);
    int getSplitAttribute(const pcl::PointCloud<pcl::PointXYZ> pc);
    double getSplitValue(pcl::PointCloud<pcl::PointXYZ> pc);
    static bool cmp(const pcl::PointXYZ & value1, const pcl::PointXYZ & value2);
    void set_uncertinty(double rho_uc, double phi_uc, double theta_uc, double horizontal_angle_uncertainty_uc, double vertical_angle_uncertainty_uc);
    IntervalVector calculate_point_interval(pcl::PointXYZ point);
    static void set_static_value(int splitattribute);
    static int get_splitattribute();
};


#endif //IMUDATA_KDTREE_H
