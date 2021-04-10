//
// Created by noah on 27.02.21.
//
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <ibex.h>
#include "Parameters.h"

using namespace std;
using namespace ibex;
#ifndef IMUDATA_MEASUREMENT_H
#define IMUDATA_MEASUREMENT_H
class Measurement
{
public:
    vector<pair<Interval,Eigen::Matrix4d>> tf2imu;
    vector<pair<Interval,geometry_msgs::PoseStamped>> ground_truth;
    vector<geometry_msgs::TransformStamped> tf_static;
public:
    void add_ground_truth (geometry_msgs::PoseStampedConstPtr m,Parameters parameters);
    void add_tf_static (rosbag::MessageInstance  m, bool & tf_data_aquired);

    Eigen::Matrix4d tf_mms_cam();
    Eigen::Matrix4d tf_cam_imu();
    Eigen::Matrix4d tf_cam_velodyne();
    void transform_gt_imu(Eigen::Matrix4d tf_mms_cam, Eigen::Matrix4d tf_cam_imu, double start_time, double end_time);
    Eigen::Matrix4d calculate_relative_transformation_imu(double start_time, double end_time);
};

#endif //IMUDATA_MEASUREMENT_H
