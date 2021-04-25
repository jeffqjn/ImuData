//
// Created by noah on 27.02.21.
//
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <codac/codac.h>
#include "Parameters.h"
#include "LiDAR_PointCloud.h"
#include "Measurement.h"
using namespace std;

#ifndef IMUDATA_IMU_H
#define IMUDATA_IMU_H
class IMU
{
public:
    std::vector<std::pair<ibex::Interval,ibex::IntervalVector>> vel_data;
    std::vector<std::pair<ibex::Interval,ibex::IntervalVector>> acc_data;
    std::vector<std::pair<ibex::Interval,Eigen::Vector3d>> acc_actual_data;
    std::vector<std::pair<ibex::Interval,Eigen::Vector3d>> vel_actual_data;
    bool angular_velocity_ready=false;

public:
    void add_vel_measurement(sensor_msgs::ImuConstPtr imupointer,Parameters & parameters);
    ibex::IntervalMatrix vel2rotatation(long double start_compute_time, long double  end_compute_time);
    Eigen::Vector3d acc2pose(double start_compute_time, double  end_compute_time, Eigen::Vector3d start_vel);
    Eigen::Vector3d set_calibration();
    IntervalVector set_calibration_interval();
    void calculate_ground_truth(Parameters parameters,LiDAR_PointCloud pointCloud,Eigen::Matrix4d transform1,Eigen::Matrix4d transform2);
    codac::TubeVector acc2pose_tube(double start_compute_time, double  end_compute_time,IntervalVector calib,IntervalVector start_vel);
    static void toEulerAngle(const Eigen::Quaterniond & q, double & roll, double & pitch , double & yaw);
    ibex::IntervalMatrix calculate_rodrigues_rotation(ibex::IntervalVector angular_vel, double delta_t);
    //Debug
    void debug(Measurement m);
    bool if_contain(codac::TubeVector x, double start_compute_time , double end_compute_time, Measurement m);
    bool angular_velocity_if_ready();
};
#endif //IMUDATA_IMU_H
