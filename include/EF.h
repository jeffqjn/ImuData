//
// Created by noah on 01.03.21.
//
#include <ibex.h>
#include <tubex/tubex.h>
#include <tubex/tubex_VIBesFigTube.h>
#include <tubex/tubex_VIBesFig.h>
#include "Parameters.h"
#include "Measurement.h"
#include <geometry_msgs/Vector3Stamped.h>

using namespace  std;
using namespace ibex;
#ifndef IMUDATA_EF_H
#define IMUDATA_EF_H
class EF
{
public:
    std::vector<pair<Interval,IntervalVector>> velocity_NED;
    std::vector<pair<Interval,Eigen::Quaterniond>> quaternion;
    std::vector<pair<Interval,IntervalVector>> velocity;
    std::vector<pair<Interval,IntervalVector>> LLH_Position;
    std::vector<pair<Interval,IntervalVector>> Body_Position;

public:
    void add_velocity(const geometry_msgs::Vector3StampedConstPtr& v,Parameters parameters);
    void add_LLH(const geometry_msgs::Vector3StampedConstPtr& v,Parameters parameters);
    void add_quaternion(const geometry_msgs::QuaternionStampedConstPtr& q,Parameters parameters);
    void ned_vel_transform(Parameters parameter);
    IntervalMatrix calculate_transform_matrix(Eigen::Quaterniond q);
    IntervalVector state_estimation(Parameters parameters,Measurement m);
    IntervalMatrix calculate_inverse_matrix(IntervalMatrix ori);
    void draw(tubex::TubeVector x, Measurement measurement);
};
#endif //IMUDATA_EF_H
