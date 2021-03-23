//
// Created by noah on 27.02.21.
//
#include <iostream>
#include <vector>
#include <yaml.h>
#include <rosbag/bag.h>

using namespace std;
#ifndef IMUDATA_PARAMETERS_H
#define IMUDATA_PARAMETERS_H
class Parameters
{
private:
    double GYRO_SCALE_FACTOR;
    double GYRO_BIAS;
    double Gyro_WALKING_BIAS;
    double ACC_SCALE_FACTOR;
    double ACC_BIAS;
    double ACC_WALKING_BIAS;
    double START_COMPUTE_TIME;
    double END_COMPUTE_TIME;
    string BAG_PATH;
    int Iterations;
    double rho;
    double phi;
    double theta;
    double horizontal_angle_uncertainty;
    double vertical_angle_uncertainty;
    double NED_Velocity_N;
    double NED_Velocity_E;
    double NED_Velocity_D;
    double Latitude_uc;
    double Longitude_uc;
    double Height_uc;
    double bag_start_time;
    double bag_end_time;
    vector<string> topics;
    double PF_threshold;
public:
    void set_GYRO_SCALE_FACTOR(double value);
    void set_GYRO_BIAS(double value);
    void set_Gyro_WALKING_BIAS(double value);

    void set_ACC_SCALE_FACTOR(double value);
    void set_ACC_BIAS(double value);
    void set_ACC_WALKING_BIAS(double value);

    void set_START_COMPUTE_TIME(double value);
    void set_END_COMPUTE_TIME(double value);
    void set_Bag_Path(string path);
    void set_Iterations(int times);
    void set_LiDAR_error_model_param(double rho, double phi, double theta,double horizontal_angle_uncertainty, double vertical_angle_uncertainty);
    void set_NED_Velocity_uncertainty(double N, double E, double D);
    void set_LLH_uncertainty(double latitude, double longitude, double height);
    void set_bag_start_end_time(double s_time, double e_time);
    double get_GYRO_SCALE_FACTOR();
    double get_GYRO_BIAS();
    double get_Gyro_WALKING_BIAS();

    double get_ACC_SCALE_FACTOR();
    double get_ACC_BIAS();
    double get_ACC_WALKING_BIAS();

    double get_START_COMPUTE_TIME();
    double get_END_COMPUTE_TIME();
    string get_Bag_Path();
    int get_Iterations();
    double get_PF_threshold();
    void get_LiDAR_error_model_param(double &rho, double &phi, double &theta, double &horizontal_angle_uncertainty, double &vertical_angle_uncertainty );
    void get_NED_Velocity_uncertainty(double &N, double &E, double &D);
    void get_LLH_uncertainty(double & latitude, double & longitude, double & height);
    void get_bag_start_end_time(double &s_time, double &e_time);
    void load_topics(vector<string> topics);
    void get_topics(vector<string> &t);
    void set_PF_threshold(double threshold);
    bool if_in_time_period(rosbag::MessageInstance const  m);
};
#endif //IMUDATA_PARAMETERS_H
