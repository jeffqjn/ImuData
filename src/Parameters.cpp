#include "Parameters.h"
using namespace  std;
void Parameters::set_GYRO_SCALE_FACTOR(double value)
{
    this->GYRO_SCALE_FACTOR=value;
}
void Parameters::set_GYRO_BIAS(double value)
{
    this->GYRO_BIAS=value;
}
void Parameters::set_Gyro_WALKING_BIAS(double value)
{
    this->Gyro_WALKING_BIAS=value;
}
void Parameters::set_ACC_SCALE_FACTOR(double value)
{
    this->ACC_SCALE_FACTOR=value;
}
void Parameters::set_ACC_BIAS(double value)
{
    this->ACC_BIAS=value;
}
void Parameters::set_ACC_WALKING_BIAS(double value)
{
    this->ACC_WALKING_BIAS=value;
}
void Parameters::set_START_COMPUTE_TIME( double value)
{
    this->START_COMPUTE_TIME=value;
}
void Parameters::set_END_COMPUTE_TIME( double value)
{
    this->END_COMPUTE_TIME=value;
}
void Parameters::set_Bag_Path(string path) {
    this->BAG_PATH=path;
}
void Parameters::set_Iterations(int times) {
    this->Iterations=times;
}
void Parameters::set_calcluate_interval(int interval)
{
    this->calculate_interval=interval;
}
double Parameters::get_GYRO_SCALE_FACTOR()
{
    return Parameters::GYRO_SCALE_FACTOR;
}
double Parameters::get_GYRO_BIAS()
{
    return Parameters::GYRO_BIAS;
}
double Parameters::get_Gyro_WALKING_BIAS()
{
    return Parameters::Gyro_WALKING_BIAS;
}
 double Parameters::get_START_COMPUTE_TIME()
{
    return Parameters::START_COMPUTE_TIME;
}
 double Parameters::get_END_COMPUTE_TIME()
{
    return Parameters::END_COMPUTE_TIME;
}
string Parameters::get_Bag_Path() {
    return Parameters::BAG_PATH;
}
int Parameters::get_Iterations() {
    return Parameters::Iterations;
}
double Parameters::get_ACC_SCALE_FACTOR()
{
    return Parameters::ACC_SCALE_FACTOR;
}
double Parameters::get_ACC_BIAS()
{
    return Parameters::ACC_BIAS;
}
double Parameters::get_ACC_WALKING_BIAS()
{
    return Parameters::ACC_WALKING_BIAS;
}
int Parameters::get_calculate_interval()
{
    return Parameters::calculate_interval;
}
void Parameters::set_LiDAR_error_model_param(double rho, double phi, double theta,double horizontal_angle_uncertainty, double vertical_angle_uncertainty)
{
    this->rho=rho;
    this->phi=phi;
    this->theta=theta;
    this->horizontal_angle_uncertainty=horizontal_angle_uncertainty;
    this->vertical_angle_uncertainty=vertical_angle_uncertainty;
}
void Parameters::get_LiDAR_error_model_param(double &rho, double &phi, double &theta,double &horizontal_angle_uncertainty, double &vertical_angle_uncertainty)
{
    rho=this->rho;
    phi=this->phi;
    theta=this->theta;
    horizontal_angle_uncertainty=this->horizontal_angle_uncertainty;
    vertical_angle_uncertainty=this->vertical_angle_uncertainty;
}
void Parameters::set_NED_Velocity_uncertainty(double N, double E, double D)
{
    this->NED_Velocity_N=N;
    this->NED_Velocity_E=E;
    this->NED_Velocity_D=D;
}

void Parameters::get_NED_Velocity_uncertainty(double &N, double &E, double &D)
{
    N=this->NED_Velocity_N;
    E=this->NED_Velocity_E;
    D=this->NED_Velocity_D;
}
void Parameters::set_LLH_uncertainty(double latitude, double longitude, double height)
{
    this->Latitude_uc=latitude;
    this->Longitude_uc=longitude;
    this->Height_uc=height;

}
void Parameters::get_LLH_uncertainty(double & latitude, double & longitude, double & height)
{
    latitude=this->Latitude_uc;
    longitude=this->Longitude_uc;
    height=this->Height_uc;
}



void Parameters::set_bag_start_end_time(double s_time, double e_time)
{
  this->bag_start_time=s_time;
  this->bag_end_time=e_time;
}
void Parameters::get_bag_start_end_time(double &s_time, double &e_time)
{
    s_time=this->bag_start_time;
    e_time=this->bag_end_time;
}
void Parameters::load_topics(vector<string> topics)
{
    this->topics=topics;
}
void Parameters::get_topics(vector<string> &t)
{
    for( auto item: topics)
    {
        t.emplace_back(item);
    }
}
bool Parameters::if_in_time_period(rosbag::MessageInstance const  m)
{
   return m.getTime().toSec()>(get_START_COMPUTE_TIME()-1) && m.getTime().toSec()<(get_END_COMPUTE_TIME()+1);
}
void Parameters::set_PF_threshold(double threshold)
{
    this->PF_threshold=threshold;
}
double Parameters::get_PF_threshold()
{
    return this->PF_threshold;
}
//
// Created by noah on 23.01.21.
//

