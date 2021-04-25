#include "angular_v.h"

//load parameters from yaml

void load_from_config(Parameters & parameters)
{
    YAML::Node config = YAML::LoadFile(YAML_PATH);

    //load paramters in class Parameters
    //Gyro-Scale-Factor
    parameters.set_GYRO_SCALE_FACTOR(config["Gyro-scale-factor"].as<double>());
    //Gyro-Bias
    parameters.set_GYRO_BIAS(config["Gyro-bias"].as<double>());
    //Gyro-Walking-Bias
    parameters.set_Gyro_WALKING_BIAS(config["Gyro-walking-bias"].as<double>());
    //Start-Compute-Time
    parameters.set_START_COMPUTE_TIME(config["start_compute_time"].as<double>());
    //End-Compute-Time
    parameters.set_END_COMPUTE_TIME(config["end_compute_time"].as<double>());
    //Bag_Path
    parameters.set_Bag_Path(config["Bag_Path"].as<string>());
    //Iterations
    parameters.set_Iterations(config["ICP_Iterations"].as<int>());
    //Acc-Scale-Factor
    parameters.set_ACC_SCALE_FACTOR(config["Acc-scale-factor"].as<double>());
    //Acc-Bias
    parameters.set_ACC_BIAS(config["Acc-bias"].as<double>());
    //Acc-Walking-Bias
    parameters.set_ACC_WALKING_BIAS(config["Acc-walking-bias"].as<double>());
    //LiDAR-error-model set up
    parameters.set_LiDAR_error_model_param(config["rho"].as<double>(),config["phi"].as<double>(),config["theta"].as<double>(),config["horizontal_angle_uncertainty"].as<double>(),config["vertical_angle_uncertainty"].as<double>());
    //NED Uncertainty
    parameters.set_NED_Velocity_uncertainty(config["NED_Velocity_NC_N"].as<double>(),config["NED_Velocity_NC_E"].as<double>(),config["NED_Velocity_NC_D"].as<double>());
    //LLH Uncertainty
    parameters.set_LLH_uncertainty(config["LLH_NC_L1"].as<double>(),config["LLH_NC_L2"].as<double>(),config["LLH_NC_H"].as<double>());
    //Topics
    parameters.load_topics(config["topics"].as<vector<string>>());
    //PF Threshold
    parameters.set_PF_threshold(config["particle_filter_threshold"].as<double>());
    //calculate interval
    parameters.set_calcluate_interval(config["particle_filter_calculate_interval"].as<int>());
}





double double_round(double dVal, int iPlaces) {
    double dRetval;
    double dMod = 0.0000001;
    if (dVal<0.0) dMod=-0.0000001;
    dRetval=dVal;
    dRetval+=(5.0/pow(10.0,iPlaces+1.0));
    dRetval*=pow(10.0,iPlaces);
    dRetval=floor(dRetval+dMod);
    dRetval/=pow(10.0,iPlaces);
    return(dRetval);
//const double multiplier = std::pow(10.0,iPlaces);
//return std::ceil(dVal*multiplier) / multiplier;
}


//
// Created by noah on 23.01.21.
//

