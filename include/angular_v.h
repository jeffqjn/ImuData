#ifndef ANGULAR_V_ANGULAR_V_H
#define ANGULAR_V_ANGULAR_V_H

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <ibex.h>
#include <codac/codac.h>
#include <codac/codac_VIBesFigTube.h>
#include <codac/codac_VIBesFig.h>
#include <vibes.h>
#include <algorithm>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
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
#include <yaml.h>
#include <time.h>
#include "Parameters.h"
#include "LiDAR_PointCloud.h"
#include "Measurement.h"
#include "IMU.h"
#include "KdTree.h"
#include "Particle_Filter.h"

#include "std_msgs/String.h"
#include <omp.h>
#include <chrono>
//#include "EF.h"
#define YAML_PATH "/home/jeffqjn/CLionProjects/ImuData/include/Configuration.yml"

using namespace std;
using namespace ibex;

//Utility
static void toEulerAngle(const Eigen::Quaterniond & q, double & roll, double & pitch , double & yaw);
//IntervalVector IntervalrotationMatrixtoEulerAngle(IntervalMatrix & matrix);
double double_round(double dVal, int iPlaces);
void load_from_config(Parameters & parameters);









#endif //ANGULAR_V_ANGULAR_V_H
