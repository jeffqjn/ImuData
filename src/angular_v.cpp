#include "angular_v.h"
 int KdTree::splitAttribute_static=0;
Eigen::Vector3d rotationMatrixtoEulerAngle(Eigen::Matrix3d & matrix)
{
    Eigen::Vector3d euler_t;
    euler_t=matrix.eulerAngles(2,1,0);
    return euler_t;
}

//long curTime()
//{
//    std::chrono::milliseconds ms=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
//    return ms.count();
//}

void test()
{
//    Interval x(-0.05,0.05);
//    Interval y(-0.05,0.05);
//    Interval z(-0.015,0.015);
//    Interval rho=sqrt(pow(x,2)+pow(y,2)+pow(z,2));
//    Interval theta=acos(y/rho);
//    Interval phi=atan(y/x);
//    cout <<rho<<endl;
//    cout <<theta<<endl;
//    cout <<phi<<endl;
//
//    cout <<rho.diam()/2<<endl;
//    cout <<theta.diam()/2<<endl;
//    cout <<phi.diam()/2<<endl;
double summe;
    long start,end;
    start= curTime();

//#pragma omp  parallel for  reduction(+:summe) default(none) num_threads(8)
for(int i=0;i<727379967;i++)
{
    summe+=0.01;
}
    end= curTime();
cout<<end-start<<endl;
}

bool if_ground_truth(rosbag::MessageInstance const & m)
{
    return m.getTopic()=="/ground_truth";
}
bool if_tf_static(rosbag::MessageInstance const & m)
{
    return m.getTopic()=="/tf_static";
}
bool if_imu_data(rosbag::MessageInstance const & m)
{
    return m.getTopic()=="/imu/data";
}
bool if_velodyne_points(rosbag::MessageInstance const & m)
{
    return m.getTopic()=="/velodyne_points";
}

int main(int argc, char ** argv)
{
    //Instanziieren
    Measurement measurement;
    rosbag::Bag bag;
    Parameters parameters;
    LiDAR_PointCloud points(0.999998,0.000560097,0.014516);
    IMU imu;
    //TODO maybe can be deleted
    KdTree kd;
    Particle_Filter PF;

    //Loading Parameters from yaml
    load_from_config(parameters);

    //open rosbag
    bag.open(parameters.get_Bag_Path());

    //get topics
    vector<string> topics;
    parameters.get_topics(topics);

    vector<pair<Interval,Eigen::Matrix4d>> tf2imu;
    vector<geometry_msgs::PoseStamped> ground_truth;
    vector<pair<geometry_msgs::TransformStamped,geometry_msgs::TransformStamped>> tf_static;

    double rho, phi,theta;
    double horizontal_angle_uncertainty_uc,vertical_angle_uncertainty_uc;
    parameters.get_LiDAR_error_model_param(rho,phi,theta,horizontal_angle_uncertainty_uc,vertical_angle_uncertainty_uc);
    kd.set_uncertinty(rho,phi,theta,horizontal_angle_uncertainty_uc,vertical_angle_uncertainty_uc);

    bool static_data_aqcuired=false;
    double start_time,end_time;
    int index=0;
    bool get_two=false;

    IntervalMatrix rotation(3,3);
    IntervalVector state(3,Interval(0));
    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> est_pos;
    geometry_msgs::TransformStamped end;
    geometry_msgs::Vector3 from_beginning_to_end_translation;

    rosbag::View view(bag,rosbag::TopicQuery(topics));
    parameters.set_bag_start_end_time(view.getBeginTime().toSec(),view.getEndTime().toSec());

    start_time=parameters.get_START_COMPUTE_TIME();
    end_time=parameters.get_END_COMPUTE_TIME();

    BOOST_FOREACH( rosbag::MessageInstance const  m, view)
    {
            if (parameters.if_in_time_period(m))
            {
                if (if_ground_truth(m))
                {
                    measurement.add_ground_truth(m.instantiate<geometry_msgs::PoseStamped>(),parameters);
                }
                else if (if_tf_static(m) && !static_data_aqcuired)
                {
                    measurement.add_tf_static(m,static_data_aqcuired);
                }
                else if (if_imu_data(m))
                {
                    imu.add_vel_measurement(m.instantiate<sensor_msgs::Imu>(),parameters);
                    if(get_two)
                    {
                        if(points.data_exist())
                        {
                            //TODO change the function and variable name
                            vector<pair<Eigen::Vector3d, Eigen::Vector3d>> temp = PF.particle_filter_set_up(
                                            parameters, imu, kd, points,argc,argv);
                            est_pos.emplace_back(make_pair(temp[0].first, temp[0].second));
                            get_two = false;
                            index = 1;
                        }
                    }
                }
                else if (if_velodyne_points(m))
                {
                    points.add_pointcloud(m.instantiate<sensor_msgs::PointCloud2>(),parameters,kd,argc,argv);
                    if(++index%2==0)
                    {
                        get_two= true;
                    }
                }
            }
    }

    //mms->camera
    Eigen::Matrix4d transform1=Eigen::Matrix4d::Identity();
    //camera->imu
    Eigen::Matrix4d transform2=Eigen::Matrix4d::Identity();

    transform1=measurement.tf_mms_cam();
    //transform2=measurement.tf_cam_imu();
    transform2=measurement.tf_cam_velodyne();

    Eigen::Matrix4d relativ_transformation_imu=Eigen::Matrix4d::Identity();
    measurement.transform_gt_imu(transform1,transform2,parameters);
    relativ_transformation_imu=measurement.calculate_relative_transformation_imu(parameters);
    Eigen::Matrix4d relativ_transformation_mms=Eigen::Matrix4d::Identity();
    cout<<relativ_transformation_imu<<endl;
    //relativ_transformation_mms=transform1.inverse()*relativ_transformation_imu;
    //relativ_transformation_mms=transform2.inverse()*relativ_transformation_mms;
    Eigen::Matrix3d rot=relativ_transformation_imu.block(0,0,3,3);
    Eigen::Vector3d e=rotationMatrixtoEulerAngle(rot);
    cout<<e<<endl;
    cout<<relativ_transformation_imu<<endl;

    //Testing!!!
    bool erg;
    erg=rotation[0][0].contains(relativ_transformation_mms(0,0))&rotation[0][1].contains(relativ_transformation_mms(0,1))&rotation[0][2].contains(relativ_transformation_mms(0,2))&rotation[1][0].contains(relativ_transformation_mms(1,0))&rotation[1][1].contains(relativ_transformation_mms(1,1))&rotation[1][2].contains(relativ_transformation_mms(1,2))&rotation[2][0].contains(relativ_transformation_mms(2,0))&rotation[2][1].contains(relativ_transformation_mms(2,1))&rotation[2][2].contains(relativ_transformation_mms(2,2));
    cout<<erg<<endl;
    bag.close();
    return 0;
}
//
// Created by noah on 11.01.21.
//
//1557924740.177743
//1557924740.2766137