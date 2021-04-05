#include "LiDAR_PointCloud.h"
LiDAR_PointCloud::LiDAR_PointCloud(double leading_diagonal_threshold, double sub_diagonal_threshold, double tranlation_threshold){
    //build threshold matrix
    threshold_matrix(0,0)=leading_diagonal_threshold;
    threshold_matrix(1,1)=leading_diagonal_threshold;
    threshold_matrix(2,2)=leading_diagonal_threshold;
    threshold_matrix(0,1)=sub_diagonal_threshold;
    threshold_matrix(0,2)=sub_diagonal_threshold;
    threshold_matrix(1,0)=sub_diagonal_threshold;
    threshold_matrix(1,2)=sub_diagonal_threshold;
    threshold_matrix(2,0)=sub_diagonal_threshold;
    threshold_matrix(2,1)=sub_diagonal_threshold;
    threshold_matrix(0,3)=tranlation_threshold;
    threshold_matrix(1,3)=tranlation_threshold;
    threshold_matrix(2,3)=tranlation_threshold;
    threshold_matrix(3,3)=1;
    threshold_matrix(3,0)=0;
    threshold_matrix(3,1)=0;
    threshold_matrix(3,2)=0;
    //build Template
    Template=Eigen::Matrix4d::Identity ();
}
pcl::PointCloud<pcl::PointXYZ>::Ptr LiDAR_PointCloud::cloud_convert(const boost::shared_ptr<const sensor_msgs::PointCloud2> & input) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    return temp_cloud;
}

void LiDAR_PointCloud::show_pointcloud(int argc, char ** argv)
{
    ros::init(argc,argv,"Pointcloud_show");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub=nh.advertise<sensor_msgs::PointCloud2>("pcl_output",1);
    sensor_msgs::PointCloud2 output1;

    pcl::toROSMsg(ground,output1);
    output1.header.frame_id="mms";
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        pcl_pub.publish(output1);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void LiDAR_PointCloud::add_pointcloud(sensor_msgs::PointCloud2ConstPtr m, Parameters parameters,KdTree &kdTree ,int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZI> temp;

    if(m->header.stamp.toSec()>parameters.get_START_COMPUTE_TIME() &&m->header.stamp.toSec()<parameters.get_END_COMPUTE_TIME())
    {
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc2;
        pc2 = cloud_convert(m);
        pointclouds.emplace_back(make_pair(m->header.stamp.toSec(),*pc2));
    }
}
void LiDAR_PointCloud::pointxyz2pointxyzi(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc2,pcl::PointCloud<pcl::PointXYZI> &temp)
{
    for(auto item:*pc2)
    {
        pcl::PointXYZI temp1;
        temp1.x=item.x;
        temp1.y=item.y;
        temp1.z=item.z;
        temp.points.emplace_back(temp1);
    }
}

//Eigen::Matrix4d LiDAR_PointCloud::transformation_matrix_round(Eigen::Matrix4d transformation_matrix)
//{
//    Eigen::Matrix4d temp=Eigen::Matrix4d::Identity();
//    temp=transformation_matrix.array().abs().matrix();
//    if(temp(0,0)>threshold_matrix(0,0) && temp(1,1)>threshold_matrix(1,1) && temp(2,2)>threshold_matrix(2,2)&& temp(0,1)<threshold_matrix(0,1) && temp(0,2)<threshold_matrix(0,2) && temp(1,0)<threshold_matrix(1,0) && temp(1,2)<threshold_matrix(1,2) && temp(2,0)<threshold_matrix(2,0) &&temp(2,1)<threshold_matrix(2,1) && temp(0,3)<threshold_matrix(0,3) && temp(1,3)<threshold_matrix(1,3) && temp(2,3)<threshold_matrix(2,3))
//    {
//        return Template;
//    }
//    else
//    {
//        return transformation_matrix;
//    }
//}

//bool LiDAR_PointCloud::compare_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc1, pcl::PointCloud<pcl::PointXYZ>::Ptr pc2, int Iteration) {
////The ICP algorithm
//pcl::console::TicToc time;
//pcl::PointCloud<pcl::PointXYZ> temp1;
//pcl::PointCloud<pcl::PointXYZ> temp2;
//Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
//Eigen::Matrix4d round_transformation_matrix = Eigen::Matrix4d::Identity ();
//temp1=*pc1;
//temp2=*pc2;
////time.tic();
//pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
//icp.setMaximumIterations(Iteration);
//icp.setInputSource(temp1.makeShared());
//icp.setInputTarget(temp2.makeShared());
//icp.align(temp1);
//icp.setMaximumIterations(1);
////cout << "Applied " << 5 << " ICP iteration(s) in " << time.toc () << " ms" <<endl;
//
//if (icp.hasConverged())
//{
//    transformation_matrix = icp.getFinalTransformation ().cast<double>();
//    //set precision
//    round_transformation_matrix=transformation_matrix_round(transformation_matrix);
//    if(round_transformation_matrix.isIdentity())
//    {
//        return true;
//    }
//    return false;
//
//}
//else
//{
//    PCL_ERROR ("\nICP has not converged.\n");
//    return false;
//}
//}
bool LiDAR_PointCloud::data_exist()
{
    return pointclouds.size()==2;
}
void LiDAR_PointCloud::get_label(sensor_msgs::PointCloud2ConstPtr m,pcl::PointCloud<pcl::PointXYZI> &temp)
{

    floorSegmentParams params;
    std::vector<int> label;
    params.visualize         = false;  // show with true
    params.r_min_square      = 0.1 * 0.1;
    params.r_max_square      = 100 * 100;
    params.n_bins            = 100;
    params.n_segments        = 180;
    params.max_dist_to_line  = 0.15;
    params.max_slope         = 1;
    params.max_error_square  = 0.01;
    params.long_threshold    = 2.0;
    params.max_long_height   = 0.1;
    params.max_start_height  = 0.2;
    params.sensor_height     = 1.73;
    params.line_search_angle = 0.2;
    params.n_threads         = 4;
    floorSegmentation segmenter(params);
    segmenter.segment(temp, &label);
    labels.emplace_back(make_pair(m->header.stamp.toSec(),label));

}
//
// Created by noah on 25.01.21.
//

