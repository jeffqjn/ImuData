//
// Created by noah on 18.01.21.
//
#include <Measurement.h>

void Measurement::add_tf_static(const rosbag::MessageInstance m, bool & tf_data_aquired) {
    tf::tfMessageConstPtr tf_static1=m.instantiate<tf::tfMessage>();
    // 0: mms -> camera_right
    // 1: camera_right -> velodyne
    // 2: camera_right -> imu
    tf_static.emplace_back(tf_static1->transforms[0]);
    tf_static.emplace_back(tf_static1->transforms[1]);
    tf_static.emplace_back(tf_static1->transforms[2]);
    tf_data_aquired=true;
}
Eigen::Matrix4d Measurement::tf_mms_cam() {
    Eigen::Matrix4d transform1=Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rotations_temp= Eigen::Matrix3d::Identity();
    Eigen::Quaterniond value_temp;
    value_temp.x()=tf_static[0].transform.rotation.x;
    value_temp.y()=tf_static[0].transform.rotation.y;
    value_temp.z()=tf_static[0].transform.rotation.z;
    value_temp.w()=tf_static[0].transform.rotation.w;
    rotations_temp=value_temp.matrix();
    transform1(0,3)=tf_static[0].transform.translation.x;
    transform1(1,3)=tf_static[0].transform.translation.y;
    transform1(2,3)=tf_static[0].transform.translation.z;
    transform1(3,3)=1;
    transform1.block(0,0,3,3)=rotations_temp;
    return transform1;
}
Eigen::Matrix4d  Measurement::tf_cam_velodyne()
{
    Eigen::Matrix4d transform2=Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rotations_temp= Eigen::Matrix3d::Identity();
    Eigen::Quaterniond value_temp;
    value_temp.x()=tf_static[1].transform.rotation.x;
    value_temp.y()=tf_static[1].transform.rotation.y;
    value_temp.z()=tf_static[1].transform.rotation.z;
    value_temp.w()=tf_static[1].transform.rotation.w;
    rotations_temp=value_temp.matrix();
    transform2(0,3)=tf_static[1].transform.translation.x;
    transform2(1,3)=tf_static[1].transform.translation.y;
    transform2(2,3)=tf_static[1].transform.translation.z;
    transform2(3,3)=1;
    transform2.block(0,0,3,3)=rotations_temp;
    return transform2;
}
Eigen::Matrix4d Measurement::tf_cam_imu() {
    Eigen::Matrix4d transform2=Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rotations_temp;
    Eigen::Quaterniond value_temp;
    value_temp.x()=tf_static[2].transform.rotation.x;
    value_temp.y()=tf_static[2].transform.rotation.y;
    value_temp.z()=tf_static[2].transform.rotation.z;
    value_temp.w()=tf_static[2].transform.rotation.w;
    rotations_temp=value_temp.matrix();
    transform2(0,3)=tf_static[2].transform.translation.x;
    transform2(1,3)=tf_static[2].transform.translation.y;
    transform2(2,3)=tf_static[2].transform.translation.z;
    transform2(3,3)=1;
    transform2.block(0,0,3,3)=rotations_temp;
    return transform2;

}
void Measurement::add_ground_truth(const geometry_msgs::PoseStampedConstPtr m, Parameters parameters) {
    double start_time= parameters.get_START_COMPUTE_TIME();
    double end_time= parameters.get_END_COMPUTE_TIME();
    bool contain=false;

    if(ground_truth.empty())
    {
        ground_truth.emplace_back(make_pair(Interval(m->header.stamp.toSec()),*m));
    }
    else
    {
        ground_truth.emplace_back(make_pair(Interval(ground_truth.back().first.ub(),m->header.stamp.toSec()),*m));
    }
//    if(m->header.stamp.toSec()-start_time>0 && m->header.stamp.toSec()-start_time<0.005 && m->header.stamp.toSec()<end_time)
//    {
//        ground_truth.emplace_back(make_pair(Interval(ground_truth.back().first.ub(),m->header.stamp.toSec()),*m));
//    }
//    else if (start_time-m->header.stamp.toSec()>0 && start_time-m->header.stamp.toSec()<0.005)
//    {
//        ground_truth.emplace_back(make_pair(Interval(m->header.stamp.toSec()-0.005,m->header.stamp.toSec()),*m));
//    }
//    else if (m->header.stamp.toSec()-start_time>0.005 &&m->header.stamp.toSec()<end_time)
//    {
//        ground_truth.emplace_back(make_pair(Interval(ground_truth.back().first.ub(),m->header.stamp.toSec()),*m));
//    }
//
//    else if (m->header.stamp.toSec()-end_time<0.005 && m->header.stamp.toSec()-end_time>0)
//    {
//        ground_truth.emplace_back(make_pair(Interval(ground_truth.back().first.ub(),m->header.stamp.toSec()),*m));
//    }
//    if(ground_truth.empty())
//    {
//
//        ground_truth.emplace_back(make_pair(Interval(m->header.stamp.toSec()),*m));
//    }
//    else
//    {
//        ground_truth.emplace_back(make_pair(Interval(ground_truth.back().first.ub(),m->header.stamp.toSec()),*m));
//    }
}
Eigen::Matrix4d Measurement::calculate_relative_transformation_imu(Parameters parameters) {
    Eigen::Matrix4d relativ_transformation=Eigen::Matrix4d::Identity();
    double width;
    double alpha;
    Eigen::Matrix3d rotation_begin=Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rotation_end=Eigen::Matrix3d::Identity();
    Eigen::Matrix4d transformation_begin=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transformation_end=Eigen::Matrix4d::Identity();

    Eigen::Vector3d translation_begin;
    Eigen::Vector3d translation_end;

    width = tf2imu[1].first.ub() - tf2imu[1].first.lb();
    alpha = (parameters.get_START_COMPUTE_TIME()-tf2imu[1].first.lb())/width;
    //need interpolation
    if (alpha != 1) {
        Eigen::Quaterniond outcome;
        Eigen::Vector3d translation_begin_index_current;
        Eigen::Vector3d translation_begin_index_last;
        rotation_begin=tf2imu[1].second.block(0,0,3,3);
        Eigen::Quaterniond value_current(rotation_begin);

        rotation_begin=Eigen::Matrix3d::Identity();
        rotation_begin=tf2imu[0].second.block(0,0,3,3);
        Eigen::Quaterniond value_last(rotation_begin);

        rotation_begin=Eigen::Matrix3d::Identity();

        translation_begin_index_current[0]=tf2imu[1].second(0,3);
        translation_begin_index_current[1]=tf2imu[1].second(1,3);
        translation_begin_index_current[2]=tf2imu[1].second(2,3);

        translation_begin_index_last[0]=tf2imu[0].second(0,3);
        translation_begin_index_last[1]=tf2imu[0].second(1,3);
        translation_begin_index_last[2]=tf2imu[0].second(2,3);
        translation_begin=translation_begin_index_last+alpha*(translation_begin_index_current-translation_begin_index_last);
        outcome = value_last.slerp(alpha, value_current);

        rotation_begin=Eigen::Matrix3d::Identity();
        rotation_begin=outcome.matrix();
        transformation_begin.block(0,0,3,3)=rotation_begin;
        transformation_begin(0,3)=translation_begin[0];
        transformation_begin(1,3)=translation_begin[1];
        transformation_begin(2,3)=translation_begin[2];
        transformation_begin(3,3)=1;
    } else {//otherwise don't need interpolation, extraction the translation and rotation of start_frame_index
        //rotation
        transformation_begin=tf2imu[1].second;
    }

    //interpolation to END_COMPUTE_TIME
    width = tf2imu[3].first.ub() - tf2imu[3].first.lb();
    alpha = (parameters.get_END_COMPUTE_TIME()-tf2imu[3].first.lb())/width;
    //need interpolation
    if (alpha != 1) {
        //rotation
        //slerp
        Eigen::Quaterniond outcome;
        Eigen::Vector3d translation_end_index_current;
        Eigen::Vector3d translation_end_index_last;


        rotation_end=tf2imu[3].second.block(0,0,3,3);
        Eigen::Quaterniond value_current(rotation_end);

        rotation_end=Eigen::Matrix3d::Identity();

        rotation_end=tf2imu[2].second.block(0,0,3,3);
        Eigen::Quaterniond value_last(rotation_end);

        rotation_end=Eigen::Matrix3d::Identity();

        translation_end_index_current[0]=tf2imu[3].second(0,3);
        translation_end_index_current[1]=tf2imu[3].second(1,3);
        translation_end_index_current[2]=tf2imu[3].second(2,3);

        translation_end_index_last[0]=tf2imu[2].second(0,3);
        translation_end_index_last[1]=tf2imu[2].second(1,3);
        translation_end_index_last[2]=tf2imu[2].second(2,3);

        translation_end=translation_end_index_last+alpha*(translation_end_index_current-translation_end_index_last);
        outcome = value_last.slerp(alpha, value_current);
        rotation_end=outcome.matrix();
        transformation_end.block(0,0,3,3)=rotation_end;
        transformation_end(0,3)=translation_end[0];
        transformation_end(1,3)=translation_end[1];
        transformation_end(2,3)=translation_end[2];
        transformation_end(3,3)=1;
        //?? the fourth row
    } else {//otherwise don't need interpolation, extraction the translation and rotation of start_frame_index
        //rotation
        transformation_end=tf2imu[3].second;
    }
    relativ_transformation=transformation_begin.inverse()*transformation_end;
    //relativ_transformation=transformation_end.inverse()*transformation_begin;
    //relativ_transformation=transformation_begin*transformation_end.inverse();
    return relativ_transformation;
}

void Measurement::transform_gt_imu(Eigen::Matrix4d tf_mms_cam, Eigen::Matrix4d tf_cam_imu, Parameters parameters ) {
//    int index=0;
//    int start_index=-1, end_index=-1;
//    double start_time=parameters.get_START_COMPUTE_TIME();
//    double end_time=parameters.get_END_COMPUTE_TIME();
//    for(int i=0;i<ground_truth.size();i++)
//    {
//
//        if(ground_truth[i].first.contains(start_time))
//        {
//            start_index=i;
//        }
//        else if(ground_truth[i].first.contains(end_time))
//        {
//            end_index=i;
//        }
//        if(start_index!=-1 && end_index!=-1)
//        {
//            break;
//        }
//    }
//    Eigen::Matrix4d transform_to_imu = Eigen::Matrix4d::Identity();
//    Eigen::Matrix3d rotations=Eigen::Matrix3d ::Identity();
//    Eigen::Quaterniond value;
//    Eigen::Matrix4d transform_to_imu_last= Eigen::Matrix4d::Identity();
//    Eigen::Matrix3d rotations_last=Eigen::Matrix3d ::Identity();
//    Eigen::Quaterniond value_last;
//    geometry_msgs::PoseStamped temp;
//    // get the transformationsmatrix of the frame that contains start_compute_time
//    value.x() = ground_truth[start_index].second.pose.orientation.x;
//    value.y() = ground_truth[start_index].second.pose.orientation.y;
//    value.z() = ground_truth[start_index].second.pose.orientation.z;
//    value.w() = ground_truth[start_index].second.pose.orientation.w;
//    rotations = value.matrix();
//    transform_to_imu.block(0, 0, 3, 3) = rotations;
//    transform_to_imu(0, 3) = ground_truth[start_index].second.pose.position.x;
//    transform_to_imu(1, 3) = ground_truth[start_index].second.pose.position.y;
//    transform_to_imu(2, 3) = ground_truth[start_index].second.pose.position.z;
//    transform_to_imu(3, 3) = 1;
//
//    transform_to_imu = transform_to_imu * tf_mms_cam;
//    transform_to_imu = transform_to_imu * tf_cam_imu;
//    // get the transformationsmatrix of the latest frame that contains start_compute_time
//    value_last.x() = ground_truth[start_index-1].second.pose.orientation.x;
//    value_last.y() = ground_truth[start_index-1].second.pose.orientation.y;
//    value_last.z() = ground_truth[start_index-1].second.pose.orientation.z;
//    value_last.w() = ground_truth[start_index-1].second.pose.orientation.w;
//    rotations_last = value_last.matrix();
//    transform_to_imu_last.block(0, 0, 3, 3) = rotations_last;
//    transform_to_imu_last(0, 3) = ground_truth[start_index-1].second.pose.position.x;
//    transform_to_imu_last(1, 3) = ground_truth[start_index-1].second.pose.position.y;
//    transform_to_imu_last(2, 3) = ground_truth[start_index-1].second.pose.position.z;
//    transform_to_imu_last(3, 3) = 1;
////    transform_to_imu_last=tf_mms_cam*transform_to_imu_last;
////    transform_to_imu_last=tf_cam_imu*transform_to_imu_last;
//    //transform_to_imu_last = transform_to_imu_last * tf_mms_cam;
//    //transform_to_imu_last = transform_to_imu_last * tf_cam_imu;
//    transform_to_imu_last = transform_to_imu_last * tf_mms_cam;
//    transform_to_imu_last = transform_to_imu_last * tf_cam_imu;
//    tf2imu.emplace_back(make_pair(ground_truth[start_index-1].first, transform_to_imu_last));
//    tf2imu.emplace_back(make_pair(ground_truth[start_index].first, transform_to_imu));
//
//    transform_to_imu=Eigen::Matrix4d::Identity();
//    transform_to_imu_last=Eigen::Matrix4d::Identity();
//    rotations=Eigen::Matrix3d::Identity();
//    rotations_last=Eigen::Matrix3d::Identity();
//
//    // get the transformationsmatrix of the frame that contains end_compute_time
//    value.x() = ground_truth[end_index].second.pose.orientation.x;
//    value.y() = ground_truth[end_index].second.pose.orientation.y;
//    value.z() = ground_truth[end_index].second.pose.orientation.z;
//    value.w() = ground_truth[end_index].second.pose.orientation.w;
//    rotations = value.matrix();
//    transform_to_imu.block(0, 0, 3, 3) = rotations;
//    transform_to_imu(0, 3) = ground_truth[end_index].second.pose.position.x;
//    transform_to_imu(1, 3) = ground_truth[end_index].second.pose.position.y;
//    transform_to_imu(2, 3) = ground_truth[end_index].second.pose.position.z;
//    transform_to_imu(3, 3) = 1;
//    //transform_to_imu = transform_to_imu * tf_mms_cam;
//    //transform_to_imu = transform_to_imu * tf_cam_imu;
////    transform_to_imu=tf_mms_cam*transform_to_imu;
////    transform_to_imu=tf_cam_imu*transform_to_imu;
//    transform_to_imu = transform_to_imu * tf_mms_cam;
//    transform_to_imu = transform_to_imu * tf_cam_imu;
//    // get the transformationsmatrix of the latest frame that contains end_compute_time
//    value_last.x() = ground_truth[end_index-1].second.pose.orientation.x;
//    value_last.y() = ground_truth[end_index-1].second.pose.orientation.y;
//    value_last.z() = ground_truth[end_index-1].second.pose.orientation.z;
//    value_last.w() = ground_truth[end_index-1].second.pose.orientation.w;
//    rotations_last = value_last.matrix();
//    transform_to_imu_last.block(0, 0, 3, 3) = rotations_last;
//    transform_to_imu_last(0, 3) = ground_truth[end_index-1].second.pose.position.x;
//    transform_to_imu_last(1, 3) = ground_truth[end_index-1].second.pose.position.y;
//    transform_to_imu_last(2, 3) = ground_truth[end_index-1].second.pose.position.z;
//    transform_to_imu_last(3, 3) = 1;
//    //transform_to_imu_last = transform_to_imu_last * tf_mms_cam;
//    //transform_to_imu_last = transform_to_imu_last * tf_cam_imu;
////    transform_to_imu_last=tf_mms_cam*transform_to_imu_last;
////    transform_to_imu_last=tf_cam_imu*transform_to_imu_last;
//    transform_to_imu_last = transform_to_imu_last * tf_mms_cam;
//    transform_to_imu_last = transform_to_imu_last * tf_cam_imu;
//    tf2imu.emplace_back(make_pair(ground_truth[end_index-1].first, transform_to_imu_last));
//    tf2imu.emplace_back(make_pair(ground_truth[end_index].first, transform_to_imu));
    int index=0;
    double start_time=parameters.get_START_COMPUTE_TIME();
    double end_time=parameters.get_END_COMPUTE_TIME();
    for (const auto& item :ground_truth) {
        if (item.first.contains(start_time) || item.first.contains(end_time)) {
            Eigen::Matrix4d transform_to_imu = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d rotations;
            Eigen::Quaterniond value;
            Eigen::Matrix4d transform_to_imu_last= Eigen::Matrix4d::Identity();
            Eigen::Matrix3d rotations_last;
            Eigen::Quaterniond value_last;
            geometry_msgs::PoseStamped temp;

            value.x() = item.second.pose.orientation.x;
            value.y() = item.second.pose.orientation.y;
            value.z() = item.second.pose.orientation.z;
            value.w() = item.second.pose.orientation.w;
            rotations = value.matrix();
            transform_to_imu.block(0, 0, 3, 3) = rotations;
            transform_to_imu(0, 3) = item.second.pose.position.x;
            transform_to_imu(1, 3) = item.second.pose.position.y;
            transform_to_imu(2, 3) = item.second.pose.position.z;
            transform_to_imu(3, 3) = 1;
            transform_to_imu = transform_to_imu * tf_mms_cam;
            transform_to_imu = transform_to_imu * tf_cam_imu;

            value_last.x() = ground_truth[index-1].second.pose.orientation.x;
            value_last.y() = ground_truth[index-1].second.pose.orientation.y;
            value_last.z() = ground_truth[index-1].second.pose.orientation.z;
            value_last.w() = ground_truth[index-1].second.pose.orientation.w;
            rotations_last = value.matrix();
            transform_to_imu_last.block(0, 0, 3, 3) = rotations_last;
            transform_to_imu_last(0, 3) = ground_truth[index-1].second.pose.position.x;
            transform_to_imu_last(1, 3) = ground_truth[index-1].second.pose.position.y;
            transform_to_imu_last(2, 3) = ground_truth[index-1].second.pose.position.z;
            transform_to_imu_last(3, 3) = 1;
            transform_to_imu_last = transform_to_imu_last * tf_mms_cam;
            transform_to_imu_last = transform_to_imu_last * tf_cam_imu;
            tf2imu.emplace_back(make_pair(ground_truth[index-1].first, transform_to_imu_last));
            tf2imu.emplace_back(make_pair(item.first, transform_to_imu));
        }
        index++;
    }
}
