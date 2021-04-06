#include "Particle_Filter.h"
void Particle_Filter::add_marker_to_array(IntervalVector point,
                         visualization_msgs::MarkerArray &marker_array, double r, double g,
                         double b, double a){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = marker_array.markers.size();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point[0].mid();
    marker.pose.position.y = point[1].mid();
    if(point.size()>2){
        marker.pose.position.z = point[2].mid();
        marker.scale.z = point[2].diam();
    } else {
        marker.pose.position.z = 0.0;
        marker.scale.z = 0.0;
    }
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = point[0].diam();
    marker.scale.y = point[1].diam();
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker_array.markers.push_back(marker);
}

void Particle_Filter::show_boxes(int argc, char** argv)
{
    ros::init(argc,argv,"boxes_show");
    ros::NodeHandle nh;
    visualization_msgs::MarkerArray  temp;
    int count=1000;
    for(int i=0;i<marker_array.markers.size();i++)
    {
        if(count<0)
        {
            break;
        }
        temp.markers.emplace_back(marker_array.markers[i]);
        temp.markers.emplace_back(marker_array.markers[i+flag1]);
        temp.markers.emplace_back(marker_array.markers[i+flag2]);
        count--;
    }

    ros::Publisher boxes_pub=nh.advertise<visualization_msgs::MarkerArray>("boxes_output",0);
    ros::Rate loop_rate(0.1);
    while(ros::ok())
    {
        boxes_pub.publish(temp);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
long curTime()
{
    std::chrono::milliseconds ms=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    return ms.count();
}

void Particle_Filter::build_LiDAR_Interval(Parameters &parameters,LiDAR_PointCloud &pointcloud)
{
    double rho_uncertinty;
    double phi_uncertinty;
    double theta_uncertinty;
    double horizontal_angle_uncertainty;
    double vertical_angle_uncertainty;
    pcl::PointCloud<pcl::PointXYZ> *pc;

    double x_coordinate;
    double y_coordinate;
    double z_coordinate;
    double rho;
    double phi;
    double theta;
    Interval interval_rho;
    Interval interval_phi;
    Interval interval_theta;
    IntervalVector coordinate(3,Interval(0));
    IntervalVector horizontal_opening_angle(3,Interval(0));
    IntervalVector vertical_opening_angle(3,Interval(0));

    parameters.get_LiDAR_error_model_param(rho_uncertinty,phi_uncertinty,theta_uncertinty,horizontal_angle_uncertainty,vertical_angle_uncertainty);

    if(pointclouds_Interval[0].second.empty())
    {
        pc=&pointcloud.pointclouds[0].second;
    }
    else
    {
        pc=&transform_last_use_particle;
    }
    for (unsigned int i = 0; i < pc->points.size(); ++i)
    {

        x_coordinate=pc->points[i].x;
        y_coordinate=pc->points[i].y;
        z_coordinate=pc->points[i].z;

        rho=sqrt(pow(x_coordinate,2)+pow(y_coordinate,2)+pow(z_coordinate,2));
        theta=acos(z_coordinate/rho);
        phi=atan2(y_coordinate,x_coordinate);

        interval_rho=Interval(rho).inflate(rho_uncertinty);
        interval_phi=Interval(phi).inflate(phi_uncertinty);
        interval_theta=Interval(theta).inflate(theta_uncertinty);

        coordinate[0]=interval_rho*sin(interval_theta)*cos(interval_phi);
        coordinate[1]=interval_rho*sin(interval_theta)*sin(interval_phi);
        coordinate[2]=interval_rho*cos(interval_theta);
        //cout<<coordinate<<endl;
        //additional uncertinty due to the initial footprint of the laser beam

//        horizontal_opening_angle[0]=sin(Interval(phi))+Interval(-horizontal_angle_uncertainty,+horizontal_angle_uncertainty);
//        horizontal_opening_angle[1]=-cos(Interval(phi))+Interval(-horizontal_angle_uncertainty,+horizontal_angle_uncertainty);
//        horizontal_opening_angle[2]=Interval(0)+Interval(-horizontal_angle_uncertainty,+horizontal_angle_uncertainty);

        horizontal_opening_angle[0]=sin(phi)*Interval(0).inflate(horizontal_angle_uncertainty);
        horizontal_opening_angle[1]=-cos(phi)*Interval(0).inflate(horizontal_angle_uncertainty);
        horizontal_opening_angle[2]=Interval(0)*Interval(0).inflate(horizontal_angle_uncertainty);

//        vertical_opening_angle[0]=cos(Interval(theta))*cos(Interval(phi))+Interval(-vertical_angle_uncertainty,+vertical_angle_uncertainty);
//        vertical_opening_angle[1]=cos(Interval(theta))*sin(Interval(phi))+Interval(-vertical_angle_uncertainty,+vertical_angle_uncertainty);
//        vertical_opening_angle[2]=-sin(Interval(theta))+Interval(-vertical_angle_uncertainty,+vertical_angle_uncertainty);

        vertical_opening_angle[0]=cos(theta)*cos(phi)*Interval(0).inflate(vertical_angle_uncertainty);
        vertical_opening_angle[1]=cos(theta)*sin(phi)*Interval(0).inflate(vertical_angle_uncertainty);
        vertical_opening_angle[2]=-sin(theta)*Interval(0).inflate(vertical_angle_uncertainty);

        coordinate+=horizontal_opening_angle+vertical_opening_angle;
        //cout<<coordinate<<endl;
        LiDARpoints.emplace_back(coordinate);
    }
    //convert LiDAR-point into spherical coordinates
    //parallelism
//    std::vector<std::thread> thread_vec(4);
//    std::mutex               mutex;
//    int cloud_size= pc->points.size();
//    for ( int i = 0; i < 3; ++i)
//    {
//        const  int start_index = cloud_size / 4 * i;
//        const  int end_index   = cloud_size / 4 * (i + 1);
//        thread_vec[i] =
//                std::thread(&Particle_Filter::intervalCalcuateThread,this, pc,start_index,end_index,rho_uncertinty, phi_uncertinty,theta_uncertinty, horizontal_angle_uncertainty, vertical_angle_uncertainty ,&mutex);
//    }
//     const int left_index_start = cloud_size / 4*3;
//     const int left_index_end   = cloud_size ;
//
//    thread_vec[3] =
//            std::thread(&Particle_Filter::intervalCalcuateThread,this, pc,left_index_start,left_index_end,rho_uncertinty, phi_uncertinty,theta_uncertinty, horizontal_angle_uncertainty, vertical_angle_uncertainty ,&mutex);
//
//    for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it)
//    {
//        it->join();
//    }
    if(pointclouds_Interval[0].second.empty())
    {
        pointclouds_Interval[0]=make_pair(pc->header.stamp,LiDARpoints);
    }
    else
    {
        pointclouds_Interval[1]=make_pair(pc->header.stamp,LiDARpoints);
    }
    LiDARpoints.clear();
}
void Particle_Filter::intervalCalcuateThread(pcl::PointCloud<pcl::PointXYZ> *pc,int start_index, int end_index, double rho_uncertinty,double phi_uncertinty,double theta_uncertinty,double horizontal_angle_uncertainty,double vertical_angle_uncertainty,std::mutex* mutex)
{
    double x_coordinate;
    double y_coordinate;
    double z_coordinate;
    double rho;
    double phi;
    double theta;
    Interval interval_rho;
    Interval interval_phi;
    Interval interval_theta;
    IntervalVector coordinate(3,Interval(0));
    IntervalVector horizontal_opening_angle(3,Interval(0));
    IntervalVector vertical_opening_angle(3,Interval(0));

    for (unsigned int i = start_index; i < end_index; ++i)
    {
        x_coordinate=pc->points[i].x;
        y_coordinate=pc->points[i].y;
        z_coordinate=pc->points[i].z;

        rho=sqrt(x_coordinate*x_coordinate+y_coordinate*y_coordinate+z_coordinate*z_coordinate);
        theta=acos(z_coordinate/rho);
        phi=atan(y_coordinate/x_coordinate);
        interval_rho=(rho-rho_uncertinty,rho+rho_uncertinty);
        interval_phi=(phi-phi_uncertinty,phi+phi_uncertinty);
        interval_theta=(theta-theta_uncertinty,theta+theta_uncertinty);

        coordinate[0]=interval_rho*sin(interval_theta)*cos(interval_phi);
        coordinate[1]=interval_rho*sin(interval_theta)*sin(interval_phi);
        coordinate[2]=interval_rho*cos(interval_theta);

        //additional uncertinty due to the initial footprint of the laser beam
        horizontal_opening_angle[0]=sin(Interval(phi))+Interval(-horizontal_angle_uncertainty,+horizontal_angle_uncertainty);
        horizontal_opening_angle[1]=-cos(Interval(phi))+Interval(-horizontal_angle_uncertainty,+horizontal_angle_uncertainty);
        horizontal_opening_angle[2]=Interval(0)+Interval(-horizontal_angle_uncertainty,+horizontal_angle_uncertainty);

        vertical_opening_angle[0]=cos(Interval(theta))*cos(Interval(phi))+Interval(-vertical_angle_uncertainty,+vertical_angle_uncertainty);
        vertical_opening_angle[1]=cos(Interval(theta))*sin(Interval(phi))+Interval(-vertical_angle_uncertainty,+vertical_angle_uncertainty);
        vertical_opening_angle[2]=-sin(Interval(theta))+Interval(-vertical_angle_uncertainty,+vertical_angle_uncertainty);
        coordinate+=horizontal_opening_angle+vertical_opening_angle;
        mutex->lock();
        LiDARpoints.emplace_back(coordinate);
        mutex->unlock();
    }
}
void plot(vector<double> & distance, vector<double> sum)
{
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::scatter(distance,sum);
    matplotlibcpp::title("Sample figure");
    // Save the image (file format is determined by the extension)
    matplotlibcpp::show();
}
bool GreaterSort(Particle_Filter::particle_weighted a, Particle_Filter::particle_weighted b)
{
    return a.weight_normal>b.weight_normal;
}
void Particle_Filter::pointxyz2pointxyzi(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc2,pcl::PointCloud<pcl::PointXYZI> &temp)
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
void Particle_Filter::get_label(pcl::PointCloud<pcl::PointXYZI> &temp, vector<int> &label)
{

    floorSegmentParams params;
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


}
void Particle_Filter::show_pointcloud_original(int argc, char** argv, LiDAR_PointCloud & pointcloud)
{
    pcl::PointCloud<pcl::PointXYZ> * match;
    match= &pointcloud.pointclouds[start_index].second;
    //add points to pointcloud(truth)
    for (int i = 0; i < match->points.size(); ++i) {
        pcl::PointXYZRGB temp;
        temp.x=match->points[i].x;
        temp.y=match->points[i].y;
        temp.z=match->points[i].z;
        temp.r=255;
        temp.g=0;
        temp.b=0;
        need_show_truth.points.emplace_back(temp);
    }
    //add points to pointcloud(transformed)
    for (int i = 0; i < transform_last_use_particle.points.size(); ++i) {
        pcl::PointXYZRGB temp;
        temp.x=transform_last_use_particle.points[i].x;
        temp.y=transform_last_use_particle.points[i].y;
        temp.z=transform_last_use_particle.points[i].z;
        temp.r=0;
        temp.g=0;
        temp.b=255;
        need_show_transformed.points.emplace_back(temp);
    }
//    for (int i = 0; i < pointcloud.pointclouds[1].second.points.size(); ++i) {
//        pcl::PointXYZRGB temp;
//        temp.x=pointcloud.pointclouds[1].second.points[i].x;
//        temp.y=pointcloud.pointclouds[1].second.points[i].y;
//        temp.z=pointcloud.pointclouds[1].second.points[i].z;
//        temp.r=0;
//        temp.g=0;
//        temp.b=255;
//        need_show_transformed.points.emplace_back(temp);
//    }
    pointcloud_show(argc,argv);
}
vector<IntervalVector> intervalbuild_debug(vector<int> &ind, Parameters parameters, LiDAR_PointCloud pointcloud)
{
    double rho_uncertinty;
    double phi_uncertinty;
    double theta_uncertinty;
    double horizontal_angle_uncertainty;
    double vertical_angle_uncertainty;
    pcl::PointCloud<pcl::PointXYZ> *pc;
    pc=&pointcloud.pointclouds[1].second;

    double x_coordinate;
    double y_coordinate;
    double z_coordinate;
    double rho;
    double phi;
    double theta;
    Interval interval_rho;
    Interval interval_phi;
    Interval interval_theta;
    IntervalVector coordinate(3,Interval(0));
    IntervalVector horizontal_opening_angle(3,Interval(0));
    IntervalVector vertical_opening_angle(3,Interval(0));
    vector<IntervalVector> erg;
    parameters.get_LiDAR_error_model_param(rho_uncertinty,phi_uncertinty,theta_uncertinty,horizontal_angle_uncertainty,vertical_angle_uncertainty);

    for (unsigned int i = 0; i < ind.size(); ++i)
    {
        x_coordinate=pc->points[ind[i]].x;
        y_coordinate=pc->points[ind[i]].y;
        z_coordinate=pc->points[ind[i]].z;
        rho=sqrt(pow(x_coordinate,2)+pow(y_coordinate,2)+pow(z_coordinate,2));
        theta=acos(z_coordinate/rho);
        phi=atan2(y_coordinate,x_coordinate);
        interval_rho=Interval(rho).inflate(rho_uncertinty);
        interval_phi=Interval(phi).inflate(phi_uncertinty);
        interval_theta=Interval(theta).inflate(theta_uncertinty);

        coordinate[0]=interval_rho*sin(interval_theta)*cos(interval_phi);
        coordinate[1]=interval_rho*sin(interval_theta)*sin(interval_phi);
        coordinate[2]=interval_rho*cos(interval_theta);
        //cout<<coordinate<<endl;
        //additional uncertinty due to the initial footprint of the laser beam
        horizontal_opening_angle[0]=sin(phi);
        horizontal_opening_angle[1]=-cos(phi);
        horizontal_opening_angle[2]=Interval(0);
        horizontal_opening_angle[0]=horizontal_opening_angle[0]*Interval(0).inflate(horizontal_angle_uncertainty);
        horizontal_opening_angle[1]=horizontal_opening_angle[1]*Interval(0).inflate(horizontal_angle_uncertainty);
        horizontal_opening_angle[2]=horizontal_opening_angle[2]*Interval(0).inflate(horizontal_angle_uncertainty);

        vertical_opening_angle[0]=cos(theta)*cos(phi);
        vertical_opening_angle[1]=cos(theta)*sin(phi);
        vertical_opening_angle[2]=-sin(theta);
        vertical_opening_angle[0]=vertical_opening_angle[0]*Interval(0).inflate(vertical_angle_uncertainty);
        vertical_opening_angle[1]=vertical_opening_angle[1]*Interval(0).inflate(vertical_angle_uncertainty);
        vertical_opening_angle[2]=vertical_opening_angle[2]*Interval(0).inflate(vertical_angle_uncertainty);
        coordinate+=horizontal_opening_angle+vertical_opening_angle;

        erg.emplace_back(coordinate);
    }
    return erg;
}
bool intersection_debug(vector<IntervalVector> &erg, IntervalVector & origin)
{
    bool temp=false;
    for(auto item : erg)
    {
        cout<<"truth:"<<item<<endl;
        cout<<"origin:"<<origin<<endl;
        if(item[0].intersects(origin[0]) && item[1].intersects(origin[1]) && item[2].intersects(origin[2]))
        {
            temp= true;
        }
    }
    return temp;
}
vector<Eigen::Vector3d> Particle_Filter::get_ground_truth(Parameters &parameters, Measurement &measurement, IMU &imu)
{
    IntervalMatrix rotation(3,3);
    rotation= imu.vel2rotatation(parameters.get_START_COMPUTE_TIME(), parameters.get_END_COMPUTE_TIME());
    //mms->camera
    Eigen::Matrix4d transform1=Eigen::Matrix4d::Identity();
    //camera->imu
    Eigen::Matrix4d transform2=Eigen::Matrix4d::Identity();
    transform1=measurement.tf_mms_cam();
    transform2=measurement.tf_cam_imu();
    Eigen::Matrix4d relativ_transformation_imu=Eigen::Matrix4d::Identity();
    measurement.transform_gt_imu(transform1,transform2,parameters);
    relativ_transformation_imu=measurement.calculate_relative_transformation_imu(parameters);
    vector<vector<bool>> test;
    vector<bool> temp;
    temp.emplace_back(rotation[0][0].contains(relativ_transformation_imu(0,0)));
    temp.emplace_back(rotation[0][1].contains(relativ_transformation_imu(0,1)));
    temp.emplace_back(rotation[0][2].contains(relativ_transformation_imu(0,2)));
    test.emplace_back(temp);
    temp.clear();

    temp.emplace_back(rotation[1][0].contains(relativ_transformation_imu(1,0)));
    temp.emplace_back(rotation[1][1].contains(relativ_transformation_imu(1,1)));
    temp.emplace_back(rotation[1][2].contains(relativ_transformation_imu(1,2)));
    test.emplace_back(temp);
    temp.clear();

    temp.emplace_back(rotation[2][0].contains(relativ_transformation_imu(2,0)));
    temp.emplace_back(rotation[2][1].contains(relativ_transformation_imu(2,1)));
    temp.emplace_back(rotation[2][2].contains(relativ_transformation_imu(2,2)));
    test.emplace_back(temp);
    temp.clear();

    for( int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cout<<test[i][j]<<" ";
        }
        cout<<endl;
    }
    cout<<" "<<endl;
}
void Particle_Filter::get_start_end_cloud_index(LiDAR_PointCloud &pointcloud ,Parameters & parameters,int &start_index, int &end_index)
{
    int i;
    if(pointcloud.pointclouds[0].first>parameters.get_START_COMPUTE_TIME())
    {
        start_index=0;
    }
    for(i=1;i<pointcloud.pointclouds.size();i++)
    {
        if(pointcloud.pointclouds[i].first<parameters.get_START_COMPUTE_TIME() && pointcloud.pointclouds[i-1].first<parameters.get_START_COMPUTE_TIME())
        {
            start_index=i;
        }
        if(pointcloud.pointclouds[i].first>parameters.get_END_COMPUTE_TIME() && pointcloud.pointclouds[i-1].first<parameters.get_END_COMPUTE_TIME())
        {
            end_index=i;
        }
        if(start_index!=-1 && end_index!=-1)
        {
            break;
        }
    }
    if(i==pointcloud.pointclouds.size())
    {
        end_index=pointcloud.pointclouds.size()-1;
    }
}
vector<pair<Eigen::Vector3d,Eigen::Vector3d>> Particle_Filter::particle_filter_set_up(Parameters &parameters,IMU &imu, KdTree & kd,  LiDAR_PointCloud &pointcloud ,Measurement &measurement,int argc, char ** argv){

    int count=0;

    pointclouds_Interval.resize(2);
    get_start_end_cloud_index(pointcloud,parameters,start_index, end_index);
    //6 Dimension transformation IntervalVector
    IntervalVector box_6D=create_6D_box(imu,pointcloud);
    //use end_time to build KD-Tree
    //tree_after_transform.setInputCloud(pointcloud.pointclouds[0].second.makeShared());
    tree_after_transform.setInputCloud(pointcloud.pointclouds[start_index].second.makeShared());
    build_LiDAR_Interval(parameters,pointcloud);
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> particle=generate_particle(box_6D,2,3,5,15,15 ,2);//233332 //345552
    vector<double> distance;
    vector<particle_weighted> sums;
    long start,end;
    start= curTime();

    //DEBUG
    //    cout<<particle[1463].first<<endl;
    //    cout<<particle[1463].second<<endl;

    int particle_size=particle.size();
    int point_cloud_size=pointcloud.pointclouds[end_index].second.points.size();
    pcl::PointCloud<pcl::PointXYZ> estimation;
    vector<IntervalVector> estimation_interval;

    Eigen::Quaterniond ground_truth;
    Eigen::Quaterniond particle_q;
    ground_truth = Eigen::AngleAxisd(-2.33287e-05, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(-0.000138494, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(-0.000794976, Eigen::Vector3d::UnitZ());

    pcl::PointCloud<pcl::PointXYZI> temp_matched;
    pointxyz2pointxyzi(pointcloud.pointclouds[start_index].second.makeShared(),temp_matched);
    get_label(temp_matched,label_matched);

    for(int j=0;j<particle_size;j++)
    {
//        Ground_Truth
//        particle[j].first[0]=-2.33287e-05;
//        particle[j].first[1]=-0.000138494;
//        particle[j].first[2]=-0.000794976;
//
//        particle[j].second[0]=  -0.0623501;
//        particle[j].second[1]= -0.00126607;
//        particle[j].second[2]=  0.00118102;
//2915
        particle[j].first[0]=-0.00293548;
        particle[j].first[1]=0.00692661;
        particle[j].first[2]=0.0191461;

        particle[j].second[0]=-1.61129;
        particle[j].second[1]=-2.30754;
        particle[j].second[2]=  0;




        //particle_q = Eigen::AngleAxisd(particle[j].first[0], Eigen::Vector3d::UnitX())
        //               * Eigen::AngleAxisd(particle[j].first[1], Eigen::Vector3d::UnitY())
        //               * Eigen::AngleAxisd(particle[j].first[2], Eigen::Vector3d::UnitZ());
        //double dis_q= particle_q.angularDistance(ground_truth);
        //double dis= sqrt( pow(particle[j].second[0]-(-0.0623501),2)+pow(particle[j].second[1]-(-0.00126607),2)+pow(particle[j].second[2]-(0.00118102),2));

        transform_use_particle(pointcloud.pointclouds[end_index].second,particle[j].first, particle[j].second);

        //DEBUG
        show_pointcloud_original(argc,argv,pointcloud);

        pcl::PointCloud<pcl::PointXYZI> temp;
        pointxyz2pointxyzi(transform_last_use_particle.makeShared(),temp);
        get_label(temp,label_transformed);

        build_LiDAR_Interval(parameters,pointcloud);
        //parallelism
        std::vector<std::thread> thread_vec(8);
        std::mutex               mutex;
        for ( int i = 0; i < 7; ++i)
        {
              int start_index = point_cloud_size / 8 * i;
              int end_index   = point_cloud_size / 8 * (i + 1);

            thread_vec[i] =
                    std::thread(&Particle_Filter::particleCalcuateThread,this,&pointcloud, start_index,end_index, &mutex);
        }
        const int left_index_start = point_cloud_size / 8*7;
        const int left_index_end   = point_cloud_size ;
        thread_vec[7] =
                std::thread(&Particle_Filter::particleCalcuateThread,this,&pointcloud, left_index_start,left_index_end,&mutex);

        for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it)
        {
            it->join();
        }

        //DEBUG
        //cout<<matched.points.size()<<endl;
        //cout<<unmatched.points.size()<<endl;
        //cout<<summe<<endl;
        //add_point2pointcloud(pointcloud);
        //pointcloud_show_match(argc,argv);
        //show_boxes(argc,argv);
        //cout<<particle[j].first<<endl;
        //cout<<particle[j].second<<endl;
        //cout<<summe<<endl;

        //distance.emplace_back(sqrt(dis*dis+dis_q*dis_q));
        sums.emplace_back(particle_weighted(j,summe));
        cout<<count++<<endl;
        update_max(summe,j);
        update_min(summe,j);
        pointclouds_Interval[1].second.clear();
        label_transformed.clear();
        matched.clear();
        unmatched.clear();
        marker_array.markers.clear();
        summe=0;

    }
    cout<<max_value<<endl;
    cout<<max_index<<endl;
    cout<<min_value<<endl;
    cout<<min_index<<endl;
    //plot(distance,sums);

    double gesamt_sum=0;
    for(auto s : sums)
    {
        gesamt_sum+=s.weight;
    }
    //normalization
    //vector<double> sums_normal(sums.size());

    for(int i=0;i<sums.size();i++)
    {
        sums[i].weight_normal=sums[i].weight/gesamt_sum;

    }
    //sort
    sort(sums.begin(),sums.end(),GreaterSort);
    //resampling
    int particle_number=sums.size();
    vector<particle_weighted> resample_weight(sums.size(),particle_weighted(0,0));
    int np=0;
    int k=0;

    for(int i=0;i<particle_number;i++)
    {
        np=round(sums[i].weight_normal*particle_number);
        for(int j=0;j<np;j++)
        {
            resample_weight[k++]=sums[i];
            if(k==particle_number)
            {
                goto out;
            }
        }
    }
    while (k < particle_number)
    {
        resample_weight[k++] = sums[0];
    }
    out:

    //calculate average use first 15% points
    gesamt_sum=0;
    Eigen::Vector3d erg1;
    Eigen::Vector3d erg2;
    erg1[0]=0;
    erg1[1]=0;
    erg1[2]=0;
    erg2[0]=0;
    erg2[1]=0;
    erg2[2]=0;
    for(int i=0;i<particle_number*0.15;i++)
    {
        gesamt_sum+=resample_weight[i].weight_normal;
    }
    for(int i=0;i<particle_number*0.15;i++)
    {
        resample_weight[i].weight_normal=resample_weight[i].weight_normal/gesamt_sum;
    }
    for(int i=0;i<particle_number*0.15;i++)
    {
        erg1+=particle[resample_weight[i].particle_index].first*resample_weight[i].weight_normal;
        erg2+=particle[resample_weight[i].particle_index].second*resample_weight[i].weight_normal;
    }
    cout<<erg1<<endl;
    cout<<erg2<<endl;
    //ground_truth compare
    //GNSS integieren
    //Random choose




    //resampling
    int null_count=0;
    for(int i=0;i<weights.size();i++)
    {
        if(weights[i].weight<parameters.get_PF_threshold())
        {
            weights[i].weight=-1;
            null_count++;
        }
    }

//    for(int i=0;i<weights.size();i++)
//    {
//       if(weights[i].weight==-1)
//       {
//           weights[i].weight=weights[select[s++%10]].weight;
//           weights[i].index=weights[select[s++%10]].index;
//       }
//    }
//    //get average
//    Eigen::Vector3d translation;
//    Eigen::Vector3d rotation;
//    translation[0]=0;
//    translation[1]=0;
//    translation[2]=0;
//    rotation[0]=0;
//    rotation[1]=0;
//    rotation[2]=0;
//    for(auto item:weights)
//    {
//        translation+=particle[item.index].second;
//        rotation+=particle[item.index].first;
//    }
//    translation=translation/weights.size();
//    rotation=rotation/weights.size();
//    cout<<translation<<endl;
//    cout<<rotation<<endl;

    end= curTime();
    cout<<particle[max_index].first<<endl;
    cout<<particle[max_index].second<<endl;
    cout<<max_value<<endl;
    cout<<end-start<<endl;
    //plot(distance,sums);
    //pcl::PointCloud<pcl::PointXYZ> transformed;
    //transform_use_particle(pointcloud.pointclouds[0].second,particle[max_index].first,particle[max_index].second,transformed);
    //show_pointcloud( argc,  argv,transformed,pointcloud.pointclouds[1].second );

    //Weight Normalization
    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> result;
    result.emplace_back(make_pair(particle[max_index].first,particle[max_index].second));
    transform_use_particle(pointcloud.pointclouds[0].second,result[0].first,result[0].second);
    transform_use_particle_Interval(pointcloud.pointclouds_Interval[0].second,result[0].first,result[0].second,estimation_interval);
    pcl::PointCloud<pcl::PointXYZ> erg;
    erg.header.stamp=pointcloud.pointclouds[1].first;
    for( auto item : estimation)
    {
        erg.points.emplace_back(item);
    }
    pointcloud.pointclouds.emplace_back(make_pair(pointcloud.pointclouds[1].first,erg));
    pointcloud.pointclouds_Interval.emplace_back(make_pair(pointcloud.pointclouds_Interval[1].first,estimation_interval));
    pointcloud.pointclouds.erase(pointcloud.pointclouds.begin());
    pointcloud.pointclouds.erase(pointcloud.pointclouds.begin());
    pointcloud.pointclouds_Interval.erase(pointcloud.pointclouds_Interval.begin());
    pointcloud.pointclouds_Interval.erase(pointcloud.pointclouds_Interval.begin());
    imu.vel_data.erase(imu.vel_data.begin(),imu.vel_data.end()-2);
    return result;
}
void Particle_Filter::update_max(double s, int index)
{
    if(s>max_value)
    {
        max_value=s;
        max_index=index;
    }
}
void Particle_Filter::update_min(double s, int index)
{
    if(s<min_value)
    {
        min_value=s;
        min_index=index;
    }
}
void Particle_Filter::particleCalcuateThread(LiDAR_PointCloud *pointCloud,int start_index, int end_index,std::mutex* mutex)
{
    vector<int> indices;
    vector<float> distances;
    for(int i=start_index; i<end_index;i++)
    {
        double radius = sqrt(pow(pointclouds_Interval[1].second[i][0].diam(),2)+ pow(pointclouds_Interval[1].second[i][1].diam(),2)+ pow(pointclouds_Interval[1].second[i][2].diam(),2))/2;
        if (tree_after_transform.radiusSearch(transform_last_use_particle.points[i], radius, indices, distances) >0)
            //if (tree_after_transform.nearestKSearch(transform_last_use_particle.points[i], 1, indices, distances) >0)
        {
            summe+= calculate_weight(*pointCloud, indices,if_matched, pointclouds_Interval[1].second[i],i, mutex);
            indices.clear();
            distances.clear();
        }
        else
        {
            pcl::PointXYZRGB temp;
            temp.x=transform_last_use_particle.points[i].x;
            temp.y=transform_last_use_particle.points[i].y;
            temp.z=transform_last_use_particle.points[i].z;
            temp.r=0;
            temp.g=0;
            temp.b=255;
            mutex->lock();
            unmatched.points.emplace_back(temp);
            //add_marker_to_array(point_after_transform,marker_array,0,0,0,0.5);
            mutex->unlock();
        }
    }
}

vector<pair<Eigen::Vector3d,Eigen::Vector3d>> Particle_Filter::generate_particle(IntervalVector box_6d, int num0,int num1, int num2, int num3, int num4, int num5)
{
    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> erg;
    Eigen::Vector3d temp1;
    Eigen::Vector3d temp2;
    double d1=(box_6d[0].ub()-box_6d[0].lb())/num0;
    double d2=(box_6d[1].ub()-box_6d[1].lb())/num1;
    double d3=(box_6d[2].ub()-box_6d[2].lb())/num2;
    double d4=(box_6d[3].ub()-box_6d[3].lb())/num3;
    double d5=(box_6d[4].ub()-box_6d[4].lb())/num4;
    double d6=(box_6d[5].ub()-box_6d[5].lb())/num5;
    for(int i=0;i<num0;i++)
    {
        for(int j=0;j<num1;j++)
        {
            for(int k=0;k<num2;k++)
            {
                for(int l=0;l<num3;l++)
                {
                    for(int m=0;m<num4;m++)
                    {
                        for(int n=0;n<num5;n++)
                        {
                            temp1[0]=box_6d[0].lb()+i*d1;
                            temp1[1]=box_6d[1].lb()+j*d2;
                            temp1[2]=box_6d[2].lb()+k*d3;
                            temp2[0]=box_6d[3].lb()+l*d4;
                            temp2[1]=box_6d[4].lb()+m*d5;
                            temp2[2]=box_6d[5].lb()+n*d6;
                            erg.emplace_back(make_pair(temp1,temp2));
                        }
                    }
                }
            }
        }
    }
    return erg;
}

void Particle_Filter::transform_use_particle(pcl::PointCloud<pcl::PointXYZ> pointcloud, Eigen::Vector3d &angle, Eigen::Vector3d &translation){
    transform_last_use_particle.header.stamp=pointcloud.header.stamp;
    transformationsmatrix.block(0,0,3,3)=eulerAnglesToRotationMatrix(angle);
    transformationsmatrix(0,3)=translation[0];
    transformationsmatrix(1,3)=translation[1];
    transformationsmatrix(2,3)=translation[2];
    pcl::transformPointCloud(pointcloud,transform_last_use_particle,transformationsmatrix);
}

void Particle_Filter::transform_use_particle_Interval(vector<IntervalVector> pointcloud_interval, Eigen::Vector3d &angle, Eigen::Vector3d &translation,vector<IntervalVector> &after_transform_interval)
{
    IntervalMatrix tranformation_matrix(4,4);
    IntervalVector pos_temp(4);
    IntervalVector after(4);
    IntervalVector temp(3);

    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            tranformation_matrix[i][j]=transformationsmatrix(i,j);
        }
    }
    //#pragma omp  parallel  for default(none)    num_threads(8) firstprivate(pointcloud_interval) shared(pos_temp) shared(after)  firstprivate(temp) shared(tranformation_matrix) shared(after_transform_interval)
    for(int i=0;i< pointcloud_interval.size();i++)
    {
        pos_temp[0]=pointcloud_interval[i][0];
        pos_temp[1]=pointcloud_interval[i][1];
        pos_temp[2]=pointcloud_interval[i][2];
        pos_temp[3]=1;
        after= tranformation_matrix*pos_temp;

        temp[0]=after[0];
        temp[1]=after[1];
        temp[2]=after[2];
        //#pragma  omp critical
        after_transform_interval.emplace_back(temp);
    }
}

Eigen::Matrix3d Particle_Filter::eulerAnglesToRotationMatrix(Eigen::Vector3d &theta)
{
    Eigen::Matrix3d  R_x;
    R_x <<
        1, 0, 0,
            0, cos(theta[0]), -sin(theta[0]),
            0, sin(theta[0]), cos(theta[0]);

    Eigen::Matrix3d  R_y;
    R_y<<
       cos(theta[1]), 0, sin(theta[1]),
            0       ,1,     0,
            -sin(theta[1]), 0, cos(theta[1]);
    Eigen::Matrix3d  R_z;
    R_z<<
       cos(theta[2])   , -sin(theta[2]), 0,
            sin(theta[2])   ,  cos(theta[2]), 0,
            0,  0,  1;
    Eigen::Matrix3d R= R_z*R_y*R_x;
    return  R;

}

IntervalVector Particle_Filter::create_6D_box(IMU imu, LiDAR_PointCloud pointcloud)
{
    double start_time=pointcloud.pointclouds[start_index].first;
    double end_time=pointcloud.pointclouds[end_index].first;
    IntervalMatrix rotation(3,3);
    IntervalVector box_6D(6);
    IntervalVector Euler_Angle(3);
    Interval velocity_interval_xy(-4.16,13.8);
    Interval velocity_interval_z(-2.,2.);
    rotation=imu.vel2rotatation(start_time,end_time);

    Euler_Angle=IntervalrotationMatrixtoEulerAngle(rotation);

    box_6D[3]=velocity_interval_xy*(end_time-start_time);
    box_6D[4]=velocity_interval_xy*(end_time-start_time);
    box_6D[5]=velocity_interval_z*(end_time-start_time);
    box_6D[0]=Euler_Angle[0];
    box_6D[1]=Euler_Angle[1];
    box_6D[2]=Euler_Angle[2];
    return box_6D;
}
//TODO maybe through volume
double Particle_Filter::calculate_weight(LiDAR_PointCloud &pointcloud, vector<int> &indices, vector<bool> &if_matched, IntervalVector & point_after_transform,int k, std::mutex* mutex)
{
    double volume=0;
    double s=0;
    bool b_matched=false;

    //cout<<indices.size()<<endl;
    for(int i=0;i<indices.size();i++) {
        //if (label[k] == pointcloud.labels[1].second[indices[i]]) {
            IntervalVector current = pointclouds_Interval[0].second[indices[i]];
            bool x = current[0].intersects(point_after_transform[0]);
            bool y = current[1].intersects(point_after_transform[1]);
            bool z = current[2].intersects(point_after_transform[2]);
            if (x && y && z /*&& !if_matched[indices[i]]&&(intersect_volume>=volume/8)*/) {
                if(label_transformed[k]==1 && label_matched[indices[i]]==1)
                {
                    s=s+0.5;
                    pcl::PointXYZRGB temp;
                    temp.x=transform_last_use_particle.points[k].x;
                    temp.y=transform_last_use_particle.points[k].y;
                    temp.z=transform_last_use_particle.points[k].z;
                    temp.r=255;
                    temp.g=0;
                    temp.b=0;
                    mutex->lock();
                    matched.points.emplace_back(temp);
                    add_marker_to_array(point_after_transform,marker_array,255,0,0,0.5);
                    add_marker_to_array(current,marker_array,0,255,0,0.5);
                    mutex->unlock();
                    b_matched= true;
                    break;
                }
                else if(label_transformed[k]==0 && label_matched[indices[i]]==0)
                {
                    s=s+2;
                    pcl::PointXYZRGB temp;
                    temp.x=transform_last_use_particle.points[k].x;
                    temp.y=transform_last_use_particle.points[k].y;
                    temp.z=transform_last_use_particle.points[k].z;
                    temp.r=255;
                    temp.g=0;
                    temp.b=0;
                    mutex->lock();
                    matched.points.emplace_back(temp);
                    add_marker_to_array(point_after_transform,marker_array,255,0,0,0.5);
                    add_marker_to_array(current,marker_array,0,255,0,0.5);
                    mutex->unlock();
                    b_matched=true;
                    break;
                }
                else
                {
                    s=s+0;
                }
                // add matched point to matched array
            }
    }
    if(!b_matched)
    {
        pcl::PointXYZRGB temp;
        temp.x=transform_last_use_particle.points[k].x;
        temp.y=transform_last_use_particle.points[k].y;
        temp.z=transform_last_use_particle.points[k].z;
        temp.r=0;
        temp.g=0;
        temp.b=255;
        mutex->lock();
        unmatched.points.emplace_back(temp);
        add_marker_to_array(point_after_transform,marker_array,0,0,0,0.5);
        mutex->unlock();
    }

    return s;
}
void Particle_Filter::add_point2pointcloud(LiDAR_PointCloud pointCloud)
{
    pcl::PointXYZRGB temp;
    bool find=false;
    int j;
    for(int i=0;i<match.size();i++)
    {
        add_marker_to_array(pointclouds_Interval[1].second[match[i]],marker_array,0,0,255,0.5);
        temp.x=pointCloud.pointclouds[1].second.points[match[i]].x;
        temp.y=pointCloud.pointclouds[1].second.points[match[i]].y;
        temp.z=pointCloud.pointclouds[1].second.points[match[i]].z;
        temp.r=0;
        temp.g=0;
        temp.b=255;
        need_show_truth.points.emplace_back(temp);
    }
    flag1=marker_array.markers.size();
    for(int i=0;i<point_index.size();i++)
    {
        add_marker_to_array(pointclouds_Interval[1].second[point_index[i]],marker_array,255,0,0,0.5);
        temp.x=pointCloud.pointclouds[1].second.points[point_index[i]].x;
        temp.y=pointCloud.pointclouds[1].second.points[point_index[i]].y;
        temp.z=pointCloud.pointclouds[1].second.points[point_index[i]].z;
        temp.r=255;
        temp.g=0;
        temp.b=0;
        need_show_transformed.points.emplace_back(temp);
    }
    flag2=marker_array.markers.size();
    for(int i=0;i<pointCloud.pointclouds[1].second.points.size();i++)
    {
        for( j=0;j<match.size();j++)
        {
            if(i==match[j])
            {break;}
        }
        if(j==match.size())
        {
            add_marker_to_array(pointclouds_Interval[1].second[i],marker_array,0,0,0,0.5);
            temp.x=pointCloud.pointclouds[1].second.points[i].x;
            temp.y=pointCloud.pointclouds[1].second.points[i].y;
            temp.z=pointCloud.pointclouds[1].second.points[i].z;
            temp.r=0;
            temp.g=0;
            temp.b=0;
            need_show_truth.points.emplace_back(temp);
        }
    }
}
Eigen::Vector3d Particle_Filter::estimation_position(vector<pair<Eigen::Vector3d,Eigen::Vector3d>>est_pos,Eigen::Vector3d &initial_pos)
{
    Eigen::Vector4d pos;
    Eigen::Vector3d result;
    pos[0]=initial_pos[0];
    pos[1]=initial_pos[1];
    pos[2]=initial_pos[2];
    pos[3]=1;
    for(auto item: est_pos) {
        Eigen::Matrix4d transformationsmatrix = Eigen::Matrix4d::Identity();
        transformationsmatrix.block(0, 0, 3, 3) = eulerAnglesToRotationMatrix(item.first);
        transformationsmatrix(0, 3) = item.second[0];
        transformationsmatrix(1, 3) = item.second[1];
        transformationsmatrix(2, 3) = item.second[2];
        pos=transformationsmatrix*pos;
    }
    result[0]=pos[0];
    result[1]=pos[1];
    result[2]=pos[2];
    cout<<result<<endl;
    return result;
}
IntervalVector Particle_Filter::IntervalrotationMatrixtoEulerAngle(IntervalMatrix & matrix)
{
    Interval roll ,pitch ,yaw;
    Interval roll1 ,pitch1 ,yaw1;
    IntervalVector temp(3,Interval(0));
    Interval sy= sqrt(matrix[0][0]*matrix[0][0]+matrix[1][0]*matrix[1][0]);
//    if(matrix[2][0].ub()<1)
//    {
//        if(matrix[2][0].lb()>-1)
//        {
//            pitch=asin(-matrix[2][0]);
//            yaw= atan2(matrix[1][0],matrix[0][0]);
//            roll= atan2(matrix[2][1],matrix[2][2]);
//        }
//        else
//        {
//            pitch=+M_PI/2;
//            yaw= -atan2(-matrix[1][2], matrix[1][1]);
//            roll= Interval(0);
//        }
//    }
//    else
//    {
//        pitch= - M_PI/2;
//        yaw= atan2(-matrix[1][2], matrix[1][1]);
//        roll = Interval(0);
//    }

//            pitch=-asin(matrix[2][0]);
//            yaw= atan2(matrix[2][1],matrix[2][2]);
//            roll= atan2(matrix[1][0],matrix[0][0]);
//
    if(sy.ub()<1e-6)
    {
        roll=atan2(-matrix[1][2],matrix[1][1]);
        pitch=atan2(-matrix[2][0],sy);
        yaw=0;

    }
    else
    {
        roll=atan2(matrix[2][1],matrix[2][2]);
        pitch=atan2(-matrix[2][0],sy);
        yaw=atan2(matrix[1][0],matrix[0][0]);
    }

    //57.295779513
    temp[0]=roll;
    temp[1]=pitch;
    temp[2]=yaw;
    cout<<temp<<endl;
    return temp;
}
void Particle_Filter::pointcloud_show_match( int argc,char **argv)
{
    ros::init(argc,argv,"Pointcloud_compare");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub=nh.advertise<sensor_msgs::PointCloud2>("matched_output",1);
    ros::Publisher truth_pub=nh.advertise<sensor_msgs::PointCloud2>("unmatched_output",1);
    sensor_msgs::PointCloud2 output1;
    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg(matched,output1);
    pcl::toROSMsg(unmatched,output2);
    output1.header.frame_id="map";
    output2.header.frame_id="map";
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        pcl_pub.publish(output1);
        truth_pub.publish(output2);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Particle_Filter::pointcloud_show( int argc,char **argv)
{
    ros::init(argc,argv,"Pointcloud_compare");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub=nh.advertise<sensor_msgs::PointCloud2>("transformed_output",1);
    ros::Publisher truth_pub=nh.advertise<sensor_msgs::PointCloud2>("truth_output",1);
//    ros::Publisher pcl_pub=nh.advertise<sensor_msgs::PointCloud2>("matched_output",1);
//    ros::Publisher truth_pub=nh.advertise<sensor_msgs::PointCloud2>("unmatched_output",1);
    sensor_msgs::PointCloud2 output1;
    sensor_msgs::PointCloud2 output2;

    pcl::toROSMsg(need_show_transformed,output1);
    pcl::toROSMsg(need_show_truth,output2);
//    pcl::toROSMsg(matched,output1);
//    pcl::toROSMsg(unmatched,output2);
    output1.header.frame_id="map";
    output2.header.frame_id="map";
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        pcl_pub.publish(output1);
        truth_pub.publish(output2);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
//
// Created by jeffqjn on 12.03.21.
//

