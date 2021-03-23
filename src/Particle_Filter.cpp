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
    ros::Publisher boxes_pub=nh.advertise<visualization_msgs::MarkerArray>("boxes_output",0);
    ros::Rate loop_rate(0.1);
    while(ros::ok())
    {
        boxes_pub.publish(marker_array);
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

    parameters.get_LiDAR_error_model_param(rho_uncertinty,phi_uncertinty,theta_uncertinty,horizontal_angle_uncertainty,vertical_angle_uncertainty);

    if(pointclouds_Interval[1].second.empty())
    {
        pc=&pointcloud.pointclouds[1].second;
    }
    else
    {
        pc=&transform_last_use_particle;
    }
    //convert LiDAR-point into spherical coordinates
    //parallelism
    std::vector<std::thread> thread_vec(4);
    std::mutex               mutex;
    int cloud_size= pc->points.size();
    for ( int i = 0; i < 3; ++i)
    {
        const  int start_index = cloud_size / 4 * i;
        const  int end_index   = cloud_size / 4 * (i + 1);
        thread_vec[i] =
                std::thread(&Particle_Filter::intervalCalcuateThread,this, pc,start_index,end_index,rho_uncertinty, phi_uncertinty,theta_uncertinty, horizontal_angle_uncertainty, vertical_angle_uncertainty ,&mutex);
    }
     const int left_index_start = cloud_size / 4*3;
     const int left_index_end   = cloud_size ;

    thread_vec[3] =
            std::thread(&Particle_Filter::intervalCalcuateThread,this, pc,left_index_start,left_index_end,rho_uncertinty, phi_uncertinty,theta_uncertinty, horizontal_angle_uncertainty, vertical_angle_uncertainty ,&mutex);

    for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it)
    {
        it->join();
    }
    if(pointclouds_Interval[1].second.empty())
    {
        pointclouds_Interval[1]=make_pair(pc->header.stamp,LiDARpoints);
    }
    else
    {
        pointclouds_Interval[0]=make_pair(pc->header.stamp,LiDARpoints);
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

//    Interval roh_u;
//    Interval theta_u;
//    Interval phi_u;

//    roh_u=sqrt(pow(Interval(-0.05,0.05),2)+(Interval(-0.05,0.05),2)+(Interval(-0.015,0.015),2));
//    theta_u=acos(Interval(-0.015,0.015)/roh_u);
//    phi_u=atan(Interval(-0.05,0.05)/Interval(-0.05,0.05));
//    cout<<"rho"<<roh_u<<endl;
//    cout<<"theta"<<theta_u<<endl;
//    cout<<"phi"<<phi_u<<endl;
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
    return a.weight>b.weight;
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
vector<pair<Eigen::Vector3d,Eigen::Vector3d>> Particle_Filter::particle_filter_set_up(Parameters &parameters,IMU &imu, KdTree & kd,  LiDAR_PointCloud &pointcloud ,int argc, char ** argv){

    int max_index;
    int count=0;
    double max_value, min_value;

    pointclouds_Interval.resize(2);

    //6 Dimension transformation IntervalVector
    IntervalVector box_6D=create_6D_box(imu,pointcloud);

    //use end_time to build KD-Tree

    tree_after_transform.setInputCloud(pointcloud.pointclouds[1].second.makeShared());


    build_LiDAR_Interval(parameters,pointcloud);
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> particle=generate_particle(box_6D,2,3,3,3,3,2);//233332

    // one LiDAR Point can only matched once
    if_matched.resize(pointcloud.pointclouds[1].second.points.size(),false);

    long start,end;
    start= curTime();

    int particle_size=particle.size();
    int point_cloud_size=pointcloud.pointclouds[0].second.points.size();

    pcl::PointCloud<pcl::PointXYZ> estimation;
    vector<IntervalVector> estimation_interval;

//    for(int num=0;num<pointclouds_Interval[1].second.size();num++)
//    {
//        add_marker_to_array(pointclouds_Interval[1].second[num],
//                marker_array, 255,0,
//                0, 0.5);
//    }
    for(int j=0;j<particle_size;j++)
    {
//        Ground_Truth
//        particle[j].first[0]=3.13981;
//        particle[j].first[1]=3.14148;
//        particle[j].first[2]=3.14133;
//
//        particle[j].second[0]=-0.129968;
//        particle[j].second[1]=-0.00358497;
//        particle[j].second[2]=-0.0100606;

        particle[j].first[0]=0.00175254;
        particle[j].first[1]=-0.000119835;
        particle[j].first[2]=-0.00042162;

        particle[j].second[0]=-0.13036;
        particle[j].second[1]=0.0063506;
        particle[j].second[2]=-0.00140698;

        transform_use_particle(pointcloud.pointclouds[0].second,particle[j].first, particle[j].second);

//                for(int num=0;num<pointclouds_Interval[0].second.size();num++)
//        {
//            add_marker_to_array(pointclouds_Interval[0].second[num],
//                                marker_array, 0,0,
//                                255, 0.5);
//        }
        pointcloud_show(argc,argv,pointcloud.pointclouds[1].second);
        pcl::PointCloud<pcl::PointXYZI> temp;
        pointxyz2pointxyzi(transform_last_use_particle.makeShared(),temp);
        get_label(temp,label);

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


        //cout<<summe<<endl;
        //show_pointcloud(argc,argv);
        //DEBUG
        //sums.emplace_back(summe);
        //cout<<transform_last_use_particle.points.size()<<endl;
        cout<<count++<<endl;
//        if(summe>max_value)
//        {
//            max_value=summe;
//            max_index=j;
//        }
//        if(summe<min_value)
//        {min_value=summe;}
        weights.emplace_back(particle_weighted(j,summe));
        pointclouds_Interval[0].second.clear();
        label.clear();
        for(int l=0; l<if_matched.size();l++)
        {
            if_matched[l]= false;
        }
        summe=0.;
        cout<<marker_array.markers.size()<<endl;
        show_boxes(argc,argv);

    }

    sort(weights.begin(),weights.end(),GreaterSort);

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
    vector<int> select{0,1,2,3,4,5,6,7,8,9,10};
    int s=0;
    for(int i=0;i<weights.size();i++)
    {
       if(weights[i].weight==-1)
       {
           weights[i].weight=weights[select[s++%10]].weight;
           weights[i].index=weights[select[s++%10]].index;
       }
    }
    //get average
    Eigen::Vector3d translation;
    Eigen::Vector3d rotation;
    translation[0]=0;
    translation[1]=0;
    translation[2]=0;
    rotation[0]=0;
    rotation[1]=0;
    rotation[2]=0;
    for(auto item:weights)
    {
        translation+=particle[item.index].second;
        rotation+=particle[item.index].first;
    }
    translation=translation/weights.size();
    rotation=rotation/weights.size();
    cout<<translation<<endl;
    cout<<rotation<<endl;

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

void Particle_Filter::particleCalcuateThread(LiDAR_PointCloud *pointCloud,int start_index, int end_index,std::mutex* mutex)
{
    vector<int> indices;
    vector<float> distances;
    for(int i=start_index; i<end_index;i++)
    {
        double radius = sqrt(pow(pointclouds_Interval[1].second[i][0].diam(),2)+ pow(pointclouds_Interval[1].second[i][1].diam(),2)+ pow(pointclouds_Interval[1].second[i][2].diam(),2));
        if (tree_after_transform.radiusSearch(transform_last_use_particle.points[i], radius, indices, distances) >0)
        {
            summe+= calculate_weight(*pointCloud, indices,if_matched, pointclouds_Interval[1].second[i],i, mutex);
            indices.clear();
            distances.clear();
        }
        //DEBUG
        first= true;
        c=3;
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
    double start_time=pointcloud.pointclouds[0].first;
    double end_time=pointcloud.pointclouds[1].first;
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
    double summe=0;

    for(int i=0;i<indices.size();i++) {
        if (label[k] == pointcloud.labels[1].second[indices[i]]) {
            IntervalVector current = pointclouds_Interval[1].second[indices[i]];
            bool x = current[0].intersects(point_after_transform[0]);
            bool y = current[1].intersects(point_after_transform[1]);
            bool z = current[2].intersects(point_after_transform[2]);
//        double volume=point_after_transform[0].diam()*point_after_transform[1].diam()*point_after_transform[2].diam();
//        Interval x(max(current[0].lb(),point_after_transform[0].lb()),min(current[0].ub(),point_after_transform[0].ub()));
//        Interval y(max(current[1].lb(),point_after_transform[1].lb()),min(current[1].ub(),point_after_transform[1].ub()));
//        Interval z(max(current[2].lb(),point_after_transform[2].lb()),min(current[2].ub(),point_after_transform[2].ub()));
//        double intersect_volume=abs(x.ub()-x.lb())*abs(y.ub()-y.lb())*abs(z.ub()-z.lb());
            if (x && y && z && !if_matched[indices[i]]/*&&(intersect_volume>=volume/8)*/) {
//            pcl::PointXYZRGB truth;
//            truth.x=pointcloud.pointclouds[1].second.points[indices[i]].x;
//            truth.y=pointcloud.pointclouds[1].second.points[indices[i]].y;
//            truth.z=pointcloud.pointclouds[1].second.points[indices[i]].z;
//            truth.r=255;
//            truth.g=0;
//            truth.b=0;
//            need_show_truth.points.emplace_back(truth);
//            pcl::PointXYZRGB transformed;
//            transformed.x=transform_last_use_particle.points[k].x;
//            transformed.y=transform_last_use_particle.points[k].y;
//            transformed.z=transform_last_use_particle.points[k].z;
//            transformed.r=0;
//            transformed.g=0;
//            transformed.b=255;
//            need_show_transformed.points.emplace_back(transformed);
//
//                       Interval x(max(current[0].lb(),point_after_transform[0].lb()),min(current[0].ub(),point_after_transform[0].ub()));
//           Interval y(max(current[1].lb(),point_after_transform[1].lb()),min(current[1].ub(),point_after_transform[1].ub()));
//           Interval z(max(current[2].lb(),point_after_transform[2].lb()),min(current[2].ub(),point_after_transform[2].ub()));
//            volume+=abs(x.ub()-x.lb())*abs(y.ub()-y.lb())*abs(z.ub()-z.lb());
//DEBUG
//if(d>0) {
    if (first) {
        mutex->lock();
        add_marker_to_array(point_after_transform, marker_array, 0, 0, 255, 0.5);
        mutex->unlock();
        first = false;
    }
   // if (c-- != 0) {
        mutex->lock();
        add_marker_to_array(current, marker_array, 255, 0, 0, 0.5);
        mutex->unlock();
   // }
   // d--;
//}
                if(label[k]==1)
                {
                    summe=summe+0.5;
                }
                else
                {
                    summe=summe+2;
                }

                if_matched[indices[i]] = true;
            }
            // return summe;
        }

//        pcl::PointXYZRGB truth;
//        truth.x=pointcloud.pointclouds[1].second.points[indices[i]].x;
//        truth.y=pointcloud.pointclouds[1].second.points[indices[i]].y;
//        truth.z=pointcloud.pointclouds[1].second.points[indices[i]].z;
//        truth.r=0;
//        truth.g=0;
//        truth.b=0;
//        need_show_truth.points.emplace_back(truth);
//        pcl::PointXYZRGB transformed;
//        transformed.x=transform_last_use_particle.points[k].x;
//        transformed.y=transform_last_use_particle.points[k].y;
//        transformed.z=transform_last_use_particle.points[k].z;
//        transformed.r=0;
//        transformed.g=0;
//        transformed.b=0;
//        need_show_transformed.points.emplace_back(transformed);

    }

    return summe;
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
    IntervalVector temp(3,Interval(0));
    Interval sy= sqrt(matrix[0][0]*matrix[0][0]+matrix[1][0]*matrix[1][0]);
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
    temp[0]=roll*180/M_PI;
    temp[1]=pitch*180/M_PI;
    temp[2]=yaw*180/M_PI;
    return temp;
}

void Particle_Filter::pointcloud_show( int argc,char **argv,pcl::PointCloud<pcl::PointXYZ> &match)
{
    ros::init(argc,argv,"Pointcloud_compare");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub=nh.advertise<sensor_msgs::PointCloud2>("transformed_output",1);
    ros::Publisher truth_pub=nh.advertise<sensor_msgs::PointCloud2>("truth_output",1);
    sensor_msgs::PointCloud2 output1;
    sensor_msgs::PointCloud2 output2;

    //add points to pointcloud(truth)
    for (int i = 0; i < match.points.size(); ++i) {
        pcl::PointXYZRGB temp;
        temp.x=match.points[i].x;
        temp.y=match.points[i].y;
        temp.z=match.points[i].z;
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

    pcl::toROSMsg(need_show_transformed,output1);
    pcl::toROSMsg(need_show_truth,output2);
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

