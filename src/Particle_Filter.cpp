#include "Particle_Filter.h"
void Particle_Filter::show_pointcloud(int argc, char** argv)
{
    ros::init(argc,argv,"Pointcloud_show");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub=nh.advertise<sensor_msgs::PointCloud2>("pcl_output",1);
    ros::Publisher truth_pub=nh.advertise<sensor_msgs::PointCloud2>("truth_output",1);
    sensor_msgs::PointCloud2 output1;
    sensor_msgs::PointCloud2 output2;

    pcl::toROSMsg(need_show_transformed,output1);
    pcl::toROSMsg(need_show_truth,output2);
    output1.header.frame_id="mms";
    output2.header.frame_id="mms";
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
       pcl_pub.publish(output1);
       truth_pub.publish(output2);
       ros::spinOnce();
       loop_rate.sleep();
    }
}

long curTime()
{
    std::chrono::milliseconds ms=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    return ms.count();
}

void Particle_Filter::build_LiDAR_Interval(Parameters parameters,double timestamp,pcl::PointCloud<pcl::PointXYZ> &pc)
{
    double x_coordinate;
    double y_coordinate;
    double z_coordinate;

    double rho;
    double phi;
    double theta;

    double rho_uncertinty;
    double phi_uncertinty;
    double theta_uncertinty;
    double horizontal_angle_uncertainty;
    double vertical_angle_uncertainty;

    Interval interval_rho;
    Interval interval_phi;
    Interval interval_theta;

    IntervalVector coordinate(3,Interval(0));
    IntervalVector horizontal_opening_angle(3,Interval(0));
    IntervalVector vertical_opening_angle(3,Interval(0));
    vector<IntervalVector> LiDARpoints;

    //convert LiDAR-point into spherical coordinates
    for (int i=0;i<pc.points.size();i++)
    {
        x_coordinate=pc.points[i].x;
        y_coordinate=pc.points[i].y;
        z_coordinate=pc.points[i].z;

        rho=sqrt(x_coordinate*x_coordinate+y_coordinate*y_coordinate+z_coordinate*z_coordinate);
        theta=acos(z_coordinate/rho);
        phi=atan(y_coordinate/x_coordinate);

        parameters.get_LiDAR_error_model_param(rho_uncertinty,phi_uncertinty,theta_uncertinty,horizontal_angle_uncertainty,vertical_angle_uncertainty);
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
        //addition
//     coordinate[0]=coordinate[0]+horizontal_opening_angle[0]+vertical_opening_angle[0];
//     coordinate[1]=coordinate[1]+horizontal_opening_angle[1]+vertical_opening_angle[1];
//     coordinate[2]=coordinate[2]+horizontal_opening_angle[2]+vertical_opening_angle[2];

        coordinate+=horizontal_opening_angle+vertical_opening_angle;
        LiDARpoints.emplace_back(coordinate);
    }

    pointclouds_Interval.emplace_back(make_pair(timestamp,LiDARpoints));
}
void plot(vector<double> & distance, vector<double> sum)
{
    matplotlibcpp::figure_size(1200, 780);
    //matplotlibcpp::plot(distance,sum);
    matplotlibcpp::scatter(distance,sum);
    matplotlibcpp::title("Sample figure");
//    // Enable legend.
//    matplotlibcpp::legend();
    // Save the image (file format is determined by the extension)
    matplotlibcpp::show();
}
//TODO calculate weight  when the weight smaller than threshold delete and duplicate the max weighted particle
//TODO ground point less weighted other greater weighted
//TODO resampling calculate the average
//TODO parallelism
vector<pair<Eigen::Vector3d,Eigen::Vector3d>> Particle_Filter::particle_filter_set_up(Parameters &parameters,IMU &imu, KdTree & kd, LiDAR_PointCloud &pointcloud ,int argc, char ** argv){
    double  start_time, end_time;
    int max_value=0,max_index,min_value=30000;
    start_time=pointcloud.pointclouds[0].first;
    end_time=pointcloud.pointclouds[1].first;
    int count=0;
    IntervalVector box_6D=create_6D_box(imu,start_time,end_time);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree_after_transform;
    tree_after_transform.setInputCloud(pointcloud.pointclouds[1].second.makeShared());
    build_LiDAR_Interval(parameters,end_time,pointcloud.pointclouds[1].second);
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> particle=generate_particle(box_6D,4,3,3,5,5,5);//233332
    vector<bool> if_matched(pointcloud.pointclouds[1].second.points.size(),false);
    long start,end;
    start= curTime();
    int particle_size=particle.size();
    int point_cloud_size=pointcloud.pointclouds[0].second.points.size();
    //#pragma omp  parallel for reduction(+:count) default(none)  num_threads(8)  firstprivate(particle) firstprivate(tree_after_transform) firstprivate(weight) firstprivate(max_value) firstprivate(max_value)
    vector<int> indices;
    vector<float> distances;

    pcl::PointCloud<pcl::PointXYZ> estimation;
    vector<IntervalVector> estimation_interval;
    vector<IntervalVector> transform_last_use_particle_interval;
    vector<double> ground_truth;
    vector<double> distance;
    vector<double> sums;
    ground_truth.emplace_back(-0.00133664);
    ground_truth.emplace_back(-0.00793514);
    ground_truth.emplace_back(-0.0455487);
    ground_truth.emplace_back(-0.0623499);
    ground_truth.emplace_back(-0.00126607);
    ground_truth.emplace_back(0.00118102);

    for(int j=0;j<particle_size;j++)
    {
        particle[j].first[0]=-0.00133664;
        particle[j].first[1]= -0.00793514;
        particle[j].first[2]=-0.0455487;

        particle[j].second[0]=-0.0623499;
        particle[j].second[1]=-0.00126607;
        particle[j].second[2]=0.00118102;

//        particle[j].first[0]=-1;
//        particle[j].first[1]= -1;
//        particle[j].first[2]=-1;
//
//        particle[j].second[0]=1;
//        particle[j].second[1]=1;
//        particle[j].second[2]=0;
        double d=sqrt(pow(particle[j].first[0]-ground_truth[0],2)+pow(particle[j].first[1]-ground_truth[1],2)+pow(particle[j].first[2]-ground_truth[2],2)+pow(particle[j].second[0]-ground_truth[3],2)+pow(particle[j].second[1]-ground_truth[4],2)+pow(particle[j].second[2]-ground_truth[5],2));
        distance.emplace_back(d);
        transform_use_particle(pointcloud.pointclouds[0].second,particle[j].first, particle[j].second,transform_last_use_particle);
        build_LiDAR_Interval(parameters,start_time,transform_last_use_particle);
        //show_pointcloud(argc,argv,transform_last_use_particle,pointcloud.pointclouds[1].second);
        int summe=0;
        //#pragma omp  parallel  for default(none) reduction(+:summe)    num_threads(8) firstprivate(transform_last_use_particle) firstprivate(pointcloud) firstprivate(indices) firstprivate(distances) shared(tree_after_transform) shared(point_cloud_size) shared(need_show_truth) shared(need_show_transformed) shared(if_matched)
        for (int i = 0; i < point_cloud_size; i++) {

            double radius = sqrt(pow(pointclouds_Interval[1].second[i][0].diam(),2)+ pow(pointclouds_Interval[1].second[i][1].diam(),2)+ pow(pointclouds_Interval[1].second[i][2].diam(),2));
            if (tree_after_transform.radiusSearch(transform_last_use_particle.points[i], radius, indices, distances) >
                0) {
                summe+= calculate_weight(pointcloud, indices,if_matched, pointclouds_Interval[1].second[i],i);
                indices.clear();
                distances.clear();
            }
        }
        cout<<summe<<endl;
        show_pointcloud(argc,argv);
        //DEBUG
        sums.emplace_back(summe);
        cout<<summe<<endl;
        //cout<<transform_last_use_particle.points.size()<<endl;
        cout<<count++<<endl;
        if(summe>max_value)
        {
            max_value=summe;
            max_index=j;
        }
        if(summe<min_value)
        {min_value=summe;}
        pointclouds_Interval.erase(pointclouds_Interval.end()-1);
        for(int l=0; l<if_matched.size();l++)
        {
            if_matched[l]= false;
        }

    }
    end= curTime();
    cout<<particle[max_index].first<<endl;
    cout<<particle[max_index].second<<endl;
    cout<<max_value<<endl;
    cout<<end-start<<endl;
    plot(distance,sums);
    pcl::PointCloud<pcl::PointXYZ> transformed;
    transform_use_particle(pointcloud.pointclouds[0].second,particle[max_index].first,particle[max_index].second,transformed);
    //show_pointcloud( argc,  argv,transformed,pointcloud.pointclouds[1].second );

    //Weight Normalization
    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> result;
    result.emplace_back(make_pair(particle[max_index].first,particle[max_index].second));
    transform_use_particle(pointcloud.pointclouds[0].second,result[0].first,result[0].second,estimation);
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

void Particle_Filter::transform_use_particle(pcl::PointCloud<pcl::PointXYZ> pointcloud, Eigen::Vector3d &angle, Eigen::Vector3d &translation, pcl::PointCloud<pcl::PointXYZ> &estimation){
    estimation.header.stamp=pointcloud.header.stamp;
    transformationsmatrix.block(0,0,3,3)=eulerAnglesToRotationMatrix(angle);
    transformationsmatrix(0,3)=translation[0];
    transformationsmatrix(1,3)=translation[1];
    transformationsmatrix(2,3)=translation[2];
    pcl::transformPointCloud(pointcloud,estimation,transformationsmatrix);
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

IntervalVector Particle_Filter::create_6D_box(IMU imu, double start_time, double end_time)
{
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
//TODO parallelism
//TODO ground-ground match non-ground-non-ground match
int Particle_Filter::calculate_weight(LiDAR_PointCloud &pointcloud, vector<int> &indices, vector<bool> &if_matched, IntervalVector & point_after_transform,int k)
{
    double volume=0;
    int summe=0;
    int r1=255,g1=0,b1=0;
    int r2=0,g2=0,b2=255;
    int32_t rgb_matched=(static_cast<uint32_t>(r1)<<255 | static_cast<uint32_t>(g1)<<0 | static_cast<uint32_t>(b1)<<0);
    int32_t rgb_nomatched=(static_cast<uint32_t>(r2)<<0 | static_cast<uint32_t>(g2)<<0 | static_cast<uint32_t>(b2)<<255);
    for(int i=0;i<indices.size();i++)
    {
        IntervalVector current= pointclouds_Interval[0].second[indices[i]];
        bool x=current[0].intersects(point_after_transform[0]);
        bool y=current[1].intersects(point_after_transform[1]);
        bool z=current[2].intersects(point_after_transform[2]);
//        double volume=point_after_transform[0].diam()*point_after_transform[1].diam()*point_after_transform[2].diam();
//        Interval x(max(current[0].lb(),point_after_transform[0].lb()),min(current[0].ub(),point_after_transform[0].ub()));
//        Interval y(max(current[1].lb(),point_after_transform[1].lb()),min(current[1].ub(),point_after_transform[1].ub()));
//        Interval z(max(current[2].lb(),point_after_transform[2].lb()),min(current[2].ub(),point_after_transform[2].ub()));
//        double intersect_volume=abs(x.ub()-x.lb())*abs(y.ub()-y.lb())*abs(z.ub()-z.lb());
        if(x&&y&&z&&!if_matched[indices[i]]/*&&(intersect_volume>=volume/8)*/)
        {
            pcl::PointXYZRGB truth;
            truth.x=pointcloud.pointclouds[1].second.points[indices[i]].x;
            truth.y=pointcloud.pointclouds[1].second.points[indices[i]].y;
            truth.z=pointcloud.pointclouds[1].second.points[indices[i]].z;
            truth.r=255;
            truth.g=0;
            truth.b=0;
            need_show_truth.points.emplace_back(truth);
            pcl::PointXYZRGB transformed;
            transformed.x=transform_last_use_particle.points[k].x;
            transformed.y=transform_last_use_particle.points[k].y;
            transformed.z=transform_last_use_particle.points[k].z;
            transformed.r=0;
            transformed.g=0;
            transformed.b=255;
            need_show_transformed.points.emplace_back(transformed);

                       Interval x(max(current[0].lb(),point_after_transform[0].lb()),min(current[0].ub(),point_after_transform[0].ub()));
           Interval y(max(current[1].lb(),point_after_transform[1].lb()),min(current[1].ub(),point_after_transform[1].ub()));
           Interval z(max(current[2].lb(),point_after_transform[2].lb()),min(current[2].ub(),point_after_transform[2].ub()));
            volume+=abs(x.ub()-x.lb())*abs(y.ub()-y.lb())*abs(z.ub()-z.lb());
            summe++;
            if_matched[indices[i]]=true;
           // return summe;
        }

        pcl::PointXYZRGB truth;
        truth.x=pointcloud.pointclouds[1].second.points[indices[i]].x;
        truth.y=pointcloud.pointclouds[1].second.points[indices[i]].y;
        truth.z=pointcloud.pointclouds[1].second.points[indices[i]].z;
        truth.r=0;
        truth.g=0;
        truth.b=0;
        need_show_truth.points.emplace_back(truth);
        pcl::PointXYZRGB transformed;
        transformed.x=transform_last_use_particle.points[k].x;
        transformed.y=transform_last_use_particle.points[k].y;
        transformed.z=transform_last_use_particle.points[k].z;
        transformed.r=0;
        transformed.g=0;
        transformed.b=0;
        need_show_transformed.points.emplace_back(transformed);

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

void Particle_Filter::pointcloud_show( int argc,char **argv,pcl::PointCloud<pcl::PointXYZ> transformed, pcl::PointCloud<pcl::PointXYZ> match)
{
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    ros::Publisher pub= nh.advertise<std_msgs::String>("output",1);
    ros::Rate loop(1);
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss<<"AH";
        msg.data=ss.str();
        ROS_INFO("test");
        pub.publish(msg);
        ros::spinOnce();
        loop.sleep();
    }
}
//
// Created by jeffqjn on 12.03.21.
//
