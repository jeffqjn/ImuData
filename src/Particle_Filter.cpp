#include "Particle_Filter.h"
void Particle_Filter::add_marker_to_array(IntervalVector point,
                         visualization_msgs::MarkerArray &marker_array, double r, double g,
                         double b, double a){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
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
    visualization_msgs::MarkerArray  temp1;
    visualization_msgs::MarkerArray  temp2;
    visualization_msgs::MarkerArray  temp3;
    visualization_msgs::MarkerArray  temp4;
    visualization_msgs::MarkerArray  temp5;
    int count=1000;
    for(int i=0;i<count;i++)
    {
        temp1.markers.emplace_back(unmatched_marker_array.markers[i]);
        temp2.markers.emplace_back(matched_marker_array.markers[i]);
        temp3.markers.emplace_back(truth_marker_array.markers[i]);
        temp4.markers.emplace_back(boden_truth_marker_array.markers[i]);
        temp5.markers.emplace_back(boden_transformed_marker_array.markers[i]);
    }

    ros::Publisher unmateched_boxes_pub=nh.advertise<visualization_msgs::MarkerArray>("unmatched_boxes_output",0);
    ros::Publisher matched_boxes_pub=nh.advertise<visualization_msgs::MarkerArray>("matched_boxes_output",0);
    ros::Publisher truth_boxes_pub=nh.advertise<visualization_msgs::MarkerArray>("truth_boxes_output",0);
    ros::Publisher boden_truth_boxes_pub=nh.advertise<visualization_msgs::MarkerArray>("boden_truth_boxes_output",0);
    ros::Publisher boden_transformed_boxes_pub=nh.advertise<visualization_msgs::MarkerArray>("boden_transformed_boxes_output",0);

    ros::Rate loop_rate(0.1);
    while(ros::ok())
    {
        unmateched_boxes_pub.publish(temp1);
        matched_boxes_pub.publish(temp2);
        truth_boxes_pub.publish(temp3);
        boden_truth_boxes_pub.publish(temp4);
        boden_transformed_boxes_pub.publish(temp5);
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
        pc=&pointcloud.pointclouds[current_index_first].second;
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
    //matplotlibcpp::scatter(distance,sum);
    matplotlibcpp::title("Sample figure");
    // Save the image (file format is determined by the extension)
    matplotlibcpp::show();
}
bool sort_test(pair<double,double> a, pair<double,double> b)
{
    return a.second>b.second;
}
vector<double> Particle_Filter::calculate_distance()
{
    vector<double> temp;
    for(int i=0;i<particle.size();i++)
    {
        //auch negativ SHOW
        //double d= sqrt(pow(particle[i].second[0]-ground_truth[1][0],2)+pow(particle[i].second[1]-ground_truth[1][1],2)+pow(particle[i].second[2]-ground_truth[1][2],2));
        double d= (particle[i].second[0]-ground_truth.back().second[0])+(particle[i].second[1]-ground_truth.back().second[1])+(particle[i].second[2]-ground_truth.back().second[2]);
        temp.emplace_back(d);
    }
    return temp;
}
void Particle_Filter::plot_debug()
{

    vector<double> dd;
    vector<double> s;
    vector<double> d_r;
    bool first1=true;
    dd=calculate_distance();
    d_r=calculate_rotation_distance();
    vector<vector<double>> show_x;
    vector<vector<double>> show_y;
    vector<vector<double>> show_z;
    vector<vector<double>> show_d_x;
    vector<vector<double>> show_d_y;
    vector<vector<double>> show_d_z;
    vector<vector<double>> show_legend_x;
    vector<vector<double>> show_legend_y;
    vector<vector<double>> show_legend_z;
    vector<double> show_x_truth;
    vector<double> show_d_x_truth;

    vector<double> show_ground_truth_x;
    vector<double> show_ground_truth_d_x;
    int times=0;

    vector<double> temp_x;
    vector<double> temp_d_x;
    vector<double> temp_y;
    vector<double> temp_d_y;
    vector<double> temp_z;
    vector<double> temp_d_z;
    vector<double> show_legend_temp;


//    while(times<10) {
//        for (int i = 0; i < 20; i++) {
//            temp_x.emplace_back(debug_sums[i+times*20]);
//            temp_d_x.emplace_back(dd[i+times*20]);
//            if(first1) {
//                show_legend_temp.emplace_back(d_r[i + times * 20]);
//                first1= false;
//            }
//        }
//        show_x.emplace_back(temp_x);
//        show_d_x.emplace_back(temp_d_x);
//        show_legend_x.emplace_back(show_legend_temp);
//        temp_x.clear();
//        temp_d_x.clear();
//        show_legend_temp.clear();
//        times++;
//        first1= true;
//    }
//        matplotlibcpp::figure();
//    matplotlibcpp::figure_size(1200, 780);
//    for(int i=0;i<show_x.size();i++)
//    {
//        string temp;
//       temp="Rotation roll distance to ground_truth: "+ to_string(show_legend_x[i][0]);
//       matplotlibcpp::named_plot(temp,show_d_x[i],show_x[i]);
//    }
//    matplotlibcpp::legend();
//    matplotlibcpp::title("Effect of different rotations along x-axis and combined translation to the matching result");
//    matplotlibcpp::save("/home/jeffqjn/Desktop/result/r_x_t.png");
//    temp_x.clear();
//    temp_d_x.clear();
//    temp_y.clear();
//    temp_d_y.clear();
//    temp_z.clear();
//    temp_d_z.clear();
//    show_legend_temp.clear();
//    show_x.clear();
//    show_d_x.clear();
//    show_y.clear();
//    show_d_y.clear();
//    show_z.clear();
//    show_d_z.clear();
//    show_legend_x.clear();
//    times=0;
//    while(times<10) {
//        for (int i = 200; i < 220; i++) {
//            temp_x.emplace_back(debug_sums[i+times*20]);
//            temp_d_x.emplace_back(dd[i+times*20]);
//            if(first1) {
//                show_legend_temp.emplace_back(d_r[i + times * 20]);
//                first1= false;
//            }
//        }
//        show_x.emplace_back(temp_x);
//        show_d_x.emplace_back(temp_d_x);
//        show_legend_x.emplace_back(show_legend_temp);
//        temp_x.clear();
//        temp_d_x.clear();
//        show_legend_temp.clear();
//        times++;
//        first1= true;
//    }
//    matplotlibcpp::figure();
//    matplotlibcpp::figure_size(1200, 780);
//    for(int i=0;i<show_x.size();i++)
//    {
//        string temp;
//        temp="Rotation pitch distance to ground_truth: "+ to_string(show_legend_x[i][0]);
//        matplotlibcpp::named_plot(temp,show_d_x[i],show_x[i]);
//    }
//    matplotlibcpp::legend();
//    matplotlibcpp::title("Effect of different rotations along y-axis and combined translation to the matching result");
//    matplotlibcpp::save("/home/jeffqjn/Desktop/result/r_y_t.png");
//
//    temp_x.clear();
//    temp_d_x.clear();
//    temp_y.clear();
//    temp_d_y.clear();
//    temp_z.clear();
//    temp_d_z.clear();
//    show_legend_temp.clear();
//    show_x.clear();
//    show_d_x.clear();
//    show_y.clear();
//    show_d_y.clear();
//    show_z.clear();
//    show_d_z.clear();
//    show_legend_x.clear();
//    times=0;
//    while(times<10) {
//        for (int i = 400; i < 420; i++) {
//            temp_x.emplace_back(debug_sums[i+times*20]);
//            temp_d_x.emplace_back(dd[i+times*20]);
//            if(first1) {
//                show_legend_temp.emplace_back(d_r[i + times * 20]);
//                first1= false;
//            }
//        }
//        show_x.emplace_back(temp_x);
//        show_d_x.emplace_back(temp_d_x);
//        show_legend_x.emplace_back(show_legend_temp);
//        temp_x.clear();
//        temp_d_x.clear();
//        show_legend_temp.clear();
//        times++;
//        first1= true;
//    }
//    matplotlibcpp::figure();
//    matplotlibcpp::figure_size(1200, 780);
//    for(int i=0;i<show_x.size();i++)
//    {
//        string temp;
//        temp="Rotation yaw distance to ground_truth: "+ to_string(show_legend_x[i][0]);
//        matplotlibcpp::named_plot(temp,show_d_x[i],show_x[i]);
//    }
//    matplotlibcpp::legend();
//    matplotlibcpp::title("Effect of different rotations along z-axis and combined translation to the matching result");
//    matplotlibcpp::save("/home/jeffqjn/Desktop/result/r_z_t.png");

    while(times<10) {
        for (int i = 0; i < 20; i++) {
            temp_x.emplace_back(debug_sums[i+times*60]);
            temp_d_x.emplace_back(dd[i+times*60]);
            if(first1) {
                show_legend_temp.emplace_back(d_r[i + times * 60]);
                first1= false;
            }
        }
        show_x.emplace_back(temp_x);
        show_d_x.emplace_back(temp_d_x);
        show_legend_x.emplace_back(show_legend_temp);
        temp_x.clear();
        temp_d_x.clear();
        show_legend_temp.clear();
        for (int i = 20; i < 40; i++) {
            temp_y.emplace_back(debug_sums[i+times*60]);
            temp_d_y.emplace_back(dd[i+times*60]);
        }
        show_y.emplace_back(temp_y);
        show_d_y.emplace_back(temp_d_y);
        temp_y.clear();
        temp_d_y.clear();
        show_legend_temp.clear();
        for (int i = 40; i < 60; i++) {
            temp_z.emplace_back(debug_sums[i+times*60]);
            temp_d_z.emplace_back(dd[i+times*60]);
        }
        show_z.emplace_back(temp_z);
        show_d_z.emplace_back(temp_d_z);
        temp_z.clear();
        temp_d_z.clear();
        show_legend_temp.clear();
        times++;
        first1= true;
    }

    matplotlibcpp::figure();
    matplotlibcpp::figure_size(1200, 780);
    for(int i=0;i<show_x.size();i++)
    {
        string temp;
       temp="Rotation roll distance to ground_truth: "+ to_string(show_legend_x[i][0]);
       matplotlibcpp::named_plot(temp,show_d_x[i],show_x[i]);
    }
    matplotlibcpp::legend();
    matplotlibcpp::title("Effect of different rotations along x-axis and translation along x-axis to the matching result");
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/r_x_t_x.png");
    matplotlibcpp::figure();
    matplotlibcpp::figure_size(1200, 780);
    for(int i=0;i<show_y.size();i++)
    {
        string temp;
        temp="Rotation roll distance to ground_truth: "+ to_string(show_legend_x[i][0]);
        matplotlibcpp::named_plot(temp,show_d_y[i],show_y[i]);
    }
    matplotlibcpp::legend();
    matplotlibcpp::title("Effect of different rotations along x-axis and translation along y-axis to the matching result");

    matplotlibcpp::save("/home/jeffqjn/Desktop/result/r_x_t_y.png");
    matplotlibcpp::figure();
    matplotlibcpp::figure_size(1200, 780);
    for(int i=0;i<show_z.size();i++)
    {
        string temp;
        temp="Rotation roll distance to ground_truth: "+ to_string(show_legend_x[i][0]);
        matplotlibcpp::named_plot(temp,show_d_z[i],show_z[i]);
    }
    matplotlibcpp::legend();
    matplotlibcpp::title("Effect of different rotations along x-axis and translation along z-axis to the matching result");
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/r_x_t_z.png");

    temp_x.clear();
    temp_d_x.clear();
    temp_y.clear();
    temp_d_y.clear();
    temp_z.clear();
    temp_d_z.clear();
    show_legend_temp.clear();
    show_x.clear();
    show_d_x.clear();
    show_y.clear();
    show_d_y.clear();
    show_z.clear();
    show_d_z.clear();
    show_legend_x.clear();
    times=0;

    while(times<10) {
        for (int i = 600; i < 620; i++) {
            temp_x.emplace_back(debug_sums[i+times*60]);
            temp_d_x.emplace_back(dd[i+times*60]);
            if(first1) {
                show_legend_temp.emplace_back(d_r[i + times * 60]);
                first1= false;
            }
        }
        show_x.emplace_back(temp_x);
        show_d_x.emplace_back(temp_d_x);
        show_legend_x.emplace_back(show_legend_temp);
        temp_x.clear();
        temp_d_x.clear();
        show_legend_temp.clear();
        for (int i = 620; i < 640; i++) {
            temp_y.emplace_back(debug_sums[i+times*60]);
            temp_d_y.emplace_back(dd[i+times*60]);
        }
        show_y.emplace_back(temp_y);
        show_d_y.emplace_back(temp_d_y);
        temp_y.clear();
        temp_d_y.clear();
        show_legend_temp.clear();
        for (int i = 640; i < 660; i++) {
            temp_z.emplace_back(debug_sums[i+times*60]);
            temp_d_z.emplace_back(dd[i+times*60]);
        }
        show_z.emplace_back(temp_z);
        show_d_z.emplace_back(temp_d_z);
        temp_z.clear();
        temp_d_z.clear();
        show_legend_temp.clear();
        times++;
        first1= true;
    }

    matplotlibcpp::figure();
    matplotlibcpp::figure_size(1200, 780);
    for(int i=0;i<show_x.size();i++)
    {
        string temp;
        temp="Rotation pitch distance to ground_truth: "+ to_string(show_legend_x[i][0]);
        matplotlibcpp::named_plot(temp,show_d_x[i],show_x[i]);
    }
    matplotlibcpp::legend();
    matplotlibcpp::title("Effect of different rotations along y-axis and translation along x-axis to the matching result");
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/r_y_t_x.png");
    matplotlibcpp::figure();
    matplotlibcpp::figure_size(1200, 780);
    for(int i=0;i<show_y.size();i++)
    {
        string temp;
        temp="Rotation pitch distance to ground_truth: "+ to_string(show_legend_x[i][0]);
        matplotlibcpp::named_plot(temp,show_d_y[i],show_y[i]);
    }
    matplotlibcpp::legend();
    matplotlibcpp::title("Effect of different rotations along y-axis and translation along y-axis to the matching result");

    matplotlibcpp::save("/home/jeffqjn/Desktop/result/r_y_t_y.png");
    matplotlibcpp::figure();
    matplotlibcpp::figure_size(1200, 780);
    for(int i=0;i<show_z.size();i++)
    {
        string temp;
        temp="Rotation pitch distance to ground_truth: "+ to_string(show_legend_x[i][0]);
        matplotlibcpp::named_plot(temp,show_d_z[i],show_z[i]);
    }
    matplotlibcpp::legend();
    matplotlibcpp::title("Effect of different rotations along y-axis and translation along z-axis to the matching result");
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/r_y_t_z.png");

    temp_x.clear();
    temp_d_x.clear();
    temp_y.clear();
    temp_d_y.clear();
    temp_z.clear();
    temp_d_z.clear();
    show_legend_temp.clear();
    show_x.clear();
    show_d_x.clear();
    show_y.clear();
    show_d_y.clear();
    show_z.clear();
    show_d_z.clear();
    show_legend_x.clear();
    times=0;
    while(times<10) {
        for (int i = 1200; i < 1220; i++) {
            temp_x.emplace_back(debug_sums[i+times*60]);
            temp_d_x.emplace_back(dd[i+times*60]);
            if(first1) {
                show_legend_temp.emplace_back(d_r[i + times * 60]);
                first1= false;
            }
        }
        show_x.emplace_back(temp_x);
        show_d_x.emplace_back(temp_d_x);
        show_legend_x.emplace_back(show_legend_temp);
        temp_x.clear();
        temp_d_x.clear();
        show_legend_temp.clear();
        for (int i = 1220; i < 1240; i++) {
            temp_y.emplace_back(debug_sums[i+times*60]);
            temp_d_y.emplace_back(dd[i+times*60]);
        }
        show_y.emplace_back(temp_y);
        show_d_y.emplace_back(temp_d_y);
        temp_y.clear();
        temp_d_y.clear();
        show_legend_temp.clear();
        for (int i = 1240; i < 1260; i++) {
            temp_z.emplace_back(debug_sums[i+times*60]);
            temp_d_z.emplace_back(dd[i+times*60]);
        }
        show_z.emplace_back(temp_z);
        show_d_z.emplace_back(temp_d_z);
        temp_z.clear();
        temp_d_z.clear();
        show_legend_temp.clear();
        times++;
        first1= true;
    }

    matplotlibcpp::figure();
    matplotlibcpp::figure_size(1200, 780);
    for(int i=0;i<show_x.size();i++)
    {
        string temp;
        temp="Rotation yaw distance to ground_truth: "+ to_string(show_legend_x[i][0]);
        matplotlibcpp::named_plot(temp,show_d_x[i],show_x[i]);
    }
    matplotlibcpp::legend();
    matplotlibcpp::title("Effect of different rotations along z-axis and translation along x-axis to the matching result");
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/r_z_t_x.png");
    matplotlibcpp::figure();
    matplotlibcpp::figure_size(1200, 780);
    for(int i=0;i<show_y.size();i++)
    {
        string temp;
        temp="Rotation yaw distance to ground_truth: "+ to_string(show_legend_x[i][0]);
        matplotlibcpp::named_plot(temp,show_d_y[i],show_y[i]);
    }
    matplotlibcpp::legend();
    matplotlibcpp::title("Effect of different rotations along z-axis and translation along y-axis to the matching result");

    matplotlibcpp::save("/home/jeffqjn/Desktop/result/r_z_t_y.png");
    matplotlibcpp::figure();
    matplotlibcpp::figure_size(1200, 780);
    for(int i=0;i<show_z.size();i++)
    {
        string temp;
        temp="Rotation yaw distance to ground_truth: "+ to_string(show_legend_x[i][0]);
        matplotlibcpp::named_plot(temp,show_d_z[i],show_z[i]);
    }
    matplotlibcpp::legend();
    matplotlibcpp::title("Effect of different rotations along z-axis and translation along z-axis to the matching result");
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/r_z_t_z.png");

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
    params.sensor_height     = 1.78253;
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
//    for (int i = 0; i < transform_last_use_particle.points.size(); ++i) {
//        pcl::PointXYZRGB temp;
//        temp.x=transform_last_use_particle.points[i].x;
//        temp.y=transform_last_use_particle.points[i].y;
//        temp.z=transform_last_use_particle.points[i].z;
//        temp.r=0;
//        temp.g=0;
//        temp.b=255;
//        need_show_transformed.points.emplace_back(temp);
//    }
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
void Particle_Filter::get_ground_truth(Parameters &parameters, Measurement &measurement, IMU &imu)
{
    vector<Eigen::Vector3d> erg;
    IntervalMatrix rotation(3,3);
    rotation= imu.vel2rotatation(start_time, end_time);
    //mms->camera
    Eigen::Matrix4d transform1=Eigen::Matrix4d::Identity();
    //camera->imu
    Eigen::Matrix4d transform2=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transform3=Eigen::Matrix4d::Identity();
    transform1=measurement.tf_mms_cam();
    transform2=measurement.tf_cam_velodyne();
    transform3=measurement.tf_cam_imu();
    Eigen::Matrix4d relativ_transformation_imu=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d relativ_transformation_imu2=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d relativ_transformation_imu3=Eigen::Matrix4d::Identity();
    measurement.transform_gt_imu(transform1,transform2,start_time,end_time);
    relativ_transformation_imu=measurement.calculate_relative_transformation_imu(start_time,end_time);
    relativ_transformation_imu2= (transform1*transform2).inverse()*relativ_transformation_imu*(transform1*transform2);
    relativ_transformation_imu3= (transform1*transform3).inverse()*relativ_transformation_imu*(transform1*transform3);
    IntervalMatrix erg1(3,3);
    IntervalMatrix left(3,3);
    IntervalMatrix right(3,3);
    Eigen::Matrix3d left_temp;
    Eigen::Matrix3d right_temp;
    Eigen::Matrix4d temp1;
    Eigen::Matrix4d temp2;
    temp1=transform2.inverse()*transform3;
    temp2=temp1.inverse();
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            left[i][j]=temp1(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            right[i][j]=temp2(i,j);
        }
    }
    erg1=left*rotation*right;
    vector<vector<bool>> test;
    vector<bool> temp;
    temp.emplace_back(erg1[0][0].contains(relativ_transformation_imu2(0,0)));
    temp.emplace_back(erg1[0][1].contains(relativ_transformation_imu2(0,1)));
    temp.emplace_back(erg1[0][2].contains(relativ_transformation_imu2(0,2)));
    test.emplace_back(temp);
    temp.clear();

    temp.emplace_back(erg1[1][0].contains(relativ_transformation_imu2(1,0)));
    temp.emplace_back(erg1[1][1].contains(relativ_transformation_imu2(1,1)));
    temp.emplace_back(erg1[1][2].contains(relativ_transformation_imu2(1,2)));
    test.emplace_back(temp);
    temp.clear();

    temp.emplace_back(erg1[2][0].contains(relativ_transformation_imu2(2,0)));
    temp.emplace_back(erg1[2][1].contains(relativ_transformation_imu2(2,1)));
    temp.emplace_back(erg1[2][2].contains(relativ_transformation_imu2(2,2)));
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




//    vector<vector<bool>> test;
//    vector<bool> temp;
//    temp.emplace_back(rotation[0][0].contains(relativ_transformation_imu2(0,0)));
//    temp.emplace_back(rotation[0][1].contains(relativ_transformation_imu2(0,1)));
//    temp.emplace_back(rotation[0][2].contains(relativ_transformation_imu2(0,2)));
//    test.emplace_back(temp);
//    temp.clear();
//
//    temp.emplace_back(rotation[1][0].contains(relativ_transformation_imu2(1,0)));
//    temp.emplace_back(rotation[1][1].contains(relativ_transformation_imu2(1,1)));
//    temp.emplace_back(rotation[1][2].contains(relativ_transformation_imu2(1,2)));
//    test.emplace_back(temp);
//    temp.clear();
//
//    temp.emplace_back(rotation[2][0].contains(relativ_transformation_imu2(2,0)));
//    temp.emplace_back(rotation[2][1].contains(relativ_transformation_imu2(2,1)));
//    temp.emplace_back(rotation[2][2].contains(relativ_transformation_imu2(2,2)));
//    test.emplace_back(temp);
//    temp.clear();
//
//    for( int i=0;i<3;i++)
//    {
//        for(int j=0;j<3;j++)
//        {
//            cout<<test[i][j]<<" ";
//        }
//        cout<<endl;
//    }
    cout<<" "<<endl;
    cout<<relativ_transformation_imu3<<endl;
    cout<<rotation<<endl;

    //change interval matrix into Lidar coordinate

    IntervalVector t1= IntervalrotationMatrixtoEulerAngle(rotation);
    Eigen::Matrix3d r=relativ_transformation_imu2.block(0,0,3,3);
    Eigen::Vector3d ro=rotationMatrixtoEulerAngle(r);
    Eigen::Vector3d ta;

    ta[0]=relativ_transformation_imu2(0,3);
    ta[1]=relativ_transformation_imu2(1,3);
    ta[2]=relativ_transformation_imu2(2,3);
    ground_truth.emplace_back(make_pair(ro,ta));
    measurement.tf2imu.clear();
}
Eigen::Vector3d Particle_Filter::rotationMatrixtoEulerAngle(Eigen::Matrix3d & matrix)
{
    Eigen::Vector3d euler_t;
    //euler_t=matrix.eulerAngles(2,1,0);
    //euler_t= euler_t*57.29578;
    if(matrix(2,0)<1)
    {
        if(matrix(2,0)>-1)
        {
            euler_t[1]=asin(-matrix(2,0));
            euler_t[2]=atan2(matrix(1,0), matrix(0,0));
            euler_t[0]=atan2(matrix(2,1), matrix(2,2));
        }
        else
        {
            euler_t[1]=+M_PI/2;
            euler_t[2]=-atan2(-matrix(1,2), matrix(1,1));
            euler_t[0]=0;
        }
    }
    else
    {
        euler_t[1]=-M_PI/2;
        euler_t[2]=atan2(-matrix(1,2), matrix(1,1));
        euler_t[0]=0;
    }
    return euler_t;
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
void Particle_Filter::show_all(int argc, char ** argv, LiDAR_PointCloud &pointcloud)
{
    //original start_index pointcloud
    pcl::PointCloud<pcl::PointXYZ> * match;
    match= &pointcloud.pointclouds[start_index].second;

    //add points to pointcloud(truth)
    for (int i = 0; i < match->points.size(); ++i) {
        pcl::PointXYZRGB temp;
        temp.x=match->points[i].x;
        temp.y=match->points[i].y;
        temp.z=match->points[i].z;
        temp.r=160;
        temp.g=160;
        temp.b=160;
        need_show_truth.points.emplace_back(temp);
    }
    for(int i=0;i<label_matched.size();i++)
    {
        if(label_matched[i]==1)
        {
            pcl::PointXYZRGB temp;
            temp.x=pointcloud.pointclouds[0].second.points[i].x;
            temp.y=pointcloud.pointclouds[0].second.points[i].y;
            temp.z=pointcloud.pointclouds[0].second.points[i].z;
            temp.r=160;
            temp.g=160;
            temp.b=160;
            boden_truth.points.emplace_back(temp);
            add_marker_to_array(pointclouds_Interval[0].second[i],boden_truth_marker_array,255,0,0,0.5);
        }
    }
    for(int i=0;i<label_transformed.size();i++)
    {
        if(label_transformed[i]==1)
        {
            pcl::PointXYZRGB temp;
            temp.x=transform_last_use_particle.points[i].x;
            temp.y=transform_last_use_particle.points[i].y;
            temp.z=transform_last_use_particle.points[i].z;
            temp.r=255;
            temp.g=0;
            temp.b=0;
            boden_transformed.points.emplace_back(temp);
            add_marker_to_array(pointclouds_Interval[1].second[i],boden_transformed_marker_array,255,0,0,0.5);
        }
    }
    for(int i=0;i<pointclouds_Interval[0].second.size();i++)
    {
        if(!pointclouds_Interval[0].second[i].is_empty()) {
            add_marker_to_array(pointclouds_Interval[0].second[i], truth_marker_array, 0, 0, 0, 0.3);
        }
    }
    pointcloud_show(argc, argv);

}
void Particle_Filter::particle_filter_parallelism(Parameters &parameters, LiDAR_PointCloud &pointcloud, int particle_index)
{
    non_ground_weight=parameters.get_particle_non_ground_weight();
    ground_weight=parameters.get_particle_ground_weight();
    int point_cloud_size=pointcloud.pointclouds[current_index_second].second.points.size();
    transform_use_particle(pointcloud.pointclouds[current_index_second].second,particle[particle_index].first, particle[particle_index].second);
    pcl::PointCloud<pcl::PointXYZI> temp;
    pointxyz2pointxyzi(transform_last_use_particle.makeShared(),temp);
    get_label(temp,label_transformed);
    build_LiDAR_Interval(parameters,pointcloud);

    //parallelism
    std::vector<std::thread> thread_vec(8);
    std::mutex               mutex;
    for ( int i = 0; i < 7; ++i)
    {
        int start_index_thread = point_cloud_size / 8 * i;
        int end_index_thread   = point_cloud_size / 8 * (i + 1);

        thread_vec[i] =
                std::thread(&Particle_Filter::particleCalcuateThread,this,&pointcloud, start_index_thread,end_index_thread, &mutex);
    }
    const int left_index_start = point_cloud_size / 8*7;
    const int left_index_end   = point_cloud_size ;
    thread_vec[7] =
            std::thread(&Particle_Filter::particleCalcuateThread,this,&pointcloud, left_index_start,left_index_end,&mutex);

    for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it)
    {
        it->join();
    }
}
void Particle_Filter::copy_pointcloud(pcl::PointCloud<pcl::PointXYZ> & from, pcl::PointCloud<pcl::PointXYZ> & to)
{
    for(auto item: from.points)
    {
        to.points.emplace_back(item);
    }
    to.header.stamp=from.header.stamp;
    to.header.frame_id=from.header.stamp;
}
void Particle_Filter::particle_filter_do(Parameters &parameters,IMU &imu, KdTree & kd, LiDAR_PointCloud &pointcloud , Measurement &measurement,int argc, char ** argv)
{
    int count=0;
    vector<pair<double,double>> temp;
    pcl::PointCloud<pcl::PointXYZI> temp_matched;
    //pcl::PointCloud<pcl::PointXYZ> transform_last_use_particle_calculated;
    //6 Dimension transformation IntervalVector
    //DEBUG move out
    //box_6D=create_6D_box(imu,pointcloud,measurement);
    //use end_time to build KD-Tree
    tree_after_transform.setInputCloud(pointcloud.pointclouds[current_index_first].second.makeShared());
    pointxyz2pointxyzi(pointcloud.pointclouds[current_index_first].second.makeShared(),temp_matched);
    get_label(temp_matched,label_matched);
     build_LiDAR_Interval(parameters,pointcloud);
    //DEBUG move out
    //particle=generate_particle(box_6D,2,4,4,5,5 ,2);//233332 //345552    //234772
    vector<double> distance;
    int particle_size=particle.size();
    cout<<pointclouds_Interval[0].second.size()<<endl;
    cout<<pointclouds_Interval[1].second.size()<<endl;
    for(int j=0;j<particle_size;j++)
    {
//        particle[j].first[0]=-0.00108146;
//        particle[j].first[1]=0.000829525;
//        particle[j].first[2]=0.000971853;
//        particle[j].second[0]=1.28915;
//        particle[j].second[1]=-0.0184414;
//        particle[j].second[2]=0.0422609;
        particle_filter_parallelism(parameters,pointcloud,j);
        //DEBUG
        //show_all(argc,argv,pointcloud);
        debug_sums.emplace_back(summe);
        //DEBUG
        sums.emplace_back(particle_weighted(j,summe));
        cout<<count++<<endl;
        update_max(summe,j);
        update_min(summe,j);
        pointclouds_Interval[1].second.clear();
        label_transformed.clear();
        label_matched.clear();
        matched.clear();
        unmatched.clear();
        unmatched_marker_array.markers.clear();
        matched_marker_array.markers.clear();
        boden_transformed_marker_array.markers.clear();
        boden_truth_marker_array.markers.clear();
        truth_marker_array.markers.clear();
        summe=0;
    }
    //DEBUG
    gesamte_sums=calcualte_gesamte_sums();
    calcualte_normalized_sums(gesamte_sums);
    //sort
    sort(sums.begin(),sums.end(),GreaterSort);
    resampling();
    calculate_average();
    cout<<result.size()<<endl;
    cout<<result.back().first<<endl;
    cout<<result.back().second<<endl;
    sums.clear();
    resample_weight.clear();
    transform_last_use_particle.points.clear();
}

double Particle_Filter::calcualte_gesamte_sums()
{
    double gesamt_sum=0;
    for(auto s : sums)
    {
        gesamt_sum+=s.weight;
    }
    return gesamt_sum;

}
void Particle_Filter::calcualte_normalized_sums(double gesamt_sum)
{
    for(int i=0;i<sums.size();i++)
    {
        sums[i].weight_normal=sums[i].weight/gesamt_sum;
    }
}
void Particle_Filter::resampling()
{
    //resampling
    particle_number=sums.size();
    resample_weight.resize(sums.size(),particle_weighted(0,0));
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
               return;
            }
        }
    }
    while (k < particle_number)
    {
        resample_weight[k++] = sums[0];
    }
}

void Particle_Filter::calculate_average()
{
    //calculate average use first 15% points
    gesamte_sums=0;
    Eigen::Vector3d erg1;
    Eigen::Vector3d erg2;
    erg1[0]=0;
    erg1[1]=0;
    erg1[2]=0;
    erg2[0]=0;
    erg2[1]=0;
    erg2[2]=0;
    for(int i=0;i<particle_number*0.05;i++)
    {
        gesamte_sums+=resample_weight[i].weight_normal;
    }
    for(int i=0;i<particle_number*0.05;i++)
    {
        resample_weight[i].weight_normal=resample_weight[i].weight_normal/gesamte_sums;
    }
    for(int i=0;i<particle_number*0.05;i++)
    {
        erg1+=particle[resample_weight[i].particle_index].first*resample_weight[i].weight_normal;
        erg2+=particle[resample_weight[i].particle_index].second*resample_weight[i].weight_normal;
    }
    result.emplace_back(make_pair(erg1,erg2));
}
vector<pair<Eigen::Vector3d,Eigen::Vector3d>> Particle_Filter::generate_particle_translation(IntervalVector box_6d, Eigen::Vector3d rotation,int num3, int num4, int num5)
{
    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> erg;
    Eigen::Vector3d temp1;
    Eigen::Vector3d temp2;
    temp1=rotation;
    Eigen::Vector3d truth_translation= ground_truth.back().second;
//    double schritt_x=box_6d[3].diam()/num3;
//    double schritt_y=box_6d[4].diam()/num4;
//    double schritt_z=box_6d[5].diam()/num5;
   //create particle for x direction
//   for(int i=0;i<num3;i++)
//   {
//       temp2[0]=truth_translation[0]+i*schritt_x;
//       temp2[1]=truth_translation[1];
//       temp2[2]=truth_translation[2];
//       erg.emplace_back(make_pair(temp1,temp2));
//   }
//   //create particle for y direction
//    for(int i=0;i<num4;i++)
//    {
//        temp2[0]=truth_translation[0];
//        temp2[1]=truth_translation[1]+i*schritt_y;
//        temp2[2]=truth_translation[2];
//        erg.emplace_back(make_pair(temp1,temp2));
//    }
//    //create particle for z direction
//    for(int i=0;i<num5;i++)
//    {
//        temp2[0]=truth_translation[0];
//        temp2[1]=truth_translation[1];
//        temp2[2]=truth_translation[2]+i*schritt_z;
//        erg.emplace_back(make_pair(temp1,temp2));
//    }

    //all pictures have the same distance
    double schritt_x=0.02;
    double schritt_y=0.02;
    double schritt_z=0.02;
    int half_x,half_y,half_z;
    half_x=num3/2;
    half_y=num4/2;
    half_z=num5/2;
       for(int i=0;i<num3;i++)
   {
//       temp2[0]=(truth_translation[0]-schritt_x*(half_x-1))+i*schritt_x;
//       temp2[1]=truth_translation[1];
//       temp2[2]=truth_translation[2];
//       erg.emplace_back(make_pair(temp1,temp2));
//       temp2[0]=truth_translation[0]-i*schritt_x;
//       temp2[1]=truth_translation[1];
//       temp2[2]=truth_translation[2];
//       erg.emplace_back(make_pair(temp1,temp2));
           temp2[0]=(truth_translation[0]-schritt_x*(half_x-1))+i*schritt_x;
           temp2[1]=truth_translation[1];
           temp2[2]=truth_translation[2];
           erg.emplace_back(make_pair(temp1,temp2));
   }
   //create particle for y direction
    for(int i=0;i<num4;i++)
    {
        temp2[0]=truth_translation[0];
        temp2[1]=(truth_translation[1]-schritt_y*(half_y-1))+i*schritt_y;
        temp2[2]=truth_translation[2];
        erg.emplace_back(make_pair(temp1,temp2));
//        temp2[0]=truth_translation[0];
//        temp2[1]=truth_translation[1]-i*schritt_y;
//        temp2[2]=truth_translation[2];
//        erg.emplace_back(make_pair(temp1,temp2));
    }
    //create particle for z direction
    for(int i=0;i<num5;i++)
    {
        temp2[0]=truth_translation[0];
        temp2[1]=truth_translation[1];
        temp2[2]=(truth_translation[2]-schritt_z*(half_z-1))+i*schritt_z;
        erg.emplace_back(make_pair(temp1,temp2));
//        temp2[0]=truth_translation[0];
//        temp2[1]=truth_translation[1];
//        temp2[2]=truth_translation[2]-i*schritt_z;
//        erg.emplace_back(make_pair(temp1,temp2));
    }
    return erg;
}
vector<Eigen::Vector3d> Particle_Filter::generate_particle_rotation(IntervalVector box_6d,int num0, int num1, int num2)
{
    vector<Eigen::Vector3d> erg;
    vector<Eigen::Vector3d> erg1;
    vector<Eigen::Vector3d> erg2;
    vector<Eigen::Vector3d> erg3;
    Eigen::Vector3d temp1;

    double d1=(box_6d[0].ub()-box_6d[0].lb())/num0;
    double d2=(box_6d[1].ub()-box_6d[1].lb())/num1;
    double d3=(box_6d[2].ub()-box_6d[2].lb())/num2;

    for(int i=0;i<num0;i++) {
        temp1[0]=ground_truth.back().first[0]+i*d1*5;
        temp1[1]=ground_truth.back().first[1];
        temp1[2]=ground_truth.back().first[2];
        erg1.emplace_back(temp1);
    }
    //erg1=particle_sort(erg1);
    for(int j=0;j<num1;j++) {
        temp1[0]=ground_truth.back().first[0];
        temp1[1]=ground_truth.back().first[1]+j*d2*5;
        temp1[2]=ground_truth.back().first[2];
        erg2.emplace_back(temp1);
    }
    //erg2=particle_sort(erg2);
    for(int k=0;k<num2;k++) {
        temp1[0]=ground_truth.back().first[0];
        temp1[1]=ground_truth.back().first[1];
        temp1[2]=ground_truth.back().first[2]+k*d3*5;
        erg3.emplace_back(temp1);
    }
    //erg3=particle_sort(erg3);
    erg.insert(erg.end(),erg1.begin(),erg1.end());
    erg.insert(erg.end(),erg2.begin(),erg2.end());
    erg.insert(erg.end(),erg3.begin(),erg3.end());
//                            temp1[0]=box_6d[0].lb()+i*d1;
//                            temp1[1]=box_6d[1].lb()+j*d2;
//                            temp1[2]=box_6d[2].lb()+k*d3;
//                            erg.emplace_back(temp1);

    return erg;
}
bool sortparticle(Particle_Filter::particle_sorted a, Particle_Filter::particle_sorted b)
{
    return a.d<b.d;
}
vector<double> Particle_Filter::calculate_rotation_distance()
{
    vector<double> temp;
    for(auto item: particle)
    {
        temp.emplace_back(calculate_angular_distance(item.first));
    }
    return temp;
}
vector<Eigen::Vector3d> Particle_Filter::particle_sort(vector<Eigen::Vector3d> temp)
{
    vector<Particle_Filter::particle_sorted> sorted;
    vector<Eigen::Vector3d> erg;
    for(auto item: temp)
    {
        double dis=calculate_angular_distance(item);
        sorted.emplace_back(particle_sorted(dis,item));
    }
    sort(sorted.begin(),sorted.end(),sortparticle);
    for(auto item: sorted)
    {
        erg.emplace_back(item.v);
    }
    return erg;
}
double Particle_Filter::calculate_angular_distance(Eigen::Vector3d item)
{
    Eigen::Quaterniond p = Eigen::AngleAxisd(item[0], Eigen::Vector3d::UnitX())
                     * Eigen::AngleAxisd(item[1], Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(item[2], Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond ground_truth_q = Eigen::AngleAxisd(ground_truth.back().first[0], Eigen::Vector3d::UnitX())
                                        * Eigen::AngleAxisd(ground_truth.back().first[1], Eigen::Vector3d::UnitY())
                                        * Eigen::AngleAxisd(ground_truth.back().first[2], Eigen::Vector3d::UnitZ());
    return p.angularDistance(ground_truth_q);
}
void Particle_Filter::create_pic(Parameters &parameters,IMU &imu, KdTree & kd,  LiDAR_PointCloud &pointcloud ,Measurement &measurement,int argc, char ** argv)
{
    int index=0;
    vector<pair<Eigen::Vector3d,Eigen::Vector3d>> particle_temp;
    get_start_end_cloud_index(pointcloud,parameters,start_index, end_index);
    for(int i=1;i<10;i++)
    {
        //2 pointclouds - 10 pointclouds
        current_index_first=start_index;
        current_index_second=current_index_first+1;
        start_time=pointcloud.pointclouds[current_index_first].first;
        end_time=pointcloud.pointclouds[current_index_second].first;
        box_6D=create_6D_box(imu,pointcloud,measurement);
        get_ground_truth(parameters,measurement,imu);
        //plot translation with truth rotation
        vector<pair<Eigen::Vector3d,Eigen::Vector3d>> translation_particle;
        //generate rotation
        vector<Eigen::Vector3d> rotation_particle=generate_particle_rotation(box_6D,10,10,10);

        for(int j=0;j<rotation_particle.size();j++)
        {
            translation_particle=generate_particle_translation(box_6D,rotation_particle[j],20,20 ,20);
            particle.insert(particle.end(),translation_particle.begin(),translation_particle.end());
            translation_particle.clear();
        }
        cout<<particle.size()<<endl;

        particle_filter_do(parameters,imu,kd,pointcloud,measurement,argc,argv);
        plot_debug();


//        while(index<particle_temp.size()) {
//            for (int i = index; i < particle_temp.size(); i++) {
//                particle.emplace_back(particle_temp[i]);
//                if ((i + 1) % 149 == 0)    //343
//                {
//                    index = i + 1;
//                    break;
//                }
//            }
//        }
    }




}
vector<pair<Eigen::Vector3d,Eigen::Vector3d>> Particle_Filter::particle_filter_set_up(Parameters &parameters,IMU &imu, KdTree & kd,  LiDAR_PointCloud &pointcloud ,Measurement &measurement,int argc, char ** argv){
    int rounds=0;
    int index=0;
    calculate_interval=parameters.get_calculate_interval();
    pointclouds_Interval.resize(2);
    create_pic(parameters,imu,kd,pointcloud,measurement,argc,argv);
    get_start_end_cloud_index(pointcloud,parameters,start_index, end_index);
    cout<<start_index<<endl;
    cout<<end_index<<endl;

    current_index_first=start_index;
    current_index_second=current_index_first+calculate_interval;
    for(;current_index_second<=end_index;)
    {
        //cout<<index++<<endl;
        start_time=pointcloud.pointclouds[current_index_first].first;
        end_time=pointcloud.pointclouds[current_index_second].first;
        //box_6D=create_6D_box(imu,pointcloud);
        //vector<pair<Eigen::Vector3d,Eigen::Vector3d>> particle_temp=generate_particle_translation(box_6D,1,1,1,149,1 ,1);//233332 //345552    //234772
        //particle.emplace_back(make_pair(ground_truth[0],ground_truth[1]));
        //particle_filter_do(parameters,imu,kd,pointcloud,measurement,argc,argv);
        //particle.clear();
        //pointclouds_Interval[0].second.clear();
        //pointclouds_Interval[1].second.clear();
        //mode=0;
//        while(index<particle_temp.size())
//        {
//            for(int i=index;i<particle_temp.size();i++)
//            {
//                particle.emplace_back(particle_temp[i]);
//                if((i+1)%149==0)    //343
//                {
//                    index=i+1;
//                    break;
//                }
//            }
//
//            particle_filter_do(parameters,imu,kd,pointcloud,measurement,argc,argv);
//            particle.clear();
//            pointclouds_Interval[0].second.clear();
//            pointclouds_Interval[1].second.clear();
//        }
        //plot_debug();
        get_ground_truth(parameters,measurement,imu);
        particle_filter_do(parameters,imu,kd,pointcloud,measurement,argc,argv);
        particle.clear();
        pointclouds_Interval[0].second.clear();
        pointclouds_Interval[1].second.clear();
        current_index_first=current_index_first+calculate_interval;
        current_index_second=current_index_second+calculate_interval;
    }
    //show();
    show_path(argc,argv);
}
void Particle_Filter::show()
{
    show_error();
}
void Particle_Filter::show_error()
{
    vector<int> sequence;
    vector<double> dis;
    double ddd;
    for(int i=0;i<ground_truth.size();i++)
    {
        sequence.emplace_back(i+1);
        ddd=sqrt(pow(ground_truth[i].second[0]-result[i].second[0],2));
        dis.emplace_back(ddd);
    }
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::scatter(sequence,dis);
    matplotlibcpp::xlim(0, 15);
    matplotlibcpp::title("Translation-comparision of particle filter output and ground-truth in x direction");
    // Save the image (file format is determined by the extension)
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/error_x.png");
    //sequence.clear();
    dis.clear();
    //y

    for(int i=0;i<ground_truth.size();i++)
    {
        //sequence.emplace_back(i+1);
        ddd=sqrt(pow(ground_truth[i].second[1]-result[i].second[1],2));
        dis.emplace_back(ddd);
    }
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::scatter(sequence,dis);
    matplotlibcpp::xlim(0, 20);
    matplotlibcpp::title("Translation-comparision of particle filter output and ground-truth in y direction");
    // Save the image (file format is determined by the extension)
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/error_y.png");
    //sequence.clear();
    dis.clear();
    //z
    for(int i=0;i<ground_truth.size();i++)
    {
        //sequence.emplace_back(i+1);
        ddd=sqrt(pow(ground_truth[i].second[2]-result[i].second[2],2));
        dis.emplace_back(ddd);
    }
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::scatter(sequence,dis);
    matplotlibcpp::xlim(0, 20);
    matplotlibcpp::title("Translation-comparision of particle filter output and ground-truth in z direction");
    // Save the image (file format is determined by the extension)
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/error_z.png");
    //sequence.clear();
    dis.clear();

    //roll
    for(int i=0;i<ground_truth.size();i++)
    {
        //sequence.emplace_back(i+1);
        ddd=sqrt(pow(ground_truth[i].first[0]-result[i].first[0],2));
        dis.emplace_back(ddd);
    }
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::scatter(sequence,dis);
    matplotlibcpp::xlim(0, 20);
    matplotlibcpp::title("Rotation-comparision of particle filter output and ground-truth in roll direction");
    // Save the image (file format is determined by the extension)
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/error_roll.png");
    //sequence.clear();
    dis.clear();
    //pitch
    for(int i=0;i<ground_truth.size();i++)
    {
        //sequence.emplace_back(i+1);
        ddd=sqrt(pow(ground_truth[i].first[1]-result[i].first[1],2));
        dis.emplace_back(ddd);
    }
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::scatter(sequence,dis);
    matplotlibcpp::xlim(0, 20);
    matplotlibcpp::title("Rotation-comparision of particle filter output and ground-truth in pitch direction");
    // Save the image (file format is determined by the extension)
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/error_pitch.png");
    //sequence.clear();
    dis.clear();
//yaw
    for(int i=0;i<ground_truth.size();i++)
    {
        //sequence.emplace_back(i+1);
        ddd=sqrt(pow(ground_truth[i].first[2]-result[i].first[2],2));
        dis.emplace_back(ddd);
    }
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::scatter(sequence,dis);
    matplotlibcpp::xlim(0, 20);
    matplotlibcpp::title("Rotation-comparision of particle filter output and ground-truth in yaw direction");
    // Save the image (file format is determined by the extension)
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/error_yaw.png");
    //sequence.clear();
    dis.clear();

    //distance
    for(int i=0;i<ground_truth.size();i++)
    {
        //sequence.emplace_back(i+1);
        ddd=sqrt(pow(ground_truth[i].second[0]-result[i].second[0],2)+pow(ground_truth[i].second[1]-result[i].second[1],2)+pow(ground_truth[i].second[2]-result[i].second[2],2));
        dis.emplace_back(ddd);
    }
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::scatter(sequence,dis);
    matplotlibcpp::xlim(0, 20);
    matplotlibcpp::title("Translation-comparision of particle filter output and ground-truth");
    // Save the image (file format is determined by the extension)
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/error_t.png");
   // sequence.clear();
    dis.clear();
    //rotation distance
    for(int i=0;i<ground_truth.size();i++)
    {
        Eigen::Quaterniond p = Eigen::AngleAxisd(result[i].first[0], Eigen::Vector3d::UnitX())
                               * Eigen::AngleAxisd(result[i].first[1], Eigen::Vector3d::UnitY())
                               * Eigen::AngleAxisd(result[i].first[2], Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond ground_truth_q = Eigen::AngleAxisd(ground_truth[i].first[0], Eigen::Vector3d::UnitX())
                                            * Eigen::AngleAxisd(ground_truth[i].first[1], Eigen::Vector3d::UnitY())
                                            * Eigen::AngleAxisd(ground_truth[i].first[2], Eigen::Vector3d::UnitZ());
        ddd=p.angularDistance(ground_truth_q);
        //sequence.emplace_back(i+1);
        dis.emplace_back(ddd);
    }
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::scatter(sequence,dis);
    matplotlibcpp::xlim(0, 20);
    matplotlibcpp::title("Rotation-comparision of particle filter output and ground-truth");
    // Save the image (file format is determined by the extension)
    matplotlibcpp::save("/home/jeffqjn/Desktop/result/error_r.png");
    sequence.clear();
    dis.clear();
}
void pose_initialize(geometry_msgs::PoseStamped  &temp)
{
    temp.pose.position.x=0;
    temp.pose.position.y=0;
    temp.pose.position.z=0;

    temp.pose.orientation.x=0;
    temp.pose.orientation.y=0;
    temp.pose.orientation.z=0;
    temp.pose.orientation.w=-1;
}
void Particle_Filter::show_path(int argc, char** argv) {
    Eigen::Matrix4d result_temp;
    //process the data
    for(int i=0;i<result.size();i++)
    {
        Eigen::Quaterniond p = Eigen::AngleAxisd(result[i].first[0], Eigen::Vector3d::UnitX())
                               * Eigen::AngleAxisd(result[i].first[1], Eigen::Vector3d::UnitY())
                               * Eigen::AngleAxisd(result[i].first[2], Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d temp= p.matrix();
        result_temp.block(0,0,3,3)=temp;
        result_temp(0,3)=result[i].second[0];
        result_temp(1,3)=result[i].second[1];
        result_temp(2,3)=result[i].second[2];
        result_temp(3,0)=0;
        result_temp(3,1)=0;
        result_temp(3,2)=0;
        result_temp(3,3)=1;

        result_matrix.emplace_back(result_temp.inverse());

    }
    //calculate pose
    nav_msgs::Path path;
    geometry_msgs::PoseStamped temp;
    Eigen::Matrix4d temp_m=Eigen::Matrix4d::Identity();
    Eigen::Matrix3d temp_q;

    int count = 1;
    pose_initialize(temp);
    while (count <= result_matrix.size()) {
        for (int i = 0; i < count; i++) {
            temp_m=result_matrix[i]*temp_m;
        }
        temp_q=temp_m.block(0,0,3,3);
        Eigen::Quaterniond temp_qq(temp_q);
        temp.pose.orientation.x=temp_qq.x();
        temp.pose.orientation.y=temp_qq.y();
        temp.pose.orientation.z=temp_qq.z();
        temp.pose.orientation.w=temp_qq.w();

        temp.pose.position.x=temp_m(0,3);
        temp.pose.position.y=temp_m(1,3);
        temp.pose.position.z=temp_m(2,3);

        path.poses.emplace_back(temp);

        count++;
    }

    //show
    ros::init(argc,argv,"Trajectoryshow");
    ros::NodeHandle nh;

    ros::Publisher path_pub=nh.advertise<nav_msgs::Path>("Trajectory_show",1);
    path.header.frame_id="map";
    ros::Rate loop_rate(1);

    while(ros::ok())
    {
        path_pub.publish(path);
        ros::spinOnce();
        loop_rate.sleep();
    }
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
            add_marker_to_array(pointclouds_Interval[1].second[i],unmatched_marker_array,0,0,255,0.3);
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
    //Eigen::Matrix3d R= R_x*R_y*R_z;
    return  R;

}

IntervalVector Particle_Filter::create_6D_box(IMU imu, LiDAR_PointCloud pointcloud,Measurement measurement)
{
    double start_compute_time=pointcloud.pointclouds[current_index_first].first;
    double end_compute_time=pointcloud.pointclouds[current_index_second].first;
    IntervalMatrix rotation(3,3);
    IntervalVector box_6D(6);
    IntervalVector Euler_Angle(3);
    Interval velocity_interval_xy(-4.16,13.8);
    Interval velocity_interval_z(-2.,2.);
    rotation=imu.vel2rotatation(start_compute_time,end_compute_time);
    IntervalMatrix rotation_velodyne(3,3);
    IntervalMatrix left(3,3);
    IntervalMatrix right(3,3);
    Eigen::Matrix3d left_temp;
    Eigen::Matrix3d right_temp;
    Eigen::Matrix4d temp1;
    Eigen::Matrix4d temp2;
    Eigen::Matrix4d transform2= measurement.tf_cam_velodyne();
    Eigen::Matrix4d transform3= measurement.tf_cam_imu();
    temp1=transform2.inverse()*transform3;
    temp2=temp1.inverse();
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            left[i][j]=temp1(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            right[i][j]=temp2(i,j);
        }
    }
    rotation_velodyne=left*rotation*right;
    Euler_Angle=IntervalrotationMatrixtoEulerAngle(rotation_velodyne);
    //if(particle_filter_first) {
        box_6D[3] = velocity_interval_xy * (end_compute_time - start_compute_time);
        box_6D[4] = velocity_interval_xy * (end_compute_time - start_compute_time);
        box_6D[5] = velocity_interval_z * (end_compute_time - start_compute_time);
       // particle_filter_first=false;
   // }
//    else
//    {
//        IntervalVector temp=speed_prediction();
//        box_6D[3] = temp[0]* (end_compute_time - start_compute_time);
//        box_6D[4] = temp[1]* (end_compute_time - start_compute_time);
//        box_6D[5] = temp[2]* (end_compute_time - start_compute_time);
//    }
    box_6D[0]=Euler_Angle[0];
    box_6D[1]=Euler_Angle[1];
    box_6D[2]=Euler_Angle[2];
    return box_6D;
}

IntervalVector Particle_Filter::speed_prediction()
{
    IntervalVector v(3);
    v[0]=result.back().second[0]/(calculate_interval*0.1);
    v[1]=result.back().second[1]/(calculate_interval*0.1);
    v[2]=result.back().second[2]/(calculate_interval*0.1);
    v.inflate(3);
    return v;
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
                    s=s+ground_weight;
                    pcl::PointXYZRGB temp;
                    temp.x=transform_last_use_particle.points[k].x;
                    temp.y=transform_last_use_particle.points[k].y;
                    temp.z=transform_last_use_particle.points[k].z;
                    temp.r=255;
                    temp.g=0;
                    temp.b=0;
                    mutex->lock();
                    matched.points.emplace_back(temp);
                    add_marker_to_array(point_after_transform,matched_marker_array,255,0,0,0.3);
                    //add_marker_to_array(current,truth_marker_array,0,255,0,0.5);
                    mutex->unlock();
                    b_matched= true;
                    break;
                }
                else if(label_transformed[k]==0 && label_matched[indices[i]]==0)
                {
                    s=s+non_ground_weight;
                    pcl::PointXYZRGB temp;
                    temp.x=transform_last_use_particle.points[k].x;
                    temp.y=transform_last_use_particle.points[k].y;
                    temp.z=transform_last_use_particle.points[k].z;
                    temp.r=255;
                    temp.g=0;
                    temp.b=0;
                    mutex->lock();
                    matched.points.emplace_back(temp);
                    add_marker_to_array(point_after_transform,matched_marker_array,255,0,0,0.3);
                    //add_marker_to_array(current,truth_marker_array,0,255,0,0.5);
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
        add_marker_to_array(point_after_transform,unmatched_marker_array,0,0,255,0.3);
        mutex->unlock();
    }
    return s;
}
//void Particle_Filter::add_point2pointcloud(LiDAR_PointCloud pointCloud)
//{
//    pcl::PointXYZRGB temp;
//    bool find=false;
//    int j;
//    for(int i=0;i<match.size();i++)
//    {
//        add_marker_to_array(pointclouds_Interval[1].second[match[i]],marker_array,0,0,255,0.5);
//        temp.x=pointCloud.pointclouds[1].second.points[match[i]].x;
//        temp.y=pointCloud.pointclouds[1].second.points[match[i]].y;
//        temp.z=pointCloud.pointclouds[1].second.points[match[i]].z;
//        temp.r=0;
//        temp.g=0;
//        temp.b=255;
//        need_show_truth.points.emplace_back(temp);
//    }
//    flag1=marker_array.markers.size();
//    for(int i=0;i<point_index.size();i++)
//    {
//        add_marker_to_array(pointclouds_Interval[1].second[point_index[i]],marker_array,255,0,0,0.5);
//        temp.x=pointCloud.pointclouds[1].second.points[point_index[i]].x;
//        temp.y=pointCloud.pointclouds[1].second.points[point_index[i]].y;
//        temp.z=pointCloud.pointclouds[1].second.points[point_index[i]].z;
//        temp.r=255;
//        temp.g=0;
//        temp.b=0;
//        need_show_transformed.points.emplace_back(temp);
//    }
//    flag2=marker_array.markers.size();
//    for(int i=0;i<pointCloud.pointclouds[1].second.points.size();i++)
//    {
//        for( j=0;j<match.size();j++)
//        {
//            if(i==match[j])
//            {break;}
//        }
//        if(j==match.size())
//        {
//            add_marker_to_array(pointclouds_Interval[1].second[i],marker_array,0,0,0,0.5);
//            temp.x=pointCloud.pointclouds[1].second.points[i].x;
//            temp.y=pointCloud.pointclouds[1].second.points[i].y;
//            temp.z=pointCloud.pointclouds[1].second.points[i].z;
//            temp.r=0;
//            temp.g=0;
//            temp.b=0;
//            need_show_truth.points.emplace_back(temp);
//        }
//    }
//}
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

    ros::Publisher transformed_matched_pub=nh.advertise<sensor_msgs::PointCloud2>("transformed_matched",1);
    ros::Publisher transformed_unmatched_pub=nh.advertise<sensor_msgs::PointCloud2>("transformed_unmatched",1);
    ros::Publisher truth_pub=nh.advertise<sensor_msgs::PointCloud2>("truth_output",1);
    ros::Publisher boden_truth_pub=nh.advertise<sensor_msgs::PointCloud2>("boden_truth_output",1);
    ros::Publisher boden_transformed_pub=nh.advertise<sensor_msgs::PointCloud2>("boden_transformed_output",1);
    ros::Publisher unmateched_boxes_pub=nh.advertise<visualization_msgs::MarkerArray>("unmatched_boxes_output",0);
    ros::Publisher matched_boxes_pub=nh.advertise<visualization_msgs::MarkerArray>("matched_boxes_output",0);
    ros::Publisher truth_boxes_pub=nh.advertise<visualization_msgs::MarkerArray>("truth_boxes_output",0);
    ros::Publisher boden_truth_boxes_pub=nh.advertise<visualization_msgs::MarkerArray>("boden_truth_boxes_output",0);
    ros::Publisher boden_transformed_boxes_pub=nh.advertise<visualization_msgs::MarkerArray>("boden_transformed_boxes_output",0);

    sensor_msgs::PointCloud2 output1;
    sensor_msgs::PointCloud2 output2;
    sensor_msgs::PointCloud2 output3;
    sensor_msgs::PointCloud2 output4;
    sensor_msgs::PointCloud2 output5;
    visualization_msgs::MarkerArray  temp1;
    visualization_msgs::MarkerArray  temp2;
    visualization_msgs::MarkerArray  temp3;
    visualization_msgs::MarkerArray  temp4;
    visualization_msgs::MarkerArray  temp5;

    pcl::toROSMsg(matched,output1);
    pcl::toROSMsg(unmatched,output2);
    pcl::toROSMsg(need_show_truth,output3);
    pcl::toROSMsg(boden_truth,output4);
    pcl::toROSMsg(boden_transformed,output5);
    output1.header.frame_id="map";
    output2.header.frame_id="map";
    output3.header.frame_id="map";
    output4.header.frame_id="map";
    output5.header.frame_id="map";
    ros::Rate loop_rate(1);

//    int count=3000;
//    for(int i=0;i<count;i++)
//    {
//        temp1.markers.emplace_back(unmatched_marker_array.markers[i]);
//        temp2.markers.emplace_back(matched_marker_array.markers[i]);
//        temp3.markers.emplace_back(truth_marker_array.markers[i]);
//        temp4.markers.emplace_back(boden_truth_marker_array.markers[i]);
//        temp5.markers.emplace_back(boden_transformed_marker_array.markers[i]);
//    }
    while(ros::ok())
    {
        transformed_matched_pub.publish(output1);
        transformed_unmatched_pub.publish(output2);
        truth_pub.publish(output3);
        boden_truth_pub.publish(output4);
        boden_transformed_pub.publish(output5);
        unmateched_boxes_pub.publish(unmatched_marker_array);
        matched_boxes_pub.publish(matched_marker_array);
        truth_boxes_pub.publish(truth_marker_array);
        boden_truth_boxes_pub.publish(boden_truth_marker_array);
        boden_transformed_boxes_pub.publish(boden_transformed_marker_array);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
//
// Created by jeffqjn on 12.03.21.
//

