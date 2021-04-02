//
// Created by noah on 18.01.21.
//
#include <IMU.h>
IntervalMatrix IMU::vel2rotatation(double start_compute_time, double  end_compute_time)
{
    int stamp_index=vel_data.size()-1;
    Interval current_stamp;
    IntervalVector current_vel(3);
    IntervalMatrix overall_rotation=Matrix::eye(3);
    double delta_t;

    int start_index=-1;
    int end_index=-1;
    //find the corresponding stamp of imu

    //find LiDAR cloud time in imu data
    for(int i=0 ;i <vel_data.size();i++)
    {
        if(vel_data[i].first.contains(start_compute_time))
        {
            start_index=i;
        }
        else if(vel_data[i].first.contains(end_compute_time))
        {
            end_index=i;
        }
        if(start_index!=-1 && end_index!=-1)
        {
            break;
        }

    }
    for (int i=start_index;i<=end_index;i++)
    {
        delta_t=min(vel_data[i].first.ub(),end_compute_time)-max(start_compute_time,vel_data[i].first.lb());
        IntervalMatrix Matrix_temp(3,3);
        Matrix_temp=calculate_rodrigues_rotation(vel_data[i].second,delta_t);
        Matrix_temp &=IntervalMatrix(3,3,Interval(-1,1));
        overall_rotation*=Matrix_temp;
    }
    overall_rotation &=IntervalMatrix(3,3,Interval(-1,1));
    return overall_rotation;
}

Eigen::Vector3d IMU::acc2pose(double start_compute_time, double  end_compute_time, Eigen::Vector3d start_vel)
{
    //attitude of the vehicle,each step use angular velocity to update the matrix, represent the transform between body frame and global frame
    Eigen::Matrix3d C=Eigen::Matrix3d::Identity();
    Eigen::Matrix3d B=Eigen::Matrix3d::Identity();
    Eigen::Matrix3d B2=Eigen::Matrix3d::Identity();
    Eigen::Matrix3d temp=Eigen::Matrix3d::Identity();
    double delta;
    double dt;
    double term_a;
    double term_b;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d acc_body;
    Eigen::Vector3d acc_global;
    Eigen::Vector3d acc_calib;
    Eigen::Vector3d acc_truth;
    Eigen::Vector3d g;
    Eigen::Vector3d v;
    Eigen::Vector3d x;
    //Initialization
    v=start_vel;

    std::map<double, ibex::Vector> a_map;
    std::map<double, ibex::Vector> v_map;
    ibex::Vector a_value(3);
    ibex::Vector v_value(3);
    double index_a=0;
    double index_v=0;
    vibes::beginDrawing();
    codac::VIBesFigTube tube_ax("a_x");
    codac::VIBesFigTube tube_ay("a_y");
    codac::VIBesFigTube tube_az("a_z");
    codac::VIBesFigTube tube_vx("v_x");
    codac::VIBesFigTube tube_vy("v_y");
    codac::VIBesFigTube tube_vz("v_z");
    x[0]=0;
    x[1]=0;
    x[2]=0;
    acc_calib=set_calibration();
    for(int i=0;i<acc_actual_data.size();i++)
    {
        dt=vel_actual_data[i].first.ub()-vel_actual_data[i].first.lb();
        delta=sqrt(pow(vel_actual_data[i].second[0]*dt,2)+pow(vel_actual_data[i].second[1]*dt,2)+pow(vel_actual_data[i].second[2]*dt,2));
        //set up matrix B
        B(0,0)=0;
        B(1,1)=0;
        B(2,2)=0;
        B(0,1)=-vel_actual_data[i].second[2]*dt;
        B(0,2)=vel_actual_data[i].second[1]*dt;
        B(1,0)=vel_actual_data[i].second[2]*dt;
        B(1,2)=-vel_actual_data[i].second[0]*dt;
        B(2,0)=-vel_actual_data[i].second[1]*dt;
        B(2,1)=vel_actual_data[i].second[0]*dt;

        //square the matrix B
        B2(0,0)=B(0,1)*B(1,0)+B(0,2)*B(2,0);
        B2(0,1)=B(2,1)*B(0,2);
        B2(0,2)=B(1,2)*B(0,1);
        B2(1,0)=B(1,2)*B(2,0);
        B2(1,1)=B(0,1)*B(1,0)+B(1,2)*B(2,1);
        B2(1,2)=B(1,0)*B(0,2);
        B2(2,0)=B(2,1)*B(1,0);
        B2(2,1)=B(2,0)*B(0,1);
        B2(2,2)=B(2,0)*B(0,2)+B(1,2)*B(2,1);
        term_a=sin(delta)/delta;
        term_b=(1-cos(delta))/pow(delta,2);
        C=C*(Eigen::Matrix3d::Identity()+term_a*B+term_b*B2);

        if(vel_actual_data[i].first.ub()>start_compute_time)
        {
            if(vel_actual_data[i].first.contains(start_compute_time))
            {
                dt=vel_actual_data[i].first.ub()-start_compute_time;
            }
            else if(vel_actual_data[i].first.contains(end_compute_time))
            {
                dt=end_compute_time-vel_actual_data[i].first.lb();
            }
            else
            {
                dt=vel_actual_data[i].first.ub()-vel_actual_data[i].first.lb();
            }
            //Dealing with acceleration
            acc_body=acc_actual_data[i].second;
            acc_global=C*acc_body;
            acc_truth=acc_global-acc_calib;
            x=x+dt*v;

            a_value[0]=acc_body[0];
            a_value[1]=acc_body[1];
            a_value[2]=acc_body[2];

            v_value[0]=v[0];
            v_value[1]=v[1];
            v_value[2]=v[2];
            a_map.insert(make_pair(index_a,a_value));
            v_map.insert(make_pair(index_v,v_value));
            index_a++;
            index_v++;

            if(vel_actual_data[i].first.contains(end_compute_time))
            {break;}
        }
    }

    codac::TrajectoryVector truth_trajectory_a(a_map);
    codac::TrajectoryVector truth_trajectory_v(v_map);
    tube_ax.add_trajectory(&truth_trajectory_a[0],"a_x");
    tube_ay.add_trajectory(&truth_trajectory_a[1],"a_y");
    tube_az.add_trajectory(&truth_trajectory_a[2],"a_z");

    tube_vx.add_trajectory(&truth_trajectory_v[0],"v_x");
    tube_vy.add_trajectory(&truth_trajectory_v[1],"v_y");
    tube_vz.add_trajectory(&truth_trajectory_v[2],"v_z");

    tube_ax.show();
    tube_ay.show();
    tube_az.show();
    tube_vx.show();
    tube_vy.show();
    tube_vz.show();

    vibes::endDrawing();
    cout<<x<<endl;
    cout<<v<<endl;
    cout<<endl;
    return x;
}
Eigen::Vector3d IMU::set_calibration()
{
    Eigen::Vector3d calib;
    calib[0]=-0.291576;
    calib[1]=-0.383224;
    calib[2]=-9.83375;
    int count=0;
//    for(int i=46366;i<=46683;i++) {
//        calib += acc_actual_data[i].second;
//        count++;
//    }
//    calib=calib/count;
//    cout<<calib<<endl;

    return calib;
}
void IMU::add_vel_measurement(sensor_msgs::ImuConstPtr  imupointer, Parameters & parameters) {

    //In order to get the interval of current timestamp
    double start_current_timestamp, end_current_timestamp;
    double GYRO_SCALE_FACTOR= parameters.get_GYRO_SCALE_FACTOR();
    double GYRO_BIAS= parameters.get_GYRO_BIAS();
    double GYRO_WALKING_BIAS= parameters.get_Gyro_WALKING_BIAS();

    double ACC_SCALE_FACTOR= parameters.get_ACC_SCALE_FACTOR();
    double ACC_BIAS= parameters.get_ACC_BIAS();
    double ACC_WALKING_BIAS= parameters.get_ACC_WALKING_BIAS();

    double start_time=parameters.get_START_COMPUTE_TIME();
    double end_time=parameters.get_END_COMPUTE_TIME();

    Eigen::Vector3d acceleration=Eigen::Vector3d::Identity();
    Eigen::Vector3d angular_velocity=Eigen::Vector3d::Identity();




        //Introduce noise, measurement noise ignored
        //Gyroscope
        Interval gyro_scale_factor = (-GYRO_SCALE_FACTOR, GYRO_SCALE_FACTOR);
        IntervalVector gyro_bias(3, Interval(-GYRO_BIAS, GYRO_BIAS));
        IntervalVector gyro_walking_bias(3, Interval(-GYRO_WALKING_BIAS, GYRO_WALKING_BIAS));
        IntervalVector actual_vel(3);
        //add noise to each component
        actual_vel[0] = imupointer->angular_velocity.x + gyro_scale_factor * imupointer->angular_velocity.x + gyro_bias[0]/* +
                        gyro_walking_bias[0] * (end_current_timestamp - start_time)*/;
        actual_vel[1] = imupointer->angular_velocity.y + gyro_scale_factor * imupointer->angular_velocity.y + gyro_bias[1]/* +
                        gyro_walking_bias[1] * (end_current_timestamp - start_time)*/;
        actual_vel[2] = imupointer->angular_velocity.z + gyro_scale_factor * imupointer->angular_velocity.z + gyro_bias[2] /*+
                        gyro_walking_bias[2] * (end_current_timestamp - start_time)*/;

        //add raw data to vel_actual_data
        angular_velocity[0]=imupointer->angular_velocity.x;
        angular_velocity[1]=imupointer->angular_velocity.y;
        angular_velocity[2]=imupointer->angular_velocity.z;


        //Accelerometer
        Interval acc_scale_factor = (-ACC_SCALE_FACTOR, ACC_SCALE_FACTOR);
        IntervalVector acc_bias(3, Interval(-ACC_BIAS, ACC_BIAS));
        IntervalVector acc_walking_bias(3, Interval(-ACC_WALKING_BIAS, ACC_WALKING_BIAS));
        IntervalVector actual_acc(3);
        //add noise to each component
        actual_acc[0] = imupointer->linear_acceleration.x + acc_scale_factor * imupointer->linear_acceleration.x + acc_bias[0]/* +
                        acc_walking_bias[0] * (end_current_timestamp - start_time)*/;
        actual_acc[1] = imupointer->linear_acceleration.y + acc_scale_factor * imupointer->linear_acceleration.y + acc_bias[1]/* +
                        acc_walking_bias[1] * (end_current_timestamp - start_time)*/;
        actual_acc[2] = imupointer->linear_acceleration.z + acc_scale_factor * imupointer->linear_acceleration.z + acc_bias[2] /*+
                        acc_walking_bias[2] * (end_current_timestamp - start_time)*/;


        //add raw data to acc_actual_data
        acceleration[0]=imupointer->linear_acceleration.x;
        acceleration[1]=imupointer->linear_acceleration.y;
        acceleration[2]=imupointer->linear_acceleration.z;

    if(imupointer->header.stamp.toSec()-start_time>0 && imupointer->header.stamp.toSec()-start_time<0.00200 &&imupointer->header.stamp.toSec()<end_time)
    {
        this->vel_data.emplace_back(make_pair(Interval(imupointer->header.stamp.toSec()-0.00200, imupointer->header.stamp.toSec()), actual_vel));
        this->vel_actual_data.emplace_back(make_pair(Interval(imupointer->header.stamp.toSec()-0.00200, imupointer->header.stamp.toSec()),angular_velocity));
        this->acc_data.emplace_back(make_pair(Interval(imupointer->header.stamp.toSec()-0.00200, imupointer->header.stamp.toSec()), actual_acc));
        this->acc_actual_data.emplace_back(make_pair(Interval(imupointer->header.stamp.toSec()-0.00200, imupointer->header.stamp.toSec()),acceleration));
    }
    else if (imupointer->header.stamp.toSec()-start_time>0.00200 && imupointer->header.stamp.toSec()<end_time)
    {
        this->vel_data.emplace_back(make_pair(Interval(vel_data.back().first.ub(), imupointer->header.stamp.toSec()), actual_vel));
        this->vel_actual_data.emplace_back(make_pair(Interval(vel_actual_data.back().first.ub(), imupointer->header.stamp.toSec()),angular_velocity));
        this->acc_data.emplace_back(make_pair(Interval(acc_data.back().first.ub(), imupointer->header.stamp.toSec()), actual_acc));
        this->acc_actual_data.emplace_back(make_pair(Interval(acc_actual_data.back().first.ub(), imupointer->header.stamp.toSec()),acceleration));
    }
    else if (imupointer->header.stamp.toSec()-end_time<0.00200 && imupointer->header.stamp.toSec()-end_time>0)
    {
        this->vel_data.emplace_back(make_pair(Interval(vel_data.back().first.ub(), imupointer->header.stamp.toSec()), actual_vel));
        this->vel_actual_data.emplace_back(make_pair(Interval(vel_actual_data.back().first.ub(), imupointer->header.stamp.toSec()),angular_velocity));
        this->acc_data.emplace_back(make_pair(Interval(acc_data.back().first.ub(), imupointer->header.stamp.toSec()), actual_acc));
        this->acc_actual_data.emplace_back(make_pair(Interval(acc_actual_data.back().first.ub(), imupointer->header.stamp.toSec()),acceleration));
    }

}

void IMU::calculate_ground_truth(Parameters parameters,LiDAR_PointCloud pointCloud,Eigen::Matrix4d transform1,Eigen::Matrix4d transform2)
{
    double start_compute_time=parameters.get_START_COMPUTE_TIME();
    double end_compute_time=parameters.get_END_COMPUTE_TIME();
//    set_index(pointCloud,parameters);
//    cout<<start_compute_time_index<<endl;
//    cout<<end_compute_time_index<<endl;
//    get_start_vel();
//acc2pose_tube(1557924740.000799894, 1557924743.332916);
//acc2pose(1557924740.0001,1557924776.004964);

    // 1557924753.332916

}

codac::TubeVector IMU::acc2pose_tube(double start_compute_time, double  end_compute_time,IntervalVector calib,IntervalVector start_vel)
{
    Interval tdomain(start_compute_time,end_compute_time);
    double dt;
    //vector of tdomain
    vector<Interval> v_tdomains;
    //vector of value domain
    vector<IntervalVector> v_codomains_a;
    vector<IntervalVector> v_codomains_v;
    vector<IntervalVector> v_codomains_x;
    IntervalVector velocity_init(3,Interval(-2.7,+13.8));
    IntervalVector null_init(3,Interval(-0.0001,0.0001));
    IntervalVector position_init(3,Interval(-100,+100));
    IntervalVector acc_calib(3);
    IntervalMatrix C(3,3,Interval(0));
    IntervalMatrix C_Next(3,3,Interval(0));
    C[0][0]=Interval(1);
    C[1][1]=Interval(1);
    C[2][2]=Interval(1);
    IntervalVector vel;
    Interval term_a;
    Interval term_b;
    Interval theta;
    IntervalVector erg(3,Interval(0));



    for(int i=0;i<acc_data.size();i++)
    {
        //dt=acc_data[i].first.ub()-acc_data[i].first.lb();
        if(acc_data[i].first.ub()>=start_compute_time)
        {
            if(acc_data[i].first.contains(start_compute_time))
            {
                theta=Interval(start_compute_time,acc_data[i].first.ub());
                dt=acc_data[i].first.ub()-start_compute_time;
                C_Next=calculate_rodrigues_rotation(vel_data[i].second,dt);
            }
            else if (acc_data[i].first.contains(end_compute_time))
            {
                theta=Interval(acc_data[i].first.lb(),end_compute_time);
                dt=end_compute_time-acc_data[i].first.lb();
                C_Next=calculate_rodrigues_rotation(vel_data[i].second,dt);
            }
            else
            {
                theta=Interval(acc_data[i].first.lb(),acc_data[i].first.ub());
                dt=acc_data[i].first.ub()-acc_data[i].first.lb();
                C_Next=calculate_rodrigues_rotation(vel_data[i].second,dt);
            }
            C*=C_Next;
            acc_calib=C*acc_data[i].second-calib;

            v_tdomains.emplace_back(theta);
            v_codomains_a.emplace_back(acc_calib);
        }
        if(acc_data[i].first.contains(end_compute_time))
        {
            break;
        }
    }
    v_codomains_v.emplace_back(start_vel);
    for(int i=1;i<v_codomains_a.size();i++)
    {
        v_codomains_v.emplace_back(velocity_init);
    }
    v_codomains_x.emplace_back(null_init);
    for(int i=1;i<v_codomains_a.size();i++)
    {
        v_codomains_x.emplace_back(position_init);
    }
    codac::TubeVector a(v_tdomains,v_codomains_a);
    codac::TubeVector v(v_tdomains,v_codomains_v);
    codac::TubeVector x(v_tdomains,v_codomains_x);
    codac::ctc::deriv.contract(v,a,codac::TimePropag::FORWARD);
    codac::ctc::deriv.contract(x,v,codac::TimePropag::FORWARD);
    cout<<*x[0].slice(v[0].nb_slices()-1)<<endl;
    cout<<*x[1].slice(v[0].nb_slices()-1)<<endl;
    cout<<*x[2].slice(v[0].nb_slices()-1)<<endl;
    return a;

}
IntervalVector IMU::set_calibration_interval()
{
    IntervalVector calib_temp(3,Interval(0));
    //calib_temp[0]=Interval(-0.3268308888790001, -0.306830888879);
    //calib_temp[1]=Interval(0.1093293319684999, 0.1293293319685001);
    //calib_temp[2]=Interval(-9.872295298765004, -9.852295298764998);
    calib_temp[0]=Interval(-0.3152546158);
    calib_temp[1]=Interval(0.1187356537);
    calib_temp[2]=Interval(-9.813229153);
    return calib_temp;
}
void IMU::debug(Measurement m)
{
    IntervalVector calib_temp(3,Interval(0));
    double start_compute_time=1557924740.00383;
    double end_compute_time=1557924743.000799894;
    double dt;
    Eigen::Vector3d truth;
    Eigen::Vector3d truth1;
    Eigen::Vector3d start_vel;
    IntervalVector start_vel_interval(3,Interval(-0.0001,0.0001));

    start_vel[0]=0;
    start_vel[1]=0;
    start_vel[2]=0;
    //find the best calib
//    for(int i=46366;i<=46683;i++)
//    {
//        calib_temp+=acc_data[i].second;
//    }
//    calib_temp[0]=calib_temp[0]/318;
//    calib_temp[1]=calib_temp[1]/318;
//    calib_temp[2]=calib_temp[2]/318;
//    cout<<calib_temp<<endl;
//    set_calibration();
    //here we need to find the calibration data

    truth1=acc2pose(start_compute_time,end_compute_time,start_vel);
    codac::TubeVector erg=acc2pose_tube(start_compute_time, end_compute_time,set_calibration_interval(),start_vel_interval);
    bool result=if_contain(erg,start_compute_time,end_compute_time,m);

}
bool IMU::if_contain(codac::TubeVector x, double start_compute_time , double end_compute_time, Measurement m)
{
    int dt=0;
    double index=0;
    double index1=0;
    int incresment=1;
    IntervalVector box_x;
    IntervalVector box_y;
    IntervalVector box_z;
ibex::Vector truth(3);
    ibex::Vector truth_acc(3);
   vibes::beginDrawing();
    codac::VIBesFigTube tube_x("tube_x");
    codac::VIBesFigTube tube_y("tube_y");
    codac::VIBesFigTube tube_z("tube_z");
    tube_x.set_properties(100,100,600,300);
    tube_y.set_properties(100,100,600,300);
    tube_z.set_properties(100,100,600,300);

   std::map<double, ibex::Vector> temp;
    std::map<double, ibex::Vector> temp2;
//   tube.draw_box()
//   tube.add_tube(&x[0],"tube_x");
    for(int i=0;i<x[0].nb_slices();i++)
    {
        box_x=x[0].slice(i)->box();
        box_y=x[1].slice(i)->box();
        box_z=x[2].slice(i)->box();
        for (int j=0;j<m.ground_truth.size();j++)
        {
            if(m.ground_truth[j].first.contains(box_x[0].ub()))
            {
                truth[0]=m.ground_truth[j].second.pose.position.x;
                truth[1]=m.ground_truth[j].second.pose.position.y;
                truth[2]=m.ground_truth[j].second.pose.position.z;
                temp.insert(make_pair(index,truth));
                index++;
                break;
            }
        }
        box_x[0]=Interval(dt,dt+incresment);
        box_y[0]=Interval(dt,dt+incresment);
        box_z[0]=Interval(dt,dt+incresment);

        dt++;

        tube_x.draw_box(box_x,"lightGray");
        tube_y.draw_box(box_y,"lightGray");
        tube_z.draw_box(box_z,"lightGray");
    }
    tube_x.show();
    tube_y.show();
    tube_z.show();

    for(int i=0;i<acc_actual_data.size();i++)
    {
        truth_acc[0]=acc_actual_data[i].second[0];
        truth_acc[1]=acc_actual_data[i].second[1];
        truth_acc[2]=acc_actual_data[i].second[2];
        temp2.insert(make_pair(index1,truth_acc));
        index1++;

    }
    codac::TrajectoryVector truth_trajectory(temp);
    codac::TrajectoryVector truth_acc_trajectory(temp2);
    tube_x.add_trajectory(&truth_trajectory[0],"truth_traj");
    tube_y.add_trajectory(&truth_trajectory[1],"truth_traj");
    tube_z.add_trajectory(&truth_trajectory[2],"truth_traj");
    tube_x.add_trajectory(&truth_acc_trajectory[0],"truth_traj");
    tube_y.add_trajectory(&truth_acc_trajectory[1],"truth_traj");
    tube_z.add_trajectory(&truth_acc_trajectory[2],"truth_traj");
    //tube.axis_limits(start_compute_time,end_compute_time,-5,0);
    tube_x.show();
    tube_y.show();
    tube_z.show();
   vibes::endDrawing();
    return true;

}
 void IMU::toEulerAngle(const Eigen::Quaterniond & q, double & roll, double & pitch , double & yaw)
{
    //roll
    double sinr_cosp=+2.0*(q.w()*q.x()+q.y()*q.z());
    double cosr_cosp=+1.0-2.0*(q.x()*q.x()+q.y()*q.y());
    roll=atan2(sinr_cosp,cosr_cosp)*180/M_PI;

    //pitch
    double sinp=+2.0*(q.w()*q.y()-q.z()*q.x());
    if (fabs(sinp)>=1)
    {
        pitch=copysign(M_PI/2,sinp)*180/M_PI;
    }
    else
    {
        pitch=asin(sinp);
    }

    //yaw
    double siny_cosp=+2.0*(q.w()*q.z()+q.x()*q.y());
    double cosy_cosp=+1.0-2.0*(q.y()*q.y()+q.z()*q.z());
    yaw=atan2(siny_cosp,cosy_cosp)*180/M_PI;
}

IntervalMatrix IMU::calculate_rodrigues_rotation(IntervalVector angular_vel, double delta_t)
{
    IntervalMatrix skew_symmetric_matrix(3,3,0.0);
    IntervalMatrix skew_symmetric_matrix2(3,3,0.0);
    Interval theta;
    Interval term_a;
    Interval term_b;
    angular_vel[0]=angular_vel[0]*delta_t;
    angular_vel[1]=angular_vel[1]*delta_t;
    angular_vel[2]=angular_vel[2]*delta_t;
    //set up skew-symmetric matrix
    skew_symmetric_matrix[0][1]= -angular_vel[2];
    skew_symmetric_matrix[0][2]= angular_vel[1];
    skew_symmetric_matrix[1][0]= angular_vel[2];
    skew_symmetric_matrix[1][2]= -angular_vel[0];
    skew_symmetric_matrix[2][0]= -angular_vel[1];
    skew_symmetric_matrix[2][1]= angular_vel[0];
    skew_symmetric_matrix[0][0]= 0;
    skew_symmetric_matrix[1][1]= 0;
    skew_symmetric_matrix[2][2]= 0;

    //square the skew-symmetric matrix
    skew_symmetric_matrix2[0][0]=-sqr(angular_vel[2])-sqr(angular_vel[1]);
    skew_symmetric_matrix2[1][1]=-sqr(angular_vel[2])-sqr(angular_vel[0]);
    skew_symmetric_matrix2[2][2]=-sqr(angular_vel[1])-sqr(angular_vel[0]);
    skew_symmetric_matrix2[0][1]=angular_vel[0]*angular_vel[1];
    skew_symmetric_matrix2[1][0]=angular_vel[0]*angular_vel[1];
    skew_symmetric_matrix2[0][2]=angular_vel[0]*angular_vel[2];
    skew_symmetric_matrix2[2][0]=angular_vel[0]*angular_vel[2];
    skew_symmetric_matrix2[1][2]=angular_vel[2]*angular_vel[1];
    skew_symmetric_matrix2[2][1]=angular_vel[2]*angular_vel[1];

    theta=sqrt(sqr(angular_vel[0])+sqr(angular_vel[1])+sqr(angular_vel[2]));

    double term_a_lb=sin(theta.ub())/theta.ub();
    //Situation 1 when the lb of term a greater then the maximal feasible vaule of sin(theta)/theta
    if(term_a_lb>=1.)
    {
        term_a=Interval(1.);
    }
    else
    {
        //the maximum is at x=0, so need to know contain 0?
        if (theta.contains(0))
        {
            term_a=Interval(term_a_lb,1.);
        }
        else
        {
            term_a=Interval(term_a_lb,sin(theta.lb())/theta.lb());
        }
    }
    double term_b_lb=2*(sin(theta.ub()/2)*sin(theta.ub()/2))/(theta.ub()*theta.ub());
    //Situation 1 when the lb of term a greater then the maximal feasible vaule
    if(term_b_lb>=0.5)
    {
        term_b=Interval(0.5);
    }
    else
    {
        //the maximum is at x=0, so need to know contain 0?
        if (theta.contains(0.0))
        {
            term_b = Interval(term_b_lb, 0.5);
        }
        else
        {
            if(term_b_lb>(2*sin(theta.lb()/2)*sin(theta.lb()/2))/(theta.lb()*theta.lb()))
            {
                term_b=Interval((2*sin(theta.lb()/2)*sin(theta.lb()/2))/(theta.lb()*theta.lb()),term_b_lb);
            }
            else {
                term_b = Interval(term_b_lb,
                                  (2 * sin(theta.lb() / 2) * sin(theta.lb() / 2)) / (theta.lb() * theta.lb()));
            }
        }
    }
    IntervalMatrix eye=Matrix::eye(3);
    IntervalMatrix R=eye+term_a*skew_symmetric_matrix+term_b*skew_symmetric_matrix2;
    return R;
}