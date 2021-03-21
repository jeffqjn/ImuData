#include "EF.h"

void EF::add_velocity(const geometry_msgs::Vector3StampedConstPtr& v, Parameters parameters){

    double N_uc,E_uc,D_uc;
    IntervalVector NED_vel(3,Interval(0));

        parameters.get_NED_Velocity_uncertainty(N_uc,E_uc,D_uc);
        NED_vel[0]=Interval(v->vector.x-N_uc,v->vector.x+N_uc);
        NED_vel[1]=Interval(v->vector.y-E_uc,v->vector.y+E_uc);
        NED_vel[2]=Interval(v->vector.z-D_uc,v->vector.z+D_uc);
        cout<<NED_vel[2]<<endl;
        if (velocity_NED.empty())
        {
            velocity_NED.emplace_back(make_pair(Interval(v->header.stamp.toSec()),NED_vel));
        }
        else
        {
            velocity_NED.emplace_back(make_pair(Interval(velocity_NED.back().first.ub(),v->header.stamp.toSec()),NED_vel));
        }

}
void EF::add_LLH(const geometry_msgs::Vector3StampedConstPtr& v,Parameters parameters)
{
    double Latitude_uc, Longitude_uc, Height_uc;
    parameters.get_LLH_uncertainty(Latitude_uc,Longitude_uc,Height_uc);
    IntervalVector LLH(3,Interval(0));
    LLH[0]=Interval(v->vector.x-Latitude_uc,v->vector.x+Latitude_uc);
    LLH[1]=Interval(v->vector.y-Longitude_uc,v->vector.y+Longitude_uc);
    LLH[2]=Interval(v->vector.z-Height_uc,v->vector.z+Height_uc);
//    Eigen::Vector3d LLH;
//    LLH[0]=v->vector.x;
//    LLH[1]=v->vector.y;
//    LLH[2]=v->vector.z;
    if (LLH_Position.empty())
    {
        LLH_Position.emplace_back(make_pair(Interval(v->header.stamp.toSec()),LLH));
    }
    else
    {
        LLH_Position.emplace_back(make_pair(Interval(LLH_Position.back().first.ub(),v->header.stamp.toSec()),LLH));
    }

}

void EF::add_quaternion(const geometry_msgs::QuaternionStampedConstPtr& q, Parameters parameters){
    Eigen::Quaterniond quaterniond;
    double start_time, end_time;
    start_time=parameters.get_START_COMPUTE_TIME();
    end_time=parameters.get_END_COMPUTE_TIME();
    //parameters.get_bag_start_end_time(start_time,end_time);

        quaterniond.w()=q->quaternion.w;
        quaterniond.x()=q->quaternion.x;
        quaterniond.y()=q->quaternion.y;
        quaterniond.z()=q->quaternion.z;
        if(quaternion.empty())
        {
            quaternion.emplace_back(make_pair(Interval(q->header.stamp.toSec()),quaterniond));
        }
        else
        {
            quaternion.emplace_back(make_pair(Interval(quaternion.back().first.ub(),q->header.stamp.toSec()),quaterniond));
        }


}
void EF::ned_vel_transform(Parameters parameters)
{
    double start_time=parameters.get_START_COMPUTE_TIME();
    double end_time=parameters.get_END_COMPUTE_TIME();
    double time;
    double e=0.08181919;
    Interval N_e;
    int start_index=-1;
    int end_index=-1;
    Eigen::Quaterniond q;
    Eigen::Quaterniond transform;
    IntervalVector vel(3);
    IntervalMatrix transform_matrix(3,3);


    double start_compute_time = parameters.get_START_COMPUTE_TIME();
    double end_compute_time = parameters.get_END_COMPUTE_TIME();

    Interval tdomain(start_compute_time, end_compute_time);

    int dt=0;
    int incresment=1;
    double index=0;
    vibes::beginDrawing();
    tubex::VIBesFigTube tube_x("tube_x");
    tubex::VIBesFigTube tube_y("tube_y");
    tubex::VIBesFigTube tube_z("tube_z");
    tube_x.set_properties(100,100,600,300);
    tube_y.set_properties(100,100,600,300);
    tube_z.set_properties(100,100,600,300);
    std::map<double, ibex::Vector> temp;
    IntervalVector box_x;
    IntervalVector box_y;
    IntervalVector box_z;
    ibex::Vector truth(3);

    //vector of tdomain
    vector<Interval> v_tdomains;
    vector<IntervalVector> v_codomains_v;
    vector<IntervalVector> v_codomains_x;
    IntervalVector null_init(3, Interval(-0.0001, 0.0001));
    IntervalVector position_init(3, Interval(-100, +100));

    for (int i=0;i<velocity_NED.size();i++)
    {
        if (velocity_NED[i].first.contains(start_time))
        {
            start_index=i;
        }
        else if (velocity_NED[i].first.contains(end_time))
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
        if (i==start_index)
        {
            transform=quaternion[i].second;
            transform_matrix=calculate_transform_matrix(transform);
            vel=transform_matrix*velocity_NED[i].second;
            v_tdomains.emplace_back(Interval(start_time,velocity_NED[i].first.ub()));
            v_codomains_v.emplace_back(vel);
        }
        else if (i==end_index)
        {
            vel=transform_matrix*velocity_NED[i].second;
            v_tdomains.emplace_back(Interval(velocity_NED[i].first.lb(),end_time));
            v_codomains_v.emplace_back(vel);
        }
        else
        {
            vel=transform_matrix*velocity_NED[i].second;
            v_tdomains.emplace_back(velocity_NED[i].first);
            v_codomains_v.emplace_back(vel);
        }
    }

    for (auto item : velocity) {
        v_tdomains.emplace_back(item.first);
        v_codomains_v.emplace_back(item.second);
    }
    v_codomains_x.emplace_back(null_init);
    for (; v_codomains_x.size() < v_codomains_v.size();) {
        v_codomains_x.emplace_back(position_init);
    }

    tubex::TubeVector v(v_tdomains, v_codomains_v);
    tubex::TubeVector x(v_tdomains, v_codomains_x);


    tubex::ctc::deriv.contract(x, v,tubex::TimePropag::FORWARD);


    for(int i=0;i<x[0].nb_slices();i++)
    {
        box_x=x[0].slice(i)->box();
        box_y=x[1].slice(i)->box();
        box_z=x[2].slice(i)->box();
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
    
}
IntervalMatrix EF::calculate_transform_matrix(Eigen::Quaterniond q)
{
    Eigen::Matrix3d matrix;
    Eigen::Quaterniond q_inverse;
    q_inverse=q;
    IntervalVector q_interval(4);
    IntervalMatrix transform_matrix(3,3);
    q_interval[0]=(q_inverse.w()-0.1,q_inverse.w()+0.1);
    q_interval[1]=(q_inverse.x()-0.1,q_inverse.x()+0.1);
    q_interval[2]=(q_inverse.y()-0.1,q_inverse.y()+0.1);
    q_interval[3]=(q_inverse.z()-0.1,q_inverse.z()+0.1);

    transform_matrix[0][0]=2*(q_interval[0]*q_interval[0]+q_interval[1]*q_interval[1])-1;
    transform_matrix[0][1]=2*(q_interval[1]*q_interval[2]-q_interval[0]*q_interval[3]);
    transform_matrix[0][2]=2*(q_interval[1]*q_interval[3]+q_interval[0]*q_interval[2]);
    transform_matrix[1][0]=2*(q_interval[1]*q_interval[2]+q_interval[0]*q_interval[3]);
    transform_matrix[1][1]=2*(q_interval[0]*q_interval[0]+q_interval[2]*q_interval[2])-1;
    transform_matrix[1][2]=2*(q_interval[2]*q_interval[3]-q_interval[0]*q_interval[1]);
    transform_matrix[2][0]=2*(q_interval[1]*q_interval[3]-q_interval[0]*q_interval[2]);
    transform_matrix[2][1]=2*(q_interval[2]*q_interval[3]+q_interval[0]*q_interval[1]);
    transform_matrix[2][2]=2*(q_interval[0]*q_interval[0]+q_interval[3]*q_interval[3])-1;
    //IntervalMatrix inverse_transform_matrix=calculate_inverse_matrix(transform_matrix);

    return transform_matrix;

}
IntervalVector EF::state_estimation(Parameters parameters,Measurement m) {
    vector<Interval> v_tdomains;
    vector<IntervalVector> v_codomains_v;
    vector<IntervalVector> v_codomains_x;
    IntervalVector null_init(3, Interval(-0.0001, 0.0001));
    IntervalVector position_init(3, Interval(-100, +100));
    double start_time,end_time;
    int start_index=-1,end_index=-1;
    IntervalVector vel(3);

    IntervalVector ECEF_ref(3);
    IntervalMatrix ned_ecef(3,3);
    Eigen::Matrix3d ned_ecef_inverse=Eigen::Matrix3d::Identity();
    IntervalMatrix ned_ecef_inverse_interval(3,3);
    start_time=parameters.get_START_COMPUTE_TIME();
    end_time=parameters.get_END_COMPUTE_TIME();
    Eigen::Quaterniond target_quaternion;
    IntervalMatrix inverse_transform(3,3);

    //get indexs of start_compute_time, end_compute_time
    for (int i=0;i<velocity_NED.size();i++)
    {
        if (velocity_NED[i].first.contains(start_time))
        {
            start_index=i;
        }
        else if (velocity_NED[i].first.contains(end_time))
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
        if (i==start_index)
        {
            vel=velocity_NED[i].second;
            v_tdomains.emplace_back(Interval(start_time,velocity_NED[i].first.ub()));
            v_codomains_v.emplace_back(vel);
        }
        else if (i==end_index)
        {
            vel=velocity_NED[i].second;
            v_tdomains.emplace_back(Interval(velocity_NED[i].first.lb(),end_time));
            v_codomains_v.emplace_back(vel);
        }
        else
        {
            vel=velocity_NED[i].second;
            v_tdomains.emplace_back(velocity_NED[i].first);
            v_codomains_v.emplace_back(vel);
        }
    }

    v_codomains_x.emplace_back(null_init);
    for (; v_codomains_x.size() < v_codomains_v.size();) {
        v_codomains_x.emplace_back(position_init);
    }
    tubex::TubeVector v_ned(v_tdomains, v_codomains_v);
    tubex::TubeVector x_ned(v_tdomains, v_codomains_x);

    //integration the NED Velocity -> NED Position
    tubex::ctc::deriv.contract(x_ned, v_ned,tubex::TimePropag::FORWARD);

    //calculate ECEF_REF
    Interval N_e=6378137/sqrt(1-pow(0.08181919,2)*sin(LLH_Position[start_index].second[1])*sin(LLH_Position[start_index].second[1]));

    ECEF_ref[0]=(N_e+LLH_Position[start_index].second[2])*cos(LLH_Position[start_index].second[1])*cos(LLH_Position[start_index].second[0]);
    ECEF_ref[1]=(N_e+LLH_Position[start_index].second[2])*cos(LLH_Position[start_index].second[1])*sin(LLH_Position[start_index].second[0]);
    ECEF_ref[2]=(N_e*(1-pow(0.08181919,2))+LLH_Position[start_index].second[2])*sin(LLH_Position[start_index].second[1]);

    target_quaternion=quaternion[start_index].second;
    IntervalMatrix transform= calculate_transform_matrix(target_quaternion);

    IntervalMatrix transform1(3,3);
    IntervalMatrix transform2(3,3);

    transform1[0][0]=0.0129648;
    transform1[0][1]=-0.00459268;
    transform1[0][2]=0.999906;
    transform1[1][0]=0.999899;
    transform1[1][1]=0.00590588;
    transform1[1][2]=-0.0129374;
    transform1[2][0]=-0.00584589;
    transform1[2][1]=0.999973;
    transform1[2][2]=0.00466879;

    transform2[0][0]=-0.0141904;
    transform2[0][1]=0.999899;
    transform2[0][2]=-0.00136026;
    transform2[1][0]=-0.099215;
    transform2[1][1]=-5.43574e-05;
    transform2[1][2]=0.995067;
    transform2[2][0]=0.994965;
    transform2[2][1]=0.0142554;
    transform2[2][2]=0.0992057;

    for(int i=start_index;i<=end_index;i++)
    {
        ned_ecef[0][0]=-sin(LLH_Position[i].second[1]-LLH_Position[start_index].second[1])*cos(LLH_Position[i].second[0]-LLH_Position[start_index].second[0]);
        ned_ecef[0][1]=-sin(LLH_Position[i].second[1]-LLH_Position[start_index].second[1])*sin(LLH_Position[i].second[0]-LLH_Position[start_index].second[0]);
        ned_ecef[0][2]=cos(LLH_Position[i].second[1]-LLH_Position[start_index].second[1]);
        ned_ecef[1][0]=-sin(LLH_Position[i].second[0]-LLH_Position[start_index].second[0]);
        ned_ecef[1][1]=cos(LLH_Position[i].second[0]-LLH_Position[start_index].second[0]);
        ned_ecef[1][2]=0;
        ned_ecef[2][0]=-cos(LLH_Position[i].second[1]-LLH_Position[start_index].second[1])*cos(LLH_Position[i].second[0]-LLH_Position[start_index].second[0]);
        ned_ecef[2][1]=-cos(LLH_Position[i].second[1]-LLH_Position[start_index].second[1])*sin(LLH_Position[i].second[0]-LLH_Position[start_index].second[0]);
        ned_ecef[2][2]=-sin(LLH_Position[i].second[1]-LLH_Position[start_index].second[1]);

        ned_ecef_inverse_interval=calculate_inverse_matrix(ned_ecef);
        //cout<<LLH_Position[i].first<<endl;
        //cout<<x_ned[0].slice(i-start_index)->tdomain()<<endl;

        IntervalVector temp(3);
        temp[0]=x_ned[0].slice(i-start_index)->codomain();
        temp[1]=x_ned[1].slice(i-start_index)->codomain();
        temp[2]=x_ned[2].slice(i-start_index)->codomain();

        IntervalVector ecef_pos(3);
        cout<<ned_ecef_inverse_interval*temp<<endl;
        ecef_pos=ned_ecef_inverse_interval*temp;
        IntervalVector erg(3);
        cout<<ecef_pos<<endl;
        erg=transform*ecef_pos;
        erg=transform2*erg;
        erg=transform1*erg;
        cout<<erg<<endl;
        Body_Position.emplace_back(make_pair(x_ned[0].slice(i-start_index)->tdomain(),erg));
    }
    cout<<Body_Position[0].second<<endl;
    IntervalVector temp=Body_Position[0].second;
    //for(int i=0;i<Body_Position.size();i++)
//    {
//        cout<<Body_Position[i].second<<endl;
//        Body_Position[i].second=Body_Position[i].second-temp;
//        cout<<Body_Position[i].second<<endl;
//    }
    int dt=0;
    int incresment=1;
    double index=0;
    std::map<double, ibex::Vector> map_temp;
    ibex::Vector truth(3);
    vibes::beginDrawing();
    tubex::VIBesFigTube tube_x("tube_x");
    tubex::VIBesFigTube tube_y("tube_y");
    tubex::VIBesFigTube tube_z("tube_z");
    tube_x.set_properties(100,100,600,300);
    tube_y.set_properties(100,100,600,300);
    tube_z.set_properties(100,100,600,300);
    IntervalVector box_x(2);
    IntervalVector box_y(2);
    IntervalVector box_z(2);
    for(int i=0;i<Body_Position.size();i++)
    {
        box_x[0]=Body_Position[i].first;
        box_y[0]=Body_Position[i].first;
        box_z[0]=Body_Position[i].first;

        for (int j=0;j<m.ground_truth.size();j++)
        {
            if(m.ground_truth[j].first.contains(box_x[0].ub()))
            {
                truth[0]=m.ground_truth[j].second.pose.position.x;
                truth[1]=m.ground_truth[j].second.pose.position.y;
                truth[2]=m.ground_truth[j].second.pose.position.z;
                map_temp.insert(make_pair(index,truth));
                index++;
                break;
            }
        }
        box_x[0]=Interval(dt,dt+incresment);
        box_y[0]=Interval(dt,dt+incresment);
        box_z[0]=Interval(dt,dt+incresment);
        dt++;
//        box_x[1]=Body_Position[i].second[0];
//        box_y[1]=Body_Position[i].second[1];
//        box_z[1]=Body_Position[i].second[2];
        box_x[1]=x_ned[0].slice(i)->codomain();
        box_y[1]=x_ned[1].slice(i)->codomain();
        box_z[1]=x_ned[2].slice(i)->codomain();

        tube_x.draw_box(box_x,"lightGray");
        tube_y.draw_box(box_y,"lightGray");
        tube_z.draw_box(box_z,"lightGray");
    }
    tube_x.show();
    tube_y.show();
    tube_z.show();
    tubex::TrajectoryVector truth_trajectory(map_temp);
    tube_x.add_trajectory(&truth_trajectory[0],"truth_traj");
    tube_y.add_trajectory(&truth_trajectory[1],"truth_traj");
    tube_z.add_trajectory(&truth_trajectory[2],"truth_traj");
    tube_x.show();
    tube_y.show();
    tube_z.show();
    vibes::endDrawing();
}
IntervalMatrix EF::calculate_inverse_matrix(IntervalMatrix ori)
{
//   Interval det = ori[0][0] * (ori[1][1]  * ori[2][2]  - ori[2][1]  * ori[1][2] ) -
//                    ori[0][1]  * (ori[1][0]  * ori[2][2]  - ori[1][2]  * ori[2][0] ) +
//                    ori[0][2]  * (ori[1][0]  * ori[2][1]  - ori[1][1]  * ori[2][0] );
//    Interval invdet = 1 / det;
//    IntervalMatrix m_inv(3,3);
//    m_inv[0][0] = (ori[1][1] * ori[2][2] - ori[2][1] * ori[1][2]) * invdet;
//    m_inv[0][1] = (ori[0][2] * ori[2][1] - ori[0][1] * ori[2][2]) * invdet;
//    m_inv[0][2] = (ori[0][1] * ori[1][2] - ori[0][2] * ori[1][1]) * invdet;
//    m_inv[1][0] = (ori[1][2] * ori[2][0] - ori[1][0] * ori[2][2]) * invdet;
//    m_inv[1][1] = (ori[0][0] * ori[2][2] - ori[0][2] * ori[2][0]) * invdet;
//    m_inv[1][2] = (ori[1][0] * ori[0][2] - ori[0][0] * ori[1][2]) * invdet;
//    m_inv[2][0] = (ori[1][0] * ori[2][1] - ori[2][0] * ori[1][1]) * invdet;
//    m_inv[2][1] = (ori[2][0] * ori[0][1] - ori[0][0] * ori[2][1]) * invdet;
//    m_inv[2][2] = (ori[0][0] * ori[1][1] - ori[1][0] * ori[0][1]) * invdet;
    return ori.transpose();
}
void EF::draw(tubex::TubeVector x, Measurement m)
{
    int dt=0;
    int incresment=1;
    double index=0;
    vibes::beginDrawing();
    tubex::VIBesFigTube tube_x("tube_x");
    tubex::VIBesFigTube tube_y("tube_y");
    tubex::VIBesFigTube tube_z("tube_z");
    tube_x.set_properties(100,100,600,300);
    tube_y.set_properties(100,100,600,300);
    tube_z.set_properties(100,100,600,300);
    std::map<double, ibex::Vector> temp;
    IntervalVector box_x;
    IntervalVector box_y;
    IntervalVector box_z;
    ibex::Vector truth(3);
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
    tubex::TrajectoryVector truth_trajectory(temp);
    tube_x.add_trajectory(&truth_trajectory[0],"truth_traj");
    tube_y.add_trajectory(&truth_trajectory[1],"truth_traj");
    tube_z.add_trajectory(&truth_trajectory[2],"truth_traj");
    tube_x.show();
    tube_y.show();
    tube_z.show();
    vibes::endDrawing();
}
//void EF::add_LLH_position(const geometry_msgs::Vector3StampedConstPtr& v)
//{
//    IntervalVector LLH(3);
//    LLH[0]=v->vector.x;
//    LLH[1]=v->vector.y;
//    LLH[2]=v->vector.z;
//
//    if (LLH_position.empty())
//    {
//        LLH_position.emplace_back(make_pair(Interval(v->header.stamp.toSec()),LLH));
//    }
//    else
//    {
//        LLH_position.emplace_back(make_pair(Interval(LLH_position.back().first.ub(),v->header.stamp.toSec()),LLH));
//    }
//}
//
// Created by noah on 15.02.21.
//

