//
// Created by noah on 27.02.21.
//

#include "KdTree.h"

KdTree::KdNode* KdTree::create_kd_tree(pcl::PointCloud<pcl::PointXYZ> pc)
{
    if(pc.points.empty())
    {
        return nullptr;
    }
    int n= pc.points.size();
    if(n==1)
    {
        IntervalVector erg(3);
        erg= calculate_point_interval(pc.points[0]);
        //erg[0]=
        return new KdNode(erg,-1);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    //kdtree.setInputCloud();
    splitAttribute= getSplitAttribute(pc);
    set_static_value(splitAttribute);
    double splitValue= getSplitValue(pc);

    //split data according splitAttribute and splitValue
    pcl::PointCloud<pcl::PointXYZ> left;
    pcl::PointCloud<pcl::PointXYZ> right;
    KdNode * splitNode;
    int flag=0;
    for(int i=0;i<n ;i++)
    {
        if(splitAttribute==0)
        {
            if(flag==0 && pc.points[i].x==splitValue)
            {
                IntervalVector data=calculate_point_interval(pc.points[i]);
                splitNode = new KdNode(data,0);
                flag=1;
                continue;
            }
            if(pc.points[i].x<splitValue)
            {
                left.points.emplace_back(pc.points[i]);
            }
            else
            {
                right.points.emplace_back(pc.points[i]);
            }
        }
        else if (splitAttribute==1)
        {
            if(flag==0 && pc.points[i].y==splitValue)
            {
                IntervalVector data=calculate_point_interval(pc.points[i]);
                splitNode = new KdNode(data,1);
                flag=1;
                continue;
            }
            if(pc.points[i].y<splitValue)
            {
                left.points.emplace_back(pc.points[i]);
            }
            else
            {
                right.points.emplace_back(pc.points[i]);
            }
        }
        else if (splitAttribute==2)
        {
            if(flag==0 && pc.points[i].z==splitValue)
            {
                IntervalVector data=calculate_point_interval(pc.points[i]);
                splitNode = new KdNode(data,2);
                flag=1;
                continue;
            }
            if(pc.points[i].z<splitValue)
            {
                left.points.emplace_back(pc.points[i]);
            }
            else
            {
                right.points.emplace_back(pc.points[i]);
            }
        }
    }
    splitNode->leftchild=create_kd_tree(left);
    splitNode->rightchild=create_kd_tree(right);
    return splitNode;
}
void KdTree::add_Tree2Forest(pcl::KdTreeFLANN<pcl::PointXYZ>::ConstPtr tree, ros::Time time){
    Forest.emplace_back(make_pair(time,tree));
    cout<<Forest.size()<<endl;
}
double KdTree::getVariance(pcl::PointCloud<pcl::PointXYZ> pc,int index)
{
    int size= pc.points.size();
    double sum= 0;
    bool get_x=false;
    bool get_y=false;
    bool get_z=false;
     if (index==0)
     {
         for (int i=0 ;i<size;i++)
         {
             sum+=pc.points[i].x;
         }
         double avg= sum/size;
         sum =0;
         for (int i=0 ;i<size;i++)
         {
             sum+=pow(pc.points[i].x-avg,2);
         }
     }
     else if (index==1)
     {
         for (int i=0 ;i<size;i++)
         {
             sum+=pc.points[i].y;
         }
         double avg= sum/size;
         sum =0;
         for (int i=0 ;i<size;i++)
         {
             sum+=pow(pc.points[i].y-avg,2);
         }
     }
     else if (index ==2)
     {
         for (int i=0 ;i<size;i++)
         {
             sum+=pc.points[i].z;
         }
         double avg= sum/size;
         sum =0;
         for (int i=0 ;i<size;i++)
         {
             sum+=pow(pc.points[i].z-avg,2);
         }
     }
     return sum/size;

}
 void KdTree::set_static_value(int splitattribute)
{
    splitAttribute_static=splitattribute;
}
int KdTree::getSplitAttribute(const pcl::PointCloud<pcl::PointXYZ> pc)
{

    int splitAttribute =0;
    //calculate the variance of x
    double maxVar= getVariance(pc,0);
    for(int i=1;i<3;i++)
    {
        double temp = getVariance(pc,i);
        if(temp>maxVar)
        {
            splitAttribute=i;
            maxVar=temp;
        }
    }
    return splitAttribute;
}

bool KdTree::cmp(const pcl::PointXYZ & value1, const pcl::PointXYZ & value2)
{
    if(splitAttribute_static==0)
    {
        return value1.x<value2.x;
    }
    else if (splitAttribute_static==1)
    {
        return value1.y<value2.y;
    }
    else if (splitAttribute_static==2)
    {
        return value1.z<value2.z;
    }
}
double KdTree::getSplitValue(pcl::PointCloud<pcl::PointXYZ> pc)
{
    std::sort(pc.points.begin(),pc.points.end(),cmp);
    int size= pc.points.size();
    size=size/2;
    double erg;
    if(splitAttribute==0)
    {
       erg= pc.points[size].x;
    }
    else if (splitAttribute==1)
    {
        erg= pc.points[size].y;
    }
    else if (splitAttribute==2)
    {
        erg= pc.points[size].z;
    }
    return erg;
}
void KdTree::set_uncertinty(double rho_uc, double phi_uc, double theta_uc, double horizontal_angle_uncertainty_uc, double vertical_angle_uncertainty_uc)
{
    this->rho_uc=rho_uc;
    this->phi_uc=phi_uc;
    this->theta_uc=theta_uc;
    this->horizontal_angle_uncertainty_uc=horizontal_angle_uncertainty_uc;
    this->vertical_angle_uncertainty_uc=vertical_angle_uncertainty_uc;
}
IntervalVector KdTree::calculate_point_interval(pcl::PointXYZ point)
{
    Interval interval_rho;
    Interval interval_phi;
    Interval interval_theta;
    IntervalVector coordinate(3,Interval(0));
    IntervalVector horizontal_opening_angle(3,Interval(0));
    IntervalVector vertical_opening_angle(3,Interval(0));
    double x_coordinate;
    double y_coordinate;
    double z_coordinate;
    double rho;
    double phi;
    double theta;

    x_coordinate=point.x;
    y_coordinate=point.y;
    z_coordinate=point.z;

    rho=sqrt(x_coordinate*x_coordinate+y_coordinate*y_coordinate+z_coordinate*z_coordinate);
    theta=acos(z_coordinate/rho);
    phi=atan(y_coordinate/x_coordinate);

    interval_rho=(rho-rho_uc,rho+rho_uc);
    interval_phi=(phi-phi_uc,phi+phi_uc);
    interval_theta=(theta-theta_uc,theta+theta_uc);

    coordinate[0]=interval_rho*sin(interval_theta)*cos(interval_phi);
    coordinate[1]=interval_rho*sin(interval_theta)*sin(interval_phi);
    coordinate[2]=interval_rho*cos(interval_theta);


    //additional uncertinty due to the initial footprint of the laser beam
    horizontal_opening_angle[0]=sin(Interval(phi))+Interval(-horizontal_angle_uncertainty_uc,+horizontal_angle_uncertainty_uc);
    horizontal_opening_angle[1]=-cos(Interval(phi))+Interval(-horizontal_angle_uncertainty_uc,+horizontal_angle_uncertainty_uc);
    horizontal_opening_angle[2]=Interval(0)+Interval(-horizontal_angle_uncertainty_uc,+horizontal_angle_uncertainty_uc);

    vertical_opening_angle[0]=cos(Interval(theta))*cos(Interval(phi))+Interval(-vertical_angle_uncertainty_uc,+vertical_angle_uncertainty_uc);
    vertical_opening_angle[1]=cos(Interval(theta))*sin(Interval(phi))+Interval(-vertical_angle_uncertainty_uc,+vertical_angle_uncertainty_uc);
    vertical_opening_angle[2]=-sin(Interval(theta))+Interval(-vertical_angle_uncertainty_uc,+vertical_angle_uncertainty_uc);
    //addition
//     coordinate[0]=coordinate[0]+horizontal_opening_angle[0]+vertical_opening_angle[0];
//     coordinate[1]=coordinate[1]+horizontal_opening_angle[1]+vertical_opening_angle[1];
//     coordinate[2]=coordinate[2]+horizontal_opening_angle[2]+vertical_opening_angle[2];

    coordinate+=horizontal_opening_angle+vertical_opening_angle;
    return  coordinate;
}


