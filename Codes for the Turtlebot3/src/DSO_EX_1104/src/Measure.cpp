#include<iostream>
#include<ros/ros.h>
#include<geometry_msgs/TransformStamped.h>
#include<message_filters/subscriber.h>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include<string>

using namespace std;
using namespace Eigen;

void viconCallback(const geometry_msgs::TransformStampedConstPtr &Ptr)
{
    Vector2d Position;
    Quaterniond Orientation;
    Position(0)=Ptr->transform.translation.x;
    Position(1)=Ptr->transform.translation.y;
    Orientation.x()=Ptr->transform.rotation.x;
    Orientation.y()=Ptr->transform.rotation.y;
    Orientation.z()=Ptr->transform.rotation.z;
    Orientation.w()=Ptr->transform.rotation.w;
    Matrix3d R=Orientation.toRotationMatrix();
    double theta=atan2(R(1,0),R(0,0));
    cout<<"Pos:"<<Position.transpose()<<endl<<"Theta:"<<theta<<endl;
}

int main(int argc,char** argv)
{
    string ObjectName=argv[1];
    ros::init(argc,argv,"ViconMeasure");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    message_filters::Subscriber<geometry_msgs::TransformStamped> Sub(nh,"/vicon/"+ObjectName+"/"+ObjectName,1);
    Sub.registerCallback(viconCallback);
    while(ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
    }
    cout<<"Exit.\n";
    return 0;
}