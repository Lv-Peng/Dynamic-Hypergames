#include<iostream>
#include<boost/algorithm/string.hpp>
#include<ros/ros.h>
#include<geometry_msgs/TransformStamped.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/String.h>
#include<message_filters/subscriber.h>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include<string>
#include<control_msgs/FollowJointTrajectoryActionGoal.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>

using namespace std;
using namespace Eigen;

control_msgs::FollowJointTrajectoryActionGoal goal;
// goal.
vector<Vector2d> GoalPoint;
geometry_msgs::Twist cmd;
double angleOffset=-0.936;
int FrameCnt=0;

struct CalibrationDuals
{
    Vector2d spacePoint;
    Vector2d imagePoint;
};

CalibrationDuals cali1,cali2,cali3;

void Mapping(Vector2d &p)
{
    Vector2d vec1=cali2.imagePoint-cali1.imagePoint;
    Vector2d vec2=cali3.imagePoint-cali1.imagePoint;
    Matrix2d A;
    A.col(0)=vec1;A.col(1)=vec2;
    Vector2d k=A.inverse()*(p-cali1.imagePoint);
    Vector2d vec1_=cali2.spacePoint-cali1.spacePoint;
    Vector2d vec2_=cali3.spacePoint-cali1.spacePoint;
    p=cali1.spacePoint+k(0)*vec1_+k(1)*vec2_;
}

void GoalPointDecode(const std_msgs::StringConstPtr &Ptr)
{
    GoalPoint.clear();
    vector<string> SplitedStr,temp;
    boost::split(SplitedStr,Ptr->data,boost::is_any_of(" \n\t"));
    int cnt=0;
    GoalPoint.clear();
    double x,y;
    for (int i=0;i<SplitedStr.size();i++)
    {
        // cout<<i<<": "<<SplitedStr[i]<<"  size: "<<SplitedStr[i].size()<<endl;
        if(SplitedStr[i].size()>0)temp.push_back(SplitedStr[i]);
    }
    SplitedStr=temp;

    x=stod(SplitedStr[0]);
    y=stod(SplitedStr[1]);
    Vector2d p(x,y);
    cout << "image_point" << x << "  " << y << endl;
    Mapping(p);
    GoalPoint.push_back(p);
    //cout << "goal.size1" << GoalPoint.size()<<endl;
}

void cmdThreshold(Vector2d &vw)
{
    // cout<<"ori vw:"<<vw.transpose()<<endl;
    double v_threshold_high=2,v_threshold_low=0.001;
    double w_threshold_high=1.5,w_threshold_low=0.001;
    if(abs(vw(0))>v_threshold_high)vw(0)=vw(0)>0?v_threshold_high:-v_threshold_high;
    else if(abs(vw(0))<v_threshold_low)vw(0)=vw(0)>0?v_threshold_low:-v_threshold_low;

    if(abs(vw(1))>w_threshold_high)vw(1)=vw(1)>0?w_threshold_high:-w_threshold_high;
    else if(abs(vw(1))<w_threshold_low)vw(1)=vw(1)>0?w_threshold_low:-w_threshold_low;
}

void viconCallback(const geometry_msgs::TransformStampedConstPtr &Ptr)
{
    //cout << "viconCallback" << endl;
    Vector2d Position;
    Quaterniond Orientation;
    Position(0)=Ptr->transform.translation.x;
    Position(1)=Ptr->transform.translation.y;
    Orientation.x()=Ptr->transform.rotation.x;
    Orientation.y()=Ptr->transform.rotation.y;
    Orientation.z()=Ptr->transform.rotation.z;
    Orientation.w()=Ptr->transform.rotation.w;
    Matrix3d R=Orientation.toRotationMatrix();
    double theta=atan2(R(1,0),R(0,0))-angleOffset;
    Vector2d temp;
    double r=0.1;
    //cout << "goal.size2" << GoalPoint.size()<<endl;
    if(GoalPoint.size()>0)
    {
        Vector2d d_=GoalPoint[0]-Position;
        if(d_.norm()<0.1)GoalPoint.erase(GoalPoint.begin());
        else
        {
            Matrix2d M;
            M(0,0)=cos(theta);M(0,1)=-r*sin(theta);M(1,0)=sin(theta);M(1,1)=r*cos(theta);
            temp=1.0*M.inverse()*d_;
        }
        cmdThreshold(temp);
        //cout<<"GoalPoint:\n"<<GoalPoint[0].transpose()<<endl;
         cout<<"GoalPoint:\n"<<GoalPoint[0].transpose()<<"\nd_:\n"<<d_.transpose()<<"\nPosition:\n"<<Position.transpose()<<"\nSum:\n"<<(Position+d_).transpose()<<endl<<endl;
    }
    else
    {
        temp=Vector2d::Zero();
    }
    cmd.linear.x=temp(0);
    cmd.angular.z=temp(1);

    if(GoalPoint.size()>0)
    {
        // cout<<"Goal: "<<GoalPoint[0].transpose()<<"  Frame:"<<FrameCnt<<endl<<endl;
        FrameCnt+=1;
    }
}

int main(int argc,char** argv)
{
    // goal.goal.trajectory.points
    // trajectory_msgs::JointTrajectoryPoint pnt;
    // pnt.

    cali1.spacePoint=Vector2d(1.855,0.895);cali1.imagePoint=Vector2d(2,0);
    cali2.spacePoint=Vector2d(1.415,1.35);cali2.imagePoint=Vector2d(1,1);
    cali3.spacePoint=Vector2d(0.957,0.931);cali3.imagePoint=Vector2d(0,0);

    string ObjectName=argv[1];
    ros::init(argc,argv,"PointActuation_"+ObjectName);

    ros::NodeHandle nh;
    ros::Rate rate(100);
    ros::Publisher Move;
    message_filters::Subscriber<geometry_msgs::TransformStamped> Sub(nh,"/vicon/"+ObjectName+"/"+ObjectName,1);
    message_filters::Subscriber<std_msgs::String> PointSub(nh,"/"+ObjectName+"/positions",1);
    //cout << "before viconcallback" << endl;
    Sub.registerCallback(&viconCallback);
    PointSub.registerCallback(&GoalPointDecode);
    Move=nh.advertise<geometry_msgs::Twist>("/"+ObjectName+"/cmd_vel",100);

    cmd.angular.z=1;
    Move.publish(cmd);
    cmd.angular.z=0;

    while(ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
        Move.publish(cmd);
	//cout<<"cmd"<<cmd<<"\n";
    //break;
     //if(GoalPoint.size() > 0)
     //         cout<<"GoalPoint" <<GoalPoint[0][0] << GoalPoint[0][1]<<"\n";
    }
    cout<<"Exit.\n";
    return 0;
}
