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
#include<time.h>
#include<tf2_msgs/TFMessage.h>

using namespace std;
using namespace Eigen;

tf2_msgs::TFMessage tf;

// goal.
vector<Vector2d> GoalPoint;
vector<string> SplitedStr;
geometry_msgs::Twist cmd;
double angleOffset=-0.0568693;//Burger10
//double angleOffset=-1.54237;//Burger9
int FrameCnt=0;
double theta=angleOffset;
double theta_ = 0;
bool flag = 0;
bool flag_stop = 0;

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

void push_goal(double x, double y)
{
    Vector2d p(x,y);
    //cout <<"point: " <<  p[0] << " " << p[1] << endl;
    GoalPoint.push_back(p);
}

void GoalPointDecode(const std_msgs::StringConstPtr &Ptr)
{
    GoalPoint.clear();
    vector<string> temp;
    boost::split(SplitedStr,Ptr->data,boost::is_any_of(" \n\t"));
    int cnt=0;
    GoalPoint.clear();
    double x,y;
    for (int i=0;i<SplitedStr.size();i++)
    {
        
        if(SplitedStr[i].size()>0)temp.push_back(SplitedStr[i]);
    }
    SplitedStr=temp;
    // cout << SplitedStr.size()<< endl;
    for (int i=0;i<SplitedStr.size();i++)
    {
        
        
        if(SplitedStr[i]=="x1")
        {x = 0;y = 0; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x2")
        {x = 0.373152;y = 0.024573;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x3")
        {x = 0.72;y = 0.053374;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x4")
        {x = 1.02524;y = 0.07993; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x5")
        {x = 1.42708;y = 0.157659;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x6")
        {x = 1.78075;y = 0.230284; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x7")
        {x = 2.10247;y = 0.29954;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x8")
        {x = 2.44898;y = 0.351263;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x9")
        {x = 0.928889;y = -0.283789; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x10")
        {x = 1.2;y = 0.91;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x11")
        {x = 0.69016;y = 0.41; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x12")
        {x = 0.979971;y = 0.412606;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x13")
        {x = 0.15;y = 0.91;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x14")
        {x = -0.2;y = 0.91; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x15")
        {x = 2.03445;y = 0.587509;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x16")
        {x = 2.36497;y = 0.710589; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x17")
        {x = 0.722603;y =  -0.915761;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x18")
        {x = 1.2;y = 0.56;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x19")
        {x = 0.85;y = 0.56; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x20")
        {x = 0.5;y = 0.56;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x21")
        {x = 0.15;y = 0.56; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x22")
        {x = 1.58066;y = 0.855096;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x23")
        {x = 1.91938;y = 0.951452;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x24")
        {x = 2.27932;y = 1.04876; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x25")
        {x = 0.45404;y = 0.224876;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x26")
        {x = 1.2;y = 0.21; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x27")
        {x = 0.85;y = 0.21;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x28")
        {x = 0.5;y = 0.21;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x29")
        {x = 1.17054;y = 1.02276; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x30")
        {x = 1.48319;y = 1.16286;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x31")
        {x = 1.79114;y = 1.34744; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x32")
        {x = -0.9;y = 0.21;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x33")
        {x = 0.197962;y = 0.453635;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x34")
        {x = 1.2;y = -0.14; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x35")
        {x = 0.85;y = -0.14;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x36")
        {x = 0.5;y = -0.14; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x37")
        {x = 1.00059;y = 1.32997;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x38")
        {x = 1.30661;y = 1.4821;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x39")
        {x = 1.64533;y = 1.64456; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x40")
        {x = -0.9;y = -0.14;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x41")
        {x = -0.0817697;y = 0.707396; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x42")
        {x = 1.2;y = -0.49;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x43")
        {x = 0.85;y = -0.49;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x44")
        {x = 0.533385;y = 1.48371; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x45")
        {x = 0.826007;y = 1.62884;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x46")
        {x = 1.17905;y = 1.82777; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x47")
        {x = 1.47384;y = 1.99041;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x48")
        {x = -0.9;y = -0.49;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x49")
        {x = -0.337436;y = 0.943926; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x50")
        {x = -0.101964;y = 1.14836;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x51")
        {x = 0.147342;y = 1.37786; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x52")
        {x = 0.44651;y = 1.64764;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x53")
        {x = 0.15;y = -0.84;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x54")
        {x = -0.2;y = -0.84; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x55")
        {x = 1.25938;y = 2.30801;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x56")
        {x = 1.5718;y = 2.49472; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x57")
        {x = -0.635898;y = 1.22697;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x58")
        {x = -0.382703;y = 1.45492;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x59")
        {x = 0.85;y = -1.19; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x60")
        {x = 0.19257;y = 1.92576;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x61")
        {x = 0.463117;y = 2.13509; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x62")
        {x = 0.743243;y = 2.34697;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x63")
        {x = 1.04275;y = 2.56711;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x64")
        {x = 1.33895;y = 2.77599; push_goal(x,y);continue;}

    }

    
   //Mapping(p);
    //sleep(10);
    
}

void cmdThreshold(Vector2d &vw)
{
    
    double v_threshold_high=2,v_threshold_low=0.001;
    double w_threshold_high=1.5,w_threshold_low=0.001;
    if(abs(vw(0))>v_threshold_high)vw(0)=vw(0)>0?v_threshold_high:-v_threshold_high;
    else if(abs(vw(0))<v_threshold_low)vw(0)=vw(0)>0?v_threshold_low:-v_threshold_low;

    if(abs(vw(1))>w_threshold_high)vw(1)=vw(1)>0?w_threshold_high:-w_threshold_high;
    else if(abs(vw(1))<w_threshold_low)vw(1)=vw(1)>0?w_threshold_low:-w_threshold_low;
}

void viconCallback(const geometry_msgs::TransformStampedConstPtr &Ptr)
{
    Vector2d Position;
    Position(0) = Ptr->transform.translation.x;
    Position(1) = Ptr->transform.translation.y;
    cout << "pos_vicon: " << Position[0] << " " << Position[1] << endl;
    Quaterniond Orientation;
    Orientation.x()=Ptr->transform.rotation.x;
    Orientation.y()=Ptr->transform.rotation.y;
    Orientation.z()=Ptr->transform.rotation.z;
    Orientation.w()=Ptr->transform.rotation.w;
    Matrix3d R=Orientation.toRotationMatrix();
    theta=atan2(R(1,0),R(0,0)) + angleOffset;
    cout << "theta_vicon: " << theta << endl;
    
}

void slamCallback(const tf2_msgs::TFMessageConstPtr &Ptr)
{
    
    Vector2d Position;
    Quaterniond Orientation;
    string a;
    a = Ptr->transforms[0].child_frame_id;
    if (a=="base_footprint")
        {Position(0) = Ptr->transforms[0].transform.translation.x;
        Position(1) = Ptr->transforms[0].transform.translation.y;
        cout << "pos_slam: " << Position[0] << " " << Position[1] << endl;
        Orientation.x() = Ptr->transforms[0].transform.rotation.x;
        Orientation.y() = Ptr->transforms[0].transform.rotation.y;
        Orientation.z() = Ptr->transforms[0].transform.rotation.z;
        Orientation.w() = Ptr->transforms[0].transform.rotation.w;
    Matrix3d R=Orientation.toRotationMatrix();

    theta=atan2(R(1,0),R(0,0)) ;
    cout << "theta_slam: " << theta << endl;
    
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
    //message_filters::Subscriber<geometry_msgs::TransformStamped> Sub1(nh,"/vicon/"+ObjectName+"/"+ObjectName,1);
    message_filters::Subscriber<tf2_msgs::TFMessage> Sub(nh, "/tf",1);
    message_filters::Subscriber<std_msgs::String> PointSub(nh,"/"+ObjectName+"/positions",1);
    // double x = 0;
    // double y = 0;
    // Vector2d p(x,y);
    //cout << "image_point" << x << "  " << y << endl;
    //Mapping(p);
    //GoalPoint.push_back(p);
    
    //cout << "before viconcallback" << endl;
    PointSub.registerCallback(&GoalPointDecode);
    
   
    Move=nh.advertise<geometry_msgs::Twist>("/"+ObjectName+"/cmd_vel",100);

    cmd.angular.z=1;
    Move.publish(cmd);
    cmd.angular.z=0;
    //Sub1.registerCallback(&viconCallback);
    Sub.registerCallback(&slamCallback);
    
    while(ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
        
        Move.publish(cmd);      
    }
    cout<<"Exit.\n";
    return 0;
}
