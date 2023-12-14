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


// goal.
vector<Vector2d> GoalPoint;
vector<string> SplitedStr;
geometry_msgs::Twist cmd;
//double angleOffset=-0.33385;//Burger10
double angleOffset = 1.54995-3.10492;
int FrameCnt=0;
double theta=angleOffset;
double theta_ = 0;
bool flag = 0;
bool flag_stop = 0;
bool flag_slam = 0;

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
    cout << "111"<< endl;
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
        {x = 1.55;y = 1.277; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x2")
        {x = 1.18;y = 1.277;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x3")
        {x = 0.85;y = 1.277;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x4")
        {x = 1.0889;y = 0.0352568; push_goal(x,y);continue;}//x = 0.5;y = 1.277; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x5")
        {x = 1.48769;y = 0.0484593;push_goal(x,y); continue;}//x = 0.15;y = 1.277;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x6")
        {x = -0.2;y = 1.277; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x7")
        {x = -0.55;y = 1.277;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x8")
        {x = -0.9;y = 1.277;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x9")
        {x = 1.55;y = 0.91; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x10")
        {x = 1.2;y = 0.91;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x11")
        {x = 0.85;y = 0.91; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x12")
        {x = 1.02615;y = 0.348197;continue;}//x = 0.5;y = 0.91;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x13")
        {x = 0.15;y = 0.91;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x14")
        {x = -0.2;y = 0.91; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x15")
        {x = -0.55;y = 0.91;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x16")
        {x = -0.9;y = 0.91; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x17")
        {x = 1.55;y =  0.56;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x18")
        {x = 1.2;y = 0.56;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x19")
        {x = 0.85;y = 0.56; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x20")
        {x = 0.5;y = 0.56;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x21")
        {x = 0.15;y = 0.56; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x22")
        {x = -0.18;y = 0.58;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x23")
        {x = -0.55;y = 0.56;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x24")
        {x = -0.9;y = 0.56; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x25")
        {x = 1.55;y = 0.21;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x26")
        {x = 1.2;y = 0.21; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x27")
        {x = 0.85;y = 0.21;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x28")
        {x = 0.5;y = 0.21;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x29")
        {x = 0.15;y = 0.21; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x30")
        {x = -0.22;y = 0.23;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x31")
        {x = -0.55;y = 0.21; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x32")
        {x = -0.9;y = 0.21;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x33")
        {x = 1.55;y = -0.14;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x34")
        {x = 1.2;y = -0.14; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x35")
        {x = 0.85;y = -0.14;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x36")
        {x = 0.5;y = -0.14; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x37")
        {x = 0.15;y = -0.14;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x38")
        {x = -0.2;y = -0.14;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x39")
        {x = -0.55;y = -0.14; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x40")
        {x = -0.9;y = -0.14;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x41")
        {x = 1.55;y = -0.49; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x42")
        {x = 1.2;y = -0.49;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x43")
        {x = 0.85;y = -0.49;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x44")
        {x = 0.5;y = -0.49; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x45")
        {x = 0.15;y = -0.49;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x46")
        {x = -0.2;y = -0.49; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x47")
        {x = -0.55;y = -0.49;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x48")
        {x = -0.9;y = -0.49;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x49")
        {x = 1.55;y = -0.84; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x50")
        {x = 1.2;y = -0.84;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x51")
        {x = 0.85;y = -0.84; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x52")
        {x = 0.5;y = -0.84;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x53")
        {x = 0.15;y = -0.84;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x54")
        {x = -0.2;y = -0.84; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x55")
        {x = -0.55;y = -0.84;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x56")
        {x = -0.9;y = -0.84; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x57")
        {x = 1.55;y = -1.19;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x58")
        {x = 1.2;y = -1.19;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x59")
        {x = 0.85;y = -1.19; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x60")
        {x = 0.5;y = -1.19;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x61")
        {x = 0.15;y = -1.19; push_goal(x,y);continue;}
        if(SplitedStr[i]=="x62")
        {x = -0.2;y = -1.19;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x63")
        {x = -0.55;y = -1.19;push_goal(x,y); continue;}
        if(SplitedStr[i]=="x64")
        {x = -0.861587;y = -1.1719; push_goal(x,y);continue;}

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
    if (flag_slam == 1)
    {
        return;
    }
   // cout <<"hello";
    Vector2d Position;
    Quaterniond Orientation;
    double theta = 0;
    double theta_ = 0;
    Position(0)=Ptr->transform.translation.x;
    Position(1)=Ptr->transform.translation.y;
    Orientation.x()=Ptr->transform.rotation.x;
    Orientation.y()=Ptr->transform.rotation.y;
    Orientation.z()=Ptr->transform.rotation.z;
    Orientation.w()=Ptr->transform.rotation.w;
    Matrix3d R=Orientation.toRotationMatrix();
    theta=atan2(R(1,0),R(0,0)) + angleOffset ;
    double r=0.1;
    double gain = 0.4;
    double gain2 = 0.2;
    //cout << Position[0] << " " << Position[1] << endl;
    //cout << theta << endl;
    double a= GoalPoint[FrameCnt][0];
    double b = GoalPoint[FrameCnt][1];
   //cout << "aaa";
    //theta_ = atan2(( Position[1]-GoalPoint[0][1]),(Position[0] - GoalPoint[0][0] ));
    //theta_ = atan2(( GoalPoint[0][1] - Position[1]),(GoalPoint[0][0]  - Position[0] ));
    theta_ = atan2(b-Position[1],a-Position[0]);
    cmd.angular.z = 0;
    cmd.linear.x = 0;
    //if(((abs(theta_ - theta))> 0.15) || ((flag==0)&&(abs(theta_ - theta))> 0.1) )

    if(sin(abs(theta_ - theta))> 0.1) 
    {
        flag = 0;
        cmd.angular.z = gain*(sin(theta_ - theta));
    }
    else
    {cmd.angular.z = 0;
    flag = 1;
         Vector2d x(a,b);
        Vector2d d_=x-Position;
        if(d_.norm()>0.1)
        {

            //cmd.linear.x = gain2 * d_.norm();
            //cmd.linear.x = -gain2 * 0.5;//Burger10
            cmd.linear.x = gain2 * 0.2;
        }
        else 
        {
            cmd.linear.x = 0;
            cmd.angular.z = 0;
            
        }
    }

     double linear = cmd.linear.x;
     double angular = cmd.angular.z;
    
    
    
    
    if (FrameCnt >= GoalPoint.size())
    {cmd.angular.z = 0;
    cmd.linear.x = 0;
    flag_stop = 1;
    }
    if(linear==0 && angular==0 && GoalPoint.size()> 0)
    {
        flag_slam = 0;
        FrameCnt+=1;
        cmd.angular.z = 0;
    cmd.linear.x = 0;
        cout << SplitedStr[FrameCnt] << endl;
        //GoalPoint.pop_back();
        //cout << GoalPoint[FrameCnt][1] << endl;
    }
}

int main(int argc,char** argv)
{
    // goal.goal.trajectory.points
    // trajectory_msgs::JointTrajectoryPoint pnt;
    // pnt.
    //cout <<"hello";
    cali1.spacePoint=Vector2d(1.855,0.895);cali1.imagePoint=Vector2d(2,0);
    cali2.spacePoint=Vector2d(1.415,1.35);cali2.imagePoint=Vector2d(1,1);
    cali3.spacePoint=Vector2d(0.957,0.931);cali3.imagePoint=Vector2d(0,0);

    string ObjectName=argv[1];
    ros::init(argc,argv,"PointActuation_"+ObjectName);
    //FrameCnt = 0;
    ros::NodeHandle nh;
    ros::Rate rate(10);
    ros::Publisher Move;
    //message_filters::Subscriber<tf2_msgs::TFMessage> Sub1(nh, "/tf",1);
    message_filters::Subscriber<geometry_msgs::TransformStamped> Sub(nh,"/vicon/"+ObjectName+"/"+ObjectName,1);
    message_filters::Subscriber<std_msgs::String> PointSub(nh,"/"+ObjectName+"/positions",1);
    // double x = 0;
    // double y = 0;
    // Vector2d p(x,y);
    //cout << "image_point" << x << "  " << y << endl;
    //Mapping(p);
    //GoalPoint.push_back(p);
    
    //cout << "before viconcallback" << endl;
    PointSub.registerCallback(&GoalPointDecode);
    
   
    Move=nh.advertise<geometry_msgs::Twist>("/"+ObjectName+"/cmd_vel",1000);

    cmd.angular.z=1;
    Move.publish(cmd);
    cmd.angular.z=0;
    //FrameCnt = 0;
    //Sub1.registerCallback(&slamCallback);
    cmd.linear.x = 0;
    cmd.angular.z = 0;
    
    while(ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
        Sub.registerCallback(&viconCallback);
        Move.publish(cmd);      
          if(flag_stop)
          break;
	//cout<<"cmd"<<cmd<<"\n";
    //break;
     //if(GoalPoint.size() > 0)
     //         cout<<"GoalPoint" <<GoalPoint[0][0] << GoalPoint[0][1]<<"\n";
    }
    cout<<"Exit.\n";
    return 0;
}
