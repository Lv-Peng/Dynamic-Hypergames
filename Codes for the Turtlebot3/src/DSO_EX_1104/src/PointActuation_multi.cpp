#include<iostream>
#include<boost/algorithm/string.hpp>
#include<ros/ros.h>
#include<geometry_msgs/TransformStamped.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/String.h>
#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include<string>

using namespace std;
using namespace Eigen;

//About Map
struct CalibrationDuals
{
    Vector2d spacePoint;
    Vector2d imagePoint;
};

CalibrationDuals cali1,cali2,cali3;

class Vehicle
{
private:
    double angle_offset;
    string Name;
public:
    Vector2d Position;
    Quaterniond Orientation;
    string name(){return Name;}
    double r;
    double theta()
    {
        Matrix3d R=Orientation.toRotationMatrix();
        return atan2(R(1,0),R(0,0))-angle_offset;
    }
    Vehicle(string name_, double r_=0.2, double angleOffset=0){Name=name_;r=r_;angle_offset=angleOffset;}
    ~Vehicle(){}
};

vector<vector<Vector2d>> GoalPoints;
vector<Vehicle> vehicles;
vector<geometry_msgs::Twist> cmds;
int FrameCnt=0;
int NumAgents;

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

//



void GoalPointDecode(const std_msgs::StringConstPtr &Ptr)
{
    vector<string> SplitedStr,temp;
    boost::split(SplitedStr,Ptr->data,boost::is_any_of(" \n\t"));
    int cnt=0;
    double x,y;
    for (int i=0;i<SplitedStr.size();i++)
    {
        // cout<<i<<": "<<SplitedStr[i]<<"  size: "<<SplitedStr[i].size()<<endl;
        if(SplitedStr[i].size()>0)temp.push_back(SplitedStr[i]);
    }
    SplitedStr=temp;
    for(int i=0;i<NumAgents;i++)
    {
        GoalPoints[i].clear();

        x=stod(SplitedStr[2*i]);
        y=stod(SplitedStr[2*i+1]);
        Vector2d p(x,y);
        Mapping(p);
        GoalPoints[i].push_back(p);
    }
}

void cmdThreshold(Vector2d &vw)
{
    // cout<<"ori vw:"<<vw.transpose()<<endl;
    double v_threshold_high=1,v_threshold_low=0.001;
    double w_threshold_high=1.5,w_threshold_low=0.001;
    if(abs(vw(0))>v_threshold_high)vw(0)=vw(0)>0?v_threshold_high:-v_threshold_high;
    else if(abs(vw(0))<v_threshold_low)vw(0)=vw(0)>0?v_threshold_low:-v_threshold_low;

    if(abs(vw(1))>w_threshold_high)vw(1)=vw(1)>0?w_threshold_high:-w_threshold_high;
    else if(abs(vw(1))<w_threshold_low)vw(1)=vw(1)>0?w_threshold_low:-w_threshold_low;
}

void viconConvert(const geometry_msgs::TransformStampedConstPtr Ptr,Vehicle &vehicle)
{
    vehicle.Position=Vector2d(Ptr->transform.translation.x,Ptr->transform.translation.y);
    vehicle.Orientation.w()=Ptr->transform.rotation.w;
    vehicle.Orientation.x()=Ptr->transform.rotation.x;
    vehicle.Orientation.y()=Ptr->transform.rotation.y;
    vehicle.Orientation.z()=Ptr->transform.rotation.z;
}

void viconCallback(const geometry_msgs::TransformStampedConstPtr &Ptr0,const geometry_msgs::TransformStampedConstPtr &Ptr1,const geometry_msgs::TransformStampedConstPtr &Ptr2)
{
    viconConvert(Ptr0,vehicles[0]);
    viconConvert(Ptr1,vehicles[1]);
    viconConvert(Ptr2,vehicles[2]);

    Vector2d temp;

    for(int i=0;i<NumAgents;i++)
    {
        if(GoalPoints[i].size()>0)
        {
            Vector2d d_=GoalPoints[i][0]-vehicles[i].Position;
            if(d_.norm()<0.1)GoalPoints[i].erase(GoalPoints[i].begin());
            else
            {
                Matrix2d M;
                double theta=vehicles[i].theta();
                M(0,0)=cos(theta);M(0,1)=-vehicles[i].r*sin(theta);M(1,0)=sin(theta);M(1,1)=vehicles[i].r*cos(theta);
                temp=0.3*M.inverse()*-d_;
            }
            cmdThreshold(temp);
            // cout<<"d:"<<d_.norm()<<"   v and w:"<<temp.transpose()<<endl;
        }
        else
        {
            temp=Vector2d::Zero();
        }
        cmds[i].linear.x=temp(0);
        cmds[i].angular.z=temp(1);
    }

    bool IsAllAgentsFree=true;
    for(int i=0;i<NumAgents;i++)if(GoalPoints[i].size()>0)IsAllAgentsFree=false;

    //INFO OUTPUT
    if(!IsAllAgentsFree)
    {
        cout<<"Frame:"<<FrameCnt<<endl<<"GoalPoints:\n";
        for(int i=0;i<NumAgents;i++)
        {
            cout<<"Agent"<<i<<":";
            for(int j=0;j<GoalPoints[i].size();j++)
            {
                cout<<GoalPoints[i][j].transpose()<<",\t";
            }
            cout<<endl<<"\tv and w:"<<cmds[i].linear.x<<"  "<<cmds[i].angular.z<<endl;
        }
        cout<<endl;
        FrameCnt+=1;
    }
}


int main(int argc,char** argv)
{
    NumAgents=stoi(argv[1]);

    if(argc<2+NumAgents)cerr<<"Please Specify the Name of agents\n";

    // cali1.spacePoint=Vector2d(0.74,1.14);cali1.imagePoint=Vector2d(-2.5,2.5);
    // cali2.spacePoint=Vector2d(0.85,-1.19);cali2.imagePoint=Vector2d(2.5,2.5);
    // cali3.spacePoint=Vector2d(-0.95,0.36);cali3.imagePoint=Vector2d(-1,-1.5);

    cali1.spacePoint=Vector2d(1.855,0.895);cali1.imagePoint=Vector2d(2,0);
    cali2.spacePoint=Vector2d(1.415,1.35);cali2.imagePoint=Vector2d(1,1);
    cali3.spacePoint=Vector2d(0.957,0.931);cali3.imagePoint=Vector2d(0,0);

    ros::init(argc,argv,"PointActuation");

    ros::NodeHandle nh;
    ros::Rate rate(10);
    vector<ros::Publisher> Move;

    for(int i=0;i<NumAgents;i++)
    {
        vehicles.push_back(Vehicle(argv[i+2]));
        Move.push_back(nh.advertise<geometry_msgs::Twist>("/"+vehicles[i].name()+"/cmd_vel",1000));
        cmds.push_back(geometry_msgs::Twist());
        GoalPoints.push_back(vector<Vector2d>());

        cmds[i].angular.z=1;
        Move[i].publish(cmds[i]);
    }

    message_filters::Subscriber<geometry_msgs::TransformStamped> Sub0(nh,"/vicon/"+vehicles[0].name()+"/"+vehicles[0].name(),1);
    message_filters::Subscriber<geometry_msgs::TransformStamped> Sub1(nh,"/vicon/"+vehicles[1].name()+"/"+vehicles[1].name(),1);
    message_filters::Subscriber<geometry_msgs::TransformStamped> Sub2(nh,"/vicon/"+vehicles[2].name()+"/"+vehicles[2].name(),1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TransformStamped,geometry_msgs::TransformStamped,geometry_msgs::TransformStamped> myPolicy;
    message_filters::Synchronizer<myPolicy> sync1(myPolicy(10),Sub0,Sub1,Sub2);
    sync1.registerCallback(boost::bind(&viconCallback,_1,_2,_3));

    message_filters::Subscriber<std_msgs::String> PointSub(nh,"/dso/positions",1);
    PointSub.registerCallback(&GoalPointDecode);


    while(ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
        for(int i=0;i<NumAgents;i++)
        {
            Move[i].publish(cmds[i]);
        }
    }
    cout<<"Exit.\n";
    return 0;
}