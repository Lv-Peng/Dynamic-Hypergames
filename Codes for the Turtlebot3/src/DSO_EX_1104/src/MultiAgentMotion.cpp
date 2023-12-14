#include<iostream>
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/TransformStamped.h>
#include<vector>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Geometry>
#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<fstream>
#include<boost/algorithm/string.hpp>

using namespace std;
using namespace Eigen;

vector<vector<Vector2d>> GoalPoints{};

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
    Vector2d k=A.inverse()*p;
    Vector2d vec1_=cali2.spacePoint-cali1.spacePoint;
    Vector2d vec2_=cali3.spacePoint-cali1.spacePoint;
    p=cali1.spacePoint+k(0)*vec1_+k(1)*vec2_;
}


class Vehicle
{
private:
    double angle_offset;
    string Name;
public:
    Vector2d pos;
    Quaterniond ori;
    string name(){return Name;}
    double r;
    double theta()
    {
        Matrix3d R=ori.toRotationMatrix();
        return atan2(R(1,0),R(0,0))-angle_offset;
    }
    Vehicle(string name_, double r_=0.2, double angleOffset=0){Name=name_;r=r_;angle_offset=angleOffset;}
    ~Vehicle(){}
};

vector<Vehicle> vehicles{Vehicle("Burger0",0.3,0),Vehicle("Burger1",0.3,0),Vehicle("Burger2",0.3,0)};


class WaffleMove
{
private:
    ros::NodeHandle nh;
    ros::Publisher Move;
    int VehicleNum;
    vector<geometry_msgs::Twist> cmds;

    void ViconConvert(Vehicle &v,geometry_msgs::TransformStampedConstPtr Ptr)
    {
        v.pos(0)=Ptr->transform.translation.x;
        v.pos(1)=Ptr->transform.translation.y;
        v.ori.w()=Ptr->transform.rotation.w;
        v.ori.x()=Ptr->transform.rotation.x;
        v.ori.y()=Ptr->transform.rotation.y;
        v.ori.z()=Ptr->transform.rotation.z;
    }    

    void ViconCallBack(const geometry_msgs::TransformStampedConstPtr &Ptr,const geometry_msgs::TransformStampedConstPtr &Ptr1,const geometry_msgs::TransformStampedConstPtr &Ptr2)
    {
        //Obtain Messages
        ViconConvert(vehicles[0],Ptr);
        ViconConvert(vehicles[1],Ptr1);
        ViconConvert(vehicles[2],Ptr2);
        dataProcess();

        //Publish Commands
        for(int i=0;i<VehicleNum;i++)
        {
            Move=nh.advertise<geometry_msgs::Twist>("/"+vehicles[i].name()+"/cmd_vel",10);
            Move.publish(cmds[i]);
        }
    }
    
    void dataProcess();

    void GoalPointDecode(const std_msgs::StringConstPtr &Ptr)
    {
        vector<string> SplitedStr,temp;
        boost::split(SplitedStr,Ptr->data,boost::is_any_of(" \n\t"));
        int cnt=0;
        GoalPoints.clear();
        double x1,y1,x2,y2,x3,y3;
        for (int i=0;i<SplitedStr.size();i++)
        {
            // cout<<i<<": "<<SplitedStr[i]<<"  size: "<<SplitedStr[i].size()<<endl;
            if(SplitedStr[i].size()>0)temp.push_back(SplitedStr[i]);
        }
        SplitedStr=temp;

        x1=stod(SplitedStr[0]);
        y1=stod(SplitedStr[1]);
        x2=stod(SplitedStr[2]);
        y2=stod(SplitedStr[3]);
        x3=stod(SplitedStr[4]);
        y3=stod(SplitedStr[5]);
        Vector2d v1(x1,y1),v2(x2,y2),v3(x3,y3);
        Mapping(v1);
        Mapping(v2);
        Mapping(v3);
        GoalPoints.push_back(vector<Vector2d> {v1});
        GoalPoints.push_back(vector<Vector2d> {v2});
        GoalPoints.push_back(vector<Vector2d> {v3});
        // while(cnt<VehicleNum)
        // {
        //     boost::split(temp,SplitedStr[cnt],boost::is_any_of(" ,"));
        //     vector<Vector2d> Goal_i;
        //     double x,y;
        //     x=stod(temp[0]);
        //     y=stod(temp[1]);
        //     Goal_i.push_back(Vector2d(x,y));
        //     GoalPoints.push_back(Goal_i);
        //     cnt+=1;
        // }
        for(int i=0;i<GoalPoints.size();i++)
        {
            cout<<i+1<<":  ";
            for(int j=0;j<GoalPoints[i].size();j++)
            {
                cout<<GoalPoints[i][j].transpose()<<",\t";
            }
            cout<<endl;
        }

    }
    
public:
    WaffleMove()
    {
        ros::NodeHandle nh;
        ros::Rate rate(10);
        VehicleNum=vehicles.size();
        for(int i=0;i<VehicleNum;i++)
        {
            geometry_msgs::Twist cmd;
            cmd.angular.x=0;cmd.angular.y=0;cmd.angular.z=0;
            cmd.linear.x=0;cmd.linear.y=0;cmd.linear.z=0;
            cmds.push_back(cmd);
        }

        message_filters::Subscriber<geometry_msgs::TransformStamped> sub0(nh,"/vicon/"+vehicles[0].name()+"/"+vehicles[0].name(),1);
        message_filters::Subscriber<geometry_msgs::TransformStamped> sub1(nh,"/vicon/"+vehicles[1].name()+"/"+vehicles[1].name(),1);
        message_filters::Subscriber<geometry_msgs::TransformStamped> sub2(nh,"/vicon/"+vehicles[2].name()+"/"+vehicles[2].name(),1);
        message_filters::Subscriber<std_msgs::String> PointSub(nh,"/dso/positions",1);

        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TransformStamped,geometry_msgs::TransformStamped,geometry_msgs::TransformStamped> myPolicy;
        message_filters::Synchronizer<myPolicy> sync(myPolicy(10),sub0,sub1,sub2);
        sync.registerCallback(boost::bind(&WaffleMove::ViconCallBack,this,_1,_2,_3));
        PointSub.registerCallback(&WaffleMove::GoalPointDecode,this);

        while(ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
        cout<<"Exit\n";
    }
    
    ~WaffleMove(){}
};



void WaffleMove::dataProcess()//在这里处理数据
{
    //Multi-Agent Motion
    for(int i=0;i<GoalPoints.size();i++)
    {
        if(GoalPoints[i].size()==0){cmds[i].linear.x=0;cmds[i].angular.z=0;}
        else
        {
            if((vehicles[i].pos-GoalPoints[i][0]).norm()<0.1)GoalPoints[i].erase(GoalPoints[i].begin());
            Vector2d d_=GoalPoints[i][0]-vehicles[i].pos;
            double theta=vehicles[i].theta();
            Matrix2d M;
            M(0,0)=cos(theta);M(0,1)=-vehicles[0].r*sin(theta);M(1,0)=sin(theta);M(1,1)=vehicles[i].r*cos(theta);
            Vector2d temp=3*M.inverse()*d_;
            cmds[i].linear.x=temp(0);cmds[i].angular.z=temp(1);
        }
    }

    
}

int main(int argc,char** argv)
{
    cali1.spacePoint=Vector2d(0.712,1.071);cali1.imagePoint=Vector2d(-2.5,2.5);
    cali2.spacePoint=Vector2d(0.796,-1.133);cali2.imagePoint=Vector2d(2.5,2.5);
    cali3.spacePoint=Vector2d(-0.95,0.363);cali3.imagePoint=Vector2d(-1,-1.5);

    ros::init(argc,argv,"dso");
    WaffleMove mywaffle;
    return 0;
}