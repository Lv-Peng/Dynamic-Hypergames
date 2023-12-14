#include<iostream>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/TransformStamped.h>
#include<vector>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Geometry>
#include<NavigationLib.h>
#include<ctime>
#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<fstream>
#include<boost/algorithm/string.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;

NavigationLib::Path pth;
vector<vector<Vector2d>> goalPoints{};
bool flag=false;

static vector<Vector2d> picVecs,realVecs;//校准参数

//场地参数
// const Vector2d p1(-1.3,0.8),p2(0.84,-1.23);

// Vector2d PointMap(Vector2d source)  //提供图像和动捕的校准信息
// {
//     double x=source(0)/800*(p2(0)-p1(0))+p1(0),z=source(1)/800*(p2(1)-p1(1))+p1(1);
//     return Vector2d(x,z);
// }

// vector<Vector2d> PointMap(vector<Vector2d> source)
// {
//     vector<Vector2d> v;
//     for(int i=0;i<source.size();i++)v.push_back(PointMap(source[i]));
//     return v;
// }

Vector2d SpecialTransform(Vector2d a,Vector2d b,Vector2d c,Vector2d d,Vector2d Mdesired)
{
    //MATLAB地图尺寸
    Vector2d Ma(-2.5,2.5),Mb(2.5,2.5),Mc(2.5,-2.5),Md(-2.5,-2.5);
    double width=Mb(0)-Ma(0),height=Ma(1)-Md(1);
    double prop_width=(Mdesired(0)-Ma(0))/width,prop_height=(Mdesired(1)-Md(1))/height;
    Vector2d mid1=c+(d-c)*prop_width,mid2=a+(b-a)*prop_width;
    return mid1+(mid2-mid1)*prop_height;
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

vector<Vehicle> vehicles{Vehicle("Burger0",0.2,0.056)};
//Vehicle("Burger1",0.2,0),Vehicle("Burger2",0.2,0)};


class WaffleMove
{
private:
    ros::NodeHandle nh;
    ros::Publisher Move;
    int num;
    vector<geometry_msgs::Twist> cmds;
    void ViconConvert(Vehicle &v,geometry_msgs::TransformStampedConstPtr Ptr)
    {
        v.pos(0)=Ptr->transform.translation.x;
        v.pos(1)=Ptr->transform.translation.y;
        v.ori.w()=Ptr->transform.rotation.w;
        v.ori.x()=Ptr->transform.rotation.x;
        v.ori.y()=Ptr->transform.rotation.y;
        v.ori.z()=Ptr->transform.rotation.z;
        cout<<v.theta()<<endl;
        // special
        // ofstream outputTxt("./pos.txt");
        // outputTxt<<v.pos.transpose()<<endl;
    }    

    void ViconCallBack(const geometry_msgs::TransformStampedConstPtr &Ptr)
    {
        //Obtain Messages
        // cout<<"2\n";
        ViconConvert(vehicles[0],Ptr);
        dataProcess();

        //Publish Commands
        for(int i=0;i<num;i++)
        {
            Move=nh.advertise<geometry_msgs::Twist>("/"+vehicles[i].name()+"/cmd_vel",10);
            cout<<"/"+vehicles[i].name()+"/cmd_vel"<<endl;
            Move.publish(cmds[i]);
        }
    }

    void ViconCallBack_multi(const geometry_msgs::TransformStampedConstPtr &Ptr1,const geometry_msgs::TransformStampedConstPtr &Ptr2,const geometry_msgs::TransformStampedConstPtr &Ptr3)
    {
        //Obtain Messages
        // cout<<"2\n";
        ViconConvert(vehicles[0],Ptr1);
        ViconConvert(vehicles[1],Ptr2);
        ViconConvert(vehicles[2],Ptr3);
        dataProcess();

        //Publish Commands
        for(int i=0;i<num;i++)
        {
            Move=nh.advertise<geometry_msgs::Twist>("/"+vehicles[i].name()+"/cmd_vel",10);
            Move.publish(cmds[i]);
        }
    }
    
    void dataProcess();
    
public:
    NavigationLib::Region R;
    WaffleMove(vector<vector<Vector2d>> map)
    {
        // //测试
        // Vector2d tmppp;
        // for(int i=0;i<map.size();i++)
        // {
        //     tmppp(0)=0;tmppp(1)=0;
        //     for(int j=0;j<map[i].size();j++)
        //     {
        //         tmppp+=map[i][j];
        //     }
        //     tmppp/=map[i].size();
        //     a.push_back(tmppp);
        // }
        // //测试



        Mat ima=Mat::zeros(800,800,CV_8UC3);
        R=NavigationLib::Region(map);
        // R.DrawMap(ima,1,1);
        ros::NodeHandle nh;
        ros::Rate rate(10);
        num=vehicles.size();
        for(int i=0;i<num;i++)
        {
            geometry_msgs::Twist cmd;
            cmd.angular.x=0;cmd.angular.y=0;cmd.angular.z=0;
            cmd.linear.x=0;cmd.linear.y=0;cmd.linear.z=0;
            cmds.push_back(cmd);
        }
        message_filters::Subscriber<geometry_msgs::TransformStamped> sub0(nh,"/vicon/"+vehicles[0].name()+"/"+vehicles[0].name(),1);
        

        // message_filters::Subscriber<geometry_msgs::TransformStamped> sub1(nh,"/vicon/"+vehicles[0].name()+"/"+vehicles[0].name(),1),sub2(nh,"/vicon/"+vehicles[1].name()+"/"+vehicles[1].name(),1),sub3(nh,"/vicon/"+vehicles[2].name()+"/"+vehicles[2].name(),1);
        // for(int i=0;i<vehicles.size();i++)subs_.push_back(message_filters::Subscriber<geometry_msgs::TransformStamped>(nh,"/vicon/"+vehicles[i].name()+"/"+vehicles[i].name(),1));
        //多机器人的时候用这个
        // typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TransformStamped,geometry_msgs::TransformStamped,geometry_msgs::TransformStamped> myPolicy;
        // message_filters::Synchronizer<myPolicy> sync(myPolicy(10),sub1,sub2,sub3);
        // sync.registerCallback(boost::bind(&WaffleMove::ViconCallBack_multi,this,_1,_2,_3));
        sub0.registerCallback(&WaffleMove::ViconCallBack,this);

        while(ros::ok())
        {
            ros::spinOnce();
            // cout<<"1\n";
            // ros::spin();
            // rate.sleep();
            // char key=waitKey(1);
            // if(key==27)break;
        }
        cout<<"Exit\n";
    }
    
    ~WaffleMove(){}
};



void WaffleMove::dataProcess()//在这里处理数据
{
    // cmds[0].angular.z=1;
    if(!flag)
    {
        // NavigationLib::Path pth=R.SimpleNavigation(vector<int>{3,1,2,4,1,3,2},Vector2d(0,0));
        // a=R.PathConvert(pth);
        // a=PointMap(a);//转换到实际坐标
        flag=true;
    }

    // //special 
    // ifstream reader("./goals.txt");
    // string temp_str;
    // while(getline(reader,temp_str))
    // {
    //     vector<string> SplitedStr;
    //     boost::split(SplitedStr,temp_str,boost::is_any_of(" "));
    //     int cnt=0;
    //     vector<double> Xs;
    //     while(cnt<SplitedStr.size())
    //     {
    //         if(cnt%2==0)Xs.push_back(stod(SplitedStr[cnt]));
    //         else a.push_back(Vector2d(Xs[cnt],stod(SplitedStr[cnt])));
    //     }
    // }

    //

    for(int agent=0;agent<vehicles.size();agent++)
    {
        cout<<agent<<":";
        for(int ii=0;ii<goalPoints[agent].size();ii++)cout<<goalPoints[agent][ii].transpose()<<"\t\t";
        cout<<endl<<endl;
        if(goalPoints[agent].size()==0){cmds[agent].linear.x=0;cmds[agent].angular.z=0;}
        else
        {
            if((vehicles[agent].pos-goalPoints[agent][0]).norm()<0.1)goalPoints[agent].erase(goalPoints[agent].begin());
            Vector2d d_=goalPoints[agent][0]-vehicles[agent].pos;
            // cout<<"d_:"<<d_.transpose()<<endl;
            double theta=vehicles[agent].theta();
            // cout<<"theta:"<<theta<<endl;
            Matrix2d M;
            M(0,0)=cos(theta);M(0,1)=-vehicles[agent].r*sin(theta);M(1,0)=sin(theta);M(1,1)=vehicles[agent].r*cos(theta);
            Vector2d temp=3*M.inverse()*d_;
            // cout<<"v and w:"<<temp.transpose()<<endl<<endl;
            cmds[agent].linear.x=temp(0);cmds[agent].angular.z=temp(1);
        }

    }

    
}

int main(int argc,char** argv)
{
    vector<vector<Vector2d>> a{{}};

    ifstream cali("./src/create_map_test/src/picCalibration.txt");
    string s;
    if (!cali.is_open())cerr<<"error:no calibration file";
    else
    {
        vector<string> temp;
        while(getline(cali,s))
        {
            boost::split(temp,s,boost::is_any_of(": ,"));
            picVecs.push_back(Vector2d(stod(temp[1]),stod(temp[2])));
            realVecs.push_back(Vector2d(stod(temp[3]),stod(temp[4])));
        }
    }

    NavigationLib::MapGeneration(a);

    for(int i=0;i<a[0].size();i++)a[0][i]=NavigationLib::Trans(picVecs,realVecs,a[0][i]);
    

    ifstream read("./routes.txt");
    if (!read.is_open())cerr<<"error:no route file";
    else
    {
        vector<string> temp,coordinate;
        int agent=0;
        vector<Vector2d> goals;
        while(getline(read,s))
        {
            goals.clear();
            boost::split(temp,s,boost::is_any_of(";"));
            for(int i=0;i<temp.size();i++)
            {
                boost::split(coordinate,temp[i],boost::is_any_of(" "));
                Vector2d g=SpecialTransform(a[0][0],a[0][1],a[0][2],a[0][3],Vector2d(stod(coordinate[0]),stod(coordinate[1])));
                goals.push_back(g);
                cout<<g.transpose()<<";";
            }
            cout<<endl;
            goalPoints.push_back(goals);
        }
        
    }



    // cout<<a.size()<<"\nsb!\n";
    // for(int i=0;i<a.size();i++)
    // {
    //     cout<<"i="<<i<<endl;
    //     for(int j=0;j<a[i].size();j++)
    //     {cout<<a[i][j].transpose()<<",\t";}
    //     cout<<endl<<endl;;
    // }
    // NavigationLib::MapGeneration(a);

    // cout<<",,\n";
    destroyAllWindows();
    ros::init(argc,argv,"dso");
    WaffleMove mywaffle(a);
    return 0;
}