#include<iostream>
#include<ros/ros.h>
#include<string>
#include<std_msgs/String.h>
#include<message_filters/subscriber.h>
#include<control_msgs/FollowJointTrajectoryActionGoal.h>
#include<control_msgs/FollowJointTrajectoryAction.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<sensor_msgs/JointState.h>
#include<boost/algorithm/string.hpp>
#include<eigen3/Eigen/Core>//矩阵
#include<math.h>

using namespace std;

//传8个参数，第一个为时间，即多少秒达到；第二个至第七个为goal_position；第八个为角度弧度制（1：角度）
ros::Publisher Move;
control_msgs::FollowJointTrajectoryActionGoal goal;
//trajectory_msgs::JointTrajectory cmd;

double pos_0[6];
double vel_0[6];
double acc_0[6];
double goal_pos[6];
//double goal_vel[6];//sensor+input -->  goal --> algorithm  -->  goal
//sensor+input-->algo by moveit-->retrive data from /goal-->send

//goal_pos gp0, gp1;
//velocities v0, v1;
uint32_t t0 = 0, t1 = 0;

int delta_t = 0;
 ros::Time t_h ;
 ros::V_string name;
 ros::Time t;
void spline1(const uint32_t &t0, const double x0[],const uint32_t &t1, const double x1[], const int &delta_t);//0:initial  1:goal   

void urCallback(const sensor_msgs::JointStateConstPtr& Ptr)
{
    int n = 6;
    //std::cout<<"urCallback\n";
    t_h = Ptr->header.stamp;
    name = Ptr->name;
    
    //main:Sub.registerCallback(&urCallback);
    for(int j = 0; j < n; ++j)
    {
        pos_0[j] = Ptr->position[j];
        vel_0[j] = Ptr->velocity[j];
        acc_0[j] = 0;
    }
   // cout <<  "pos_0:" << pos_0[0] << ' '<< pos_0[1] << ' '<< pos_0[2] << ' '<< pos_0[3] << ' '<< pos_0[4] << ' '<< pos_0[5] << ' ' << endl;
   // cout  << "vel_0"<< vel_0[1] << ' '<< vel_0[1] << ' '<< vel_0[2] << ' '<< vel_0[3] << ' '<< vel_0[4] << ' '<< vel_0[5] << ' ' << endl;

}

void GoalPointDecode(const std_msgs::StringConstPtr &Ptr)
{
    cout << "GoalPointDecode\n";
    t = ros::Time::now();
    t0 = t.sec*1e9 + t.nsec;//
    
    vector<string> SplitedStr,temp;
    boost::split(SplitedStr,Ptr->data,boost::is_any_of(" \n\t"));
    int cnt=0;
    double t_goal,p;
    for (int i=0;i<SplitedStr.size();i++)
    {
        //cout<<i<<": "<<SplitedStr[i]<<"  size: "<<SplitedStr[i].size()<<endl;
        if(SplitedStr[i].size()>0) temp.push_back(SplitedStr[i]);
    }
    SplitedStr=temp;

    t_goal=stod(SplitedStr[0]);
    
    t_goal = int(t_goal);
    delta_t = t_goal;
    t1 = t0 + t_goal*1e9;//uint32_t + int  ???
    goal_pos[0] = stod(SplitedStr[1]);
    goal_pos[1] = stod(SplitedStr[2]);
    goal_pos[2] = stod(SplitedStr[3]);
    goal_pos[3] = stod(SplitedStr[4]);
    goal_pos[4] = stod(SplitedStr[5]);
    goal_pos[5] = stod(SplitedStr[6]);
//0:rad          1:angle
    int flag=stod(SplitedStr[7]);
    if (flag==1) 
    {
        for(int k = 0;k < 6;++k)
        {
            goal_pos[k] = goal_pos[k]*2*3.14/360;
        }
    }
    if (flag!=0 && flag!=1){cout << "error value" << endl; }
    
    cout << "goal_pos: " << goal_pos[0] << ' ' << goal_pos[1]<< ' ' << goal_pos[2]<< ' ' << goal_pos[3]<< ' ' << goal_pos[4]<< ' ' << goal_pos[5] << endl;
    
    spline1(t0, pos_0,t1, goal_pos, delta_t);
    Move.publish(goal);
    goal.goal.trajectory.points.clear();
    goal.goal.trajectory.joint_names.clear();
}

void cubic_spline(const double *time_sec,const double *pos, double *M_, const int &num )
{
    //input: t0_sec,  t0_nsec,  pos_0[6];  goal_t_sec,  goal_t_nsec,  goal_pos[6];
    //long long  t_s = goal_t_sec - t0_sec;//每一秒一个点
    //long long t_ns = goal_t_nsec - t0_nsec;
    //追赶法解方程求二阶偏导数
    double f1 = 0, f2 = 0;
    
    double *a=new double[num];                //  a:稀疏矩阵最下边一串数
    double *b=new double[num];                //  b:稀疏矩阵最中间一串数
    double *c=new double[num];                //  c:稀疏矩阵最上边一串数
    double *d=new double[num];
 
    double *f=new double[num];
 
    double *bt=new double[num];
    double *gm=new double[num];
 
    double *h=new double[num];
 
    for(int i=0;i<num;i++)
        b[i]=2;                                //  中间一串数为2
    for(int i=0;i<num-1;i++)
        h[i]=time_sec[i+1]-time_sec[i];      // 各段步长
    for(int i=1;i<num-1;i++)
        a[i]=h[i-1]/(h[i-1]+h[i]);
    a[num-1]=1;
 
    c[0]=1;
    for(int i=1;i<num-1;i++)
        c[i]=h[i]/(h[i-1]+h[i]);
 
    for(int i=0;i<num-1;i++)
        f[i]=(pos[i+1]-pos[i])/(time_sec[i+1]-time_sec[i]);
 
    for(int i=1;i<num-1;i++)
        d[i]=6*(f[i]-f[i-1])/(h[i-1]+h[i]);
 
    //  追赶法求解方程
    //boundtype_first_derivative
        d[0]=6*(f[0]-f1)/h[0];
        d[num-1]=6*(f2-f[num-2])/h[num-2];
 
        bt[0]=c[0]/b[0];
        for(int i=1;i<num-1;i++)
            bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);
 
        gm[0]=d[0]/b[0];
        for(int i=1;i<=num-1;i++)
            gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);
 
        M_[num-1]=gm[num-1];
        for(int i=num-2;i>=0;i--)
            M_[i]=gm[i]-bt[i]*M_[i+1];
    // delete a;
    // delete b;
    // delete c;
    // delete d;
    // delete f;
    // delete gm;
    // delete bt;
    // delete h;
    //return;
}

void getYbyX(double &x_in, double &y_out,const double *M_,const double *time_sec, const double *pos, int n, double &ve, double &ac)
{
    //cout << "getYbyX: " << x_in << " " << y_out<< " "  << time_sec[1] << " " << ve << " " << ac <<  " " << M_[0] << endl;//getYbyX: 1 0 1 7.70397e-43 6.95262e-310 -nan
    int klo,khi,k;
    klo=0;
    khi=n-1;
    double hh,bb,aa;
 
    //  二分法查找x所在区间段
    while(khi-klo>1)
    {
        k=(khi+klo)>>1;
        if(time_sec[k]>x_in) 
            khi=k;
        else
            klo=k;
    }
    hh=time_sec[khi]-time_sec[klo];
 
    aa=(time_sec[khi]-x_in)/hh;
    bb=(x_in-time_sec[klo])/hh;
 
    y_out=aa*pos[klo]+bb*pos[khi]+((aa*aa*aa-aa)*M_[klo]+(bb*bb*bb-bb)*M_[khi])*hh*hh/6.0;
 
    //test
    ac = (M_[klo]*(time_sec[khi]-x_in) + M_[khi]*(x_in - time_sec[klo])) / hh;
    ve = M_[khi]*(x_in - time_sec[klo]) * (x_in - time_sec[klo]) / (2 * hh)
          - M_[klo]*(time_sec[khi]-x_in) * (time_sec[khi]-x_in) / (2 * hh)
          + (pos[khi] - pos[klo])/hh
          - hh*(M_[khi] - M_[klo])/6;
    printf("[---位置、速度、加速度---]");
    printf("%0.9f, %0.9f, %0.9f\n",y_out, ve, ac);
    //test end
    //cout << "getYbyX run\n";
    return;
}
 

void spline1(const uint32_t &t0, const double x0[],const uint32_t &t1, const double x1[], const int &delta_t)//0:initial  1:goal   
{
    
    uint32_t delta_t_ = (t1-t0);//ns
    //delta_t = delta_t_*1e-9;//s
    //delta_t = 1;
    int n_ = delta_t*5;//采样时间间隔为0.01s
    int n = n_ + 1;//11个点
   
    double *t_out = new double[n];
    double *M_ = new double[n];
    

    double pos0[n], pos1[n], pos2[n], pos3[n], pos4[n],pos5[n];
    double acc0[n],acc1[n],acc2[n],acc3[n],acc4[n],acc5[n];
    double vel0[n],vel1[n],vel2[n],vel3[n],vel4[n],vel5[n];
    //cout << pos0[0] << " at 241" << endl;

    double a_0[6];
    double b_0[6];//=0spline1
    double c_0[6];
    double d_0[6];

    for(int i = 0; i < n;++i)//n个point
    {
        //double as = 0.01*i;//0.1=1/n
        t_out[i] =  0.2*i;//t0:sec, 1e9???
        //cout << t0 << " " << n << " " << as << " " << t_out[i] << endl;
        
    }

    //cout << "x0: " << x0[0] << endl;
    for(int j  = 0; j < 6; ++j)//j个joint
    {
        //cout << x0[j] << ' ' << x1[j] << endl;
        a_0[j] = x0[j];
        b_0[j] = 0;
        c_0[j] = 0;
        d_0[j] =  (x1[j] - x0[j])/delta_t/delta_t/delta_t; //如果x1-x0太小/delta_t太小了呢？
    }

    for(int i = 0; i < n; ++i)
    {
        pos0[i] = a_0[0] + b_0[0]*(t_out[i])+c_0[0]*(t_out[i])*(t_out[i])+d_0[0]*(t_out[i])*(t_out[i])*(t_out[i]);
        pos1[i] = a_0[1] + b_0[1]*(t_out[i])+c_0[1]*(t_out[i])*(t_out[i])+d_0[1]*(t_out[i])*(t_out[i])*(t_out[i]);
        pos2[i] = a_0[2] + b_0[2]*(t_out[i])+c_0[2]*(t_out[i])*(t_out[i])+d_0[2]*(t_out[i])*(t_out[i])*(t_out[i]);
        pos3[i] = a_0[3] + b_0[3]*(t_out[i])+c_0[3]*(t_out[i])*(t_out[i])+d_0[3]*(t_out[i])*(t_out[i])*(t_out[i]);
        pos4[i] = a_0[4] + b_0[4]*(t_out[i])+c_0[4]*(t_out[i])*(t_out[i])+d_0[4]*(t_out[i])*(t_out[i])*(t_out[i]);
        pos5[i] = a_0[5] + b_0[5]*(t_out[i])+c_0[5]*(t_out[i])*(t_out[i])+d_0[5]*(t_out[i])*(t_out[i])*(t_out[i]);
    }
    //cout << "spline1 run\n";
    //cout <<"a[k]:, b[k]: " << endl;
    /*for(int k = 0; k< 6;++k)
    {
        cout <<   c_0[k] << " ";//0
        cout << d_0[k] << " ";//0
    }*/
    cout << endl;
    cubic_spline(t_out, pos0, M_,n);
    
    for(int k = 0; k < n;++k)
    {
        //cout << t_out[k] << " ";//1
        getYbyX(t_out[k],pos0[k],M_,t_out,pos0,n,vel0[k],acc0[k]);   
    }
    cout << endl;
    cubic_spline(t_out, pos1,M_, n);
     for(int k = 0; k < n;++k)
    {
        getYbyX(t_out[k],pos1[k],M_,t_out,pos1,n,vel1[k],acc1[k]);
    }
    cout << endl;
    cubic_spline(t_out, pos2, M_, n);
     for(int k = 0; k < n;++k)
    {
        getYbyX(t_out[k],pos2[k],M_,t_out,pos2,n,vel2[k],acc2[k]);
    }
    cout << endl;
    cubic_spline(t_out, pos3, M_, n);
     for(int k = 0; k < n;++k)
    {
        getYbyX(t_out[k],pos3[k],M_,t_out,pos3,n,vel3[k],acc3[k]);
    }
    cout << endl;
    cubic_spline(t_out, pos4, M_, n);
     for(int k = 0; k < n;++k)
    {
        getYbyX(t_out[k],pos4[k],M_,t_out,pos4,n,vel4[k],acc4[k]);
    }
    cout << endl;
    cubic_spline(t_out, pos5, M_, n);
     for(int k = 0; k < n;++k)
    {
        getYbyX(t_out[k],pos5[k],M_,t_out,pos5,n,vel5[k],acc5[k]);
    }

    int32_t t_sec[n];
    int32_t t_nsec[n];
    for(int j = 0; j < n; j++)
    {
        t_sec[j] = int(t_out[j]);
        t_nsec[j] = 1e9*(t_out[j] - int(t_out[j]));
    }
    goal.header.seq = 0;
    goal.header.stamp = t;
    goal.goal_id.stamp = t;
    std::vector<string> nam = name;//name:ros:V_string不能直接push_back
    
    for(int k =0; k < nam.size();k++)
    {
        goal.goal.trajectory.joint_names.push_back(nam[k]);
    }
    ros::V_string name_goal = goal.goal.trajectory.joint_names;
    //for(auto elem : name_goal)
   // {
   //     cout << elem << ' ';
   // }
   // cout << 360 << endl;
    for(int i = 0; i < n; ++i)
    {
        // cout<<goal.goal.trajectory.points.size()<<'\n';
        trajectory_msgs::JointTrajectoryPoint pnt;
        // vector<double> pos({0,0,0,0,0,0});
        goal.goal.trajectory.points.push_back(pnt);
        int32_t _sec, _nsec;
        _sec = t_sec[i];
        _nsec = t_nsec[i];
        
        goal.goal.trajectory.points[i].time_from_start=ros::Duration(_sec, _nsec);
        goal.goal.trajectory.points[i].positions.push_back(pos0[i]);
        goal.goal.trajectory.points[i].positions.push_back(pos1[i]);
        goal.goal.trajectory.points[i].positions.push_back(pos2[i]);
        goal.goal.trajectory.points[i].positions.push_back(pos3[i]);
        goal.goal.trajectory.points[i].positions.push_back(pos4[i]);
        goal.goal.trajectory.points[i].positions.push_back(pos5[i]);

        goal.goal.trajectory.points[i].velocities.push_back(vel0[i]);
        goal.goal.trajectory.points[i].velocities.push_back(vel1[i]);
        goal.goal.trajectory.points[i].velocities.push_back(vel2[i]);
        goal.goal.trajectory.points[i].velocities.push_back(vel3[i]);
        goal.goal.trajectory.points[i].velocities.push_back(vel4[i]);
        goal.goal.trajectory.points[i].velocities.push_back(vel5[i]);

        goal.goal.trajectory.points[i].accelerations.push_back(acc0[i]);
        goal.goal.trajectory.points[i].accelerations.push_back(acc1[i]);
        goal.goal.trajectory.points[i].accelerations.push_back(acc2[i]);
        goal.goal.trajectory.points[i].accelerations.push_back(acc3[i]);
        goal.goal.trajectory.points[i].accelerations.push_back(acc4[i]);
        goal.goal.trajectory.points[i].accelerations.push_back(acc5[i]);
    }
    //cout << 393 << endl;
    // delete t_out;
    /* delete pos0,pos1,pos2,pos3,pos4,pos5;
    delete vel0,vel1,vel2,vel3,vel4,vel5;
    // delete acc0,acc1,acc2,acc3,acc4,acc5; */
    // delete M_;
}

void tf(const double theta[])// joint state to xyz,   theta:rad
{
    using namespace Eigen;
   
double d[6] = {398,-0.299,0,556.925,0,165};//连杆偏移
double a[6] = {0,168.3,650.979,156.240,0,0};//连杆长度
double alpha[6] = {0,M_PI/2,0,M_PI/2,-M_PI/2,M_PI/2};//连杆扭角
//double theta[6] = {M_PI/3,M_PI/4,M_PI/5,M_PI/3,M_PI/4,M_PI/5};//关节转角

    MatrixXd T01 = MatrixXd::Ones(4,4);
    MatrixXd T12 = MatrixXd::Ones(4,4);
    MatrixXd T23 = MatrixXd::Ones(4,4);
    MatrixXd T34 = MatrixXd::Ones(4,4);
    MatrixXd T45 = MatrixXd::Ones(4,4);
    MatrixXd T56 = MatrixXd::Ones(4,4);
    MatrixXd T06 = MatrixXd::Ones(4,4);
    T01 << cos(theta[0]), -sin(theta[0]), 0, a[0],
           sin(theta[0])*cos(alpha[0]), cos(theta[0])*cos(alpha[0]), -sin(alpha[0]), -sin(alpha[0])*d[0],
           sin(theta[0])*sin(alpha[0]), cos(theta[0])*sin(alpha[0]), cos(alpha[0]), cos(alpha[0])*d[0],
           0,0,0,1;
    T12 << cos(theta[1]), -sin(theta[1]), 0, a[1],
           sin(theta[1])*cos(alpha[1]), cos(theta[1])*cos(alpha[1]), -sin(alpha[1]), -sin(alpha[1])*d[1],
           sin(theta[1])*sin(alpha[1]), cos(theta[1])*sin(alpha[1]), cos(alpha[1]), cos(alpha[1])*d[1],
           0,0,0,1;
    T23 << cos(theta[2]), -sin(theta[2]), 0, a[2],
           sin(theta[2])*cos(alpha[2]), cos(theta[2])*cos(alpha[2]), -sin(alpha[2]), -sin(alpha[2])*d[2],
           sin(theta[2])*sin(alpha[2]), cos(theta[2])*sin(alpha[2]), cos(alpha[2]), cos(alpha[2])*d[2],
           0,0,0,1;
    T34 << cos(theta[3]), -sin(theta[3]), 0, a[3],
           sin(theta[3])*cos(alpha[3]), cos(theta[3])*cos(alpha[3]), -sin(alpha[3]), -sin(alpha[3])*d[3],
           sin(theta[3])*sin(alpha[3]), cos(theta[3])*sin(alpha[3]), cos(alpha[3]), cos(alpha[3])*d[3],
           0,0,0,1;
    T45 << cos(theta[4]), -sin(theta[4]), 0, a[4],
           sin(theta[4])*cos(alpha[4]), cos(theta[4])*cos(alpha[4]), -sin(alpha[4]), -sin(alpha[4])*d[4],
           sin(theta[4])*sin(alpha[4]), cos(theta[4])*sin(alpha[4]), cos(alpha[4]), cos(alpha[4])*d[4],
           0,0,0,1;
    T56 << cos(theta[5]), -sin(theta[5]), 0, a[5],
           sin(theta[5])*cos(alpha[5]), cos(theta[5])*cos(alpha[5]), -sin(alpha[5]), -sin(alpha[5])*d[5],
           sin(theta[5])*sin(alpha[5]), cos(theta[5])*sin(alpha[5]), cos(alpha[5]), cos(alpha[5])*d[5],
           0,0,0,1;
    T06 = T01*T12*T23*T34*T45*T56;
    cout << T06 << endl;

}

int main(int argc, char ** argv)
{
    //string ObjectName = argv[1];
    ros::init(argc,argv,"PointActuation");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    // ros::Publisher Move;
    
    message_filters::Subscriber<sensor_msgs::JointState> Sub(nh,"/joint_states",1);
    message_filters::Subscriber<std_msgs::String> PointSub(nh,"/position",1);
    //cout <<"before sub" << endl;
    Move=nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal",100);    
    Sub.registerCallback(&urCallback);
    //cout << "after sub" << endl;
    PointSub.registerCallback(&GoalPointDecode);

    // Move.publish(goal);
    // std::bind()
    //PointSub.registerCallback(&GoalPointDecode);
    std::cout<<"main\n";
    while(ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
        // Move.publish(goal);
    }
    cout << "Exit\n";
    return 0;
}