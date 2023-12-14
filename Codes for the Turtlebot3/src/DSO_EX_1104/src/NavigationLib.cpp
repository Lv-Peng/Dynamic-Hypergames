#include<unistd.h>
#include<iostream>
#include<vector>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Geometry>
#include<opencv2/opencv.hpp>
#include<opencv2/core.hpp>
#include<NavigationLib.h>
#include<fstream>
#include<sstream>
#include<boost/algorithm/string.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace NavigationLib;

//全局变量
static vector<Vector2d> picVecs,realVecs;//校准参数

Point2d NavigationLib::CvPoint_from_Eigen(Vector2d a){return Point2d(a(0),a(1));}

Vector3d NavigationLib::FindLine(Vector2d Pt1,Vector2d Pt2)
{
    Matrix3d A;
    A<<Pt1(0),Pt1(1),1,
            Pt2(0),Pt2(1),1,
            rand()%10,rand()%10,0;
    Vector3d c(0,0,1);
    return (A.inverse()*c).normalized();
}

Vector2d NavigationLib::GetDist_Point2Segment(Vector2d a,Vector2d b,Vector2d c)
{
    Vector2d Dir;
    if ((a-b).dot(a-c)>0)
    {
        Dir=a-(b+c)/2;
    }
    else
    {
        Vector3d l=FindLine(b,c);
        Matrix2d A;
        A<<l[0],l[1],l[1],-l[0];
        Vector2d v(l[2],l[0]*a[1]-l[1]*a[0]);
        Vector2d xx=A.inverse()*v;
        Dir=a-xx;
    }
    return Dir;
}

bool NavigationLib::IntersectionCheck(Vector2d a,Vector2d b,Vector2d c, Vector2d d)//For segments
{
    Vector3d ab=FindLine(a,b), cd=FindLine(c,d);
    Vector2d v(ab(2),cd(2));
    Matrix2d A;
    A<<ab(0),ab(1),cd(0),cd(1);
    if(abs(A.determinant())<0.001)
    {
        if(abs(ab(2)-cd(2))<0.001)
        {
            int xmax=MAX(a(0),b(0)),xmin=MIN(a(0),b(0));
            bool flag1=xmax>c(0) && xmax>d(0);
            bool flag2=xmin<c(0) && xmin<d(0);
            bool flag3=xmax<c(0) && xmax<d(0);
            bool flag4=xmin<c(0) && xmin<d(0);
            if((flag1&&flag2)||(flag3&&flag4))return false;
            else return true;
        }
        else return false;
    }
    else
    {
        Vector2d x=-A.inverse()*v;
        if ( ((a-x).dot(b-x)<0 && (c-x).dot(d-x)<0) || (abs((a-x).dot(b-x)*(c-x).dot(d-x))<0.001 && (a-x).dot(b-x)*(c-x).dot(d-x)<=0))
            return true;
    }
    return false;
}

Path NavigationLib::Bellman_Ford(MatrixXd Laplace,int begin,int finish)
{
    Path path;
    int n=Laplace.rows();
    VectorXd cost=VectorXd::Ones(n)*999999;
    cost(begin)=0;
    VectorXd pre=VectorXd::Ones(n)*-1;
    for(int i=0;i<n;i++)
    {
        for(int k=0;k<n;k++)
        {
            for(int j=0;j<n;j++)
            {
                if(cost(j)+Laplace(j,k)<cost(k)&&Laplace(j,k)>=0)
                {
                    cost(k)=cost(j)+Laplace(j,k);
                    pre(k)=j;
                }
            }
        }
    }
    int temp=finish,counter=0;
    path.path.push_back(temp);
    bool flag=true;
    while (pre(temp)!=-1)
    {
        path.path.push_back(pre(temp));
        temp=pre(temp);
        counter++;
        if(counter>=3*n)
        {
            flag=false;
            break;
        }
    }
    if(flag&&cost(finish)!=999999)
    {
        path.Distance=cost(finish);
        // path.path.push_back(begin);
        reverse(path.path.begin(),path.path.end());
        // path.path.push_back(finish);
    }
    else
    {
        path.Distance=999999;
        path.path.clear();
    }
    return path;
}


Section::Section(vector<Vector2d> input, double dense, vector<int> ap)
{
    nodes=input;
    density=dense;
    AP=ap;
    Points=GraphGeneration();
}

Section::~Section(){}

// double Section::Area()//不够完美，这个算法对于五角星类似的图形是无效的
// {
//     double area=0;
//     Vector2d a,b;
//     Vector2d randomPoint;
//     double ka,kb;
//     for(int i=0;i<nodes.size()-2;i++)
//     {
//         randomPoint=nodes[i+1];
//         a=nodes[i]-nodes[i+1],b=nodes[i+2]-nodes[i+1];
//         ka=abs(double(rand()%98+1)/100);kb=abs(double(rand()%98+1)/100);
//         if(ka+kb>1)randomPoint+=(1-ka)*a+(1-kb)*b;
//         else randomPoint+=ka*a+kb*b;
//         if(PointCheck(randomPoint))
//         area+=abs(a[0]*b[1]-a[1]*b[0])/2;
//     }
// }

Vector2d Section::PointGeneration()
{
    while(true)
    {
        int i=rand()%nodes.size();
        int j=(i+1)%nodes.size();int k=(i+2)%nodes.size();
        Vector2d a=nodes[i]-nodes[j],b=nodes[k]-nodes[j];
        double aa=(rand()%9+1)/10,bb=(rand()%9+1)/10;
        if(aa+bb>1){aa=1-aa;bb=1-bb;}
        Vector2d Random=nodes[j]+aa*a+bb*b;
        if(PointCheck(Random))return Random;
    }
}

double Section::Area()//对于凹形会多算，但是其实多算一点可以多给这个区域分配一些节点
{
    double area=0;
    Vector2d a,b;
    for(int i=0;i<nodes.size();i++)
    {
        a=nodes[i+1]-nodes[i];b=nodes[i+1]-nodes[i+2];
        area+=abs(a(0)*b(1)-a(1)*b(0))/2;
    }
    return area;
}

bool Section::PointCheck(Vector2d a)
{
    bool flag=true;
    int counter=0,i;
    Vector2d temp(-1,-1);
    for(i=1;i<nodes.size()+1;i++)
    {
        if (IntersectionCheck(temp,a,nodes[i-1],nodes[i%nodes.size()]))counter++;
    }
    if (counter%2==0)flag=false;
    return flag;
}

bool Section::SegmentCheck(Vector2d a,Vector2d b)
{
    for(int i=1;i<nodes.size()+1;i++)
    {
        if(IntersectionCheck(a,b,nodes[i-1],nodes[i&nodes.size()]))return true;
    }
    return false;
}

vector<Vector2d> Section::GraphGeneration()
{
    int n=(int)(floor(Area()*density/10000))%16+1;
    vector<Vector2d> points;
    int i=0,counter=0,j=0;
    int SizeNodes=nodes.size();
    double ka,kb;
    Vector2d D;double d;
    Vector2d dv;
    vector<Vector2d> dvs;
    vector<int> deleteList;
    double dt=1;
    Vector2d a,b,randomPoint;
    // n=1;
    while (points.size()<n)
    {
        randomPoint=nodes[(counter+1)%SizeNodes];
        a=nodes[(counter)%SizeNodes]-nodes[(counter+1)%SizeNodes],b=nodes[(counter+2)%SizeNodes]-nodes[(counter+1)%SizeNodes];
        ka=abs(double(rand()%19+1)/20);kb=abs(double(rand()%19+1)/20);
        if(ka+kb>1)randomPoint+=(1-ka)*a+(1-kb)*b;
        else randomPoint+=ka*a+kb*b;
        if(PointCheck(randomPoint))points.push_back(randomPoint);
        counter++;
        if (i>=SizeNodes-2)i=0;
        if (counter>100)break;
    }
    counter=0;
    while(counter<100)
    {
        dvs.clear();deleteList.clear();
        for(i=0;i<points.size();i++)
        {
            dv=Vector2d::Zero();
            for (j=0;j<nodes.size();j++)
            {
                D=GetDist_Point2Segment(points[i],nodes[j],nodes[(j+1)%SizeNodes]);
                d=D.norm();
                dv-=70/d*D.normalized();
            }
            for (j=0;j<points.size();j++)
            {
                if (i!=j)
                {
                    d=(points[i]-points[j]).norm();
                    dv+=20/d*(points[i]-points[j]).normalized();
                }
            }
            dvs.push_back(dv);

        }
        for (i=0;i<points.size();i++)
        {
            if (PointCheck(points[i]+dvs[i]*dt))points[i]+=dvs[i]*dt;
            else deleteList.push_back(i);
        }

        
        //先关闭删除功能
        // if (deleteList.size()>0)
        // {
        //     for (i=deleteList.size();i>0;i--)
        //     {
        //         points.erase(points.begin()+deleteList[i-1]);
        //     }
        // }
        // //可视化
        // cout<<counter<<"::___";
        // for (i=0;i<dvs.size();i++)
        //     cout<<i<<":"<<dvs[i].transpose()<<";\t";
        // cout<<endl<<endl;
        // cv::Mat image=cv::Mat::zeros(cv::Size(800,800),CV_32FC3);
        // cv::Point2d aa,bb,cc,dd,ee;
        // aa.x=nodes[0][0];aa.y=nodes[0][1];
        // bb.x=nodes[1][0];bb.y=nodes[1][1];
        // cc.x=nodes[2][0];cc.y=nodes[2][1];
        // dd.x=nodes[3][0];dd.y=nodes[3][1];
        // ee.x=nodes[4][0];ee.y=nodes[4][1];
        // cv::line(image,aa,bb,cv::Scalar(255,255,255));cv::line(image,bb,cc,cv::Scalar(255,255,255));
        // cv::line(image,cc,dd,cv::Scalar(255,255,255));cv::line(image,dd,ee,cv::Scalar(255,255,255));
        // cv::line(image,ee,aa,cv::Scalar(255,255,255));

        // for (int i=0;i<points.size();i++)
        // {
        //     cv::circle(image,cv::Point2d(points[i][0],points[i][1]),5,cv::Scalar(255,0,0),3);
        //     // cout<<points[i]<<endl;
        // }
        // cv::imshow("test",image);cv::waitKey(0);

        counter++;
    }
    // cout<<dvs.size()<<endl;
    return points;
}

Region::Region(vector<Section> S)
{
    Sections=S;
    LaplaceMat=LaplaceConstruction();
    totol_Points_num=LaplaceMat.cols();
    for (int i=0;i<S.size();i++)colors.push_back(Scalar(rand()%255,rand()%255,rand()%255));
}

Region::Region(vector<vector<Vector2d>> S, int dense)
{
    if(S.back().size()<3)S.pop_back();
    int n=S.size();
    for(int i=0;i<n;i++)Sections.push_back(Section(S[i],dense));
    LaplaceMat=LaplaceConstruction();
    totol_Points_num=LaplaceMat.cols();
    for (int i=0;i<S.size();i++)colors.push_back(Scalar(rand()%255,rand()%255,rand()%255));
}

Region::~Region(){}

int Region::totalID(int SectionID,int nodeID)
{
    if(SectionID>Sections.size()-1)cout<<"Section Overrated\n";
    int ID=0;
    int i=0;
    while(i<SectionID)
    {
        ID+=Sections[i].Points.size();
        i++;
    }
    ID+=nodeID;
    return ID;
}

vector<int> Region::partialID(int totalID)
{
    if(totalID>totol_Points_num)return(vector<int>{-1,-1});
    int PointID=totalID;
    int SecID=0;
    while(true)
    {
        int tmp=Sections[SecID].Points.size();
        if(PointID-tmp<0)break;
        PointID-=tmp;
        SecID++;
    }
    return (vector<int>{SecID,PointID});
}

Vector2d Region::Point_(int totalID)
{
    vector<int> p=partialID(totalID);
    return Sections[p[0]].Points[p[1]];
}

void Region::DrawRegions(Mat Image, int width, int height, bool show, String winname)
{
    // Mat Image=Mat::zeros(width,height,CV_8U);
    if(Sections.size()>0)
    {
        for(int i=0;i<Sections.size();i++)
        {
            int n=Sections[i].nodes.size();
            for(int j=0;j<=n;j++)
            {
                int x1=Sections[i].nodes[j%n](0)*width;int y1=Sections[i].nodes[j%n](1)*height;
                int x2=Sections[i].nodes[(j+1)%n](0)*width;int y2=Sections[i].nodes[(j+1)%n](1)*height;
                Point2d a(x1,y1),b(x2,y2);
                line(Image,a,b,colors[i],3);
            }
        }
    }
    if(show)imshow(winname,Image);
}

void Region::DrawMap(Mat &Image, int width, int height, bool show, String winname)
{
    vector<Vector2d> temp;
    // Mat Image=Mat::zeros(width,height,CV_8U);
    if(Sections.size()>0)
    {
        for(int i=0;i<Sections.size();i++)
        {
            int n=Sections[i].nodes.size();
            for(int j=0;j<=Sections[i].nodes.size();j++)
            {
                int x1=Sections[i].nodes[j%n](0)*width;int y1=Sections[i].nodes[j%n](1)*height;
                int x2=Sections[i].nodes[(j+1)%n](0)*width;int y2=Sections[i].nodes[(j+1)%n](1)*height;
                Point2d a(x1,y1),b(x2,y2);
                line(Image,a,b,colors[i],6);
            }
            
            for(int j=0;j<Sections[i].Points.size();j++)
            {
                int x1=Sections[i].Points[j](0)*width;int y1=Sections[i].Points[j](1)*height;
                Point2d a(x1,y1);
                circle(Image,a,8,colors[i],5);
                temp.push_back(Sections[i].Points[j]);
            }
        }
    }
    int n=LaplaceMat.cols();
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<n;j++)
        {
            if(LaplaceMat(i,j)>0)line(Image,CvPoint_from_Eigen(temp[i]),CvPoint_from_Eigen(temp[j]),Scalar(255,255,255),1);
        }
    }
    if(show)imshow(winname,Image);
}

MatrixXd Region::LaplaceConstruction()
{
    int n_S=Sections.size();
    vector<int> Sections_Points_size;
    int total_size=0;
    for(int i=0;i<n_S;i++)
    {
        Sections_Points_size.push_back(Sections[i].Points.size());
        total_size+=Sections_Points_size[i];
    }
    
    MatrixXd L(total_size,total_size);
    L=-1*MatrixXd::Ones(total_size,total_size)+MatrixXd::Identity(total_size,total_size);

    int pre=0;
    for(int i=0;i<n_S;i++)
    {
        for(int j=0;j<Sections_Points_size[i]-1;j++)
        {
            for(int k=j+1;k<Sections_Points_size[i];k++)
            {
                L(pre+j,pre+k)=(Sections[i].Points[j]-Sections[i].Points[k]).norm();
                L(pre+k,pre+j)=L(pre+j,pre+k);
            }
        }
        pre+=Sections_Points_size[i];
    }
    
    int pre1=0,pre2=0;
    bool flag=true;
    for(int i=0;i<n_S-1;i++)
    {
        for(int j=0;j<Sections_Points_size[i];j++)
        {
            for(int ii=i+1;ii<n_S;ii++)
            {
                if(ii!=i)
                {
                    for(int jj=0;jj<Sections_Points_size[ii];jj++)
                    {
                        flag=true;
                        for (int k=0;k<n_S;k++)
                        {
                            if((k!=i)&&(k!=ii))
                            {
                                if(Sections[k].SegmentCheck(Sections[i].Points[j],Sections[ii].Points[jj]))
                                    flag=false;
                            }
                        }
                        if(flag)
                        {
                            int t1=totalID(i,j),t2=totalID(ii,jj);
                            L(t1,t2)=(Sections[i].Points[j]-Sections[ii].Points[jj]).norm();
                            L(t2,t1)= L(t1,t2);
                        }
                    }
                }
            }
        }
    }
    cout<<L<<endl;
    return L;
}

Path Region::Navigation(vector<int> desiredTransition, Vector2d StartPoint)
{
    Path pth;
    int id;
    int n_S=Sections.size();
    for(id=0;id<n_S;id++)
    {
        if(Sections[id].PointCheck(StartPoint))break;
    }
    double d=999999;
    int i;
    for(i=0;i<Sections[id].Points.size();i++)
    {
        double di=(Sections[id].Points[i]-StartPoint).norm();
        if(di<d){d=di;break;}
    }
    pth.Distance=d;
    pth.path.push_back(totalID(id,i));
    return pth+Navigation(desiredTransition,vector<int>{id,i});
}

Path Region::Navigation(vector<int> desiredTransition, vector<int> StartID)
{
    Path pth;
    if(desiredTransition.size()==0)
    {
        pth.Distance=0;
        pth.path={};
        return pth;
    }
    // cout<<"bp\n";
    pth.Distance=999999;
    int n_S=Sections.size();
    // cout<<"desiredTr="<<desiredTransition[0]<<endl;
    for(int i=0;i<Sections[desiredTransition[0]].Points.size();i++)
    {
        cout<<"startID=";for(int i=0;i<StartID.size();i++)cout<<StartID[i]<<"  ";cout<<endl;
        cout<<"desiredTr="<<desiredTransition[0]<<'\t'<<i<<endl;
        Path tmppth=Bellman_Ford(LaplaceMat,totalID(StartID[0],StartID[1]),totalID(desiredTransition[0],i));
        // cout<<"i="<<i<<endl;
        if(tmppth<pth)
            pth=tmppth;
    }
    desiredTransition.erase(desiredTransition.begin());
    return pth+Navigation(desiredTransition,partialID(pth.path.back()));
}

Path Region::SimpleNavigation(vector<int> desiredTransition, Vector2d StartPoint)
{
    Path pth;
    pth.Distance=0;pth.path={};
    if(desiredTransition.size()==0){cout<<"Kidding?";return pth;}
    else
    {
        for(int i=0;i<desiredTransition.size();)
        {
            int n=desiredTransition[0];
            pth.Distance+=(Sections[n].Points[0]-StartPoint).norm();
            pth.path.push_back(n);
            StartPoint=Sections[n].Points[0];
            desiredTransition.erase(desiredTransition.begin());
        }
    }
    return pth;
}

void Region::DrawPath(Mat &Image, Path pth, int width, int height, bool show, String winname)//path only
{
    int n=pth.path.size();
    Scalar color(0,255,255);
    for(int i=0;i<n-1;i++)
    {
        int a=pth.path[i],b=pth.path[i+1];
        vector<int> a_=partialID(a),b_=partialID(b);
        Point2d A,B;
        A=CvPoint_from_Eigen(Sections[a_[0]].Points[a_[1]]);
        B=CvPoint_from_Eigen(Sections[b_[0]].Points[b_[1]]);
        line(Image,A,B,color,2);
    }
    if(show)imshow(winname,Image);
}

void Region::DrawPath(Mat &Image, Path pth, Vector2d StartPoint, int width, int height, bool show, String winname)//path only
{
    int n=pth.path.size();
    vector<int> v=partialID(pth.path[0]);
    Point2d A=CvPoint_from_Eigen(StartPoint),B=CvPoint_from_Eigen(Sections[v[0]].Points[v[1]]);
    Scalar color(0,255,255);
    line(Image,A,B,color,2);
    for(int i=0;i<n-1;i++)
    {
        int a=pth.path[i],b=pth.path[i+1];
        vector<int> a_=partialID(a),b_=partialID(b);
        
        A=CvPoint_from_Eigen(Sections[a_[0]].Points[a_[1]]);
        B=CvPoint_from_Eigen(Sections[b_[0]].Points[b_[1]]);
        line(Image,A,B,color,2);
    }
    if(show)imshow(winname,Image);
}

void NavigationLib::on_Mouse(int event, int x, int y, int flags, void* param)
{
    vector<vector<Vector2d>> *Ss=(vector<vector<Vector2d>> *)param,Ss_=*Ss;
    switch (event)
    {
    case CV_EVENT_LBUTTONDOWN:
        Ss->back().push_back(Vector2d(x,y));
        break;
    
    case CV_EVENT_RBUTTONDOWN:
        if(Ss->back().size()>2)
            Ss->push_back(vector<Vector2d>{});
        break;
    
    default:
        break;
    }
}

void NavigationLib::MapGeneration(vector<vector<Vector2d>> &Ss)
{
    bool flag;
    Mat ima,ima_0,image;
    VideoCapture cap("rtsp://admin:hkv12345@192.168.1.240");
    Mat_<double> cam(3,3),distort(1,5);
    cam(0,0)=2268.42326;cam(0,1)=0;cam(0,2)=1321.49839;
    cam(1,0)=0;cam(1,1)=2265.29064;cam(1,2)=759.70445;
    cam(2,0)=0;cam(2,1)=0;cam(2,2)=1;
    distort(0,0)=-0.686545;distort(0,1)=0.346731;distort(0,2)=-0.026478;distort(0,3)=-0.000648;distort(0,4)=0;
    cout<<"here\n";
    flag=cap.read(ima);
    if(!flag)cerr<<"read failure.\n";
    undistort(ima,ima_0,cam,distort);
    resize(ima_0,ima_0,Size(800,450));
    image=ima_0(Rect(0,0,600,450));
    imshow("Draw",image);
    setMouseCallback("Draw",on_Mouse,(void*)&Ss);
    while (true)
    {
        image=ima_0(Rect(0,0,600,450));
        for(int i=0;i<Ss.size();i++)
        {
            if(Ss[i].size()>0)
            {
                if(i<Ss.size()-1)
                    for(int j=0;j<Ss[i].size();j++)
                    {
                        int n=Ss[i].size();
                        line(image,CvPoint_from_Eigen(Ss[i][j]),CvPoint_from_Eigen(Ss[i][(j+1)%n]),Scalar(255,255,255),5);
                    }
                else
                    for(int j=0;j<Ss[i].size()-1;j++)
                    {
                        int n=Ss[i].size();
                        line(image,CvPoint_from_Eigen(Ss[i][j]),CvPoint_from_Eigen(Ss[i][j+1]),Scalar(255,255,255),3);
                    }
            }
        }
        // sleep(1);
        imshow("Draw",image);
        char key=waitKey(1);
        switch (key)
        {
        case 100://Delete
            if(Ss.back().size()>0)Ss.back().pop_back();
            else if(Ss.size()>1)Ss.pop_back();
            cout<<"deleted\n";
            break;
        
        default:
            break;
        }
        if(key==27)break;
    }
    
    
}

vector<Vector2d> Region::PathConvert(Path pth)
{
    vector<Vector2d> v;
    if(pth.path.size()==0){cout<<"Kidding?\n";return v;};
    for(int i=0;i<pth.path.size();i++)v.push_back(Point_(pth.path[i]));
    return v;
}

bool vertexFilter(vector<Point> input)
{
    vector<double> a;
    double mean=0;
    for(int i=0;i<input.size();i++)
    {
        Point delta;
        if(i==input.size()-1)delta=input[i]-input[0];
        else delta=input[i]-input[i+1];
        double dis=sqrt(delta.x*delta.x+delta.y*delta.y);
        a.push_back(dis);
        mean+=dis;
    }
    mean/=a.size();
    if(mean<20)return false;
    else return true;
}

Vector2d NavigationLib::Trans(vector<Vector2d> pic,vector<Vector2d> real, Vector2d pic_point)
{
    Vector2d displacement=pic_point-pic[0];
    if(displacement.norm()<=5)return real[0];
    else
    {
        Vector2d vec1=pic[1]-pic[0],vec2=pic[2]-pic[0];
        Matrix2d A;
        A(0,0)=vec1(0);A(0,1)=vec2(0);A(1,0)=vec1(1);A(1,1)=vec2(1);
        Vector2d par=A.inverse()*displacement;
        // cout<<par.transpose()<<endl;
        return (1000*real[0]+par(0)*1000*(real[1]-real[0])+par(1)*1000*(real[2]-real[0]))/1000;
    }
}

void MouseTrans(int event, int x, int y, int flags, void* param)
{
    if(event==EVENT_LBUTTONDOWN)
    {
        Vector2d a=Trans(picVecs,realVecs,Vector2d(x,y));
        cout<<a.transpose()<<endl;
    }
}

void NavigationLib::PicSeg(vector<vector<Vector2d>> &Ss)
{
    //read calibration file
    ifstream a("./src/create_map_test/src/picCalibration.txt");
    string s;
    if (!a.is_open())cerr<<"error:no calibration file";
    else
    {
        vector<string> temp;
        while(getline(a,s))
        {
            // cout<<s<<endl;
            boost::split(temp,s,boost::is_any_of(": ,"));
            // for (int i=0;i<temp.size();i++)cout<<temp[i]<<"\n";
            // cout<<endl;
            picVecs.push_back(Vector2d(stod(temp[1]),stod(temp[2])));
            realVecs.push_back(Vector2d(stod(temp[3]),stod(temp[4])));
        }
        
    }
    //image segmentation
    Mat ima,ima_,ima_0;bool flag;
    VideoCapture cap("rtsp://admin:hkv12345@192.168.1.240");
    Mat_<double> cam(3,3),distort(1,5);
    cam(0,0)=2268.42326;cam(0,1)=0;cam(0,2)=1321.49839;
    cam(1,0)=0;cam(1,1)=2265.29064;cam(1,2)=759.70445;
    cam(2,0)=0;cam(2,1)=0;cam(2,2)=1;
    distort(0,0)=-0.686545;distort(0,1)=0.346731;distort(0,2)=-0.026478;distort(0,3)=-0.000648;distort(0,4)=0;
    
    flag=cap.read(ima);
    if(!flag)cerr<<"read failure.\n";
    undistort(ima,ima_0,cam,distort);
    resize(ima_0,ima_0,Size(800,450));
    ima_0=ima_0(Rect(0,0,600,450));
    ima_=ima_0;
    imwrite("1.jpg",ima_);
    
    Mat image_gray,image_thresh;
    vector<vector<Point>> contours;
    vector<Vector2d> vertex;
    cvtColor(ima_,image_gray,CV_BGR2GRAY);
    threshold(image_gray,image_thresh,100,255,ThresholdTypes::THRESH_OTSU);
    medianBlur(image_thresh,image_thresh,5);
    findContours(image_thresh,contours,RETR_LIST,CHAIN_APPROX_SIMPLE);
    drawContours(image_gray,contours,-1,Scalar(255,0,0),1);
    // cout<<contours[0][0]<<endl;
    int idx=0;

    for(int i=0;i<contours.size();i++)
    {
        vertex.clear();
        vector<Point> output;
        double ep=0.01*arcLength(contours[i],true);
        approxPolyDP(contours[i],output,ep,true);
        if(vertexFilter(output))
        {
            for(int j=0;j<output.size();j++)
            {
                circle(ima_,output[j],3,Scalar(255,255,0));
                if(j<output.size()-1)line(ima_,output[j],output[j+1],Scalar(0,0,255),2);
                else line(ima_,output[j],output[0],Scalar(0,0,255),2);
                vertex.push_back(Trans(picVecs,realVecs,Vector2d(output[j].x,output[j].y)));
            }
            putText(ima_,to_string(idx+1),output[0],1,3,Scalar(255,0,0),1);
            cout<<"vertex_size:"<<vertex.size()<<endl;
            Ss.push_back(vertex);
            // vertexs.push_back(output);
            idx+=1;
        }
    }
    // line(ima_,Point(600,0),Point(600,100),Scalar(255,255,0),3);
    imshow("vertexes",ima_);
    void *dcdc;
    setMouseCallback("vertexes",MouseTrans,dcdc);

    // imshow("contours",image_gray);
    // imshow("thresh",image_thresh);
    waitKey(0);
}