#include<iostream>
#include<vector>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Geometry>
#include<opencv2/opencv.hpp>
#include<opencv2/core.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;

namespace NavigationLib
{

Point2d CvPoint_from_Eigen(Vector2d a);
Vector3d FindLine(Vector2d Pt1,Vector2d Pt2);//Line Func:ax+by+c=0, return(a,b,c)
Vector2d GetDist_Point2Segment(Vector2d a,Vector2d b,Vector2d c);//return a vector representing the distance from point a to segment(not line) bc
bool IntersectionCheck(Vector2d a,Vector2d b,Vector2d c, Vector2d d);//check if the segment ab and cd intersects

struct Path//a variable encoding the route and its distance
{
    vector<int> path{};
    double Distance=0;
    Path operator+(Path pth2)
    {
        Path pth;
        pth.Distance=this->Distance+pth2.Distance;
        pth.path.insert(pth.path.end(),this->path.begin(),this->path.end());
        pth.path.insert(pth.path.end(),pth2.path.begin(),pth2.path.end());
        return pth;
    }
    bool operator<(Path pth2){return(this->Distance<pth2.Distance);}
    bool operator<=(Path pth2){return(this->Distance<=pth2.Distance);}
    bool operator>(Path pth2){return(this->Distance>pth2.Distance);}
    bool operator>=(Path pth2){return(this->Distance>=pth2.Distance);}
    bool operator==(Path pth2){return(this->Distance==pth2.Distance);}
};


Path Bellman_Ford(MatrixXd Laplace,int begin,int finish);//graph search algorithm, input a Laplace Matrix and return the path(route+distance) between begin and finish

class Section//polytopes
{
private:
    vector<Vector2d> GraphGeneration();
    double density;
public:
    vector<int> AP;
    vector<Vector2d> nodes;//vertex of the ploytope section
    vector<Vector2d> Points;//generated points inside the section
    Vector2d PointGeneration();
    double Area();
    bool PointCheck(Vector2d a);//check if the point is in this section
    bool SegmentCheck(Vector2d a,Vector2d b);
    Section(vector<Vector2d> input,double dense=1,vector<int> ap={1});
    ~Section();
};

class Region//Set of polytopes, the whole region in the experiment
{
private:
MatrixXd LaplaceConstruction();
public:
vector<Section> Sections;
vector<Scalar> colors;//color for each section
int totol_Points_num;
MatrixXd LaplaceMat;
int totalID(int SectionID,int nodeID);
vector<int> partialID(int totalID);
Vector2d Point_(int totalID);
void DrawRegions(Mat Image, int width=800, int height=800, bool show=true, String winname="Regions");
void DrawMap(Mat &Image, int width=800, int height=800, bool show=true, String winname="Map");
Path Navigation(vector<int> desiredTransition, Vector2d StartPoint);//基于Bellman-Ford,可以自编图算法重载
Path Navigation(vector<int> desiredTransition, vector<int> StartID);
Path SimpleNavigation(vector<int> desiredTransition, Vector2d StartPoint);
void DrawPath(Mat &Image, Path pth, int width=800, int height=800, bool show=true, String winname="Path");//path only
void DrawPath(Mat &Image, Path pth, Vector2d StartPoint, int width=800, int height=800, bool show=true, String winname="Path");
vector<Vector2d> PathConvert(Path pth);
Region(vector<Section> S={});
Region(vector<vector<Vector2d>> Ss, int dense=0);
~Region();
};

void on_Mouse(int event, int x, int y, int flags, void* param);
void MapGeneration(vector<vector<Vector2d>> &Ss);

void PicSeg(vector<vector<Vector2d>> &Ss);

Vector2d Trans(vector<Vector2d> pic,vector<Vector2d> real, Vector2d pic_point);

}