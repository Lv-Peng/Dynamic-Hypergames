#include<iostream>
#include<fstream>
#include<opencv2/opencv.hpp>
#include <opencv4/opencv2/imgproc/types_c.h>
#include <cmath>
#include <random>
#include <vector>

using namespace std;

constexpr int kImageSize = 15;
constexpr int kImageResolution = 50;

struct Point {
  Point(double x, double y) : x(x), y(y) {}
  double x = 0.0;
  double y = 0.0;
};

class Node {
 public:
  Node(double x, double y) : point_(x, y), parent_(nullptr), cost_(0.0) {}
  const Point& point() const { return point_; }
  void set_parent(Node* parent) { parent_ = parent; }
  Node* parent() { return parent_; }

 private:
  Point point_;
  std::vector<double> path_x_;
  std::vector<double> path_y_;
  Node* parent_ = nullptr;
  double cost_ = 0.0;
};

class RRT {
 public:
  RRT(Node* start_node, Node* goal_node,
      const vector<vector<double>>& obstacle_list,
      double step_size = 1.0, int goal_sample_rate = 5)
      : start_node_(start_node),
        goal_node_(goal_node),
        obstacle_list_(obstacle_list),
        step_size_(step_size),
        goal_sample_rate_(goal_sample_rate),
        goal_gen_(goal_rd_()),
        goal_dis_(std::uniform_int_distribution<int>(0, 100)),
        area_gen_(area_rd_()),
        area_dis_(std::uniform_real_distribution<double>(0, 15)) {}
  Node* GetNearestNode(const std::vector<double>& random_position);
  bool CollisionCheck(Node*);
  std::vector<Node*> Planning();

 private:
  Node* start_node_;
  Node* goal_node_;
  std::vector<std::vector<double>> obstacle_list_;
  std::vector<Node*> node_list_;
  double step_size_;

  int goal_sample_rate_;

  std::random_device goal_rd_;
  std::mt19937 goal_gen_;
  std::uniform_int_distribution<int> goal_dis_;

  std::random_device area_rd_;
  std::mt19937 area_gen_;
  std::uniform_real_distribution<double> area_dis_;
};

Node* RRT::GetNearestNode(const std::vector<double>& random_position) {
  int min_id = -1;
  double min_distance = std::numeric_limits<double>::max();
  for (int i = 0; i < node_list_.size(); i++) {
    double square_distance =
        std::pow(node_list_[i]->point().x - random_position[0], 2) +
        std::pow(node_list_[i]->point().y - random_position[1], 2);
    if (square_distance < min_distance) {
      min_distance = square_distance;
      min_id = i;
    }
  }

  return node_list_[min_id];
}

bool RRT::CollisionCheck(Node* newNode) {
  for (auto item : obstacle_list_) {
    if (std::sqrt(std::pow((item[0] - newNode->point().x), 2) +
                  std::pow((item[1] - newNode->point().y), 2)) <= item[2])
      return false;
  }
  return true;
}

vector<Node*> RRT::Planning() {
  cv::namedWindow("RRT");

  int count = 0;

  
  cv::Mat background(kImageSize * kImageResolution,
                     kImageSize * kImageResolution, CV_8UC3,
                     cv::Scalar(255, 255, 255));

  circle(background,
         cv::Point(start_node_->point().x * kImageResolution,
                   start_node_->point().y * kImageResolution),
         20, cv::Scalar(0, 0, 255), -1);
  circle(background,
         cv::Point(goal_node_->point().x * kImageResolution,
                   goal_node_->point().y * kImageResolution),
         20, cv::Scalar(255, 0, 0), -1);

  for (auto item : obstacle_list_) {
    circle(background,
           cv::Point(item[0] * kImageResolution, item[1] * kImageResolution),
           item[2] * kImageResolution, cv::Scalar(0, 0, 0), -1);
  }

  node_list_.push_back(start_node_);
  while (1) {
    std::vector<double> random_position;
    if (goal_dis_(goal_gen_) > goal_sample_rate_) {
      double randX = area_dis_(goal_gen_);
      double randY = area_dis_(goal_gen_);
      random_position.push_back(randX);
      random_position.push_back(randY);
    } else {
      random_position.push_back(goal_node_->point().x);
      random_position.push_back(goal_node_->point().y);
    }

    Node* nearestNode = GetNearestNode(random_position);
    double theta = atan2(random_position[1] - nearestNode->point().y,
                         random_position[0] - nearestNode->point().x);
    Node* newNode = new Node(nearestNode->point().x + step_size_ * cos(theta),
                             nearestNode->point().y + step_size_ * sin(theta));
    newNode->set_parent(nearestNode);

    if (!CollisionCheck(newNode)) continue;
    node_list_.push_back(newNode);

    line(background,
         cv::Point(static_cast<int>(newNode->point().x * kImageResolution),
                   static_cast<int>(newNode->point().y * kImageResolution)),
         cv::Point(static_cast<int>(nearestNode->point().x * kImageResolution),
                   static_cast<int>(nearestNode->point().y * kImageResolution)),
         cv::Scalar(0, 255, 0), 10);

    count++;
    imshow("RRT", background);
    cv::waitKey(5);

    if (sqrt(pow(newNode->point().x - goal_node_->point().x, 2) +
             pow(newNode->point().y - goal_node_->point().y, 2)) <=
        step_size_) {
      std::cout << "The path has been found!" << std::endl;
      break;
    }
  }

  std::vector<Node*> path;
  path.push_back(goal_node_);
  Node* tmp_node = node_list_.back();
  while (tmp_node->parent() != nullptr) {
    line(
        background,
        cv::Point(static_cast<int>(tmp_node->point().x * kImageResolution),
                  static_cast<int>(tmp_node->point().y * kImageResolution)),
        cv::Point(
            static_cast<int>(tmp_node->parent()->point().x * kImageResolution),
            static_cast<int>(tmp_node->parent()->point().y * kImageResolution)),
        cv::Scalar(255, 0, 255), 10);
    path.push_back(tmp_node);
    tmp_node = tmp_node->parent();
  }

  imshow("RRT", background);
  cv::waitKey(0);
  path.push_back(start_node_);
 
  return path;
}

cv::Mat map_get()//全局：大津法 0:障碍物 255:others
{
  
  using namespace cv;

    Mat img = imread("/home/yons/fmy/map.pgm");
        if (img.empty())
        {
            cout << "Error:could not load image" << endl;
            //return ;
        }

    Mat gray;
    cvtColor(img, gray, CV_BGR2GRAY);

    Mat dst;
    threshold(gray, dst, 125, 255, CV_THRESH_OTSU);
    
    //imshow("src", img);
    //imshow("gray", gray);
    //imshow("dst", dst);
    waitKey(0);

    return dst;
}

void map_to_obstacle(cv::Mat dst)
{
  ofstream f;
  f.open("/home/yons/fmy/map.txt");
  int h = dst.rows;
  int w = dst.cols;
  f<<h<<" "<<w<<endl;
  for(int i = 0; i < h; ++i)
  {
    for(int j = 0; j < w; ++j)
    {
      if((int)dst.at<uchar>(i,j)==0) f<<i<<" "<<j<<endl;
    }
  }
  //cout << "finish!" << endl;
  return;
}

vector<vector<double>>MapData(string _MapPath)
{
	ifstream f;
	f.open(_MapPath);
	string str;
	vector<vector<double> > num;
	bool  FirstLine = true;
	while (getline(f, str))      
	{
		if (FirstLine)
		{
			istringstream in(str);   
			int a;
			in >> a;
			int height = a;
		    in >> a;
			int wight = a;
			num.resize(height, vector<double>(wight, 255));
			FirstLine = false;
		}
		else
		{
			istringstream input(str);   
			vector<int> tmp;
			int a;
			while (input >> a)         
				tmp.push_back(a);
			num[tmp[0]][tmp[1]] = 0;
		}
	}
	return num;
}

vector<vector<double>>MapData_(string _MapPath)
{
	ifstream f;
	f.open(_MapPath);
	string str;
	vector<vector<double> > num;
	bool  FirstLine = true;
  int height;
  int wight;
	while (getline(f, str))      
	{
		if (FirstLine)
		{
			istringstream in(str);   
			int a;
			in >> a;
			height = a;
		    in >> a;
			wight = a;
			//num.resize(height, vector<double>(wight, 255));
			FirstLine = false;
		}
		else
		{
			istringstream input(str);   
			vector<double> tmp;
			int a;
			while (input >> a)          
				tmp.push_back(double(a)*kImageSize*5/height-31);//要调
        tmp.push_back(0.2);
         num.push_back(tmp);
      
			//num[tmp[0]][tmp[1]] = 0;
		}
    
	}
  /* for(auto &array: num) {
        for(auto &arr: array ) {
            std::cout << arr << " ";
        }
        std::cout << "\n";
    } */
	return num;
}


int main(int argc, char* argv[]) {
  // (x, y, r)
  //vector<vector<double>> obstacle_list1{{7, 5, 1},  {5, 6, 2}, {5, 8, 2},
  //                                   {5, 10, 2}, {9, 5, 2}, {11, 5, 2}};
  map_to_obstacle(map_get());
  vector<vector<double>> obstacle_list = MapData_("/home/yons/fmy/map.txt");
  /* cout << "map: " << endl;
  for(auto &array: obstacle_list) {
        for(auto &arr: array ) {
            cout << arr << " ";
        }
        cout << "\n";
    }  */
  Node* start_node = new Node(5.0, 5.0);
  Node* goal_node = new Node(9.0, 9.0);

  RRT rrt(start_node, goal_node, obstacle_list, 0.1, 5);
  vector<Node*> path = rrt.Planning();

  cout << "path: " << endl;
  for(auto &m: path)
  {
    cout  << "("<< m->point().x << "," << m->point().y << ")" << endl;
  } 
  
  return 0;
}
