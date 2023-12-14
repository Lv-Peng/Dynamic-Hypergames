#include<opencv2/opencv.hpp>
#include<iostream>
#include<unistd.h>

using namespace std;
using namespace cv;

int main()
{
    string path="/home/yons/zky/OriImage";
    int cnt=0;
    while(true)
    {
        VideoCapture cap("rtsp://admin:hkv12345@192.168.1.240");
        Mat I;
        cap.read(I);
        imwrite(path+to_string(cnt)+".png",I);
        cnt+=1;
        cout<<"Image"<<cnt<<endl;
        // imshow("ccc",I);
        waitKey(1000);
    }


    return 0;
}